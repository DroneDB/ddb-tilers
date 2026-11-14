// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "cog/tiler/block_cache.h"
#include "cog/tiler/cog_tiler.h"
#include "cog/tiler/json_lite.h"
#include "cogtiler/synth_raster.h"
#include "copc/build/copc_builder.h"
#include "copc/cache/node_cache.h"
#include "copc/tiler/copc_tiler.h"
#include "copcbuild/synth_las.h"
#include "copctiler/synth_cloud.h"
#include "core/crs/crs.h"
#include "core/exec/thread_pool.h"
#include "core/tile/compat.h"
#include "ddbt/ddbt.h"
#include "test_env.h"

TEST(Api, Version) {
    const std::string v = ddbtVersion();
    EXPECT_EQ(v, "0.5.0");
}

TEST(Api, LastErrorIsEmptyWhenNothingFailed) {
    char* json = nullptr;
    ASSERT_EQ(ddbtCpuInfo(&json), DDBT_ERR_NONE);
    ddbtFree(json);
    EXPECT_STREQ(ddbtLastError(), "");
}

TEST(Api, MemoryTileFmtReportsAMissingInput) {
    uint8_t* buf = nullptr;
    int size = 0;
    EXPECT_EQ(ddbtMemoryTileFmt("/nonexistent.tif", 10, 1, 2, &buf, &size, 256, false, false, nullptr, "webp"),
              DDBT_ERR_NOTFOUND);
    EXPECT_EQ(buf, nullptr);
    EXPECT_EQ(size, 0);
    EXPECT_STREQ(ddbtLastError(), "/nonexistent.tif does not exist");
}

TEST(Api, MemoryTileFmtValidatesArguments) {
    uint8_t* buf = nullptr;
    int size = 0;
    EXPECT_EQ(ddbtMemoryTileFmt(nullptr, 1, 0, 0, &buf, &size, 256, false, false, nullptr, "png"), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtMemoryTileFmt("x.tif", 1, 0, 0, nullptr, &size, 256, false, false, nullptr, "png"), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtMemoryTileFmt("x.tif", 1, 0, 0, &buf, &size, 100, false, false, nullptr, "png"), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtMemoryTileFmt("x.tif", 1, 0, 0, &buf, &size, 256, false, false, nullptr, "bmp"), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(buf, nullptr);
}

TEST(Api, LastErrorIsClearedByASuccessfulCall) {
    ASSERT_EQ(ddbtBuildCog("a.tif", "", nullptr), DDBT_ERR_INVALIDARGS);
    EXPECT_NE(std::string(ddbtLastError()), "");
    char* json = nullptr;
    ASSERT_EQ(ddbtCpuInfo(&json), DDBT_ERR_NONE);
    ddbtFree(json);
    EXPECT_STREQ(ddbtLastError(), "");
}

TEST(Api, BuildCogOfAMissingInputIsNotFound) {
    EXPECT_EQ(ddbtBuildCog("in.tif", "out.tif", nullptr), DDBT_ERR_NOTFOUND);
    EXPECT_STRNE(ddbtLastError(), "");
}

TEST(Api, BuildCopcOfAMissingInputIsNotFound) {
    EXPECT_EQ(ddbtBuildCopc("in.laz", "out.copc.laz", "{}"), DDBT_ERR_NOTFOUND);
    EXPECT_STRNE(ddbtLastError(), "");
}

TEST(Api, BuildCopcRejectsBadArguments) {
    EXPECT_EQ(ddbtBuildCopc("", "out.copc.laz", nullptr), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtBuildCopc("in.laz", nullptr, nullptr), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtBuildCopc("in.laz", "out.copc.laz", "[1]"), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtBuildCopc("in.laz", "out.copc.laz", "{\"bogus\": 1}"), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtBuildCopc("in.laz", "out.copc.laz", "{\"threads\": -1}"), DDBT_ERR_INVALIDARGS);
}

TEST(Api, CpuInfoIsJsonWithTheDispatchTarget) {
    char* json = nullptr;
    ASSERT_EQ(ddbtCpuInfo(&json), DDBT_ERR_NONE);
    ASSERT_NE(json, nullptr);
    const std::string s = json;
    ddbtFree(json);
    EXPECT_EQ(s.front(), '{');
    EXPECT_EQ(s.back(), '}');
    EXPECT_NE(s.find("\"dispatchTarget\":\""), std::string::npos) << s;
    EXPECT_NE(s.find("\"compiledTargets\":["), std::string::npos) << s;
    EXPECT_NE(s.find("\"supportedTargets\":["), std::string::npos) << s;
    EXPECT_NE(s.find("\"version\":\"0.5.0\""), std::string::npos) << s;
    EXPECT_EQ(ddbtCpuInfo(nullptr), DDBT_ERR_INVALIDARGS);
}

TEST(Api, FreeNullIsNoop) { EXPECT_EQ(ddbtFree(nullptr), DDBT_ERR_NONE); }

// ---------------------------------------------------------------------------------------------
// The exports for a host library: ddbtConfigure, ddbtTileInfo, ddbtBuildCog2 / ddbtBuildCopc2.

namespace {

namespace json = ddbt::cog::tiler::json;

std::filesystem::path ApiScratch(const std::string& name) {
    return ddbt::copc::tiler::test::ScratchDir("api-" + name);
}

json::Value TileInfoOf(const std::string& path, int tileSize) {
    char* out = nullptr;
    EXPECT_EQ(ddbtTileInfo(path.c_str(), tileSize, &out), DDBT_ERR_NONE) << ddbtLastError();
    if (out == nullptr) return json::Value();
    const auto v = json::Parse(out);
    ddbtFree(out);
    EXPECT_TRUE(v && v->IsObject());
    return v ? *v : json::Value();
}

double Num(const json::Value& v, const char* key) {
    const json::Value* m = v.Find(key);
    EXPECT_NE(m, nullptr) << key;
    return m != nullptr ? m->num : 0;
}

struct ProgressLog {
    std::vector<std::pair<double, std::string>> calls;
    static void Record(double fraction, const char* phase, void* user) {
        static_cast<ProgressLog*>(user)->calls.emplace_back(fraction, phase != nullptr ? phase : "");
    }
    void ExpectMonotonicTo1(const std::vector<std::string>& phases) const {
        ASSERT_FALSE(calls.empty());
        for (size_t i = 1; i < calls.size(); ++i) EXPECT_GE(calls[i].first, calls[i - 1].first) << i;
        EXPECT_DOUBLE_EQ(calls.back().first, 1.0);
        for (const auto& [f, phase] : calls) {
            EXPECT_GE(f, 0.0);
            EXPECT_NE(std::find(phases.begin(), phases.end(), phase), phases.end()) << phase;
        }
    }
};

}  // namespace

TEST(HostApi, ConfigureAppliesEverySetting) {
    const auto mode = ddbt::copc::tiler::GetMode();
    const size_t tileBudget = ddbt::cog::tiler::TileCacheBudget();
    const size_t blockBudget = ddbt::cog::tiler::BlockCacheBudget();
    const size_t nodeBudget = ddbt::copc::cache::NodeCache::Global().BudgetBytes();
    const std::string projDir = ddbt::crs::ProjDataDir();

    ASSERT_EQ(ddbtConfigure(R"({"copcTilerMode": "modern", "tileCacheMb": 3, "blockCacheMb": 5,
                                "copcNodeCacheMb": 7, "copcTileCacheMb": 9, "copcBuildMemMb": 300})"),
              DDBT_ERR_NONE)
        << ddbtLastError();
    EXPECT_EQ(ddbt::copc::tiler::GetMode(), ddbt::copc::tiler::Mode::kModern);
    EXPECT_EQ(ddbt::cog::tiler::TileCacheBudget(), size_t{3} << 20);
    EXPECT_EQ(ddbt::cog::tiler::BlockCacheBudget(), size_t{5} << 20);
    EXPECT_EQ(ddbt::copc::cache::NodeCache::Global().BudgetBytes(), size_t{7} << 20);
    EXPECT_EQ(ddbt::copc::tiler::TileCacheBudget(), size_t{9} << 20);
    const auto budget = ddbt::copc::build::DefaultCopcMemoryBudget();
    ASSERT_TRUE(budget.ok());
    EXPECT_EQ(*budget, uint64_t{300} << 20);

    // The host's setting wins over the environment; 0 restores the default order.
    {
        const ddbt::test::ScopedEnv env("DDBT_COPC_BUILD_MEM_MB", "123");
        EXPECT_EQ(*ddbt::copc::build::DefaultCopcMemoryBudget(), uint64_t{300} << 20);
        ASSERT_EQ(ddbtConfigure(R"({"copcBuildMemMb": 0})"), DDBT_ERR_NONE);
        EXPECT_EQ(*ddbt::copc::build::DefaultCopcMemoryBudget(), uint64_t{123} << 20);
    }

    ASSERT_EQ(ddbtConfigure(R"({"projDataDir": "/some/where"})"), DDBT_ERR_NONE);
    EXPECT_EQ(ddbt::crs::ProjDataDir(), "/some/where");

    ddbt::copc::tiler::SetMode(mode);
    ddbt::cog::tiler::SetTileCacheBudget(tileBudget);
    ddbt::cog::tiler::SetBlockCacheBudget(blockBudget);
    ddbt::copc::cache::NodeCache::Global().SetBudget(nodeBudget);
    ddbt::crs::SetProjDataDir(projDir);
}

TEST(HostApi, ConfigureRefusesBadSettingsAndChangesNothing) {
    const auto mode = ddbt::copc::tiler::GetMode();
    const size_t tileBudget = ddbt::cog::tiler::TileCacheBudget();
    for (const char* bad : {"", "[1]", "{\"bogus\": 1}", "{\"tileCacheMb\": -1}", "{\"tileCacheMb\": 1.5}",
                            "{\"tileCacheMb\": \"3\"}", "{\"copcTilerMode\": \"fancy\"}", "{\"projDataDir\": 3}",
                            "{\"threads\": 5000}", "{\"copcBuildMemMb\": 99999999999}",
                            // a valid setting next to a bad one is not applied either
                            "{\"copcTilerMode\": \"modern\", \"tileCacheMb\": -1}"}) {
        EXPECT_EQ(ddbtConfigure(bad), DDBT_ERR_INVALIDARGS) << bad;
        EXPECT_STRNE(ddbtLastError(), "") << bad;
    }
    EXPECT_EQ(ddbtConfigure(nullptr), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbt::copc::tiler::GetMode(), mode);
    EXPECT_EQ(ddbt::cog::tiler::TileCacheBudget(), tileBudget);
}

TEST(HostApi, ConfigureRefusesThreadsOnceThePoolRunsAndChangesNothing) {
    (void)ddbt::exec::DefaultPool();  // the pool exists from here on
    const size_t tileBudget = ddbt::cog::tiler::TileCacheBudget();
    EXPECT_EQ(ddbtConfigure(R"({"threads": 2, "tileCacheMb": 1})"), DDBT_ERR_EXCEPTION);
    EXPECT_NE(std::string(ddbtLastError()).find("already running"), std::string::npos) << ddbtLastError();
    EXPECT_EQ(ddbt::cog::tiler::TileCacheBudget(), tileBudget);
}

TEST(HostApi, TileInfoOfARasterIsGdalTilersBoundsAndZooms) {
    namespace syn = ddbt::cog::tiler::testing;
    const auto dir = ApiScratch("tileinfo-raster");
    const std::string path = (dir / "r.tif").string();
    syn::WriteSynth(path, syn::Spec(512, 1, 1));
    for (const int size : {256, 512}) {
        const json::Value v = TileInfoOf(path, size);
        ASSERT_NE(v.Find("type"), nullptr);
        EXPECT_EQ(v.Find("type")->str, "raster");
        // EPSG:3857 file: GDALTiler's bounds are its geotransform's (0.25 m pixels, 512 x 512).
        const double gt[6] = {syn::kOriginX, syn::kPixel, 0, syn::kOriginY, 0, -syn::kPixel};
        const ddbt::tile::compat::RasterTiling t(size, gt, 512, 512);
        EXPECT_DOUBLE_EQ(Num(v, "minX"), t.bounds.oMinX);
        EXPECT_DOUBLE_EQ(Num(v, "minY"), t.bounds.oMinY);
        EXPECT_DOUBLE_EQ(Num(v, "maxX"), t.bounds.oMaxX);
        EXPECT_DOUBLE_EQ(Num(v, "maxY"), t.bounds.oMaxY);
        EXPECT_EQ(Num(v, "minZoom"), t.zooms.min);
        EXPECT_EQ(Num(v, "maxZoom"), t.zooms.max);
        EXPECT_DOUBLE_EQ(Num(v, "minX"), syn::kOriginX);
        EXPECT_DOUBLE_EQ(Num(v, "maxX"), syn::kOriginX + 512 * syn::kPixel);
        EXPECT_GE(Num(v, "maxZoom"), Num(v, "minZoom"));
    }
    std::filesystem::remove_all(dir);
}

#ifdef __linux__
namespace {
// Open descriptors of this process on `path` (Linux: the links of /proc/self/fd).
int OpenDescriptorsOf(const std::string& path) {
    const auto target = std::filesystem::weakly_canonical(path);
    int n = 0;
    for (const auto& e : std::filesystem::directory_iterator("/proc/self/fd")) {
        std::error_code ec;
        const auto p = std::filesystem::read_symlink(e.path(), ec);
        if (!ec && p == target) ++n;
    }
    return n;
}
}  // namespace

TEST(HostApi, ReleaseSourcesClosesTheFilesOfTheTilers) {
    namespace syn = ddbt::cog::tiler::testing;
    const auto dir = ApiScratch("release");
    const std::string path = (dir / "r.tif").string();
    syn::WriteSynth(path, syn::Spec(512, 1, 1));
    char* json = nullptr;
    ASSERT_EQ(ddbtTileInfo(path.c_str(), 256, &json), DDBT_ERR_NONE);
    ddbtFree(json);
    uint8_t* buf = nullptr;
    int size = 0;
    const auto tile = [&] {
        const json::Value v = TileInfoOf(path, 256);
        const int z = static_cast<int>(Num(v, "maxZoom"));
        const double res = 2 * 20037508.342789244 / (256.0 * double(1u << z));
        const int x = static_cast<int>((Num(v, "minX") + 20037508.342789244) / (256 * res));
        const int y = static_cast<int>((20037508.342789244 - Num(v, "maxY")) / (256 * res));
        ASSERT_EQ(ddbtMemoryTileFmt(path.c_str(), z, x, y, &buf, &size, 256, true, false, nullptr, "png"), DDBT_ERR_NONE)
            << ddbtLastError();
        ddbtFree(buf);
    };
    tile();
    EXPECT_GE(OpenDescriptorsOf(path), 1);
    ASSERT_EQ(ddbtReleaseSources(), DDBT_ERR_NONE);
    EXPECT_EQ(OpenDescriptorsOf(path), 0);
    tile();  // opened again on the next tile
    EXPECT_GE(OpenDescriptorsOf(path), 1);
    ASSERT_EQ(ddbtReleaseSources(), DDBT_ERR_NONE);
    EXPECT_EQ(OpenDescriptorsOf(path), 0);
    std::filesystem::remove_all(dir);
}
#endif

TEST(HostApi, TileInfoOfAPointCloudFollowsTheMode) {
    namespace syn = ddbt::copc::tiler::test;
    const auto dir = ApiScratch("tileinfo-cloud");
    syn::CloudSpec spec;
    spec.points = 2000;
    const std::string path = syn::WriteCloud(dir, "c", spec);
    const auto mode = ddbt::copc::tiler::GetMode();
    for (const auto m : {ddbt::copc::tiler::Mode::kCompat, ddbt::copc::tiler::Mode::kModern}) {
        ddbt::copc::tiler::SetMode(m);
        const json::Value v = TileInfoOf(path, 256);
        ASSERT_NE(v.Find("type"), nullptr);
        EXPECT_EQ(v.Find("type")->str, "pointcloud");
        const auto e = ddbt::copc::tiler::Extent(path, 256);
        EXPECT_DOUBLE_EQ(Num(v, "minX"), e.minX);
        EXPECT_DOUBLE_EQ(Num(v, "maxY"), e.maxY);
        const auto z = ddbt::tile::compat::PointCloudZoomRange(ddbt::tile::compat::GlobalMercator(256), e.minX, e.minY,
                                                               e.maxX, e.maxY, 128);
        EXPECT_EQ(Num(v, "minZoom"), z.min);
        EXPECT_EQ(Num(v, "maxZoom"), z.max);
        // A tile of the reported range renders.
        uint8_t* buf = nullptr;
        int size = 0;
        const ddbt::tile::compat::GlobalMercator merc(256);
        const auto range = ddbt::tile::compat::MinMaxCoordsForZ(merc, e.minX, e.minY, e.maxX, e.maxY, z.max);
        EXPECT_EQ(ddbtMemoryTileFmt(path.c_str(), z.max, static_cast<int>(range.min.x), static_cast<int>(range.min.y),
                                    &buf, &size, 256, false, true, nullptr, "png"),
                  DDBT_ERR_NONE)
            << ddbtLastError();
        ddbtFree(buf);
    }
    ddbt::copc::tiler::SetMode(mode);
    std::filesystem::remove_all(dir);
}

namespace {
// A remote reader over local files: the URL https://host.test/<name> is the file <dir>/<name>.
struct FakeRemote {
    std::filesystem::path dir;
    std::string version = "v1";
    ddbtErr statCode = DDBT_ERR_NONE;
    ddbtErr readCode = DDBT_ERR_NONE;
    std::atomic<int> headerReads{0};
    std::atomic<int> reads{0};

    std::filesystem::path FileOf(const char* url) const {
        const std::string u(url);
        return dir / u.substr(u.rfind('/') + 1);
    }
    static void Say(char* error, size_t errorSize, const char* text) {
        std::snprintf(error, errorSize, "%s", text);
    }
    static ddbtErr Stat(const char* url, uint64_t* outSize, char* version, size_t versionSize, char* error,
                        size_t errorSize, void* user) {
        auto& self = *static_cast<FakeRemote*>(user);
        if (self.statCode != DDBT_ERR_NONE) {
            Say(error, errorSize, "HTTP 404");
            return self.statCode;
        }
        std::error_code ec;
        *outSize = std::filesystem::file_size(self.FileOf(url), ec);
        if (ec) return DDBT_ERR_NOTFOUND;
        std::snprintf(version, versionSize, "%s", self.version.c_str());
        return DDBT_ERR_NONE;
    }
    static ddbtErr Read(const char* url, uint64_t offset, uint8_t* buffer, size_t size, char* error,
                        size_t errorSize, void* user) {
        auto& self = *static_cast<FakeRemote*>(user);
        ++self.reads;
        if (offset == 0) ++self.headerReads;
        if (self.readCode != DDBT_ERR_NONE) {
            Say(error, errorSize, "connection reset");
            return self.readCode;
        }
        std::ifstream in(self.FileOf(url), std::ios::binary);
        in.seekg(static_cast<std::streamoff>(offset));
        in.read(reinterpret_cast<char*>(buffer), static_cast<std::streamsize>(size));
        return in.gcount() == static_cast<std::streamsize>(size) ? DDBT_ERR_NONE : DDBT_ERR_EXCEPTION;
    }
    ddbtRemoteReader Reader() {
        return ddbtRemoteReader{sizeof(ddbtRemoteReader), &FakeRemote::Stat, &FakeRemote::Read, this};
    }
};

std::vector<uint8_t> TileBytes(const std::string& input, int z, int x, int y) {
    uint8_t* buf = nullptr;
    int size = 0;
    if (ddbtMemoryTileFmt(input.c_str(), z, x, y, &buf, &size, 256, false, true, nullptr, "png") != DDBT_ERR_NONE)
        return {};
    std::vector<uint8_t> bytes(buf, buf + size);
    ddbtFree(buf);
    return bytes;
}
}  // namespace

TEST(HostApi, ARemotePointCloudIsReadThroughTheHostsRemoteReader) {
    namespace syn = ddbt::copc::tiler::test;
    const auto dir = ApiScratch("remote-cloud");
    syn::CloudSpec spec;
    spec.points = 4000;
    const std::string local = syn::WriteCloud(dir, "c", spec);
    const std::string name = std::filesystem::path(local).filename().string();
    const std::string url = "https://host.test/" + name;
    char* out = nullptr;
    uint8_t* buf = nullptr;
    int size = 0;

    // No reader: not implemented, as before
    ASSERT_EQ(ddbtSetRemoteReader(nullptr), DDBT_ERR_NONE);
    EXPECT_EQ(ddbtTileInfo(url.c_str(), 256, &out), DDBT_ERR_NOT_IMPLEMENTED);
    EXPECT_EQ(ddbtMemoryTileFmt(url.c_str(), 0, 0, 0, &buf, &size, 256, false, true, nullptr, "png"),
              DDBT_ERR_NOT_IMPLEMENTED);

    FakeRemote remote;
    remote.dir = dir;
    const ddbtRemoteReader reader = remote.Reader();
    ASSERT_EQ(ddbtSetRemoteReader(&reader), DDBT_ERR_NONE) << ddbtLastError();

    // The bounds, the zooms and the tiles of the local file
    const json::Value v = TileInfoOf(url, 256);
    const json::Value l = TileInfoOf(local, 256);
    for (const char* k : {"minX", "minY", "maxX", "maxY", "minZoom", "maxZoom"}) EXPECT_EQ(Num(v, k), Num(l, k)) << k;
    const ddbt::tile::compat::GlobalMercator merc(256);
    const int z = static_cast<int>(Num(v, "maxZoom"));
    const auto range = ddbt::tile::compat::MinMaxCoordsForZ(merc, Num(v, "minX"), Num(v, "minY"), Num(v, "maxX"),
                                                            Num(v, "maxY"), z);
    for (int x = static_cast<int>(range.min.x); x <= static_cast<int>(range.max.x); ++x)
        for (int y = static_cast<int>(range.min.y); y <= static_cast<int>(range.max.y); ++y) {
            const auto remoteTile = TileBytes(url, z, x, y);
            ASSERT_FALSE(remoteTile.empty()) << ddbtLastError();
            EXPECT_EQ(remoteTile, TileBytes(local, z, x, y)) << z << "/" << x << "/" << y;
        }

    // An unchanged object is not opened again; another version is
    const int headers = remote.headerReads.load();
    EXPECT_GE(headers, 1);
    TileBytes(url, z, static_cast<int>(range.min.x), static_cast<int>(range.min.y));
    EXPECT_EQ(remote.headerReads.load(), headers);
    remote.version = "v2";
    EXPECT_FALSE(TileBytes(url, z, static_cast<int>(range.min.x), static_cast<int>(range.min.y)).empty());
    EXPECT_EQ(remote.headerReads.load(), headers + 1);

    // The host's codes and reasons
    EXPECT_EQ(ddbtTileInfo("https://host.test/missing.copc.laz", 256, &out), DDBT_ERR_NOTFOUND);
    remote.statCode = DDBT_ERR_EXCEPTION;
    EXPECT_EQ(ddbtTileInfo(url.c_str(), 256, &out), DDBT_ERR_EXCEPTION);
    EXPECT_NE(std::string(ddbtLastError()).find("HTTP 404"), std::string::npos) << ddbtLastError();
    remote.statCode = DDBT_ERR_NONE;
    remote.version = "v3";
    remote.readCode = DDBT_ERR_EXCEPTION;
    EXPECT_EQ(ddbtTileInfo(url.c_str(), 256, &out), DDBT_ERR_EXCEPTION);
    EXPECT_NE(std::string(ddbtLastError()).find("connection reset"), std::string::npos) << ddbtLastError();
    remote.readCode = DDBT_ERR_NONE;

    // Only http(s) URLs go to the reader, and rasters are not read remotely
    EXPECT_EQ(ddbtTileInfo("s3://bucket/c.copc.laz", 256, &out), DDBT_ERR_NOT_IMPLEMENTED);
    EXPECT_EQ(ddbtTileInfo("https://host.test/r.tif", 256, &out), DDBT_ERR_NOT_IMPLEMENTED);
    EXPECT_EQ(out, nullptr);

    // Bad readers change nothing
    ddbtRemoteReader bad = reader;
    bad.read = nullptr;
    EXPECT_EQ(ddbtSetRemoteReader(&bad), DDBT_ERR_INVALIDARGS);
    bad = reader;
    bad.structSize = 4;
    EXPECT_EQ(ddbtSetRemoteReader(&bad), DDBT_ERR_INVALIDARGS);
    EXPECT_FALSE(TileBytes(url, z, static_cast<int>(range.min.x), static_cast<int>(range.min.y)).empty());

    ASSERT_EQ(ddbtSetRemoteReader(nullptr), DDBT_ERR_NONE);
    ASSERT_EQ(ddbtReleaseSources(), DDBT_ERR_NONE);
    std::filesystem::remove_all(dir);
}

TEST(HostApi, TileInfoRefusesBadArguments) {
    char* out = nullptr;
    EXPECT_EQ(ddbtTileInfo(nullptr, 256, &out), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtTileInfo("x.tif", 256, nullptr), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtTileInfo("/nonexistent.tif", 256, &out), DDBT_ERR_NOTFOUND);
    EXPECT_EQ(ddbtTileInfo("/nonexistent.copc.laz", 256, &out), DDBT_ERR_NOTFOUND);
    EXPECT_EQ(ddbtTileInfo("https://example.com/a.tif", 256, &out), DDBT_ERR_NOT_IMPLEMENTED);
    const auto dir = ApiScratch("tileinfo-args");
    const std::string path = (dir / "r.tif").string();
    ddbt::cog::tiler::testing::WriteSynth(path, ddbt::cog::tiler::testing::Spec(16, 1, 1));
    EXPECT_NE(ddbtTileInfo(path.c_str(), 100, &out), DDBT_ERR_NONE);
    EXPECT_EQ(out, nullptr);
    std::filesystem::remove_all(dir);
}

TEST(HostApi, BuildCog2ReportsProgressToOne) {
    namespace syn = ddbt::cog::tiler::testing;
    const auto dir = ApiScratch("cog2");
    const std::string in = (dir / "in.tif").string(), out = (dir / "out.tif").string();
    syn::SynthSpec spec = syn::Spec(512, 3, 1);
    spec.tile = 256;
    syn::WriteSynth(in, spec);
    ProgressLog log;
    ASSERT_EQ(ddbtBuildCog2(in.c_str(), out.c_str(), nullptr, &ProgressLog::Record, &log), DDBT_ERR_NONE)
        << ddbtLastError();
    log.ExpectMonotonicTo1({"warping", "overviews", "compressing", "statistics"});
    EXPECT_TRUE(std::filesystem::exists(out));
    // Without a callback: the same file.
    const std::string out2 = (dir / "out2.tif").string();
    ASSERT_EQ(ddbtBuildCog2(in.c_str(), out2.c_str(), nullptr, nullptr, nullptr), DDBT_ERR_NONE) << ddbtLastError();
    EXPECT_EQ(std::filesystem::file_size(out), std::filesystem::file_size(out2));
    std::filesystem::remove_all(dir);
}

TEST(HostApi, BuildCopc2ReportsProgressToOne) {
    namespace syn = ddbt::copc::testing;
    const auto dir = ApiScratch("copc2");
    const std::string in = (dir / "in.las").string(), out = (dir / "out.copc.laz").string();
    syn::SynthLas s;
    s.format = 3;
    syn::FillRandom(s, 20000, 7);
    ASSERT_TRUE(syn::WriteBytes(in, syn::BuildLasBytes(s)));
    ProgressLog log;
    ASSERT_EQ(ddbtBuildCopc2(in.c_str(), out.c_str(), nullptr, &ProgressLog::Record, &log), DDBT_ERR_NONE)
        << ddbtLastError();
    log.ExpectMonotonicTo1({"planning", "reading", "building", "writing"});
    EXPECT_TRUE(std::filesystem::exists(out));
    EXPECT_EQ(ddbtBuildCopc2("", out.c_str(), nullptr, &ProgressLog::Record, &log), DDBT_ERR_INVALIDARGS);
    std::filesystem::remove_all(dir);
}
