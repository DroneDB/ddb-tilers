// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdio>
#include <cstdint>
#include <climits>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "cog/tiff/geotiff.h"
#include "cog/tiff/tiff_reader.h"
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
#include "core/encode/tile_encoder.h"
#include "core/exec/thread_pool.h"
#include "core/tile/compat.h"
#include "ddbt/ddbt.h"
#include "gdal_palette_average.h"
#include "test_env.h"

TEST(Api, Version) {
    const std::string v = ddbtVersion();
    EXPECT_EQ(v, "0.12.0");
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
    EXPECT_NE(s.find("\"version\":\"0.12.0\""), std::string::npos) << s;
    EXPECT_EQ(ddbtCpuInfo(nullptr), DDBT_ERR_INVALIDARGS);
}

TEST(Api, FreeNullIsNoop) { EXPECT_EQ(ddbtFree(nullptr), DDBT_ERR_NONE); }

// ---------------------------------------------------------------------------------------------
// The exports for a host library: ddbtConfigure, ddbtTileInfo, ddbtBuildCog2 / ddbtBuildCopc2.

namespace {

namespace json = ddbt::cog::tiler::json;

// `text` as a JSON string, its quotes included: the path of a file put in options (a Windows path has backslashes)
std::string JsonString(const std::string& text) {
    std::string out = "\"";
    for (const char c : text) {
        if (c == '\\' || c == '"') out += '\\';
        out += c;
    }
    return out + "\"";
}

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
    const uint64_t maxPoints = ddbt::copc::tiler::TileMaxPoints();
    const uint64_t maxNodePoints = ddbt::copc::tiler::TileMaxNodePoints();

    ASSERT_EQ(ddbtConfigure(R"({"copcTilerMode": "modern", "tileCacheMb": 3, "blockCacheMb": 5,
                                "copcNodeCacheMb": 7, "copcTileCacheMb": 9, "copcBuildMemMb": 300,
                                "copcTileMaxPoints": 123456, "copcTileMaxNodePoints": 7890})"),
              DDBT_ERR_NONE)
        << ddbtLastError();
    EXPECT_EQ(ddbt::copc::tiler::TileMaxPoints(), 123456u);
    EXPECT_EQ(ddbt::copc::tiler::TileMaxNodePoints(), 7890u);
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
    ddbt::copc::tiler::SetTileMaxPoints(maxPoints);
    ddbt::copc::tiler::SetTileMaxNodePoints(maxNodePoints);
}

json::Value GetConfig() {
    char* out = nullptr;
    EXPECT_EQ(ddbtGetConfig(&out), DDBT_ERR_NONE) << ddbtLastError();
    const auto v = json::Parse(out != nullptr ? out : "");
    ddbtFree(out);
    return v ? *v : json::Value{};
}

TEST(HostApi, GetConfigGivesEverySettingInEffect) {
    const uint64_t maxPoints = ddbt::copc::tiler::TileMaxPoints();
    const uint64_t maxNodePoints = ddbt::copc::tiler::TileMaxNodePoints();
    const size_t copcTileBudget = ddbt::copc::tiler::TileCacheBudget();
    const size_t tileBudget = ddbt::cog::tiler::TileCacheBudget();
    const size_t blockBudget = ddbt::cog::tiler::BlockCacheBudget();
    const size_t nodeBudget = ddbt::copc::cache::NodeCache::Global().BudgetBytes();
    const std::string projDir = ddbt::crs::ProjDataDir();
    ASSERT_EQ(ddbtConfigure(R"({"copcTileMaxPoints": 100000, "copcTileMaxNodePoints": 0, "copcTileCacheMb": 11})"),
              DDBT_ERR_NONE);
    const json::Value c = GetConfig();
    ASSERT_TRUE(c.IsObject());
    for (const char* key : {"projDataDir", "threads", "copcTilerMode", "blockCacheMb", "tileCacheMb", "copcNodeCacheMb",
                            "copcTileCacheMb", "copcBuildMemMb", "copcTileMaxPoints", "copcTileMaxNodePoints"})
        EXPECT_NE(c.Find(key), nullptr) << key;
    EXPECT_EQ(c.obj.size(), 10u);
    EXPECT_EQ(c.Find("copcTileMaxPoints")->u, 100000u);
    EXPECT_EQ(c.Find("copcTileMaxNodePoints")->u, 0u);
    EXPECT_EQ(c.Find("copcTileCacheMb")->u, 11u);
    EXPECT_EQ(c.Find("copcTilerMode")->str, ddbt::copc::tiler::ModeName(ddbt::copc::tiler::GetMode()));
    EXPECT_EQ(c.Find("threads")->u, ddbt::exec::DefaultPoolThreads());
    // What it gives is what ddbtConfigure takes (threads aside: the pool may run already)
    json::Value back = c;
    back.obj.erase("threads");
    ASSERT_EQ(ddbtConfigure(json::Dump(back, 0).c_str()), DDBT_ERR_NONE) << ddbtLastError();
    back.obj.erase("projDataDir");  // "" and a folder both read back as they are
    json::Value again = GetConfig();
    again.obj.erase("threads");
    again.obj.erase("projDataDir");
    EXPECT_EQ(json::Dump(again, 0), json::Dump(back, 0));
    ddbt::copc::tiler::SetTileMaxPoints(maxPoints);
    ddbt::copc::tiler::SetTileMaxNodePoints(maxNodePoints);
    ddbt::copc::tiler::SetTileCacheBudget(copcTileBudget);
    ddbt::cog::tiler::SetTileCacheBudget(tileBudget);
    ddbt::cog::tiler::SetBlockCacheBudget(blockBudget);
    ddbt::copc::cache::NodeCache::Global().SetBudget(nodeBudget);
    ddbt::copc::build::SetDefaultCopcMemoryBudget(0);
    ddbt::crs::SetProjDataDir(projDir);
    EXPECT_EQ(ddbtGetConfig(nullptr), DDBT_ERR_INVALIDARGS);
}

TEST(HostApi, ConfigureRefusesBadSettingsAndChangesNothing) {
    const auto mode = ddbt::copc::tiler::GetMode();
    const size_t tileBudget = ddbt::cog::tiler::TileCacheBudget();
    for (const char* bad : {"", "[1]", "{\"bogus\": 1}", "{\"tileCacheMb\": -1}", "{\"tileCacheMb\": 1.5}",
                            "{\"tileCacheMb\": \"3\"}", "{\"copcTilerMode\": \"fancy\"}", "{\"projDataDir\": 3}",
                            "{\"threads\": 5000}", "{\"copcBuildMemMb\": 99999999999}",
                            "{\"copcTileMaxPoints\": -1}", "{\"copcTileMaxNodePoints\": 1099511627777}",
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

// ddbtEncodeImage gives the bytes of the tile encoders for the same image and options, and refuses
// bad arguments with DDBT_ERR_INVALIDARGS and a message.
TEST(HostApi, EncodeImageGivesTheBytesOfTheTileEncoders) {
    namespace encode = ddbt::encode;
    const uint32_t w = 37, h = 23;
    std::vector<uint8_t> rgba(size_t{w} * h * 4);
    for (size_t i = 0; i < rgba.size(); ++i) rgba[i] = static_cast<uint8_t>(i * 7 + i / 5);
    for (size_t p = 0; p < size_t{w} * h; ++p) rgba[p * 4 + 3] = p % 3 == 0 ? 0 : 255;
    std::vector<uint8_t> opaque = rgba;
    for (size_t p = 0; p < size_t{w} * h; ++p) opaque[p * 4 + 3] = 255;
    const std::vector<uint8_t> clear(size_t{w} * h * 4, 0);
    const uint8_t palette[] = {0, 0, 0, 0, 255, 0, 0, 255, 0, 255, 0, 128};
    std::vector<uint8_t> indices(size_t{w} * h);
    for (size_t i = 0; i < indices.size(); ++i) indices[i] = static_cast<uint8_t>(i % 3);

    struct Case {
        const uint8_t* pixels;
        uint32_t channels;
        const uint8_t* palette;
        uint32_t paletteSize;
    };
    const Case cases[] = {{rgba.data(), 4, nullptr, 0}, {opaque.data(), 4, nullptr, 0},
                          {clear.data(), 4, nullptr, 0}, {rgba.data(), 3, nullptr, 0},
                          {rgba.data(), 2, nullptr, 0},  {rgba.data(), 1, nullptr, 0},
                          {indices.data(), 1, palette, 3}};
    for (const char* format : {"png", "jpeg", "webp", "image/webp"}) {
        for (const int quality : {0, 60}) {
            for (const Case& c : cases) {
                if (c.palette != nullptr && std::string(format) != "png") continue;
                std::vector<encode::PaletteEntry> entries;
                for (uint32_t i = 0; i < c.paletteSize; ++i)
                    entries.push_back(
                        {c.palette[4 * i], c.palette[4 * i + 1], c.palette[4 * i + 2], c.palette[4 * i + 3]});
                encode::TileImage view = encode::TileImage::Packed(c.pixels, w, h, c.channels);
                view.palette = entries.empty() ? nullptr : entries.data();
                view.paletteSize = c.paletteSize;
                encode::EncodeOptions options;
                options.format = *encode::ParseTileFormat(format);
                options.quality = quality;
                const encode::ByteBuffer expected = encode::EncodeTile(view, options);

                const ddbtImage image{c.pixels, w, h, c.channels, 0, c.palette, c.paletteSize};
                uint8_t* out = nullptr;
                int size = 0;
                ASSERT_EQ(ddbtEncodeImage(&image, format, quality, &out, &size), DDBT_ERR_NONE)
                    << ddbtLastError();
                EXPECT_STREQ(ddbtLastError(), "");
                ASSERT_EQ(static_cast<size_t>(size), expected.size()) << format << " " << c.channels;
                EXPECT_EQ(std::memcmp(out, expected.data(), expected.size()), 0)
                    << format << " " << c.channels;
                ddbtFree(out);
            }
        }
    }

    // A stride wider than a row reads the same pixels
    std::vector<uint8_t> padded(size_t{w + 3} * 4 * h, 0xAB);
    for (uint32_t y = 0; y < h; ++y)
        std::memcpy(padded.data() + size_t{y} * (w + 3) * 4, rgba.data() + size_t{y} * w * 4, size_t{w} * 4);
    const ddbtImage tight{rgba.data(), w, h, 4, 0, nullptr, 0};
    const ddbtImage strided{padded.data(), w, h, 4, size_t{w + 3} * 4, nullptr, 0};
    uint8_t *a = nullptr, *b = nullptr;
    int sa = 0, sb = 0;
    ASSERT_EQ(ddbtEncodeImage(&tight, "png", 0, &a, &sa), DDBT_ERR_NONE);
    ASSERT_EQ(ddbtEncodeImage(&strided, nullptr, 0, &b, &sb), DDBT_ERR_NONE);
    ASSERT_EQ(sa, sb);
    EXPECT_EQ(std::memcmp(a, b, static_cast<size_t>(sa)), 0);
    ddbtFree(a);
    ddbtFree(b);

    // Bad arguments
    uint8_t* out = nullptr;
    int size = 0;
    const auto refused = [&](const ddbtImage* image, const char* format, int quality, const char* message) {
        EXPECT_EQ(ddbtEncodeImage(image, format, quality, &out, &size), DDBT_ERR_INVALIDARGS) << message;
        EXPECT_NE(std::string(ddbtLastError()).find(message), std::string::npos) << ddbtLastError();
        EXPECT_EQ(out, nullptr);
    };
    refused(nullptr, "png", 0, "Invalid image: NULL");
    refused(&tight, "gif", 0, "Unsupported image format: gif");
    refused(&tight, "jpeg", 101, "Invalid quality: 101");
    refused(&tight, "jpeg", -1, "Invalid quality: -1");
    const ddbtImage noPixels{nullptr, w, h, 4, 0, nullptr, 0};
    refused(&noPixels, "png", 0, "Invalid image: image has no pixels");
    const ddbtImage empty{rgba.data(), 0, h, 4, 0, nullptr, 0};
    refused(&empty, "png", 0, "Invalid image: image has an empty size");
    const ddbtImage fiveChannels{rgba.data(), w, h, 5, 0, nullptr, 0};
    refused(&fiveChannels, "png", 0, "Invalid image: image must have 1 to 4 channels");
    const ddbtImage shortStride{rgba.data(), w, h, 4, size_t{w} * 4 - 1, nullptr, 0};
    refused(&shortStride, "png", 0, "Invalid image: image stride is shorter than a row");
    const ddbtImage paletteRgb{rgba.data(), w, h, 3, 0, palette, 3};
    refused(&paletteRgb, "png", 0, "Invalid image: a palette image must have 1 channel");
    const ddbtImage noEntries{indices.data(), w, h, 1, 0, palette, 0};
    refused(&noEntries, "png", 0, "Invalid palette: 0 entries");
    const ddbtImage sizeWithoutPalette{indices.data(), w, h, 1, 0, nullptr, 3};
    refused(&sizeWithoutPalette, "png", 0, "Invalid palette: NULL with 3 entries");
    EXPECT_EQ(ddbtEncodeImage(&tight, "png", 0, nullptr, &size), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtEncodeImage(&tight, "png", 0, &out, nullptr), DDBT_ERR_INVALIDARGS);
}

// The phase of every report is the one DroneDB's cogPhaseOfFraction gives its fraction (with a warp:
// warping < 0.1875 <= overviews < 0.25 <= compressing < 0.5 <= statistics), so a host that names the
// phase from the fraction names the same one. The 14 rows of blocks of the warped raster report
// fractions on both sides of every boundary.
TEST(HostApi, BuildCog2NamesThePhaseOfTheFractionItReports) {
    namespace syn = ddbt::cog::tiler::testing;
    const auto dir = ApiScratch("cog2-phases");
    const std::string in = (dir / "in.tif").string(), out = (dir / "out.tif").string();
    syn::SynthSpec spec = syn::Spec(4096, 1, 1);
    spec.tile = 256;
    syn::WriteSynth(in, spec);
    ProgressLog log;
    ASSERT_EQ(ddbtBuildCog2(in.c_str(), out.c_str(), nullptr, &ProgressLog::Record, &log), DDBT_ERR_NONE)
        << ddbtLastError();
    ASSERT_GE(log.calls.size(), 8u);
    // The raster is warped: its first reports are "warping"
    ASSERT_EQ(log.calls.front().second, "warping");
    for (const auto& [f, phase] : log.calls) {
        const char* expected = f < 0.1875 ? "warping"
                               : f < 0.25 ? "overviews"
                               : f < 0.5  ? "compressing"
                                          : "statistics";
        EXPECT_EQ(phase, expected) << "fraction " << f;
    }
    std::filesystem::remove_all(dir);
}

// ddbtIsOptimizedCog: a COG the builder wrote (EPSG:3857, 256 px tiles, overviews) is optimized and is
// encoded again on its own grid; the same raster without overviews, or in strips, is not.
TEST(HostApi, IsOptimizedCogIsTheRuleOfTheBuilder) {
    namespace syn = ddbt::cog::tiler::testing;
    const auto dir = ApiScratch("is-optimized-cog");
    const std::string tiled = (dir / "tiled.tif").string(), stripped = (dir / "stripped.tif").string();
    const std::string cog = (dir / "cog.tif").string(), text = (dir / "text.tif").string();
    syn::SynthSpec spec = syn::Spec(1024, 3, 1);
    spec.tile = 256;
    syn::WriteSynth(tiled, spec);
    spec.tile = 0;
    syn::WriteSynth(stripped, spec);
    ASSERT_EQ(ddbtBuildCog(tiled.c_str(), cog.c_str(), nullptr), DDBT_ERR_NONE) << ddbtLastError();
    std::ofstream(text) << "not a raster";

    bool optimized = true;
    ASSERT_EQ(ddbtIsOptimizedCog(cog.c_str(), &optimized), DDBT_ERR_NONE) << ddbtLastError();
    EXPECT_TRUE(optimized);
    ASSERT_EQ(ddbtIsOptimizedCog(tiled.c_str(), &optimized), DDBT_ERR_NONE) << ddbtLastError();
    EXPECT_FALSE(optimized) << "no overviews";
    ASSERT_EQ(ddbtIsOptimizedCog(stripped.c_str(), &optimized), DDBT_ERR_NONE) << ddbtLastError();
    EXPECT_FALSE(optimized) << "in strips";

    optimized = true;
    EXPECT_EQ(ddbtIsOptimizedCog(text.c_str(), &optimized), DDBT_ERR_EXCEPTION) << ddbtLastError();
    EXPECT_FALSE(optimized);
    EXPECT_EQ(ddbtIsOptimizedCog((dir / "missing.tif").string().c_str(), &optimized), DDBT_ERR_NOTFOUND);
    EXPECT_EQ(ddbtIsOptimizedCog("", &optimized), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtIsOptimizedCog(nullptr, &optimized), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtIsOptimizedCog(cog.c_str(), nullptr), DDBT_ERR_INVALIDARGS);
    std::filesystem::remove_all(dir);
}

namespace {

// One descriptor of a LAS extra-bytes VLR (192 bytes): data type, name, and the rest zero.
void PutExtraBytesDescriptor(std::vector<uint8_t>& vlr, uint8_t dataType, const std::string& name) {
    ddbt::copc::testing::SynthExtraBytes e;
    e.dataType = dataType;
    if (dataType == 0) e.options = 2;  // an undocumented type: `options` is its size, 2 bytes
    e.name = name;
    ddbt::copc::testing::AppendExtraBytesDescriptor(vlr, e);
}

// The summary of ddbtPointCloudInfo, parsed.
ddbt::cog::tiler::json::Value PointCloudInfoOf(const std::string& path, bool readBounds = true) {
    char* out = nullptr;
    EXPECT_EQ(ddbtPointCloudInfo(path.c_str(), readBounds, &out), DDBT_ERR_NONE) << ddbtLastError();
    if (out == nullptr) return {};
    auto v = ddbt::cog::tiler::json::Parse(out);
    ddbtFree(out);
    return v ? *v : ddbt::cog::tiler::json::Value{};
}

std::vector<std::string> StringsOf(const ddbt::cog::tiler::json::Value* array) {
    std::vector<std::string> out;
    if (array != nullptr)
        for (const auto& v : array->arr) out.push_back(v.str);
    return out;
}

}  // namespace

// ddbtPointCloudInfo reports what PDAL reports for DroneDB's index: the count and bounds of the
// header of a LAS / LAZ file, its WKT record as stored, and PDAL's dimension names (the extra bytes
// of a LAS 1.4 file only, an undocumented type left out, an array as name0, name1..., invalid
// characters as '_', a name equal to a dimension of the format merged into it).
TEST(HostApi, PointCloudInfoOfALasFileIsWhatPdalReports) {
    namespace syn = ddbt::copc::testing;
    const auto dir = ApiScratch("pcinfo-las");
    std::vector<uint8_t> vlr;
    PutExtraBytesDescriptor(vlr, 9, "Pulse width");   // f32
    PutExtraBytesDescriptor(vlr, 0, "opaque");        // undocumented: no dimension
    PutExtraBytesDescriptor(vlr, 13, "normal");       // deprecated 2-element u16 array
    PutExtraBytesDescriptor(vlr, 1, "1st/value");     // invalid characters
    PutExtraBytesDescriptor(vlr, 3, "intensity");     // the format's Intensity

    for (const int minor : {2, 4}) {
        SCOPED_TRACE(minor);
        syn::SynthLas las;
        las.minor = minor;
        las.format = minor == 4 ? 8 : 3;
        // Sizes: f32 4, the 2 undocumented bytes, u16[2] 4, u8 1, u16 2
        las.extraBytes = 4 + 2 + 4 + 1 + 2;
        las.extraBytesVlr = vlr;
        const std::string wkt =
            "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563]],"
            "PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433]]";
        las.wkt = minor == 4 ? wkt : std::string();
        syn::FillRandom(las, 2000, 7);
        const std::string path = (dir / ("v" + std::to_string(minor) + ".las")).string();
        ASSERT_TRUE(syn::WriteBytes(path, syn::BuildLasBytes(las)));

        const auto info = PointCloudInfoOf(path);
        ASSERT_TRUE(info.IsObject());
        EXPECT_EQ(info.Find("kind")->str, "las");
        EXPECT_EQ(info.Find("pointCount")->u, 2000u);
        ASSERT_NE(info.Find("bounds"), nullptr);
        EXPECT_EQ(info.Find("bounds")->arr.size(), 6u);
        std::vector<std::string> expected = {
            "X",        "Y",        "Z",       "Intensity",     "ReturnNumber", "NumberOfReturns",
            "ScanDirectionFlag",    "EdgeOfFlightLine",         "Classification",
            "Synthetic", "KeyPoint", "Withheld", "Overlap",     "ScanAngleRank", "UserData",
            "PointSourceId",        "GpsTime"};
        if (minor == 4) {
            expected.push_back("ScanChannel");
            expected.insert(expected.end(), {"Red", "Green", "Blue", "Infrared"});
            expected.insert(expected.end(), {"Pulse width", "normal0", "normal1", "_st_value"});
            EXPECT_EQ(info.Find("wkt")->str, las.wkt) << "the WKT record as stored";
        } else {
            expected.insert(expected.end(), {"Red", "Green", "Blue"});
            EXPECT_EQ(info.Find("wkt")->str, "");
        }
        EXPECT_EQ(StringsOf(info.Find("dimensions")), expected);
    }
    std::filesystem::remove_all(dir);
}

// A PLY file: its vertex properties as the dimensions, and the exact count and bounds of a read of
// its points; text files and errors
TEST(HostApi, PointCloudInfoOfPlyAndTextFilesReadsTheirPoints) {
    const auto dir = ApiScratch("pcinfo-ply");
    const std::string ply = (dir / "cloud.ply").string(), xyz = (dir / "cloud.xyz").string();
    {
        std::ofstream out(ply);
        out << "ply\nformat ascii 1.0\nelement vertex 3\n"
               "property float x\nproperty float y\nproperty float z\n"
               "property float nx\nproperty uchar diffuse_red\nproperty uchar views\nend_header\n"
               "1 2 3 0 10 1\n-4 5 6 0 20 2\n7 -8 9.5 0 30 3\n";
    }
    {
        std::ofstream out(xyz);
        out << "1 2 3\n4 5 6\n-1 0 2\n10 11 -12\n";
    }
    const auto p = PointCloudInfoOf(ply);
    EXPECT_EQ(p.Find("kind")->str, "ply");
    EXPECT_EQ(p.Find("pointCount")->u, 3u);
    EXPECT_EQ(StringsOf(p.Find("dimensions")),
              (std::vector<std::string>{"x", "y", "z", "nx", "diffuse_red", "views"}));
    std::vector<double> bounds;
    for (const auto& v : p.Find("bounds")->arr) bounds.push_back(v.num);
    EXPECT_EQ(bounds, (std::vector<double>{-4, -8, 3, 7, 5, 9.5}));

    const auto t = PointCloudInfoOf(xyz);
    EXPECT_EQ(t.Find("kind")->str, "xyz");
    EXPECT_EQ(t.Find("pointCount")->u, 4u);
    bounds.clear();
    for (const auto& v : t.Find("bounds")->arr) bounds.push_back(v.num);
    EXPECT_EQ(bounds, (std::vector<double>{-1, 0, -12, 10, 11, 6}));

    char* out = nullptr;
    EXPECT_EQ(ddbtPointCloudInfo((dir / "missing.las").string().c_str(), true, &out), DDBT_ERR_NOTFOUND);
    EXPECT_EQ(out, nullptr);
    EXPECT_EQ(ddbtPointCloudInfo("", true, &out), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtPointCloudInfo(ply.c_str(), true, nullptr), DDBT_ERR_INVALIDARGS);

    // Without readBounds, a PLY file (an exact count in its header) is not read: no bounds; a text
    // file is read for its count, so its bounds come with it
    EXPECT_EQ(PointCloudInfoOf(ply, false).Find("bounds"), nullptr);
    EXPECT_EQ(PointCloudInfoOf(ply, false).Find("pointCount")->u, 3u);
    EXPECT_NE(PointCloudInfoOf(xyz, false).Find("bounds"), nullptr);
    std::filesystem::remove_all(dir);
}

namespace {

// The points ddbtReadPoints hands over
struct ReadPoints {
    std::vector<double> xyz;
    std::vector<uint16_t> rgb;
    bool sawNoColour = false;
    uint64_t stopAfter = UINT64_MAX;
    static int Record(uint64_t count, const double* xyz, const uint16_t* rgb, void* user) {
        auto& self = *static_cast<ReadPoints*>(user);
        self.xyz.insert(self.xyz.end(), xyz, xyz + 3 * count);
        if (rgb != nullptr) self.rgb.insert(self.rgb.end(), rgb, rgb + 3 * count);
        else self.sawNoColour = true;
        return self.xyz.size() / 3 >= self.stopAfter ? 1 : 0;
    }
};

}  // namespace

// ddbtReadPoints hands over every point in file order: the coordinates of a LAS file as its records
// and scale and offset give them, its 16-bit colours as stored, the 8-bit colours of a PLY file
// scaled by 257, no colours for a file without; a callback that stops it makes the call CANCELED
TEST(HostApi, ReadPointsGivesEveryPointInFileOrder) {
    namespace syn = ddbt::copc::testing;
    const auto dir = ApiScratch("read-points");
    syn::SynthLas las;
    las.format = 3;
    syn::FillRandom(las, 700000, 11);  // several tasks of the reader
    const std::string lasPath = (dir / "cloud.las").string();
    ASSERT_TRUE(syn::WriteBytes(lasPath, syn::BuildLasBytes(las)));

    ReadPoints got;
    ASSERT_EQ(ddbtReadPoints(lasPath.c_str(), &ReadPoints::Record, &got), DDBT_ERR_NONE) << ddbtLastError();
    ASSERT_EQ(got.xyz.size(), 3 * las.count);
    ASSERT_EQ(got.rgb.size(), 3 * las.count);
    EXPECT_FALSE(got.sawNoColour);
    const auto layout = ddbt::copc::las::LayoutOf(las.format);
    for (uint64_t i = 0; i < las.count; ++i) {
        const uint8_t* r = las.records.data() + i * las.RecordSize();
        for (size_t a = 0; a < 3; ++a) {
            int32_t v;
            std::memcpy(&v, r + 4 * a, 4);
            ASSERT_EQ(got.xyz[3 * i + a], v * las.scale[a] + las.offset[a]) << "point " << i;
            uint16_t c;
            std::memcpy(&c, r + layout.rgb + 2 * a, 2);
            ASSERT_EQ(got.rgb[3 * i + a], c) << "point " << i;
        }
    }

    const std::string ply = (dir / "cloud.ply").string(), xyz = (dir / "cloud.xyz").string();
    {
        std::ofstream out(ply);
        out << "ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\n"
               "property float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n"
               "end_header\n1 2 3 10 20 255\n-4 5 6 0 1 2\n";
    }
    std::ofstream(xyz) << "1 2 3\n4 5 6\n";
    ReadPoints p;
    ASSERT_EQ(ddbtReadPoints(ply.c_str(), &ReadPoints::Record, &p), DDBT_ERR_NONE) << ddbtLastError();
    EXPECT_EQ(p.xyz, (std::vector<double>{1, 2, 3, -4, 5, 6}));
    EXPECT_EQ(p.rgb, (std::vector<uint16_t>{10 * 257, 20 * 257, 255 * 257, 0, 1 * 257, 2 * 257}));
    ReadPoints t;
    ASSERT_EQ(ddbtReadPoints(xyz.c_str(), &ReadPoints::Record, &t), DDBT_ERR_NONE) << ddbtLastError();
    EXPECT_EQ(t.xyz, (std::vector<double>{1, 2, 3, 4, 5, 6}));
    EXPECT_TRUE(t.sawNoColour);
    EXPECT_TRUE(t.rgb.empty());

    ReadPoints stopped;
    stopped.stopAfter = 1;
    EXPECT_EQ(ddbtReadPoints(lasPath.c_str(), &ReadPoints::Record, &stopped), DDBT_ERR_CANCELED);
    EXPECT_LT(stopped.xyz.size() / 3, las.count);
    EXPECT_EQ(ddbtReadPoints((dir / "missing.las").string().c_str(), &ReadPoints::Record, &stopped),
              DDBT_ERR_NOTFOUND);
    EXPECT_EQ(ddbtReadPoints(lasPath.c_str(), nullptr, nullptr), DDBT_ERR_INVALIDARGS);
    std::filesystem::remove_all(dir);
}

namespace {

// A raster opened with ddbtRasterOpen, closed when it goes
struct OpenRaster {
    ddbtRaster* raster = nullptr;
    explicit OpenRaster(const std::string& path) {
        EXPECT_EQ(ddbtRasterOpen(path.c_str(), &raster), DDBT_ERR_NONE) << ddbtLastError();
    }
    ~OpenRaster() { ddbtRasterClose(raster); }
    ddbt::cog::tiler::json::Value Info() const {
        char* out = nullptr;
        EXPECT_EQ(ddbtRasterInfo(raster, &out), DDBT_ERR_NONE) << ddbtLastError();
        auto v = ddbt::cog::tiler::json::Parse(out != nullptr ? out : "{}");
        ddbtFree(out);
        return v ? *v : ddbt::cog::tiler::json::Value{};
    }
};

}  // namespace

// ddbtRasterInfo reports the size, type, blocks, bands and georeferencing of a raster as GDAL does
TEST(HostApi, RasterInfoReportsTheFile) {
    namespace syn = ddbt::cog::tiler::testing;
    const auto dir = ApiScratch("raster-info");
    const std::string path = (dir / "r.tif").string();
    syn::SynthSpec spec = syn::Spec(300, 3, 4);
    spec.tile = 128;
    spec.nodata = "-9999";
    syn::WriteSynth(path, spec);
    OpenRaster r(path);
    ASSERT_NE(r.raster, nullptr);
    const auto info = r.Info();
    EXPECT_EQ(info.Find("width")->u, 300u);
    EXPECT_EQ(info.Find("height")->u, 300u);
    EXPECT_EQ(info.Find("type")->str, "Float32");
    EXPECT_TRUE(info.Find("tiled")->b);
    EXPECT_EQ(info.Find("blockWidth")->u, 128u);
    EXPECT_EQ(info.Find("bands")->arr.size(), 3u);
    EXPECT_EQ(info.Find("bands")->arr[0].Find("nodata")->num, -9999.0);
    EXPECT_EQ(info.Find("mask")->str, "nodata");
    ASSERT_NE(info.Find("geoTransform"), nullptr);
    EXPECT_EQ(info.Find("geoTransform")->arr[0].num, syn::kOriginX);
    EXPECT_EQ(info.Find("geoTransform")->arr[1].num, syn::kPixel);
    ASSERT_NE(info.Find("wkt"), nullptr);
    EXPECT_NE(info.Find("wkt")->str.find("Pseudo-Mercator"), std::string::npos) << info.Find("wkt")->str;
    std::filesystem::remove_all(dir);
}

namespace {

// The descriptors of this process open on `path` (Linux: /proc/self/fd; -1 where it cannot be told)
int DescriptorsOn(const std::filesystem::path& path) {
#if defined(__linux__)
    std::error_code ec;
    const auto target = std::filesystem::canonical(path, ec);
    if (ec) return -1;
    int n = 0;
    for (const auto& fd : std::filesystem::directory_iterator("/proc/self/fd", ec)) {
        std::error_code lec;
        if (std::filesystem::read_symlink(fd.path(), lec) == target) ++n;
    }
    return ec ? -1 : n;
#else
    (void)path;
    return -1;
#endif
}

}  // namespace

// A raster opened privately is closed with its handle, so the host can rename the file it checked
// (Windows renames no file a process holds open); a shared open keeps the file for the next call
TEST(HostApi, APrivateRasterClosesItsFileWithItsHandle) {
    namespace syn = ddbt::cog::tiler::testing;
    const auto dir = ApiScratch("raster-private");
    const auto shared = dir / "shared.tif", checked = dir / "checked.tif";
    syn::WriteSynth(shared.string(), syn::Spec(64, 1, 1));
    syn::WriteSynth(checked.string(), syn::Spec(64, 1, 1));

    ddbtRaster* raster = nullptr;
    ASSERT_EQ(ddbtRasterOpenEx(checked.string().c_str(), DDBT_RASTER_OPEN_PRIVATE, &raster), DDBT_ERR_NONE)
        << ddbtLastError();
    char* json = nullptr;
    ASSERT_EQ(ddbtRasterInfo(raster, &json), DDBT_ERR_NONE) << ddbtLastError();
    ddbtFree(json);
    if (DescriptorsOn(checked) >= 0) {
        EXPECT_GT(DescriptorsOn(checked), 0) << "the open handle reads the file";
    }
    ddbtRasterClose(raster);
    if (DescriptorsOn(checked) >= 0) {
        EXPECT_EQ(DescriptorsOn(checked), 0);
    }
    std::error_code ec;
    std::filesystem::rename(checked, dir / "published.tif", ec);
    EXPECT_FALSE(ec) << ec.message();

    // The shared open: the file stays open after the handle, until the sources are released
    ASSERT_EQ(ddbtRasterOpenEx(shared.string().c_str(), 0, &raster), DDBT_ERR_NONE) << ddbtLastError();
    ddbtRasterClose(raster);
    if (DescriptorsOn(shared) >= 0) {
        EXPECT_GT(DescriptorsOn(shared), 0);
    }
    ddbtReleaseSources();
    if (DescriptorsOn(shared) >= 0) {
        EXPECT_EQ(DescriptorsOn(shared), 0);
    }

    EXPECT_EQ(ddbtRasterOpenEx(shared.string().c_str(), 0x2u, &raster), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(raster, nullptr);
    std::filesystem::remove_all(dir);
}

// ddbtRasterRead reads a window as GDAL's RasterIO with nearest resampling: every sample at full
// resolution, and for a reduction the source pixel GDAL takes (2 i + 1 for a factor of 2)
TEST(HostApi, RasterReadReadsAWindowAsGdal) {
    namespace syn = ddbt::cog::tiler::testing;
    const auto dir = ApiScratch("raster-read");
    const std::string path = (dir / "r.tif").string();
    syn::SynthSpec spec = syn::Spec(200, 2, 4);
    spec.tile = 64;
    spec.value = [](int band0, uint32_t x, uint32_t y) {
        return static_cast<float>(static_cast<uint32_t>(band0) * 100000u + y * 1000u + x);
    };
    syn::WriteSynth(path, spec);
    OpenRaster r(path);
    ASSERT_NE(r.raster, nullptr);
    const int bands[2] = {2, 1};

    std::vector<float> full(2 * 50 * 40);
    std::vector<uint8_t> mask(50 * 40, 0);
    ASSERT_EQ(ddbtRasterRead(r.raster, 30, 70, 50, 40, 50, 40, bands, 2, DDBT_SAMPLE_FLOAT32, full.data(), mask.data()),
              DDBT_ERR_NONE)
        << ddbtLastError();
    for (uint32_t y = 0; y < 40; ++y)
        for (uint32_t x = 0; x < 50; ++x) {
            ASSERT_EQ(full[y * 50 + x], spec.value(1, 30 + x, 70 + y)) << x << "," << y;
            ASSERT_EQ(full[2000 + y * 50 + x], spec.value(0, 30 + x, 70 + y)) << x << "," << y;
        }
    EXPECT_TRUE(std::all_of(mask.begin(), mask.end(), [](uint8_t m) { return m == 255; })) << "no mask: 255";

    std::vector<double> half(25 * 20);
    ASSERT_EQ(ddbtRasterRead(r.raster, 30, 70, 50, 40, 25, 20, bands + 1, 1, DDBT_SAMPLE_FLOAT64, half.data(), nullptr),
              DDBT_ERR_NONE)
        << ddbtLastError();
    for (uint32_t y = 0; y < 20; ++y)
        for (uint32_t x = 0; x < 25; ++x)
            ASSERT_EQ(half[y * 25 + x], spec.value(0, 30 + 2 * x + 1, 70 + 2 * y + 1)) << x << "," << y;

    std::vector<float> native(4);
    ASSERT_EQ(ddbtRasterRead(r.raster, 199, 199, 1, 1, 1, 1, bands + 1, 1, DDBT_SAMPLE_NATIVE, native.data(), nullptr),
              DDBT_ERR_NONE);
    EXPECT_EQ(native[0], spec.value(0, 199, 199));

    std::vector<float> any(64);
    EXPECT_EQ(ddbtRasterRead(r.raster, 190, 0, 20, 1, 20, 1, bands, 1, DDBT_SAMPLE_FLOAT32, any.data(), nullptr),
              DDBT_ERR_INVALIDARGS) << "a window past the right edge";
    EXPECT_EQ(ddbtRasterRead(r.raster, -1, 0, 2, 1, 2, 1, bands, 1, DDBT_SAMPLE_FLOAT32, any.data(), nullptr),
              DDBT_ERR_INVALIDARGS);
    const int badBand = 3;
    EXPECT_EQ(ddbtRasterRead(r.raster, 0, 0, 2, 1, 2, 1, &badBand, 1, DDBT_SAMPLE_FLOAT32, any.data(), nullptr),
              DDBT_ERR_INVALIDARGS);
    std::filesystem::remove_all(dir);
}

// ddbtRasterReadLevel reads the pixels of a level one for one: level 0 is the raster itself; a level
// the raster has not and a window outside the level are refused
TEST(HostApi, RasterReadLevelReadsTheLevelPixelForPixel) {
    namespace syn = ddbt::cog::tiler::testing;
    const auto dir = ApiScratch("raster-read-level");
    const std::string path = (dir / "r.tif").string();
    syn::SynthSpec spec = syn::Spec(120, 2, 4);
    spec.tile = 32;
    spec.value = [](int band0, uint32_t x, uint32_t y) {
        return static_cast<float>(static_cast<uint32_t>(band0) * 100000u + y * 1000u + x);
    };
    syn::WriteSynth(path, spec);
    OpenRaster r(path);
    ASSERT_NE(r.raster, nullptr);
    const int bands[2] = {2, 1};
    std::vector<double> got(2 * 30 * 20);
    std::vector<uint8_t> mask(30 * 20, 0);
    ASSERT_EQ(ddbtRasterReadLevel(r.raster, 0, 85, 95, 30, 20, bands, 2, DDBT_SAMPLE_FLOAT64, got.data(), mask.data()),
              DDBT_ERR_NONE)
        << ddbtLastError();
    for (uint32_t y = 0; y < 20; ++y)
        for (uint32_t x = 0; x < 30; ++x) {
            ASSERT_EQ(got[y * 30 + x], spec.value(1, 85 + x, 95 + y)) << x << "," << y;
            ASSERT_EQ(got[600 + y * 30 + x], spec.value(0, 85 + x, 95 + y)) << x << "," << y;
        }
    EXPECT_TRUE(std::all_of(mask.begin(), mask.end(), [](uint8_t m) { return m == 255; })) << "no mask: 255";

    EXPECT_EQ(ddbtRasterReadLevel(r.raster, 1, 0, 0, 1, 1, bands, 1, DDBT_SAMPLE_FLOAT64, got.data(), nullptr),
              DDBT_ERR_INVALIDARGS) << "no overview";
    EXPECT_EQ(ddbtRasterReadLevel(r.raster, 0, 100, 0, 21, 1, bands, 1, DDBT_SAMPLE_FLOAT64, got.data(), nullptr),
              DDBT_ERR_INVALIDARGS) << "a window past the right edge";
    EXPECT_EQ(ddbtRasterReadLevel(r.raster, -1, 0, 0, 1, 1, bands, 1, DDBT_SAMPLE_FLOAT64, got.data(), nullptr),
              DDBT_ERR_INVALIDARGS);
    std::filesystem::remove_all(dir);
}

namespace {

// The RGBA of pixel (x, y) the test writes: the alpha 0 on a diagonal band
uint8_t WrittenSample(int band, int x, int y) {
    if (band == 3) return (x + y) % 7 == 0 ? 0 : 255;
    return static_cast<uint8_t>((x * 3 + y * 5 + band * 40) % 256);
}

int FillRgba(void*, int x, int y, int w, int h, uint8_t* out, size_t stride) {
    for (int r = 0; r < h; ++r)
        for (int c = 0; c < w; ++c)
            for (int b = 0; b < 4; ++b) out[static_cast<size_t>(r) * stride + static_cast<size_t>(c) * 4 + b] =
                WrittenSample(b, x + c, y + r);
    return 0;
}

int FillFails(void*, int, int, int, int, uint8_t*, size_t) { return 1; }

}  // namespace

// ddbtWriteRaster writes the pixels the host fills, an RGB + alpha GeoTIFF with the overviews of
// GDALBuildOverviews AVERAGE (the alpha masks the colours), in the CRS of another file
TEST(HostApi, WriteRasterWritesTheHostsPixelsAndTheirAverageOverviews) {
    namespace syn = ddbt::cog::tiler::testing;
    const auto dir = ApiScratch("write-raster");
    const std::string crsFile = (dir / "crs.tif").string();
    syn::WriteSynth(crsFile, syn::Spec(64, 1, 1));
    const std::string out = (dir / "out.tif").string();
    const std::string options = R"({"width": 300, "height": 201, "colorInterp": ["Red", "Green", "Blue", "Alpha"],
        "blockSize": 64, "overviewFactors": [2, 4], "geoTransform": [1000, 2, 0, 5000, 0, -2],
        "geoKeysFrom": )" + JsonString(crsFile) + "}";
    ASSERT_EQ(ddbtWriteRaster(out.c_str(), options.c_str(), FillRgba, nullptr, nullptr, nullptr), DDBT_ERR_NONE)
        << ddbtLastError();

    OpenRaster r(out);
    ASSERT_NE(r.raster, nullptr);
    const auto info = r.Info();
    EXPECT_EQ(info.Find("mask")->str, "alpha");
    ASSERT_EQ(info.Find("overviews")->arr.size(), 2u);
    EXPECT_EQ(info.Find("overviews")->arr[0].arr[0].u, 150u);
    EXPECT_EQ(info.Find("overviews")->arr[0].arr[1].u, 101u);  // ceil(201 / 2)
    EXPECT_EQ(info.Find("overviews")->arr[1].arr[1].u, 51u);
    EXPECT_EQ(info.Find("geoTransform")->arr[0].num, 1000.0);
    EXPECT_NE(info.Find("wkt")->str.find("Pseudo-Mercator"), std::string::npos);

    const int bands[4] = {1, 2, 3, 4};
    std::vector<uint8_t> full(300 * 201 * 4);
    ASSERT_EQ(ddbtRasterRead(r.raster, 0, 0, 300, 201, 300, 201, bands, 4, DDBT_SAMPLE_NATIVE, full.data(), nullptr),
              DDBT_ERR_NONE);
    for (int b = 0; b < 4; ++b)
        for (int y = 0; y < 201; ++y)
            for (int x = 0; x < 300; ++x)
                ASSERT_EQ(full[static_cast<size_t>(b) * 300 * 201 + static_cast<size_t>(y) * 300 + x],
                          WrittenSample(b, x, y)) << b << " " << x << "," << y;

    // Pixel (3, 4) of the first overview: the source 2 x 2 at (6, 8), whose (6, 8) is transparent:
    // the samples of the other three averaged, the alpha's too (255, since one of them is not 0)
    std::vector<uint8_t> level(150 * 101 * 4);
    ASSERT_EQ(ddbtRasterReadLevel(r.raster, 1, 0, 0, 150, 101, bands, 4, DDBT_SAMPLE_NATIVE, level.data(), nullptr),
              DDBT_ERR_NONE) << ddbtLastError();
    const auto at = [&](int b, int x, int y) { return level[static_cast<size_t>(b) * 150 * 101 + static_cast<size_t>(y) * 150 + x]; };
    for (int b = 0; b < 4; ++b) {
        double total = 0;
        int counted = 0;
        for (int y = 8; y < 10; ++y)
            for (int x = 6; x < 8; ++x)
                if (WrittenSample(3, x, y) != 0) {
                    total += WrittenSample(b, x, y);
                    ++counted;
                }
        ASSERT_EQ(counted, 3);
        EXPECT_EQ(at(b, 3, 4), static_cast<uint8_t>(total / counted + 0.5)) << "band " << b + 1;
    }
    EXPECT_EQ(at(3, 3, 4), 255);

    EXPECT_EQ(ddbtWriteRaster((dir / "failed.tif").string().c_str(), options.c_str(), FillFails, nullptr, nullptr, nullptr),
              DDBT_ERR_EXCEPTION);
    EXPECT_FALSE(std::filesystem::exists(dir / "failed.tif")) << "nothing is left of a write the host stopped";
    EXPECT_EQ(ddbtWriteRaster((dir / "bad.tif").string().c_str(), R"({"width": 10})", FillRgba, nullptr, nullptr, nullptr),
              DDBT_ERR_INVALIDARGS);
    std::filesystem::remove_all(dir);
}

namespace {

// The items of the metadata domain `name` of a ddbtRasterMetadata object (its "domains"), none when absent
std::vector<std::string> DomainItems(const ddbt::cog::tiler::json::Value& owner, const std::string& name) {
    std::vector<std::string> items;
    if (const auto* domains = owner.Find("domains"))
        for (const auto& d : domains->arr)
            if (d.Find("name")->str == name)
                for (const auto& i : d.Find("items")->arr) items.push_back(i.str);
    return items;
}

ddbt::cog::tiler::json::Value RasterMetadata(ddbtRaster* raster) {
    char* out = nullptr;
    EXPECT_EQ(ddbtRasterMetadata(raster, &out), DDBT_ERR_NONE) << ddbtLastError();
    auto v = ddbt::cog::tiler::json::Parse(out != nullptr ? out : "{}");
    ddbtFree(out);
    return v ? *v : ddbt::cog::tiler::json::Value{};
}

}  // namespace

// ddbtWriteRaster writes the metadata a host copies from its source where GTiff writes it, so a reader
// gets it back as GDAL reads it: the dataset items (a TIFFTAG_* item as its tag), the items, scale,
// offset and unit of each band (a scale of 1 and an offset of 0 write nothing), the XMP packet; values
// with XML characters come back as they were given
TEST(HostApi, WriteRasterWritesTheMetadataOfItsSourceWhereGtiffWritesIt) {
    const auto dir = ApiScratch("write-metadata");
    const std::string out = (dir / "out.tif").string();
    const std::string xmp = R"(<x:xmpmeta xmlns:x="adobe:ns:meta/"><rdf:RDF/></x:xmpmeta>)";
    const std::string options = R"({"width": 40, "height": 30, "colorInterp": ["Red", "Green", "Blue", "Alpha"],
        "blockSize": 16, "geoTransform": [1000, 2, 0, 5000, 0, -2], "crs": "EPSG:3857",
        "metadata": [["ZETA", "last <&> \"first\""], ["TIFFTAG_SOFTWARE", "a tool"], ["ALPHA", "1"]],
        "bandMetadata": [[["WAVELENGTH", "650"]], [], [["WAVELENGTH", "550"], ["FWHM", "40"]], []],
        "scales": [0.1, 1, 0.25, 1], "offsets": [-5, 0, 0, 0], "units": ["Celsius", "", "", ""],
        "descriptions": ["red & hot", "", "", ""], "xmp": )" + std::string("\"") +
                                R"(<x:xmpmeta xmlns:x=\"adobe:ns:meta/\"><rdf:RDF/></x:xmpmeta>")" + "}";
    ASSERT_EQ(ddbtWriteRaster(out.c_str(), options.c_str(), FillRgba, nullptr, nullptr, nullptr), DDBT_ERR_NONE)
        << ddbtLastError();

    OpenRaster r(out);
    ASSERT_NE(r.raster, nullptr);
    const auto info = r.Info();
    const auto& bands = info.Find("bands")->arr;
    ASSERT_EQ(bands.size(), 4u);
    EXPECT_EQ(bands[0].Find("scale")->num, 0.1);
    EXPECT_EQ(bands[0].Find("offset")->num, -5.0);
    EXPECT_EQ(bands[0].Find("unit")->str, "Celsius");
    EXPECT_EQ(bands[0].Find("description")->str, "red & hot");
    EXPECT_EQ(bands[1].Find("scale"), nullptr) << "a scale of 1 and an offset of 0 write nothing";
    EXPECT_EQ(bands[1].Find("unit"), nullptr);
    EXPECT_EQ(bands[2].Find("scale")->num, 0.25);
    EXPECT_EQ(bands[2].Find("offset")->num, 0.0) << "an offset of 0 is written with a scale that is not 1";

    const auto md = RasterMetadata(r.raster);
    // As GDAL lists them: the TIFF tags, the GDAL_METADATA items in their order, then the GeoKeys'
    EXPECT_EQ(DomainItems(md, ""), (std::vector<std::string>{"TIFFTAG_SOFTWARE=a tool", "ZETA=last <&> \"first\"",
                                                              "ALPHA=1", "AREA_OR_POINT=Area"}));
    EXPECT_EQ(DomainItems(md, "xml:XMP"), std::vector<std::string>{xmp});
    const auto& mdBands = md.Find("bands")->arr;
    ASSERT_EQ(mdBands.size(), 4u);
    EXPECT_EQ(DomainItems(mdBands[0], ""), std::vector<std::string>{"WAVELENGTH=650"});
    EXPECT_TRUE(DomainItems(mdBands[1], "").empty());
    EXPECT_EQ(DomainItems(mdBands[2], ""), (std::vector<std::string>{"WAVELENGTH=550", "FWHM=40"}));

    // A list of one entry per band names every band or none
    const std::string fewer = R"({"width": 8, "height": 8, "colorInterp": ["Gray", "Gray"], "scales": [2]})";
    EXPECT_EQ(ddbtWriteRaster((dir / "fewer.tif").string().c_str(), fewer.c_str(), FillRgba, nullptr, nullptr, nullptr),
              DDBT_ERR_INVALIDARGS);
    const std::string notPairs = R"({"width": 8, "height": 8, "colorInterp": ["Gray"], "metadata": [["A"]]})";
    EXPECT_EQ(ddbtWriteRaster((dir / "pairs.tif").string().c_str(), notPairs.c_str(), FillRgba, nullptr, nullptr,
                              nullptr),
              DDBT_ERR_INVALIDARGS);
    std::filesystem::remove_all(dir);
}

// A raster whose AREA_OR_POINT item is Point is written as GTiff writes it (PixelIsPoint, the origin at the
// centre of its first pixel), so a reader gets back the geotransform it was given and the item; GeoKeys
// copied from a PixelIsPoint file without that item are written PixelIsArea, as GTiff writes a raster
// created with that geotransform
TEST(HostApi, WriteRasterWritesThePixelIsPointOfItsAreaOrPointItem) {
    const auto dir = ApiScratch("write-point");
    const auto write = [&](const std::string& name, const std::string& extra) {
        const std::string out = (dir / name).string();
        const std::string options = R"({"width": 20, "height": 10, "colorInterp": ["Red", "Green", "Blue", "Alpha"],
            "blockSize": 16, "geoTransform": [1000, 2, 0, 5000, 0, -2])" + extra + "}";
        EXPECT_EQ(ddbtWriteRaster(out.c_str(), options.c_str(), FillRgba, nullptr, nullptr, nullptr), DDBT_ERR_NONE)
            << ddbtLastError();
        return out;
    };
    const auto check = [](const std::string& file, const std::string& areaOrPoint) {
        OpenRaster r(file);
        ASSERT_NE(r.raster, nullptr);
        const auto info = r.Info();
        const auto& gt = info.Find("geoTransform")->arr;
        EXPECT_EQ(gt[0].num, 1000.0);
        EXPECT_EQ(gt[3].num, 5000.0);
        const auto items = DomainItems(RasterMetadata(r.raster), "");
        EXPECT_NE(std::find(items.begin(), items.end(), "AREA_OR_POINT=" + areaOrPoint), items.end()) << file;
    };
    const std::string point = write("point.tif", R"(, "crs": "EPSG:3857", "metadata": [["AREA_OR_POINT", "Point"]])");
    check(point, "Point");
    check(write("from-point.tif", R"(, "geoKeysFrom": )" + JsonString(point)), "Area");
    check(write("from-point-kept.tif", R"(, "geoKeysFrom": )" + JsonString(point) + R"(, "metadata": [["area_or_point", "point"]])"),
          "Point");
    std::filesystem::remove_all(dir);
}

// A raster without georeferencing is opened for reading (a host reads the pixels of a photo copied to
// a GeoTIFF), and the tiler still refuses it
TEST(HostApi, RasterWithoutGeoreferencingIsReadNotTiled) {
    namespace syn = ddbt::cog::tiler::testing;
    const auto dir = ApiScratch("raster-plain");
    const std::string path = (dir / "plain.tif").string();
    syn::SynthSpec spec = syn::Spec(64, 1, 1);
    spec.georeferenced = false;
    spec.value = [](int, uint32_t x, uint32_t y) { return static_cast<float>((x + y) % 251); };
    syn::WriteSynth(path, spec);
    OpenRaster r(path);
    ASSERT_NE(r.raster, nullptr);
    const auto info = r.Info();
    EXPECT_EQ(info.Find("geoTransform"), nullptr);
    EXPECT_EQ(info.Find("wkt"), nullptr);
    EXPECT_EQ(info.Find("type")->str, "Byte");
    uint8_t v = 0;
    const int band = 1;
    ASSERT_EQ(ddbtRasterRead(r.raster, 10, 20, 1, 1, 1, 1, &band, 1, DDBT_SAMPLE_NATIVE, &v, nullptr), DDBT_ERR_NONE)
        << ddbtLastError();
    EXPECT_EQ(v, 30);
    char* json = nullptr;
    EXPECT_NE(ddbtTileInfo(path.c_str(), 256, &json), DDBT_ERR_NONE) << "the tiler needs georeferencing";
    ddbtFree(json);

    ddbtRaster* none = nullptr;
    EXPECT_EQ(ddbtRasterOpen((dir / "missing.tif").string().c_str(), &none), DDBT_ERR_NOTFOUND);
    EXPECT_EQ(none, nullptr);
    EXPECT_EQ(ddbtRasterOpen("", &none), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtRasterClose(nullptr), DDBT_ERR_NONE);
    std::filesystem::remove_all(dir);
}

namespace {

// The index of pixel (x, y) of a palette raster the test writes: areas of one class, other classes
// scattered among them
uint8_t PaletteIndex(int x, int y) {
    const uint32_t area = (static_cast<uint32_t>(x / 3) * 2654435761u) ^ (static_cast<uint32_t>(y / 2) * 40503u);
    const uint32_t pixel = (static_cast<uint32_t>(x) * 73856093u) ^ (static_cast<uint32_t>(y) * 19349663u);
    return static_cast<uint8_t>(((pixel >> 7) % 5 == 0 ? pixel >> 11 : area >> 13) % 48);
}

int FillPalette(void*, int x, int y, int w, int h, uint8_t* out, size_t stride) {
    for (int r = 0; r < h; ++r)
        for (int c = 0; c < w; ++c) out[static_cast<size_t>(r) * stride + static_cast<size_t>(c)] = PaletteIndex(x + c, y + r);
    return 0;
}

int FillGreys(void*, int x, int y, int w, int h, uint8_t* out, size_t stride) {
    for (int r = 0; r < h; ++r)
        for (int c = 0; c < w; ++c)
            out[static_cast<size_t>(r) * stride + static_cast<size_t>(c)] =
                static_cast<uint8_t>(((x + c) * 7 + (y + r) * 13 + ((x + c) * (y + r)) % 11) % 256);
    return 0;
}

// Band `band` of level `level` of a written raster, and its mask into `mask` when given
std::vector<uint8_t> ReadLevel(ddbtRaster* raster, int level, int w, int h, int band = 1,
                               std::vector<uint8_t>* mask = nullptr) {
    std::vector<uint8_t> px(static_cast<size_t>(w) * static_cast<size_t>(h));
    if (mask != nullptr) mask->resize(px.size());
    EXPECT_EQ(ddbtRasterReadLevel(raster, level, 0, 0, w, h, &band, 1, DDBT_SAMPLE_NATIVE, px.data(),
                                  mask != nullptr ? mask->data() : nullptr),
              DDBT_ERR_NONE)
        << ddbtLastError();
    return px;
}

// The 40 colours of the palettes the tests write, and their colour table as ddbtWriteRaster takes it
std::vector<std::array<uint8_t, 3>> TestColours() {
    std::vector<std::array<uint8_t, 3>> rgb;
    for (int i = 0; i < 40; ++i)
        rgb.push_back({static_cast<uint8_t>(i * 67 % 256), static_cast<uint8_t>((i * 131 + 40) % 256),
                       static_cast<uint8_t>((i * 29 + 7) % 256)});
    return rgb;
}

std::string TableJson(const std::vector<std::array<uint8_t, 3>>& rgb) {
    std::string table;
    for (size_t i = 0; i < rgb.size(); ++i)
        table += std::string(i > 0 ? ", " : "") + "[" + std::to_string(rgb[i][0]) + ", " + std::to_string(rgb[i][1]) +
                 ", " + std::to_string(rgb[i][2]) + ", 255]";
    return "[" + table + "]";
}

// The second sample of the pixels the tests write beside a palette band (an internal mask, an alpha band): 255 on
// diagonals 11 pixels apart and 0 elsewhere, so that an overview pixel is 255 or 0 by the rows its window keeps (a
// window that loses the rows above its chunk loses their 255)
uint8_t Opacity(int x, int y) { return (x + 3 * y) % 11 == 0 ? 255 : 0; }

// A palette index then an opacity per pixel
int FillPaletteAndOpacity(void*, int x, int y, int w, int h, uint8_t* out, size_t stride) {
    for (int r = 0; r < h; ++r)
        for (int c = 0; c < w; ++c) {
            uint8_t* p = out + static_cast<size_t>(r) * stride + static_cast<size_t>(c) * 2;
            p[0] = PaletteIndex(x + c, y + r);
            p[1] = Opacity(x + c, y + r);
        }
    return 0;
}

// Every level of `want` against the levels of a written raster (band `band`, or its mask)
void ExpectLevels(ddbtRaster* raster, int w, int h, const std::vector<int>& factors,
                  const std::vector<std::vector<uint8_t>>& want, int band, bool mask) {
    for (size_t l = 0; l < factors.size(); ++l) {
        const int lw = (w + factors[l] - 1) / factors[l], lh = (h + factors[l] - 1) / factors[l];
        std::vector<uint8_t> levelMask;
        const std::vector<uint8_t> level = ReadLevel(raster, static_cast<int>(l) + 1, lw, lh, band, mask ? &levelMask : nullptr);
        const std::vector<uint8_t>& got = mask ? levelMask : level;
        size_t differ = 0;
        for (size_t i = 0; i < got.size(); ++i) differ += got[i] != want[l][i];
        EXPECT_EQ(differ, 0u) << (mask ? "mask" : "band " + std::to_string(band)) << ", level " << l + 1;
    }
}

}  // namespace

// ddbtWriteRaster gives a palette band the overviews GDALBuildOverviews AVERAGE computes for it, sample
// for sample (tests/unit/gdal_palette_average.h): the colours of the pixels averaged and the nearest
// entry of the table GTiff reads back (40 colours, 216 black ones after them, the nodata entry
// transparent), each level from the one above in the chunks GDAL reads it in (10241 samples wide: 512
// rows, so some rows of level 1 lose the source rows above their chunk)
TEST(HostApi, WriteRasterAveragesAColourTableAsGdalAveragesAPaletteBand) {
    const auto dir = ApiScratch("write-palette");
    const int w = 10241, h = 701;
    const std::vector<int> factors{2, 4, 8};
    const std::vector<std::array<uint8_t, 3>> rgb = TestColours();
    const std::string out = (dir / "palette.tif").string();
    const std::string options = R"({"width": 10241, "height": 701, "colorInterp": ["Palette"], "colorTable": )" +
                                TableJson(rgb) + R"(, "nodata": 5, "blockSize": 256, "overviewFactors": [2, 4, 8]})";
    ASSERT_EQ(ddbtWriteRaster(out.c_str(), options.c_str(), FillPalette, nullptr, nullptr, nullptr), DDBT_ERR_NONE)
        << ddbtLastError();

    std::vector<uint8_t> src(static_cast<size_t>(w) * static_cast<size_t>(h));
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) src[static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x)] = PaletteIndex(x, y);
    const auto readBack = ddbt::test::GtiffReadBackTable(rgb, true, 5);
    const auto expected = ddbt::test::GdalPaletteOverviews(src, w, h, 256, factors, readBack, true, 5);

    OpenRaster r(out);
    ASSERT_NE(r.raster, nullptr);
    const auto info = r.Info();
    const auto* overviews = info.Find("overviews");
    ASSERT_NE(overviews, nullptr);
    ASSERT_EQ(overviews->arr.size(), factors.size());
    const auto* colours = info.Find("bands")->arr[0].Find("colorTable");
    ASSERT_NE(colours, nullptr);
    ASSERT_EQ(colours->arr.size(), 256u);
    EXPECT_EQ(colours->arr[5].arr[3].u, 0u) << "the nodata entry is transparent";
    EXPECT_EQ(ReadLevel(r.raster, 0, w, h), src);
    for (size_t l = 0; l < factors.size(); ++l) {
        const int lw = (w + factors[l] - 1) / factors[l], lh = (h + factors[l] - 1) / factors[l];
        ASSERT_EQ(overviews->arr[l].arr[0].u, static_cast<uint64_t>(lw));
        ASSERT_EQ(overviews->arr[l].arr[1].u, static_cast<uint64_t>(lh));
        const std::vector<uint8_t> level = ReadLevel(r.raster, static_cast<int>(l) + 1, lw, lh);
        size_t differ = 0;
        for (size_t i = 0; i < level.size(); ++i) differ += level[i] != expected[l][i];
        EXPECT_EQ(differ, 0u) << "level " << l + 1;
    }
    std::filesystem::remove_all(dir);
}

// A colour table GTiff reads back as the identity (every entry i the grey i, opaque) has the overviews
// of any 8-bit band (GTiff regenerates them as for a band without a table)
TEST(HostApi, WriteRasterAveragesAnIdentityTableAsAGreyBand) {
    const auto dir = ApiScratch("write-identity");
    std::string identity;
    for (int i = 0; i < 256; ++i)
        identity += std::string(i > 0 ? ", " : "") + "[" + std::to_string(i) + ", " + std::to_string(i) + ", " +
                    std::to_string(i) + "]";
    const std::string size = R"("width": 301, "height": 203, "blockSize": 64, "overviewFactors": [2, 4, 8])";
    const std::string palette = (dir / "palette.tif").string(), grey = (dir / "grey.tif").string();
    ASSERT_EQ(ddbtWriteRaster(palette.c_str(), (R"({"colorInterp": ["Palette"], "colorTable": [)" + identity + "], " + size + "}").c_str(),
                              FillGreys, nullptr, nullptr, nullptr),
              DDBT_ERR_NONE)
        << ddbtLastError();
    ASSERT_EQ(ddbtWriteRaster(grey.c_str(), (R"({"colorInterp": ["Gray"], )" + size + "}").c_str(), FillGreys, nullptr,
                              nullptr, nullptr),
              DDBT_ERR_NONE)
        << ddbtLastError();
    OpenRaster p(palette), g(grey);
    ASSERT_NE(p.raster, nullptr);
    ASSERT_NE(g.raster, nullptr);
    for (int l = 1; l <= 3; ++l) {
        const int f = 1 << l;
        EXPECT_EQ(ReadLevel(p.raster, l, (301 + f - 1) / f, (203 + f - 1) / f),
                  ReadLevel(g.raster, l, (301 + f - 1) / f, (203 + f - 1) / f))
            << "level " << l;
    }

    std::filesystem::remove_all(dir);
}

// Beside an internal mask or an alpha band, GDALBuildOverviews AVERAGE computes every level of a palette band from the
// full resolution, in the chunks of its largest factor (tests/unit/gdal_palette_average.h), and the mask and the alpha
// band each from the level above with themselves as their mask, in chunks; a second band of a palette is alpha, as
// GTiff's CreateCopy writes it. 10241 samples wide: chunks of 512 rows, a window across one loses its rows above it.
TEST(HostApi, WriteRasterAveragesAPaletteBesideAMaskOrAnAlphaBandFromTheFullResolution) {
    const auto dir = ApiScratch("write-palette-full");
    const int w = 10241, h = 701, block = 256;
    const std::vector<int> factors{2, 4, 8};
    const std::vector<std::array<uint8_t, 3>> rgb = TestColours();
    std::vector<uint8_t> indices(static_cast<size_t>(w) * static_cast<size_t>(h)), opacity(indices.size());
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            indices[static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x)] = PaletteIndex(x, y);
            opacity[static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x)] = Opacity(x, y);
        }
    const auto palette = ddbt::test::GdalPaletteOverviewsFromFullResolution(
        indices, w, h, block, factors, ddbt::test::GtiffReadBackTable(rgb, false, 0), false, 0);
    const auto opacities = ddbt::test::GdalMaskOverviews(opacity, w, h, block, factors);
    const auto cascade =
        ddbt::test::GdalPaletteOverviews(indices, w, h, block, factors, ddbt::test::GtiffReadBackTable(rgb, false, 0), false, 0);
    ASSERT_NE(palette[1], cascade[1]) << "the test tells the full resolution from the cascade";
    const std::string size = R"("width": 10241, "height": 701, "blockSize": 256, "overviewFactors": [2, 4, 8], )";
    for (const bool alpha : {false, true}) {
        SCOPED_TRACE(alpha ? "an alpha band (said Undefined)" : "an internal mask");
        const std::string out = (dir / (alpha ? "alpha.tif" : "mask.tif")).string();
        const std::string options = "{" + size + R"("colorTable": )" + TableJson(rgb) +
                                    (alpha ? R"(, "colorInterp": ["Palette", "Undefined"]})" : R"(, "colorInterp": ["Palette"], "mask": true})");
        ASSERT_EQ(ddbtWriteRaster(out.c_str(), options.c_str(), FillPaletteAndOpacity, nullptr, nullptr, nullptr),
                  DDBT_ERR_NONE)
            << ddbtLastError();
        OpenRaster r(out);
        ASSERT_NE(r.raster, nullptr);
        const auto info = r.Info();
        const auto& bands = info.Find("bands")->arr;
        ASSERT_EQ(bands.size(), alpha ? 2u : 1u);
        ASSERT_NE(bands[0].Find("colorTable"), nullptr) << "GDAL reads the colour table of band 1 beside another band";
        EXPECT_EQ(info.Find("mask")->str, alpha ? "alpha" : "perDataset");
        if (alpha) {
            EXPECT_EQ(bands[1].Find("colorInterp")->str, "Alpha");
        }
        EXPECT_EQ(ReadLevel(r.raster, 0, w, h), indices);
        ExpectLevels(r.raster, w, h, factors, palette, 1, false);
        ExpectLevels(r.raster, w, h, factors, opacities, alpha ? 2 : 1, !alpha);
    }
    std::filesystem::remove_all(dir);
}

// ddbtRasterRead gives the mask of band 1 as GDAL reads it: an internal mask, also beside an alpha band (GTiff's
// GetMaskBand prefers it); ddbtWriteRaster writes both, the alpha band averaged with itself as its mask
TEST(HostApi, RasterReadGivesTheInternalMaskBesideAnAlphaBand) {
    const auto dir = ApiScratch("mask-alpha");
    const std::string out = (dir / "rgba.tif").string();
    const int w = 300, h = 200;
    // Red, green, blue, alpha and the mask of each pixel: the alpha is 0 where the mask is not, and the other way
    const auto fill = [](void*, int x, int y, int fw, int fh, uint8_t* o, size_t stride) {
        for (int r = 0; r < fh; ++r)
            for (int c = 0; c < fw; ++c) {
                uint8_t* p = o + static_cast<size_t>(r) * stride + static_cast<size_t>(c) * 5;
                const int px = x + c, py = y + r;
                p[0] = static_cast<uint8_t>(px);
                p[1] = static_cast<uint8_t>(py);
                p[2] = static_cast<uint8_t>(px + py);
                p[3] = (px / 10) % 2 == 0 ? 0 : 255;
                p[4] = (py / 10) % 2 == 0 ? 0 : 255;
            }
        return 0;
    };
    const std::string options =
        R"({"width": 300, "height": 200, "blockSize": 64, "colorInterp": ["Red", "Green", "Blue", "Alpha"], "mask": true})";
    ASSERT_EQ(ddbtWriteRaster(out.c_str(), options.c_str(), +fill, nullptr, nullptr, nullptr), DDBT_ERR_NONE)
        << ddbtLastError();
    OpenRaster r(out);
    ASSERT_NE(r.raster, nullptr);
    EXPECT_EQ(r.Info().Find("mask")->str, "perDataset");
    std::vector<uint8_t> mask;
    ReadLevel(r.raster, 0, w, h, 1, &mask);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            ASSERT_EQ(mask[static_cast<size_t>(y) * w + static_cast<size_t>(x)], (y / 10) % 2 == 0 ? 0 : 255) << x << "," << y;
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

TEST(HostApi, BuildCogGeoprojectsAPhotoByItsGcps) {
    const auto dir = ApiScratch("cog-photo");
    const std::string in = std::string(DDBT_TEST_FIXTURES) + "/photo/geo_photo.jpg";
    const std::string gcps =
        R"("gcps": [[0, 0, 12.4960204442829, 41.9028236117254], [0, 144, 12.4962577650602, 41.9025521698054],)"
        R"( [192, 144, 12.4967594868457, 41.9027677644874], [192, 0, 12.4965623038112, 41.9030564539819]])";
    // Malformed options are refused before anything is read.
    for (const char* bad : {R"({"gcps": []})", R"({"gcps": [[0, 0, 1]]})", R"({"gcps": [[0, 0, 1, "a"]]})",
                            R"({"targetSrs": "source"})", R"({"outScale": 0.5})"}) {
        EXPECT_EQ(ddbtBuildCog(in.c_str(), (dir / "bad.tif").string().c_str(), bad), DDBT_ERR_INVALIDARGS) << bad;
    }
    const std::string badTarget = "{" + gcps + R"(, "targetSrs": "EPSG:4326"})";
    EXPECT_EQ(ddbtBuildCog(in.c_str(), (dir / "bad.tif").string().c_str(), badTarget.c_str()), DDBT_ERR_INVALIDARGS);
    const std::string badScale = "{" + gcps + R"(, "outScale": -1})";
    EXPECT_EQ(ddbtBuildCog(in.c_str(), (dir / "bad.tif").string().c_str(), badScale.c_str()), DDBT_ERR_INVALIDARGS);
    // The EPSG:3857 COG, and the photo in the GCPs' CRS at half its size.
    const std::string cog = (dir / "cog.tif").string(), wgs84 = (dir / "wgs84.tif").string();
    ASSERT_EQ(ddbtBuildCog(in.c_str(), cog.c_str(), ("{" + gcps + "}").c_str()), DDBT_ERR_NONE) << ddbtLastError();
    // libddb's GCPs at the corners of the photo at half its size (192 x 144 -> 96 x 72).
    const std::string half =
        R"("gcps": [[0, 0, 12.4960204442829, 41.9028236117254], [0, 72, 12.4962577650602, 41.9025521698054],)"
        R"( [96, 72, 12.4967594868457, 41.9027677644874], [96, 0, 12.4965623038112, 41.9030564539819]])";
    const std::string source = "{" + half + R"(, "gcpSrs": "EPSG:4326", "targetSrs": "source", "outScale": 0.5})";
    ASSERT_EQ(ddbtBuildCog(in.c_str(), wgs84.c_str(), source.c_str()), DDBT_ERR_NONE) << ddbtLastError();
    auto a = ddbt::cog::tiff::TiffReader::Open(cog), b = ddbt::cog::tiff::TiffReader::Open(wgs84);
    ASSERT_TRUE(a.ok() && b.ok());
    auto ga = ddbt::cog::tiff::ReadGeoreferencing(**a), gb = ddbt::cog::tiff::ReadGeoreferencing(**b);
    ASSERT_TRUE(ga.ok() && gb.ok() && ga->crs && gb->crs);
    EXPECT_TRUE(ga->crs->Equivalent(ddbt::crs::Crs::FromEpsg(3857)));
    EXPECT_TRUE(gb->crs->Equivalent(ddbt::crs::Crs::FromEpsg(4326)));
    // The footprint at half the resolution: gdalwarp's grid of 120 x 82 pixels (239 x 163 at full
    // size, tests/fixtures/photo/geoproject.tsv).
    EXPECT_EQ((*b)->Ifd((*b)->MainImage()).width, 120u);
    EXPECT_EQ((*b)->Ifd((*b)->MainImage()).height, 82u);
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

TEST(HostApi, BuildCopc2TransformsToTheTargetCrsAndReportsTheOctree) {
    namespace syn = ddbt::copc::testing;
    const auto dir = ApiScratch("copc2-crs");
    const std::string in = (dir / "in.las").string(), out = (dir / "out.copc.laz").string();
    const std::string report = (dir / "report.json").string();
    syn::SynthLas s;
    s.format = 6;
    s.minor = 4;
    s.scale = {1e-7, 1e-7, 0.01};
    s.offset = {13.84, 46.68, 650};
    s.wkt = ddbt::crs::Crs::FromEpsg(4326).ToWkt(ddbt::crs::WktFlavor::Wkt1Gdal);
    syn::FillRandom(s, 20000, 7, std::array<int32_t, 3>{70000, 70000, 7350});
    ASSERT_TRUE(syn::WriteBytes(in, syn::BuildLasBytes(s)));
    const std::string options = R"({"targetCrs": "EPSG:32633", "report": )" + JsonString(report) + "}";
    ASSERT_EQ(ddbtBuildCopc2(in.c_str(), out.c_str(), options.c_str(), nullptr, nullptr), DDBT_ERR_NONE) << ddbtLastError();
    std::ifstream f(report);
    const std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    const auto j = json::Parse(text);
    ASSERT_TRUE(j && j->IsObject()) << text;
    ASSERT_NE(j->Find("transformed"), nullptr);
    EXPECT_TRUE(j->Find("transformed")->b);
    ASSERT_NE(j->Find("sourceCrs"), nullptr);
    EXPECT_EQ(j->Find("sourceCrs")->str, s.wkt);
    ASSERT_NE(j->Find("crs"), nullptr);
    EXPECT_TRUE(ddbt::crs::Crs::FromWkt(j->Find("crs")->str).Equivalent(ddbt::crs::Crs::FromEpsg(32633)));
    ASSERT_NE(j->Find("maxNodePoints"), nullptr);
    EXPECT_TRUE(j->Find("maxNodePoints")->isUint);
    ASSERT_NE(j->Find("warnings"), nullptr);
    EXPECT_EQ(j->Find("warnings")->kind, json::Value::Kind::kArray);
    const json::Value* hist = j->Find("depthHistogram");
    ASSERT_NE(hist, nullptr);
    ASSERT_FALSE(hist->arr.empty());
    uint64_t points = 0;
    for (const json::Value& level : hist->arr) {
        ASSERT_NE(level.Find("depth"), nullptr);
        ASSERT_NE(level.Find("nodes"), nullptr);
        ASSERT_NE(level.Find("points"), nullptr);
        points += level.Find("points")->u;
    }
    EXPECT_EQ(points, 20000u);
    EXPECT_EQ(ddbtBuildCopc2(in.c_str(), out.c_str(), R"({"targetCrs": 32633})", nullptr, nullptr), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtBuildCopc2(in.c_str(), out.c_str(), R"({"targetCrs": "no such crs"})", nullptr, nullptr),
              DDBT_ERR_INVALIDARGS);
    std::filesystem::remove_all(dir);
}

// "dropCrs": the COPC states no CRS, and the report says the CRS was dropped
TEST(HostApi, BuildCopc2DropsTheCrsWhenAsked) {
    namespace syn = ddbt::copc::testing;
    const auto dir = ApiScratch("copc2-drop");
    const std::string in = (dir / "in.las").string(), out = (dir / "out.copc.laz").string();
    const std::string report = (dir / "report.json").string();
    syn::SynthLas s;
    s.format = 6;
    s.minor = 4;
    s.scale = {0.001, 0.001, 0.001};
    s.offset = {62, 47, 14};
    s.wkt = ddbt::crs::Crs::FromEpsg(4978).ToWkt(ddbt::crs::WktFlavor::Wkt1Gdal);
    syn::FillRandom(s, 5000, 9, std::array<int32_t, 3>{150000, 142000, 26000});
    ASSERT_TRUE(syn::WriteBytes(in, syn::BuildLasBytes(s)));
    const std::string options = R"({"dropCrs": true, "report": )" + JsonString(report) + "}";
    ASSERT_EQ(ddbtBuildCopc2(in.c_str(), out.c_str(), options.c_str(), nullptr, nullptr), DDBT_ERR_NONE) << ddbtLastError();
    std::ifstream f(report);
    const std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    const auto j = json::Parse(text);
    ASSERT_TRUE(j && j->IsObject()) << text;
    ASSERT_NE(j->Find("crsDropped"), nullptr);
    EXPECT_TRUE(j->Find("crsDropped")->b);
    EXPECT_FALSE(j->Find("transformed")->b);
    EXPECT_EQ(j->Find("crs")->str, "");
    EXPECT_EQ(j->Find("sourceCrs")->str, s.wkt);
    char* info = nullptr;
    ASSERT_EQ(ddbtPointCloudInfo(out.c_str(), false, &info), DDBT_ERR_NONE) << ddbtLastError();
    const auto summary = json::Parse(std::string(info));
    ddbtFree(info);
    ASSERT_TRUE(summary && summary->IsObject());
    EXPECT_EQ(summary->Find("wkt")->str, "");
    EXPECT_EQ(ddbtBuildCopc2(in.c_str(), out.c_str(), R"({"dropCrs": "yes"})", nullptr, nullptr), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtBuildCopc2(in.c_str(), out.c_str(), R"({"dropCrs": true, "targetCrs": "EPSG:32633"})", nullptr, nullptr),
              DDBT_ERR_INVALIDARGS);
    std::filesystem::remove_all(dir);
}
