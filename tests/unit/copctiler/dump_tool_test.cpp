// SPDX-License-Identifier: MPL-2.0
// A tile dump tool for the comparisons with the baseline (skipped unless configured):
//
//   DDBT_COPC_DUMP_REQUESTS=set.jsonl DDBT_COPC_DUMP_OUT=dir [DDBT_COPC_DUMP_MODE=compat|hybrid|modern]
//   [DDBT_COPC_DUMP_THREADS=N] ddbt_unit_tests --gtest_filter=CopcTilerTool.DumpTiles
//
// renders every request of a ddbt-bench request file (path, z, x, y, size, format, tms) with
// copc::tiler::RenderTile and writes the tiles as `ddbt-bench tile --dump-tiles` does
// (<index>_<z>_<x>_<y>.<ext> and index.jsonl with ok, bytes, hash or error), so that
// bench/harness/tilecmp.py and bench/micro/copctiler/compare_dumps.py compare them with the
// baseline's dump. N threads render in parallel (default 1); the tile cache is bypassed.
// DDBT_COPC_DUMP_INFO=1 adds what each render did (nodes, points, depth limit, transform and fit
// residual, compat raw fallback, blank) to index.jsonl.
#include <gtest/gtest.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "cog/tiler/tile_error.h"
#include "copc/tiler/cloud.h"
#include "copc/tiler/copc_tiler.h"

namespace {

using namespace ddbt::copc::tiler;

std::string Field(const std::string& line, const std::string& key) {
    const std::string k = "\"" + key + "\"";
    size_t at = line.find(k);
    if (at == std::string::npos) return {};
    at = line.find(':', at + k.size());
    if (at == std::string::npos) return {};
    ++at;
    while (at < line.size() && line[at] == ' ') ++at;
    if (at < line.size() && line[at] == '"') {
        const size_t end = line.find('"', at + 1);
        return line.substr(at + 1, end - at - 1);
    }
    size_t end = at;
    while (end < line.size() && line[end] != ',' && line[end] != '}') ++end;
    return line.substr(at, end - at);
}

uint64_t Fnv1a(const uint8_t* p, size_t n) {
    uint64_t h = 0xcbf29ce484222325ull;
    for (size_t i = 0; i < n; ++i) h = (h ^ p[i]) * 0x100000001b3ull;
    return h;
}

std::string JsonEscape(const std::string& s) {
    std::string o;
    for (const char c : s) {
        if (c == '"' || c == '\\') o += '\\';
        o += c;
    }
    return o;
}

TEST(CopcTilerTool, DumpTiles) {
    const char* req = std::getenv("DDBT_COPC_DUMP_REQUESTS");
    const char* out = std::getenv("DDBT_COPC_DUMP_OUT");
    if (req == nullptr || out == nullptr) GTEST_SKIP() << "set DDBT_COPC_DUMP_REQUESTS and DDBT_COPC_DUMP_OUT";
    Mode mode = GetMode();
    if (const char* m = std::getenv("DDBT_COPC_DUMP_MODE")) {
        const auto parsed = ParseMode(m);
        ASSERT_TRUE(parsed.has_value()) << "DDBT_COPC_DUMP_MODE=" << m << ": compat, hybrid or modern";
        mode = *parsed;
    }
    const bool withInfo = std::getenv("DDBT_COPC_DUMP_INFO") != nullptr;
    int threads = 1;
    if (const char* t = std::getenv("DDBT_COPC_DUMP_THREADS")) threads = std::max(1, std::atoi(t));

    std::vector<std::string> lines;
    {
        std::ifstream f(req);
        ASSERT_TRUE(f) << req;
        for (std::string l; std::getline(f, l);)
            if (!l.empty()) lines.push_back(l);
    }
    std::filesystem::create_directories(out);
    std::vector<std::string> index(lines.size());
    std::atomic<size_t> next{0};
    auto work = [&] {
        for (size_t i; (i = next.fetch_add(1)) < lines.size();) {
            const std::string& l = lines[i];
            TileRequest r;
            r.path = Field(l, "path");
            r.z = std::atoi(Field(l, "z").c_str());
            r.x = std::atoi(Field(l, "x").c_str());
            r.y = std::atoi(Field(l, "y").c_str());
            const std::string size = Field(l, "size");
            r.tileSize = size.empty() ? 256 : std::atoi(size.c_str());
            r.tms = Field(l, "tms") != "false";
            const std::string format = Field(l, "format");
            r.format = ddbt::encode::ParseTileFormat(format).value_or(ddbt::encode::TileFormat::kPng);
            r.noCache = true;
            r.mode = mode;
            char name[128];
            std::snprintf(name, sizeof name, "%06zu_%d_%d_%d.%s", i, r.z, r.x, r.y,
                          ddbt::encode::TileFormatExtension(r.format));
            std::ostringstream j;
            j << "{\"i\": " << i << ", \"file\": \"" << name << "\", \"path\": \"" << JsonEscape(r.path)
              << "\", \"z\": " << r.z << ", \"x\": " << r.x << ", \"y\": " << r.y << ", \"size\": " << r.tileSize
              << ", \"format\": \"" << (format.empty() ? "png" : format) << "\"";
            try {
                const auto bytes = RenderTile(r);
                std::ofstream f(std::filesystem::path(out) / name, std::ios::binary);
                f.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
                char hash[32];
                std::snprintf(hash, sizeof hash, "%016llx",
                              static_cast<unsigned long long>(Fnv1a(bytes.data(), bytes.size())));
                j << ", \"ok\": true, \"bytes\": " << bytes.size() << ", \"hash\": \"" << hash << "\"";
                if (withInfo) {
                    // What the render did (a second render: RenderTile does not report it).
                    RenderInfo info;
                    std::vector<uint8_t> rgba;
                    RenderRgba(*Cloud::Get(r.path), r, rgba, &info);
                    static const char* kTransforms[] = {"none", "exact", "affine", "quadratic", "cubic"};
                    j << ", \"nodes\": " << info.nodes << ", \"points\": " << info.inWindow << ", \"depthEnd\": "
                      << info.depthEnd << ", \"transform\": \"" << kTransforms[static_cast<int>(info.transform)]
                      << "\", \"fitResidual\": " << info.fitResidual << ", \"rawFallback\": "
                      << (info.rawFallback ? "true" : "false") << ", \"blank\": " << (info.blank ? "true" : "false");
                }
                j << "}";
            } catch (const std::exception& e) {
                j << ", \"ok\": false, \"error\": \"" << JsonEscape(e.what()) << "\"}";
            }
            index[i] = j.str();
        }
    };
    std::vector<std::thread> pool;
    for (int t = 1; t < threads; ++t) pool.emplace_back(work);
    work();
    for (auto& t : pool) t.join();
    std::ofstream idx(std::filesystem::path(out) / "index.jsonl");
    for (const auto& s : index) idx << s << "\n";
}

}  // namespace
