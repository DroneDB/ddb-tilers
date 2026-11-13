// SPDX-License-Identifier: MPL-2.0
//
// The COPC structure check as a command: DDBT_COPC_CHECK=a.copc.laz:b.copc.laz runs CheckCopc
// on each file and prints its report (skipped when unset). Used by the validation scripts.
#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <string>

#include "copc/build/copc_check.h"

namespace ddbt::copc::build {
namespace {

TEST(CopcCheckTool, ChecksFilesFromTheEnvironment) {
    const char* env = std::getenv("DDBT_COPC_CHECK");
    if (env == nullptr || *env == '\0') GTEST_SKIP() << "DDBT_COPC_CHECK not set";
    std::string list = env;
    size_t start = 0;
    while (start <= list.size()) {
        const size_t end = std::min(list.find(':', start), list.size());
        const std::string path = list.substr(start, end - start);
        start = end + 1;
        if (path.empty()) continue;
        auto r = CheckCopc(path);
        ASSERT_TRUE(r.ok()) << path << ": " << r.status().ToString();
        std::printf("%s: header %llu hierarchy %llu decoded %llu points, %llu nodes, depth %d, outside %llu (beyond "
                    "tolerance %llu), chunk table %s\n",
                    path.c_str(), static_cast<unsigned long long>(r->headerPoints), static_cast<unsigned long long>(r->hierarchyPoints),
                    static_cast<unsigned long long>(r->decodedPoints), static_cast<unsigned long long>(r->dataNodes), r->maxDepth,
                    static_cast<unsigned long long>(r->outside), static_cast<unsigned long long>(r->outsideTolerance),
                    r->chunkTableMatches ? "matches" : "DIFFERS");
        for (size_t d = 0; d < r->depths.size(); ++d) {
            const CopcDepthStats& s = r->depths[d];
            std::printf("  depth %2zu: %6llu nodes %11llu points (%.0f / node); inner %llu nodes, %llu points, %llu share a "
                        "grid cell\n",
                        d, static_cast<unsigned long long>(s.nodes), static_cast<unsigned long long>(s.points),
                        s.nodes ? double(s.points) / double(s.nodes) : 0.0, static_cast<unsigned long long>(s.innerNodes),
                        static_cast<unsigned long long>(s.innerPoints), static_cast<unsigned long long>(s.innerSharedCells));
        }
        for (const std::string& e : r->errors) std::printf("  ERROR %s\n", e.c_str());
        EXPECT_TRUE(r->ok()) << path;
        if (end == list.size()) break;
    }
}

}  // namespace
}  // namespace ddbt::copc::build
