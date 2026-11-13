// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "ddbt/ddbt.h"

TEST(Api, Version) {
    const std::string v = ddbtVersion();
    EXPECT_EQ(v, "0.1.0");
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
    EXPECT_NE(s.find("\"version\":\"0.1.0\""), std::string::npos) << s;
    EXPECT_EQ(ddbtCpuInfo(nullptr), DDBT_ERR_INVALIDARGS);
}

TEST(Api, FreeNullIsNoop) { EXPECT_EQ(ddbtFree(nullptr), DDBT_ERR_NONE); }
