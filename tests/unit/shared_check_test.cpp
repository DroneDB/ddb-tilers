// SPDX-License-Identifier: MPL-2.0
//
// Links the shared library (libddbt.so / ddbt.dll): the C API is exported and works without
// the static third-party code being re-exported.

#include <gtest/gtest.h>

#include <string>

#include "ddbt/ddbt.h"

TEST(SharedLibrary, ExportsTheCApi) {
    EXPECT_STREQ(ddbtVersion(), "0.2.0");
    char* json = nullptr;
    ASSERT_EQ(ddbtCpuInfo(&json), DDBT_ERR_NONE);
    EXPECT_NE(std::string(json).find("dispatchTarget"), std::string::npos);
    ddbtFree(json);
    EXPECT_EQ(ddbtBuildCog("a", "b", nullptr), DDBT_ERR_NOTFOUND);
    EXPECT_EQ(ddbtBuildCopc("a", "b", nullptr), DDBT_ERR_NOTFOUND);
}
