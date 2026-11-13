// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "core/simd/simd.h"

namespace {

std::vector<uint8_t> RandomBytes(size_t n, uint32_t seed) {
    std::mt19937 rng(seed);
    std::vector<uint8_t> v(n);
    for (auto& b : v) b = static_cast<uint8_t>(rng() & 0xFFu);
    return v;
}

// Restores automatic target selection when a test forces one.
struct ForcedTargetGuard {
    ~ForcedTargetGuard() { ddbt::simd::ForceTargetForTest(0); }
};

}  // namespace

TEST(Dispatch, SumBytesMatchesScalar) {
    for (const size_t n : {size_t{0}, size_t{1}, size_t{15}, size_t{16}, size_t{31}, size_t{32}, size_t{63},
                           size_t{64}, size_t{65}, size_t{1000}, size_t{4097}, size_t{1} << 20}) {
        const auto data = RandomBytes(n, static_cast<uint32_t>(n) + 7u);
        EXPECT_EQ(ddbt::simd::SumBytes(data.data(), n), ddbt::simd::SumBytesScalar(data.data(), n)) << "n=" << n;
    }
}

TEST(Dispatch, SumBytesAllOnesDoesNotOverflow) {
    const std::vector<uint8_t> data(size_t{1} << 22, 0xFF);
    EXPECT_EQ(ddbt::simd::SumBytes(data.data(), data.size()), uint64_t{255} * data.size());
}

TEST(Dispatch, EveryCompiledAndSupportedTargetAgreesWithScalar) {
    const auto bits = ddbt::simd::SupportedTargetBits();
    ASSERT_FALSE(bits.empty());
    const auto data = RandomBytes(100003, 42);
    const uint64_t expected = ddbt::simd::SumBytesScalar(data.data(), data.size());

    ForcedTargetGuard guard;
    for (const int64_t bit : bits) {
        ddbt::simd::ForceTargetForTest(bit);
        // The code that runs reports its own target: forcing really selects that kernel.
        EXPECT_EQ(ddbt::simd::ActiveTargetName(), ddbt::simd::TargetNameOf(bit));
        EXPECT_EQ(ddbt::simd::SumBytes(data.data(), data.size()), expected)
            << "target " << ddbt::simd::TargetNameOf(bit);
    }
}

TEST(Dispatch, TargetListsAreConsistent) {
    const auto compiled = ddbt::simd::CompiledTargetNames();
    const auto supported = ddbt::simd::SupportedTargetNames();
    ASSERT_FALSE(compiled.empty());
    ASSERT_FALSE(supported.empty());
    // Every supported target was compiled in.
    for (const auto& s : supported) {
        EXPECT_NE(std::find(compiled.begin(), compiled.end(), s), compiled.end()) << s;
    }
    // The newest targets are excluded on purpose (compile time).
    for (const auto& c : compiled) {
        EXPECT_NE(c, "AVX3_SPR");
        EXPECT_NE(c, "AVX10_2");
    }
    // The automatic choice is the best supported target.
    EXPECT_EQ(ddbt::simd::ActiveTargetName(), supported.front());
}
