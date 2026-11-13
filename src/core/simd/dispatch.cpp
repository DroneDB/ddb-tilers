// SPDX-License-Identifier: MPL-2.0
//
// Highway dynamic dispatch: this file is compiled once per enabled target (foreach_target.h
// re-includes it), the per-target code lives in HWY_NAMESPACE, and HWY_DYNAMIC_DISPATCH picks
// the best supported one at runtime.

#include "core/simd/simd.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "core/simd/dispatch.cpp"
#include "hwy/foreach_target.h"  // IWYU pragma: keep, must come before highway.h
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace ddbt::simd {
namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;

uint64_t SumBytesImpl(const uint8_t* HWY_RESTRICT data, size_t size) {
    const hn::ScalableTag<uint8_t> d;
    const size_t n = hn::Lanes(d);
    // SumsOf8 adds each group of eight bytes into a u64 lane: no overflow for any input size.
    auto acc = hn::Zero(hn::Repartition<uint64_t, decltype(d)>());
    size_t i = 0;
    for (; i + n <= size; i += n) {
        acc = hn::Add(acc, hn::SumsOf8(hn::LoadU(d, data + i)));
    }
    uint64_t total = hn::ReduceSum(hn::Repartition<uint64_t, decltype(d)>(), acc);
    for (; i < size; ++i) total += data[i];
    return total;
}

const char* TargetNameImpl() { return hwy::TargetName(HWY_TARGET); }

}  // namespace HWY_NAMESPACE
}  // namespace ddbt::simd
HWY_AFTER_NAMESPACE();

#if HWY_ONCE

namespace ddbt::simd {

HWY_EXPORT(SumBytesImpl);
HWY_EXPORT(TargetNameImpl);

uint64_t SumBytes(const uint8_t* data, size_t size) {
    return HWY_DYNAMIC_DISPATCH(SumBytesImpl)(data, size);
}

uint64_t SumBytesScalar(const uint8_t* data, size_t size) noexcept {
    uint64_t total = 0;
    for (size_t i = 0; i < size; ++i) total += data[i];
    return total;
}

std::string ActiveTargetName() { return HWY_DYNAMIC_DISPATCH(TargetNameImpl)(); }

std::string StaticTargetName() { return hwy::TargetName(HWY_STATIC_TARGET); }

std::vector<int64_t> SupportedTargetBits() {
    // Best first (Highway orders the list that way).
    return hwy::SupportedAndGeneratedTargets();
}

std::vector<std::string> SupportedTargetNames() {
    std::vector<std::string> names;
    for (const int64_t t : SupportedTargetBits()) names.emplace_back(hwy::TargetName(t));
    return names;
}

std::vector<std::string> CompiledTargetNames() {
    std::vector<std::string> names;
    int64_t bits = HWY_TARGETS;
    // Highway target bits: a lower bit is a better target on the same architecture.
    while (bits != 0) {
        const int64_t bit = bits & -bits;
        names.emplace_back(hwy::TargetName(bit));
        bits &= bits - 1;
    }
    return names;
}

std::string TargetNameOf(int64_t targetBit) { return hwy::TargetName(targetBit); }

void ForceTargetForTest(int64_t targetBit) { hwy::SetSupportedTargetsForTest(targetBit); }

namespace {
#if HWY_ARCH_X86
// Every x86 target at least as capable as AVX2, then as SSE4 (a CPU, or a forced target, that runs
// one of them has the instructions of the lesser one).
constexpr int64_t kAvx2OrBetter = HWY_AVX2 | HWY_AVX3 | HWY_AVX3_DL | HWY_AVX3_ZEN4 | HWY_AVX3_SPR | HWY_AVX10_2;
constexpr int64_t kSse4OrBetter = HWY_SSE4 | kAvx2OrBetter;
#else
constexpr int64_t kAvx2OrBetter = 0;
constexpr int64_t kSse4OrBetter = 0;
#endif
}  // namespace

bool HasSse4() noexcept { return (hwy::SupportedTargets() & kSse4OrBetter) != 0; }
bool HasAvx2() noexcept { return (hwy::SupportedTargets() & kAvx2OrBetter) != 0; }

}  // namespace ddbt::simd

#endif  // HWY_ONCE
