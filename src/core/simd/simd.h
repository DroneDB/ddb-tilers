// SPDX-License-Identifier: MPL-2.0
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ddbt::simd {

/// Sum of all bytes, computed by the best Highway target the running CPU supports (runtime
/// dispatch). Trivial kernel that proves the multi-target build and the dispatch.
uint64_t SumBytes(const uint8_t* data, size_t size);

/// Reference implementation of SumBytes.
uint64_t SumBytesScalar(const uint8_t* data, size_t size) noexcept;

/// Name of the target SumBytes dispatches to right now ("AVX2", "AVX3_ZEN4", ...): reported by
/// the code that is actually executing, not guessed from CPUID.
std::string ActiveTargetName();

/// Name of the target the library was statically compiled for (the -march baseline).
std::string StaticTargetName();

/// Targets compiled into the library AND supported by the running CPU, best first.
std::vector<std::string> SupportedTargetNames();

/// Same, as Highway target bits (for tests that force a target).
std::vector<int64_t> SupportedTargetBits();

/// Targets compiled into the library, best first (supported or not).
std::vector<std::string> CompiledTargetNames();

/// Name of a Highway target bit.
std::string TargetNameOf(int64_t targetBit);

/// Force dispatch to the given target bit (tests only); 0 restores the CPU detection.
void ForceTargetForTest(int64_t targetBit);

/// Whether the running CPU runs Highway's SSE4 target (SSE4.2, CLMUL, AES) / AVX2 target (AVX2,
/// BMI1, BMI2, FMA, F16C). The hand-written paths outside Highway (CRC-32C, fpnge) are chosen
/// with these, so they follow ForceTargetForTest like the Highway kernels.
bool HasSse4() noexcept;
bool HasAvx2() noexcept;

/// HasAvx2() on a CPU whose BMI2 PDEP / PEXT are fast: AMD (and Hygon) CPUs before Zen 3 run them
/// as microcode, hundreds of cycles for a 21-bit mask, slower than the portable bit spreading.
bool HasFastPdep() noexcept;

}  // namespace ddbt::simd
