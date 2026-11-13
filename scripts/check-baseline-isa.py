#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Check that a library built for the plain x86-64 baseline runs on any x86-64 CPU.

Disassembles the library (objdump) and lists the functions that use instructions above the
baseline (AVX / AVX-512 registers, VEX arithmetic, BMI1/BMI2, LZCNT, CRC32, CLMUL, AES). Each must
be code that is only called after a run-time check of the CPU:

  - Highway kernels (namespaces N_SSSE3, N_SSE4, N_AVX2, N_AVX3, N_AVX3_DL, N_AVX3_ZEN4);
  - our hand-written paths behind simd::HasSse4() / HasAvx2() (the fpnge variants and the code of
    fpnge.cc they contain, Crc32cSse42);
  - the run-time dispatched code of the vcpkg libraries (libjpeg-turbo jsimd_*, libdeflate,
    libwebp *_AVX2 / *_SSE41, zstd's BMI2 paths).

Anything else is a function the compiler or a header compiled above the baseline, which would
crash with an illegal instruction on an older CPU: the script lists it and exits 1.

    scripts/check-baseline-isa.py build/linux-gcc-release/lib/libddbt.so
"""
import argparse
import collections
import re
import subprocess
import sys

ABOVE_BASELINE = re.compile(
    r"%ymm|%zmm|%k[0-7]\b"
    r"|\bv(add|sub|mul|div|fmadd|fnmadd|fmsub|fnmsub|perm|pshuf|broadcast|pbroadcast|movdq|movap|movup"
    r"|pxor|por|pand|zeroupper|cvt|insert|extract|gather|blend|pcmp|psub|padd|sqrt|min|max|round)\w*\b"
    r"|\b(pdep|pext|shlx|shrx|sarx|rorx|bzhi|andn|blsr|blsi|blsmsk|lzcnt|crc32[bwlq]|pclmulqdq|aesenc\w*|popcnt)\b")

DISPATCHED = re.compile(
    # Highway targets above the baseline
    r"\bN_(SSSE3|SSE4|AVX2|AVX3|AVX3_DL|AVX3_ZEN4|AVX3_SPR|AVX10_2)::"
    r"|\bhwy::"
    # ours, behind simd::HasSse4() / HasAvx2()
    r"|\bddbt_fpnge_encode_(sse4|avx2)\b|Crc32cSse42"
    # fpnge.cc, compiled only inside the variants (its anonymous-namespace code)
    r"|\(anonymous namespace\)::(SelectPredictor|CopyRow|EncodeOneRow|ProcessRow|CollectSymbolCounts|TryPredictor"
    r"|Crc32::|ForAllRLESymbols|BitWriter|WriteHuffmanCode|hadd|PredictPixels|UpdateAdler32|HuffmanTable)"
    # vcpkg libraries with their own run-time dispatch
    r"|^jsimd_|_avx2\b|_avx2[._]|_avx512|_avx\b|_avx[._]|_bmi2|_sse41|_sse4|_ssse3|_AVX2\b|_SSE41\b|_SSE2\b"
    r"|Init\w*(AVX2|SSE41|SSE2)\b|pclmul|^ZSTD_|^HUF_|^crc32_x86|^adler32_x86")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("library")
    a = ap.parse_args()
    dis = subprocess.run(["objdump", "-d", "--no-show-raw-insn", "-C", a.library],
                         capture_output=True, text=True, check=True).stdout
    counts = collections.Counter()
    sample = {}
    function = None
    for line in dis.splitlines():
        m = re.match(r"^[0-9a-f]+ <(.*)>:$", line)
        if m:
            function = m.group(1)
            continue
        if function and ABOVE_BASELINE.search(line):
            counts[function] += 1
            sample.setdefault(function, line.strip())
    stray = sorted((f for f in counts if not DISPATCHED.search(f)), key=lambda f: -counts[f])
    print("%d functions use instructions above x86-64, %d behind a run-time check, %d not"
          % (len(counts), len(counts) - len(stray), len(stray)))
    for f in stray[:50]:
        print("  %4d  %s\n        %s" % (counts[f], f[:150], sample[f][:100]))
    return 1 if stray else 0


if __name__ == "__main__":
    sys.exit(main())
