// SPDX-License-Identifier: MPL-2.0
// XXH64 (Yann Collet's xxHash, 64-bit variant), written from the published specification:
// https://github.com/Cyan4973/xxHash/blob/dev/doc/xxhash_spec.md
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace ddbtb
{

class Xxh64
{
  public:
    static uint64_t hash(const void *data, size_t len, uint64_t seed = 0)
    {
        const auto *p = static_cast<const uint8_t *>(data);
        const uint8_t *const end = p + len;
        uint64_t h;
        if (len >= 32)
        {
            uint64_t v1 = seed + P1 + P2;
            uint64_t v2 = seed + P2;
            uint64_t v3 = seed;
            uint64_t v4 = seed - P1;
            const uint8_t *const limit = end - 32;
            do
            {
                v1 = round(v1, read64(p));
                v2 = round(v2, read64(p + 8));
                v3 = round(v3, read64(p + 16));
                v4 = round(v4, read64(p + 24));
                p += 32;
            } while (p <= limit);
            h = rotl(v1, 1) + rotl(v2, 7) + rotl(v3, 12) + rotl(v4, 18);
            h = merge(h, v1);
            h = merge(h, v2);
            h = merge(h, v3);
            h = merge(h, v4);
        }
        else
            h = seed + P5;
        h += static_cast<uint64_t>(len);
        while (p + 8 <= end)
        {
            h ^= round(0, read64(p));
            h = rotl(h, 27) * P1 + P4;
            p += 8;
        }
        if (p + 4 <= end)
        {
            h ^= static_cast<uint64_t>(read32(p)) * P1;
            h = rotl(h, 23) * P2 + P3;
            p += 4;
        }
        while (p < end)
        {
            h ^= static_cast<uint64_t>(*p) * P5;
            h = rotl(h, 11) * P1;
            ++p;
        }
        h ^= h >> 33;
        h *= P2;
        h ^= h >> 29;
        h *= P3;
        h ^= h >> 32;
        return h;
    }

  private:
    static constexpr uint64_t P1 = 0x9E3779B185EBCA87ULL;
    static constexpr uint64_t P2 = 0xC2B2AE3D27D4EB4FULL;
    static constexpr uint64_t P3 = 0x165667B19E3779F9ULL;
    static constexpr uint64_t P4 = 0x85EBCA77C2B2AE63ULL;
    static constexpr uint64_t P5 = 0x27D4EB2F165667C5ULL;

    static uint64_t rotl(uint64_t x, int r) { return (x << r) | (x >> (64 - r)); }
    // Little-endian reads (all supported targets are little-endian).
    static uint64_t read64(const uint8_t *p)
    {
        uint64_t v;
        std::memcpy(&v, p, 8);
        return v;
    }
    static uint32_t read32(const uint8_t *p)
    {
        uint32_t v;
        std::memcpy(&v, p, 4);
        return v;
    }
    static uint64_t round(uint64_t acc, uint64_t input)
    {
        acc += input * P2;
        acc = rotl(acc, 31);
        return acc * P1;
    }
    static uint64_t merge(uint64_t acc, uint64_t v)
    {
        acc ^= round(0, v);
        return acc * P1 + P4;
    }
};

} // namespace ddbtb
