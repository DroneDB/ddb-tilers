// SPDX-License-Identifier: MPL-2.0
#pragma once

// Little / big endian loads and stores at any (unaligned) address. memcpy makes them legal for
// any alignment; compilers turn them into single mov / movbe instructions.

#include <bit>
#include <cstdint>
#include <cstring>
#include <type_traits>

#if defined(_MSC_VER) && !defined(__clang__)
#include <stdlib.h>
#endif

namespace ddbt {

static_assert(std::endian::native == std::endian::little || std::endian::native == std::endian::big,
              "mixed-endian platforms are not supported");

constexpr uint8_t ByteSwap(uint8_t v) noexcept { return v; }

constexpr uint16_t ByteSwap(uint16_t v) noexcept {
    return static_cast<uint16_t>((v >> 8) | (v << 8));
}

constexpr uint32_t ByteSwap(uint32_t v) noexcept {
    if (std::is_constant_evaluated()) {
        return ((v & 0xFFu) << 24) | ((v & 0xFF00u) << 8) | ((v >> 8) & 0xFF00u) | (v >> 24);
    }
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_bswap32(v);
#elif defined(_MSC_VER)
    return _byteswap_ulong(v);
#else
    return ((v & 0xFFu) << 24) | ((v & 0xFF00u) << 8) | ((v >> 8) & 0xFF00u) | (v >> 24);
#endif
}

constexpr uint64_t ByteSwap(uint64_t v) noexcept {
    if (std::is_constant_evaluated()) {
        return (uint64_t{ByteSwap(static_cast<uint32_t>(v))} << 32) | ByteSwap(static_cast<uint32_t>(v >> 32));
    }
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_bswap64(v);
#elif defined(_MSC_VER)
    return _byteswap_uint64(v);
#else
    return (uint64_t{ByteSwap(static_cast<uint32_t>(v))} << 32) | ByteSwap(static_cast<uint32_t>(v >> 32));
#endif
}

namespace detail {
template <size_t N>
struct UintOfSize;
template <> struct UintOfSize<1> { using type = uint8_t; };
template <> struct UintOfSize<2> { using type = uint16_t; };
template <> struct UintOfSize<4> { using type = uint32_t; };
template <> struct UintOfSize<8> { using type = uint64_t; };

template <typename T>
concept EndianValue = (std::is_integral_v<T> || std::is_floating_point_v<T>) &&
                      (sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4 || sizeof(T) == 8);

template <EndianValue T, std::endian E>
inline T Load(const void* p) noexcept {
    using U = typename UintOfSize<sizeof(T)>::type;
    U u;
    std::memcpy(&u, p, sizeof(U));
    if constexpr (E != std::endian::native) u = ByteSwap(u);
    return std::bit_cast<T>(u);
}

template <EndianValue T, std::endian E>
inline void Store(void* p, T v) noexcept {
    using U = typename UintOfSize<sizeof(T)>::type;
    U u = std::bit_cast<U>(v);
    if constexpr (E != std::endian::native) u = ByteSwap(u);
    std::memcpy(p, &u, sizeof(U));
}
}  // namespace detail

/// Reads a T stored little-endian at p (any alignment). T: integer or float/double.
template <detail::EndianValue T>
inline T LoadLE(const void* p) noexcept { return detail::Load<T, std::endian::little>(p); }

/// Reads a T stored big-endian at p (any alignment).
template <detail::EndianValue T>
inline T LoadBE(const void* p) noexcept { return detail::Load<T, std::endian::big>(p); }

/// Writes v little-endian at p (any alignment).
template <detail::EndianValue T>
inline void StoreLE(void* p, T v) noexcept { detail::Store<T, std::endian::little>(p, v); }

/// Writes v big-endian at p (any alignment).
template <detail::EndianValue T>
inline void StoreBE(void* p, T v) noexcept { detail::Store<T, std::endian::big>(p, v); }

/// Reads with a byte order chosen at run time (TIFF "II" / "MM").
template <detail::EndianValue T>
inline T LoadEndian(const void* p, bool bigEndian) noexcept { return bigEndian ? LoadBE<T>(p) : LoadLE<T>(p); }

template <detail::EndianValue T>
inline void StoreEndian(void* p, T v, bool bigEndian) noexcept {
    if (bigEndian) StoreBE<T>(p, v);
    else StoreLE<T>(p, v);
}

}  // namespace ddbt
