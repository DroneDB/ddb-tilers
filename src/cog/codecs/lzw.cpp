// SPDX-License-Identifier: MPL-2.0
//
// TIFF LZW decoder.
//
// Every string an LZW code stands for is a copy of bytes already written: the entry added
// after code k is "string(k-1) + first byte of string(k)", and string(k-1) and string(k) are
// written next to each other, so the new entry is exactly the output range
// [start(k-1), start(k-1) + length(k-1) + 1). The table therefore stores (offset, length) into
// the output buffer instead of prefix chains: decoding a code is one forward copy (a 16-byte
// "wild" copy when there is room), with no per-byte chain walk. The source range of a copy
// always ends at or before the destination start, except for the KwKwK case (code == next free
// entry), whose last byte is the first byte of the same string.
//
// Code widths follow libtiff (TIFF 6.0 "early change"): 9 bits, 10 bits once the next free
// entry is 511, 11 at 1023, 12 at 2047; at most 4096 entries.

#include <cstdint>
#include <cstring>
#include <span>
#include <string>

#include "cog/codecs/codecs.h"
#include "core/util/endian.h"

namespace ddbt::cog::codecs {

namespace {

constexpr uint32_t kClearCode = 256;
constexpr uint32_t kEoiCode = 257;
constexpr uint32_t kFirstCode = 258;
constexpr uint32_t kMaxEntries = 4096;
constexpr uint32_t kMaxBits = 12;

struct Entry {
    uint32_t offset;
    uint32_t length;
};

// MSB-first bit reader. `bits_` holds `count_` valid bits at its top; the bits below may hold
// the following input bits (the branchless refill ORs whole 8-byte loads), never garbage.
class BitReader {
public:
    BitReader(const uint8_t* data, size_t size) noexcept : data_(data), size_(size) {}

    // Ensures at least 12 bits when the input has them; returns the available count.
    inline uint32_t Refill() noexcept {
        if (count_ >= kMaxBits) return count_;
        if (pos_ + 8 <= size_) {
            const uint64_t v = LoadBE<uint64_t>(data_ + pos_);
            bits_ |= v >> count_;
            const uint32_t bytes = (63 - count_) >> 3;
            pos_ += bytes;
            count_ += bytes * 8;
        } else {
            while (count_ <= 56 && pos_ < size_) {
                bits_ |= uint64_t{data_[pos_++]} << (56 - count_);
                count_ += 8;
            }
        }
        return count_;
    }

    inline uint32_t Take(uint32_t n) noexcept {
        const uint32_t v = static_cast<uint32_t>(bits_ >> (64 - n));
        bits_ <<= n;
        count_ -= n;
        return v;
    }

private:
    const uint8_t* data_;
    size_t size_;
    size_t pos_ = 0;
    uint64_t bits_ = 0;
    uint32_t count_ = 0;
};

// Copies `len` bytes from out[src] to out[dst] (src + len <= dst, or the KwKwK case handled by
// the caller). `room` says whether 16 bytes past dst + len are writable.
inline void CopyString(uint8_t* out, size_t src, size_t dst, size_t len, bool room) noexcept {
    if (room) {
        // Wild copy: every byte read for the first `len` outputs lies before dst, so it is
        // never overwritten by an earlier chunk of the same copy.
        size_t i = 0;
        do {
            uint8_t tmp[16];
            std::memcpy(tmp, out + src + i, 16);
            std::memcpy(out + dst + i, tmp, 16);
            i += 16;
        } while (i < len);
    } else {
        for (size_t i = 0; i < len; ++i) out[dst + i] = out[src + i];
    }
}

}  // namespace

Status DecodeLzw(std::span<const uint8_t> in, std::span<uint8_t> out) {
    if (in.size() >= 2 && in[0] == 0 && (in[1] & 1) != 0) {
        return Unsupported("LZW: old-style (pre TIFF 5.0, LSB-first) LZW is not supported");
    }
    if (out.size() > UINT32_MAX) return Unsupported("LZW: block larger than 4 GiB");
    if (out.empty()) return Status::Ok();

    Entry table[kMaxEntries];
    BitReader reader(in.data(), in.size());
    uint8_t* const dst = out.data();
    const size_t end = out.size();
    // The wild copy may write up to 15 bytes past a string: allowed while pos + len + 16 <= end.
    size_t pos = 0;
    uint32_t nbits = 9;
    uint32_t nextFree = kFirstCode;
    uint32_t prevOffset = 0;
    uint32_t prevLength = 0;
    bool havePrev = false;

    while (pos < end) {
        if (reader.Refill() < nbits) {
            // No EOI: libtiff treats the end of the data as EOI ("not terminated with EOI").
            break;
        }
        const uint32_t code = reader.Take(nbits);
        if (code == kClearCode) {
            nbits = 9;
            nextFree = kFirstCode;
            havePrev = false;
            continue;
        }
        if (code == kEoiCode) break;

        if (!havePrev) {
            if (code > 255) return Corrupt("LZW: corrupted table (first code after Clear is not a literal)");
            dst[pos] = static_cast<uint8_t>(code);
            prevOffset = static_cast<uint32_t>(pos);
            prevLength = 1;
            ++pos;
            havePrev = true;
            continue;
        }

        uint32_t offset;
        uint32_t length;
        if (code < 256) {
            dst[pos] = static_cast<uint8_t>(code);
            offset = static_cast<uint32_t>(pos);
            length = 1;
            ++pos;
        } else if (code < nextFree) {
            const Entry e = table[code];
            length = e.length;
            offset = static_cast<uint32_t>(pos);
            if (pos + length + 16 <= end) {
                CopyString(dst, e.offset, pos, length, true);
                pos += length;
            } else {
                const size_t n = length <= end - pos ? length : end - pos;
                CopyString(dst, e.offset, pos, n, false);
                pos += n;
                if (n < length) break;  // the block is full
            }
        } else if (code == nextFree) {
            // KwKwK: string(prev) followed by its own first byte.
            length = prevLength + 1;
            offset = static_cast<uint32_t>(pos);
            const size_t n = length <= end - pos ? length : end - pos;
            const size_t head = n < prevLength ? n : prevLength;
            CopyString(dst, prevOffset, pos, head, pos + head + 16 <= end);
            if (n > prevLength) dst[pos + prevLength] = dst[prevOffset];
            pos += n;
            if (n < length) break;
        } else {
            return Corrupt("LZW: corrupted table (code " + std::to_string(code) + " past the next free entry " +
                           std::to_string(nextFree) + ")");
        }

        if (nextFree < kMaxEntries) {
            table[nextFree] = Entry{prevOffset, prevLength + 1};
            ++nextFree;
            if (nextFree == (1u << nbits) - 1 && nbits < kMaxBits) ++nbits;
        }
        prevOffset = offset;
        prevLength = length;
    }

    if (pos < end) {
        return Corrupt("LZW: not enough data (" + std::to_string(end - pos) + " of " + std::to_string(end) +
                       " bytes missing)");
    }
    return Status::Ok();
}

}  // namespace ddbt::cog::codecs
