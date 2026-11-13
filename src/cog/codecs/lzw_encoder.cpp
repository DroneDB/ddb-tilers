// SPDX-License-Identifier: MPL-2.0
//
// TIFF LZW encoder.
//
// The string table is an open-addressing hash of (prefix code, next byte) -> code with 8192
// slots (at most 4094 entries: load <= 0.5). Each slot key carries a 12-bit epoch, so a table
// reset (every ~3800 codes on noisy data) is one increment instead of clearing the arrays; the
// arrays are cleared only when the epoch wraps. Codes go out through a 64-bit MSB-first
// accumulator flushed 4 bytes at a time.
//
// Stream rules (libtiff's tif_lzw.c, which GDAL uses and every TIFF reader decodes):
//   - a Clear code first; codes start at 9 bits;
//   - after an entry is added, when the next free entry exceeds the largest code of the current
//     width, the width grows (9 -> 10 once entry 511 exists, ...): the "early change" of TIFF;
//   - when the next free entry reaches 4094, a Clear code is written (at 12 bits) and the table
//     and width restart;
//   - at the end the pending string is written, the next free entry is bumped as if an entry had
//     been added (the decoder does so), and EOI goes out at the resulting width.
#include <cstring>
#include <memory>

#include "cog/codecs/encoders.h"

namespace ddbt::cog::codecs {

namespace {

constexpr uint32_t kClear = 256;
constexpr uint32_t kEoi = 257;
constexpr uint32_t kFirst = 258;
constexpr uint32_t kFull = 4094;  // CODE_MAX - 1 in libtiff
constexpr uint32_t kSlotsLog2 = 13;
constexpr uint32_t kSlots = 1u << kSlotsLog2;
constexpr uint32_t kEpochShift = 20;
constexpr uint32_t kEpochLimit = 1u << 12;

struct Table {
    uint32_t keys[kSlots];   // (epoch << 20) | (prefix << 8) | byte; epoch 0 = never used
    uint16_t codes[kSlots];
    uint32_t epoch = 0;

    void Reset() {
        if (++epoch == kEpochLimit) {
            std::memset(keys, 0, sizeof(keys));
            epoch = 1;
        }
    }
};

inline uint32_t Hash(uint32_t key) { return (key * 2654435761u) >> (32 - kSlotsLog2); }

class BitWriter {
public:
    explicit BitWriter(uint8_t* out) : out_(out) {}
    inline void Put(uint32_t code, uint32_t bits) {
        acc_ = (acc_ << bits) | code;
        count_ += bits;
        if (count_ >= 32) {
            count_ -= 32;
            const uint32_t v = static_cast<uint32_t>(acc_ >> count_);
            out_[0] = static_cast<uint8_t>(v >> 24);
            out_[1] = static_cast<uint8_t>(v >> 16);
            out_[2] = static_cast<uint8_t>(v >> 8);
            out_[3] = static_cast<uint8_t>(v);
            out_ += 4;
        }
    }
    /// Flushes the remaining bits, padded with zeros to a byte; returns the end pointer.
    uint8_t* Finish() {
        while (count_ >= 8) {
            count_ -= 8;
            *out_++ = static_cast<uint8_t>(acc_ >> count_);
        }
        if (count_ > 0) *out_++ = static_cast<uint8_t>(acc_ << (8 - count_));
        count_ = 0;
        return out_;
    }

private:
    uint8_t* out_;
    uint64_t acc_ = 0;
    uint32_t count_ = 0;
};

Table& ThreadTable() {
    thread_local std::unique_ptr<Table> table;
    if (!table) {
        table = std::make_unique<Table>();
        std::memset(table->keys, 0, sizeof(table->keys));
    }
    return *table;
}

}  // namespace

size_t LzwEncodedBound(size_t n) noexcept {
    // At most one code per input byte, 12 bits each, plus one Clear per 3836 codes, the first
    // Clear and EOI, and the 4-byte flush granularity.
    return n + n / 2 + n / 1024 + 16;
}

void EncodeLzw(std::span<const uint8_t> in, encode::ByteBuffer& out) {
    const size_t start = out.size();
    out.resize(start + LzwEncodedBound(in.size()));
    BitWriter w(out.data() + start);
    Table& t = ThreadTable();
    t.Reset();
    uint32_t epochBits = t.epoch << kEpochShift;

    uint32_t nbits = 9;
    uint32_t maxcode = 511;
    uint32_t freeEnt = kFirst;
    w.Put(kClear, nbits);

    const uint8_t* p = in.data();
    const uint8_t* const end = p + in.size();
    if (p != end) {
        uint32_t ent = *p++;
        for (; p != end; ++p) {
            const uint32_t c = *p;
            const uint32_t key = epochBits | (ent << 8) | c;
            uint32_t h = Hash(key);
            bool found = false;
            while (true) {
                const uint32_t k = t.keys[h];
                if (k == key) {
                    ent = t.codes[h];
                    found = true;
                    break;
                }
                if ((k >> kEpochShift) != t.epoch) break;  // empty slot of this epoch
                h = (h + 1) & (kSlots - 1);
            }
            if (found) continue;
            w.Put(ent, nbits);
            t.keys[h] = key;
            t.codes[h] = static_cast<uint16_t>(freeEnt++);
            ent = c;
            if (freeEnt == kFull) {
                w.Put(kClear, nbits);
                t.Reset();
                epochBits = t.epoch << kEpochShift;
                freeEnt = kFirst;
                nbits = 9;
                maxcode = 511;
            } else if (freeEnt > maxcode) {
                ++nbits;
                maxcode = (1u << nbits) - 1;
            }
        }
        w.Put(ent, nbits);
        ++freeEnt;
        if (freeEnt == kFull) {
            w.Put(kClear, nbits);
            nbits = 9;
        } else if (freeEnt > maxcode) {
            ++nbits;
        }
    }
    w.Put(kEoi, nbits);
    uint8_t* e = w.Finish();
    out.resize(static_cast<size_t>(e - out.data()));
}

}  // namespace ddbt::cog::codecs
