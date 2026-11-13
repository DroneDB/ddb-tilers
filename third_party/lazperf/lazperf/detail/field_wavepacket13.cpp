// SPDX-License-Identifier: Apache-2.0
// ddbt fork (new file): see field_wavepacket13.hpp.

#include "../las.hpp"

#include <cstring>

namespace lazperf
{
namespace detail
{

Wavepacket13Base::Wavepacket13Base() : have_last_(false), last_{}, last_diff_32_(0),
    sym_last_offset_diff_(0), packet_index_(256),
    offset_diff_{ models::arithmetic(4), models::arithmetic(4), models::arithmetic(4),
        models::arithmetic(4) }
{}

namespace
{
// Little-endian loads (utils::unpack<uint64_t> sign-extends its bytes).
uint64_t le64(const unsigned char *p)
{
    uint64_t v = 0;
    for (int i = 7; i >= 0; --i)
        v = (v << 8) | p[i];
    return v;
}
uint32_t le32(const unsigned char *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
} // namespace

Wavepacket13Base::Packet Wavepacket13Base::unpack(const unsigned char *p)
{
    Packet w;
    w.offset = le64(p);
    w.size = le32(p + 8);
    w.returnPoint = (int32_t)le32(p + 12);
    w.x = (int32_t)le32(p + 16);
    w.y = (int32_t)le32(p + 20);
    w.z = (int32_t)le32(p + 24);
    return w;
}

void Wavepacket13Base::pack(const Packet& w, unsigned char *p)
{
    char *c = (char *)p;
    utils::pack(w.offset, c);
    utils::pack((uint32_t)w.size, c + 8);
    utils::pack((uint32_t)w.returnPoint, c + 12);
    utils::pack((uint32_t)w.x, c + 16);
    utils::pack((uint32_t)w.y, c + 20);
    utils::pack((uint32_t)w.z, c + 24);
}

// COMPRESSOR

Wavepacket13Compressor::Wavepacket13Compressor(encoders::arithmetic<OutCbStream>& encoder) :
    enc_(encoder), inited_(false), ic_offset_diff_(32, 1), ic_packet_size_(32, 1),
    ic_return_point_(32, 1), ic_xyz_(32, 3)
{}

const char *Wavepacket13Compressor::compress(const char *buf)
{
    const unsigned char *in = (const unsigned char *)buf;
    if (!inited_)
    {
        ic_offset_diff_.init();
        ic_packet_size_.init();
        ic_return_point_.init();
        ic_xyz_.init();
        inited_ = true;
    }
    if (!have_last_)
    {
        enc_.getOutStream().putBytes(in, 29);
        std::memcpy(last_, in + 1, 28);
        have_last_ = true;
        return buf + 29;
    }
    enc_.encodeSymbol(packet_index_, in[0]);
    const Packet cur = unpack(in + 1);
    const Packet last = unpack(last_);
    const int64_t diff64 = (int64_t)(cur.offset - last.offset);
    const int32_t diff32 = (int32_t)diff64;
    if (diff64 == (int64_t)diff32)
    {
        if (diff32 == 0)
        {
            enc_.encodeSymbol(offset_diff_[sym_last_offset_diff_], 0);
            sym_last_offset_diff_ = 0;
        }
        else if (diff32 == (int32_t)last.size)
        {
            enc_.encodeSymbol(offset_diff_[sym_last_offset_diff_], 1);
            sym_last_offset_diff_ = 1;
        }
        else
        {
            enc_.encodeSymbol(offset_diff_[sym_last_offset_diff_], 2);
            sym_last_offset_diff_ = 2;
            ic_offset_diff_.compress(enc_, last_diff_32_, diff32, 0);
            last_diff_32_ = diff32;
        }
    }
    else
    {
        enc_.encodeSymbol(offset_diff_[sym_last_offset_diff_], 3);
        sym_last_offset_diff_ = 3;
        enc_.writeInt((uint32_t)(cur.offset & 0xFFFFFFFFu));
        enc_.writeInt((uint32_t)(cur.offset >> 32));
    }
    ic_packet_size_.compress(enc_, (int32_t)last.size, (int32_t)cur.size, 0);
    ic_return_point_.compress(enc_, last.returnPoint, cur.returnPoint, 0);
    ic_xyz_.compress(enc_, last.x, cur.x, 0);
    ic_xyz_.compress(enc_, last.y, cur.y, 1);
    ic_xyz_.compress(enc_, last.z, cur.z, 2);
    std::memcpy(last_, in + 1, 28);
    return buf + 29;
}

// DECOMPRESSOR

Wavepacket13Decompressor::Wavepacket13Decompressor(decoders::arithmetic<InCbStream>& decoder) :
    dec_(decoder), inited_(false), ic_offset_diff_(32, 1), ic_packet_size_(32, 1),
    ic_return_point_(32, 1), ic_xyz_(32, 3)
{}

char *Wavepacket13Decompressor::decompress(char *buf)
{
    unsigned char *out = (unsigned char *)buf;
    if (!inited_)
    {
        ic_offset_diff_.init();
        ic_packet_size_.init();
        ic_return_point_.init();
        ic_xyz_.init();
        inited_ = true;
    }
    if (!have_last_)
    {
        dec_.getInStream().getBytes(out, 29);
        std::memcpy(last_, out + 1, 28);
        have_last_ = true;
        return buf + 29;
    }
    out[0] = (unsigned char)dec_.decodeSymbol(packet_index_);
    const Packet last = unpack(last_);
    Packet cur;
    sym_last_offset_diff_ = dec_.decodeSymbol(offset_diff_[sym_last_offset_diff_]);
    if (sym_last_offset_diff_ == 0)
        cur.offset = last.offset;
    else if (sym_last_offset_diff_ == 1)
        cur.offset = last.offset + last.size;
    else if (sym_last_offset_diff_ == 2)
    {
        last_diff_32_ = ic_offset_diff_.decompress(dec_, last_diff_32_, 0);
        cur.offset = last.offset + (uint64_t)(int64_t)last_diff_32_;
    }
    else
        cur.offset = dec_.readInt64();
    cur.size = (uint32_t)ic_packet_size_.decompress(dec_, (int32_t)last.size, 0);
    cur.returnPoint = ic_return_point_.decompress(dec_, last.returnPoint, 0);
    cur.x = ic_xyz_.decompress(dec_, last.x, 0);
    cur.y = ic_xyz_.decompress(dec_, last.y, 1);
    cur.z = ic_xyz_.decompress(dec_, last.z, 2);
    pack(cur, out + 1);
    std::memcpy(last_, out + 1, 28);
    return buf + 29;
}

} // namespace detail
} // namespace lazperf
