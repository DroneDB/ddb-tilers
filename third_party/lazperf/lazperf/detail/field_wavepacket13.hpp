// SPDX-License-Identifier: Apache-2.0
// ddbt fork (new file): the WAVEPACKET13 (version 1) item of point formats 4 and 5 in
// point-wise chunked LAZ (compressor 2), written from LASzip's algorithm: the descriptor index
// with a 256-symbol model, the packet offset as "same", "previous offset + previous size", a
// 32-bit difference (integer coder) or a raw 64-bit value, chosen by a 4-symbol model in the
// context of the previous choice, then the packet size, the return point location and x / y / z
// (float bit patterns as int32) each predicted from the previous packet.
#pragma once

#include <array>
#include <cstdint>

namespace lazperf
{
namespace detail
{

class Wavepacket13Base
{
protected:
    Wavepacket13Base();

    struct Packet
    {
        uint64_t offset;
        uint32_t size;
        int32_t returnPoint;
        int32_t x, y, z;
    };
    static Packet unpack(const unsigned char *p);
    static void pack(const Packet& w, unsigned char *p);

    bool have_last_;
    unsigned char last_[28];
    int32_t last_diff_32_;
    uint32_t sym_last_offset_diff_;
    models::arithmetic packet_index_;
    std::array<models::arithmetic, 4> offset_diff_;
};

class Wavepacket13Compressor : public Wavepacket13Base
{
public:
    Wavepacket13Compressor(encoders::arithmetic<OutCbStream>& encoder);
    const char *compress(const char *buf);

private:
    encoders::arithmetic<OutCbStream>& enc_;
    bool inited_;
    compressors::integer ic_offset_diff_, ic_packet_size_, ic_return_point_, ic_xyz_;
};

class Wavepacket13Decompressor : public Wavepacket13Base
{
public:
    Wavepacket13Decompressor(decoders::arithmetic<InCbStream>& decoder);
    char *decompress(char *buf);

private:
    decoders::arithmetic<InCbStream>& dec_;
    bool inited_;
    decompressors::integer ic_offset_diff_, ic_packet_size_, ic_return_point_, ic_xyz_;
};

} // namespace detail
} // namespace lazperf
