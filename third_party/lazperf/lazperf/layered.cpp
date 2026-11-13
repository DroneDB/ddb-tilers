// SPDX-License-Identifier: Apache-2.0
// ddbt fork (new file): see layered.hpp.
//
// The order of the reads is that of point_decompressor_6/7/8::decompress (lazperf.cpp): the
// first point raw (point14, rgb, nir, extra bytes), then the chunk's point count, the layer
// sizes (point14, rgb, nir, bytes) and the layers. The scanner channel handed from the point14
// field to the others is exactly what upstream hands (0 unless the first point or a channel
// change set it), so records match upstream's byte for byte.

#include "layered.hpp"

#include <cstring>
#include <string>
#include <vector>

#include "las.hpp"
#include "streams.hpp"

namespace lazperf
{

struct layered_decompressor::Private
{
    Private(int format, int ebCount, uint32_t layers) : format_(format), eb_(ebCount),
        stream_(nullptr, 0), point_(stream_), rgb_(stream_), nir_(stream_),
        bytes_(stream_, (size_t)ebCount)
    {
        // ddbt fork: formats 9 and 10 are 6 and 8 plus a WAVEPACKET14 layer, skipped.
        if (format < 6 || format > 10)
            throw error("layered_decompressor: point format " + std::to_string(format) +
                " is not 6 to 10");
        if (ebCount < 0)
            throw error("layered_decompressor: negative extra byte count");
        hasRgb_ = format == 7 || format == 8 || format == 10;
        hasNir_ = format == 8 || format == 10;
        hasWave_ = format >= 9;
        uint32_t present = layer::point14 | layer::extra_bytes;
        if (hasRgb_)
            present |= layer::rgb;
        if (hasNir_)
            present |= layer::nir;
        if (eb_ == 0)
            present &= ~layer::extra_bytes;
        layers_ = (layers | layer::xy) & present;
        decRgb_ = (layers_ & layer::rgb) != 0;
        decNir_ = (layers_ & layer::nir) != 0;
        decEb_ = (layers_ & layer::extra_bytes) != 0;
        point_.setLayers(layers_);
        ebScratch_.resize((size_t)eb_);
    }

    void begin(const char *data, size_t size)
    {
        if (dirty_)
        {
            point_.reset();
            if (hasRgb_)
                rgb_.reset();
            if (hasNir_)
                nir_.reset();
            if (eb_)
                bytes_.reset();
        }
        base_ = reinterpret_cast<const unsigned char *>(data);
        stream_.reset(base_, size);
        first_ = true;
        chunkCount_ = 0;
        decoded_ = 0;
        dirty_ = true;
    }

    // After the first point: the chunk header and the layers (skipped ones are only passed).
    void finishFirst()
    {
        stream_ >> chunkCount_;
        point_.readSizes();
        if (hasRgb_)
            rgb_.readSizes();
        if (hasNir_)
            nir_.readSizes();
        if (hasWave_)
            stream_ >> waveBytes_;
        if (eb_)
            bytes_.readSizes();

        point_.readData();
        if (hasRgb_)
            rgb_.readData(decRgb_);
        if (hasNir_)
            nir_.readData(decNir_);
        if (hasWave_)
            stream_.skip(waveBytes_);
        if (eb_)
            bytes_.readData(decEb_);
        first_ = false;
    }

    int format_;
    int eb_;
    uint32_t layers_ = 0;
    bool hasRgb_ = false;
    bool hasNir_ = false;
    bool hasWave_ = false;      // ddbt fork: formats 9 / 10
    uint32_t waveBytes_ = 0;
    bool decRgb_ = false;
    bool decNir_ = false;
    bool decEb_ = false;
    InCbStream stream_;
    detail::Point14Decompressor point_;
    detail::Rgb14Decompressor rgb_;
    detail::Nir14Decompressor nir_;
    detail::Byte14Decompressor bytes_;
    std::vector<char> ebScratch_;
    const unsigned char *base_ = nullptr;
    uint32_t chunkCount_ = 0;
    uint64_t decoded_ = 0;
    bool first_ = true;
    bool dirty_ = false;
};

layered_decompressor::layered_decompressor(int format, int ebCount, uint32_t layers) :
    p_(new Private(format, ebCount, layers))
{}

layered_decompressor::~layered_decompressor()
{}

int layered_decompressor::format() const
{ return p_->format_; }

int layered_decompressor::ebCount() const
{ return p_->eb_; }

uint32_t layered_decompressor::layers() const
{ return p_->layers_; }

size_t layered_decompressor::pointSize() const
{
    return sizeof(las::point14) + (p_->hasRgb_ ? sizeof(las::rgb14) : 0) +
        (p_->hasNir_ ? sizeof(las::nir14) : 0) + (p_->hasWave_ ? 29 : 0) + (size_t)p_->eb_;
}

void layered_decompressor::begin(const char *data, size_t size)
{
    p_->begin(data, size);
}

char *layered_decompressor::decompress(char *out)
{
    Private& p = *p_;
    int sc = 0;
    const las::point14& pt = p.point_.decompressNext(sc);
    std::memcpy(out, &pt, sizeof(las::point14));
    out += sizeof(las::point14);
    if (p.hasRgb_)
    {
        if (p.decRgb_)
            p.rgb_.decompress(out, sc);
        else if (p.first_)
            p.stream_.skip(sizeof(las::rgb));
        out += sizeof(las::rgb14);
    }
    if (p.hasNir_)
    {
        if (p.decNir_)
            p.nir_.decompress(out, sc);
        else if (p.first_)
            p.stream_.skip(sizeof(las::nir14));
        out += sizeof(las::nir14);
    }
    if (p.hasWave_)
    {
        // Wave packets are not decoded: zeros (the first point's raw packet is skipped).
        if (p.first_)
            p.stream_.skip(29);
        std::memset(out, 0, 29);
        out += 29;
    }
    if (p.eb_)
    {
        if (p.decEb_)
            p.bytes_.decompress(out, sc);
        else if (p.first_)
            p.stream_.skip((size_t)p.eb_);
        out += p.eb_;
    }
    if (p.first_)
        p.finishFirst();
    ++p.decoded_;
    return out;
}

void layered_decompressor::decompress(const soa_sink& s, size_t count)
{
    Private& p = *p_;
    for (size_t i = 0; i < count; ++i)
    {
        int sc = 0;
        const las::point14& pt = p.point_.decompressNext(sc);
        if (s.x)
            s.x[i] = pt.x();
        if (s.y)
            s.y[i] = pt.y();
        if (s.z)
            s.z[i] = pt.z();
        if (p.hasRgb_)
        {
            if (p.decRgb_)
            {
                const las::rgb14& c = p.rgb_.decompressNext(sc);
                if (s.r)
                    s.r[i] = c.r;
                if (s.g)
                    s.g[i] = c.g;
                if (s.b)
                    s.b[i] = c.b;
            }
            else if (p.first_)
                p.stream_.skip(sizeof(las::rgb));
        }
        if (p.hasNir_)
        {
            if (p.decNir_)
            {
                const las::nir14& n = p.nir_.decompressNext(sc);
                if (s.nir)
                    s.nir[i] = n.val;
            }
            else if (p.first_)
                p.stream_.skip(sizeof(las::nir14));
        }
        if (p.hasWave_ && p.first_)
            p.stream_.skip(29);
        if (p.eb_)
        {
            if (p.decEb_)
                p.bytes_.decompress(p.ebScratch_.data(), sc);
            else if (p.first_)
                p.stream_.skip((size_t)p.eb_);
        }
        if (p.first_)
            p.finishFirst();
    }
    p.decoded_ += count;
}

uint64_t layered_decompressor::decoded() const
{ return p_->decoded_; }

uint32_t layered_decompressor::chunkPointCount() const
{ return p_->chunkCount_; }

size_t layered_decompressor::consumed() const
{
    return p_->base_ ? (size_t)(p_->stream_.position() - p_->base_) : 0;
}

} // namespace lazperf
