/*
===============================================================================

  FILE:  streams.hpp

  CONTENTS:
    Stream abstractions

  PROGRAMMERS:

    uday.karan@gmail.com - Hobu, Inc.

  COPYRIGHT:

    (c) 2014, Uday Verma, Hobu, Inc.

    This is free software; you can redistribute and/or modify it under the
    terms of the Apache Public License 2.0 published by the Apache Software
    Foundation. See the COPYING file for more information.

    This software is distributed WITHOUT ANY WARRANTY and without even the
    implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

  CHANGE HISTORY:

===============================================================================
*/

#ifndef __streams_hpp__
#define __streams_hpp__

#include <cstring>
#include <vector>
#include <iostream>

#include "lazperf.hpp"
#include "excepts.hpp"
#include "filestream.hpp"
#include "portable_endian.hpp"

namespace lazperf
{

struct OutCbStream
{
    OutCbStream(OutputCb outCb) : outCb_(outCb)
    {}

    void putBytes(const unsigned char *b, size_t len)
    {
        outCb_(b, len);
    }

    void putByte(const unsigned char b)
    {
        outCb_(&b, 1);
    }

    OutputCb outCb_;
};

struct InCbStream
{
    InCbStream(InputCb inCb) : inCb_(inCb)
    {}

    // ddbt fork: direct mode. The stream reads a borrowed buffer of known size (bounds-checked,
    // no callback per read), and skip() moves past bytes without touching them.
    InCbStream(const unsigned char *data, size_t size) : direct_(true), cur_(data), end_(data + size)
    {}

    // ddbt fork: points a direct-mode stream at a new buffer.
    void reset(const unsigned char *data, size_t size)
    {
        direct_ = true;
        cur_ = data;
        end_ = data + size;
    }

    bool direct() const
    { return direct_; }

    // ddbt fork: direct mode only: the current read position (layer views borrow from it).
    const unsigned char *position() const
    { return cur_; }

    unsigned char getByte()
    {
        unsigned char c;
        getBytes(&c, 1);
        return c;
    }

    void getBytes(unsigned char *b, size_t len)
    {
        if (direct_)  // ddbt fork
        {
            if ((size_t)(end_ - cur_) < len)
                throw error("LAZ chunk is truncated");
            std::memcpy(b, cur_, len);
            cur_ += len;
            return;
        }
        inCb_(b, len);
    }

    // ddbt fork: moves past `len` bytes (read and dropped in callback mode).
    void skip(size_t len)
    {
        if (direct_)
        {
            if ((size_t)(end_ - cur_) < len)
                throw error("LAZ chunk is truncated");
            cur_ += len;
            return;
        }
        std::vector<unsigned char> scratch(len);
        if (len)
            inCb_(scratch.data(), len);
    }

    InputCb inCb_;

private:
    bool direct_ = false;  // ddbt fork
    const unsigned char *cur_ = nullptr;
    const unsigned char *end_ = nullptr;
};

// ddbt fork: input of an arithmetic decoder: the bytes of one layer. From a direct-mode
// InCbStream it borrows them (no copy); from a callback stream it copies them. Reads past the
// end give 0 (what a well-formed stream's padding holds) instead of undefined behaviour.
struct InViewStream
{
    unsigned char getByte()
    {
        return cur_ < end_ ? *cur_++ : 0;
    }

    void getBytes(unsigned char *b, size_t len)
    {
        while (len--)
            *b++ = getByte();
    }

    void copy(InCbStream& in, size_t bytes)
    {
        if (in.direct())
        {
            const unsigned char *p = in.position();
            in.skip(bytes);  // bounds check
            cur_ = p;
            end_ = p + bytes;
        }
        else
        {
            own_.resize(bytes);
            in.getBytes(own_.data(), bytes);
            cur_ = own_.data();
            end_ = cur_ + bytes;
        }
    }

    void clear()
    {
        cur_ = end_ = nullptr;
    }

private:
    const unsigned char *cur_ = nullptr;
    const unsigned char *end_ = nullptr;
    std::vector<unsigned char> own_;
};

struct MemoryStream
{
    MemoryStream() : buf(), idx(0)
    {}

    void putBytes(const unsigned char* b, size_t len)
    {
        while(len --)
            buf.push_back(*b++);
    }

    void putByte(const unsigned char b)
    {
        buf.push_back(b);
    }

    OutputCb outCb()
    {
        using namespace std::placeholders;

        return std::bind(&MemoryStream::putBytes, this, _1, _2);
    }

    unsigned char getByte()
    {
        return buf[idx++];
    }

    void getBytes(unsigned char *b, int len)
    {
        for (int i = 0 ; i < len ; i ++)
            b[i] = getByte();
    }

    InputCb inCb()
    {
        using namespace std::placeholders;

        return std::bind(&MemoryStream::getBytes, this, _1, _2);
    }

    uint32_t numBytesPut() const
    {
        return buf.size();
    }

    // Copy bytes from the source stream to this stream.
    template <typename TSrc>
    void copy(TSrc& in, size_t bytes)
    {
        buf.resize(bytes);
        in.getBytes(buf.data(), bytes);
    }

    const uint8_t *data() const
    { return buf.data(); }

    const std::vector<unsigned char>& buffer() const
    { return buf; }

    std::vector<unsigned char>& buffer()
    { return buf; }

    std::vector<unsigned char> buf; // cuz I'm ze faste
    size_t idx;
};

template <typename TStream>
TStream& operator << (TStream& stream, uint32_t u)
{
    uint32_t uLe = htole32(u);
    stream.putBytes(reinterpret_cast<const unsigned char *>(&uLe), sizeof(uLe));
    return stream;
}

template <typename TStream>
TStream& operator >> (TStream& stream, uint32_t& u)
{
    uint32_t uLe;
    stream.getBytes(reinterpret_cast<unsigned char *>(&uLe), sizeof(u));
    u = le32toh(uLe);
    return stream;
}

} // namespace lazperf

#endif // __streams_hpp__
