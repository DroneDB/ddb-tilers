/*
===============================================================================

  FILE:  field_nir14.hpp

  CONTENTS:


  PROGRAMMERS:

    martin.isenburg@rapidlasso.com  -  http://rapidlasso.com
    uday.karan@gmail.com - Hobu, Inc.

  COPYRIGHT:

    (c) 2007-2014, martin isenburg, rapidlasso - tools to catch reality
    (c) 2014, Uday Verma, Hobu, Inc.

    This is free software; you can redistribute and/or modify it under the
    terms of the Apache Public License 2.0 published by the Apache Software
    Foundation. See the COPYING file for more information.

    This software is distributed WITHOUT ANY WARRANTY and without even the
    implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

  CHANGE HISTORY:

===============================================================================
*/

#include "../las.hpp"

namespace lazperf
{
namespace detail
{

// COMPRESSOR

void Nir14Compressor::writeSizes()
{
    nir_enc_.done();
    stream_ << nir_enc_.num_encoded();
}

void Nir14Compressor::writeData()
{
    LAZDEBUG(std::cerr << "NIR       : " <<
        utils::sum(nir_enc_.encoded_bytes(), nir_enc_.num_encoded()) << "\n");

    if (nir_enc_.num_encoded())
        stream_.putBytes(nir_enc_.encoded_bytes(), nir_enc_.num_encoded());
}

const char *Nir14Compressor::compress(const char *buf, int& sc)
{
    const las::nir14 nir(buf);

    // don't have the first data yet, just push it to our
    // have last stuff and move on
    if (last_channel_ == -1)
    {
        ChannelCtx& c = chan_ctxs_[sc];
        stream_.putBytes((const unsigned char*)&nir, sizeof(las::nir14));
        c.last_ = nir;
        c.have_last_ = true;
        last_channel_ = sc;
        return buf + sizeof(las::nir14);
    }

    ChannelCtx& c = chan_ctxs_[sc];
    las::nir14 *pLastNir = &chan_ctxs_[last_channel_].last_;
    if (!c.have_last_)
    {
        c.have_last_ = true;
        c.last_ = *pLastNir;
        pLastNir = &c.last_;
    }
    // This mess is because of the broken-ness of the handling for last in v3, where
    // 'last_point' only gets updated on the first context switch in the LASzip code.
    las::nir14& lastNir = *pLastNir;

    bool lowChange = (lastNir.val & 0xFF) != (nir.val & 0xFF);
    bool highChange = (lastNir.val & 0xFF00) != (nir.val & 0xFF00);
    int32_t sym = (int32_t)lowChange | (highChange << 1);
    if (sym)
        nir_enc_.makeValid();
    nir_enc_.encodeSymbol(c.used_model_, sym);

    if (lowChange)
    {
        int32_t diff =  (nir.val & 0xFF) - (lastNir.val & 0xFF);
        nir_enc_.encodeSymbol(c.diff_model_[0], uint8_t(diff));
    }
    if (highChange)
    {
        int32_t diff =  (nir.val >> 8) - (lastNir.val >> 8);
        nir_enc_.encodeSymbol(c.diff_model_[1], uint8_t(diff));
    }

    lastNir = nir;
    last_channel_ = sc;
    return buf + sizeof(las::nir14);
}

// DECOMPRESSOR

void Nir14Decompressor::dumpSums()
{
    std::cout << "NIR      : " << sumNir.value() << "\n";
}

void Nir14Decompressor::readSizes()
{
    stream_ >> nir_cnt_;
}

void Nir14Decompressor::readData()
{
    nir_dec_.initStream(stream_, nir_cnt_);
}

// ddbt fork
void Nir14Decompressor::readData(bool decode)
{
    if (decode)
        readData();
    else
        stream_.skip(nir_cnt_);
}

// ddbt fork
void Nir14Decompressor::reset()
{
    for (ChannelCtx& c : chan_ctxs_)
    {
        if (!c.have_last_)
            continue;
        c.have_last_ = false;
        c.last_ = las::nir14();
        c.used_model_.reset();
        for (auto& m : c.diff_model_)
            m.reset();
    }
    last_channel_ = -1;
    nir_cnt_ = 0;
    nir_dec_.reset();
}

char *Nir14Decompressor::decompress(char *buf, int& sc)
{
    // ddbt fork: the decoding moved to decompressNext(); this packs the value.
    las::nir14 nir = decompressNext(sc);
    nir.pack(buf);
    return buf + sizeof(las::nir14);
}

const las::nir14& Nir14Decompressor::decompressNext(int sc)
{
    if (last_channel_ == -1)
    {
        ChannelCtx& c = chan_ctxs_[sc];
        char buf[sizeof(las::nir14)];  // ddbt fork: local (was: the caller's buffer)
        stream_.getBytes((unsigned char*)buf, sizeof(las::nir14));
        c.last_.unpack(buf);
        c.have_last_ = true;
        last_channel_ = sc;
        return c.last_;
    }
    if (nir_cnt_ == 0)
        return chan_ctxs_[last_channel_].last_;  // ddbt fork (was: copied to the buffer)

    ChannelCtx& c = chan_ctxs_[sc];
    las::nir14 *pLastNir = &chan_ctxs_[last_channel_].last_;
    if (sc != last_channel_)
    {
        last_channel_ = sc;
        if (!c.have_last_)
        {
            c.have_last_ = true;
            c.last_ = *pLastNir;
            pLastNir = &chan_ctxs_[last_channel_].last_;
        }
    }
    las::nir14& lastNir = *pLastNir;

    uint32_t sym = nir_dec_.decodeSymbol(c.used_model_);

    las::nir14 nir;

    if (sym & (1 << 0))
    {
        uint8_t corr = (uint8_t)nir_dec_.decodeSymbol(c.diff_model_[0]);
        nir.val = uint8_t(corr + (lastNir.val & 0xFF));
    }
    else
        nir.val = lastNir.val & 0xFF;

    if (sym & (1 << 1))
    {
        uint8_t corr = (uint8_t)nir_dec_.decodeSymbol(c.diff_model_[1]);
        nir.val |= (static_cast<uint16_t>(uint8_t(corr + (lastNir.val >> 8))) << 8);
    }
    else
        nir.val |= lastNir.val & 0xFF00;
    LAZDEBUG(sumNir.add(nir));

    lastNir = nir;
    return lastNir;  // ddbt fork (was: packed into the caller's buffer)
}

} // namespace detail
} // namespace lazperf
