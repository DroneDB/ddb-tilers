/*
===============================================================================

  FILE:  field_rgb14.hpp

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

namespace lazperf
{
namespace detail
{

class Rgb14Base
{
protected:
    struct ChannelCtx
    {
        int have_last_;
        las::rgb14 last_;
        models::arithmetic used_model_;
        std::array<models::arithmetic, 6> diff_model_;

        ChannelCtx() : have_last_{false}, used_model_(128),
            diff_model_{ models::arithmetic(256), models::arithmetic(256),
                models::arithmetic(256), models::arithmetic(256),
                models::arithmetic(256), models::arithmetic(256) }
        {}
    };

    std::array<ChannelCtx, 4> chan_ctxs_;
    int last_channel_ = -1;
};

class Rgb14Compressor : public Rgb14Base
{
public:
    Rgb14Compressor(OutCbStream& stream) : stream_(stream), rgb_enc_(false)
    {}

    void writeSizes();
    void writeData();
    const char *compress(const char *buf, int& sc);

private:
    OutCbStream& stream_;
    encoders::arithmetic<MemoryStream> rgb_enc_;
};

class Rgb14Decompressor : public Rgb14Base
{
public:
    Rgb14Decompressor(InCbStream& stream) : stream_(stream)
    {}

    void dumpSums();
    void readSizes();
    void readData();
    char *decompress(char *buf, int& sc);

    // ddbt fork: decodes the next colour and returns it (decompress() packs it).
    const las::rgb14& decompressNext(int sc);
    // ddbt fork: skips the layer bytes when the colour is not decoded (decompress() must then
    // not be called for the points of the chunk; the first point's raw colour is skipped by
    // the caller).
    void readData(bool decode);
    // ddbt fork: back to the state of a new decompressor, keeping the models.
    void reset();

private:
    InCbStream& stream_;
    uint32_t rgb_cnt_ = 0;
    decoders::arithmetic<InViewStream> rgb_dec_;  // ddbt fork: was MemoryStream
    utils::Summer sumRgb;
};

} // namespace detail
} // namespace laszip
