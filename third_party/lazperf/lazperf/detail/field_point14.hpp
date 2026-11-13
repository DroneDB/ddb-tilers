/*
===============================================================================

  FILE:  field_point14.hpp

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

class Point14Base
{
protected:
    // ddbt fork: `decoder` selects the side whose integer coders the channel contexts create.
    explicit Point14Base(bool decoder);

    struct ChannelCtx
    {
        int ctx_num_;  //ABELL - For debug.
        std::vector<models::arithmetic> changed_values_model_;
        models::arithmetic scanner_channel_model_;
        models::arithmetic rn_gps_same_model_;
        std::vector<models::arithmetic> nr_model_;
        std::vector<models::arithmetic> rn_model_;
        std::vector<models::arithmetic> class_model_;
        std::vector<models::arithmetic> flag_model_;
        std::vector<models::arithmetic> user_data_model_;

        models::arithmetic gpstime_multi_model_;
        models::arithmetic gpstime_0diff_model_;

        compressors::integer dx_compr_;
        compressors::integer dy_compr_;
        compressors::integer z_compr_;
        compressors::integer intensity_compr_;
        compressors::integer scan_angle_compr_;
        compressors::integer point_source_id_compr_;
        compressors::integer gpstime_compr_;

        decompressors::integer dx_decomp_;
        decompressors::integer dy_decomp_;
        decompressors::integer z_decomp_;
        decompressors::integer intensity_decomp_;
        decompressors::integer scan_angle_decomp_;
        decompressors::integer point_source_id_decomp_;
        decompressors::integer gpstime_decomp_;

        bool have_last_;
        las::point14 last_;
        std::array<uint16_t, 8> last_intensity_;
        std::array<int32_t, 8> last_z_;
        std::array<utils::streaming_median<int>, 12> last_x_diff_median5_;
        std::array<utils::streaming_median<int>, 12> last_y_diff_median5_;
        uint32_t last_gps_seq_;
        uint32_t next_gps_seq_;
        std::array<double, 4> last_gpstime_;
        std::array<int32_t, 4> last_gpstime_diff_;
        std::array<int32_t, 4> multi_extreme_counter_;
        bool gps_time_change_;

        // ddbt fork: only the integer coders of one side are initialised (the other side's
        // never allocate models), and the context number is a constructor argument.
        ChannelCtx(bool decoder, int num) : ctx_num_(num),
            changed_values_model_(8, models::arithmetic(128)),
            scanner_channel_model_(3), rn_gps_same_model_(13),
            nr_model_(16, models::arithmetic(16)), rn_model_(16, models::arithmetic(16)),
            class_model_(64, models::arithmetic(256)), flag_model_(64, models::arithmetic(64)),
            user_data_model_(64, models::arithmetic(256)), gpstime_multi_model_(515),
            gpstime_0diff_model_(5),
            dx_compr_(32, 2), dy_compr_(32, 22), z_compr_(32, 20), intensity_compr_(16, 4),
            scan_angle_compr_(16, 2), point_source_id_compr_(16), gpstime_compr_(32, 9),
            dx_decomp_(32, 2), dy_decomp_(32, 22), z_decomp_(32, 20), intensity_decomp_(16, 4),
            scan_angle_decomp_(16, 2), point_source_id_decomp_(16), gpstime_decomp_(32, 9),
            have_last_{false}, last_gps_seq_{0}, next_gps_seq_{0},
            last_gpstime_{}, last_gpstime_diff_{}, multi_extreme_counter_{},
            gps_time_change_{}
        {
            //ABELL - Move the init into the ctor, I think.
            // Also, the encoder should be passed to the ctor.
            if (!decoder)  // ddbt fork
            {
                dx_compr_.init();
                dy_compr_.init();
                z_compr_.init();
                intensity_compr_.init();
                scan_angle_compr_.init();
                point_source_id_compr_.init();
                gpstime_compr_.init();
            }
            else
            {
                dx_decomp_.init();
                dy_decomp_.init();
                z_decomp_.init();
                intensity_decomp_.init();
                scan_angle_decomp_.init();
                point_source_id_decomp_.init();
                gpstime_decomp_.init();
            }

            for (auto& xd : last_x_diff_median5_)
                xd.init();
            for (auto& yd : last_y_diff_median5_)
                yd.init();
        }

        // ddbt fork: back to the state of a new decoder-side context. Only the models of the
        // `layers` are reset: the others were never touched since the last reset.
        void resetDecoder(uint32_t layers);
    };  // ChannelCtx

    // ddbt fork: the four contexts are created on first use (most chunks only use channel 0).
    // A context's address never changes once created.
    class ChannelCtxs
    {
    public:
        explicit ChannelCtxs(bool decoder) : decoder_(decoder)
        {}

        ChannelCtx& operator[](size_t i)
        {
            if (!p_[i])
                p_[i].reset(new ChannelCtx(decoder_, (int)i));
            return *p_[i];
        }

        // The context if it was created, else nullptr.
        ChannelCtx *created(size_t i)
        { return p_[i].get(); }

    private:
        bool decoder_;
        std::array<std::unique_ptr<ChannelCtx>, 4> p_;
    };

    ChannelCtxs chan_ctxs_;
    int last_channel_;
};

class Point14Compressor : public Point14Base
{
public:
    Point14Compressor(OutCbStream& stream) : Point14Base(false), stream_(stream)
    {}

    void writeSizes();
    void writeData();
    const char *compress(const char *buf, int& sc);

private:
    void encodeGpsTime(const las::point14& point, ChannelCtx& c);

    OutCbStream& stream_;
    encoders::arithmetic<MemoryStream> xy_enc_ = true;
    encoders::arithmetic<MemoryStream> z_enc_ = true;
    encoders::arithmetic<MemoryStream> class_enc_ = false;
    encoders::arithmetic<MemoryStream> flags_enc_ = false;
    encoders::arithmetic<MemoryStream> intensity_enc_ = false;
    encoders::arithmetic<MemoryStream> scan_angle_enc_ = false;
    encoders::arithmetic<MemoryStream> user_data_enc_ = false;
    encoders::arithmetic<MemoryStream> point_source_id_enc_ = false;
    encoders::arithmetic<MemoryStream> gpstime_enc_ = false;
};

class Point14Decompressor : public Point14Base
{
public:
    Point14Decompressor(InCbStream& stream) : Point14Base(true), stream_(stream)
    {}

    void dumpSums();
    void readSizes();
    void readData();
    char *decompress(char *buf, int& sc);

    // ddbt fork: decodes the next point into its channel context and returns it (decompress()
    // packs it into a record; the SoA path reads its fields).
    const las::point14& decompressNext(int& sc);
    // ddbt fork: the layers to decode (layer::point14 bits; xy is always added). The bytes of
    // the other layers are skipped by readData() and their fields keep stale values.
    void setLayers(uint32_t layers);
    uint32_t layers() const
    { return layers_; }
    // ddbt fork: back to the state of a new decompressor for the next chunk, keeping the
    // allocated contexts and models.
    void reset();

private:
    void decodeGpsTime(ChannelCtx& c);

    // ddbt fork: a reference, as the other 1.4 fields hold it: the stream may be a direct
    // (buffer) stream whose position must be shared, not a copy of a stateless callback.
    InCbStream& stream_;
    // ddbt fork: InViewStream: layer bytes are borrowed from a direct stream, not copied.
    decoders::arithmetic<InViewStream> xy_dec_;
    decoders::arithmetic<InViewStream> z_dec_;
    decoders::arithmetic<InViewStream> class_dec_;
    decoders::arithmetic<InViewStream> flags_dec_;
    decoders::arithmetic<InViewStream> intensity_dec_;
    decoders::arithmetic<InViewStream> scan_angle_dec_;
    decoders::arithmetic<InViewStream> user_data_dec_;
    decoders::arithmetic<InViewStream> point_source_id_dec_;
    decoders::arithmetic<InViewStream> gpstime_dec_;
    uint32_t layers_ = layer::point14;  // ddbt fork
    uint32_t touched_ = 0;              // ddbt fork: layers decoded since the last reset()
    std::vector<uint32_t> sizes_;
    utils::Summer sumChange;
    utils::Summer sumReturn;
    utils::Summer sumX;
    utils::Summer sumY;
    utils::Summer sumZ;
    utils::Summer sumClass;
    utils::Summer sumFlags;
    utils::Summer sumIntensity;
    utils::Summer sumScanAngle;
    utils::Summer sumUserData;
    utils::Summer sumPointSourceId;
    utils::Summer sumGpsTime;
};

} // namespace detail
} // namespace lazperf
