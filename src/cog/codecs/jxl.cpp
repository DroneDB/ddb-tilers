// SPDX-License-Identifier: MPL-2.0
//
// JPEG-XL blocks of a TIFF (compression 50002, and 52546 of DNG 1.7), decoded as GDAL's TIFF JXL
// codec decodes them (frmts/gtiff/tif_jxl.c, JXLDecode): one codestream per tile or strip, its
// colour channels and its alpha (when libjxl reports the first extra channel as alpha) as the
// main image, the other extra channels after them in each pixel; one channel per block for BAND
// interleave.
#include "cog/codecs/codecs.h"

#include <jxl/decode.h>

#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace ddbt::cog::codecs {

namespace {

struct DecoderDeleter {
    void operator()(JxlDecoder* d) const noexcept { JxlDecoderDestroy(d); }
};

// The decoder of this thread, reset for each block
JxlDecoder* ThreadDecoder() {
    thread_local std::unique_ptr<JxlDecoder, DecoderDeleter> decoder(JxlDecoderCreate(nullptr));
    if (decoder) JxlDecoderReset(decoder.get());
    return decoder.get();
}

}  // namespace

Status DecodeJxl(std::span<const uint8_t> in, const JxlBlockInfo& b, std::span<uint8_t> out) {
    JxlDataType type;
    size_t bytes;
    if (b.bitsPerSample == 8 && b.sampleFormat != 3) {
        type = JXL_TYPE_UINT8;
        bytes = 1;
    } else if (b.bitsPerSample == 16 && b.sampleFormat != 3) {
        type = JXL_TYPE_UINT16;
        bytes = 2;
    } else if (b.bitsPerSample == 16 && b.sampleFormat == 3) {
        type = JXL_TYPE_FLOAT16;
        bytes = 2;
    } else if (b.bitsPerSample == 32 && b.sampleFormat == 3) {
        type = JXL_TYPE_FLOAT;
        bytes = 4;
    } else {
        return Unsupported("JPEG-XL with " + std::to_string(b.bitsPerSample) + "-bit samples of SampleFormat " +
                           std::to_string(b.sampleFormat));
    }
    const size_t pixels = static_cast<size_t>(b.width) * b.height;
    const size_t channelSize = pixels * bytes;
    if (out.size() < channelSize * b.samples) return InvalidArgument("JPEG-XL: output buffer too small");

    JxlDecoder* d = ThreadDecoder();
    if (d == nullptr) return InternalError("JxlDecoderCreate failed");
    if (JxlDecoderSubscribeEvents(d, JXL_DEC_BASIC_INFO | JXL_DEC_FULL_IMAGE) != JXL_DEC_SUCCESS)
        return InternalError("JxlDecoderSubscribeEvents failed");
    if (JxlDecoderSetInput(d, in.data(), in.size()) != JXL_DEC_SUCCESS) return Corrupt("JPEG-XL: cannot set the input");
    JxlDecoderCloseInput(d);
    if (JxlDecoderProcessInput(d) != JXL_DEC_BASIC_INFO) return Corrupt("JPEG-XL: no basic info");
    JxlBasicInfo info;
    if (JxlDecoderGetBasicInfo(d, &info) != JXL_DEC_SUCCESS) return Corrupt("JPEG-XL: no basic info");
    if (info.xsize != b.width || info.ysize != b.height)
        return Corrupt("JPEG-XL: codestream of " + std::to_string(info.xsize) + " x " + std::to_string(info.ysize) +
                       " pixels in a block of " + std::to_string(b.width) + " x " + std::to_string(b.height));
    if (info.bits_per_sample != b.bitsPerSample)
        return Corrupt("JPEG-XL: codestream of " + std::to_string(info.bits_per_sample) + "-bit samples, " +
                       std::to_string(b.bitsPerSample) + " expected");
    if (info.num_color_channels + info.num_extra_channels != b.samples)
        return Corrupt("JPEG-XL: invalid number of channels");

    // libjxl sets alpha_bits for gray + alpha + ... and for R, G, B, other, alpha: the first extra
    // channel is part of the main image when it is the alpha
    bool alphaEmbedded = false;
    if (info.alpha_bits != 0) {
        if ((info.num_color_channels == 3 || info.num_color_channels == 1) && info.num_extra_channels == 1) {
            alphaEmbedded = true;
        } else if (info.num_color_channels == 3 && info.num_extra_channels > 1) {
            JxlExtraChannelInfo eci{};
            alphaEmbedded = JxlDecoderGetExtraChannelInfo(d, 0, &eci) == JXL_DEC_SUCCESS && eci.type == JXL_CHANNEL_ALPHA;
        }
    }
    const uint32_t firstExtra = alphaEmbedded ? 1 : 0;
    const uint32_t mainChannels = info.num_color_channels + (alphaEmbedded ? 1 : 0);
    const uint32_t extras = info.num_extra_channels - firstExtra;

    JxlPixelFormat format{};
    format.data_type = type;
    format.endianness = JXL_NATIVE_ENDIAN;
    format.align = 0;
    thread_local std::vector<uint8_t> mainBuffer, extraBuffer;
    uint8_t* mainOut = out.data();
    if (extras > 0) {
        format.num_channels = 1;
        extraBuffer.resize(channelSize * extras);
        for (uint32_t i = 0; i < extras; ++i) {
            size_t size = 0;
            if (JxlDecoderExtraChannelBufferSize(d, &format, &size, i + firstExtra) != JXL_DEC_SUCCESS || size != channelSize)
                return Corrupt("JPEG-XL: unexpected extra channel size");
            if (JxlDecoderSetExtraChannelBuffer(d, &format, extraBuffer.data() + i * channelSize, channelSize,
                                                i + firstExtra) != JXL_DEC_SUCCESS)
                return Corrupt("JPEG-XL: cannot set an extra channel buffer");
        }
        mainBuffer.resize(channelSize * mainChannels);
        mainOut = mainBuffer.data();
    }
    format.num_channels = mainChannels;
    if (JxlDecoderProcessInput(d) != JXL_DEC_NEED_IMAGE_OUT_BUFFER) return Corrupt("JPEG-XL: cannot decode the image");
    if (JxlDecoderSetImageOutBuffer(d, &format, mainOut, channelSize * mainChannels) != JXL_DEC_SUCCESS)
        return Corrupt("JPEG-XL: cannot set the output buffer");
    if (JxlDecoderProcessInput(d) != JXL_DEC_FULL_IMAGE) return Corrupt("JPEG-XL: cannot decode the image");

    if (extras > 0) {
        // Each pixel: its main channels, then its extra channels
        const size_t mainPix = mainChannels * bytes, fullPix = static_cast<size_t>(b.samples) * bytes;
        for (size_t p = 0; p < pixels; ++p) {
            std::memcpy(out.data() + p * fullPix, mainBuffer.data() + p * mainPix, mainPix);
            for (uint32_t e = 0; e < extras; ++e)
                std::memcpy(out.data() + p * fullPix + mainPix + e * bytes, extraBuffer.data() + e * channelSize + p * bytes,
                            bytes);
        }
    }
    return Status::Ok();
}

}  // namespace ddbt::cog::codecs
