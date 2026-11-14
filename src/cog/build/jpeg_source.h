// SPDX-License-Identifier: MPL-2.0
//
// JpegSource: a JPEG file (JFIF, EXIF, Adobe) as GDAL's JPEG driver reads it (frmts/jpeg/
// jpgdataset.cpp), decoded with libjpeg-turbo:
//
//   - 8-bit samples are Byte bands, 12-bit ones UInt16 bands with NBITS 12; other precisions
//     (lossless JPEG of 2 to 16 bits) are refused, as GDAL refuses them;
//   - libjpeg's defaults (islow IDCT, fancy upsampling), every scanline at full resolution; the
//     EXIF orientation is not applied (GDAL's APPLY_ORIENTATION is off by default);
//   - grey: one band; RGB: three; YCbCr: converted to RGB by libjpeg (GDAL_JPEG_TO_RGB);
//     8-bit CMYK, and YCCK converted to CMYK by libjpeg, become RGB with GDAL's arithmetic
//     (R = C * K / 255, ...: the inverted CMYK of Adobe files); 12-bit CMYK and YCCK stay four
//     bands (Cyan, Magenta, Yellow, Black / YCbCr_Y, YCbCr_Cb, YCbCr_Cr, Black);
//   - a "Premature end of JPEG file" fails the read (GDAL's default), other libjpeg warnings
//     do not; a progressive image whose coefficients need more than 500 MB is refused (GDAL's
//     limit, JPEGMEM unset);
//   - the validity mask GDAL appends to the JPEGs it writes (a zlib-compressed bitmask after the
//     end of the image, then its offset) is the per-dataset mask, with GDAL's guess of its bit
//     order.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cog/build/image_source.h"
#include "core/io/byte_source.h"

namespace ddbt::cog::build {

class JpegSource final : public ImageSource {
public:
    static Expected<std::unique_ptr<JpegSource>> Open(const std::string& path, uint64_t memoryBudget);

protected:
    Status Decode(RowSink& sink) const override;

private:
    JpegSource(std::string path, uint64_t memoryBudget) : ImageSource(std::move(path), memoryBudget) {}
    Status DecodeMask(RowSink& sink) const;

    std::vector<uint8_t> bytes_;  // the whole file
    size_t imageBytes_ = 0;       // the JPEG datastream (before an appended mask)
    int precision_ = 8;
    int jpegColorSpace_ = 0;      // libjpeg's J_COLOR_SPACE of the datastream
    int outColorSpace_ = 0;       // what libjpeg decodes to
    int outComponents_ = 3;       // the components of a decoded scanline
    bool cmykToRgb_ = false;      // GDAL's CMYK -> RGB of 8-bit CMYK / YCCK
    size_t maskOffset_ = 0, maskSize_ = 0;
};

/// The file starts as a JPEG datastream (SOI then a marker), as GDAL's JPEG driver identifies it.
bool IsJpegFile(const io::ByteSource& bytes);

}  // namespace ddbt::cog::build
