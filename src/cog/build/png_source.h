// SPDX-License-Identifier: MPL-2.0
//
// PngSource: a PNG file as GDAL's PNG driver reads it (frmts/png/pngdataset.cpp), decoded with
// libspng (the samples as stored: no gamma, no tRNS expansion, no palette expansion):
//
//   - the bands of the colour type: grey, grey + alpha, RGB, RGBA, or the indices of a palette
//     image with its colour table; 16-bit samples are UInt16 bands (host order), samples of 1, 2
//     or 4 bits are unpacked to bytes with their NBITS;
//   - tRNS: the grey value of a grey image, the red, green and blue values of an RGB image (every
//     band's nodata, which must then be one value), the index of a palette image whose only
//     fully transparent entry it is, as GDAL makes them nodata values;
//   - critical chunks with a bad CRC fail, ancillary ones are skipped (libpng's defaults).
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cog/build/image_source.h"
#include "core/io/byte_source.h"

namespace ddbt::cog::build {

class PngSource final : public ImageSource {
public:
    static Expected<std::unique_ptr<PngSource>> Open(const std::string& path, uint64_t memoryBudget);

protected:
    Status Decode(RowSink& sink) const override;

private:
    PngSource(std::string path, uint64_t memoryBudget) : ImageSource(std::move(path), memoryBudget) {}

    std::vector<uint8_t> bytes_;  // the whole file
    int bitDepth_ = 8;
    bool interlaced_ = false;
};

/// The file starts with the PNG signature.
bool IsPngFile(const io::ByteSource& bytes);

}  // namespace ddbt::cog::build
