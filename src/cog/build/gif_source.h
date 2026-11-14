// SPDX-License-Identifier: MPL-2.0
//
// GifSource: a GIF file as GDAL's GIF driver reads it (frmts/gif/gifdataset.cpp,
// gifabstractdataset.cpp), decoded with giflib (DGifSlurp, which de-interlaces): the indices of
// the first image as one palette band (GIF images are as large as the first image they hold, not
// as the logical screen), the image's colour map (else the global one) as its colour table, and
// the transparent index of its graphic control extension as nodata. GDAL makes every further
// image of the same size another band; such files (animations) are refused.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cog/build/image_source.h"
#include "core/io/byte_source.h"

namespace ddbt::cog::build {

class GifSource final : public ImageSource {
public:
    static Expected<std::unique_ptr<GifSource>> Open(const std::string& path, uint64_t memoryBudget);

protected:
    Status Decode(RowSink& sink) const override;

private:
    GifSource(std::string path, uint64_t memoryBudget) : ImageSource(std::move(path), memoryBudget) {}

    std::vector<uint8_t> bytes_;  // the whole file
};

/// The file starts with "GIF87a" or "GIF89a".
bool IsGifFile(const io::ByteSource& bytes);

}  // namespace ddbt::cog::build
