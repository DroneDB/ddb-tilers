// SPDX-License-Identifier: MPL-2.0
//
// WebpSource: a WebP file as GDAL's WEBP driver reads it (frmts/webp/webpdataset.cpp), decoded
// with libwebp: RGB, or RGBA when the file has alpha (the fourth band is the alpha band), the
// size and the alpha flag read from the first 1024 bytes as GDAL reads them (a file whose header
// does not fit there is not opened), the whole image decoded at once (WebPDecodeRGB(A)Into).
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cog/build/image_source.h"
#include "core/io/byte_source.h"

namespace ddbt::cog::build {

class WebpSource final : public ImageSource {
public:
    static Expected<std::unique_ptr<WebpSource>> Open(const std::string& path, uint64_t memoryBudget);

protected:
    Status Decode(RowSink& sink) const override;

private:
    WebpSource(std::string path, uint64_t memoryBudget) : ImageSource(std::move(path), memoryBudget) {}

    std::vector<uint8_t> bytes_;  // the whole file
};

/// A RIFF WEBP file of a VP8, VP8L or VP8X chunk (GDAL's identification).
bool IsWebpFile(const io::ByteSource& bytes);

}  // namespace ddbt::cog::build
