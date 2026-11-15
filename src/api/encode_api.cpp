// SPDX-License-Identifier: MPL-2.0
// ddbtEncodeImage: the tile encoders (src/core/encode) for images a host renders itself
// (thumbnails, map renders), with the rules and defaults of the tiles.
#include <cstdint>
#include <string>
#include <vector>

#include "api/api_common.h"
#include "cog/tiler/tile_error.h"
#include "core/encode/tile_encoder.h"
#include "core/encode/tile_image.h"
#include "ddbt/ddbt.h"

ddbtErr ddbtEncodeImage(const ddbtImage* image, const char* format, int quality, uint8_t** outBuffer,
                        int* outBufferSize) {
    namespace encode = ddbt::encode;
    using ddbt::cog::tiler::FailInvalid;
    if (outBuffer == nullptr || outBufferSize == nullptr)
        return ddbt::api::Fail(DDBT_ERR_INVALIDARGS, "Invalid output buffer: NULL");
    return ddbt::api::GuardedBuffer(outBuffer, outBufferSize, [&] {
        if (image == nullptr) FailInvalid("Invalid image: NULL");
        if (quality < 0 || quality > 100)
            FailInvalid("Invalid quality: " + std::to_string(quality) + " (0 to 100)");
        if (image->palette == nullptr && image->paletteSize != 0)
            FailInvalid("Invalid palette: NULL with " + std::to_string(image->paletteSize) + " entries");
        if (image->palette != nullptr && (image->paletteSize < 1 || image->paletteSize > 256))
            FailInvalid("Invalid palette: " + std::to_string(image->paletteSize) + " entries (1 to 256)");

        std::vector<encode::PaletteEntry> palette(image->palette != nullptr ? image->paletteSize : 0);
        for (uint32_t i = 0; i < palette.size(); ++i)
            palette[i] = {image->palette[4 * i], image->palette[4 * i + 1], image->palette[4 * i + 2],
                          image->palette[4 * i + 3]};
        encode::TileImage view;
        view.pixels = image->pixels;
        view.width = image->width;
        view.height = image->height;
        view.channels = image->channels;
        view.stride = image->stride;
        view.palette = palette.empty() ? nullptr : palette.data();
        view.paletteSize = image->paletteSize;
        if (const char* invalid = encode::ValidateTileImage(view))
            FailInvalid(std::string("Invalid image: ") + invalid);

        encode::EncodeOptions options;
        options.format = ddbt::api::ParseFormat(format);
        options.quality = quality;
        return encode::EncodeTile(view, options);
    });
}
