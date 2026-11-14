// SPDX-License-Identifier: MPL-2.0
//
// The rules are those of GDAL 3.13.3's GIF driver (GIFDataset::Open, GIFAbstractRasterBand): see
// gif_source.h.
#include "cog/build/gif_source.h"

#include <gif_lib.h>

#include <algorithm>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace ddbt::cog::build {

namespace {

struct Reader {
    const uint8_t* data;
    size_t size;
    size_t at;
};

extern "C" int ReadGifBytes(GifFileType* gif, GifByteType* out, int n) {
    auto* r = static_cast<Reader*>(gif->UserData);
    const size_t want = n > 0 ? static_cast<size_t>(n) : 0;
    const size_t got = std::min(want, r->size - r->at);
    std::memcpy(out, r->data + r->at, got);
    r->at += got;
    return static_cast<int>(got);
}

// The file read whole by giflib (DGifSlurp), then visited; the images are freed afterwards.
Status Slurp(const std::vector<uint8_t>& bytes, const std::string& path,
             const std::function<Status(const GifFileType&)>& visit) {
    Reader r{bytes.data(), bytes.size(), 0};
    int error = 0;
    GifFileType* gif = DGifOpen(&r, ReadGifBytes, &error);
    if (gif == nullptr)
        return Status(StatusCode::kCorrupt, std::string("GIF: ") + GifErrorString(error) + ": " + path);
    Status s;
    if (DGifSlurp(gif) != GIF_OK || gif->SavedImages == nullptr || gif->ImageCount < 1) {
        s = Status(StatusCode::kCorrupt, std::string("GIF: ") + GifErrorString(gif->Error) + ": " + path);
    } else {
        s = visit(*gif);
    }
    DGifCloseFile(gif, &error);
    return s;
}

}  // namespace

bool IsGifFile(const io::ByteSource& bytes) {
    uint8_t head[6];
    if (bytes.Size() < sizeof(head) || !bytes.Read(0, head).ok()) return false;
    return std::memcmp(head, "GIF87a", 6) == 0 || std::memcmp(head, "GIF89a", 6) == 0;
}

Expected<std::unique_ptr<GifSource>> GifSource::Open(const std::string& path, uint64_t memoryBudget) {
    auto src = io::OpenByteSource(path, io::SourceKind::kPread);
    if (!src.ok()) return src.status();
    const uint64_t size = (*src)->Size();
    if (size > (uint64_t{1} << 40)) return Status(StatusCode::kUnsupported, "GIF file too large: " + path);
    auto bytes = (*src)->ReadVector(0, static_cast<size_t>(size));
    if (!bytes.ok()) return bytes.status();
    std::unique_ptr<GifSource> s(new GifSource(path, memoryBudget));
    s->bytes_ = std::move(*bytes);
    ImageInfo info;
    DDBT_TRY(Slurp(s->bytes_, path, [&](const GifFileType& gif) {
        const SavedImage& first = gif.SavedImages[0];
        const int w = first.ImageDesc.Width, h = first.ImageDesc.Height;
        if (w <= 0 || h <= 0) return Status(StatusCode::kCorrupt, "GIF: empty image: " + path);
        // GIFDataset::Open: every image of the first one's size with a colour map is a band.
        int bands = 0;
        for (int i = 0; i < gif.ImageCount; ++i) {
            const SavedImage& im = gif.SavedImages[i];
            if (im.ImageDesc.Width != w || im.ImageDesc.Height != h) continue;
            if (im.ImageDesc.ColorMap == nullptr && gif.SColorMap == nullptr) continue;
            ++bands;
        }
        if (bands != 1 || (first.ImageDesc.ColorMap == nullptr && gif.SColorMap == nullptr))
            return Status(StatusCode::kUnsupported,
                          "GIF: only files of one image with a colour map are supported (GDAL reads " +
                              std::to_string(bands) + " bands): " + path);
        info.width = static_cast<uint32_t>(w);
        info.height = static_cast<uint32_t>(h);
        info.bands = 1;
        info.interps = {"Palette"};
        // The transparent index of the graphic control extensions (the last one that sets it).
        int transparent = -1;
        for (int e = 0; e < first.ExtensionBlockCount; ++e) {
            const ExtensionBlock& b = first.ExtensionBlocks[e];
            if (b.Function != 0xF9 || b.ByteCount < 4) continue;
            if (!(b.Bytes[0] & 0x1)) continue;
            transparent = b.Bytes[3];
        }
        if (transparent >= 0) {
            info.noData = transparent;
            info.noDataText = std::to_string(transparent);
        }
        const ColorMapObject* map = first.ImageDesc.ColorMap != nullptr ? first.ImageDesc.ColorMap : gif.SColorMap;
        std::vector<std::array<uint8_t, 3>> table(static_cast<size_t>(std::max(map->ColorCount, 0)));
        for (size_t i = 0; i < table.size(); ++i)
            table[i] = {map->Colors[i].Red, map->Colors[i].Green, map->Colors[i].Blue};
        info.colorTable = std::move(table);
        return Status::Ok();
    }));
    DDBT_TRY(s->SetInfo(std::move(info)));
    return s;
}

Status GifSource::Decode(RowSink& sink) const {
    const ImageInfo& info = Info();
    return Slurp(bytes_, Path(), [&](const GifFileType& gif) {
        const SavedImage& first = gif.SavedImages[0];
        if (first.RasterBits == nullptr) return Status(StatusCode::kCorrupt, "GIF: no image data: " + Path());
        for (uint32_t y = 0; y < info.height; ++y)
            DDBT_TRY(sink.Row(y, first.RasterBits + static_cast<size_t>(y) * info.width));
        return Status::Ok();
    });
}

}  // namespace ddbt::cog::build
