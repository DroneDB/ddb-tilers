// SPDX-License-Identifier: MPL-2.0
//
// The rules are those of GDAL 3.13.3's WEBP driver (WEBPDataset::Open, Uncompress): see
// webp_source.h.
#include "cog/build/webp_source.h"

#include <webp/decode.h>

#include <algorithm>
#include <climits>
#include <cstring>
#include <string>
#include <vector>

namespace ddbt::cog::build {

namespace {

// GDALOpenInfo's header: the bytes the driver identifies and describes the file with.
constexpr size_t kHeaderBytes = 1024;

}  // namespace

bool IsWebpFile(const io::ByteSource& bytes) {
    uint8_t h[20];
    if (bytes.Size() < sizeof(h) || !bytes.Read(0, h).ok()) return false;
    return std::memcmp(h, "RIFF", 4) == 0 && std::memcmp(h + 8, "WEBP", 4) == 0 &&
           (std::memcmp(h + 12, "VP8 ", 4) == 0 || std::memcmp(h + 12, "VP8L", 4) == 0 ||
            std::memcmp(h + 12, "VP8X", 4) == 0);
}

Expected<std::unique_ptr<WebpSource>> WebpSource::Open(const std::string& path, uint64_t memoryBudget) {
    auto src = io::OpenByteSource(path, io::SourceKind::kPread);
    if (!src.ok()) return src.status();
    const uint64_t size = (*src)->Size();
    // GDAL reads the file with a 32-bit size.
    if (size > UINT32_MAX) return Status(StatusCode::kUnsupported, "WebP file too large: " + path);
    auto bytes = (*src)->ReadVector(0, static_cast<size_t>(size));
    if (!bytes.ok()) return bytes.status();
    std::unique_ptr<WebpSource> s(new WebpSource(path, memoryBudget));
    s->bytes_ = std::move(*bytes);
    const size_t header = std::min(kHeaderBytes, s->bytes_.size());
    int w = 0, h = 0;
    if (!WebPGetInfo(s->bytes_.data(), header, &w, &h))
        return Status(StatusCode::kCorrupt, "WebP: cannot read the header: " + path);
    WebPBitstreamFeatures features{};
    if (WebPGetFeatures(s->bytes_.data(), header, &features) != VP8_STATUS_OK)
        return Status(StatusCode::kCorrupt, "WebP: cannot read the features: " + path);
    ImageInfo info;
    info.width = static_cast<uint32_t>(w);
    info.height = static_cast<uint32_t>(h);
    info.bands = features.has_alpha ? 4 : 3;
    info.interps = {"Red", "Green", "Blue", "Alpha"};
    info.interps.resize(static_cast<size_t>(info.bands));
    if (features.has_alpha) info.alphaBand = 3;
    if (w > 0 && h > 0 && w > INT_MAX / (h * info.bands))
        return Status(StatusCode::kUnsupported, "WebP: too large image: " + path);
    DDBT_TRY(s->SetInfo(std::move(info)));
    return s;
}

Status WebpSource::Decode(RowSink& sink) const {
    const ImageInfo& info = Info();
    const size_t stride = RowBytes();
    std::vector<uint8_t> image(stride * info.height);
    const uint8_t* r =
        info.bands == 4
            ? WebPDecodeRGBAInto(bytes_.data(), bytes_.size(), image.data(), image.size(), static_cast<int>(stride))
            : WebPDecodeRGBInto(bytes_.data(), bytes_.size(), image.data(), image.size(), static_cast<int>(stride));
    if (r == nullptr) return Status(StatusCode::kCorrupt, "WebP: cannot decode " + Path());
    for (uint32_t y = 0; y < info.height; ++y) DDBT_TRY(sink.Row(y, image.data() + static_cast<size_t>(y) * stride));
    return Status::Ok();
}

}  // namespace ddbt::cog::build
