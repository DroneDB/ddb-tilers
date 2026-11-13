// SPDX-License-Identifier: MPL-2.0
// GDALTiler::tile (plain and with visualisation parameters) over a RasterView. Compiled without
// floating-point contraction (sources.cmake): the stretch and the formulas must round as
// DroneDB's do.
#include "cog/tiler/cog_tiler.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

#include "cog/tiler/cog_source.h"
#include "cog/tiler/kernels.h"
#include "cog/tiler/raster_io.h"
#include "cog/tiler/stats.h"
#include "cog/tiler/stretch.h"
#include "cog/tiler/tile_error.h"
#include "core/cache/sharded_lru.h"
#include "core/tile/compat.h"

namespace ddbt::cog::tiler {

namespace {

using tile::compat::GQResult;

// Mode ----------------------------------------------------------------------------------------

std::atomic<int> g_mode{-1};

SamplingMode ModeFromEnv() {
    const char* e = std::getenv("DDBT_TILER_MODE");
    if (e != nullptr && std::strcmp(e, "default") == 0) return SamplingMode::kDefault;
    return SamplingMode::kCompat;
}

// Encoded-tile cache ----------------------------------------------------------------------------

struct EncodedKey {
    uint64_t source = 0;
    int z = 0, x = 0, y = 0;
    int size = 0;
    uint8_t format = 0, tms = 0, mode = 0;
    std::string style;
    bool operator==(const EncodedKey&) const = default;
};

struct EncodedKeyHash {
    size_t operator()(const EncodedKey& k) const noexcept {
        uint64_t h = k.source * 0x9E3779B97F4A7C15ull;
        const auto mix = [&h](uint64_t v) {
            h ^= v + 0x9E3779B97F4A7C15ull + (h << 6) + (h >> 2);
        };
        mix(static_cast<uint32_t>(k.z));
        mix(static_cast<uint32_t>(k.x));
        mix(static_cast<uint32_t>(k.y));
        mix(static_cast<uint64_t>(k.size) << 24 | static_cast<uint64_t>(k.format) << 16 | k.tms << 8 | k.mode);
        mix(std::hash<std::string>{}(k.style));
        return static_cast<size_t>(h);
    }
};

using TileCache = cache::ShardedLru<EncodedKey, std::vector<uint8_t>, EncodedKeyHash>;

TileCache& Tiles() {
    static TileCache cache([] {
        size_t mb = 256;
        if (const char* e = std::getenv("DDBT_TILE_CACHE_MB")) {
            const long long v = std::atoll(e);
            if (v >= 0) mb = static_cast<size_t>(v);
        }
        return mb << 20;
    }(),
                           64);
    return cache;
}

// The visualisation parameters in the encoded-tile key: every field prefixed with its length, so
// no two parameter sets give the same key whatever their text holds.
std::string StyleKey(const VisParams* v) {
    if (v == nullptr) return {};
    std::string k = "v";
    for (const std::string* f : {&v->preset, &v->bands, &v->formula, &v->bandFilter, &v->colormap, &v->rescale}) {
        k += std::to_string(f->size());
        k += ':';
        k += *f;
    }
    if (v->nodata) {
        char buf[40];
        std::snprintf(buf, sizeof(buf), "n%.17g", *v->nodata);
        k += buf;
    } else {
        k += '-';
    }
    return k;
}

// Working memory ----------------------------------------------------------------------------------

// Per-thread working buffers of the renders. They are reused from tile to tile; one grown past
// kTileScratchKeepBytes by a large tile is released when the request ends.
struct Scratch {
    std::vector<uint8_t> image, alpha, samples, plane;
};

Scratch& ThreadScratch() {
    thread_local Scratch s;
    return s;
}

void TrimScratch() noexcept {
    Scratch& s = ThreadScratch();
    for (std::vector<uint8_t>* v : {&s.image, &s.alpha, &s.samples, &s.plane})
        if (v->capacity() > kTileScratchKeepBytes) std::vector<uint8_t>().swap(*v);
}

struct ScratchTrim {
    ~ScratchTrim() { TrimScratch(); }
};

// The working memory a request may use (DDBT_TILE_REQUEST_MB, 1024 MiB by default).
size_t RequestBudgetBytes() {
    size_t mb = 1024;
    if (const char* e = std::getenv("DDBT_TILE_REQUEST_MB")) {
        const long long v = std::atoll(e);
        if (v > 0) mb = static_cast<size_t>(v);
    }
    return mb << 20;
}

// Refuses a render whose buffers would exceed the request budget, before allocating them.
void RequireRequestMemory(size_t bytes) {
    const size_t budget = RequestBudgetBytes();
    if (bytes > budget)
        Fail("The tile needs " + std::to_string((bytes + (size_t{1} << 20) - 1) >> 20) +
             " MiB of working memory, more than the " + std::to_string(budget >> 20) +
             " MiB a request may use (DDBT_TILE_REQUEST_MB)");
}

size_t ImageBytes(int size, int colorBands) {
    return static_cast<size_t>(size) * static_cast<size_t>(size) * (static_cast<size_t>(colorBands) + 1);
}

// Images ----------------------------------------------------------------------------------------

/// The tile picture: colorBands + alpha, interleaved, zeroed (transparent) unless the caller
/// writes every pixel.
struct Image {
    int size = 0;
    int colorBands = 1;
    std::vector<uint8_t>* px = nullptr;
    size_t Channels() const { return static_cast<size_t>(colorBands) + 1; }
    uint8_t* Data() { return px->data(); }
};

Image MakeImage(int size, int colorBands, bool zeroed = true) {
    std::vector<uint8_t>& buffer = ThreadScratch().image;
    Image im;
    im.size = size;
    im.colorBands = colorBands;
    if (zeroed) buffer.assign(ImageBytes(size, colorBands), 0);
    else buffer.resize(ImageBytes(size, colorBands));
    im.px = &buffer;
    return im;
}

encode::EncodeOptions Options(encode::TileFormat format) {
    encode::EncodeOptions o;
    o.format = format;
    return o;
}

/// DroneDB's TileEncoder for an image of 2 colour bands (2 data bands selected or shown): the
/// MEM dataset it hands to GDAL has the 2 bands (+ the alpha unless it is all 255), so the PNG
/// driver writes grey + alpha (band 1, band 2) when the tile is opaque and RGB (band 1, band 2,
/// alpha) otherwise; a fully transparent one is the blank of RGB zeros, opaque. WebP takes 3
/// colour bands (1, 2, 2); the JPEG driver refuses 2 bands.
encode::ByteBuffer EncodeTwoBands(const uint8_t* px, int size, bool allTransparent, encode::TileFormat format) {
    const size_t n = static_cast<size_t>(size) * static_cast<size_t>(size);
    if (format == encode::TileFormat::kJpeg)
        Fail("Cannot encode image: JPEG driver doesn't support 2 bands.  Must be 1 (grey), 3 (RGB) or 4 bands (CMYK).");
    bool opaque = true;
    for (size_t i = 0; i < n && opaque; ++i) opaque = px[i * 3 + 2] == 255;
    std::vector<uint8_t> out;
    encode::TileImage t;
    if (format == encode::TileFormat::kWebp) {
        out.resize(n * 4);
        for (size_t i = 0; i < n; ++i) {
            out[i * 4] = px[i * 3];
            out[i * 4 + 1] = px[i * 3 + 1];
            out[i * 4 + 2] = px[i * 3 + 1];
            out[i * 4 + 3] = px[i * 3 + 2];
        }
        t = encode::TileImage::Packed(out.data(), static_cast<uint32_t>(size), static_cast<uint32_t>(size), 4);
    } else if (opaque) {
        out.resize(n * 2);
        for (size_t i = 0; i < n; ++i) {
            out[i * 2] = px[i * 3];
            out[i * 2 + 1] = px[i * 3 + 1];
        }
        t = encode::TileImage::Packed(out.data(), static_cast<uint32_t>(size), static_cast<uint32_t>(size), 2);
    } else {
        out.resize(n * 3);
        if (!allTransparent) std::memcpy(out.data(), px, n * 3);
        else std::memset(out.data(), 0, n * 3);
        t = encode::TileImage::Packed(out.data(), static_cast<uint32_t>(size), static_cast<uint32_t>(size), 3);
    }
    encode::ByteBuffer bytes;
    encode::EncodeTile(t, Options(format), bytes);
    return bytes;
}

encode::ByteBuffer BlankTile(int size, int colorBands, encode::TileFormat format) {
    if (colorBands == 2) {
        const std::vector<uint8_t> zeros(static_cast<size_t>(size) * static_cast<size_t>(size) * 3, 0);
        return EncodeTwoBands(zeros.data(), size, true, format);
    }
    // The encoder keeps blank bytes for the process behind a mutex; each thread remembers the
    // references it got (they stay valid), so blank tiles take no shared lock after the first.
    struct Memo {
        int size;
        uint32_t channels;
        encode::TileFormat format;
        const std::vector<uint8_t>* bytes;
    };
    thread_local std::vector<Memo> memo;
    const uint32_t channels = colorBands >= 3 ? 3u : 1u;
    const std::vector<uint8_t>* found = nullptr;
    for (const auto& m : memo)
        if (m.size == size && m.channels == channels && m.format == format) found = m.bytes;
    if (found == nullptr) {
        found = &encode::BlankTileBytes(static_cast<uint32_t>(size), static_cast<uint32_t>(size), channels,
                                        Options(format));
        memo.push_back({size, channels, format, found});
    }
    const std::vector<uint8_t>& bytes = *found;
    encode::ByteBuffer out;
    out.assign(bytes.data(), bytes.size());
    return out;
}

encode::ByteBuffer Encode(Image& im, encode::TileFormat format, std::optional<bool> allOpaque) {
    if (im.colorBands == 2) {
        const size_t n = static_cast<size_t>(im.size) * static_cast<size_t>(im.size);
        bool transparent = true;
        for (size_t i = 0; i < n && transparent; ++i) transparent = im.Data()[i * 3 + 2] == 0;
        return EncodeTwoBands(im.Data(), im.size, transparent, format);
    }
    encode::TileImage t =
        encode::TileImage::Packed(im.Data(), static_cast<uint32_t>(im.size), static_cast<uint32_t>(im.size),
                                  static_cast<uint32_t>(im.Channels()));
    if (allOpaque && *allOpaque) {
        t.alphaKnown = true;
        t.allOpaque = true;
        t.allTransparent = false;
    }
    encode::ByteBuffer out;
    encode::EncodeTile(t, Options(format), out);
    return out;
}

/// vis_render.cpp defaultStretch (exact statistics).
std::vector<std::pair<double, double>> DefaultStretch(const CogSource& src, const std::vector<int>& bands,
                                                      const std::optional<double>& nodata) {
    std::vector<BandStatistics> stats;
    for (const int b : bands) stats.push_back(GetBandStatistics(src, b, nodata));
    std::vector<std::pair<double, double>> ranges;
    if (bands.size() == 1) {
        double lo = stats[0].min, hi = stats[0].max;
        if (lo == hi) hi += 0.1;
        if (lo == hi) Fail("Cannot scale values due to source min/max being equal");
        ranges.emplace_back(lo, hi);
        return ranges;
    }
    for (const auto& s : stats) ranges.push_back(StretchRange(s));
    return ranges;
}

// Placement --------------------------------------------------------------------------------------

/// Where the buffer of a read lands in the tile.
struct Placement {
    int x = 0, y = 0, w = 0, h = 0;
};

// The tile ------------------------------------------------------------------------------------------

struct Ctx {
    const TileRequest& req;
    std::shared_ptr<const CogSource> src;
    tile::compat::RasterTiling tiling;
    int row = 0;  // the TMS row (InternalRow)
    SamplingMode mode = SamplingMode::kCompat;
};

/// The read of a tile: the window and its placement (compat: geoQuery; default: the whole
/// tile). False when the tile has nothing to read (blank).
bool TileRead(Ctx& c, const RasterView& view, SamplePlan& plan, Placement& at) {
    GQResult g;
    if (!tile::compat::TileWindow(c.tiling, c.req.z, c.req.x, c.row, g)) return false;
    if (c.mode == SamplingMode::kCompat || view.Warped()) {
        plan = PlanRead(view, g.r.x, g.r.y, g.r.xsize, g.r.ysize, g.w.xsize, g.w.ysize);
        at = {g.w.x, g.w.y, g.w.xsize, g.w.ysize};
    } else {
        const auto b = c.tiling.mercator.tileBounds(c.req.x, c.row, c.req.z);
        plan = PlanTile(view, b.min.x, b.min.y, b.max.x, b.max.y, c.req.tileSize);
        at = {0, 0, c.req.tileSize, c.req.tileSize};
    }
    return true;
}

void PlaceRgba(Image& im, const Placement& at, const uint8_t* rgba) {
    const size_t ch = im.Channels();  // 4
    for (int j = 0; j < at.h; ++j) {
        uint8_t* d = im.Data() + (static_cast<size_t>(at.y + j) * static_cast<size_t>(im.size) + static_cast<size_t>(at.x)) * ch;
        std::memcpy(d, rgba + static_cast<size_t>(j) * static_cast<size_t>(at.w) * 4, static_cast<size_t>(at.w) * 4);
    }
}

/// GDALTiler::readBands + readAlpha (the plain tile; also the vis tile without a band choice).
encode::ByteBuffer ReadBandsTile(Ctx& c, const RasterView& view, int colorBands, const std::vector<int>* selected) {
    const int size = c.req.tileSize;
    SamplePlan plan;
    Placement at;
    if (!TileRead(c, view, plan, at)) return BlankTile(size, colorBands, c.req.format);

    // The mask first: a tile whose mask blocks are all 0 is blank without decoding imagery.
    const MaskSummary ms = PlanMaskSummary(view, plan);
    if (ms == MaskSummary::kAllZero) return BlankTile(size, colorBands, c.req.format);

    const CogSource& src = view.Source();
    const PixelType type = src.Type();
    const size_t ss = PixelTypeSize(type);
    const size_t pixels = static_cast<size_t>(at.w) * static_cast<size_t>(at.h);
    std::vector<int> bands;
    if (selected) bands = *selected;
    else
        for (int b = 1; b <= colorBands; ++b) bands.push_back(b);
    RequireRequestMemory(pixels * (ss * bands.size() + 2) + ImageBytes(size, colorBands));
    std::vector<uint8_t>& alpha = ThreadScratch().alpha;
    std::vector<uint8_t>& samples = ThreadScratch().samples;
    alpha.resize(pixels);
    samples.resize(pixels * ss * bands.size());

    bool bandsRead = false;
    const bool needBand1ForMask = view.AlphaBand() == 0 &&
                                  (src.Mask() == MaskKind::kNodata ||
                                   (view.NodataOverride() && src.Mask() != MaskKind::kPerDataset));
    if (ms == MaskSummary::kAllOne) {
        std::memset(alpha.data(), 255, pixels);
    } else if (needBand1ForMask && !bands.empty() && bands[0] == 1) {
        GatherBands(view, plan, bands, samples.data(), pixels * ss);
        bandsRead = true;
        GatherAlpha(view, plan, alpha.data(), samples.data());
    } else {
        GatherAlpha(view, plan, alpha.data());
        if (!needBand1ForMask && kernels::AllBytesEqual(alpha.data(), pixels, 0))
            return BlankTile(size, colorBands, c.req.format);  // nothing visible: no imagery decode
    }
    if (!bandsRead) GatherBands(view, plan, bands, samples.data(), pixels * ss);

    // A read that covers the tile writes every pixel: the image needs no clearing.
    const bool full = at.x == 0 && at.y == 0 && at.w == size && at.h == size;
    Image im = MakeImage(size, colorBands, !full);
    std::vector<std::pair<double, double>> ranges;
    if (type != PixelType::kU8) {
        if (selected) {
            // readSelectedBands: each band over mean +- 2.33 sigma of its statistics.
            for (const int b : bands) ranges.push_back(StretchRange(GetBandStatistics(src, b, view.NodataOverride())));
        } else {
            ranges = DefaultStretch(src, bands, view.NodataOverride());
        }
    }
    // The colour planes (Byte bands as they are, others stretched a row at a time into `plane`)
    // and the alpha, interleaved into the image row by row. A channel without a band (never
    // with the callers' band lists, one per colour channel) stays 0, as in the zeroed image.
    const size_t w = static_cast<size_t>(at.w);
    const size_t colours = static_cast<size_t>(colorBands);
    std::vector<uint8_t>& plane = ThreadScratch().plane;
    plane.assign(w * (colours + 1), 0);
    const uint8_t* rowPlanes[4];
    for (size_t j = 0; j < static_cast<size_t>(at.h); ++j) {
        for (size_t b = 0; b < colours; ++b) {
            if (b >= bands.size()) {
                rowPlanes[b] = plane.data() + colours * w;  // the zero row
            } else if (type == PixelType::kU8) {
                rowPlanes[b] = samples.data() + b * pixels + j * w;
            } else {
                StretchSamples(type, samples.data() + (b * pixels + j * w) * ss, w, ranges[b].first,
                               ranges[b].second, selected != nullptr, plane.data() + b * w, 1);
                rowPlanes[b] = plane.data() + b * w;
            }
        }
        rowPlanes[colours] = alpha.data() + j * w;
        kernels::InterleavePlanes(rowPlanes, im.Channels(), w,
                                  im.Data() + ((static_cast<size_t>(at.y) + j) * static_cast<size_t>(size) +
                                               static_cast<size_t>(at.x)) * im.Channels());
    }
    std::optional<bool> opaque;
    if (full && kernels::AllBytesEqual(alpha.data(), pixels, 255)) opaque = true;
    return Encode(im, c.req.format, opaque);
}

// Visualisation renders (vis_render.cpp) -----------------------------------------------------------

/// detectBandNodata of the view.
struct ViewNodata {
    int alphaBandIdx = -1;  // 0-based
    std::vector<std::optional<double>> bands;
    bool datasetMask = false;
};

ViewNodata DetectViewNodata(const RasterView& view) {
    ViewNodata n;
    const int count = view.BandCount();
    n.bands.resize(static_cast<size_t>(count));
    for (int b = 0; b < count; ++b) {
        if (view.Interp(b + 1) == ColorInterp::kAlpha) n.alphaBandIdx = b;
        n.bands[static_cast<size_t>(b)] = view.Nodata(b + 1);
    }
    if (n.alphaBandIdx < 0 && count > 0) n.datasetMask = view.HasDatasetMask();
    return n;
}

inline bool IsNodataFloat(float v, const std::optional<double>& nd) {
    if (!nd) return false;
    if (std::isnan(*nd)) return std::isnan(v);
    return v == static_cast<float>(*nd);
}

/// readPhysicalBands: the bands marked in `keep` as float, masked with the sentinel, scaled. The
/// mask is the dataset mask, the warped grid outside the source, a zero of the alpha band and the
/// NoData value of any band marked in `test` (NodataMask::AnyBand over the bands it reads). Bands
/// are gathered one at a time through one buffer, so memory does not grow with the band count; a
/// band neither kept nor able to mask (no NoData value, not alpha) is not read.
void ReadPhysicalBands(const RasterView& view, const ViewNodata& nd, const std::vector<bool>& keep,
                       const std::vector<bool>& test, const SamplePlan& plan, std::vector<std::vector<float>>& bands,
                       std::vector<float*>& ptrs) {
    const CogSource& src = view.Source();
    const int count = view.BandCount();
    const size_t pixels = static_cast<size_t>(plan.bufW) * static_cast<size_t>(plan.bufH);
    const PixelType type = src.Type();
    const size_t ss = PixelTypeSize(type);
    bands.assign(static_cast<size_t>(count), {});
    ptrs.assign(static_cast<size_t>(count), nullptr);
    std::vector<uint8_t> masked(pixels, 0);
    if (nd.datasetMask) {
        GatherDatasetMask(view, plan, masked.data());
        for (size_t i = 0; i < pixels; ++i) masked[i] = masked[i] == 0 ? 1 : 0;
    }
    // A warped view off the source (inside the COG grid): the built COG's transparency.
    const bool warpTransparent =
        src.AlphaBand() > 0 || src.Mask() != MaskKind::kAllValid || view.NodataOverride().has_value();
    if (plan.pointwise && warpTransparent)
        for (size_t i = 0; i < pixels; ++i)
            if (plan.sx[i] == -2) masked[i] = 1;
    std::vector<uint8_t> raw;
    for (int b = 0; b < count; ++b) {
        const size_t bi = static_cast<size_t>(b);
        const bool isAlpha = b == nd.alphaBandIdx;
        const std::optional<double>& bandNodata = nd.bands[bi];
        const bool masks = (test[bi] || keep[bi]) && (isAlpha || bandNodata.has_value());
        if (!keep[bi] && !masks) continue;
        raw.resize(pixels * ss);
        const int band1 = b + 1;
        GatherBands(view, plan, std::span<const int>(&band1, 1), raw.data(), pixels * ss);
        float* out = nullptr;
        if (keep[bi]) {
            bands[bi].resize(pixels);
            out = bands[bi].data();
            ptrs[bi] = out;
        }
        for (size_t i = 0; i < pixels; ++i) {
            const float v = static_cast<float>(SampleToDouble(type, raw.data() + i * ss));
            if (out != nullptr) out[i] = v;
            if (masks && masked[i] == 0 && (isAlpha ? v == 0.0f : IsNodataFloat(v, bandNodata))) masked[i] = 1;
        }
    }
    for (int b = 0; b < count; ++b) {
        float* v = ptrs[static_cast<size_t>(b)];
        if (v == nullptr) continue;
        for (size_t i = 0; i < pixels; ++i)
            if (masked[i] != 0) v[i] = kNodataSentinel;
        const BandMeta& bm = src.Band(b + 1);
        if (!((bm.hasScale && bm.scale != 1.0) || (bm.hasOffset && bm.offset != 0.0))) continue;
        const double m = bm.hasScale ? bm.scale : 1.0;
        const double k = bm.hasOffset ? bm.offset : 0.0;
        for (size_t i = 0; i < pixels; ++i)
            if (v[i] != kNodataSentinel) v[i] = static_cast<float>(static_cast<double>(v[i]) * m + k);
    }
}

/// The working memory of ReadPhysicalBands for `kept` float bands, plus `perPixel` more bytes.
size_t PhysicalReadBytes(size_t pixels, PixelType type, size_t kept, size_t perPixel) {
    return pixels * (PixelTypeSize(type) + 1 + 4 * kept + perPixel);
}

RasterDataKind DataKindOf(const RasterView& view) {
    RasterDataKind k;
    k.floatingPoint = true;
    for (int b = 1; b <= view.BandCount(); ++b) {
        if (view.Interp(b) == ColorInterp::kAlpha) continue;
        k.bandCount++;
        const bool isFloat = PixelTypeIsFloat(view.Type());
        if (k.floatingPoint && (k.typeName.empty() || !isFloat)) k.typeName = PixelTypeName(view.Type());
        if (!isFloat) k.floatingPoint = false;
    }
    return k;
}

struct BandStyle {
    int band = 0;
    const Colormap* colormap = nullptr;
    std::pair<float, float> range;
};

/// planVisBandStyle on the file.
std::optional<BandStyle> PlanVisBandStyle(const CogSource& src, const VisParams& vis, int maxBand) {
    if (!vis.formula.empty() || vis.colormap.empty()) return std::nullopt;
    const std::vector<int> chosen =
        vis.bands.empty() && vis.preset.empty() ? std::vector<int>{} : ResolveVisBands(src, vis, maxBand);
    const int band = StyledBand(src.DataBandCount(), vis.colormap, chosen);
    if (band == 0) return std::nullopt;
    BandStyle plan;
    plan.band = band;
    plan.colormap = FindColormap(vis.colormap);
    if (plan.colormap == nullptr) FailInvalid("Unknown colormap: " + vis.colormap);
    if (!vis.rescale.empty()) {
        plan.range = ParseRescaleRange(vis.rescale);
        return plan;
    }
    std::pair<double, double> stretch;
    try {
        stretch = DefaultStretch(src, {band}, vis.nodata)[0];
    } catch (const NoValidPixelsError&) {
        plan.range = {0.0f, 1.0f};
        return plan;
    }
    const BandMeta& bm = src.Band(band);
    const double m = bm.hasScale ? bm.scale : 1.0, k = bm.hasOffset ? bm.offset : 0.0;
    const auto [lo, hi] = stretch;
    plan.range = {static_cast<float>(std::min(lo * m + k, hi * m + k)), static_cast<float>(std::max(lo * m + k, hi * m + k))};
    return plan;
}

encode::ByteBuffer StyledTile(Ctx& c, const BandStyle& style, const VisParams& vis) {
    const RasterView view(*c.src, vis.nodata);
    const int size = c.req.tileSize;
    SamplePlan plan;
    Placement at;
    if (!TileRead(c, view, plan, at)) return BlankTile(size, 3, c.req.format);
    const ViewNodata nd = DetectViewNodata(view);
    const int count = view.BandCount();
    if (style.band < 1 || style.band > count)
        FailInvalid("Band index " + std::to_string(style.band) + " out of range (1-" + std::to_string(count) + ")");
    std::vector<bool> read(static_cast<size_t>(count), false);
    read[static_cast<size_t>(style.band - 1)] = true;
    if (nd.alphaBandIdx >= 0) read[static_cast<size_t>(nd.alphaBandIdx)] = true;
    const size_t pixels = static_cast<size_t>(at.w) * static_cast<size_t>(at.h);
    RequireRequestMemory(PhysicalReadBytes(pixels, view.Type(), nd.alphaBandIdx >= 0 ? 2 : 1, 4) + ImageBytes(size, 3));
    std::vector<std::vector<float>> bands;
    std::vector<float*> ptrs;
    ReadPhysicalBands(view, nd, read, read, plan, bands, ptrs);
    std::vector<uint8_t> rgba(pixels * 4);
    ApplyColormap(ptrs[static_cast<size_t>(style.band - 1)], rgba.data(), pixels, *style.colormap, style.range.first,
                  style.range.second, kNodataSentinel);
    Image im = MakeImage(size, 3);
    PlaceRgba(im, at, rgba.data());
    return Encode(im, c.req.format, std::nullopt);
}

encode::ByteBuffer FormulaTile(Ctx& c, const RasterView& view, const VisParams& vis) {
    const int size = c.req.tileSize;
    SamplePlan plan;
    Placement at;
    if (!TileRead(c, view, plan, at)) return BlankTile(size, 3, c.req.format);
    const CogSource& src = view.Source();
    // planFormula
    const int count = view.BandCount();
    const BandFilter filter = !vis.bandFilter.empty() ? ParseFilter(vis.bandFilter, count) : AutoDetectFilter(src);
    const Formula* formula = FindFormula(vis.formula);
    if (formula == nullptr) FailRefused("Unknown formula: " + vis.formula);
    const std::string refusal = RenderRefusal(*formula, filter, DataKindOf(view));
    if (!refusal.empty()) FailRefused(refusal);
    const std::string cmapId = vis.colormap.empty() ? "rdylgn" : vis.colormap;
    const Colormap* cmap = FindColormap(cmapId);
    if (cmap == nullptr) FailInvalid("Unknown colormap: " + cmapId);
    const ViewNodata nd = DetectViewNodata(view);
    const TemperatureUnit unit = BandTemperatureUnit(count >= 1 ? src.Band(1).unitType : std::string());

    // The bands ApplyFormula looks at: those of the filter (each one's NoData / non-finite values
    // void the result) and, for the thermal formulas, band 1. Every band can mask a pixel with its
    // NoData value (NodataMask::AnyBand), read as a stream.
    std::vector<bool> keep(static_cast<size_t>(count), false);
    size_t kept = 0;
    const auto mark = [&](int b) {
        if (b >= 0 && b < count && !keep[static_cast<size_t>(b)]) {
            keep[static_cast<size_t>(b)] = true;
            kept++;
        }
    };
    for (const char symbol : {'R', 'G', 'B', 'N', 'e'}) mark(filter.Get(symbol));
    if (ThermalFormulaUnit(formula->id)) mark(0);
    const std::vector<bool> test(static_cast<size_t>(count), true);
    const size_t pixels = static_cast<size_t>(at.w) * static_cast<size_t>(at.h);
    RequireRequestMemory(PhysicalReadBytes(pixels, view.Type(), kept, 12) + ImageBytes(size, 3));
    std::vector<std::vector<float>> bands;
    std::vector<float*> ptrs;
    ReadPhysicalBands(view, nd, keep, test, plan, bands, ptrs);
    std::vector<float> result(pixels);
    ApplyFormula(*formula, filter, ptrs, result.data(), pixels, kNodataSentinel, unit);

    std::pair<float, float> range;
    if (!vis.rescale.empty()) {
        range = ParseRescaleRange(vis.rescale);
    } else if (formula->hasRange && formula->rangeMin != formula->rangeMax) {
        range = {static_cast<float>(formula->rangeMin), static_cast<float>(formula->rangeMax)};
    } else {
        std::vector<float> valid;
        valid.reserve(pixels);
        for (const float v : result)
            if (v != kNodataSentinel && !std::isnan(v)) valid.push_back(v);
        if (valid.empty()) {
            range = {0.0f, 1.0f};
        } else {
            std::sort(valid.begin(), valid.end());
            range = {valid[static_cast<size_t>(static_cast<double>(valid.size()) * 0.02)],
                     valid[std::min(valid.size() - 1, static_cast<size_t>(static_cast<double>(valid.size()) * 0.98))]};
        }
    }
    std::vector<uint8_t> rgba(pixels * 4);
    ApplyColormap(result.data(), rgba.data(), pixels, *cmap, range.first, range.second, kNodataSentinel);
    Image im = MakeImage(size, 3);
    PlaceRgba(im, at, rgba.data());
    return Encode(im, c.req.format, std::nullopt);
}

encode::ByteBuffer PlainTile(Ctx& c) {
    const RasterView view(*c.src);
    const int nBands = view.DataBandCount();
    const int colorBands = std::max(1, std::min(3, nBands));
    return ReadBandsTile(c, view, colorBands, nullptr);
}

encode::ByteBuffer VisTile(Ctx& c, const VisParams& vis) {
    const bool plain = vis.Plain();
    const auto box = tile::compat::MinMaxCoordsForZ(c.tiling.mercator, c.tiling.bounds.oMinX, c.tiling.bounds.oMinY,
                                                    c.tiling.bounds.oMaxX, c.tiling.bounds.oMaxY, c.req.z);
    if (!box.Contains(c.req.x, c.row)) {
        if (!plain) Fail("Out of bounds");
        return PlainTile(c);
    }
    const int nBands = c.src->DataBandCount();
    if (const auto style = PlanVisBandStyle(*c.src, vis, nBands)) return StyledTile(c, *style, vis);
    if (plain) return PlainTile(c);

    const RasterView view(*c.src, vis.nodata);
    if (!vis.formula.empty()) return FormulaTile(c, view, vis);

    const std::vector<int> selected = ResolveVisBands(*c.src, vis, nBands);
    const int outBands = std::max(1, std::min(3, selected.empty() ? nBands : static_cast<int>(selected.size())));
    if (selected.empty()) return ReadBandsTile(c, view, outBands, nullptr);
    // readSelectedBands: Byte bands as they are, others over mean +- 2.33 sigma.
    const std::vector<int> chosen(selected.begin(), selected.begin() + outBands);
    return ReadBandsTile(c, view, outBands, &chosen);
}

}  // namespace

// Stretch ---------------------------------------------------------------------------------------

/// vis_render.cpp stretchRange.
std::pair<double, double> StretchRange(const BandStatistics& s) {
    double lo = std::max(s.mean - 2.33 * s.stdDev, s.min);
    double hi = std::min(s.mean + 2.33 * s.stdDev, s.max);
    if (lo >= hi) {
        lo = s.min;
        hi = s.max;
    }
    if (lo >= hi) hi = lo + 1.0;
    return {lo, hi};
}

/// A 2^bits LUT of StretchToByte for 8 / 16-bit samples (bit-identical: the same expression per
/// value), kept per thread for the last few (source, band, range).
static const uint8_t* StretchLut(PixelType type, double lo, double hi) {
    struct Entry {
        PixelType type;
        double lo, hi;
        std::vector<uint8_t> lut;
    };
    thread_local std::vector<Entry> luts;
    for (auto& e : luts)
        if (e.type == type && e.lo == lo && e.hi == hi) return e.lut.data();
    Entry e{type, lo, hi, {}};
    switch (type) {
        case PixelType::kU8:
            e.lut.resize(256);
            for (int v = 0; v < 256; ++v) e.lut[static_cast<size_t>(v)] = StretchToByte(v, lo, hi);
            break;
        case PixelType::kI8:
            e.lut.resize(256);
            for (int v = -128; v < 128; ++v) e.lut[static_cast<uint8_t>(static_cast<int8_t>(v))] = StretchToByte(v, lo, hi);
            break;
        case PixelType::kU16:
            e.lut.resize(65536);
            for (int v = 0; v < 65536; ++v) e.lut[static_cast<size_t>(v)] = StretchToByte(v, lo, hi);
            break;
        case PixelType::kI16:
            e.lut.resize(65536);
            for (int v = -32768; v < 32768; ++v)
                e.lut[static_cast<uint16_t>(static_cast<int16_t>(v))] = StretchToByte(v, lo, hi);
            break;
        default: return nullptr;
    }
    if (luts.size() >= 8) luts.erase(luts.begin());
    luts.push_back(std::move(e));
    return luts.back().lut.data();
}

/// StretchToByte of n samples of type T (viaFloat: rounded to float first).
template <typename T>
static void StretchTyped(const uint8_t* src, size_t n, double lo, double hi, bool viaFloat, uint8_t* dst,
                         size_t stride) {
    for (size_t i = 0; i < n; ++i) {
        T sample;
        std::memcpy(&sample, src + i * sizeof(T), sizeof(T));
        double v = static_cast<double>(sample);
        if (viaFloat) v = static_cast<double>(static_cast<float>(v));
        dst[i * stride] = StretchToByte(v, lo, hi);
    }
}

/// Stretches n native samples to bytes at dst (every `stride` bytes). viaFloat: the samples
/// were read as Float32 first (readSelectedBands), so 32-bit integers round to float. Float
/// samples with a range whose results stay in [0, 256) go through the Highway kernels (the same
/// double operations, lane by lane).
void StretchSamples(PixelType type, const uint8_t* src, size_t n, double lo, double hi, bool viaFloat, uint8_t* dst,
                    size_t stride) {
    if (const uint8_t* lut = StretchLut(type, lo, hi)) {
        if (PixelTypeSize(type) == 1) {
            for (size_t i = 0; i < n; ++i) dst[i * stride] = lut[src[i]];
        } else {
            for (size_t i = 0; i < n; ++i) {
                uint16_t v;
                std::memcpy(&v, src + i * 2, 2);
                dst[i * stride] = lut[v];
            }
        }
        return;
    }
    if (stride == 1 && kernels::StretchVectorizable(lo, hi)) {
        if (type == PixelType::kF32) return kernels::StretchF32(src, n, lo, hi, dst);
        if (type == PixelType::kF64 && !viaFloat) return kernels::StretchF64(src, n, lo, hi, dst);
    }
    switch (type) {
        case PixelType::kU32: return StretchTyped<uint32_t>(src, n, lo, hi, viaFloat, dst, stride);
        case PixelType::kI32: return StretchTyped<int32_t>(src, n, lo, hi, viaFloat, dst, stride);
        case PixelType::kF32: return StretchTyped<float>(src, n, lo, hi, viaFloat, dst, stride);
        case PixelType::kF64: return StretchTyped<double>(src, n, lo, hi, viaFloat, dst, stride);
        default: break;  // 8 / 16-bit samples took the LUT
    }
}


SamplingMode GetSamplingMode() {
    int m = g_mode.load(std::memory_order_relaxed);
    if (m < 0) {
        m = static_cast<int>(ModeFromEnv());
        g_mode.store(m, std::memory_order_relaxed);
    }
    return static_cast<SamplingMode>(m);
}

void SetSamplingMode(SamplingMode mode) { g_mode.store(static_cast<int>(mode), std::memory_order_relaxed); }

cache::CacheStats TileCacheStats() { return Tiles().Stats(); }
void SetTileCacheBudget(size_t bytes) { Tiles().SetBudget(bytes); }
size_t TileCacheBudget() { return Tiles().BudgetBytes(); }
void ClearTileCache() { Tiles().Clear(); }

size_t TileScratchBytes() {
    const Scratch& s = ThreadScratch();
    return std::max({s.image.capacity(), s.alpha.capacity(), s.samples.capacity(), s.plane.capacity()});
}

namespace {
void RequireTileSize(int tileSize) {
    if (tileSize <= 0 || (tileSize & (tileSize - 1)) != 0) Fail("Tile size must be a power of 2 greater than 0");
    if (tileSize > kMaxTileSize) FailInvalid("Tile size must be at most " + std::to_string(kMaxTileSize));
}
}  // namespace

tile::compat::TileExtent Extent(const std::string& path, int tileSize) {
    RequireTileSize(tileSize);
    const RasterView geometry(*CogSource::Get(path));
    const tile::compat::RasterTiling t(tileSize, geometry.GeoTransform().data(), geometry.Width(), geometry.Height());
    tile::compat::TileExtent e;
    e.minX = t.bounds.oMinX;
    e.minY = t.bounds.oMinY;
    e.maxX = t.bounds.oMaxX;
    e.maxY = t.bounds.oMaxY;
    e.zooms = t.zooms;
    return e;
}

encode::ByteBuffer RenderTile(const TileRequest& req) {
    RequireTileSize(req.tileSize);
    if (req.z < 0 || req.z > kMaxZoom || req.x < 0 || req.y < 0 || req.x >= (1 << req.z) || req.y >= (1 << req.z))
        FailInvalid("Invalid tile coordinates");
    std::shared_ptr<const CogSource> src = CogSource::Get(req.path);
    const SamplingMode mode = GetSamplingMode();

    EncodedKey key;
    key.source = src->Id();
    key.z = req.z;
    key.x = req.x;
    key.y = req.y;
    key.size = req.tileSize;
    key.format = static_cast<uint8_t>(req.format);
    key.tms = req.tms ? 1 : 0;
    key.mode = static_cast<uint8_t>(mode);
    key.style = StyleKey(req.vis);
    TileCache& tiles = Tiles();
    const bool useCache = !req.noCache && tiles.BudgetBytes() > 0;
    if (useCache) {
        if (auto hit = tiles.Get(key)) {
            encode::ByteBuffer out;
            out.assign(hit->data(), hit->size());
            return out;
        }
    }

    const ScratchTrim trim;  // a large tile's buffers are released when the request ends
    const RasterView geometry(*src);
    Ctx c{req, src, tile::compat::RasterTiling(req.tileSize, geometry.GeoTransform().data(), geometry.Width(),
                                               geometry.Height()),
          tile::compat::InternalRow(req.y, req.z, req.tms), mode};
    encode::ByteBuffer out = req.vis != nullptr ? VisTile(c, *req.vis) : PlainTile(c);
    if (useCache) tiles.Put(key, std::vector<uint8_t>(out.data(), out.data() + out.size()));
    return out;
}

}  // namespace ddbt::cog::tiler
