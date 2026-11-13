// SPDX-License-Identifier: MPL-2.0
#include "cog/build/cog_builder.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <stdexcept>

#include "cog/build/pipeline.h"
#include "cog/build/source.h"
#include "cog/build/stats.h"
#include "cog/codecs/jpeg.h"
#include "cog/pyramid/overview.h"
#include "cog/warp/warp_producer.h"
#include "core/exec/parallel_for.h"
#include "core/exec/task_group.h"

namespace ddbt::cog::build {

const char* TransparencyName(Transparency t) noexcept {
    switch (t) {
        case Transparency::kNone: return "none";
        case Transparency::kMask: return "mask";
        case Transparency::kAlpha: return "alpha";
        case Transparency::kNoData: return "nodata";
    }
    return "none";
}

bool IsFloatType(tiff::SampleType t) {
    return t == tiff::SampleType::kF16 || t == tiff::SampleType::kF32 || t == tiff::SampleType::kF64;
}

// Photometric and ExtraSamples GDAL's GTiff CreateCopy writes for bands with these colour
// interpretations (RGB when the first three are red, green, blue; alpha extra samples as
// unassociated alpha).
void SetPhotometric(Plan& p, const std::vector<std::string>& interps) {
    const int n = p.bands;
    int colour = 1;
    if (n >= 3 && interps.size() >= 3 && interps[0] == "Red" && interps[1] == "Green" && interps[2] == "Blue") {
        p.photometric = tiff::photometric::kRgb;
        colour = 3;
    } else {
        p.photometric = tiff::photometric::kMinIsBlack;
    }
    p.extraSamples.clear();
    for (int b = colour; b < n; ++b)
        p.extraSamples.push_back(static_cast<size_t>(b) < interps.size() && interps[static_cast<size_t>(b)] == "Alpha"
                                     ? tiff::extrasample::kUnassociatedAlpha
                                     : tiff::extrasample::kUnspecified);
}

// GDAL's GetMaskBand alpha rule: the last band of a 2 / 4 band 8 / 16-bit raster.
bool AlphaMaskRule(int bands, int alphaBand, tiff::SampleType t) {
    return (bands == 2 || bands == 4) && alphaBand == bands - 1 &&
           (t == tiff::SampleType::kU8 || t == tiff::SampleType::kU16);
}

namespace {

using encode::ByteBuffer;
using encode::Codec;

// ---------------------------------------------------------------------------------------------
// Plan helpers

int ConfiguredJpegQuality() {
    const char* e = std::getenv("DDB_COG_JPEG_QUALITY");
    if (e == nullptr) return 75;
    return std::clamp(std::atoi(e), 1, 100);
}


// GDAL_METADATA of the output: the source's items (minus its IMAGE_STRUCTURE domain) and the
// COG driver's OVERVIEW_RESAMPLING=CUBIC last.
std::string OutputMetadata(const std::string& src, bool dropStatistics) {
    std::string items;
    if (!src.empty()) {
        const size_t open = src.find("<GDALMetadata>");
        const size_t close = src.rfind("</GDALMetadata>");
        if (open != std::string::npos && close != std::string::npos && close > open) {
            std::string body = src.substr(open + 14, close - open - 14);
            size_t pos = 0;
            while (pos < body.size()) {
                size_t nl = body.find('\n', pos);
                if (nl == std::string::npos) nl = body.size();
                const std::string line = body.substr(pos, nl - pos);
                // gdalwarp does not copy band statistics (STATISTICS_*): the warp changes them.
                const bool stats = dropStatistics && line.find("name=\"STATISTICS_") != std::string::npos;
                if (line.find("<Item") != std::string::npos && line.find("domain=\"IMAGE_STRUCTURE\"") == std::string::npos &&
                    !stats)
                    items += line + "\n";
                pos = nl + 1;
            }
        }
    }
    return "<GDALMetadata>\n" + items +
           "  <Item name=\"OVERVIEW_RESAMPLING\" domain=\"IMAGE_STRUCTURE\">CUBIC</Item>\n</GDALMetadata>\n";
}


void SetLossless(Plan& p, const BuildOptions& o) {
    p.codec = o.losslessCodec == Codec::kJpeg || o.losslessCodec == Codec::kNone ? Codec::kLzw : o.losslessCodec;
    p.predictor = IsFloatType(p.type) ? 3 : 2;
    p.level = o.level;
}


Expected<Plan> OwnGridPlan(const SourceRaster& src, const BuildOptions& o) {
    Plan p;
    p.ownGrid = true;
    p.width = src.Width();
    p.height = src.Height();
    p.blockSize = src.BlockWidth();
    p.geoTransform = *src.Georef().geoTransform;
    p.overviews = pyramid::OverviewSizes(p.width, p.height, p.blockSize, false);
    p.bands = src.Bands();
    p.type = src.Type();
    std::vector<std::string> interps;
    for (int b = 0; b < p.bands; ++b) interps.push_back(src.ColorInterp(b));
    SetPhotometric(p, interps);
    if (src.NoData()) {
        p.transparency = Transparency::kNoData;
        p.nodata = src.NoData();
        p.nodataText = src.NoDataText();
    } else if (src.MaskIfd()) {
        p.transparency = Transparency::kMask;
    } else if (src.AlphaBand() >= 0) {
        p.transparency = Transparency::kAlpha;
        p.alphaBand = src.AlphaBand();
        p.alphaMasksOverviews = AlphaMaskRule(p.bands, p.alphaBand, p.type);
    }
    // cogOwnGridCreationOptions: JPEG at the source's quality for a 3-band 8-bit JPEG source
    // without nodata, LZW + predictor otherwise.
    const std::optional<int> q = src.IsJpeg() ? std::optional<int>(src.JpegQuality().value_or(ConfiguredJpegQuality()))
                                              : std::nullopt;
    const bool jpeg = !o.noJpeg && q && p.bands == 3 && p.type == tiff::SampleType::kU8 && !src.NoData();
    if (jpeg) {
        p.codec = Codec::kJpeg;
        p.jpegQuality = std::min(*q, 100);
        p.photometric = tiff::photometric::kYCbCr;
    } else {
        SetLossless(p, o);
    }
    p.gdalMetadata = OutputMetadata(src.Main().gdalMetadata, false);
    return p;
}

// ---------------------------------------------------------------------------------------------
// Own grid: tile (col, row) of the output is block (col, row) of the source.

class OwnGridProducer final : public TileProducer {
public:
    OwnGridProducer(const SourceRaster& src, const Plan& plan) : src_(src), plan_(plan) {}

    void Produce(uint32_t col, Strip& s) override {
        thread_local std::vector<uint8_t> block, mask;
        block.resize(src_.BlockBytes());
        auto rows = src_.ReadBlock(col, s.row, block.data());
        if (!rows.ok()) throw std::runtime_error(std::string(rows.status().message()));
        const uint32_t bw = src_.BlockWidth();
        const uint32_t x0 = col * bw;
        const uint32_t vw = std::min(bw, s.width - x0);
        const size_t pb = s.pixelBytes;
        for (uint32_t y = 0; y < s.rows; ++y)
            std::memcpy(s.Pixel(x0, y), block.data() + static_cast<size_t>(y) * bw * pb, static_cast<size_t>(vw) * pb);
        if (plan_.MaskOut()) {
            mask.resize(static_cast<size_t>(bw) * src_.BlockHeight());
            const Status st = src_.ReadMaskBlock(col, s.row, mask.data());
            if (!st.ok()) throw std::runtime_error(std::string(st.message()));
            for (uint32_t y = 0; y < s.rows; ++y)
                std::memcpy(s.MaskAt(x0, y), mask.data() + static_cast<size_t>(y) * bw, vw);
        }
        decoded_.fetch_add(1, std::memory_order_relaxed);
    }
    uint64_t BlocksDecoded() const override { return decoded_.load(); }

private:
    const SourceRaster& src_;
    const Plan& plan_;
    std::atomic<uint64_t> decoded_{0};
};

// ---------------------------------------------------------------------------------------------
// The pipeline

class Pipeline {
public:
    Pipeline(const Plan& plan, TileProducer& producer, writer::CogWriter& w, const BuildOptions& o)
        : plan_(plan), producer_(producer), writer_(w), options_(o) {
        const uint32_t B = plan.blockSize;
        levels_.push_back(MakeLevel(plan.width, plan.height, B));
        for (const auto& [w2, h2] : plan.overviews)
            levels_.push_back(MakeLevel(w2, h2, B));
        if (options_.statistics) {
            std::vector<StatsAccumulator::MaskKind> masks(static_cast<size_t>(plan.bands), StatsAccumulator::MaskKind::kNone);
            int alpha = -1;
            for (int b = 0; b < plan.bands; ++b) {
                if (plan.MaskOut()) {
                    masks[static_cast<size_t>(b)] = StatsAccumulator::MaskKind::kMask;
                } else if (plan.transparency == Transparency::kAlpha && plan.alphaMasksOverviews && b != plan.alphaBand) {
                    masks[static_cast<size_t>(b)] = StatsAccumulator::MaskKind::kAlphaBand;
                    alpha = plan.alphaBand;
                }
            }
            stats_ = std::make_unique<StatsAccumulator>(plan.bands, plan.type, std::move(masks), alpha, plan.nodata);
        }
        jpegStats_ = options_.statistics && plan.codec == Codec::kJpeg;
        // Fill value of a sparse block: what a reader gets for it.
        fill_.resize(plan.PixelBytes());
        tiff::FillSamples({fill_.data(), fill_.size()}, plan.type, plan.nodata);
    }

    Status Run() {
        try {
            RunImpl();
        } catch (const std::exception& e) {
            return Status(StatusCode::kInternal, e.what());
        }
        return Status::Ok();
    }

    uint64_t PeakStripBytes() const noexcept { return peakBytes_; }

private:
    struct Level {
        uint32_t width = 0, height = 0, blocksDown = 0, blocksAcross = 0;
        // The rows of this level the next level still reads: the latest strip and a tail of
        // the rows before it (a few rows: the taps' reach).
        std::shared_ptr<Strip> cur;
        std::vector<uint8_t> tail, tailMask;
        uint32_t tailY0 = 0, tailRows = 0;
        // The strip of the next level being filled, and its next pixel row to compute.
        std::shared_ptr<Strip> child;
        uint32_t childNextY = 0;
    };

    // A row of level 0 in flight: its compression tasks, and the strip of decoded pixels for
    // the statistics (JPEG: the decoded tiles; otherwise the strip itself).
    struct RowJob {
        uint32_t row = 0;
        std::shared_ptr<Strip> strip;
        std::shared_ptr<Strip> decoded;
        std::unique_ptr<exec::TaskGroup> group;
    };

    // A strip whose memory is counted while it lives (the peak is reported).
    std::shared_ptr<Strip> Track(std::unique_ptr<Strip> s) {
        const int64_t bytes = static_cast<int64_t>(s->Bytes());
        Account(bytes);
        return std::shared_ptr<Strip>(s.release(), [this, bytes](Strip* p) {
            Account(-bytes);
            delete p;
        });
    }

    std::shared_ptr<Strip> NewStrip(uint32_t level, uint32_t row) {
        const Level& L = levels_[level];
        auto s = std::make_unique<Strip>();
        s->level = level;
        s->row = row;
        s->y0 = row * plan_.blockSize;
        s->rows = std::min(plan_.blockSize, L.height - s->y0);
        s->width = L.width;
        s->pixelBytes = plan_.PixelBytes();
        s->pixels = LargeBytes(static_cast<size_t>(s->rows) * s->width * s->pixelBytes);
        if (plan_.StripMask()) s->mask = LargeBytes(static_cast<size_t>(s->rows) * s->width);
        return Track(std::move(s));
    }

    void Account(int64_t delta) {
        const uint64_t now = live_.fetch_add(static_cast<uint64_t>(delta)) + static_cast<uint64_t>(delta);
        uint64_t peak = peakBytes_.load();
        while (now > peak && !peakBytes_.compare_exchange_weak(peak, now)) {
        }
    }

    // The alpha band as the strip's mask (kAlpha with GDAL's alpha mask rule).
    void AlphaToMask(Strip& s, uint32_t x0, uint32_t x1) const {
        const size_t ss = plan_.SampleSize();
        for (uint32_t y = 0; y < s.rows; ++y) {
            for (uint32_t x = x0; x < x1; ++x) {
                const uint8_t* a = s.Pixel(x, y) + static_cast<size_t>(plan_.alphaBand) * ss;
                uint8_t m;
                if (ss == 1) {
                    m = *a;
                } else {
                    uint16_t v;
                    std::memcpy(&v, a, 2);
                    m = static_cast<uint8_t>(v / 257);
                }
                *s.MaskAt(x, y) = m;
            }
        }
    }

    // Copies tile (col) of the strip into a full block (zero padded) and reports whether every
    // valid pixel equals the sparse fill.
    bool CopyTile(const Strip& s, uint32_t col, uint8_t* block, uint8_t* mask, bool& maskAllZero) const {
        const uint32_t B = plan_.blockSize;
        const uint32_t x0 = col * B;
        const uint32_t vw = std::min(B, s.width - x0);
        const size_t pb = s.pixelBytes;
        std::memset(block, 0, static_cast<size_t>(B) * B * pb);
        bool allFill = true;
        for (uint32_t y = 0; y < s.rows; ++y) {
            const uint8_t* src = s.Pixel(x0, y);
            uint8_t* dst = block + static_cast<size_t>(y) * B * pb;
            std::memcpy(dst, src, static_cast<size_t>(vw) * pb);
            if (allFill) {
                for (uint32_t x = 0; x < vw && allFill; ++x)
                    allFill = std::memcmp(src + static_cast<size_t>(x) * pb, fill_.data(), pb) == 0;
            }
        }
        maskAllZero = false;
        if (mask != nullptr) {
            std::memset(mask, 0, static_cast<size_t>(B) * B);
            maskAllZero = true;
            for (uint32_t y = 0; y < s.rows; ++y) {
                const uint8_t* m = s.MaskAt(x0, y);
                std::memcpy(mask + static_cast<size_t>(y) * B, m, vw);
                for (uint32_t x = 0; x < vw && maskAllZero; ++x) maskAllZero = m[x] == 0;
            }
        }
        return allFill;
    }

    // Encodes block `col` of a strip and hands it to the writer; for JPEG statistics, decodes
    // the encoded tile into `decoded`.
    void CompressTile(const Strip& s, uint32_t col, Strip* decoded) {
        const uint32_t B = plan_.blockSize;
        thread_local std::vector<uint8_t> block, mask;
        block.resize(static_cast<size_t>(B) * B * s.pixelBytes);
        const bool maskOut = plan_.MaskOut();
        if (maskOut) mask.resize(static_cast<size_t>(B) * B);
        bool maskAllZero = false;
        const bool allFill = CopyTile(s, col, block.data(), maskOut ? mask.data() : nullptr, maskAllZero);
        const uint32_t x0 = col * B;
        const uint32_t vw = std::min(B, s.width - x0);
        ByteBuffer image, maskBytes;
        const bool sparse = maskOut ? maskAllZero : allFill;
        if (!sparse) {
            encode::BlockFormat f;
            f.width = f.height = B;
            f.samples = static_cast<uint32_t>(plan_.bands);
            f.type = plan_.type;
            f.codec = plan_.codec;
            f.predictor = plan_.predictor;
            f.level = plan_.level;
            f.jpegQuality = plan_.jpegQuality;
            if (plan_.codec == Codec::kJpeg) encode::SpreadJpegEdges(block.data(), B, B, f.samples, vw, s.rows);
            encode::EncodeBlock(f, block.data(), image);
            if (maskOut) encode::EncodeMaskBlock(mask.data(), B, B, maskBytes);
        }
        if (decoded != nullptr) {
            // The pixels a reader decodes from this tile (sparse: zeros, masked out anyway).
            if (sparse) {
                for (uint32_t y = 0; y < s.rows; ++y) std::memset(decoded->Pixel(x0, y), 0, static_cast<size_t>(vw) * s.pixelBytes);
            } else {
                thread_local std::vector<uint8_t> rgb;
                rgb.resize(static_cast<size_t>(B) * B * 3);
                codecs::JpegDecodeOptions jo;
                jo.tables = jpegTables_;
                jo.ycbcrToRgb = true;
                const Status st = codecs::DecodeJpeg({image.data(), image.size()}, jo, 3, 0, 0, B, B, rgb.data(),
                                                     static_cast<size_t>(B) * 3);
                if (!st.ok()) throw std::runtime_error("JPEG decode of a written tile: " + std::string(st.message()));
                for (uint32_t y = 0; y < s.rows; ++y)
                    std::memcpy(decoded->Pixel(x0, y), rgb.data() + static_cast<size_t>(y) * B * 3, static_cast<size_t>(vw) * 3);
            }
        }
        Status st = writer_.Put(s.level, col, s.row, false, std::move(image));
        if (st.ok() && maskOut) st = writer_.Put(s.level, col, s.row, true, std::move(maskBytes));
        if (!st.ok()) throw std::runtime_error(std::string(st.message()));
    }

    void SpawnCompression(exec::TaskGroup& g, const std::shared_ptr<Strip>& s, const std::shared_ptr<Strip>& decoded) {
        const uint32_t across = levels_[s->level].blocksAcross;
        for (uint32_t c = 0; c < across; ++c) {
            g.Spawn([this, s, decoded, c] { CompressTile(*s, c, decoded.get()); });
        }
    }

    static Level MakeLevel(uint32_t w, uint32_t h, uint32_t B) {
        Level l;
        l.width = w;
        l.height = h;
        l.blocksDown = (h + B - 1) / B;
        l.blocksAcross = (w + B - 1) / B;
        return l;
    }

    // Source rows [first, end) the cubic taps of destination rows [d0, d1) read.
    static std::pair<int, int> TapRange(uint32_t srcSize, uint32_t dstSize, int d0, int d1) {
        const pyramid::Taps t = pyramid::CubicTaps(static_cast<int>(srcSize), static_cast<int>(dstSize), d0, d1);
        return {t.SrcBegin(), t.SrcEnd()};
    }

    // Level k got a new strip: computes every row of level k + 1 whose taps are now available,
    // emits the strips of level k + 1 it completes (recursively), keeps the tail rows the next
    // ones read.
    void Cascade(uint32_t k, exec::TaskGroup& g) {
        Level& P = levels_[k];
        Level& C = levels_[k + 1];
        const uint32_t B = plan_.blockSize;
        const Strip& cur = *P.cur;
        const int availEnd = static_cast<int>(cur.y0 + cur.rows);
        // Last child row computable: its taps end within the rows received.
        uint32_t yEnd = P.childNextY;
        while (yEnd < C.height) {
            const auto [b0, e0] = TapRange(P.height, C.height, static_cast<int>(yEnd), static_cast<int>(yEnd) + 1);
            (void)b0;
            if (e0 > availEnd) break;
            ++yEnd;
        }
        while (P.childNextY < yEnd) {
            const uint32_t childRow = P.childNextY / B;
            if (!P.child) {
                P.child = NewStrip(k + 1, childRow);
            }
            Strip& child = *P.child;
            const uint32_t chunkEnd = std::min(yEnd, child.y0 + child.rows);
            const int d0 = static_cast<int>(P.childNextY), d1 = static_cast<int>(chunkEnd);
            const pyramid::Taps rowTaps =
                pyramid::CubicTaps(static_cast<int>(P.height), static_cast<int>(C.height), d0, d1);
            exec::ParallelFor(0, C.blocksAcross, 1,
                              [&](size_t c) { DownsampleTile(P, child, static_cast<uint32_t>(c), rowTaps, d0, d1); });
            P.childNextY = chunkEnd;
            if (chunkEnd == child.y0 + child.rows) {
                if (plan_.transparency == Transparency::kAlpha && plan_.alphaMasksOverviews)
                    AlphaToMask(child, 0, child.width);
                std::shared_ptr<Strip> done = std::move(P.child);
                P.child.reset();
                AddStrip(k + 1, done, g);
            }
        }
        // Keep the rows the next child rows read.
        const size_t pb = plan_.PixelBytes();
        const size_t rowBytes = static_cast<size_t>(P.width) * pb;
        const bool hasMask = plan_.StripMask();
        if (P.childNextY < C.height) {
            const int keepFrom = TapRange(P.height, C.height, static_cast<int>(P.childNextY),
                                          static_cast<int>(P.childNextY) + 1).first;
            const uint32_t from = static_cast<uint32_t>(std::max(keepFrom, 0));
            const uint32_t n = static_cast<uint32_t>(availEnd) > from ? static_cast<uint32_t>(availEnd) - from : 0;
            std::vector<uint8_t> tail(static_cast<size_t>(n) * rowBytes), tailMask;
            if (hasMask) tailMask.resize(static_cast<size_t>(n) * P.width);
            for (uint32_t i = 0; i < n; ++i) {
                const uint32_t y = from + i;
                std::memcpy(tail.data() + static_cast<size_t>(i) * rowBytes, RowOf(P, y), rowBytes);
                if (hasMask) std::memcpy(tailMask.data() + static_cast<size_t>(i) * P.width, MaskRowOf(P, y), P.width);
            }
            Account(static_cast<int64_t>(tail.size() + tailMask.size()) -
                    static_cast<int64_t>(P.tail.size() + P.tailMask.size()));
            P.tail = std::move(tail);
            P.tailMask = std::move(tailMask);
            P.tailY0 = from;
            P.tailRows = n;
        } else {
            Account(-static_cast<int64_t>(P.tail.size() + P.tailMask.size()));
            P.tail.clear();
            P.tailMask.clear();
            P.tailRows = 0;
        }
        P.cur.reset();
    }

    // Row y of level k from its window (tail rows, then the current strip).
    const uint8_t* RowOf(const Level& P, uint32_t y) const {
        if (P.cur && y >= P.cur->y0) return P.cur->Pixel(0, y - P.cur->y0);
        return P.tail.data() + static_cast<size_t>(y - P.tailY0) * P.width * plan_.PixelBytes();
    }
    const uint8_t* MaskRowOf(const Level& P, uint32_t y) const {
        if (P.cur && y >= P.cur->y0) return P.cur->MaskAt(0, y - P.cur->y0);
        return P.tailMask.data() + static_cast<size_t>(y - P.tailY0) * P.width;
    }

    void AddStrip(uint32_t k, const std::shared_ptr<Strip>& s, exec::TaskGroup& g) {
        if (k > 0) SpawnCompression(g, s, nullptr);
        if (k + 1 >= levels_.size()) return;
        Level& L = levels_[k];
        L.cur = s;
        Cascade(k, g);
    }

    // Rows [d0, d1) of tile `col` of level k + 1 from the window of level k.
    void DownsampleTile(const Level& P, Strip& child, uint32_t col, const pyramid::Taps& rowTaps, int d0, int d1) {
        const uint32_t B = plan_.blockSize;
        const int tx0 = static_cast<int>(col * B);
        const int tx1 = static_cast<int>(std::min((col + 1) * B, child.width));
        // The horizontal pass holds (source rows x destination columns x bands) doubles: wide
        // pixels (many bands) are done in column chunks to keep it near 4 MiB per thread.
        const size_t perColumn = static_cast<size_t>(rowTaps.SrcEnd() - rowTaps.SrcBegin()) * 8 *
                                 static_cast<size_t>(plan_.bands);
        const int chunk = static_cast<int>(std::clamp<size_t>((size_t{4} << 20) / std::max<size_t>(perColumn, 1), 16, B));
        for (int cx = tx0; cx < tx1; cx += chunk) DownsampleColumns(P, child, cx, std::min(tx1, cx + chunk), rowTaps, d0, d1);
    }

    void DownsampleColumns(const Level& P, Strip& child, int x0, int x1, const pyramid::Taps& rowTaps, int d0, int d1) {
        const pyramid::Taps colTaps = pyramid::CubicTaps(static_cast<int>(P.width), static_cast<int>(child.width), x0, x1);
        const int cb = colTaps.SrcBegin(), ce = colTaps.SrcEnd();
        const int rb = rowTaps.SrcBegin(), re = rowTaps.SrcEnd();
        const int ww = ce - cb, wh = re - rb;
        const size_t sww = static_cast<size_t>(ww), swh = static_cast<size_t>(wh);
        const size_t pb = plan_.PixelBytes();
        thread_local std::vector<uint8_t> win, wmask, ones;
        win.resize(sww * swh * pb);
        const bool hasMask = plan_.StripMask();
        if (hasMask) wmask.resize(sww * swh);
        for (int y = rb; y < re; ++y) {
            std::memcpy(win.data() + static_cast<size_t>(y - rb) * sww * pb,
                        RowOf(P, static_cast<uint32_t>(y)) + static_cast<size_t>(cb) * pb, sww * pb);
            if (hasMask)
                std::memcpy(wmask.data() + static_cast<size_t>(y - rb) * sww, MaskRowOf(P, static_cast<uint32_t>(y)) + cb,
                            sww);
        }
        const int dw = x1 - x0, dh = d1 - d0;
        uint8_t* dst = child.Pixel(static_cast<uint32_t>(x0), static_cast<uint32_t>(d0) - child.y0);
        uint8_t* dstMask = hasMask ? child.MaskAt(static_cast<uint32_t>(x0), static_cast<uint32_t>(d0) - child.y0) : nullptr;
        const size_t dstStride = child.Stride();
        pyramid::DownsampleSpec spec;
        spec.type = plan_.type;
        spec.bands = plan_.bands;
        spec.hasNoData = plan_.nodata.has_value();
        spec.noData = plan_.nodata.value_or(0);
        if (plan_.transparency == Transparency::kNoData) {
            spec.mask = pyramid::MaskMode::kNoData;
            pyramid::Downsample(spec, win.data(), sww * pb, cb, rb, ww, wh, nullptr, colTaps, rowTaps, dw, dh, dst, dstStride);
        } else if (hasMask && plan_.transparency == Transparency::kAlpha) {
            // Colour bands with the alpha as their mask, the alpha band with an all-valid one (its
            // own mask band in GDAL): one band at a time.
            spec.mask = pyramid::MaskMode::kExternal;
            ones.assign(sww * swh, 255);
            pyramid::DownsampleSpec one = spec;
            one.bands = 1;
            const size_t ss = plan_.SampleSize();
            thread_local std::vector<uint8_t> bandWin, bandDst;
            for (int b = 0; b < plan_.bands; ++b) {
                bandWin.resize(sww * swh * ss);
                for (size_t i = 0; i < sww * swh; ++i)
                    std::memcpy(bandWin.data() + i * ss, win.data() + i * pb + static_cast<size_t>(b) * ss, ss);
                bandDst.resize(static_cast<size_t>(dw) * static_cast<size_t>(dh) * ss);
                pyramid::Downsample(one, bandWin.data(), sww * ss, cb, rb, ww, wh,
                                    b == plan_.alphaBand ? ones.data() : wmask.data(), colTaps, rowTaps, dw, dh,
                                    bandDst.data(), static_cast<size_t>(dw) * ss);
                for (int y = 0; y < dh; ++y)
                    for (int x = 0; x < dw; ++x)
                        std::memcpy(dst + static_cast<size_t>(y) * dstStride + static_cast<size_t>(x) * pb +
                                        static_cast<size_t>(b) * ss,
                                    bandDst.data() + (static_cast<size_t>(y) * static_cast<size_t>(dw) + static_cast<size_t>(x)) * ss,
                                    ss);
            }
        } else if (hasMask) {
            spec.mask = pyramid::MaskMode::kExternal;
            pyramid::Downsample(spec, win.data(), sww * pb, cb, rb, ww, wh, wmask.data(), colTaps, rowTaps, dw, dh, dst,
                                dstStride);
            pyramid::DownsampleMask(wmask.data(), cb, rb, ww, wh, colTaps, rowTaps, dw, dh, dstMask, child.width);
        } else {
            spec.mask = pyramid::MaskMode::kNone;
            pyramid::Downsample(spec, win.data(), sww * pb, cb, rb, ww, wh, nullptr, colTaps, rowTaps, dw, dh, dst, dstStride);
        }
    }

    void RunStats(exec::TaskGroup& g, const std::shared_ptr<Strip>& s) {
        for (int b = 0; b < plan_.bands; ++b) {
            g.Spawn([this, s, b] {
                stats_->AccumulateRows(b, s->pixels.get(), s->Stride(), s->width, s->rows, s->mask.get());
            });
        }
    }

    void Report(double f) {
        if (!options_.progress) return;
        // DroneDB's phase names over the fraction of the single pass (cogPhaseOfFraction with a
        // warp; without one there is no "warping" phase).
        const double x = f * 0.5;
        const char* phase;
        if (!plan_.ownGrid && x < 0.5 * 3.0 / 8.0) phase = "warping";
        else if (x < (plan_.ownGrid ? 0.1 : 0.25)) phase = "overviews";
        else if (x < 0.5) phase = "compressing";
        else phase = "statistics";
        options_.progress(f * 0.95, phase);
    }

    void RunImpl() {
        if (jpegStats_) jpegTables_ = encode::JpegTables(plan_.jpegQuality);
        Level& L0 = levels_[0];
        std::deque<RowJob> jobs;
        std::unique_ptr<exec::TaskGroup> statsGroup;
        std::shared_ptr<Strip> statsStrip;  // keeps the strip alive while its statistics run
        auto finishJob = [&](RowJob& j) {
            j.group->Wait();
            if (stats_) {
                if (statsGroup) statsGroup->Wait();
                statsGroup = std::make_unique<exec::TaskGroup>();
                statsStrip = j.decoded ? j.decoded : j.strip;
                RunStats(*statsGroup, statsStrip);
            }
        };
        for (uint32_t r = 0; r < L0.blocksDown; ++r) {
            producer_.BeginRow(r);
            auto strip = NewStrip(0, r);
            exec::ParallelFor(0, L0.blocksAcross, 1, [&](size_t c) { producer_.Produce(static_cast<uint32_t>(c), *strip); });
            producer_.EndRow(r);
            if (plan_.transparency == Transparency::kAlpha && plan_.alphaMasksOverviews) AlphaToMask(*strip, 0, strip->width);
            RowJob job;
            job.row = r;
            job.strip = strip;
            job.group = std::make_unique<exec::TaskGroup>();
            if (jpegStats_) {
                auto d = std::make_unique<Strip>();
                d->level = 0;
                d->row = r;
                d->y0 = strip->y0;
                d->rows = strip->rows;
                d->width = strip->width;
                d->pixelBytes = strip->pixelBytes;
                d->pixels = LargeBytes(static_cast<size_t>(strip->rows) * strip->width * strip->pixelBytes);
                // The mask the statistics see (the pyramid keeps reading the strip's own).
                if (strip->mask) {
                    d->mask = LargeBytes(static_cast<size_t>(strip->rows) * strip->width);
                    std::memcpy(d->mask.get(), strip->mask.get(), static_cast<size_t>(strip->rows) * strip->width);
                }
                job.decoded = Track(std::move(d));
            }
            SpawnCompression(*job.group, strip, job.decoded);
            // Overview rows that this row completes (their compression joins this row's group).
            AddStrip(0, strip, *job.group);
            jobs.push_back(std::move(job));
            // Two rows in flight (production overlaps compression) unless that would not fit
            // the memory budget.
            const size_t inFlight = 3 * strip->Bytes() > options_.memoryBudget ? 0 : 1;
            while (jobs.size() > inFlight) {
                finishJob(jobs.front());
                jobs.pop_front();
            }
            if (inFlight == 0 && statsGroup) {
                // Low on memory: the statistics release the strip before the next one exists.
                statsGroup->Wait();
                statsStrip.reset();
            }
            Report(static_cast<double>(r + 1) / L0.blocksDown);
        }
        while (!jobs.empty()) {
            finishJob(jobs.front());
            jobs.pop_front();
        }
        if (statsGroup) statsGroup->Wait();
        for (Level& L : levels_) {
            L.cur.reset();
            L.child.reset();
        }
    }

    const Plan& plan_;
    TileProducer& producer_;
    writer::CogWriter& writer_;
    const BuildOptions& options_;
    std::vector<Level> levels_;
    std::unique_ptr<StatsAccumulator> stats_;
    bool jpegStats_ = false;
    std::vector<uint8_t> jpegTables_;
    std::vector<uint8_t> fill_;
    std::atomic<uint64_t> live_{0}, peakBytes_{0};

public:
    StatsAccumulator* Stats() { return stats_.get(); }
};

}  // namespace

Status BuildCog(const std::string& input, const std::string& output, const BuildOptions& options, BuildReport* report) {
    const auto t0 = std::chrono::steady_clock::now();
    auto src = SourceRaster::Open(input);
    if (!src.ok()) return src.status();
    const SourceRaster& source = **src;
    if (!source.Georef().crs) return Status(StatusCode::kInvalidArgument, "Cannot build COG: input file has no coordinate reference system: " + input);
    if (!source.Georef().geoTransform)
        return Status(StatusCode::kInvalidArgument, "Cannot build COG: input file has no geotransform: " + input);

    Plan plan;
    std::unique_ptr<TileProducer> producer;
    const bool ownGrid = source.IsOptimizedCog();
    if (ownGrid) {
        auto p = OwnGridPlan(source, options);
        if (!p.ok()) return p.status();
        plan = std::move(*p);
        producer = std::make_unique<OwnGridProducer>(source, plan);
    } else {
        warp::WarpSettings ws;
        ws.losslessCodec = options.losslessCodec == Codec::kJpeg ? Codec::kLzw : options.losslessCodec;
        ws.level = options.level;
        ws.jpegQuality = options.jpegQuality > 0 ? options.jpegQuality : ConfiguredJpegQuality();
        ws.noJpeg = options.noJpeg;
        ws.evenTileOrigin = options.evenTileOrigin;
        auto p = warp::WarpPlan(source, ws);
        if (!p.ok()) return p.status();
        plan = std::move(*p);
        plan.gdalMetadata = OutputMetadata(source.Main().gdalMetadata, true);
        auto prod = warp::MakeWarpProducer(source, plan, options.memoryBudget);
        if (!prod.ok()) return prod.status();
        producer = std::move(*prod);
    }

    writer::ImageSpec spec;
    spec.width = plan.width;
    spec.height = plan.height;
    spec.blockSize = plan.blockSize;
    spec.samples = static_cast<uint16_t>(plan.bands);
    spec.type = plan.type;
    spec.photometric = plan.photometric;
    spec.extraSamples = plan.extraSamples;
    spec.codec = plan.codec;
    spec.predictor = plan.predictor;
    if (plan.codec == Codec::kJpeg) spec.jpegTables = encode::JpegTables(plan.jpegQuality);
    spec.hasMask = plan.MaskOut();
    spec.noDataText = plan.nodataText;
    spec.gdalMetadata = plan.gdalMetadata;
    spec.extraTags = source.MetadataTags();
    spec.geoTransform = plan.geoTransform;
    spec.overviews = plan.overviews;
    auto w = writer::CogWriter::Create(output, spec, options.writer);
    if (!w.ok()) return w.status();
    writer::CogWriter& cw = **w;

    Pipeline pipe(plan, *producer, cw, options);
    Status s = pipe.Run();
    if (!s.ok()) {
        cw.Abort();
        return s;
    }
    s = cw.Finish();
    if (!s.ok()) return s;
    if (options.statistics) {
        const auto stats = pipe.Stats()->Result(static_cast<uint64_t>(plan.width) * plan.height);
        s = WriteStatsSidecar(output, stats);
        if (!s.ok()) return s;
    }
    if (options.progress) options.progress(1.0, "statistics");
    if (report != nullptr) {
        report->ownGrid = plan.ownGrid;
        report->width = plan.width;
        report->height = plan.height;
        report->levels = static_cast<uint32_t>(plan.overviews.size() + 1);
        report->bands = plan.bands;
        report->codec = encode::CodecName(plan.codec);
        report->jpegQuality = plan.codec == Codec::kJpeg ? plan.jpegQuality : 0;
        report->transparency = TransparencyName(plan.transparency);
        report->file = cw.Stats();
        report->peakStripBytes = pipe.PeakStripBytes();
        report->sourceBlocksDecoded = producer->BlocksDecoded();
        report->peakSourceCacheBytes = producer->PeakCacheBytes();
        report->secondsTotal = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    }
    return Status::Ok();
}

}  // namespace ddbt::cog::build
