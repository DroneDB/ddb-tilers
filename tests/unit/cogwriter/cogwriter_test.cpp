// SPDX-License-Identifier: MPL-2.0
//
// COG builder, write side: the LZW encoder against our decoder (sizes, patterns, table resets,
// code width changes), forward predictors against the decoders' inverse, block encoding round
// trips for every codec, 1-bit masks, JPEG tables and quality estimation, the CogWriter's layout
// (ghost header, IFD order and kinds, block order, leaders / trailers, sparse blocks, out-of-
// order puts, BigTIFF, insert-range and copy fallbacks) read back by our TiffReader.

#include <gtest/gtest.h>

// Index arithmetic of the synthetic images mixes int and size_t freely.
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wdouble-promotion"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <tuple>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "cog/codecs/codecs.h"
#include "cog/codecs/encoders.h"
#include "cog/codecs/jpeg.h"
#include "cog/codecs/predictor.h"
#include "cog/encode/block_encoder.h"
#include "cog/tiff/geotiff.h"
#include "cog/tiff/tiff_reader.h"
#include "cog/writer/cog_writer.h"
#include "core/util/endian.h"

namespace ddbt::cog {
namespace {

namespace fs = std::filesystem;
using encode::ByteBuffer;

class TempDir {
public:
    TempDir() {
        const char* base = std::getenv("DDBT_TEST_TMP");
        fs::path root = base ? fs::path(base) : fs::temp_directory_path();
        path_ = root / ("ddbt-cogwriter-" + std::to_string(std::random_device{}()) + "-" + std::to_string(counter_++));
        fs::create_directories(path_);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path_, ec);
    }
    std::string File(const std::string& name) const { return (path_ / name).string(); }
    size_t Entries() const { return static_cast<size_t>(std::distance(fs::directory_iterator(path_), fs::directory_iterator())); }

private:
    static inline std::atomic<int> counter_{0};
    fs::path path_;
};

std::vector<uint8_t> LzwRoundTrip(const std::vector<uint8_t>& in) {
    ByteBuffer enc;
    codecs::EncodeLzw({in.data(), in.size()}, enc);
    EXPECT_LE(enc.size(), codecs::LzwEncodedBound(in.size()));
    std::vector<uint8_t> out(in.size());
    const Status s = codecs::DecodeLzw({enc.data(), enc.size()}, {out.data(), out.size()});
    EXPECT_TRUE(s.ok()) << s.message();
    return out;
}

TEST(LzwEncoder, RoundTripsAcrossSizesAndPatterns) {
    std::mt19937 rng(7);
    for (size_t n : {size_t{0}, size_t{1}, size_t{2}, size_t{3}, size_t{255}, size_t{256}, size_t{511}, size_t{4096},
                     size_t{65536}, size_t{300000}}) {
        std::vector<uint8_t> noise(n), runs(n), ramp(n), few(n);
        for (size_t i = 0; i < n; ++i) {
            noise[i] = static_cast<uint8_t>(rng());
            runs[i] = static_cast<uint8_t>((i / 37) & 0xFF);
            ramp[i] = static_cast<uint8_t>(i);
            few[i] = static_cast<uint8_t>(rng() % 3);
        }
        EXPECT_EQ(LzwRoundTrip(noise), noise) << n;
        EXPECT_EQ(LzwRoundTrip(runs), runs) << n;
        EXPECT_EQ(LzwRoundTrip(ramp), ramp) << n;
        EXPECT_EQ(LzwRoundTrip(few), few) << n;
    }
}

TEST(LzwEncoder, KwKwKAndLongRunsAcrossManyTableResets) {
    // One byte repeated: every new code is the KwKwK case; 2 MB forces many table resets.
    std::vector<uint8_t> same(size_t{2} << 20, 0x41);
    EXPECT_EQ(LzwRoundTrip(same), same);
    // Concatenated pseudo-random blocks of growing alphabets: exercises every code width.
    std::mt19937 rng(11);
    std::vector<uint8_t> mixed;
    for (int a = 1; a <= 256; a *= 2)
        for (int i = 0; i < 20000; ++i) mixed.push_back(static_cast<uint8_t>(rng() % static_cast<unsigned>(a)));
    EXPECT_EQ(LzwRoundTrip(mixed), mixed);
}

TEST(LzwEncoder, ConcurrentEncodesMatchSerial) {
    std::vector<std::vector<uint8_t>> inputs(16);
    std::mt19937 rng(3);
    for (auto& v : inputs) {
        v.resize(100000);
        for (auto& b : v) b = static_cast<uint8_t>(rng() % 17);
    }
    std::vector<ByteBuffer> serial(inputs.size()), parallel(inputs.size());
    for (size_t i = 0; i < inputs.size(); ++i) codecs::EncodeLzw({inputs[i].data(), inputs[i].size()}, serial[i]);
    std::vector<std::thread> threads;
    for (size_t i = 0; i < inputs.size(); ++i)
        threads.emplace_back([&, i] { codecs::EncodeLzw({inputs[i].data(), inputs[i].size()}, parallel[i]); });
    for (auto& t : threads) t.join();
    for (size_t i = 0; i < inputs.size(); ++i) {
        ASSERT_EQ(serial[i].size(), parallel[i].size());
        EXPECT_EQ(0, std::memcmp(serial[i].data(), parallel[i].data(), serial[i].size()));
    }
}

TEST(Predictors, ForwardThenUndoIsIdentity) {
    std::mt19937 rng(5);
    for (size_t bytes : {size_t{1}, size_t{2}, size_t{4}, size_t{8}}) {
        for (uint32_t spp : {1u, 3u, 4u, 5u}) {
            const uint32_t w = 37, h = 5;
            std::vector<uint8_t> data(static_cast<size_t>(w) * h * spp * bytes);
            for (auto& b : data) b = static_cast<uint8_t>(rng());
            std::vector<uint8_t> work = data;
            encode::ApplyHorizontalPredictor(work.data(), w, h, spp, bytes);
            codecs::UndoHorizontalPredictor(work.data(), h, static_cast<size_t>(w) * spp * bytes, w, spp, bytes);
            EXPECT_EQ(work, data) << bytes << " " << spp;
            if (bytes >= 2) {
                std::vector<uint8_t> fp(data.size());
                encode::ApplyFloatingPointPredictor(data.data(), fp.data(), w, h, spp, bytes);
                codecs::UndoFloatingPointPredictor(fp.data(), h, static_cast<size_t>(w) * spp * bytes, w, spp, bytes);
                EXPECT_EQ(fp, data) << "fp " << bytes << " " << spp;
            }
        }
    }
}

std::vector<uint8_t> Decode(encode::Codec c, const ByteBuffer& in, size_t size) {
    std::vector<uint8_t> out(size);
    Status s;
    switch (c) {
        case encode::Codec::kLzw: s = codecs::DecodeLzw({in.data(), in.size()}, {out.data(), out.size()}); break;
        case encode::Codec::kDeflate: s = codecs::DecodeDeflate({in.data(), in.size()}, {out.data(), out.size()}); break;
        case encode::Codec::kZstd: s = codecs::DecodeZstd({in.data(), in.size()}, {out.data(), out.size()}); break;
        default: s = codecs::DecodeNone({in.data(), in.size()}, {out.data(), out.size()});
    }
    EXPECT_TRUE(s.ok()) << s.message();
    return out;
}

TEST(BlockEncoder, LosslessCodecsAndPredictorsRoundTrip) {
    std::mt19937 rng(9);
    const uint32_t B = 64;
    for (const auto codec : {encode::Codec::kNone, encode::Codec::kLzw, encode::Codec::kDeflate, encode::Codec::kZstd}) {
        for (const auto type : {tiff::SampleType::kU8, tiff::SampleType::kU16, tiff::SampleType::kF32}) {
            const uint32_t spp = 3;
            const size_t ss = tiff::SampleTypeSize(type);
            std::vector<uint8_t> px(static_cast<size_t>(B) * B * spp * ss);
            for (size_t i = 0; i < px.size(); ++i) px[i] = static_cast<uint8_t>((i % 97) + (rng() % 4));
            const int predictor = type == tiff::SampleType::kF32 ? 3 : 2;
            encode::BlockFormat f{B, B, spp, type, codec, predictor, 0, 75};
            ByteBuffer out;
            encode::EncodeBlock(f, px.data(), out);
            std::vector<uint8_t> dec = Decode(codec, out, px.size());
            if (predictor == 2)
                codecs::UndoHorizontalPredictor(dec.data(), B, static_cast<size_t>(B) * spp * ss, B, spp, ss);
            else
                codecs::UndoFloatingPointPredictor(dec.data(), B, static_cast<size_t>(B) * spp * ss, B, spp, ss);
            EXPECT_EQ(dec, px) << encode::CodecName(codec) << " " << tiff::SampleTypeName(type);
        }
    }
}

TEST(BlockEncoder, JpegTablesQualityAndAbbreviatedTiles) {
    for (int q : {1, 10, 50, 75, 90, 100}) {
        const std::vector<uint8_t> tables = encode::JpegTables(q);
        // SOI, two DQT of 67 bytes each, EOI: as libtiff writes JPEGTables with JPEGTABLESMODE=1
        // (below quality 25 some values exceed 255: 16-bit tables, as libtiff's force_baseline
        // FALSE gives).
        ASSERT_EQ(tables.size(), q >= 25 ? 142u : 270u) << q;
        EXPECT_EQ(tables[0], 0xFF);
        EXPECT_EQ(tables[1], 0xD8);
        EXPECT_EQ(tables[2], 0xFF);
        EXPECT_EQ(tables[3], 0xDB);
        EXPECT_EQ(encode::GuessJpegQuality(tables, 3), q);
    }
    const uint32_t B = 256;
    std::vector<uint8_t> rgb(static_cast<size_t>(B) * B * 3);
    for (uint32_t y = 0; y < B; ++y)
        for (uint32_t x = 0; x < B; ++x) {
            uint8_t* p = rgb.data() + (static_cast<size_t>(y) * B + x) * 3;
            p[0] = static_cast<uint8_t>(x);
            p[1] = static_cast<uint8_t>(y);
            p[2] = static_cast<uint8_t>((x + y) / 2);
        }
    encode::BlockFormat f{B, B, 3, tiff::SampleType::kU8, encode::Codec::kJpeg, 1, 0, 75};
    ByteBuffer tile;
    encode::EncodeBlock(f, rgb.data(), tile);
    // No DQT in the tile (the tables are in JPEGTables), no JFIF APP0.
    bool dqt = false, app0 = false;
    for (size_t i = 2; i + 4 <= tile.size();) {
        ASSERT_EQ(tile.data()[i], 0xFF);
        const uint8_t m = tile.data()[i + 1];
        if (m == 0xDA) break;
        dqt |= m == 0xDB;
        app0 |= m == 0xE0;
        i += 2 + ((static_cast<size_t>(tile.data()[i + 2]) << 8) | tile.data()[i + 3]);
    }
    EXPECT_FALSE(dqt);
    EXPECT_FALSE(app0);
    const std::vector<uint8_t> tables = encode::JpegTables(75);
    codecs::JpegDecodeOptions o;
    o.tables = tables;
    o.ycbcrToRgb = true;
    std::vector<uint8_t> dec(rgb.size());
    const Status s = codecs::DecodeJpeg({tile.data(), tile.size()}, o, 3, 0, 0, B, B, dec.data(), static_cast<size_t>(B) * 3);
    ASSERT_TRUE(s.ok()) << s.message();
    double se = 0;
    for (size_t i = 0; i < rgb.size(); ++i) se += (double(rgb[i]) - dec[i]) * (double(rgb[i]) - dec[i]);
    const double psnr = 10 * std::log10(255.0 * 255.0 / (se / static_cast<double>(rgb.size())));
    EXPECT_GT(psnr, 35.0);
    // Without the tables the tile does not decode.
    codecs::JpegDecodeOptions bare;
    bare.ycbcrToRgb = true;
    EXPECT_FALSE(codecs::DecodeJpeg({tile.data(), tile.size()}, bare, 3, 0, 0, B, B, dec.data(), static_cast<size_t>(B) * 3).ok());
}

TEST(BlockEncoder, MaskBlocksArePackedMsbFirstAndDeflated) {
    const uint32_t W = 37, H = 5;
    std::vector<uint8_t> m(static_cast<size_t>(W) * H);
    for (size_t i = 0; i < m.size(); ++i) m[i] = (i % 3 == 0) ? 0 : static_cast<uint8_t>(1 + i % 200);
    ByteBuffer out;
    encode::EncodeMaskBlock(m.data(), W, H, out);
    const size_t rowBytes = (W + 7) / 8;
    std::vector<uint8_t> bits(rowBytes * H);
    ASSERT_TRUE(codecs::DecodeDeflate({out.data(), out.size()}, {bits.data(), bits.size()}).ok());
    std::vector<uint8_t> back(m.size());
    tiff::ExpandBits(bits.data(), rowBytes, W, H, 1, back.data(), W);
    for (size_t i = 0; i < m.size(); ++i) EXPECT_EQ(back[i], m[i] != 0 ? 1 : 0) << i;
}

TEST(BlockEncoder, JpegEdgeSpreadReplicatesTheLastColumnAndRow) {
    const uint32_t B = 8;
    std::vector<uint8_t> px(B * B * 3, 0);
    for (uint32_t y = 0; y < 5; ++y)
        for (uint32_t x = 0; x < 6; ++x)
            for (uint32_t c = 0; c < 3; ++c) px[(y * B + x) * 3 + c] = static_cast<uint8_t>(10 * y + x + c);
    encode::SpreadJpegEdges(px.data(), B, B, 3, 6, 5);
    for (uint32_t y = 0; y < B; ++y)
        for (uint32_t x = 0; x < B; ++x) {
            const uint32_t sy = std::min(y, 4u), sx = std::min(x, 5u);
            EXPECT_EQ(px[(y * B + x) * 3], static_cast<uint8_t>(10 * sy + sx)) << x << "," << y;
        }
}

// A small COG: 300 x 200 RGB (16-pixel blocks so there are several levels), LZW + predictor,
// with a mask, written through the CogWriter with blocks put in a scrambled order.
struct Synthetic {
    uint32_t W = 300, H = 200, B = 64;
    std::vector<std::pair<uint32_t, uint32_t>> ovr{{150, 100}, {75, 50}};
    std::vector<std::vector<uint8_t>> pixels;  // per level, RGB
    std::vector<std::vector<uint8_t>> masks;   // per level, bytes 0 / 255
    Synthetic() {
        std::vector<std::pair<uint32_t, uint32_t>> lv{{W, H}};
        lv.insert(lv.end(), ovr.begin(), ovr.end());
        for (size_t l = 0; l < lv.size(); ++l) {
            const auto [w, h] = lv[l];
            std::vector<uint8_t> p(static_cast<size_t>(w) * h * 3), m(static_cast<size_t>(w) * h);
            for (uint32_t y = 0; y < h; ++y)
                for (uint32_t x = 0; x < w; ++x) {
                    const size_t i = static_cast<size_t>(y) * w + x;
                    p[3 * i] = static_cast<uint8_t>(x + l);
                    p[3 * i + 1] = static_cast<uint8_t>(y * 3);
                    p[3 * i + 2] = static_cast<uint8_t>(x ^ y);
                    m[i] = (x + y) % 7 == 0 ? 0 : 255;
                    // The top-left block of level 0 is fully transparent: written sparse.
                    if (l == 0 && x < B && y < B) m[i] = 0;
                }
            pixels.push_back(std::move(p));
            masks.push_back(std::move(m));
        }
    }
    writer::ImageSpec Spec() const {
        writer::ImageSpec s;
        s.width = W;
        s.height = H;
        s.blockSize = B;
        s.samples = 3;
        s.type = tiff::SampleType::kU8;
        s.photometric = 2;
        s.codec = encode::Codec::kLzw;
        s.predictor = 2;
        s.hasMask = true;
        s.gdalMetadata = "<GDALMetadata>\n</GDALMetadata>\n";
        s.geoTransform = {-1000.0, 0.5, 0.0, 2000.0, 0.0, -0.5};
        s.overviews = ovr;
        return s;
    }
    void Put(writer::CogWriter& w, uint32_t level, uint32_t col, uint32_t row) const {
        const uint32_t lw = level == 0 ? W : ovr[level - 1].first;
        const uint32_t lh = level == 0 ? H : ovr[level - 1].second;
        std::vector<uint8_t> block(static_cast<size_t>(B) * B * 3, 0), mask(static_cast<size_t>(B) * B, 0);
        bool any = false;
        for (uint32_t y = 0; y < B && row * B + y < lh; ++y)
            for (uint32_t x = 0; x < B && col * B + x < lw; ++x) {
                const size_t i = static_cast<size_t>(row * B + y) * lw + col * B + x;
                std::memcpy(&block[(static_cast<size_t>(y) * B + x) * 3], &pixels[level][3 * i], 3);
                mask[static_cast<size_t>(y) * B + x] = masks[level][i];
                any |= masks[level][i] != 0;
            }
        ByteBuffer img, msk;
        if (any) {
            encode::BlockFormat f{B, B, 3, tiff::SampleType::kU8, encode::Codec::kLzw, 2, 0, 75};
            encode::EncodeBlock(f, block.data(), img);
            encode::EncodeMaskBlock(mask.data(), B, B, msk);
        }
        ASSERT_TRUE(w.Put(level, col, row, false, std::move(img)).ok());
        ASSERT_TRUE(w.Put(level, col, row, true, std::move(msk)).ok());
    }
};

void CheckReadBack(const std::string& path, const Synthetic& syn, bool bigTiff) {
    auto r = tiff::TiffReader::Open(path);
    ASSERT_TRUE(r.ok()) << r.status().message();
    const tiff::TiffReader& rd = **r;
    EXPECT_EQ(rd.IsBigTiff(), bigTiff);
    const tiff::GhostHeader& g = rd.Ghost();
    EXPECT_TRUE(g.present && g.layoutIfdsBeforeData && g.blockOrderRowMajor && g.blockLeaderSizeAsUint4 &&
                g.blockTrailerLast4BytesRepeated && !g.knownIncompatibleEdition);
    // GDAL writes " MASK_INTERLEAVED_WITH_IMAGERY=YES" after the space that follows
    // KNOWN_INCOMPATIBLE_EDITION=NO (so it can become YES in place); TiffReader's line parser
    // does not trim that space, so the flag is checked on the text.
    EXPECT_NE(g.text.find("MASK_INTERLEAVED_WITH_IMAGERY=YES"), std::string::npos);
    // IFD order: full resolution, its mask, overviews, overview masks.
    ASSERT_EQ(rd.IfdCount(), 6u);
    EXPECT_EQ(rd.Ifd(0).kind, tiff::SubfileKind::kFullResolution);
    EXPECT_EQ(rd.Ifd(1).kind, tiff::SubfileKind::kMask);
    EXPECT_EQ(rd.Ifd(2).kind, tiff::SubfileKind::kOverview);
    EXPECT_EQ(rd.Ifd(3).kind, tiff::SubfileKind::kOverview);
    EXPECT_EQ(rd.Ifd(4).kind, tiff::SubfileKind::kOverviewMask);
    EXPECT_EQ(rd.Ifd(5).kind, tiff::SubfileKind::kOverviewMask);
    uint64_t lastIfd = 0;
    for (const auto& ifd : rd.Ifds()) {
        EXPECT_GT(ifd.offset, lastIfd);
        lastIfd = ifd.offset;
    }
    // Every block: decoded content, leader and trailer (checked by the reader), data after the
    // IFDs, the smallest overview first, image block followed by its mask block.
    const std::vector<uint32_t> imageIfds{0, 2, 3}, maskIfds{1, 4, 5};
    uint64_t prevLevelFirst = UINT64_MAX;
    for (size_t l = 0; l < 3; ++l) {
        const tiff::IfdInfo& im = rd.Ifd(imageIfds[l]);
        const tiff::IfdInfo& mk = rd.Ifd(maskIfds[l]);
        const uint32_t lw = im.width, lh = im.height;
        uint64_t last = 0, first = UINT64_MAX;
        for (uint32_t row = 0; row < im.blocksDown; ++row) {
            for (uint32_t col = 0; col < im.blocksAcross; ++col) {
                const uint32_t idx = im.BlockIndex(col, row);
                auto loc = rd.Locate(im.index, idx);
                auto mloc = rd.Locate(mk.index, idx);
                ASSERT_TRUE(loc.ok() && mloc.ok());
                if (l == 0 && row == 0 && col == 0) {
                    EXPECT_TRUE(loc->IsSparse());
                    EXPECT_TRUE(mloc->IsSparse());
                    continue;
                }
                EXPECT_GT(loc->offset, last);
                EXPECT_EQ(mloc->offset, loc->offset + loc->size + 8);
                last = mloc->offset;
                first = std::min(first, loc->offset);
                EXPECT_GT(loc->offset, rd.Ifds().back().offset);
                auto b = rd.ReadBlock(im.index, idx);
                ASSERT_TRUE(b.ok()) << b.status().message();
                auto mb = rd.ReadBlock(mk.index, idx);
                ASSERT_TRUE(mb.ok()) << mb.status().message();
                std::vector<uint8_t> mbytes(static_cast<size_t>(syn.B) * syn.B);
                tiff::ExpandBits(mb->data(), mb->rowStride, syn.B, syn.B, 255, mbytes.data(), syn.B);
                for (uint32_t y = 0; y < syn.B && row * syn.B + y < lh; ++y)
                    for (uint32_t x = 0; x < syn.B && col * syn.B + x < lw; ++x) {
                        const size_t i = static_cast<size_t>(row * syn.B + y) * lw + col * syn.B + x;
                        ASSERT_EQ(0, std::memcmp(b->data() + (static_cast<size_t>(y) * syn.B + x) * 3, &syn.pixels[l][3 * i], 3));
                        ASSERT_EQ(mbytes[static_cast<size_t>(y) * syn.B + x], syn.masks[l][i]);
                    }
            }
        }
        if (l > 0) {
            EXPECT_LT(first, prevLevelFirst);
        }
        prevLevelFirst = first;
    }
    auto geo = tiff::ReadGeoreferencing(rd, path);
    ASSERT_TRUE(geo.ok());
    ASSERT_TRUE(geo->geoTransform.has_value());
    EXPECT_EQ((*geo->geoTransform)[0], -1000.0);
    EXPECT_EQ((*geo->geoTransform)[1], 0.5);
    EXPECT_EQ((*geo->geoTransform)[5], -0.5);
    EXPECT_EQ(geo->crsDefinition, "EPSG:3857");
}

void WriteSynthetic(const std::string& path, const Synthetic& syn, const writer::WriterOptions& o, bool reverse) {
    auto w = writer::CogWriter::Create(path, syn.Spec(), o);
    ASSERT_TRUE(w.ok()) << w.status().message();
    std::vector<std::tuple<uint32_t, uint32_t, uint32_t>> blocks;
    for (uint32_t l = 0; l < (*w)->Levels(); ++l)
        for (uint32_t r = 0; r < (*w)->BlocksDown(l); ++r)
            for (uint32_t c = 0; c < (*w)->BlocksAcross(l); ++c) blocks.emplace_back(l, c, r);
    if (reverse) std::reverse(blocks.begin(), blocks.end());
    for (const auto& [l, c, r] : blocks) syn.Put(**w, l, c, r);
    const Status s = (*w)->Finish();
    ASSERT_TRUE(s.ok()) << s.message();
}

TEST(CogWriter, LayoutReadsBackWithInsertRange) {
    TempDir dir;
    Synthetic syn;
    const std::string path = dir.File("a.tif");
    WriteSynthetic(path, syn, {}, false);
    CheckReadBack(path, syn, false);
    EXPECT_EQ(dir.Entries(), 1u);  // no temporary left
}

TEST(CogWriter, CopyFallbackOutOfOrderPutsAndOverviewSpill) {
    TempDir dir;
    Synthetic syn;
    writer::WriterOptions o;
    o.insertRange = false;
    o.overviewMemory = 100;  // spill nearly every overview block
    const std::string path = dir.File("b.tif");
    WriteSynthetic(path, syn, o, true);
    CheckReadBack(path, syn, false);
    EXPECT_EQ(dir.Entries(), 1u);
}

TEST(CogWriter, ForcedBigTiff) {
    TempDir dir;
    Synthetic syn;
    writer::WriterOptions o;
    o.forceBigTiff = true;
    const std::string path = dir.File("c.tif");
    WriteSynthetic(path, syn, o, false);
    CheckReadBack(path, syn, true);
}

TEST(CogWriter, GhostHeaderTextIsGdals) {
    EXPECT_EQ(writer::GhostHeader(false),
              "GDAL_STRUCTURAL_METADATA_SIZE=000140 bytes\nLAYOUT=IFDS_BEFORE_DATA\nBLOCK_ORDER=ROW_MAJOR\n"
              "BLOCK_LEADER=SIZE_AS_UINT4\nBLOCK_TRAILER=LAST_4_BYTES_REPEATED\nKNOWN_INCOMPATIBLE_EDITION=NO\n ");
    EXPECT_EQ(writer::GhostHeader(true).substr(0, 42), "GDAL_STRUCTURAL_METADATA_SIZE=000174 bytes");
}

TEST(CogWriter, MissingBlockFailsAndLeavesNothing) {
    TempDir dir;
    Synthetic syn;
    const std::string path = dir.File("d.tif");
    {
        auto w = writer::CogWriter::Create(path, syn.Spec(), {});
        ASSERT_TRUE(w.ok());
        syn.Put(**w, 0, 0, 0);
        EXPECT_FALSE((*w)->Finish().ok());
        // Putting a block twice is refused too.
        auto w2 = writer::CogWriter::Create(dir.File("e.tif"), syn.Spec(), {});
        ASSERT_TRUE(w2.ok());
        syn.Put(**w2, 1, 0, 0);
        EXPECT_FALSE((*w2)->Put(1, 0, 0, false, ByteBuffer()).ok());
    }
    EXPECT_FALSE(fs::exists(path));
    EXPECT_EQ(dir.Entries(), 0u);
}

}  // namespace
}  // namespace ddbt::cog
