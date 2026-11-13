// SPDX-License-Identifier: MPL-2.0
//
// TiffReader on synthetic files (testing::TiffBuilder): both byte orders, classic and BigTIFF,
// every field type, SubIFDs and subfile kinds, strips, sparse blocks and nodata fills, GDAL's
// ghost header leader / trailer, the lazy block index over a non-mapped source (with racing
// first accesses), concurrent decodes against serial ones, malformed directories and a
// mutation fuzz that must only ever produce error statuses (run under ASan / UBSan).

#include <gtest/gtest.h>

#include <libdeflate.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "cog/tiff/geotiff.h"
#include "cog/tiff/tiff_reader.h"
#include "core/util/checked_math.h"
#include "tiff_test_util.h"

namespace ddbt::cog::tiff {
namespace {

using testing::TiffBuilder;
using Entries = std::vector<TiffBuilder::Entry>;

std::shared_ptr<const io::ByteSource> Memory(std::vector<uint8_t> bytes) {
    return std::make_shared<io::MemorySource>(std::move(bytes), "<test>");
}

// A source without Data(): the block index goes through the lazy (call_once) path.
std::shared_ptr<const io::ByteSource> Pread(std::vector<uint8_t> bytes, std::atomic<int>* reads = nullptr) {
    auto data = std::make_shared<std::vector<uint8_t>>(std::move(bytes));
    const uint64_t size = data->size();
    return std::make_shared<io::RangeCallbackSource>(
        size,
        [data, reads](uint64_t off, std::span<uint8_t> out) -> Status {
            if (!RangeInside(off, out.size(), data->size())) return OutOfRange("read past the end");
            std::memcpy(out.data(), data->data() + off, out.size());
            if (reads) reads->fetch_add(1);
            return Status::Ok();
        },
        "<pread>");
}

std::vector<uint8_t> Deflate(const std::vector<uint8_t>& in) {
    libdeflate_compressor* c = libdeflate_alloc_compressor(6);
    std::vector<uint8_t> out(libdeflate_zlib_compress_bound(c, in.size()));
    out.resize(libdeflate_zlib_compress(c, in.data(), in.size(), out.data(), out.size()));
    libdeflate_free_compressor(c);
    return out;
}

// width x height uint16 image, tiled tw x th, uncompressed or DEFLATE, value = x + 100 y.
std::vector<uint8_t> MakeU16Tiled(bool big, bool be, uint32_t width, uint32_t height, uint32_t tw, uint32_t th,
                                  bool deflate = false, Entries extra = {}) {
    TiffBuilder b(big, be);
    const uint32_t across = (width + tw - 1) / tw, down = (height + th - 1) / th;
    std::vector<uint64_t> offsets, counts;
    for (uint32_t ty = 0; ty < down; ++ty) {
        for (uint32_t tx = 0; tx < across; ++tx) {
            std::vector<uint8_t> tile(size_t(tw) * th * 2);
            for (uint32_t y = 0; y < th; ++y)
                for (uint32_t x = 0; x < tw; ++x)
                    StoreEndian<uint16_t>(tile.data() + (size_t(y) * tw + x) * 2,
                                          static_cast<uint16_t>((tx * tw + x) + 100 * (ty * th + y)), be);
            if (deflate) tile = Deflate(tile);
            offsets.push_back(b.AddData(tile));
            counts.push_back(tile.size());
        }
    }
    Entries e = {b.Long(tag::kImageWidth, {width}),
                 b.Long(tag::kImageLength, {height}),
                 b.Short(tag::kBitsPerSample, {16}),
                 b.Short(tag::kCompression, {static_cast<uint16_t>(deflate ? 8 : 1)}),
                 b.Short(tag::kPhotometric, {1}),
                 b.Short(tag::kSamplesPerPixel, {1}),
                 b.Long(tag::kTileWidth, {tw}),
                 b.Long(tag::kTileLength, {th})};
    if (big) {
        e.push_back(b.Long8(tag::kTileOffsets, offsets));
        e.push_back(b.Long8(tag::kTileByteCounts, counts));
    } else {
        std::vector<uint32_t> o32(offsets.begin(), offsets.end()), c32(counts.begin(), counts.end());
        e.push_back(b.Long(tag::kTileOffsets, o32));
        e.push_back(b.Long(tag::kTileByteCounts, c32));
    }
    for (auto& x : extra) e.push_back(std::move(x));
    b.AddIfd(e);
    return b.bytes();
}

TEST(TiffReader, ByteOrdersAndBigTiff) {
    for (bool big : {false, true}) {
        for (bool be : {false, true}) {
            auto r = TiffReader::Open(Memory(MakeU16Tiled(big, be, 40, 20, 16, 16)));
            ASSERT_TRUE(r.ok()) << r.status().ToString();
            EXPECT_EQ((*r)->IsBigTiff(), big);
            EXPECT_EQ((*r)->IsBigEndian(), be);
            const IfdInfo& i = (*r)->Ifd(0);
            EXPECT_EQ(i.width, 40u);
            EXPECT_EQ(i.blocksAcross, 3u);
            EXPECT_EQ(i.blocksDown, 2u);
            EXPECT_EQ(i.sampleType, SampleType::kU16);
            for (uint32_t t = 0; t < i.blockCount; ++t) {
                auto blk = (*r)->ReadBlock(0, t);
                ASSERT_TRUE(blk.ok()) << blk.status().ToString();
                const uint32_t tx = i.BlockColumn(t), ty = i.BlockRow(t);
                for (uint32_t y = 0; y < 16; ++y) {
                    for (uint32_t x = 0; x < 16; ++x) {
                        uint16_t v;
                        std::memcpy(&v, blk->data() + (y * 16 + x) * 2, 2);
                        ASSERT_EQ(v, (tx * 16 + x) + 100 * (ty * 16 + y)) << big << be;
                    }
                }
            }
        }
    }
}

TEST(TiffReader, EveryFieldType) {
    for (bool big : {false, true}) {
        for (bool be : {false, true}) {
            TiffBuilder b(big, be);
            Entries e = {b.Long(tag::kImageWidth, {4}), b.Long(tag::kImageLength, {1}), b.Short(tag::kBitsPerSample, {8}),
                         b.Long(tag::kStripOffsets, {8}), b.Long(tag::kStripByteCounts, {4})};
            e.push_back(b.Bytes(60001, 1, {1, 2, 250}));                                             // BYTE
            e.push_back(b.Ascii(60002, "hello"));                                                    // ASCII
            e.push_back(b.Short(60003, {1, 65535}));                                                 // SHORT
            e.push_back(b.Long(60004, {7, 4000000000u}));                                            // LONG
            e.push_back({60005, 5, 2, b.Values(std::vector<uint32_t>{1, 4, 3, 0})});                 // RATIONAL
            e.push_back(b.Bytes(60006, 6, {0xFF, 0x7F}));                                            // SBYTE
            e.push_back(b.Bytes(60007, 7, {9, 8, 7, 6, 5, 4, 3, 2, 1}));                             // UNDEFINED
            e.push_back({60008, 8, 2, b.Values(std::vector<int16_t>{-2, 300})});                     // SSHORT
            e.push_back({60009, 9, 1, b.Values(std::vector<int32_t>{-70000})});                      // SLONG
            e.push_back({60010, 10, 1, b.Values(std::vector<int32_t>{-3, 4})});                      // SRATIONAL
            e.push_back({60011, 11, 2, b.Values(std::vector<float>{1.5f, -2.25f})});                 // FLOAT
            e.push_back(b.Double(60012, {3.141592653589793}));                                       // DOUBLE
            e.push_back({60013, 13, 1, b.Values(std::vector<uint32_t>{0})});                         // IFD
            e.push_back(b.Long8(60016, {uint64_t{1} << 40}));                                        // LONG8
            e.push_back({60017, 17, 1, b.Values(std::vector<int64_t>{-5})});                         // SLONG8
            e.push_back({60018, 18, 1, b.Values(std::vector<uint64_t>{0})});                         // IFD8
            e.push_back({60099, 99, 3, {1, 2, 3}});                                                  // unknown type
            b.AddIfd(e);
            auto r = TiffReader::Open(Memory(b.bytes()));
            ASSERT_TRUE(r.ok()) << r.status().ToString();
            const TiffReader& rd = **r;
            const IfdInfo& i = rd.Ifd(0);
            auto u = [&](uint16_t t) { return *rd.ReadTagUInts(*i.FindTag(t), 100); };
            auto d = [&](uint16_t t) { return *rd.ReadTagDoubles(*i.FindTag(t), 100); };
            EXPECT_EQ(u(60001), (std::vector<uint64_t>{1, 2, 250}));
            EXPECT_EQ(*rd.ReadTagString(*i.FindTag(60002), 100), "hello");
            EXPECT_EQ(u(60003), (std::vector<uint64_t>{1, 65535}));
            EXPECT_EQ(u(60004), (std::vector<uint64_t>{7, 4000000000u}));
            EXPECT_EQ(d(60005), (std::vector<double>{0.25, 0.0}));
            EXPECT_EQ(d(60006), (std::vector<double>{-1, 127}));
            EXPECT_EQ(rd.ReadTagBytes(*i.FindTag(60007), 100)->size(), 9u);
            EXPECT_EQ(d(60008), (std::vector<double>{-2, 300}));
            EXPECT_EQ(d(60009), (std::vector<double>{-70000}));
            EXPECT_EQ(d(60010), (std::vector<double>{-0.75}));
            EXPECT_EQ(d(60011), (std::vector<double>{1.5, -2.25}));
            EXPECT_EQ(d(60012), (std::vector<double>{3.141592653589793}));
            EXPECT_EQ(u(60016), (std::vector<uint64_t>{uint64_t{1} << 40}));
            EXPECT_EQ(d(60017), (std::vector<double>{-5}));
            EXPECT_EQ(rd.ReadTagBytes(*i.FindTag(60099), 100).code(), StatusCode::kUnsupported);
            EXPECT_EQ(rd.ReadTagUInts(*i.FindTag(60011), 100).code(), StatusCode::kCorrupt);
            EXPECT_EQ(rd.ReadTagUInts(*i.FindTag(60004), 1).code(), StatusCode::kResourceExhausted);
        }
    }
}

Entries SmallImage(TiffBuilder& b, uint32_t w, uint32_t h, uint32_t nst, uint64_t dataOffset) {
    return {b.Long(tag::kNewSubfileType, {nst}), b.Long(tag::kImageWidth, {w}),       b.Long(tag::kImageLength, {h}),
            b.Short(tag::kBitsPerSample, {8}),     b.Short(tag::kPhotometric, {nst & 4 ? uint16_t{4} : uint16_t{1}}),
            b.Long(tag::kRowsPerStrip, {h}),       b.Long(tag::kStripOffsets, {static_cast<uint32_t>(dataOffset)}),
            b.Long(tag::kStripByteCounts, {w * h})};
}

TEST(TiffReader, SubIfdsAndSubfileKinds) {
    TiffBuilder b(false, false);
    const uint64_t data = b.AddData(std::vector<uint8_t>(64 * 64, 3));
    const uint64_t ov1 = b.AddIfd(SmallImage(b, 32, 32, 1, data), false);
    const uint64_t ov2 = b.AddIfd(SmallImage(b, 16, 16, 1, data), false);
    Entries main = SmallImage(b, 64, 64, 0, data);
    main.push_back(b.Long(tag::kSubIfds, {static_cast<uint32_t>(ov1), static_cast<uint32_t>(ov2)}));
    b.AddIfd(main);
    b.AddIfd(SmallImage(b, 64, 64, 4, data));
    b.AddIfd(SmallImage(b, 32, 32, 5, data));
    b.AddIfd(SmallImage(b, 64, 64, 0, data));  // second page
    auto r = TiffReader::Open(Memory(b.bytes()));
    ASSERT_TRUE(r.ok()) << r.status().ToString();
    const TiffReader& rd = **r;
    ASSERT_EQ(rd.IfdCount(), 6u);
    EXPECT_EQ(rd.MainImage(), 0u);
    EXPECT_EQ(rd.Ifd(1).parent, 0);
    EXPECT_EQ(rd.Ifd(1).kind, SubfileKind::kOverview);
    EXPECT_EQ(rd.Ifd(2).width, 16u);
    EXPECT_EQ(rd.Ifd(3).kind, SubfileKind::kMask);
    EXPECT_EQ(rd.Ifd(4).kind, SubfileKind::kOverviewMask);
    EXPECT_EQ(rd.Ifd(5).kind, SubfileKind::kPage);
    EXPECT_EQ(rd.Overviews(), (std::vector<uint32_t>{1, 2}));
    EXPECT_EQ(rd.MaskOf(0), 3u);
    EXPECT_EQ(rd.MaskOf(1), 4u);
    EXPECT_FALSE(rd.MaskOf(2).has_value());
}

TEST(TiffReader, StripsAndShortLastStrip) {
    TiffBuilder b(false, true);
    std::vector<uint32_t> offs, counts;
    for (uint32_t s = 0; s < 4; ++s) {
        const uint32_t rows = s < 3 ? 3 : 1;
        std::vector<uint8_t> strip(7 * rows);
        for (size_t k = 0; k < strip.size(); ++k) strip[k] = static_cast<uint8_t>(s * 50 + k);
        offs.push_back(static_cast<uint32_t>(b.AddData(strip)));
        counts.push_back(static_cast<uint32_t>(strip.size()));
    }
    b.AddIfd({b.Long(tag::kImageWidth, {7}), b.Long(tag::kImageLength, {10}), b.Short(tag::kBitsPerSample, {8}),
              b.Long(tag::kRowsPerStrip, {3}), b.Long(tag::kStripOffsets, offs), b.Long(tag::kStripByteCounts, counts)});
    auto r = TiffReader::Open(Memory(b.bytes()));
    ASSERT_TRUE(r.ok());
    const IfdInfo& i = (*r)->Ifd(0);
    EXPECT_FALSE(i.tiled);
    EXPECT_EQ(i.blockCount, 4u);
    EXPECT_EQ(i.BlockRows(3), 1u);
    auto blk = (*r)->ReadBlock(0, 3);
    ASSERT_TRUE(blk.ok());
    EXPECT_EQ(blk->height, 1u);
    EXPECT_EQ(blk->data()[6], 156);
}

TEST(TiffReader, SparseBlocksTakeTheNodataValue) {
    struct Case {
        uint16_t bits, format;
        const char* nodata;
        SampleType type;
    };
    for (const Case& c : {Case{16, 1, "7", SampleType::kU16}, Case{16, 2, "-1e10", SampleType::kI16},
                          Case{32, 3, "nan", SampleType::kF32}, Case{8, 1, "", SampleType::kU8},
                          Case{64, 3, "-inf", SampleType::kF64}, Case{8, 2, "-3.6", SampleType::kI8}}) {
        TiffBuilder b(false, false);
        Entries e = {b.Long(tag::kImageWidth, {8}),        b.Long(tag::kImageLength, {8}),
                     b.Short(tag::kBitsPerSample, {c.bits}), b.Short(tag::kSampleFormat, {c.format}),
                     b.Long(tag::kTileWidth, {16}),         b.Long(tag::kTileLength, {16}),
                     b.Long(tag::kTileOffsets, {0}),        b.Long(tag::kTileByteCounts, {0})};
        if (*c.nodata) e.push_back(b.Ascii(tag::kGdalNodata, c.nodata));
        b.AddIfd(e);
        auto r = TiffReader::Open(Memory(b.bytes()));
        ASSERT_TRUE(r.ok());
        auto blk = (*r)->ReadBlock(0, 0);
        ASSERT_TRUE(blk.ok());
        EXPECT_TRUE(blk->sparse);
        EXPECT_EQ(blk->sampleType, c.type);
        const uint8_t* p = blk->data();
        switch (c.type) {
            case SampleType::kU16: EXPECT_EQ(LoadLE<uint16_t>(p + 254 * 2), 7); break;
            case SampleType::kI16: EXPECT_EQ(LoadLE<int16_t>(p + 254 * 2), -32768); break;
            case SampleType::kF32: EXPECT_TRUE(std::isnan(LoadLE<float>(p + 4 * 100))); break;
            case SampleType::kF64: EXPECT_EQ(LoadLE<double>(p + 8 * 3), -std::numeric_limits<double>::infinity()); break;
            case SampleType::kI8: EXPECT_EQ(static_cast<int8_t>(p[17]), -4); break;
            default: EXPECT_EQ(p[255], 0); break;
        }
    }
}

std::vector<uint8_t> GhostFile(bool corruptLeader, bool incompatible) {
    TiffBuilder b(false, false);
    std::string ghost =
        "LAYOUT=IFDS_BEFORE_DATA\nBLOCK_ORDER=ROW_MAJOR\nBLOCK_LEADER=SIZE_AS_UINT4\nBLOCK_TRAILER=LAST_4_BYTES_REPEATED\n"
        "KNOWN_INCOMPATIBLE_EDITION=";
    ghost += incompatible ? "YES\n" : "NO\n";
    ghost += " ";
    b.AddGhost(ghost);
    std::vector<uint8_t> tile(16 * 16);
    for (size_t k = 0; k < tile.size(); ++k) tile[k] = static_cast<uint8_t>(k);
    const std::vector<uint8_t> z = Deflate(tile);
    std::vector<uint8_t> framed(4);
    StoreLE<uint32_t>(framed.data(), static_cast<uint32_t>(z.size() + (corruptLeader ? 1 : 0)));
    framed.insert(framed.end(), z.begin(), z.end());
    framed.insert(framed.end(), z.end() - 4, z.end());
    const uint64_t off = b.AddData(framed) + 4;
    b.AddIfd({b.Long(tag::kImageWidth, {16}), b.Long(tag::kImageLength, {16}), b.Short(tag::kBitsPerSample, {8}),
              b.Short(tag::kCompression, {8}), b.Long(tag::kTileWidth, {16}), b.Long(tag::kTileLength, {16}),
              b.Long(tag::kTileOffsets, {static_cast<uint32_t>(off)}),
              b.Long(tag::kTileByteCounts, {static_cast<uint32_t>(z.size())})});
    return b.bytes();
}

TEST(TiffReader, GhostHeaderLeaderAndTrailer) {
    for (bool mapped : {true, false}) {
        auto ok = TiffReader::Open(mapped ? Memory(GhostFile(false, false)) : Pread(GhostFile(false, false)));
        ASSERT_TRUE(ok.ok()) << ok.status().ToString();
        EXPECT_TRUE((*ok)->Ghost().present);
        EXPECT_TRUE((*ok)->Ghost().blockLeaderSizeAsUint4);
        EXPECT_TRUE((*ok)->Ghost().layoutIfdsBeforeData);
        auto blk = (*ok)->ReadBlock(0, 0);
        ASSERT_TRUE(blk.ok()) << blk.status().ToString();
        EXPECT_EQ(blk->data()[200], 200);
        auto bad = TiffReader::Open(mapped ? Memory(GhostFile(true, false)) : Pread(GhostFile(true, false)));
        ASSERT_TRUE(bad.ok());
        EXPECT_EQ((*bad)->ReadBlock(0, 0).code(), StatusCode::kCorrupt);
        auto edited = TiffReader::Open(Memory(GhostFile(true, true)));
        ASSERT_TRUE(edited.ok());
        EXPECT_TRUE((*edited)->ReadBlock(0, 0).ok());  // KNOWN_INCOMPATIBLE_EDITION: no check
    }
}

TEST(TiffReader, GhostHeaderLinesWithGdalPadding) {
    // GDAL's COG driver writes the mask line after a leading space: " MASK_INTERLEAVED_WITH_IMAGERY=YES".
    TiffBuilder b(false, false);
    b.AddGhost(
        "LAYOUT=IFDS_BEFORE_DATA\nBLOCK_ORDER=ROW_MAJOR\nBLOCK_LEADER=SIZE_AS_UINT4\n"
        "BLOCK_TRAILER=LAST_4_BYTES_REPEATED\nKNOWN_INCOMPATIBLE_EDITION=NO\n MASK_INTERLEAVED_WITH_IMAGERY=YES\n");
    const std::vector<uint8_t> pixels(16 * 16, 7);
    const uint64_t off = b.AddData(pixels);
    b.AddIfd({b.Long(tag::kImageWidth, {16}), b.Long(tag::kImageLength, {16}), b.Short(tag::kBitsPerSample, {8}),
              b.Short(tag::kCompression, {1}), b.Long(tag::kTileWidth, {16}), b.Long(tag::kTileLength, {16}),
              b.Long(tag::kTileOffsets, {static_cast<uint32_t>(off)}), b.Long(tag::kTileByteCounts, {16 * 16})});
    auto r = TiffReader::Open(Memory(b.bytes()));
    ASSERT_TRUE(r.ok()) << r.status().ToString();
    EXPECT_TRUE((*r)->Ghost().maskInterleavedWithImagery);
    EXPECT_FALSE((*r)->Ghost().knownIncompatibleEdition);
    EXPECT_TRUE((*r)->Ghost().blockTrailerLast4BytesRepeated);
}

TEST(TiffReader, OversizedCompressedBlockNamesItsIfd) {
    TiffReaderOptions o;
    o.maxCompressedBlockBytes = 100;
    auto r = TiffReader::Open(Memory(GhostFile(false, false)), o);
    ASSERT_TRUE(r.ok()) << r.status().ToString();
    const auto blk = (*r)->ReadBlock(0, 0);
    EXPECT_EQ(blk.code(), StatusCode::kResourceExhausted);
    EXPECT_NE(blk.status().message().find("block 0 of IFD 0: "), std::string_view::npos) << blk.status().message();
}

TEST(TiffReader, GhostHeaderOfGdalCogWithMask) {
    // R1: a COG written by GDAL with an internal mask (brighton-beach).
    const char* r1 = "/mnt/data/repos/test_data/ortho/brighton-beach-cog.tif";
    auto r = TiffReader::Open(r1);
    if (!r.ok() && r.code() == StatusCode::kNotFound) GTEST_SKIP() << "R1 missing";
    ASSERT_TRUE(r.ok()) << r.status().ToString();
    const GhostHeader& g = (*r)->Ghost();
    EXPECT_TRUE(g.present);
    EXPECT_TRUE(g.layoutIfdsBeforeData);
    EXPECT_TRUE(g.blockOrderRowMajor);
    EXPECT_TRUE(g.blockLeaderSizeAsUint4);
    EXPECT_TRUE(g.blockTrailerLast4BytesRepeated);
    EXPECT_FALSE(g.knownIncompatibleEdition);
    EXPECT_TRUE(g.maskInterleavedWithImagery);
}

TEST(TiffReader, LazyBlockIndexWithRacingFirstAccess) {
    for (int round = 0; round < 20; ++round) {
        std::atomic<int> reads{0};
        auto r = TiffReader::Open(Pread(MakeU16Tiled(round % 2 == 1, false, 300, 200, 16, 16, true), &reads));
        ASSERT_TRUE(r.ok());
        const TiffReader& rd = **r;
        EXPECT_FALSE(rd.BlockIndexLoaded(0));
        const int readsAfterOpen = reads.load();
        std::vector<std::thread> threads;
        std::atomic<int> failures{0};
        for (int t = 0; t < 16; ++t) {
            threads.emplace_back([&, t] {
                const uint32_t idx = static_cast<uint32_t>(t * 37) % rd.Ifd(0).blockCount;
                auto blk = rd.ReadBlock(0, idx);
                if (!blk.ok()) failures.fetch_add(1);
            });
        }
        for (auto& th : threads) th.join();
        EXPECT_EQ(failures.load(), 0);
        EXPECT_TRUE(rd.BlockIndexLoaded(0));
        // Two array reads for the index (offsets, counts), then one read per block.
        EXPECT_EQ(reads.load() - readsAfterOpen, 2 + 16);
    }
}

TEST(TiffReader, ConcurrentDecodesEqualSerialOnes) {
    auto r = TiffReader::Open(Memory(MakeU16Tiled(true, true, 1000, 700, 64, 32, true)));
    ASSERT_TRUE(r.ok());
    const TiffReader& rd = **r;
    const uint32_t n = rd.Ifd(0).blockCount;
    std::vector<std::vector<uint8_t>> serial(n);
    for (uint32_t i = 0; i < n; ++i) {
        auto b = rd.ReadBlock(0, i);
        ASSERT_TRUE(b.ok());
        serial[i].assign(b->data(), b->data() + b->size());
    }
    std::atomic<int> mismatches{0};
    std::vector<std::thread> threads;
    for (int t = 0; t < 32; ++t) {
        threads.emplace_back([&, t] {
            std::mt19937 rng(static_cast<uint32_t>(t));
            for (int k = 0; k < 300; ++k) {
                const uint32_t i = static_cast<uint32_t>(rng() % n);
                auto b = rd.ReadBlock(0, i);
                if (!b.ok() || std::memcmp(b->data(), serial[i].data(), serial[i].size()) != 0) mismatches.fetch_add(1);
            }
        });
    }
    for (auto& th : threads) th.join();
    EXPECT_EQ(mismatches.load(), 0);
}

TEST(TiffReader, MalformedFilesGiveErrors) {
    // Not a TIFF / truncated headers.
    EXPECT_EQ(TiffReader::Open(Memory({})).code(), StatusCode::kCorrupt);
    EXPECT_EQ(TiffReader::Open(Memory({'I', 'I', 42, 0})).code(), StatusCode::kCorrupt);
    EXPECT_EQ(TiffReader::Open(Memory({'X', 'X', 42, 0, 8, 0, 0, 0})).code(), StatusCode::kCorrupt);
    EXPECT_EQ(TiffReader::Open(Memory({'I', 'I', 43, 0, 4, 0, 0, 0, 16, 0, 0, 0, 0, 0, 0, 0})).code(), StatusCode::kCorrupt);
    // First IFD past the end.
    EXPECT_EQ(TiffReader::Open(Memory({'I', 'I', 42, 0, 0xFF, 0xFF, 0, 0})).code(), StatusCode::kCorrupt);
    // IFD that points to itself: the loop is cut, the first directory is kept.
    {
        auto bytes = MakeU16Tiled(false, false, 16, 16, 16, 16);
        const uint32_t ifd = LoadLE<uint32_t>(bytes.data() + 4);
        const uint16_t n = LoadLE<uint16_t>(bytes.data() + ifd);
        StoreLE<uint32_t>(bytes.data() + ifd + 2 + n * 12, ifd);
        auto r = TiffReader::Open(Memory(bytes));
        ASSERT_TRUE(r.ok());
        EXPECT_EQ((*r)->IfdCount(), 1u);
    }
    // Huge dimensions with tiny tiles: too many blocks, refused at open.
    {
        TiffBuilder b(false, false);
        b.AddIfd({b.Long(tag::kImageWidth, {0xFFFFFFFFu}), b.Long(tag::kImageLength, {0xFFFFFFFFu}),
                  b.Long(tag::kTileWidth, {16}), b.Long(tag::kTileLength, {16}), b.Long(tag::kTileOffsets, {0}),
                  b.Long(tag::kTileByteCounts, {0})});
        EXPECT_EQ(TiffReader::Open(Memory(b.bytes())).code(), StatusCode::kResourceExhausted);
    }
    // One huge strip: opens, but the block is refused (limit), never allocated.
    {
        TiffBuilder b(false, false);
        b.AddIfd({b.Long(tag::kImageWidth, {100000}), b.Long(tag::kImageLength, {100000}),
                  b.Short(tag::kBitsPerSample, {32}), b.Long(tag::kStripOffsets, {8}),
                  b.Long(tag::kStripByteCounts, {100})});
        auto r = TiffReader::Open(Memory(b.bytes()));
        ASSERT_TRUE(r.ok());
        EXPECT_EQ((*r)->ReadBlock(0, 0).code(), StatusCode::kResourceExhausted);
    }
    // Block offset past the end, missing ByteCounts, unsupported sample format, zero size.
    {
        TiffBuilder b(false, false);
        b.AddIfd({b.Long(tag::kImageWidth, {4}), b.Long(tag::kImageLength, {4}), b.Short(tag::kBitsPerSample, {8}),
                  b.Long(tag::kStripOffsets, {1000000}), b.Long(tag::kStripByteCounts, {16})});
        auto r = TiffReader::Open(Memory(b.bytes()));
        ASSERT_TRUE(r.ok());
        EXPECT_EQ((*r)->ReadBlock(0, 0).code(), StatusCode::kCorrupt);
        EXPECT_EQ((*r)->ReadBlock(0, 1).code(), StatusCode::kOutOfRange);
    }
    {
        TiffBuilder b(false, false);
        b.AddIfd({b.Long(tag::kImageWidth, {4}), b.Long(tag::kImageLength, {4}), b.Short(tag::kBitsPerSample, {8}),
                  b.Long(tag::kStripOffsets, {8})});
        auto r = TiffReader::Open(Memory(b.bytes()));
        ASSERT_TRUE(r.ok());
        EXPECT_EQ((*r)->ReadBlock(0, 0).code(), StatusCode::kCorrupt);
    }
    {
        TiffBuilder b(false, false);
        b.AddIfd({b.Long(tag::kImageWidth, {4}), b.Long(tag::kImageLength, {4}), b.Short(tag::kBitsPerSample, {12}),
                  b.Long(tag::kStripOffsets, {8}), b.Long(tag::kStripByteCounts, {24})});
        auto r = TiffReader::Open(Memory(b.bytes()));
        ASSERT_TRUE(r.ok());
        EXPECT_EQ((*r)->ReadBlock(0, 0).code(), StatusCode::kUnsupported);
    }
    {
        TiffBuilder b(false, false);
        b.AddIfd({b.Long(tag::kImageWidth, {0}), b.Long(tag::kImageLength, {4})});
        EXPECT_EQ(TiffReader::Open(Memory(b.bytes())).code(), StatusCode::kCorrupt);
    }
}

// Byte mutations of valid files: open, every block of every IFD, raw reads and georeferencing
// must return statuses (ASan / UBSan report anything else).
TEST(TiffReader, MutationFuzz) {
    std::vector<std::vector<uint8_t>> seeds;
    seeds.push_back(MakeU16Tiled(false, false, 40, 30, 16, 16, true));
    seeds.push_back(MakeU16Tiled(true, true, 40, 30, 16, 16, false));
    seeds.push_back(GhostFile(false, false));
    {
        TiffBuilder b(false, false);
        const uint64_t data = b.AddData(std::vector<uint8_t>(64 * 64, 3));
        const uint64_t ov1 = b.AddIfd(SmallImage(b, 32, 32, 1, data), false);
        Entries main = SmallImage(b, 64, 64, 0, data);
        main.push_back(b.Long(tag::kSubIfds, {static_cast<uint32_t>(ov1)}));
        main.push_back(b.Short(tag::kGeoKeyDirectory, {1, 1, 0, 3, 1024, 0, 1, 1, 3072, 0, 1, 32633, 1026, 34737, 5, 0}));
        main.push_back(b.Ascii(tag::kGeoAsciiParams, "test|"));
        main.push_back(b.Double(tag::kModelPixelScale, {1, 1, 0}));
        main.push_back(b.Double(tag::kModelTiepoint, {0, 0, 0, 500000, 4000000, 0}));
        b.AddIfd(main);
        b.AddIfd(SmallImage(b, 64, 64, 4, data));
        seeds.push_back(b.bytes());
    }
    std::mt19937 rng(77);
    int opened = 0;
    for (int iter = 0; iter < 4000; ++iter) {
        std::vector<uint8_t> bytes = seeds[size_t(iter) % seeds.size()];
        const int flips = 1 + static_cast<int>(rng() % 6);
        for (int f = 0; f < flips; ++f) {
            const size_t pos = rng() % bytes.size();
            switch (rng() % 4) {
                case 0: bytes[pos] = static_cast<uint8_t>(rng()); break;
                case 1: bytes[pos] = 0xFF; break;
                case 2: bytes[pos] = 0; break;
                default: bytes[pos] ^= static_cast<uint8_t>(1u << (rng() % 8)); break;
            }
        }
        if (rng() % 8 == 0) bytes.resize(rng() % bytes.size());
        TiffReaderOptions opt;
        opt.maxBlockBytes = 1 << 20;
        auto r = TiffReader::Open(iter % 2 ? Memory(bytes) : Pread(bytes), opt);
        if (!r.ok()) continue;
        ++opened;
        const TiffReader& rd = **r;
        for (uint32_t i = 0; i < rd.IfdCount(); ++i) {
            const uint32_t n = std::min<uint32_t>(rd.Ifd(i).blockCount, 64);
            for (uint32_t k = 0; k < n; ++k) {
                rd.ReadBlock(i, k).status().IgnoreError();
                rd.ReadBlockRaw(i, k).status().IgnoreError();
            }
        }
        ReadGeoreferencing(rd).status().IgnoreError();
    }
    EXPECT_GT(opened, 100);
}

TEST(GeoTiff, WorldFileAndKeys) {
    auto gt = ParseWorldFile("2.5\n0\n0\n-2.5\n1000.25\n2000.75\n");
    ASSERT_TRUE(gt.has_value());
    EXPECT_EQ((*gt)[0], 999.0);
    EXPECT_EQ((*gt)[3], 2002.0);
    EXPECT_EQ((*gt)[5], -2.5);
    EXPECT_FALSE(ParseWorldFile("0\n0\n0\n-2.5\n1\n2\n").has_value());
    EXPECT_FALSE(ParseWorldFile("1\n2\n3\n").has_value());

    TiffBuilder b(false, true);
    const uint64_t data = b.AddData(std::vector<uint8_t>(16, 0));
    Entries e = SmallImage(b, 4, 4, 0, data);
    // Keys: model projected, raster PixelIsPoint, PCS 32633, citation in ASCII, a key whose
    // ASCII range is out of bounds (dropped), a double key.
    e.push_back(b.Short(tag::kGeoKeyDirectory, {1, 1, 0, 5, 1024, 0, 1, 1, 1025, 0, 1, 2, 1026, 34737, 6, 0, 3072, 0, 1,
                                                32633, 4097, 34737, 99, 2}));
    e.push_back(b.Ascii(tag::kGeoAsciiParams, "hello|"));
    e.push_back(b.Double(tag::kModelPixelScale, {2, 3, 0}));
    e.push_back(b.Double(tag::kModelTiepoint, {0, 0, 0, 500000, 4000000, 0}));
    b.AddIfd(e);
    auto r = TiffReader::Open(Memory(b.bytes()));
    ASSERT_TRUE(r.ok());
    auto g = ReadGeoreferencing(**r);
    ASSERT_TRUE(g.ok());
    EXPECT_EQ(g->keys.Ascii(geokey::kGTCitation), "hello");
    EXPECT_FALSE(g->keys.Has(geokey::kVerticalCitation));
    EXPECT_TRUE(g->pixelIsPoint);
    ASSERT_TRUE(g->geoTransform.has_value());
    EXPECT_EQ((*g->geoTransform)[0], 499999.0);   // shifted by half a pixel
    EXPECT_EQ((*g->geoTransform)[3], 4000001.5);
    EXPECT_EQ(g->crsDefinition, "EPSG:32633");
    ASSERT_TRUE(g->crs.has_value());
    EXPECT_EQ(g->crs->Epsg(), 32633);
}

// ExpandBits (eight pixels per table lookup) and SummarizeBits (from the packed bits) against
// the bit-by-bit definitions: widths with and without a partial last byte, padding bits set to
// garbage, rows of all 0, all 1 and mixed, `one` 1 and 255.
TEST(TiffBits, ExpandAndSummarizeEqualTheBitByBitDefinition) {
    std::mt19937 rng(11);
    for (uint32_t width : {1u, 7u, 8u, 9u, 63u, 64u, 65u, 512u, 513u}) {
        for (int pattern = 0; pattern < 4; ++pattern) {
            const uint32_t rows = 5;
            const size_t stride = (width + 7) / 8 + 3;  // padding bytes after each row
            std::vector<uint8_t> bits(stride * rows);
            for (auto& b : bits) b = static_cast<uint8_t>(rng());
            auto bit = [&](uint32_t r, uint32_t x) { return (bits[r * stride + x / 8] >> (7 - x % 8)) & 1; };
            auto setBit = [&](uint32_t r, uint32_t x, int v) {
                uint8_t& b = bits[r * stride + x / 8];
                const uint8_t m = static_cast<uint8_t>(0x80u >> (x % 8));
                b = static_cast<uint8_t>(v ? (b | m) : (b & ~m));
            };
            for (uint32_t r = 0; r < rows; ++r)
                for (uint32_t x = 0; x < width; ++x) {
                    if (pattern == 0) setBit(r, x, 0);
                    if (pattern == 1) setBit(r, x, 1);
                    if (pattern == 2) setBit(r, x, r == rows - 1 && x == width - 1 ? 0 : 1);  // one 0, last
                }
            for (uint8_t one : {uint8_t{1}, uint8_t{255}}) {
                std::vector<uint8_t> out(width * rows + 7, 0xAB);
                ExpandBits(bits.data(), stride, width, rows, one, out.data(), width);
                for (uint32_t r = 0; r < rows; ++r)
                    for (uint32_t x = 0; x < width; ++x)
                        ASSERT_EQ(out[r * width + x], bit(r, x) ? one : 0) << width << " " << pattern << " " << r << " " << x;
            }
            for (uint32_t rs : {rows, 1u, 0u}) {
                bool e0 = false, e1 = false;
                for (uint32_t r = 0; r < rs; ++r)
                    for (uint32_t x = 0; x < width; ++x) (bit(r, x) ? e1 : e0) = true;
                const BitsSummary s = SummarizeBits(bits.data(), stride, width, rs);
                EXPECT_EQ(s.any0, e0) << width << " " << pattern << " rows " << rs;
                EXPECT_EQ(s.any1, e1) << width << " " << pattern << " rows " << rs;
            }
        }
    }
}

}  // namespace
}  // namespace ddbt::cog::tiff
