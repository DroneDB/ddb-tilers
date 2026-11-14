// SPDX-License-Identifier: MPL-2.0
#include "cog/writer/cog_writer.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>

#include "cog/tiff/tiff_tags.h"
#include "core/io/scratch_file.h"
#include "core/util/endian.h"

namespace ddbt::cog::writer {

namespace {

using encode::ByteBuffer;

constexpr uint64_t kClassicLimit = 0xFFFFFFFFull;

// ---------------------------------------------------------------------------------------------
// TIFF directory serialisation

struct Entry {
    uint16_t tag = 0;
    uint16_t type = 0;
    uint64_t count = 0;
    std::vector<uint8_t> data;  // little-endian values
};

template <typename T>
void PutLE(std::vector<uint8_t>& v, T x) {
    uint8_t b[sizeof(T)];
    StoreLE<T>(b, x);
    v.insert(v.end(), b, b + sizeof(T));
}

Entry Shorts(uint16_t tag, const std::vector<uint16_t>& vals) {
    Entry e{tag, 3, vals.size(), {}};
    for (const uint16_t v : vals) PutLE<uint16_t>(e.data, v);
    return e;
}
Entry Short(uint16_t tag, uint16_t v) { return Shorts(tag, {v}); }
Entry Long(uint16_t tag, uint32_t v) {
    Entry e{tag, 4, 1, {}};
    PutLE<uint32_t>(e.data, v);
    return e;
}
// SHORT when the value fits (libtiff writes ImageWidth / TileWidth as SHORT when it can).
Entry ShortOrLong(uint16_t tag, uint32_t v) { return v <= 0xFFFF ? Short(tag, static_cast<uint16_t>(v)) : Long(tag, v); }
Entry Doubles(uint16_t tag, const std::vector<double>& vals) {
    Entry e{tag, 12, vals.size(), {}};
    for (const double v : vals) PutLE<double>(e.data, v);
    return e;
}
Entry Ascii(uint16_t tag, const std::string& s) {
    Entry e{tag, 2, s.size() + 1, {}};
    e.data.assign(s.begin(), s.end());
    e.data.push_back(0);
    return e;
}
Entry Undefined(uint16_t tag, const std::vector<uint8_t>& bytes) { return Entry{tag, 7, bytes.size(), bytes}; }
Entry Rationals(uint16_t tag, const std::vector<std::pair<uint32_t, uint32_t>>& vals) {
    Entry e{tag, 5, vals.size(), {}};
    for (const auto& [n, d] : vals) {
        PutLE<uint32_t>(e.data, n);
        PutLE<uint32_t>(e.data, d);
    }
    return e;
}

struct Ifd {
    std::vector<Entry> entries;
    // Indexes of the TileOffsets / TileByteCounts entries (filled at layout time).
    size_t offsetsEntry = 0, countsEntry = 0;
    uint64_t offset = 0;      // file offset of the directory
    uint64_t dataOffset = 0;  // file offset of its out-of-line values

    void Sort() {
        std::stable_sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) { return a.tag < b.tag; });
    }
    size_t DirBytes(bool big) const { return big ? 8 + 20 * entries.size() + 8 : 2 + 12 * entries.size() + 4; }
    size_t OutOfLineBytes(bool big) const {
        size_t n = 0;
        const size_t inl = big ? 8 : 4;
        for (const Entry& e : entries)
            if (e.data.size() > inl) n += (e.data.size() + 1) & ~size_t{1};
        return n;
    }
    // Serialises the directory and its values; `next` = offset of the next IFD (0: last).
    void Write(bool big, uint64_t next, std::vector<uint8_t>& out) const {
        const size_t inl = big ? 8 : 4;
        std::vector<uint8_t> dir, data;
        if (big)
            PutLE<uint64_t>(dir, entries.size());
        else
            PutLE<uint16_t>(dir, static_cast<uint16_t>(entries.size()));
        uint64_t valuePos = dataOffset;
        for (const Entry& e : entries) {
            PutLE<uint16_t>(dir, e.tag);
            PutLE<uint16_t>(dir, e.type);
            if (big)
                PutLE<uint64_t>(dir, e.count);
            else
                PutLE<uint32_t>(dir, static_cast<uint32_t>(e.count));
            if (e.data.size() <= inl) {
                std::vector<uint8_t> v = e.data;
                v.resize(inl, 0);
                dir.insert(dir.end(), v.begin(), v.end());
            } else {
                if (big)
                    PutLE<uint64_t>(dir, valuePos);
                else
                    PutLE<uint32_t>(dir, static_cast<uint32_t>(valuePos));
                data.insert(data.end(), e.data.begin(), e.data.end());
                if (data.size() & 1) data.push_back(0);
                valuePos = dataOffset + data.size();
            }
        }
        if (big)
            PutLE<uint64_t>(dir, next);
        else
            PutLE<uint32_t>(dir, static_cast<uint32_t>(next));
        out.insert(out.end(), dir.begin(), dir.end());
        out.insert(out.end(), data.begin(), data.end());
    }
};

// GDAL's GeoKeys for EPSG:3857 (GTIFSetFromOGISDefnEx of EPSG:3857, GDAL 3.x), in its order.
const std::vector<uint16_t> kGeoKeys3857 = {1, 1, 0, 7, 1024, 0, 1, 1, 1025, 0, 1, 1, 1026, 34737, 25, 0,
                                            2049, 34737, 7, 25, 2054, 0, 1, 9102, 3072, 0, 1, 3857, 3076, 0, 1, 9001};
const char kGeoAscii3857[] = "WGS 84 / Pseudo-Mercator|WGS 84|";

struct BlockRec {
    uint64_t offset = 0;  // L0: relative to the start of the full-resolution data (data, not leader)
    uint32_t size = 0;
    bool present = false;
    bool inRam = false;
    ByteBuffer bytes;  // overview blocks kept in memory
};

}  // namespace

std::string GhostHeader(bool mask) {
    std::string md = "LAYOUT=IFDS_BEFORE_DATA\nBLOCK_ORDER=ROW_MAJOR\nBLOCK_LEADER=SIZE_AS_UINT4\n"
                     "BLOCK_TRAILER=LAST_4_BYTES_REPEATED\nKNOWN_INCOMPATIBLE_EDITION=NO\n ";
    if (mask) md += "MASK_INTERLEAVED_WITH_IMAGERY=YES\n";
    char head[64];
    std::snprintf(head, sizeof(head), "GDAL_STRUCTURAL_METADATA_SIZE=%06d bytes\n", static_cast<int>(md.size()));
    return head + md;
}

struct CogWriter::Impl {
    std::string path;
    ImageSpec spec;
    WriterOptions options;
    WriterStats stats;

    struct Level {
        uint32_t width, height, across, down;
        std::vector<BlockRec> image, mask;
    };
    std::vector<Level> levels;

    mutable std::mutex mutex;
    // Full-resolution data file (becomes the output).
    io::ScratchFile data;
    uint64_t dataEnd = 0;
    uint64_t nextSeq = 0;
    std::map<uint64_t, ByteBuffer> pending;  // reorder buffer of full-resolution blocks
    uint64_t pendingBytes = 0;
    // Overview spill.
    io::ScratchFile spill;
    uint64_t spillEnd = 0;
    uint64_t ramBytes = 0;
    bool finished = false;
    Status sticky = Status::Ok();

    uint32_t Kinds() const { return spec.hasMask ? 2 : 1; }

    ~Impl() { Abort(); }

    void Abort() noexcept {
        data.Discard();
        spill.Discard();
    }

    // Writes leader + bytes + trailer at the end of the data file.
    Status WriteFullRes(uint64_t seq, ByteBuffer& b) {
        const uint64_t block = seq / Kinds();
        const bool isMask = (seq % Kinds()) == 1;
        BlockRec& rec = isMask ? levels[0].mask[block] : levels[0].image[block];
        rec.present = true;
        if (b.empty()) {
            ++stats.sparseBlocks;
            return Status::Ok();
        }
        const uint32_t n = static_cast<uint32_t>(b.size());
        uint8_t lead[4], trail[4] = {0, 0, 0, 0};
        StoreLE<uint32_t>(lead, n);
        std::memcpy(trail, b.data() + (n >= 4 ? n - 4 : 0), std::min<uint32_t>(n, 4));
        DDBT_TRY(data.WriteAt(dataEnd, lead));
        DDBT_TRY(data.WriteAt(dataEnd + 4, {b.data(), n}));
        DDBT_TRY(data.WriteAt(dataEnd + 4 + n, trail));
        rec.offset = dataEnd + 4;
        rec.size = n;
        dataEnd += 8 + static_cast<uint64_t>(n);
        stats.fullResBytes += n;
        return Status::Ok();
    }

    Status PutOverview(uint32_t level, uint64_t block, bool isMask, ByteBuffer&& b) {
        BlockRec& rec = isMask ? levels[level].mask[block] : levels[level].image[block];
        rec.present = true;
        if (b.empty()) {
            ++stats.sparseBlocks;
            return Status::Ok();
        }
        rec.size = static_cast<uint32_t>(b.size());
        stats.overviewBytes += b.size();
        if (ramBytes + b.size() <= options.overviewMemory) {
            ramBytes += b.size();
            rec.inRam = true;
            rec.bytes = std::move(b);
            return Status::Ok();
        }
        if (!spill.IsOpen()) {
            DDBT_ASSIGN_OR_RETURN(spill, io::ScratchFile::Create(path + ".ovr", io::ScratchKind::kAnonymous));
        }
        DDBT_TRY(spill.WriteAt(spillEnd, {b.data(), b.size()}));
        rec.offset = spillEnd;
        spillEnd += b.size();
        stats.spilledBytes += b.size();
        return Status::Ok();
    }

    // The directories, with placeholder offsets.
    std::vector<Ifd> BuildIfds(bool big) const {
        std::vector<Ifd> ifds;
        const uint16_t bits = static_cast<uint16_t>(tiff::SampleTypeSize(spec.type) * 8 * (spec.complex ? 2 : 1));
        uint16_t format = 1;
        switch (spec.type) {
            case tiff::SampleType::kI8: case tiff::SampleType::kI16: case tiff::SampleType::kI32:
            case tiff::SampleType::kI64: format = 2; break;
            case tiff::SampleType::kF16: case tiff::SampleType::kF32: case tiff::SampleType::kF64: format = 3; break;
            default: format = 1;
        }
        if (spec.complex) format = format == 3 ? 6 : 5;  // complex IEEE float / complex integer
        const uint16_t offType = big ? 16 : 4;
        auto blockArrays = [&](Ifd& d, size_t blocks) {
            Entry off{tiff::tag::kTileOffsets, offType, blocks, {}};
            off.data.resize(blocks * (big ? 8 : 4));
            Entry cnt{tiff::tag::kTileByteCounts, 4, blocks, {}};
            cnt.data.resize(blocks * 4);
            d.entries.push_back(std::move(off));
            d.entries.push_back(std::move(cnt));
        };
        auto imageIfd = [&](uint32_t level) {
            const Level& l = levels[level];
            Ifd d;
            if (level > 0) d.entries.push_back(Long(tiff::tag::kNewSubfileType, 1));
            d.entries.push_back(ShortOrLong(tiff::tag::kImageWidth, l.width));
            d.entries.push_back(ShortOrLong(tiff::tag::kImageLength, l.height));
            d.entries.push_back(Shorts(tiff::tag::kBitsPerSample, std::vector<uint16_t>(spec.samples, bits)));
            d.entries.push_back(Short(tiff::tag::kCompression, encode::CompressionTag(spec.codec)));
            d.entries.push_back(Short(tiff::tag::kPhotometric, spec.photometric));
            d.entries.push_back(Short(tiff::tag::kSamplesPerPixel, spec.samples));
            d.entries.push_back(Short(tiff::tag::kPlanarConfig, 1));
            if (spec.codec != encode::Codec::kJpeg && spec.predictor > 1)
                d.entries.push_back(Short(tiff::tag::kPredictor, static_cast<uint16_t>(spec.predictor)));
            d.entries.push_back(ShortOrLong(tiff::tag::kTileWidth, spec.blockSize));
            d.entries.push_back(ShortOrLong(tiff::tag::kTileLength, spec.blockSize));
            blockArrays(d, static_cast<size_t>(l.across) * l.down);
            if (!spec.extraSamples.empty()) d.entries.push_back(Shorts(tiff::tag::kExtraSamples, spec.extraSamples));
            if (!spec.colorMap.empty()) d.entries.push_back(Shorts(tiff::tag::kColorMap, spec.colorMap));
            d.entries.push_back(Shorts(tiff::tag::kSampleFormat, std::vector<uint16_t>(spec.samples, format)));
            if (spec.codec == encode::Codec::kJpeg) {
                if (!spec.jpegTables.empty()) d.entries.push_back(Undefined(tiff::tag::kJpegTables, spec.jpegTables));
                if (spec.photometric == tiff::photometric::kYCbCr) {
                    d.entries.push_back(Shorts(tiff::tag::kYCbCrSubsampling, {2, 2}));
                    d.entries.push_back(
                        Rationals(tiff::tag::kReferenceBlackWhite, {{0, 1}, {255, 1}, {128, 1}, {255, 1}, {128, 1}, {255, 1}}));
                }
            }
            if (level == 0 && spec.georeferenced) {
                const auto& gt = spec.geoTransform;
                if (gt[2] == 0.0 && gt[4] == 0.0) {
                    d.entries.push_back(Doubles(tiff::tag::kModelPixelScale, {gt[1], -gt[5], 0.0}));
                    d.entries.push_back(Doubles(tiff::tag::kModelTiepoint, {0.0, 0.0, 0.0, gt[0], gt[3], 0.0}));
                } else {
                    d.entries.push_back(Doubles(tiff::tag::kModelTransformation,
                                                {gt[1], gt[2], 0, gt[0], gt[4], gt[5], 0, gt[3], 0, 0, 0, 0, 0, 0, 0, 1}));
                }
                d.entries.push_back(Shorts(tiff::tag::kGeoKeyDirectory, kGeoKeys3857));
                d.entries.push_back(Ascii(tiff::tag::kGeoAsciiParams, kGeoAscii3857));
            }
            if (level == 0)
                for (const RawTag& t : spec.extraTags) d.entries.push_back(Entry{t.tag, t.type, t.count, t.data});
            if (level == 0 && !spec.gdalMetadata.empty())
                d.entries.push_back(Ascii(tiff::tag::kGdalMetadata, spec.gdalMetadata));
            if (!spec.noDataText.empty()) d.entries.push_back(Ascii(tiff::tag::kGdalNodata, spec.noDataText));
            d.Sort();
            return d;
        };
        auto maskIfd = [&](uint32_t level) {
            const Level& l = levels[level];
            Ifd d;
            d.entries.push_back(Long(tiff::tag::kNewSubfileType, level == 0 ? 4 : 5));
            d.entries.push_back(ShortOrLong(tiff::tag::kImageWidth, l.width));
            d.entries.push_back(ShortOrLong(tiff::tag::kImageLength, l.height));
            d.entries.push_back(Short(tiff::tag::kBitsPerSample, 1));
            d.entries.push_back(Short(tiff::tag::kCompression, 8));
            d.entries.push_back(Short(tiff::tag::kPhotometric, tiff::photometric::kMask));
            d.entries.push_back(Short(tiff::tag::kSamplesPerPixel, 1));
            d.entries.push_back(Short(tiff::tag::kPlanarConfig, 1));
            d.entries.push_back(Short(tiff::tag::kPredictor, 1));
            d.entries.push_back(ShortOrLong(tiff::tag::kTileWidth, spec.blockSize));
            d.entries.push_back(ShortOrLong(tiff::tag::kTileLength, spec.blockSize));
            blockArrays(d, static_cast<size_t>(l.across) * l.down);
            d.entries.push_back(Short(tiff::tag::kSampleFormat, 1));
            d.Sort();
            return d;
        };
        ifds.push_back(imageIfd(0));
        if (spec.hasMask) ifds.push_back(maskIfd(0));
        for (uint32_t l = 1; l < levels.size(); ++l) ifds.push_back(imageIfd(l));
        if (spec.hasMask)
            for (uint32_t l = 1; l < levels.size(); ++l) ifds.push_back(maskIfd(l));
        for (Ifd& d : ifds) {
            for (size_t i = 0; i < d.entries.size(); ++i) {
                if (d.entries[i].tag == tiff::tag::kTileOffsets) d.offsetsEntry = i;
                if (d.entries[i].tag == tiff::tag::kTileByteCounts) d.countsEntry = i;
            }
        }
        return ifds;
    }

    // Level and kind of directory i (same order as BuildIfds).
    std::pair<uint32_t, bool> IfdLevel(size_t i) const {
        const uint32_t n = static_cast<uint32_t>(levels.size());
        if (!spec.hasMask) return {static_cast<uint32_t>(i), false};
        if (i == 0) return {0, false};
        if (i == 1) return {0, true};
        if (i < n + 1) return {static_cast<uint32_t>(i - 1), false};
        return {static_cast<uint32_t>(i - n), true};
    }

    struct Layout {
        bool big = false;
        std::vector<Ifd> ifds;
        uint64_t headerEnd = 0;     // end of the IFDs and their values
        uint64_t prefixEnd = 0;     // end of the overview data
        uint64_t fullResStart = 0;  // where the full-resolution data begins (prefix, padded)
        std::vector<std::vector<uint64_t>> ovrImage, ovrMask;  // file offsets of overview blocks
        uint64_t fileEnd = 0;
    };

    Layout ComputeLayout(bool big, uint64_t alignment) const {
        Layout L;
        L.big = big;
        L.ifds = BuildIfds(big);
        uint64_t pos = (big ? 16 : 8) + GhostHeader(spec.hasMask).size();
        pos += pos & 1;
        for (Ifd& d : L.ifds) {
            d.offset = pos;
            pos += d.DirBytes(big);
            d.dataOffset = pos;
            pos += d.OutOfLineBytes(big);
            pos += pos & 1;
        }
        L.headerEnd = pos;
        L.ovrImage.resize(levels.size());
        L.ovrMask.resize(levels.size());
        for (size_t l = levels.size(); l-- > 1;) {
            const Level& lv = levels[l];
            L.ovrImage[l].assign(lv.image.size(), 0);
            L.ovrMask[l].assign(lv.mask.size(), 0);
            for (size_t b = 0; b < lv.image.size(); ++b) {
                if (lv.image[b].size > 0) {
                    L.ovrImage[l][b] = pos + 4;
                    pos += 8 + lv.image[b].size;
                }
                if (spec.hasMask && lv.mask[b].size > 0) {
                    L.ovrMask[l][b] = pos + 4;
                    pos += 8 + lv.mask[b].size;
                }
            }
        }
        L.prefixEnd = pos;
        L.fullResStart = alignment > 1 ? (pos + alignment - 1) / alignment * alignment : pos;
        L.fileEnd = L.fullResStart + dataEnd;
        // Fill the block arrays.
        for (size_t i = 0; i < L.ifds.size(); ++i) {
            const auto [level, isMask] = IfdLevel(i);
            const Level& lv = levels[level];
            const std::vector<BlockRec>& recs = isMask ? lv.mask : lv.image;
            Ifd& d = L.ifds[i];
            std::vector<uint8_t>& off = d.entries[d.offsetsEntry].data;
            std::vector<uint8_t>& cnt = d.entries[d.countsEntry].data;
            for (size_t b = 0; b < recs.size(); ++b) {
                uint64_t o = 0;
                if (recs[b].size > 0)
                    o = level == 0 ? L.fullResStart + recs[b].offset : (isMask ? L.ovrMask[level][b] : L.ovrImage[level][b]);
                if (big)
                    StoreLE<uint64_t>(off.data() + 8 * b, o);
                else
                    StoreLE<uint32_t>(off.data() + 4 * b, static_cast<uint32_t>(o));
                StoreLE<uint32_t>(cnt.data() + 4 * b, recs[b].size);
            }
        }
        return L;
    }

    Status WritePrefix(io::ScratchFile& out, const Layout& L) {
        std::vector<uint8_t> head;
        if (L.big) {
            const uint8_t h[16] = {'I', 'I', 43, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            head.assign(h, h + 16);
            StoreLE<uint64_t>(head.data() + 8, L.ifds[0].offset);
        } else {
            const uint8_t h[8] = {'I', 'I', 42, 0, 0, 0, 0, 0};
            head.assign(h, h + 8);
            StoreLE<uint32_t>(head.data() + 4, static_cast<uint32_t>(L.ifds[0].offset));
        }
        const std::string ghost = GhostHeader(spec.hasMask);
        head.insert(head.end(), ghost.begin(), ghost.end());
        if (head.size() & 1) head.push_back(0);
        for (size_t i = 0; i < L.ifds.size(); ++i) {
            const uint64_t next = i + 1 < L.ifds.size() ? L.ifds[i + 1].offset : 0;
            L.ifds[i].Write(L.big, next, head);
            if (head.size() & 1) head.push_back(0);
        }
        if (head.size() != L.headerEnd) return Status(StatusCode::kInternal, "COG writer: directory layout mismatch");
        // Overview data, smallest level first, streamed through a staging buffer.
        std::vector<uint8_t> stage;
        stage.reserve(size_t{8} << 20);
        stage.swap(head);
        uint64_t stageAt = 0;
        auto flush = [&]() -> Status {
            if (stage.empty()) return Status::Ok();
            DDBT_TRY(out.WriteAt(stageAt, stage));
            stageAt += stage.size();
            stage.clear();
            return Status::Ok();
        };
        std::vector<uint8_t> spillBuf;
        auto put = [&](const BlockRec& r) -> Status {
            if (r.size == 0) return Status::Ok();
            uint8_t lead[4];
            StoreLE<uint32_t>(lead, r.size);
            stage.insert(stage.end(), lead, lead + 4);
            const uint8_t* p = nullptr;
            if (r.inRam) {
                p = r.bytes.data();
            } else {
                spillBuf.resize(r.size);
                DDBT_TRY(spill.ReadAt(r.offset, spillBuf));
                p = spillBuf.data();
            }
            stage.insert(stage.end(), p, p + r.size);
            uint8_t trail[4] = {0, 0, 0, 0};
            std::memcpy(trail, p + (r.size >= 4 ? r.size - 4 : 0), std::min<uint32_t>(r.size, 4));
            stage.insert(stage.end(), trail, trail + 4);
            if (stage.size() >= (size_t{8} << 20)) return flush();
            return Status::Ok();
        };
        for (size_t l = levels.size(); l-- > 1;) {
            const Level& lv = levels[l];
            for (size_t b = 0; b < lv.image.size(); ++b) {
                DDBT_TRY(put(lv.image[b]));
                if (spec.hasMask) DDBT_TRY(put(lv.mask[b]));
            }
        }
        DDBT_TRY(flush());
        if (stageAt != L.prefixEnd) return Status(StatusCode::kInternal, "COG writer: overview layout mismatch");
        return Status::Ok();
    }

    Status Finish() {
        std::lock_guard lock(mutex);
        if (finished) return Status(StatusCode::kInternal, "COG writer: already finished");
        if (!sticky.ok()) return sticky;
        if (!pending.empty()) return Status(StatusCode::kInternal, "COG writer: full-resolution blocks missing");
        for (const Level& lv : levels) {
            for (const BlockRec& r : lv.image)
                if (!r.present) return Status(StatusCode::kInternal, "COG writer: a block was never put");
            for (const BlockRec& r : lv.mask)
                if (spec.hasMask && !r.present) return Status(StatusCode::kInternal, "COG writer: a mask block was never put");
        }
        const uint64_t blk = data.BlockSize();

        auto fits = [](const Layout& L) { return L.fileEnd <= kClassicLimit; };
        bool big = options.forceBigTiff;
        bool inserted = false;
        Layout L;
        // The insert-range attempt needs a block-aligned prefix.
        const bool tryInsert = options.insertRange && dataEnd > 0;
        if (tryInsert) {
            L = ComputeLayout(big, blk);
            if (!big && !fits(L)) L = ComputeLayout(big = true, blk);
            inserted = data.InsertRangeAtStart(L.fullResStart);
        }
        if (inserted) {
            DDBT_TRY(WritePrefix(data, L));
            // The padding between the overview data and the full-resolution data is a hole.
        } else {
            L = ComputeLayout(big = options.forceBigTiff, 1);
            if (!big && !fits(L)) L = ComputeLayout(big = true, 1);
            DDBT_ASSIGN_OR_RETURN(io::ScratchFile out, io::ScratchFile::Create(path + ".cog"));
            DDBT_TRY(WritePrefix(out, L));
            // Copy the full-resolution data after the prefix.
            uint64_t copied = 0;
            std::vector<uint8_t> buf(size_t{8} << 20);
            while (copied < dataEnd) {
                const size_t n = static_cast<size_t>(std::min<uint64_t>(buf.size(), dataEnd - copied));
                const std::span<uint8_t> chunk(buf.data(), n);
                DDBT_TRY(data.ReadAt(copied, chunk));
                DDBT_TRY(out.WriteAt(L.fullResStart + copied, chunk));
                copied += n;
            }
            data = std::move(out);  // discards the full-resolution file
        }
        DDBT_TRY(data.Publish(path, L.fileEnd, options.durable));
        spill.Discard();
        finished = true;
        stats.fileSize = L.fileEnd;
        stats.prefixBytes = L.prefixEnd;
        stats.bigTiff = big;
        stats.insertedRange = inserted;
        for (Level& lv : levels) {
            for (BlockRec& r : lv.image) r.bytes = ByteBuffer();
            for (BlockRec& r : lv.mask) r.bytes = ByteBuffer();
        }
        ramBytes = 0;
        return Status::Ok();
    }
};

CogWriter::CogWriter(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
CogWriter::~CogWriter() = default;

Expected<std::unique_ptr<CogWriter>> CogWriter::Create(const std::string& path, ImageSpec spec,
                                                       const WriterOptions& options) {
    if (spec.width == 0 || spec.height == 0 || spec.blockSize == 0 || spec.blockSize % 16 != 0)
        return Status(StatusCode::kInvalidArgument, "COG writer: bad image or block size");
    if (spec.samples == 0 || tiff::SampleTypeSize(spec.type) == 0)
        return Status(StatusCode::kInvalidArgument, "COG writer: bad sample layout");
    auto impl = std::make_unique<Impl>();
    impl->path = path;
    impl->spec = std::move(spec);
    impl->options = options;
    auto addLevel = [&](uint32_t w, uint32_t h) {
        Impl::Level l;
        l.width = w;
        l.height = h;
        l.across = (w + impl->spec.blockSize - 1) / impl->spec.blockSize;
        l.down = (h + impl->spec.blockSize - 1) / impl->spec.blockSize;
        l.image.resize(static_cast<size_t>(l.across) * l.down);
        if (impl->spec.hasMask) l.mask.resize(l.image.size());
        impl->levels.push_back(std::move(l));
    };
    addLevel(impl->spec.width, impl->spec.height);
    for (const auto& [w, h] : impl->spec.overviews) addLevel(w, h);
    DDBT_ASSIGN_OR_RETURN(impl->data, io::ScratchFile::Create(path + ".cog"));
    return std::unique_ptr<CogWriter>(new CogWriter(std::move(impl)));
}

uint32_t CogWriter::Levels() const noexcept { return static_cast<uint32_t>(impl_->levels.size()); }
uint32_t CogWriter::BlocksAcross(uint32_t level) const { return impl_->levels.at(level).across; }
uint32_t CogWriter::BlocksDown(uint32_t level) const { return impl_->levels.at(level).down; }

Status CogWriter::Put(uint32_t level, uint32_t col, uint32_t row, bool mask, encode::ByteBuffer&& bytes) {
    Impl& I = *impl_;
    if (level >= I.levels.size()) return Status(StatusCode::kInvalidArgument, "COG writer: bad level");
    const Impl::Level& lv = I.levels[level];
    if (col >= lv.across || row >= lv.down || (mask && !I.spec.hasMask))
        return Status(StatusCode::kInvalidArgument, "COG writer: bad block");
    const uint64_t block = static_cast<uint64_t>(row) * lv.across + col;
    std::lock_guard lock(I.mutex);
    if (I.finished) return Status(StatusCode::kInternal, "COG writer: finished");
    if (!I.sticky.ok()) return I.sticky;
    const BlockRec& rec = mask ? lv.mask[block] : lv.image[block];
    if (rec.present) return Status(StatusCode::kInvalidArgument, "COG writer: block put twice");
    if (level > 0) {
        Status s = I.PutOverview(level, block, mask, std::move(bytes));
        if (!s.ok()) I.sticky = s;
        return s;
    }
    const uint64_t seq = block * I.Kinds() + (mask ? 1 : 0);
    if (seq != I.nextSeq) {
        if (I.pending.count(seq)) return Status(StatusCode::kInvalidArgument, "COG writer: block put twice");
        I.pendingBytes += bytes.size();
        I.pending.emplace(seq, std::move(bytes));
        return Status::Ok();
    }
    Status s = I.WriteFullRes(seq, bytes);
    ++I.nextSeq;
    while (s.ok() && !I.pending.empty() && I.pending.begin()->first == I.nextSeq) {
        auto it = I.pending.begin();
        I.pendingBytes -= it->second.size();
        s = I.WriteFullRes(it->first, it->second);
        I.pending.erase(it);
        ++I.nextSeq;
    }
    if (!s.ok()) I.sticky = s;
    return s;
}

Status CogWriter::Finish() {
    Status s = impl_->Finish();
    if (!s.ok()) impl_->Abort();
    return s;
}

void CogWriter::Abort() noexcept { impl_->Abort(); }

const WriterStats& CogWriter::Stats() const noexcept { return impl_->stats; }

uint64_t CogWriter::BufferedBytes() const noexcept {
    std::lock_guard lock(impl_->mutex);
    return impl_->pendingBytes + impl_->ramBytes;
}

}  // namespace ddbt::cog::writer
