// SPDX-License-Identifier: MPL-2.0
#include "copc/readers/las_reader.h"

#include <lazperf/excepts.hpp>
#include <lazperf/lazperf.hpp>
#include <lazperf/layered.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <string>

#include "copc/las/las_crs.h"
#include "copc/las/point_format.h"
#include "copc/readers/convert.h"
#include "core/crs/crs.h"
#include "core/util/endian.h"

namespace ddbt::copc::readers {

namespace {

constexpr size_t kBatchPoints = 1u << 16;

/// A bounded input callback over a chunk: reads past the end give zeros for a few bytes (an
/// arithmetic decoder may look ahead), then an error.
struct BoundedInput {
    const uint8_t* p;
    size_t left;
    size_t slack = 16;
    void operator()(unsigned char* b, size_t n) {
        const size_t take = std::min(n, left);
        std::memcpy(b, p, take);
        p += take;
        left -= take;
        if (take < n) {
            const size_t rest = n - take;
            if (rest > slack) throw lazperf::error("LAZ chunk is truncated");
            slack -= rest;
            std::memset(b + take, 0, rest);
        }
    }
};

/// The calling thread's layered decoder of formats 6-8 with every layer (reused across chunks).
lazperf::layered_decompressor& ThreadLayered(int format, int eb) {
    struct Entry {
        int format, eb;
        std::unique_ptr<lazperf::layered_decompressor> d;
    };
    thread_local std::vector<Entry> entries;
    for (Entry& e : entries)
        if (e.format == format && e.eb == eb) return *e.d;
    if (entries.size() >= 4) entries.erase(entries.begin());
    entries.push_back({format, eb, std::make_unique<lazperf::layered_decompressor>(format, eb, lazperf::layer::all)});
    return *entries.back().d;
}

struct Scratch {
    std::vector<uint8_t> in, out, chunk;
    std::vector<double> xyz;
};
Scratch& ThreadScratch() {
    thread_local Scratch s;
    return s;
}

/// Decodes the first `limit` points of a LAZ chunk into raw records of the file's format.
Status DecodeLazChunk(const las::LasHeader& h, std::span<const uint8_t> chunk, uint32_t chunkCount, uint32_t limit,
                      uint8_t* out) {
    const int f = h.pointFormat;
    const size_t rs = h.pointRecordLength;
    const int eb = static_cast<int>(h.ExtraBytesPerPoint());
    if (limit == 0) return Status();
    try {
        if (f >= 6 && f <= 10) {
            lazperf::layered_decompressor& d = ThreadLayered(f, eb);
            d.begin(reinterpret_cast<const char*>(chunk.data()), chunk.size());
            char* p = reinterpret_cast<char*>(out);
            p = d.decompress(p);
            if (d.chunkPointCount() != chunkCount)
                return Corrupt("LAZ chunk holds " + std::to_string(d.chunkPointCount()) + " points, the chunk table says " +
                               std::to_string(chunkCount));
            for (uint32_t i = 1; i < limit; ++i) p = d.decompress(p);
            return Status();
        }
        if (f <= 5) {
            BoundedInput in{chunk.data(), chunk.size()};
            lazperf::las_decompressor::ptr d = lazperf::build_las_decompressor(std::ref(in), f, static_cast<size_t>(eb));
            if (!d) return Unsupported("LAZ point format " + std::to_string(f));
            char* p = reinterpret_cast<char*>(out);
            for (uint32_t i = 0; i < limit; ++i, p += rs) d->decompress(p);
            return Status();
        }
    } catch (const std::bad_alloc&) {
        throw;
    } catch (const std::exception& e) {
        return Corrupt(std::string("LAZ chunk: ") + e.what());
    }
    return Unsupported("compressed point format " + std::to_string(f));
}

std::string WktOf(const las::LasMetadata& meta) {
    auto crs = las::ResolveCrs(meta);
    if (crs.ok() && crs->Valid()) {
        std::string w = crs->ToWkt(crs::WktFlavor::Wkt1Gdal);
        if (!w.empty()) return w;
    }
    if (meta.wkt) return *meta.wkt;
    return {};
}

}  // namespace

Expected<std::vector<LazChunk>> ReadLazChunkTable(const io::ByteSource& source, const las::LasMetadata& meta) {
    const las::LasHeader& h = meta.header;
    if (!meta.laz) return Corrupt("compressed LAS file without a LAZ VLR");
    const las::LazVlr& laz = *meta.laz;
    if (laz.compressor != 2 && laz.compressor != 3)
        return Unsupported("LAZ compressor " + std::to_string(laz.compressor) + " (only chunked LAZ is read)");
    const uint64_t size = source.Size();
    if (uint64_t{h.pointDataOffset} + 8 > size) return Corrupt("LAZ file too short for its chunk table offset");
    uint8_t b8[8];
    DDBT_TRY(source.Read(h.pointDataOffset, b8));
    int64_t tableOffset = LoadLE<int64_t>(b8);
    if (tableOffset == -1) {
        // LASzip on an unseekable stream: the real offset is in the last 8 bytes of the file.
        if (size < 8) return Corrupt("LAZ file too short");
        DDBT_TRY(source.Read(size - 8, b8));
        tableOffset = LoadLE<int64_t>(b8);
    }
    const uint64_t dataStart = uint64_t{h.pointDataOffset} + 8;
    if (tableOffset < static_cast<int64_t>(dataStart) || static_cast<uint64_t>(tableOffset) + 8 > size)
        return Corrupt("LAZ chunk table offset " + std::to_string(tableOffset) + " is outside the file");
    uint8_t th[8];
    DDBT_TRY(source.Read(static_cast<uint64_t>(tableOffset), th));
    const uint32_t version = LoadLE<uint32_t>(th);
    const uint32_t count = LoadLE<uint32_t>(th + 4);
    if (version != 0) return Corrupt("LAZ chunk table version " + std::to_string(version));
    std::vector<LazChunk> chunks;
    if (count == 0) {
        if (h.pointCount != 0) return Corrupt("LAZ chunk table is empty");
        return chunks;
    }
    const bool variable = laz.VariableChunks();
    if (!variable && laz.chunkSize == 0) return Corrupt("LAZ chunk size is 0");
    // Each entry takes at least a bit or so; bound the count by the bytes available.
    const uint64_t tableBytes = size - static_cast<uint64_t>(tableOffset) - 8;
    if (count > (tableBytes + 64) * 16) return Corrupt("LAZ chunk table count is implausible");
    std::vector<lazperf::chunk> table;
    try {
        std::vector<uint8_t> bytes(static_cast<size_t>(std::min<uint64_t>(tableBytes, uint64_t{count} * 16 + 64)));
        DDBT_TRY(source.Read(static_cast<uint64_t>(tableOffset) + 8, bytes));
        BoundedInput in{bytes.data(), bytes.size(), 64};
        table = lazperf::decompress_chunk_table(std::ref(in), count, variable);
    } catch (const std::bad_alloc&) {
        throw;
    } catch (const std::exception& e) {
        return Corrupt(std::string("LAZ chunk table: ") + e.what());
    }
    chunks.reserve(count);
    uint64_t off = dataStart, first = 0;
    for (uint32_t i = 0; i < count; ++i) {
        LazChunk c;
        c.offset = off;
        c.bytes = table[i].offset;
        c.firstPoint = first;
        if (variable) {
            c.count = static_cast<uint32_t>(table[i].count);
        } else {
            const uint64_t left = h.pointCount > first ? h.pointCount - first : 0;
            c.count = static_cast<uint32_t>(std::min<uint64_t>(laz.chunkSize, left));
        }
        off += c.bytes;
        first += c.count;
        if (off > static_cast<uint64_t>(tableOffset)) return Corrupt("LAZ chunk " + std::to_string(i) + " runs past the chunk table");
        if (c.count != 0) chunks.push_back(c);
    }
    if (first != h.pointCount)
        return Corrupt("LAZ chunk table holds " + std::to_string(first) + " points, the header says " +
                       std::to_string(h.pointCount));
    return chunks;
}

void ConvertLasRecords(int inFormat, size_t inLength, const uint8_t* in, size_t count, const OutputSpec& spec,
                       const std::array<double, 3>& inScale, const std::array<double, 3>& inOffset, uint8_t* out,
                       double* xyzOut) {
    const las::PointLayout L = las::LayoutOf(inFormat);
    const size_t outSize = spec.RecordSize();
    const size_t inEb = inLength - L.baseSize;
    const size_t ebCopy = std::min<size_t>(inEb, spec.extraBytes);
    const bool outRgb = spec.format >= 7, outNir = spec.format == 8;
    for (size_t i = 0; i < count; ++i, in += inLength, out += outSize) {
        if (xyzOut != nullptr) {
            for (size_t a = 0; a < 3; ++a)
                xyzOut[3 * i + a] = LoadLE<int32_t>(in + 4 * a) * inScale[a] + inOffset[a];
        } else {
            std::memcpy(out, in, 12);
        }
        std::memcpy(out + rec::kIntensity, in + 12, 2);
        if (L.extended) {
            // Bytes 14-29: returns, flags, class, user data, scan angle, point source, GPS time.
            std::memcpy(out + rec::kReturns, in + 14, 16);
        } else {
            LegacyReturnsAndClass(in[14], in[15], out + rec::kReturns);
            out[rec::kUserData] = in[17];
            StoreLE<int16_t>(out + rec::kScanAngle, ScanAngleFromDegrees(static_cast<float>(static_cast<int8_t>(in[16]))));
            std::memcpy(out + rec::kPointSource, in + 18, 2);
            if (L.HasGpsTime()) std::memcpy(out + rec::kGpsTime, in + L.gpsTime, 8);
            else std::memset(out + rec::kGpsTime, 0, 8);
        }
        if (outRgb) {
            if (L.HasRgb()) std::memcpy(out + rec::kRgb, in + L.rgb, 6);
            else std::memset(out + rec::kRgb, 0, 6);
        }
        if (outNir) {
            if (L.HasNir()) std::memcpy(out + rec::kNir, in + L.nir, 2);
            else std::memset(out + rec::kNir, 0, 2);
        }
        uint8_t* eb = out + spec.BaseSize();
        if (ebCopy) std::memcpy(eb, in + L.baseSize, ebCopy);
        if (spec.extraBytes > ebCopy) std::memset(eb + ebCopy, 0, spec.extraBytes - ebCopy);
    }
}

Expected<std::unique_ptr<LasReader>> LasReader::Open(const std::string& path, const ReaderOptions& options) {
    // Positional reads, not a mapping: the pages of a mapped input would count in the resident
    // set of the build (9.9 GB for a 1.33 G point LAZ) although only one chunk per thread is in use.
    DDBT_ASSIGN_OR_RETURN(std::unique_ptr<io::ByteSource> src, io::OpenByteSource(path, io::SourceKind::kPread));
    return Open(std::shared_ptr<const io::ByteSource>(std::move(src)), path, options);
}

Expected<std::unique_ptr<LasReader>> LasReader::Open(std::shared_ptr<const io::ByteSource> source, std::string path,
                                                     const ReaderOptions& options) {
    std::unique_ptr<LasReader> r(new LasReader());
    r->source_ = std::move(source);
    r->pointsPerTask_ = std::max<uint64_t>(options.pointsPerTask, 1024);
    DDBT_ASSIGN_OR_RETURN(r->meta_, las::ReadMetadata(*r->source_));
    const las::LasHeader& h = r->meta_.header;
    SourceInfo& info = r->info_;
    info.path = std::move(path);
    info.kind = h.compressed ? "laz" : "las";
    info.pointCount = h.pointCount;
    info.isLas = true;
    info.lasFormat = h.pointFormat;
    info.lasScale = h.scale;
    info.lasOffset = h.offset;
    info.lasExtraBytes = h.ExtraBytesPerPoint();
    info.lasHeader = h;
    const las::PointLayout L = h.Layout();
    info.hasRgb = L.HasRgb();
    info.hasNir = L.HasNir();
    info.hasGpsTime = L.HasGpsTime();
    info.hasIntensity = true;
    info.rgb16 = true;
    // Header bounds are used to plan when they are finite and ordered.
    bool ok = true;
    for (size_t a = 0; a < 3; ++a) {
        info.min[a] = h.min[a];
        info.max[a] = h.max[a];
        if (!std::isfinite(h.min[a]) || !std::isfinite(h.max[a]) || h.min[a] > h.max[a]) ok = false;
    }
    info.boundsKnown = ok && h.pointCount > 0;
    if (const las::VlrRecord* eb = r->meta_.FindVlr("LASF_Spec", 4); eb != nullptr && eb->payloadKept && !r->meta_.extraBytes.empty()) {
        info.extraBytesVlr = eb->payload;
        info.extraBytes = r->meta_.extraBytes;
    }
    info.wkt = WktOf(r->meta_);

    if (h.compressed) {
        DDBT_ASSIGN_OR_RETURN(r->chunks_, ReadLazChunkTable(*r->source_, r->meta_));
    } else {
        const uint64_t need = uint64_t{h.pointDataOffset} + h.pointCount * uint64_t{h.pointRecordLength};
        if (need > r->source_->Size())
            return Corrupt("LAS file holds fewer point records than its header says (" + std::to_string(h.pointCount) + ")");
    }
    return r;
}

size_t LasReader::TaskCount() const noexcept {
    if (meta_.header.compressed) return chunks_.size();
    return static_cast<size_t>((meta_.header.pointCount + pointsPerTask_ - 1) / pointsPerTask_);
}

uint64_t LasReader::TaskPoints(size_t task) const noexcept {
    if (meta_.header.compressed) return task < chunks_.size() ? chunks_[task].count : 0;
    const uint64_t first = task * pointsPerTask_;
    if (first >= meta_.header.pointCount) return 0;
    return std::min<uint64_t>(pointsPerTask_, meta_.header.pointCount - first);
}

Status LasReader::ReadRawRecords(size_t task, uint32_t count, std::vector<uint8_t>& out) const {
    const las::LasHeader& h = meta_.header;
    const size_t rs = h.pointRecordLength;
    const uint64_t n = TaskPoints(task);
    if (count > n) return InvalidArgument("ReadRawRecords: more points than the task holds");
    out.resize(count * rs);
    if (h.compressed) {
        const LazChunk& c = chunks_[task];
        std::span<const uint8_t> bytes;
        std::vector<uint8_t> copy;
        if (source_->Data() != nullptr) {
            bytes = {source_->Data() + c.offset, static_cast<size_t>(c.bytes)};
        } else {
            copy.resize(static_cast<size_t>(c.bytes));
            DDBT_TRY(source_->Read(c.offset, copy));
            bytes = copy;
        }
        return DecodeLazChunk(h, bytes, c.count, count, out.data());
    }
    return source_->Read(h.pointDataOffset + task * pointsPerTask_ * rs, std::span<uint8_t>(out.data(), count * rs));
}

Status LasReader::ReadTask(size_t task, uint32_t limit, bool stride, const OutputSpec& spec, BatchSink& sink) const {
    const las::LasHeader& h = meta_.header;
    const size_t rs = h.pointRecordLength;
    const uint64_t n = TaskPoints(task);
    if (n == 0) return Status();
    const bool passthrough = spec.quantized && spec.scale == h.scale && spec.offset == h.offset;
    Scratch& s = ThreadScratch();
    const uint8_t* recs = nullptr;
    uint64_t total = std::min<uint64_t>(n, limit);
    if (h.compressed) {
        const LazChunk& c = chunks_[task];
        std::span<const uint8_t> bytes;
        if (source_->Data() != nullptr) {
            bytes = {source_->Data() + c.offset, static_cast<size_t>(c.bytes)};
        } else {
            s.chunk.resize(static_cast<size_t>(c.bytes));
            DDBT_TRY(source_->Read(c.offset, s.chunk));
            bytes = s.chunk;
        }
        s.in.resize(static_cast<size_t>(total) * rs);
        DDBT_TRY(DecodeLazChunk(h, bytes, c.count, static_cast<uint32_t>(total), s.in.data()));
        recs = s.in.data();
    } else {
        const uint64_t first = task * pointsPerTask_;
        const uint64_t off = h.pointDataOffset + first * rs;
        if (stride && total < n) {
            s.in.resize(static_cast<size_t>(total) * rs);
            for (uint64_t i = 0; i < total; ++i) {
                const uint64_t k = i * n / total;
                DDBT_TRY(source_->Read(off + k * rs, std::span<uint8_t>(s.in.data() + i * rs, rs)));
            }
            recs = s.in.data();
        } else if (source_->Data() != nullptr) {
            recs = source_->Data() + off;
        } else {
            s.in.resize(static_cast<size_t>(total) * rs);
            DDBT_TRY(source_->Read(off, std::span<uint8_t>(s.in.data(), s.in.size())));
            recs = s.in.data();
        }
    }
    const size_t outSize = spec.RecordSize();
    for (uint64_t done = 0; done < total;) {
        const size_t m = static_cast<size_t>(std::min<uint64_t>(kBatchPoints, total - done));
        s.out.resize(m * outSize);
        double* xyz = nullptr;
        if (!passthrough) {
            s.xyz.resize(m * 3);
            xyz = s.xyz.data();
        }
        ConvertLasRecords(h.pointFormat, rs, recs + done * rs, m, spec, h.scale, h.offset, s.out.data(), xyz);
        Batch b{m, s.out.data(), xyz};
        DDBT_TRY(sink.Consume(b));
        done += m;
    }
    return Status();
}

Status LasReader::Read(size_t task, const OutputSpec& spec, BatchSink& sink) const {
    return ReadTask(task, UINT32_MAX, false, spec, sink);
}

Status LasReader::Sample(size_t task, size_t maxPoints, const OutputSpec& spec, BatchSink& sink) const {
    return ReadTask(task, static_cast<uint32_t>(std::min<size_t>(maxPoints, UINT32_MAX)), true, spec, sink);
}

}  // namespace ddbt::copc::readers
