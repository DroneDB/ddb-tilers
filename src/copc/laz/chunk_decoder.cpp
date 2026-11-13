// SPDX-License-Identifier: MPL-2.0
#include "copc/laz/chunk_decoder.h"

#include <lazperf/layered.hpp>
#include <lazperf/lazperf.hpp>
#include <lazperf/readers.hpp>

#include <cstring>
#include <exception>
#include <string>

#include "copc/las/point_format.h"

namespace ddbt::copc::laz {

static_assert(kLayerXY == lazperf::layer::xy && kLayerZ == lazperf::layer::z &&
              kLayerClassification == lazperf::layer::classification && kLayerFlags == lazperf::layer::flags &&
              kLayerIntensity == lazperf::layer::intensity && kLayerScanAngle == lazperf::layer::scan_angle &&
              kLayerUserData == lazperf::layer::user_data && kLayerPointSource == lazperf::layer::point_source &&
              kLayerGpsTime == lazperf::layer::gps_time && kLayerRgb == lazperf::layer::rgb &&
              kLayerNir == lazperf::layer::nir && kLayerExtraBytes == lazperf::layer::extra_bytes &&
              kLayersAll == lazperf::layer::all);

struct ChunkDecoder::Impl {
    Impl(int format, int extraBytes, uint32_t layers) : d(format, extraBytes, layers) {}
    lazperf::layered_decompressor d;
};

ChunkDecoder::ChunkDecoder(int format, int extraBytes) : format_(format), extraBytes_(extraBytes) {}
ChunkDecoder::~ChunkDecoder() = default;

Expected<std::unique_ptr<ChunkDecoder>> ChunkDecoder::Create(int format, int extraBytes, uint32_t layers) {
    if (format < 6 || format > 8)
        return Unsupported("layered LAZ decoding is for point formats 6-8, not " + std::to_string(format));
    if (extraBytes < 0 || extraBytes > 65535 - 38) return InvalidArgument("extra bytes out of range");
    std::unique_ptr<ChunkDecoder> d(new ChunkDecoder(format, extraBytes));
    d->impl_ = std::make_unique<Impl>(format, extraBytes, layers);
    return d;
}

uint32_t ChunkDecoder::layers() const noexcept { return impl_->d.layers(); }
size_t ChunkDecoder::recordSize() const noexcept { return impl_->d.pointSize(); }

Status ChunkDecoder::DecodeSoa(std::span<const uint8_t> chunk, uint32_t pointCount, const SoaOut& out) {
    if (pointCount == 0) return Status();
    ++chunks_;
    lazperf::layered_decompressor& d = impl_->d;
    try {
        d.begin(reinterpret_cast<const char*>(chunk.data()), chunk.size());
        lazperf::soa_sink s;
        s.x = out.x;
        s.y = out.y;
        s.z = out.z;
        s.r = out.r;
        s.g = out.g;
        s.b = out.b;
        s.nir = out.nir;
        d.decompress(s, 1);
        if (d.chunkPointCount() != pointCount)
            return Corrupt("LAZ chunk holds " + std::to_string(d.chunkPointCount()) + " points, " +
                           std::to_string(pointCount) + " expected");
        auto advance = [](auto* p) { return p ? p + 1 : p; };
        s.x = advance(s.x);
        s.y = advance(s.y);
        s.z = advance(s.z);
        s.r = advance(s.r);
        s.g = advance(s.g);
        s.b = advance(s.b);
        s.nir = advance(s.nir);
        d.decompress(s, pointCount - 1);
    } catch (const std::bad_alloc&) {
        throw;
    } catch (const std::exception& e) {
        return Corrupt(std::string("LAZ chunk: ") + e.what());
    }
    return Status();
}

Status ChunkDecoder::DecodeRecords(std::span<const uint8_t> chunk, uint32_t pointCount, std::span<uint8_t> out) {
    const size_t rs = recordSize();
    if (out.size() / rs < pointCount) return InvalidArgument("DecodeRecords: output smaller than the points");
    if (pointCount == 0) return Status();
    ++chunks_;
    lazperf::layered_decompressor& d = impl_->d;
    try {
        d.begin(reinterpret_cast<const char*>(chunk.data()), chunk.size());
        char* p = reinterpret_cast<char*>(out.data());
        p = d.decompress(p);
        if (d.chunkPointCount() != pointCount)
            return Corrupt("LAZ chunk holds " + std::to_string(d.chunkPointCount()) + " points, " +
                           std::to_string(pointCount) + " expected");
        for (uint32_t i = 1; i < pointCount; ++i) p = d.decompress(p);
    } catch (const std::bad_alloc&) {
        throw;
    } catch (const std::exception& e) {
        return Corrupt(std::string("LAZ chunk: ") + e.what());
    }
    return Status();
}

Expected<ChunkDecoder*> ThreadDecoder(int format, int extraBytes, uint32_t layers) {
    struct Entry {
        int format;
        int extraBytes;
        uint32_t layers;
        uint64_t lastUse;
        std::unique_ptr<ChunkDecoder> decoder;
    };
    struct Pool {
        std::vector<Entry> entries;
        uint64_t clock = 0;
    };
    constexpr size_t kMaxPerThread = 8;
    thread_local Pool pool;
    ++pool.clock;
    for (Entry& e : pool.entries) {
        if (e.format == format && e.extraBytes == extraBytes && e.layers == layers) {
            e.lastUse = pool.clock;
            return e.decoder.get();
        }
    }
    DDBT_ASSIGN_OR_RETURN(std::unique_ptr<ChunkDecoder> d, ChunkDecoder::Create(format, extraBytes, layers));
    if (pool.entries.size() >= kMaxPerThread) {
        size_t oldest = 0;
        for (size_t i = 1; i < pool.entries.size(); ++i)
            if (pool.entries[i].lastUse < pool.entries[oldest].lastUse) oldest = i;
        pool.entries.erase(pool.entries.begin() + static_cast<ptrdiff_t>(oldest));
    }
    pool.entries.push_back({format, extraBytes, layers, pool.clock, std::move(d)});
    return pool.entries.back().decoder.get();
}

namespace {

Status CheckFormat(int format, int extraBytes) {
    // Formats 4 and 5: the fork's WAVEPACKET13 item (third_party/lazperf/FORK.md).
    if (format < 0 || format > 8)
        return Unsupported("laz-perf reads point formats 0-8, not " + std::to_string(format));
    if (extraBytes < 0 || extraBytes > 65535 - las::BaseRecordSize(format)) return InvalidArgument("extra bytes out of range");
    return Status();
}

}  // namespace

Status DecodeChunkReference(int format, int extraBytes, std::span<const uint8_t> chunk, uint32_t pointCount,
                            std::span<uint8_t> out) {
    DDBT_TRY(CheckFormat(format, extraBytes));
    const size_t rs = las::BaseRecordSize(format) + static_cast<size_t>(extraBytes);
    if (out.size() / rs < pointCount) return InvalidArgument("DecodeChunkReference: output smaller than the points");
    if (pointCount == 0) return Status();
    if (chunk.empty()) return Corrupt("LAZ chunk is empty");
    try {
        lazperf::reader::chunk_decompressor d(format, extraBytes, reinterpret_cast<const char*>(chunk.data()));
        char* p = reinterpret_cast<char*>(out.data());
        for (uint32_t i = 0; i < pointCount; ++i, p += rs) d.decompress(p);
    } catch (const std::bad_alloc&) {
        throw;
    } catch (const std::exception& e) {
        return Corrupt(std::string("LAZ chunk: ") + e.what());
    }
    return Status();
}

Expected<std::vector<uint8_t>> CompressChunk(int format, int extraBytes, std::span<const uint8_t> records,
                                             uint32_t pointCount) {
    DDBT_TRY(CheckFormat(format, extraBytes));
    const size_t rs = las::BaseRecordSize(format) + static_cast<size_t>(extraBytes);
    if (records.size() / rs < pointCount) return InvalidArgument("CompressChunk: fewer records than points");
    std::vector<uint8_t> out;
    try {
        lazperf::OutputCb cb = [&out](const unsigned char* b, size_t n) { out.insert(out.end(), b, b + n); };
        lazperf::las_compressor::ptr c = lazperf::build_las_compressor(cb, format, static_cast<size_t>(extraBytes));
        const char* p = reinterpret_cast<const char*>(records.data());
        for (uint32_t i = 0; i < pointCount; ++i, p += rs) c->compress(p);
        c->done();
    } catch (const std::bad_alloc&) {
        throw;
    } catch (const std::exception& e) {
        return InternalError(std::string("LAZ compress: ") + e.what());
    }
    return out;
}

std::vector<uint8_t> CompressChunkTable(std::span<const ChunkTableEntry> chunks, bool variable) {
    std::vector<lazperf::chunk> table;
    table.reserve(chunks.size());
    for (const ChunkTableEntry& c : chunks) table.push_back({c.count, c.bytes});
    std::vector<uint8_t> out;
    lazperf::compress_chunk_table([&out](const unsigned char* b, size_t n) { out.insert(out.end(), b, b + n); }, table,
                                  variable);
    return out;
}

}  // namespace ddbt::copc::laz
