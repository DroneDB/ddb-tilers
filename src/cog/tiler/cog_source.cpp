// SPDX-License-Identifier: MPL-2.0
#include "cog/tiler/cog_source.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <mutex>
#include <span>
#include <unordered_map>

#ifdef _WIN32
#include <windows.h>

#include "core/io/platform.h"
#else
#include <sys/stat.h>
#endif

#include "cog/tiff/tiff_tags.h"
#include "cog/tiler/tile_error.h"
#include "cog/tiler/warp.h"
#include "core/cache/sharded_lru.h"
#include "core/crs/crs.h"
#include "core/io/byte_source.h"

namespace ddbt::cog::tiler {

namespace {

std::atomic<uint64_t> g_nextId{1};

struct FileState {
    bool exists = false;
    bool regular = false;
    FileIdentity id;
};

#ifdef _WIN32
// FILETIME (100 ns ticks since 1601) -> nanoseconds since 1970.
int64_t FileTimeNs(const FILETIME& t) {
    const int64_t ticks = static_cast<int64_t>((static_cast<uint64_t>(t.dwHighDateTime) << 32) | t.dwLowDateTime);
    return (ticks - 116444736000000000LL) * 100;
}
#endif

FileState StatFile(const std::string& path) {
    FileState st;
#ifndef _WIN32
    struct stat s {};
    if (::stat(path.c_str(), &s) != 0) return st;
    st.exists = true;
    st.regular = S_ISREG(s.st_mode);
    st.id.size = static_cast<uint64_t>(s.st_size);
    st.id.mtimeNs = static_cast<int64_t>(s.st_mtim.tv_sec) * 1000000000 + s.st_mtim.tv_nsec;
    st.id.ctimeNs = static_cast<int64_t>(s.st_ctim.tv_sec) * 1000000000 + s.st_ctim.tv_nsec;
    st.id.inode = static_cast<uint64_t>(s.st_ino);
    st.id.device = static_cast<uint64_t>(s.st_dev);
#else
    const auto wide = io::detail::ToWidePath(path);
    HANDLE h = INVALID_HANDLE_VALUE;
    if (wide.ok())
        h = CreateFileW(wide->c_str(), FILE_READ_ATTRIBUTES, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                        nullptr, OPEN_EXISTING, FILE_FLAG_BACKUP_SEMANTICS, nullptr);
    BY_HANDLE_FILE_INFORMATION info{};
    if (h != INVALID_HANDLE_VALUE) {
        const BOOL ok = GetFileInformationByHandle(h, &info);
        ::CloseHandle(h);
        if (ok) {
            st.exists = true;
            st.regular = (info.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0;
            st.id.size = (static_cast<uint64_t>(info.nFileSizeHigh) << 32) | info.nFileSizeLow;
            st.id.mtimeNs = FileTimeNs(info.ftLastWriteTime);
            st.id.ctimeNs = FileTimeNs(info.ftCreationTime);
            st.id.inode = (static_cast<uint64_t>(info.nFileIndexHigh) << 32) | info.nFileIndexLow;
            st.id.device = info.dwVolumeSerialNumber;
            return st;
        }
    }
    // Not openable for its attributes: whether it exists (the open then reports the error).
    std::error_code ec;
    const std::filesystem::path p(std::u8string(path.begin(), path.end()));
    const auto status = std::filesystem::status(p, ec);
    if (ec || !std::filesystem::exists(status)) return st;
    st.exists = true;
    st.regular = std::filesystem::is_regular_file(status);
#endif
    return st;
}

// The absolute path with "." / ".." and symbolic links resolved (the path as given when that fails).
std::string CanonicalOf(const std::string& path) {
    std::error_code ec;
    const std::filesystem::path p(std::u8string(path.begin(), path.end()));
    const std::filesystem::path abs = std::filesystem::absolute(p, ec);
    if (ec) return path;
    const std::filesystem::path canon = std::filesystem::weakly_canonical(abs, ec);
    if (ec) return path;
    const std::u8string u = canon.u8string();
    return std::string(u.begin(), u.end());
}

std::string IdentityKeyOf(const std::string& canonicalPath, const FileIdentity& id) {
    return canonicalPath + "|" + std::to_string(id.size) + "|" + std::to_string(id.mtimeNs) + "|" +
           std::to_string(id.ctimeNs) + "|" + std::to_string(id.inode) + "|" + std::to_string(id.device);
}

// The TIFF magic, read through the UTF-8 aware byte source.
bool LooksLikeTiff(const std::string& path) {
    auto src = io::OpenByteSource(path, io::SourceKind::kPread, io::AccessHint::kSequential);
    if (!src.ok() || (*src)->Size() < 4) return false;
    uint8_t m[4] = {0, 0, 0, 0};
    if (!(*src)->Read(0, std::span<uint8_t>(m, 4)).ok()) return false;
    const bool ii = m[0] == 'I' && m[1] == 'I' && (m[2] == 42 || m[2] == 43) && m[3] == 0;
    const bool mm = m[0] == 'M' && m[1] == 'M' && m[2] == 0 && (m[3] == 42 || m[3] == 43);
    return ii || mm;
}

int MaxBands() {
    if (const char* e = std::getenv("DDBT_TILER_MAX_BANDS")) {
        const long v = std::strtol(e, nullptr, 10);
        if (v > 0 && v <= 65535) return static_cast<int>(v);
    }
    return CogSource::kDefaultMaxBands;
}

std::optional<PixelType> PixelTypeOf(tiff::SampleType t) {
    switch (t) {
        case tiff::SampleType::kU8: return PixelType::kU8;
        case tiff::SampleType::kI8: return PixelType::kI8;
        case tiff::SampleType::kU16: return PixelType::kU16;
        case tiff::SampleType::kI16: return PixelType::kI16;
        case tiff::SampleType::kU32: return PixelType::kU32;
        case tiff::SampleType::kI32: return PixelType::kI32;
        case tiff::SampleType::kF32: return PixelType::kF32;
        case tiff::SampleType::kF64: return PixelType::kF64;
        case tiff::SampleType::kF16: return PixelType::kF16;
        case tiff::SampleType::kI64: return PixelType::kI64;
        case tiff::SampleType::kU64: return PixelType::kU64;
        case tiff::SampleType::kBit1: return PixelType::kU8;  // GDAL reads 1-bit samples as bytes 0 and 1
        default: return std::nullopt;
    }
}

// The registry ------------------------------------------------------------------------------
//
// Requests look their path up as they spell it (no canonicalisation on the hot path: one stat()
// and one map lookup), each slot checked against the file's current identity. A miss resolves the
// canonical path and shares the live source of that canonical path and identity, if any: spellings
// of one path ("a.tif", "./a.tif", "dir/../a.tif", links) and the files' versions map to one source.

struct Slot {
    std::mutex mutex;
    std::condition_variable cv;
    bool done = false;
    std::shared_ptr<const CogSource> source;
    std::function<void()> throwError;  // a failed open: throws a waiter's own copy of the error
    FileIdentity id;  // immutable once inserted
    uint64_t lastUse = 0;
};

struct Shard {
    std::mutex mutex;
    std::unordered_map<std::string, std::shared_ptr<Slot>> map;
};

struct CanonEntry {
    FileIdentity id;
    std::weak_ptr<const CogSource> source;  // the slots own the sources
};

struct CanonShard {
    std::mutex mutex;
    std::unordered_map<std::string, CanonEntry> map;
};

constexpr size_t kShards = 16;
constexpr size_t kMaxPerShard = 64;  // sources kept per shard (1024 in total)

Shard* Shards() {
    static Shard shards[kShards];
    return shards;
}

CanonShard& CanonShardOf(const std::string& canon) {
    static CanonShard shards[kShards];
    return shards[std::hash<std::string>{}(canon) % kShards];
}

// The live source of (canonical path, identity), or null.
std::shared_ptr<const CogSource> FindCanonical(const std::string& canon, const FileIdentity& id) {
    CanonShard& cs = CanonShardOf(canon);
    std::lock_guard lock(cs.mutex);
    const auto it = cs.map.find(canon);
    if (it == cs.map.end() || !(it->second.id == id)) return nullptr;
    return it->second.source.lock();
}

// Registers `fresh` as the source of (canonical path, identity); returns the live one when another
// spelling registered it first.
std::shared_ptr<const CogSource> ShareCanonical(const std::string& canon, const FileIdentity& id,
                                                std::shared_ptr<const CogSource> fresh) {
    CanonShard& cs = CanonShardOf(canon);
    std::lock_guard lock(cs.mutex);
    auto [it, inserted] = cs.map.try_emplace(canon);
    if (!inserted && it->second.id == id)
        if (auto live = it->second.source.lock()) return live;
    it->second = {id, fresh};
    if (cs.map.size() > 2 * kMaxPerShard)
        std::erase_if(cs.map, [](const auto& e) { return e.second.source.expired(); });
    return fresh;
}

std::atomic<uint64_t> g_useClock{0};

// The NoData value GDAL's GTiff driver reports for the GDAL_NODATA of a band of `type`: the value of a
// 32-bit float band is snapped to the float maximum when it is within 1e-10 of it
// (GDALAdjustNoDataCloseToFloatMax), then rounded to a float
std::optional<double> GdalNodataOf(std::optional<double> nodata, PixelType type) {
    if (!nodata || type != PixelType::kF32) return nodata;
    constexpr double kMaxFloat = std::numeric_limits<float>::max();
    double v = *nodata;
    if (std::fabs(v - -kMaxFloat) < 1e-10 * kMaxFloat) v = -kMaxFloat;
    else if (std::fabs(v - kMaxFloat) < 1e-10 * kMaxFloat) v = kMaxFloat;
    return static_cast<double>(static_cast<float>(v));
}

}  // namespace

std::optional<FileIdentity> IdentifyFile(const std::string& path) {
    const FileState st = StatFile(path);
    if (!st.exists || !st.regular) return std::nullopt;
    return st.id;
}

size_t PixelTypeSize(PixelType t) noexcept {
    switch (t) {
        case PixelType::kU8:
        case PixelType::kI8: return 1;
        case PixelType::kU16:
        case PixelType::kI16:
        case PixelType::kF16: return 2;
        case PixelType::kU32:
        case PixelType::kI32:
        case PixelType::kF32: return 4;
        case PixelType::kF64:
        case PixelType::kI64:
        case PixelType::kU64: return 8;
    }
    return 1;
}

bool PixelTypeIsFloat(PixelType t) noexcept {
    return t == PixelType::kF32 || t == PixelType::kF64 || t == PixelType::kF16;
}

const char* PixelTypeName(PixelType t) noexcept {
    switch (t) {
        case PixelType::kU8: return "Byte";
        case PixelType::kI8: return "Int8";
        case PixelType::kU16: return "UInt16";
        case PixelType::kI16: return "Int16";
        case PixelType::kU32: return "UInt32";
        case PixelType::kI32: return "Int32";
        case PixelType::kF32: return "Float32";
        case PixelType::kF64: return "Float64";
        case PixelType::kF16: return "Float16";
        case PixelType::kI64: return "Int64";
        case PixelType::kU64: return "UInt64";
    }
    return "Unknown";
}

CogSource::~CogSource() = default;

const tiff::GdalMetadata& CogSource::Metadata() const {
    std::call_once(metadataOnce_, [this] {
        tiff::GdalDatasetShape shape;
        shape.bandCount = BandCount();
        for (size_t l = 1; l < levels_.size(); ++l) shape.overviewIfds.push_back(levels_[l].ifd);
        metadata_ = tiff::ReadGdalMetadata(*reader_, shape, georef_.keys);
        // DroneDB's collectMatchingMetadata: the description and the software, then every domain
        // but those that name files, then the default domain of each band
        for (const char* item : {"TIFFTAG_IMAGEDESCRIPTION", "TIFFTAG_SOFTWARE"})
            if (const auto v = metadata_.dataset.Item("", item)) datasetMetadata_.push_back(*v);
        for (const std::string& domain : metadata_.DomainList())
            for (const std::string& s : *metadata_.dataset.Items(domain)) datasetMetadata_.push_back(s);
        for (const BandMeta& band : bands_)
            datasetMetadata_.insert(datasetMetadata_.end(), band.metadata.begin(), band.metadata.end());
    });
    return metadata_;
}

const std::vector<std::string>& CogSource::DatasetMetadata() const {
    Metadata();
    return datasetMetadata_;
}

bool CogSource::IsJpeg() const noexcept { return reader_->Ifd(levels_[0].ifd).compression == 7; }

int CogSource::DataBandCount() const noexcept {
    const int n = BandCount();
    if (mask_ == MaskKind::kAlpha || n == 4 || n == 2 || bands_.back().interp == ColorInterp::kAlpha) return n - 1;
    return n;
}

std::shared_ptr<const CogSource> CogSource::Open(const std::string& path, SourceUse use) {
    const FileState st = StatFile(path);
    if (!st.exists) throw TileError(TileErrorCode::kNotFound, path + " does not exist");
    std::shared_ptr<CogSource> s(new CogSource());
    s->Init(path, CanonicalOf(path), st.id, use);
    return s;
}

std::shared_ptr<const CogSource> CogSource::Get(const std::string& path, SourceUse use) {
    const FileState st = StatFile(path);
    if (!st.exists) throw TileError(TileErrorCode::kNotFound, path + " does not exist");
    // A source opened for reading is not the tiler's (it may not be georeferenced, it has no warp):
    // the registries keep them apart by the key
    const std::string key = use == SourceUse::kRead ? std::string("\x01read\x01") + path : path;
    Shard& shard = Shards()[std::hash<std::string>{}(key) % kShards];
    std::shared_ptr<Slot> slot;
    bool leader = false;
    {
        std::lock_guard lock(shard.mutex);
        auto it = shard.map.find(key);
        if (it != shard.map.end() && !(it->second->id == st.id)) {
            shard.map.erase(it);  // the file changed: a new source (requests holding the old one keep it)
            it = shard.map.end();
        }
        if (it == shard.map.end()) {
            if (shard.map.size() >= kMaxPerShard) {
                // Drop the least recently used finished slot.
                auto victim = shard.map.end();
                for (auto j = shard.map.begin(); j != shard.map.end(); ++j) {
                    std::lock_guard sl(j->second->mutex);
                    if (!j->second->done) continue;
                    if (victim == shard.map.end() || j->second->lastUse < victim->second->lastUse) victim = j;
                }
                if (victim != shard.map.end()) shard.map.erase(victim);
            }
            slot = std::make_shared<Slot>();
            slot->id = st.id;
            shard.map.emplace(key, slot);
            leader = true;
        } else {
            slot = it->second;
        }
    }
    if (leader) {
        std::shared_ptr<const CogSource> src;
        std::exception_ptr err;
        try {
            const std::string canon = CanonicalOf(path);
            const std::string canonKey =
                use == SourceUse::kRead ? std::string("\x01read\x01") + canon : canon;
            src = FindCanonical(canonKey, st.id);
            if (!src) {
                std::shared_ptr<CogSource> s(new CogSource());
                s->Init(path, canon, st.id, use);
                src = ShareCanonical(canonKey, st.id, std::move(s));
            }
        } catch (...) {
            err = std::current_exception();
        }
        // The waiters' copies of the error, built on this thread (no exception object shared).
        std::function<void()> throwError = err ? cache::WaiterThrower(err) : std::function<void()>();
        {
            std::lock_guard sl(slot->mutex);
            slot->source = src;
            slot->throwError = std::move(throwError);
            slot->done = true;
            slot->lastUse = g_useClock.fetch_add(1, std::memory_order_relaxed);
        }
        slot->cv.notify_all();
        if (err) {
            // A failure is not kept: the next request tries again.
            std::lock_guard lock(shard.mutex);
            auto it = shard.map.find(key);
            if (it != shard.map.end() && it->second == slot) shard.map.erase(it);
            std::rethrow_exception(err);
        }
        return src;
    }
    std::unique_lock sl(slot->mutex);
    slot->cv.wait(sl, [&] { return slot->done; });
    if (slot->throwError) slot->throwError();
    slot->lastUse = g_useClock.fetch_add(1, std::memory_order_relaxed);
    return slot->source;
}

void CogSource::ClearRegistry() {
    for (size_t i = 0; i < kShards; ++i) {
        std::lock_guard lock(Shards()[i].mutex);
        Shards()[i].map.clear();
    }
    // The canonical index only holds weak references: its entries expire with the slots.
}

void CogSource::Init(const std::string& path, const std::string& canonicalPath, const FileIdentity& identity,
                     SourceUse use) {
    path_ = path;
    use_ = use;
    canonicalPath_ = canonicalPath;
    identity_ = identity;
    identityKey_ = IdentityKeyOf(canonicalPath, identity);
    id_ = g_nextId.fetch_add(1, std::memory_order_relaxed);

    if (!LooksLikeTiff(path))
        throw TileError(TileErrorCode::kNotImplemented,
                        path + " is not a GeoTIFF: ddbt tiles GeoTIFF / COG files, other rasters must be converted to "
                               "a COG first (not implemented yet)");

    // Positional reads by default: one read per block (a cold block is one I/O request, where a
    // mapping faults it in page by page); DDBT_TILER_IO=mmap maps the file instead. The mapping
    // is unsafe for files rewritten in place: a page past a truncation raises SIGBUS (the process
    // dies) where a read returns an error; pread stays the default for that reason too.
    const char* ioKind = std::getenv("DDBT_TILER_IO");
    const io::SourceKind kind =
        ioKind != nullptr && std::strcmp(ioKind, "mmap") == 0 ? io::SourceKind::kMmap : io::SourceKind::kPread;
    auto bytes = io::OpenByteSource(path, kind, io::AccessHint::kRandom);
    if (!bytes.ok()) ThrowIfError(bytes.status(), "Cannot open " + path);
    reader_ = ValueOrThrow(tiff::TiffReader::Open(std::shared_ptr<const io::ByteSource>(std::move(*bytes))),
                           "Cannot open " + path);
    const tiff::TiffReader& r = *reader_;
    const tiff::IfdInfo& main = r.Ifd(r.MainImage());

    // Limits: the tiler computes with int pixel coordinates, and every band costs metadata,
    // statistics and (formulas over NoData) a read per tile.
    if (main.width > static_cast<uint64_t>(INT_MAX) || main.height > static_cast<uint64_t>(INT_MAX))
        throw TileError(TileErrorCode::kException, path + " is " + std::to_string(main.width) + " x " +
                                                       std::to_string(main.height) +
                                                       " pixels: the tiler reads rasters up to 2147483647 pixels a side");
    const int maxBands = MaxBands();
    if (main.samplesPerPixel > maxBands)
        throw TileError(TileErrorCode::kException,
                        path + " has " + std::to_string(main.samplesPerPixel) + " bands: the tiler reads rasters of up to " +
                            std::to_string(maxBands) + " bands (DDBT_TILER_MAX_BANDS)");

    // A complex raster reads as the Float64 magnitudes of its samples (block_cache.cpp)
    const auto type = PixelTypeOf(main.complex ? tiff::SampleType::kF64 : main.sampleType);
    if (!type)
        throw TileError(TileErrorCode::kException, "Unsupported sample type " +
                                                       std::string(tiff::SampleTypeName(main.sampleType)) + " in " +
                                                       path);
    type_ = *type;
    pixelInterleaved_ = main.IsPixelInterleaved();
    ThrowIfError(r.CheckDecodable(main.index), "Cannot read " + path);

    // Pyramid: the main image, then the overviews GDAL would expose (same band layout).
    SourceLevel l0;
    l0.ifd = main.index;
    l0.maskIfd = r.MaskOf(main.index);
    l0.width = main.width;
    l0.height = main.height;
    levels_.push_back(l0);
    for (const uint32_t o : r.Overviews()) {
        const tiff::IfdInfo& ov = r.Ifd(o);
        if (ov.samplesPerPixel != main.samplesPerPixel || ov.sampleType != main.sampleType ||
            ov.planarConfig != main.planarConfig || ov.width > main.width || ov.height > main.height ||
            ov.width == 0 || ov.height == 0)
            continue;
        if (!r.CheckDecodable(o).ok()) continue;
        SourceLevel l;
        l.ifd = o;
        l.maskIfd = r.MaskOf(o);
        l.width = ov.width;
        l.height = ov.height;
        levels_.push_back(l);
    }

    // Bands: colour interpretation as GTiffRasterBand's constructor sets it.
    const int n = main.samplesPerPixel;
    bands_.resize(static_cast<size_t>(n));
    // A single band of 1-bit samples is GDAL's GTiffBitmapBand: a palette band (its table black and white)
    const bool bitmap = main.sampleType == tiff::SampleType::kBit1 && n == 1;
    const bool palette = (main.photometric == tiff::photometric::kPalette && !main.colorMap.empty()) || bitmap;
    const bool rgb = main.photometric == tiff::photometric::kRgb ||
                     (main.photometric == tiff::photometric::kYCbCr && main.compression == 7);
    for (int b = 1; b <= n; ++b) {
        ColorInterp ci = ColorInterp::kUndefined;
        bool lookExtra = false;
        if (palette && b == 1) {
            ci = ColorInterp::kPalette;
        } else if (rgb) {
            if (b == 1) ci = ColorInterp::kRed;
            else if (b == 2) ci = ColorInterp::kGreen;
            else if (b == 3) ci = ColorInterp::kBlue;
            else lookExtra = true;
        } else if (main.photometric == tiff::photometric::kYCbCr) {
            if (b == 1) ci = ColorInterp::kYCbCrY;
            else if (b == 2) ci = ColorInterp::kYCbCrCb;
            else if (b == 3) ci = ColorInterp::kYCbCrCr;
            else lookExtra = true;
        } else if (main.photometric == tiff::photometric::kSeparated) {
            if (b == 1) ci = ColorInterp::kCyan;
            else if (b == 2) ci = ColorInterp::kMagenta;
            else if (b == 3) ci = ColorInterp::kYellow;
            else if (b == 4) ci = ColorInterp::kBlack;
            else lookExtra = true;
        } else if (main.photometric == tiff::photometric::kMinIsBlack && b == 1) {
            ci = ColorInterp::kGray;
        } else {
            lookExtra = true;
        }
        if (lookExtra) {
            const int count = static_cast<int>(main.extraSamples.size());
            const int base = n - count;
            ci = ColorInterp::kUndefined;
            if (count > 0 && b > base && b - base - 1 < count) {
                const uint16_t v = main.extraSamples[static_cast<size_t>(b - base - 1)];
                if (v == tiff::extrasample::kAssociatedAlpha || v == tiff::extrasample::kUnassociatedAlpha)
                    ci = ColorInterp::kAlpha;
            }
        }
        bands_[static_cast<size_t>(b - 1)].interp = ci;
        BandMeta& meta = bands_[static_cast<size_t>(b - 1)];
        meta.nodata = GdalNodataOf(main.noData, type_);
        // GDAL reads the NoData value of a 64-bit integer band from its text as an integer
        if (main.noData && type_ == PixelType::kI64) meta.nodataInt64 = std::strtoll(main.noDataText.c_str(), nullptr, 10);
        if (main.noData && type_ == PixelType::kU64) meta.nodataUInt64 = std::strtoull(main.noDataText.c_str(), nullptr, 10);
    }

    // GDAL_METADATA: band roles (scale, offset, unit type, description, colour interpretation)
    const std::vector<tiff::GdalBandMetadata> bandMetadata = tiff::GdalMetadataBands(main.gdalMetadata, n);
    for (int b = 1; b <= n; ++b) {
        BandMeta& bm = bands_[static_cast<size_t>(b - 1)];
        const tiff::GdalBandRoles& role = bandMetadata[static_cast<size_t>(b - 1)].roles;
        if (const auto* items = bandMetadata[static_cast<size_t>(b - 1)].domains.Items("")) bm.metadata = *items;
        bm.hasScale = bm.hasOffset = role.hasOffsetScale;
        bm.scale = role.scale;
        bm.offset = role.offset;
        bm.unitType = role.unitType;
        bm.description = role.description;
        // GTiffBitmapBand::GetColorInterpretation answers Palette whatever GDAL_METADATA says
        if (role.colorInterp && !bitmap) bm.interp = *role.colorInterp;
    }
    for (int b = 1; b <= n; ++b)
        if (bands_[static_cast<size_t>(b - 1)].interp == ColorInterp::kAlpha && alphaBand_ == 0) alphaBand_ = b;

    // The mask of band 1 (GTiffRasterBand::GetMaskBand, then GDALRasterBand::GetMaskBand).
    if (l0.maskIfd) {
        mask_ = MaskKind::kPerDataset;
    } else if (main.noData) {
        mask_ = MaskKind::kNodata;
    } else if ((n == 2 && bands_[1].interp == ColorInterp::kAlpha) ||
               (n == 4 && bands_[3].interp == ColorInterp::kAlpha)) {
        mask_ = (type_ == PixelType::kU8 || type_ == PixelType::kU16) ? MaskKind::kAlpha : MaskKind::kAllValid;
    } else {
        mask_ = MaskKind::kAllValid;
    }

    // Georeferencing (DroneDB: hasGeoreference, then the projection).
    georef_ = ValueOrThrow(tiff::ReadGeoreferencing(r, path), "Cannot read the georeferencing of " + path);
    if (use == SourceUse::kRead) {
        // A raster read as it is: its georeferencing, whatever it is, is reported, never required
        if (georef_.geoTransform) gt_ = *georef_.geoTransform;
        return;
    }
    if (!georef_.geoTransform || (georef_.geoTransform == std::array<double, 6>{0, 1, 0, 0, 0, 1} &&
                                  georef_.gcps.empty()))
        throw TileError(TileErrorCode::kException, path + " is not georeferenced.");
    gt_ = *georef_.geoTransform;
    if (!georef_.crs)
        throw TileError(TileErrorCode::kException,
                        "Cannot read spatial reference system for " + path + ". Is PROJ available?");
    if (std::abs(gt_[1]) < std::numeric_limits<double>::epsilon() ||
        std::abs(gt_[5]) < std::numeric_limits<double>::epsilon())
        throw TileError(TileErrorCode::kException, "Invalid geotransform: pixel size is zero");

    static const crs::Crs webMercator = crs::Crs::FromEpsg(3857);
    webMercator_ = georef_.crs->Equivalent(webMercator);
    if (!webMercator_) warp_ = WarpModel::Create(*this);
}

}  // namespace ddbt::cog::tiler
