// SPDX-License-Identifier: MPL-2.0
#include "cog/tiler/cog_source.h"

#include <algorithm>
#include <climits>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
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

std::string XmlUnescape(std::string_view s) {
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] != '&') {
            out += s[i];
            continue;
        }
        const size_t semi = s.find(';', i);
        if (semi == std::string_view::npos) {
            out += s[i];
            continue;
        }
        const std::string_view ent = s.substr(i + 1, semi - i - 1);
        if (ent == "amp") out += '&';
        else if (ent == "lt") out += '<';
        else if (ent == "gt") out += '>';
        else if (ent == "quot") out += '"';
        else if (ent == "apos") out += '\'';
        else if (!ent.empty() && ent[0] == '#') {
            const long code = ent.size() > 1 && (ent[1] == 'x' || ent[1] == 'X')
                                  ? std::strtol(std::string(ent.substr(2)).c_str(), nullptr, 16)
                                  : std::strtol(std::string(ent.substr(1)).c_str(), nullptr, 10);
            if (code > 0 && code < 0x80) {
                out += static_cast<char>(code);
            } else if (code >= 0x80 && code < 0x800) {
                out += static_cast<char>(0xC0 | (code >> 6));
                out += static_cast<char>(0x80 | (code & 0x3F));
            } else if (code >= 0x800 && code < 0x10000) {
                out += static_cast<char>(0xE0 | (code >> 12));
                out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
                out += static_cast<char>(0x80 | (code & 0x3F));
            }
        } else {
            out.append(s.substr(i, semi - i + 1));
        }
        i = semi;
    }
    return out;
}

std::string AttrOf(std::string_view tag, std::string_view name) {
    size_t pos = 0;
    while ((pos = tag.find(name, pos)) != std::string_view::npos) {
        const bool boundary = pos == 0 || tag[pos - 1] == ' ' || tag[pos - 1] == '\t' || tag[pos - 1] == '\n';
        size_t p = pos + name.size();
        while (p < tag.size() && (tag[p] == ' ')) ++p;
        if (boundary && p < tag.size() && tag[p] == '=') {
            ++p;
            while (p < tag.size() && tag[p] == ' ') ++p;
            if (p < tag.size() && (tag[p] == '"' || tag[p] == '\'')) {
                const char q = tag[p];
                const size_t end = tag.find(q, p + 1);
                if (end != std::string_view::npos) return XmlUnescape(tag.substr(p + 1, end - p - 1));
            }
        }
        pos += name.size();
    }
    return {};
}

ColorInterp ColorInterpFromName(std::string_view n) {
    std::string s(n);
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (s == "gray" || s == "grey") return ColorInterp::kGray;
    if (s == "palette") return ColorInterp::kPalette;
    if (s == "red") return ColorInterp::kRed;
    if (s == "green") return ColorInterp::kGreen;
    if (s == "blue") return ColorInterp::kBlue;
    if (s == "alpha") return ColorInterp::kAlpha;
    if (s == "undefined") return ColorInterp::kUndefined;
    return ColorInterp::kOther;
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

}  // namespace

const char* ColorInterpName(ColorInterp ci) noexcept {
    switch (ci) {
        case ColorInterp::kUndefined: return "Undefined";
        case ColorInterp::kGray: return "Gray";
        case ColorInterp::kPalette: return "Palette";
        case ColorInterp::kRed: return "Red";
        case ColorInterp::kGreen: return "Green";
        case ColorInterp::kBlue: return "Blue";
        case ColorInterp::kAlpha: return "Alpha";
        case ColorInterp::kYCbCrY: return "YCbCr_Y";
        case ColorInterp::kYCbCrCb: return "YCbCr_Cb";
        case ColorInterp::kYCbCrCr: return "YCbCr_Cr";
        case ColorInterp::kCyan: return "Cyan";
        case ColorInterp::kMagenta: return "Magenta";
        case ColorInterp::kYellow: return "Yellow";
        case ColorInterp::kBlack: return "Black";
        case ColorInterp::kOther: return "Other";
    }
    return "Undefined";
}

size_t PixelTypeSize(PixelType t) noexcept {
    switch (t) {
        case PixelType::kU8:
        case PixelType::kI8: return 1;
        case PixelType::kU16:
        case PixelType::kI16: return 2;
        case PixelType::kU32:
        case PixelType::kI32:
        case PixelType::kF32: return 4;
        case PixelType::kF64: return 8;
    }
    return 1;
}

bool PixelTypeIsFloat(PixelType t) noexcept { return t == PixelType::kF32 || t == PixelType::kF64; }

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
    }
    return "Unknown";
}

std::vector<GdalMetadataItem> ParseGdalMetadata(std::string_view xml) {
    std::vector<GdalMetadataItem> items;
    size_t pos = 0;
    while ((pos = xml.find("<Item", pos)) != std::string_view::npos) {
        const size_t tagEnd = xml.find('>', pos);
        if (tagEnd == std::string_view::npos) break;
        const std::string_view tag = xml.substr(pos + 5, tagEnd - pos - 5);
        GdalMetadataItem it;
        it.name = AttrOf(tag, "name");
        const std::string sample = AttrOf(tag, "sample");
        it.sample = sample.empty() ? -1 : std::atoi(sample.c_str());
        it.role = AttrOf(tag, "role");
        it.domain = AttrOf(tag, "domain");
        if (!tag.empty() && tag.back() == '/') {
            pos = tagEnd + 1;
        } else {
            const size_t close = xml.find("</Item>", tagEnd);
            if (close == std::string_view::npos) break;
            it.value = XmlUnescape(xml.substr(tagEnd + 1, close - tagEnd - 1));
            pos = close + 7;
        }
        items.push_back(std::move(it));
    }
    return items;
}

CogSource::~CogSource() = default;

bool CogSource::IsJpeg() const noexcept { return reader_->Ifd(levels_[0].ifd).compression == 7; }

int CogSource::DataBandCount() const noexcept {
    const int n = BandCount();
    if (mask_ == MaskKind::kAlpha || n == 4 || n == 2 || bands_.back().interp == ColorInterp::kAlpha) return n - 1;
    return n;
}

std::shared_ptr<const CogSource> CogSource::Open(const std::string& path) {
    const FileState st = StatFile(path);
    if (!st.exists) throw TileError(TileErrorCode::kNotFound, path + " does not exist");
    std::shared_ptr<CogSource> s(new CogSource());
    s->Init(path, CanonicalOf(path), st.id);
    return s;
}

std::shared_ptr<const CogSource> CogSource::Get(const std::string& path) {
    const FileState st = StatFile(path);
    if (!st.exists) throw TileError(TileErrorCode::kNotFound, path + " does not exist");
    Shard& shard = Shards()[std::hash<std::string>{}(path) % kShards];
    std::shared_ptr<Slot> slot;
    bool leader = false;
    {
        std::lock_guard lock(shard.mutex);
        auto it = shard.map.find(path);
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
            shard.map.emplace(path, slot);
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
            src = FindCanonical(canon, st.id);
            if (!src) {
                std::shared_ptr<CogSource> s(new CogSource());
                s->Init(path, canon, st.id);
                src = ShareCanonical(canon, st.id, std::move(s));
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
            auto it = shard.map.find(path);
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

void CogSource::Init(const std::string& path, const std::string& canonicalPath, const FileIdentity& identity) {
    path_ = path;
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
    const bool palette = main.photometric == tiff::photometric::kPalette && !main.colorMap.empty();
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
        bands_[static_cast<size_t>(b - 1)].nodata = main.noData;
    }

    // GDAL_METADATA: band roles (scale, offset, unit type, description, colour interpretation),
    // band items and dataset items.
    std::vector<std::string> datasetItems;
    std::vector<std::pair<std::string, std::string>> domainItems;  // domain, "K=V"
    for (const auto& it : ParseGdalMetadata(main.gdalMetadata)) {
        if (it.sample >= 0 && it.sample < n) {
            BandMeta& bm = bands_[static_cast<size_t>(it.sample)];
            if (it.role == "scale") {
                bm.hasScale = true;
                bm.scale = std::atof(it.value.c_str());
            } else if (it.role == "offset") {
                bm.hasOffset = true;
                bm.offset = std::atof(it.value.c_str());
            } else if (it.role == "unittype") {
                bm.unitType = it.value;
            } else if (it.role == "description") {
                bm.description = it.value;
            } else if (it.role == "colorinterp") {
                bm.interp = ColorInterpFromName(it.value);
            } else if (it.domain.empty()) {
                bm.metadata.push_back(it.name + "=" + it.value);
            }
        } else if (it.sample < 0) {
            if (it.domain.empty()) datasetItems.push_back(it.name + "=" + it.value);
            else domainItems.emplace_back(it.domain, it.name + "=" + it.value);
        }
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

    // Metadata strings for the sensor profiles (the TIFF tags GDAL exposes, GDAL_METADATA
    // items, the IMAGE_STRUCTURE domain, band items).
    if (!main.imageDescription.empty()) datasetMetadata_.push_back(main.imageDescription);
    if (!main.software.empty()) datasetMetadata_.push_back(main.software);
    if (!main.imageDescription.empty()) datasetMetadata_.push_back("TIFFTAG_IMAGEDESCRIPTION=" + main.imageDescription);
    if (!main.software.empty()) datasetMetadata_.push_back("TIFFTAG_SOFTWARE=" + main.software);
    if (const auto* dt = main.FindTag(tiff::tag::kDateTime)) {
        auto s = r.ReadTagString(*dt, 4096);
        if (s.ok()) datasetMetadata_.push_back("TIFFTAG_DATETIME=" + *s);
    }
    for (auto& s : datasetItems) datasetMetadata_.push_back(std::move(s));
    for (auto& [d, s] : domainItems) datasetMetadata_.push_back(std::move(s));
    for (const auto& bm : bands_)
        for (const auto& s : bm.metadata) datasetMetadata_.push_back(s);

    // Georeferencing (DroneDB: hasGeoreference, then the projection).
    georef_ = ValueOrThrow(tiff::ReadGeoreferencing(r, path), "Cannot read the georeferencing of " + path);
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
