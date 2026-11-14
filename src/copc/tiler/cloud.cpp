// SPDX-License-Identifier: MPL-2.0
#include "copc/tiler/cloud.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <future>
#include <limits>
#include <sstream>
#include <string_view>
#include <vector>

#include "cog/tiler/tile_error.h"
#include "core/io/byte_source.h"
#include "core/io/remote_source.h"
#include "core/tile/compat.h"
#include "core/util/status.h"

#ifndef _WIN32
#include <sys/stat.h>
#endif

namespace ddbt::copc::tiler {

using cog::tiler::TileError;
using cog::tiler::TileErrorCode;

namespace {

// ---------------------------------------------------------------------------------------------
// Minimal JSON scanning for the components of a compound CRS (PROJJSON).

size_t SkipString(std::string_view s, size_t i) {
    // s[i] == '"'
    for (++i; i < s.size(); ++i) {
        if (s[i] == '\\') {
            ++i;
        } else if (s[i] == '"') {
            return i + 1;
        }
    }
    return std::string_view::npos;
}

// End (exclusive) of the JSON value starting at s[i].
size_t SkipValue(std::string_view s, size_t i) {
    if (i >= s.size()) return std::string_view::npos;
    if (s[i] == '"') return SkipString(s, i);
    if (s[i] == '{' || s[i] == '[') {
        int depth = 0;
        while (i < s.size()) {
            const char c = s[i];
            if (c == '"') {
                i = SkipString(s, i);
                if (i == std::string_view::npos) return i;
                continue;
            }
            if (c == '{' || c == '[') ++depth;
            if (c == '}' || c == ']') {
                if (--depth == 0) return i + 1;
            }
            ++i;
        }
        return std::string_view::npos;
    }
    while (i < s.size() && s[i] != ',' && s[i] != '}' && s[i] != ']') ++i;
    return i;
}

size_t SkipSpace(std::string_view s, size_t i) {
    while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) ++i;
    return i;
}

// The n-th element of the array value of the first "key" member found at the top level of the
// object `json` (nullopt when absent).
std::optional<std::string_view> TopLevelArrayElement(std::string_view json, std::string_view key, int n) {
    size_t i = SkipSpace(json, 0);
    if (i >= json.size() || json[i] != '{') return std::nullopt;
    ++i;
    while (true) {
        i = SkipSpace(json, i);
        if (i >= json.size() || json[i] != '"') return std::nullopt;
        const size_t kEnd = SkipString(json, i);
        if (kEnd == std::string_view::npos) return std::nullopt;
        const std::string_view k = json.substr(i + 1, kEnd - i - 2);
        i = SkipSpace(json, kEnd);
        if (i >= json.size() || json[i] != ':') return std::nullopt;
        i = SkipSpace(json, i + 1);
        if (k == key) {
            if (i >= json.size() || json[i] != '[') return std::nullopt;
            ++i;
            for (int e = 0;; ++e) {
                i = SkipSpace(json, i);
                const size_t end = SkipValue(json, i);
                if (end == std::string_view::npos) return std::nullopt;
                if (e == n) return json.substr(i, end - i);
                i = SkipSpace(json, end);
                if (i >= json.size() || json[i] != ',') return std::nullopt;
                ++i;
            }
        }
        const size_t end = SkipValue(json, i);
        if (end == std::string_view::npos) return std::nullopt;
        i = SkipSpace(json, end);
        if (i >= json.size() || json[i] != ',') return std::nullopt;
        ++i;
    }
}

// ---------------------------------------------------------------------------------------------

// The identity of an input when a request comes: a changed file (or remote object) gets a new
// Cloud. A local file is its size and modification time, a URL its size and the version the host's
// remote reader gives.
struct FileStat {
    bool exists = false;
    uint64_t size = 0;
    std::string version;
};

FileStat StatLocal(const std::string& path) {
    FileStat st;
    int64_t mtimeNs = 0;
#ifndef _WIN32
    struct stat s {};
    if (::stat(path.c_str(), &s) != 0 || !S_ISREG(s.st_mode)) return st;
    st.size = static_cast<uint64_t>(s.st_size);
    mtimeNs = static_cast<int64_t>(s.st_mtim.tv_sec) * 1000000000 + s.st_mtim.tv_nsec;
#else
    std::error_code ec;
    const std::filesystem::path p = std::filesystem::u8path(path);
    if (!std::filesystem::is_regular_file(p, ec)) return st;
    st.size = std::filesystem::file_size(p, ec);
    mtimeNs = std::filesystem::last_write_time(p, ec).time_since_epoch().count();
#endif
    st.exists = true;
    st.version = std::to_string(mtimeNs);
    return st;
}

[[noreturn]] void FailOpen(const std::string& path, const Status& s) {
    const TileErrorCode code = s.code() == StatusCode::kNotFound      ? TileErrorCode::kNotFound
                               : s.code() == StatusCode::kUnsupported ? TileErrorCode::kNotImplemented
                                                                      : TileErrorCode::kException;
    throw TileError(code, code == TileErrorCode::kNotFound ? path + " does not exist" : std::string(s.message()));
}

FileStat StatInput(const std::string& path) {
    if (!io::IsRemotePath(path)) {
        FileStat st = StatLocal(path);
        if (!st.exists) throw TileError(TileErrorCode::kNotFound, path + " does not exist");
        return st;
    }
    auto remote = io::StatRemote(path);
    if (!remote.ok()) FailOpen(path, remote.status());
    return FileStat{true, remote->size, "remote:" + remote->version};
}

// The bytes of the input; nullptr when a local file cannot be opened (the caller refuses it as
// before, "Cannot get COPC info").
std::shared_ptr<const io::ByteSource> OpenSource(const std::string& path, uint64_t size) {
    if (io::IsRemotePath(path)) {
        auto source = io::OpenRemoteSource(path, size);
        if (!source.ok()) FailOpen(path, source.status());
        return std::move(source).value();
    }
    auto source = io::OpenByteSource(path);
    if (!source.ok()) return nullptr;
    return std::move(source).value();
}

std::string Box3Wkt(const las::LasHeader& h) {
    std::ostringstream o;
    o.precision(15);
    o << "BOX3D(" << h.min[0] << " " << h.min[1] << " " << h.min[2] << ", " << h.max[0] << " " << h.max[1] << " "
      << h.max[2] << ")";
    return o.str();
}

void Grow(std::optional<Rect>& r, double x, double y) {
    if (!std::isfinite(x) || !std::isfinite(y)) return;
    if (!r) {
        r = Rect{x, y, x, y};
        return;
    }
    r->minX = std::min(r->minX, x);
    r->minY = std::min(r->minY, y);
    r->maxX = std::max(r->maxX, x);
    r->maxY = std::max(r->maxY, y);
}

struct Slot {
    uint64_t size = 0;
    std::string version;
    std::shared_future<std::shared_ptr<const Cloud>> cloud;
};

std::mutex& RegistryMutex() {
    static std::mutex m;
    return m;
}

std::unordered_map<std::string, std::shared_ptr<Slot>>& Registry() {
    static std::unordered_map<std::string, std::shared_ptr<Slot>> r;
    return r;
}

}  // namespace

crs::Crs HorizontalCrs(const crs::Crs& c) {
    if (!c.Valid() || !c.IsCompound()) return c;
    try {
        const std::string json = c.ToProjJson();
        if (const auto first = TopLevelArrayElement(json, "components", 0)) return crs::Crs::FromProjJson(*first);
    } catch (const crs::CrsError&) {
    }
    return c;
}

double VerticalUnitInMeters(const crs::Crs& c) {
    if (!c.Valid() || !c.IsCompound()) return 1.0;
    try {
        const std::string json = c.ToProjJson();
        const auto second = TopLevelArrayElement(json, "components", 1);
        if (!second) return 1.0;
        const size_t at = second->find("\"conversion_factor\"");
        if (at == std::string_view::npos) return 1.0;  // "unit": "metre"
        size_t i = second->find(':', at);
        if (i == std::string_view::npos) return 1.0;
        const std::string num(second->substr(i + 1, 40));
        const double f = std::strtod(num.c_str(), nullptr);
        return f > 0.0 ? f : 1.0;
    } catch (const crs::CrsError&) {
        return 1.0;
    }
}

double PdalOptionDouble(double v) {
    // pdal::Option(name, const double&) -> Utils::toString(value, 15): std::setprecision(15) in
    // the classic locale (%.15g); the stage reads it back with an istringstream.
    if (!std::isfinite(v)) return v;
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.15g", v);
    return std::strtod(buf, nullptr);
}

std::shared_ptr<const Cloud> Cloud::Get(const std::string& path) {
    const FileStat st = StatInput(path);

    std::shared_ptr<Slot> slot;
    std::promise<std::shared_ptr<const Cloud>> promise;
    bool leader = false;
    {
        std::lock_guard lock(RegistryMutex());
        auto& reg = Registry();
        auto it = reg.find(path);
        if (it != reg.end() && it->second->size == st.size && it->second->version == st.version) {
            slot = it->second;
        } else {
            slot = std::make_shared<Slot>();
            slot->size = st.size;
            slot->version = st.version;
            slot->cloud = promise.get_future().share();
            reg[path] = slot;
            leader = true;
        }
    }
    if (!leader) return slot->cloud.get();

    try {
        auto cloud = OpenImpl(path, st.size);
        promise.set_value(cloud);
        return cloud;
    } catch (...) {
        promise.set_exception(std::current_exception());
        std::lock_guard lock(RegistryMutex());
        auto& reg = Registry();
        if (auto it = reg.find(path); it != reg.end() && it->second == slot) reg.erase(it);
        throw;
    }
}

void Cloud::ClearRegistry() {
    std::lock_guard lock(RegistryMutex());
    Registry().clear();
}

std::shared_ptr<const Cloud> Cloud::Open(const std::string& path) {
    return OpenImpl(path, StatInput(path).size);
}

std::shared_ptr<const Cloud> Cloud::OpenImpl(const std::string& path, uint64_t size) {
    std::shared_ptr<Cloud> c(new Cloud());
    c->path_ = path;

    // PointCloudTiler: getCopcInfo (readers.copc's preview), then the WKT, then the footprint.
    auto source = OpenSource(path, size);
    if (!source) throw TileError(TileErrorCode::kInvalidArgs, "Cannot get COPC info for " + path);
    auto file = CopcFile::Open(std::move(source));
    if (!file.ok()) {
        // A remote read can fail for reasons of its own (the network, the server): they are said
        if (io::IsRemotePath(path) && file.status().code() != StatusCode::kUnsupported &&
            file.status().code() != StatusCode::kCorrupt)
            throw TileError(TileErrorCode::kException,
                            "Cannot get COPC info for " + path + ": " + std::string(file.status().message()));
        throw TileError(TileErrorCode::kInvalidArgs, "Cannot get COPC info for " + path);
    }
    c->file_ = std::move(file).value();
    const las::LasHeader& h = c->file_->header();

    auto cr = c->file_->Crs();
    if (!cr.ok() || !cr->Valid()) throw TileError(TileErrorCode::kInvalidArgs, "COPC file has no WKT SRS: " + path);
    c->crs_ = std::move(cr).value();
    c->geocentric_ = c->crs_.IsGeocentric();

    for (int a = 0; a < 3; ++a) {
        c->scaling_.scale[a] = h.scale[static_cast<size_t>(a)];
        c->scaling_.offset[a] = h.offset[static_cast<size_t>(a)];
    }
    c->zMin_ = h.min[2];
    c->zMax_ = h.max[2];
    c->compatRampMin_ = PdalOptionDouble(h.min[2]);
    c->compatRampMax_ = PdalOptionDouble(h.max[2]);
    c->hasColor_ = c->file_->hasRgb();
    if (c->hasColor_) {
        auto c16 = c->file_->Uses16BitColor();
        if (!c16.ok()) throw TileError(TileErrorCode::kException, "Cannot read the colours of " + path + ": " +
                                                                      std::string(c16.status().message()));
        c->color16_ = *c16;
    }

    const crs::Crs merc = crs::Crs::FromEpsg(3857);

    // The baseline's footprint (pointcloud.cpp populatePointCloudFootprint): the horizontal CRS,
    // the min and max corners and the centroid, heights in metres within +-30 km.
    try {
        const crs::Crs horizontal = HorizontalCrs(c->crs_);
        const double zToMeters = VerticalUnitInMeters(c->crs_);
        std::shared_ptr<const crs::Transform> t;
        try {
            t = crs::TransformCache::Global().Get(horizontal, merc);
        } catch (const crs::CrsError&) {
            const std::string& name = horizontal.Name();
            throw std::runtime_error("No coordinate transformation from " +
                                     (name.empty() ? std::string("the CRS of the file") : name) + " to EPSG:3857");
        }
        double geoMinX = h.min[0], geoMinY = h.min[1], geoMaxX = h.max[0], geoMaxY = h.max[1];
        double cx = (h.min[0] + h.max[0]) / 2.0, cy = (h.min[1] + h.max[1]) / 2.0;
        if (!t->TransformPoint(geoMinX, geoMinY) || !t->TransformPoint(geoMaxX, geoMaxY) ||
            !t->TransformPoint(cx, cy))
            throw std::runtime_error("Cannot transform the bounds " + Box3Wkt(h) + " to EPSG:3857");
        const double geoMinZ = h.min[2] * zToMeters;
        const double geoMaxZ = h.max[2] * zToMeters;
        if (geoMinZ < -30000 || geoMaxZ > 30000 || (geoMinX == -90 && geoMaxX == 90)) {
            c->footprintError_ = "its bounds do not map to EPSG:3857";
        } else {
            c->compatFootprint_ = Rect{geoMinX, geoMinY, geoMaxX, geoMaxY};
            const tile::compat::GlobalMercator m(256);
            c->compatMinZoom_ = m.zoomForLength(std::min(geoMaxX - geoMinX, geoMaxY - geoMinY));
        }
    } catch (const std::exception& e) {
        c->footprintError_ = e.what();
    }

    // The two transforms of PointCloudTiler (CoordsTransformer(3857, wkt), (wkt, 3857)). A cloud
    // without a footprint (a CRS without a transformation to EPSG:3857, a site grid) has none
    // either: it opens, and every tile is refused with the reason the footprint gives, as
    // PointCloudTiler's constructor refused it before it made them.
    try {
        c->toMercator_ = crs::TransformCache::Global().Get(c->crs_, merc);
        c->toCloud_ = crs::TransformCache::Global().Get(merc, c->crs_);
    } catch (const crs::CrsError&) {
        if (c->compatFootprint_) throw TileError(TileErrorCode::kException, "Failed to create coordinate transformation");
        c->toMercator_.reset();
        c->toCloud_.reset();
        return c;
    }

    // Densified outline for the default mode.
    constexpr int kSide = 8;
    if (!c->geocentric_) {
        std::vector<double> xs, ys;
        for (int s = 0; s < 4; ++s)
            for (int k = 0; k < kSide; ++k) {
                const double t = static_cast<double>(k) / kSide;
                switch (s) {
                    case 0: xs.push_back(h.min[0] + t * (h.max[0] - h.min[0])); ys.push_back(h.min[1]); break;
                    case 1: xs.push_back(h.max[0]); ys.push_back(h.min[1] + t * (h.max[1] - h.min[1])); break;
                    case 2: xs.push_back(h.max[0] - t * (h.max[0] - h.min[0])); ys.push_back(h.max[1]); break;
                    default: xs.push_back(h.min[0]); ys.push_back(h.max[1] - t * (h.max[1] - h.min[1])); break;
                }
            }
        c->toMercator_->TransformPoints(xs.data(), ys.data(), nullptr, xs.size());
        for (size_t i = 0; i < xs.size(); ++i) Grow(c->footprint_, xs[i], ys[i]);
    } else {
        // The 12 edges of the box, kSide points each (z matters for a geocentric CRS).
        std::vector<double> xs, ys, zs;
        const double lo[3] = {h.min[0], h.min[1], h.min[2]}, hi[3] = {h.max[0], h.max[1], h.max[2]};
        for (int axis = 0; axis < 3; ++axis)
            for (int corner = 0; corner < 4; ++corner)
                for (int k = 0; k <= kSide; ++k) {
                    double p[3];
                    const int o1 = (axis + 1) % 3, o2 = (axis + 2) % 3;
                    p[axis] = lo[axis] + (hi[axis] - lo[axis]) * k / kSide;
                    p[o1] = (corner & 1) ? hi[o1] : lo[o1];
                    p[o2] = (corner & 2) ? hi[o2] : lo[o2];
                    xs.push_back(p[0]);
                    ys.push_back(p[1]);
                    zs.push_back(p[2]);
                }
        c->toMercator_->TransformPoints(xs.data(), ys.data(), zs.data(), xs.size());
        double hmin = std::numeric_limits<double>::infinity(), hmax = -hmin;
        for (size_t i = 0; i < xs.size(); ++i) {
            Grow(c->footprint_, xs[i], ys[i]);
            if (std::isfinite(xs[i]) && std::isfinite(zs[i])) {
                hmin = std::min(hmin, zs[i]);
                hmax = std::max(hmax, zs[i]);
            }
        }
        if (hmin <= hmax) {
            c->heightMin_ = hmin;
            c->heightMax_ = hmax;
        }
    }
    return c;
}

uint32_t Cloud::DroneDbRank(uint64_t nodeId) const {
    std::call_once(rankOnce_, [this] {
        // copc_utils.cpp readCopcIndex: pages from a stack (the root first, a page's links pushed
        // in entry order and popped last-in first-out), entries with points in page order.
        const io::ByteSource& src = file_->source();
        std::vector<std::pair<uint64_t, uint64_t>> pages{{file_->info().rootHierOffset, file_->info().rootHierSize}};
        uint32_t rank = 0;
        while (!pages.empty()) {
            const auto [offset, size] = pages.back();
            pages.pop_back();
            if (size % 32 != 0 || size > (64u << 20)) break;
            auto page = src.ReadVector(offset, static_cast<size_t>(size));
            if (!page.ok()) break;
            const uint8_t* p = page->data();
            for (size_t e = 0; e + 32 <= page->size(); e += 32) {
                uint64_t off;
                int32_t bytes, count;
                std::memcpy(&off, p + e + 16, 8);
                std::memcpy(&bytes, p + e + 24, 4);
                std::memcpy(&count, p + e + 28, 4);
                if (count == -1)
                    pages.emplace_back(off, static_cast<uint64_t>(static_cast<uint32_t>(bytes)));
                else if (count > 0)
                    rank_.emplace(off, rank++);
            }
        }
    });
    const auto it = rank_.find(nodeId);
    return it == rank_.end() ? ~0u : it->second;
}

}  // namespace ddbt::copc::tiler
