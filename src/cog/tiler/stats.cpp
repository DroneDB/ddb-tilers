// SPDX-License-Identifier: MPL-2.0
// The arithmetic is in stats_kernel.cpp (compiled without floating-point contraction).
#include "cog/tiler/stats.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <limits>
#include <list>
#include <map>
#include <mutex>
#include <random>
#include <sstream>
#include <thread>
#include <unordered_map>

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

#include "cog/tiler/cog_source.h"
#include "cog/tiler/json_lite.h"
#include "cog/tiler/stats_kernel.h"
#include "core/exec/parallel_for.h"
#include "core/exec/task_group.h"

namespace ddbt::cog::tiler {

namespace {

std::atomic<uint64_t> g_fromSidecar{0}, g_computed{0};

struct PassEntry {
    std::mutex mutex;                // held while the pass runs (waiters block on it)
    std::atomic<bool> done{false};   // bands is complete and immutable from then on
    std::vector<std::optional<BandStatistics>> bands;  // nullopt: no pixel with a value
};

// Per-thread memo of the entries a thread used last: the hot path (every non-Byte tile asks for
// its statistics) takes no shared lock once the statistics are known.
struct ThreadMemo {
    std::vector<std::pair<std::string, std::shared_ptr<PassEntry>>> entries;
    uint64_t generation = 0;
};
std::atomic<uint64_t> g_generation{1};

// The process-wide statistics: an LRU of files (by CogSource::IdentityKey: canonical path and
// file identity, so every source object of one file version shares them), each with an LRU of
// its NoData keys ("" for the band's own NoData, else the override), each with its passes (one per
// file for a pixel-interleaved file, one per band otherwise; at most the band count).
struct NodataPasses {
    std::string nodataKey;
    std::map<int, std::shared_ptr<PassEntry>> passes;  // by band (0: every band of the file)
};

struct FileStats {
    std::string identity;
    std::list<NodataPasses> byNodata;  // most recently used first, at most kMaxStatsNodataKeysPerFile
};

std::mutex g_mapMutex;
std::list<FileStats> g_files;  // most recently used first, at most kMaxStatsFiles
std::unordered_map<std::string, std::list<FileStats>::iterator> g_fileIndex;

// The pass entry of (file, NoData key, band), created when missing; the LRUs updated.
std::shared_ptr<PassEntry> PassEntryOf(const std::string& identity, const std::string& nodataKey, int band) {
    std::lock_guard lock(g_mapMutex);
    auto fit = g_fileIndex.find(identity);
    if (fit == g_fileIndex.end()) {
        g_files.push_front(FileStats{identity, {}});
        fit = g_fileIndex.emplace(identity, g_files.begin()).first;
        if (g_files.size() > kMaxStatsFiles) {
            g_fileIndex.erase(g_files.back().identity);
            g_files.pop_back();
        }
    } else if (fit->second != g_files.begin()) {
        g_files.splice(g_files.begin(), g_files, fit->second);
    }
    std::list<NodataPasses>& byNodata = fit->second->byNodata;
    auto nit = std::find_if(byNodata.begin(), byNodata.end(),
                            [&](const NodataPasses& n) { return n.nodataKey == nodataKey; });
    if (nit == byNodata.end()) {
        byNodata.push_front(NodataPasses{nodataKey, {}});
        nit = byNodata.begin();
        if (byNodata.size() > kMaxStatsNodataKeysPerFile) byNodata.pop_back();
    } else if (nit != byNodata.begin()) {
        byNodata.splice(byNodata.begin(), byNodata, nit);
    }
    auto& e = nit->passes[band];
    if (!e) e = std::make_shared<PassEntry>();
    return e;
}

namespace fs = std::filesystem;

fs::path U8Path(const std::string& s) { return fs::path(std::u8string(s.begin(), s.end())); }

fs::path SidecarOf(const std::string& raster) { return U8Path(raster + ".stats.json"); }

// readFreshSidecar: exists, not older than the raster, parses, "exact": true.
std::optional<json::Value> ReadFreshSidecar(const std::string& raster) {
    std::error_code ec;
    const fs::path sidecar = SidecarOf(raster);
    const auto st = fs::last_write_time(sidecar, ec);
    if (ec) return std::nullopt;
    const auto rt = fs::last_write_time(U8Path(raster), ec);
    if (ec || st < rt) return std::nullopt;
    std::ifstream in(sidecar, std::ios::binary);
    if (!in) return std::nullopt;
    std::stringstream ss;
    ss << in.rdbuf();
    auto v = json::Parse(ss.str());
    if (!v || !v->IsObject()) return std::nullopt;
    const json::Value* exact = v->Find("exact");
    if (exact == nullptr || exact->kind != json::Value::Kind::kBool || !exact->b) return std::nullopt;
    return v;
}

std::optional<BandStatistics> FromSidecar(const json::Value& j, int band, const std::optional<double>& nodata) {
    const json::Value* bands = nullptr;
    if (!nodata) {
        bands = j.Find("bands");
    } else if (const json::Value* o = j.Find("overrides")) {
        bands = o->Find(NodataKey(nodata));
    }
    if (bands == nullptr) return std::nullopt;
    const json::Value* b = bands->Find(std::to_string(band));
    if (b == nullptr || !b->IsObject()) return std::nullopt;
    const json::Value *mn = b->Find("min"), *mx = b->Find("max"), *me = b->Find("mean"), *sd = b->Find("std");
    if (!mn || !mx || !me || !sd || !mn->IsNumber() || !mx->IsNumber() || !me->IsNumber() || !sd->IsNumber())
        return std::nullopt;
    BandStatistics s;
    s.min = mn->num;
    s.max = mx->num;
    s.mean = me->num;
    s.stdDev = sd->num;
    if (const json::Value* v = b->Find("valid"); v && v->isUint) s.validCount = v->u;
    if (const json::Value* v = b->Find("total"); v && v->isUint) s.pixelCount = v->u;
    return s;
}

json::Value StatsToJson(const BandStatistics& s) {
    json::Value o = json::Object();
    o.obj["min"] = json::Number(s.min);
    o.obj["max"] = json::Number(s.max);
    o.obj["mean"] = json::Number(s.mean);
    o.obj["std"] = json::Number(s.stdDev);
    o.obj["valid"] = json::Unsigned(s.validCount);
    o.obj["total"] = json::Unsigned(s.pixelCount);
    return o;
}

void WriteSidecar(const std::string& raster, int band, const BandStatistics& s, const std::optional<double>& nodata) {
    try {
        json::Value j = ReadFreshSidecar(raster).value_or(json::Object());
        if (!nodata) {
            json::Value& b = j.obj["bands"];
            b.kind = json::Value::Kind::kObject;
            b.obj[std::to_string(band)] = StatsToJson(s);
        } else {
            json::Value& o = j.obj["overrides"];
            o.kind = json::Value::Kind::kObject;
            json::Value& k = o.obj[NodataKey(nodata)];
            k.kind = json::Value::Kind::kObject;
            k.obj[std::to_string(band)] = StatsToJson(s);
        }
        j.obj["version"] = json::Unsigned(2);
        j.obj["exact"] = json::Bool(true);
        const std::time_t t = std::time(nullptr);
        std::tm tm{};
#ifdef _WIN32
        gmtime_s(&tm, &t);
#else
        gmtime_r(&t, &tm);
#endif
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%FT%TZ", &tm);
        j.obj["computedAt"] = json::String(stamp);
        // Written to a temporary of this process and thread (pid + random suffix), renamed over the
        // sidecar: readers see the old or the new file, never a partial one. Not locked across
        // writers: two processes adding entries at once may drop one of them (the last rename
        // wins); a dropped entry is computed again by the next request that misses it.
        const fs::path target = SidecarOf(raster);
        thread_local std::mt19937_64 rng([] {
            std::random_device rd;
            return (static_cast<uint64_t>(rd()) << 32) ^ rd() ^
                   std::hash<std::thread::id>{}(std::this_thread::get_id()) ^
                   static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
        }());
#ifdef _WIN32
        const long long pid = _getpid();
#else
        const long long pid = ::getpid();
#endif
        char suffix[64];
        std::snprintf(suffix, sizeof(suffix), ".%lld.%016llx.tmp", pid, static_cast<unsigned long long>(rng()));
        fs::path temp = target;
        temp += suffix;
        {
            std::ofstream out(temp, std::ios::trunc | std::ios::binary);
            out << json::Dump(j, 2);
            if (!out) return;
        }
        std::error_code ec;
        fs::rename(temp, target, ec);
        if (ec) fs::remove(temp, ec);
    } catch (...) {
        // DroneDB logs and goes on: a sidecar that cannot be written is not an error of the tile.
    }
}

// The mask of band b of the file used by the statistics (BandAccumulator): per dataset or alpha.
enum class StatsMask { kNone, kInternal, kAlphaBand };

StatsMask MaskOfBand(const CogSource& src, int b) {
    if (src.Levels()[0].maskIfd) return StatsMask::kInternal;
    if (src.Band(b).nodata) return StatsMask::kNone;  // GMF_NODATA
    const int n = src.BandCount();
    const bool alphaType = src.Type() == PixelType::kU8 || src.Type() == PixelType::kU16;
    if (alphaType && n == 2 && b == 1 && src.Band(2).interp == ColorInterp::kAlpha) return StatsMask::kAlphaBand;
    if (alphaType && n == 4 && b <= 3 && src.Band(4).interp == ColorInterp::kAlpha) return StatsMask::kAlphaBand;
    return StatsMask::kNone;
}

tiff::SampleType ToSampleType(PixelType t) {
    switch (t) {
        case PixelType::kU8: return tiff::SampleType::kU8;
        case PixelType::kI8: return tiff::SampleType::kI8;
        case PixelType::kU16: return tiff::SampleType::kU16;
        case PixelType::kI16: return tiff::SampleType::kI16;
        case PixelType::kU32: return tiff::SampleType::kU32;
        case PixelType::kI32: return tiff::SampleType::kI32;
        case PixelType::kF32: return tiff::SampleType::kF32;
        case PixelType::kF64: return tiff::SampleType::kF64;
    }
    return tiff::SampleType::kUnknown;
}

// What one pass reads: the full-resolution image of the bands of `want` (1-based), the planes to
// decode, the internal mask.
struct PassPlan {
    const tiff::TiffReader* r = nullptr;
    uint32_t ifd = 0;
    const tiff::IfdInfo* info = nullptr;
    size_t ss = 1;
    int W = 0, H = 0;
    std::vector<int> want;
    std::vector<stats::SampleFilter> filters;
    std::vector<StatsMask> masks;
    bool contig = true;
    std::vector<uint32_t> planes;  // planes decoded per block (contig: plane 0 only)
    int alphaBand = 0;             // 1-based
    std::optional<uint32_t> maskIfd;
    const tiff::IfdInfo* maskInfo = nullptr;

    size_t Bands() const { return want.size(); }
};

// The decoded blocks of block columns [bc0, bc1) of block row `br` (every plane of the plan), and
// the internal mask of their pixels as bytes (rows x (x1 - x0), 0 = no value). Failures go to
// `error` (the first in a fixed order).
struct Region {
    uint32_t br = 0, bc0 = 0, bc1 = 0;
    int x0 = 0, x1 = 0, y0 = 0, rows = 0;
    std::vector<tiff::DecodedBlock> blocks;  // [plane][bc - bc0]
    std::vector<uint8_t> mask;
    std::string error;

    const tiff::DecodedBlock& Block(size_t plane, uint32_t bc) const {
        return blocks[plane * static_cast<size_t>(bc1 - bc0) + (bc - bc0)];
    }
};

void SetupRegion(const PassPlan& pp, uint32_t br, uint32_t bc0, uint32_t bc1, Region& g) {
    const tiff::IfdInfo& info = *pp.info;
    g.br = br;
    g.bc0 = bc0;
    g.bc1 = bc1;
    g.x0 = static_cast<int>(bc0 * info.blockWidth);
    g.x1 = std::min(pp.W, static_cast<int>(bc1 * info.blockWidth));
    g.y0 = static_cast<int>(br * info.blockHeight);
    const int blockRows = static_cast<int>(info.BlockRows(info.BlockIndex(0, br)));
    g.rows = std::max(0, std::min(blockRows, pp.H - g.y0));
    g.blocks.clear();
    g.blocks.resize(pp.planes.size() * (bc1 - bc0));
    g.error.clear();
    if (pp.maskInfo != nullptr) g.mask.assign(static_cast<size_t>(g.rows) * static_cast<size_t>(g.x1 - g.x0), 0);
}

// Image block k of the region (k < planes * columns).
std::string DecodeRegionBlock(const PassPlan& pp, Region& g, size_t k) {
    const uint32_t cols = g.bc1 - g.bc0;
    const uint32_t bc = g.bc0 + static_cast<uint32_t>(k % cols);
    const uint32_t plane = pp.planes[k / cols];
    auto d = pp.r->ReadBlock(pp.ifd, pp.info->BlockIndex(bc, g.br, plane));
    if (!d.ok()) return "Cannot read the bands to compute their statistics: " + std::string(d.status().message());
    g.blocks[k] = std::move(*d);
    return {};
}

// The mask blocks under the region: mask block column mc of every mask block row the rows cross,
// expanded into g.mask (a sparse mask block leaves its pixels without a value).
std::pair<uint32_t, uint32_t> MaskColumns(const PassPlan& pp, const Region& g) {
    const uint32_t mw = pp.maskInfo->blockWidth;
    return {static_cast<uint32_t>(g.x0) / mw, (static_cast<uint32_t>(g.x1) + mw - 1) / mw};
}

std::string DecodeRegionMaskColumn(const PassPlan& pp, Region& g, uint32_t mc) {
    const tiff::IfdInfo& mi = *pp.maskInfo;
    const int mw = static_cast<int>(mi.blockWidth), mh = static_cast<int>(mi.blockHeight);
    const int rw = g.x1 - g.x0;
    const int cx0 = std::max(g.x0, static_cast<int>(mc) * mw), cx1 = std::min(g.x1, (static_cast<int>(mc) + 1) * mw);
    std::vector<uint8_t> row(static_cast<size_t>(mw));
    for (int y = g.y0; y < g.y0 + g.rows;) {
        const int mbr = y / mh;
        const int yEnd = std::min(g.y0 + g.rows, (mbr + 1) * mh);
        auto d = pp.r->ReadBlock(*pp.maskIfd, mi.BlockIndex(mc, static_cast<uint32_t>(mbr)));
        if (!d.ok()) return "Cannot read the mask of a band to compute its statistics";
        if (!d->sparse) {
            const int bx0 = static_cast<int>(mc) * mw;
            const int vw = std::min(mw, pp.W - bx0);
            for (; y < yEnd; ++y) {
                const int my = y - mbr * mh;
                tiff::ExpandBits(d->data() + static_cast<size_t>(my) * d->rowStride, d->rowStride,
                                 static_cast<uint32_t>(vw), 1, 255, row.data(), static_cast<size_t>(vw));
                std::memcpy(g.mask.data() + static_cast<size_t>(y - g.y0) * static_cast<size_t>(rw) + (cx0 - g.x0),
                            row.data() + (cx0 - bx0), static_cast<size_t>(cx1 - cx0));
            }
        }
        y = yEnd;
    }
    return {};
}

// Decodes a region, its blocks in parallel.
void DecodeRegion(const PassPlan& pp, Region& g) {
    const size_t nBlocks = g.blocks.size();
    const auto [mc0, mc1] = pp.maskInfo ? MaskColumns(pp, g) : std::pair<uint32_t, uint32_t>{0, 0};
    const size_t nTasks = nBlocks + (mc1 - mc0);
    std::vector<std::string> errors(nTasks);
    exec::ParallelFor(0, nTasks, 1, [&](size_t k) {
        errors[k] = k < nBlocks ? DecodeRegionBlock(pp, g, k)
                                : DecodeRegionMaskColumn(pp, g, mc0 + static_cast<uint32_t>(k - nBlocks));
    });
    for (auto& e : errors)
        if (!e.empty()) {
            g.error = std::move(e);
            return;
        }
}

// Feeds row ry of the region, block column by block column, for band i of the plan.
template <typename Acc>
void AddRegionRow(const PassPlan& pp, const Region& g, size_t i, int ry, Acc& acc) {
    const tiff::IfdInfo& info = *pp.info;
    const size_t ss = pp.ss;
    const int rw = g.x1 - g.x0;
    for (uint32_t bc = g.bc0; bc < g.bc1; ++bc) {
        const int x0 = static_cast<int>(bc * info.blockWidth);
        const int vw = std::min(static_cast<int>(info.blockWidth), pp.W - x0);
        const size_t plane = pp.contig ? 0 : i;
        const tiff::DecodedBlock& blk = g.Block(plane, bc);
        const size_t step = pp.contig ? static_cast<size_t>(blk.samples) * ss : ss;
        const uint8_t* p = blk.data() + static_cast<size_t>(ry) * blk.rowStride +
                           (pp.contig ? static_cast<size_t>(pp.want[i] - 1) * ss : 0);
        const uint8_t* alpha = nullptr;
        if (pp.masks[i] == StatsMask::kAlphaBand) {
            const tiff::DecodedBlock& ab = pp.contig ? blk : g.Block(pp.planes.size() - 1, bc);
            alpha = ab.data() + static_cast<size_t>(ry) * ab.rowStride +
                    (pp.contig ? static_cast<size_t>(pp.alphaBand - 1) * ss : 0);
        }
        const uint8_t* m = pp.masks[i] == StatsMask::kInternal
                               ? g.mask.data() + static_cast<size_t>(ry) * static_cast<size_t>(rw) + (x0 - g.x0)
                               : nullptr;
        stats::AddRun(acc, pp.filters[i], p, step, static_cast<uint32_t>(vw), m, alpha);
    }
}

// DroneDB's order and arithmetic: chunks of whole block rows decoded in parallel, the next one
// while the current one is accumulated; one serial Welford per band (bands in parallel).
std::vector<std::optional<stats::StatsResult>> CompatPass(const PassPlan& pp) {
    const tiff::IfdInfo& info = *pp.info;
    const size_t nb = pp.Bands();
    std::vector<stats::Welford> acc(nb);
    const uint32_t bd = info.blocksDown, ba = info.blocksAcross;
    Region cur, next;
    SetupRegion(pp, 0, 0, ba, cur);
    DecodeRegion(pp, cur);
    if (!cur.error.empty()) Fail(cur.error);
    for (uint32_t br = 0; br < bd; ++br) {
        const bool hasNext = br + 1 < bd;
        exec::TaskGroup g;
        if (hasNext) {
            SetupRegion(pp, br + 1, 0, ba, next);
            g.Spawn([&] { DecodeRegion(pp, next); });
        }
        exec::ParallelFor(0, nb, 1, [&](size_t i) {
            for (int ry = 0; ry < cur.rows; ++ry) AddRegionRow(pp, cur, i, ry, acc[i]);
        });
        g.Wait();
        if (hasNext) {
            if (!next.error.empty()) Fail(next.error);
            std::swap(cur, next);
        }
    }
    std::vector<std::optional<stats::StatsResult>> out(nb);
    for (size_t i = 0; i < nb; ++i) out[i] = stats::Finish(acc[i]);
    return out;
}

// Exact sums: every block (all its planes, its mask) is a chunk accumulated on any thread; chunks
// are merged in block order, in batches of a fixed size (independent of the thread count).
std::vector<std::optional<stats::StatsResult>> ExactPass(const PassPlan& pp) {
    const tiff::IfdInfo& info = *pp.info;
    const size_t nb = pp.Bands();
    const size_t units = static_cast<size_t>(info.blocksAcross) * info.blocksDown;
    constexpr size_t kBatch = 1024;
    std::vector<stats::ExactSums> total(nb);
    std::vector<stats::ExactSums> sums;
    std::vector<std::string> errors;
    for (size_t b0 = 0; b0 < units; b0 += kBatch) {
        const size_t b1 = std::min(units, b0 + kBatch);
        sums.assign((b1 - b0) * nb, stats::ExactSums{});
        errors.assign(b1 - b0, std::string());
        exec::ParallelFor(b0, b1, 1, [&](size_t u) {
            const uint32_t br = static_cast<uint32_t>(u / info.blocksAcross);
            const uint32_t bc = static_cast<uint32_t>(u % info.blocksAcross);
            Region g;
            SetupRegion(pp, br, bc, bc + 1, g);
            for (size_t k = 0; k < g.blocks.size() && g.error.empty(); ++k) g.error = DecodeRegionBlock(pp, g, k);
            if (g.error.empty() && pp.maskInfo != nullptr) {
                const auto [mc0, mc1] = MaskColumns(pp, g);
                for (uint32_t mc = mc0; mc < mc1 && g.error.empty(); ++mc) g.error = DecodeRegionMaskColumn(pp, g, mc);
            }
            if (!g.error.empty()) {
                errors[u - b0] = std::move(g.error);
                return;
            }
            for (size_t i = 0; i < nb; ++i)
                for (int ry = 0; ry < g.rows; ++ry) AddRegionRow(pp, g, i, ry, sums[(u - b0) * nb + i]);
        });
        for (const auto& e : errors)
            if (!e.empty()) Fail(e);
        for (size_t u = 0; u < b1 - b0; ++u)
            for (size_t i = 0; i < nb; ++i) stats::Merge(total[i], sums[u * nb + i]);
    }
    std::vector<std::optional<stats::StatsResult>> out(nb);
    for (size_t i = 0; i < nb; ++i) out[i] = stats::Finish(total[i]);
    return out;
}

// One exact pass over the full-resolution image for the bands of `want` (1-based).
std::vector<std::optional<BandStatistics>> ComputePass(const CogSource& src, const std::vector<int>& want,
                                                       const std::optional<double>& nodata, stats::StatsMode mode) {
    PassPlan pp;
    pp.r = &src.Reader();
    const SourceLevel& l0 = src.Levels()[0];
    pp.ifd = l0.ifd;
    pp.info = &pp.r->Ifd(l0.ifd);
    const tiff::SampleType st = ToSampleType(src.Type());
    pp.ss = PixelTypeSize(src.Type());
    pp.W = static_cast<int>(pp.info->width);
    pp.H = static_cast<int>(pp.info->height);
    pp.want = want;
    bool needInternal = false, needAlpha = false;
    for (const int b : want) {
        const std::optional<double> nd = nodata ? nodata : src.Band(b).nodata;
        pp.filters.push_back(stats::SampleFilter{st, nd.has_value(), nd.value_or(0)});
        pp.masks.push_back(MaskOfBand(src, b));
        needInternal |= pp.masks.back() == StatsMask::kInternal;
        needAlpha |= pp.masks.back() == StatsMask::kAlphaBand;
    }
    pp.alphaBand = src.BandCount() == 2 ? 2 : 4;
    pp.contig = pp.info->IsPixelInterleaved();
    if (pp.contig) {
        pp.planes.push_back(0);
    } else {
        for (const int b : want) pp.planes.push_back(static_cast<uint32_t>(b - 1));
        if (needAlpha) pp.planes.push_back(static_cast<uint32_t>(pp.alphaBand - 1));
    }
    if (needInternal) {
        pp.maskIfd = l0.maskIfd;
        pp.maskInfo = &pp.r->Ifd(*l0.maskIfd);
    }
    const auto results = mode == stats::StatsMode::kExact ? ExactPass(pp) : CompatPass(pp);
    std::vector<std::optional<BandStatistics>> out(want.size());
    const uint64_t total = static_cast<uint64_t>(pp.W) * static_cast<uint64_t>(pp.H);
    for (size_t i = 0; i < want.size(); ++i) {
        if (!results[i]) continue;
        BandStatistics s;
        s.min = results[i]->min;
        s.max = results[i]->max;
        s.mean = results[i]->mean;
        s.stdDev = results[i]->stdDev;
        s.validCount = results[i]->valid;
        s.pixelCount = total;
        out[i] = s;
    }
    return out;
}

bool SidecarWritesEnabled() {
    const char* e = std::getenv("DDBT_STATS_SIDECAR");
    return e != nullptr && std::strcmp(e, "write") == 0;
}

}  // namespace

std::string NodataKey(const std::optional<double>& nodata) {
    if (!nodata) return "";
    if (std::isnan(*nodata)) return "nan";
    char text[40];
    std::snprintf(text, sizeof(text), "%.17g", *nodata);
    return text;
}

bool IsLibddbBuildArtefact(const std::string& path) {
    std::error_code ec;
    const fs::path p = fs::weakly_canonical(fs::absolute(U8Path(path), ec), ec);
    if (ec) return false;
    const std::vector<fs::path> parts(p.begin(), p.end());
    for (size_t i = 0; i + 3 < parts.size(); ++i) {
        if (parts[i] != ".ddb" || parts[i + 1] != "build") continue;
        const std::string key = parts[i + 2].string();
        if (key.size() != 64 ||
            !std::all_of(key.begin(), key.end(), [](char c) { return std::isxdigit(static_cast<unsigned char>(c)); }))
            continue;
        std::string rel;
        for (size_t j = i + 3; j < parts.size(); ++j) rel += (rel.empty() ? "" : "/") + parts[j].string();
        return rel == "cog/cog.tif";
    }
    return false;
}

std::vector<std::optional<BandStatistics>> ComputeBandStatistics(const CogSource& source, const std::vector<int>& bands,
                                                                 const std::optional<double>& nodata) {
    return ComputePass(source, bands, nodata, stats::StatsModeFromEnv());
}

std::vector<std::optional<BandStatistics>> ComputeBandStatistics(const CogSource& source, const std::vector<int>& bands,
                                                                 const std::optional<double>& nodata,
                                                                 stats::StatsMode mode) {
    return ComputePass(source, bands, nodata, mode);
}

std::optional<BandStatistics> ReadStatsSidecar(const std::string& rasterPath, int band,
                                               const std::optional<double>& nodata) {
    const auto j = ReadFreshSidecar(rasterPath);
    if (!j) return std::nullopt;
    return FromSidecar(*j, band, nodata);
}

BandStatistics GetBandStatistics(const CogSource& source, int band, const std::optional<double>& nodata) {
    if (band < 1 || band > source.BandCount())
        Fail("Cannot read band " + std::to_string(band) + " of " + source.Path());
    const bool contig = source.PixelInterleaved();
    const std::string ndKey = NodataKey(nodata);
    const int passBand = contig ? 0 : band;
    const std::string key = source.IdentityKey() + "|" + ndKey + "|" + std::to_string(passBand);
    thread_local ThreadMemo memo;
    const uint64_t gen = g_generation.load(std::memory_order_acquire);
    if (memo.generation != gen) {
        memo.entries.clear();
        memo.generation = gen;
    }
    for (const auto& [k, e] : memo.entries) {
        if (k == key && e->done.load(std::memory_order_acquire)) {
            const auto& s = e->bands[static_cast<size_t>(band - 1)];
            if (!s) throw NoValidPixelsError("Cannot compute statistics: no pixel of the band has a value");
            return *s;
        }
    }
    const std::shared_ptr<PassEntry> entry = PassEntryOf(source.IdentityKey(), ndKey, passBand);
    if (memo.entries.size() >= 16) memo.entries.erase(memo.entries.begin());
    memo.entries.emplace_back(key, entry);
    std::lock_guard lock(entry->mutex);
    if (!entry->done.load(std::memory_order_acquire)) {
        const int n = source.BandCount();
        std::vector<int> bandsOfPass;
        if (contig) {
            for (int b = 1; b <= n; ++b) bandsOfPass.push_back(b);
        } else {
            bandsOfPass.push_back(band);
        }
        std::vector<std::optional<BandStatistics>> result(static_cast<size_t>(n));
        std::vector<int> missing;
        const auto sidecar = ReadFreshSidecar(source.Path());
        for (const int b : bandsOfPass) {
            std::optional<BandStatistics> s;
            if (sidecar) s = FromSidecar(*sidecar, b, nodata);
            if (s) {
                result[static_cast<size_t>(b - 1)] = s;
                g_fromSidecar.fetch_add(1, std::memory_order_relaxed);
            } else {
                missing.push_back(b);
            }
        }
        if (!missing.empty()) {
            const auto computed = ComputePass(source, missing, nodata, stats::StatsModeFromEnv());
            const bool write = SidecarWritesEnabled() && IsLibddbBuildArtefact(source.Path());
            for (size_t i = 0; i < missing.size(); ++i) {
                result[static_cast<size_t>(missing[i] - 1)] = computed[i];
                if (computed[i]) {
                    g_computed.fetch_add(1, std::memory_order_relaxed);
                    if (write) WriteSidecar(source.Path(), missing[i], *computed[i], nodata);
                }
            }
        }
        entry->bands = std::move(result);
        entry->done.store(true, std::memory_order_release);
    }
    const auto& s = entry->bands[static_cast<size_t>(band - 1)];
    if (!s) throw NoValidPixelsError("Cannot compute statistics: no pixel of the band has a value");
    return *s;
}

StatsCounters GetStatsCounters() {
    return {g_fromSidecar.load(std::memory_order_relaxed), g_computed.load(std::memory_order_relaxed)};
}

void ClearStatsCache() {
    std::lock_guard lock(g_mapMutex);
    g_fileIndex.clear();
    g_files.clear();
    g_generation.fetch_add(1, std::memory_order_acq_rel);
}

size_t StatsCacheEntryCount() {
    std::lock_guard lock(g_mapMutex);
    size_t n = 0;
    for (const auto& f : g_files)
        for (const auto& nd : f.byNodata) n += nd.passes.size();
    return n;
}

size_t StatsCacheFileCount() {
    std::lock_guard lock(g_mapMutex);
    return g_files.size();
}

}  // namespace ddbt::cog::tiler
