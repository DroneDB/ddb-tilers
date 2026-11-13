// SPDX-License-Identifier: MPL-2.0
#include "copc/build/copc_builder.h"

#include <algorithm>
#include <atomic>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>

#if defined(__GLIBC__)
#include <malloc.h>
#endif

#include "copc/build/bin_store.h"
#include "copc/build/copc_writer.h"
#include "copc/build/morton.h"
#include "copc/build/node_encoder.h"
#include "copc/build/sidecar.h"
#include "copc/readers/point_reader.h"
#include "core/exec/parallel_for.h"
#include "core/exec/task_group.h"
#include "core/exec/thread_pool.h"
#include "core/mem/system_memory.h"
#include "core/util/endian.h"
#include "core/util/timer.h"
#include "core/version.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace ddbt::copc::build {

namespace {

using readers::Batch;
using readers::BatchSink;
using readers::OutputSpec;
using readers::PointReader;
using readers::SourceInfo;

constexpr int kHistDepth = 7;                 // Morton histogram: 8^7 cells
constexpr size_t kSampleTarget = size_t{2} << 20;  // points sampled to plan
// Memory model of the budget, measured on P1 / P4 / P6. The fixed part and the per-thread part
// come off the budget first; the rest is shared by the store (records in memory up to 60 % of it,
// else the first bins up to 70 % plus the spill staging) and the partitions, each admitted only
// when its estimate fits what is left (the next one to commit always runs). Outside the cap: the
// mapped inputs of the PLY, E57 and text readers, the page cache, and a single partition larger
// than the shared part (it then runs alone).
constexpr uint64_t kFixedBytes = uint64_t{64} << 20;      // process, plan tables, bin map, writer
constexpr uint64_t kThreadBytes = uint64_t{12} << 20;     // per thread: reader scratch and decoder, scatter sink
constexpr uint64_t kPartitionBytes = uint64_t{4} << 20;   // per partition: grid scratch, compressor, lists
constexpr uint64_t kDefaultBudget = uint64_t{4096} << 20;  // 4 GiB
constexpr uint64_t kMinBudget = uint64_t{64} << 20;

/// Bytes per point of a partition in flight: its records loaded (spill) and their sorted copy,
/// sort entries, keys, index lists, the encoded chunks waiting to be committed.
double PartitionBytesPerPoint(size_t recordSize) { return 2.0 * static_cast<double>(recordSize) + 48.0; }

/// Bytes per point of the in-memory store: records plus the slack of the bins' last blocks.
double StoreBytesPerPoint(size_t recordSize) { return 1.25 * static_cast<double>(recordSize); }

void TuneAllocator() {
#if defined(__GLIBC__)
    static std::once_flag once;
    std::call_once(once, [] { mallopt(M_MMAP_THRESHOLD, 1 << 20); });
#endif
}

void TrimAllocator() {
#if defined(__GLIBC__)
    malloc_trim(0);
#endif
}

struct TaskRef {
    uint32_t reader;
    uint32_t task;
};

/// Bounds, counts and colour seen by the scatter pass (merged from the tasks).
struct ScatterStats {
    std::array<int32_t, 3> minI{INT32_MAX, INT32_MAX, INT32_MAX};
    std::array<int32_t, 3> maxI{INT32_MIN, INT32_MIN, INT32_MIN};
    std::array<uint64_t, 15> byReturn{};
    double gpsMin = std::numeric_limits<double>::infinity();
    double gpsMax = -std::numeric_limits<double>::infinity();
    uint16_t maxColor = 0;
    uint64_t points = 0;
    bool outside = false;

    void Merge(const ScatterStats& o) {
        for (size_t a = 0; a < 3; ++a) {
            minI[a] = std::min(minI[a], o.minI[a]);
            maxI[a] = std::max(maxI[a], o.maxI[a]);
        }
        for (size_t i = 0; i < 15; ++i) byReturn[i] += o.byReturn[i];
        gpsMin = std::min(gpsMin, o.gpsMin);
        gpsMax = std::max(gpsMax, o.gpsMax);
        maxColor = std::max(maxColor, o.maxColor);
        points += o.points;
        outside = outside || o.outside;
    }
};

/// Bounds of doubles (bounds pass) and a strided sample.
struct ProbeStats {
    std::array<double, 3> min{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity()};
    std::array<double, 3> max{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity()};
    uint64_t points = 0;
    void Merge(const ProbeStats& o) {
        for (size_t a = 0; a < 3; ++a) {
            min[a] = std::min(min[a], o.min[a]);
            max[a] = std::max(max[a], o.max[a]);
        }
        points += o.points;
    }
};

struct Bin {
    int depth;
    uint64_t prefix;
};

struct Partition {
    int depth;
    uint64_t prefix;
    uint32_t binBegin, binEnd;
    uint64_t count;
    int regionLeafDepth;  // below the leaf level: the depth of its region's leaves
};

/// A node above the partitions; child slots: >= 0 partition index, < -1 top node -(i + 2), -1 none.
struct TopNode {
    int depth;
    uint64_t prefix;
    std::array<int64_t, 8> child;
    size_t lastPartition;
};

std::string DirectoryOf(const std::string& path) {
    std::filesystem::path p = std::filesystem::path(std::u8string(path.begin(), path.end())).parent_path();
    if (p.empty()) return ".";
    const std::u8string s = p.u8string();
    return std::string(s.begin(), s.end());
}

void Today(uint16_t& doy, uint16_t& year) {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    doy = static_cast<uint16_t>(tm.tm_yday + 1);
    year = static_cast<uint16_t>(tm.tm_year + 1900);
}

/// The sink of the bounds pass: double bounds and every k-th point (as doubles).
class ProbeSink final : public BatchSink {
public:
    ProbeSink(uint64_t stride, ProbeStats& stats, std::vector<double>& sample)
        : stride_(stride), stats_(stats), sample_(sample) {}
    Status Consume(Batch& b) override {
        if (b.xyz == nullptr) return InternalError("bounds pass: a reader gave quantised coordinates");
        for (size_t i = 0; i < b.count; ++i) {
            const double* p = b.xyz + 3 * i;
            for (size_t a = 0; a < 3; ++a) {
                if (!std::isfinite(p[a])) return Corrupt("a point has a non-finite coordinate");
                stats_.min[a] = std::min(stats_.min[a], p[a]);
                stats_.max[a] = std::max(stats_.max[a], p[a]);
            }
            if (seen_++ % stride_ == 0) sample_.insert(sample_.end(), p, p + 3);
        }
        stats_.points += b.count;
        return Status();
    }

private:
    uint64_t stride_;
    uint64_t seen_ = 0;
    ProbeStats& stats_;
    std::vector<double>& sample_;
};

/// The sink of the sampling pass of LAS inputs: keys of quantised records.
class KeySampleSink final : public BatchSink {
public:
    KeySampleSink(const OutputSpec& spec, const KeyMapper& m, std::vector<uint64_t>& keys)
        : spec_(spec), m_(m), keys_(keys) {}
    Status Consume(Batch& b) override {
        if (b.xyz != nullptr) DDBT_TRY(readers::QuantizeXyz(spec_, b.xyz, b.count, b.records));
        const size_t rs = spec_.RecordSize();
        bool outside = false;
        for (size_t i = 0; i < b.count; ++i) keys_.push_back(m_.KeyOfRecord(b.records + i * rs, outside));
        return Status();
    }

private:
    const OutputSpec& spec_;
    const KeyMapper& m_;
    std::vector<uint64_t>& keys_;
};

/// The sink of the scatter pass: quantise, key, count, append to the bins.
class ScatterSink final : public BatchSink {
public:
    ScatterSink(const OutputSpec& spec, const KeyMapper& m, const std::vector<uint32_t>& binOfCell, size_t bins,
                BinStore::Writer& w, ScatterStats& st)
        : spec_(spec), m_(m), binOfCell_(binOfCell), w_(w), st_(st), counts_(bins, 0) {}

    Status Consume(Batch& b) override {
        if (b.xyz != nullptr) DDBT_TRY(readers::QuantizeXyz(spec_, b.xyz, b.count, b.records));
        const size_t rs = spec_.RecordSize();
        const size_t n = b.count;
        binIdx_.resize(n);
        touched_.clear();
        const bool rgb = spec_.format >= 7;
        const int shift = 3 * (kKeyBits - kHistDepth);
        for (size_t i = 0; i < n; ++i) {
            const uint8_t* r = b.records + i * rs;
            const int32_t x = LoadLE<int32_t>(r), y = LoadLE<int32_t>(r + 4), z = LoadLE<int32_t>(r + 8);
            st_.minI[0] = std::min(st_.minI[0], x);
            st_.maxI[0] = std::max(st_.maxI[0], x);
            st_.minI[1] = std::min(st_.minI[1], y);
            st_.maxI[1] = std::max(st_.maxI[1], y);
            st_.minI[2] = std::min(st_.minI[2], z);
            st_.maxI[2] = std::max(st_.maxI[2], z);
            const uint8_t rn = r[readers::rec::kReturns] & 0x0F;
            if (rn >= 1) ++st_.byReturn[rn - 1];
            const double g = LoadLE<double>(r + readers::rec::kGpsTime);
            st_.gpsMin = std::min(st_.gpsMin, g);
            st_.gpsMax = std::max(st_.gpsMax, g);
            if (rgb) {
                const uint16_t c = std::max({LoadLE<uint16_t>(r + 30), LoadLE<uint16_t>(r + 32), LoadLE<uint16_t>(r + 34)});
                st_.maxColor = std::max(st_.maxColor, c);
            }
            const uint64_t key = m_.Key(x, y, z, st_.outside);
            const uint32_t bin = binOfCell_[key >> shift];
            binIdx_[i] = bin;
            if (counts_[bin]++ == 0) touched_.push_back(bin);
        }
        st_.points += n;
        // Group the records by bin (counting sort) and append each group.
        size_t pos = 0;
        for (uint32_t bin : touched_) {
            const uint32_t c = counts_[bin];
            counts_[bin] = static_cast<uint32_t>(pos);
            pos += c;
        }
        grouped_.resize(n * rs);
        for (size_t i = 0; i < n; ++i)
            std::memcpy(grouped_.data() + size_t{counts_[binIdx_[i]]++} * rs, b.records + i * rs, rs);
        size_t start = 0;
        for (uint32_t bin : touched_) {
            const size_t end = counts_[bin];
            DDBT_TRY(w_.Append(bin, grouped_.data() + start * rs, end - start));
            start = end;
            counts_[bin] = 0;
        }
        return Status();
    }

private:
    const OutputSpec& spec_;
    const KeyMapper& m_;
    const std::vector<uint32_t>& binOfCell_;
    BinStore::Writer& w_;
    ScatterStats& st_;
    std::vector<uint32_t> counts_;
    std::vector<uint32_t> binIdx_;
    std::vector<uint32_t> touched_;
    std::vector<uint8_t> grouped_;
};

class Builder {
public:
    Builder(const CopcBuildOptions& o, CopcBuildReport& r) : o_(o), r_(r) {}

    Status Run(const std::vector<std::string>& inputs, const std::string& output) {
        Timer total;
        if (o_.threads > 0) {
            exec::ThreadPoolOptions po;
            po.threads = o_.threads - 1;
            po.zeroWorkers = o_.threads == 1;
            po.name = "ddbt-copc";
            ownPool_ = std::make_unique<exec::ThreadPool>(po);
            pool_ = ownPool_.get();
            threads_ = o_.threads;
        } else {
            pool_ = &exec::DefaultPool();
            threads_ = pool_->Size() + 1;
        }
        Progress(0, "planning");
        Timer t;
        DDBT_TRY(OpenInputs(inputs));
        DDBT_TRY(ChooseOutput());
        DDBT_TRY(PlanBounds());
        for (int attempt = 0;; ++attempt) {
            DDBT_TRY(PlanBins());
            r_.secondsPlan += t.ElapsedSeconds();
            t = Timer();
            Progress(0.05, "reading");
            DDBT_TRY(Scatter());
            r_.secondsScatter += t.ElapsedSeconds();
            t = Timer();
            if (!stats_.outside) break;
            if (attempt > 0) return InternalError("points outside the cube after planning from their own bounds");
            // The header bounds were wrong: plan again from the real bounds.
            r_.restarted = true;
            for (size_t a = 0; a < 3; ++a) {
                bounds_.min[a] = MulAddUnfused(stats_.minI[a], spec_.scale[a], spec_.offset[a]);
                bounds_.max[a] = MulAddUnfused(stats_.maxI[a], spec_.scale[a], spec_.offset[a]);
            }
            DDBT_TRY(ChooseQuantization(false));
            ChooseCube();
            store_.reset();
        }
        if (stats_.points == 0) return InvalidArgument("the input holds no points");
        // The plan's tables are no longer needed; give the scatter's freed buffers back.
        std::vector<uint32_t>().swap(binOfCell_);
        std::vector<double>().swap(probeSample_);
        if (o_.tuneAllocator) TrimAllocator();
        lod_ = o_.lod;
        lod_.leafLevel = LeafLevel(stats_.points, lod_.leafMax);
        r_.leafLevel = lod_.leafLevel;
        Progress(0.4, "building");
        DDBT_TRY(MakePartitions());
        DDBT_TRY(OpenWriter());
        DDBT_TRY(Process(output));
        r_.secondsProcess = t.ElapsedSeconds();
        Progress(1.0, "writing");
        if (o_.metadata) DDBT_TRY(WriteSidecar(output));
        r_.secondsTotal = total.ElapsedSeconds();
        return Status();
    }

private:
    void Progress(double f, const char* phase) {
        if (o_.progress) o_.progress(f, phase);
    }

    // ------------------------------------------------------------ planning

    Status OpenInputs(const std::vector<std::string>& inputs) {
        for (const std::string& in : inputs) {
            readers::ReaderOptions ro;
            auto r = readers::OpenReader(in, ro);
            if (!r.ok()) return r.status().WithContext(in);
            readers_.push_back(std::move(*r));
        }
        return Status();
    }

    Status ChooseOutput() {
        bool rgb = false, nir = false;
        for (const auto& r : readers_) {
            rgb = rgb || r->Info().hasRgb;
            nir = nir || r->Info().hasNir;
        }
        spec_.format = nir ? 8 : rgb ? 7 : 6;
        // Extra bytes: kept when every input is LAS with the same descriptors.
        const SourceInfo& first = readers_[0]->Info();
        bool sameEb = first.isLas && first.lasExtraBytes > 0;
        for (const auto& r : readers_) {
            const SourceInfo& i = r->Info();
            if (!i.isLas || i.lasExtraBytes != first.lasExtraBytes || i.extraBytesVlr != first.extraBytesVlr) sameEb = false;
        }
        if (sameEb) {
            spec_.extraBytes = first.lasExtraBytes;
            fields_.extraBytesVlr = first.extraBytesVlr;
        }
        fields_.format = spec_.format;
        fields_.extraBytes = spec_.extraBytes;
        for (const auto& r : readers_)
            if (!r->Info().wkt.empty()) {
                fields_.wkt = r->Info().wkt;
                break;
            }
        // Header fields from the first LAS input.
        const SourceInfo* las = nullptr;
        for (const auto& r : readers_)
            if (r->Info().isLas) {
                las = &r->Info();
                break;
            }
        fields_.software = std::string("ddbt ") + Version();
        if (las != nullptr) {
            const las::LasHeader& h = las->lasHeader;
            fields_.fileSourceId = h.fileSourceId;
            fields_.globalEncoding = static_cast<uint16_t>(h.globalEncoding & 1u);
            fields_.projectId = h.projectId;
            fields_.systemId = h.systemIdentifier.empty() ? "ddbt" : h.systemIdentifier;
            fields_.creationDoy = h.creationDayOfYear;
            fields_.creationYear = h.creationYear;
        } else {
            fields_.systemId = "ddbt";
            if (o_.creationYear != 0) {
                fields_.creationDoy = o_.creationDoy;
                fields_.creationYear = o_.creationYear;
            } else {
                Today(fields_.creationDoy, fields_.creationYear);
            }
        }
        uint64_t n = 0;
        for (const auto& r : readers_) n += r->Info().pointCount;
        expectedPoints_ = n;
        return Status();
    }

    Status PlanBounds() {
        bool first = true;
        auto grow = [&](const std::array<double, 3>& mn, const std::array<double, 3>& mx) {
            for (size_t a = 0; a < 3; ++a) {
                bounds_.min[a] = first ? mn[a] : std::min(bounds_.min[a], mn[a]);
                bounds_.max[a] = first ? mx[a] : std::max(bounds_.max[a], mx[a]);
            }
            first = false;
        };
        // Readers without trusted bounds: a bounds pass (also their sample).
        std::vector<uint32_t> probe;
        for (uint32_t i = 0; i < readers_.size(); ++i) {
            const SourceInfo& info = readers_[i]->Info();
            if (info.boundsKnown) grow(info.min, info.max);
            else probe.push_back(i);
        }
        if (!probe.empty()) {
            std::vector<TaskRef> tasks;
            uint64_t estimate = 0;
            for (uint32_t i : probe) {
                for (uint32_t t = 0; t < readers_[i]->TaskCount(); ++t) tasks.push_back({i, t});
                estimate += readers_[i]->Info().pointCount;
            }
            const uint64_t stride = std::max<uint64_t>(1, estimate / kSampleTarget);
            std::vector<ProbeStats> stats(tasks.size());
            std::vector<std::vector<double>> samples(tasks.size());
            std::vector<Status> status(tasks.size());
            OutputSpec probeSpec = spec_;
            probeSpec.quantized = false;
            exec::ParallelFor(*pool_, 0, tasks.size(), 1, [&](size_t k) {
                ProbeSink sink(stride, stats[k], samples[k]);
                status[k] = readers_[tasks[k].reader]->Read(tasks[k].task, probeSpec, sink);
            });
            ProbeStats all;
            for (size_t k = 0; k < tasks.size(); ++k) {
                DDBT_TRY(status[k]);
                all.Merge(stats[k]);
            }
            if (all.points > 0) grow(all.min, all.max);
            for (auto& s : samples) probeSample_.insert(probeSample_.end(), s.begin(), s.end());
            probeWeight_ = static_cast<double>(stride);
            // Exact counts of these inputs.
            expectedPoints_ = 0;
            for (const auto& r : readers_)
                if (r->Info().boundsKnown) expectedPoints_ += r->Info().pointCount;
            expectedPoints_ += all.points;
        }
        if (first) return InvalidArgument("the input holds no points");
        DDBT_TRY(ChooseQuantization(true));
        ChooseCube();
        return Status();
    }

    Status ChooseQuantization(bool keepLas) {
        bool allLas = !readers_.empty();
        bool same = true;
        const SourceInfo& f = readers_[0]->Info();
        std::array<double, 3> finest{0, 0, 0};
        bool anyLas = false;
        for (const auto& r : readers_) {
            const SourceInfo& i = r->Info();
            if (!i.isLas) {
                allLas = false;
                continue;
            }
            if (i.lasScale != f.lasScale || i.lasOffset != f.lasOffset) same = false;
            for (size_t a = 0; a < 3; ++a) finest[a] = anyLas ? std::min(finest[a], i.lasScale[a]) : i.lasScale[a];
            anyLas = true;
        }
        if (keepLas && allLas && same) {
            spec_.scale = f.lasScale;
            spec_.offset = f.lasOffset;
        } else {
            for (size_t a = 0; a < 3; ++a) {
                double s = anyLas ? finest[a] : 0.001;
                if (!(s > 0) || !std::isfinite(s)) s = 0.001;
                const double centre = (bounds_.min[a] + bounds_.max[a]) / 2;
                const double off = std::isfinite(centre) ? std::round(centre) + 0.0 : 0.0;  // + 0.0: no -0
                const double reach = std::max(std::abs(bounds_.max[a] - off), std::abs(bounds_.min[a] - off));
                while (reach / s > 2.0e9) s *= 10;
                spec_.scale[a] = s;
                spec_.offset[a] = off;
            }
        }
        // Every coordinate must fit an int32.
        for (size_t a = 0; a < 3; ++a) {
            const double lo = (bounds_.min[a] - spec_.offset[a]) / spec_.scale[a];
            const double hi = (bounds_.max[a] - spec_.offset[a]) / spec_.scale[a];
            if (!(lo >= -2147483648.0 && hi <= 2147483647.0))
                return OutOfRange("the bounds do not fit the LAS scale / offset");
        }
        fields_.scale = spec_.scale;
        fields_.offset = spec_.offset;
        return Status();
    }

    /// The cube grows from the bounds' minimum corner (as Untwine's): the largest extent plus
    /// one scale step on both sides. A flat cloud then lies in the lower half of the cube in z
    /// instead of straddling the middle, as with a centred cube, which splits its nodes in two.
    void ChooseCube() {
        double extent = 0;
        for (size_t a = 0; a < 3; ++a) extent = std::max(extent, bounds_.max[a] - bounds_.min[a]);
        const double quantum = std::max({spec_.scale[0], spec_.scale[1], spec_.scale[2]});
        cube_.halfSize = extent / 2 + quantum;
        for (size_t a = 0; a < 3; ++a) cube_.center[a] = (bounds_.min[a] - quantum) + cube_.halfSize;
        mapper_ = KeyMapper(cube_, spec_.scale, spec_.offset);
    }

    Status PlanBins() {
        const size_t rs = spec_.RecordSize();
        // Sample keys: LAS inputs by Sample(), others from the bounds pass.
        std::vector<std::pair<uint64_t, double>> weighted;  // (key, weight)
        {
            std::vector<TaskRef> tasks;
            uint64_t lasPoints = 0;
            for (uint32_t i = 0; i < readers_.size(); ++i)
                if (readers_[i]->Info().boundsKnown) {
                    for (uint32_t t = 0; t < readers_[i]->TaskCount(); ++t) tasks.push_back({i, t});
                    lasPoints += readers_[i]->Info().pointCount;
                }
            if (!tasks.empty()) {
                const size_t per = std::clamp<size_t>(kSampleTarget / tasks.size(), 16, 4096);
                std::vector<std::vector<uint64_t>> keys(tasks.size());
                std::vector<Status> status(tasks.size());
                exec::ParallelFor(*pool_, 0, tasks.size(), 1, [&](size_t k) {
                    KeySampleSink sink(spec_, mapper_, keys[k]);
                    status[k] = readers_[tasks[k].reader]->Sample(tasks[k].task, per, spec_, sink);
                });
                for (size_t k = 0; k < tasks.size(); ++k) {
                    DDBT_TRY(status[k]);
                    const double w = keys[k].empty() ? 0.0
                                                     : static_cast<double>(readers_[tasks[k].reader]->TaskPoints(tasks[k].task)) /
                                                           static_cast<double>(keys[k].size());
                    for (uint64_t key : keys[k]) weighted.emplace_back(key, w);
                }
            }
            if (!probeSample_.empty()) {
                const size_t n = probeSample_.size() / 3;
                std::vector<uint8_t> recs(rs * n);
                DDBT_TRY(readers::QuantizeXyz(spec_, probeSample_.data(), n, recs.data()));
                bool outside = false;
                for (size_t i = 0; i < n; ++i) weighted.emplace_back(mapper_.KeyOfRecord(recs.data() + i * rs, outside), probeWeight_);
            }
        }
        const size_t cells = size_t{1} << (3 * kHistDepth);
        std::vector<double> psum(cells + 1, 0.0);
        const int shift = 3 * (kKeyBits - kHistDepth);
        for (const auto& [key, w] : weighted) psum[(key >> shift) + 1] += w;
        for (size_t i = 0; i < cells; ++i) psum[i + 1] += psum[i];

        // Memory: the budget less the fixed part and the per-thread state is what the store and
        // the partitions share. The records stay in memory when they fit 60 % of it.
        const uint64_t n = std::max<uint64_t>(expectedPoints_, 1);
        DDBT_ASSIGN_OR_RETURN(budget_, MemoryBudget());
        r_.memoryBudget = budget_;
        const uint64_t fixed = kFixedBytes + threads_ * kThreadBytes;
        avail_ = budget_ > fixed ? budget_ - fixed : 0;
        const double perPoint = PartitionBytesPerPoint(rs);
        const double storeBytes = static_cast<double>(n) * StoreBytesPerPoint(rs);
        spill_ = o_.forceSpill || storeBytes > 0.6 * static_cast<double>(avail_);
        // Spilling, the first bins (processed first) still keep 70 % of it in memory.
        const double memFraction = o_.spillMemoryFraction >= 0 ? std::min(o_.spillMemoryFraction, 1.0)
                                   : o_.forceSpill               ? 0.0
                                                                 : 0.7;
        const double memShare = spill_ ? std::min(storeBytes, memFraction * static_cast<double>(avail_)) : storeBytes;
        // Partition size: every thread processing one at once fits what the store leaves, and
        // enough partitions keep every thread busy.
        const double procBudget = static_cast<double>(avail_) - memShare - static_cast<double>(threads_ * kPartitionBytes);
        uint64_t partMax = static_cast<uint64_t>(std::max(0.0, procBudget) / (static_cast<double>(threads_) * perPoint));
        partMax = std::min<uint64_t>(partMax, std::max<uint64_t>(n / (4 * threads_), 1));
        partMax = std::max<uint64_t>(partMax, uint64_t{4} * o_.lod.leafMax);
        if (o_.maxPartitionPoints) partMax = std::max<uint64_t>(o_.maxPartitionPoints, o_.lod.leafMax);
        partMax_ = partMax;
        r_.maxPartitionPoints = partMax;
        // Bins are cut finer than partitions (the estimate is a sample); in spill mode fewer bins
        // make larger staging buffers, so larger segments to read back.
        const double binMax = static_cast<double>(partMax) / (spill_ ? 1 : 4);

        bins_.clear();
        binOfCell_.assign(cells, 0);
        std::function<void(int, uint64_t)> plan = [&](int d, uint64_t prefix) {
            const uint64_t lo = prefix << (3 * (kHistDepth - d));
            const uint64_t hi = (prefix + 1) << (3 * (kHistDepth - d));
            const double est = psum[hi] - psum[lo];
            if (est <= binMax || d == kHistDepth) {
                const uint32_t b = static_cast<uint32_t>(bins_.size());
                bins_.push_back({d, prefix});
                std::fill(binOfCell_.begin() + static_cast<ptrdiff_t>(lo), binOfCell_.begin() + static_cast<ptrdiff_t>(hi), b);
                return;
            }
            for (uint64_t c = 0; c < 8; ++c) plan(d + 1, prefix * 8 + c);
        };
        plan(0, 0);
        r_.bins = bins_.size();
        // Bins kept in memory when spilling: the first ones up to the share (estimated counts).
        binInMemory_.assign(bins_.size(), 0);
        if (spill_) {
            double used = 0;
            const double perStored = StoreBytesPerPoint(rs);
            for (size_t b = 0; b < bins_.size(); ++b) {
                const int sh = 3 * (kHistDepth - bins_[b].depth);
                const double bytes = (psum[(bins_[b].prefix + 1) << sh] - psum[bins_[b].prefix << sh]) * perStored;
                if (used + bytes > memShare) break;
                used += bytes;
                binInMemory_[b] = 1;
            }
        }
        return Status();
    }

    uint64_t Window() const { return std::max<uint64_t>(2, 4 * threads_); }

    Expected<uint64_t> MemoryBudget() const {
        uint64_t b = o_.memoryBudget;
        if (b == 0) {
            DDBT_ASSIGN_OR_RETURN(b, DefaultCopcMemoryBudget());
        }
        return std::max(b, kMinBudget);
    }

    // ------------------------------------------------------------ scatter

    Status Scatter() {
        const size_t rs = spec_.RecordSize();
        if (spill_) {
            const std::string dir = o_.tempDir.empty() ? DirectoryOf(outputPath_) : o_.tempDir;
            // Staging: 15 % of the shared budget over the spilled bins (one buffer per bin; the
            // threads writing a full one hold one more each), 4 KiB to 1 MiB each.
            size_t spilledBins = 0;
            for (uint8_t m : binInMemory_) spilledBins += m == 0;
            const double per = 0.15 * static_cast<double>(avail_) / static_cast<double>(std::max<size_t>(spilledBins, 1) + threads_);
            const size_t staging = static_cast<size_t>(std::clamp(per, 4096.0, 1048576.0));
            DDBT_ASSIGN_OR_RETURN(store_, BinStore::CreateSpill(dir, bins_.size(), rs, staging, binInMemory_));
        } else {
            store_ = BinStore::CreateMemory(bins_.size(), rs);
        }
        r_.spilled = spill_;
        std::vector<TaskRef> tasks;
        for (uint32_t i = 0; i < readers_.size(); ++i)
            for (uint32_t t = 0; t < readers_[i]->TaskCount(); ++t) tasks.push_back({i, t});
        std::mutex mu;
        std::vector<std::unique_ptr<BinStore::Writer>> allWriters;
        std::vector<BinStore::Writer*> freeWriters;
        Status firstError;
        std::atomic<bool> failed{false};
        ScatterStats total;
        std::atomic<uint64_t> peak{0};
        exec::ParallelFor(*pool_, 0, tasks.size(), 1, [&](size_t k) {
            if (failed.load(std::memory_order_relaxed)) return;
            BinStore::Writer* w = nullptr;
            {
                std::lock_guard lock(mu);
                if (freeWriters.empty()) {
                    allWriters.push_back(store_->NewWriter());
                    w = allWriters.back().get();
                } else {
                    w = freeWriters.back();
                    freeWriters.pop_back();
                }
            }
            ScatterStats st;
            ScatterSink sink(spec_, mapper_, binOfCell_, bins_.size(), *w, st);
            Status s = readers_[tasks[k].reader]->Read(tasks[k].task, spec_, sink);
            uint64_t m = store_->MemoryBytes(), p = peak.load(std::memory_order_relaxed);
            while (m > p && !peak.compare_exchange_weak(p, m)) {
            }
            std::lock_guard lock(mu);
            if (!s.ok() && firstError.ok()) {
                firstError = s.WithContext(readers_[tasks[k].reader]->Info().path);
                failed = true;
            }
            total.Merge(st);
            freeWriters.push_back(w);
        });
        for (auto& w : allWriters) {
            Status s = w->Flush();
            if (!s.ok() && firstError.ok()) firstError = s;
        }
        allWriters.clear();
        DDBT_TRY(firstError);
        stats_ = total;
        r_.peakStoreBytes = std::max(r_.peakStoreBytes, peak.load());
        r_.spillBytes = store_->SpilledBytes();
        if (stats_.points != expectedPoints_)
            return Corrupt("read " + std::to_string(stats_.points) + " points, the inputs announce " +
                           std::to_string(expectedPoints_));
        return Status();
    }

    // ------------------------------------------------------------ partitions

    Status MakePartitions() {
        std::vector<uint64_t> binCount(bins_.size());
        for (size_t b = 0; b < bins_.size(); ++b) binCount[b] = store_->Count(static_cast<uint32_t>(b));
        partitions_.clear();
        top_.clear();
        // Returns the slot of node (d, prefix) covering bins [b0, b1). A leaf of the LOD is a
        // partition whatever its size, so every top node is an inner node of the tree.
        std::function<int64_t(int, uint64_t, uint32_t, uint32_t, int)> rec = [&](int d, uint64_t prefix, uint32_t b0,
                                                                                uint32_t b1, int regionLeaf) -> int64_t {
            if (b0 == b1) return -1;
            uint64_t cnt = 0;
            for (uint32_t b = b0; b < b1; ++b) cnt += binCount[b];
            if (cnt == 0) return -1;
            if (d == lod_.leafLevel) regionLeaf = RegionLeafDepth(cnt, lod_);
            const bool single = b1 - b0 == 1 && bins_[b0].depth == d;
            if (single || cnt <= partMax_ || IsLeaf(d, cnt, regionLeaf, lod_)) {
                partitions_.push_back({d, prefix, b0, b1, cnt, d > lod_.leafLevel ? regionLeaf : -1});
                return static_cast<int64_t>(partitions_.size() - 1);
            }
            TopNode t{d, prefix, {}, 0};
            uint32_t b = b0;
            for (uint64_t c = 0; c < 8; ++c) {
                const uint64_t want = prefix * 8 + c;
                uint32_t e = b;
                while (e < b1 && (bins_[e].prefix >> (3 * (bins_[e].depth - (d + 1)))) == want) ++e;
                t.child[c] = rec(d + 1, want, b, e, regionLeaf);
                b = e;
            }
            t.lastPartition = partitions_.size() - 1;
            top_.push_back(t);
            return -static_cast<int64_t>(top_.size() - 1) - 2;
        };
        const int64_t root = rec(0, 0, 0, static_cast<uint32_t>(bins_.size()), -1);
        if (root == -1) return InvalidArgument("the input holds no points");
        rootIsPartition_ = root >= 0;
        r_.partitions = partitions_.size();
        r_.topNodes = top_.size();
        for (const Partition& p : partitions_) r_.largestPartition = std::max(r_.largestPartition, p.count);
        return Status();
    }

    // ------------------------------------------------------------ processing

    struct Emitted {
        int depth;
        uint64_t prefix;
        uint32_t count;
        std::vector<uint8_t> chunk;
    };
    struct PartResult {
        Status status;
        std::vector<Emitted> emitted;
        NodePoints root;
    };

    Status ProcessPartition(size_t i, PartResult& out) {
        const Partition& p = partitions_[i];
        const size_t rs = spec_.RecordSize();
        std::vector<uint32_t> bins;
        for (uint32_t b = p.binBegin; b < p.binEnd; ++b) bins.push_back(b);
        LoadedRecords lr;
        DDBT_TRY(store_->Load(bins, lr));
        std::vector<SortEntry> entries;
        entries.reserve(static_cast<size_t>(lr.count));
        bool outside = false;
        const int shift = 3 * (kKeyBits - p.depth);
        for (const auto& piece : lr.pieces) {
            const size_t n = piece.size() / rs;
            for (size_t k = 0; k < n; ++k) {
                const uint8_t* rec = piece.data() + k * rs;
                const uint64_t key = mapper_.KeyOfRecord(rec, outside);
                if (p.depth > 0 && (key >> shift) != p.prefix) return InternalError("a point is outside its partition");
                entries.push_back({key, rec});
            }
        }
        const int format = spec_.format, eb = spec_.extraBytes;
        EmitFn emit = [&](int d, uint64_t prefix, const uint8_t* recs, uint32_t count) -> Status {
            Emitted e{d, prefix, count, {}};
            if (count) DDBT_TRY(EncodeNode(format, eb, recs, count, e.chunk));
            out.emitted.push_back(std::move(e));
            return Status();
        };
        return BuildPartition(p.depth, p.prefix, p.regionLeafDepth, entries, rs, lod_, emit, out.root, [&] {
            lr = LoadedRecords();
            store_->Release(bins);
        });
    }

    Status Process(const std::string& output) {
        (void)output;
        const size_t rs = spec_.RecordSize();
        const size_t P = partitions_.size();
        std::vector<PartResult> results(P);
        std::vector<NodePoints> partRoot(P), topPoints(top_.size());
        CopcWriter& w = *writer_;

        // Writes in the global emission order. A partition's chunks are ready when it is
        // committed; the children a top node emits are encoded by tasks, so the committer goes
        // on with the next partitions and writes them once the encodings ahead are done.
        struct Write {
            int depth;
            uint64_t prefix;
            uint32_t count;
            std::vector<uint8_t> records, chunk;
            uint64_t bytes;  // reserved until written
            Status status;
            std::atomic<bool> done{false};
        };
        std::deque<std::unique_ptr<Write>> writes;
        exec::TaskGroup encodes(*pool_);  // after `writes`: destroyed (waited for) first
        std::mutex mu;                    // guards the reservation state below
        uint64_t reserved = 0, peakReserved = 0;
        auto reserve = [&](int64_t delta) {
            std::lock_guard lock(mu);
            reserved = static_cast<uint64_t>(static_cast<int64_t>(reserved) + delta);
            peakReserved = std::max(peakReserved, reserved);
        };
        auto flush = [&](bool all) -> Status {
            if (all && !writes.empty()) encodes.Wait();
            while (!writes.empty() && writes.front()->done.load(std::memory_order_acquire)) {
                Write& x = *writes.front();
                DDBT_TRY(x.status);
                DDBT_TRY(w.AddNode(x.depth, x.prefix, x.count, x.chunk));
                reserve(-static_cast<int64_t>(x.bytes));
                writes.pop_front();
            }
            return Status();
        };

        auto processTop = [&](size_t t) -> Status {
            const TopNode& tn = top_[t];
            std::array<NodePoints*, 8> ch{};
            for (size_t c = 0; c < 8; ++c) {
                const int64_t s = tn.child[c];
                if (s >= 0) ch[c] = &partRoot[static_cast<size_t>(s)];
                else if (s < -1) ch[c] = &topPoints[static_cast<size_t>(-s - 2)];
            }
            EmitFn emit = [&](int d, uint64_t prefix, const uint8_t* recs, uint32_t count) -> Status {
                auto x = std::make_unique<Write>();
                x->depth = d;
                x->prefix = prefix;
                x->count = count;
                x->records.assign(recs, recs + size_t{count} * rs);
                x->bytes = x->records.size() + x->records.size() / 2;  // the records, then the chunk
                reserve(static_cast<int64_t>(x->bytes));
                Write* p = x.get();
                writes.push_back(std::move(x));
                if (count == 0) {
                    p->done.store(true, std::memory_order_release);
                } else {
                    encodes.Spawn([p, this] {
                        p->status = EncodeNode(spec_.format, spec_.extraBytes, p->records.data(), p->count, p->chunk);
                        std::vector<uint8_t>().swap(p->records);
                        p->done.store(true, std::memory_order_release);
                    });
                }
                return Status();
            };
            return BuildTopNode(tn.depth, tn.prefix, ch, rs, lod_, emit, topPoints[t]);
        };

        // Partitions run ahead of the committer, which writes them in order. A partition is
        // launched when its estimated bytes fit what the budget leaves (the store's bytes and
        // the reservations of the partitions in flight or waiting, and of the roots and top
        // nodes held); the next one to commit always runs; at most `window` run ahead. A
        // finished partition keeps only its encoded chunks and its root reserved and launches
        // what then fits; a committed one keeps its root until its top node is built.
        const size_t window = static_cast<size_t>(Window());
        const double perPoint = PartitionBytesPerPoint(rs);
        auto estimate = [&](size_t i) {
            return static_cast<uint64_t>(static_cast<double>(partitions_[i].count) * perPoint) + kPartitionBytes;
        };
        auto heldBytes = [](const NodePoints& np) { return np.keys.capacity() * sizeof(uint64_t) + np.records.capacity(); };
        std::vector<uint64_t> held(P, 0);
        std::vector<std::unique_ptr<exec::TaskGroup>> groups(P);
        // mu also guards held, next, committed, stop, groups[next].
        size_t next = 0, committed = 0;
        bool stop = false;
        std::function<void()> fill;  // launches what fits; mu held
        fill = [&] {
            while (!stop && next < P && next < committed + window) {
                const size_t i = next;
                const uint64_t e = estimate(i);
                const uint64_t store = store_->MemoryBytes();
                const uint64_t room = avail_ > store ? avail_ - store : 0;
                if (i > committed && reserved + e > room) break;
                held[i] = e;
                reserved += e;
                peakReserved = std::max(peakReserved, reserved);
                ++next;
                groups[i] = std::make_unique<exec::TaskGroup>(*pool_);
                groups[i]->Spawn([this, i, &results, &mu, &held, &reserved, &fill, &heldBytes] {
                    PartResult& r = results[i];
                    r.status = ProcessPartition(i, r);
                    uint64_t keep = heldBytes(r.root) + kPartitionBytes / 4;
                    for (const Emitted& em : r.emitted) keep += em.chunk.capacity() + sizeof(Emitted);
                    std::lock_guard lock(mu);
                    if (keep < held[i]) {
                        reserved -= held[i] - keep;
                        held[i] = keep;
                    }
                    fill();
                });
            }
        };
        Status err;
        size_t nextTop = 0;
        for (size_t i = 0; i < P && err.ok(); ++i) {
            exec::TaskGroup* g;
            {
                std::lock_guard lock(mu);
                fill();
                g = groups[i].get();
            }
            Timer waitTimer;
            g->Wait();
            r_.secondsCommitWait += waitTimer.ElapsedSeconds();
            if (!results[i].status.ok()) {
                err = std::move(results[i].status);
                break;
            }
            err = flush(false);
            for (Emitted& e : results[i].emitted) {
                if (!err.ok()) break;
                if (writes.empty()) {
                    err = w.AddNode(e.depth, e.prefix, e.count, e.chunk);
                    e.chunk = {};
                    continue;
                }
                auto x = std::make_unique<Write>();
                x->depth = e.depth;
                x->prefix = e.prefix;
                x->count = e.count;
                x->bytes = e.chunk.capacity();
                x->chunk = std::move(e.chunk);
                x->done.store(true, std::memory_order_relaxed);
                reserve(static_cast<int64_t>(x->bytes));
                writes.push_back(std::move(x));
            }
            std::vector<Emitted>().swap(results[i].emitted);
            partRoot[i] = std::move(results[i].root);
            {
                std::lock_guard lock(mu);
                groups[i].reset();
                committed = i + 1;
                reserved -= held[i];
                held[i] = heldBytes(partRoot[i]);
                reserved += held[i];
            }
            // Top nodes are built without the lock (emit takes it).
            while (err.ok() && nextTop < top_.size() && top_[nextTop].lastPartition == i) {
                // The children's points are consumed; the node's own are held until its parent.
                const TopNode& tn = top_[nextTop];
                uint64_t childBytes = 0;
                {
                    std::lock_guard lock(mu);
                    for (int64_t c : tn.child)
                        if (c >= 0) childBytes += held[static_cast<size_t>(c)];
                        else if (c < -1) childBytes += heldBytes(topPoints[static_cast<size_t>(-c - 2)]);
                    peakReserved = std::max(peakReserved, reserved + childBytes);  // its points while it is built
                }
                Timer topTimer;
                err = processTop(nextTop);
                r_.secondsTopNodes += topTimer.ElapsedSeconds();
                if (err.ok()) err = flush(false);
                std::lock_guard lock(mu);
                for (int64_t c : tn.child)
                    if (c >= 0) held[static_cast<size_t>(c)] = 0;
                reserved -= childBytes;
                reserved += heldBytes(topPoints[nextTop]);
                ++nextTop;
            }
            {
                std::lock_guard lock(mu);
                fill();
            }
            Progress(0.4 + 0.55 * static_cast<double>(i + 1) / static_cast<double>(P), "building");
        }
        {
            std::lock_guard lock(mu);
            stop = true;
            r_.peakProcessBytes = peakReserved;
        }
        groups.clear();  // waits for partitions still running after an error
        DDBT_TRY(err);
        DDBT_TRY(flush(true));
        NodePoints& root = rootIsPartition_ ? partRoot[0] : topPoints.back();
        std::vector<uint8_t> chunk;
        DDBT_TRY(EncodeNode(spec_.format, spec_.extraBytes, root.records.data(), root.size(), chunk));
        DDBT_TRY(w.AddNode(0, 0, static_cast<uint32_t>(root.size()), chunk));

        CopcFinalInfo fin;
        for (size_t a = 0; a < 3; ++a) {
            fin.min[a] = MulAddUnfused(stats_.minI[a], spec_.scale[a], spec_.offset[a]);
            fin.max[a] = MulAddUnfused(stats_.maxI[a], spec_.scale[a], spec_.offset[a]);
        }
        fin.pointsByReturn = stats_.byReturn;
        fin.cube = cube_;
        fin.spacing = 2 * cube_.halfSize / 128;
        fin.gpsTimeMin = std::isfinite(stats_.gpsMin) ? stats_.gpsMin : 0;
        fin.gpsTimeMax = std::isfinite(stats_.gpsMax) ? stats_.gpsMax : 0;
        CopcWriteStats ws;
        DDBT_TRY(w.Finish(fin, &ws));
        if (ws.points != stats_.points)
            return InternalError("wrote " + std::to_string(ws.points) + " points of " + std::to_string(stats_.points));
        final_ = fin;
        r_.points = ws.points;
        r_.nodes = ws.nodes;
        r_.maxDepth = ws.maxDepth;
        r_.fileSize = ws.fileSize;
        r_.format = spec_.format;
        r_.extraBytes = spec_.extraBytes;
        r_.scale = spec_.scale;
        r_.offset = spec_.offset;
        r_.cube = cube_;
        r_.spacing = fin.spacing;
        r_.color16 = stats_.maxColor > 255;
        return Status();
    }

    Status WriteSidecar(const std::string& output) {
        SidecarInput in;
        in.points = r_.points;
        in.min = final_.min;
        in.max = final_.max;
        in.wkt = fields_.wkt;
        in.format = spec_.format;
        in.color = spec_.format >= 7;
        in.color16 = r_.color16;
        in.spacing = r_.spacing;
        in.cube = cube_;
        for (const auto& d : readers_[0]->Info().extraBytes)
            if (spec_.extraBytes) in.extraDimensions.push_back(d.name);
        const std::string path = o_.metadataPath.empty() ? output + ".meta.json" : o_.metadataPath;
        return WriteSidecarFile(path, in);
    }

public:
    Status Prepare(const std::string& output) {
        outputPath_ = output;
        return Status();
    }
    Status OpenWriter() {
        const uint64_t estimate = expectedPoints_ * 12;
        DDBT_ASSIGN_OR_RETURN(writer_, CopcWriter::Create(outputPath_, fields_, estimate));
        return Status();
    }
    Status RunAll(const std::vector<std::string>& inputs) { return Run(inputs, outputPath_); }

private:
    const CopcBuildOptions& o_;
    CopcBuildReport& r_;
    std::unique_ptr<exec::ThreadPool> ownPool_;
    exec::ThreadPool* pool_ = nullptr;
    size_t threads_ = 1;
    std::vector<std::unique_ptr<PointReader>> readers_;
    OutputSpec spec_;
    CopcHeaderFields fields_;
    ProbeStats bounds_;
    std::vector<double> probeSample_;
    double probeWeight_ = 1;
    uint64_t expectedPoints_ = 0;
    Cube cube_;
    KeyMapper mapper_;
    uint64_t budget_ = 0;
    uint64_t avail_ = 0;  // the budget less the fixed part and the per-thread state
    bool spill_ = false;
    uint64_t partMax_ = 0;
    LodParams lod_;
    std::vector<Bin> bins_;
    std::vector<uint32_t> binOfCell_;
    std::vector<uint8_t> binInMemory_;
    std::unique_ptr<BinStore> store_;
    ScatterStats stats_;
    std::vector<Partition> partitions_;
    std::vector<TopNode> top_;
    bool rootIsPartition_ = false;
    std::string outputPath_;
    std::unique_ptr<CopcWriter> writer_;
    CopcFinalInfo final_;

    friend Status ddbt::copc::build::BuildCopc(const std::string&, const std::string&, const CopcBuildOptions&,
                                               CopcBuildReport*);
};

}  // namespace

namespace {
std::atomic<uint64_t> g_defaultBudgetOverride{0};
}  // namespace

void SetDefaultCopcMemoryBudget(uint64_t bytes) noexcept {
    g_defaultBudgetOverride.store(bytes, std::memory_order_relaxed);
}

Expected<uint64_t> DefaultCopcMemoryBudget() {
    if (const uint64_t set = g_defaultBudgetOverride.load(std::memory_order_relaxed); set != 0) return set;
    if (const char* env = std::getenv("DDBT_COPC_BUILD_MEM_MB"); env != nullptr && *env != '\0') {
        uint64_t mb = 0;
        const char* end = env + std::strlen(env);
        const auto [ptr, ec] = std::from_chars(env, end, mb);
        if (ec != std::errc() || ptr != end || mb == 0 || mb > (uint64_t{1} << 40))
            return InvalidArgument(std::string("DDBT_COPC_BUILD_MEM_MB is not a positive integer: ") + env);
        return mb << 20;
    }
    const uint64_t total = mem::PhysicalMemoryBytes();
    return total ? std::min(kDefaultBudget, total / 2) : kDefaultBudget;
}

Status BuildCopc(const std::string& input, const std::string& output, const CopcBuildOptions& options,
                 CopcBuildReport* report) {
    CopcBuildReport local;
    CopcBuildReport& r = report ? *report : local;
    r = CopcBuildReport();
    std::vector<std::string> inputs{input};
    inputs.insert(inputs.end(), options.extraInputs.begin(), options.extraInputs.end());
    for (const std::string& in : inputs)
        if (in == output) return InvalidArgument("the output is one of the inputs: " + output);
    if (options.tuneAllocator) TuneAllocator();
    Builder b(options, r);
    DDBT_TRY(b.Prepare(output));
    return b.RunAll(inputs);
}

}  // namespace ddbt::copc::build
