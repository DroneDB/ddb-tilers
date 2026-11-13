// SPDX-License-Identifier: MPL-2.0
// ddbt-bench tile: closed-loop tile request runner over a dlopen()ed backend.
#include "common.hpp"
#include "json.hpp"
#include "os.hpp"
#include "xxh64.hpp"

#include <atomic>
#include <barrier>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>

namespace fs = std::filesystem;

namespace ddbtb
{
namespace
{

using Clock = std::chrono::steady_clock;

// DDBMemoryTileFmt / ddbtMemoryTileFmt. DDBErr and ddbtErr are 4-byte enums: int.
using TileFmtFn = int (*)(const char *, int, int, int, uint8_t **, int *, int, bool, bool, const char *,
                          const char *);
using FreeFn = int (*)(void *);
using StrFn = const char *(*)();
using RegisterFn = int (*)(bool);

struct Backend
{
    std::string kind, lib;
    void *handle = nullptr;
    TileFmtFn tile = nullptr;
    FreeFn release = nullptr;
    StrFn lastError = nullptr;
    StrFn version = nullptr;
    double initMs = 0;

    void load(const std::string &k, const std::string &path)
    {
        kind = k;
        lib = path;
        std::string err;
        handle = os::libOpen(path, err);
        if (!handle)
            throw std::runtime_error("cannot load " + path + ": " + err);
        auto sym = [&](const char *name, bool required) {
            void *s = os::libSym(handle, name);
            if (!s && required)
                throw std::runtime_error(path + " does not export " + name);
            return s;
        };
        const auto t0 = Clock::now();
        if (kind == "legacy")
        {
            tile = reinterpret_cast<TileFmtFn>(sym("DDBMemoryTileFmt", true));
            release = reinterpret_cast<FreeFn>(sym("DDBVSIFree", true));
            lastError = reinterpret_cast<StrFn>(sym("DDBGetLastError", false));
            version = reinterpret_cast<StrFn>(sym("DDBGetVersion", false));
            auto reg = reinterpret_cast<RegisterFn>(sym("DDBRegisterProcess", true));
            if (reg(false) != 0)
                throw std::runtime_error(std::string("DDBRegisterProcess failed: ") +
                                         (lastError ? lastError() : "?"));
        }
        else if (kind == "ddbt")
        {
            tile = reinterpret_cast<TileFmtFn>(sym("ddbtMemoryTileFmt", true));
            release = reinterpret_cast<FreeFn>(sym("ddbtFree", true));
            lastError = reinterpret_cast<StrFn>(sym("ddbtLastError", false));
            version = reinterpret_cast<StrFn>(sym("ddbtVersion", false));
        }
        else
            throw UsageError("--backend must be legacy or ddbt");
        initMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    }
};

struct Request
{
    std::string path, format, pathHash;
    int z = 0, x = 0, y = 0, size = 256;
    bool tms = true;
};

std::vector<Request> loadRequests(const std::string &file, const Args &a)
{
    std::ifstream in(file);
    if (!in)
        throw std::runtime_error("cannot read " + file);
    std::vector<Request> r;
    std::string line;
    int lineNo = 0;
    const std::string fmt = a.get("format");
    const long long size = a.getInt("size", 0);
    while (std::getline(in, line))
    {
        ++lineNo;
        size_t i = line.find_first_not_of(" \t\r");
        if (i == std::string::npos || line[i] == '#')
            continue;
        JsonValue v;
        std::string err;
        if (!parseJson(line, v, err) || v.type != JsonValue::Type::Object)
            throw std::runtime_error(file + ":" + std::to_string(lineNo) + ": " + (err.empty() ? "not an object" : err));
        Request q;
        q.path = v.str("path");
        if (q.path.empty() || !v.get("z") || !v.get("x") || !v.get("y"))
            throw std::runtime_error(file + ":" + std::to_string(lineNo) + ": path, z, x and y are required");
        q.z = static_cast<int>(v.num("z", 0));
        q.x = static_cast<int>(v.num("x", 0));
        q.y = static_cast<int>(v.num("y", 0));
        q.size = static_cast<int>(v.num("size", 256));
        q.format = v.str("format", "png");
        q.tms = v.boolean("tms", true);
        q.pathHash = v.str("hash", v.str("pathHash"));
        if (!fmt.empty())
            q.format = fmt;
        if (size > 0)
            q.size = static_cast<int>(size);
        r.push_back(std::move(q));
    }
    const long long limit = a.getInt("limit", 0);
    if (limit > 0 && static_cast<size_t>(limit) < r.size())
        r.resize(static_cast<size_t>(limit));
    return r;
}

// Per-round measurements, preallocated before the run.
struct RoundData
{
    bool warmup = false;
    std::vector<int64_t> latNs;
    std::vector<int32_t> bytes; // -1: failed
    std::vector<uint64_t> hash;
    double wallS = 0;
    os::ResUsage u0, u1;
    int64_t rssPeakKb = 0, rssMeanKb = 0, hwmEndKb = 0;
    std::map<std::string, int64_t> errors;
};

// RSS sampler: VmRSS/VmHWM of this process every period.
class RssSampler
{
  public:
    explicit RssSampler(int periodMs) : period_(periodMs)
    {
        th_ = std::thread([this] { loop(); });
    }
    ~RssSampler()
    {
        stop_ = true;
        th_.join();
    }
    void reset()
    {
        std::lock_guard<std::mutex> l(m_);
        peak_ = 0;
        sum_ = 0;
        count_ = 0;
        sampleNow();
    }
    void read(int64_t &peak, int64_t &mean)
    {
        std::lock_guard<std::mutex> l(m_);
        sampleNow();
        peak = peak_;
        mean = count_ ? sum_ / count_ : 0;
    }

  private:
    void sampleNow()
    {
        const os::MemNow m = os::selfMem();
        if (m.rss_kb >= 0)
        {
            peak_ = std::max(peak_, m.rss_kb);
            sum_ += m.rss_kb;
            ++count_;
        }
    }
    void loop()
    {
        while (!stop_)
        {
            {
                std::lock_guard<std::mutex> l(m_);
                sampleNow();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(period_));
        }
    }
    int period_;
    std::atomic<bool> stop_{false};
    std::mutex m_;
    int64_t peak_ = 0, sum_ = 0, count_ = 0;
    std::thread th_;
};

std::string extFor(const std::string &fmt)
{
    if (fmt == "jpeg" || fmt == "jpg" || fmt == "image/jpeg")
        return "jpg";
    if (fmt == "webp" || fmt == "image/webp")
        return "webp";
    return "png";
}

void writeLatency(JsonWriter &w, const char *name, std::vector<int64_t> lat)
{
    std::sort(lat.begin(), lat.end());
    double sum = 0;
    for (int64_t v : lat)
        sum += static_cast<double>(v);
    const double ms = 1e-6;
    w.key(name).beginObject(true);
    w.kv("n", static_cast<int64_t>(lat.size()));
    if (!lat.empty())
    {
        w.kv("mean", sum / static_cast<double>(lat.size()) * ms);
        w.kv("min", static_cast<double>(lat.front()) * ms);
        w.kv("p50", percentileSorted(lat, 0.50) * ms);
        w.kv("p90", percentileSorted(lat, 0.90) * ms);
        w.kv("p95", percentileSorted(lat, 0.95) * ms);
        w.kv("p99", percentileSorted(lat, 0.99) * ms);
        w.kv("p999", percentileSorted(lat, 0.999) * ms);
        w.kv("max", static_cast<double>(lat.back()) * ms);
    }
    w.endObject();
}

const char *usage = R"(usage: ddbt-bench tile --backend legacy|ddbt --lib <lib.so> --requests <file.jsonl>
         [--threads N] [--pin CPULIST] [--schedule shared|static] [--warmup-rounds K]
         [--rounds R] [--format png|jpeg|webp] [--size 256|512] [--limit N]
         [--dump-tiles DIR] [--hashes] [--latencies] [--rss-period-ms 10]
         [--clean-exit] --out <file.json|->
Requests (JSON lines): {"path","z","x","y","size":256,"format":"png","tms":true[,"hash"]}
Each round runs every request once on N closed-loop workers. Worker i is pinned to the
i-th CPU of --pin (modulo its length). --dump-tiles writes the tiles of the first measured
round as DIR/<index>_<z>_<x>_<y>.<ext> plus DIR/index.jsonl.)";

} // namespace

int cmdTile(int argc, char **argv)
{
    Args a(argc, argv, 2, {"hashes", "latencies", "clean-exit", "help"});
    if (a.has("help"))
    {
        std::puts(usage);
        return 0;
    }
    a.allowOnly({"backend", "lib", "requests", "threads", "pin", "schedule", "warmup-rounds", "rounds", "format",
                 "size", "limit", "dump-tiles", "hashes", "latencies", "rss-period-ms", "clean-exit", "out"});
    const std::string backendKind = a.require("backend");
    const std::string libPath = a.require("lib");
    const std::string reqFile = a.require("requests");
    const std::string outPath = a.require("out");
    const int threads = static_cast<int>(a.getInt("threads", 1));
    const int warmupRounds = static_cast<int>(a.getInt("warmup-rounds", 0));
    const int rounds = static_cast<int>(a.getInt("rounds", 1));
    const std::vector<int> pin = parseCpuList(a.get("pin"));
    const std::string schedule = a.get("schedule", "shared");
    const std::string dumpDir = a.get("dump-tiles");
    const bool wantHashes = a.has("hashes") || !dumpDir.empty();
    const bool wantLat = a.has("latencies");
    if (threads < 1 || rounds < 1 || warmupRounds < 0)
        throw UsageError("--threads and --rounds must be >= 1, --warmup-rounds >= 0");
    if (schedule != "shared" && schedule != "static")
        throw UsageError("--schedule must be shared or static");

    const std::vector<Request> reqs = loadRequests(reqFile, a);
    if (reqs.empty())
        throw std::runtime_error("no requests in " + reqFile);
    const size_t n = reqs.size();

    RssSampler sampler(static_cast<int>(a.getInt("rss-period-ms", 10)));
    const os::MemNow memBeforeLoad = os::selfMem();

    Backend be;
    be.load(backendKind, libPath);
    const os::MemNow memAfterInit = os::selfMem();

    const int totalRounds = warmupRounds + rounds;
    std::vector<RoundData> rd(static_cast<size_t>(totalRounds));
    for (int r = 0; r < totalRounds; ++r)
    {
        rd[r].warmup = r < warmupRounds;
        rd[r].latNs.assign(n, 0);
        rd[r].bytes.assign(n, -1);
        rd[r].hash.assign(n, 0);
    }
    std::vector<std::string> dumps(dumpDir.empty() ? 0 : n);

    std::atomic<size_t> next{0};
    std::barrier sync(threads + 1);
    std::vector<std::map<std::string, int64_t>> workerErrors(static_cast<size_t>(threads));
    std::vector<std::string> pinErrors(static_cast<size_t>(threads));

    const bool shared = schedule == "shared";
    auto worker = [&](int wi) {
        if (!pin.empty())
            os::pinCurrentThread(pin[static_cast<size_t>(wi) % pin.size()], pinErrors[wi]);
        for (int r = 0; r < totalRounds; ++r)
        {
            sync.arrive_and_wait(); // start of round
            RoundData &d = rd[static_cast<size_t>(r)];
            const bool dumpThis = !dumps.empty() && r == warmupRounds;
            size_t staticI = static_cast<size_t>(wi);
            for (;;)
            {
                size_t i;
                if (shared)
                    i = next.fetch_add(1, std::memory_order_relaxed);
                else
                {
                    i = staticI;
                    staticI += static_cast<size_t>(threads);
                }
                if (i >= n)
                    break;
                const Request &q = reqs[i];
                uint8_t *buf = nullptr;
                int size = 0;
                const auto t0 = Clock::now();
                const int rc = be.tile(q.path.c_str(), q.z, q.x, q.y, &buf, &size, q.size, q.tms, false,
                                       q.pathHash.empty() ? nullptr : q.pathHash.c_str(), q.format.c_str());
                const auto t1 = Clock::now();
                d.latNs[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
                if (rc == 0 && buf)
                {
                    d.bytes[i] = size;
                    d.hash[i] = Xxh64::hash(buf, static_cast<size_t>(size));
                    if (dumpThis)
                        dumps[i].assign(reinterpret_cast<const char *>(buf), static_cast<size_t>(size));
                    be.release(buf);
                }
                else
                {
                    std::string e = "error " + std::to_string(rc);
                    if (rc == 0)
                        e = "no buffer returned";
                    else if (be.lastError)
                        if (const char *m = be.lastError(); m && *m)
                            e += ": " + std::string(m);
                    ++workerErrors[static_cast<size_t>(wi)][e];
                    if (buf)
                        be.release(buf);
                }
            }
            sync.arrive_and_wait(); // end of round
        }
    };

    std::vector<std::thread> pool;
    for (int t = 0; t < threads; ++t)
        pool.emplace_back(worker, t);

    const auto runT0 = Clock::now();
    for (int r = 0; r < totalRounds; ++r)
    {
        RoundData &d = rd[static_cast<size_t>(r)];
        next.store(0);
        sampler.reset();
        d.u0 = os::selfUsage();
        const auto t0 = Clock::now();
        sync.arrive_and_wait();
        sync.arrive_and_wait();
        const auto t1 = Clock::now();
        d.u1 = os::selfUsage();
        d.wallS = std::chrono::duration<double>(t1 - t0).count();
        sampler.read(d.rssPeakKb, d.rssMeanKb);
        d.hwmEndKb = os::selfMem().hwm_kb;
        for (auto &we : workerErrors)
        {
            for (auto &[k, v] : we)
                d.errors[k] += v;
            we.clear();
        }
    }
    const double totalS = std::chrono::duration<double>(Clock::now() - runT0).count();
    for (auto &t : pool)
        t.join();

    // ---- hashes across rounds (determinism) and of the whole set
    const RoundData &first = rd[static_cast<size_t>(warmupRounds)];
    int64_t mismatches = 0;
    for (size_t i = 0; i < n; ++i)
        for (const auto &d : rd)
            if (d.bytes[i] >= 0 && first.bytes[i] >= 0 && d.hash[i] != first.hash[i])
            {
                ++mismatches;
                break;
            }
    const uint64_t setHash = Xxh64::hash(first.hash.data(), n * sizeof(uint64_t));

    // ---- dump tiles
    if (!dumps.empty())
    {
        fs::create_directories(dumpDir);
        std::string index;
        for (size_t i = 0; i < n; ++i)
        {
            const Request &q = reqs[i];
            char name[128];
            std::snprintf(name, sizeof name, "%06zu_%d_%d_%d.%s", i, q.z, q.x, q.y, extFor(q.format).c_str());
            if (first.bytes[i] >= 0)
            {
                std::ofstream f(fs::path(dumpDir) / name, std::ios::binary);
                f.write(dumps[i].data(), static_cast<std::streamsize>(dumps[i].size()));
                if (!f)
                    throw std::runtime_error(std::string("cannot write tile ") + name);
            }
            std::string line;
            JsonWriter w(line);
            w.beginObject(true)
                .kv("i", static_cast<int64_t>(i))
                .kv("file", first.bytes[i] >= 0 ? std::string(name) : std::string())
                .kv("path", q.path)
                .kv("z", q.z)
                .kv("x", q.x)
                .kv("y", q.y)
                .kv("size", q.size)
                .kv("format", q.format)
                .kv("ok", first.bytes[i] >= 0)
                .kv("bytes", static_cast<int64_t>(first.bytes[i]))
                .kv("hash", JsonWriter::hex64(first.hash[i]))
                .endObject();
            index += line;
        }
        writeOutput((fs::path(dumpDir) / "index.jsonl").string(), index);
    }

    // ---- JSON
    std::string out;
    JsonWriter w(out);
    w.beginObject();
    w.kv("schema", "ddbt-bench-tile/1");
    w.kv("backend", be.kind).kv("lib", be.lib);
    w.kv("lib_version", be.version ? be.version() : "");
    w.kv("requests_file", reqFile).kv("n_requests", static_cast<int64_t>(n));
    w.kv("threads", threads).kv("schedule", schedule);
    w.key("pin").beginArray(true);
    for (int c : pin)
        w.value(c);
    w.endArray();
    {
        std::string pe;
        for (const auto &e : pinErrors)
            if (!e.empty())
                pe += (pe.empty() ? "" : "; ") + e;
        if (!pe.empty())
            w.kv("pin_errors", pe);
    }
    w.kv("warmup_rounds", warmupRounds).kv("rounds", rounds);
    w.kv("init_ms", be.initMs);
    w.key("rss_kb").beginObject(true);
    w.kv("before_load", memBeforeLoad.rss_kb).kv("after_init", memAfterInit.rss_kb);
    w.kv("hwm_end", os::selfMem().hwm_kb);
    w.endObject();
    w.kv("total_s", totalS);
    w.kv("set_hash", JsonWriter::hex64(setHash));
    w.kv("hash_mismatches", mismatches);

    w.key("round_results").beginArray();
    for (int r = 0; r < totalRounds; ++r)
    {
        const RoundData &d = rd[static_cast<size_t>(r)];
        int64_t bytes = 0, failures = 0, ok = 0;
        for (size_t i = 0; i < n; ++i)
        {
            if (d.bytes[i] >= 0)
            {
                bytes += d.bytes[i];
                ++ok;
            }
            else
                ++failures;
        }
        w.beginObject();
        w.kv("round", r).kv("warmup", d.warmup);
        w.kv("wall_s", d.wallS);
        w.kv("tiles", static_cast<int64_t>(n)).kv("ok", ok).kv("failures", failures);
        w.kv("tiles_per_s", d.wallS > 0 ? static_cast<double>(n) / d.wallS : 0.0);
        w.kv("bytes", bytes);
        writeLatency(w, "latency_ms", d.latNs); // every request (failures are served requests too)
        {
            std::vector<int64_t> okLat;
            okLat.reserve(n);
            for (size_t i = 0; i < n; ++i)
                if (d.bytes[i] >= 0)
                    okLat.push_back(d.latNs[i]);
            writeLatency(w, "latency_ok_ms", std::move(okLat)); // successful requests only
        }
        w.key("cpu").beginObject(true);
        w.kv("user_s", d.u1.user_s - d.u0.user_s).kv("sys_s", d.u1.sys_s - d.u0.sys_s);
        w.kv("utilization", d.wallS > 0 ? (d.u1.user_s - d.u0.user_s + d.u1.sys_s - d.u0.sys_s) / d.wallS : 0.0);
        w.endObject();
        w.key("rusage").beginObject(true);
        w.kv("minflt", d.u1.minflt - d.u0.minflt).kv("majflt", d.u1.majflt - d.u0.majflt);
        w.kv("inblock", d.u1.inblock - d.u0.inblock).kv("oublock", d.u1.oublock - d.u0.oublock);
        w.kv("nvcsw", d.u1.nvcsw - d.u0.nvcsw).kv("nivcsw", d.u1.nivcsw - d.u0.nivcsw);
        w.endObject();
        w.key("rss_kb").beginObject(true);
        w.kv("peak", d.rssPeakKb).kv("mean", d.rssMeanKb).kv("hwm", d.hwmEndKb);
        w.endObject();
        // Distinct error texts, most frequent first, at most 20 (messages may embed coordinates).
        {
            std::vector<std::pair<std::string, int64_t>> errs(d.errors.begin(), d.errors.end());
            std::stable_sort(errs.begin(), errs.end(), [](const auto &x, const auto &y) { return x.second > y.second; });
            int64_t other = 0;
            w.key("errors").beginObject(true);
            for (size_t e = 0; e < errs.size(); ++e)
            {
                if (e < 20)
                    w.kv(errs[e].first, errs[e].second);
                else
                    other += errs[e].second;
            }
            if (other)
                w.kv("(other errors)", other);
            w.endObject();
        }
        if (wantLat)
        {
            w.key("latency_us").beginArray(true);
            for (int64_t v : d.latNs)
                w.value(static_cast<double>(v / 100) / 10.0);
            w.endArray();
        }
        w.endObject();
    }
    w.endArray();

    if (wantHashes)
    {
        w.key("requests").beginArray();
        for (size_t i = 0; i < n; ++i)
        {
            const Request &q = reqs[i];
            w.beginObject(true);
            w.kv("i", static_cast<int64_t>(i)).kv("z", q.z).kv("x", q.x).kv("y", q.y);
            w.kv("ok", first.bytes[i] >= 0).kv("bytes", static_cast<int64_t>(first.bytes[i]));
            w.kv("hash", JsonWriter::hex64(first.hash[i]));
            w.endObject();
        }
        w.endArray();
    }
    w.endObject();
    writeOutput(outPath, out);

    // Skip the backend's static destructors (GDAL/PROJ teardown is not what is measured and
    // can be slow or noisy) unless asked.
    if (!a.has("clean-exit"))
    {
        std::fflush(nullptr);
        std::_Exit(0);
    }
    return 0;
}

} // namespace ddbtb
