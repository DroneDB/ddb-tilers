// SPDX-License-Identifier: MPL-2.0
// pread vs mmap throughput on a 1 GiB file (page cache warm), random and sequential.
//
// The file is created on first use in $DDBT_BENCH_TMP, else /mnt/data/ddb-tilers-work/tmp when
// it exists, else the system temp directory, and deleted at exit. Cold-cache scans (the file's
// pages dropped with posix_fadvise before each pass) only run with DDBT_BENCH_COLD=1, as they
// hit the disk and would disturb other measurements on the host.
#include <benchmark/benchmark.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include "core/exec/topology.h"
#include "core/io/byte_source.h"
#include "core/io/file.h"
#include "core/io/out_file.h"

namespace {

namespace fs = std::filesystem;
using namespace ddbt::io;

constexpr uint64_t kFileSize = uint64_t{1} << 30;

class BenchFile {
public:
    static BenchFile& Get() {
        static BenchFile f;
        return f;
    }
    const std::string& path() const { return path_; }
    ~BenchFile() {
        std::error_code ec;
        if (!path_.empty()) fs::remove(path_, ec);
    }

private:
    BenchFile() {
        fs::path dir;
        if (const char* env = std::getenv("DDBT_BENCH_TMP"); env != nullptr && *env != '\0') dir = env;
        else if (fs::is_directory("/mnt/data/ddb-tilers-work/tmp")) dir = "/mnt/data/ddb-tilers-work/tmp";
        else dir = fs::temp_directory_path();
        path_ = (dir / ("ddbt_io_bench_" + std::to_string(static_cast<unsigned long long>(
                                                std::hash<std::string>{}(dir.string()) ^ std::random_device{}())) +
                        ".bin"))
                    .string();
        auto out = OutFile::Create(path_);
        if (!out.ok()) std::abort();
        std::vector<uint8_t> chunk(size_t{8} << 20);
        uint64_t s = 0x9E3779B97F4A7C15ULL;
        for (uint64_t off = 0; off < kFileSize; off += chunk.size()) {
            for (size_t i = 0; i < chunk.size(); i += 8) {
                s ^= s << 13;
                s ^= s >> 7;
                s ^= s << 17;
                std::memcpy(chunk.data() + i, &s, 8);
            }
            if (!out->Append(chunk).ok()) std::abort();
        }
        if (!out->Finish().ok()) std::abort();
    }
    std::string path_;
};

// One source per kind, shared by the benchmark threads (sources are thread-safe).
ByteSource& Source(int kind) {
    static std::mutex m;
    static std::unique_ptr<ByteSource> sources[2];
    std::lock_guard lock(m);
    if (!sources[kind]) {
        auto s = OpenByteSource(BenchFile::Get().path(), kind == 0 ? SourceKind::kPread : SourceKind::kMmap);
        if (!s.ok()) std::abort();
        sources[kind] = std::move(*s);
    }
    return *sources[kind];
}

void RandomArgs(benchmark::Benchmark* b) {
    const int max = static_cast<int>(ddbt::exec::AllowedCpus().size());
    for (int kind : {0, 1}) {
        for (int64_t block : {int64_t{4} << 10, int64_t{64} << 10, int64_t{1} << 20}) b->Args({kind, block});
    }
    b->ArgNames({"mmap", "block"});
    b->Threads(1)->Threads(8 < max ? 8 : max)->Threads(max);
}

// Reads one block at a random block-aligned offset into a private buffer per iteration.
void BM_Io_RandomRead(benchmark::State& state) {
    const int kind = static_cast<int>(state.range(0));
    const auto block = static_cast<size_t>(state.range(1));
    ByteSource& src = Source(kind);
    std::vector<uint8_t> buf(block);
    const uint64_t blocks = kFileSize / block;
    uint64_t s = 0x2545F4914F6CDD1DULL * (static_cast<uint64_t>(static_cast<unsigned>(state.thread_index())) + 1u);
    for (auto _ : state) {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        if (!src.Read((s % blocks) * block, buf).ok()) state.SkipWithError("read failed");
        benchmark::DoNotOptimize(buf.data());
    }
    state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(block));
}
BENCHMARK(BM_Io_RandomRead)->Apply(RandomArgs)->UseRealTime();

// Whole-file sequential pass in 1 MiB reads, single thread.
void BM_Io_SequentialScan(benchmark::State& state) {
    ByteSource& src = Source(static_cast<int>(state.range(0)));
    std::vector<uint8_t> buf(size_t{1} << 20);
    for (auto _ : state) {
        for (uint64_t off = 0; off < kFileSize; off += buf.size()) {
            if (!src.Read(off, buf).ok()) state.SkipWithError("read failed");
        }
        benchmark::DoNotOptimize(buf.data());
    }
    state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(kFileSize));
}
BENCHMARK(BM_Io_SequentialScan)->Arg(0)->Arg(1)->ArgName("mmap")->Unit(benchmark::kMillisecond)->UseRealTime();

// Cold: drop the file from the page cache, open a fresh source, scan it once.
void BM_Io_ColdScan(benchmark::State& state) {
    const char* enabled = std::getenv("DDBT_BENCH_COLD");
    if (enabled == nullptr || std::string(enabled) != "1") {
        state.SkipWithError("set DDBT_BENCH_COLD=1 to run (hits the disk)");
        for (auto _ : state) {
        }
        return;
    }
    const int kind = static_cast<int>(state.range(0));
    const std::string& path = BenchFile::Get().path();
    std::vector<uint8_t> buf(size_t{1} << 20);
    for (auto _ : state) {
        state.PauseTiming();
        {
            auto f = File::Open(path);
            if (!f.ok() || !f->DropCache().ok()) state.SkipWithError("cannot drop the page cache");
        }
        state.ResumeTiming();
        auto src = OpenByteSource(path, kind == 0 ? SourceKind::kPread : SourceKind::kMmap, AccessHint::kSequential);
        if (!src.ok()) state.SkipWithError("open failed");
        for (uint64_t off = 0; src.ok() && off < kFileSize; off += buf.size()) {
            if (!(*src)->Read(off, buf).ok()) state.SkipWithError("read failed");
        }
    }
    state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(kFileSize));
}
BENCHMARK(BM_Io_ColdScan)->Arg(0)->Arg(1)->ArgName("mmap")->Iterations(3)->Unit(benchmark::kMillisecond)->UseRealTime();

}  // namespace
