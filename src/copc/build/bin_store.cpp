// SPDX-License-Identifier: MPL-2.0
#include "copc/build/bin_store.h"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <mutex>

#include "core/io/platform.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace ddbt::copc::build {

namespace {

// ---------------------------------------------------------------- memory

class MemoryStore final : public BinStore {
public:
    MemoryStore(size_t bins, size_t rs) : bins_(bins), rs_(rs) {}

    class MemWriter final : public Writer {
    public:
        explicit MemWriter(MemoryStore& s) : s_(s) {}
        Status Append(uint32_t bin, const uint8_t* records, size_t count) override {
            s_.Append(bin, records, count);
            return Status();
        }
        Status Flush() override { return Status(); }

    private:
        MemoryStore& s_;
    };

    std::unique_ptr<Writer> NewWriter() override { return std::make_unique<MemWriter>(*this); }
    uint64_t Count(uint32_t bin) const override { return bins_[bin].count; }

    Status Load(std::span<const uint32_t> bins, LoadedRecords& out) override {
        out = LoadedRecords();
        for (uint32_t b : bins) AddPieces(b, out);
        return Status();
    }

    void Release(std::span<const uint32_t> bins) override {
        for (uint32_t b : bins) {
            Bin& bin = bins_[b];
            std::lock_guard lock(bin.mu);
            for (const Block& blk : bin.blocks) bytes_.fetch_sub(blk.cap, std::memory_order_relaxed);
            std::vector<Block>().swap(bin.blocks);
        }
    }

    uint64_t MemoryBytes() const noexcept override { return bytes_.load(std::memory_order_relaxed); }

    void Append(uint32_t b, const uint8_t* records, size_t count) {
        Bin& bin = bins_[b];
        std::lock_guard lock(bin.mu);
        size_t bytes = count * rs_;
        while (bytes > 0) {
            if (bin.blocks.empty() || bin.blocks.back().used == bin.blocks.back().cap) {
                const uint64_t have = bin.count * rs_;
                size_t want = static_cast<size_t>(std::clamp<uint64_t>(have / 4, 64u << 10, 8u << 20));
                want = std::max(rs_, want / rs_ * rs_);
                Block blk;
                blk.data.reset(new uint8_t[want]);
                blk.cap = want;
                bytes_.fetch_add(want, std::memory_order_relaxed);
                bin.blocks.push_back(std::move(blk));
            }
            Block& blk = bin.blocks.back();
            const size_t n = std::min(bytes, blk.cap - blk.used);
            std::memcpy(blk.data.get() + blk.used, records, n);
            blk.used += n;
            records += n;
            bytes -= n;
        }
        bin.count += count;
    }

    /// Adds the blocks of bin `b` to `out.pieces`.
    void AddPieces(uint32_t b, LoadedRecords& out) const {
        for (const Block& blk : bins_[b].blocks)
            if (blk.used) out.pieces.emplace_back(blk.data.get(), blk.used);
        out.count += bins_[b].count;
    }

private:
    struct Block {
        std::unique_ptr<uint8_t[]> data;
        size_t used = 0, cap = 0;
    };
    struct Bin {
        std::mutex mu;
        std::vector<Block> blocks;
        uint64_t count = 0;
    };

    std::vector<Bin> bins_;
    size_t rs_;
    std::atomic<uint64_t> bytes_{0};
};

// ---------------------------------------------------------------- spill

class SpillFile {
public:
    static Expected<std::unique_ptr<SpillFile>> Create(const std::string& dir) {
        std::unique_ptr<SpillFile> f(new SpillFile());
        static std::atomic<uint32_t> serial{0};
#ifdef _WIN32
        const std::string path = dir + "\\ddbt-copc-spill." + std::to_string(GetCurrentProcessId()) + "." +
                                 std::to_string(serial.fetch_add(1)) + ".tmp";
        DDBT_ASSIGN_OR_RETURN(std::wstring w, io::detail::ToWidePath(path));
        HANDLE h = CreateFileW(w.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, CREATE_NEW,
                               FILE_ATTRIBUTE_TEMPORARY | FILE_FLAG_DELETE_ON_CLOSE, nullptr);
        if (h == INVALID_HANDLE_VALUE) return Win32Status("create " + path, GetLastError());
        f->h_ = h;
#else
        const std::string path = dir + "/ddbt-copc-spill." + std::to_string(getpid()) + "." +
                                 std::to_string(serial.fetch_add(1)) + ".tmp";
        const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
        if (fd < 0) return ErrnoStatus("create " + path, errno);
        ::unlink(path.c_str());
        f->h_ = fd;
#endif
        f->path_ = path;
        return f;
    }
    ~SpillFile() {
        if (h_ != io::detail::kInvalidHandle) io::detail::CloseHandle(h_);
    }
    Status Write(uint64_t off, const uint8_t* p, size_t n) { return io::detail::WriteAt(h_, off, p, n, path_); }
    Status Read(uint64_t off, uint8_t* p, size_t n) {
        size_t got = 0;
        DDBT_TRY(io::detail::ReadAt(h_, off, p, n, &got, path_));
        if (got != n) return IoError("spill file: short read");
#if !defined(_WIN32) && defined(POSIX_FADV_DONTNEED)
        ::posix_fadvise(h_, static_cast<off_t>(off), static_cast<off_t>(n), POSIX_FADV_DONTNEED);
#endif
        return Status();
    }

private:
    SpillFile() = default;
    io::detail::NativeHandle h_ = io::detail::kInvalidHandle;
    std::string path_;
};

class SpillStore final : public BinStore {
public:
    SpillStore(std::unique_ptr<SpillFile> f, size_t bins, size_t rs, size_t staging, std::vector<uint8_t> inMemory)
        : file_(std::move(f)),
          bins_(bins),
          rs_(rs),
          staging_(std::max(rs, staging / rs * rs)),
          inMemory_(std::move(inMemory)),
          mem_(bins, rs) {
        inMemory_.resize(bins, 0);
    }

    struct Segment {
        uint64_t offset;
        uint64_t count;
    };

    /// Writers share one staging buffer per bin (under the bin's mutex), so the segments are
    /// as large as the budget allows whatever the number of threads.
    class SpillWriter final : public Writer {
    public:
        explicit SpillWriter(SpillStore& s) : s_(s) {}
        Status Append(uint32_t bin, const uint8_t* records, size_t count) override {
            if (s_.inMemory_[bin]) {
                s_.mem_.Append(bin, records, count);
                return Status();
            }
            return s_.Stage(bin, records, count);
        }
        Status Flush() override { return s_.FlushStaged(); }

    private:
        SpillStore& s_;
    };

    std::unique_ptr<Writer> NewWriter() override { return std::make_unique<SpillWriter>(*this); }
    uint64_t Count(uint32_t bin) const override { return inMemory_[bin] ? mem_.Count(bin) : bins_[bin].count; }

    Status Load(std::span<const uint32_t> bins, LoadedRecords& out) override {
        out = LoadedRecords();
        uint64_t spilled = 0;
        for (uint32_t b : bins)
            if (!inMemory_[b]) spilled += bins_[b].count;
        out.owned.resize(static_cast<size_t>(spilled * rs_));
        uint8_t* w = out.owned.data();
        for (uint32_t b : bins) {
            if (inMemory_[b]) continue;
            for (const Segment& s : bins_[b].segments) {
                DDBT_TRY(file_->Read(s.offset, w, static_cast<size_t>(s.count * rs_)));
                w += s.count * rs_;
            }
        }
        if (spilled) out.pieces.emplace_back(out.owned.data(), out.owned.size());
        out.count = spilled;
        for (uint32_t b : bins)
            if (inMemory_[b]) mem_.AddPieces(b, out);
        return Status();
    }
    void Release(std::span<const uint32_t> bins) override {
        for (uint32_t b : bins) {
            if (inMemory_[b]) mem_.Release(std::span<const uint32_t>(&b, 1));
            else std::vector<Segment>().swap(bins_[b].segments);
        }
    }
    uint64_t SpilledBytes() const noexcept override { return end_.load(std::memory_order_relaxed); }
    uint64_t MemoryBytes() const noexcept override {
        return memBytes_.load(std::memory_order_relaxed) + mem_.MemoryBytes();
    }

private:
    struct Bin {
        std::mutex mu;
        std::vector<uint8_t> stage;
        std::vector<Segment> segments;
        uint64_t count = 0;
    };

    Status Stage(uint32_t bin, const uint8_t* records, size_t count) {
        Bin& b = bins_[bin];
        size_t bytes = count * rs_;
        while (bytes > 0) {
            std::vector<uint8_t> full;
            size_t n;
            {
                std::lock_guard lock(b.mu);
                if (b.stage.capacity() == 0) {
                    b.stage.reserve(staging_);
                    memBytes_.fetch_add(b.stage.capacity(), std::memory_order_relaxed);
                }
                n = std::min(bytes, staging_ - b.stage.size());
                b.stage.insert(b.stage.end(), records, records + n);
                b.count += n / rs_;
                if (b.stage.size() == staging_) full.swap(b.stage);
            }
            records += n;
            bytes -= n;
            if (!full.empty()) {
                Status st = WriteSegment(bin, full.data(), full.size());
                memBytes_.fetch_sub(full.capacity(), std::memory_order_relaxed);
                DDBT_TRY(st);
            }
        }
        return Status();
    }

    Status FlushStaged() {
        for (uint32_t i = 0; i < bins_.size(); ++i) {
            std::vector<uint8_t> part;
            {
                std::lock_guard lock(bins_[i].mu);
                part.swap(bins_[i].stage);
            }
            if (part.capacity() == 0) continue;
            Status st = part.empty() ? Status() : WriteSegment(i, part.data(), part.size());
            memBytes_.fetch_sub(part.capacity(), std::memory_order_relaxed);
            DDBT_TRY(st);
        }
        return Status();
    }

    Status WriteSegment(uint32_t bin, const uint8_t* p, size_t bytes) {
        const uint64_t off = end_.fetch_add(bytes, std::memory_order_relaxed);
        DDBT_TRY(file_->Write(off, p, bytes));
        Bin& b = bins_[bin];
        std::lock_guard lock(b.mu);
        b.segments.push_back({off, bytes / rs_});
        return Status();
    }

    std::unique_ptr<SpillFile> file_;
    std::vector<Bin> bins_;
    size_t rs_;
    size_t staging_;
    std::vector<uint8_t> inMemory_;  // bins kept in memory (mem_)
    MemoryStore mem_;
    std::atomic<uint64_t> end_{0};
    std::atomic<uint64_t> memBytes_{0};
};

}  // namespace

std::unique_ptr<BinStore> BinStore::CreateMemory(size_t bins, size_t recordSize) {
    return std::make_unique<MemoryStore>(bins, recordSize);
}

Expected<std::unique_ptr<BinStore>> BinStore::CreateSpill(const std::string& directory, size_t bins, size_t recordSize,
                                                          size_t stagingBytes, std::vector<uint8_t> inMemory) {
    DDBT_ASSIGN_OR_RETURN(std::unique_ptr<SpillFile> f, SpillFile::Create(directory));
    return std::unique_ptr<BinStore>(
        std::make_unique<SpillStore>(std::move(f), bins, recordSize, stagingBytes, std::move(inMemory)));
}

}  // namespace ddbt::copc::build
