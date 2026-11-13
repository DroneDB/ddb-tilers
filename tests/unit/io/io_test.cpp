// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "core/io/byte_source.h"
#include "core/io/coalesce.h"
#include "core/io/file.h"
#include "core/io/io_stats.h"
#include "core/io/mapped_file.h"
#include "core/io/out_file.h"

namespace fs = std::filesystem;
using namespace ddbt::io;
using ddbt::StatusCode;

namespace {

// UTF-8 std::string <-> path (fs::u8path is deprecated in C++20).
fs::path U8(const std::string& s) { return fs::path(std::u8string(s.begin(), s.end())); }
std::string Utf8(const fs::path& p) {
    const std::u8string u = p.u8string();
    return std::string(u.begin(), u.end());
}

// Scratch directory per test, removed at the end (TEST_TMPDIR / testing::TempDir()).
class ScratchDir {
public:
    ScratchDir() {
        const auto* info = testing::UnitTest::GetInstance()->current_test_info();
        path_ = fs::path(testing::TempDir()) /
                ("ddbt_io_" + std::string(info->test_suite_name()) + "_" + info->name() + "_" +
                 std::to_string(std::random_device{}()));
        fs::create_directories(path_);
    }
    ~ScratchDir() {
        std::error_code ec;
        fs::remove_all(path_, ec);
    }
    std::string File(const std::string& name) const { return Utf8(path_ / U8(name)); }
    const fs::path& path() const { return path_; }

private:
    fs::path path_;
};

std::vector<uint8_t> Pattern(size_t n, uint32_t seed) {
    std::vector<uint8_t> v(n);
    std::mt19937 rng(seed);
    for (auto& b : v) b = static_cast<uint8_t>(rng());
    return v;
}

void WriteFile(const std::string& path, const std::vector<uint8_t>& data) {
    std::ofstream f(U8(path), std::ios::binary);
    f.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
}

std::vector<uint8_t> ReadWhole(const std::string& path) {
    std::ifstream f(U8(path), std::ios::binary);
    return std::vector<uint8_t>(std::istreambuf_iterator<char>(f), {});
}

size_t CountEntries(const fs::path& dir) {
    size_t n = 0;
    for ([[maybe_unused]] const auto& e : fs::directory_iterator(dir)) ++n;
    return n;
}

}  // namespace

TEST(File, OpenMissingIsNotFound) {
    ScratchDir dir;
    auto f = File::Open(dir.File("nope.bin"));
    ASSERT_FALSE(f.ok());
    EXPECT_EQ(f.code(), StatusCode::kNotFound);
    EXPECT_NE(f.status().message().find("nope.bin"), std::string_view::npos);
    EXPECT_EQ(MappedFile::Open(dir.File("nope.bin")).code(), StatusCode::kNotFound);
}

TEST(File, ReadsAndBounds) {
    ScratchDir dir;
    const auto data = Pattern(100000, 1);
    const std::string path = dir.File("a.bin");
    WriteFile(path, data);
    auto f = File::Open(path, AccessHint::kRandom);
    ASSERT_TRUE(f.ok()) << f.status().ToString();
    EXPECT_EQ(f->Size(), data.size());
    std::vector<uint8_t> buf(1000);
    ASSERT_TRUE(f->ReadAt(5000, buf).ok());
    EXPECT_TRUE(std::equal(buf.begin(), buf.end(), data.begin() + 5000));
    EXPECT_EQ(f->ReadAt(data.size() - 10, buf).code(), StatusCode::kOutOfRange);
    EXPECT_EQ(f->ReadAt(UINT64_MAX - 5, buf).code(), StatusCode::kOutOfRange);
    auto n = f->ReadAtMost(data.size() - 10, buf);
    ASSERT_TRUE(n.ok());
    EXPECT_EQ(*n, 10u);
    EXPECT_TRUE(f->ReadAt(data.size(), std::span<uint8_t>()).ok());
    f->Prefetch(0, data.size());
    EXPECT_TRUE(f->DropCache().ok());
    File first = std::move(*f);
    File moved = std::move(first);
    EXPECT_FALSE(first.IsOpen());
    EXPECT_TRUE(moved.IsOpen());
}

TEST(File, Utf8Path) {
    ScratchDir dir;
    const std::string path = dir.File("ortofoto_\xC3\xA5\xC3\xA4\xC3\xB6_\xE6\x97\xA5\xE6\x9C\xAC.tif");
    const auto data = Pattern(4096, 2);
    WriteFile(path, data);
    auto src = OpenByteSource(path, SourceKind::kPread);
    ASSERT_TRUE(src.ok()) << src.status().ToString();
    auto all = (*src)->ReadVector(0, data.size());
    ASSERT_TRUE(all.ok());
    EXPECT_EQ(*all, data);
}

// Random-offset reads through every ByteSource kind, from many threads, against the reference.
TEST(ByteSource, RandomOffsetsMatchReferenceConcurrently) {
    ScratchDir dir;
    const auto data = Pattern(size_t{3} << 20, 3);
    const std::string path = dir.File("ref.bin");
    WriteFile(path, data);

    std::vector<std::unique_ptr<ByteSource>> sources;
    sources.push_back(std::move(OpenByteSource(path, SourceKind::kPread).value()));
    sources.push_back(std::move(OpenByteSource(path, SourceKind::kMmap).value()));
    sources.push_back(std::make_unique<MemorySource>(std::span<const uint8_t>(data)));
    sources.push_back(std::make_unique<RangeCallbackSource>(
        data.size(),
        [&data](uint64_t off, std::span<uint8_t> out) {
            std::memcpy(out.data(), data.data() + off, out.size());
            return ddbt::Status::Ok();
        },
        "cb"));
    EXPECT_EQ(sources[0]->Data(), nullptr);
    EXPECT_NE(sources[1]->Data(), nullptr);
    EXPECT_EQ(sources[3]->Data(), nullptr);

    for (const auto& src : sources) {
        ASSERT_EQ(src->Size(), data.size());
        std::atomic<int> mismatches{0};
        std::vector<std::thread> ts;
        for (int t = 0; t < 8; ++t) {
            ts.emplace_back([&, t] {
                std::mt19937_64 rng(static_cast<uint64_t>(t) * 77 + 1);
                std::vector<uint8_t> buf;
                for (int i = 0; i < 300; ++i) {
                    const size_t len = 1 + rng() % 70000;
                    const uint64_t off = rng() % (data.size() - len + 1);
                    buf.resize(len);
                    src->Prefetch(off, len);
                    if (!src->Read(off, buf).ok() || std::memcmp(buf.data(), data.data() + off, len) != 0)
                        mismatches.fetch_add(1);
                }
            });
        }
        for (auto& th : ts) th.join();
        EXPECT_EQ(mismatches.load(), 0) << src->Name();
        std::vector<uint8_t> one(2);
        EXPECT_EQ(src->Read(data.size() - 1, one).code(), StatusCode::kOutOfRange) << src->Name();
    }
    auto view = sources[1]->View(10, 20);
    ASSERT_EQ(view.size(), 20u);
    EXPECT_EQ(view[0], data[10]);
    EXPECT_TRUE(sources[0]->View(10, 20).empty());
    EXPECT_TRUE(sources[1]->View(data.size(), 1).empty());
}

TEST(ByteSource, CallbackErrorsPropagate) {
    RangeCallbackSource src(100, [](uint64_t, std::span<uint8_t>) { return ddbt::IoError("HTTP 503"); }, "s3://x");
    std::vector<uint8_t> buf(10);
    const auto s = src.Read(0, buf);
    EXPECT_EQ(s.code(), StatusCode::kIoError);
    EXPECT_NE(s.message().find("s3://x"), std::string_view::npos);
    EXPECT_NE(s.message().find("HTTP 503"), std::string_view::npos);
    EXPECT_EQ(src.ReadVector(95, 10).code(), StatusCode::kOutOfRange);
}

TEST(MappedFile, EmptyFileAndAdvice) {
    ScratchDir dir;
    const std::string path = dir.File("empty.bin");
    WriteFile(path, {});
    auto m = MappedFile::Open(path);
    ASSERT_TRUE(m.ok());
    EXPECT_EQ(m->size(), 0u);
    EXPECT_EQ(m->data(), nullptr);
    m->Advise(MapAdvice::kWillNeed);  // no-op, no crash

    const auto data = Pattern(1 << 20, 4);
    WriteFile(path, data);
    auto m2 = MappedFile::Open(path, AccessHint::kSequential);
    ASSERT_TRUE(m2.ok());
    for (auto a : {MapAdvice::kNormal, MapAdvice::kRandom, MapAdvice::kSequential, MapAdvice::kWillNeed, MapAdvice::kDontNeed})
        m2->Advise(a, 12345, 100000);
    EXPECT_EQ(std::memcmp(m2->data(), data.data(), data.size()), 0);  // DONTNEED on a file mapping keeps contents
}

TEST(Coalesce, MergesNearbyRanges) {
    const std::vector<ByteRange> ranges = {
        {1000, 100}, {0, 100}, {150, 50}, {100000, 10}, {1100, 0}, {1050, 100}, {300000, 500000},
    };
    CoalesceOptions o;
    o.maxGap = 1000;
    o.maxLength = 400000;
    std::vector<uint32_t> order;
    const auto reads = CoalesceRanges(ranges, o, order);
    ASSERT_EQ(order.size(), 6u);  // empty range dropped
    ASSERT_EQ(reads.size(), 3u);
    EXPECT_EQ(reads[0].offset, 0u);
    EXPECT_EQ(reads[0].length, 1150u);  // 0..100, 150..200, 1000..1100, 1050..1150 (gaps <= 1000)
    EXPECT_EQ(reads[0].count, 4u);
    EXPECT_EQ(reads[1].offset, 100000u);
    EXPECT_EQ(reads[1].count, 1u);
    EXPECT_EQ(reads[2].length, 500000u);  // single range larger than maxLength: kept whole
    EXPECT_EQ(order[reads[0].first], 1u);

    o.maxLength = 1100;  // splits the first group
    EXPECT_EQ(CoalesceRanges(ranges, o, order).size(), 4u);
}

TEST(Coalesce, ReadRangesMatchesReference) {
    const auto data = Pattern(1 << 20, 5);
    std::atomic<int> calls{0};
    RangeCallbackSource remote(data.size(), [&](uint64_t off, std::span<uint8_t> out) {
        calls.fetch_add(1);
        std::memcpy(out.data(), data.data() + off, out.size());
        return ddbt::Status::Ok();
    });
    MemorySource local{std::span<const uint8_t>(data)};
    std::mt19937 rng(9);
    std::vector<ByteRange> ranges;
    for (int i = 0; i < 200; ++i) {
        const uint64_t len = 1 + rng() % 5000;
        ranges.push_back({rng() % (data.size() - len), len});
    }
    for (const ByteSource* src : {static_cast<const ByteSource*>(&remote), static_cast<const ByteSource*>(&local)}) {
        std::vector<std::vector<uint8_t>> bufs;
        std::vector<std::span<uint8_t>> outs;
        for (const auto& r : ranges) bufs.emplace_back(r.length);
        for (auto& b : bufs) outs.emplace_back(b);
        calls = 0;
        ASSERT_TRUE(ReadRanges(*src, ranges, outs).ok());
        for (size_t i = 0; i < ranges.size(); ++i)
            ASSERT_EQ(std::memcmp(bufs[i].data(), data.data() + ranges[i].offset, ranges[i].length), 0);
    }
    // 200 ranges in 1 MiB with a 64 KiB gap allowance: a handful of merged reads.
    std::vector<uint8_t> scratch;
    std::vector<std::vector<uint8_t>> bufs;
    std::vector<std::span<uint8_t>> outs;
    for (const auto& r : ranges) bufs.emplace_back(r.length);
    for (auto& b : bufs) outs.emplace_back(b);
    calls = 0;
    ASSERT_TRUE(ReadRanges(remote, ranges, outs).ok());
    EXPECT_LT(calls.load(), 5);

    std::vector<ByteRange> bad = {{data.size() - 1, 2}};
    std::vector<uint8_t> b2(2);
    std::vector<std::span<uint8_t>> o2 = {b2};
    EXPECT_EQ(ReadRanges(remote, bad, o2).code(), StatusCode::kOutOfRange);
}

TEST(OutFile, AtomicPublish) {
    ScratchDir dir;
    const std::string path = dir.File("out.tif");
    WriteFile(path, {9, 9, 9});  // existing output stays intact until Finish

    const auto a = Pattern(5000, 6);
    const auto big = Pattern(300000, 7);
    OutFileOptions o;
    o.bufferSize = 64 << 10;
    o.preallocate = 10 << 20;
    auto out = OutFile::Create(path, o);
    ASSERT_TRUE(out.ok()) << out.status().ToString();
    EXPECT_NE(out->TempPath(), path);
    EXPECT_TRUE(fs::exists(U8(out->TempPath())));
    ASSERT_TRUE(out->Append(std::span<const uint8_t>(a.data(), 16)).ok());  // header placeholder
    ASSERT_TRUE(out->Append(a).ok());
    ASSERT_TRUE(out->Append(big).ok());  // bypasses the buffer
    EXPECT_EQ(out->Tell(), 16 + a.size() + big.size());

    // Readers of the final path still see the old file.
    EXPECT_EQ(ReadWhole(path), (std::vector<uint8_t>{9, 9, 9}));

    const uint8_t header[4] = {'I', 'I', 42, 0};
    ASSERT_TRUE(out->WriteAt(0, header).ok());  // flushed region
    const uint8_t tail[3] = {1, 2, 3};
    ASSERT_TRUE(out->Append(tail).ok());
    const uint8_t patch[2] = {7, 7};
    ASSERT_TRUE(out->WriteAt(out->Tell() - 2, patch).ok());  // inside the pending buffer
    const std::string temp = out->TempPath();
    ASSERT_TRUE(out->Finish().ok());
    EXPECT_FALSE(fs::exists(U8(temp)));
    EXPECT_FALSE(out->IsOpen());

    std::vector<uint8_t> expect(a.begin(), a.begin() + 16);
    expect.insert(expect.end(), a.begin(), a.end());
    expect.insert(expect.end(), big.begin(), big.end());
    expect.insert(expect.end(), {1, 7, 7});
    std::copy(header, header + 4, expect.begin());
    EXPECT_EQ(ReadWhole(path), expect);  // also proves the preallocation was trimmed
    EXPECT_EQ(fs::file_size(U8(path)), expect.size());
    EXPECT_EQ(CountEntries(dir.path()), 1u);
}

TEST(OutFile, AbortAndDestructorRemoveTemp) {
    ScratchDir dir;
    const std::string path = dir.File("never.bin");
    {
        auto out = OutFile::Create(path);
        ASSERT_TRUE(out.ok());
        ASSERT_TRUE(out->Append(Pattern(1000, 1)).ok());
    }  // destroyed without Finish
    EXPECT_EQ(CountEntries(dir.path()), 0u);
    auto out = OutFile::Create(path);
    ASSERT_TRUE(out.ok());
    out->Abort();
    out->Abort();
    EXPECT_EQ(CountEntries(dir.path()), 0u);
    EXPECT_EQ(out->Append(Pattern(10, 1)).code(), StatusCode::kInvalidArgument);
    EXPECT_FALSE(fs::exists(U8(path)));

    auto bad = OutFile::Create(dir.File("missing_dir/x.bin"));
    EXPECT_EQ(bad.code(), StatusCode::kNotFound);
}

TEST(OutFile, ConcurrentPositionalWrites) {
    ScratchDir dir;
    const std::string path = dir.File("tiles.bin");
    auto out = OutFile::Create(path);
    ASSERT_TRUE(out.ok());
    constexpr size_t kBlock = 4096;
    constexpr int kBlocks = 256;
    // Reserve the region with one append, then threads fill disjoint blocks positionally.
    ASSERT_TRUE(out->Append(std::vector<uint8_t>(kBlock * kBlocks, 0)).ok());
    std::vector<std::thread> ts;
    for (int t = 0; t < 8; ++t) {
        ts.emplace_back([&, t] {
            for (int b = t; b < kBlocks; b += 8) {
                std::vector<uint8_t> blk(kBlock, static_cast<uint8_t>(b));
                ASSERT_TRUE(out->WriteAt(static_cast<uint64_t>(b) * kBlock, blk).ok());
            }
        });
    }
    for (auto& th : ts) th.join();
    ASSERT_TRUE(out->Finish().ok());
    const auto got = ReadWhole(path);
    ASSERT_EQ(got.size(), kBlock * kBlocks);
    for (int b = 0; b < kBlocks; ++b) ASSERT_EQ(got[static_cast<size_t>(b) * kBlock + 17], static_cast<uint8_t>(b));
}

TEST(IoStats, CountsReadsAndWrites) {
    ScratchDir dir;
    const std::string path = dir.File("s.bin");
    const auto before = IoStats::Snapshot();
    {
        auto out = OutFile::Create(path);
        ASSERT_TRUE(out.ok());
        ASSERT_TRUE(out->Append(Pattern(1000, 1)).ok());
        ASSERT_TRUE(out->Finish().ok());
    }
    auto f = File::Open(path);
    ASSERT_TRUE(f.ok());
    std::vector<uint8_t> buf(500);
    ASSERT_TRUE(f->ReadAt(0, buf).ok());
    const auto d = IoStats::Snapshot() - before;
    EXPECT_GE(d.writes, 1u);
    EXPECT_GE(d.writeBytes, 1000u);
    EXPECT_GE(d.reads, 1u);
    EXPECT_GE(d.readBytes, 500u);
    EXPECT_GE(d.opens, 2u);
}
