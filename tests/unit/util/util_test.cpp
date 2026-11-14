// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include <cstdint>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "core/util/checked_math.h"
#include "core/util/counters.h"
#include "core/util/endian.h"
#include "core/util/status.h"
#include "core/util/timer.h"

namespace {

using ddbt::Expected;
using ddbt::Status;
using ddbt::StatusCode;

Expected<int> ParsePositive(int v) {
    if (v <= 0) return ddbt::InvalidArgument("not positive");
    return v;
}

Status UsesTry(int v, int* out) {
    DDBT_ASSIGN_OR_RETURN(*out, ParsePositive(v));
    DDBT_TRY(Status::Ok());
    return Status::Ok();
}

Status Chain(int v) {
    DDBT_TRY(ParsePositive(v));
    return Status::Ok();
}

}  // namespace

TEST(Status, OkIsCheapAndDefault) {
    Status s;
    EXPECT_TRUE(s.ok());
    EXPECT_EQ(s.code(), StatusCode::kOk);
    EXPECT_EQ(s.ToString(), "OK");
    EXPECT_TRUE(s.message().empty());
    EXPECT_EQ(sizeof(Status), sizeof(void*));
}

TEST(Status, ErrorCarriesCodeAndMessage) {
    Status s = ddbt::IoError("disk on fire");
    EXPECT_FALSE(s.ok());
    EXPECT_EQ(s.code(), StatusCode::kIoError);
    EXPECT_EQ(s.message(), "disk on fire");
    EXPECT_EQ(s.ToString(), "IoError: disk on fire");
    Status copy = s;
    EXPECT_EQ(copy.ToString(), s.ToString());
    Status ctx = s.WithContext("reading a.tif");
    EXPECT_EQ(ctx.message(), "reading a.tif: disk on fire");
    EXPECT_EQ(ctx.code(), StatusCode::kIoError);
    // OK code with a message stays OK
    EXPECT_TRUE(Status(StatusCode::kOk, "ignored").ok());
}

TEST(Status, ErrnoMapsNotFound) {
    EXPECT_EQ(ddbt::ErrnoStatus("open x", ENOENT).code(), StatusCode::kNotFound);
    const Status s = ddbt::ErrnoStatus("open x", EACCES);
    EXPECT_EQ(s.code(), StatusCode::kIoError);
    EXPECT_NE(s.message().find("open x"), std::string_view::npos);
    // A full device is its own code: the host may try again once there is room
    EXPECT_EQ(ddbt::ErrnoStatus("write x", ENOSPC).code(), StatusCode::kNoSpace);
#ifdef _WIN32
    EXPECT_EQ(ddbt::Win32Status("write x", ERROR_DISK_FULL).code(), StatusCode::kNoSpace);
    EXPECT_EQ(ddbt::Win32Status("write x", ERROR_HANDLE_DISK_FULL).code(), StatusCode::kNoSpace);
#endif
}

TEST(Expected, ValueAndErrorPaths) {
    Expected<int> a = ParsePositive(5);
    ASSERT_TRUE(a.ok());
    EXPECT_EQ(*a, 5);
    Expected<int> b = ParsePositive(-1);
    ASSERT_FALSE(b.ok());
    EXPECT_EQ(b.code(), StatusCode::kInvalidArgument);
    EXPECT_EQ(b.value_or(7), 7);
    // an OK status cannot make a valueless Expected
    Expected<int> c = Status::Ok();
    EXPECT_FALSE(c.ok());
    EXPECT_EQ(c.code(), StatusCode::kInternal);

    Expected<std::string> s = "abc";
    ASSERT_TRUE(s.ok());
    EXPECT_EQ(s->size(), 3u);
    std::string moved = std::move(s).value();
    EXPECT_EQ(moved, "abc");

    Expected<std::unique_ptr<int>> p = std::make_unique<int>(3);
    ASSERT_TRUE(p.ok());
    EXPECT_EQ(**p, 3);
}

TEST(Expected, Macros) {
    int out = 0;
    EXPECT_TRUE(UsesTry(4, &out).ok());
    EXPECT_EQ(out, 4);
    EXPECT_EQ(UsesTry(0, &out).code(), StatusCode::kInvalidArgument);
    EXPECT_TRUE(Chain(1).ok());
    EXPECT_EQ(Chain(-3).code(), StatusCode::kInvalidArgument);
}

TEST(CheckedMath, AddSubMul) {
    uint32_t u = 0;
    EXPECT_TRUE(ddbt::CheckedAdd<uint32_t>(1, 2, &u));
    EXPECT_EQ(u, 3u);
    EXPECT_FALSE(ddbt::CheckedAdd<uint32_t>(0xFFFFFFFFu, 1u, &u));
    EXPECT_FALSE(ddbt::CheckedSub<uint32_t>(1u, 2u, &u));
    EXPECT_FALSE(ddbt::CheckedMul<uint32_t>(0x10000u, 0x10000u, &u));
    EXPECT_TRUE(ddbt::CheckedMul<uint32_t>(0x10000u, 0xFFFFu, &u));

    int64_t s = 0;
    EXPECT_FALSE(ddbt::CheckedAdd<int64_t>(std::numeric_limits<int64_t>::max(), 1, &s));
    EXPECT_FALSE(ddbt::CheckedSub<int64_t>(std::numeric_limits<int64_t>::min(), 1, &s));
    EXPECT_FALSE(ddbt::CheckedMul<int64_t>(std::numeric_limits<int64_t>::min(), -1, &s));
    EXPECT_TRUE(ddbt::CheckedMul<int64_t>(-3, 4, &s));
    EXPECT_EQ(s, -12);

    uint64_t big = 0;
    EXPECT_FALSE(ddbt::CheckedMul<uint64_t>(uint64_t{1} << 32, uint64_t{1} << 32, &big));
}

TEST(CheckedMath, CastAlignRange) {
    uint8_t b = 0;
    EXPECT_TRUE(ddbt::CheckedCast<uint8_t>(255, &b));
    EXPECT_FALSE(ddbt::CheckedCast<uint8_t>(256, &b));
    EXPECT_FALSE(ddbt::CheckedCast<uint8_t>(-1, &b));
    int32_t i = 0;
    EXPECT_FALSE(ddbt::CheckedCast<int32_t>(uint64_t{1} << 31, &i));

    uint64_t a = 0;
    EXPECT_TRUE(ddbt::CheckedAlignUp<uint64_t>(65, 64, &a));
    EXPECT_EQ(a, 128u);
    EXPECT_FALSE(ddbt::CheckedAlignUp<uint64_t>(~uint64_t{0} - 3, 64, &a));

    EXPECT_TRUE(ddbt::RangeInside(10, 5, 15));
    EXPECT_FALSE(ddbt::RangeInside(10, 6, 15));
    EXPECT_FALSE(ddbt::RangeInside(16, 0, 15));
    EXPECT_FALSE(ddbt::RangeInside(~uint64_t{0}, 2, 100));

    auto m = ddbt::Mul3OrError<uint64_t>(1u << 30, 1u << 30, 1u << 10, "tile size");
    EXPECT_EQ(m.code(), StatusCode::kOverflow);
    EXPECT_NE(m.status().message().find("tile size"), std::string_view::npos);
    auto ok = ddbt::Mul3OrError<uint64_t>(512, 512, 4);
    ASSERT_TRUE(ok.ok());
    EXPECT_EQ(*ok, 512u * 512u * 4u);
    EXPECT_EQ(ddbt::CastOrError<uint16_t>(70000).code(), StatusCode::kOverflow);
}

TEST(Endian, LoadStoreUnaligned) {
    alignas(8) uint8_t buf[32] = {};
    for (size_t off = 0; off < 8; ++off) {
        ddbt::StoreLE<uint32_t>(buf + off, 0x11223344u);
        EXPECT_EQ(buf[off], 0x44);
        EXPECT_EQ(buf[off + 3], 0x11);
        EXPECT_EQ(ddbt::LoadLE<uint32_t>(buf + off), 0x11223344u);
        EXPECT_EQ(ddbt::LoadBE<uint32_t>(buf + off), 0x44332211u);

        ddbt::StoreBE<uint64_t>(buf + off, 0x0102030405060708ull);
        EXPECT_EQ(buf[off], 0x01);
        EXPECT_EQ(buf[off + 7], 0x08);
        EXPECT_EQ(ddbt::LoadBE<uint64_t>(buf + off), 0x0102030405060708ull);
        EXPECT_EQ(ddbt::LoadLE<uint64_t>(buf + off), 0x0807060504030201ull);

        ddbt::StoreBE<uint16_t>(buf + off, 0xABCD);
        EXPECT_EQ(buf[off], 0xAB);
        EXPECT_EQ(ddbt::LoadLE<uint16_t>(buf + off), 0xCDAB);

        ddbt::StoreLE<int16_t>(buf + off, -2);
        EXPECT_EQ(ddbt::LoadLE<int16_t>(buf + off), -2);
    }
    ddbt::StoreBE<double>(buf + 3, 1.5);
    EXPECT_EQ(ddbt::LoadBE<double>(buf + 3), 1.5);
    EXPECT_EQ(buf[3], 0x3F);
    ddbt::StoreLE<float>(buf + 1, -0.25f);
    EXPECT_EQ(ddbt::LoadLE<float>(buf + 1), -0.25f);
    EXPECT_EQ(ddbt::LoadEndian<uint16_t>(buf + 3, true), ddbt::LoadBE<uint16_t>(buf + 3));
    static_assert(ddbt::ByteSwap(uint32_t{0x01020304}) == 0x04030201u);
    static_assert(ddbt::ByteSwap(uint64_t{0x0102030405060708ull}) == 0x0807060504030201ull);
}

TEST(Counters, RegistryAndJson) {
    ddbt::Counters reg;
    ddbt::Counter& a = reg.Get("b.second");
    ddbt::Counter& b = reg.Get("a.first");
    EXPECT_EQ(&reg.Get("b.second"), &a);
    a.Add(3);
    b.Add();
    b.Max(10);
    b.Max(5);
    reg.Get("q\"uote");
    EXPECT_EQ(reg.ToJson(), "{\"a.first\":10,\"b.second\":3,\"q\\\"uote\":0}");
    reg.ResetAll();
    EXPECT_EQ(a.Load(), 0u);
    EXPECT_EQ(&ddbt::Counters::Global(), &ddbt::Counters::Global());
}

TEST(Counters, ConcurrentAddAndRegister) {
    ddbt::Counters reg;
    constexpr int kThreads = 16;
    constexpr int kIters = 20000;
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&reg, t] {
            ddbt::Counter& c = reg.Get("shared");
            for (int i = 0; i < kIters; ++i) c.Add();
            reg.Get("own." + std::to_string(t)).Add(static_cast<uint64_t>(t));
        });
    }
    for (auto& th : threads) th.join();
    EXPECT_EQ(reg.Get("shared").Load(), uint64_t{kThreads} * kIters);
    EXPECT_EQ(reg.Snapshot().size(), size_t{kThreads} + 1);
}

TEST(Timer, Monotonic) {
    ddbt::Timer t;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    EXPECT_GE(t.ElapsedNs(), 2'000'000u);
    EXPECT_GE(t.ElapsedMs(), 2.0);
    ddbt::Counter c;
    {
        ddbt::ScopedTimer st(c);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_GE(c.Load(), 1'000'000u);
}
