// SPDX-License-Identifier: MPL-2.0
//
// 32 threads parse CRSs, get transforms from the shared cache (same and different keys at the
// same time) and transform concurrently; every result must equal the single-threaded one bit for
// bit. Run under the linux-tsan preset for the data-race check.

#include <gtest/gtest.h>

#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <latch>
#include <random>
#include <thread>
#include <vector>

#include "core/crs/crs.h"
#include "core/crs/grid_approx.h"
#include "core/crs/transform.h"

namespace {

using ddbt::crs::Crs;
using ddbt::crs::Direction;
using ddbt::crs::GridApprox;
using ddbt::crs::GridApproxOptions;
using ddbt::crs::Transform;
using ddbt::crs::TransformCache;
using ddbt::crs::TransformOptions;

struct Job {
    int epsg;
    double x0, y0, span;
};

const Job kJobs[] = {
    {31370, 140000, 160000, 20000}, {3008, 140000, 6390000, 20000}, {2154, 640000, 6850000, 25000},
    {32617, 310000, 4450000, 20000}, {6599, 1500000, 300000, 200000}, {4326, -10, 35, 40},
};

std::vector<double> Points(const Job& j, size_t n, uint64_t seed, bool ys) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> d(0, j.span);
    std::vector<double> v(n);
    for (size_t i = 0; i < n; ++i) {
        const double a = d(rng), b = d(rng);
        v[i] = ys ? j.y0 + b : j.x0 + a;
    }
    return v;
}

TEST(CrsThreads, ConcurrentTransformsMatchSerial) {
    constexpr size_t kN = 3000;
    // Serial reference with a private cache (its own prepared objects).
    TransformCache serialCache;
    std::vector<std::vector<double>> refX, refY;
    for (size_t j = 0; j < std::size(kJobs); ++j) {
        TransformOptions o;
        o.analyticWebMercator = false;  // exercise PROJ for 4326 too
        const auto t = serialCache.Get(Crs::FromEpsg(kJobs[j].epsg), Crs::FromEpsg(3857), o);
        auto x = Points(kJobs[j], kN, j, false), y = Points(kJobs[j], kN, j, true);
        ASSERT_EQ(t->TransformPoints(x.data(), y.data(), nullptr, kN), 0u);
        refX.push_back(std::move(x));
        refY.push_back(std::move(y));
    }

    TransformCache shared;
    constexpr int kThreads = 32;
    std::latch start(kThreads);  // all threads begin together (sleeping, not spinning: TSan)
    std::atomic<size_t> mismatches{0}, failures{0};
    std::vector<std::thread> threads;
    for (int ti = 0; ti < kThreads; ++ti) {
        threads.emplace_back([&, ti] {
            start.arrive_and_wait();
            for (int round = 0; round < 3; ++round) {
                for (size_t k = 0; k < std::size(kJobs); ++k) {
                    const size_t j = (k + static_cast<size_t>(ti)) % std::size(kJobs);
                    TransformOptions o;
                    o.analyticWebMercator = false;
                    // Parse in this thread, get from the shared cache (single-flight on a miss)
                    const auto t = shared.Get(Crs::FromUserInput("EPSG:" + std::to_string(kJobs[j].epsg)),
                                              Crs::FromEpsg(3857), o);
                    auto x = Points(kJobs[j], kN, j, false), y = Points(kJobs[j], kN, j, true);
                    // Batches of varied sizes, 1 included
                    size_t off = 0, step = 1 + static_cast<size_t>(ti) % 7;
                    while (off < kN) {
                        const size_t m = std::min(step, kN - off);
                        failures += t->TransformPoints(x.data() + off, y.data() + off, nullptr, m);
                        off += m;
                        step = step * 3 + 1;
                    }
                    for (size_t i = 0; i < kN; ++i) {
                        if (std::bit_cast<uint64_t>(x[i]) != std::bit_cast<uint64_t>(refX[j][i]) ||
                            std::bit_cast<uint64_t>(y[i]) != std::bit_cast<uint64_t>(refY[j][i]))
                            ++mismatches;
                    }
                    // Inverse on the same shared object
                    failures += t->TransformPoints(x.data(), y.data(), nullptr, kN, Direction::Inverse);
                }
            }
        });
    }
    for (auto& t : threads) t.join();
    EXPECT_EQ(failures.load(), 0u);
    EXPECT_EQ(mismatches.load(), 0u);
    EXPECT_EQ(shared.Size(), std::size(kJobs));
}

TEST(CrsThreads, SharedGridApprox) {
    const auto t = TransformCache::Global().Get(Crs::FromEpsg(31370), Crs::FromEpsg(3857));
    GridApproxOptions go;
    go.maxError = 0.01;
    const GridApprox g = GridApprox::Build(t, 140000, 160000, 160000, 180000, go);
    const Job j{31370, 139000, 159000, 22000};  // partly outside: exact fallback runs concurrently too
    auto rx = Points(j, 5000, 9, false), ry = Points(j, 5000, 9, true);
    ASSERT_EQ(g.TransformPoints(rx.data(), ry.data(), rx.size()), 0u);
    std::atomic<size_t> mismatches{0};
    std::vector<std::thread> threads;
    for (int ti = 0; ti < 32; ++ti) {
        threads.emplace_back([&] {
            auto x = Points(j, 5000, 9, false), y = Points(j, 5000, 9, true);
            g.TransformPoints(x.data(), y.data(), x.size());
            for (size_t i = 0; i < x.size(); ++i)
                if (x[i] != rx[i] || y[i] != ry[i]) ++mismatches;
        });
    }
    for (auto& th : threads) th.join();
    EXPECT_EQ(mismatches.load(), 0u);
}

TEST(CrsThreads, TransformsDestroyedWhileThreadsLive) {
    // A thread keeps clones of transforms that are gone; they are dropped, not used.
    std::thread th([] {
        for (int i = 0; i < 80; ++i) {
            const auto t = Transform::Create(Crs::FromEpsg(2154), Crs::FromEpsg(3857));
            double x = 652000, y = 6862000;
            EXPECT_TRUE(t->TransformPoint(x, y));
        }
    });
    th.join();
}

}  // namespace
