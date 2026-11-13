// SPDX-License-Identifier: MPL-2.0
#pragma once

// Watchdog for tests whose failure mode is a deadlock: aborts the test binary with a message
// if the scope is not left in time, instead of hanging the run.

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>

namespace ddbt::test {

class Watchdog {
public:
    Watchdog(std::chrono::seconds limit, const char* what)
        : thread_([this, limit, what] {
              std::unique_lock lock(mutex_);
              if (!cv_.wait_for(lock, limit, [this] { return done_; })) {
                  std::fprintf(stderr, "watchdog: %s did not finish within %lld s (deadlock?)\n", what,
                               static_cast<long long>(limit.count()));
                  std::fflush(stderr);
                  std::abort();
              }
          }) {}
    ~Watchdog() {
        {
            std::lock_guard lock(mutex_);
            done_ = true;
        }
        cv_.notify_all();
        thread_.join();
    }
    Watchdog(const Watchdog&) = delete;
    Watchdog& operator=(const Watchdog&) = delete;

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    bool done_ = false;
    std::thread thread_;  // last: starts after the members above exist
};

}  // namespace ddbt::test
