// SPDX-License-Identifier: MPL-2.0
#pragma once

#include <atomic>
#include <memory>

namespace ddbt::exec {

/// Shared cancellation flag. Copies share the flag; Child() makes a token that is cancelled when
/// either it or any ancestor is cancelled (a request token with a per-tile child, say).
/// Cancellation is cooperative: code polls IsCancelled() at convenient points.
class CancelToken {
public:
    CancelToken() : s_(std::make_shared<State>()) {}

    /// New token linked to this one.
    CancelToken Child() const {
        CancelToken c;
        c.s_->parent = s_;
        return c;
    }

    void Cancel() const noexcept { s_->flag.store(true, std::memory_order_release); }

    bool IsCancelled() const noexcept {
        for (const State* s = s_.get(); s != nullptr; s = s->parent.get()) {
            if (s->flag.load(std::memory_order_acquire)) return true;
        }
        return false;
    }

    friend bool operator==(const CancelToken& a, const CancelToken& b) noexcept { return a.s_ == b.s_; }

private:
    struct State {
        std::atomic<bool> flag{false};
        std::shared_ptr<const State> parent;
    };
    std::shared_ptr<State> s_;
};

}  // namespace ddbt::exec
