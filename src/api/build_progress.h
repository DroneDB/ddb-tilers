// SPDX-License-Identifier: MPL-2.0
#pragma once

// Forwards a builder's progress (fraction, phase) to a C callback of ddbtBuildCog2 /
// ddbtBuildCopc2. Calls are serialised (the callback never runs concurrently with itself), the
// fraction never goes back, and an exception thrown by the callback is not propagated into the
// builder (a C callback must not throw; one that does is ignored).

#include <functional>
#include <memory>
#include <mutex>

#include "ddbt/ddbt.h"

namespace ddbt::api {

class BuildProgress {
public:
    BuildProgress(ddbtProgressCallback callback, void* userData)
        : state_(callback != nullptr ? std::make_shared<State>(callback, userData) : nullptr) {}

    /// The builder's progress function; empty when there is no callback.
    std::function<void(double, const char*)> Fn() const {
        if (!state_) return {};
        return [s = state_](double fraction, const char* phase) {
            const std::lock_guard<std::mutex> lock(s->mutex);
            if (fraction < s->last) fraction = s->last;
            s->last = fraction;
            try {
                s->callback(fraction, phase, s->userData);
            } catch (...) {
            }
        };
    }

private:
    struct State {
        State(ddbtProgressCallback cb, void* data) : callback(cb), userData(data) {}
        ddbtProgressCallback callback;
        void* userData;
        std::mutex mutex;
        double last = 0;
    };
    std::shared_ptr<State> state_;
};

}  // namespace ddbt::api
