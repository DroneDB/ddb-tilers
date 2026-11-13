// SPDX-License-Identifier: MPL-2.0

#include "core/crs/proj_context.h"

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <mutex>
#include <vector>

#ifndef DDBT_PROJ_DATA_DEFAULT
#define DDBT_PROJ_DATA_DEFAULT ""
#endif

namespace ddbt::crs {

namespace {

std::mutex g_dirMutex;
std::string g_dirOverride;  // SetProjDataDir
bool g_dirOverrideSet = false;

// PROJ's last error message on this thread (a context is only used by the thread that logs).
thread_local std::string t_lastMessage;

void LogToThread(void*, int level, const char* msg) {
    if (msg != nullptr && level <= PJ_LOG_ERROR) t_lastMessage = msg;
}

// One context per thread, with its log sink and the thread's copies of shared objects.
struct ThreadState {
    struct Entry {
        uint64_t id;
        std::weak_ptr<const void> alive;
        PJ* pj;
    };
    PJ_CONTEXT* ctx = nullptr;
    std::vector<Entry> copies;

    ThreadState() { ctx = detail::NewContext(); }
    ~ThreadState() {
        for (auto& e : copies) proj_destroy(e.pj);
        copies.clear();
        proj_context_destroy(ctx);
    }
    ThreadState(const ThreadState&) = delete;
    ThreadState& operator=(const ThreadState&) = delete;

    void DropDead() {
        auto dead = [](const Entry& e) { return e.alive.expired(); };
        for (auto& e : copies)
            if (dead(e)) proj_destroy(e.pj);
        copies.erase(std::remove_if(copies.begin(), copies.end(), dead), copies.end());
    }
};

ThreadState& State() {
    thread_local ThreadState state;
    return state;
}

}  // namespace

void SetProjDataDir(std::string dir) {
    std::lock_guard lock(g_dirMutex);
    g_dirOverride = std::move(dir);
    g_dirOverrideSet = !g_dirOverride.empty();
}

std::string ProjDataDir() {
    {
        std::lock_guard lock(g_dirMutex);
        if (g_dirOverrideSet) return g_dirOverride;
    }
    if (const char* env = std::getenv("DDBT_PROJ_DATA"); env != nullptr && *env != '\0') return env;
    return DDBT_PROJ_DATA_DEFAULT;
}

namespace detail {

PJ_CONTEXT* NewContext() {
    PJ_CONTEXT* ctx = proj_context_create();
    if (ctx == nullptr) throw CrsError("proj_context_create failed");
    proj_context_set_enable_network(ctx, 0);
    const std::string dir = ProjDataDir();
    if (!dir.empty()) {
        const char* paths[] = {dir.c_str()};
        proj_context_set_search_paths(ctx, 1, paths);
    }
    proj_log_level(ctx, PJ_LOG_ERROR);
    proj_log_func(ctx, nullptr, LogToThread);
    return ctx;
}

PJ_CONTEXT* ThreadContext() { return State().ctx; }

std::string LastError(PJ_CONTEXT* ctx) {
    std::string msg = std::move(t_lastMessage);
    t_lastMessage.clear();
    if (!msg.empty()) return msg;
    const int err = proj_context_errno(ctx);
    if (err != 0) {
        const char* s = proj_context_errno_string(ctx, err);
        if (s != nullptr) return s;
    }
    return "PROJ error";
}

void ClearLastError() noexcept { t_lastMessage.clear(); }

PJ* ThreadCopy(uint64_t id, const std::weak_ptr<const void>& alive, const void* owner, MakeFn make) {
    ThreadState& st = State();
    for (const auto& e : st.copies)
        if (e.id == id) return e.pj;
    if (st.copies.size() >= 32) st.DropDead();
    PJ* pj = make(owner, st.ctx);
    if (pj == nullptr) return nullptr;
    st.copies.push_back({id, alive, pj});
    return pj;
}

uint64_t NextId() noexcept {
    static std::atomic<uint64_t> next{1};
    return next.fetch_add(1, std::memory_order_relaxed);
}

}  // namespace detail

}  // namespace ddbt::crs
