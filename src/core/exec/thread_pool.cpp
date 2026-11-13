// SPDX-License-Identifier: MPL-2.0
#include "core/exec/thread_pool.h"

#include <algorithm>
#include <charconv>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include "core/exec/topology.h"

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <immintrin.h>
#define DDBT_HAVE_MM_PAUSE 1
#endif

namespace ddbt::exec {

namespace detail {
void CpuRelax() noexcept {
#ifdef DDBT_HAVE_MM_PAUSE
    _mm_pause();
#else
    std::this_thread::yield();
#endif
}
}  // namespace detail

namespace {

struct WorkerTls {
    ThreadPool* pool = nullptr;
    int index = -1;
};
thread_local WorkerTls tlsWorker;

uint64_t NextRandom(uint64_t& s) noexcept {
    // xorshift64*
    s ^= s >> 12;
    s ^= s << 25;
    s ^= s >> 27;
    return s * 0x2545F4914F6CDD1DULL;
}

constexpr unsigned kInteractiveTag = static_cast<unsigned>(Priority::kInteractive);
constexpr unsigned kBulkTag = static_cast<unsigned>(Priority::kBulk);

// Store-load barrier between publishing something and reading a counter that the other side
// increments before re-checking (Dekker). GCC's TSan does not model fences, so sanitized builds
// read the counter with an RMW instead.
inline uint32_t LoadAfterPublish(std::atomic<uint32_t>& counter) noexcept {
#if defined(__SANITIZE_THREAD__)
    return counter.fetch_add(0, std::memory_order_seq_cst);
#else
    std::atomic_thread_fence(std::memory_order_seq_cst);
    return counter.load(std::memory_order_relaxed);
#endif
}

}  // namespace

// ---- parkers -----------------------------------------------------------------------------------

namespace detail {
namespace {

constexpr unsigned kParkerChunkBits = 8;
constexpr uint32_t kParkerChunkSize = 1u << kParkerChunkBits;
constexpr uint32_t kMaxParkers = 1u << kParkerIdBits;
constexpr uint32_t kParkerChunks = kMaxParkers >> kParkerChunkBits;

// Chunks are allocated on demand and never freed: a parker may be woken after its thread left.
std::atomic<Parker*> gParkerChunks[kParkerChunks];
std::mutex gParkerMutex;
uint32_t gNextParker = 1;  // 0 = none
uint32_t gFreeParkers = 0;  // free list through Parker::nextFree

Parker& ParkerSlot(uint32_t id) noexcept {
    return gParkerChunks[id >> kParkerChunkBits].load(std::memory_order_acquire)[id & (kParkerChunkSize - 1)];
}

uint32_t AcquireParker() noexcept {
    std::lock_guard lock(gParkerMutex);
    if (gFreeParkers != 0) {
        const uint32_t id = gFreeParkers;
        gFreeParkers = ParkerSlot(id).nextFree;
        return id;
    }
    const uint32_t id = gNextParker;
    if (id >= kMaxParkers) {
        std::fputs("ddbt: more than 2^20 threads waiting on thread pools at once\n", stderr);
        std::abort();
    }
    const uint32_t chunk = id >> kParkerChunkBits;
    if (gParkerChunks[chunk].load(std::memory_order_relaxed) == nullptr) {
        auto* p = new Parker[kParkerChunkSize];  // std::bad_alloc here terminates (noexcept)
        for (uint32_t k = 0; k < kParkerChunkSize; ++k) p[k].id = (chunk << kParkerChunkBits) | k;
        gParkerChunks[chunk].store(p, std::memory_order_release);
    }
    ++gNextParker;
    return id;
}

// The calling thread's parker id, which also identifies its injected tasks (Task::origin), and
// its count of injected pushes. A reused id continues its previous owner's count, so tasks a
// dead thread left in a queue never look like the new owner's own pushes.
struct ThreadSlot {
    uint32_t id = 0;
    uint64_t seq = 0;
    ~ThreadSlot() {
        if (id == 0) return;
        std::lock_guard lock(gParkerMutex);
        Parker& p = ParkerSlot(id);
        p.spawnSeq = seq;
        p.nextFree = gFreeParkers;
        gFreeParkers = id;
    }
};
thread_local ThreadSlot tlsSlot;

inline ThreadSlot& Slot() noexcept {
    if (tlsSlot.id == 0) [[unlikely]] {
        tlsSlot.id = AcquireParker();
        tlsSlot.seq = ParkerSlot(tlsSlot.id).spawnSeq;
    }
    return tlsSlot;
}

}  // namespace

Parker& ThisThreadParker() noexcept { return ParkerSlot(Slot().id); }

Parker& ParkerById(uint32_t id) noexcept { return ParkerSlot(id); }

}  // namespace detail

// ---- injection queue ---------------------------------------------------------------------------

void ThreadPool::InjectionQueue::LinkLocked(detail::Task* t) noexcept {
    t->next = nullptr;
    t->prev = tail;
    if (tail != nullptr) tail->next = t;
    else head = t;
    tail = t;
    if (t->restricted) {
        t->snext = nullptr;
        t->sprev = rtail;
        if (rtail != nullptr) rtail->snext = t;
        else rhead = t;
        rtail = t;
        restrictedSize.fetch_add(1, std::memory_order_seq_cst);
    }
    size.fetch_add(1, std::memory_order_seq_cst);
}

// Caller holds the mutex.
void ThreadPool::InjectionQueue::Unlink(detail::Task* t) noexcept {
    if (t->prev != nullptr) t->prev->next = t->next;
    else head = t->next;
    if (t->next != nullptr) t->next->prev = t->prev;
    else tail = t->prev;
    t->prev = t->next = nullptr;
    if (t->restricted) {
        if (t->sprev != nullptr) t->sprev->snext = t->snext;
        else rhead = t->snext;
        if (t->snext != nullptr) t->snext->sprev = t->sprev;
        else rtail = t->sprev;
        t->sprev = t->snext = nullptr;
        restrictedSize.fetch_sub(1, std::memory_order_relaxed);
    }
    size.fetch_sub(1, std::memory_order_relaxed);
}

detail::Task* ThreadPool::InjectionQueue::TryPop() noexcept {
    if (size.load(std::memory_order_seq_cst) == 0) return nullptr;
    std::lock_guard lock(mutex);
    detail::Task* t = head;
    if (t != nullptr) Unlink(t);
    return t;
}

detail::Task* ThreadPool::InjectionQueue::TryPopUncontended() noexcept {
    if (size.load(std::memory_order_seq_cst) == 0) return nullptr;
    // Workers only try the lock: when it is busy another thread is pushing or taking, and
    // queueing up behind it would only slow the producer down. Nothing is lost: a worker
    // re-checks `size` before it sleeps.
    std::unique_lock lock(mutex, std::try_to_lock);
    if (!lock.owns_lock()) return nullptr;
    detail::Task* t = head;
    if (t != nullptr) Unlink(t);
    return t;
}

detail::Task* ThreadPool::InjectionQueue::TryPopNewestOwn(uint32_t origin, uint64_t seqFloor) noexcept {
    if (size.load(std::memory_order_seq_cst) == 0) return nullptr;
    std::lock_guard lock(mutex);
    detail::Task* t = tail;
    if (t == nullptr || t->origin != origin || t->seq <= seqFloor) return nullptr;
    Unlink(t);
    return t;
}

bool ThreadPool::InjectionQueue::HasNewestOwn(uint32_t origin, uint64_t seqFloor) noexcept {
    if (size.load(std::memory_order_seq_cst) == 0) return false;
    std::lock_guard lock(mutex);
    return tail != nullptr && tail->origin == origin && tail->seq > seqFloor;
}

namespace {

bool InScope(const detail::Task* t, const detail::IsolationState& scope, uint32_t origin, bool own) noexcept {
    // Tasks of the isolated tree, or tasks this thread injected since its current task / scope
    // started (its own subtasks).
    if (scope.node != nullptr && t->isolation != nullptr && detail::IsWithin(t->isolation, scope.node)) return true;
    return own && t->origin == origin && t->seq > scope.seqFloor;
}

}  // namespace

detail::Task* ThreadPool::InjectionQueue::TryPopFor(const detail::IsolationState& scope, uint32_t origin,
                                                    bool own) noexcept {
    if (restrictedSize.load(std::memory_order_seq_cst) == 0) return nullptr;
    std::lock_guard lock(mutex);
    // Newest first: own subtasks run depth-first, like from a worker's deque.
    for (detail::Task* t = rtail; t != nullptr; t = t->sprev) {
        if (!InScope(t, scope, origin, own)) continue;
        Unlink(t);
        return t;
    }
    return nullptr;
}

bool ThreadPool::InjectionQueue::HasFor(const detail::IsolationState& scope, uint32_t origin, bool own) noexcept {
    if (restrictedSize.load(std::memory_order_seq_cst) == 0) return false;
    std::lock_guard lock(mutex);
    for (detail::Task* t = rtail; t != nullptr; t = t->sprev) {
        if (InScope(t, scope, origin, own)) return true;
    }
    return false;
}

// ---- isolation ---------------------------------------------------------------------------------

namespace detail {
IsolationState& CurrentIsolation() noexcept {
    thread_local IsolationState state;
    return state;
}
}  // namespace detail

IsolationScope::IsolationScope(const void* key) noexcept : saved_(detail::CurrentIsolation()) {
    node_.key = key;
    node_.parent = saved_.node;
    detail::IsolationState& cur = detail::CurrentIsolation();
    cur.node = &node_;
    cur.seqFloor = detail::Slot().seq;
    // Own-deque tasks pushed from now on belong to this scope.
    if (tlsWorker.pool != nullptr) {
        cur.floor = tlsWorker.pool->workers_[static_cast<size_t>(tlsWorker.index)]->deque.BottomIndex();
    }
}

IsolationScope::~IsolationScope() { detail::CurrentIsolation() = saved_; }

bool IsolationScope::Inside(const void* key) noexcept {
    for (const IsolationNode* n = detail::CurrentIsolation().node; n != nullptr; n = n->parent) {
        if (n->key == key) return true;
    }
    return false;
}

// ---- pool --------------------------------------------------------------------------------------

ThreadPool::ThreadPool(ThreadPoolOptions options) : options_(std::move(options)) {
    size_t n = options_.threads;
    if (n == 0 && !options_.zeroWorkers) n = DefaultThreadCount();

    // Placement: worker i takes the i-th CPU of the domain-ordered CPU list, so workers fill
    // domains in proportion to their size. Pinning (optional) is to the whole domain.
    const Topology& topo = Topology::Get();
    std::vector<int> domainOfSlot;
    for (size_t d = 0; d < topo.l3.size(); ++d) {
        for (size_t k = 0; k < topo.l3[d].cpus.size(); ++k) domainOfSlot.push_back(static_cast<int>(d));
    }
    if (domainOfSlot.empty()) domainOfSlot.push_back(0);
    // Interleave so that a pool smaller than the machine still spreads over the domains:
    // slot order d0, d1, d0, d1, ... weighted by the domain sizes.
    std::vector<int> order;
    {
        std::vector<size_t> remaining;
        for (const auto& dom : topo.l3) remaining.push_back(dom.cpus.size());
        if (remaining.empty()) remaining.push_back(1);
        size_t left = 0;
        for (size_t r : remaining) left += r;
        while (left > 0) {
            for (size_t d = 0; d < remaining.size(); ++d) {
                if (remaining[d] > 0) {
                    order.push_back(static_cast<int>(d));
                    --remaining[d];
                    --left;
                }
            }
        }
    }

    std::vector<int> domainOfWorker(n);
    workers_.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        auto w = std::make_unique<Worker>();
        domainOfWorker[i] = order[i % order.size()];
        if (options_.pinToL3 && !topo.l3.empty()) w->cpus = topo.l3[static_cast<size_t>(domainOfWorker[i])].cpus;
        w->rng = 0x9E3779B97F4A7C15ULL * (i + 1);
        workers_.push_back(std::move(w));
    }
    for (size_t i = 0; i < n; ++i) {
        std::vector<uint32_t>& v = workers_[i]->victims;
        // Same domain first, then the others, each tier rotated so victims are spread.
        for (int tier = 0; tier < 2; ++tier) {
            for (size_t k = 1; k < n; ++k) {
                const size_t j = (i + k) % n;
                const bool same = domainOfWorker[j] == domainOfWorker[i];
                if ((tier == 0) == same) v.push_back(static_cast<uint32_t>(j));
            }
        }
    }
    for (size_t i = 0; i < n; ++i) {
        workers_[i]->thread = std::thread([this, i] { WorkerMain(static_cast<uint32_t>(i)); });
    }
}

ThreadPool::~ThreadPool() {
    stopping_.store(true, std::memory_order_seq_cst);
    WakeAll();
    for (auto& w : workers_) {
        if (w->thread.joinable()) w->thread.join();
    }
    // No workers (or they all left): run what is left on this thread.
    while (TryRunOne()) {
    }
}

ThreadPool* ThreadPool::Current() noexcept { return tlsWorker.pool; }
int ThreadPool::CurrentWorkerIndex() noexcept { return tlsWorker.pool != nullptr ? tlsWorker.index : -1; }

void ThreadPool::Push(detail::Task* task, Priority priority) {
    task->bulk = priority == Priority::kBulk ? 1 : 0;
    if (tlsWorker.pool == this) {
        // May throw bad_alloc (nothing pushed then).
        workers_[static_cast<size_t>(tlsWorker.index)]->deque.Push(task, static_cast<unsigned>(priority));
        NotifyOne();
        return;
    }
    detail::ThreadSlot& slot = detail::Slot();
    constexpr uint64_t kSeqMask = (uint64_t{1} << 42) - 1;  // 4.4e12 pushes per thread id
    task->origin = slot.id & ((1u << detail::kParkerIdBits) - 1);
    task->seq = ++slot.seq & kSeqMask;
    // A thread whose waits are restricted (isolated, or nested too deep) looks for its own
    // pushes in the restricted list only, so they go there too.
    task->restricted = task->isolation != nullptr || detail::CurrentIsolation().node != nullptr ||
                       detail::HelpDepth() >= kMaxHelpDepth;
    detail::Parker* isolatedHelper = Inject(task, static_cast<int>(priority));
    injected_.fetch_add(1, std::memory_order_relaxed);
    if (isolatedHelper != nullptr) {
        // An isolated helper of this task's tree was parked: it takes the task.
        isolatedHelper->Unpark();
        return;
    }
    NotifyOne();
}

detail::Parker* ThreadPool::Inject(detail::Task* t, int queue) noexcept {
    InjectionQueue& q = inject_[queue];
    std::lock_guard lock(q.mutex);
    q.LinkLocked(t);
    if (t->isolation == nullptr) return nullptr;
    // Pairs with RegisterPark + the re-check of a parking isolated helper. Matching walks the
    // task's isolation chain: done under the queue lock, while the queued task keeps its tree
    // (the scopes on the stacks of its ancestors) alive.
    if (LoadAfterPublish(parkedIsolated_) == 0) return nullptr;
    std::lock_guard parkLock(parkMutex_);
    for (ParkEntry* e = parked_; e != nullptr; e = e->next) {
        if (e->general || e->node == nullptr || !detail::IsWithin(t->isolation, e->node)) continue;
        // Unlist it, so that the next task of the tree wakes another helper.
        if (e->prev != nullptr) e->prev->next = e->next;
        else parked_ = e->next;
        if (e->next != nullptr) e->next->prev = e->prev;
        e->listed = false;
        e->wokenForWork = true;
        parkedIsolated_.fetch_sub(1, std::memory_order_relaxed);
        return e->parker;
    }
    return nullptr;
}

void ThreadPool::NotifyOne() noexcept {
    // Pairs with the seq_cst updates of searching_ / sleepers_ / parkedGeneral_ followed by the
    // queue re-check in a thread about to sleep.
    const uint32_t sleepers = LoadAfterPublish(sleepers_);
    const uint32_t helpers = parkedGeneral_.load(std::memory_order_seq_cst);
    if (sleepers == 0 && helpers == 0) return;
    // A worker looking for work will find this task, and the last searcher to find work wakes
    // the next sleeper: no system call per push.
    if (searching_.load(std::memory_order_relaxed) != 0) return;
    if (sleepers != 0) {
        epoch_.fetch_add(1, std::memory_order_release);
        epoch_.notify_one();
        return;
    }
    // Every idle worker is busy: wake a parked helper that may run anything.
    detail::Parker* p = nullptr;
    {
        std::lock_guard lock(parkMutex_);
        for (ParkEntry* e = parked_; e != nullptr; e = e->next) {
            if (!e->general) continue;
            if (e->prev != nullptr) e->prev->next = e->next;
            else parked_ = e->next;
            if (e->next != nullptr) e->next->prev = e->prev;
            e->listed = false;
            e->wokenForWork = true;
            parkedGeneral_.fetch_sub(1, std::memory_order_relaxed);
            p = e->parker;
            break;
        }
    }
    if (p != nullptr) p->Unpark();
}

void ThreadPool::WakeAll() noexcept {
    epoch_.fetch_add(1, std::memory_order_release);
    epoch_.notify_all();
    std::lock_guard lock(parkMutex_);
    for (ParkEntry* e = parked_; e != nullptr;) {
        ParkEntry* next = e->next;
        e->prev = e->next = nullptr;
        e->listed = false;
        (e->general ? parkedGeneral_ : parkedIsolated_).fetch_sub(1, std::memory_order_relaxed);
        e->parker->Unpark();
        e = next;
    }
    parked_ = nullptr;
}

void ThreadPool::RegisterPark(ParkEntry& entry, const WaitContext& ctx, detail::Parker& parker) noexcept {
    entry.parker = &parker;
    entry.general = ctx.mode == WaitMode::kAny || ctx.mode == WaitMode::kOwnFirst ||
                    ctx.mode == WaitMode::kInteractiveFirst;
    entry.node = ctx.mode == WaitMode::kIsolated ? ctx.scope.node : nullptr;
    entry.wokenForWork = false;
    std::lock_guard lock(parkMutex_);
    entry.prev = nullptr;
    entry.next = parked_;
    if (parked_ != nullptr) parked_->prev = &entry;
    parked_ = &entry;
    entry.listed = true;
    (entry.general ? parkedGeneral_ : parkedIsolated_).fetch_add(1, std::memory_order_seq_cst);
}

bool ThreadPool::UnregisterPark(ParkEntry& entry) noexcept {
    std::lock_guard lock(parkMutex_);
    if (entry.listed) {
        if (entry.prev != nullptr) entry.prev->next = entry.next;
        else parked_ = entry.next;
        if (entry.next != nullptr) entry.next->prev = entry.prev;
        entry.prev = entry.next = nullptr;
        entry.listed = false;
        (entry.general ? parkedGeneral_ : parkedIsolated_).fetch_sub(1, std::memory_order_relaxed);
    }
    return entry.wokenForWork;
}

void ThreadPool::PassOnWake() noexcept {
    if (!HasWork()) return;
    NotifyOne();
    // The work may be a task of an isolated tree whose other waiters are parked: rare (the
    // helper's condition came true right after it was woken), so simply let them all re-check.
    if (parkedIsolated_.load(std::memory_order_seq_cst) == 0) return;
    std::lock_guard lock(parkMutex_);
    for (ParkEntry* e = parked_; e != nullptr; e = e->next) {
        if (!e->general) e->parker->Unpark();
    }
}

bool ThreadPool::HasWork() const noexcept {
    if (inject_[0].size.load(std::memory_order_seq_cst) != 0) return true;
    if (inject_[1].size.load(std::memory_order_seq_cst) != 0) return true;
    for (const auto& w : workers_) {
        if (!w->deque.Empty()) return true;
    }
    return false;
}

detail::Task* ThreadPool::StealFrom(int self, bool interactiveOnly) {
    const size_t n = workers_.size();
    if (n == 0) return nullptr;
    auto accept = [interactiveOnly](unsigned tag) { return !interactiveOnly || tag == kInteractiveTag; };
    if (self >= 0) {
        Worker& me = *workers_[static_cast<size_t>(self)];
        const size_t v = me.victims.size();
        if (v == 0) return nullptr;
        // Start at a random victim within the same-domain tier, then walk the list.
        const size_t start = static_cast<size_t>(NextRandom(me.rng) % v);
        for (size_t k = 0; k < v; ++k) {
            Worker& victim = *workers_[me.victims[(start + k) % v]];
            if (detail::Task* t = victim.deque.StealIf(accept)) {
                me.stolen.fetch_add(1, std::memory_order_relaxed);
                return t;
            }
        }
        return nullptr;
    }
    thread_local uint64_t rng = 0x853C49E6748FEA9BULL ^ reinterpret_cast<uintptr_t>(&rng);
    const size_t start = static_cast<size_t>(NextRandom(rng) % n);
    for (size_t k = 0; k < n; ++k) {
        if (detail::Task* t = workers_[(start + k) % n]->deque.StealIf(accept)) return t;
    }
    return nullptr;
}

detail::Task* ThreadPool::TakeInjected(int queue, int self) {
    // One task at a time, run at once. Moving several into this worker's deque would break the
    // invariant the isolation rules rely on (a worker's deque only holds descendants of the
    // frames on its own stack): a moved task could end up below the floor of an isolated wait
    // the worker enters next, out of reach of the isolated thread whose tree it belongs to.
    return self < 0 ? inject_[queue].TryPop() : inject_[queue].TryPopUncontended();
}

detail::Task* ThreadPool::FindWork(int self) {
    if (self >= 0) {
        auto& deque = workers_[static_cast<size_t>(self)]->deque;
        // A bulk task of our own can wait while interactive requests are queued.
        if (InteractivePending() && deque.BottomTag() == static_cast<int>(kBulkTag)) {
            if (detail::Task* t = TakeInjected(0, self)) return t;
        }
        if (detail::Task* t = deque.Pop()) return t;
    } else {
        // A helping thread outside the pool first runs its own newest subtask (depth-first,
        // like a worker's own deque); FIFO pops alone nest its waits breadth-first, as deep as
        // the task tree is wide.
        const uint64_t floor = detail::CurrentIsolation().seqFloor;
        if (detail::Task* t = inject_[0].TryPopNewestOwn(detail::Slot().id, floor)) return t;
        if (detail::Task* t = inject_[1].TryPopNewestOwn(detail::Slot().id, floor)) return t;
    }
    if (detail::Task* t = TakeInjected(0, self)) return t;
    if (detail::Task* t = StealFrom(self, false)) return t;
    if (detail::Task* t = TakeInjected(1, self)) return t;
    return nullptr;
}

int ThreadPool::SelfIndex() const noexcept { return tlsWorker.pool == this ? tlsWorker.index : -1; }

void ThreadPool::RunTask(detail::Task* t, int self, const IsolationNode* inherit, bool interactiveRegion) noexcept {
    // The task runs in its own tree's isolation, or (an untagged task run by an isolated wait)
    // in the waiter's: isolation belongs to the region. Own-deque tasks it pushes are above
    // `floor`. Likewise a task run by an interactive wait is interactive work.
    detail::IsolationState& cur = detail::CurrentIsolation();
    const detail::IsolationState saved = cur;
    Priority& priority = detail::CurrentPriority();
    const Priority savedPriority = priority;
    cur.node = t->isolation != nullptr ? t->isolation : inherit;
    cur.seqFloor = detail::Slot().seq;
    if (self >= 0) cur.floor = workers_[static_cast<size_t>(self)]->deque.BottomIndex();
    priority = interactiveRegion ? Priority::kInteractive : t->priority();
    t->invoke(t);
    cur = saved;
    priority = savedPriority;
}

bool ThreadPool::TryRunOne() {
    const int self = SelfIndex();
    detail::Task* t = FindWork(self);
    if (t == nullptr) return false;
    helped_.fetch_add(1, std::memory_order_relaxed);
    RunTask(t, self, nullptr, false);
    return true;
}

bool ThreadPool::TryRunInteractive() {
    if (detail::CurrentIsolation().node != nullptr || detail::HelpDepth() >= kMaxHelpDepth) return false;
    if (!InteractivePending()) return false;
    const int self = SelfIndex();
    detail::Task* t = TakeInjected(0, self);
    if (t == nullptr) return false;
    helped_.fetch_add(1, std::memory_order_relaxed);
    detail::HelpDepthScope depth;
    RunTask(t, self, nullptr, false);
    return true;
}

detail::Task* ThreadPool::FindIsolatedWork(int self, const detail::IsolationState& iso) {
    if (self >= 0) {
        if (detail::Task* t = workers_[static_cast<size_t>(self)]->deque.PopAbove(iso.floor)) return t;
    }
    // Nothing injected can be ours when we are not isolated and pushed nothing since our floor.
    const bool own = detail::Slot().seq > iso.seqFloor;
    if (iso.node == nullptr && !own) return nullptr;
    if (detail::Task* t = inject_[0].TryPopFor(iso, detail::Slot().id, own)) return t;
    if (detail::Task* t = inject_[1].TryPopFor(iso, detail::Slot().id, own)) return t;
    return nullptr;
}

detail::Task* ThreadPool::FindOwnInjected(const detail::IsolationState& iso) {
    if (detail::Slot().seq <= iso.seqFloor) return nullptr;
    const detail::IsolationState own{nullptr, iso.floor, iso.seqFloor};
    if (detail::Task* t = inject_[0].TryPopFor(own, detail::Slot().id, true)) return t;
    if (detail::Task* t = inject_[1].TryPopFor(own, detail::Slot().id, true)) return t;
    return nullptr;
}

bool ThreadPool::HasIsolatedWork(int self, const detail::IsolationState& iso) noexcept {
    if (self >= 0 && workers_[static_cast<size_t>(self)]->deque.BottomIndex() > iso.floor &&
        !workers_[static_cast<size_t>(self)]->deque.Empty())
        return true;
    const bool own = detail::Slot().seq > iso.seqFloor;
    if (iso.node == nullptr && !own) return false;
    return inject_[0].HasFor(iso, detail::Slot().id, own) || inject_[1].HasFor(iso, detail::Slot().id, own);
}

detail::Task* ThreadPool::FindInteractiveWork(int self, const detail::IsolationState& iso) {
    if (self >= 0) {
        auto& deque = workers_[static_cast<size_t>(self)]->deque;
        // Own subtasks (pushed since the current task started), then an interactive task of an
        // outer frame; a bulk one stays for the thieves.
        if (detail::Task* t = deque.PopAbove(iso.floor)) return t;
        if (detail::Task* t = deque.PopIf([](unsigned tag) { return tag == kInteractiveTag; })) return t;
    } else {
        if (detail::Task* t = inject_[0].TryPopNewestOwn(detail::Slot().id, iso.seqFloor)) return t;
        if (detail::Task* t = inject_[1].TryPopNewestOwn(detail::Slot().id, iso.seqFloor)) return t;
    }
    if (detail::Task* t = TakeInjected(0, self)) return t;
    return StealFrom(self, true);
}

bool ThreadPool::HasInteractiveWork(int self, const detail::IsolationState& iso) noexcept {
    if (inject_[0].size.load(std::memory_order_seq_cst) != 0) return true;
    if (self >= 0) {
        const auto& deque = workers_[static_cast<size_t>(self)]->deque;
        if (!deque.Empty() &&
            (deque.BottomIndex() > iso.floor || deque.BottomTag() == static_cast<int>(kInteractiveTag)))
            return true;
    } else if (detail::Slot().seq > iso.seqFloor && inject_[1].HasNewestOwn(detail::Slot().id, iso.seqFloor)) {
        return true;
    }
    for (size_t i = 0; i < workers_.size(); ++i) {
        if (static_cast<int>(i) != self && workers_[i]->deque.TopTag() == static_cast<int>(kInteractiveTag)) return true;
    }
    return false;
}

ThreadPool::WaitContext ThreadPool::BeginWait(Priority priority) const noexcept {
    WaitContext c;
    c.scope = detail::CurrentIsolation();
    c.self = SelfIndex();
    const bool deep = detail::HelpDepth() >= kMaxHelpDepth;
    const bool hasWorkers = !workers_.empty();
    if (c.scope.node != nullptr || (deep && hasWorkers)) {
        c.mode = WaitMode::kIsolated;
    } else if (deep) {
        c.mode = WaitMode::kOwnFirst;
    } else if (priority == Priority::kInteractive || detail::CurrentPriority() == Priority::kInteractive) {
        c.mode = hasWorkers ? WaitMode::kInteractive : WaitMode::kInteractiveFirst;
    } else {
        c.mode = WaitMode::kAny;
    }
    return c;
}

bool ThreadPool::TryHelp(const WaitContext& ctx) {
    detail::Task* t = nullptr;
    const IsolationNode* inherit = nullptr;
    bool interactive = false;
    switch (ctx.mode) {
        case WaitMode::kAny:
            t = FindWork(ctx.self);
            break;
        case WaitMode::kIsolated:
            t = FindIsolatedWork(ctx.self, ctx.scope);
            inherit = ctx.scope.node;
            interactive = detail::CurrentPriority() == Priority::kInteractive;
            break;
        case WaitMode::kOwnFirst:
            t = FindOwnInjected(ctx.scope);
            if (t == nullptr) t = FindWork(ctx.self);
            break;
        case WaitMode::kInteractive:
            t = FindInteractiveWork(ctx.self, ctx.scope);
            interactive = true;
            break;
        case WaitMode::kInteractiveFirst:
            t = FindInteractiveWork(ctx.self, ctx.scope);
            interactive = t != nullptr;
            if (t == nullptr) t = FindWork(ctx.self);
            break;
    }
    if (t == nullptr) return false;
    helped_.fetch_add(1, std::memory_order_relaxed);
    RunTask(t, ctx.self, inherit, interactive);
    return true;
}

bool ThreadPool::HasHelpWork(const WaitContext& ctx) noexcept {
    switch (ctx.mode) {
        case WaitMode::kIsolated:
            return HasIsolatedWork(ctx.self, ctx.scope);
        case WaitMode::kInteractive:
            return HasInteractiveWork(ctx.self, ctx.scope);
        default:
            return HasWork();
    }
}

void ThreadPool::WorkerMain(uint32_t index) {
    tlsWorker.pool = this;
    tlsWorker.index = static_cast<int>(index);
    Worker& me = *workers_[index];
    SetCurrentThreadName(options_.name + "-" + std::to_string(index));
    if (!me.cpus.empty()) PinCurrentThread(me.cpus);
    const int self = static_cast<int>(index);

    bool woken = false;  // just woken from sleep
    for (;;) {
        if (detail::Task* t = FindWork(self)) {
            // A woken worker that found work passes the wake-up on while more work is queued, so
            // a burst of pushes ramps up every worker (pushes themselves wake one sleeper only
            // while nobody searches).
            if (woken && HasWork()) NotifyOne();
            woken = false;
            me.executed.fetch_add(1, std::memory_order_relaxed);
            RunTask(t, self, nullptr, false);
            continue;
        }
        woken = false;
        // Searching: spin looking for work. While anyone searches, pushes wake nobody.
        searching_.fetch_add(1, std::memory_order_seq_cst);
        detail::Task* found = nullptr;
        for (uint32_t round = 0; round < options_.spinRounds && found == nullptr; ++round) {
            for (uint32_t i = 0, n = 1u << std::min<uint32_t>(round / 8, 6); i < n; ++i) detail::CpuRelax();
            found = FindWork(self);
        }
        const uint32_t searchersLeft = searching_.fetch_sub(1, std::memory_order_seq_cst) - 1;
        if (found != nullptr) {
            // The last searcher to find work hands the search over to a sleeper: parallelism
            // ramps up one wake-up at a time instead of one per push.
            if (searchersLeft == 0) NotifyOne();
            me.executed.fetch_add(1, std::memory_order_relaxed);
            RunTask(found, self, nullptr, false);
            continue;
        }

        const uint32_t e = epoch_.load(std::memory_order_acquire);
        sleepers_.fetch_add(1, std::memory_order_seq_cst);
        if (HasWork()) {
            sleepers_.fetch_sub(1, std::memory_order_relaxed);
            continue;
        }
        if (stopping_.load(std::memory_order_seq_cst)) {
            sleepers_.fetch_sub(1, std::memory_order_relaxed);
            break;
        }
        me.sleeps.fetch_add(1, std::memory_order_relaxed);
        epoch_.wait(e, std::memory_order_acquire);
        sleepers_.fetch_sub(1, std::memory_order_relaxed);
        woken = true;
    }
    tlsWorker = WorkerTls{};
}

ThreadPoolStats ThreadPool::Stats() const noexcept {
    ThreadPoolStats s;
    for (const auto& w : workers_) {
        s.executed += w->executed.load(std::memory_order_relaxed);
        s.stolen += w->stolen.load(std::memory_order_relaxed);
        s.sleeps += w->sleeps.load(std::memory_order_relaxed);
    }
    s.helped = helped_.load(std::memory_order_relaxed);
    s.injected = injected_.load(std::memory_order_relaxed);
    return s;
}

// ---- default pool ------------------------------------------------------------------------------

size_t DefaultThreadCount() {
    if (const char* env = std::getenv("DDBT_THREADS"); env != nullptr && *env != '\0') {
        size_t v = 0;
        const auto r = std::from_chars(env, env + std::strlen(env), v);
        if (r.ec == std::errc() && v > 0) return std::min<size_t>(v, 1024);
    }
    const size_t n = AllowedCpus().size();
    return n > 0 ? n : 1;
}

namespace {
std::mutex gDefaultMutex;
std::atomic<ThreadPool*> gDefaultPool{nullptr};
size_t gDefaultThreads = 0;
}  // namespace

ThreadPool& DefaultPool() {
    if (ThreadPool* p = gDefaultPool.load(std::memory_order_acquire)) return *p;
    std::lock_guard lock(gDefaultMutex);
    if (ThreadPool* p = gDefaultPool.load(std::memory_order_relaxed)) return *p;
    ThreadPoolOptions o;
    o.threads = gDefaultThreads != 0 ? gDefaultThreads : DefaultThreadCount();
    // Not destroyed at exit on purpose: tasks may still run during static destruction of other
    // objects, and joining threads from a static destructor (or after main returns) is fragile.
    // ShutdownDefaultPool() destroys it explicitly.
    auto* p = new ThreadPool(o);
    gDefaultPool.store(p, std::memory_order_release);
    return *p;
}

bool SetDefaultPoolThreads(size_t threads) {
    std::lock_guard lock(gDefaultMutex);
    if (gDefaultPool.load(std::memory_order_relaxed) != nullptr) return false;
    gDefaultThreads = threads;
    return true;
}

void ShutdownDefaultPool() {
    std::unique_lock lock(gDefaultMutex);
    ThreadPool* p = gDefaultPool.load(std::memory_order_relaxed);
    if (p == nullptr) return;
    if (ThreadPool::Current() == p) {
        throw std::logic_error("ShutdownDefaultPool: called from a worker of the default pool");
    }
    gDefaultPool.store(nullptr, std::memory_order_release);
    lock.unlock();
    delete p;  // drains the queues, joins the workers
}

}  // namespace ddbt::exec
