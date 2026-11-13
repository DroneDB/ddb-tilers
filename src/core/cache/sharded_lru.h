// SPDX-License-Identifier: MPL-2.0
#pragma once

// ShardedLru<K, V>: concurrent LRU cache with a byte budget and single-flight loading.
//
//   ShardedLru<BlockKey, Block, BlockKeyHash, BlockCost> blocks(1ull << 30);   // 1 GiB
//   std::shared_ptr<const Block> b = blocks.GetOrLoad(key, [&] { return DecodeBlock(key); });
//
// - Keys are spread over N shards (default 64, a power of two) by a Fibonacci hash of
//   Hash(key); each shard has its own mutex, hash map and intrusive LRU list. The byte budget
//   is global, not split per shard: a split budget evicts from a shard that holds a few large
//   entries (decoded 1 MiB blocks under a 1 GiB / 64 budget: 16 per shard) while the cache as
//   a whole is far under budget. Over budget, the entry least recently used of all shards
//   (the oldest LRU tail, by a global use counter) is evicted, outside the inserting shard's
//   lock, one shard lock at a time. An entry whose cost exceeds the budget is returned but not
//   cached.
// - Values are std::shared_ptr<const V>: an entry evicted while someone uses it stays alive
//   until the last user drops it (the budget tracks cached bytes, not in-use bytes).
// - Cost(const V&) -> size_t gives an entry's bytes (default: sizeof(V), or size() * element
//   size for containers).
// - GetOrLoad is single-flight: concurrent misses on one key run the loader once; the others
//   wait for its result. A waiting pool worker runs other pool tasks meanwhile instead of
//   blocking (ThreadPool::HelpUntil, woken only by this flight or by work it may run); a
//   thread outside any pool blocks on a condition variable. If the loader throws, the
//   exception is rethrown to the loading thread and a copy of it to every waiter, nothing is
//   cached, and the next GetOrLoad tries again.
// - Waiters get their own exception object (WaiterThrower): no exception object is shared
//   between threads, so none is destroyed on one thread while another reads it (a race only in
//   theory, through libstdc++'s uninstrumented reference counts, but reported by TSan). The
//   standard exception types (bad_alloc, logic_error, runtime_error and their standard
//   subclasses but system_error) are rebuilt from their message; a class deriving from
//   WaiterCopyable rebuilds itself; any other exception is still shared with the waiters.
// - Erase, Clear and Put detach an in-flight load of their keys: callers already waiting for
//   it still get its result, but it is not cached, and later GetOrLoad calls do not join it
//   (they load again, or get the value Put stored). So no call made after Erase/Clear/Put
//   returns a value loaded before it.
// - Deadlock freedom: the loader runs inside an exec::IsolationScope. Task groups it creates
//   are isolated: while the loader (or any task of its tree) waits, the thread only runs
//   tasks of that tree, never unrelated work that might itself wait for this load. Waiters
//   outside the tree may help with anything. What remains forbidden is a cyclic dependency
//   between keys: a loader that (directly or through its tasks) needs its own key gets
//   std::logic_error when detected on the same thread, and deadlocks across threads.
// - An optional MemBudget is charged for cached bytes; when it is full, the shard evicts its
//   LRU entries to make room, and the new entry is not cached if that is not enough.

#include <algorithm>
#include <atomic>
#include <bit>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/cache/cache_stats.h"
#include "core/exec/thread_pool.h"
#include "core/mem/mem_budget.h"

namespace ddbt::cache {

namespace detail {

template <typename T, typename = void>
struct HasSizeAndValueType : std::false_type {};
template <typename T>
struct HasSizeAndValueType<T, std::void_t<decltype(std::declval<const T&>().size()), typename T::value_type>>
    : std::true_type {};

}  // namespace detail

/// An exception class whose single-flight waiters each get their own copy: MakeThrower() returns
/// a function that throws, at every call, a new exception of the same dynamic type and content,
/// built from data the function owns (copied strings: no state shared with this object). Each
/// derived class must override it, or its waiters get the base type (WaiterThrower then falls
/// back to sharing the exception).
class WaiterCopyable {
public:
    virtual ~WaiterCopyable() = default;
    virtual std::function<void()> MakeThrower() const = 0;
};

/// Called on the thread that caught `error` (single-flight leaders): a function that throws a new
/// copy of it at every call, for the waiters (see the header comment). Never throws: on failure,
/// and for exception types it cannot rebuild, the function rethrows `error` itself.
inline std::function<void()> WaiterThrower(const std::exception_ptr& error) noexcept {
    const auto shared = [error] { std::rethrow_exception(error); };
    try {
        std::function<void()> copy;
        try {
            std::rethrow_exception(error);
        } catch (const WaiterCopyable& c) {
            copy = c.MakeThrower();
        } catch (const std::bad_alloc& e) {
            if (typeid(e) == typeid(std::bad_alloc)) copy = [] { throw std::bad_alloc(); };
        } catch (const std::exception& e) {
            const std::type_info& t = typeid(e);
            std::string m = e.what();
            if (t == typeid(std::runtime_error)) copy = [m] { throw std::runtime_error(m); };
            else if (t == typeid(std::logic_error)) copy = [m] { throw std::logic_error(m); };
            else if (t == typeid(std::invalid_argument)) copy = [m] { throw std::invalid_argument(m); };
            else if (t == typeid(std::out_of_range)) copy = [m] { throw std::out_of_range(m); };
            else if (t == typeid(std::length_error)) copy = [m] { throw std::length_error(m); };
            else if (t == typeid(std::domain_error)) copy = [m] { throw std::domain_error(m); };
            else if (t == typeid(std::range_error)) copy = [m] { throw std::range_error(m); };
            else if (t == typeid(std::overflow_error)) copy = [m] { throw std::overflow_error(m); };
            else if (t == typeid(std::underflow_error)) copy = [m] { throw std::underflow_error(m); };
        } catch (...) {
        }
        if (!copy) return shared;
        // The copy must have the dynamic type of the original (a WaiterCopyable subclass that did
        // not override MakeThrower would give its base): checked once, here.
        const std::type_info* original = nullptr;
        const std::type_info* copied = nullptr;
        try {
            std::rethrow_exception(error);
        } catch (const std::exception& e) {
            original = &typeid(e);
        } catch (...) {
        }
        try {
            copy();
        } catch (const std::exception& e) {
            copied = &typeid(e);
        } catch (...) {
        }
        if (original == nullptr || copied == nullptr || *original != *copied) return shared;
        return copy;
    } catch (...) {
        return shared;
    }
}

/// Default cost: size() * sizeof(value_type) + sizeof(V) for containers, else sizeof(V).
template <typename V>
struct DefaultCost {
    size_t operator()(const V& v) const noexcept {
        if constexpr (detail::HasSizeAndValueType<V>::value) {
            return sizeof(V) + static_cast<size_t>(v.size()) * sizeof(typename V::value_type);
        } else {
            return sizeof(V);
        }
    }
};

struct ShardedLruOptions {
    /// Total bytes of cached values (sum of Cost) across all shards.
    size_t budgetBytes = size_t{256} << 20;
    /// Rounded up to a power of two.
    size_t shards = 64;
    /// Optional shared budget charged for cached bytes.
    mem::MemBudget* memBudget = nullptr;
};

template <typename K, typename V, typename Hash = std::hash<K>, typename Cost = DefaultCost<V>,
          typename KeyEq = std::equal_to<K>>
class ShardedLru {
public:
    using ValuePtr = std::shared_ptr<const V>;

    explicit ShardedLru(size_t budgetBytes, size_t shards = 64, Cost cost = Cost(), Hash hash = Hash())
        : ShardedLru(ShardedLruOptions{budgetBytes, shards, nullptr}, std::move(cost), std::move(hash)) {}

    explicit ShardedLru(const ShardedLruOptions& options, Cost cost = Cost(), Hash hash = Hash())
        : cost_(std::move(cost)), hash_(std::move(hash)), memBudget_(options.memBudget) {
        const size_t n = std::bit_ceil(std::max<size_t>(1, options.shards));
        shardBits_ = static_cast<unsigned>(std::countr_zero(n));
        shards_ = std::make_unique<Shard[]>(n);
        shardCount_ = n;
        SetBudget(options.budgetBytes);
    }

    ShardedLru(const ShardedLru&) = delete;
    ShardedLru& operator=(const ShardedLru&) = delete;

    ~ShardedLru() { Clear(); }

    /// Cached value or nullptr (counted as a miss).
    ValuePtr Get(const K& key) {
        Shard& s = ShardOf(key);
        std::lock_guard lock(s.mutex);
        auto it = s.map.find(key);
        if (it == s.map.end()) {
            ++s.stats.misses;
            return nullptr;
        }
        ++s.stats.hits;
        Touch(s, it->second);
        return it->second.value;
    }

    /// True if `key` is cached (no LRU update, no stats).
    bool Contains(const K& key) const {
        const Shard& s = ShardOf(key);
        std::lock_guard lock(s.mutex);
        return s.map.find(key) != s.map.end();
    }

    /// Inserts or replaces. Returns `value` (cached or not). A load of `key` in flight is
    /// detached: it will not overwrite `value`.
    ValuePtr Put(const K& key, ValuePtr value) {
        if (!value) return value;
        const size_t cost = cost_(*value);
        Shard& s = ShardOf(key);
        std::vector<ValuePtr> graveyard;  // destroyed after the locks are released
        {
            std::lock_guard lock(s.mutex);
            s.inflight.erase(key);
            InsertLocked(s, key, value, cost, graveyard);
        }
        EnforceBudget(graveyard);
        return value;
    }

    ValuePtr Put(const K& key, V value) { return Put(key, std::make_shared<const V>(std::move(value))); }

    /// Removes `key`; an in-flight load of it is detached (not cached, not joined by later
    /// calls). True if it was cached.
    bool Erase(const K& key) {
        Shard& s = ShardOf(key);
        ValuePtr victim;
        std::lock_guard lock(s.mutex);
        s.inflight.erase(key);
        auto it = s.map.find(key);
        if (it == s.map.end()) return false;
        victim = RemoveLocked(s, it);
        return true;
    }

    /// Removes everything; in-flight loads are detached (not cached, not joined by later
    /// calls).
    void Clear() {
        for (size_t i = 0; i < shardCount_; ++i) {
            Shard& s = shards_[i];
            std::vector<ValuePtr> graveyard;
            std::lock_guard lock(s.mutex);
            s.inflight.clear();
            graveyard.reserve(s.map.size());
            while (s.head.next != &s.head) {
                Node* n = s.head.prev;
                graveyard.push_back(RemoveLocked(s, s.map.find(*n->key)));
            }
        }
    }

    /// Cached value, or the result of `loader()` (V, std::shared_ptr<V> or
    /// std::shared_ptr<const V>), loaded once for concurrent callers. Rethrows loader
    /// exceptions. A null shared_ptr from the loader is returned but not cached.
    template <typename Loader>
    ValuePtr GetOrLoad(const K& key, Loader&& loader);

    /// Changes the total budget; a cache over its new budget evicts at once.
    void SetBudget(size_t budgetBytes) {
        budget_.store(budgetBytes, std::memory_order_relaxed);
        std::vector<ValuePtr> graveyard;
        EnforceBudget(graveyard);
    }

    size_t BudgetBytes() const noexcept { return budget_.load(std::memory_order_relaxed); }
    size_t ShardCount() const noexcept { return shardCount_; }

    CacheStats Stats() const {
        CacheStats total;
        for (size_t i = 0; i < shardCount_; ++i) {
            const Shard& s = shards_[i];
            std::lock_guard lock(s.mutex);
            const CacheStats& x = s.stats;
            total.hits += x.hits;
            total.misses += x.misses;
            total.loads += x.loads;
            total.coalesced += x.coalesced;
            total.loadFailures += x.loadFailures;
            total.inserts += x.inserts;
            total.evictions += x.evictions;
            total.rejected += x.rejected;
            total.entries += s.map.size();
            total.bytes += s.bytes;
        }
        return total;
    }

private:
    struct Node {
        Node* prev = this;
        Node* next = this;
        const K* key = nullptr;
        ValuePtr value;
        size_t cost = 0;
        uint64_t lastUse = 0;  // useClock_ at the last insert or hit
    };

    struct Flight {
        std::mutex mutex;
        std::condition_variable cv;
        bool done = false;
        std::atomic<bool> doneFlag{false};
        ValuePtr value;
        std::function<void()> throwError;  // set when the loader failed: throws a waiter's copy
        std::vector<exec::detail::Parker*> parkers;  // parked helping waiters to wake
    };

    struct alignas(64) Shard {
        mutable std::mutex mutex;
        std::unordered_map<K, Node, Hash, KeyEq> map;
        std::unordered_map<K, std::shared_ptr<Flight>, Hash, KeyEq> inflight;
        Node head;  // sentinel: head.next = most recent, head.prev = least recent
        size_t bytes = 0;
        CacheStats stats;
    };

    size_t ShardIndex(const K& key) const noexcept {
        if (shardBits_ == 0) return 0;
        const uint64_t h = static_cast<uint64_t>(hash_(key)) * 0x9E3779B97F4A7C15ULL;
        return static_cast<size_t>(h >> (64 - shardBits_));
    }
    Shard& ShardOf(const K& key) noexcept { return shards_[ShardIndex(key)]; }
    const Shard& ShardOf(const K& key) const noexcept { return shards_[ShardIndex(key)]; }

    static void Unlink(Node& n) noexcept {
        n.prev->next = n.next;
        n.next->prev = n.prev;
        n.prev = n.next = &n;
    }
    static void LinkFront(Shard& s, Node& n) noexcept {
        n.next = s.head.next;
        n.prev = &s.head;
        s.head.next->prev = &n;
        s.head.next = &n;
    }
    void Touch(Shard& s, Node& n) noexcept {
        n.lastUse = useClock_.fetch_add(1, std::memory_order_relaxed);
        if (s.head.next == &n) return;
        Unlink(n);
        LinkFront(s, n);
    }

    ValuePtr RemoveLocked(Shard& s, typename std::unordered_map<K, Node, Hash, KeyEq>::iterator it) {
        Node& n = it->second;
        Unlink(n);
        s.bytes -= n.cost;
        total_.fetch_sub(n.cost, std::memory_order_relaxed);
        if (memBudget_ != nullptr) memBudget_->Release(n.cost);
        ValuePtr v = std::move(n.value);
        s.map.erase(it);
        return v;
    }

    // Evicts the least recently used entry of all shards until the cache fits its budget. Called
    // with no shard lock held; takes one shard lock at a time.
    void EnforceBudget(std::vector<ValuePtr>& graveyard) {
        while (total_.load(std::memory_order_relaxed) > budget_.load(std::memory_order_relaxed)) {
            size_t victimShard = shardCount_;
            uint64_t oldest = UINT64_MAX;
            for (size_t i = 0; i < shardCount_; ++i) {
                Shard& s = shards_[i];
                std::lock_guard lock(s.mutex);
                const Node* tail = s.head.prev;
                if (tail != &s.head && tail->lastUse < oldest) {
                    oldest = tail->lastUse;
                    victimShard = i;
                }
            }
            if (victimShard == shardCount_) return;  // empty
            Shard& s = shards_[victimShard];
            std::lock_guard lock(s.mutex);
            Node* victim = s.head.prev;
            if (victim == &s.head) continue;  // emptied meanwhile: look again
            const auto it = s.map.find(*victim->key);
            if (it == s.map.end()) continue;  // cannot happen: the LRU list holds mapped nodes only
            graveyard.push_back(RemoveLocked(s, it));
            ++s.stats.evictions;
        }
    }

    void InsertLocked(Shard& s, const K& key, const ValuePtr& value, size_t cost, std::vector<ValuePtr>& graveyard) {
        if (cost > budget_.load(std::memory_order_relaxed)) {
            ++s.stats.rejected;
            if (auto it = s.map.find(key); it != s.map.end()) graveyard.push_back(RemoveLocked(s, it));
            return;
        }
        if (auto it = s.map.find(key); it != s.map.end()) graveyard.push_back(RemoveLocked(s, it));
        if (memBudget_ != nullptr) {
            // Make room under the shared budget with this shard's own LRU entries.
            while (!memBudget_->TryReserve(cost)) {
                Node* victim = s.head.prev;
                if (victim == &s.head) {
                    ++s.stats.rejected;
                    return;
                }
                graveyard.push_back(RemoveLocked(s, s.map.find(*victim->key)));
                ++s.stats.evictions;
            }
        }
        typename std::unordered_map<K, Node, Hash, KeyEq>::iterator it;
        try {
            it = s.map.try_emplace(key).first;
        } catch (...) {
            if (memBudget_ != nullptr) memBudget_->Release(cost);
            throw;
        }
        Node& n = it->second;
        n.key = &it->first;
        n.value = value;
        n.cost = cost;
        n.lastUse = useClock_.fetch_add(1, std::memory_order_relaxed);
        LinkFront(s, n);
        s.bytes += cost;
        total_.fetch_add(cost, std::memory_order_relaxed);
        ++s.stats.inserts;
    }

    template <typename R>
    static ValuePtr ToValuePtr(R&& r) {
        using T = std::decay_t<R>;
        if constexpr (std::is_same_v<T, ValuePtr> || std::is_same_v<T, std::shared_ptr<V>>) {
            return ValuePtr(std::forward<R>(r));
        } else {
            return std::make_shared<const V>(std::forward<R>(r));
        }
    }

    template <typename Loader>
    static ValuePtr RunLoader(Loader& loader, const Flight* flight) {
        exec::IsolationScope scope(flight);
        return ToValuePtr(loader());
    }

    static ValuePtr WaitFor(Flight& f) {
        if (exec::ThreadPool* pool = exec::ThreadPool::Current()) {
            // value / error are written before doneFlag is released.
            pool->HelpUntil([&f] { return f.doneFlag.load(std::memory_order_acquire); },
                            [&f](exec::detail::Parker& parker) {
                                std::lock_guard lock(f.mutex);
                                if (f.done) return true;
                                if (std::find(f.parkers.begin(), f.parkers.end(), &parker) == f.parkers.end()) {
                                    f.parkers.push_back(&parker);
                                }
                                return false;
                            });
        } else {
            std::unique_lock lock(f.mutex);
            f.cv.wait(lock, [&f] { return f.done; });
        }
        if (f.throwError) f.throwError();
        return f.value;
    }

    Cost cost_;
    Hash hash_;
    mem::MemBudget* memBudget_;
    std::unique_ptr<Shard[]> shards_;
    size_t shardCount_ = 1;
    unsigned shardBits_ = 0;
    std::atomic<size_t> budget_{0};
    std::atomic<size_t> total_{0};       // bytes cached in all shards
    std::atomic<uint64_t> useClock_{0};  // global use order of the entries (Node::lastUse)
};

template <typename K, typename V, typename Hash, typename Cost, typename KeyEq>
template <typename Loader>
typename ShardedLru<K, V, Hash, Cost, KeyEq>::ValuePtr ShardedLru<K, V, Hash, Cost, KeyEq>::GetOrLoad(const K& key,
                                                                                                    Loader&& loader) {
    Shard& s = ShardOf(key);
    std::shared_ptr<Flight> flight;
    bool leader = false;
    {
        std::lock_guard lock(s.mutex);
        if (auto it = s.map.find(key); it != s.map.end()) {
            ++s.stats.hits;
            Touch(s, it->second);
            return it->second.value;
        }
        ++s.stats.misses;
        if (auto f = s.inflight.find(key); f != s.inflight.end()) {
            flight = f->second;
            ++s.stats.coalesced;
        } else {
            flight = std::make_shared<Flight>();
            s.inflight.emplace(key, flight);
            leader = true;
            ++s.stats.loads;
        }
    }

    if (!leader) {
        if (exec::IsolationScope::Inside(flight.get())) {
            throw std::logic_error("ShardedLru::GetOrLoad: the loader of a key needs that same key");
        }
        return WaitFor(*flight);
    }

    ValuePtr value;
    std::exception_ptr error;
    try {
        value = RunLoader(loader, flight.get());
    } catch (...) {
        error = std::current_exception();
    }
    // The flight must complete whatever happens below, or its waiters would wait forever: a
    // failure to cache (allocation) is reported to this caller after the waiters got the value.
    std::exception_ptr cacheError;
    try {
        const size_t cost = (!error && value) ? cost_(*value) : 0;
        std::vector<ValuePtr> graveyard;  // destroyed after the locks are released
        {
            std::lock_guard lock(s.mutex);
            // Erase / Clear / Put detached the flight if it is no longer listed: do not cache then.
            bool attached = false;
            if (auto f = s.inflight.find(key); f != s.inflight.end() && f->second == flight) {
                s.inflight.erase(f);
                attached = true;
            }
            if (error) {
                ++s.stats.loadFailures;
            } else if (value && attached) {
                InsertLocked(s, key, value, cost, graveyard);
            }
        }
        EnforceBudget(graveyard);
    } catch (...) {
        cacheError = std::current_exception();
        std::lock_guard lock(s.mutex);
        if (auto f = s.inflight.find(key); f != s.inflight.end() && f->second == flight) s.inflight.erase(f);
    }
    // The waiters' copies of the error: built here, on the thread that caught it.
    std::function<void()> throwError = error ? WaiterThrower(error) : std::function<void()>();
    std::vector<exec::detail::Parker*> parked;
    {
        std::lock_guard lock(flight->mutex);
        flight->value = value;
        flight->throwError = std::move(throwError);
        flight->done = true;
        flight->doneFlag.store(true, std::memory_order_release);
        parked.swap(flight->parkers);
    }
    flight->cv.notify_all();
    // Only the threads waiting for this flight (parkers are never freed).
    for (exec::detail::Parker* p : parked) p->Unpark();
    if (error) std::rethrow_exception(error);
    if (cacheError) std::rethrow_exception(cacheError);
    return value;
}

}  // namespace ddbt::cache
