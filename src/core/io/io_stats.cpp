// SPDX-License-Identifier: MPL-2.0
#include "core/io/io_stats.h"

#include "core/util/counters.h"

namespace ddbt::io {

namespace {

struct IoCounters {
    Counter& reads = Counters::Global().Get("io.reads");
    Counter& readBytes = Counters::Global().Get("io.read_bytes");
    Counter& mappedReads = Counters::Global().Get("io.mapped_reads");
    Counter& mappedReadBytes = Counters::Global().Get("io.mapped_read_bytes");
    Counter& callbackReads = Counters::Global().Get("io.callback_reads");
    Counter& callbackReadBytes = Counters::Global().Get("io.callback_read_bytes");
    Counter& writes = Counters::Global().Get("io.writes");
    Counter& writeBytes = Counters::Global().Get("io.write_bytes");
    Counter& prefetches = Counters::Global().Get("io.prefetches");
    Counter& opens = Counters::Global().Get("io.opens");
};

IoCounters& C() {
    static IoCounters* const c = new IoCounters();  // leaked like the registry it points into
    return *c;
}

}  // namespace

void IoStats::AddRead(uint64_t bytes) noexcept {
    C().reads.Add(1);
    C().readBytes.Add(bytes);
}

void IoStats::AddMappedRead(uint64_t bytes) noexcept {
    C().mappedReads.Add(1);
    C().mappedReadBytes.Add(bytes);
}

void IoStats::AddCallbackRead(uint64_t bytes) noexcept {
    C().callbackReads.Add(1);
    C().callbackReadBytes.Add(bytes);
}

void IoStats::AddWrite(uint64_t bytes) noexcept {
    C().writes.Add(1);
    C().writeBytes.Add(bytes);
}

void IoStats::AddPrefetch() noexcept { C().prefetches.Add(1); }
void IoStats::AddOpen() noexcept { C().opens.Add(1); }

IoStatsSnapshot IoStats::Snapshot() noexcept {
    IoCounters& c = C();
    IoStatsSnapshot s;
    s.reads = c.reads.Load();
    s.readBytes = c.readBytes.Load();
    s.mappedReads = c.mappedReads.Load();
    s.mappedReadBytes = c.mappedReadBytes.Load();
    s.callbackReads = c.callbackReads.Load();
    s.callbackReadBytes = c.callbackReadBytes.Load();
    s.writes = c.writes.Load();
    s.writeBytes = c.writeBytes.Load();
    s.prefetches = c.prefetches.Load();
    s.opens = c.opens.Load();
    return s;
}

void IoStats::Reset() noexcept {
    IoCounters& c = C();
    for (Counter* x : {&c.reads, &c.readBytes, &c.mappedReads, &c.mappedReadBytes, &c.callbackReads,
                       &c.callbackReadBytes, &c.writes, &c.writeBytes, &c.prefetches, &c.opens}) {
        x->Reset();
    }
}

IoStatsSnapshot operator-(const IoStatsSnapshot& b, const IoStatsSnapshot& a) noexcept {
    IoStatsSnapshot d;
    d.reads = b.reads - a.reads;
    d.readBytes = b.readBytes - a.readBytes;
    d.mappedReads = b.mappedReads - a.mappedReads;
    d.mappedReadBytes = b.mappedReadBytes - a.mappedReadBytes;
    d.callbackReads = b.callbackReads - a.callbackReads;
    d.callbackReadBytes = b.callbackReadBytes - a.callbackReadBytes;
    d.writes = b.writes - a.writes;
    d.writeBytes = b.writeBytes - a.writeBytes;
    d.prefetches = b.prefetches - a.prefetches;
    d.opens = b.opens - a.opens;
    return d;
}

}  // namespace ddbt::io
