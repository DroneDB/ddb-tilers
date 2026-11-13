// SPDX-License-Identifier: MPL-2.0
#include "copc/build/copc_writer.h"

#include <lazperf/lazperf.hpp>

#include <algorithm>
#include <cstring>
#include <exception>
#include <functional>
#include <unordered_map>

#include "copc/build/morton.h"
#include "core/io/out_file.h"
#include "core/util/endian.h"

namespace ddbt::copc::build {

namespace {

constexpr size_t kHeader14 = 375;
constexpr size_t kVlrHeader = 54;
constexpr size_t kEvlrHeader = 60;

class Bytes {
public:
    std::vector<uint8_t> v;
    template <typename T>
    void Put(T x) {
        const size_t n = v.size();
        v.resize(n + sizeof(T));
        StoreLE<T>(v.data() + n, x);
    }
    void PutStr(const std::string& s, size_t width) {
        const size_t n = v.size();
        v.resize(n + width, 0);
        std::memcpy(v.data() + n, s.data(), std::min(s.size(), width));
    }
    void PutRaw(const void* p, size_t n) {
        const uint8_t* b = static_cast<const uint8_t*>(p);
        v.insert(v.end(), b, b + n);
    }
};

void VlrHeader(Bytes& b, const std::string& user, uint16_t record, uint16_t length, const std::string& desc) {
    b.Put<uint16_t>(0);
    b.PutStr(user, 16);
    b.Put<uint16_t>(record);
    b.Put<uint16_t>(length);
    b.PutStr(desc, 32);
}

std::vector<uint8_t> LazVlrPayload(int format, uint16_t eb) {
    Bytes b;
    b.Put<uint16_t>(3);  // layered chunked
    b.Put<uint16_t>(0);  // arithmetic coder
    b.Put<uint8_t>(3);
    b.Put<uint8_t>(4);
    b.Put<uint16_t>(3);
    b.Put<uint32_t>(0);              // options
    b.Put<uint32_t>(0xFFFFFFFFu);    // variable-size chunks
    b.Put<int64_t>(-1);              // special EVLRs: none
    b.Put<int64_t>(-1);
    const uint16_t items = static_cast<uint16_t>(1 + (format >= 7 ? 1 : 0) + (eb ? 1 : 0));
    b.Put<uint16_t>(items);
    b.Put<uint16_t>(10);  // POINT14
    b.Put<uint16_t>(30);
    b.Put<uint16_t>(3);
    if (format == 7) {
        b.Put<uint16_t>(11);  // RGB14
        b.Put<uint16_t>(6);
        b.Put<uint16_t>(3);
    } else if (format == 8) {
        b.Put<uint16_t>(12);  // RGBNIR14
        b.Put<uint16_t>(8);
        b.Put<uint16_t>(3);
    }
    if (eb) {
        b.Put<uint16_t>(14);  // BYTE14
        b.Put<uint16_t>(eb);
        b.Put<uint16_t>(3);
    }
    return std::move(b.v);
}

struct NodeRec {
    int depth;
    uint64_t prefix;
    uint64_t offset;
    int32_t bytes;
    int32_t count;
};

uint64_t MapKey(int depth, uint64_t prefix) { return (static_cast<uint64_t>(depth) << 58) | prefix; }

}  // namespace

std::vector<uint8_t> CopcVlrBytes(const CopcHeaderFields& f, const Cube& cube, double spacing, uint64_t rootHierOffset,
                                  uint64_t rootHierSize, double gpsMin, double gpsMax, uint32_t* vlrCount) {
    Bytes b;
    uint32_t n = 0;
    VlrHeader(b, "copc", 1, 160, "COPC info VLR");
    b.Put<double>(cube.center[0]);
    b.Put<double>(cube.center[1]);
    b.Put<double>(cube.center[2]);
    b.Put<double>(cube.halfSize);
    b.Put<double>(spacing);
    b.Put<uint64_t>(rootHierOffset);
    b.Put<uint64_t>(rootHierSize);
    b.Put<double>(gpsMin);
    b.Put<double>(gpsMax);
    for (int i = 0; i < 11; ++i) b.Put<uint64_t>(0);
    ++n;
    const std::vector<uint8_t> laz = LazVlrPayload(f.format, f.extraBytes);
    VlrHeader(b, "laszip encoded", 22204, static_cast<uint16_t>(laz.size()), "lazperf variant");
    b.PutRaw(laz.data(), laz.size());
    ++n;
    if (!f.wkt.empty() && f.wkt.size() + 1 <= 65535) {
        VlrHeader(b, "LASF_Projection", 2112, static_cast<uint16_t>(f.wkt.size() + 1), "");
        b.PutRaw(f.wkt.data(), f.wkt.size());
        b.Put<uint8_t>(0);
        ++n;
    }
    if (!f.extraBytesVlr.empty() && f.extraBytesVlr.size() <= 65535) {
        VlrHeader(b, "LASF_Spec", 4, static_cast<uint16_t>(f.extraBytesVlr.size()), "");
        b.PutRaw(f.extraBytesVlr.data(), f.extraBytesVlr.size());
        ++n;
    }
    if (vlrCount) *vlrCount = n;
    return std::move(b.v);
}

struct CopcWriter::State {
    CopcHeaderFields f;
    io::OutFile out;
    uint32_t vlrCount = 0;
    uint64_t pointDataOffset = 0;
    std::vector<NodeRec> nodes;
    std::vector<lazperf::chunk> chunkTable;
    uint64_t points = 0;
};

CopcWriter::CopcWriter() : s_(std::make_unique<State>()) {}
CopcWriter::~CopcWriter() = default;

Expected<std::unique_ptr<CopcWriter>> CopcWriter::Create(const std::string& path, const CopcHeaderFields& fields,
                                                         uint64_t preallocate) {
    if (fields.format < 6 || fields.format > 8) return InvalidArgument("COPC point format must be 6, 7 or 8");
    std::unique_ptr<CopcWriter> w(new CopcWriter());
    State& s = *w->s_;
    s.f = fields;
    io::OutFileOptions o;
    o.preallocate = preallocate;
    DDBT_ASSIGN_OR_RETURN(s.out, io::OutFile::Create(path, o));
    const std::vector<uint8_t> vlrs = CopcVlrBytes(fields, Cube{}, 0, 0, 0, 0, 0, &s.vlrCount);
    s.pointDataOffset = kHeader14 + vlrs.size();
    std::vector<uint8_t> zeros(s.pointDataOffset + 8, 0);
    DDBT_TRY(s.out.Append(zeros));
    return w;
}

Status CopcWriter::AddNode(int depth, uint64_t prefix, uint32_t count, std::span<const uint8_t> chunk) {
    State& s = *s_;
    if (count == 0) {
        s.nodes.push_back({depth, prefix, 0, 0, 0});
        return Status();
    }
    if (chunk.size() > static_cast<size_t>(INT32_MAX)) return OverflowError("COPC node chunk larger than 2 GiB");
    const uint64_t off = s.out.Tell();
    DDBT_TRY(s.out.Append(chunk));
    s.nodes.push_back({depth, prefix, off, static_cast<int32_t>(chunk.size()), static_cast<int32_t>(count)});
    s.chunkTable.push_back({count, chunk.size()});
    s.points += count;
    return Status();
}

Status CopcWriter::Finish(const CopcFinalInfo& info, CopcWriteStats* stats) {
    State& s = *s_;
    const CopcHeaderFields& f = s.f;
    // Chunk table.
    const uint64_t tableOffset = s.out.Tell();
    {
        Bytes b;
        b.Put<uint32_t>(0);
        b.Put<uint32_t>(static_cast<uint32_t>(s.chunkTable.size()));
        try {
            lazperf::compress_chunk_table([&b](const unsigned char* p, size_t n) { b.PutRaw(p, n); }, s.chunkTable,
                                          true);
        } catch (const std::exception& e) {
            return InternalError(std::string("LAZ chunk table: ") + e.what());
        }
        DDBT_TRY(s.out.Append(b.v));
    }
    // Hierarchy.
    std::unordered_map<uint64_t, size_t> index;
    index.reserve(s.nodes.size() * 2);
    for (size_t i = 0; i < s.nodes.size(); ++i) index.emplace(MapKey(s.nodes[i].depth, s.nodes[i].prefix), i);
    if (index.find(MapKey(0, 0)) == index.end()) {
        // An empty cloud: a root without points.
        s.nodes.push_back({0, 0, 0, 0, 0});
        index.emplace(MapKey(0, 0), s.nodes.size() - 1);
    }
    // Subtree node counts (deepest first).
    std::vector<size_t> order(s.nodes.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return s.nodes[a].depth > s.nodes[b].depth; });
    std::vector<uint64_t> subtree(s.nodes.size(), 1);
    int maxDepth = 0;
    for (size_t i : order) {
        const NodeRec& n = s.nodes[i];
        maxDepth = std::max(maxDepth, n.depth);
        if (n.depth == 0) continue;
        auto it = index.find(MapKey(n.depth - 1, n.prefix >> 3));
        if (it == index.end()) return InternalError("COPC hierarchy: a node without its parent");
        subtree[it->second] += subtree[i];
    }

    const uint64_t evlrOffset = s.out.Tell();
    DDBT_TRY(s.out.Append(std::vector<uint8_t>(kEvlrHeader, 0)));
    const uint64_t hierStart = s.out.Tell();
    uint64_t pages = 0;
    constexpr int kLevelBreak = 4;
    constexpr uint64_t kMinHierarchySize = 50;
    struct PageRef {
        uint64_t offset;
        int32_t bytes;
    };
    auto entryBytes = [&](Bytes& b, const NodeRec& n, uint64_t off, int32_t bytes, int32_t count) {
        uint32_t x, y, z;
        MortonDecode(n.prefix, x, y, z);
        b.Put<int32_t>(n.depth);
        b.Put<int32_t>(static_cast<int32_t>(x));
        b.Put<int32_t>(static_cast<int32_t>(y));
        b.Put<int32_t>(static_cast<int32_t>(z));
        b.Put<uint64_t>(off);
        b.Put<int32_t>(bytes);
        b.Put<int32_t>(count);
    };
    Status err;
    std::function<PageRef(size_t)> emitPage = [&](size_t rootIdx) -> PageRef {
        Bytes page;
        const NodeRec& root = s.nodes[rootIdx];
        entryBytes(page, root, root.offset, root.bytes, root.count);
        const int stop = root.depth + kLevelBreak;
        std::function<void(size_t)> children = [&](size_t pi) {
            const NodeRec& p = s.nodes[pi];
            for (size_t c = 0; c < 8; ++c) {
                auto it = index.find(MapKey(p.depth + 1, p.prefix * 8 + static_cast<uint64_t>(c)));
                if (it == index.end()) continue;
                const NodeRec& ch = s.nodes[it->second];
                if (ch.depth != stop || subtree[it->second] <= kMinHierarchySize) {
                    entryBytes(page, ch, ch.offset, ch.bytes, ch.count);
                    children(it->second);
                } else {
                    const PageRef ref = emitPage(it->second);
                    entryBytes(page, ch, ref.offset, ref.bytes, -1);
                }
            }
        };
        children(rootIdx);
        PageRef ref{s.out.Tell(), static_cast<int32_t>(page.v.size())};
        if (err.ok()) err = s.out.Append(page.v);
        ++pages;
        return ref;
    };
    const PageRef rootPage = emitPage(index.at(MapKey(0, 0)));
    DDBT_TRY(err);
    const uint64_t hierEnd = s.out.Tell();
    {
        Bytes e;
        e.Put<uint16_t>(0);
        e.PutStr("copc", 16);
        e.Put<uint16_t>(1000);
        e.Put<uint64_t>(hierEnd - hierStart);
        e.PutStr("EPT Hierarchy", 32);
        DDBT_TRY(s.out.WriteAt(evlrOffset, e.v));
    }
    // VLRs with the root page, header, chunk table offset.
    uint32_t vlrCount = 0;
    const std::vector<uint8_t> vlrs = CopcVlrBytes(f, info.cube, info.spacing, rootPage.offset,
                                                   static_cast<uint64_t>(rootPage.bytes), info.gpsTimeMin, info.gpsTimeMax,
                                                   &vlrCount);
    if (kHeader14 + vlrs.size() != s.pointDataOffset) return InternalError("COPC VLR area changed size");
    Bytes h;
    h.PutStr("LASF", 4);
    h.Put<uint16_t>(f.fileSourceId);
    h.Put<uint16_t>(static_cast<uint16_t>(f.globalEncoding | (1u << 4)));
    h.PutRaw(f.projectId.data(), 16);
    h.Put<uint8_t>(1);
    h.Put<uint8_t>(4);
    h.PutStr(f.systemId, 32);
    h.PutStr(f.software, 32);
    h.Put<uint16_t>(f.creationDoy);
    h.Put<uint16_t>(f.creationYear);
    h.Put<uint16_t>(static_cast<uint16_t>(kHeader14));
    h.Put<uint32_t>(static_cast<uint32_t>(s.pointDataOffset));
    h.Put<uint32_t>(vlrCount);
    h.Put<uint8_t>(static_cast<uint8_t>(f.format | 0x80));
    h.Put<uint16_t>(static_cast<uint16_t>((f.format == 8 ? 38 : f.format == 7 ? 36 : 30) + f.extraBytes));
    h.Put<uint32_t>(0);  // legacy count: 0 for formats 6-10
    for (int i = 0; i < 5; ++i) h.Put<uint32_t>(0);
    for (size_t a = 0; a < 3; ++a) h.Put<double>(f.scale[a]);
    for (size_t a = 0; a < 3; ++a) h.Put<double>(f.offset[a]);
    for (size_t a = 0; a < 3; ++a) {
        h.Put<double>(info.max[a]);
        h.Put<double>(info.min[a]);
    }
    h.Put<uint64_t>(0);  // waveform data
    h.Put<uint64_t>(evlrOffset);
    h.Put<uint32_t>(1);
    h.Put<uint64_t>(s.points);
    for (size_t i = 0; i < 15; ++i) h.Put<uint64_t>(info.pointsByReturn[i]);
    if (h.v.size() != kHeader14) return InternalError("LAS 1.4 header is not 375 bytes");
    DDBT_TRY(s.out.WriteAt(0, h.v));
    DDBT_TRY(s.out.WriteAt(kHeader14, vlrs));
    uint8_t to[8];
    StoreLE<uint64_t>(to, tableOffset);
    DDBT_TRY(s.out.WriteAt(s.pointDataOffset, to));
    const uint64_t size = s.out.Size();
    DDBT_TRY(s.out.Finish());
    if (stats) {
        stats->points = s.points;
        stats->nodes = s.nodes.size();
        stats->chunks = s.chunkTable.size();
        stats->pages = pages;
        stats->fileSize = size;
        stats->maxDepth = maxDepth;
    }
    return Status();
}

}  // namespace ddbt::copc::build
