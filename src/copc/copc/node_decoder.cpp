// SPDX-License-Identifier: MPL-2.0
#include "copc/copc/node_decoder.h"

#include <algorithm>
#include <limits>
#include <string>

#include "core/util/checked_math.h"

namespace ddbt::copc {
namespace {

std::string NodeName(const HierarchyNode& n) {
    return "COPC node " + std::to_string(n.key.d) + "-" + std::to_string(n.key.x) + "-" + std::to_string(n.key.y) +
           "-" + std::to_string(n.key.z);
}

inline uint32_t Channel8(uint16_t v, bool color16) noexcept {
    return color16 ? static_cast<uint32_t>(v >> 8) : std::min<uint32_t>(v, 255u);
}

}  // namespace

Expected<std::span<const uint8_t>> NodeChunk(const CopcFile& file, const HierarchyNode& node,
                                             std::vector<uint8_t>& scratch) {
    const io::ByteSource& src = file.source();
    if (node.byteSize < 0 || !RangeInside(node.offset, static_cast<uint64_t>(node.byteSize), src.Size()))
        return Corrupt(NodeName(node) + ": chunk range outside the file");
    const size_t size = static_cast<size_t>(node.byteSize);
    std::span<const uint8_t> view = src.View(node.offset, size);
    if (!view.empty() || size == 0) return view;
    scratch.resize(size);
    DDBT_TRY(src.Read(node.offset, scratch));
    return std::span<const uint8_t>(scratch);
}

Status DecodeNodeRaw(const CopcFile& file, const HierarchyNode& node, uint32_t layers, RawPoints& out) {
    if (node.IsLink() || node.pointCount < 0) return InvalidArgument(NodeName(node) + " is not a data node");
    const size_t n = static_cast<size_t>(node.pointCount);
    layers |= laz::kLayerXY;
    const bool z = (layers & laz::kLayerZ) != 0;
    const bool rgb = file.hasRgb() && (layers & laz::kLayerRgb) != 0;
    const bool nir = file.hasNir() && (layers & laz::kLayerNir) != 0;
    out.x.resize(n);
    out.y.resize(n);
    if (z) out.z.resize(n); else out.z.clear();
    if (rgb) {
        out.r.resize(n);
        out.g.resize(n);
        out.b.resize(n);
    } else {
        out.r.clear();
        out.g.clear();
        out.b.clear();
    }
    if (nir) out.nir.resize(n); else out.nir.clear();
    if (n == 0) return Status();

    thread_local std::vector<uint8_t> scratch;
    DDBT_ASSIGN_OR_RETURN(std::span<const uint8_t> chunk, NodeChunk(file, node, scratch));
    DDBT_ASSIGN_OR_RETURN(laz::ChunkDecoder * dec, laz::ThreadDecoder(file.pointFormat(), file.extraBytes(), layers));
    laz::SoaOut o;
    o.x = out.x.data();
    o.y = out.y.data();
    o.z = z ? out.z.data() : nullptr;
    if (rgb) {
        o.r = out.r.data();
        o.g = out.g.data();
        o.b = out.b.data();
    }
    o.nir = nir ? out.nir.data() : nullptr;
    Status s = dec->DecodeSoa(chunk, static_cast<uint32_t>(n), o);
    if (!s.ok()) return s.WithContext(NodeName(node));
    return Status();
}

Status NodeDecoder::DecodeRaw(const HierarchyNode& node, uint32_t layers, RawPoints& out) const {
    return DecodeNodeRaw(*file_, node, layers, out);
}

Expected<CompactNode> NodeDecoder::DecodeCompact(const HierarchyNode& node) const {
    bool color16 = false;
    if (file_->hasRgb()) {
        DDBT_ASSIGN_OR_RETURN(color16, file_->Uses16BitColor());
    }
    thread_local RawPoints raw;
    DDBT_TRY(DecodeNodeRaw(*file_, node, laz::kLayersXyzRgb, raw));
    return CompactNode::FromRaw(raw, color16);
}

CompactNode CompactNode::FromRaw(const RawPoints& raw, bool color16) {
    CompactNode c;
    const size_t n = raw.size();
    c.count_ = static_cast<uint32_t>(n);
    const std::vector<int32_t>* axes[3] = {&raw.x, &raw.y, &raw.z};
    size_t at = 0;
    for (int a = 0; a < 3; ++a) {
        const std::vector<int32_t>& v = *axes[a];
        int32_t lo = 0, hi = 0;
        if (!v.empty()) {
            const auto [mn, mx] = std::minmax_element(v.begin(), v.end());
            lo = *mn;
            hi = *mx;
        }
        c.base_[static_cast<size_t>(a)] = lo;
        const bool wide = static_cast<int64_t>(hi) - lo >= 65536;
        if (wide) c.wideMask_ = static_cast<uint8_t>(c.wideMask_ | (1u << a));
        c.axisOffset_[static_cast<size_t>(a)] = at;
        at = mem::AlignUp(at + n * (wide ? 4u : 2u), 64);
    }
    const bool color = !raw.r.empty();
    if (color) {
        c.colorOffset_ = at;
        at += n * 4;
    }
    c.bufBytes_ = std::max<size_t>(at, 64);
    c.buf_ = mem::MakeAlignedBytes(c.bufBytes_);
    for (int a = 0; a < 3; ++a) {
        const std::vector<int32_t>& v = *axes[a];
        uint8_t* dst = c.buf_.get() + c.axisOffset_[static_cast<size_t>(a)];
        if (v.empty()) {  // axis not decoded: zeros
            std::fill_n(dst, n * (c.wide(a) ? 4u : 2u), uint8_t{0});
            continue;
        }
        if (c.wide(a)) {
            std::copy(v.begin(), v.end(), reinterpret_cast<int32_t*>(dst));
        } else {
            const int32_t b = c.base_[static_cast<size_t>(a)];
            uint16_t* o = reinterpret_cast<uint16_t*>(dst);
            for (size_t i = 0; i < n; ++i) o[i] = static_cast<uint16_t>(v[i] - b);
        }
    }
    if (color) {
        uint32_t* o = reinterpret_cast<uint32_t*>(c.buf_.get() + c.colorOffset_);
        for (size_t i = 0; i < n; ++i)
            o[i] = Channel8(raw.r[i], color16) | (Channel8(raw.g[i], color16) << 8) |
                   (Channel8(raw.b[i], color16) << 16) | 0xFF000000u;
    }
    return c;
}

}  // namespace ddbt::copc
