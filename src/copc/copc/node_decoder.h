// SPDX-License-Identifier: MPL-2.0
//
// NodeDecoder: decodes COPC nodes (one LAZ chunk each) of a CopcFile. Const and thread-safe:
// each thread uses its own decoder from a thread-local pool keyed by (format, extra bytes,
// layers) (laz::ThreadDecoder), reset between chunks instead of rebuilt.
//
// Two outputs:
//  - RawPoints: structure of arrays, int32 x/y/z (LAS integers: value = int * scale + offset)
//    and uint16 r/g/b(/nir), straight from the layered decoder.
//  - CompactNode: what a node cache keeps. Per axis, uint16 offsets from a base int32 when the
//    node's integer extent fits (max - min < 65536), else the int32 values; colour as RGBA8
//    (one uint32 per point, R in the low byte, A = 255) reduced with the per-cloud 16-bit
//    decision of CopcFile::Uses16BitColor(); no colour array for format 6. 10 bytes per point
//    for a typical coloured node (6 without colour, up to 16).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "copc/copc/copc_file.h"
#include "copc/copc/hierarchy.h"
#include "copc/laz/chunk_decoder.h"
#include "core/mem/aligned.h"
#include "core/util/status.h"

namespace ddbt::copc {

struct RawPoints {
    std::vector<int32_t> x, y, z;
    std::vector<uint16_t> r, g, b, nir;  // empty when not decoded / absent
    size_t size() const noexcept { return x.size(); }
};

class CompactNode {
public:
    CompactNode() = default;
    CompactNode(CompactNode&&) noexcept = default;
    CompactNode& operator=(CompactNode&&) noexcept = default;

    uint32_t size() const noexcept { return count_; }
    /// Integer coordinate base per axis (the minimum of the node's points).
    const std::array<int32_t, 3>& base() const noexcept { return base_; }
    /// Axis a (0 x, 1 y, 2 z) stored as int32 values (else uint16 offsets from base()[a]).
    bool wide(int a) const noexcept { return (wideMask_ >> a) & 1u; }
    const uint16_t* offsets16(int a) const noexcept {
        return wide(a) ? nullptr : reinterpret_cast<const uint16_t*>(buf_.get() + axisOffset_[static_cast<size_t>(a)]);
    }
    const int32_t* values32(int a) const noexcept {
        return wide(a) ? reinterpret_cast<const int32_t*>(buf_.get() + axisOffset_[static_cast<size_t>(a)]) : nullptr;
    }
    /// LAS integer coordinate of point i on axis a.
    int32_t Coord(int a, size_t i) const noexcept {
        return wide(a) ? values32(a)[i] : static_cast<int32_t>(base_[static_cast<size_t>(a)] + offsets16(a)[i]);
    }
    bool hasColor() const noexcept { return colorOffset_ != kNone; }
    /// RGBA8 per point (R | G << 8 | B << 16 | A << 24), nullptr without colour.
    const uint32_t* rgba() const noexcept {
        return hasColor() ? reinterpret_cast<const uint32_t*>(buf_.get() + colorOffset_) : nullptr;
    }
    /// Bytes held (object and buffer): what a cache charges.
    size_t Bytes() const noexcept { return sizeof(CompactNode) + bufBytes_; }

    /// Builds the compact form of decoded points (colour arrays empty: no colour).
    static CompactNode FromRaw(const RawPoints& raw, bool color16);

private:
    static constexpr size_t kNone = ~size_t{0};
    mem::AlignedBytes buf_;
    size_t bufBytes_ = 0;
    std::array<size_t, 3> axisOffset_{};
    size_t colorOffset_ = kNone;
    std::array<int32_t, 3> base_{};
    uint32_t count_ = 0;
    uint8_t wideMask_ = 0;
};

class NodeDecoder {
public:
    explicit NodeDecoder(std::shared_ptr<const CopcFile> file) : file_(std::move(file)) {}

    const CopcFile& file() const noexcept { return *file_; }

    /// Decodes the layers `layers` (laz::kLayer* bits; XY always) of `node` into `out` (resized
    /// to the node's point count; colour / NIR arrays only when decoded).
    Status DecodeRaw(const HierarchyNode& node, uint32_t layers, RawPoints& out) const;

    /// XYZ (+RGB when the format has it) in the compact form.
    Expected<CompactNode> DecodeCompact(const HierarchyNode& node) const;

private:
    std::shared_ptr<const CopcFile> file_;
};

/// The node's chunk bytes: a view of the mapping when the source is memory-resident, else read
/// into `scratch`. Corrupt when the range is outside the file.
Expected<std::span<const uint8_t>> NodeChunk(const CopcFile& file, const HierarchyNode& node,
                                             std::vector<uint8_t>& scratch);

/// NodeDecoder::DecodeRaw without a shared owner.
Status DecodeNodeRaw(const CopcFile& file, const HierarchyNode& node, uint32_t layers, RawPoints& out);

}  // namespace ddbt::copc
