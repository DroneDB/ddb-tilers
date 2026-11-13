// SPDX-License-Identifier: MPL-2.0
#include "core/io/coalesce.h"

#include <algorithm>
#include <cstring>
#include <numeric>

#include "core/io/byte_source.h"
#include "core/util/checked_math.h"

namespace ddbt::io {

std::vector<CoalescedRead> CoalesceRanges(std::span<const ByteRange> ranges, const CoalesceOptions& options,
                                          std::vector<uint32_t>& order) {
    order.clear();
    order.reserve(ranges.size());
    for (uint32_t i = 0; i < ranges.size(); ++i) {
        if (ranges[i].length != 0) order.push_back(i);
    }
    std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
        return ranges[a].offset != ranges[b].offset ? ranges[a].offset < ranges[b].offset : a < b;
    });

    std::vector<CoalescedRead> out;
    uint64_t end = 0;  // end of the current merged read
    for (uint32_t k = 0; k < order.size(); ++k) {
        const ByteRange& r = ranges[order[k]];
        const uint64_t rEnd = r.offset + std::min(r.length, UINT64_MAX - r.offset);
        if (!out.empty()) {
            CoalescedRead& cur = out.back();
            const bool near = r.offset <= end || r.offset - end <= options.maxGap;
            const uint64_t newEnd = std::max(end, rEnd);
            if (near && newEnd - cur.offset <= options.maxLength) {
                end = newEnd;
                cur.length = end - cur.offset;
                ++cur.count;
                continue;
            }
        }
        out.push_back(CoalescedRead{r.offset, rEnd - r.offset, k, 1});
        end = rEnd;
    }
    return out;
}

Status ReadRanges(const ByteSource& source, std::span<const ByteRange> ranges, std::span<const std::span<uint8_t>> outs,
                  const CoalesceOptions& options) {
    if (ranges.size() != outs.size()) return InvalidArgument("ReadRanges: ranges and outputs differ in count");
    const uint64_t size = source.Size();
    for (size_t i = 0; i < ranges.size(); ++i) {
        if (outs[i].size() != ranges[i].length) return InvalidArgument("ReadRanges: output size differs from range length");
        if (!RangeInside(ranges[i].offset, ranges[i].length, size)) {
            return OutOfRange("ReadRanges: range " + std::to_string(ranges[i].offset) + "+" + std::to_string(ranges[i].length) +
                              " past the end of '" + std::string(source.Name()) + "'");
        }
    }

    if (const uint8_t* d = source.Data(); d != nullptr) {
        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i].length != 0) std::memcpy(outs[i].data(), d + ranges[i].offset, outs[i].size());
        }
        return Status::Ok();
    }

    std::vector<uint32_t> order;
    const std::vector<CoalescedRead> reads = CoalesceRanges(ranges, options, order);
    std::vector<uint8_t> staging;
    for (const CoalescedRead& rd : reads) {
        if (rd.count == 1) {
            const uint32_t i = order[rd.first];
            DDBT_TRY(source.Read(ranges[i].offset, outs[i]));
            continue;
        }
        staging.resize(static_cast<size_t>(rd.length));
        DDBT_TRY(source.Read(rd.offset, staging));
        for (uint32_t k = rd.first; k < rd.first + rd.count; ++k) {
            const uint32_t i = order[k];
            std::memcpy(outs[i].data(), staging.data() + (ranges[i].offset - rd.offset), outs[i].size());
        }
    }
    return Status::Ok();
}

}  // namespace ddbt::io
