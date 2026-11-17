// SPDX-License-Identifier: MPL-2.0
#include "copc/build/node_encoder.h"

#include <lazperf/lazperf.hpp>

#include <cstring>
#include <exception>
#include <string>

#include "copc/las/point_format.h"

namespace ddbt::copc::build {

Status EncodeNode(int format, int recordFormat, int extraBytes, const uint8_t* records, size_t count,
                  std::vector<uint8_t>& out) {
    out.clear();
    if (format < 6 || recordFormat < format || recordFormat > 8)
        return InvalidArgument("EncodeNode: point format " + std::to_string(format) + " of records of format " +
                               std::to_string(recordFormat));
    const size_t base = las::BaseRecordSize(format), recordBase = las::BaseRecordSize(recordFormat);
    const size_t eb = static_cast<size_t>(extraBytes);
    const size_t rs = recordBase + eb;
    try {
        lazperf::OutputCb cb = [&out](const unsigned char* b, size_t n) { out.insert(out.end(), b, b + n); };
        lazperf::las_compressor::ptr c = lazperf::build_las_compressor(cb, format, eb);
        const char* p = reinterpret_cast<const char*>(records);
        if (base == recordBase) {
            for (size_t i = 0; i < count; ++i, p += rs) c->compress(p);
        } else {
            std::vector<char> point(base + eb);
            for (size_t i = 0; i < count; ++i, p += rs) {
                std::memcpy(point.data(), p, base);
                std::memcpy(point.data() + base, p + recordBase, eb);
                c->compress(point.data());
            }
        }
        c->done();
    } catch (const std::bad_alloc&) {
        throw;
    } catch (const std::exception& e) {
        return InternalError(std::string("LAZ compress: ") + e.what());
    }
    return Status();
}

}  // namespace ddbt::copc::build
