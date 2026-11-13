// SPDX-License-Identifier: MPL-2.0
#include "copc/build/node_encoder.h"

#include <lazperf/lazperf.hpp>

#include <exception>
#include <string>

namespace ddbt::copc::build {

Status EncodeNode(int format, int extraBytes, const uint8_t* records, size_t count, std::vector<uint8_t>& out) {
    out.clear();
    if (format < 6 || format > 8) return InvalidArgument("EncodeNode: point format " + std::to_string(format));
    const size_t rs = (format == 8 ? 38u : format == 7 ? 36u : 30u) + static_cast<size_t>(extraBytes);
    try {
        lazperf::OutputCb cb = [&out](const unsigned char* b, size_t n) { out.insert(out.end(), b, b + n); };
        lazperf::las_compressor::ptr c = lazperf::build_las_compressor(cb, format, static_cast<size_t>(extraBytes));
        const char* p = reinterpret_cast<const char*>(records);
        for (size_t i = 0; i < count; ++i, p += rs) c->compress(p);
        c->done();
    } catch (const std::bad_alloc&) {
        throw;
    } catch (const std::exception& e) {
        return InternalError(std::string("LAZ compress: ") + e.what());
    }
    return Status();
}

}  // namespace ddbt::copc::build
