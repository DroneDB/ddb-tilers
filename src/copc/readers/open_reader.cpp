// SPDX-License-Identifier: MPL-2.0
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <string>

#include "copc/readers/e57_reader.h"
#include "copc/readers/las_reader.h"
#include "copc/readers/ply_reader.h"
#include "copc/readers/text_reader.h"
#include "copc/readers/point_reader.h"
#include "core/io/file.h"
#include "core/util/endian.h"

namespace ddbt::copc::readers {

namespace {

std::string LowerExtension(const std::string& path) {
    const size_t slash = path.find_last_of("/\\");
    const size_t dot = path.find_last_of('.');
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) return {};
    std::string e = path.substr(dot + 1);
    for (char& c : e) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return e;
}

}  // namespace

Expected<std::unique_ptr<PointReader>> OpenReader(const std::string& path, const ReaderOptions& options) {
    DDBT_ASSIGN_OR_RETURN(io::File f, io::File::Open(path));
    uint8_t magic[8] = {};
    const uint64_t size = f.Size();
    if (size >= 4) DDBT_TRY(f.ReadAt(0, std::span<uint8_t>(magic, static_cast<size_t>(std::min<uint64_t>(size, 8)))));
    if (std::memcmp(magic, "LASF", 4) == 0) {
        DDBT_ASSIGN_OR_RETURN(std::unique_ptr<LasReader> r, LasReader::Open(path, options));
        return std::unique_ptr<PointReader>(std::move(r));
    }
    const std::string ext = LowerExtension(path);
    if (std::memcmp(magic, "ASTM-E57", 8) == 0) {
        DDBT_ASSIGN_OR_RETURN(std::unique_ptr<E57Reader> r, E57Reader::Open(path, options));
        return std::unique_ptr<PointReader>(std::move(r));
    }
    if (std::memcmp(magic, "ply", 3) == 0) {
        DDBT_ASSIGN_OR_RETURN(std::unique_ptr<PlyReader> r, PlyReader::Open(path, options));
        return std::unique_ptr<PointReader>(std::move(r));
    }
    if (ext == "pts" || ext == "xyz" || ext == "txt" || ext == "csv") {
        DDBT_ASSIGN_OR_RETURN(std::unique_ptr<TextReader> r, TextReader::Open(path, ext == "pts", options));
        return std::unique_ptr<PointReader>(std::move(r));
    }
    return Unsupported(path + ": not a supported point cloud (extension '" + ext + "')");
}

Status QuantizeXyz(const OutputSpec& spec, const double* xyz, size_t count, uint8_t* records) {
    const size_t rs = spec.RecordSize();
    for (size_t i = 0; i < count; ++i, records += rs) {
        for (size_t a = 0; a < 3; ++a) {
            // As PDAL's LAS writer: round((v - offset) / scale).
            const double v = std::round((xyz[3 * i + a] - spec.offset[a]) / spec.scale[a]);
            if (!(v >= -2147483648.0 && v <= 2147483647.0))
                return OutOfRange("coordinate " + std::to_string(xyz[3 * i + a]) + " does not fit the LAS scale / offset");
            StoreLE<int32_t>(records + 4 * a, static_cast<int32_t>(v));
        }
    }
    return Status();
}

}  // namespace ddbt::copc::readers
