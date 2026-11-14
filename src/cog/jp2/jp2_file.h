// SPDX-License-Identifier: MPL-2.0
//
// The boxes of a JPEG 2000 file as GDAL's JPEG 2000 drivers read them (gcore/gdaljp2box.cpp,
// gcore/gdaljp2metadata.cpp, gcore/gdaljp2abstractdataset.cpp, frmts/opjlike): where the
// codestream is, the colour boxes of the header (colr, pclr, cdef), and the georeferencing,
// with GDAL's default priority (GDAL_GEOREF_SOURCES = PAM,INTERNAL,WORLDFILE without PAM, which
// libddb disables): a GeoJP2 box (the degenerate GeoTIFF of the GeoTIFF UUID box), else the
// GMLJP2 coverage description, else the MSIG world file box; then a world file next to the file
// when there is no geotransform yet. A raw codestream (.j2k) has no boxes: only its world file.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "cog/tiff/geotiff.h"
#include "core/io/byte_source.h"
#include "core/util/status.h"

namespace ddbt::cog::jp2 {

/// One box: its 4-character type, where its data starts and how long it is.
struct Box {
    std::string type;            // "jp2h", "uuid", ...
    std::array<uint8_t, 16> uuid{};  // uuid boxes: the UUID (the data follows it)
    uint64_t offset = 0;         // the box header
    uint64_t length = 0;         // the whole box
    uint64_t dataOffset = 0;
    uint64_t DataLength() const noexcept { return length - (dataOffset - offset); }
    /// asoc, jp2h, res and jumb hold boxes.
    bool IsSuperBox() const noexcept;
};

/// The boxes in [begin, end) of `src` in order (GDALJP2Box's ReadFirst / ReadNext): a box of
/// length 0 runs to the end, reading stops at an unreadable or invalid box header.
std::vector<Box> ReadBoxes(const io::ByteSource& src, uint64_t begin, uint64_t end);
/// The boxes inside a superbox (none for another box).
std::vector<Box> Children(const io::ByteSource& src, const Box& parent);

/// GDAL's colour space of the image (eColorSpace).
enum class ColourSpace : uint8_t { kUnknown, kSrgb, kGray, kSycc };

/// What GDAL reads from a JPEG 2000 file besides the samples.
struct Jp2File {
    bool jp2 = false;  // a JP2 file (boxes), else a raw codestream
    uint64_t codestreamOffset = 0, codestreamLength = 0;

    // The colr box of the header (enumerated colour spaces): set when present.
    std::optional<ColourSpace> colourSpace;
    // The pclr box GDAL turns into a colour table (up to 256 8-bit RGB / RGBA entries).
    std::optional<std::vector<std::array<uint8_t, 4>>> palette;
    // The cdef boxes, as stored (interpreted once the number of components is known).
    std::vector<std::vector<uint8_t>> channelDefinitions;

    // Georeferencing boxes.
    std::vector<std::vector<uint8_t>> geoTiffBoxes;  // at most 2
    std::vector<uint8_t> msig;                       // an MSIG world file box
    std::vector<std::pair<std::string, std::string>> labelledXml;  // gml.data: label -> XML

    // Metadata GDAL reports in the default domain.
    std::string gdalMultiDomainMetadata;  // the GDAL metadata xml box
    std::optional<std::array<double, 2>> resolution;  // the resd / resc box: x, y in pixels/cm
};

/// GDAL's identification of a JPEG 2000 file: a codestream (SOC and SIZ markers) or a JP2
/// signature box, in the first 16 bytes.
bool IsJpeg2000(const io::ByteSource& src);

/// Reads the boxes of `src`. NotFound-like Corrupt when there is no codestream ("No code-stream
/// in JP2 file").
Expected<Jp2File> ReadJp2File(const io::ByteSource& src);

/// The interpretation of the cdef boxes GDAL makes for `bands` components (each box of the right
/// size in turn): the components that are red, green, blue (by association; -1 when a box names
/// none) and alpha (-1: none).
struct ChannelRoles {
    int red = 0, green = 1, blue = 2, alpha = -1;
};
ChannelRoles ReadChannelDefinitions(const std::vector<std::vector<uint8_t>>& boxes, int bands);

/// The georeferencing GDAL gives the file: GeoJP2, GMLJP2, MSIG in that order, then a world file
/// next to `path` when there is no geotransform. `rpc` is set when the GeoTIFF box used carries
/// RPC coefficients.
tiff::Georeferencing ReadJp2Georeferencing(const Jp2File& file, std::string_view path, bool* rpc = nullptr);

/// The default-domain metadata GDAL reports (the GDAL metadata box's, then the resolution box's
/// TIFFTAG_XRESOLUTION / YRESOLUTION / RESOLUTIONUNIT), in GDAL's order.
std::vector<std::pair<std::string, std::string>> Jp2Metadata(const Jp2File& file);

}  // namespace ddbt::cog::jp2
