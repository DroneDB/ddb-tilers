// SPDX-License-Identifier: MPL-2.0
//
// TIFF / BigTIFF / GeoTIFF / GDAL tag numbers and field types.
#pragma once

#include <cstddef>
#include <cstdint>

namespace ddbt::cog::tiff {

namespace tag {
inline constexpr uint16_t kNewSubfileType = 254;
inline constexpr uint16_t kSubfileType = 255;
inline constexpr uint16_t kImageWidth = 256;
inline constexpr uint16_t kImageLength = 257;
inline constexpr uint16_t kBitsPerSample = 258;
inline constexpr uint16_t kCompression = 259;
inline constexpr uint16_t kPhotometric = 262;
inline constexpr uint16_t kFillOrder = 266;
inline constexpr uint16_t kImageDescription = 270;
inline constexpr uint16_t kStripOffsets = 273;
inline constexpr uint16_t kOrientation = 274;
inline constexpr uint16_t kSamplesPerPixel = 277;
inline constexpr uint16_t kRowsPerStrip = 278;
inline constexpr uint16_t kStripByteCounts = 279;
inline constexpr uint16_t kPlanarConfig = 284;
inline constexpr uint16_t kSoftware = 305;
inline constexpr uint16_t kDateTime = 306;
inline constexpr uint16_t kPredictor = 317;
inline constexpr uint16_t kColorMap = 320;
inline constexpr uint16_t kTileWidth = 322;
inline constexpr uint16_t kTileLength = 323;
inline constexpr uint16_t kTileOffsets = 324;
inline constexpr uint16_t kTileByteCounts = 325;
inline constexpr uint16_t kSubIfds = 330;
inline constexpr uint16_t kExtraSamples = 338;
inline constexpr uint16_t kSampleFormat = 339;
inline constexpr uint16_t kSMinSampleValue = 340;
inline constexpr uint16_t kSMaxSampleValue = 341;
inline constexpr uint16_t kJpegTables = 347;
inline constexpr uint16_t kYCbCrCoefficients = 529;
inline constexpr uint16_t kYCbCrSubsampling = 530;
inline constexpr uint16_t kYCbCrPositioning = 531;
inline constexpr uint16_t kReferenceBlackWhite = 532;
inline constexpr uint16_t kModelPixelScale = 33550;
inline constexpr uint16_t kModelTiepoint = 33922;
inline constexpr uint16_t kModelTransformation = 34264;
inline constexpr uint16_t kGeoKeyDirectory = 34735;
inline constexpr uint16_t kGeoDoubleParams = 34736;
inline constexpr uint16_t kGeoAsciiParams = 34737;
inline constexpr uint16_t kGdalMetadata = 42112;
inline constexpr uint16_t kGdalNodata = 42113;
inline constexpr uint16_t kLercParameters = 50674;
}  // namespace tag

/// TIFF field types (BigTIFF adds 16..18).
enum class TagType : uint16_t {
    kByte = 1,
    kAscii = 2,
    kShort = 3,
    kLong = 4,
    kRational = 5,
    kSByte = 6,
    kUndefined = 7,
    kSShort = 8,
    kSLong = 9,
    kSRational = 10,
    kFloat = 11,
    kDouble = 12,
    kIfd = 13,
    kLong8 = 16,
    kSLong8 = 17,
    kIfd8 = 18,
};

/// Size in bytes of one value of a field type; 0 for unknown types (whose entries are kept but
/// never decoded).
constexpr size_t TagTypeSize(uint16_t type) noexcept {
    switch (type) {
        case 1: case 2: case 6: case 7: return 1;
        case 3: case 8: return 2;
        case 4: case 9: case 11: case 13: return 4;
        case 5: case 10: case 12: case 16: case 17: case 18: return 8;
        default: return 0;
    }
}

/// Photometric interpretation values.
namespace photometric {
inline constexpr uint16_t kMinIsWhite = 0;
inline constexpr uint16_t kMinIsBlack = 1;
inline constexpr uint16_t kRgb = 2;
inline constexpr uint16_t kPalette = 3;
inline constexpr uint16_t kMask = 4;
inline constexpr uint16_t kSeparated = 5;
inline constexpr uint16_t kYCbCr = 6;
inline constexpr uint16_t kCieLab = 8;
inline constexpr uint16_t kLogL = 32844;
inline constexpr uint16_t kLogLuv = 32845;
}  // namespace photometric

/// ExtraSamples values.
namespace extrasample {
inline constexpr uint16_t kUnspecified = 0;
inline constexpr uint16_t kAssociatedAlpha = 1;
inline constexpr uint16_t kUnassociatedAlpha = 2;
}  // namespace extrasample

}  // namespace ddbt::cog::tiff
