// SPDX-License-Identifier: MPL-2.0
//
// CogSource: one GeoTIFF / COG as the tiler reads it, immutable and shared by every request for
// the same version of a file (FileIdentity: size, mtime, ctime, inode and device). CogSource::Get()
// looks it up in a sharded process-wide registry (a stat() per call; a changed or replaced file gets
// a new source, the old one lives on while requests hold it; spellings of one path share a source). It holds the TiffReader, the georeferencing, the pyramid levels (main image + internal
// overviews, with their masks), and what GDAL's GTiff driver derives per band: colour
// interpretation (photometric, ExtraSamples, GDAL_METADATA COLORINTERP), nodata (GDAL_NODATA),
// scale / offset / unit / description (GDAL_METADATA), and the mask band of band 1
// (GTiffRasterBand::GetMaskBand order: internal mask, nodata, alpha of 2 / 4 band rasters, all
// valid). For a source that is not in EPSG:3857 it also holds the warp model (warp.h).
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "cog/tiff/geotiff.h"
#include "cog/tiff/tiff_reader.h"

namespace ddbt::cog::tiler {

class WarpModel;
struct SensorDetection;

/// One version of a file: what the registry and the statistics cache compare. POSIX: st_size,
/// st_mtim, st_ctim, st_ino, st_dev; Windows: the size, last-write and creation times, file index
/// and volume serial number (GetFileInformationByHandle). A file replaced by a rename gets a new
/// inode, one rewritten in place a new ctime (POSIX), even with its size and mtime restored.
struct FileIdentity {
    uint64_t size = 0;
    int64_t mtimeNs = 0;
    int64_t ctimeNs = 0;
    uint64_t inode = 0;
    uint64_t device = 0;
    bool operator==(const FileIdentity&) const = default;
};

/// GDAL's GDALColorInterp values the tiler needs.
enum class ColorInterp : uint8_t {
    kUndefined,
    kGray,
    kPalette,
    kRed,
    kGreen,
    kBlue,
    kAlpha,
    kYCbCrY,
    kYCbCrCb,
    kYCbCrCr,
    kCyan,
    kMagenta,
    kYellow,
    kBlack,
    kOther,
};

/// GDALGetColorInterpretationName.
const char* ColorInterpName(ColorInterp ci) noexcept;

/// One band, as GDAL reports it.
struct BandMeta {
    ColorInterp interp = ColorInterp::kUndefined;
    std::optional<double> nodata;  // GDAL_NODATA (the same for every band of a GeoTIFF)
    double scale = 1.0, offset = 0.0;
    bool hasScale = false, hasOffset = false;
    std::string unitType;      // GDAL_METADATA role="unittype"
    std::string description;   // GDAL_METADATA role="description"
    std::vector<std::string> metadata;  // "KEY=VALUE" items of the band's default domain
};

/// What GDALGetMaskBand(band 1) returns for the file (GTiff + GDALRasterBand::GetMaskBand).
enum class MaskKind : uint8_t {
    kAllValid,    // GMF_ALL_VALID
    kPerDataset,  // internal TIFF mask (GMF_PER_DATASET)
    kNodata,      // GDALNoDataMaskBand of band 1 (GMF_NODATA)
    kAlpha,       // band 2 of 2 / band 4 of 4 is alpha (GMF_ALPHA | GMF_PER_DATASET)
};

/// One level of the pyramid: the main image or an internal overview.
struct SourceLevel {
    uint32_t ifd = 0;
    std::optional<uint32_t> maskIfd;
    uint32_t width = 0, height = 0;
};

/// Supported sample types (GDAL data types).
enum class PixelType : uint8_t { kU8, kI8, kU16, kI16, kU32, kI32, kF32, kF64 };
size_t PixelTypeSize(PixelType t) noexcept;
bool PixelTypeIsFloat(PixelType t) noexcept;
/// GDALGetDataTypeName: "Byte", "Int8", "UInt16", ...
const char* PixelTypeName(PixelType t) noexcept;

class CogSource {
public:
    /// Bands a source may have (DDBT_TILER_MAX_BANDS overrides it): a file with more is refused
    /// at open (each band costs statistics, metadata and, for formulas over NoData, a read).
    static constexpr int kDefaultMaxBands = 1024;

    /// The shared source of `path` for its current FileIdentity: opened and parsed on first
    /// use (concurrent first users wait for one open). Throws TileError: kNotFound when the
    /// file does not exist, kNotImplemented for a file that is not a TIFF, kException for a
    /// TIFF the tiler cannot read (or with more than the maximum band count, or wider or taller
    /// than INT_MAX pixels).
    static std::shared_ptr<const CogSource> Get(const std::string& path);
    /// Opens without the registry (tests).
    static std::shared_ptr<const CogSource> Open(const std::string& path);
    /// Forgets every registered source (tests).
    static void ClearRegistry();

    ~CogSource();
    CogSource(const CogSource&) = delete;
    CogSource& operator=(const CogSource&) = delete;

    const std::string& Path() const noexcept { return path_; }
    /// The absolute path with "." / ".." and symbolic links resolved (weakly_canonical).
    const std::string& CanonicalPath() const noexcept { return canonicalPath_; }
    uint64_t FileSize() const noexcept { return identity_.size; }
    /// Modification time, nanoseconds since the epoch.
    int64_t MTimeNs() const noexcept { return identity_.mtimeNs; }
    const FileIdentity& Identity() const noexcept { return identity_; }
    /// The canonical path and the FileIdentity as one string: the same for every source of the
    /// same file version (the statistics cache keys on it).
    const std::string& IdentityKey() const noexcept { return identityKey_; }
    /// Process-unique id of this source object: per-source cache keys use it.
    uint64_t Id() const noexcept { return id_; }

    const tiff::TiffReader& Reader() const noexcept { return *reader_; }
    const std::shared_ptr<const tiff::TiffReader>& SharedReader() const noexcept { return reader_; }
    const tiff::Georeferencing& Georef() const noexcept { return georef_; }
    /// The geotransform (DroneDB requires one: a raster without one is "not georeferenced").
    const std::array<double, 6>& GeoTransform() const noexcept { return gt_; }

    int Width() const noexcept { return static_cast<int>(levels_[0].width); }
    int Height() const noexcept { return static_cast<int>(levels_[0].height); }
    /// Main image first, then the overviews GDAL exposes, largest first.
    const std::vector<SourceLevel>& Levels() const noexcept { return levels_; }
    int BandCount() const noexcept { return static_cast<int>(bands_.size()); }
    const BandMeta& Band(int band1) const { return bands_.at(static_cast<size_t>(band1 - 1)); }
    PixelType Type() const noexcept { return type_; }
    bool PixelInterleaved() const noexcept { return pixelInterleaved_; }
    /// JPEG compression (JPEG COGs: the reader decodes YCbCr to RGB).
    bool IsJpeg() const noexcept;

    /// 1-based band whose colour interpretation is alpha (GDALTiler::FindAlphaBand), 0 if none.
    int AlphaBand() const noexcept { return alphaBand_; }
    MaskKind Mask() const noexcept { return mask_; }
    /// DroneDB's dataBandCount(ds) on the file: all but a last band that is alpha.
    int DataBandCount() const noexcept;

    /// The CRS is EPSG:3857 (GDALTiler::sameProjection): tiles read the file directly.
    bool IsWebMercator() const noexcept { return webMercator_; }
    /// The warp model of a source in another CRS (null for EPSG:3857).
    const WarpModel* Warp() const noexcept { return warp_.get(); }

    /// Every metadata string the sensor profiles are matched against (sensorprofile.cpp
    /// collectMatchingMetadata): TIFF description and software, dataset items, band items.
    const std::vector<std::string>& DatasetMetadata() const noexcept { return datasetMetadata_; }

    /// The memo of DetectSensor (vis.h): `detect` runs once per source, on first use; every
    /// caller gets the same detection, alive while it holds it.
    template <typename Detect>
    std::shared_ptr<const SensorDetection> SensorMemo(Detect&& detect) const {
        std::call_once(sensorOnce_, [&] { sensor_ = detect(); });
        return sensor_;
    }

private:
    CogSource() = default;
    void Init(const std::string& path, const std::string& canonicalPath, const FileIdentity& identity);

    std::string path_;
    std::string canonicalPath_;
    FileIdentity identity_;
    std::string identityKey_;
    uint64_t id_ = 0;
    std::shared_ptr<const tiff::TiffReader> reader_;
    tiff::Georeferencing georef_;
    std::array<double, 6> gt_{};
    std::vector<SourceLevel> levels_;
    std::vector<BandMeta> bands_;
    PixelType type_ = PixelType::kU8;
    bool pixelInterleaved_ = true;
    int alphaBand_ = 0;
    MaskKind mask_ = MaskKind::kAllValid;
    bool webMercator_ = false;
    std::unique_ptr<WarpModel> warp_;
    std::vector<std::string> datasetMetadata_;
    mutable std::once_flag sensorOnce_;
    mutable std::shared_ptr<const SensorDetection> sensor_;
};

/// GDAL_METADATA XML items (name, sample (-1 dataset), role, domain, value).
struct GdalMetadataItem {
    std::string name;
    int sample = -1;
    std::string role;
    std::string domain;
    std::string value;
};
std::vector<GdalMetadataItem> ParseGdalMetadata(std::string_view xml);

}  // namespace ddbt::cog::tiler
