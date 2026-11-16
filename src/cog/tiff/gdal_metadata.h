// SPDX-License-Identifier: MPL-2.0
//
// The metadata GDAL's GTiff driver (GDAL 3.13) reports for the main image of a TIFF, string for
// string: the domains of the dataset and of each band, with what GDAL derives from the TIFF tags
// (the default domain's TIFFTAG_* items and AREA_OR_POINT, IMAGE_STRUCTURE, xml:XMP,
// COLOR_PROFILE, RPC) and the items and band roles of the GDAL_METADATA tag.
//
// Not read: the EXIF and GPS directories (GDAL writes the EXIF domain of a raster into
// GDAL_METADATA, so a TIFF GDAL wrote has none), files next to the TIFF (.aux.xml, .IMD, .RPB,
// .hdr), the derived and sub datasets (they name files), the reversibility GDAL reads from the
// first WebP or JPEG-XL block, the JPEG quality of YCbCr tables libjpeg 9e made. libddb's TIFFs
// (its COGs and the GeoTIFFs its raster helper writes) are DEFLATE, LZW, ZSTD, JPEG or not
// compressed.
#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "cog/tiff/color_interp.h"

namespace ddbt::cog::tiff {

/// Whether `a` and `b` are the same text but for the case of ASCII letters (GDAL's EQUAL).
bool EqualNoCase(std::string_view a, std::string_view b);

class TiffReader;
struct GeoKeyDirectory;

/// Metadata domains as GDALMultiDomainMetadata keeps them: the domains in the order they were
/// made, their items "NAME=VALUE" (the one document of an xml: or json: domain), an item set
/// again replacing the one of the same name (names compared without case) where it is.
class MetadataDomains {
public:
    struct Domain {
        std::string name;
        std::vector<std::string> items;
        bool sorted = false;  // set whole (SetItems): GDAL keeps its name=value items sorted by name
    };

    /// GDALMultiDomainMetadata::SetMetadataItem.
    void SetItem(std::string_view domain, std::string_view name, std::string_view value);
    /// GDALMultiDomainMetadata::SetMetadata: the items of `domain` replaced.
    void SetItems(std::string_view domain, std::vector<std::string> items);
    /// The value of item `name` of `domain` (CSLFetchNameValue).
    std::optional<std::string> Item(std::string_view domain, std::string_view name) const;
    /// The items of `domain`, or nullptr when it has none (GDAL's GetMetadata gives null).
    const std::vector<std::string>* Items(std::string_view domain) const;
    /// Every domain made, in the order made (a domain may have no item).
    const std::vector<Domain>& All() const noexcept { return domains_; }

private:
    Domain& Of(std::string_view domain);
    std::vector<Domain> domains_;
};

/// What a band of GDAL_METADATA says through its roles (GTiffRasterBand): a scale or an offset
/// gives both (GetScale and GetOffset succeed, 1 and 0 when not given).
struct GdalBandRoles {
    bool hasOffsetScale = false;
    double scale = 1.0;
    double offset = 0.0;
    std::string unitType;
    std::string description;
    std::optional<ColorInterp> colorInterp;  // a role "colorinterp" (kUndefined for an unknown name)
};

/// An item of GDAL_METADATA as GDAL reads it: CPLGetXMLValue of an <Item> whose content is one
/// text token, decoded twice (by the parser, then by CPLUnescapeString, as GDAL does for
/// backward compatibility). `sample` is -1 for the dataset.
struct GdalMetadataItem {
    std::string name;
    int sample = -1;
    std::string role;
    std::string domain;
    std::string value;
};

/// The items of a GDAL_METADATA document GDAL reads (the <Item> children of <GDALMetadata> with a
/// name and a value); none when the document is not well-formed (CPLParseXMLString fails).
std::vector<GdalMetadataItem> ParseGdalMetadata(std::string_view xml);

/// What GDAL gives a band from GDAL_METADATA: its roles and its domains.
struct GdalBandMetadata {
    GdalBandRoles roles;
    MetadataDomains domains;
};

/// What GDAL takes for bands 1..bandCount from a GDAL_METADATA document.
std::vector<GdalBandMetadata> GdalMetadataBands(std::string_view xml, int bandCount);

/// What the GTiff driver needs to know of the dataset besides its tags.
struct GdalDatasetShape {
    int bandCount = 0;  // GDAL's bands (4 when read through libtiff's RGBA interface)
    /// The IFDs of GDAL's overviews of the main image, largest first (GTiffDataset::m_apoOverviewDS).
    std::vector<uint32_t> overviewIfds;
};

/// The metadata of the dataset and of its bands.
struct GdalMetadata {
    MetadataDomains dataset;
    std::vector<GdalBandMetadata> bands;  // bands 1..bandCount (their domains with NBITS)

    /// GDALGetMetadataDomainList of the dataset: its domains with items, in GDAL's order.
    std::vector<std::string> DomainList() const;
};

/// The metadata GDAL reports for the main image of `reader` once every domain was asked for
/// (GetMetadataDomainList, then GetMetadata of each domain), `keys` its GeoKeys.
GdalMetadata ReadGdalMetadata(const TiffReader& reader, const GdalDatasetShape& shape, const GeoKeyDirectory& keys);

/// GDAL's names of the TIFF compressions (GTIFFGetCompressionMethodName), nullptr for a code GDAL
/// does not name.
const char* GdalCompressionName(uint16_t compression) noexcept;

/// The quantization segments libjpeg writes into the JPEG tables of a TIFF at `quality` (1 to 100) as
/// libtiff asks for them (without forcing baseline: 16-bit tables below a quality of about 25), the
/// luminance table, and the chrominance table for YCbCr; each segment from its length bytes.
std::vector<uint8_t> StandardJpegQuantTables(int quality, bool ycbcr);

/// The quality GDAL guesses for JPEG tables (GTiffDataset::GuessJPEGQuality: the quantization
/// tables libjpeg makes at a quality of 1 to 100, as libtiff writes them, a chrominance table for
/// YCbCr); nullopt when no quality gives them.
std::optional<int> GuessJpegQuality(std::span<const uint8_t> jpegTables, bool ycbcr);

}  // namespace ddbt::cog::tiff
