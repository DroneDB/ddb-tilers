// SPDX-License-Identifier: MPL-2.0
#include "copc/las/las_crs.h"

#include "cog/tiff/geotiff.h"
#include "core/util/checked_math.h"

namespace ddbt::copc::las {
namespace {

// The GeoKeyDirectory of the LAS GeoTIFF VLRs, resolved the way cog/tiff's ReadGeoKeys does
// for a TIFF (libgeotiff rules: '|'-terminated ASCII values, out-of-range values dropped).
Expected<cog::tiff::GeoKeyDirectory> GeoKeysOf(const GeoKeyVlrs& g) {
    constexpr uint16_t kDirectory = 34735, kDoubles = 34736, kAscii = 34737;
    cog::tiff::GeoKeyDirectory dir;
    const std::vector<uint16_t>& s = g.directory;
    if (s.size() < 4) return Corrupt("GeoKeyDirectory VLR: shorter than its header");
    dir.version = s[0];
    dir.revision = s[1];
    dir.minorRevision = s[2];
    const size_t n = s[3];
    if (n > (s.size() - 4) / 4) return Corrupt("GeoKeyDirectory VLR: more keys than entries");
    for (size_t k = 0; k < n; ++k) {
        const uint16_t* e = s.data() + 4 + k * 4;
        cog::tiff::GeoKey key;
        key.id = e[0];
        key.location = e[1];
        const uint64_t count = e[2];
        const uint64_t off = e[3];
        if (key.location == 0) {
            key.shorts.push_back(e[3]);
        } else if (key.location == kDirectory) {
            if (!RangeInside(off, count, s.size())) continue;
            key.shorts.assign(s.begin() + static_cast<ptrdiff_t>(off), s.begin() + static_cast<ptrdiff_t>(off + count));
        } else if (key.location == kDoubles) {
            if (!RangeInside(off, count, g.doubles.size())) continue;
            key.doubles.assign(g.doubles.begin() + static_cast<ptrdiff_t>(off),
                               g.doubles.begin() + static_cast<ptrdiff_t>(off + count));
        } else if (key.location == kAscii) {
            if (!RangeInside(off, count, g.ascii.size())) continue;
            std::string v = g.ascii.substr(static_cast<size_t>(off), static_cast<size_t>(count));
            if (!v.empty() && (v.back() == '|' || v.back() == '\0')) v.pop_back();
            const size_t nul = v.find('\0');
            if (nul != std::string::npos) v.resize(nul);
            key.ascii = std::move(v);
        } else {
            continue;
        }
        dir.keys.emplace(key.id, std::move(key));
    }
    return dir;
}

}  // namespace

Expected<CrsDefinition> FindCrsDefinition(const LasMetadata& meta) {
    CrsDefinition d;
    if (meta.wkt2 && !meta.wkt2->empty()) return CrsDefinition{CrsSource::kWkt2, *meta.wkt2};
    if (meta.projJson && !meta.projJson->empty()) return CrsDefinition{CrsSource::kProjJson, *meta.projJson};
    const bool useWkt = (meta.header.globalEncoding & kGlobalEncodingWkt) != 0 || meta.header.pointFormat >= 6;
    const bool haveWkt = meta.wkt && !meta.wkt->empty();
    const bool haveKeys = meta.geoKeys && !meta.geoKeys->directory.empty();
    auto fromKeys = [&]() -> Expected<CrsDefinition> {
        DDBT_ASSIGN_OR_RETURN(cog::tiff::GeoKeyDirectory dir, GeoKeysOf(*meta.geoKeys));
        DDBT_ASSIGN_OR_RETURN(std::string def, cog::tiff::GeoKeysToCrsDefinition(dir));
        if (def.empty()) return CrsDefinition{};
        return CrsDefinition{CrsSource::kGeoKeys, std::move(def)};
    };
    if (useWkt) {
        if (haveWkt) return CrsDefinition{CrsSource::kWkt, *meta.wkt};
        if (haveKeys) return fromKeys();
    } else {
        if (haveKeys) return fromKeys();
        if (haveWkt) return CrsDefinition{CrsSource::kWkt, *meta.wkt};
    }
    return d;
}

Expected<crs::Crs> ResolveCrs(const LasMetadata& meta) {
    DDBT_ASSIGN_OR_RETURN(CrsDefinition d, FindCrsDefinition(meta));
    try {
        switch (d.source) {
            case CrsSource::kNone:
                return crs::Crs();
            case CrsSource::kWkt:
            case CrsSource::kWkt2:
                return crs::Crs::FromWkt(d.text);
            case CrsSource::kProjJson:
                return crs::Crs::FromProjJson(d.text);
            case CrsSource::kGeoKeys:
                // As cog/tiff: "EPSG:..." codes through the database, anything else is WKT.
                if (d.text.rfind("EPSG:", 0) == 0) return crs::Crs::FromUserInput(d.text);
                return crs::Crs::FromWkt(d.text);
        }
    } catch (const std::exception& e) {
        return Corrupt(std::string("LAS CRS: ") + e.what());
    }
    return crs::Crs();
}

}  // namespace ddbt::copc::las
