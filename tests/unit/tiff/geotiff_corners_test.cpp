// SPDX-License-Identifier: MPL-2.0
//
// Where the CRS read from GeoKeys puts a raster, against GDAL (corners.tsv written by
// tests/unit/tiff/data/gdal_corners_reference.py over a GeoTIFF corpus): the four corners of every
// raster, mapped to EPSG:4326 from the CRS we read and from GDAL's CRS (its WKT2), both with our
// PROJ, must be within 1 cm of each other. The corners GDAL mapped itself are compared as well,
// and only reported: a GDAL built with another PROJ can map the same CRS elsewhere (another
// operation from its proj.db, or another formula: PROJ 9.8 made EPSG:4087 ellipsoidal).

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "cog/tiff/geotiff.h"
#include "core/crs/transform.h"
#include "reference_tsv.h"

namespace ddbt::cog::tiff {
namespace {

using testing::SplitTsv;
using testing::UnescapeNewlines;

constexpr double kToleranceM = 0.01;

std::string CornersFile() {
    const char* e = std::getenv("DDBT_GEOTIFF_CORNERS");
    return e != nullptr ? e : "/mnt/data/ddb-tilers-work/fixtures/geotiff-corners/corners.tsv";
}

// Great-circle distance in metres (mean Earth radius): enough to tell 1 cm from more.
double DistanceM(double lon0, double lat0, double lon1, double lat1) {
    constexpr double kRad = 3.14159265358979323846 / 180.0;
    const double dLat = (lat1 - lat0) * kRad, dLon = (lon1 - lon0) * kRad;
    const double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
                     std::cos(lat0 * kRad) * std::cos(lat1 * kRad) * std::sin(dLon / 2) * std::sin(dLon / 2);
    return 2 * 6371008.8 * std::asin(std::min(1.0, std::sqrt(a)));
}

using Corners = std::array<double, 8>;

// The corners (lon, lat) of a w x h raster mapped from `c` to EPSG:4326; NaN where PROJ fails.
Corners CornersOf(const crs::Crs& c, const std::array<double, 6>& gt, double w, double h) {
    Corners out{};
    const auto t = crs::Transform::Create(c, crs::Crs::FromEpsg(4326));
    const double px[4] = {0, w, w, 0}, py[4] = {0, 0, h, h};
    for (int i = 0; i < 4; ++i) {
        double x = gt[0] + px[i] * gt[1] + py[i] * gt[2];
        double y = gt[3] + px[i] * gt[4] + py[i] * gt[5];
        if (!t->TransformPoint(x, y) || !std::isfinite(x) || !std::isfinite(y)) x = y = std::nan("");
        out[size_t(2 * i)] = x;
        out[size_t(2 * i + 1)] = y;
    }
    return out;
}

// Largest corner distance; +inf when a corner maps on one side only.
double WorstDistance(const Corners& a, const Corners& b) {
    double worst = 0;
    for (size_t i = 0; i < 4; ++i) {
        const bool na = std::isnan(a[2 * i]), nb = std::isnan(b[2 * i]);
        if (na && nb) continue;
        if (na != nb) return INFINITY;
        worst = std::max(worst, DistanceM(a[2 * i], a[2 * i + 1], b[2 * i], b[2 * i + 1]));
    }
    return worst;
}

TEST(GeoTiffCorners, KeysPlaceTheRasterWhereGdalDoes) {
    std::ifstream in(CornersFile());
    if (!in) GTEST_SKIP() << "no " << CornersFile() << " (run tests/unit/tiff/data/gdal_corners_reference.py)";
    size_t n = 0, bad = 0, otherProj = 0;
    std::ostringstream report;
    std::string line;
    while (std::getline(in, line)) {
        const auto f = SplitTsv(line);
        if (f.size() != 12) continue;
        const std::string& path = f[0];
        const double w = std::strtod(f[1].c_str(), nullptr), h = std::strtod(f[2].c_str(), nullptr);
        Corners gdal{};
        for (size_t i = 0; i < 8; ++i) gdal[i] = std::strtod(f[3 + i].c_str(), nullptr);
        ++n;
        auto reader = TiffReader::Open(path);
        if (!reader.ok()) {
            ++bad;
            ADD_FAILURE() << path << ": " << reader.status().ToString();
            continue;
        }
        auto g = ReadGeoreferencing(**reader, path);
        if (!g.ok() || !g->crs || !g->geoTransform) {
            ++bad;
            ADD_FAILURE() << path << ": no CRS or geotransform (" << (g.ok() ? g->crsError : g.status().ToString())
                          << "); GDAL has both";
            continue;
        }
        double crsShift = INFINITY, gdalShift = INFINITY;
        try {
            const Corners ours = CornersOf(*g->crs, *g->geoTransform, w, h);
            crsShift = WorstDistance(ours, CornersOf(crs::Crs::FromWkt(UnescapeNewlines(f[11])), *g->geoTransform, w, h));
            gdalShift = WorstDistance(ours, gdal);
        } catch (const std::exception& e) {
            ADD_FAILURE() << path << ": " << e.what();
        }
        if (crsShift > kToleranceM) {
            ++bad;
            ADD_FAILURE() << path << ": corners up to " << crsShift << " m from those of GDAL's CRS\n  ours: "
                          << g->crsDefinition;
            report << "  " << path << ": " << crsShift << " m from GDAL's CRS\n";
        } else if (gdalShift > kToleranceM) {
            ++otherProj;
            report << "  " << path << ": " << gdalShift << " m from GDAL's corners, same CRS (another PROJ)\n";
        }
    }
    std::cout << "GeoTIFF corners: " << n << " rasters, " << bad << " CRSs more than " << kToleranceM
              << " m from GDAL's; " << otherProj << " CRSs that GDAL's PROJ maps elsewhere\n"
              << report.str();
}

}  // namespace
}  // namespace ddbt::cog::tiff
