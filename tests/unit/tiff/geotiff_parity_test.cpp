// SPDX-License-Identifier: MPL-2.0
//
// Geotransform and CRS against GDAL (georef.tsv written by tests/unit/tiff/data/gdal_reference.py
// with GDAL_PAM_ENABLED=NO): for every raster of the fixtures index (corpus, test_data GeoTIFFs,
// synthetic files) the geotransform must equal GDAL's and the CRS must be equivalent to GDAL's:
// PROJ's EQUIVALENT criterion (axis order of geographic CRSs ignored), or else a 7 x 7 grid of
// points over the raster mapped from our CRS to GDAL's must not move (1e-7 m / 1e-12 deg),
// i.e. PROJ finds a null operation between them.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "cog/tiff/geotiff.h"
#include "core/crs/transform.h"

namespace ddbt::cog::tiff {
namespace {

std::string FixturesDir() {
    const char* e = std::getenv("DDBT_TIFF_FIXTURES");
    return e != nullptr ? e : "/mnt/data/ddb-tilers-work/fixtures/tiff";
}

std::vector<std::string> Split(const std::string& line) {
    std::vector<std::string> f;
    std::string cur;
    for (char c : line) {
        if (c == '\t') {
            f.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    f.push_back(cur);
    return f;
}

std::string Unescape(const std::string& s) {
    std::string o;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '\\' && i + 1 < s.size() && s[i + 1] == 'n') {
            o.push_back('\n');
            ++i;
        } else {
            o.push_back(s[i]);
        }
    }
    return o;
}

// Largest displacement of grid points over the raster when mapped from `a` to `b`.
double MaxShift(const crs::Crs& a, const crs::Crs& b, const std::array<double, 6>& gt, uint32_t w, uint32_t h) {
    auto t = crs::Transform::Create(a, b);
    double worst = 0;
    for (int j = 0; j <= 6; ++j) {
        for (int i = 0; i <= 6; ++i) {
            const double px = w * (i / 6.0), py = h * (j / 6.0);
            double x = gt[0] + px * gt[1] + py * gt[2];
            double y = gt[3] + px * gt[4] + py * gt[5];
            const double x0 = x, y0 = y;
            if (!t->TransformPoint(x, y)) continue;
            worst = std::max(worst, std::max(std::fabs(x - x0), std::fabs(y - y0)));
        }
    }
    return worst;
}

TEST(GeoTiffGdalParity, GeoTransformAndCrsMatchGdal) {
    const std::string dir = FixturesDir();
    std::ifstream index(dir + "/index.tsv");
    std::ifstream georef(dir + "/georef.tsv");
    if (!index || !georef) GTEST_SKIP() << "no fixtures in " << dir << " (run tests/unit/tiff/data/gdal_reference.py)";
    std::map<std::string, std::string> paths;
    std::string line;
    while (std::getline(index, line)) {
        const auto f = Split(line);
        if (f.size() >= 2) paths[f[0]] = f[1];
    }
    size_t n = 0, gtBad = 0, crsBad = 0, viaShift = 0;
    std::ostringstream report;
    while (std::getline(georef, line)) {
        const auto f = Split(line);
        if (f.size() < 9 || !paths.count(f[0])) continue;
        const std::string& id = f[0];
        const std::string& path = paths[id];
        auto reader = TiffReader::Open(path);
        if (!reader.ok()) {
            ADD_FAILURE() << id << ": " << reader.status().ToString();
            continue;
        }
        auto g = ReadGeoreferencing(**reader, path);
        ASSERT_TRUE(g.ok()) << id << ": " << g.status().ToString();
        ++n;
        // Geotransform.
        const bool gdalHasGt = f[1] == "1";
        std::array<double, 6> ref{};
        for (int i = 0; i < 6; ++i) ref[size_t(i)] = gdalHasGt ? std::strtod(f[size_t(2 + i)].c_str(), nullptr) : 0.0;
        bool gtOk = gdalHasGt == g->geoTransform.has_value();
        if (gtOk && gdalHasGt) {
            for (int i = 0; i < 6; ++i) gtOk &= (*g->geoTransform)[size_t(i)] == ref[size_t(i)];
        }
        if (!gtOk) {
            ++gtBad;
            ADD_FAILURE() << id << ": geotransform differs from GDAL's";
        }
        // CRS.
        const std::string gdalWkt = Unescape(f[8]);
        std::string verdict;
        if (gdalWkt == "-") {
            if (g->crs) {
                ++crsBad;
                ADD_FAILURE() << id << ": GDAL has no CRS, we built " << g->crsDefinition;
            }
            verdict = "no CRS (both)";
        } else if (!g->crs) {
            ++crsBad;
            ADD_FAILURE() << id << ": no CRS (" << g->crsError << "), GDAL has one";
            verdict = "missing";
        } else {
            const crs::Crs gdalCrs = crs::Crs::FromWkt(gdalWkt);
            if (g->crs->Equivalent(gdalCrs)) {
                verdict = "equivalent";
            } else {
                const IfdInfo& img = (*reader)->Ifd((*reader)->MainImage());
                const std::array<double, 6> gt = g->geoTransform.value_or(std::array<double, 6>{0, 1, 0, 0, 0, -1});
                double shift = -1;
                try {
                    shift = MaxShift(*g->crs, gdalCrs, gt, img.width, img.height);
                } catch (const std::exception& e) {
                    verdict = std::string("transform failed: ") + e.what();
                }
                const double tol = g->crs->IsGeographic() ? 1e-12 : 1e-7;
                if (shift >= 0 && shift <= tol) {
                    ++viaShift;
                    verdict = "same coordinates (max shift " + std::to_string(shift) + ")";
                } else {
                    ++crsBad;
                    ADD_FAILURE() << id << ": CRS not equivalent to GDAL's (max shift " << shift << ")\n  ours: "
                                  << g->crsDefinition << "\n  GDAL: " << gdalWkt;
                    verdict = "DIFFERENT";
                }
            }
        }
        report << "  " << id << ": gt " << (gtOk ? "equal" : "DIFFERENT") << ", CRS " << verdict << " ["
               << (g->crsDefinition.size() > 60 ? g->crsDefinition.substr(0, 60) + "..." : g->crsDefinition) << "]\n";
    }
    std::cout << "GeoTIFF parity: " << n << " rasters, " << gtBad << " geotransforms and " << crsBad
              << " CRSs differ; " << viaShift << " CRSs matched by coordinates only\n"
              << report.str();
}

}  // namespace
}  // namespace ddbt::cog::tiff
