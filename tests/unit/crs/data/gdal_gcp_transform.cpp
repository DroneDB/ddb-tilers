// SPDX-License-Identifier: MPL-2.0
//
// Reference tool for gen_gcp_reference.py (not built by the project): runs GDAL's general image
// transformer, as gdalwarp creates it for a raster with GCPs and no geotransform, on the cases
// the script writes to stdin, and prints the results. Built against a GDAL without Armadillo
// (such as vcpkg's), it gives the homography of GDAL's own linear solver, the one GcpTransform
// follows. Build, e.g. with the static GDAL of a vcpkg tree:
//
//   cmake_minimum_required(VERSION 3.25)
//   project(gdal_gcp_transform CXX)
//   find_package(GDAL CONFIG REQUIRED)
//   add_executable(gdal_gcp_transform gdal_gcp_transform.cpp)
//   target_link_libraries(gdal_gcp_transform PRIVATE GDAL::GDAL)
//
// Input, whitespace separated (numbers as C hexadecimal floats):
//   case <name> <crs or -> <method 0/1/2> <max order> <GCP count> <point count>
//   <GCP count> lines: pixel line x y
//   <point count> lines: pixel line (forward)
//   <point count> lines: x y (inverse)
// Output: "gdal <version>", then per case: "case <name> ok|fail <message>", then "f x y ok" and "i x y ok" per point.

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "cpl_conv.h"
#include "cpl_error.h"
#include "cpl_string.h"
#include "gdal.h"
#include "gdal_alg.h"
#include "ogr_srs_api.h"

namespace {

double Read(std::istream& in) {
    std::string s;
    in >> s;
    return std::strtod(s.c_str(), nullptr);
}

void Run(void* t, int dstToSrc, std::vector<double>& x, std::vector<double>& y, std::vector<int>& ok) {
    std::vector<double> z(x.size(), 0.0);
    ok.assign(x.size(), 0);
    GDALGenImgProjTransform(t, dstToSrc, static_cast<int>(x.size()), x.data(), y.data(), z.data(), ok.data());
}

}  // namespace

int main() {
    GDALAllRegister();
    CPLSetErrorHandler(CPLQuietErrorHandler);
    std::printf("gdal %s\n", GDALVersionInfo("RELEASE_NAME"));
    std::string tag;
    while (std::cin >> tag && tag == "case") {
        std::string name, crs;
        int method = 0, maxOrder = 0;
        size_t nGcps = 0, nPoints = 0;
        std::cin >> name >> crs >> method >> maxOrder >> nGcps >> nPoints;
        std::vector<GDAL_GCP> gcps(nGcps);
        GDALInitGCPs(static_cast<int>(nGcps), gcps.data());
        for (GDAL_GCP& g : gcps) {
            g.dfGCPPixel = Read(std::cin);
            g.dfGCPLine = Read(std::cin);
            g.dfGCPX = Read(std::cin);
            g.dfGCPY = Read(std::cin);
        }
        std::vector<double> fx(nPoints), fy(nPoints), ix(nPoints), iy(nPoints);
        for (size_t i = 0; i < nPoints; ++i) {
            fx[i] = Read(std::cin);
            fy[i] = Read(std::cin);
        }
        for (size_t i = 0; i < nPoints; ++i) {
            ix[i] = Read(std::cin);
            iy[i] = Read(std::cin);
        }

        GDALDatasetH ds = GDALCreate(GDALGetDriverByName("MEM"), "", 1000, 800, 1, GDT_Byte, nullptr);
        OGRSpatialReferenceH srs = nullptr;
        if (crs != "-") {
            srs = OSRNewSpatialReference(nullptr);
            OSRSetFromUserInput(srs, crs.c_str());
            OSRSetAxisMappingStrategy(srs, OAMS_TRADITIONAL_GIS_ORDER);
        }
        GDALSetGCPs2(ds, static_cast<int>(nGcps), gcps.data(), srs);
        char** options = nullptr;
        if (method == 1) options = CSLSetNameValue(options, "METHOD", "GCP_HOMOGRAPHY");
        if (method == 2) options = CSLSetNameValue(options, "METHOD", "GCP_POLYNOMIAL");
        if (maxOrder != 0) options = CSLSetNameValue(options, "MAX_GCP_ORDER", std::to_string(maxOrder).c_str());
        CPLErrorReset();
        void* t = GDALCreateGenImgProjTransformer2(ds, nullptr, options);
        std::printf("case %s %s %s\n", name.c_str(), t ? "ok" : "fail", t ? "" : CPLGetLastErrorMsg());
        if (t) {
            std::vector<int> ok;
            Run(t, FALSE, fx, fy, ok);
            for (size_t i = 0; i < nPoints; ++i) std::printf("f %a %a %d\n", fx[i], fy[i], ok[i]);
            Run(t, TRUE, ix, iy, ok);
            for (size_t i = 0; i < nPoints; ++i) std::printf("i %a %a %d\n", ix[i], iy[i], ok[i]);
            GDALDestroyGenImgProjTransformer(t);
        }
        CSLDestroy(options);
        if (srs) OSRDestroySpatialReference(srs);
        GDALDeinitGCPs(static_cast<int>(nGcps), gcps.data());
        GDALClose(ds);
    }
    return 0;
}
