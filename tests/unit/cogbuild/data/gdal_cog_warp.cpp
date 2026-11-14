// SPDX-License-Identifier: MPL-2.0
//
// Reference tool for the warped COG tests (not built by the project, Linux): runs GDALWarp on a
// raster with the arguments libddb's COG build gives it (-of COG -t_srs EPSG:3857 -r bilinear,
// TILING_SCHEME=GoogleMapsCompatible, ZOOM_LEVEL_STRATEGY=UPPER, its transparency and
// compression rules), and prints the grid of the output, or GDAL's error. Built against the GDAL
// libddb ships (vcpkg's static GDAL 3.13.3), e.g.:
//
//   cmake_minimum_required(VERSION 3.25)
//   project(gdal_cog_warp CXX)
//   find_package(GDAL CONFIG REQUIRED)
//   add_executable(gdal_cog_warp gdal_cog_warp.cpp)
//   target_link_libraries(gdal_cog_warp PRIVATE GDAL::GDAL)
//
// configured with -DCMAKE_PREFIX_PATH=<vcpkg_installed>/x64-linux, and run with PROJ_DATA set
// to <vcpkg_installed>/x64-linux/share/proj.
//
// Usage: gdal_cog_warp IN.tif OUT.tif [extra gdalwarp arguments...] (-wm 512 unless given)
// Output: "gdal <version>", "chunk sx sy sw sh dx dy dw dh" for every chunk GDAL's warp kernel
// processes (source and destination windows, from GDAL's debug messages), then
// "ok <width> <height> gt0 ... gt5" (%.17g) or "fail <message>".

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>

#include "cpl_conv.h"
#include "cpl_error.h"
#include "cpl_string.h"
#include "gdal.h"
#include "gdal_utils.h"

namespace {

// libddb's JPEG rule: 8-bit RGB, or RGBA whose fourth band is alpha, without nodata.
bool JpegEligible(GDALDatasetH ds, bool hasNoData) {
    const int n = GDALGetRasterCount(ds);
    if ((n != 3 && n != 4) || hasNoData) return false;
    for (int b = 1; b <= n; ++b)
        if (GDALGetRasterDataType(GDALGetRasterBand(ds, b)) != GDT_Byte) return false;
    return n == 3 || GDALGetRasterColorInterpretation(GDALGetRasterBand(ds, 4)) == GCI_AlphaBand;
}

// GDAL's debug message of each chunk the warp kernel runs: "... Src=x,y,wxh Dst=x,y,wxh".
void CPL_STDCALL Handler(CPLErr level, CPLErrorNum, const char* message) {
    if (level != CE_Debug) return;
    const char* s = std::strstr(message, "Src=");
    const char* d = std::strstr(message, "Dst=");
    int v[8];
    if (s && d && std::sscanf(s, "Src=%d,%d,%dx%d", &v[0], &v[1], &v[2], &v[3]) == 4 &&
        std::sscanf(d, "Dst=%d,%d,%dx%d", &v[4], &v[5], &v[6], &v[7]) == 4)
        std::printf("chunk %d %d %d %d %d %d %d %d\n", v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s IN OUT [gdalwarp args...]\n", argv[0]);
        return 2;
    }
    GDALAllRegister();
    CPLSetConfigOption("CPL_DEBUG", "ON");
    CPLSetErrorHandler(Handler);
    std::printf("gdal %s\n", GDALVersionInfo("RELEASE_NAME"));
    GDALDatasetH src = GDALOpen(argv[1], GA_ReadOnly);
    if (!src) {
        std::printf("fail cannot open %s\n", argv[1]);
        return 1;
    }
    GDALRasterBandH band1 = GDALGetRasterBand(src, 1);
    int hasNoData = 0;
    const double noData = GDALGetRasterNoDataValue(band1, &hasNoData);
    const int maskFlags = GDALGetMaskFlags(band1);
    const bool perDatasetMask =
        (maskFlags & GMF_PER_DATASET) != 0 && (maskFlags & GMF_ALPHA) == 0 && (maskFlags & GMF_NODATA) == 0;

    CPLStringList args;
    for (const char* a : {"-of", "COG", "-t_srs", "EPSG:3857", "-wo", "NUM_THREADS=ALL_CPUS", "-co",
                          "NUM_THREADS=ALL_CPUS", "-r", "bilinear", "-co", "TILING_SCHEME=GoogleMapsCompatible", "-co",
                          "ZOOM_LEVEL_STRATEGY=UPPER"})
        args.AddString(a);
    // The warp memory of libddb's budget (DDB_WARP_MEMORY_MB, 512 MB by default) unless the extra
    // arguments give one.
    bool wm = false;
    for (int i = 3; i < argc; ++i) wm = wm || std::string(argv[i]) == "-wm";
    if (!wm) {
        args.AddString("-wm");
        args.AddString("512");
    }
    if (hasNoData) {
        if (std::isfinite(noData) && std::fabs(noData) < 1e30) {
            std::ostringstream o;
            o << std::setprecision(15) << noData;
            args.AddString("-wo");
            args.AddString("UNIFIED_SRC_NODATA=YES");
            args.AddString("-dstnodata");
            args.AddString(o.str().c_str());
        } else {
            args.AddString("-dstalpha");
        }
    } else if (perDatasetMask) {
        args.AddString("-dstalpha");
    }
    if (JpegEligible(src, hasNoData != 0)) {
        args.AddString("-co");
        args.AddString("COMPRESS=JPEG");
        args.AddString("-co");
        args.AddString("QUALITY=75");
    } else {
        args.AddString("-co");
        args.AddString("COMPRESS=LZW");
        args.AddString("-co");
        args.AddString("PREDICTOR=YES");
    }
    for (int i = 3; i < argc; ++i) args.AddString(argv[i]);

    GDALWarpAppOptions* options = GDALWarpAppOptionsNew(args.List(), nullptr);
    CPLErrorReset();
    GDALDatasetH out = GDALWarp(argv[2], nullptr, 1, &src, options, nullptr);
    GDALWarpAppOptionsFree(options);
    if (!out) {
        std::printf("fail %s\n", CPLGetLastErrorMsg());
        GDALClose(src);
        return 1;
    }
    double gt[6] = {};
    GDALGetGeoTransform(out, gt);
    std::printf("ok %d %d %.17g %.17g %.17g %.17g %.17g %.17g\n", GDALGetRasterXSize(out), GDALGetRasterYSize(out),
                gt[0], gt[1], gt[2], gt[3], gt[4], gt[5]);
    GDALClose(out);
    GDALClose(src);
    return 0;
}
