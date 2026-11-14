// SPDX-License-Identifier: MPL-2.0
//
// Reference tool for tests/fixtures/photo/geoproject.py (not built by the project): libddb's
// geoprojection of a photo with the GDAL libddb ships (vcpkg's GDAL 3.13.3): GDALTranslate to a
// MEM dataset with -a_srs EPSG:4326 and the four GCPs of the photo's footprint (upper left at
// pixel (0, 0), lower left (0, H), lower right (W, H), upper right (W, 0), with -outsize when
// OUTSIZE is not "-"), then GDALWarp -of GTiff -co COMPRESS=JPEG -dstalpha, or the warp arguments
// that follow "--replace". Built like gdal_expected.cpp (with C++20), run with PROJ_DATA set.
//
// Usage: gdal_geoproject IN OUT WIDTH HEIGHT OUTSIZE ULX ULY LLX LLY LRX LRY URX URY
//                        [--replace WARP_ARGS... | EXTRA_WARP_ARGS...]
// Output: "chunk sx sy sw sh dx dy dw dh" for every chunk GDAL's warp kernel processes (with
// CHUNKS=1 in the environment), then "ok <width> <height> gt0 ... gt5" (%.17g) or "fail ...".

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <string>

#include "cpl_conv.h"
#include "cpl_error.h"
#include "cpl_string.h"
#include "gdal.h"
#include "gdal_utils.h"

namespace {

void CPL_STDCALL Handler(CPLErr level, CPLErrorNum, const char* message) {
    if (level != CE_Debug) {
        std::fprintf(stderr, "%s\n", message);
        return;
    }
    const char* s = std::strstr(message, "Src=");
    const char* d = std::strstr(message, "Dst=");
    int v[8];
    if (s && d && std::sscanf(s, "Src=%d,%d,%dx%d", &v[0], &v[1], &v[2], &v[3]) == 4 &&
        std::sscanf(d, "Dst=%d,%d,%dx%d", &v[4], &v[5], &v[6], &v[7]) == 4)
        std::printf("chunk %d %d %d %d %d %d %d %d\n", v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 14) {
        std::fprintf(stderr, "usage: gdal_geoproject IN OUT W H OUTSIZE ULX ULY LLX LLY LRX LRY URX URY [ARGS...]\n");
        return 2;
    }
    GDALAllRegister();
    CPLSetConfigOption("GDAL_PAM_ENABLED", "NO");
    if (std::getenv("CHUNKS")) CPLSetConfigOption("CPL_DEBUG", "ON");
    CPLSetErrorHandler(Handler);
    GDALDatasetH src = GDALOpen(argv[1], GA_ReadOnly);
    if (!src) {
        std::printf("fail cannot open %s\n", argv[1]);
        return 1;
    }
    const int width = std::atoi(argv[3]), height = std::atoi(argv[4]);
    const std::string outsize = argv[5];
    double fp[4][2];
    for (int i = 0; i < 4; ++i) {
        fp[i][0] = std::atof(argv[6 + 2 * i]);
        fp[i][1] = std::atof(argv[7 + 2 * i]);
    }
    // libddb's translateWithGcps.
    CPLStringList t;
    for (const char* a : {"-of", "MEM", "-a_srs", "EPSG:4326"}) t.AddString(a);
    int sw = width, sh = height;
    if (outsize != "-") {
        double ratio;
        t.AddString("-outsize");
        t.AddString(outsize.c_str());
        if (outsize.back() == '%') {
            ratio = std::atof(outsize.c_str()) / 100.0;
            t.AddString(outsize.c_str());
        } else {
            ratio = std::atof(outsize.c_str()) / width;
            t.AddString(std::format("{:.6f}", ratio * height).c_str());
        }
        sw = static_cast<int>(static_cast<double>(width) * ratio);
        sh = static_cast<int>(static_cast<double>(height) * ratio);
    }
    auto gcp = [&](int c, int r, const double* g) {
        t.AddString("-gcp");
        t.AddString(std::to_string(c).c_str());
        t.AddString(std::to_string(r).c_str());
        t.AddString(std::format("{:.13f}", g[0]).c_str());
        t.AddString(std::format("{:.13f}", g[1]).c_str());
    };
    gcp(0, 0, fp[0]);
    gcp(0, sh, fp[1]);
    gcp(sw, sh, fp[2]);
    gcp(sw, 0, fp[3]);
    GDALTranslateOptions* to = GDALTranslateOptionsNew(t.List(), nullptr);
    GDALDatasetH mem = GDALTranslate("", src, to, nullptr);
    GDALTranslateOptionsFree(to);
    if (!mem) {
        std::printf("fail translate %s\n", CPLGetLastErrorMsg());
        return 1;
    }
    // libddb's warpToFile, or the warp arguments given.
    CPLStringList w;
    int first = 14;
    if (argc > 14 && std::string(argv[14]) == "--replace") {
        first = 15;
    } else {
        for (const char* a : {"-of", "GTiff", "-co", "COMPRESS=JPEG", "-dstalpha"}) w.AddString(a);
    }
    for (int i = first; i < argc; ++i) w.AddString(argv[i]);
    GDALWarpAppOptions* wo = GDALWarpAppOptionsNew(w.List(), nullptr);
    GDALDatasetH out = GDALWarp(argv[2], nullptr, 1, &mem, wo, nullptr);
    GDALWarpAppOptionsFree(wo);
    if (!out) {
        std::printf("fail warp %s\n", CPLGetLastErrorMsg());
        return 1;
    }
    double gt[6] = {};
    GDALGetGeoTransform(out, gt);
    std::printf("ok %d %d %.17g %.17g %.17g %.17g %.17g %.17g\n", GDALGetRasterXSize(out), GDALGetRasterYSize(out),
                gt[0], gt[1], gt[2], gt[3], gt[4], gt[5]);
    GDALClose(out);
    GDALClose(mem);
    GDALClose(src);
    return 0;
}
