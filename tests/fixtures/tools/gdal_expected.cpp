// SPDX-License-Identifier: MPL-2.0
//
// Reference tool for gdal_expected.py (not built by the project): what the GDAL libddb ships
// (vcpkg's static GDAL 3.13.3 with its libjpeg-turbo, libpng, giflib, libwebp, libtiff and
// OpenJPEG) reads in a raster, as the tab-separated lines the parity tests check
// (tests/unit/gdal_parity.h), .aux.xml sidecars disabled as libddb disables them. The system
// GDAL can decode some files differently (another OpenJPEG rounds the irreversible colour
// transform of lossy JPEG 2000 differently), so the expectations come from this tool. Built e.g.
// with:
//
//   cmake_minimum_required(VERSION 3.25)
//   project(gdal_expected CXX)
//   set(CMAKE_CXX_STANDARD 20)
//   find_package(GDAL CONFIG REQUIRED)
//   find_package(ZLIB REQUIRED)
//   add_executable(gdal_expected gdal_expected.cpp)
//   target_link_libraries(gdal_expected PRIVATE GDAL::GDAL ZLIB::ZLIB)
//
// configured with vcpkg's toolchain on libddb's vcpkg_installed tree (VCPKG_MANIFEST_MODE=OFF),
// and run with PROJ_DATA set to its share/proj.
//
// Usage: gdal_expected FILE NAME [--grid-lossy] [DRIVER...]: the lines of FILE, reported under
// NAME, opened with one of the drivers given (any driver when none is); --grid-lossy adds the grid
// rows of a lossy file. Lines:
//
//     size     NAME WIDTH HEIGHT BANDS TYPE
//     band     NAME BAND INTERP NBITS CRC     CRC-32 of the band's samples (host order); NBITS "-"
//                                             when none; CRC "-" when the band is checked on windows
//     window   NAME BAND X Y W H CRC          the same for a window, for rasters too large to read whole
//     grid     NAME BAND STEP HEX             with --grid-lossy, for a file GDAL reports as lossy
//                                             (COMPRESSION_REVERSIBILITY) and read whole: the band's
//                                             samples at every STEP-th column and row (at most 1024),
//                                             hex, host order. A decoding in floating point (JPEG
//                                             2000's 9/7 wavelet) rounds differently in other builds
//                                             of its library: the CRC is exact where the
//                                             expectations were made (Linux), the grid within 1
//                                             elsewhere
//     nodata   NAME BAND VALUE                the band's nodata value (%.17g)
//     mask     NAME CRC                       the per-dataset mask (not an alpha band, not nodata)
//     ct       NAME ENTRIES CRC               the colour table of band 1 (CRC-32 of its R, G, B bytes)
//     gt       NAME GT0 .. GT5                the geotransform (%.17g), when there is one
//     gcp      NAME PIXEL LINE X Y Z          each ground control point (%.17g)
//     crs      NAME WKT                       the CRS (of the geotransform or of the GCPs), WKT2, with
//                                             backslashes, new lines and tabs escaped (\\, \n, \t)
//     rpc      NAME                           the file has RPC metadata
//     refused  NAME                           GDAL cannot open or read the file

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <zlib.h>

#include "cpl_conv.h"
#include "gdal.h"
#include "ogr_srs_api.h"

namespace {

constexpr long long kWholeLimit = 64LL << 20;  // bytes of a band read whole; larger rasters on windows
constexpr double kGridSamples = 1024;          // samples of a band on the grid of a lossy file, at most

std::string G(double v) {
    char b[64];
    std::snprintf(b, sizeof(b), "%.17g", v);
    return b;
}

std::string Crc(const std::vector<unsigned char>& v) {
    char b[16];
    std::snprintf(b, sizeof(b), "%08lx", crc32(0, v.data(), static_cast<uInt>(v.size())));
    return b;
}

std::string Escape(const char* s) {
    std::string out;
    for (; *s; ++s) {
        if (*s == '\\') out += "\\\\";
        else if (*s == '\n') out += "\\n";
        else if (*s == '\t') out += "\\t";
        else out += *s;
    }
    return out;
}

// The samples of band b in a window, or false when GDAL cannot read them.
bool Read(GDALRasterBandH band, int x, int y, int w, int h, std::vector<unsigned char>& out) {
    const GDALDataType t = GDALGetRasterDataType(band);
    out.assign(static_cast<size_t>(w) * h * GDALGetDataTypeSizeBytes(t), 0);
    return GDALRasterIO(band, GF_Read, x, y, w, h, out.data(), w, h, t, 0, 0) == CE_None;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: gdal_expected FILE NAME [--grid-lossy] [DRIVER...]\n");
        return 2;
    }
    int firstDriver = 3;
    const bool gridLossy = argc > 3 && std::strcmp(argv[3], "--grid-lossy") == 0;
    if (gridLossy) firstDriver = 4;
    GDALAllRegister();
    CPLSetConfigOption("GDAL_PAM_ENABLED", "NO");
    CPLPushErrorHandler(CPLQuietErrorHandler);
    const std::string name = argv[2];
    std::vector<const char*> drivers;
    for (int i = firstDriver; i < argc; ++i) drivers.push_back(argv[i]);
    drivers.push_back(nullptr);
    GDALDatasetH ds =
        GDALOpenEx(argv[1], GDAL_OF_RASTER, argc > firstDriver ? drivers.data() : nullptr, nullptr, nullptr);
    if (ds == nullptr) {
        std::printf("refused\t%s\n", name.c_str());
        return 0;
    }
    const int W = GDALGetRasterXSize(ds), H = GDALGetRasterYSize(ds), n = GDALGetRasterCount(ds);
    std::vector<std::string> lines;
    GDALRasterBandH b1 = GDALGetRasterBand(ds, 1);
    const GDALDataType t = GDALGetRasterDataType(b1);
    lines.push_back("size\t" + name + "\t" + std::to_string(W) + "\t" + std::to_string(H) + "\t" + std::to_string(n) +
                    "\t" + GDALGetDataTypeName(t));
    const long long bytes = static_cast<long long>(W) * H * GDALGetDataTypeSizeBytes(t);
    std::vector<std::array<int, 4>> windows;
    if (bytes > kWholeLimit)
        windows = {{0, 0, 1500, 1100}, {W - 1300, H - 700, 1300, 700}, {W / 2 - 600, H / 2 - 600, 1200, 1200}};
    std::vector<unsigned char> buf;
    for (int b = 1; b <= n; ++b) {
        GDALRasterBandH band = GDALGetRasterBand(ds, b);
        const char* nbits = GDALGetMetadataItem(band, "NBITS", "IMAGE_STRUCTURE");
        std::string crc = "-";
        if (windows.empty()) {
            if (!Read(band, 0, 0, W, H, buf)) {
                std::printf("refused\t%s\n", name.c_str());
                return 0;
            }
            crc = Crc(buf);
        }
        lines.push_back("band\t" + name + "\t" + std::to_string(b) + "\t" +
                        GDALGetColorInterpretationName(GDALGetRasterColorInterpretation(band)) + "\t" +
                        (nbits ? nbits : "-") + "\t" + crc);
        const char* reversibility = GDALGetMetadataItem(ds, "COMPRESSION_REVERSIBILITY", "IMAGE_STRUCTURE");
        if (gridLossy && windows.empty() && reversibility && std::strncmp(reversibility, "LOSSY", 5) == 0) {
            const int step = std::max(1, static_cast<int>(std::ceil(std::sqrt(static_cast<double>(W) * H / kGridSamples))));
            const size_t ss = static_cast<size_t>(GDALGetDataTypeSizeBytes(GDALGetRasterDataType(band)));
            std::string hex;
            char h[3];
            for (int y = 0; y < H; y += step)
                for (int x = 0; x < W; x += step)
                    for (size_t k = 0; k < ss; ++k) {
                        std::snprintf(h, sizeof(h), "%02x", buf[(static_cast<size_t>(y) * W + x) * ss + k]);
                        hex += h;
                    }
            lines.push_back("grid\t" + name + "\t" + std::to_string(b) + "\t" + std::to_string(step) + "\t" + hex);
        }
        for (const auto& w : windows) {
            if (!Read(band, w[0], w[1], w[2], w[3], buf)) {
                std::printf("refused\t%s\n", name.c_str());
                return 0;
            }
            lines.push_back("window\t" + name + "\t" + std::to_string(b) + "\t" + std::to_string(w[0]) + "\t" +
                            std::to_string(w[1]) + "\t" + std::to_string(w[2]) + "\t" + std::to_string(w[3]) + "\t" +
                            Crc(buf));
        }
        int has = 0;
        const double nd = GDALGetRasterNoDataValue(band, &has);
        if (has) lines.push_back("nodata\t" + name + "\t" + std::to_string(b) + "\t" + G(nd));
    }
    const int flags = GDALGetMaskFlags(b1);
    if ((flags & GMF_PER_DATASET) && !(flags & (GMF_ALPHA | GMF_NODATA | GMF_ALL_VALID)) && windows.empty()) {
        if (!Read(GDALGetMaskBand(b1), 0, 0, W, H, buf)) {
            std::printf("refused\t%s\n", name.c_str());
            return 0;
        }
        lines.push_back("mask\t" + name + "\t" + Crc(buf));
    }
    if (GDALColorTableH ct = GDALGetRasterColorTable(b1)) {
        std::vector<unsigned char> rgb;
        for (int i = 0; i < GDALGetColorEntryCount(ct); ++i) {
            const GDALColorEntry* e = GDALGetColorEntry(ct, i);
            rgb.push_back(static_cast<unsigned char>(e->c1));
            rgb.push_back(static_cast<unsigned char>(e->c2));
            rgb.push_back(static_cast<unsigned char>(e->c3));
        }
        lines.push_back("ct\t" + name + "\t" + std::to_string(GDALGetColorEntryCount(ct)) + "\t" + Crc(rgb));
    }
    double gt[6];
    if (GDALGetGeoTransform(ds, gt) == CE_None) {
        std::string l = "gt\t" + name;
        for (double v : gt) l += "\t" + G(v);
        lines.push_back(l);
    }
    const GDAL_GCP* gcps = GDALGetGCPs(ds);
    for (int i = 0; i < GDALGetGCPCount(ds); ++i)
        lines.push_back("gcp\t" + name + "\t" + G(gcps[i].dfGCPPixel) + "\t" + G(gcps[i].dfGCPLine) + "\t" +
                        G(gcps[i].dfGCPX) + "\t" + G(gcps[i].dfGCPY) + "\t" + G(gcps[i].dfGCPZ));
    OGRSpatialReferenceH srs = GDALGetSpatialRef(ds);
    if (srs == nullptr) srs = GDALGetGCPSpatialRef(ds);
    if (srs != nullptr) {
        const char* options[] = {"FORMAT=WKT2_2019", "MULTILINE=NO", nullptr};
        char* wkt = nullptr;
        if (OSRExportToWktEx(srs, &wkt, options) == OGRERR_NONE && wkt) lines.push_back("crs\t" + name + "\t" + Escape(wkt));
        CPLFree(wkt);
    }
    if (GDALGetMetadata(ds, "RPC") != nullptr) lines.push_back("rpc\t" + name);
    for (const auto& l : lines) std::printf("%s\n", l.c_str());
    GDALClose(ds);
    return 0;
}
