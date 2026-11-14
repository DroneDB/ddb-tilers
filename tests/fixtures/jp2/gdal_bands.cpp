// SPDX-License-Identifier: MPL-2.0
//
// Reference tool for expected.py (not built by the project): the CRC-32 of every band of a raster,
// or of a window of it, as the GDAL libddb ships reads it (vcpkg's static GDAL 3.13.3 and
// OpenJPEG 2.5.4), with .aux.xml sidecars disabled as libddb disables them. The system GDAL's
// OpenJPEG can decode lossy files differently (its build rounds the irreversible colour transform
// differently), so the expectations of the samples come from this tool when it is given. Built
// e.g. with:
//
//   cmake_minimum_required(VERSION 3.25)
//   project(gdal_bands CXX)
//   find_package(GDAL CONFIG REQUIRED)
//   find_package(ZLIB REQUIRED)
//   add_executable(gdal_bands gdal_bands.cpp)
//   target_link_libraries(gdal_bands PRIVATE GDAL::GDAL ZLIB::ZLIB)
//
// configured with vcpkg's toolchain on libddb's vcpkg_installed tree (VCPKG_MANIFEST_MODE=OFF).
//
// Usage: gdal_bands FILE [X Y W H]. Output: "<band> <crc32 as 8 hex digits>" per band, or
// "<band> fail" when GDAL cannot read it; "fail" alone when GDAL cannot open the file.

#include <cstdio>
#include <cstdlib>
#include <vector>

#include <zlib.h>

#include "cpl_conv.h"
#include "gdal.h"

int main(int argc, char** argv) {
    if (argc != 2 && argc != 6) {
        std::fprintf(stderr, "usage: gdal_bands FILE [X Y W H]\n");
        return 2;
    }
    GDALAllRegister();
    CPLSetConfigOption("GDAL_PAM_ENABLED", "NO");
    CPLPushErrorHandler(CPLQuietErrorHandler);
    GDALDatasetH ds = GDALOpen(argv[1], GA_ReadOnly);
    if (ds == nullptr) {
        std::printf("fail\n");
        return 0;
    }
    int x = 0, y = 0, w = GDALGetRasterXSize(ds), h = GDALGetRasterYSize(ds);
    if (argc == 6) {
        x = std::atoi(argv[2]);
        y = std::atoi(argv[3]);
        w = std::atoi(argv[4]);
        h = std::atoi(argv[5]);
    }
    for (int b = 1; b <= GDALGetRasterCount(ds); ++b) {
        GDALRasterBandH band = GDALGetRasterBand(ds, b);
        const GDALDataType t = GDALGetRasterDataType(band);
        std::vector<unsigned char> buf(static_cast<size_t>(w) * h * GDALGetDataTypeSizeBytes(t));
        if (GDALRasterIO(band, GF_Read, x, y, w, h, buf.data(), w, h, t, 0, 0) != CE_None) {
            std::printf("%d fail\n", b);
            continue;
        }
        std::printf("%d %08lx\n", b, crc32(0, buf.data(), static_cast<uInt>(buf.size())));
    }
    GDALClose(ds);
    return 0;
}
