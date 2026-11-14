// SPDX-License-Identifier: MPL-2.0
//
// The geoprojection of a photo (BuildOptions::photo, the "gcps" options of ddbtBuildCog2)
// against GDAL 3.13.3 (vcpkg) as libddb runs it (tests/fixtures/photo/geoproject.py,
// tests/fixtures/tools/gdal_geoproject.cpp): the photo georeferenced by the four GCPs of its
// footprint, then
//   - in the GCPs' CRS (targetSrs "source"): gdalwarp -dstalpha's grid, chunks and pixels before
//     JPEG (nearest neighbour), and the same output JPEG-compressed with a mask;
//   - in EPSG:3857: the grid and pixels of the COG one gdalwarp makes from the photo (bilinear),
//     and how close it is to libddb's COG of the photo, which warps twice (the JPEG GeoTIFF of the
//     geoprojection, then its COG);
// and the photos refused.

#include <gtest/gtest.h>

#include <libdeflate.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../test_env.h"
#include "../tiff/tiff_test_util.h"
#include "cog/build/cog_builder.h"
#include "cog/tiff/geotiff.h"
#include "cog/tiff/tiff_reader.h"
#include "cog/warp/warp_producer.h"

namespace ddbt::cog::build {
namespace {

std::string Fixture(const std::string& name) { return std::string(DDBT_TEST_FIXTURES) + "/photo/" + name; }

struct Case {
    std::string name, photo, target;
    double outScale = 1;
    uint32_t width = 0, height = 0;
    std::array<double, 6> gt{};
    std::vector<crs::Gcp> gcps;
    std::vector<std::string> crcs;
    std::vector<std::array<int, 8>> chunks;
};

std::vector<Case> Cases() {
    std::ifstream in(Fixture("geoproject.tsv"));
    std::map<std::string, Case> byName;
    std::vector<std::string> order;
    std::string line;
    while (std::getline(in, line)) {
        std::vector<std::string> f;
        std::stringstream s(line);
        std::string v;
        while (std::getline(s, v, '\t')) f.push_back(v);
        if (f.size() < 2) continue;
        Case& c = byName[f[1]];
        auto d = [&](size_t i) { return std::strtod(f[i].c_str(), nullptr); };
        if (f[0] == "case") {
            order.push_back(f[1]);
            c.name = f[1];
            c.photo = f[2];
            c.target = f[3];
            c.outScale = d(4);
            c.width = static_cast<uint32_t>(std::stoul(f[5]));
            c.height = static_cast<uint32_t>(std::stoul(f[6]));
            for (size_t i = 0; i < 6; ++i) c.gt[i] = d(7 + i);
        } else if (f[0] == "gcp") {
            c.gcps.push_back({d(2), d(3), d(4), d(5), 0});
        } else if (f[0] == "band") {
            c.crcs.push_back(f[3]);
        } else if (f[0] == "chunk") {
            std::array<int, 8> k{};
            for (size_t i = 0; i < 8; ++i) k[i] = std::stoi(f[2 + i]);
            c.chunks.push_back(k);
        }
    }
    std::vector<Case> out;
    for (const std::string& n : order) out.push_back(byName[n]);
    return out;
}

BuildOptions PhotoOptions(const Case& c, bool jpeg) {
    BuildOptions o;
    o.statistics = false;
    o.noJpeg = !jpeg;
    PhotoGeoreference g;
    g.gcps = c.gcps;
    g.sourceCrs = c.target == "source";
    g.outScale = c.outScale;
    o.photo = g;
    return o;
}

// The full-resolution bands of a GeoTIFF (and its mask as the last one, 0 / 255, when it has one).
std::vector<std::vector<uint8_t>> Bands(const std::string& path, bool withMask) {
    auto r = tiff::TiffReader::Open(path);
    EXPECT_TRUE(r.ok()) << r.status().message();
    if (!r.ok()) return {};
    const tiff::TiffReader& rd = **r;
    const tiff::IfdInfo& m = rd.Ifd(rd.MainImage());
    std::vector<uint32_t> bands(m.samplesPerPixel);
    for (uint32_t b = 0; b < m.samplesPerPixel; ++b) bands[b] = b;
    auto w = tiff::testing::ReadWindow(rd, m.index, bands, 0, 0, m.width, m.height, 1);
    EXPECT_TRUE(w.ok()) << w.status().message();
    if (!w.ok()) return {};
    std::vector<std::vector<uint8_t>> out = std::move(*w);
    if (withMask) {
        const auto mask = rd.MaskOf(m.index);
        EXPECT_TRUE(mask.has_value()) << path;
        if (!mask) return {};
        auto mw = tiff::testing::ReadWindow(rd, *mask, {0}, 0, 0, m.width, m.height, 255);
        EXPECT_TRUE(mw.ok());
        if (!mw.ok()) return {};
        out.push_back(std::move((*mw)[0]));
    }
    return out;
}

std::string Crc(const std::vector<uint8_t>& v) {
    char b[16];
    std::snprintf(b, sizeof(b), "%08x", libdeflate_crc32(0, v.data(), v.size()));
    return b;
}

std::array<double, 6> GeoTransformOf(const std::string& path) {
    auto r = tiff::TiffReader::Open(path);
    EXPECT_TRUE(r.ok());
    if (!r.ok()) return {};
    auto g = tiff::ReadGeoreferencing(**r);
    EXPECT_TRUE(g.ok() && g->geoTransform);
    if (!g.ok() || !g->geoTransform) return {};
    return *g->geoTransform;
}

class PhotoGeoproject : public testing::TestWithParam<Case> {};

TEST_P(PhotoGeoproject, IsGdalsWarpOfThePhoto) {
    const Case& c = GetParam();
    test::TempDir dir("ddbt-photo");
    const std::string output = dir.File("out.tif");
    const BuildOptions options = PhotoOptions(c, false);
    const Status s = BuildCog(Fixture(c.photo), output, options);
    ASSERT_TRUE(s.ok()) << s.message();

    // GDAL's grid, to the bit.
    auto r = tiff::TiffReader::Open(output);
    ASSERT_TRUE(r.ok());
    EXPECT_EQ((*r)->Ifd((*r)->MainImage()).width, c.width);
    EXPECT_EQ((*r)->Ifd((*r)->MainImage()).height, c.height);
    EXPECT_EQ(GeoTransformOf(output), c.gt);

    // GDAL's chunks of libddb's JPEG GeoTIFF.
    if (!c.chunks.empty()) {
        auto src = OpenBuildSource(Fixture(c.photo), options);
        ASSERT_TRUE(src.ok()) << src.status().message();
        const warp::WarpSettings ws = WarpSettingsOf(options);
        auto plan = warp::WarpPlan(**src, ws);
        ASSERT_TRUE(plan.ok()) << plan.status().message();
        auto chunks = warp::GdalWarpChunks(**src, *plan, ws);
        ASSERT_TRUE(chunks.ok()) << chunks.status().message();
        std::vector<std::array<int, 8>> ours;
        for (const warp::WarpChunk& k : *chunks) ours.push_back({k.sx, k.sy, k.sw, k.sh, k.dx, k.dy, k.dw, k.dh});
        EXPECT_EQ(ours, c.chunks);
    }

    // GDAL's pixels (the colour bands and the alpha band), before any JPEG. Into the GCPs' CRS the
    // warp is arithmetic alone (the homography, GDAL's approximate transformer): the same bits on
    // every platform. Into EPSG:3857 it goes through PROJ's transcendental functions, which
    // another C library may round 1 ulp apart: exact where the expectations are made (Linux), the
    // closeness of PhotoGeoprojectJpeg.TheCogIsCloseToLibddbsTwoWarps elsewhere.
    const auto bands = Bands(output, false);
    ASSERT_EQ(bands.size(), c.crcs.size());
#if !defined(__linux__)
    if (c.target != "source") return;
#endif
    for (size_t b = 0; b < bands.size(); ++b) EXPECT_EQ(Crc(bands[b]), c.crcs[b]) << "band " << b + 1;
}

INSTANTIATE_TEST_SUITE_P(Gdal, PhotoGeoproject, testing::ValuesIn(Cases()),
                         [](const testing::TestParamInfo<Case>& i) { return i.param.name; });

const Case& CaseNamed(const std::string& name) {
    static const std::vector<Case> cases = Cases();
    for (const Case& c : cases)
        if (c.name == name) return c;
    throw std::runtime_error("no case " + name);
}

TEST(PhotoGeoprojectJpeg, TheSourceCrsOutputIsJpegWithTheAlphaAsItsMask) {
    // libddb's geoprojection is a JPEG GeoTIFF with an alpha band (JPEG-compressed with the
    // colours); here the colours are YCbCr JPEG and the alpha the exact 1-bit mask.
    const Case c = CaseNamed("jpg_source");
    test::TempDir dir("ddbt-photo");
    const std::string jpeg = dir.File("jpeg.tif"), lossless = dir.File("lossless.tif");
    ASSERT_TRUE(BuildCog(Fixture(c.photo), jpeg, PhotoOptions(c, true)).ok());
    ASSERT_TRUE(BuildCog(Fixture(c.photo), lossless, PhotoOptions(c, false)).ok());
    auto r = tiff::TiffReader::Open(jpeg);
    ASSERT_TRUE(r.ok());
    const tiff::IfdInfo& m = (*r)->Ifd((*r)->MainImage());
    EXPECT_EQ(m.compression, 7);
    EXPECT_EQ(m.samplesPerPixel, 3);
    EXPECT_EQ(GeoTransformOf(jpeg), c.gt);
    const auto j = Bands(jpeg, true);
    const auto l = Bands(lossless, false);
    ASSERT_EQ(j.size(), 4u);
    ASSERT_EQ(l.size(), 4u);
    EXPECT_EQ(j[3], l[3]);  // the mask is the alpha band
    // The colours within JPEG's error of the lossless ones. Measured: 3.9 on average (YCbCr 4:2:0
    // at quality 75); libddb's RGB JPEG of the same pixels errs by 2.4 in the colours and by up
    // to 36 in the alpha of 6.8 % of the pixels.
    double sum = 0;
    size_t n = 0;
    for (size_t b = 0; b < 3; ++b)
        for (size_t i = 0; i < j[b].size(); ++i)
            if (l[3][i] != 0) {
                sum += std::abs(static_cast<int>(j[b][i]) - static_cast<int>(l[b][i]));
                ++n;
            }
    ASSERT_GT(n, 0u);
    EXPECT_LT(sum / static_cast<double>(n), 5.0);
}

TEST(PhotoGeoprojectJpeg, TheCogIsCloseToLibddbsTwoWarps) {
    // libddb's COG of a photo warps it twice: nearest neighbour into EPSG:4326 (a JPEG GeoTIFF
    // whose alpha band is JPEG-compressed too), then bilinear into EPSG:3857; one bilinear warp of
    // the photo gives the same grid. Measured on this photo: a mean difference of 2.4 to 2.8 per
    // band (99th percentile 24, the largest at the sharp edges the first warp moved by up to half
    // a pixel), 61 pixels in the mask that libddb's does not have (0.17 %), and 2362 pixels of
    // libddb's mask outside the photo (its lossy alpha above 0 around the edges).
    const Case c = CaseNamed("jpg_3857");
    test::TempDir dir("ddbt-photo");
    const std::string ours = dir.File("ours.tif");
    ASSERT_TRUE(BuildCog(Fixture(c.photo), ours, PhotoOptions(c, true)).ok());
    const std::string gdal = Fixture("gdal/geo_photo_2steps.tif");
    EXPECT_EQ(GeoTransformOf(ours), GeoTransformOf(gdal));
    const auto a = Bands(ours, true), b = Bands(gdal, true);
    ASSERT_EQ(a.size(), 4u);
    ASSERT_EQ(b.size(), 4u);
    ASSERT_EQ(a[0].size(), b[0].size());
    size_t both = 0, oursOnly = 0;
    std::vector<int> diffs;
    for (size_t i = 0; i < a[3].size(); ++i) {
        const bool va = a[3][i] != 0, vb = b[3][i] != 0;
        if (va && !vb) ++oursOnly;
        if (!(va && vb)) continue;
        ++both;
        for (size_t k = 0; k < 3; ++k) diffs.push_back(std::abs(static_cast<int>(a[k][i]) - static_cast<int>(b[k][i])));
    }
    ASSERT_GT(both, 30000u);
    EXPECT_LT(static_cast<double>(oursOnly), 0.005 * static_cast<double>(both));
    double sum = 0;
    for (int d : diffs) sum += d;
    EXPECT_LT(sum / static_cast<double>(diffs.size()), 4.0);
    std::sort(diffs.begin(), diffs.end());
    EXPECT_LE(diffs[diffs.size() * 99 / 100], 32);
}

TEST(PhotoGeoprojectRefusals, ASourceGeoreferencedAlreadyIsRefused) {
    // A GeoTIFF with a CRS and a geotransform: GCPs do not replace its georeferencing.
    test::TempDir dir("ddbt-photo");
    BuildOptions o = PhotoOptions(CaseNamed("jpg_3857"), false);
    const Status s = BuildCog(std::string(DDBT_TEST_FIXTURES) + "/gdal/byte_gcp.tif", dir.File("out.tif"), o);
    ASSERT_FALSE(s.ok());
    EXPECT_NE(s.message().find("georeferenced already"), std::string::npos) << s.message();
}

TEST(PhotoGeoprojectRefusals, APhotoWithoutGcpsHasNoCrs) {
    test::TempDir dir("ddbt-photo");
    BuildOptions o;
    o.statistics = false;
    const Status s = BuildCog(Fixture("geo_photo.jpg"), dir.File("out.tif"), o);
    ASSERT_FALSE(s.ok());
    EXPECT_NE(s.message().find("no coordinate reference system"), std::string::npos) << s.message();
}

}  // namespace
}  // namespace ddbt::cog::build
