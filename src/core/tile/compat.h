// SPDX-License-Identifier: MPL-2.0
#pragma once

// Faithful copies of the tile maths of the baseline (DroneDB 2.4.1, libddb): GlobalMercator
// (src/library/tiler.cpp), Tiler::getMinMaxCoordsForZ and the tms flag, GDALTiler::open's
// bounds and zoom range, GDALTiler::geoQuery / tileWindow (src/library/gdaltiler.cpp) with
// their integer truncations, and PointCloudTiler's zoom range, padding and pixel mapping
// (src/library/pctiler.cpp). Same expressions in the same order, so the results are the same
// bits (the file is compiled without floating-point contraction). Use them to reproduce the
// baseline's output (compat mode) and to compare; new code uses core/tile/web_mercator.h.
//
// Rows: GlobalMercator counts rows from the south (TMS). The baseline's `tms` flag is inverted
// with respect to its name: with tms == true the row a caller passes is an XYZ row (north
// origin) and is converted with 2^z - 1 - y; with tms == false the row is used as is, i.e. it
// must already be a TMS row. See InternalRow().

#include <optional>

namespace ddbt::tile::compat {

struct Point {
    double x = 0, y = 0;
};
struct PointI {
    int x = 0, y = 0;
};

/// Bounds of tiles or metres, inclusive for tiles (BoundingBox<T> of geo.h).
template <typename T>
struct Box {
    T min{}, max{};
    /// geo.h BoundingBox::contains: inclusive on both ends.
    bool Contains(int x, int y) const noexcept { return x >= min.x && x <= max.x && y >= min.y && y <= max.y; }
};

/// DroneDB ddb::GlobalMercator (gdal2tiles' GlobalMercator), TMS rows.
class GlobalMercator {
public:
    explicit GlobalMercator(int tileSize);

    int tileSize() const noexcept { return tileSize_; }
    double originShift() const noexcept { return originShift_; }
    double initialResolution() const noexcept { return initialResolution_; }

    /// Bounds of tile (tx, ty TMS) in EPSG:3857.
    Box<Point> tileBounds(int tx, int ty, int zoom) const noexcept;
    /// Tile for given mercator coordinates: ceil(px / size) - 1 (so a point on a tile edge
    /// belongs to the tile west / south of it), not clamped.
    PointI metersToTile(double mx, double my, int zoom) const noexcept;
    Point pixelsToMeters(double px, double py, int zoom) const noexcept;
    Point metersToPixels(double mx, double my, int zoom) const noexcept;
    PointI pixelsToTile(double px, double py) const noexcept;
    /// initialResolution / 2^zoom
    double resolution(int zoom) const noexcept;
    /// round(log2(initialResolution / meterLength * tileSize))
    int zoomForLength(double meterLength) const noexcept;
    /// First i with pixelSize > resolution(i), minus 1 (so -1 when pixelSize > resolution(0));
    /// 0 when none of the 99 levels qualifies.
    int zoomForPixelSize(double pixelSize) const noexcept;

private:
    int tileSize_;
    double originShift_;
    double initialResolution_;
    int maxZoomLevel_;
};

/// Tiler::getMinMaxCoordsForZ: metersToTile of the two corners, columns clamped to
/// [0, 2^tz - 1], rows NOT clamped (TMS rows). Tile numbers beyond int saturate.
Box<PointI> MinMaxCoordsForZ(const GlobalMercator& m, double oMinX, double oMinY, double oMaxX, double oMaxY,
                             int tz) noexcept;

/// Tiler::tmsToXYZ / xyzToTMS: int(pow(2, tz) - 1) - ty (saturated to int beyond z30).
int FlipRow(int ty, int tz) noexcept;

/// The row the baseline's GlobalMercator works with for a requested row: FlipRow(ty) when
/// `tms` is true (the caller passed an XYZ row), ty unchanged otherwise.
int InternalRow(int ty, int tz, bool tms) noexcept;

/// GDALTiler::open: EPSG:3857 bounds of a raster from its geotransform. Note the baseline's
/// quirk, kept: oMinY = gt[3] - ysize * gt[1] (the pixel WIDTH, not -gt[5]).
struct RasterBounds {
    double oMinX, oMaxX, oMaxY, oMinY;
};
RasterBounds RasterBoundsOf(const double gt[6], int rasterXSize, int rasterYSize) noexcept;

/// GDALTiler::open: [tMinZ, tMaxZ].
struct ZoomRange {
    int min, max;
};
ZoomRange RasterZoomRange(const GlobalMercator& m, const double gt[6], int rasterXSize, int rasterYSize) noexcept;

/// GDALTiler: GeoExtent / GQResult.
struct GeoExtent {
    int x = 0, y = 0, xsize = 0, ysize = 0;
};
struct GQResult {
    GeoExtent r;  ///< window read from the raster (pixels)
    GeoExtent w;  ///< where it lands in the tile (pixels of the tile)
};

/// GDALTiler::geoQuery: the raster window of a box in the raster's CRS and its place in a tile
/// of querySize px (querySize 0: native resolution). nullopt where the baseline throws (pixel
/// size smaller than DBL_EPSILON) and where a coordinate of the window is NaN or beyond int (the
/// baseline's cast is undefined there).
std::optional<GQResult> GeoQuery(const double gt[6], int rasterXSize, int rasterYSize, double ulx, double uly,
                                 double lrx, double lry, int querySize) noexcept;

/// A raster as GDALTiler sees it after open(): the dataset the tiles read (the file in
/// EPSG:3857, or the warped VRT) and the bounds and zooms derived from it.
struct RasterTiling {
    GlobalMercator mercator;
    double gt[6];
    int rasterXSize, rasterYSize;
    RasterBounds bounds;
    ZoomRange zooms;
    RasterTiling(int tileSize, const double geoTransform[6], int xsize, int ysize);
};

/// GDALTiler::tileWindow for a TMS row (InternalRow): false when the tile is outside
/// MinMaxCoordsForZ or the window is empty after rounding (the baseline emits a transparent
/// tile); g is the window otherwise.
bool TileWindow(const RasterTiling& t, int tz, int tx, int tmsRow, GQResult& g) noexcept;

/// PointCloudTiler: zoom range from the EPSG:3857 footprint and the COPC span.
ZoomRange PointCloudZoomRange(const GlobalMercator& m, double oMinX, double oMinY, double oMaxX, double oMaxY,
                              int span) noexcept;

/// PointCloudTiler::tile: the buffer (metres) added on each side of the tile before the query:
/// int(resolution(tz) * ceil(float(tileSize) / 40.0)), truncated (0 from z21 at 256 px).
int PointCloudBoundsBuffer(const GlobalMercator& m, int tz) noexcept;

/// PointCloudTiler::tile: the PixelMapping of the padded z-buffer (pointRadius 2 px).
struct PixelMapping {
    double originX = 0, shiftX = 0, scaleX = 0;
    double originY = 0, shiftY = 0, scaleY = 0;
    int width = 0, height = 0;
};
inline constexpr int kPointRadius = 2;
PixelMapping PointCloudPixelMapping(const GlobalMercator& m, const Box<Point>& tileBounds, int tz) noexcept;

}  // namespace ddbt::tile::compat
