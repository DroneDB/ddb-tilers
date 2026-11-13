// SPDX-License-Identifier: MPL-2.0
#pragma once

// Web map tile maths on the WebMercatorQuad tile matrix set (EPSG:3857, the Google / OSM / Bing
// grid): resolutions, tile bounds, point and bounds to tile, lon/lat to tile, XYZ and TMS rows,
// 256 and 512 px tiles. The constants and formulas are those of gdal2tiles' GlobalMercator
// (which DroneDB's GlobalMercator copies), so a tile here covers exactly the same metres as a
// tile of the baseline. Faithful copies of the baseline functions, including their integer
// truncations, are in core/tile/compat.h.

#include <compare>
#include <cstddef>
#include <cstdint>
#include <numbers>

namespace ddbt::tile {

/// Equatorial radius of the WGS 84 sphere used by EPSG:3857 (metres).
inline constexpr double kEarthRadius = 6378137.0;
/// Half the world width in EPSG:3857 (20037508.342789244 m), as DroneDB and gdal2tiles compute it.
inline constexpr double kOriginShift = 2.0 * std::numbers::pi * 6378137.0 / 2.0;
/// Latitude where EPSG:3857 y reaches kOriginShift (the square world), degrees.
inline constexpr double kMaxLatitude = 85.0511287798066;
/// Deepest zoom level of the packed TileKey (x and y fit 29 bits).
inline constexpr int kMaxZoom = 29;

/// Row convention. XYZ ("slippy map", Google, OSM): row 0 is the northernmost row. TMS: row 0
/// is the southernmost row (gdal2tiles' and GlobalMercator's internal convention).
enum class Scheme : uint8_t { XYZ, TMS };

/// Converts a row between XYZ and TMS at zoom z (the conversion is its own inverse).
constexpr uint32_t FlipRow(uint32_t y, int z) noexcept { return ((uint32_t{1} << z) - 1u) - y; }

/// A tile, packed in 64 bits: z (5 bits) | x (29 bits) | y (29 bits), rows always XYZ. Ordering
/// sorts by zoom, then column, then row. Valid for z <= kMaxZoom.
class TileKey {
public:
    constexpr TileKey() noexcept = default;
    constexpr TileKey(int z, uint32_t x, uint32_t y) noexcept
        : v_((static_cast<uint64_t>(z) << 58) | (static_cast<uint64_t>(x) << 29) | static_cast<uint64_t>(y)) {}
    /// From a row in the given scheme.
    static constexpr TileKey From(int z, uint32_t x, uint32_t y, Scheme s) noexcept {
        return TileKey(z, x, s == Scheme::XYZ ? y : FlipRow(y, z));
    }
    static constexpr TileKey FromPacked(uint64_t v) noexcept {
        TileKey k;
        k.v_ = v;
        return k;
    }

    constexpr int z() const noexcept { return static_cast<int>(v_ >> 58); }
    constexpr uint32_t x() const noexcept { return static_cast<uint32_t>((v_ >> 29) & kMask29); }
    /// Row, XYZ (north origin).
    constexpr uint32_t y() const noexcept { return static_cast<uint32_t>(v_ & kMask29); }
    /// Row in the given scheme.
    constexpr uint32_t Row(Scheme s) const noexcept { return s == Scheme::XYZ ? y() : FlipRow(y(), z()); }
    constexpr uint64_t packed() const noexcept { return v_; }

    /// True when z <= kMaxZoom and x, y < 2^z.
    constexpr bool Valid() const noexcept {
        return z() <= kMaxZoom && x() < (uint32_t{1} << z()) && y() < (uint32_t{1} << z());
    }
    constexpr TileKey Parent() const noexcept { return TileKey(z() - 1, x() >> 1, y() >> 1); }
    /// Child i in 0..3: bit 0 = east half, bit 1 = south half.
    constexpr TileKey Child(int i) const noexcept {
        return TileKey(z() + 1, (x() << 1) | static_cast<uint32_t>(i & 1), (y() << 1) | static_cast<uint32_t>((i >> 1) & 1));
    }

    friend constexpr bool operator==(TileKey a, TileKey b) noexcept { return a.v_ == b.v_; }
    friend constexpr auto operator<=>(TileKey a, TileKey b) noexcept { return a.v_ <=> b.v_; }

private:
    static constexpr uint64_t kMask29 = (uint64_t{1} << 29) - 1;
    uint64_t v_ = 0;
};

/// Hash for unordered containers (splitmix64 finaliser of the packed value).
struct TileKeyHash {
    size_t operator()(TileKey k) const noexcept {
        uint64_t x = k.packed() + 0x9E3779B97F4A7C15ull;
        x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
        x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
        return static_cast<size_t>(x ^ (x >> 31));
    }
};

/// Axis-aligned box (EPSG:3857 metres or degrees, depending on the function).
struct Bounds {
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    friend bool operator==(const Bounds&, const Bounds&) = default;
};

/// Inclusive range of tiles of one zoom level; rows in the scheme it was requested in. Empty
/// when minX > maxX or minY > maxY.
struct TileRange {
    int z = 0;
    int64_t minX = 0, minY = 0, maxX = -1, maxY = -1;
    bool Empty() const noexcept { return minX > maxX || minY > maxY; }
    int64_t Count() const noexcept { return Empty() ? 0 : (maxX - minX + 1) * (maxY - minY + 1); }
    bool Contains(int64_t x, int64_t y) const noexcept { return x >= minX && x <= maxX && y >= minY && y <= maxY; }
    friend bool operator==(const TileRange&, const TileRange&) = default;
};

/// Rounding of a pixel size to a zoom level.
enum class ZoomRounding : uint8_t {
    Up,       ///< finest needed: the first zoom whose resolution is <= the pixel size (GDAL COG "UPPER")
    Down,     ///< coarsest: the last zoom whose resolution is >= the pixel size (GDAL COG "LOWER")
    Nearest,  ///< the closer of the two in log2 (GDAL COG "AUTO")
};

/// The WebMercatorQuad tile matrix set for one tile size (any power of two from 1 to 4096; the
/// web uses 256 and 512). A 512 px tile at zoom z covers the same metres as a 256 px tile at z
/// and has the resolution of a 256 px tile at z + 1: EffectiveZoom() gives that z + log2(T/256).
class WebMercatorQuad {
public:
    /// Throws std::invalid_argument if tileSize is not a power of two in [1, 4096].
    explicit WebMercatorQuad(int tileSize = 256);

    int tileSize() const noexcept { return tileSize_; }
    /// log2(tileSize / 256): 0 for 256 px, 1 for 512 px, -1 for 128 px.
    int sizeShift() const noexcept { return sizeShift_; }
    /// z + log2(T/256): the zoom of a 256 px tile with the resolution of a T px tile at z.
    int EffectiveZoom(int z) const noexcept { return z + sizeShift_; }

    /// Metres per pixel at zoom z (at the equator): 2*pi*R / (T * 2^z). Same bits as
    /// GlobalMercator::resolution for every z.
    double Resolution(int z) const noexcept;
    /// Width of a tile at zoom z in metres: 2 * kOriginShift / 2^z (independent of the tile size).
    static double TileSpan(int z) noexcept;

    /// Zoom level for a pixel size in metres, with the given rounding, clamped to [0, maxZoom].
    int ZoomForResolution(double metresPerPixel, ZoomRounding rounding, int maxZoom = 30) const noexcept;

    /// EPSG:3857 bounds of a tile (row in `scheme`). Computed as GlobalMercator::tileBounds does.
    Bounds TileBounds(int z, int64_t x, int64_t y, Scheme scheme) const noexcept;
    Bounds TileBounds(TileKey k) const noexcept { return TileBounds(k.z(), k.x(), k.y(), Scheme::XYZ); }
    /// Lon/lat bounds of a tile (degrees, minX = west, minY = south).
    Bounds TileLonLatBounds(int z, int64_t x, int64_t y, Scheme scheme) const noexcept;

    /// Tile containing an EPSG:3857 point, clamped to the world (a point on a tile edge belongs to
    /// the tile east / south (XYZ) of it, i.e. floor semantics).
    TileKey MetersToTile(double mx, double my, int z) const noexcept;
    /// Tile containing a lon/lat point (degrees), mercantile's tile(): latitudes beyond
    /// +-kMaxLatitude clamp to the first / last row, longitudes outside [-180, 180) clamp.
    static TileKey LonLatToTile(double lon, double lat, int z) noexcept;

    /// Tiles intersecting an EPSG:3857 box with positive area (a box that ends exactly on a tile
    /// edge does not include the next tile), clamped to the world. Rows in `scheme`.
    TileRange TilesForBounds(const Bounds& meters, int z, Scheme scheme) const noexcept;
    /// Same for a lon/lat box (degrees; latitudes are clamped to +-kMaxLatitude).
    TileRange TilesForLonLatBounds(const Bounds& lonLat, int z, Scheme scheme) const noexcept;

private:
    int tileSize_;
    int sizeShift_;
    double initialResolution_;
};

/// Spherical Mercator, scalar, gdal2tiles / mercantile formulas (not PROJ's): use core/crs for
/// transforms that must match PROJ or GDAL.
double LonToMeters(double lon) noexcept;
double LatToMeters(double lat) noexcept;
double MetersToLon(double mx) noexcept;
double MetersToLat(double my) noexcept;

}  // namespace ddbt::tile
