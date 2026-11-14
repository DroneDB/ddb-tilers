/* ddb-tilers C API.
 *
 * Shaped after DroneDB's libddb exports (DDBMemoryTileFmt, DDBVSIFree, DDBBuild) so that the
 * bench driver can load either backend through dlopen() and so that the later libddb
 * integration is a thin shim.
 *
 * Error handling: every function returns a ddbtErr. The message of the last failed call on the
 * calling thread is available through ddbtLastError() (never NULL, "" when the last call
 * succeeded). Outputs are written only on DDBT_ERR_NONE; on any other code an output pointer
 * is left untouched.
 *
 * SPDX-License-Identifier: MPL-2.0 */
#ifndef DDBT_H
#define DDBT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#if defined(DDBT_BUILDING_SHARED)
#define DDBT_API __declspec(dllexport)
#elif defined(DDBT_SHARED)
#define DDBT_API __declspec(dllimport)
#else
#define DDBT_API
#endif
#elif defined(__GNUC__) || defined(__clang__)
#define DDBT_API __attribute__((visibility("default")))
#else
#define DDBT_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Error codes. The values 0..9 that exist in libddb's DDBErr have the same value here, so a
 * caller can treat the two enums alike; codes from 100 are specific to ddbt. */
typedef enum ddbtErr
{
    DDBT_ERR_NONE = 0,             /* no error */
    DDBT_ERR_EXCEPTION = 1,        /* generic failure (message in ddbtLastError) */
    DDBT_ERR_CANCELED = 4,         /* canceled by the caller */
    DDBT_ERR_NOTFOUND = 6,         /* the input does not exist */
    DDBT_ERR_INVALIDARGS = 7,      /* an argument is invalid: fix the call, do not retry */
    DDBT_ERR_REFUSED = 8,          /* a valid argument the input cannot honour (a formula that does not
                                      exist or that the raster cannot compute): the same call never
                                      succeeds (libddb's RefusedArgumentException) */
    DDBT_ERR_NO_SPACE = 9,         /* the device is full: the same call may succeed once there is room */
    DDBT_ERR_NOT_IMPLEMENTED = 100 /* the feature is not implemented (yet) */
} ddbtErr;

/** Library version, "major.minor.patch": never NULL, static storage. */
DDBT_API const char *ddbtVersion(void);

/** Message of the last failed call on this thread: never NULL, "" when the last call
 * succeeded. Owned by the library, valid until the next ddbt call on the same thread. The
 * accessors ddbtVersion, ddbtLastError and ddbtFree do not touch it. */
DDBT_API const char *ddbtLastError(void);

/** Generate a tile in memory. Same arguments and semantics as DroneDB 2.4.1's DDBMemoryTileFmt.
 * PNG tiles are written at zlib level 1, WebP tiles lossy at quality 80, JPEG at quality 75;
 * tiles without transparent pixels have no alpha band.
 * Inputs: a GeoTIFF / COG is tiled in place (directly when it is in EPSG:3857, else on the grid of
 * the EPSG:3857 COG DroneDB would build for it); a COPC point cloud (a name ending in ".copc.laz")
 * is tiled by the point-cloud tiler. Other rasters need a conversion to a COG first, which ddbt
 * does not do yet: DDBT_ERR_NOT_IMPLEMENTED (so do remote URLs, but a COPC at an http(s) URL with
 * a remote reader: ddbtSetRemoteReader).
 * Codes: DDBT_ERR_INVALIDARGS for a bad argument (no path, NULL outputs, negative coordinates or
 * tile size, a tile size that is not a power of two, an unknown format), DDBT_ERR_NOTFOUND when
 * the input does not exist, DDBT_ERR_EXCEPTION otherwise (libddb 2.4.1's DDBMemoryTileFmt, a 1.x
 * export, folds the first two into DDBERR_EXCEPTION; the messages are the same).
 * @param inputPath path of the input raster (GeoTIFF / COG)
 * @param tz,tx,ty tile coordinates; ty is an XYZ row (north = 0) when tms is true, a TMS row
 *        (south = 0) when tms is false: libddb 2.4.1's inverted flag, kept
 * @param outBuffer receives the encoded tile; release it with ddbtFree
 * @param outBufferSize receives the size in bytes of *outBuffer
 * @param tileSize tile size in pixels (a power of two: 256 or 512 on the web)
 * @param tms see ty
 * @param forceRecreate bypass the encoded-tile cache
 * @param inputPathHash optional hash of the resource (may be NULL; unused for local files)
 * @param outputFormat "png", "jpeg"/"jpg" or "webp" (or their MIME types), NULL or "" for PNG
 * @return DDBT_ERR_NONE on success, an error otherwise */
DDBT_API ddbtErr ddbtMemoryTileFmt(const char *inputPath, int tz, int tx, int ty, uint8_t **outBuffer,
                                   int *outBufferSize, int tileSize, bool tms, bool forceRecreate,
                                   const char *inputPathHash, const char *outputFormat);

/** Options of ddbtMemoryTileEx2: the same layout and rules as libddb's DDBRasterOptions. Set
 * structSize to sizeof(ddbtRasterOptions) (at least offsetof(ddbtRasterOptions, formula)) and
 * every other field explicitly; bytes beyond the known struct must be zero. */
typedef struct ddbtRasterOptions
{
    uint32_t structSize; /* sizeof(ddbtRasterOptions) */
    uint32_t flags;      /* DDBT_RASTER_OPT_* bits */
    double nodata;       /* with DDBT_RASTER_OPT_NODATA: a finite NoData value replacing every band's */
    double bbox[4];      /* with DDBT_RASTER_OPT_BBOX: checked (finite), not used by tiles */
    const char *formula; /* not used by tiles (the formula is an argument) */
    const char *rescaleUnit; /* with a thermal formula: the unit of rescale ("K", "C", "F", ...) */
} ddbtRasterOptions;

#define DDBT_RASTER_OPT_NODATA 0x1u
#define DDBT_RASTER_OPT_BBOX 0x2u

/** A tile with visualisation parameters: the arguments and semantics of DroneDB 2.4.1's
 * DDBMemoryTileEx2 (always PNG). A tile out of the raster's bounds fails "Out of bounds" unless
 * no parameter is set (then it is the plain, transparent tile). A colormap without a formula
 * paints a raster of one data band, or one band chosen by bands / preset, over the rescale or
 * the band's exact minimum to maximum. A formula is painted with its colormap (rdylgn by
 * default) over the rescale, its own range, else the 2nd to 98th percentile of the tile.
 * Codes: DDBT_ERR_INVALIDARGS for bad arguments, DDBT_ERR_REFUSED for a formula that does not
 * exist or that the raster cannot compute ("Unknown formula: X", "Requires bands: RN"), DDBT_ERR_NOTFOUND,
 * DDBT_ERR_NOT_IMPLEMENTED (as ddbtMemoryTileFmt), DDBT_ERR_EXCEPTION otherwise.
 * @param preset, bands, formula, bandFilter, colormap, rescale as DDBMemoryTileEx2 (each may be NULL)
 * @param options NULL, or the NoData override (and the unit of a thermal rescale)
 * @param outBuffer, outBufferSize the PNG; release it with ddbtFree */
DDBT_API ddbtErr ddbtMemoryTileEx2(const char *inputPath, int tz, int tx, int ty, int tileSize, bool tms,
                                   bool forceRecreate, const char *inputPathHash, const char *preset,
                                   const char *bands, const char *formula, const char *bandFilter,
                                   const char *colormap, const char *rescale, const ddbtRasterOptions *options,
                                   uint8_t **outBuffer, int *outBufferSize);

/** ddbtMemoryTileEx2 in a chosen format: the arguments and semantics of libddb 2.5's DDBMemoryTileEx3. The
 * visualisation tile is encoded once in outputFormat ("png", "jpeg"/"jpg" or "webp", or their MIME types;
 * NULL or "" for PNG, the bytes of ddbtMemoryTileEx2), so a host that serves WebP does not decode a PNG to
 * encode it again. An unsupported format is DDBT_ERR_INVALIDARGS, checked after the options. */
DDBT_API ddbtErr ddbtMemoryTileEx3(const char *inputPath, int tz, int tx, int ty, int tileSize, bool tms,
                                   bool forceRecreate, const char *inputPathHash, const char *preset,
                                   const char *bands, const char *formula, const char *bandFilter,
                                   const char *colormap, const char *rescale, const ddbtRasterOptions *options,
                                   const char *outputFormat, uint8_t **outBuffer, int *outBufferSize);

/** A thumbnail of a COPC point cloud: the whole cloud fitted, centred, in a square of size x size
 * pixels, as RGBA (8 bits a channel, straight alpha, transparent background; size * size * 4
 * bytes), drawn as the modern tiles are (a disc on each point with a per-pixel depth test; points
 * in their colours, a cloud without colours in the ramp over its height range). In EPSG:3857, north
 * up, when the cloud has a footprint, else in its own x, y (no CRS, or none that transforms to
 * EPSG:3857). It reads the deepest octree levels whose points, with those of every shallower level,
 * fit pointBudget (0: 2 per pixel), at least the root. A COPC at an http(s) URL is read through the
 * remote reader (ddbtSetRemoteReader).
 * Codes: DDBT_ERR_INVALIDARGS for a bad argument (no path, NULL outputs, a size outside 1..16384)
 * or a file that is not a COPC, DDBT_ERR_NOTFOUND, DDBT_ERR_NOT_IMPLEMENTED (a remote path without
 * a reader), DDBT_ERR_EXCEPTION otherwise (a cloud of zero extent or without points).
 * @param outRgba receives the pixels; release them with ddbtFree
 * @param outRgbaSize receives size * size * 4 */
DDBT_API ddbtErr ddbtRenderCopcThumbnail(const char *inputPath, int size, uint64_t pointBudget, uint8_t **outRgba,
                                         int *outRgbaSize);

/** Release a buffer or string returned by ddbt (counterpart of DDBVSIFree and DDBFree).
 * NULL is a no-op. */
DDBT_API ddbtErr ddbtFree(void *ptr);

/** Progress of a build: the fraction done (0..1, never decreasing) and the phase name.
 * Called while the build runs, possibly from another thread than the caller's, never
 * concurrently with itself. It must return quickly and must not call back into ddbt. */
typedef void (*ddbtProgressCallback)(double fraction, const char *phase, void *userData);

/** Build a Cloud Optimized GeoTIFF.
 * The input is a GeoTIFF, a JPEG 2000 file, or a photo (JPEG, PNG, GIF, WebP, TIFF or DNG without
 * georeferencing) georeferenced by the "gcps" option: [[pixel, line, x, y], ...] in "gcpSrs"
 * ("EPSG:4326" by default); "targetSrs": "source" then writes the photo in that CRS (gdalwarp's
 * grid, nearest neighbour) instead of the EPSG:3857 COG, "outScale" resamples the photo first.
 * The other options are listed in src/api/build_api.cpp.
 * @param inputPath source raster
 * @param outputPath destination file
 * @param optionsJson JSON object with options, or NULL / "" for the defaults
 * @return DDBT_ERR_NONE on success, an error otherwise (DDBT_ERR_NOT_IMPLEMENTED for an input
 *         ddbt does not handle) */
DDBT_API ddbtErr ddbtBuildCog(const char *inputPath, const char *outputPath, const char *optionsJson);

/** ddbtBuildCog with progress: phases "warping", "overviews", "compressing", "statistics"
 * (DroneDB's names; they follow the fraction, the single pass does them together).
 * @param progress NULL for none
 * @param userData passed to progress */
DDBT_API ddbtErr ddbtBuildCog2(const char *inputPath, const char *outputPath, const char *optionsJson,
                               ddbtProgressCallback progress, void *userData);

/** Build a Cloud Optimized Point Cloud (COPC).
 * @param inputPath source point cloud
 * @param outputPath destination file
 * @param optionsJson JSON object with options, or NULL / "" for the defaults
 * @return DDBT_ERR_NONE on success, an error otherwise */
DDBT_API ddbtErr ddbtBuildCopc(const char *inputPath, const char *outputPath, const char *optionsJson);

/** ddbtBuildCopc with progress: phases "planning", "reading", "building", "writing".
 * @param progress NULL for none
 * @param userData passed to progress */
DDBT_API ddbtErr ddbtBuildCopc2(const char *inputPath, const char *outputPath, const char *optionsJson,
                                ddbtProgressCallback progress, void *userData);

/** Process-wide settings of a host that embeds ddbt, as a JSON object; every member optional,
 * the others keep their value. Checked whole before anything changes.
 *   "projDataDir": folder of proj.db for the transforms created from now on ("" = the default
 *                  search: DDBT_PROJ_DATA, the build's PROJ data, then PROJ's own, PROJ_DATA)
 *   "threads": workers of the shared pool (0 = one per CPU); only before the first build or tile
 *              (DDBT_ERR_EXCEPTION afterwards, nothing changed)
 *   "blockCacheMb", "tileCacheMb": decoded-block and encoded-tile caches of the raster tiler
 *   "copcNodeCacheMb", "copcTileCacheMb": decoded-node and encoded-tile caches of the point-cloud
 *                  tiler (MiB, 0 = no cache)
 *   "copcTilerMode": "compat" (DroneDB 2.4.1's tiles), "hybrid" or "modern"
 *   "copcBuildMemMb": memory cap of COPC builds that do not give one (0 = the default:
 *                  DDBT_COPC_BUILD_MEM_MB, else 4 GiB, at most half the memory)
 * These settings win over the DDBT_* environment variables.
 * @return DDBT_ERR_INVALIDARGS for a malformed or unknown setting */
DDBT_API ddbtErr ddbtConfigure(const char *configJson);

/** The bounds and zoom range of a tile input, as DroneDB's tilers compute them when they open
 * it: {"type": "raster" | "pointcloud", "minX", "minY", "maxX", "maxY" (EPSG:3857 metres),
 * "minZoom", "maxZoom"}. A COPC file (a name ending in ".copc.laz") is a point cloud, in the
 * process-wide tile mode; anything else a raster.
 * @param tileSize tile size in pixels (a power of two)
 * @param outJson receives a NUL-terminated string; release it with ddbtFree
 * @return codes as ddbtMemoryTileFmt */
DDBT_API ddbtErr ddbtTileInfo(const char *inputPath, int tileSize, char **outJson);

/** Closes the files the tilers keep open between calls (the shared sources of rasters and point
 * clouds), so that the host can delete or replace them on any platform and release its own file
 * handles at once (DDBReleaseConnections). A file in use by a call that is running closes when that
 * call returns; the next tile of a file opens it again. The decoded-block, node and encoded-tile
 * caches are kept: their entries belong to a file version and stay valid. */
DDBT_API ddbtErr ddbtReleaseSources(void);

/** Range reads of remote inputs, done by the host (with its HTTP client, proxy, certificates and
 * credentials): the point-cloud tiler (ddbtMemoryTileFmt, ddbtTileInfo) reads a COPC at an
 * http:// or https:// URL through them. Without a reader such a URL is DDBT_ERR_NOT_IMPLEMENTED.
 * Both callbacks may run concurrently on several threads, also after ddbtSetRemoteReader replaced
 * the reader for the calls that started before. They return DDBT_ERR_NONE, DDBT_ERR_NOTFOUND when
 * the object does not exist, or another code; on an error they may write a NUL-terminated reason
 * (truncated to errorSize bytes) into error.
 *   stat: the size of the object and its version (an ETag or Last-Modified value, "" when the
 *         server gives none; NUL-terminated, at most versionSize bytes with the NUL). Called once
 *         per tile: a new size or version opens the object again.
 *   read: exactly size bytes at offset into buffer. */
typedef struct ddbtRemoteReader
{
    uint32_t structSize; /* sizeof(ddbtRemoteReader) */
    ddbtErr (*stat)(const char *url, uint64_t *outSize, char *version, size_t versionSize, char *error,
                    size_t errorSize, void *userData);
    ddbtErr (*read)(const char *url, uint64_t offset, uint8_t *buffer, size_t size, char *error, size_t errorSize,
                    void *userData);
    void *userData;
} ddbtRemoteReader;

/** Sets the remote reader of the process (its fields are copied), or removes it (NULL).
 * @return DDBT_ERR_INVALIDARGS when structSize is too small or a callback is NULL */
DDBT_API ddbtErr ddbtSetRemoteReader(const ddbtRemoteReader *reader);

/** Information about the build and the CPU dispatch, as a JSON object (Highway targets
 * compiled in, supported by this CPU, and the one selected at runtime).
 * @param outJson receives a NUL-terminated string; release it with ddbtFree */
DDBT_API ddbtErr ddbtCpuInfo(char **outJson);

#ifdef __cplusplus
}
#endif

#endif /* DDBT_H */
