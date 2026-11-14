// SPDX-License-Identifier: MPL-2.0
//
// TiffReader: classic TIFF and BigTIFF (both byte orders), the IFD chain and SubIFDs, every
// field type, GDAL's ghost header, and block (tile or strip) reads and decodes.
//
//   auto reader = TiffReader::Open(path);                 // Expected<std::shared_ptr<...>>
//   const IfdInfo& img = (*reader)->Ifd((*reader)->MainImage());
//   DecodedBlock b = *(*reader)->ReadBlock(img.index, img.BlockIndex(col, row, plane));
//
// Open() parses the header and every IFD (their small tags are read eagerly); the block index
// (TileOffsets / TileByteCounts or StripOffsets / StripByteCounts) of an IFD is read on first
// use, once (std::call_once): COG overviews nobody asks for are never loaded. When the source
// is memory-mapped, single entries are read straight from the mapping instead.
//
// A reader is immutable after Open() and safe to share between any number of threads. It does
// no caching of blocks (the tiler owns the decoded-block cache).
//
// Defensive parsing: every count, offset and size from the file goes through checked math and
// is checked against the file size and the limits in TiffReaderOptions, so a malformed file
// gives a Corrupt / Unsupported / Overflow status, never an out-of-bounds access or an
// allocation larger than the limits.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "cog/tiff/libtiff_codec.h"
#include "cog/tiff/tiff_tags.h"
#include "core/io/byte_source.h"
#include "core/mem/aligned.h"
#include "core/mem/buffer_pool.h"
#include "core/util/status.h"

namespace ddbt::cog::tiff {

/// Type of the decoded samples.
enum class SampleType : uint8_t {
    kUnknown,
    kBit1,  // 1-bit (masks, bilevel): rows of packed bits, most significant bit first
    kU8,
    kI8,
    kU16,
    kI16,
    kU32,
    kI32,
    kU64,
    kI64,
    kF16,  // IEEE half (raw bits)
    kF32,
    kF64,
};

std::string_view SampleTypeName(SampleType t) noexcept;
/// Bytes per sample (0 for kBit1 / kUnknown).
size_t SampleTypeSize(SampleType t) noexcept;

/// What an IFD is, from NewSubfileType (bit 0: reduced resolution, bit 2: transparency mask).
enum class SubfileKind : uint8_t {
    kFullResolution,  // the main image (the first full-resolution IFD)
    kOverview,        // reduced-resolution version of the main image
    kMask,            // transparency mask of the full-resolution image
    kOverviewMask,    // transparency mask of an overview
    kPage,            // another full-resolution image (multi-page TIFF), or other flags
};

std::string_view SubfileKindName(SubfileKind k) noexcept;

/// One directory entry, as stored. Values are read through TiffReader::Read*Tag.
struct TagEntry {
    uint16_t tag = 0;
    uint16_t type = 0;
    uint64_t count = 0;
    uint64_t valueOffset = 0;      // file offset of the values when !isInline
    bool isInline = false;         // values fit in the entry (<= 4 bytes classic, <= 8 BigTIFF)
    std::array<uint8_t, 8> inlineBytes{};
    /// Size of the values in bytes (count * type size; 0 for unknown types or on overflow).
    uint64_t ByteSize() const noexcept;
};

/// GDAL's ghost header ("GDAL_STRUCTURAL_METADATA_SIZE=... bytes" after the TIFF header).
struct GhostHeader {
    bool present = false;
    std::string text;  // the key=value lines
    bool layoutIfdsBeforeData = false;              // LAYOUT=IFDS_BEFORE_DATA
    bool blockOrderRowMajor = false;                // BLOCK_ORDER=ROW_MAJOR
    bool blockLeaderSizeAsUint4 = false;            // BLOCK_LEADER=SIZE_AS_UINT4
    bool blockTrailerLast4BytesRepeated = false;    // BLOCK_TRAILER=LAST_4_BYTES_REPEATED
    bool knownIncompatibleEdition = false;          // KNOWN_INCOMPATIBLE_EDITION=YES
    bool maskInterleavedWithImagery = false;        // MASK_INTERLEAVED_WITH_IMAGERY=YES
};

/// Everything about one IFD that the readers need, parsed at open.
struct IfdInfo {
    uint32_t index = 0;        // position in TiffReader::Ifds()
    int32_t parent = -1;       // a SubIFD: index of the IFD that lists it, else -1
    uint64_t offset = 0;       // file offset of the directory
    uint32_t newSubfileType = 0;
    SubfileKind kind = SubfileKind::kFullResolution;

    uint32_t width = 0;
    uint32_t height = 0;
    bool tiled = false;
    uint32_t blockWidth = 0;   // tile width, or the image width for strips
    uint32_t blockHeight = 0;  // tile length, or RowsPerStrip clamped to the image height
    uint32_t blocksAcross = 0;
    uint32_t blocksDown = 0;
    uint32_t planes = 1;       // samplesPerPixel for BAND interleave, else 1
    uint32_t blockCount = 0;   // blocksAcross * blocksDown * planes

    uint16_t samplesPerPixel = 1;
    uint16_t bitsPerSample = 1;
    uint16_t sampleFormat = 1;  // 1 uint, 2 int, 3 IEEE float, 4 void, 5/6 complex
    SampleType sampleType = SampleType::kUnknown;  // of each component of a complex sample
    bool complex = false;  // SampleFormat 5 / 6: every sample is two components of sampleType
    // Read through libtiff's RGBA interface, as GDAL reads it (CMYK, CIELab, LogL / LogLuv, YCbCr
    // without JPEG, old JPEG): the fields above then describe the decoded 8-bit RGB / RGBA
    // pixels, and fileSamplesPerPixel the stored samples
    bool viaRgba = false;
    uint16_t fileSamplesPerPixel = 0;
    uint16_t planarConfig = 1;  // 1 PIXEL (contig), 2 BAND (separate)
    uint16_t photometric = photometric::kMinIsBlack;
    bool hasPhotometric = false;
    uint16_t compression = 1;
    uint16_t predictor = 1;
    uint16_t fillOrder = 1;
    std::vector<uint16_t> extraSamples;
    std::array<uint16_t, 2> ycbcrSubsampling{2, 2};
    uint16_t ycbcrPositioning = 1;
    std::vector<uint16_t> colorMap;  // 3 * 2^bitsPerSample entries (R, G, B), when present
    std::vector<uint8_t> jpegTables;
    std::array<uint32_t, 2> lercParameters{0, 0};  // version, additional compression
    bool hasLercParameters = false;

    std::optional<double> noData;  // GDAL_NODATA parsed as a double
    std::string noDataText;        // GDAL_NODATA as stored (exact for 64-bit integers)
    std::string gdalMetadata;      // GDAL_METADATA XML, as stored
    std::string imageDescription;
    std::string software;

    /// Every entry of the directory, sorted by tag (duplicates dropped, the first one kept).
    std::vector<TagEntry> tags;

    // Block layout helpers -------------------------------------------------------------------

    bool IsPixelInterleaved() const noexcept { return planarConfig != 2; }
    /// Components of a sample: 2 for complex samples (real, imaginary), else 1.
    uint32_t Components() const noexcept { return complex ? 2u : 1u; }
    /// 24-bit floats (FP24), which decode to Float32.
    bool IsFloat24() const noexcept { return sampleFormat == 3 && bitsPerSample == 24; }
    /// Values per pixel inside one block (spp for PIXEL, 1 for BAND; twice that for complex
    /// samples, real and imaginary interleaved).
    uint32_t SamplesPerBlockPixel() const noexcept { return (IsPixelInterleaved() ? samplesPerPixel : 1u) * Components(); }
    /// Block index of (column, row, plane) as TIFF orders them (plane-major).
    uint32_t BlockIndex(uint32_t col, uint32_t row, uint32_t plane = 0) const noexcept {
        return (plane * blocksDown + row) * blocksAcross + col;
    }
    uint32_t BlockColumn(uint32_t block) const noexcept { return block % blocksAcross; }
    uint32_t BlockRow(uint32_t block) const noexcept { return (block / blocksAcross) % blocksDown; }
    uint32_t BlockPlane(uint32_t block) const noexcept { return block / (blocksAcross * blocksDown); }
    /// Rows a block holds: blockHeight for tiles; the last strip may hold fewer.
    uint32_t BlockRows(uint32_t block) const noexcept;
    /// Samples stored with another width than their type's (NBITS: 2 to 31 bits, packed), which
    /// decode to the unsigned type that holds them.
    bool IsPacked() const noexcept;
    /// Bytes per decoded row of a block (packed bits rounded up to bytes for 1-bit data).
    uint64_t BlockRowBytes() const noexcept;
    /// Bytes per stored row of a block, before packed samples are unpacked.
    uint64_t StoredRowBytes() const noexcept;
    /// Decoded size of block `index` (BlockRows * BlockRowBytes, before any JPEG YCbCr->RGB,
    /// which keeps 3 samples). 0 on overflow.
    uint64_t DecodedBlockBytes(uint32_t block) const noexcept;
    const TagEntry* FindTag(uint16_t tag) const noexcept;
    /// The last ExtraSamples value is unassociated alpha (GDAL's alpha band).
    bool HasAlpha() const noexcept;
};

/// File position and size of one block.
struct BlockLocation {
    uint64_t offset = 0;
    uint64_t size = 0;
    /// No data stored (offset 0 or size 0): a sparse block, filled with nodata or 0.
    bool IsSparse() const noexcept { return offset == 0 || size == 0; }
};

/// Owning, 64-byte aligned buffer: from a BufferPool when the reader has one, else a plain
/// aligned allocation.
class BlockBuffer {
public:
    BlockBuffer() = default;
    static BlockBuffer Allocate(size_t size, mem::BufferPool* pool);
    uint8_t* data() const noexcept { return data_; }
    size_t size() const noexcept { return size_; }
    std::span<uint8_t> span() const noexcept { return {data_, size_}; }
    bool empty() const noexcept { return data_ == nullptr; }

private:
    mem::PooledBuffer pooled_;
    mem::AlignedBytes owned_;
    uint8_t* data_ = nullptr;
    size_t size_ = 0;
};

/// A decoded block: host byte order, predictor undone, native layout (pixel-interleaved
/// samples for PIXEL, one band for BAND); JPEG YCbCr blocks are converted to RGB.
struct DecodedBlock {
    BlockBuffer buffer;
    uint32_t width = 0;   // pixels per row (the tile width, or the image width for strips)
    uint32_t height = 0;  // rows (tile length; the last strip may be shorter)
    uint32_t samples = 1; // samples per pixel in this buffer
    uint32_t bitsPerSample = 8;
    SampleType sampleType = SampleType::kU8;
    size_t rowStride = 0;  // bytes per row
    bool sparse = false;   // no data in the file: filled with the nodata value or 0
    uint8_t* data() const noexcept { return buffer.data(); }
    size_t size() const noexcept { return buffer.size(); }
};

/// Compressed bytes of a block: a view into the mapping when the source is memory-resident,
/// else a copy owned by `owned`.
struct RawBlock {
    BlockLocation location;
    std::span<const uint8_t> bytes;
    std::vector<uint8_t> owned;
    bool IsSparse() const noexcept { return location.IsSparse(); }
};

struct TiffReaderOptions {
    /// Directories followed along the IFD chain and SubIFD trees.
    uint32_t maxIfds = 65536;
    /// Entries in one directory.
    uint32_t maxEntriesPerIfd = 4096;
    /// Bytes of one tag read eagerly at open (ColorMap, GDAL_METADATA, GeoKeys, ...).
    uint64_t maxTagBytes = uint64_t{64} << 20;
    /// Blocks of one IFD (TileOffsets / StripOffsets entries).
    uint64_t maxBlocksPerIfd = uint64_t{1} << 28;
    /// Decoded size of one block (ReadBlock refuses larger blocks).
    uint64_t maxBlockBytes = uint64_t{1} << 30;
    /// Compressed size of one block.
    uint64_t maxCompressedBlockBytes = uint64_t{1} << 30;
    /// Check BLOCK_LEADER / BLOCK_TRAILER of GDAL's ghost header on every raw read.
    bool validateGhostLeader = true;
    /// Optional pool for decoded blocks (must outlive the blocks).
    mem::BufferPool* pool = nullptr;
};

class TiffReader {
public:
    /// Parses a TIFF from any ByteSource (kept alive by the reader).
    static Expected<std::shared_ptr<const TiffReader>> Open(std::shared_ptr<const io::ByteSource> source,
                                                            const TiffReaderOptions& options = {});
    /// Opens a local file (memory-mapped on Linux, pread elsewhere).
    static Expected<std::shared_ptr<const TiffReader>> Open(std::string_view utf8Path,
                                                            const TiffReaderOptions& options = {});

    TiffReader(const TiffReader&) = delete;
    TiffReader& operator=(const TiffReader&) = delete;
    ~TiffReader();

    bool IsBigTiff() const noexcept { return bigTiff_; }
    bool IsBigEndian() const noexcept { return bigEndian_; }
    const GhostHeader& Ghost() const noexcept { return ghost_; }
    const io::ByteSource& Source() const noexcept { return *source_; }
    const std::shared_ptr<const io::ByteSource>& SharedSource() const noexcept { return source_; }
    const TiffReaderOptions& Options() const noexcept { return options_; }

    /// Every directory: the main chain in file order, each followed by its SubIFDs (depth first).
    const std::vector<IfdInfo>& Ifds() const noexcept { return ifds_; }
    const IfdInfo& Ifd(uint32_t index) const { return ifds_.at(index); }
    size_t IfdCount() const noexcept { return ifds_.size(); }

    /// Index of the main image (first full-resolution IFD of the main chain).
    uint32_t MainImage() const noexcept { return main_; }
    /// Overviews of the main image, largest first.
    std::vector<uint32_t> Overviews() const;
    /// The mask IFD of `ifd` (same size, mask kind), if any.
    std::optional<uint32_t> MaskOf(uint32_t ifd) const;

    // Tag values ----------------------------------------------------------------------------

    /// Raw bytes of an entry's values (file byte order). Limited to maxBytes.
    Expected<std::vector<uint8_t>> ReadTagBytes(const TagEntry& e, uint64_t maxBytes) const;
    /// Integer types (BYTE, SHORT, LONG, LONG8, IFD, IFD8 and signed ones) as uint64 (signed
    /// values sign-extended then cast).
    Expected<std::vector<uint64_t>> ReadTagUInts(const TagEntry& e, uint64_t maxCount) const;
    /// Any numeric type as double (rationals divided; a zero denominator gives 0 like libtiff).
    Expected<std::vector<double>> ReadTagDoubles(const TagEntry& e, uint64_t maxCount) const;
    /// ASCII (or BYTE/UNDEFINED) as a string, cut at the first NUL.
    Expected<std::string> ReadTagString(const TagEntry& e, uint64_t maxBytes) const;

    // Blocks ----------------------------------------------------------------------------------

    /// Offset and size of block `index` of `ifd` (loads the block index on first use).
    Expected<BlockLocation> Locate(uint32_t ifd, uint32_t index) const;
    /// Loads the whole block index of `ifd` now (otherwise done on first use).
    Status LoadBlockIndex(uint32_t ifd) const;
    /// True when the block index of `ifd` is resident (loaded, or served from the mapping).
    bool BlockIndexLoaded(uint32_t ifd) const;

    /// Compressed bytes of a block (no decoding). A sparse block gives an empty RawBlock with
    /// IsSparse(). With a ghost header leader / trailer, they are checked against the size.
    Expected<RawBlock> ReadBlockRaw(uint32_t ifd, uint32_t index) const;
    /// Whether the block has no stored data.
    Expected<bool> IsSparse(uint32_t ifd, uint32_t index) const;

    /// Decodes a block into a new 64-byte aligned buffer.
    Expected<DecodedBlock> ReadBlock(uint32_t ifd, uint32_t index) const;
    /// Decodes a block into `out` (exactly DecodedSize(ifd, index) bytes, aligned to the sample
    /// size); returns whether it was sparse.
    Expected<bool> ReadBlockInto(uint32_t ifd, uint32_t index, std::span<uint8_t> out) const;
    /// Size ReadBlockInto needs (0 when the block cannot be decoded).
    uint64_t DecodedSize(uint32_t ifd, uint32_t index) const noexcept;
    /// Decodes already-read compressed bytes of block `index` (the raw path of a caller that
    /// fetched or cached them itself) into `out`.
    Status DecodeBlock(uint32_t ifd, uint32_t index, std::span<const uint8_t> compressed, std::span<uint8_t> out) const;

    /// Fills `out` the way a sparse block of `ifd` reads (nodata value, else 0).
    void FillSparse(uint32_t ifd, std::span<uint8_t> out) const;

    /// Checks that `ifd` can be decoded (sample type, codec, photometric); the message says why
    /// not.
    Status CheckDecodable(uint32_t ifd) const;

    struct IfdState;  // lazily loaded per-IFD data (block index)

private:
    TiffReader() = default;
    Status Parse();
    Expected<IfdInfo> ParseIfd(uint64_t offset, std::vector<uint64_t>* subIfds, uint64_t* next);
    Status FinishIfd(IfdInfo& info);
    Status ReadBlockIndex(uint32_t ifd, IfdState& st) const;
    Expected<std::span<const uint8_t>> ReadRaw(uint32_t ifd, uint32_t index, const BlockLocation& loc,
                                               std::vector<uint8_t>& storage) const;
    Expected<BlockLocation> LocateMapped(const IfdInfo& info, const IfdState& st, uint32_t index) const;

    std::shared_ptr<const io::ByteSource> source_;
    TiffReaderOptions options_;
    bool bigTiff_ = false;
    bool bigEndian_ = false;
    GhostHeader ghost_;
    std::vector<IfdInfo> ifds_;
    std::vector<std::unique_ptr<IfdState>> states_;
    uint32_t main_ = 0;
    // The blocks of the compressions ddbt has no codec for (libtiff_codec.h), opened on first use
    mutable std::once_flag libtiffOnce_;
    mutable std::unique_ptr<LibtiffCodec> libtiff_;
    const LibtiffCodec& Libtiff() const;
};

/// The value GDAL fills a sparse block with, converted to the sample type (rounded and clamped
/// for integers), written into `out` (count samples). Sets 0 when `value` is not set.
void FillSamples(std::span<uint8_t> out, SampleType type, std::optional<double> value);

/// Converts rows of 24-bit floats (FP24, host order) to Float32 as GDAL does (CPLTripleToFloat).
void ExpandFloat24(const uint8_t* in, size_t inRowStride, uint64_t samples, uint32_t rows, uint8_t* out,
                   size_t outRowStride);

/// Unpacks rows of `samples` samples of `bits` bits (2 to 31, MSB first, each row padded to a
/// byte) into unsigned samples of `outBytes` bytes (1, 2 or 4), host order.
void UnpackSamples(const uint8_t* in, size_t inRowStride, uint64_t samples, uint32_t rows, uint32_t bits,
                   size_t outBytes, uint8_t* out, size_t outRowStride);

/// Expands rows of packed 1-bit samples (MSB first) to bytes 0 / `one` (GDAL reads a 1-bit
/// mask as 0 / 255).
void ExpandBits(const uint8_t* in, size_t inRowStride, uint32_t width, uint32_t rows, uint8_t one, uint8_t* out,
                size_t outRowStride);

/// Whether rows of packed 1-bit samples (MSB first) hold any 0 and any 1, over `width` samples
/// of each of `rows` rows (bits past `width` are ignored).
struct BitsSummary {
    bool any0 = false;
    bool any1 = false;
};
BitsSummary SummarizeBits(const uint8_t* in, size_t inRowStride, uint32_t width, uint32_t rows);

}  // namespace ddbt::cog::tiff
