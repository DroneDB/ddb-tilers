// SPDX-License-Identifier: MPL-2.0
//
// TiffReader: header, ghost header, IFD chain / SubIFDs, tag values, block index.

#include "cog/tiff/tiff_reader.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>

#include "core/util/checked_math.h"
#include "core/util/endian.h"

namespace ddbt::cog::tiff {

// Per-IFD state loaded on first use.
struct TiffReader::IfdState {
    TagEntry offsetsTag;
    TagEntry countsTag;
    bool hasOffsets = false;
    bool hasCounts = false;
    bool mapped = false;  // entries read from the mapping, no arrays
    std::once_flag once;
    std::atomic<bool> loaded{false};
    Status loadStatus;
    std::vector<uint64_t> offsets;
    std::vector<uint64_t> counts;
};

namespace {

std::string Hex(uint64_t v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "0x%llx", static_cast<unsigned long long>(v));
    return buf;
}

uint64_t LoadUInt(const uint8_t* p, size_t size, bool be) {
    switch (size) {
        case 1: return p[0];
        case 2: return LoadEndian<uint16_t>(p, be);
        case 4: return LoadEndian<uint32_t>(p, be);
        case 8: return LoadEndian<uint64_t>(p, be);
        default: return 0;
    }
}

bool IsIntegerType(uint16_t type) {
    switch (static_cast<TagType>(type)) {
        case TagType::kByte:
        case TagType::kShort:
        case TagType::kLong:
        case TagType::kSByte:
        case TagType::kUndefined:
        case TagType::kSShort:
        case TagType::kSLong:
        case TagType::kIfd:
        case TagType::kLong8:
        case TagType::kSLong8:
        case TagType::kIfd8: return true;
        default: return false;
    }
}

// Decodes value i of an integer-typed array (file byte order `be`).
uint64_t DecodeIntValue(const uint8_t* p, uint16_t type, bool be) {
    switch (static_cast<TagType>(type)) {
        case TagType::kByte:
        case TagType::kUndefined: return p[0];
        case TagType::kSByte: return static_cast<uint64_t>(static_cast<int64_t>(static_cast<int8_t>(p[0])));
        case TagType::kShort: return LoadEndian<uint16_t>(p, be);
        case TagType::kSShort: return static_cast<uint64_t>(static_cast<int64_t>(LoadEndian<int16_t>(p, be)));
        case TagType::kLong:
        case TagType::kIfd: return LoadEndian<uint32_t>(p, be);
        case TagType::kSLong: return static_cast<uint64_t>(static_cast<int64_t>(LoadEndian<int32_t>(p, be)));
        case TagType::kLong8:
        case TagType::kIfd8:
        case TagType::kSLong8: return LoadEndian<uint64_t>(p, be);
        default: return 0;
    }
}

double DecodeDoubleValue(const uint8_t* p, uint16_t type, bool be) {
    switch (static_cast<TagType>(type)) {
        case TagType::kSByte:
        case TagType::kSShort:
        case TagType::kSLong:
        case TagType::kSLong8: return static_cast<double>(static_cast<int64_t>(DecodeIntValue(p, type, be)));
        case TagType::kRational: {
            const uint32_t n = LoadEndian<uint32_t>(p, be), d = LoadEndian<uint32_t>(p + 4, be);
            return n == 0 || d == 0 ? 0.0 : static_cast<double>(n) / static_cast<double>(d);
        }
        case TagType::kSRational: {
            const int32_t n = LoadEndian<int32_t>(p, be), d = LoadEndian<int32_t>(p + 4, be);
            return n == 0 || d == 0 ? 0.0 : static_cast<double>(n) / static_cast<double>(d);
        }
        case TagType::kFloat: return static_cast<double>(LoadEndian<float>(p, be));
        case TagType::kDouble: return LoadEndian<double>(p, be);
        default: return static_cast<double>(DecodeIntValue(p, type, be));
    }
}

SampleType ToSampleType(uint16_t format, uint16_t bits) {
    // SampleFormat 4 (void) reads as unsigned, as GDAL does.
    const bool isUint = format == 1 || format == 4;
    // Integer samples of another width (NBITS: 2 to 31 bits, packed) read as the unsigned type
    // that holds them, as GDAL reads them (Byte, UInt16, UInt32)
    if (bits >= 2 && bits < 32 && bits != 8 && bits != 16 && (format == 1 || format == 2 || format == 4))
        return bits < 8 ? SampleType::kU8 : bits < 16 ? SampleType::kU16 : SampleType::kU32;
    switch (bits) {
        case 1: return isUint ? SampleType::kBit1 : SampleType::kUnknown;
        case 8: return isUint ? SampleType::kU8 : format == 2 ? SampleType::kI8 : SampleType::kUnknown;
        case 16:
            return isUint ? SampleType::kU16 : format == 2 ? SampleType::kI16 : format == 3 ? SampleType::kF16 : SampleType::kUnknown;
        case 32:
            return isUint ? SampleType::kU32 : format == 2 ? SampleType::kI32 : format == 3 ? SampleType::kF32 : SampleType::kUnknown;
        case 64:
            return isUint ? SampleType::kU64 : format == 2 ? SampleType::kI64 : format == 3 ? SampleType::kF64 : SampleType::kUnknown;
        default: return SampleType::kUnknown;
    }
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Small helpers of the public types

std::string_view SampleTypeName(SampleType t) noexcept {
    switch (t) {
        case SampleType::kBit1: return "Bit1";
        case SampleType::kU8: return "UInt8";
        case SampleType::kI8: return "Int8";
        case SampleType::kU16: return "UInt16";
        case SampleType::kI16: return "Int16";
        case SampleType::kU32: return "UInt32";
        case SampleType::kI32: return "Int32";
        case SampleType::kU64: return "UInt64";
        case SampleType::kI64: return "Int64";
        case SampleType::kF16: return "Float16";
        case SampleType::kF32: return "Float32";
        case SampleType::kF64: return "Float64";
        default: return "Unknown";
    }
}

size_t SampleTypeSize(SampleType t) noexcept {
    switch (t) {
        case SampleType::kU8:
        case SampleType::kI8: return 1;
        case SampleType::kU16:
        case SampleType::kI16:
        case SampleType::kF16: return 2;
        case SampleType::kU32:
        case SampleType::kI32:
        case SampleType::kF32: return 4;
        case SampleType::kU64:
        case SampleType::kI64:
        case SampleType::kF64: return 8;
        default: return 0;
    }
}

std::string_view SubfileKindName(SubfileKind k) noexcept {
    switch (k) {
        case SubfileKind::kFullResolution: return "full-resolution";
        case SubfileKind::kOverview: return "overview";
        case SubfileKind::kMask: return "mask";
        case SubfileKind::kOverviewMask: return "overview-mask";
        case SubfileKind::kPage: return "page";
    }
    return "?";
}

uint64_t TagEntry::ByteSize() const noexcept {
    uint64_t n = 0;
    const size_t s = TagTypeSize(type);
    if (s == 0 || !CheckedMul<uint64_t>(count, s, &n)) return 0;
    return n;
}

uint32_t IfdInfo::BlockRows(uint32_t block) const noexcept {
    if (tiled || blockHeight == 0) return blockHeight;
    const uint64_t row = BlockRow(block);
    const uint64_t first = row * blockHeight;
    if (first >= height) return 0;
    return static_cast<uint32_t>(std::min<uint64_t>(blockHeight, height - first));
}

namespace {
// Bytes of a row of `samples` samples of `bits` bits, packed and padded to a byte
uint64_t PackedRowBytes(uint64_t samples, uint64_t bits) noexcept {
    uint64_t total = 0;
    if (!CheckedMul<uint64_t>(samples, bits, &total)) return 0;
    return (total + 7) / 8;
}
}  // namespace

bool IfdInfo::IsPacked() const noexcept {
    return sampleType != SampleType::kBit1 && !IsFloat24() && SampleTypeSize(sampleType) * 8 * Components() != bitsPerSample;
}

uint64_t IfdInfo::BlockRowBytes() const noexcept {
    // decoded values: the type's bits (packed samples unpacked, FP24 as Float32, each component
    // of a complex sample)
    const uint64_t bits = sampleType == SampleType::kBit1 ? 1 : SampleTypeSize(sampleType) * 8;
    return PackedRowBytes(uint64_t{blockWidth} * SamplesPerBlockPixel(), bits);
}

uint64_t IfdInfo::StoredRowBytes() const noexcept {
    return PackedRowBytes(uint64_t{blockWidth} * (SamplesPerBlockPixel() / Components()), bitsPerSample);
}

uint64_t IfdInfo::DecodedBlockBytes(uint32_t block) const noexcept {
    uint64_t n = 0;
    if (!CheckedMul<uint64_t>(BlockRowBytes(), BlockRows(block), &n)) return 0;
    return n;
}

const TagEntry* IfdInfo::FindTag(uint16_t t) const noexcept {
    auto it = std::lower_bound(tags.begin(), tags.end(), t, [](const TagEntry& e, uint16_t v) { return e.tag < v; });
    return it != tags.end() && it->tag == t ? &*it : nullptr;
}

bool IfdInfo::HasAlpha() const noexcept {
    return !extraSamples.empty() && (extraSamples.back() == extrasample::kUnassociatedAlpha ||
                                     extraSamples.back() == extrasample::kAssociatedAlpha);
}

BlockBuffer BlockBuffer::Allocate(size_t size, mem::BufferPool* pool) {
    BlockBuffer b;
    if (pool != nullptr) {
        b.pooled_ = pool->Acquire(size);
        b.data_ = b.pooled_.data();
    } else {
        b.owned_ = mem::MakeAlignedBytes(size == 0 ? 1 : size, mem::kCacheLine);
        b.data_ = b.owned_.get();
    }
    b.size_ = size;
    return b;
}

// ---------------------------------------------------------------------------------------------
// Open / parse

TiffReader::~TiffReader() = default;

Expected<std::shared_ptr<const TiffReader>> TiffReader::Open(std::string_view utf8Path,
                                                             const TiffReaderOptions& options) {
    auto src = io::OpenByteSource(utf8Path, io::SourceKind::kAuto, io::AccessHint::kRandom);
    if (!src.ok()) return std::move(src).TakeStatus();
    std::shared_ptr<const io::ByteSource> shared(std::move(*src));
    return Open(std::move(shared), options);
}

Expected<std::shared_ptr<const TiffReader>> TiffReader::Open(std::shared_ptr<const io::ByteSource> source,
                                                             const TiffReaderOptions& options) {
    if (!source) return InvalidArgument("TiffReader: null source");
    std::shared_ptr<TiffReader> r(new TiffReader());
    r->source_ = std::move(source);
    r->options_ = options;
    Status st = r->Parse();
    if (!st.ok()) return st.WithContext(std::string(r->source_->Name()));
    return std::shared_ptr<const TiffReader>(std::move(r));
}

namespace {

void ParseGhostLines(GhostHeader& g) {
    size_t pos = 0;
    const std::string& t = g.text;
    while (pos < t.size()) {
        size_t end = t.find('\n', pos);
        if (end == std::string::npos) end = t.size();
        const std::string_view line(t.data() + pos, end - pos);
        pos = end + 1;
        const size_t eq = line.find('=');
        if (eq == std::string_view::npos) continue;
        // GDAL pads lines with spaces: its COG driver writes " MASK_INTERLEAVED_WITH_IMAGERY=YES"
        // (a leading space) after "KNOWN_INCOMPATIBLE_EDITION=NO".
        const auto trim = [](std::string_view s) {
            const auto space = [](char c) { return c == ' ' || c == '\t' || c == '\r'; };
            while (!s.empty() && space(s.front())) s.remove_prefix(1);
            while (!s.empty() && space(s.back())) s.remove_suffix(1);
            return s;
        };
        const std::string_view key = trim(line.substr(0, eq));
        const std::string_view value = trim(line.substr(eq + 1));
        if (key == "LAYOUT") g.layoutIfdsBeforeData = value == "IFDS_BEFORE_DATA";
        else if (key == "BLOCK_ORDER") g.blockOrderRowMajor = value == "ROW_MAJOR";
        else if (key == "BLOCK_LEADER") g.blockLeaderSizeAsUint4 = value == "SIZE_AS_UINT4";
        else if (key == "BLOCK_TRAILER") g.blockTrailerLast4BytesRepeated = value == "LAST_4_BYTES_REPEATED";
        else if (key == "KNOWN_INCOMPATIBLE_EDITION") g.knownIncompatibleEdition = value == "YES";
        else if (key == "MASK_INTERLEAVED_WITH_IMAGERY") g.maskInterleavedWithImagery = value == "YES";
    }
}

}  // namespace

Status TiffReader::Parse() {
    const uint64_t fileSize = source_->Size();
    if (fileSize < 8) return Corrupt("not a TIFF file (shorter than a header)");
    uint8_t hdr[16] = {};
    DDBT_TRY(source_->Read(0, std::span<uint8_t>(hdr, static_cast<size_t>(std::min<uint64_t>(fileSize, 16)))));
    if (hdr[0] == 'I' && hdr[1] == 'I') bigEndian_ = false;
    else if (hdr[0] == 'M' && hdr[1] == 'M') bigEndian_ = true;
    else return Corrupt("not a TIFF file (bad byte order mark)");
    const uint16_t magic = LoadEndian<uint16_t>(hdr + 2, bigEndian_);
    uint64_t firstIfd = 0;
    size_t headerSize = 0;
    if (magic == 42) {
        bigTiff_ = false;
        firstIfd = LoadEndian<uint32_t>(hdr + 4, bigEndian_);
        headerSize = 8;
    } else if (magic == 43) {
        bigTiff_ = true;
        if (fileSize < 16) return Corrupt("BigTIFF header truncated");
        if (LoadEndian<uint16_t>(hdr + 4, bigEndian_) != 8 || LoadEndian<uint16_t>(hdr + 6, bigEndian_) != 0) {
            return Corrupt("BigTIFF header: unexpected offset size");
        }
        firstIfd = LoadEndian<uint64_t>(hdr + 8, bigEndian_);
        headerSize = 16;
    } else {
        return Corrupt("not a TIFF file (magic " + std::to_string(magic) + ")");
    }

    // GDAL ghost header.
    {
        static constexpr std::string_view kKey = "GDAL_STRUCTURAL_METADATA_SIZE=";
        uint8_t buf[64] = {};
        const uint64_t avail = fileSize > headerSize ? fileSize - headerSize : 0;
        const size_t n = static_cast<size_t>(std::min<uint64_t>(avail, sizeof(buf)));
        if (n > kKey.size() + 6) {
            DDBT_TRY(source_->Read(headerSize, std::span<uint8_t>(buf, n)));
            const std::string_view s(reinterpret_cast<const char*>(buf), n);
            if (s.substr(0, kKey.size()) == kKey) {
                size_t i = kKey.size();
                uint64_t size = 0;
                while (i < s.size() && s[i] >= '0' && s[i] <= '9' && size < 100000000) size = size * 10 + static_cast<uint64_t>(s[i++] - '0');
                const size_t lineEnd = s.find('\n', i);
                if (lineEnd != std::string_view::npos && size > 0 && size <= (uint64_t{1} << 20)) {
                    const uint64_t start = headerSize + lineEnd + 1;
                    if (RangeInside(start, size, fileSize)) {
                        std::string text(static_cast<size_t>(size), '\0');
                        DDBT_TRY(source_->Read(start, std::span<uint8_t>(reinterpret_cast<uint8_t*>(text.data()), text.size())));
                        const size_t nul = text.find('\0');
                        if (nul != std::string::npos) text.resize(nul);
                        ghost_.present = true;
                        ghost_.text = std::move(text);
                        ParseGhostLines(ghost_);
                    }
                }
            }
        }
    }

    // Main chain, each IFD followed by its SubIFD trees (depth first).
    std::unordered_set<uint64_t> visited;
    // Depth-first walk with an explicit stack of chains.
    std::vector<std::pair<uint64_t, int32_t>> chainStack;  // (next offset of a chain, parent)
    chainStack.emplace_back(firstIfd, -1);
    if (firstIfd == 0) return Corrupt("no image directory");
    while (!chainStack.empty()) {
        auto [offset, parent] = chainStack.back();
        chainStack.pop_back();
        while (offset != 0) {
            if (ifds_.size() >= options_.maxIfds) return ResourceExhausted("more than " + std::to_string(options_.maxIfds) + " directories");
            if (!visited.insert(offset).second) {
                if (ifds_.empty()) return Corrupt("IFD loop");
                break;  // libtiff: loop in the chain, stop following it
            }
            std::vector<uint64_t> subIfds;
            uint64_t next = 0;
            auto info = ParseIfd(offset, &subIfds, &next);
            if (!info.ok()) {
                // The first directory must parse; a broken later one ends the chain (libtiff
                // reports the error when that directory is selected).
                if (ifds_.empty()) return info.status();
                break;
            }
            info->index = static_cast<uint32_t>(ifds_.size());
            info->parent = parent;
            const int32_t self = static_cast<int32_t>(info->index);
            ifds_.push_back(std::move(*info));
            // Continue this chain after the SubIFDs of this directory.
            if (!subIfds.empty()) {
                chainStack.emplace_back(next, parent);
                for (auto it = subIfds.rbegin(); it != subIfds.rend(); ++it) {
                    if (*it != 0) chainStack.emplace_back(*it, self);
                }
                // SubIFDs listed in the tag are each a chain head; only follow their own
                // chains if they have next pointers (handled when popped).
                break;
            }
            offset = next;
        }
    }

    // Per-IFD state and classification.
    bool haveMain = false;
    for (IfdInfo& info : ifds_) {
        if (info.kind == SubfileKind::kFullResolution) {
            if (!haveMain && info.parent < 0) {
                main_ = info.index;
                haveMain = true;
            } else {
                info.kind = SubfileKind::kPage;
            }
        }
    }
    if (!haveMain) {
        // No plain full-resolution directory in the main chain: use the first one.
        main_ = 0;
    }
    states_.reserve(ifds_.size());
    for (IfdInfo& info : ifds_) {
        auto st = std::make_unique<IfdState>();
        const uint16_t offTag = info.tiled ? tag::kTileOffsets : tag::kStripOffsets;
        const uint16_t cntTag = info.tiled ? tag::kTileByteCounts : tag::kStripByteCounts;
        if (const TagEntry* e = info.FindTag(offTag)) {
            st->offsetsTag = *e;
            st->hasOffsets = true;
        }
        if (const TagEntry* e = info.FindTag(cntTag)) {
            st->countsTag = *e;
            st->hasCounts = true;
        }
        // Entries straight from the mapping when both arrays are valid integer arrays inside
        // the file.
        const uint64_t size = source_->Size();
        auto arrayOk = [&](const TagEntry& e) {
            const size_t ts = TagTypeSize(e.type);
            return IsIntegerType(e.type) && ts >= 2 && (e.isInline || RangeInside(e.valueOffset, e.ByteSize(), size));
        };
        st->mapped = source_->Data() != nullptr && st->hasOffsets && st->hasCounts && arrayOk(st->offsetsTag) &&
                     arrayOk(st->countsTag);
        if (st->mapped) st->loaded.store(true, std::memory_order_release);
        states_.push_back(std::move(st));
    }
    return Status::Ok();
}

Expected<IfdInfo> TiffReader::ParseIfd(uint64_t offset, std::vector<uint64_t>* subIfds, uint64_t* next) {
    const uint64_t fileSize = source_->Size();
    const size_t countSize = bigTiff_ ? 8 : 2;
    const size_t entrySize = bigTiff_ ? 20 : 12;
    const size_t nextSize = bigTiff_ ? 8 : 4;
    if (!RangeInside(offset, countSize, fileSize)) return Corrupt("IFD at " + Hex(offset) + " is past the end of the file");
    uint8_t cbuf[8];
    DDBT_TRY(source_->Read(offset, std::span<uint8_t>(cbuf, countSize)));
    const uint64_t count = bigTiff_ ? LoadEndian<uint64_t>(cbuf, bigEndian_) : LoadEndian<uint16_t>(cbuf, bigEndian_);
    if (count == 0) return Corrupt("IFD at " + Hex(offset) + " has no entries");
    if (count > options_.maxEntriesPerIfd) {
        return ResourceExhausted("IFD at " + Hex(offset) + " has " + std::to_string(count) + " entries");
    }
    const uint64_t bytes = count * entrySize + nextSize;
    std::vector<uint8_t> buf;
    if (!RangeInside(offset + countSize, count * entrySize, fileSize)) {
        return Corrupt("IFD at " + Hex(offset) + " is truncated");
    }
    // The next-IFD pointer may be missing at the very end of a file: treat as 0.
    const bool haveNext = RangeInside(offset + countSize, bytes, fileSize);
    buf.resize(static_cast<size_t>(haveNext ? bytes : count * entrySize));
    DDBT_TRY(source_->Read(offset + countSize, std::span<uint8_t>(buf)));
    *next = haveNext ? LoadUInt(buf.data() + count * entrySize, nextSize, bigEndian_) : 0;

    IfdInfo info;
    info.offset = offset;
    info.tags.reserve(static_cast<size_t>(count));
    const size_t fieldSize = bigTiff_ ? 8 : 4;
    for (uint64_t i = 0; i < count; ++i) {
        const uint8_t* p = buf.data() + i * entrySize;
        TagEntry e;
        e.tag = LoadEndian<uint16_t>(p, bigEndian_);
        e.type = LoadEndian<uint16_t>(p + 2, bigEndian_);
        e.count = bigTiff_ ? LoadEndian<uint64_t>(p + 4, bigEndian_) : LoadEndian<uint32_t>(p + 4, bigEndian_);
        const uint8_t* field = p + (bigTiff_ ? 12 : 8);
        const uint64_t byteSize = e.ByteSize();
        if (TagTypeSize(e.type) != 0 && byteSize <= fieldSize && (e.count == 0 || byteSize > 0)) {
            e.isInline = true;
            std::memcpy(e.inlineBytes.data(), field, fieldSize);
        } else {
            e.isInline = false;
            e.valueOffset = LoadUInt(field, fieldSize, bigEndian_);
        }
        info.tags.push_back(e);
    }
    std::stable_sort(info.tags.begin(), info.tags.end(), [](const TagEntry& a, const TagEntry& b) { return a.tag < b.tag; });
    info.tags.erase(std::unique(info.tags.begin(), info.tags.end(),
                                [](const TagEntry& a, const TagEntry& b) { return a.tag == b.tag; }),
                    info.tags.end());

    if (const TagEntry* e = info.FindTag(tag::kSubIfds)) {
        auto v = ReadTagUInts(*e, 4096);
        if (v.ok()) *subIfds = std::move(*v);
    }
    DDBT_TRY(FinishIfd(info).WithContext("IFD at " + Hex(offset)));
    return info;
}

Status TiffReader::FinishIfd(IfdInfo& info) {
    auto getUInt = [&](uint16_t t, uint64_t fallback, uint64_t* out) -> Status {
        *out = fallback;
        const TagEntry* e = info.FindTag(t);
        if (e == nullptr || e->count == 0) return Status::Ok();
        if (!IsIntegerType(e->type)) return Corrupt("tag " + std::to_string(t) + " is not an integer");
        auto v = ReadTagUInts(*e, options_.maxTagBytes);
        if (!v.ok()) return v.status();
        *out = (*v)[0];
        return Status::Ok();
    };
    auto getPerSample = [&](uint16_t t, uint64_t fallback, uint64_t* out) -> Status {
        *out = fallback;
        const TagEntry* e = info.FindTag(t);
        if (e == nullptr || e->count == 0) return Status::Ok();
        if (!IsIntegerType(e->type)) return Corrupt("tag " + std::to_string(t) + " is not an integer");
        auto v = ReadTagUInts(*e, 65536);
        if (!v.ok()) return v.status();
        *out = (*v)[0];
        for (uint64_t x : *v) {
            if (x != (*v)[0]) return Unsupported("tag " + std::to_string(t) + " differs between samples");
        }
        return Status::Ok();
    };

    uint64_t v = 0;
    DDBT_TRY(getUInt(tag::kNewSubfileType, 0, &v));
    info.newSubfileType = static_cast<uint32_t>(v);
    if (info.FindTag(tag::kNewSubfileType) == nullptr) {
        uint64_t old = 0;
        DDBT_TRY(getUInt(tag::kSubfileType, 0, &old));
        if (old == 2) info.newSubfileType = 1;  // OFILETYPE_REDUCEDIMAGE
        else if (old == 3) info.newSubfileType = 2;  // OFILETYPE_PAGE
    }
    if (info.newSubfileType & 4u) info.kind = (info.newSubfileType & 1u) ? SubfileKind::kOverviewMask : SubfileKind::kMask;
    else if (info.newSubfileType & 1u) info.kind = SubfileKind::kOverview;
    else info.kind = SubfileKind::kFullResolution;

    uint64_t width = 0, height = 0;
    DDBT_TRY(getUInt(tag::kImageWidth, 0, &width));
    DDBT_TRY(getUInt(tag::kImageLength, 0, &height));
    if (width == 0 || height == 0) return Corrupt("missing or zero ImageWidth / ImageLength");
    if (width > UINT32_MAX || height > UINT32_MAX) return Corrupt("image size out of range");
    info.width = static_cast<uint32_t>(width);
    info.height = static_cast<uint32_t>(height);

    DDBT_TRY(getUInt(tag::kSamplesPerPixel, 1, &v));
    if (v == 0 || v > 65535) return Corrupt("SamplesPerPixel out of range");
    info.samplesPerPixel = static_cast<uint16_t>(v);
    DDBT_TRY(getPerSample(tag::kBitsPerSample, 1, &v));
    if (v == 0 || v > 128) return Corrupt("BitsPerSample out of range");  // 128: CFloat64
    info.bitsPerSample = static_cast<uint16_t>(v);
    DDBT_TRY(getPerSample(tag::kSampleFormat, 1, &v));
    info.sampleFormat = static_cast<uint16_t>(std::min<uint64_t>(v, 65535));
    DDBT_TRY(getUInt(tag::kCompression, 1, &v));
    info.compression = static_cast<uint16_t>(std::min<uint64_t>(v, 65535));
    if (const TagEntry* e = info.FindTag(tag::kPhotometric); e != nullptr && e->count > 0) {
        DDBT_TRY(getUInt(tag::kPhotometric, 1, &v));
        info.photometric = static_cast<uint16_t>(std::min<uint64_t>(v, 65535));
        info.hasPhotometric = true;
    }
    DDBT_TRY(getUInt(tag::kPlanarConfig, 1, &v));
    if (v != 1 && v != 2) return Corrupt("PlanarConfiguration " + std::to_string(v));
    info.planarConfig = static_cast<uint16_t>(v);
    if (info.samplesPerPixel == 1) info.planarConfig = 1;
    DDBT_TRY(getUInt(tag::kPredictor, 1, &v));
    info.predictor = static_cast<uint16_t>(std::min<uint64_t>(v, 65535));
    DDBT_TRY(getUInt(tag::kFillOrder, 1, &v));
    info.fillOrder = static_cast<uint16_t>(std::min<uint64_t>(v, 65535));
    // Complex samples (SampleFormat 5 / 6) are two components, real then imaginary, each of the
    // integer / float type of half the bits (GDAL's CInt16, CInt32, CFloat16, CFloat32, CFloat64)
    info.complex = info.sampleFormat == 5 || info.sampleFormat == 6;
    if (info.complex) {
        const uint16_t half = static_cast<uint16_t>(info.bitsPerSample / 2);
        const bool whole = info.bitsPerSample % 2 == 0 && (info.sampleFormat == 6 ? half >= 16 : half == 16 || half == 32);
        info.sampleType = whole ? ToSampleType(info.sampleFormat == 5 ? 2 : 3, half) : SampleType::kUnknown;
    } else if (info.sampleFormat == 3 && info.bitsPerSample == 24) {
        // GDAL reads 24-bit floats (FP24) as Float32
        info.sampleType = SampleType::kF32;
    } else {
        info.sampleType = ToSampleType(info.sampleFormat, info.bitsPerSample);
    }

    if (const TagEntry* e = info.FindTag(tag::kExtraSamples)) {
        auto x = ReadTagUInts(*e, 65536);
        if (x.ok()) {
            for (uint64_t s : *x) info.extraSamples.push_back(static_cast<uint16_t>(std::min<uint64_t>(s, 65535)));
        }
    }
    if (const TagEntry* e = info.FindTag(tag::kYCbCrSubsampling)) {
        auto x = ReadTagUInts(*e, 16);
        if (x.ok() && x->size() >= 2) {
            info.ycbcrSubsampling = {static_cast<uint16_t>(std::min<uint64_t>((*x)[0], 65535)),
                                     static_cast<uint16_t>(std::min<uint64_t>((*x)[1], 65535))};
        }
    }
    DDBT_TRY(getUInt(tag::kYCbCrPositioning, 1, &v));
    info.ycbcrPositioning = static_cast<uint16_t>(std::min<uint64_t>(v, 65535));
    if (const TagEntry* e = info.FindTag(tag::kColorMap); e != nullptr && info.bitsPerSample <= 16) {
        const uint64_t expected = 3ull << info.bitsPerSample;
        if (e->count >= expected) {
            auto x = ReadTagUInts(*e, expected + 65536);
            if (x.ok()) {
                info.colorMap.reserve(static_cast<size_t>(expected));
                for (uint64_t i = 0; i < expected; ++i) info.colorMap.push_back(static_cast<uint16_t>((*x)[i]));
            }
        }
    }
    if (const TagEntry* e = info.FindTag(tag::kJpegTables)) {
        auto x = ReadTagBytes(*e, options_.maxTagBytes);
        if (x.ok()) info.jpegTables = std::move(*x);
    }
    if (const TagEntry* e = info.FindTag(tag::kLercParameters)) {
        auto x = ReadTagUInts(*e, 16);
        if (x.ok() && x->size() >= 2) {
            info.lercParameters = {static_cast<uint32_t>((*x)[0]), static_cast<uint32_t>((*x)[1])};
            info.hasLercParameters = true;
        }
    }
    if (const TagEntry* e = info.FindTag(tag::kGdalNodata)) {
        auto x = ReadTagString(*e, 4096);
        if (x.ok()) {
            info.noDataText = std::move(*x);
            // GDAL: CPLAtofM, which accepts "nan", "inf", "-inf" like strtod.
            const char* s = info.noDataText.c_str();
            while (*s == ' ') ++s;
            if (*s != '\0') {
                char* end = nullptr;
                errno = 0;
                const double d = std::strtod(s, &end);
                if (end != s) info.noData = d;
            }
        }
    }
    if (const TagEntry* e = info.FindTag(tag::kGdalMetadata)) {
        auto x = ReadTagString(*e, options_.maxTagBytes);
        if (x.ok()) info.gdalMetadata = std::move(*x);
    }
    if (const TagEntry* e = info.FindTag(tag::kImageDescription)) {
        auto x = ReadTagString(*e, options_.maxTagBytes);
        if (x.ok()) info.imageDescription = std::move(*x);
    }
    if (const TagEntry* e = info.FindTag(tag::kSoftware)) {
        auto x = ReadTagString(*e, 65536);
        if (x.ok()) info.software = std::move(*x);
    }

    // Block layout.
    const TagEntry* tw = info.FindTag(tag::kTileWidth);
    const TagEntry* tl = info.FindTag(tag::kTileLength);
    if (tw != nullptr && tl != nullptr) {
        uint64_t w = 0, h = 0;
        DDBT_TRY(getUInt(tag::kTileWidth, 0, &w));
        DDBT_TRY(getUInt(tag::kTileLength, 0, &h));
        if (w == 0 || h == 0 || w > (1u << 24) || h > (1u << 24)) return Corrupt("tile size out of range");
        info.tiled = true;
        info.blockWidth = static_cast<uint32_t>(w);
        info.blockHeight = static_cast<uint32_t>(h);
    } else {
        uint64_t rps = 0;
        DDBT_TRY(getUInt(tag::kRowsPerStrip, UINT32_MAX, &rps));
        if (rps == 0 || rps > info.height) rps = info.height;
        info.tiled = false;
        info.blockWidth = info.width;
        info.blockHeight = static_cast<uint32_t>(rps);
    }
    info.blocksAcross = static_cast<uint32_t>((uint64_t{info.width} + info.blockWidth - 1) / info.blockWidth);
    info.blocksDown = static_cast<uint32_t>((uint64_t{info.height} + info.blockHeight - 1) / info.blockHeight);
    info.planes = info.planarConfig == 2 ? info.samplesPerPixel : 1u;

    // GDAL's RGBA interface (GTiffRGBABand): 8-bit CIELab, LogL, LogLuv, CMYK and YCbCr without
    // JPEG compression (subsampled or not; old JPEG included) read as libtiff's TIFFReadRGBA*
    // converts them, 8-bit RGB (YCbCr) or RGBA (the others; the alpha band last), one block per
    // tile or strip whatever the planar configuration
    const uint16_t ph = info.photometric;
    const bool bitmap = info.bitsPerSample == 1 && info.samplesPerPixel == 1;
    if (info.kind != SubfileKind::kMask && info.kind != SubfileKind::kOverviewMask && !bitmap &&
        info.bitsPerSample <= 8 &&
        (ph == photometric::kCieLab || ph == photometric::kLogL || ph == photometric::kLogLuv ||
         ph == photometric::kSeparated || (ph == photometric::kYCbCr && info.compression != 7))) {
        info.viaRgba = true;
        info.fileSamplesPerPixel = info.samplesPerPixel;
        info.samplesPerPixel = ph == photometric::kYCbCr ? 3 : 4;
        info.bitsPerSample = 8;
        info.sampleFormat = 1;
        info.sampleType = SampleType::kU8;
        info.complex = false;
        info.planarConfig = 1;
        info.predictor = 1;
        info.fillOrder = 1;
        info.photometric = photometric::kRgb;
        info.extraSamples.clear();
        if (info.samplesPerPixel == 4) info.extraSamples.push_back(extrasample::kUnassociatedAlpha);
        info.colorMap.clear();
        info.planes = 1;
    }
    uint64_t blocks = 0;
    if (!CheckedMul<uint64_t>(uint64_t{info.blocksAcross} * info.blocksDown, info.planes, &blocks) ||
        blocks > options_.maxBlocksPerIfd || blocks > UINT32_MAX) {
        return ResourceExhausted("too many blocks (" + std::to_string(info.blocksAcross) + " x " +
                                 std::to_string(info.blocksDown) + " x " + std::to_string(info.planes) + ")");
    }
    info.blockCount = static_cast<uint32_t>(blocks);
    return Status::Ok();
}

// ---------------------------------------------------------------------------------------------
// Tag values

Expected<std::vector<uint8_t>> TiffReader::ReadTagBytes(const TagEntry& e, uint64_t maxBytes) const {
    if (TagTypeSize(e.type) == 0) return Unsupported("tag " + std::to_string(e.tag) + ": unknown field type " + std::to_string(e.type));
    const uint64_t size = e.ByteSize();
    if (e.count != 0 && size == 0) return OverflowError("tag " + std::to_string(e.tag) + ": size overflow");
    if (size > maxBytes) {
        return ResourceExhausted("tag " + std::to_string(e.tag) + ": " + std::to_string(size) + " bytes (limit " +
                                 std::to_string(maxBytes) + ")");
    }
    std::vector<uint8_t> out(static_cast<size_t>(size));
    if (size == 0) return out;
    if (e.isInline) {
        std::memcpy(out.data(), e.inlineBytes.data(), out.size());
        return out;
    }
    if (!RangeInside(e.valueOffset, size, source_->Size())) {
        return Corrupt("tag " + std::to_string(e.tag) + ": values past the end of the file");
    }
    DDBT_TRY(source_->Read(e.valueOffset, std::span<uint8_t>(out)));
    return out;
}

Expected<std::vector<uint64_t>> TiffReader::ReadTagUInts(const TagEntry& e, uint64_t maxCount) const {
    if (!IsIntegerType(e.type)) return Corrupt("tag " + std::to_string(e.tag) + ": not an integer type");
    if (e.count > maxCount) {
        return ResourceExhausted("tag " + std::to_string(e.tag) + ": " + std::to_string(e.count) + " values (limit " +
                                 std::to_string(maxCount) + ")");
    }
    auto bytes = ReadTagBytes(e, std::numeric_limits<uint64_t>::max());
    if (!bytes.ok()) return std::move(bytes).TakeStatus();
    const size_t ts = TagTypeSize(e.type);
    std::vector<uint64_t> out(static_cast<size_t>(e.count));
    for (size_t i = 0; i < out.size(); ++i) out[i] = DecodeIntValue(bytes->data() + i * ts, e.type, bigEndian_);
    return out;
}

Expected<std::vector<double>> TiffReader::ReadTagDoubles(const TagEntry& e, uint64_t maxCount) const {
    if (e.type == static_cast<uint16_t>(TagType::kAscii)) return Corrupt("tag " + std::to_string(e.tag) + ": ASCII, not numeric");
    if (e.count > maxCount) {
        return ResourceExhausted("tag " + std::to_string(e.tag) + ": " + std::to_string(e.count) + " values (limit " +
                                 std::to_string(maxCount) + ")");
    }
    auto bytes = ReadTagBytes(e, std::numeric_limits<uint64_t>::max());
    if (!bytes.ok()) return std::move(bytes).TakeStatus();
    const size_t ts = TagTypeSize(e.type);
    std::vector<double> out(static_cast<size_t>(e.count));
    for (size_t i = 0; i < out.size(); ++i) out[i] = DecodeDoubleValue(bytes->data() + i * ts, e.type, bigEndian_);
    return out;
}

Expected<std::string> TiffReader::ReadTagString(const TagEntry& e, uint64_t maxBytes) const {
    if (TagTypeSize(e.type) != 1) return Corrupt("tag " + std::to_string(e.tag) + ": not a string");
    auto bytes = ReadTagBytes(e, maxBytes);
    if (!bytes.ok()) return std::move(bytes).TakeStatus();
    std::string s(bytes->begin(), bytes->end());
    const size_t nul = s.find('\0');
    if (nul != std::string::npos) s.resize(nul);
    return s;
}

// ---------------------------------------------------------------------------------------------
// Block index

Status TiffReader::ReadBlockIndex(uint32_t ifd, IfdState& st) const {
    const IfdInfo& info = ifds_[ifd];
    if (!st.hasOffsets) return Corrupt(std::string(info.tiled ? "TileOffsets" : "StripOffsets") + " missing");
    if (!st.hasCounts) return Corrupt(std::string(info.tiled ? "TileByteCounts" : "StripByteCounts") + " missing");
    const uint64_t limit = std::max<uint64_t>(info.blockCount, 1) + 0;
    auto load = [&](const TagEntry& e, std::vector<uint64_t>& out) -> Status {
        if (!IsIntegerType(e.type)) return Corrupt("block index tag " + std::to_string(e.tag) + " is not an integer array");
        // Extra entries are ignored (libtiff trims); missing ones read as 0 (sparse).
        TagEntry trimmed = e;
        if (trimmed.count > limit) trimmed.count = limit;
        auto v = ReadTagUInts(trimmed, limit);
        if (!v.ok()) return v.status();
        out = std::move(*v);
        out.resize(info.blockCount, 0);
        return Status::Ok();
    };
    DDBT_TRY(load(st.offsetsTag, st.offsets));
    DDBT_TRY(load(st.countsTag, st.counts));
    return Status::Ok();
}

Status TiffReader::LoadBlockIndex(uint32_t ifd) const {
    if (ifd >= ifds_.size()) return OutOfRange("IFD " + std::to_string(ifd) + " does not exist");
    IfdState& st = *states_[ifd];
    if (st.mapped) return Status::Ok();
    std::call_once(st.once, [&] {
        st.loadStatus = ReadBlockIndex(ifd, st);
        st.loaded.store(true, std::memory_order_release);
    });
    return st.loadStatus;
}

bool TiffReader::BlockIndexLoaded(uint32_t ifd) const {
    return ifd < states_.size() && states_[ifd]->loaded.load(std::memory_order_acquire);
}

Expected<BlockLocation> TiffReader::LocateMapped(const IfdInfo& info, const IfdState& st, uint32_t index) const {
    (void)info;
    const uint8_t* base = source_->Data();
    auto entry = [&](const TagEntry& e) -> uint64_t {
        if (index >= e.count) return 0;
        const size_t ts = TagTypeSize(e.type);
        const uint8_t* p = e.isInline ? e.inlineBytes.data() + static_cast<size_t>(index) * ts
                                      : base + e.valueOffset + static_cast<uint64_t>(index) * ts;
        return DecodeIntValue(p, e.type, bigEndian_);
    };
    return BlockLocation{entry(st.offsetsTag), entry(st.countsTag)};
}

Expected<BlockLocation> TiffReader::Locate(uint32_t ifd, uint32_t index) const {
    if (ifd >= ifds_.size()) return OutOfRange("IFD " + std::to_string(ifd) + " does not exist");
    const IfdInfo& info = ifds_[ifd];
    if (index >= info.blockCount) {
        return OutOfRange("block " + std::to_string(index) + " of IFD " + std::to_string(ifd) + " (" +
                          std::to_string(info.blockCount) + " blocks)");
    }
    const IfdState& st = *states_[ifd];
    if (st.mapped) return LocateMapped(info, st, index);
    DDBT_TRY(LoadBlockIndex(ifd));
    return BlockLocation{st.offsets[index], st.counts[index]};
}

Expected<bool> TiffReader::IsSparse(uint32_t ifd, uint32_t index) const {
    auto loc = Locate(ifd, index);
    if (!loc.ok()) return std::move(loc).TakeStatus();
    return loc->IsSparse();
}

std::vector<uint32_t> TiffReader::Overviews() const {
    std::vector<uint32_t> out;
    for (const IfdInfo& i : ifds_) {
        if (i.kind == SubfileKind::kOverview) out.push_back(i.index);
    }
    std::stable_sort(out.begin(), out.end(), [&](uint32_t a, uint32_t b) { return ifds_[a].width > ifds_[b].width; });
    return out;
}

std::optional<uint32_t> TiffReader::MaskOf(uint32_t ifd) const {
    if (ifd >= ifds_.size()) return std::nullopt;
    const IfdInfo& img = ifds_[ifd];
    SubfileKind want;
    if (img.kind == SubfileKind::kFullResolution || img.kind == SubfileKind::kPage) want = SubfileKind::kMask;
    else if (img.kind == SubfileKind::kOverview) want = SubfileKind::kOverviewMask;
    else return std::nullopt;
    for (const IfdInfo& i : ifds_) {
        if (i.kind == want && i.width == img.width && i.height == img.height) return i.index;
    }
    return std::nullopt;
}

}  // namespace ddbt::cog::tiff
