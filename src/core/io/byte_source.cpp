// SPDX-License-Identifier: MPL-2.0
#include "core/io/byte_source.h"

#include <cstring>

#include "core/io/io_stats.h"
#include "core/util/checked_math.h"

namespace ddbt::io {

namespace {
Status OutOfRangeRead(std::string_view name, uint64_t offset, size_t length, uint64_t size) {
    return OutOfRange("read of " + std::to_string(length) + " bytes at " + std::to_string(offset) + " past the end of '" +
                      std::string(name) + "' (" + std::to_string(size) + " bytes)");
}
}  // namespace

Expected<std::vector<uint8_t>> ByteSource::ReadVector(uint64_t offset, size_t length) const {
    if (!RangeInside(offset, length, Size())) return OutOfRangeRead(Name(), offset, length, Size());
    std::vector<uint8_t> v(length);
    DDBT_TRY(Read(offset, v));
    return v;
}

std::span<const uint8_t> ByteSource::View(uint64_t offset, size_t length) const noexcept {
    const uint8_t* d = Data();
    if (d == nullptr || !RangeInside(offset, length, Size())) return {};
    return {d + offset, length};
}

Expected<std::unique_ptr<FileSource>> FileSource::Open(std::string_view utf8Path, AccessHint hint) {
    DDBT_ASSIGN_OR_RETURN(File f, File::Open(utf8Path, hint));
    return std::make_unique<FileSource>(std::move(f));
}

Expected<std::unique_ptr<MappedSource>> MappedSource::Open(std::string_view utf8Path, AccessHint hint) {
    DDBT_ASSIGN_OR_RETURN(MappedFile m, MappedFile::Open(utf8Path, hint));
    return std::make_unique<MappedSource>(std::move(m));
}

Status MappedSource::Read(uint64_t offset, std::span<uint8_t> out) const {
    if (!RangeInside(offset, out.size(), map_.size())) return OutOfRangeRead(Name(), offset, out.size(), map_.size());
    if (!out.empty()) std::memcpy(out.data(), map_.data() + offset, out.size());
    IoStats::AddMappedRead(out.size());
    return Status::Ok();
}

MemorySource::MemorySource(std::vector<uint8_t> bytes, std::string name)
    : owned_(std::move(bytes)), view_(owned_), name_(std::move(name)) {}

MemorySource::MemorySource(std::span<const uint8_t> borrowed, std::string name) noexcept
    : view_(borrowed), name_(std::move(name)) {}

Status MemorySource::Read(uint64_t offset, std::span<uint8_t> out) const {
    if (!RangeInside(offset, out.size(), view_.size())) return OutOfRangeRead(name_, offset, out.size(), view_.size());
    if (!out.empty()) std::memcpy(out.data(), view_.data() + offset, out.size());
    return Status::Ok();
}

RangeCallbackSource::RangeCallbackSource(uint64_t size, ReadFn read, std::string name, PrefetchFn prefetch)
    : size_(size), read_(std::move(read)), prefetch_(std::move(prefetch)), name_(std::move(name)) {}

Status RangeCallbackSource::Read(uint64_t offset, std::span<uint8_t> out) const {
    if (!RangeInside(offset, out.size(), size_)) return OutOfRangeRead(name_, offset, out.size(), size_);
    if (out.empty()) return Status::Ok();
    if (!read_) return InvalidArgument("RangeCallbackSource '" + name_ + "' has no read callback");
    Status s = read_(offset, out);
    if (!s.ok()) return s.WithContext("range read of '" + name_ + "'");
    IoStats::AddCallbackRead(out.size());
    return Status::Ok();
}

void RangeCallbackSource::Prefetch(uint64_t offset, uint64_t length) const noexcept {
    if (!prefetch_) return;
    try {
        prefetch_(offset, length);
        IoStats::AddPrefetch();
    } catch (...) {
        // A prefetch is only a hint: a throwing callback must not take the reader down.
    }
}

Expected<std::unique_ptr<ByteSource>> OpenByteSource(std::string_view utf8Path, SourceKind kind, AccessHint hint) {
    if (kind == SourceKind::kAuto) {
#ifdef _WIN32
        kind = SourceKind::kPread;
#else
        kind = SourceKind::kMmap;
#endif
    }
    if (kind == SourceKind::kMmap) {
        DDBT_ASSIGN_OR_RETURN(std::unique_ptr<MappedSource> m, MappedSource::Open(utf8Path, hint));
        return std::unique_ptr<ByteSource>(std::move(m));
    }
    DDBT_ASSIGN_OR_RETURN(std::unique_ptr<FileSource> f, FileSource::Open(utf8Path, hint));
    return std::unique_ptr<ByteSource>(std::move(f));
}

}  // namespace ddbt::io
