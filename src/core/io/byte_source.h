// SPDX-License-Identifier: MPL-2.0
#pragma once

// ByteSource: random-access, thread-safe, read-only bytes (a local file through pread or a
// mapping, memory, or a remote object through a user range callback). Readers (TIFF, COPC) are
// written against this interface only.

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "core/io/file.h"
#include "core/io/mapped_file.h"
#include "core/util/status.h"

namespace ddbt::io {

class ByteSource {
public:
    virtual ~ByteSource() = default;

    /// Total size in bytes.
    virtual uint64_t Size() const noexcept = 0;

    /// Reads exactly out.size() bytes at `offset`. OutOfRange when the range is not inside the
    /// source. Safe to call concurrently from any number of threads.
    virtual Status Read(uint64_t offset, std::span<uint8_t> out) const = 0;

    /// The whole content when it is memory-resident (mapping, memory), else nullptr: readers can
    /// then decode in place without copying.
    virtual const uint8_t* Data() const noexcept { return nullptr; }

    /// Hint that [offset, offset + length) will be read soon. Default: no-op.
    virtual void Prefetch(uint64_t offset, uint64_t length) const noexcept {
        (void)offset;
        (void)length;
    }

    /// For error messages (path or URL).
    virtual std::string_view Name() const noexcept = 0;

    /// Read into a new vector.
    Expected<std::vector<uint8_t>> ReadVector(uint64_t offset, size_t length) const;

    /// Zero-copy view when Data() is available and the range is inside, else an empty span.
    std::span<const uint8_t> View(uint64_t offset, size_t length) const noexcept;

protected:
    ByteSource() = default;
    ByteSource(const ByteSource&) = default;
    ByteSource& operator=(const ByteSource&) = default;
};

/// Positional reads on a File.
class FileSource final : public ByteSource {
public:
    explicit FileSource(File file) noexcept : file_(std::move(file)) {}
    static Expected<std::unique_ptr<FileSource>> Open(std::string_view utf8Path, AccessHint hint = AccessHint::kRandom);

    uint64_t Size() const noexcept override { return file_.Size(); }
    Status Read(uint64_t offset, std::span<uint8_t> out) const override { return file_.ReadAt(offset, out); }
    void Prefetch(uint64_t offset, uint64_t length) const noexcept override { file_.Prefetch(offset, length); }
    std::string_view Name() const noexcept override { return file_.Path(); }
    const File& file() const noexcept { return file_; }

private:
    File file_;
};

/// Reads served by memcpy from a mapping (Data() is available).
class MappedSource final : public ByteSource {
public:
    explicit MappedSource(MappedFile map) noexcept : map_(std::move(map)) {}
    static Expected<std::unique_ptr<MappedSource>> Open(std::string_view utf8Path, AccessHint hint = AccessHint::kRandom);

    uint64_t Size() const noexcept override { return map_.size(); }
    Status Read(uint64_t offset, std::span<uint8_t> out) const override;
    const uint8_t* Data() const noexcept override { return map_.data(); }
    void Prefetch(uint64_t offset, uint64_t length) const noexcept override { map_.Prefetch(offset, length); }
    std::string_view Name() const noexcept override { return map_.Path(); }
    const MappedFile& map() const noexcept { return map_; }

private:
    MappedFile map_;
};

/// Bytes in memory, owned (vector) or borrowed (span that must outlive the source).
class MemorySource final : public ByteSource {
public:
    explicit MemorySource(std::vector<uint8_t> bytes, std::string name = "<memory>");
    explicit MemorySource(std::span<const uint8_t> borrowed, std::string name = "<memory>") noexcept;

    uint64_t Size() const noexcept override { return view_.size(); }
    Status Read(uint64_t offset, std::span<uint8_t> out) const override;
    const uint8_t* Data() const noexcept override { return view_.data(); }
    std::string_view Name() const noexcept override { return name_; }

private:
    std::vector<uint8_t> owned_;
    std::span<const uint8_t> view_;
    std::string name_;
};

/// Range reads through a user callback (HTTP range requests, object stores). The callback must
/// fill `out` completely or return an error; it is called concurrently from several threads.
class RangeCallbackSource final : public ByteSource {
public:
    using ReadFn = std::function<Status(uint64_t offset, std::span<uint8_t> out)>;
    using PrefetchFn = std::function<void(uint64_t offset, uint64_t length)>;

    RangeCallbackSource(uint64_t size, ReadFn read, std::string name = "<remote>", PrefetchFn prefetch = {});

    uint64_t Size() const noexcept override { return size_; }
    Status Read(uint64_t offset, std::span<uint8_t> out) const override;
    void Prefetch(uint64_t offset, uint64_t length) const noexcept override;
    std::string_view Name() const noexcept override { return name_; }

private:
    uint64_t size_;
    ReadFn read_;
    PrefetchFn prefetch_;
    std::string name_;
};

enum class SourceKind : uint8_t {
    kAuto,    // mmap on Linux / macOS, pread on Windows
    kPread,
    kMmap,
};

/// Opens a local file as a ByteSource of the requested kind.
Expected<std::unique_ptr<ByteSource>> OpenByteSource(std::string_view utf8Path, SourceKind kind = SourceKind::kAuto,
                                                     AccessHint hint = AccessHint::kRandom);

}  // namespace ddbt::io
