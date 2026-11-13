// SPDX-License-Identifier: MPL-2.0
#include "core/mem/arena.h"

#include <algorithm>

#include "core/mem/aligned.h"
#include "core/mem/mem_budget.h"

namespace ddbt::mem {

Arena::Arena(ArenaOptions options) : options_(options) {
    if (options_.chunkSize < 4096) options_.chunkSize = 4096;
}

Arena::Arena(Arena&& other) noexcept
    : options_(other.options_),
      chunks_(std::move(other.chunks_)),
      dedicated_(std::move(other.dedicated_)),
      current_(std::exchange(other.current_, 0)),
      cur_(std::exchange(other.cur_, nullptr)),
      end_(std::exchange(other.end_, nullptr)),
      used_(std::exchange(other.used_, 0)),
      reserved_(std::exchange(other.reserved_, 0)) {
    other.chunks_.clear();
    other.dedicated_.clear();
}

Arena& Arena::operator=(Arena&& other) noexcept {
    if (this != &other) {
        Release();
        options_ = other.options_;
        chunks_ = std::move(other.chunks_);
        dedicated_ = std::move(other.dedicated_);
        current_ = std::exchange(other.current_, 0);
        cur_ = std::exchange(other.cur_, nullptr);
        end_ = std::exchange(other.end_, nullptr);
        used_ = std::exchange(other.used_, 0);
        reserved_ = std::exchange(other.reserved_, 0);
        other.chunks_.clear();
        other.dedicated_.clear();
    }
    return *this;
}

Arena::~Arena() { Release(); }

Arena::Chunk Arena::NewChunk(size_t size) {
    Chunk c{};
    c.size = size;
    c.large = options_.hugePages && size >= kHugePageSize;
    if (c.large) {
        c.data = static_cast<uint8_t*>(LargeAlloc(size, true));
    } else {
        c.data = static_cast<uint8_t*>(AlignedAlloc(size, kCacheLine));
    }
    reserved_ += size;
    if (options_.budget != nullptr) options_.budget->ForceReserve(size);
    return c;
}

void Arena::FreeChunk(const Chunk& c) noexcept {
    if (c.large) LargeFree(c.data, c.size, true);
    else AlignedFree(c.data);
    reserved_ -= c.size;
    if (options_.budget != nullptr) options_.budget->Release(c.size);
}

void* Arena::AllocateSlow(size_t size, size_t alignment) {
    if (alignment == 0 || (alignment & (alignment - 1)) != 0) throw std::bad_alloc();
    // Oversized: its own chunk, so a big request does not waste the rest of a regular chunk.
    if (size > options_.chunkSize / 4 || alignment > kCacheLine) {
        if (size > SIZE_MAX - alignment) throw std::bad_alloc();
        const size_t need = size + (alignment > kCacheLine ? alignment : 0);
        dedicated_.reserve(dedicated_.size() + 1);
        Chunk c = NewChunk(std::max<size_t>(need, 1));
        dedicated_.push_back(c);
        const uintptr_t p = (reinterpret_cast<uintptr_t>(c.data) + (alignment - 1)) & ~(uintptr_t{alignment} - 1);
        used_ += size;
        return reinterpret_cast<void*>(p);
    }
    // Next regular chunk: reuse one kept by Reset(), else allocate.
    if (cur_ != nullptr) ++current_;
    if (current_ >= chunks_.size()) {
        chunks_.reserve(chunks_.size() + 1);
        chunks_.push_back(NewChunk(options_.chunkSize));
        current_ = chunks_.size() - 1;
    }
    cur_ = chunks_[current_].data;
    end_ = cur_ + chunks_[current_].size;
    // A fresh chunk is cache-line aligned and size <= chunkSize / 4: always fits.
    const uintptr_t p = (reinterpret_cast<uintptr_t>(cur_) + (alignment - 1)) & ~(uintptr_t{alignment} - 1);
    cur_ = reinterpret_cast<uint8_t*>(p + size);
    used_ += size;
    return reinterpret_cast<void*>(p);
}

void Arena::Reset() noexcept {
    for (const Chunk& c : dedicated_) FreeChunk(c);
    dedicated_.clear();
    current_ = 0;
    used_ = 0;
    if (chunks_.empty()) {
        cur_ = end_ = nullptr;
    } else {
        cur_ = chunks_[0].data;
        end_ = cur_ + chunks_[0].size;
    }
}

void Arena::Release() noexcept {
    for (const Chunk& c : dedicated_) FreeChunk(c);
    for (const Chunk& c : chunks_) FreeChunk(c);
    dedicated_.clear();
    chunks_.clear();
    current_ = 0;
    cur_ = end_ = nullptr;
    used_ = 0;
}

}  // namespace ddbt::mem
