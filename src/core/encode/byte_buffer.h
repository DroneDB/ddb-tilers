// SPDX-License-Identifier: MPL-2.0
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>
#include <utility>

namespace ddbt::encode {

/// Growable byte buffer on malloc/realloc/free: the bytes released by release() are freed with
/// std::free (ddbtFree). The encoders write into one and keep its capacity, so a buffer reused
/// across tiles allocates once.
class ByteBuffer {
public:
    ByteBuffer() = default;
    ByteBuffer(const ByteBuffer&) = delete;
    ByteBuffer& operator=(const ByteBuffer&) = delete;
    ByteBuffer(ByteBuffer&& other) noexcept
        : data_(std::exchange(other.data_, nullptr)),
          size_(std::exchange(other.size_, 0)),
          capacity_(std::exchange(other.capacity_, 0)) {}
    ByteBuffer& operator=(ByteBuffer&& other) noexcept {
        if (this != &other) {
            std::free(data_);
            data_ = std::exchange(other.data_, nullptr);
            size_ = std::exchange(other.size_, 0);
            capacity_ = std::exchange(other.capacity_, 0);
        }
        return *this;
    }
    ~ByteBuffer() { std::free(data_); }

    uint8_t* data() noexcept { return data_; }
    const uint8_t* data() const noexcept { return data_; }
    size_t size() const noexcept { return size_; }
    size_t capacity() const noexcept { return capacity_; }
    bool empty() const noexcept { return size_ == 0; }

    void clear() noexcept { size_ = 0; }

    /// Capacity of at least n bytes (contents kept). Throws std::bad_alloc.
    void reserve(size_t n) {
        if (n <= capacity_) return;
        void* p = std::realloc(data_, n);
        if (p == nullptr) throw std::bad_alloc();
        data_ = static_cast<uint8_t*>(p);
        capacity_ = n;
    }

    /// Size n; new bytes are uninitialized.
    void resize(size_t n) {
        if (n > capacity_) reserve(Grown(n));
        size_ = n;
    }

    void append(const void* src, size_t n) {
        if (n == 0) return;
        const size_t at = size_;
        resize(size_ + n);
        std::memcpy(data_ + at, src, n);
    }

    void assign(const void* src, size_t n) {
        clear();
        append(src, n);
    }

    /// Hands the bytes to the caller (free with std::free / ddbtFree); the buffer is left empty.
    /// The allocation is trimmed to the size first, so a tile does not pin its worst-case bound.
    uint8_t* release() noexcept {
        uint8_t* p = data_;
        if (p != nullptr && size_ < capacity_) {
            // Shrinking realloc: on failure the original block stays valid.
            if (void* q = std::realloc(p, size_ == 0 ? 1 : size_)) p = static_cast<uint8_t*>(q);
        }
        data_ = nullptr;
        size_ = 0;
        capacity_ = 0;
        return p;
    }

private:
    size_t Grown(size_t n) const noexcept {
        const size_t doubled = capacity_ + capacity_ / 2;
        return doubled > n ? doubled : n;
    }

    uint8_t* data_ = nullptr;
    size_t size_ = 0;
    size_t capacity_ = 0;
};

}  // namespace ddbt::encode
