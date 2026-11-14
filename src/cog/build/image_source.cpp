// SPDX-License-Identifier: MPL-2.0
#include "cog/build/image_source.h"

#include <algorithm>
#include <cstring>
#include <exception>
#include <filesystem>
#include <limits>
#include <new>
#include <span>

namespace ddbt::cog::build {

namespace {

// The rows Decode() writes, into memory or into the scratch file.
class StoreSink final : public RowSink {
public:
    StoreSink(uint8_t* pixels, uint8_t* mask, io::ScratchFile* file, uint64_t maskOffset, size_t rowBytes,
              size_t maskRowBytes, uint32_t height)
        : pixels_(pixels),
          mask_(mask),
          file_(file),
          maskOffset_(maskOffset),
          rowBytes_(rowBytes),
          maskRowBytes_(maskRowBytes),
          height_(height) {}

    Status Row(uint32_t y, const uint8_t* samples) override {
        if (y >= height_) return Status(StatusCode::kInternal, "image row out of range");
        rows_ = std::max(rows_, y + 1);
        if (file_ != nullptr) return file_->WriteAt(static_cast<uint64_t>(y) * rowBytes_, {samples, rowBytes_});
        std::memcpy(pixels_ + static_cast<size_t>(y) * rowBytes_, samples, rowBytes_);
        return Status::Ok();
    }
    Status MaskRow(uint32_t y, const uint8_t* mask) override {
        if (y >= height_ || maskRowBytes_ == 0) return Status(StatusCode::kInternal, "mask row out of range");
        maskRows_ = std::max(maskRows_, y + 1);
        if (file_ != nullptr)
            return file_->WriteAt(maskOffset_ + static_cast<uint64_t>(y) * maskRowBytes_, {mask, maskRowBytes_});
        std::memcpy(mask_ + static_cast<size_t>(y) * maskRowBytes_, mask, maskRowBytes_);
        return Status::Ok();
    }
    bool Complete() const { return rows_ == height_ && (maskRowBytes_ == 0 || maskRows_ == height_); }

private:
    uint8_t* pixels_;
    uint8_t* mask_;
    io::ScratchFile* file_;
    uint64_t maskOffset_;
    size_t rowBytes_, maskRowBytes_;
    uint32_t height_;
    uint32_t rows_ = 0, maskRows_ = 0;
};

}  // namespace

Status ImageSource::SetInfo(ImageInfo info) {
    if (info.width == 0 || info.height == 0 || info.bands <= 0 || tiff::SampleTypeSize(info.type) == 0)
        return Status(StatusCode::kCorrupt, "Invalid image dimensions in " + path_);
    info_ = std::move(info);
    info_.interps.resize(static_cast<size_t>(info_.bands), "Undefined");
    // Strips of about 4 MiB, 256 rows at most.
    const size_t rowBytes = RowBytes();
    blockHeight_ = static_cast<uint32_t>(std::clamp<size_t>((size_t{4} << 20) / std::max<size_t>(rowBytes, 1), 1, 256));
    blockHeight_ = std::min(blockHeight_, info_.height);
    return Status::Ok();
}

std::string ImageSource::ColorInterp(int b) const {
    if (b < 0 || b >= info_.bands) return "Undefined";
    return info_.interps[static_cast<size_t>(b)];
}

std::optional<int> ImageSource::Nbits(int b) const {
    if (b < 0 || b >= info_.bands) return std::nullopt;
    return info_.nbits;
}

Status ImageSource::Load() const {
    std::call_once(once_, [this] {
        try {
            const size_t rowBytes = RowBytes();
            const size_t maskRowBytes = info_.mask ? info_.width : 0;
            const uint64_t bytes = (static_cast<uint64_t>(rowBytes) + maskRowBytes) * info_.height;
            io::ScratchFile* file = nullptr;
            if (bytes > memoryBudget_ / 2 || bytes > std::numeric_limits<size_t>::max() / 2) {
                // Larger than half the budget: the rows go to a scratch file.
                std::error_code ec;
                const std::filesystem::path dir = std::filesystem::temp_directory_path(ec);
                auto f = io::ScratchFile::Create(((ec ? std::filesystem::path(".") : dir) / "ddbt-image").string(),
                                                 io::ScratchKind::kAnonymous);
                if (!f.ok()) {
                    loadStatus_ = f.status();
                    return;
                }
                file_ = std::move(*f);
                file_.Preallocate(bytes);
                file = &file_;
                spilled_ = true;
                maskOffset_ = static_cast<uint64_t>(rowBytes) * info_.height;
            } else {
                pixels_ = LargeBytes(rowBytes * info_.height);
                if (maskRowBytes != 0) mask_ = LargeBytes(maskRowBytes * info_.height);
            }
            StoreSink sink(pixels_.get(), mask_.get(), file, maskOffset_, rowBytes, maskRowBytes, info_.height);
            Status s = Decode(sink);
            if (s.ok() && !sink.Complete()) s = Status(StatusCode::kCorrupt, "Incomplete image in " + path_);
            if (!s.ok()) {
                pixels_.Reset();
                mask_.Reset();
                file_.Discard();
            }
            loadStatus_ = std::move(s);
        } catch (const std::bad_alloc&) {
            loadStatus_ = Status(StatusCode::kResourceExhausted, "Out of memory decoding " + path_);
        } catch (const std::exception& e) {
            loadStatus_ = Status(StatusCode::kInternal, e.what());
        }
    });
    return loadStatus_;
}

Expected<uint32_t> ImageSource::ReadBlock(uint32_t col, uint32_t row, uint8_t* out) const {
    if (col != 0 || row >= BlocksDown()) return Status(StatusCode::kOutOfRange, "block out of range");
    DDBT_TRY(Load());
    const uint32_t y0 = row * blockHeight_;
    const uint32_t rows = std::min(blockHeight_, info_.height - y0);
    const size_t rowBytes = RowBytes();
    const size_t n = static_cast<size_t>(rows) * rowBytes;
    if (spilled_) {
        DDBT_TRY(file_.ReadAt(static_cast<uint64_t>(y0) * rowBytes, {out, n}));
    } else {
        std::memcpy(out, pixels_.get() + static_cast<size_t>(y0) * rowBytes, n);
    }
    return rows;
}

const uint8_t* ImageSource::ResidentBlock(uint32_t col, uint32_t row) const {
    if (col != 0 || row >= BlocksDown() || !Load().ok() || spilled_) return nullptr;
    return pixels_.get() + static_cast<size_t>(row) * blockHeight_ * RowBytes();
}

const uint8_t* ImageSource::ResidentMaskBlock(uint32_t col, uint32_t row) const {
    if (!info_.mask || col != 0 || row >= BlocksDown() || !Load().ok() || spilled_) return nullptr;
    return mask_.get() + static_cast<size_t>(row) * blockHeight_ * info_.width;
}

Status ImageSource::ReadMaskBlock(uint32_t col, uint32_t row, uint8_t* out) const {
    if (!info_.mask) return Status(StatusCode::kInvalidArgument, "no mask");
    if (col != 0 || row >= BlocksDown()) return Status(StatusCode::kOutOfRange, "block out of range");
    DDBT_TRY(Load());
    const uint32_t y0 = row * blockHeight_;
    const uint32_t rows = std::min(blockHeight_, info_.height - y0);
    const size_t n = static_cast<size_t>(rows) * info_.width;
    if (spilled_) return file_.ReadAt(maskOffset_ + static_cast<uint64_t>(y0) * info_.width, {out, n});
    std::memcpy(out, mask_.get() + static_cast<size_t>(y0) * info_.width, n);
    return Status::Ok();
}

}  // namespace ddbt::cog::build
