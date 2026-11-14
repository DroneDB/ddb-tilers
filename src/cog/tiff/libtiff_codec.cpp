// SPDX-License-Identifier: MPL-2.0
#include "cog/tiff/libtiff_codec.h"

#include <tiffio.h>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

namespace ddbt::cog::tiff {

namespace {

// libtiff reads the file through the source; the handle is used under its lock, so a position
// suffices.
struct Client {
    std::shared_ptr<const io::ByteSource> source;
    uint64_t position = 0;
};

tmsize_t ReadProc(thandle_t h, void* buf, tmsize_t size) {
    auto* c = static_cast<Client*>(h);
    const uint64_t total = c->source->Size();
    if (size <= 0 || c->position >= total) return 0;
    const uint64_t n = std::min<uint64_t>(static_cast<uint64_t>(size), total - c->position);
    if (!c->source->Read(c->position, std::span<uint8_t>(static_cast<uint8_t*>(buf), static_cast<size_t>(n))).ok())
        return -1;
    c->position += n;
    return static_cast<tmsize_t>(n);
}

tmsize_t WriteProc(thandle_t, void*, tmsize_t) { return -1; }

toff_t SeekProc(thandle_t h, toff_t off, int whence) {
    auto* c = static_cast<Client*>(h);
    switch (whence) {
        case SEEK_SET: c->position = off; break;
        case SEEK_CUR: c->position += off; break;
        case SEEK_END: c->position = c->source->Size() + off; break;
        default: return static_cast<toff_t>(-1);
    }
    return c->position;
}

int CloseProc(thandle_t) { return 0; }
toff_t SizeProc(thandle_t h) { return static_cast<Client*>(h)->source->Size(); }
int MapProc(thandle_t, void**, toff_t*) { return 0; }
void UnmapProc(thandle_t, void*, toff_t) {}

// The last error libtiff reported on this thread (for the message of a failed decode)
thread_local std::string tlsError;

int ErrorHandler(TIFF*, void*, const char* module, const char* fmt, va_list ap) {
    char text[512];
    std::vsnprintf(text, sizeof text, fmt, ap);
    tlsError = std::string(module != nullptr ? module : "libtiff") + ": " + text;
    return 1;
}

int WarningHandler(TIFF*, void*, const char*, const char*, va_list) { return 1; }

}  // namespace

struct LibtiffCodec::Handle {
    Client client;
    TIFF* tif = nullptr;
    uint64_t directory = 0;
    std::mutex mutex;
    std::vector<uint8_t> input;  // TIFFReadFromUserBuffer reverses the bits of FillOrder 2 in place
};

bool LibtiffCodec::Supports(uint16_t compression) noexcept { return TIFFIsCODECConfigured(compression) != 0; }

LibtiffCodec::LibtiffCodec(std::shared_ptr<const io::ByteSource> source) : handle_(std::make_unique<Handle>()) {
    handle_->client.source = std::move(source);
}

LibtiffCodec::~LibtiffCodec() {
    if (handle_ && handle_->tif != nullptr) TIFFClose(handle_->tif);
}

Status LibtiffCodec::SelectDirectory(uint64_t ifdOffset) const {
    Handle& h = *handle_;
    if (h.tif == nullptr) {
        TIFFOpenOptions* options = TIFFOpenOptionsAlloc();
        TIFFOpenOptionsSetErrorHandlerExtR(options, ErrorHandler, nullptr);
        TIFFOpenOptionsSetWarningHandlerExtR(options, WarningHandler, nullptr);
        // "h": the header only (the directory is chosen below), "m": no mapping (the source reads)
        h.tif = TIFFClientOpenExt(std::string(h.client.source->Name()).c_str(), "rhm", &h.client, ReadProc,
                                  WriteProc, SeekProc, CloseProc, SizeProc, MapProc, UnmapProc, options);
        TIFFOpenOptionsFree(options);
        if (h.tif == nullptr) return Corrupt("libtiff cannot open the file: " + tlsError);
        h.directory = 0;
    }
    if (h.directory != ifdOffset) {
        if (!TIFFSetSubDirectory(h.tif, ifdOffset)) {
            h.directory = 0;
            return Corrupt("libtiff cannot read the directory at " + std::to_string(ifdOffset) + ": " + tlsError);
        }
        h.directory = ifdOffset;
    }
    return Status::Ok();
}

Status LibtiffCodec::Decode(uint64_t ifdOffset, uint32_t index, std::span<const uint8_t> raw,
                            std::span<uint8_t> out) const {
    Handle& h = *handle_;
    std::lock_guard lock(h.mutex);
    tlsError.clear();
    DDBT_TRY(SelectDirectory(ifdOffset));
    h.input.assign(raw.begin(), raw.end());
    if (!TIFFReadFromUserBuffer(h.tif, index, h.input.data(), static_cast<tmsize_t>(h.input.size()), out.data(),
                                static_cast<tmsize_t>(out.size())))
        return Corrupt("libtiff cannot decode block " + std::to_string(index) + ": " + tlsError);
    return Status::Ok();
}

Status LibtiffCodec::DecodeRgba(uint64_t ifdOffset, bool tiled, uint32_t x, uint32_t y, uint32_t blockWidth,
                                uint32_t blockHeight, uint32_t rows, uint32_t bands, uint8_t* out, size_t outStride) const {
    Handle& h = *handle_;
    std::lock_guard lock(h.mutex);
    tlsError.clear();
    DDBT_TRY(SelectDirectory(ifdOffset));
    thread_local std::vector<uint32_t> raster;
    raster.assign(static_cast<size_t>(blockWidth) * blockHeight, 0);
    const int ok = tiled ? TIFFReadRGBATile(h.tif, x, y, raster.data()) : TIFFReadRGBAStrip(h.tif, y, raster.data());
    if (!ok) return Corrupt("libtiff cannot decode the RGBA block at (" + std::to_string(x) + ", " + std::to_string(y) + "): " + tlsError);
    // The raster's origin is its lower left: a tile is flipped over its height, a strip over its rows
    const uint32_t flip = tiled ? blockHeight : rows;
    for (uint32_t r = 0; r < rows; ++r) {
        const uint32_t* src = raster.data() + static_cast<size_t>(flip - r - 1) * blockWidth;
        uint8_t* dst = out + static_cast<size_t>(r) * outStride;
        for (uint32_t c = 0; c < blockWidth; ++c) {
            const uint32_t p = src[c];
            dst[c * bands + 0] = static_cast<uint8_t>(TIFFGetR(p));
            dst[c * bands + 1] = static_cast<uint8_t>(TIFFGetG(p));
            dst[c * bands + 2] = static_cast<uint8_t>(TIFFGetB(p));
            if (bands == 4) dst[c * bands + 3] = static_cast<uint8_t>(TIFFGetA(p));
        }
    }
    return Status::Ok();
}

}  // namespace ddbt::cog::tiff
