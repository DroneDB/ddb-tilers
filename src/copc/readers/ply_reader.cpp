// SPDX-License-Identifier: MPL-2.0
//
// PLY reader: "format ascii 1.0", "binary_little_endian 1.0", "binary_big_endian 1.0"; the
// vertex element's scalar properties (x, y, z required; red / green / blue, diffuse_*,
// intensity, classification and the other names fields.h knows; the rest ignored, as DroneDB's
// conversion to LAS drops them). Binary: tasks are vertex ranges read at a fixed stride. ASCII:
// tasks are newline-aligned byte ranges; a newline count gives each task its first line index
// so the vertex lines end where the element ends. 8-bit colour is scaled by 257.
#include "copc/readers/ply_reader.h"

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "copc/readers/fields.h"
#include "copc/readers/mapped_input.h"
#include "copc/readers/text_util.h"
#include "core/io/byte_source.h"
#include "core/util/endian.h"

namespace ddbt::copc::readers {

namespace {

constexpr size_t kBatch = 1u << 16;

enum class PType : uint8_t { kI8, kU8, kI16, kU16, kI32, kU32, kF32, kF64, kBad };

PType TypeOf(const std::string& t) {
    if (t == "char" || t == "int8") return PType::kI8;
    if (t == "uchar" || t == "uint8") return PType::kU8;
    if (t == "short" || t == "int16") return PType::kI16;
    if (t == "ushort" || t == "uint16") return PType::kU16;
    if (t == "int" || t == "int32") return PType::kI32;
    if (t == "uint" || t == "uint32") return PType::kU32;
    if (t == "float" || t == "float32") return PType::kF32;
    if (t == "double" || t == "float64") return PType::kF64;
    return PType::kBad;
}

size_t SizeOf(PType t) {
    switch (t) {
        case PType::kI8:
        case PType::kU8: return 1;
        case PType::kI16:
        case PType::kU16: return 2;
        case PType::kI32:
        case PType::kU32:
        case PType::kF32: return 4;
        case PType::kF64: return 8;
        case PType::kBad: break;
    }
    return 0;
}

template <bool kBig>
double LoadAs(PType t, const uint8_t* p) {
    auto ld = [p]<typename T>(T) { return kBig ? LoadBE<T>(p) : LoadLE<T>(p); };
    switch (t) {
        case PType::kI8: return static_cast<int8_t>(p[0]);
        case PType::kU8: return p[0];
        case PType::kI16: return ld(int16_t{});
        case PType::kU16: return ld(uint16_t{});
        case PType::kI32: return ld(int32_t{});
        case PType::kU32: return ld(uint32_t{});
        case PType::kF32: return ld(float{});
        case PType::kF64: return ld(double{});
        case PType::kBad: break;
    }
    return 0;
}

std::vector<std::string> Words(std::string_view line) {
    std::vector<std::string> w;
    size_t i = 0;
    while (i < line.size()) {
        while (i < line.size() && text::IsSpace(line[i])) ++i;
        size_t j = i;
        while (j < line.size() && !text::IsSpace(line[j])) ++j;
        if (j > i) w.emplace_back(line.substr(i, j - i));
        i = j;
    }
    return w;
}

}  // namespace

struct PlyReader::Prop {
    std::string name;
    PType type = PType::kBad;
    size_t offset = 0;
    Field field = Field::kNone;
};

PlyReader::PlyReader() = default;
PlyReader::~PlyReader() = default;

Expected<std::unique_ptr<PlyReader>> PlyReader::Open(const std::string& path, const ReaderOptions& options) {
    DDBT_ASSIGN_OR_RETURN(std::unique_ptr<io::ByteSource> src, OpenMappedInput(path, "PLY"));
    if (src->Size() == 0) return InvalidArgument(path + ": empty file");
    std::unique_ptr<PlyReader> r(new PlyReader());
    r->source_ = std::shared_ptr<const io::ByteSource>(std::move(src));
    const char* data = reinterpret_cast<const char*>(r->source_->Data());
    const uint64_t size = r->source_->Size();
    // Header lines up to "end_header".
    uint64_t pos = 0;
    std::vector<std::string> lines;
    for (;;) {
        if (pos >= size || lines.size() > 100000) return Corrupt(path + ": PLY header without end_header");
        const void* nl = std::memchr(data + pos, '\n', static_cast<size_t>(size - pos));
        if (nl == nullptr) return Corrupt(path + ": PLY header without end_header");
        const uint64_t e = static_cast<uint64_t>(static_cast<const char*>(nl) - data);
        std::string line(data + pos, static_cast<size_t>(e - pos));
        if (!line.empty() && line.back() == '\r') line.pop_back();
        pos = e + 1;
        if (lines.empty() && line != "ply") return Corrupt(path + ": not a PLY file");
        lines.push_back(line);
        if (line == "end_header") break;
    }
    struct Element {
        std::string name;
        uint64_t count = 0;
        size_t stride = 0;
        bool hasList = false;
    };
    std::vector<Element> elems;
    bool sawVertex = false;
    int vertexElem = -1;
    for (const std::string& line : lines) {
        const auto w = Words(line);
        if (w.empty()) continue;
        if (w[0] == "format") {
            if (w.size() < 2) return Corrupt(path + ": PLY format line");
            if (w[1] == "ascii") r->fmt_ = Fmt::kAscii;
            else if (w[1] == "binary_little_endian") r->fmt_ = Fmt::kLittle;
            else if (w[1] == "binary_big_endian") r->fmt_ = Fmt::kBig;
            else return Unsupported(path + ": PLY format " + w[1]);
        } else if (w[0] == "element") {
            if (w.size() < 3) return Corrupt(path + ": PLY element line");
            Element e;
            e.name = w[1];
            e.count = std::strtoull(w[2].c_str(), nullptr, 10);
            elems.push_back(e);
            if (e.name == "vertex" && !sawVertex) {
                sawVertex = true;
                vertexElem = static_cast<int>(elems.size() - 1);
            }
        } else if (w[0] == "property") {
            if (elems.empty()) return Corrupt(path + ": PLY property before any element");
            Element& e = elems.back();
            if (w.size() >= 2 && w[1] == "list") {
                e.hasList = true;
                if (static_cast<int>(elems.size() - 1) == vertexElem)
                    return Unsupported(path + ": PLY vertex element with a list property");
                continue;
            }
            if (w.size() < 3) return Corrupt(path + ": PLY property line");
            const PType t = TypeOf(w[1]);
            if (t == PType::kBad) return Unsupported(path + ": PLY property type " + w[1]);
            if (static_cast<int>(elems.size() - 1) == vertexElem) {
                Prop p;
                p.name = w[2];
                p.type = t;
                p.offset = e.stride;
                p.field = FieldOfName(w[2]);
                r->props_.push_back(p);
            }
            e.stride += SizeOf(t);
        }
    }
    if (vertexElem < 0) return Corrupt(path + ": PLY file without a vertex element");
    const Element& ve = elems[static_cast<size_t>(vertexElem)];
    r->count_ = ve.count;
    r->stride_ = ve.stride;
    bool hx = false, hy = false, hz = false;
    double colourMaxByType = 0;
    bool floatColour = false;
    for (const Prop& p : r->props_) {
        hx = hx || p.field == Field::kX;
        hy = hy || p.field == Field::kY;
        hz = hz || p.field == Field::kZ;
        if (p.field == Field::kRed || p.field == Field::kGreen || p.field == Field::kBlue) {
            r->info_.hasRgb = true;
            if (p.type == PType::kU8 || p.type == PType::kI8) colourMaxByType = std::max(colourMaxByType, 255.0);
            else if (p.type == PType::kF32 || p.type == PType::kF64) floatColour = true;
            else colourMaxByType = 65535.0;
        }
        if (p.field == Field::kInfrared) r->info_.hasNir = true;
        if (p.field == Field::kIntensity) r->info_.hasIntensity = true;
        if (p.field == Field::kGpsTime) r->info_.hasGpsTime = true;
    }
    if (!hx || !hy || !hz) return Corrupt(path + ": PLY vertex element without x, y and z");
    // Where the vertex data starts.
    r->dataStart_ = pos;
    if (r->fmt_ != Fmt::kAscii) {
        for (int i = 0; i < vertexElem; ++i) {
            const Element& e = elems[static_cast<size_t>(i)];
            if (e.hasList) return Unsupported(path + ": PLY element '" + e.name + "' with lists before the vertices");
            r->dataStart_ += e.count * e.stride;
        }
        if (r->dataStart_ + r->count_ * r->stride_ > size)
            return Corrupt(path + ": PLY file shorter than its vertex element");
        r->pointsPerTask_ = std::max<uint64_t>(options.pointsPerTask, 1024);
    } else {
        // Skip the lines of the elements before the vertices.
        uint64_t skip = 0;
        for (int i = 0; i < vertexElem; ++i) skip += elems[static_cast<size_t>(i)].count;
        for (uint64_t k = 0; k < skip; ++k) {
            const void* nl = std::memchr(data + r->dataStart_, '\n', static_cast<size_t>(size - r->dataStart_));
            if (nl == nullptr) return Corrupt(path + ": PLY file shorter than its elements");
            r->dataStart_ = static_cast<uint64_t>(static_cast<const char*>(nl) - data) + 1;
        }
        r->blocks_ = text::LineAlignedBlocks(data + r->dataStart_, data + size, options.bytesPerTask);
        r->firstLine_.resize(r->blocks_.size(), 0);
        for (size_t b = 1; b < r->blocks_.size(); ++b)
            r->firstLine_[b] = r->firstLine_[b - 1] +
                               text::CountNewlines(data + r->dataStart_ + r->blocks_[b - 1],
                                                   static_cast<size_t>(r->blocks_[b] - r->blocks_[b - 1]));
    }
    // Colour scale: integer types by their width; float colour by the values of a sample.
    if (floatColour) {
        double mx = 0;
        const uint64_t n = std::min<uint64_t>(r->count_, 10000);
        for (uint64_t i = 0; i < n && r->fmt_ != Fmt::kAscii; ++i)
            for (const Prop& p : r->props_)
                if (p.field == Field::kRed || p.field == Field::kGreen || p.field == Field::kBlue) {
                    const uint8_t* q = r->source_->Data() + r->dataStart_ + (i * r->count_ / n) * r->stride_ + p.offset;
                    mx = std::max(mx, r->fmt_ == Fmt::kBig ? LoadAs<true>(p.type, q) : LoadAs<false>(p.type, q));
                }
        colourMaxByType = std::max(colourMaxByType, r->fmt_ == Fmt::kAscii ? 255.0 : mx);
    }
    r->colourScale_ = ColourScaleForMax(colourMaxByType);
    SourceInfo& info = r->info_;
    info.path = path;
    info.kind = "ply";
    info.pointCount = r->count_;
    info.countExact = true;
    info.boundsKnown = false;
    info.rgb16 = r->colourScale_ == 1.0;
    for (const Prop& p : r->props_)
        if (p.field != Field::kNone) info.dimensions.push_back(FieldName(p.field));
    return r;
}

size_t PlyReader::TaskCount() const noexcept {
    if (fmt_ == Fmt::kAscii) return blocks_.size() - 1;
    return static_cast<size_t>((count_ + pointsPerTask_ - 1) / pointsPerTask_);
}

uint64_t PlyReader::TaskPoints(size_t task) const noexcept {
    if (fmt_ == Fmt::kAscii) {
        const uint64_t a = std::min(firstLine_[task], count_), b = std::min(firstLine_[task + 1], count_);
        return b - a;
    }
    const uint64_t first = task * pointsPerTask_;
    return first >= count_ ? 0 : std::min<uint64_t>(pointsPerTask_, count_ - first);
}

Status PlyReader::Read(size_t task, const OutputSpec& spec, BatchSink& sink) const {
    const size_t rs = spec.RecordSize();
    thread_local std::vector<uint8_t> recs;
    thread_local std::vector<double> xyz;
    recs.resize(kBatch * rs);
    xyz.resize(kBatch * 3);
    size_t n = 0;
    auto flush = [&]() -> Status {
        if (n == 0) return Status();
        Batch b{n, recs.data(), xyz.data()};
        n = 0;
        return sink.Consume(b);
    };
    if (fmt_ != Fmt::kAscii) {
        const uint64_t first = task * pointsPerTask_;
        const uint64_t count = TaskPoints(task);
        const uint8_t* base = source_->Data() + dataStart_ + first * stride_;
        const bool big = fmt_ == Fmt::kBig;
        for (uint64_t i = 0; i < count; ++i) {
            const uint8_t* v = base + i * stride_;
            uint8_t* r = recs.data() + n * rs;
            InitRecord(r, rs);
            double* p = xyz.data() + 3 * n;
            for (const Prop& pr : props_) {
                if (pr.field == Field::kNone) continue;
                const double val = big ? LoadAs<true>(pr.type, v + pr.offset) : LoadAs<false>(pr.type, v + pr.offset);
                SetField(pr.field, val, r, p, spec.format, colourScale_, 1.0);
            }
            if (++n == kBatch) DDBT_TRY(flush());
        }
        return flush();
    }
    const char* data = reinterpret_cast<const char*>(source_->Data());
    const char* p = data + dataStart_ + blocks_[task];
    const char* end = data + dataStart_ + blocks_[task + 1];
    const char* fileEnd = data + source_->Size();
    uint64_t line = firstLine_[task];
    while (p < end && line < count_) {
        const char* eol = static_cast<const char*>(std::memchr(p, '\n', static_cast<size_t>(fileEnd - p)));
        if (eol == nullptr) eol = fileEnd;
        uint8_t* r = recs.data() + n * rs;
        InitRecord(r, rs);
        double* q = xyz.data() + 3 * n;
        const char* c = p;
        size_t k = 0;
        double val;
        for (; k < props_.size() && text::NextDouble(c, eol, ' ', val); ++k)
            SetField(props_[k].field, val, r, q, spec.format, colourScale_, 1.0);
        if (k == 0) {
            // A blank line: not a vertex (it still counts as a line).
        } else if (k < props_.size()) {
            return Corrupt(source_->Name().empty() ? "PLY vertex line with too few values"
                                                   : std::string(source_->Name()) + ": PLY vertex line " +
                                                         std::to_string(line + 1) + " has too few values");
        } else if (++n == kBatch) {
            DDBT_TRY(flush());
        }
        ++line;
        p = eol + 1;
    }
    return flush();
}

}  // namespace ddbt::copc::readers
