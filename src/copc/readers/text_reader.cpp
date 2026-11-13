// SPDX-License-Identifier: MPL-2.0
//
// XYZ and PTS text point clouds.
//
// XYZ: DroneDB's detection (pointcloud.cpp detectTextFormat): leading "//" comment and blank
// lines are skipped (the last comment that names columns gives the column names, CloudCompare's
// export), then an optional point-count line, then an optional header line (any letter other
// than e / E), the separator from the first data line (semicolon, comma, tab, else spaces;
// DroneDB tries the comma first),
// columns from the header, else the comment, else by count (3: XYZ, 4: + Intensity, 6: + RGB,
// 7: + RGB + Intensity, 9: + RGB + ReturnNumber, NumberOfReturns, UserData). Names map to LAS
// dimensions as PDAL and CloudCompare name them. A decimal comma is accepted when the separator
// is not a comma (European locales).
//
// PTS (Leica): a point-count line, then X Y Z [Intensity [R G B]] (3, 4 or 7 values; 6 values are
// read as X Y Z R G B); every line holding one integer starts another scan and is skipped.
// Intensity i (-2048..2047) becomes (i + 2048) * 16.
//
// Colour: values up to 1 are scaled by 65535, up to 255 by 257, else kept (decided from lines
// sampled across the file). Lines with fewer values than the columns need for X / Y / Z are
// skipped. Tasks are newline-aligned byte ranges.
#include "copc/readers/text_reader.h"

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "copc/readers/fields.h"
#include "copc/readers/text_util.h"
#include "core/io/byte_source.h"

namespace ddbt::copc::readers {

namespace {

constexpr size_t kBatch = 1u << 16;

bool IsBlank(std::string_view l) {
    for (char c : l)
        if (!text::IsSpace(c)) return false;
    return true;
}
bool IsComment(std::string_view l) {
    size_t i = 0;
    while (i < l.size() && (l[i] == ' ' || l[i] == '\t')) ++i;
    return l.substr(i, 2) == "//";
}
bool IsSingleInteger(std::string_view l) {
    size_t i = 0, j = l.size();
    while (i < j && text::IsSpace(l[i])) ++i;
    while (j > i && text::IsSpace(l[j - 1])) --j;
    if (j > i + 1 && l[i] == '"' && l[j - 1] == '"') {
        ++i;
        --j;
    }
    if (i == j) return false;
    for (size_t k = i; k < j; ++k)
        if (l[k] < '0' || l[k] > '9') return false;
    return true;
}
bool LooksLikeHeader(std::string_view l) {
    for (char c : l)
        if (((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')) && c != 'e' && c != 'E') return true;
    return false;
}
// DroneDB's order (comma, semicolon, tab, space), except that a semicolon wins over a comma:
// "1,5;2,5" is a European line (decimal commas), never comma-separated.
char DetectSeparator(std::string_view l) {
    if (l.find(';') != std::string_view::npos) return ';';
    if (l.find(',') != std::string_view::npos) return ',';
    if (l.find('\t') != std::string_view::npos) return '\t';
    return ' ';
}
std::vector<std::string> Split(std::string_view l, char sep) {
    std::vector<std::string> out;
    size_t i = 0;
    auto trim = [](std::string_view t) {
        size_t a = 0, b = t.size();
        while (a < b && text::IsSpace(t[a])) ++a;
        while (b > a && text::IsSpace(t[b - 1])) --b;
        t = t.substr(a, b - a);
        if (t.size() >= 2 && t.front() == '"' && t.back() == '"') t = t.substr(1, t.size() - 2);
        return std::string(t);
    };
    if (sep == ' ') {
        while (i < l.size()) {
            while (i < l.size() && text::IsSpace(l[i])) ++i;
            size_t j = i;
            while (j < l.size() && !text::IsSpace(l[j])) ++j;
            if (j > i) out.push_back(trim(l.substr(i, j - i)));
            i = j;
        }
        return out;
    }
    while (i <= l.size()) {
        size_t j = l.find(sep, i);
        if (j == std::string_view::npos) j = l.size();
        std::string t = trim(l.substr(i, j - i));
        if (!t.empty()) out.push_back(std::move(t));
        i = j + 1;
    }
    return out;
}
std::vector<std::string> DefaultColumns(size_t n) {
    switch (n) {
        case 3: return {"X", "Y", "Z"};
        case 4: return {"X", "Y", "Z", "Intensity"};
        case 6: return {"X", "Y", "Z", "Red", "Green", "Blue"};
        case 7: return {"X", "Y", "Z", "Red", "Green", "Blue", "Intensity"};
        case 9: return {"X", "Y", "Z", "Red", "Green", "Blue", "ReturnNumber", "NumberOfReturns", "UserData"};
        default: break;
    }
    std::vector<std::string> c = {"X", "Y", "Z"};
    for (size_t i = 3; i < n; ++i) c.push_back("Extra" + std::to_string(i - 3));
    return c;
}

/// Parses one number of a line with a decimal comma.
bool NextDoubleComma(const char*& p, const char* end, char sep, double& v) {
    while (p < end && (text::IsSpace(*p) || *p == sep)) ++p;
    if (p >= end || *p == '\n') return false;
    char buf[64];
    size_t n = 0;
    while (p < end && !text::IsSpace(*p) && *p != sep && *p != '\n') {
        if (n < sizeof(buf) - 1) buf[n++] = *p == ',' ? '.' : *p;
        ++p;
    }
    const char* s = buf;
    const char* e = buf + n;
    if (s < e && *s == '+') ++s;
    if (std::from_chars(s, e, v).ec != std::errc()) v = 0;
    return true;
}

}  // namespace

TextReader::TextReader() = default;
TextReader::~TextReader() = default;

Expected<std::unique_ptr<TextReader>> TextReader::Open(const std::string& path, bool pts, const ReaderOptions& options) {
    DDBT_ASSIGN_OR_RETURN(std::unique_ptr<io::ByteSource> src, io::OpenByteSource(path));
    std::unique_ptr<TextReader> r(new TextReader());
    r->pts_ = pts;
    r->source_ = std::shared_ptr<const io::ByteSource>(std::move(src));
    const uint64_t size = r->source_->Size();
    if (size == 0) return InvalidArgument(path + ": empty file");
    if (r->source_->Data() == nullptr) return Unsupported("text reader needs a memory-mapped file");
    const char* data = reinterpret_cast<const char*>(r->source_->Data());
    // The first lines.
    std::vector<std::string_view> lines;
    std::vector<uint64_t> lineStart;
    for (uint64_t pos = 0; pos < size && lines.size() < 64;) {
        const void* nl = std::memchr(data + pos, '\n', static_cast<size_t>(size - pos));
        const uint64_t e = nl ? static_cast<uint64_t>(static_cast<const char*>(nl) - data) : size;
        lineStart.push_back(pos);
        lines.emplace_back(data + pos, static_cast<size_t>(e - pos));
        pos = e + 1;
    }
    lineStart.push_back(size);
    size_t idx = 0;
    std::vector<std::string> columns;
    char sep = ' ';
    if (pts) {
        while (idx < lines.size() && IsBlank(lines[idx])) ++idx;
        if (idx < lines.size() && IsSingleInteger(lines[idx])) ++idx;
        if (idx >= lines.size()) return Corrupt(path + ": PTS file without points");
        sep = DetectSeparator(lines[idx]);
        const size_t n = Split(lines[idx], sep).size();
        if (n == 3) columns = {"X", "Y", "Z"};
        else if (n == 4) columns = {"X", "Y", "Z", "Intensity"};
        else if (n == 6) columns = {"X", "Y", "Z", "Red", "Green", "Blue"};
        else if (n >= 7) columns = {"X", "Y", "Z", "Intensity", "Red", "Green", "Blue"};
        else return Corrupt(path + ": PTS line with " + std::to_string(n) + " values");
    } else {
        // Column names from the last comment line before the data.
        std::vector<std::string> commentCols;
        size_t lastComment = SIZE_MAX;
        for (size_t i = 0; i < lines.size(); ++i) {
            if (IsBlank(lines[i])) continue;
            if (!IsComment(lines[i])) break;
            lastComment = i;
        }
        if (lastComment != SIZE_MAX) {
            std::string_view l = lines[lastComment];
            l = l.substr(l.find("//") + 2);
            if (LooksLikeHeader(l)) commentCols = Split(l, DetectSeparator(l));
        }
        while (idx < lines.size() && (IsComment(lines[idx]) || IsBlank(lines[idx]))) ++idx;
        if (idx < lines.size() && IsSingleInteger(lines[idx])) ++idx;
        if (idx >= lines.size()) return Corrupt(path + ": no data lines");
        std::vector<std::string> inlineCols;
        sep = DetectSeparator(lines[idx]);
        if (LooksLikeHeader(lines[idx])) {
            inlineCols = Split(lines[idx], sep);
            ++idx;
        }
        if (idx < lines.size() && !IsBlank(lines[idx])) sep = DetectSeparator(lines[idx]);
        const size_t dataCols = idx < lines.size() ? Split(lines[idx], sep).size() : 0;
        if (inlineCols.size() >= 3) columns = inlineCols;
        else if (commentCols.size() >= 3 && (dataCols == 0 || commentCols.size() == dataCols)) columns = commentCols;
        else if (dataCols > 0) columns = DefaultColumns(dataCols);
        else columns = {"X", "Y", "Z"};
    }
    r->sep_ = sep;
    r->dataStart_ = lineStart[std::min(idx, lineStart.size() - 1)];
    // A decimal comma: the separator is not a comma and a data value holds one.
    if (sep != ',' && idx < lines.size() && lines[idx].find(',') != std::string_view::npos) r->decimalComma_ = true;
    bool hx = false, hy = false, hz = false;
    double colourMax = 0;
    for (const std::string& c : columns) {
        const Field f = FieldOfName(c);
        r->fields_.push_back(f);
        hx = hx || f == Field::kX;
        hy = hy || f == Field::kY;
        hz = hz || f == Field::kZ;
        if (f == Field::kRed || f == Field::kGreen || f == Field::kBlue) r->info_.hasRgb = true;
        if (f == Field::kInfrared) r->info_.hasNir = true;
        if (f == Field::kIntensity) r->info_.hasIntensity = true;
        if (f == Field::kGpsTime) r->info_.hasGpsTime = true;
        if (f != Field::kNone) r->info_.dimensions.push_back(FieldName(f));
    }
    if (!hx || !hy || !hz) return Corrupt(path + ": no X, Y and Z columns");
    r->minValues_ = 0;
    for (size_t i = 0; i < r->fields_.size(); ++i)
        if (r->fields_[i] == Field::kX || r->fields_[i] == Field::kY || r->fields_[i] == Field::kZ) r->minValues_ = i + 1;
    r->blocks_ = text::LineAlignedBlocks(data + r->dataStart_, data + size, options.bytesPerTask);
    if (pts) {
        // PDAL reads the number of points the count line announces. Several scans (a count line
        // after the announced points) are all read, their count lines skipped.
        r->firstLine_.assign(r->blocks_.size(), 0);
        for (size_t b = 1; b < r->blocks_.size(); ++b)
            r->firstLine_[b] = r->firstLine_[b - 1] +
                               text::CountNewlines(data + r->dataStart_ + r->blocks_[b - 1],
                                                   static_cast<size_t>(r->blocks_[b] - r->blocks_[b - 1]));
        size_t c = 0;
        while (c < lines.size() && IsBlank(lines[c])) ++c;
        if (c < lines.size() && IsSingleInteger(lines[c]) && lineStart[c + 1] == r->dataStart_) {
            const uint64_t announced = std::strtoull(std::string(lines[c]).c_str(), nullptr, 10);
            // The line after the announced points.
            uint64_t pos = r->dataStart_;
            uint64_t k = 0;
            while (k < announced && pos < size) {
                const void* nl = std::memchr(data + pos, '\n', static_cast<size_t>(size - pos));
                pos = nl ? static_cast<uint64_t>(static_cast<const char*>(nl) - data) + 1 : size;
                ++k;
            }
            bool more = false;
            if (pos < size) {
                const void* nl = std::memchr(data + pos, '\n', static_cast<size_t>(size - pos));
                const uint64_t e = nl ? static_cast<uint64_t>(static_cast<const char*>(nl) - data) : size;
                more = IsSingleInteger(std::string_view(data + pos, static_cast<size_t>(e - pos)));
            }
            if (!more) r->maxLine_ = announced;
        }
    }

    // Colour scale and an estimate of the count from lines sampled across the file.
    {
        uint64_t sampledLines = 0, sampledBytes = 0;
        const uint64_t span = size - r->dataStart_;
        for (int s = 0; s < 64 && span > 0; ++s) {
            uint64_t pos = r->dataStart_ + span * static_cast<uint64_t>(s) / 64;
            if (pos > r->dataStart_) {
                const void* nl = std::memchr(data + pos - 1, '\n', static_cast<size_t>(size - pos + 1));
                if (nl == nullptr) continue;
                pos = static_cast<uint64_t>(static_cast<const char*>(nl) - data) + 1;
            }
            for (int k = 0; k < 200 && pos < size; ++k) {
                const void* nl = std::memchr(data + pos, '\n', static_cast<size_t>(size - pos));
                const uint64_t e = nl ? static_cast<uint64_t>(static_cast<const char*>(nl) - data) : size;
                const char* c = data + pos;
                double v;
                for (size_t col = 0; col < r->fields_.size(); ++col) {
                    const bool ok = r->decimalComma_ ? NextDoubleComma(c, data + e, sep, v) : text::NextDouble(c, data + e, sep, v);
                    if (!ok) break;
                    const Field f = r->fields_[col];
                    if (f == Field::kRed || f == Field::kGreen || f == Field::kBlue || f == Field::kInfrared)
                        colourMax = std::max(colourMax, v);
                }
                sampledBytes += e + 1 - pos;
                ++sampledLines;
                pos = e + 1;
            }
        }
        r->colourScale_ = ColourScaleForMax(colourMax);
        r->info_.pointCount = sampledLines ? static_cast<uint64_t>(static_cast<double>(span) * static_cast<double>(sampledLines) /
                                                                   static_cast<double>(std::max<uint64_t>(sampledBytes, 1)))
                                           : 0;
    }
    SourceInfo& info = r->info_;
    info.path = path;
    info.kind = pts ? "pts" : "xyz";
    info.countExact = false;
    info.boundsKnown = false;
    info.rgb16 = r->colourScale_ == 1.0;
    return r;
}

size_t TextReader::TaskCount() const noexcept { return blocks_.size() - 1; }

uint64_t TextReader::TaskPoints(size_t task) const noexcept {
    const uint64_t bytes = blocks_[task + 1] - blocks_[task];
    const uint64_t all = blocks_.back();
    return all ? bytes * info_.pointCount / all : 0;
}

Status TextReader::Read(size_t task, const OutputSpec& spec, BatchSink& sink) const {
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
    const char* data = reinterpret_cast<const char*>(source_->Data());
    const char* p = data + dataStart_ + blocks_[task];
    const char* end = data + dataStart_ + blocks_[task + 1];
    const double intensityScale = 1.0;
    uint64_t line = firstLine_.empty() ? 0 : firstLine_[task];
    while (p < end && line < maxLine_) {
        ++line;
        const char* eol = static_cast<const char*>(std::memchr(p, '\n', static_cast<size_t>(end - p)));
        if (eol == nullptr) eol = end;
        uint8_t* r = recs.data() + n * rs;
        InitRecord(r, rs);
        double* q = xyz.data() + 3 * n;
        const char* c = p;
        size_t k = 0;
        double v;
        for (; k < fields_.size(); ++k) {
            const bool ok = decimalComma_ ? NextDoubleComma(c, eol, sep_, v) : text::NextDouble(c, eol, sep_, v);
            if (!ok) break;
            const Field f = fields_[k];
            if (pts_ && f == Field::kIntensity) v = (v + 2048.0) * 16.0;
            SetField(f, v, r, q, spec.format, colourScale_, intensityScale);
        }
        p = eol + 1;
        if (k < minValues_) continue;  // blank, count or scan header line, or a short line: not a point
        if (++n == kBatch) DDBT_TRY(flush());
    }
    return flush();
}

}  // namespace ddbt::copc::readers
