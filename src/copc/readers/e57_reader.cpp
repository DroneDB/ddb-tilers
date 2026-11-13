// SPDX-License-Identifier: MPL-2.0
#include "copc/readers/e57_reader.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#include "copc/readers/fields.h"
#include "core/util/checked_math.h"
#include "core/util/endian.h"

#if defined(__SSE4_2__)
#include <nmmintrin.h>
#endif

namespace ddbt::copc::readers {

// ------------------------------------------------------------------ CRC-32C

namespace {
struct Crc32cTable {
    uint32_t t[256];
    Crc32cTable() {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c >> 1) ^ ((c & 1u) ? 0x82F63B78u : 0u);
            t[i] = c;
        }
    }
};
}  // namespace

uint32_t Crc32c(const uint8_t* p, size_t n) noexcept {
    uint32_t crc = 0xFFFFFFFFu;
#if defined(__SSE4_2__) && defined(__x86_64__)
    uint64_t c = crc;
    for (; n >= 8; n -= 8, p += 8) c = _mm_crc32_u64(c, LoadLE<uint64_t>(p));
    crc = static_cast<uint32_t>(c);
    for (; n > 0; --n, ++p) crc = _mm_crc32_u8(crc, *p);
#else
    static const Crc32cTable table;
    for (; n > 0; --n, ++p) crc = table.t[(crc ^ *p) & 0xFFu] ^ (crc >> 8);
#endif
    return crc ^ 0xFFFFFFFFu;
}

// ------------------------------------------------------------------ XML

const XmlElement* XmlElement::Child(std::string_view n) const noexcept {
    for (const XmlElement& c : children)
        if (c.name == n) return &c;
    return nullptr;
}
const std::string* XmlElement::Attribute(std::string_view n) const noexcept {
    for (const auto& [k, v] : attributes)
        if (k == n) return &v;
    return nullptr;
}

namespace {

class XmlParser {
public:
    explicit XmlParser(std::string_view s) : s_(s) {}

    Expected<XmlElement> Document() {
        SkipMisc();
        XmlElement root;
        DDBT_TRY(Element(root, 0));
        return root;
    }

private:
    Status Fail(const std::string& what) { return Corrupt("E57 XML: " + what + " at byte " + std::to_string(i_)); }
    bool Starts(std::string_view t) const { return s_.substr(i_, t.size()) == t; }
    void SkipSpace() {
        while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\r' || s_[i_] == '\n')) ++i_;
    }
    void SkipMisc() {
        for (;;) {
            SkipSpace();
            if (Starts("<?")) {
                const size_t e = s_.find("?>", i_);
                i_ = e == std::string_view::npos ? s_.size() : e + 2;
            } else if (Starts("<!--")) {
                const size_t e = s_.find("-->", i_);
                i_ = e == std::string_view::npos ? s_.size() : e + 3;
            } else if (Starts("<!")) {
                const size_t e = s_.find('>', i_);
                i_ = e == std::string_view::npos ? s_.size() : e + 1;
            } else {
                return;
            }
        }
    }
    std::string Name() {
        const size_t b = i_;
        while (i_ < s_.size() && !std::strchr(" \t\r\n/>=", s_[i_])) ++i_;
        return std::string(s_.substr(b, i_ - b));
    }
    static void Decode(std::string_view in, std::string& out) {
        for (size_t k = 0; k < in.size(); ++k) {
            if (in[k] != '&') {
                out += in[k];
                continue;
            }
            const size_t e = in.find(';', k);
            if (e == std::string_view::npos) {
                out += in[k];
                continue;
            }
            const std::string_view ent = in.substr(k + 1, e - k - 1);
            if (ent == "lt") out += '<';
            else if (ent == "gt") out += '>';
            else if (ent == "amp") out += '&';
            else if (ent == "quot") out += '"';
            else if (ent == "apos") out += '\'';
            else if (!ent.empty() && ent[0] == '#') {
                const unsigned long cp = ent.size() > 1 && (ent[1] == 'x' || ent[1] == 'X')
                                             ? std::strtoul(std::string(ent.substr(2)).c_str(), nullptr, 16)
                                             : std::strtoul(std::string(ent.substr(1)).c_str(), nullptr, 10);
                if (cp < 0x80) {
                    out += static_cast<char>(cp);
                } else if (cp < 0x800) {
                    out += static_cast<char>(0xC0 | (cp >> 6));
                    out += static_cast<char>(0x80 | (cp & 0x3F));
                } else {
                    out += static_cast<char>(0xE0 | ((cp >> 12) & 0x0F));
                    out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
                    out += static_cast<char>(0x80 | (cp & 0x3F));
                }
            } else {
                out.append(in.substr(k, e - k + 1));
            }
            k = e;
        }
    }
    Status Element(XmlElement& el, int depth) {
        if (depth > 200) return Fail("nesting too deep");
        if (!Starts("<")) return Fail("element expected");
        ++i_;
        el.name = Name();
        if (el.name.empty()) return Fail("element name expected");
        for (;;) {
            SkipSpace();
            if (i_ >= s_.size()) return Fail("unterminated tag");
            if (Starts("/>")) {
                i_ += 2;
                return Status();
            }
            if (s_[i_] == '>') {
                ++i_;
                break;
            }
            std::string an = Name();
            SkipSpace();
            if (i_ >= s_.size() || s_[i_] != '=') return Fail("attribute without value");
            ++i_;
            SkipSpace();
            if (i_ >= s_.size() || (s_[i_] != '"' && s_[i_] != '\'')) return Fail("unquoted attribute");
            const char q = s_[i_++];
            const size_t e = s_.find(q, i_);
            if (e == std::string_view::npos) return Fail("unterminated attribute");
            std::string v;
            Decode(s_.substr(i_, e - i_), v);
            el.attributes.emplace_back(std::move(an), std::move(v));
            i_ = e + 1;
        }
        // Content.
        for (;;) {
            if (i_ >= s_.size()) return Fail("unterminated element " + el.name);
            if (Starts("</")) {
                i_ += 2;
                const std::string n = Name();
                if (n != el.name) return Fail("mismatched end tag " + n);
                SkipSpace();
                if (i_ >= s_.size() || s_[i_] != '>') return Fail("bad end tag");
                ++i_;
                return Status();
            }
            if (Starts("<![CDATA[")) {
                const size_t e = s_.find("]]>", i_);
                if (e == std::string_view::npos) return Fail("unterminated CDATA");
                el.text.append(s_.substr(i_ + 9, e - i_ - 9));
                i_ = e + 3;
            } else if (Starts("<!--")) {
                const size_t e = s_.find("-->", i_);
                i_ = e == std::string_view::npos ? s_.size() : e + 3;
            } else if (Starts("<?")) {
                const size_t e = s_.find("?>", i_);
                i_ = e == std::string_view::npos ? s_.size() : e + 2;
            } else if (s_[i_] == '<') {
                el.children.emplace_back();
                DDBT_TRY(Element(el.children.back(), depth + 1));
            } else {
                const size_t e = s_.find('<', i_);
                const size_t end = e == std::string_view::npos ? s_.size() : e;
                Decode(s_.substr(i_, end - i_), el.text);
                i_ = end;
            }
        }
    }

    std::string_view s_;
    size_t i_ = 0;
};

}  // namespace

Expected<XmlElement> ParseXml(std::string_view xml) { return XmlParser(xml).Document(); }

// ------------------------------------------------------------------ E57

namespace {

enum class Role : uint8_t {
    kOther,
    kX,
    kY,
    kZ,
    kRange,
    kAzimuth,
    kElevation,
    kRed,
    kGreen,
    kBlue,
    kIntensity,
    kReturnIndex,
    kReturnCount,
    kTime,
    kClassification,
};

Role RoleOf(const std::string& n) {
    if (n == "cartesianX") return Role::kX;
    if (n == "cartesianY") return Role::kY;
    if (n == "cartesianZ") return Role::kZ;
    if (n == "sphericalRange") return Role::kRange;
    if (n == "sphericalAzimuth") return Role::kAzimuth;
    if (n == "sphericalElevation") return Role::kElevation;
    if (n == "colorRed") return Role::kRed;
    if (n == "colorGreen") return Role::kGreen;
    if (n == "colorBlue") return Role::kBlue;
    if (n == "intensity") return Role::kIntensity;
    if (n == "returnIndex") return Role::kReturnIndex;
    if (n == "returnCount") return Role::kReturnCount;
    if (n == "timeStamp") return Role::kTime;
    if (n == "classification") return Role::kClassification;
    return Role::kOther;
}

double ToDouble(const std::string& s, double def = 0) {
    if (s.empty()) return def;
    char* e = nullptr;
    const double v = std::strtod(s.c_str(), &e);
    return e == s.c_str() ? def : v;
}
int64_t ToInt(const std::string& s, int64_t def) {
    if (s.empty()) return def;
    char* e = nullptr;
    const long long v = std::strtoll(s.c_str(), &e, 10);
    return e == s.c_str() ? def : static_cast<int64_t>(v);
}

/// Value of a numeric E57 node (Integer, ScaledInteger, Float); `def` when absent.
double NodeValue(const XmlElement* e, double def) {
    if (e == nullptr) return def;
    const std::string* type = e->Attribute("type");
    if (type != nullptr && *type == "ScaledInteger") {
        const std::string* sc = e->Attribute("scale");
        const std::string* of = e->Attribute("offset");
        return static_cast<double>(ToInt(e->text, 0)) * (sc ? ToDouble(*sc, 1) : 1.0) + (of ? ToDouble(*of, 0) : 0.0);
    }
    return ToDouble(e->text, 0.0);
}

inline double Unfused3(double x, double a, double y, double b, double z, double c, double t) {
    double p = x * a, q = y * b, r = z * c;
#if defined(__GNUC__) && (defined(__x86_64__) || defined(__i386__))
    __asm__("" : "+x"(p), "+x"(q), "+x"(r));
#endif
    return p + q + r + t;
}

}  // namespace

struct E57Reader::Scan {
    struct FieldDef {
        std::string name;
        Role role = Role::kOther;
        enum Kind : uint8_t { kF32, kF64, kInt, kScaled } kind = kInt;
        int64_t min = 0, max = 0;
        double scale = 1, offset = 0;
        int width = 0;
        // Stream chunks in logical file offsets, with the stream bytes before each.
        std::vector<uint64_t> chunkLogical, chunkBytes, chunkBefore;
        uint64_t totalBytes = 0;
    };
    std::vector<FieldDef> fields;
    uint64_t records = 0;
    bool hasPose = false;
    double rot[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    double trans[3] = {0, 0, 0};
    double colourFactor[3] = {1, 1, 1};
    double intensityFactor = 1;
    double classFactor = 1;
    bool spherical = false;
};

E57Reader::E57Reader() = default;
E57Reader::~E57Reader() = default;
size_t E57Reader::ScanCount() const noexcept { return scans_.size(); }
uint64_t E57Reader::TaskPoints(size_t task) const noexcept { return tasks_[task].count; }

Status E57Reader::ReadLogical(uint64_t logical, uint8_t* out, size_t n) const {
    const uint64_t data = pageSize_ - 4;
    const uint8_t* base = source_->Data();
    const uint64_t size = source_->Size();
    while (n > 0) {
        const uint64_t page = logical / data, within = logical % data;
        const uint64_t phys = page * pageSize_;
        if (phys + pageSize_ > size) return Corrupt("E57 read past the end of the file");
        const uint8_t* pg = base + phys;
        if (Crc32c(pg, static_cast<size_t>(data)) != LoadBE<uint32_t>(pg + data))
            return Corrupt("E57 page " + std::to_string(page) + " fails its CRC-32C check");
        const size_t take = static_cast<size_t>(std::min<uint64_t>(n, data - within));
        std::memcpy(out, pg + within, take);
        out += take;
        n -= take;
        logical += take;
    }
    return Status();
}

Expected<std::unique_ptr<E57Reader>> E57Reader::Open(const std::string& path, const ReaderOptions& options) {
    DDBT_ASSIGN_OR_RETURN(std::unique_ptr<io::ByteSource> src, io::OpenByteSource(path));
    if (src->Data() == nullptr) return Unsupported("E57 reader needs a memory-mapped file");
    std::unique_ptr<E57Reader> r(new E57Reader());
    r->source_ = std::shared_ptr<const io::ByteSource>(std::move(src));
    const uint8_t* d = r->source_->Data();
    const uint64_t size = r->source_->Size();
    if (size < 48 || std::memcmp(d, "ASTM-E57", 8) != 0) return Corrupt(path + ": not an E57 file");
    const uint32_t major = LoadLE<uint32_t>(d + 8);
    const uint64_t xmlPhys = LoadLE<uint64_t>(d + 24), xmlLen = LoadLE<uint64_t>(d + 32);
    r->pageSize_ = LoadLE<uint64_t>(d + 40);
    if (major != 1) return Unsupported(path + ": E57 version " + std::to_string(major));
    if (r->pageSize_ < 64 || r->pageSize_ > (1u << 20) || size % r->pageSize_ != 0)
        return Corrupt(path + ": E57 page size " + std::to_string(r->pageSize_));
    const uint64_t dataPerPage = r->pageSize_ - 4;
    auto toLogical = [&](uint64_t phys) { return phys / r->pageSize_ * dataPerPage + phys % r->pageSize_; };
    if (xmlLen > (256u << 20) || xmlPhys >= size) return Corrupt(path + ": E57 XML section out of range");
    std::string xml(static_cast<size_t>(xmlLen), '\0');
    DDBT_TRY(r->ReadLogical(toLogical(xmlPhys), reinterpret_cast<uint8_t*>(xml.data()), xml.size()));
    DDBT_ASSIGN_OR_RETURN(XmlElement root, ParseXml(xml));
    if (root.name != "e57Root") return Corrupt(path + ": E57 XML root is " + root.name);
    const XmlElement* data3D = root.Child("data3D");
    if (data3D == nullptr) return Corrupt(path + ": E57 file without data3D");

    SourceInfo& info = r->info_;
    for (const XmlElement& sc : data3D->children) {
        Scan s;
        const XmlElement* pts = sc.Child("points");
        if (pts == nullptr) continue;
        const std::string* fo = pts->Attribute("fileOffset");
        const std::string* rc = pts->Attribute("recordCount");
        if (fo == nullptr || rc == nullptr) return Corrupt(path + ": E57 points without fileOffset / recordCount");
        s.records = static_cast<uint64_t>(ToInt(*rc, 0));
        const XmlElement* proto = pts->Child("prototype");
        if (proto == nullptr) return Corrupt(path + ": E57 points without a prototype");
        for (const XmlElement& f : proto->children) {
            Scan::FieldDef fd;
            fd.name = f.name;
            fd.role = RoleOf(f.name);
            const std::string* type = f.Attribute("type");
            const std::string t = type ? *type : "";
            if (t == "Float") {
                const std::string* prec = f.Attribute("precision");
                fd.kind = prec != nullptr && *prec == "single" ? Scan::FieldDef::kF32 : Scan::FieldDef::kF64;
                fd.width = fd.kind == Scan::FieldDef::kF32 ? 32 : 64;
            } else if (t == "Integer" || t == "ScaledInteger") {
                fd.kind = t == "Integer" ? Scan::FieldDef::kInt : Scan::FieldDef::kScaled;
                const std::string* mn = f.Attribute("minimum");
                const std::string* mx = f.Attribute("maximum");
                fd.min = mn ? ToInt(*mn, std::numeric_limits<int64_t>::min()) : std::numeric_limits<int64_t>::min();
                fd.max = mx ? ToInt(*mx, std::numeric_limits<int64_t>::max()) : std::numeric_limits<int64_t>::max();
                if (fd.kind == Scan::FieldDef::kScaled) {
                    const std::string* sc2 = f.Attribute("scale");
                    const std::string* of = f.Attribute("offset");
                    fd.scale = sc2 ? ToDouble(*sc2, 1) : 1.0;
                    fd.offset = of ? ToDouble(*of, 0) : 0.0;
                }
                const uint64_t range = static_cast<uint64_t>(fd.max) - static_cast<uint64_t>(fd.min);
                fd.width = fd.max <= fd.min ? 0 : (range == UINT64_MAX ? 64 : static_cast<int>(std::bit_width(range)));
            } else {
                return Unsupported(path + ": E57 field " + f.name + " of type " + t);
            }
            s.fields.push_back(std::move(fd));
        }
        bool hx = false, hy = false, hz = false, hr = false, ha = false, he = false;
        for (const auto& f : s.fields) {
            hx |= f.role == Role::kX;
            hy |= f.role == Role::kY;
            hz |= f.role == Role::kZ;
            hr |= f.role == Role::kRange;
            ha |= f.role == Role::kAzimuth;
            he |= f.role == Role::kElevation;
            if (f.role == Role::kRed) info.hasRgb = true;
            if (f.role == Role::kIntensity) info.hasIntensity = true;
            if (f.role == Role::kTime) info.hasGpsTime = true;
        }
        if (!(hx && hy && hz) && !(hr && ha && he)) return Unsupported(path + ": E57 scan without cartesian or spherical coordinates");
        s.spherical = !(hx && hy && hz);
        // Pose (PDAL's Scan::getPose).
        if (const XmlElement* pose = sc.Child("pose")) {
            s.hasPose = true;
            if (const XmlElement* rot = pose->Child("rotation")) {
                const double q0 = NodeValue(rot->Child("w"), 0), q1 = NodeValue(rot->Child("x"), 0),
                             q2 = NodeValue(rot->Child("y"), 0), q3 = NodeValue(rot->Child("z"), 0);
                const double q11 = q1 * q1, q22 = q2 * q2, q33 = q3 * q3, q03 = q0 * q3, q13 = q1 * q3, q23 = q2 * q3,
                             q02 = q0 * q2, q12 = q1 * q2, q01 = q0 * q1;
                s.rot[0][0] = 1 - 2.0 * (q22 + q33);
                s.rot[1][1] = 1 - 2.0 * (q11 + q33);
                s.rot[2][2] = 1 - 2.0 * (q11 + q22);
                s.rot[0][1] = 2.0 * (q12 - q03);
                s.rot[1][0] = 2.0 * (q12 + q03);
                s.rot[0][2] = 2.0 * (q13 + q02);
                s.rot[2][0] = 2.0 * (q13 - q02);
                s.rot[1][2] = 2.0 * (q23 - q01);
                s.rot[2][1] = 2.0 * (q23 + q01);
            }
            if (const XmlElement* t = pose->Child("translation")) {
                s.trans[0] = NodeValue(t->Child("x"), 0);
                s.trans[1] = NodeValue(t->Child("y"), 0);
                s.trans[2] = NodeValue(t->Child("z"), 0);
            }
        }
        // Rescale factors (PDAL's Scan::decodeHeader): float(range of the type / (max - min)).
        auto limits = [&](const char* box, const char* field, double& mn, double& mx) -> bool {
            const XmlElement* b = sc.Child(box);
            if (b != nullptr) {
                mn = NodeValue(b->Child(std::string(field) + "Minimum"), 0);
                mx = NodeValue(b->Child(std::string(field) + "Maximum"), 0);
                return true;
            }
            for (const auto& f : s.fields)
                if (f.name == field) {
                    if (f.kind == Scan::FieldDef::kInt) {
                        mn = static_cast<double>(f.min);
                        mx = static_cast<double>(f.max);
                    } else if (f.kind == Scan::FieldDef::kScaled) {
                        mn = static_cast<double>(f.min) * f.scale + f.offset;
                        mx = static_cast<double>(f.max) * f.scale + f.offset;
                    } else {
                        const XmlElement* node = proto->Child(field);
                        const std::string* a = node ? node->Attribute("minimum") : nullptr;
                        const std::string* b2 = node ? node->Attribute("maximum") : nullptr;
                        mn = a ? ToDouble(*a) : 0;
                        mx = b2 ? ToDouble(*b2) : 0;
                    }
                    return true;
                }
            return false;
        };
        auto factor = [](double top, double mn, double mx) {
            if (!(mx > mn)) return 1.0;
            return static_cast<double>(static_cast<float>(top / (mx - mn)));
        };
        const char* colours[3] = {"colorRed", "colorGreen", "colorBlue"};
        for (int c = 0; c < 3; ++c) {
            double mn = 0, mx = 0;
            if (limits("colorLimits", colours[c], mn, mx)) s.colourFactor[c] = factor(65535.0, mn, mx);
        }
        {
            double mn = 0, mx = 0;
            if (limits("intensityLimits", "intensity", mn, mx)) s.intensityFactor = factor(65535.0, mn, mx);
            if (limits("classificationLimits", "classification", mn, mx)) s.classFactor = factor(255.0, mn, mx);
        }
        // Index the data packets of the CompressedVector.
        const uint64_t secPhys = static_cast<uint64_t>(ToInt(*fo, -1));
        if (secPhys + 32 > size) return Corrupt(path + ": E57 CompressedVector offset out of range");
        uint8_t sh[32];
        DDBT_TRY(r->ReadLogical(toLogical(secPhys), sh, 32));
        if (sh[0] != 1) return Corrupt(path + ": E57 CompressedVector section id " + std::to_string(sh[0]));
        const uint64_t secLen = LoadLE<uint64_t>(sh + 8), dataPhys = LoadLE<uint64_t>(sh + 16);
        const uint64_t secEnd = toLogical(secPhys) + secLen;
        uint64_t at = toLogical(dataPhys);
        if (s.records > 0 && (dataPhys >= size || secEnd > toLogical(size))) return Corrupt(path + ": E57 section out of range");
        while (s.records > 0 && at + 4 <= secEnd) {
            uint8_t ph[4];
            DDBT_TRY(r->ReadLogical(at, ph, 4));
            const uint64_t len = uint64_t{LoadLE<uint16_t>(ph + 2)} + 1;
            if (ph[0] == 1) {
                uint8_t cnt[2];
                DDBT_TRY(r->ReadLogical(at + 4, cnt, 2));
                const uint16_t streams = LoadLE<uint16_t>(cnt);
                if (streams != s.fields.size())
                    return Corrupt(path + ": E57 data packet with " + std::to_string(streams) + " bytestreams for " +
                                   std::to_string(s.fields.size()) + " fields");
                std::vector<uint8_t> lens(2u * streams);
                DDBT_TRY(r->ReadLogical(at + 6, lens.data(), lens.size()));
                uint64_t off = at + 6 + 2u * streams;
                for (size_t f = 0; f < streams; ++f) {
                    const uint64_t n = LoadLE<uint16_t>(lens.data() + 2 * f);
                    if (n) {
                        Scan::FieldDef& fd = s.fields[f];
                        fd.chunkLogical.push_back(off);
                        fd.chunkBytes.push_back(n);
                        fd.chunkBefore.push_back(fd.totalBytes);
                        fd.totalBytes += n;
                    }
                    off += n;
                }
                if (off > at + len) return Corrupt(path + ": E57 data packet larger than its length");
            } else if (ph[0] != 0 && ph[0] != 2) {
                return Corrupt(path + ": E57 packet type " + std::to_string(ph[0]));
            }
            at += len;
        }
        for (const auto& f : s.fields) {
            uint64_t bits = 0;
            if (!CheckedMul(s.records, static_cast<uint64_t>(f.width), &bits) || bits > UINT64_MAX - 7 ||
                (bits + 7) / 8 > f.totalBytes)
                return Corrupt(path + ": E57 field " + f.name + " holds fewer values than the record count");
        }
        const uint32_t idx = static_cast<uint32_t>(r->scans_.size());
        const uint64_t per = std::max<uint64_t>(options.pointsPerTask, 4096);
        for (uint64_t first = 0; first < s.records; first += per)
            r->tasks_.push_back({idx, first, std::min(per, s.records - first)});
        info.pointCount += s.records;
        r->scans_.push_back(std::move(s));
    }
    info.path = path;
    info.kind = "e57";
    info.countExact = true;
    info.boundsKnown = false;
    info.rgb16 = false;
    info.dimensions = {"X", "Y", "Z"};
    if (info.hasIntensity) info.dimensions.push_back("Intensity");
    if (info.hasRgb) {
        info.dimensions.push_back("Red");
        info.dimensions.push_back("Green");
        info.dimensions.push_back("Blue");
    }
    if (const XmlElement* cm = root.Child("coordinateMetadata"); cm != nullptr && !cm->text.empty()) info.wkt = cm->text;
    return r;
}

Status E57Reader::Read(size_t task, const OutputSpec& spec, BatchSink& sink) const {
    const Task& t = tasks_[task];
    const Scan& s = scans_[t.scan];
    const size_t nf = s.fields.size();
    thread_local std::vector<std::vector<uint8_t>> streams;
    thread_local std::vector<std::vector<double>> values;
    streams.resize(nf);
    values.resize(nf);
    // Each field's values of the records [first, first + count).
    for (size_t f = 0; f < nf; ++f) {
        const Scan::FieldDef& fd = s.fields[f];
        std::vector<double>& out = values[f];
        out.resize(static_cast<size_t>(t.count));
        if (fd.width == 0) {
            const double v = fd.kind == Scan::FieldDef::kScaled ? static_cast<double>(fd.min) * fd.scale + fd.offset
                                                                : static_cast<double>(fd.min);
            std::fill(out.begin(), out.end(), v);
            continue;
        }
        // records * width fits 64 bits (checked at open).
        const uint64_t bit0 = t.first * static_cast<uint64_t>(fd.width);
        const uint64_t byte0 = bit0 / 8;
        const uint64_t byte1 = ((t.first + t.count) * static_cast<uint64_t>(fd.width) + 7) / 8;
        std::vector<uint8_t>& buf = streams[f];
        buf.assign(static_cast<size_t>(byte1 - byte0) + 16, 0);
        // Copy the stream bytes [byte0, byte1) from the packets.
        size_t c = static_cast<size_t>(std::upper_bound(fd.chunkBefore.begin(), fd.chunkBefore.end(), byte0) - fd.chunkBefore.begin()) - 1;
        uint64_t pos = byte0;
        uint8_t* w = buf.data();
        while (pos < byte1) {
            if (c >= fd.chunkLogical.size()) return Corrupt("E57 field " + fd.name + " ends early");
            const uint64_t in = pos - fd.chunkBefore[c];
            const uint64_t take = std::min(fd.chunkBytes[c] - in, byte1 - pos);
            DDBT_TRY(ReadLogical(fd.chunkLogical[c] + in, w, static_cast<size_t>(take)));
            w += take;
            pos += take;
            ++c;
        }
        const uint8_t* p = buf.data();
        const uint64_t shift0 = bit0 % 8;
        const int wd = fd.width;
        const uint64_t mask = wd >= 64 ? ~uint64_t{0} : ((uint64_t{1} << wd) - 1);
        for (uint64_t i = 0; i < t.count; ++i) {
            const uint64_t bit = shift0 + i * static_cast<uint64_t>(wd);
            const uint8_t* q = p + bit / 8;
            const unsigned sh = static_cast<unsigned>(bit % 8);
            uint64_t raw = LoadLE<uint64_t>(q) >> sh;
            if (sh + static_cast<unsigned>(wd) > 64) raw |= static_cast<uint64_t>(q[8]) << (64 - sh);
            raw &= mask;
            double v;
            switch (fd.kind) {
                case Scan::FieldDef::kF32: v = std::bit_cast<float>(static_cast<uint32_t>(raw)); break;
                case Scan::FieldDef::kF64: v = std::bit_cast<double>(raw); break;
                case Scan::FieldDef::kInt: v = static_cast<double>(static_cast<int64_t>(raw + static_cast<uint64_t>(fd.min))); break;
                default:
                    v = static_cast<double>(static_cast<int64_t>(raw + static_cast<uint64_t>(fd.min))) * fd.scale + fd.offset;
                    break;
            }
            out[static_cast<size_t>(i)] = v;
        }
    }
    // Records.
    const size_t rs = spec.RecordSize();
    constexpr size_t kBatch = 1u << 16;
    thread_local std::vector<uint8_t> recs;
    thread_local std::vector<double> xyz;
    recs.resize(kBatch * rs);
    xyz.resize(kBatch * 3);
    int fx = -1, fy = -1, fz = -1, fr = -1, fa = -1, fe = -1;
    for (size_t f = 0; f < nf; ++f) {
        switch (s.fields[f].role) {
            case Role::kX: fx = static_cast<int>(f); break;
            case Role::kY: fy = static_cast<int>(f); break;
            case Role::kZ: fz = static_cast<int>(f); break;
            case Role::kRange: fr = static_cast<int>(f); break;
            case Role::kAzimuth: fa = static_cast<int>(f); break;
            case Role::kElevation: fe = static_cast<int>(f); break;
            default: break;
        }
    }
    for (uint64_t done = 0; done < t.count;) {
        const size_t n = static_cast<size_t>(std::min<uint64_t>(kBatch, t.count - done));
        for (size_t i = 0; i < n; ++i) {
            const size_t k = static_cast<size_t>(done) + i;
            uint8_t* r = recs.data() + i * rs;
            InitRecord(r, rs);
            double x, y, z;
            if (s.spherical) {
                const double range = values[static_cast<size_t>(fr)][k], az = values[static_cast<size_t>(fa)][k],
                             el = values[static_cast<size_t>(fe)][k];
                x = range * std::cos(el) * std::cos(az);
                y = range * std::cos(el) * std::sin(az);
                z = range * std::sin(el);
            } else {
                x = values[static_cast<size_t>(fx)][k];
                y = values[static_cast<size_t>(fy)][k];
                z = values[static_cast<size_t>(fz)][k];
            }
            double* q = xyz.data() + 3 * i;
            if (s.hasPose) {
                q[0] = Unfused3(x, s.rot[0][0], y, s.rot[0][1], z, s.rot[0][2], s.trans[0]);
                q[1] = Unfused3(x, s.rot[1][0], y, s.rot[1][1], z, s.rot[1][2], s.trans[1]);
                q[2] = Unfused3(x, s.rot[2][0], y, s.rot[2][1], z, s.rot[2][2], s.trans[2]);
            } else {
                q[0] = x;
                q[1] = y;
                q[2] = z;
            }
            for (size_t f = 0; f < nf; ++f) {
                const double v = values[f][k];
                switch (s.fields[f].role) {
                    case Role::kRed: SetField(Field::kRed, v * s.colourFactor[0], r, q, spec.format, 1.0, 1.0); break;
                    case Role::kGreen: SetField(Field::kGreen, v * s.colourFactor[1], r, q, spec.format, 1.0, 1.0); break;
                    case Role::kBlue: SetField(Field::kBlue, v * s.colourFactor[2], r, q, spec.format, 1.0, 1.0); break;
                    case Role::kIntensity: SetField(Field::kIntensity, v * s.intensityFactor, r, q, spec.format, 1.0, 1.0); break;
                    case Role::kClassification: SetField(Field::kClassification, v * s.classFactor, r, q, spec.format, 1.0, 1.0); break;
                    case Role::kReturnIndex: SetField(Field::kReturnNumber, v + 1, r, q, spec.format, 1.0, 1.0); break;
                    case Role::kReturnCount: SetField(Field::kNumberOfReturns, v, r, q, spec.format, 1.0, 1.0); break;
                    case Role::kTime: SetField(Field::kGpsTime, v, r, q, spec.format, 1.0, 1.0); break;
                    default: break;
                }
            }
        }
        Batch b{n, recs.data(), xyz.data()};
        DDBT_TRY(sink.Consume(b));
        done += n;
    }
    return Status();
}

}  // namespace ddbt::copc::readers
