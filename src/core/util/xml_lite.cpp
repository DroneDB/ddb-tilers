// SPDX-License-Identifier: MPL-2.0
#include "core/util/xml_lite.h"

#include <cstdint>

namespace ddbt::xml {

namespace {

constexpr int kMaxDepth = 256;

std::string StripPrefix(std::string_view name) {
    const size_t colon = name.find(':');
    return std::string(colon == std::string_view::npos ? name : name.substr(colon + 1));
}

bool IsSpace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }
bool IsNameChar(char c) { return !IsSpace(c) && c != '>' && c != '/' && c != '=' && c != '<' && c != '\0'; }

void AppendUtf8(std::string& out, uint32_t cp) {
    if (cp < 0x80) {
        out += static_cast<char>(cp);
    } else if (cp < 0x800) {
        out += static_cast<char>(0xC0 | (cp >> 6));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
        out += static_cast<char>(0xE0 | (cp >> 12));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | (cp >> 18));
        out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    }
}

// Text with entity and character references decoded (an unknown reference is kept as is).
std::string Decode(std::string_view s) {
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] != '&') {
            out += s[i];
            continue;
        }
        const size_t semi = s.find(';', i);
        if (semi == std::string_view::npos || semi - i > 12) {
            out += s[i];
            continue;
        }
        const std::string_view ref = s.substr(i + 1, semi - i - 1);
        if (ref == "lt") out += '<';
        else if (ref == "gt") out += '>';
        else if (ref == "amp") out += '&';
        else if (ref == "quot") out += '"';
        else if (ref == "apos") out += '\'';
        else if (ref.size() > 1 && ref[0] == '#') {
            uint32_t cp = 0;
            const bool hex = ref[1] == 'x' || ref[1] == 'X';
            bool ok = ref.size() > (hex ? 2u : 1u);
            for (size_t k = hex ? 2 : 1; ok && k < ref.size(); ++k) {
                const char c = ref[k];
                uint32_t d;
                if (c >= '0' && c <= '9') d = static_cast<uint32_t>(c - '0');
                else if (hex && c >= 'a' && c <= 'f') d = static_cast<uint32_t>(c - 'a' + 10);
                else if (hex && c >= 'A' && c <= 'F') d = static_cast<uint32_t>(c - 'A' + 10);
                else ok = false;
                if (ok) cp = cp * (hex ? 16u : 10u) + d;
                if (cp > 0x10FFFF) ok = false;
            }
            if (!ok) {
                out += s[i];
                continue;
            }
            AppendUtf8(out, cp);
        } else {
            out += s[i];
            continue;
        }
        i = semi;
    }
    return out;
}

class Parser {
public:
    explicit Parser(std::string_view s) : s_(s) {}

    std::optional<Element> Document() {
        SkipMisc();
        if (!At("<")) return std::nullopt;
        Element root;
        if (!ParseElement(root, 0)) return std::nullopt;
        SkipMisc();
        return root;
    }

private:
    bool At(std::string_view t) const { return s_.substr(p_, t.size()) == t; }

    void SkipSpace() {
        while (p_ < s_.size() && IsSpace(s_[p_])) ++p_;
    }

    // Skips what may surround or precede elements: spaces, the XML declaration, processing
    // instructions, comments and the document type.
    void SkipMisc() {
        for (;;) {
            SkipSpace();
            if (At("<?")) {
                const size_t e = s_.find("?>", p_);
                p_ = e == std::string_view::npos ? s_.size() : e + 2;
            } else if (At("<!--")) {
                const size_t e = s_.find("-->", p_);
                p_ = e == std::string_view::npos ? s_.size() : e + 3;
            } else if (At("<!DOCTYPE") || At("<!doctype")) {
                // A document type without an internal subset, or with one in brackets.
                int depth = 0;
                while (p_ < s_.size()) {
                    const char c = s_[p_++];
                    if (c == '[') ++depth;
                    else if (c == ']') --depth;
                    else if (c == '>' && depth <= 0) break;
                }
            } else {
                return;
            }
        }
    }

    bool ParseName(std::string& name) {
        const size_t b = p_;
        while (p_ < s_.size() && IsNameChar(s_[p_])) ++p_;
        if (p_ == b) return false;
        name.assign(s_.substr(b, p_ - b));
        return true;
    }

    bool ParseElement(Element& e, int depth) {
        if (depth > kMaxDepth || !At("<")) return false;
        ++p_;
        std::string qname;
        if (!ParseName(qname)) return false;
        e.name = StripPrefix(qname);
        for (;;) {
            SkipSpace();
            if (p_ >= s_.size()) return false;
            if (At("/>")) {
                p_ += 2;
                return true;
            }
            if (s_[p_] == '>') {
                ++p_;
                break;
            }
            std::string an;
            if (!ParseName(an)) return false;
            SkipSpace();
            if (!At("=")) return false;
            ++p_;
            SkipSpace();
            if (p_ >= s_.size() || (s_[p_] != '"' && s_[p_] != '\'')) return false;
            const char q = s_[p_++];
            const size_t end = s_.find(q, p_);
            if (end == std::string_view::npos) return false;
            e.attributes.emplace_back(StripPrefix(an), Decode(s_.substr(p_, end - p_)));
            p_ = end + 1;
        }
        // Content
        for (;;) {
            if (p_ >= s_.size()) return false;
            if (At("</")) {
                p_ += 2;
                std::string closing;
                if (!ParseName(closing) || closing != qname) return false;
                SkipSpace();
                if (!At(">")) return false;
                ++p_;
                return true;
            }
            if (At("<!--")) {
                const size_t end = s_.find("-->", p_);
                if (end == std::string_view::npos) return false;
                p_ = end + 3;
            } else if (At("<![CDATA[")) {
                const size_t end = s_.find("]]>", p_);
                if (end == std::string_view::npos) return false;
                e.text.append(s_.substr(p_ + 9, end - p_ - 9));
                p_ = end + 3;
            } else if (At("<?")) {
                const size_t end = s_.find("?>", p_);
                if (end == std::string_view::npos) return false;
                p_ = end + 2;
            } else if (s_[p_] == '<') {
                Element child;
                if (!ParseElement(child, depth + 1)) return false;
                e.children.push_back(std::move(child));
            } else {
                const size_t end = s_.find('<', p_);
                if (end == std::string_view::npos) return false;
                e.text += Decode(s_.substr(p_, end - p_));
                p_ = end;
            }
        }
    }

    std::string_view s_;
    size_t p_ = 0;
};

}  // namespace

const std::string* Element::Attribute(std::string_view attributeName) const {
    for (const auto& [k, v] : attributes)
        if (k == attributeName) return &v;
    return nullptr;
}

const Element* Element::Child(std::string_view childName) const {
    for (const Element& c : children)
        if (c.name == childName) return &c;
    return nullptr;
}

const Element* Element::Path(std::string_view path) const {
    const Element* cur = this;
    while (cur != nullptr && !path.empty()) {
        const size_t dot = path.find('.');
        cur = cur->Child(path.substr(0, dot));
        path = dot == std::string_view::npos ? std::string_view() : path.substr(dot + 1);
    }
    return cur;
}

const Element* Element::Find(std::string_view elementName) const {
    if (name == elementName) return this;
    for (const Element& c : children)
        if (const Element* f = c.Find(elementName)) return f;
    return nullptr;
}

std::optional<Element> Parse(std::string_view text) { return Parser(text).Document(); }

std::string Escape(std::string_view text, bool attribute) {
    std::string out;
    out.reserve(text.size());
    for (const char c : text) {
        if (c == '&') out += "&amp;";
        else if (c == '<') out += "&lt;";
        else if (c == '>') out += "&gt;";
        else if (c == '"' && attribute) out += "&quot;";
        else out += c;
    }
    return out;
}

}  // namespace ddbt::xml
