// SPDX-License-Identifier: MPL-2.0
#include "cog/tiler/json_lite.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace ddbt::cog::tiler::json {

namespace {

class Parser {
public:
    explicit Parser(std::string_view t) : t_(t) {}

    std::optional<Value> Run() {
        Value v;
        if (!ParseValue(v, 0)) return std::nullopt;
        Ws();
        if (p_ != t_.size()) return std::nullopt;
        return v;
    }

private:
    void Ws() {
        while (p_ < t_.size() && (t_[p_] == ' ' || t_[p_] == '\n' || t_[p_] == '\r' || t_[p_] == '\t')) ++p_;
    }
    bool Lit(std::string_view s) {
        if (t_.substr(p_, s.size()) != s) return false;
        p_ += s.size();
        return true;
    }
    bool ParseString(std::string& out) {
        if (p_ >= t_.size() || t_[p_] != '"') return false;
        ++p_;
        while (p_ < t_.size()) {
            const char c = t_[p_++];
            if (c == '"') return true;
            if (c != '\\') {
                out += c;
                continue;
            }
            if (p_ >= t_.size()) return false;
            const char e = t_[p_++];
            switch (e) {
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    if (p_ + 4 > t_.size()) return false;
                    const unsigned cp = static_cast<unsigned>(std::strtoul(std::string(t_.substr(p_, 4)).c_str(), nullptr, 16));
                    p_ += 4;
                    if (cp < 0x80) {
                        out += static_cast<char>(cp);
                    } else if (cp < 0x800) {
                        out += static_cast<char>(0xC0 | (cp >> 6));
                        out += static_cast<char>(0x80 | (cp & 0x3F));
                    } else {
                        out += static_cast<char>(0xE0 | (cp >> 12));
                        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
                        out += static_cast<char>(0x80 | (cp & 0x3F));
                    }
                    break;
                }
                default: return false;
            }
        }
        return false;
    }
    bool ParseValue(Value& v, int depth) {
        if (depth > 64) return false;
        Ws();
        if (p_ >= t_.size()) return false;
        const char c = t_[p_];
        if (c == '{') {
            ++p_;
            v.kind = Value::Kind::kObject;
            Ws();
            if (p_ < t_.size() && t_[p_] == '}') {
                ++p_;
                return true;
            }
            for (;;) {
                Ws();
                std::string key;
                if (!ParseString(key)) return false;
                Ws();
                if (p_ >= t_.size() || t_[p_] != ':') return false;
                ++p_;
                Value child;
                if (!ParseValue(child, depth + 1)) return false;
                v.obj[key] = std::move(child);
                Ws();
                if (p_ < t_.size() && t_[p_] == ',') {
                    ++p_;
                    continue;
                }
                if (p_ < t_.size() && t_[p_] == '}') {
                    ++p_;
                    return true;
                }
                return false;
            }
        }
        if (c == '[') {
            ++p_;
            v.kind = Value::Kind::kArray;
            Ws();
            if (p_ < t_.size() && t_[p_] == ']') {
                ++p_;
                return true;
            }
            for (;;) {
                Value child;
                if (!ParseValue(child, depth + 1)) return false;
                v.arr.push_back(std::move(child));
                Ws();
                if (p_ < t_.size() && t_[p_] == ',') {
                    ++p_;
                    continue;
                }
                if (p_ < t_.size() && t_[p_] == ']') {
                    ++p_;
                    return true;
                }
                return false;
            }
        }
        if (c == '"') {
            v.kind = Value::Kind::kString;
            return ParseString(v.str);
        }
        if (Lit("true")) {
            v.kind = Value::Kind::kBool;
            v.b = true;
            return true;
        }
        if (Lit("false")) {
            v.kind = Value::Kind::kBool;
            return true;
        }
        if (Lit("null")) return true;
        // Number
        const size_t start = p_;
        if (p_ < t_.size() && (t_[p_] == '-' || t_[p_] == '+')) ++p_;
        bool integer = true;
        while (p_ < t_.size()) {
            const char d = t_[p_];
            if (d >= '0' && d <= '9') {
                ++p_;
            } else if (d == '.' || d == 'e' || d == 'E' || d == '-' || d == '+') {
                integer = false;
                ++p_;
            } else {
                break;
            }
        }
        if (p_ == start) return false;
        const std::string s(t_.substr(start, p_ - start));
        char* end = nullptr;
        v.kind = Value::Kind::kNumber;
        v.num = std::strtod(s.c_str(), &end);
        if (end == s.c_str()) return false;
        if (integer && s[0] != '-') {
            v.isUint = true;
            v.u = std::strtoull(s.c_str(), nullptr, 10);
        }
        return true;
    }

    std::string_view t_;
    size_t p_ = 0;
};

void Escape(std::string& out, const std::string& s) {
    out += '"';
    for (const char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    out += '"';
}

void DumpTo(std::string& out, const Value& v, int indent, int level) {
    const std::string pad(static_cast<size_t>(indent * (level + 1)), ' ');
    const std::string padEnd(static_cast<size_t>(indent * level), ' ');
    switch (v.kind) {
        case Value::Kind::kNull: out += "null"; break;
        case Value::Kind::kBool: out += v.b ? "true" : "false"; break;
        case Value::Kind::kNumber: {
            char buf[40];
            if (v.isUint) {
                std::snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(v.u));
            } else if (std::isfinite(v.num)) {
                std::snprintf(buf, sizeof(buf), "%.17g", v.num);
                // nlohmann writes a double with a fraction or exponent ("1.0").
                std::string s = buf;
                if (s.find_first_of(".eEn") == std::string::npos) s += ".0";
                out += s;
                break;
            } else {
                std::snprintf(buf, sizeof(buf), "null");
            }
            out += buf;
            break;
        }
        case Value::Kind::kString: Escape(out, v.str); break;
        case Value::Kind::kArray: {
            if (v.arr.empty()) {
                out += "[]";
                break;
            }
            out += "[\n";
            for (size_t i = 0; i < v.arr.size(); ++i) {
                out += pad;
                DumpTo(out, v.arr[i], indent, level + 1);
                if (i + 1 < v.arr.size()) out += ',';
                out += '\n';
            }
            out += padEnd + "]";
            break;
        }
        case Value::Kind::kObject: {
            if (v.obj.empty()) {
                out += "{}";
                break;
            }
            out += "{\n";
            size_t i = 0;
            for (const auto& [k, child] : v.obj) {
                out += pad;
                Escape(out, k);
                out += ": ";
                DumpTo(out, child, indent, level + 1);
                if (++i < v.obj.size()) out += ',';
                out += '\n';
            }
            out += padEnd + "}";
            break;
        }
    }
}

}  // namespace

const Value* Value::Find(std::string_view key) const {
    if (kind != Kind::kObject) return nullptr;
    const auto it = obj.find(std::string(key));
    return it == obj.end() ? nullptr : &it->second;
}

std::optional<Value> Parse(std::string_view text) { return Parser(text).Run(); }

std::string Dump(const Value& v, int indent) {
    std::string out;
    DumpTo(out, v, indent, 0);
    return out;
}

Value Object() {
    Value v;
    v.kind = Value::Kind::kObject;
    return v;
}
Value Number(double d) {
    Value v;
    v.kind = Value::Kind::kNumber;
    v.num = d;
    return v;
}
Value Unsigned(uint64_t u) {
    Value v;
    v.kind = Value::Kind::kNumber;
    v.isUint = true;
    v.u = u;
    v.num = static_cast<double>(u);
    return v;
}
Value Bool(bool b) {
    Value v;
    v.kind = Value::Kind::kBool;
    v.b = b;
    return v;
}
Value String(std::string s) {
    Value v;
    v.kind = Value::Kind::kString;
    v.str = std::move(s);
    return v;
}

}  // namespace ddbt::cog::tiler::json
