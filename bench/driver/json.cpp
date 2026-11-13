// SPDX-License-Identifier: MPL-2.0
#include "json.hpp"

#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace ddbtb
{

// ---------------------------------------------------------------- writer

void JsonWriter::newline()
{
    out_.push_back('\n');
    out_.append(stack_.size() * 2, ' ');
}

void JsonWriter::beforeValue()
{
    if (afterKey_)
    {
        afterKey_ = false;
        return;
    }
    if (stack_.empty())
        return;
    Level &l = stack_.back();
    if (!l.empty)
        out_.push_back(',');
    l.empty = false;
    if (l.inlined)
    {
        if (out_.back() == ',')
            out_.push_back(' ');
    }
    else
        newline();
}

JsonWriter &JsonWriter::beginObject(bool inlined)
{
    beforeValue();
    out_.push_back('{');
    // A container inside an inline container is inline as well.
    const bool parentInline = !stack_.empty() && stack_.back().inlined;
    stack_.push_back({false, inlined || parentInline, true});
    return *this;
}

JsonWriter &JsonWriter::endObject()
{
    const Level l = stack_.back();
    stack_.pop_back();
    if (!l.inlined && !l.empty)
        newline();
    out_.push_back('}');
    if (stack_.empty())
        out_.push_back('\n');
    return *this;
}

JsonWriter &JsonWriter::beginArray(bool inlined)
{
    beforeValue();
    out_.push_back('[');
    const bool parentInline = !stack_.empty() && stack_.back().inlined;
    stack_.push_back({true, inlined || parentInline, true});
    return *this;
}

JsonWriter &JsonWriter::endArray()
{
    const Level l = stack_.back();
    stack_.pop_back();
    if (!l.inlined && !l.empty)
        newline();
    out_.push_back(']');
    if (stack_.empty())
        out_.push_back('\n');
    return *this;
}

JsonWriter &JsonWriter::key(std::string_view k)
{
    beforeValue();
    out_.push_back('"');
    out_.append(escape(k));
    out_.append("\": ");
    afterKey_ = true;
    return *this;
}

std::string JsonWriter::escape(std::string_view s)
{
    std::string r;
    r.reserve(s.size() + 8);
    for (const char ch : s)
    {
        const auto c = static_cast<unsigned char>(ch);
        switch (c)
        {
        case '"':
            r += "\\\"";
            break;
        case '\\':
            r += "\\\\";
            break;
        case '\n':
            r += "\\n";
            break;
        case '\r':
            r += "\\r";
            break;
        case '\t':
            r += "\\t";
            break;
        default:
            if (c < 0x20)
            {
                char buf[8];
                std::snprintf(buf, sizeof buf, "\\u%04x", c);
                r += buf;
            }
            else
                r.push_back(ch);
        }
    }
    return r;
}

std::string JsonWriter::hex64(uint64_t v)
{
    char buf[24];
    std::snprintf(buf, sizeof buf, "%016llx", static_cast<unsigned long long>(v));
    return buf;
}

JsonWriter &JsonWriter::value(std::string_view v)
{
    beforeValue();
    out_.push_back('"');
    out_.append(escape(v));
    out_.push_back('"');
    return *this;
}

JsonWriter &JsonWriter::value(bool v)
{
    beforeValue();
    out_.append(v ? "true" : "false");
    return *this;
}

JsonWriter &JsonWriter::valueI64(int64_t v)
{
    beforeValue();
    char buf[32];
    const auto r = std::to_chars(buf, buf + sizeof buf, v);
    out_.append(buf, r.ptr);
    return *this;
}

JsonWriter &JsonWriter::valueU64(uint64_t v)
{
    beforeValue();
    char buf[32];
    const auto r = std::to_chars(buf, buf + sizeof buf, v);
    out_.append(buf, r.ptr);
    return *this;
}

JsonWriter &JsonWriter::value(double v)
{
    if (!std::isfinite(v))
        return null();
    beforeValue();
    char buf[64];
    // Shortest round-trip representation.
    const auto r = std::to_chars(buf, buf + sizeof buf, v);
    out_.append(buf, r.ptr);
    return *this;
}

JsonWriter &JsonWriter::null()
{
    beforeValue();
    out_.append("null");
    return *this;
}

JsonWriter &JsonWriter::raw(std::string_view json)
{
    beforeValue();
    out_.append(json);
    return *this;
}

// ---------------------------------------------------------------- DOM

const JsonValue *JsonValue::get(std::string_view k) const
{
    if (type != Type::Object)
        return nullptr;
    for (const auto &[name, v] : o)
        if (name == k)
            return &v;
    return nullptr;
}

std::string JsonValue::str(std::string_view k, const std::string &def) const
{
    const JsonValue *v = get(k);
    return v && v->type == Type::String ? v->s : def;
}

double JsonValue::num(std::string_view k, double def) const
{
    const JsonValue *v = get(k);
    return v && v->type == Type::Number ? v->n : def;
}

bool JsonValue::boolean(std::string_view k, bool def) const
{
    const JsonValue *v = get(k);
    if (!v)
        return def;
    if (v->type == Type::Bool)
        return v->b;
    if (v->type == Type::Number)
        return v->n != 0.0;
    return def;
}

// ---------------------------------------------------------------- parser

namespace
{
class Parser
{
  public:
    Parser(std::string_view t, std::string &err) : t_(t), err_(err) {}

    bool parseDocument(JsonValue &out)
    {
        ws();
        if (!parseValue(out, 0))
            return false;
        ws();
        if (p_ != t_.size())
            return fail("trailing characters");
        return true;
    }

  private:
    bool fail(const char *what)
    {
        err_ = std::string(what) + " at offset " + std::to_string(p_);
        return false;
    }
    void ws()
    {
        while (p_ < t_.size() && (t_[p_] == ' ' || t_[p_] == '\t' || t_[p_] == '\n' || t_[p_] == '\r'))
            ++p_;
    }
    bool lit(std::string_view w)
    {
        if (t_.substr(p_, w.size()) != w)
            return false;
        p_ += w.size();
        return true;
    }
    static void appendUtf8(std::string &s, uint32_t cp)
    {
        if (cp < 0x80)
            s.push_back(static_cast<char>(cp));
        else if (cp < 0x800)
        {
            s.push_back(static_cast<char>(0xC0 | (cp >> 6)));
            s.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        }
        else if (cp < 0x10000)
        {
            s.push_back(static_cast<char>(0xE0 | (cp >> 12)));
            s.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            s.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        }
        else
        {
            s.push_back(static_cast<char>(0xF0 | (cp >> 18)));
            s.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
            s.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            s.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        }
    }
    bool hex4(uint32_t &v)
    {
        if (p_ + 4 > t_.size())
            return false;
        v = 0;
        for (int i = 0; i < 4; ++i)
        {
            const char c = t_[p_++];
            v <<= 4;
            if (c >= '0' && c <= '9')
                v |= static_cast<uint32_t>(c - '0');
            else if (c >= 'a' && c <= 'f')
                v |= static_cast<uint32_t>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F')
                v |= static_cast<uint32_t>(c - 'A' + 10);
            else
                return false;
        }
        return true;
    }
    bool parseString(std::string &s)
    {
        ++p_; // opening quote
        while (p_ < t_.size())
        {
            const char c = t_[p_++];
            if (c == '"')
                return true;
            if (c != '\\')
            {
                s.push_back(c);
                continue;
            }
            if (p_ >= t_.size())
                break;
            const char e = t_[p_++];
            switch (e)
            {
            case '"':
            case '\\':
            case '/':
                s.push_back(e);
                break;
            case 'b':
                s.push_back('\b');
                break;
            case 'f':
                s.push_back('\f');
                break;
            case 'n':
                s.push_back('\n');
                break;
            case 'r':
                s.push_back('\r');
                break;
            case 't':
                s.push_back('\t');
                break;
            case 'u':
            {
                uint32_t cp;
                if (!hex4(cp))
                    return fail("bad \\u escape");
                if (cp >= 0xD800 && cp < 0xDC00 && lit("\\u"))
                {
                    uint32_t lo;
                    if (!hex4(lo))
                        return fail("bad surrogate");
                    cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                }
                appendUtf8(s, cp);
                break;
            }
            default:
                return fail("bad escape");
            }
        }
        return fail("unterminated string");
    }
    bool parseValue(JsonValue &v, int depth)
    {
        if (depth > 64)
            return fail("nesting too deep");
        if (p_ >= t_.size())
            return fail("unexpected end");
        const char c = t_[p_];
        if (c == '{')
        {
            v.type = JsonValue::Type::Object;
            ++p_;
            ws();
            if (p_ < t_.size() && t_[p_] == '}')
            {
                ++p_;
                return true;
            }
            for (;;)
            {
                ws();
                if (p_ >= t_.size() || t_[p_] != '"')
                    return fail("expected key");
                std::string k;
                if (!parseString(k))
                    return false;
                ws();
                if (p_ >= t_.size() || t_[p_] != ':')
                    return fail("expected ':'");
                ++p_;
                ws();
                JsonValue child;
                if (!parseValue(child, depth + 1))
                    return false;
                v.o.emplace_back(std::move(k), std::move(child));
                ws();
                if (p_ < t_.size() && t_[p_] == ',')
                {
                    ++p_;
                    continue;
                }
                if (p_ < t_.size() && t_[p_] == '}')
                {
                    ++p_;
                    return true;
                }
                return fail("expected ',' or '}'");
            }
        }
        if (c == '[')
        {
            v.type = JsonValue::Type::Array;
            ++p_;
            ws();
            if (p_ < t_.size() && t_[p_] == ']')
            {
                ++p_;
                return true;
            }
            for (;;)
            {
                ws();
                JsonValue child;
                if (!parseValue(child, depth + 1))
                    return false;
                v.a.push_back(std::move(child));
                ws();
                if (p_ < t_.size() && t_[p_] == ',')
                {
                    ++p_;
                    continue;
                }
                if (p_ < t_.size() && t_[p_] == ']')
                {
                    ++p_;
                    return true;
                }
                return fail("expected ',' or ']'");
            }
        }
        if (c == '"')
        {
            v.type = JsonValue::Type::String;
            return parseString(v.s);
        }
        if (lit("true"))
        {
            v.type = JsonValue::Type::Bool;
            v.b = true;
            return true;
        }
        if (lit("false"))
        {
            v.type = JsonValue::Type::Bool;
            v.b = false;
            return true;
        }
        if (lit("null"))
        {
            v.type = JsonValue::Type::Null;
            return true;
        }
        // Number
        const size_t start = p_;
        while (p_ < t_.size() && (std::isdigit(static_cast<unsigned char>(t_[p_])) || t_[p_] == '-' ||
                                  t_[p_] == '+' || t_[p_] == '.' || t_[p_] == 'e' || t_[p_] == 'E'))
            ++p_;
        if (start == p_)
            return fail("unexpected character");
        const std::string num(t_.substr(start, p_ - start));
        char *end = nullptr;
        v.type = JsonValue::Type::Number;
        v.n = std::strtod(num.c_str(), &end);
        if (end != num.c_str() + num.size())
            return fail("bad number");
        return true;
    }

    std::string_view t_;
    std::string &err_;
    size_t p_ = 0;
};
} // namespace

bool parseJson(std::string_view text, JsonValue &out, std::string &err)
{
    out = JsonValue{};
    Parser p(text, err);
    return p.parseDocument(out);
}

} // namespace ddbtb
