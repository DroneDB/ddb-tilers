// SPDX-License-Identifier: MPL-2.0
// Minimal JSON writer and parser for ddbt-bench (no external dependencies).
#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace ddbtb
{

// Streaming writer. Containers are pretty-printed unless opened "inline" (one line).
class JsonWriter
{
  public:
    explicit JsonWriter(std::string &out) : out_(out) {}

    JsonWriter &beginObject(bool inlined = false);
    JsonWriter &endObject();
    JsonWriter &beginArray(bool inlined = false);
    JsonWriter &endArray();
    JsonWriter &key(std::string_view k);

    JsonWriter &value(std::string_view v);
    JsonWriter &value(const char *v) { return v ? value(std::string_view(v)) : null(); }
    JsonWriter &value(const std::string &v) { return value(std::string_view(v)); }
    JsonWriter &value(bool v);
    template <typename T>
        requires(std::is_integral_v<T> && !std::is_same_v<T, bool>)
    JsonWriter &value(T v)
    {
        if constexpr (std::is_signed_v<T>)
            return valueI64(static_cast<int64_t>(v));
        else
            return valueU64(static_cast<uint64_t>(v));
    }
    JsonWriter &value(double v);
    JsonWriter &null();
    // Raw, already-serialized JSON (must be valid).
    JsonWriter &raw(std::string_view json);

    template <typename T> JsonWriter &kv(std::string_view k, const T &v)
    {
        key(k);
        return value(v);
    }

    static std::string escape(std::string_view s);
    static std::string hex64(uint64_t v);

  private:
    struct Level
    {
        bool array;
        bool inlined;
        bool empty;
    };
    void beforeValue();
    void newline();
    JsonWriter &valueI64(int64_t v);
    JsonWriter &valueU64(uint64_t v);

    std::string &out_;
    std::vector<Level> stack_;
    bool afterKey_ = false;
};

// Parsed JSON value (small DOM, enough for request files and configuration).
struct JsonValue
{
    enum class Type
    {
        Null,
        Bool,
        Number,
        String,
        Array,
        Object
    };
    Type type = Type::Null;
    bool b = false;
    double n = 0.0;
    std::string s;
    std::vector<JsonValue> a;
    std::vector<std::pair<std::string, JsonValue>> o;

    const JsonValue *get(std::string_view k) const;
    bool isNull() const { return type == Type::Null; }
    std::string str(std::string_view k, const std::string &def = {}) const;
    double num(std::string_view k, double def) const;
    bool boolean(std::string_view k, bool def) const;
};

// Parses text into out. Returns false and sets err (with offset) on malformed input.
bool parseJson(std::string_view text, JsonValue &out, std::string &err);

} // namespace ddbtb
