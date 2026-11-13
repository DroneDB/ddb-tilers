// SPDX-License-Identifier: MPL-2.0
//
// A small JSON reader and writer for the statistics sidecar (<raster>.stats.json): objects,
// arrays, strings, numbers (as double, or uint64 when they are non-negative integers), booleans,
// null. Malformed text gives nullopt; no exceptions.
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ddbt::cog::tiler::json {

struct Value {
    enum class Kind : uint8_t { kNull, kBool, kNumber, kString, kArray, kObject };
    Kind kind = Kind::kNull;
    bool b = false;
    double num = 0;
    bool isUint = false;  // the number is a non-negative integer that fits uint64
    uint64_t u = 0;
    std::string str;
    std::vector<Value> arr;
    std::map<std::string, Value> obj;

    bool IsObject() const noexcept { return kind == Kind::kObject; }
    bool IsNumber() const noexcept { return kind == Kind::kNumber; }
    /// Member `key` of an object, or null.
    const Value* Find(std::string_view key) const;
};

std::optional<Value> Parse(std::string_view text);

/// Serialises with 2-space indentation, like nlohmann::json::dump(2) (keys sorted, numbers
/// with 17 significant digits, integers as integers).
std::string Dump(const Value& v, int indent = 2);

Value Object();
Value Number(double v);
Value Unsigned(uint64_t v);
Value Bool(bool v);
Value String(std::string v);

}  // namespace ddbt::cog::tiler::json
