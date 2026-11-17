// SPDX-License-Identifier: MPL-2.0
//
// The tab-separated reference files the GDAL scripts of tests/unit/tiff/data write.
#pragma once

#include <string>
#include <vector>

namespace ddbt::cog::tiff::testing {

/// The fields of a line, split at tabs.
inline std::vector<std::string> SplitTsv(const std::string& line) {
    std::vector<std::string> f;
    std::string cur;
    for (char c : line) {
        if (c == '\t') {
            f.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    f.push_back(cur);
    return f;
}

/// A field whose newlines were written as "\n" (a WKT).
inline std::string UnescapeNewlines(const std::string& s) {
    std::string o;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '\\' && i + 1 < s.size() && s[i + 1] == 'n') {
            o.push_back('\n');
            ++i;
        } else {
            o.push_back(s[i]);
        }
    }
    return o;
}

}  // namespace ddbt::cog::tiff::testing
