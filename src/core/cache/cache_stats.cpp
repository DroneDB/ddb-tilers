// SPDX-License-Identifier: MPL-2.0
#include "core/cache/cache_stats.h"

#include <utility>

namespace ddbt::cache {

std::string ToJson(const CacheStats& s) {
    const std::pair<const char*, uint64_t> fields[] = {
        {"hits", s.hits},       {"misses", s.misses},           {"loads", s.loads},
        {"coalesced", s.coalesced}, {"load_failures", s.loadFailures},
        {"inserts", s.inserts}, {"evictions", s.evictions},     {"rejected", s.rejected},
        {"entries", s.entries}, {"bytes", s.bytes},
    };
    std::string out = "{";
    for (const auto& [name, value] : fields) {
        if (out.size() > 1) out += ',';
        out += '"';
        out += name;
        out += "\":";
        out += std::to_string(value);
    }
    out += '}';
    return out;
}

}  // namespace ddbt::cache
