// SPDX-License-Identifier: MPL-2.0
#include "core/util/counters.h"

namespace ddbt {

Counters& Counters::Global() {
    // Leaked on purpose: counters may be touched by detached threads and static destructors.
    static Counters* const g = new Counters();
    return *g;
}

Counter& Counters::Get(std::string_view name) {
    std::lock_guard lock(mutex_);
    if (auto it = byName_.find(name); it != byName_.end()) return *it->second;
    Counter& c = storage_.emplace_back();
    byName_.emplace(std::string(name), &c);
    return c;
}

std::vector<std::pair<std::string, uint64_t>> Counters::Snapshot() const {
    std::lock_guard lock(mutex_);
    std::vector<std::pair<std::string, uint64_t>> out;
    out.reserve(byName_.size());
    for (const auto& [name, c] : byName_) out.emplace_back(name, c->Load());
    return out;
}

std::string Counters::ToJson() const {
    std::string out = "{";
    bool first = true;
    for (const auto& [name, value] : Snapshot()) {
        if (!first) out += ',';
        first = false;
        out += '"';
        for (const char ch : name) {
            const auto u = static_cast<unsigned char>(ch);
            if (ch == '"' || ch == '\\') {
                out += '\\';
                out += ch;
            } else if (u < 0x20) {
                out += "\\u00";
                out += "0123456789abcdef"[u >> 4];
                out += "0123456789abcdef"[u & 0xF];
            } else {
                out += ch;
            }
        }
        out += "\":";
        out += std::to_string(value);
    }
    out += '}';
    return out;
}

void Counters::ResetAll() {
    std::lock_guard lock(mutex_);
    for (auto& c : storage_) c.Reset();
}

}  // namespace ddbt
