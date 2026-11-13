// SPDX-License-Identifier: MPL-2.0
#include "core/exec/topology.h"

#include <algorithm>
#include <charconv>
#include <fstream>
#include <map>
#include <sstream>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <sched.h>
#endif

namespace ddbt::exec {

bool ParseCpuList(std::string_view text, std::vector<int>* out) {
    out->clear();
    while (!text.empty() && (text.back() == '\n' || text.back() == ' ' || text.back() == '\r')) text.remove_suffix(1);
    if (text.empty()) return true;
    size_t pos = 0;
    while (pos <= text.size()) {
        const size_t comma = std::min(text.find(',', pos), text.size());
        const std::string_view item = text.substr(pos, comma - pos);
        const size_t dash = item.find('-');
        int lo = 0;
        int hi = 0;
        const auto parse = [](std::string_view s, int* v) {
            if (s.empty()) return false;
            const auto r = std::from_chars(s.data(), s.data() + s.size(), *v);
            return r.ec == std::errc() && r.ptr == s.data() + s.size() && *v >= 0;
        };
        if (dash == std::string_view::npos) {
            if (!parse(item, &lo)) return false;
            hi = lo;
        } else {
            if (!parse(item.substr(0, dash), &lo) || !parse(item.substr(dash + 1), &hi) || hi < lo) return false;
        }
        if (hi - lo > 1 << 16) return false;
        for (int c = lo; c <= hi; ++c) out->push_back(c);
        pos = comma + 1;
        if (comma == text.size()) break;
    }
    std::sort(out->begin(), out->end());
    out->erase(std::unique(out->begin(), out->end()), out->end());
    return true;
}

std::string FormatCpuList(std::vector<int> cpus) {
    std::sort(cpus.begin(), cpus.end());
    cpus.erase(std::unique(cpus.begin(), cpus.end()), cpus.end());
    std::string s;
    for (size_t i = 0; i < cpus.size();) {
        size_t j = i;
        while (j + 1 < cpus.size() && cpus[j + 1] == cpus[j] + 1) ++j;
        if (!s.empty()) s += ',';
        s += std::to_string(cpus[i]);
        if (j > i) {
            s += '-';
            s += std::to_string(cpus[j]);
        }
        i = j + 1;
    }
    return s;
}

int Topology::DomainOf(int cpu) const noexcept {
    for (size_t d = 0; d < l3.size(); ++d) {
        if (std::binary_search(l3[d].cpus.begin(), l3[d].cpus.end(), cpu)) return static_cast<int>(d);
    }
    return -1;
}

std::string Topology::ToString() const {
    std::string s = std::to_string(cpus.size()) + " CPUs, " + std::to_string(l3.size()) + " L3 domain" +
                    (l3.size() == 1 ? "" : "s") + ":";
    for (size_t d = 0; d < l3.size(); ++d) {
        s += d == 0 ? " [" : ", [";
        s += FormatCpuList(l3[d].cpus);
        s += ']';
        if (l3[d].cacheBytes != 0) s += ' ' + std::to_string(l3[d].cacheBytes >> 20) + " MiB";
    }
    return s;
}

namespace {

void Finalize(Topology* t) {
    // Every allowed CPU belongs to exactly one domain; unknown ones join a catch-all domain.
    std::vector<int> missing;
    for (int c : t->cpus) {
        if (t->DomainOf(c) < 0) missing.push_back(c);
    }
    if (!missing.empty()) t->l3.push_back(L3Domain{missing, 0});
    std::sort(t->l3.begin(), t->l3.end(), [](const L3Domain& a, const L3Domain& b) { return a.cpus.front() < b.cpus.front(); });
}

[[maybe_unused]] bool ReadSmallFile(const std::string& path, std::string* out) {
    std::ifstream f(path);
    if (!f) return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    *out = ss.str();
    return true;
}

[[maybe_unused]] uint64_t ParseCacheSize(std::string s) {
    while (!s.empty() && (s.back() == '\n' || s.back() == ' ')) s.pop_back();
    uint64_t mult = 1;
    if (!s.empty() && (s.back() == 'K' || s.back() == 'k')) mult = 1024;
    if (!s.empty() && (s.back() == 'M' || s.back() == 'm')) mult = 1024 * 1024;
    if (mult != 1) s.pop_back();
    uint64_t v = 0;
    const auto r = std::from_chars(s.data(), s.data() + s.size(), v);
    return r.ec == std::errc() ? v * mult : 0;
}

}  // namespace

Topology Topology::FromSysfs(const std::string& root, const std::vector<int>& allowed) {
    Topology t;
    t.cpus = allowed;
    std::sort(t.cpus.begin(), t.cpus.end());
    std::map<std::string, L3Domain> byList;  // shared_cpu_list text -> domain
    for (int cpu : t.cpus) {
        const std::string cacheDir = root + "/cpu" + std::to_string(cpu) + "/cache";
        for (int idx = 0; idx < 8; ++idx) {
            const std::string dir = cacheDir + "/index" + std::to_string(idx);
            std::string level;
            if (!ReadSmallFile(dir + "/level", &level)) break;
            if (level.empty() || level[0] != '3') continue;
            std::string list;
            std::vector<int> shared;
            if (!ReadSmallFile(dir + "/shared_cpu_list", &list) || !ParseCpuList(list, &shared) || shared.empty()) break;
            L3Domain& d = byList[FormatCpuList(shared)];
            if (std::find(d.cpus.begin(), d.cpus.end(), cpu) == d.cpus.end()) d.cpus.push_back(cpu);
            std::string size;
            if (d.cacheBytes == 0 && ReadSmallFile(dir + "/size", &size)) d.cacheBytes = ParseCacheSize(size);
            break;
        }
    }
    for (auto& [key, d] : byList) {
        std::sort(d.cpus.begin(), d.cpus.end());
        if (!d.cpus.empty()) t.l3.push_back(std::move(d));
    }
    Finalize(&t);
    return t;
}

#ifdef _WIN32

std::vector<int> AllowedCpus() {
    std::vector<int> cpus;
    const WORD groups = GetActiveProcessorGroupCount();
    for (WORD g = 0; g < groups; ++g) {
        const DWORD n = GetActiveProcessorCount(g);
        for (DWORD i = 0; i < n && i < 64; ++i) cpus.push_back(static_cast<int>(g) * 64 + static_cast<int>(i));
    }
    return cpus;
}

Topology Topology::Detect() {
    Topology t;
    t.cpus = AllowedCpus();
    DWORD len = 0;
    GetLogicalProcessorInformationEx(RelationCache, nullptr, &len);
    std::vector<uint8_t> buf(len);
    if (len != 0 && GetLogicalProcessorInformationEx(RelationCache,
                                                     reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(buf.data()), &len)) {
        for (DWORD off = 0; off < len;) {
            const auto* info = reinterpret_cast<const SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(buf.data() + off);
            if (info->Relationship == RelationCache && info->Cache.Level == 3 &&
                (info->Cache.Type == CacheUnified || info->Cache.Type == CacheData)) {
                L3Domain d;
                d.cacheBytes = info->Cache.CacheSize;
                const WORD groupCount = info->Cache.GroupCount == 0 ? 1 : info->Cache.GroupCount;
                for (WORD gi = 0; gi < groupCount; ++gi) {
                    const GROUP_AFFINITY& ga = info->Cache.GroupMasks[gi];
                    for (int bit = 0; bit < 64; ++bit) {
                        if (ga.Mask & (KAFFINITY{1} << bit)) d.cpus.push_back(static_cast<int>(ga.Group) * 64 + bit);
                    }
                }
                std::sort(d.cpus.begin(), d.cpus.end());
                std::erase_if(d.cpus, [&](int c) { return !std::binary_search(t.cpus.begin(), t.cpus.end(), c); });
                if (!d.cpus.empty()) t.l3.push_back(std::move(d));
            }
            off += info->Size;
        }
    }
    Finalize(&t);
    return t;
}

bool PinCurrentThread(const std::vector<int>& cpus) {
    if (cpus.empty()) return false;
    GROUP_AFFINITY ga{};
    ga.Group = static_cast<WORD>(cpus.front() / 64);
    for (int c : cpus) {
        if (c / 64 == ga.Group) ga.Mask |= KAFFINITY{1} << (c % 64);
    }
    return SetThreadGroupAffinity(GetCurrentThread(), &ga, nullptr) != 0;
}

void SetCurrentThreadName(const std::string& name) {
    const int n = MultiByteToWideChar(CP_UTF8, 0, name.c_str(), -1, nullptr, 0);
    if (n <= 0) return;
    std::wstring w(static_cast<size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, name.c_str(), -1, w.data(), n);
    SetThreadDescription(GetCurrentThread(), w.c_str());
}

void SetCurrentThreadBackground(bool background) noexcept {
    SetThreadPriority(GetCurrentThread(), background ? THREAD_PRIORITY_BELOW_NORMAL : THREAD_PRIORITY_NORMAL);
}

#else

std::vector<int> AllowedCpus() {
    std::vector<int> cpus;
#ifdef __linux__
    cpu_set_t set;
    CPU_ZERO(&set);
    if (sched_getaffinity(0, sizeof(set), &set) == 0) {
        for (size_t c = 0; c < CPU_SETSIZE; ++c) {
            if (CPU_ISSET(c, &set)) cpus.push_back(static_cast<int>(c));
        }
    }
#endif
    if (cpus.empty()) {
        const unsigned n = std::max(1u, std::thread::hardware_concurrency());
        for (unsigned c = 0; c < n; ++c) cpus.push_back(static_cast<int>(c));
    }
    return cpus;
}

Topology Topology::Detect() { return FromSysfs("/sys/devices/system/cpu", AllowedCpus()); }

bool PinCurrentThread(const std::vector<int>& cpus) {
#ifdef __linux__
    if (cpus.empty()) return false;
    cpu_set_t set;
    CPU_ZERO(&set);
    for (int c : cpus) {
        if (c >= 0 && c < CPU_SETSIZE) CPU_SET(static_cast<size_t>(c), &set);
    }
    return pthread_setaffinity_np(pthread_self(), sizeof(set), &set) == 0;
#else
    (void)cpus;
    return false;
#endif
}

void SetCurrentThreadName(const std::string& name) {
#ifdef __linux__
    pthread_setname_np(pthread_self(), name.substr(0, 15).c_str());  // 16 bytes with the NUL
#else
    (void)name;
#endif
}

void SetCurrentThreadBackground(bool) noexcept {}

#endif

const Topology& Topology::Get() {
    static const Topology t = Detect();
    return t;
}

}  // namespace ddbt::exec
