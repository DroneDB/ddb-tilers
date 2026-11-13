// SPDX-License-Identifier: MPL-2.0
#include "core/mem/system_memory.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fstream>
#include <string>
#endif

namespace ddbt::mem {
namespace {

#ifndef _WIN32
uint64_t ReadMeminfo(const char* field) {
    std::ifstream f("/proc/meminfo");
    std::string key, unit;
    uint64_t value = 0;
    while (f >> key >> value >> unit)
        if (key == field) return value * 1024;
    return 0;
}
#endif

}  // namespace

uint64_t AvailableMemoryBytes() {
#ifdef _WIN32
    MEMORYSTATUSEX m{};
    m.dwLength = sizeof(m);
    if (GlobalMemoryStatusEx(&m)) return m.ullAvailPhys;
    return 0;
#else
    return ReadMeminfo("MemAvailable:");
#endif
}

uint64_t PhysicalMemoryBytes() {
#ifdef _WIN32
    MEMORYSTATUSEX m{};
    m.dwLength = sizeof(m);
    if (GlobalMemoryStatusEx(&m)) return m.ullTotalPhys;
    return 0;
#else
    return ReadMeminfo("MemTotal:");
#endif
}

}  // namespace ddbt::mem
