// SPDX-License-Identifier: MPL-2.0
#pragma once

// Physical memory of the host, for the default budgets of caches and builders.

#include <cstdint>

namespace ddbt::mem {

/// Available physical memory in bytes (MemAvailable on Linux), 0 when unknown.
uint64_t AvailableMemoryBytes();

/// Physical memory in bytes (MemTotal on Linux), 0 when unknown.
uint64_t PhysicalMemoryBytes();

}  // namespace ddbt::mem
