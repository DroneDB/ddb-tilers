# SPDX-License-Identifier: MPL-2.0
# mem: aligned / large allocations, MemBudget, Arena, BufferPool, the host's physical memory.
list(APPEND DDBT_SOURCES
  core/mem/aligned.cpp
  core/mem/arena.cpp
  core/mem/buffer_pool.cpp
  core/mem/mem_budget.cpp
  core/mem/system_memory.cpp)
