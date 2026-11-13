# SPDX-License-Identifier: MPL-2.0
# exec: work-stealing ThreadPool, TaskGroup, ParallelFor (header), CancelToken (header), Topology.
list(APPEND DDBT_SOURCES
  core/exec/task_group.cpp
  core/exec/thread_pool.cpp
  core/exec/topology.cpp)
