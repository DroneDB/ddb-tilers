// SPDX-License-Identifier: MPL-2.0
// ddbt-bench: benchmark driver of ddb-tilers (driven by bench/harness/run.py).
#include "common.hpp"

#include <cstdio>
#include <cstring>
#include <exception>

namespace
{
const char *usage = R"(ddbt-bench: benchmark driver of ddb-tilers
usage: ddbt-bench <command> [options]   (ddbt-bench <command> --help for details)
  tile       run tile request sets through a dlopen()ed backend (legacy libddb or libddbt)
  run        run a command and measure time, rusage, process-tree RSS, I/O and swap
  pagecache  evict | warm | resident: page cache control of input files
  sysinfo    machine, topology, power policy, compiler and repository state as JSON
  hash       XXH64 of files (the tile hash function))";
}

int main(int argc, char **argv)
{
    if (argc < 2 || !std::strcmp(argv[1], "--help") || !std::strcmp(argv[1], "-h"))
    {
        std::puts(usage);
        return argc < 2 ? 2 : 0;
    }
    const char *cmd = argv[1];
    try
    {
        if (!std::strcmp(cmd, "tile"))
            return ddbtb::cmdTile(argc, argv);
        if (!std::strcmp(cmd, "run"))
            return ddbtb::cmdRun(argc, argv);
        if (!std::strcmp(cmd, "pagecache"))
            return ddbtb::cmdPagecache(argc, argv);
        if (!std::strcmp(cmd, "sysinfo"))
            return ddbtb::cmdSysinfo(argc, argv);
        if (!std::strcmp(cmd, "hash"))
            return ddbtb::cmdHash(argc, argv);
        std::fprintf(stderr, "ddbt-bench: unknown command '%s'\n%s\n", cmd, usage);
        return 2;
    }
    catch (const ddbtb::UsageError &e)
    {
        std::fprintf(stderr, "ddbt-bench %s: %s (see ddbt-bench %s --help)\n", cmd, e.what(), cmd);
        return 2;
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "ddbt-bench %s: %s\n", cmd, e.what());
        return 1;
    }
}
