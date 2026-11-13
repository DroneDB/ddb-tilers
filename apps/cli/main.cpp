// SPDX-License-Identifier: MPL-2.0
#include <cstdio>
#include <cstring>
#include <string>

#include "ddbt/ddbt.h"

namespace {

void Usage(std::FILE* out) {
    std::fputs(
        "usage: ddbt <command> [options]\n"
        "\n"
        "commands:\n"
        "  info [--cpu]                       version and build information; --cpu adds the\n"
        "                                     Highway dispatch targets (JSON)\n"
        "  cog build <input> <output> [json]  build a Cloud Optimized GeoTIFF\n"
        "  copc build <input> <output> [json] build a COPC point cloud\n"
        "  --version, --help\n",
        out);
}

int Report(ddbtErr err) {
    if (err == DDBT_ERR_NONE) return 0;
    std::fprintf(stderr, "ddbt: error %d: %s\n", static_cast<int>(err), ddbtLastError());
    return 1;
}

int CmdInfo(int argc, char** argv) {
    bool cpu = false;
    for (int i = 0; i < argc; ++i) {
        if (std::strcmp(argv[i], "--cpu") == 0) {
            cpu = true;
        } else {
            std::fprintf(stderr, "ddbt: unknown option '%s'\n", argv[i]);
            return 2;
        }
    }
    if (!cpu) {
        std::printf("ddbt %s\n", ddbtVersion());
        return 0;
    }
    char* json = nullptr;
    const ddbtErr err = ddbtCpuInfo(&json);
    if (err != DDBT_ERR_NONE) return Report(err);
    std::printf("%s\n", json);
    ddbtFree(json);
    return 0;
}

int CmdBuild(bool cog, int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        Usage(stderr);
        return 2;
    }
    const char* options = argc == 3 ? argv[2] : nullptr;
    return Report(cog ? ddbtBuildCog(argv[0], argv[1], options) : ddbtBuildCopc(argv[0], argv[1], options));
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        Usage(stderr);
        return 2;
    }
    const std::string cmd = argv[1];
    if (cmd == "--help" || cmd == "-h") {
        Usage(stdout);
        return 0;
    }
    if (cmd == "--version") {
        std::printf("ddbt %s\n", ddbtVersion());
        return 0;
    }
    if (cmd == "info") return CmdInfo(argc - 2, argv + 2);
    if ((cmd == "cog" || cmd == "copc") && argc >= 3 && std::strcmp(argv[2], "build") == 0)
        return CmdBuild(cmd == "cog", argc - 3, argv + 3);

    std::fprintf(stderr, "ddbt: unknown command '%s'\n", cmd.c_str());
    Usage(stderr);
    return 2;
}
