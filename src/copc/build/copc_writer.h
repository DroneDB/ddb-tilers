// SPDX-License-Identifier: MPL-2.0
//
// Writes a COPC file (LAS 1.4, point format 6-8, layered LAZ with variable chunks):
//
//   [header 375][COPC info VLR][LAZ VLR][WKT VLR][extra-bytes VLR][chunk table offset]
//   [node chunks ...][chunk table][hierarchy EVLR: pages, child pages first, root page last]
//
// Chunks are appended in the order AddNode is called; the hierarchy is paged as PDAL's
// writers.copc pages it (a new page every 4 levels for subtrees of more than 50 nodes). The
// header, the VLRs (the COPC info VLR holds the root page's location) and the chunk table offset
// are written last; the file appears atomically (temporary + rename).
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "copc/build/key_mapper.h"
#include "core/util/status.h"

namespace ddbt::copc::build {

struct CopcHeaderFields {
    int format = 6;
    uint16_t extraBytes = 0;
    std::array<double, 3> scale{0.01, 0.01, 0.01};
    std::array<double, 3> offset{0, 0, 0};
    uint16_t fileSourceId = 0;
    uint16_t globalEncoding = 0;  // the WKT bit is added
    std::array<uint8_t, 16> projectId{};
    std::string systemId = "ddbt";
    std::string software = "ddbt";
    uint16_t creationDoy = 0;
    uint16_t creationYear = 0;
    std::string wkt;                     // empty: no WKT VLR
    std::vector<uint8_t> extraBytesVlr;  // empty: no extra-bytes VLR
};

struct CopcFinalInfo {
    std::array<double, 3> min{}, max{};
    std::array<uint64_t, 15> pointsByReturn{};
    Cube cube;
    double spacing = 0;
    double gpsTimeMin = 0, gpsTimeMax = 0;
};

struct CopcWriteStats {
    uint64_t points = 0;
    uint64_t nodes = 0;
    uint64_t chunks = 0;
    uint64_t pages = 0;
    uint64_t fileSize = 0;
    int maxDepth = 0;
};

class CopcWriter {
public:
    static Expected<std::unique_ptr<CopcWriter>> Create(const std::string& path, const CopcHeaderFields& fields,
                                                        uint64_t preallocate = 0);
    ~CopcWriter();

    /// Appends the chunk of node (depth, prefix) (`count` points; count 0 and an empty chunk for
    /// a node kept only because it has children). One thread at a time.
    Status AddNode(int depth, uint64_t prefix, uint32_t count, std::span<const uint8_t> chunk);

    /// Writes the chunk table, the hierarchy, the header and the VLRs, and publishes the file.
    Status Finish(const CopcFinalInfo& info, CopcWriteStats* stats = nullptr);

private:
    CopcWriter();
    struct State;
    std::unique_ptr<State> s_;
};

/// The VLR area bytes (header + payload of each VLR) of a COPC file with these fields; the COPC
/// info VLR first. Exposed for tests.
std::vector<uint8_t> CopcVlrBytes(const CopcHeaderFields& fields, const Cube& cube, double spacing,
                                  uint64_t rootHierOffset, uint64_t rootHierSize, double gpsMin, double gpsMax,
                                  uint32_t* vlrCount);

}  // namespace ddbt::copc::build
