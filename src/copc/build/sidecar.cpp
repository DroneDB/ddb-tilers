// SPDX-License-Identifier: MPL-2.0
#include "copc/build/sidecar.h"

#include <exception>
#include <filesystem>
#include <fstream>

#include "cog/tiler/json_lite.h"
#include "copc/tiler/cloud.h"
#include "core/crs/crs.h"
#include "core/crs/transform.h"

namespace ddbt::copc::build {

namespace json = ddbt::cog::tiler::json;

std::vector<std::string> PdalDimensions(int format, const std::vector<std::string>& extra) {
    std::vector<std::string> d = {"X", "Y", "Z", "Intensity", "ReturnNumber", "NumberOfReturns", "ScanDirectionFlag",
                                  "EdgeOfFlightLine", "Classification", "Synthetic", "KeyPoint", "Withheld",
                                  "Overlap", "ScanAngleRank", "UserData", "PointSourceId", "GpsTime", "ScanChannel"};
    if (format >= 7) {
        d.push_back("Red");
        d.push_back("Green");
        d.push_back("Blue");
    }
    if (format == 8) d.push_back("Infrared");
    d.insert(d.end(), extra.begin(), extra.end());
    return d;
}

std::string SidecarJson(const SidecarInput& in) {
    json::Value j = json::Object();
    j.obj["pointCount"] = json::Unsigned(in.points);
    json::Value b;
    b.kind = json::Value::Kind::kArray;
    for (size_t a = 0; a < 3; ++a) b.arr.push_back(json::Number(in.min[a]));
    for (size_t a = 0; a < 3; ++a) b.arr.push_back(json::Number(in.max[a]));
    j.obj["bounds"] = b;
    j.obj["projection"] = json::String(in.wkt);
    json::Value dims;
    dims.kind = json::Value::Kind::kArray;
    for (const std::string& s : PdalDimensions(in.format, in.extraDimensions)) dims.arr.push_back(json::String(s));
    j.obj["dimensions"] = dims;
    j.obj["pointFormat"] = json::Unsigned(static_cast<uint64_t>(in.format));
    j.obj["color"] = json::Bool(in.color);
    j.obj["color16"] = json::Bool(in.color16);
    j.obj["spacing"] = json::Number(in.spacing);
    json::Value cube = json::Object();
    json::Value c;
    c.kind = json::Value::Kind::kArray;
    for (size_t a = 0; a < 3; ++a) c.arr.push_back(json::Number(in.cube.center[a]));
    cube.obj["center"] = c;
    cube.obj["halfSize"] = json::Number(in.cube.halfSize);
    j.obj["cube"] = cube;
    j.obj["span"] = json::Unsigned(128);

    // Footprint in EPSG:4326 as DroneDB's populatePointCloudFootprint computes it.
    std::string warning;
    if (!in.wkt.empty()) {
        try {
            const crs::Crs src = crs::Crs::FromWkt(in.wkt);
            const crs::Crs horizontal = tiler::HorizontalCrs(src);
            const double zToMeters = tiler::VerticalUnitInMeters(src);
            const crs::Crs dst = crs::Crs::FromEpsg(4326);
            auto t = crs::TransformCache::Global().Get(horizontal, dst);
            double x0 = in.min[0], y0 = in.min[1], x1 = in.max[0], y1 = in.max[1];
            double cx = (in.min[0] + in.max[0]) / 2.0, cy = (in.min[1] + in.max[1]) / 2.0;
            if (!t->TransformPoint(x0, y0) || !t->TransformPoint(x1, y1) || !t->TransformPoint(cx, cy)) {
                warning = "Cannot transform the bounds to EPSG:4326";
            } else {
                const double z0 = in.min[2] * zToMeters, z1 = in.max[2] * zToMeters;
                if (z0 < -30000 || z1 > 30000 || (x0 == -90 && x1 == 90)) {
                    warning = "Strange point cloud bounds";
                } else {
                    auto pt = [&](double x, double y) {
                        json::Value p;
                        p.kind = json::Value::Kind::kArray;
                        p.arr = {json::Number(x), json::Number(y), json::Number(z0)};
                        return p;
                    };
                    json::Value ring;
                    ring.kind = json::Value::Kind::kArray;
                    ring.arr = {pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1), pt(x0, y0)};
                    json::Value coords;
                    coords.kind = json::Value::Kind::kArray;
                    coords.arr.push_back(ring);
                    json::Value poly = json::Object();
                    poly.obj["type"] = json::String("Polygon");
                    poly.obj["coordinates"] = coords;
                    j.obj["polyBounds"] = poly;
                    json::Value cen = json::Object();
                    cen.obj["type"] = json::String("Point");
                    cen.obj["coordinates"] = pt(cx, cy);
                    j.obj["centroid"] = cen;
                }
            }
        } catch (const std::exception& e) {
            warning = e.what();
        }
    }
    if (!warning.empty()) j.obj["footprintWarning"] = json::String(warning);
    return json::Dump(j, 2);
}

Status WriteSidecarFile(const std::string& path, const SidecarInput& in) {
    const std::string text = SidecarJson(in);
    const std::string tmp = path + ".tmp";
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        out << text << '\n';
        if (!out) return IoError("cannot write " + tmp);
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if (ec) return IoError("cannot rename " + tmp + ": " + ec.message());
    return Status();
}

}  // namespace ddbt::copc::build
