// SPDX-License-Identifier: MPL-2.0
// The arithmetic is the tiler's (cog/tiler/stats_kernel.cpp).
#include "cog/build/stats.h"

#include <ctime>
#include <filesystem>
#include <fstream>

#include "cog/tiler/json_lite.h"

namespace ddbt::cog::build {

StatsAccumulator::StatsAccumulator(int bands, tiff::SampleType type, std::vector<MaskKind> masks, int alphaBand,
                                   std::optional<double> nodata)
    : bands_(bands), type_(type), masks_(std::move(masks)), alphaBand_(alphaBand), nodata_(nodata),
      mode_(stats::StatsModeFromEnv()), acc_(static_cast<size_t>(bands)) {}

void StatsAccumulator::AccumulateRows(int band, const uint8_t* pixels, size_t stride, uint32_t width, uint32_t rows,
                                      const uint8_t* mask) {
    Acc& a = acc_[static_cast<size_t>(band)];
    const size_t ss = tiff::SampleTypeSize(type_);
    const size_t step = ss * static_cast<size_t>(bands_);
    const MaskKind mk = masks_[static_cast<size_t>(band)];
    const stats::SampleFilter f{type_, nodata_.has_value(), nodata_.value_or(0)};
    for (uint32_t y = 0; y < rows; ++y) {
        const uint8_t* row = pixels + static_cast<size_t>(y) * stride;
        const uint8_t* p = row + static_cast<size_t>(band) * ss;
        const uint8_t* m = mk == MaskKind::kMask && mask != nullptr ? mask + static_cast<size_t>(y) * width : nullptr;
        const uint8_t* al = mk == MaskKind::kAlphaBand ? row + static_cast<size_t>(alphaBand_) * ss : nullptr;
        if (mode_ == stats::StatsMode::kExact) {
            stats::AddRun(a.exact, f, p, step, width, m, al);
        } else {
            stats::AddRun(a.welford, f, p, step, width, m, al);
        }
    }
}

std::vector<std::optional<BandStats>> StatsAccumulator::Result(uint64_t total) const {
    std::vector<std::optional<BandStats>> out(acc_.size());
    for (size_t i = 0; i < acc_.size(); ++i) {
        const auto r = mode_ == stats::StatsMode::kExact ? stats::Finish(acc_[i].exact) : stats::Finish(acc_[i].welford);
        if (!r) continue;
        BandStats s;
        s.min = r->min;
        s.max = r->max;
        s.mean = r->mean;
        s.stdDev = r->stdDev;
        s.valid = r->valid;
        s.total = total;
        out[i] = s;
    }
    return out;
}

Status WriteStatsSidecar(const std::string& rasterPath, const std::vector<std::optional<BandStats>>& bands) {
    namespace json = tiler::json;
    namespace fs = std::filesystem;
    json::Value j = json::Object();
    json::Value& b = j.obj["bands"];
    b.kind = json::Value::Kind::kObject;
    for (size_t i = 0; i < bands.size(); ++i) {
        if (!bands[i]) continue;
        const BandStats& s = *bands[i];
        json::Value o = json::Object();
        o.obj["min"] = json::Number(s.min);
        o.obj["max"] = json::Number(s.max);
        o.obj["mean"] = json::Number(s.mean);
        o.obj["std"] = json::Number(s.stdDev);
        o.obj["valid"] = json::Unsigned(s.valid);
        o.obj["total"] = json::Unsigned(s.total);
        b.obj[std::to_string(i + 1)] = std::move(o);
    }
    j.obj["version"] = json::Unsigned(2);
    j.obj["exact"] = json::Bool(true);
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%FT%TZ", &tm);
    j.obj["computedAt"] = json::String(stamp);
    const std::string target = rasterPath + ".stats.json";
    const std::string temp = target + ".tmp";
    {
        std::ofstream out(fs::path(std::u8string(temp.begin(), temp.end())), std::ios::trunc | std::ios::binary);
        out << json::Dump(j, 2);
        if (!out) return Status(StatusCode::kIoError, "Cannot write " + temp);
    }
    std::error_code ec;
    fs::rename(fs::path(std::u8string(temp.begin(), temp.end())), fs::path(std::u8string(target.begin(), target.end())),
               ec);
    if (ec) return Status(StatusCode::kIoError, "Cannot write " + target + ": " + ec.message());
    return Status::Ok();
}

}  // namespace ddbt::cog::build
