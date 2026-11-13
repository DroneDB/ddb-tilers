// SPDX-License-Identifier: MPL-2.0
//
// Visualisation parameters of a tile (DDBMemoryTileEx2): the formulas, colormaps, band filters,
// temperature units and sensor profiles of DroneDB 2.4.1 (vegetation.cpp, thermal.cpp,
// sensorprofile.cpp, vis_render.cpp), ported with the same arithmetic (float formulas, the
// colormap ramps interpolated in float, the 0..255 colormap index truncated). The tables are
// DroneDB's data (MPL-2.0, DroneDB contributors): the formulas and colormap stops are copied
// from vegetation.cpp, the sensor profiles generated from data/sensor-profiles.json
// (data/gen_sensor_profiles.py).
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ddbt::cog::tiler {

class CogSource;

/// ThumbVisParams.
struct VisParams {
    std::string preset, bands, formula, bandFilter, colormap, rescale;
    std::optional<double> nodata;
    /// No preset, bands, formula or NoData (GDALTiler: such a request is the plain tile, unless
    /// a colormap styles a single band).
    bool Plain() const noexcept { return preset.empty() && bands.empty() && formula.empty() && !nodata; }
};

/// NODATA_SENTINEL of raster_utils.h.
inline constexpr float kNodataSentinel = -9999.0f;

struct Formula {
    std::string id, name, expr, help;
    double rangeMin = 0, rangeMax = 0;
    bool hasRange = false;
    std::string requiredBands;
    bool needsReflectance = false;
};

/// The formula of `id` (exact), or null.
const Formula* FindFormula(const std::string& id);
const std::vector<Formula>& AllFormulas();

struct BandFilter {
    std::string id;
    int R = -1, G = -1, B = -1, N = -1, Re = -1, T = -1;
    bool Has(char c) const;
    int Get(char c) const;
};

/// VegetationEngine::parseFilter.
BandFilter ParseFilter(const std::string& filter, int bandCount);
/// VegetationEngine::autoDetectFilter (a single-band raster without a sensor profile maps to an
/// empty filter; DroneDB's thermal check only changes the filter's id there, never a render).
BandFilter AutoDetectFilter(const CogSource& source);

/// RasterDataKind.
struct RasterDataKind {
    std::string typeName;
    bool floatingPoint = false;
    int bandCount = 0;
};

/// VegetationEngine::renderRefusal ("" when the raster can compute the formula).
std::string RenderRefusal(const Formula& f, const BandFilter& filter, const RasterDataKind& data);

enum class TemperatureUnit { kCelsius, kKelvin, kFahrenheit };
std::optional<TemperatureUnit> ParseTemperatureUnit(const std::string& unit);
TemperatureUnit BandTemperatureUnit(const std::string& unitType);
std::optional<TemperatureUnit> ThermalFormulaUnit(const std::string& formulaId);
struct TemperatureConversion {
    double scale = 1.0, offset = 0.0;
    double Apply(double v) const { return v * scale + offset; }
};
TemperatureConversion ConvertTemperature(TemperatureUnit from, TemperatureUnit to);
/// vis_render.cpp convertRescale ("%.9g,%.9g").
std::string ConvertRescale(const std::string& rescale, TemperatureUnit from, TemperatureUnit to);

/// VegetationEngine::applyFormula over planar float bands (null: band not read; bands[0] is the
/// thermal band of the thermal formulas).
void ApplyFormula(const Formula& f, const BandFilter& filter, const std::vector<float*>& bands, float* out,
                  size_t n, float nodata, TemperatureUnit bandUnit);
/// VegetationEngine::bandsRead.
std::vector<int> BandsRead(const Formula& f, const BandFilter& filter);

struct ColormapEntry {
    uint8_t r = 0, g = 0, b = 0, a = 0;
};
struct Colormap {
    std::string id, name;
    ColormapEntry entries[256];
    bool listed = true;
};
/// The colormap of `id` (listed ones only unless includeUnlisted), or null.
const Colormap* FindColormap(const std::string& id, bool includeUnlisted = false);
const std::vector<Colormap>& AllColormaps();
/// VegetationEngine::applyColormap: RGBA interleaved, nodata / NaN transparent.
void ApplyColormap(const float* values, uint8_t* rgba, size_t n, const Colormap& cmap, float vmin, float vmax,
                   float nodata);

/// parseRescaleRange: "min,max" (std::stof), else InvalidArgs.
std::pair<float, float> ParseRescaleRange(const std::string& rescale);

// Sensor profiles ---------------------------------------------------------------------------------

struct BandMapping {
    int r = 1, g = 2, b = 3;
};
struct SensorDetection {
    bool detected = false;
    std::string sensorId;
    BandMapping defaultMapping;
    struct Band {
        int index = 0;
        std::string name;
    };
    std::vector<Band> bands;
    struct Preset {
        std::string id, type;
        BandMapping mapping;
        bool isDefault = false;
        std::string formula, colormap;
    };
    std::vector<Preset> presets;
    std::string defaultPresetId;
};
/// SensorProfileManager::detectSensor on the file, computed once per source (CogSource::SensorMemo)
/// and shared: the detection lives while the caller holds it, whatever happens to other sources.
std::shared_ptr<const SensorDetection> DetectSensor(const CogSource& source);
/// getBandMappingForPreset.
BandMapping BandMappingForPreset(const CogSource& source, const std::string& preset);
/// sensorprofile.cpp bandSymbolOf.
char BandSymbolOf(const std::string& name);

/// vis_render.cpp resolveVisBands (InvalidArgs for a bad bands list).
std::vector<int> ResolveVisBands(const CogSource& source, const VisParams& vis, int maxBand);
/// styledBand: the one band a colormap paints without a formula, 0 if none.
int StyledBand(int dataBandCount, const std::string& colormap, const std::vector<int>& chosen);

}  // namespace ddbt::cog::tiler
