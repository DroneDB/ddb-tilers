// SPDX-License-Identifier: MPL-2.0
// Ported from DroneDB 2.4.1 (MPL-2.0): src/library/vegetation.cpp, thermal.cpp (temperature
// units), sensorprofile.cpp, vis_render.cpp. Compiled without floating-point contraction
// (sources.cmake) so the float arithmetic rounds as DroneDB's does.
#include "cog/tiler/vis.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <map>
#include <sstream>
#include <tuple>

#include "cog/tiler/cog_source.h"
#include "cog/tiler/tile_error.h"

namespace ddbt::cog::tiler {

// Formulas ------------------------------------------------------------------------------------

const std::vector<Formula>& AllFormulas() {
    static const std::vector<Formula> f = {
        {"VARI", "Visual Atmospheric Resistance Index", "(G - R) / (G + R - B)", "Areas of green vegetation on RGB images", -1, 1, true, "RGB"},
        {"EXG", "Excess Green Index", "(2 * G) - (R + B)", "Emphasizes green foliage", 0, 0, false, "RGB"},
        {"GLI", "Green Leaf Index", "((G * 2) - R - B) / ((G * 2) + R + B)", "Green leaves and stems", -1, 1, true, "RGB"},
        {"vNDVI", "Visible NDVI", "0.5268 * (R^-0.1294 * G^0.3389 * B^-0.3118)", "Approximated NDVI for RGB sensors", 0, 0, false, "RGB"},
        {"NDVI", "Normalized Difference Vegetation Index", "(N - R) / (N + R)", "Amount of green vegetation", -1, 1, true, "RN"},
        {"NDWI", "Normalized Difference Water Index", "(G - N) / (G + N)", "Water content", -1, 1, true, "GN"},
        {"GNDVI", "Green NDVI", "(N - G) / (N + G)", "NDVI on green spectrum", -1, 1, true, "GN"},
        {"SAVI", "Soil Adjusted Vegetation Index", "(1.5 * (N - R)) / (N + R + 0.5)", "NDVI with soil correction", -1, 1, true, "RN", true},
        {"EVI", "Enhanced Vegetation Index", "2.5 * (N - R) / (N + 6*R - 7.5*B + 1)", "Better where NDVI saturates", -1, 1, true, "RBN", true},
        {"NDRE", "Normalized Difference Red Edge", "(N - Re) / (N + Re)", "Green vegetation in mature crops", -1, 1, true, "NRe"},
        {"NDYI", "Normalized Difference Yellowness Index", "(G - B) / (G + B)", "Yellowness detection", -1, 1, true, "GB"},
        {"MPRI", "Modified Photochemical Reflectance Index", "(G - R) / (G + R)", "Photochemical reflectance", -1, 1, true, "RG"},
        {"OSAVI", "Optimized Soil Adjusted Vegetation Index", "(N - R) / (N + R + 0.16)", "Optimized for sparse vegetation", -1, 1, true, "RN", true},
        {"GRVI", "Green Ratio Vegetation Index", "N / G", "Green ratio", 0, 0, false, "GN"},
        {"ENDVI", "Enhanced NDVI", "((N + G) - (2*B)) / ((N + G) + (2*B))", "Enhanced NDVI", 0, 0, false, "GBN"},
        {"ARVI", "Atmospherically Resistant Vegetation Index", "(N - (2*R) + B) / (N + (2*R) + B)", "Corrects atmospheric effects", -1, 1, true, "RBN"},
        {"CELSIUS", "Celsius Temperature", "pixel in degrees Celsius (converted from the unit of the band)", "Temperature in degrees Celsius", -40, 150, true, "T"},
        {"KELVIN", "Kelvin Temperature", "pixel in K (converted from the unit of the band)", "Temperature in Kelvin", 233.15, 423.15, true, "T"},
        {"FAHRENHEIT", "Fahrenheit Temperature", "pixel in degrees Fahrenheit (converted from the unit of the band)", "Temperature in degrees Fahrenheit", -40, 302, true, "T"},
    };
    return f;
}

const Formula* FindFormula(const std::string& id) {
    for (const auto& f : AllFormulas())
        if (f.id == id) return &f;
    return nullptr;
}

bool BandFilter::Has(char c) const { return Get(c) >= 0; }

int BandFilter::Get(char c) const {
    switch (c) {
        case 'R': return R;
        case 'G': return G;
        case 'B': return B;
        case 'N': return N;
        case 'e': return Re;
        case 'T': return T;
        default: return -1;
    }
}

BandFilter ParseFilter(const std::string& s, int bandCount) {
    BandFilter bf;
    bf.id = s;
    int idx = 0;
    for (size_t i = 0; i < s.size() && idx < bandCount; i++) {
        const char c = s[i];
        if (c == 'R' && i + 1 < s.size() && s[i + 1] == 'e') {
            bf.Re = idx++;
            i++;
        } else {
            switch (c) {
                case 'R': bf.R = idx++; break;
                case 'G': bf.G = idx++; break;
                case 'B': bf.B = idx++; break;
                case 'N': bf.N = idx++; break;
                case 'T': bf.T = idx++; break;
                default: idx++; break;
            }
        }
    }
    return bf;
}

namespace {

std::vector<char> RequiredSymbols(const Formula& f) {
    std::vector<char> out;
    const std::string& r = f.requiredBands;
    for (size_t i = 0; i < r.size();) {
        if (r[i] == 'R' && i + 1 < r.size() && r[i + 1] == 'e') {
            out.push_back('e');
            i += 2;
        } else {
            out.push_back(r[i]);
            ++i;
        }
    }
    return out;
}

bool IsThermalFormula(const std::string& id) { return ThermalFormulaUnit(id).has_value(); }

std::string DataRefusal(const Formula& f, const RasterDataKind& d) {
    if (!f.needsReflectance || d.floatingPoint) return "";
    return f.id + " needs reflectance values (0 to 1, a floating-point raster): the raster is " + d.typeName +
           ", whose values are not reflectance";
}

}  // namespace

std::vector<int> BandsRead(const Formula& f, const BandFilter& filter) {
    std::vector<int> bands;
    for (const char s : RequiredSymbols(f)) {
        const int b = filter.Get(s);
        if (b >= 0) bands.push_back(b);
    }
    std::sort(bands.begin(), bands.end());
    bands.erase(std::unique(bands.begin(), bands.end()), bands.end());
    return bands;
}

std::string RenderRefusal(const Formula& f, const BandFilter& filter, const RasterDataKind& data) {
    if (IsThermalFormula(f.id)) return DataRefusal(f, data);
    for (const char s : RequiredSymbols(f))
        if (!filter.Has(s)) return "Requires bands: " + f.requiredBands;
    return DataRefusal(f, data);
}

// Temperature units --------------------------------------------------------------------------

std::optional<TemperatureUnit> ParseTemperatureUnit(const std::string& unit) {
    std::string u;
    for (const char c : unit)
        if (std::isalnum(static_cast<unsigned char>(c))) u += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (u == "k" || u == "kelvin" || u == "degk") return TemperatureUnit::kKelvin;
    if (u == "c" || u == "celsius" || u == "degc" || u == "centigrade") return TemperatureUnit::kCelsius;
    if (u == "f" || u == "fahrenheit" || u == "degf") return TemperatureUnit::kFahrenheit;
    return std::nullopt;
}

TemperatureUnit BandTemperatureUnit(const std::string& unitType) {
    return ParseTemperatureUnit(unitType).value_or(TemperatureUnit::kCelsius);
}

std::optional<TemperatureUnit> ThermalFormulaUnit(const std::string& id) {
    if (id == "CELSIUS") return TemperatureUnit::kCelsius;
    if (id == "KELVIN") return TemperatureUnit::kKelvin;
    if (id == "FAHRENHEIT") return TemperatureUnit::kFahrenheit;
    return std::nullopt;
}

TemperatureConversion ConvertTemperature(TemperatureUnit from, TemperatureUnit to) {
    if (from == to) return {};
    const auto toC = [](TemperatureUnit u) -> TemperatureConversion {
        switch (u) {
            case TemperatureUnit::kKelvin: return {1.0, -273.15};
            case TemperatureUnit::kFahrenheit: return {5.0 / 9.0, -32.0 * 5.0 / 9.0};
            default: return {};
        }
    };
    const auto fromC = [](TemperatureUnit u) -> TemperatureConversion {
        switch (u) {
            case TemperatureUnit::kKelvin: return {1.0, 273.15};
            case TemperatureUnit::kFahrenheit: return {1.8, 32.0};
            default: return {};
        }
    };
    const TemperatureConversion a = toC(from), b = fromC(to);
    return {a.scale * b.scale, a.offset * b.scale + b.offset};
}

std::pair<float, float> ParseRescaleRange(const std::string& rescale) {
    const auto comma = rescale.find(',');
    const std::string mn = comma == std::string::npos ? "" : rescale.substr(0, comma);
    const std::string mx = comma == std::string::npos ? "" : rescale.substr(comma + 1);
    if (mn.empty() || mx.empty()) FailInvalid("Invalid rescale format: " + rescale + ". Expected format: min,max");
    try {
        return {std::stof(mn), std::stof(mx)};
    } catch (const std::exception&) {
        FailInvalid("Invalid rescale values: " + rescale + ". Expected numeric format: min,max");
    }
}

std::string ConvertRescale(const std::string& rescale, TemperatureUnit from, TemperatureUnit to) {
    ParseRescaleRange(rescale);
    const auto comma = rescale.find(',');
    const TemperatureConversion c = ConvertTemperature(from, to);
    double lo, hi;
    try {
        lo = c.Apply(std::stod(rescale.substr(0, comma)));
        hi = c.Apply(std::stod(rescale.substr(comma + 1)));
    } catch (const std::exception&) {
        FailInvalid("Invalid rescale values: " + rescale + ". Expected numeric format: min,max");
    }
    char text[64];
    std::snprintf(text, sizeof(text), "%.9g,%.9g", lo, hi);
    return text;
}

// Formula evaluation (vegetation.cpp) -----------------------------------------------------------

namespace {

inline bool HasNoValue(float v, float nodata) { return v == nodata || !std::isfinite(v); }

enum class Kind { kUnknown, kNDVI, kVARI, kEXG, kGLI, kVNDVI, kNDWI, kGNDVI, kSAVI, kEVI, kNDRE, kNDYI, kMPRI, kOSAVI, kGRVI, kENDVI, kARVI, kCELSIUS, kKELVIN, kFAHRENHEIT };

Kind KindOf(const std::string& id) {
    static const std::pair<const char*, Kind> kinds[] = {
        {"NDVI", Kind::kNDVI}, {"VARI", Kind::kVARI}, {"EXG", Kind::kEXG}, {"GLI", Kind::kGLI},
        {"vNDVI", Kind::kVNDVI}, {"NDWI", Kind::kNDWI}, {"GNDVI", Kind::kGNDVI}, {"SAVI", Kind::kSAVI},
        {"EVI", Kind::kEVI}, {"NDRE", Kind::kNDRE}, {"NDYI", Kind::kNDYI}, {"MPRI", Kind::kMPRI},
        {"OSAVI", Kind::kOSAVI}, {"GRVI", Kind::kGRVI}, {"ENDVI", Kind::kENDVI}, {"ARVI", Kind::kARVI},
        {"CELSIUS", Kind::kCELSIUS}, {"KELVIN", Kind::kKELVIN}, {"FAHRENHEIT", Kind::kFAHRENHEIT}};
    for (const auto& [n, k] : kinds)
        if (id == n) return k;
    return Kind::kUnknown;
}

struct In {
    const float *R, *G, *B, *N, *Re, *thermal;
    float* out;
    size_t count;
    float nodata;
    TemperatureConversion unit;
};

template <Kind K>
inline float Eval(float R, float G, float B, float N, float Re, const float* thermal, const TemperatureConversion& unit,
                  size_t i, float nodata) {
    const float EPS = 1e-10f;
    const auto over = [nodata, EPS](float num, float den) { return (std::abs(den) < EPS) ? nodata : num / den; };
    switch (K) {
        case Kind::kNDVI: return over(N - R, N + R);
        case Kind::kVARI: return over(G - R, G + R - B);
        case Kind::kEXG: return (2.0f * G) - (R + B);
        case Kind::kGLI: return over((G * 2.0f) - R - B, (G * 2.0f) + R + B);
        case Kind::kVNDVI:
            if (R > EPS && G > EPS && B > EPS)
                return 0.5268f * std::pow(R, -0.1294f) * std::pow(G, 0.3389f) * std::pow(B, -0.3118f);
            return nodata;
        case Kind::kNDWI: return over(G - N, G + N);
        case Kind::kGNDVI: return over(N - G, N + G);
        case Kind::kSAVI: return over(1.5f * (N - R), N + R + 0.5f);
        case Kind::kEVI: return over(2.5f * (N - R), N + 6.0f * R - 7.5f * B + 1.0f);
        case Kind::kNDRE: return over(N - Re, N + Re);
        case Kind::kNDYI: return over(G - B, G + B);
        case Kind::kMPRI: return over(G - R, G + R);
        case Kind::kOSAVI: return over(N - R, N + R + 0.16f);
        case Kind::kGRVI: return over(N, G);
        case Kind::kENDVI: return over((N + G) - (2.0f * B), (N + G) + (2.0f * B));
        case Kind::kARVI: return over(N - (2.0f * R) + B, N + (2.0f * R) + B);
        case Kind::kCELSIUS:
        case Kind::kKELVIN:
        case Kind::kFAHRENHEIT:
            return thermal != nullptr && !HasNoValue(thermal[i], nodata) ? static_cast<float>(unit.Apply(thermal[i]))
                                                                         : nodata;
        case Kind::kUnknown: break;
    }
    return nodata;
}

template <Kind K>
void Loop(const In& in) {
    const float nodata = in.nodata;
    for (size_t i = 0; i < in.count; i++) {
        const float R = in.R ? in.R[i] : 0, G = in.G ? in.G[i] : 0, B = in.B ? in.B[i] : 0;
        const float N = in.N ? in.N[i] : 0, Re = in.Re ? in.Re[i] : 0;
        bool nd = false;
        if (in.R && HasNoValue(R, nodata)) nd = true;
        if (in.G && HasNoValue(G, nodata)) nd = true;
        if (in.B && HasNoValue(B, nodata)) nd = true;
        if (in.N && HasNoValue(N, nodata)) nd = true;
        if (in.Re && HasNoValue(Re, nodata)) nd = true;
        if (nd) {
            in.out[i] = nodata;
            continue;
        }
        in.out[i] = Eval<K>(R, G, B, N, Re, in.thermal, in.unit, i, nodata);
    }
}

}  // namespace

void ApplyFormula(const Formula& f, const BandFilter& filter, const std::vector<float*>& bands, float* out, size_t n,
                  float nodata, TemperatureUnit bandUnit) {
    const auto get = [&](char s) -> const float* {
        const int idx = filter.Get(s);
        if (idx < 0 || idx >= static_cast<int>(bands.size())) return nullptr;
        return bands[static_cast<size_t>(idx)];
    };
    const auto target = ThermalFormulaUnit(f.id);
    const In in{get('R'), get('G'), get('B'), get('N'), get('e'),
                (!bands.empty() && bands[0] != nullptr) ? bands[0] : nullptr,
                out, n, nodata, target ? ConvertTemperature(bandUnit, *target) : TemperatureConversion{}};
    switch (KindOf(f.id)) {
        case Kind::kNDVI: return Loop<Kind::kNDVI>(in);
        case Kind::kVARI: return Loop<Kind::kVARI>(in);
        case Kind::kEXG: return Loop<Kind::kEXG>(in);
        case Kind::kGLI: return Loop<Kind::kGLI>(in);
        case Kind::kVNDVI: return Loop<Kind::kVNDVI>(in);
        case Kind::kNDWI: return Loop<Kind::kNDWI>(in);
        case Kind::kGNDVI: return Loop<Kind::kGNDVI>(in);
        case Kind::kSAVI: return Loop<Kind::kSAVI>(in);
        case Kind::kEVI: return Loop<Kind::kEVI>(in);
        case Kind::kNDRE: return Loop<Kind::kNDRE>(in);
        case Kind::kNDYI: return Loop<Kind::kNDYI>(in);
        case Kind::kMPRI: return Loop<Kind::kMPRI>(in);
        case Kind::kOSAVI: return Loop<Kind::kOSAVI>(in);
        case Kind::kGRVI: return Loop<Kind::kGRVI>(in);
        case Kind::kENDVI: return Loop<Kind::kENDVI>(in);
        case Kind::kARVI: return Loop<Kind::kARVI>(in);
        case Kind::kCELSIUS: return Loop<Kind::kCELSIUS>(in);
        case Kind::kKELVIN: return Loop<Kind::kKELVIN>(in);
        case Kind::kFAHRENHEIT: return Loop<Kind::kFAHRENHEIT>(in);
        case Kind::kUnknown: return Loop<Kind::kUnknown>(in);
    }
}

// Colormaps -----------------------------------------------------------------------------------

namespace {

using Stops = std::vector<std::tuple<float, uint8_t, uint8_t, uint8_t>>;

void Interpolate(ColormapEntry* e, const Stops& stops) {
    for (int i = 0; i < 256; i++) {
        const float t = static_cast<float>(i) / 255.0f;
        size_t lo = 0, hi = stops.size() - 1;
        for (size_t s = 0; s < stops.size() - 1; s++) {
            if (t >= std::get<0>(stops[s]) && t <= std::get<0>(stops[s + 1])) {
                lo = s;
                hi = s + 1;
                break;
            }
        }
        const float tLo = std::get<0>(stops[lo]), tHi = std::get<0>(stops[hi]);
        float frac = (tHi > tLo) ? (t - tLo) / (tHi - tLo) : 0.0f;
        frac = std::max(0.0f, std::min(1.0f, frac));
        e[i].r = static_cast<uint8_t>(std::get<1>(stops[lo]) + frac * (std::get<1>(stops[hi]) - std::get<1>(stops[lo])));
        e[i].g = static_cast<uint8_t>(std::get<2>(stops[lo]) + frac * (std::get<2>(stops[hi]) - std::get<2>(stops[lo])));
        e[i].b = static_cast<uint8_t>(std::get<3>(stops[lo]) + frac * (std::get<3>(stops[hi]) - std::get<3>(stops[lo])));
        e[i].a = 255;
    }
}

void FillDiscreteNdvi(ColormapEntry* e) {
    for (int i = 0; i < 256; i++) {
        const float t = static_cast<float>(i) / 255.0f;
        if (t < 0.2f) e[i] = {139, 0, 0, 255};
        else if (t < 0.4f) e[i] = {255, 69, 0, 255};
        else if (t < 0.6f) e[i] = {255, 215, 0, 255};
        else if (t < 0.8f) e[i] = {50, 205, 50, 255};
        else e[i] = {0, 100, 0, 255};
    }
}
void FillGrayscale(ColormapEntry* e) {
    for (int i = 0; i < 256; i++) e[i] = {static_cast<uint8_t>(i), static_cast<uint8_t>(i), static_cast<uint8_t>(i), 255};
}
void FillGreys(ColormapEntry* e) {
    for (int i = 0; i < 256; i++) {
        const uint8_t v = static_cast<uint8_t>(255 - i);
        e[i] = {v, v, v, 255};
    }
}

struct Def {
    const char* id;
    const char* name;
    bool listed;
    Stops stops;
    void (*fill)(ColormapEntry*);
};

}  // namespace

const std::vector<Colormap>& AllColormaps() {
    static const std::vector<Colormap> maps = [] {
        const std::vector<Def> defs = {
            {"rdylgn", "Red-Yellow-Green", true, {{0.0f, 215, 48, 39}, {0.25f, 253, 174, 97}, {0.5f, 254, 224, 139}, {0.75f, 166, 217, 106}, {1.0f, 26, 152, 80}}, nullptr},
            {"discrete_ndvi", "Contrast NDVI", true, {}, FillDiscreteNdvi},
            {"spectral", "Spectral", true, {{0.0f, 158, 1, 66}, {0.25f, 253, 174, 97}, {0.5f, 255, 255, 191}, {0.75f, 102, 194, 165}, {1.0f, 94, 79, 162}}, nullptr},
            {"viridis", "Viridis", true, {{0.0f, 68, 1, 84}, {0.25f, 59, 82, 139}, {0.5f, 33, 144, 140}, {0.75f, 93, 201, 99}, {1.0f, 253, 231, 37}}, nullptr},
            {"plasma", "Plasma", true, {{0.0f, 13, 8, 135}, {0.25f, 126, 3, 168}, {0.5f, 204, 71, 120}, {0.75f, 248, 149, 64}, {1.0f, 240, 249, 33}}, nullptr},
            {"inferno", "Inferno", true, {{0.0f, 0, 0, 4}, {0.25f, 87, 16, 110}, {0.5f, 188, 55, 84}, {0.75f, 249, 142, 9}, {1.0f, 252, 255, 164}}, nullptr},
            {"magma", "Magma", true, {{0.0f, 0, 0, 4}, {0.25f, 81, 18, 124}, {0.5f, 183, 55, 121}, {0.75f, 252, 137, 97}, {1.0f, 252, 253, 191}}, nullptr},
            {"grayscale", "Grayscale", true, {}, FillGrayscale},
            {"ironbow", "Ironbow", true, {{0.0f, 0, 0, 0}, {0.2f, 87, 15, 106}, {0.4f, 189, 55, 84}, {0.6f, 249, 120, 8}, {0.8f, 255, 227, 68}, {1.0f, 255, 255, 255}}, nullptr},
            {"rainbow", "Rainbow", true, {{0.0f, 150, 0, 90}, {0.17f, 0, 0, 200}, {0.33f, 0, 150, 255}, {0.5f, 0, 255, 0}, {0.67f, 255, 255, 0}, {0.83f, 255, 100, 0}, {1.0f, 200, 0, 0}}, nullptr},
            {"bugn", "Blue-Green", true, {{0.0f, 247, 252, 253}, {0.25f, 178, 226, 226}, {0.5f, 102, 194, 164}, {0.75f, 35, 139, 69}, {1.0f, 0, 68, 27}}, nullptr},
            {"whitehot", "White Hot", true, {{0.0f, 0, 0, 0}, {1.0f, 255, 255, 255}}, nullptr},
            {"blackhot", "Black Hot", true, {{0.0f, 255, 255, 255}, {1.0f, 0, 0, 0}}, nullptr},
            {"arctic", "Arctic", true, {{0.0f, 0, 0, 64}, {0.25f, 0, 100, 200}, {0.5f, 100, 200, 255}, {0.75f, 200, 240, 255}, {1.0f, 255, 255, 255}}, nullptr},
            {"lava", "Lava", true, {{0.0f, 0, 0, 0}, {0.25f, 128, 0, 0}, {0.5f, 255, 80, 0}, {0.75f, 255, 200, 0}, {1.0f, 255, 255, 200}}, nullptr},
            {"terrain", "Terrain", true, {{0.0f, 51, 51, 153}, {0.2f, 0, 180, 180}, {0.4f, 160, 217, 121}, {0.6f, 236, 216, 122}, {0.8f, 169, 116, 67}, {1.0f, 255, 255, 255}}, nullptr},
            {"greys", "Greys", true, {}, FillGreys},
            {"ryg", "Red-Yellow-Green (pure)", false, {{0.0f, 255, 0, 0}, {0.5f, 255, 255, 0}, {1.0f, 0, 255, 0}}, nullptr},
        };
        std::vector<Colormap> out;
        for (const auto& d : defs) {
            Colormap c;
            c.id = d.id;
            c.name = d.name;
            c.listed = d.listed;
            if (d.fill) d.fill(c.entries);
            else Interpolate(c.entries, d.stops);
            out.push_back(c);
        }
        return out;
    }();
    return maps;
}

const Colormap* FindColormap(const std::string& id, bool includeUnlisted) {
    for (const auto& c : AllColormaps())
        if (c.id == id && (c.listed || includeUnlisted)) return &c;
    return nullptr;
}

void ApplyColormap(const float* values, uint8_t* rgba, size_t n, const Colormap& cmap, float vmin, float vmax,
                   float nodata) {
    float range = vmax - vmin;
    if (std::abs(range) < 1e-10f) range = 1.0f;
    for (size_t i = 0; i < n; i++) {
        const float v = values[i];
        if (v == nodata || std::isnan(v)) {
            rgba[i * 4 + 0] = rgba[i * 4 + 1] = rgba[i * 4 + 2] = rgba[i * 4 + 3] = 0;
            continue;
        }
        float normalized = (v - vmin) / range;
        normalized = std::max(0.0f, std::min(1.0f, normalized));
        int idx = static_cast<int>(normalized * 255.0f);
        idx = std::max(0, std::min(255, idx));
        rgba[i * 4 + 0] = cmap.entries[idx].r;
        rgba[i * 4 + 1] = cmap.entries[idx].g;
        rgba[i * 4 + 2] = cmap.entries[idx].b;
        rgba[i * 4 + 3] = cmap.entries[idx].a;
    }
}

// Sensor profiles ------------------------------------------------------------------------------

namespace {

struct ProfileBand {
    int index;
    std::string name;
};
struct ProfilePreset {
    std::string id, type;
    BandMapping mapping;
    bool isDefault;
    std::string formula, colormap;
};
struct Profile {
    std::string id, name, category;
    int bandCount;
    std::string dataType;
    std::vector<std::string> patterns;
    int priority;
    std::vector<ProfileBand> bands;
    std::vector<ProfilePreset> presets;
};

const std::vector<Profile>& Profiles() {
    static const std::vector<Profile> profiles = [] {
        std::vector<Profile> p = {
#include "cog/tiler/data/sensor_profiles.inc"
        };
        // SensorProfileManager::ensureLoaded sorts by priority (std::sort, as DroneDB).
        std::sort(p.begin(), p.end(), [](const Profile& a, const Profile& b) { return a.priority > b.priority; });
        return p;
    }();
    return profiles;
}

std::string Lower(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string Trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

BandMapping FallbackMapping(const CogSource& src, int bandCount) {
    const int n = bandCount > 0 ? bandCount : src.BandCount();
    int r = 0, g = 0, b = 0;
    for (int i = 1; i <= n && i <= src.BandCount(); i++) {
        const ColorInterp ci = src.Band(i).interp;
        if (ci == ColorInterp::kRed && r == 0) r = i;
        else if (ci == ColorInterp::kGreen && g == 0) g = i;
        else if (ci == ColorInterp::kBlue && b == 0) b = i;
    }
    if (r > 0 && g > 0 && b > 0) return {r, g, b};
    return n >= 3 ? BandMapping{1, 2, 3} : BandMapping{1, 1, 1};
}

std::vector<SensorDetection::Preset> PresetsOfBands(const std::vector<SensorDetection::Preset>& presets,
                                                    const std::vector<SensorDetection::Band>& bands) {
    std::map<char, int> indexOf;
    for (const auto& b : bands) {
        const char s = BandSymbolOf(b.name);
        if (s != 0 && indexOf.find(s) == indexOf.end()) indexOf[s] = b.index;
    }
    static const std::map<std::string, std::string> composites = {{"true-color", "RGB"}, {"false-color-ir", "NRG"}};
    std::vector<SensorDetection::Preset> out;
    for (const auto& p : presets) {
        SensorDetection::Preset built = p;
        if (p.type == "index") {
            if (const Formula* f = FindFormula(p.formula)) {
                const auto syms = RequiredSymbols(*f);
                if (!std::all_of(syms.begin(), syms.end(), [&](char c) { return indexOf.count(c) != 0; })) continue;
            }
        } else if (const auto c = composites.find(p.id); c != composites.end()) {
            const std::string& ch = c->second;
            if (!std::all_of(ch.begin(), ch.end(), [&](char x) { return indexOf.count(x) != 0; })) continue;
            built.mapping = {indexOf[ch[0]], indexOf[ch[1]], indexOf[ch[2]]};
        }
        out.push_back(built);
    }
    return out;
}

SensorDetection Detect(const CogSource& src) {
    SensorDetection result;
    const int bandCount = src.BandCount();
    if (bandCount == 0) return result;
    const std::string dataType = PixelTypeName(src.Type());
    const bool lastAlpha = bandCount >= 2 && src.Band(bandCount).interp == ColorInterp::kAlpha;
    if (bandCount == 4 && src.Type() == PixelType::kU8 && lastAlpha) return result;
    if (bandCount == 3 && src.Type() == PixelType::kU8) return result;
    const auto& metadata = src.DatasetMetadata();
    for (const auto& p : Profiles()) {
        if (p.category != "multispectral" && p.category != "thermal") continue;
        const int effective = lastAlpha ? bandCount - 1 : bandCount;
        if (p.bandCount != effective && p.bandCount != bandCount) continue;
        if (!p.dataType.empty() && p.dataType != dataType) continue;
        if (!p.patterns.empty()) {
            bool any = false;
            for (const auto& pat : p.patterns) {
                for (const auto& md : metadata)
                    if (md.find(pat) != std::string::npos) {
                        any = true;
                        break;
                    }
                if (any) break;
            }
            if (!any) continue;
        }
        result.detected = true;
        result.sensorId = p.id;
        for (const auto& b : p.bands) result.bands.push_back({b.index, b.name});
        for (const auto& pr : p.presets)
            result.presets.push_back({pr.id, pr.type, pr.mapping, pr.isDefault, pr.formula, pr.colormap});
        if (p.patterns.empty()) {
            auto described = result.bands;
            bool named = false;
            for (auto& b : described) {
                if (b.index < 1 || b.index > bandCount) continue;
                const std::string& d = src.Band(b.index).description;
                if (!d.empty()) {
                    b.name = d;
                    if (BandSymbolOf(b.name) != 0) named = true;
                }
            }
            if (named) {
                result.bands = described;
                result.presets = PresetsOfBands(result.presets, result.bands);
            }
        }
        for (const auto& pr : result.presets)
            if (pr.isDefault) {
                result.defaultMapping = pr.mapping;
                result.defaultPresetId = pr.id;
                break;
            }
        if (result.defaultPresetId.empty())
            for (const auto& pr : result.presets)
                if (pr.type == "bands" || pr.type.empty()) {
                    result.defaultMapping = pr.mapping;
                    result.defaultPresetId = pr.id;
                    break;
                }
        if (result.defaultPresetId.empty()) result.defaultMapping = FallbackMapping(src, bandCount);
        return result;
    }
    if (bandCount > 3 && src.Type() != PixelType::kU8) {
        result.detected = true;
        const int last = lastAlpha ? bandCount - 1 : bandCount;
        for (int i = 1; i <= last; i++) {
            std::string name = src.Band(i).description;
            if (name.empty()) name = ColorInterpName(src.Band(i).interp);
            result.bands.push_back({i, name});
        }
        result.defaultMapping = FallbackMapping(src, bandCount);
    }
    return result;
}

}  // namespace

char BandSymbolOf(const std::string& name) {
    std::string n = Lower(Trim(name));
    for (auto& c : n)
        if (c == '-' || c == '_') c = ' ';
    if (n == "re" || n.find("red edge") != std::string::npos || n.find("rededge") != std::string::npos) return 'e';
    if (n == "r" || n.starts_with("red")) return 'R';
    if (n == "g" || n.starts_with("green")) return 'G';
    if (n == "b" || n.starts_with("blue")) return 'B';
    if (n == "n" || n.starts_with("nir") || n.starts_with("near infrared") || n.starts_with("nearinfrared") ||
        n.starts_with("near ir"))
        return 'N';
    return 0;
}

std::shared_ptr<const SensorDetection> DetectSensor(const CogSource& source) {
    return source.SensorMemo([&] { return std::make_shared<const SensorDetection>(Detect(source)); });
}

BandMapping BandMappingForPreset(const CogSource& source, const std::string& preset) {
    const std::shared_ptr<const SensorDetection> held = DetectSensor(source);
    const SensorDetection& det = *held;
    if (det.detected) {
        for (const auto& p : det.presets)
            if (p.id == preset && (p.type == "bands" || p.type.empty())) return p.mapping;
        return det.defaultMapping;
    }
    return FallbackMapping(source, 0);
}

BandFilter AutoDetectFilter(const CogSource& source) {
    const std::shared_ptr<const SensorDetection> held = DetectSensor(source);
    const SensorDetection& det = *held;
    if (det.detected && !det.bands.empty()) {
        BandFilter bf;
        for (const auto& b : det.bands) {
            const std::string lc = Lower(b.name);
            int* slot = nullptr;
            switch (BandSymbolOf(b.name)) {
                case 'e': slot = &bf.Re; break;
                case 'R': slot = &bf.R; break;
                case 'G': slot = &bf.G; break;
                case 'B': slot = &bf.B; break;
                case 'N': slot = &bf.N; break;
                default:
                    if (lc == "lwir" || lc.find("thermal") != std::string::npos) slot = &bf.T;
            }
            if (slot != nullptr && *slot < 0) *slot = b.index - 1;
        }
        std::string fid;
        if (bf.R >= 0) fid += "R";
        if (bf.G >= 0) fid += "G";
        if (bf.B >= 0) fid += "B";
        if (bf.N >= 0) fid += "N";
        if (bf.Re >= 0) fid += "Re";
        if (bf.T >= 0) fid += "T";
        bf.id = fid;
        return bf;
    }
    BandFilter bf;
    int dataBands = 0;
    for (int b = 1; b <= source.BandCount(); b++)
        if (source.Band(b).interp != ColorInterp::kAlpha) dataBands++;
    if (dataBands < 3) return bf;
    bf.id = "RGB";
    bf.R = 0;
    bf.G = 1;
    bf.B = 2;
    return bf;
}

std::vector<int> ResolveVisBands(const CogSource& source, const VisParams& vis, int maxBand) {
    std::vector<int> bands;
    if (!vis.bands.empty()) {
        std::istringstream ss(vis.bands);
        std::string token;
        while (std::getline(ss, token, ',')) {
            int b;
            try {
                b = std::stoi(token);
            } catch (const std::exception&) {
                FailInvalid("Invalid band value: '" + token + "'");
            }
            if (b < 1 || b > maxBand)
                FailInvalid("Band index " + std::to_string(b) + " out of range (1-" + std::to_string(maxBand) + ")");
            bands.push_back(b);
        }
    } else if (!vis.preset.empty()) {
        const BandMapping m = BandMappingForPreset(source, vis.preset);
        bands = {m.r, m.g, m.b};
    } else {
        const std::shared_ptr<const SensorDetection> det = DetectSensor(source);
        if (det->detected) bands = {det->defaultMapping.r, det->defaultMapping.g, det->defaultMapping.b};
    }
    return bands;
}

int StyledBand(int dataBandCount, const std::string& colormap, const std::vector<int>& chosen) {
    if (colormap.empty()) return 0;
    if (chosen.empty()) return dataBandCount == 1 ? 1 : 0;
    for (const int b : chosen)
        if (b != chosen.front()) return 0;
    return chosen.front();
}

}  // namespace ddbt::cog::tiler
