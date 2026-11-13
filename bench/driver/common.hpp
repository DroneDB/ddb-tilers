// SPDX-License-Identifier: MPL-2.0
// Shared helpers of the ddbt-bench subcommands.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace ddbtb
{

// Raised for invalid command lines: printed with the usage, exit code 2.
struct UsageError : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

// Simple "--key value" / "--flag" parser. Positional arguments are kept in order; everything
// after a bare "--" is kept verbatim in `rest`.
class Args
{
  public:
    Args(int argc, char **argv, int first, std::vector<std::string> flagNames)
        : flags_(std::move(flagNames))
    {
        for (int i = first; i < argc; ++i)
        {
            std::string a = argv[i];
            if (a == "--")
            {
                for (++i; i < argc; ++i)
                    rest.emplace_back(argv[i]);
                break;
            }
            if (a.size() > 2 && a.rfind("--", 0) == 0)
            {
                std::string k = a.substr(2), v;
                const size_t eq = k.find('=');
                if (eq != std::string::npos)
                {
                    v = k.substr(eq + 1);
                    k = k.substr(0, eq);
                }
                else if (std::find(flags_.begin(), flags_.end(), k) == flags_.end())
                {
                    if (i + 1 >= argc)
                        throw UsageError("missing value for --" + k);
                    v = argv[++i];
                }
                else
                    v = "1";
                opts.emplace_back(k, v);
                continue;
            }
            positional.push_back(a);
        }
    }

    bool has(std::string_view k) const
    {
        for (const auto &[n, v] : opts)
            if (n == k)
                return true;
        return false;
    }
    std::string get(std::string_view k, const std::string &def = {}) const
    {
        std::string r = def;
        for (const auto &[n, v] : opts)
            if (n == k)
                r = v; // last one wins
        return r;
    }
    std::vector<std::string> all(std::string_view k) const
    {
        std::vector<std::string> r;
        for (const auto &[n, v] : opts)
            if (n == k)
                r.push_back(v);
        return r;
    }
    std::string require(std::string_view k) const
    {
        if (!has(k))
            throw UsageError("missing --" + std::string(k));
        return get(k);
    }
    long long getInt(std::string_view k, long long def) const
    {
        if (!has(k))
            return def;
        const std::string v = get(k);
        char *end = nullptr;
        const long long r = std::strtoll(v.c_str(), &end, 10);
        if (v.empty() || *end)
            throw UsageError("--" + std::string(k) + " expects an integer, got '" + v + "'");
        return r;
    }
    double getDouble(std::string_view k, double def) const
    {
        if (!has(k))
            return def;
        const std::string v = get(k);
        char *end = nullptr;
        const double r = std::strtod(v.c_str(), &end);
        if (v.empty() || *end)
            throw UsageError("--" + std::string(k) + " expects a number, got '" + v + "'");
        return r;
    }
    // Rejects options that the command does not know (catches typos in scripts).
    void allowOnly(std::initializer_list<std::string_view> known) const
    {
        for (const auto &[n, v] : opts)
            if (std::find(known.begin(), known.end(), n) == known.end())
                throw UsageError("unknown option --" + n);
    }

    std::vector<std::pair<std::string, std::string>> opts;
    std::vector<std::string> positional;
    std::vector<std::string> rest;

  private:
    std::vector<std::string> flags_;
};

// "0,2,4-7" -> {0,2,4,5,6,7}. Empty or "none" -> {}.
inline std::vector<int> parseCpuList(const std::string &s)
{
    std::vector<int> r;
    if (s.empty() || s == "none")
        return r;
    size_t pos = 0;
    while (pos <= s.size())
    {
        size_t comma = s.find(',', pos);
        if (comma == std::string::npos)
            comma = s.size();
        const std::string part = s.substr(pos, comma - pos);
        if (!part.empty())
        {
            const size_t dash = part.find('-');
            char *end = nullptr;
            const long a = std::strtol(part.c_str(), &end, 10);
            if (dash == std::string::npos)
            {
                if (*end)
                    throw UsageError("bad CPU list '" + s + "'");
                r.push_back(static_cast<int>(a));
            }
            else
            {
                const long b = std::strtol(part.c_str() + dash + 1, &end, 10);
                if (*end || b < a)
                    throw UsageError("bad CPU list '" + s + "'");
                for (long i = a; i <= b; ++i)
                    r.push_back(static_cast<int>(i));
            }
        }
        pos = comma + 1;
    }
    return r;
}

// Percentile of sorted values with linear interpolation (numpy's default "linear" method).
template <typename T> double percentileSorted(const std::vector<T> &v, double q)
{
    if (v.empty())
        return NAN;
    const double pos = q * static_cast<double>(v.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(pos));
    const size_t hi = std::min(lo + 1, v.size() - 1);
    const double frac = pos - static_cast<double>(lo);
    return static_cast<double>(v[lo]) + (static_cast<double>(v[hi]) - static_cast<double>(v[lo])) * frac;
}

// Writes text to path ("-" or empty: stdout). Throws on failure.
inline void writeOutput(const std::string &path, const std::string &text)
{
    if (path.empty() || path == "-")
    {
        std::fwrite(text.data(), 1, text.size(), stdout);
        std::fflush(stdout);
        return;
    }
    const std::string tmp = path + ".tmp";
    FILE *f = std::fopen(tmp.c_str(), "wb");
    if (!f)
        throw std::runtime_error("cannot write " + tmp);
    const bool ok = std::fwrite(text.data(), 1, text.size(), f) == text.size();
    if (std::fclose(f) != 0 || !ok)
        throw std::runtime_error("cannot write " + tmp);
    if (std::rename(tmp.c_str(), path.c_str()) != 0)
        throw std::runtime_error("cannot rename " + tmp + " to " + path);
}

int cmdTile(int argc, char **argv);
int cmdRun(int argc, char **argv);
int cmdPagecache(int argc, char **argv);
int cmdSysinfo(int argc, char **argv);
int cmdHash(int argc, char **argv);

} // namespace ddbtb
