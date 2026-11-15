// SPDX-License-Identifier: MPL-2.0
//
// The dimension names PDAL gives the points of a LAS file of a point format, in PDAL's order: the
// fields of every format (Overlap included), GpsTime, ScanChannel (formats 6-10), Red, Green and
// Blue, Infrared, then the extra bytes by their names.
#pragma once
#include <string>
#include <vector>
#include "copc/las/las_header.h"

namespace ddbt::copc::las {

/// PDAL's dimension names of a point format 0..10 with these extra-bytes dimensions (a name equal to
/// one of the format, ignoring case, is that dimension, not another).
std::vector<std::string> PdalDimensionNames(int format, const std::vector<std::string>& extra);

/// The names PDAL's LAS reader gives the extra bytes of a file of LAS version 1.`minor`: none before
/// LAS 1.4 (PDAL reads the extra-bytes record of a 1.4 file only), none for an undocumented type,
/// name0, name1... for an array type, and every character other than a letter, a digit, '_' or a
/// space (and a first character that is not a letter) replaced by '_'.
std::vector<std::string> PdalExtraDimensionNames(int minor, const std::vector<ExtraBytesDescriptor>& extra);

}  // namespace ddbt::copc::las
