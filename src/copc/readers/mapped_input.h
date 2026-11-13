// SPDX-License-Identifier: MPL-2.0
#pragma once

// The PLY, E57, XYZ and PTS readers parse their input in place, so they need it mapped in
// memory. io::OpenByteSource's automatic choice reads with pread on Windows; these readers ask
// for a mapping explicitly, which every platform supports.

#include <memory>
#include <string>

#include "core/io/byte_source.h"
#include "core/util/status.h"

namespace ddbt::copc::readers {

/// The input mapped in memory. `reader` names the reader in the error of a file that is not
/// empty but could not be mapped. An empty file has no data (Data() == nullptr, Size() == 0).
inline Expected<std::unique_ptr<io::ByteSource>> OpenMappedInput(const std::string& path, const char* reader) {
    DDBT_ASSIGN_OR_RETURN(std::unique_ptr<io::ByteSource> src, io::OpenByteSource(path, io::SourceKind::kMmap));
    if (src->Size() > 0 && src->Data() == nullptr)
        return Unsupported(std::string(reader) + " reader needs a memory-mapped file");
    return src;
}

}  // namespace ddbt::copc::readers
