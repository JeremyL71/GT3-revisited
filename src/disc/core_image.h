#pragma once

#include "base/bytes.h"

namespace gt3r {

// CORE.GT3 holds the main game program (the boot ELF is only a loader).
//
//   file:  u16 0x0101 | u32 unpacked_size | raw DEFLATE stream
//   unpacked:
//     0x000  256-byte signature/key block
//     0x100  u32[5] header: ?, segment count, entry, load address, size
//     0x114  program image, loaded at `load_address`
//
// The engine never executes it. It identifies the disc, and GtData::load reads one
// data table from it (the built-in record names, found by a pattern search). gt3tool's
// `core` command (GT3R_PROGRAM_TOOLS builds only) writes the unpacked image.
struct CoreImage {
    u32 entry_point = 0;
    u32 load_address = 0;
    u32 image_size = 0;
    Bytes program; // bytes loaded at load_address

    static CoreImage decode(ByteSpan core_gt3);
};

} // namespace gt3r
