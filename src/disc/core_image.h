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
// This is used only for identification and as a reverse-engineering
// reference via gt3tool; the engine never executes it.
struct CoreImage {
    u32 entry_point = 0;
    u32 load_address = 0;
    u32 image_size = 0;
    Bytes program; // bytes loaded at load_address

    static CoreImage decode(ByteSpan core_gt3);
};

} // namespace gt3r
