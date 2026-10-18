#pragma once

#include "formats/gs_memory.h"

#include <string>
#include <vector>

namespace gt3r {

// "Tex1" texture set (.img files, and embedded in other containers).
//
//   0x00 "Tex1"
//   0x0C u32 file size
//   0x14 u16 texture count, u16 buffer count
//   0x18 u32 texture table offset  -> 0x28-byte records of GS registers:
//          +0x00 TEX0, +0x08 TEX1, +0x10/+0x18 (mip registers?), +0x20 CLAMP
//   0x1C u32 buffer table offset   -> 12-byte upload records:
//          u32 data offset, u16 dbp, u8 dbw, u8 dpsm, u16 width, u16 height
//
// Buffers are uploaded into GS memory in their own format; textures are then
// sampled through TEX0, often in a different format (see GsMemory).
struct Tex1 {
    struct Texture {
        Tex0 tex0;
        Clamp clamp;
        u64 tex1 = 0;
    };
    struct Buffer {
        u32 offset = 0;
        u32 dbp = 0, dbw = 0, dpsm = 0, width = 0, height = 0;
    };
    struct Image {
        u32 width = 0, height = 0;
        std::vector<u32> rgba; // RGBA8, row-major
    };

    std::vector<Texture> textures;
    std::vector<Buffer> buffers;

    static bool is_tex1(ByteSpan data);
    // `data` must begin with the "Tex1" header.
    static Tex1 parse(ByteSpan data);

    // Uploads all buffers into a fresh GS memory model.
    void upload(ByteSpan data, GsMemory& gs) const;
    // Decodes texture `i`; cropped to the CLAMP region when it uses
    // REGION_CLAMP, otherwise the full power-of-two size.
    Image decode(const GsMemory& gs, size_t i) const;
};

// Finds every embedded Tex1 set inside an arbitrary container (car models,
// course files and menus embed texture sets). Returns byte offsets.
std::vector<size_t> find_tex1_sets(ByteSpan data);

} // namespace gt3r
