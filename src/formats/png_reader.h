#pragma once

// PNG decoder for mod content (model textures): 8-bit greyscale, grey+alpha,
// RGB, RGBA and palette images (with tRNS), 1-8 bit palettes, non-interlaced.
// Returns RGBA8 (memory order R,G,B,A).

#include "base/bytes.h"

#include <optional>
#include <utility>
#include <vector>

namespace gt3r {

struct PngImage {
    u32 width = 0, height = 0;
    std::vector<u32> rgba;
};

// Throws FormatError on unsupported or corrupt files, or images wider or taller than
// max_dim (decoding allocates width x height x 4 bytes).
PngImage decode_png(ByteSpan file, u32 max_dim = 8192);
// The image size from the header, without decoding (nullopt when it is no PNG).
std::optional<std::pair<u32, u32>> png_size(ByteSpan file);

} // namespace gt3r
