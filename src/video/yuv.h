#pragma once

#include "video/mpeg2_decoder.h"

#include <vector>

namespace gt3r::video {

// Converts a decoded frame (BT.601 limited range, 4:2:0, chroma sited
// between luma samples as nearest-neighbour) to RGBA8 for PNG export.
std::vector<u32> frame_to_rgba(const Frame& f);

} // namespace gt3r::video
