#pragma once

#include "base/bytes.h"

#include <string>
#include <vector>

namespace gt3r {

// Encodes RGBA8 pixels (memory order R,G,B,A) as a PNG file.
Bytes encode_png(u32 width, u32 height, const std::vector<u32>& rgba);

// Encodes interleaved s16 PCM as a WAV file.
Bytes encode_wav(u32 sample_rate, u32 channels, const std::vector<s16>& pcm);

} // namespace gt3r
