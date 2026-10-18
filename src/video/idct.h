#pragma once

#include "base/bytes.h"

namespace gt3r::video {

// 8x8 inverse DCT (ISO/IEC 13818-2 Annex A) in 32-bit fixed point using the
// Loeffler-Ligtenberg-Moschytz factorisation (12 multiplies per 1-D pass,
// 13-bit constants, 2 extra bits of precision between passes). Accurate to
// the IEEE 1180 limits for the coefficient ranges MPEG-2 produces; see
// tests/test_video.cpp. Input and output are raster order; output is not
// clamped.
void idct_8x8(s16 block[64]);

// Double-precision reference transform (slow; for tests).
void idct_8x8_reference(const s16 in[64], double out[64]);

} // namespace gt3r::video
