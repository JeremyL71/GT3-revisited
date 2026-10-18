#pragma once

#include "base/bytes.h"

namespace gt3r {

// Sony SPU/SPU2 ADPCM ("VAG") decoder. Each 16-byte frame holds 28 samples:
//   byte 0: shift (low nibble) | filter (high nibble)
//   byte 1: flags (bit0 end, bit1 repeat, bit2 loop start)
//   bytes 2..15: 4-bit samples, low nibble first
struct PsAdpcmDecoder {
    static constexpr size_t kFrameBytes = 16;
    static constexpr size_t kFrameSamples = 28;

    s32 hist1 = 0, hist2 = 0;

    // Decodes one frame into `out` (28 samples). Returns the flags byte.
    u8 decode_frame(const u8* frame, s16* out);
    void reset() { hist1 = hist2 = 0; }
};

// Encoder for generated sounds (mod EV motors). Picks the filter and shift
// per frame that decode closest to the input, tracking the decoder's history
// so errors do not accumulate. `pcm` is cut into 28-sample frames (the last
// one zero-padded); `flags(i)` gives frame i's flags byte.
Bytes ps_adpcm_encode(const s16* pcm, size_t count, u8 (*flags)(size_t frame, size_t frames));

} // namespace gt3r
