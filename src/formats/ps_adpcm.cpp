#include "formats/ps_adpcm.h"

#include <algorithm>
#include <cmath>

namespace gt3r {

namespace {
constexpr s32 kFilters[16][2] = {
    {0, 0}, {60, 0}, {115, -52}, {98, -55}, {122, -60},
    // Values 5..15 are invalid on hardware; treat as filter 0.
};
constexpr size_t kFrame = PsAdpcmDecoder::kFrameSamples;
} // namespace

u8 PsAdpcmDecoder::decode_frame(const u8* frame, s16* out) {
    u32 shift = frame[0] & 0x0F;
    u32 filter = (frame[0] >> 4) & 0x0F;
    if (shift > 12) shift = 9; // hardware behavior for out-of-range shifts
    const s32 f0 = kFilters[filter][0];
    const s32 f1 = kFilters[filter][1];
    for (size_t i = 0; i < kFrameSamples; ++i) {
        u8 byte = frame[2 + i / 2];
        s32 nibble = (i & 1) ? (byte >> 4) : (byte & 0x0F);
        s32 sample = s32(s16(u16(nibble << 12))) >> shift;
        sample += (hist1 * f0 + hist2 * f1 + 32) >> 6;
        sample = std::clamp(sample, -32768, 32767);
        hist2 = hist1;
        hist1 = sample;
        out[i] = s16(sample);
    }
    return frame[1];
}

Bytes ps_adpcm_encode(const s16* pcm, size_t count, u8 (*flags)(size_t, size_t)) {
    const size_t frames = (count + kFrame - 1) / kFrame;
    Bytes out(frames * PsAdpcmDecoder::kFrameBytes, 0);
    s32 h1 = 0, h2 = 0;
    for (size_t f = 0; f < frames; ++f) {
        s32 in[kFrame] = {};
        for (size_t i = 0; i < kFrame && f * kFrame + i < count; ++i) in[i] = pcm[f * kFrame + i];
        double best_err = -1;
        u8 best[PsAdpcmDecoder::kFrameBytes] = {};
        s32 best_h1 = 0, best_h2 = 0;
        for (int filter = 0; filter < 5; ++filter)
            for (int shift = 0; shift <= 12; ++shift) {
                const s32 f0 = kFilters[filter][0], f1 = kFilters[filter][1];
                s32 a = h1, b = h2;
                u8 frame[PsAdpcmDecoder::kFrameBytes] = {};
                frame[0] = u8(filter << 4 | shift);
                double err = 0;
                for (size_t i = 0; i < kFrame; ++i) {
                    s32 pred = (a * f0 + b * f1 + 32) >> 6;
                    // nibble n decodes to (n << 12 >> shift): pick the nearest.
                    double want = double(in[i] - pred) * double(1 << shift) / 4096.0;
                    s32 n = s32(std::lround(want));
                    n = std::clamp(n, -8, 7);
                    s32 v = std::clamp((s32(s16(u16(n << 12))) >> shift) + pred, -32768, 32767);
                    double e = double(v - in[i]);
                    err += e * e;
                    b = a;
                    a = v;
                    frame[2 + i / 2] |= u8((n & 0xF) << ((i & 1) * 4));
                    if (best_err >= 0 && err >= best_err) break;
                }
                if (best_err < 0 || err < best_err) {
                    best_err = err;
                    std::copy(frame, frame + PsAdpcmDecoder::kFrameBytes, best);
                    best_h1 = a, best_h2 = b;
                }
            }
        best[1] = flags ? flags(f, frames) : 0;
        std::copy(best, best + PsAdpcmDecoder::kFrameBytes, out.begin() + f * PsAdpcmDecoder::kFrameBytes);
        h1 = best_h1, h2 = best_h2;
    }
    return out;
}

} // namespace gt3r
