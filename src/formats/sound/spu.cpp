#include "formats/sound/spu.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace gt3r::sound {

namespace {

// 384 entries per octave (1/32 semitone), 0x4000 = unity.
const std::array<u16, 384>& pitch_table() {
    static const std::array<u16, 384> t = [] {
        std::array<u16, 384> a{};
        for (int i = 0; i < 384; ++i) a[i] = u16(std::lround(16384.0 * std::exp2(i / 384.0)));
        return a;
    }();
    return t;
}

} // namespace

u16 note_to_pitch(int note_x256, int center_x256) {
    // Table values are 4x unity, so start two octaves down.
    int d = note_x256 - center_x256;
    int shift = -2;
    while (d < 0) {
        d += 12 * 256;
        --shift;
    }
    while (d >= 12 * 256) {
        d -= 12 * 256;
        ++shift;
    }
    u32 v = pitch_table()[d >> 3];
    return u16(shift >= 0 ? (v << shift) : (v >> -shift));
}

void pan_gains(int pan, int& left, int& right) {
    static const std::array<u16, 128> t = [] {
        std::array<u16, 128> a{};
        for (int i = 0; i < 128; ++i) a[i] = u16(std::lround(16384.0 * std::sin(i / 127.0 * std::numbers::pi / 2)));
        return a;
    }();
    pan = std::clamp(pan, 0, 127);
    left = t[127 - pan];
    right = t[pan];
}

int crossfade_gain(int i) {
    static const std::array<u16, 1025> t = [] {
        std::array<u16, 1025> a{};
        for (int k = 0; k <= 1024; ++k) a[k] = u16(std::lround(4096.0 * std::sin(k / 1024.0 * std::numbers::pi / 2)));
        return a;
    }();
    return t[std::clamp(i, 0, 1024)];
}

SpuSample decode_spu_sample(ByteSpan data, size_t offset) {
    SpuSample s;
    PsAdpcmDecoder dec;
    for (size_t o = offset; o + PsAdpcmDecoder::kFrameBytes <= data.size(); o += PsAdpcmDecoder::kFrameBytes) {
        size_t base = s.pcm.size();
        s.pcm.resize(base + PsAdpcmDecoder::kFrameSamples);
        u8 flags = dec.decode_frame(data.data() + o, s.pcm.data() + base);
        ++s.frames;
        if (flags & kFlagLoopStart) s.loop_start = base;
        if (flags & kFlagEnd) {
            s.looped = (flags & kFlagRepeat) != 0;
            break;
        }
    }
    return s;
}

std::vector<s16> render_spu_loops(ByteSpan data, size_t offset, int loops) {
    std::vector<s16> out;
    PsAdpcmDecoder dec;
    size_t loop = offset;
    for (size_t o = offset; o + PsAdpcmDecoder::kFrameBytes <= data.size();) {
        size_t base = out.size();
        out.resize(base + PsAdpcmDecoder::kFrameSamples);
        if (data[o + 1] & kFlagLoopStart) loop = o;
        u8 flags = dec.decode_frame(data.data() + o, out.data() + base);
        if (flags & kFlagEnd) {
            if (!(flags & kFlagRepeat) || loops-- <= 0) break;
            o = loop;
        } else {
            o += PsAdpcmDecoder::kFrameBytes;
        }
    }
    return out;
}

} // namespace gt3r::sound
