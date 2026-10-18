#pragma once

// SPU2 reverb ("effect") unit. The SPU2 runs the same reverb as the PS1 SPU,
// once per core: the voices whose effect-send switches (VMIXEL/VMIXER) are
// set are summed, resampled to half rate (24 kHz), run through a network of
// IIR "same side"/"different side" reflections, four comb taps and two
// all-pass filters in a ring buffer in sound RAM, resampled back to 48 kHz
// and scaled by the effect return volume (EVOL).
//
// Presets are libsd's effect modes (sceSdSetEffectAttr), whose register
// sets are the familiar PS1 reverb examples; libsd writes the offsets in
// SPU2 word units (table value x 4 samples). GT3 selects mode 4 ("Studio C",
// the PS1 "Studio Large" set) at boot with EVOL 0x0FFF.
//
// Not thread-safe; owned and called by VoiceMixer under its lock.

#include "base/bytes.h"

#include <array>
#include <vector>

namespace gt3r {

class SpuReverb {
public:
    enum Mode : int {
        Off = 0, Room, StudioA, StudioB, StudioC, Hall, Space, Echo, Delay, Pipe, ModeCount
    };
    static const char* mode_name(int mode);

    // Selects a preset and clears the work area (like sceSdSetEffectAttr
    // followed by sceSdClearEffectWorkArea).
    void set_mode(int mode);
    int mode() const { return mode_; }
    // Effect return volume (EVOL L/R), 0x7FFF = 1.0.
    void set_depth(s16 l, s16 r) { depth_l_ = l, depth_r_ = r; }
    s16 depth_l() const { return depth_l_; }
    s16 depth_r() const { return depth_r_; }
    bool active() const { return mode_ != Off && (depth_l_ || depth_r_); }

    // Feeds one 48 kHz frame of effect-send input; returns the scaled
    // reverb output for the same frame.
    void process(s32 in_l, s32 in_r, s32& out_l, s32& out_r);

private:
    struct Regs {
        u32 dAPF1, dAPF2;
        s32 vIIR, vCOMB1, vCOMB2, vCOMB3, vCOMB4, vWALL, vAPF1, vAPF2;
        u32 mLSAME, mRSAME, mLCOMB1, mRCOMB1, mLCOMB2, mRCOMB2;
        u32 dLSAME, dRSAME, mLDIFF, mRDIFF, mLCOMB3, mRCOMB3, mLCOMB4, mRCOMB4;
        u32 dLDIFF, dRDIFF, mLAPF1, mRAPF1, mLAPF2, mRAPF2;
        s32 vLIN, vRIN;
    };
    static constexpr int kTaps = 39;
    static constexpr int kHist = 64; // power of two >= kTaps

    s32 rd(u32 off) const;
    void wr(u32 off, s32 v);
    s32 run_side(bool right, s32 in);

    int mode_ = Off;
    Regs r_{};
    std::vector<s16> buf_;
    u32 pos_ = 0;
    u32 phase_ = 0;
    std::array<std::array<s32, kHist>, 2> down_{}; // 48 kHz input history
    std::array<std::array<s32, kHist>, 2> up_{};   // zero-stuffed 48 kHz output
    u32 hist_pos_ = 0;
    s16 depth_l_ = 0, depth_r_ = 0;
};

} // namespace gt3r
