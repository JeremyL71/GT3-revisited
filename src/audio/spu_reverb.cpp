#include "audio/spu_reverb.h"

#include <algorithm>

namespace gt3r {

namespace {

// libsd's effect presets: the 32 reverb registers in SPU register order
// (dAPF1 dAPF2 vIIR vCOMB1 vCOMB2 vCOMB3 vCOMB4 vWALL vAPF1 vAPF2 mLSAME
// mRSAME mLCOMB1 mRCOMB1 mLCOMB2 mRCOMB2 dLSAME dRSAME mLDIFF mRDIFF mLCOMB3
// mRCOMB3 mLCOMB4 mRCOMB4 dLDIFF dRDIFF mLAPF1 mRAPF1 mLAPF2 mRAPF2 vLIN
// vRIN); offsets in units of 4 samples (SPU2 words). The work area size is
// in units of 8 bytes, i.e. 4 samples (Studio C: 0x6FE0 bytes, which fits
// the 0x7800 bytes the game reserves per core).
struct Preset {
    u16 work_units;
    u16 regs[32];
};

constexpr Preset kPresets[SpuReverb::ModeCount] = {
    {0x0002, {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001,
              0x0001, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001, 0x0001, 0x0000, 0x0000,
              0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}},
    {0x04D8, {0x007D, 0x005B, 0x6D80, 0x54B8, 0xBED0, 0x0000, 0x0000, 0xBA80, 0x5800, 0x5300, 0x04D6,
              0x0333, 0x03F0, 0x0227, 0x0374, 0x01EF, 0x0336, 0x01B7, 0x0335, 0x01B6, 0x0334, 0x01B5,
              0x0334, 0x01B5, 0x0334, 0x01B5, 0x01B4, 0x0136, 0x00B8, 0x005C, 0x8000, 0x8000}},
    {0x03E8, {0x0033, 0x0025, 0x70F0, 0x4FA8, 0xBCE0, 0x4410, 0xC0F0, 0x9C00, 0x5280, 0x4EC0, 0x03E4,
              0x031B, 0x03A4, 0x02AF, 0x0372, 0x0266, 0x031C, 0x025D, 0x025C, 0x018E, 0x022F, 0x0135,
              0x01D2, 0x00B7, 0x018F, 0x00B5, 0x00B4, 0x0080, 0x004C, 0x0026, 0x8000, 0x8000}},
    {0x0908, {0x00B1, 0x007F, 0x70F0, 0x4FA8, 0xBCE0, 0x4510, 0xBEF0, 0xB4C0, 0x5280, 0x4EC0, 0x0904,
              0x076B, 0x0824, 0x065F, 0x07A2, 0x0616, 0x076C, 0x05ED, 0x05EC, 0x042E, 0x050F, 0x0305,
              0x0462, 0x02B7, 0x042F, 0x0265, 0x0264, 0x01B2, 0x0100, 0x0080, 0x8000, 0x8000}},
    {0x0DFC, {0x00E3, 0x00A9, 0x6F60, 0x4FA8, 0xBCE0, 0x4510, 0xBEF0, 0xA680, 0x5680, 0x52C0, 0x0DFB,
              0x0B58, 0x0D09, 0x0A3C, 0x0BD9, 0x0973, 0x0B59, 0x08DA, 0x08D9, 0x05E9, 0x07EC, 0x04B0,
              0x06EF, 0x03D2, 0x05EA, 0x031D, 0x031C, 0x0238, 0x0154, 0x00AA, 0x8000, 0x8000}},
    {0x15BC, {0x01A5, 0x0139, 0x6000, 0x5000, 0x4C00, 0xB800, 0xBC00, 0xC000, 0x6000, 0x5C00, 0x15BA,
              0x11BB, 0x14C2, 0x10BD, 0x11BC, 0x0DC1, 0x11C0, 0x0DC3, 0x0DC0, 0x09C1, 0x0BC4, 0x07C1,
              0x0A00, 0x06CD, 0x09C2, 0x05C1, 0x05C0, 0x041A, 0x0274, 0x013A, 0x8000, 0x8000}},
    {0x1ED8, {0x033D, 0x0231, 0x7E00, 0x5000, 0xB400, 0xB000, 0x4C00, 0xB000, 0x6000, 0x5400, 0x1ED6,
              0x1A31, 0x1D14, 0x183B, 0x1BC2, 0x16B2, 0x1A32, 0x15EF, 0x15EE, 0x1055, 0x1334, 0x0F2D,
              0x11F6, 0x0C5D, 0x1056, 0x0AE1, 0x0AE0, 0x07A2, 0x0464, 0x0232, 0x8000, 0x8000}},
    {0x3008, {0x0003, 0x0003, 0x7FFF, 0x7FFF, 0x0000, 0x0000, 0x0000, 0x8100, 0x0000, 0x0000, 0x1FFD,
              0x0FFD, 0x1009, 0x0009, 0x0000, 0x0000, 0x1009, 0x0009, 0x1FFF, 0x1FFF, 0x1FFE, 0x1FFE,
              0x1FFE, 0x1FFE, 0x1FFE, 0x1FFE, 0x1008, 0x1004, 0x0008, 0x0004, 0x8000, 0x8000}},
    {0x3008, {0x0003, 0x0003, 0x7FFF, 0x7FFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1FFD,
              0x0FFD, 0x1009, 0x0009, 0x0000, 0x0000, 0x1009, 0x0009, 0x1FFF, 0x1FFF, 0x1FFE, 0x1FFE,
              0x1FFE, 0x1FFE, 0x1FFE, 0x1FFE, 0x1008, 0x1004, 0x0008, 0x0004, 0x8000, 0x8000}},
    {0x0780, {0x0017, 0x0013, 0x70F0, 0x4FA8, 0xBCE0, 0x4510, 0xBEF0, 0x8500, 0x5F80, 0x54C0, 0x0371,
              0x02AF, 0x02E5, 0x01DF, 0x02B0, 0x01D7, 0x0358, 0x026A, 0x01D6, 0x011E, 0x012D, 0x00B1,
              0x011F, 0x0059, 0x01A0, 0x00E3, 0x0058, 0x0040, 0x0028, 0x0014, 0x8000, 0x8000}},
};

// Half-band low-pass used by the SPU to resample the reverb input to half
// rate and its output back to full rate (39 taps, centre 0x4000).
constexpr s32 kResample[39] = {-1,    0, 2,     0,     -10, 0,     35,   0, -103, 0,  266, 0,    -616,
                               0,     1332, 0,  -2960, 0,   10246, 16384, 10246, 0, -2960, 0, 1332, 0,
                               -616,  0,    266, 0,    -103, 0,    35,   0, -10,  0,  2,   0,    -1};

s32 clamp16(s64 v) { return s32(std::clamp<s64>(v, -32768, 32767)); }

} // namespace

const char* SpuReverb::mode_name(int mode) {
    static const char* names[ModeCount] = {"off",  "room", "studio A", "studio B", "studio C",
                                           "hall", "space", "echo",    "delay",    "pipe"};
    return mode >= 0 && mode < ModeCount ? names[mode] : "?";
}

void SpuReverb::set_mode(int mode) {
    mode_ = std::clamp(mode, 0, ModeCount - 1);
    const Preset& p = kPresets[mode_];
    auto off = [&](int i) { return u32(p.regs[i]) * 4; };
    auto vol = [&](int i) { return s32(s16(p.regs[i])); };
    r_.dAPF1 = off(0), r_.dAPF2 = off(1);
    r_.vIIR = vol(2), r_.vCOMB1 = vol(3), r_.vCOMB2 = vol(4), r_.vCOMB3 = vol(5), r_.vCOMB4 = vol(6);
    r_.vWALL = vol(7), r_.vAPF1 = vol(8), r_.vAPF2 = vol(9);
    r_.mLSAME = off(10), r_.mRSAME = off(11), r_.mLCOMB1 = off(12), r_.mRCOMB1 = off(13);
    r_.mLCOMB2 = off(14), r_.mRCOMB2 = off(15), r_.dLSAME = off(16), r_.dRSAME = off(17);
    r_.mLDIFF = off(18), r_.mRDIFF = off(19), r_.mLCOMB3 = off(20), r_.mRCOMB3 = off(21);
    r_.mLCOMB4 = off(22), r_.mRCOMB4 = off(23), r_.dLDIFF = off(24), r_.dRDIFF = off(25);
    r_.mLAPF1 = off(26), r_.mRAPF1 = off(27), r_.mLAPF2 = off(28), r_.mRAPF2 = off(29);
    r_.vLIN = vol(30), r_.vRIN = vol(31);
    buf_.assign(size_t(p.work_units) * 4, 0);
    pos_ = 0;
    phase_ = 0;
    for (auto& h : down_) h.fill(0);
    for (auto& h : up_) h.fill(0);
}

s32 SpuReverb::rd(u32 off) const { return buf_[(pos_ + off) % buf_.size()]; }
void SpuReverb::wr(u32 off, s32 v) { buf_[(pos_ + off) % buf_.size()] = s16(clamp16(v)); }

// One half-rate step of one side (PS1/SPU2 reverb formula).
s32 SpuReverb::run_side(bool right, s32 in) {
    const Regs& g = r_;
    const u32 n = u32(buf_.size());
    auto back = [n](u32 a, u32 b) { return (a + n - b % n) % n; };
    const u32 mSAME = right ? g.mRSAME : g.mLSAME, dSAME = right ? g.dRSAME : g.dLSAME;
    const u32 mDIFF = right ? g.mRDIFF : g.mLDIFF, dDIFF = right ? g.dLDIFF : g.dRDIFF;
    const u32 c1 = right ? g.mRCOMB1 : g.mLCOMB1, c2 = right ? g.mRCOMB2 : g.mLCOMB2;
    const u32 c3 = right ? g.mRCOMB3 : g.mLCOMB3, c4 = right ? g.mRCOMB4 : g.mLCOMB4;
    const u32 apf1 = right ? g.mRAPF1 : g.mLAPF1, apf2 = right ? g.mRAPF2 : g.mLAPF2;

    s32 x = s32((s64(in) * (right ? g.vRIN : g.vLIN)) >> 15);
    // Same-side and cross-side reflections (first-order IIR into the buffer).
    s32 prev = rd(back(mSAME, 1));
    s32 v = x + s32((s64(rd(dSAME)) * g.vWALL) >> 15);
    wr(mSAME, ((s64(v - prev) * g.vIIR) >> 15) + prev);
    prev = rd(back(mDIFF, 1));
    v = x + s32((s64(rd(dDIFF)) * g.vWALL) >> 15);
    wr(mDIFF, ((s64(v - prev) * g.vIIR) >> 15) + prev);
    // Early echo: four comb taps.
    s64 acc = s64(g.vCOMB1) * rd(c1) + s64(g.vCOMB2) * rd(c2) + s64(g.vCOMB3) * rd(c3) + s64(g.vCOMB4) * rd(c4);
    s32 out = clamp16(acc >> 15);
    // Late reverb: two all-pass filters.
    s32 t = rd(back(apf1, g.dAPF1));
    s32 a = clamp16(out - ((s64(g.vAPF1) * t) >> 15));
    wr(apf1, a);
    out = clamp16(((s64(a) * g.vAPF1) >> 15) + t);
    t = rd(back(apf2, g.dAPF2));
    a = clamp16(out - ((s64(g.vAPF2) * t) >> 15));
    wr(apf2, a);
    out = clamp16(((s64(a) * g.vAPF2) >> 15) + t);
    return out;
}

void SpuReverb::process(s32 in_l, s32 in_r, s32& out_l, s32& out_r) {
    if (mode_ == Off || buf_.empty()) {
        out_l = out_r = 0;
        return;
    }
    hist_pos_ = (hist_pos_ + 1) & (kHist - 1);
    down_[0][hist_pos_] = clamp16(in_l);
    down_[1][hist_pos_] = clamp16(in_r);
    auto fir = [&](const std::array<s32, kHist>& h) {
        s64 acc = 0;
        for (int k = 0; k < kTaps; ++k) acc += s64(kResample[k]) * h[(hist_pos_ - k) & (kHist - 1)];
        return acc;
    };
    // The unit alternates sides: left on even output samples, right on odd
    // ones; the buffer address advances after each pair.
    const int side = int(phase_ & 1);
    s32 y = run_side(side == 1, clamp16(fir(down_[side]) >> 15));
    up_[side][hist_pos_] = y;
    up_[side ^ 1][hist_pos_] = 0;
    if (side == 1) pos_ = (pos_ + 1) % u32(buf_.size());
    phase_ ^= 1;
    // Zero-stuffed upsampling: each output sample sees half the taps, so
    // the filter gain is doubled.
    s32 l = clamp16(fir(up_[0]) >> 14), r = clamp16(fir(up_[1]) >> 14);
    out_l = s32((s64(l) * depth_l_) >> 15);
    out_r = s32((s64(r) * depth_r_) >> 15);
}

} // namespace gt3r
