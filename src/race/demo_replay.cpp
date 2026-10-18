#include "race/demo_replay.h"

#include <algorithm>
#include <array>

namespace gt3r::race {

namespace {

class BitReader {
public:
    explicit BitReader(ByteSpan d) : d_(d) {}
    u32 read(int n) {
        u32 v = 0;
        for (int i = 0; i < n; ++i) {
            size_t byte = pos_ >> 3;
            u32 bit = 0;
            if (byte < d_.size()) bit = (d_[byte] >> (7 - (pos_ & 7))) & 1;
            else overrun_ = true;
            v = (v << 1) | bit;
            ++pos_;
        }
        return v;
    }
    u32 gamma() { // k one-bits, a zero, k value bits: 2^k + value
        int k = 0;
        while (read(1) && k < 31) ++k;
        return (1u << k) + read(k);
    }
    s32 delta() {
        static constexpr std::array<s32, 8> kBase{0, 2, 10, 42, 170, 682, 2730, 10922};
        u32 k = read(3);
        bool neg = read(1);
        s32 v = kBase[k] + s32(read(int(2 * k + 1)));
        return neg ? -v : v;
    }
    bool overrun() const { return overrun_; }
    size_t pos() const { return pos_; }
    void seek(size_t p) { pos_ = p, overrun_ = false; }

private:
    ByteSpan d_;
    size_t pos_ = 0;
    bool overrun_ = false;
};

// Field order in the bitstream with its change-mask bit and width.
struct Field {
    u32 bit;
    int bytes;
    bool is_signed;
};
constexpr std::array<Field, 9> kFields{{
    {0x100, 1, false}, // flags
    {0x080, 1, false}, // kind
    {0x010, 1, true},  // shift
    {0x002, 2, true},  // steer
    {0x001, 2, true},  // |steer|
    {0x004, 2, true},  // throttle
    {0x008, 2, true},  // brake
    {0x040, 2, true},  // axis +0xC
    {0x020, 2, true},  // axis +0xE
}};
constexpr int kSteer = 3;

s32 wrap(s32 v, const Field& f) {
    u32 m = f.bytes == 1 ? 0xFFu : 0xFFFFu;
    u32 u = u32(v) & m;
    if (f.is_signed && (u >> (f.bytes * 8 - 1))) return s32(u) - s32(m) - 1;
    return s32(u);
}

} // namespace

std::optional<DemoReplay> DemoReplay::parse(ByteSpan file, std::string* error) {
    auto fail = [&](const char* m) -> std::optional<DemoReplay> {
        if (error) *error = m;
        return std::nullopt;
    };
    if (file.size() < 0x40) return fail("too small");
    u32 hdr = rd32(file, 0), len = rd32(file, 4);
    if (hdr < 0x10 || hdr > file.size() || len > file.size() - hdr) return fail("bad envelope");
    Bytes body(len);
    for (u32 i = 0; i < len; ++i) body[i] = u8(~file[hdr + i]);
    constexpr size_t kHeader = 0x191C;
    if (body.size() < kHeader + 16 || std::string_view(reinterpret_cast<const char*>(&body[4]), 4) != "GT2K" ||
        rd16(body, 10) != 3)
        return fail("not a version-3 GT2K replay");
    DemoReplay r;
    r.time_ms = rd32(body, 0x78);
    r.automatic = body[0x518 + 0x1F8] == 0;
    r.car_record.assign(body.begin() + 0x518, body.begin() + 0x518 + 0x1D0);
    if (rd32(body, kHeader) != 0) return fail("recorder mode is not a single input stream");
    size_t off = kHeader + 4;
    u32 total = rd32(body, off), blen = rd32(body, off + 4);
    size_t padded = (size_t(blen) + 3) & ~size_t(3);
    if (blen < 8 || off + padded > body.size()) return fail("bad stream length");
    BitReader bits(ByteSpan(body).subspan(off + 8, padded - 8));
    std::array<s32, 9> st{};
    s32 prev_steer = 0;
    u32 run = 0;
    bool exhausted = false;
    for (u32 f = 1; f < total; ++f) {
        s32 cur = st[kSteer], p = prev_steer;
        prev_steer = cur;
        if (std::abs(cur) >= 16) st[kSteer] = std::clamp(2 * cur - p, -0x1000, 0x1000);
        if (run) {
            --run;
        } else if (!exhausted) {
            size_t save = bits.pos();
            u32 mask = bits.gamma() - 1;
            std::array<s32, 9> next = st;
            for (size_t i = 0; i < kFields.size(); ++i)
                if (mask & kFields[i].bit) next[i] = wrap(next[i] + bits.delta(), kFields[i]);
            u32 rn = bits.gamma() - 1;
            if (bits.overrun()) { // the data ran out: hold the values
                exhausted = true;
                bits.seek(save);
            } else {
                st = next, run = rn;
            }
        }
        DemoFrame df;
        df.flags = u8(st[0]);
        df.kind = u8(st[1]);
        df.shift = s8(st[2]);
        df.steer = s16(st[kSteer]);
        df.throttle = s16(st[5]);
        df.brake = s16(st[6]);
        r.frames.push_back(df);
    }
    return r;
}

sim::ControlInputs DemoReplay::inputs(size_t i) const {
    sim::ControlInputs in;
    in.automatic = automatic;
    in.controller_kind = u8(kind(i));
    if (i >= frames.size()) return in; // the game feeds an all-zero frame after the recording
    const DemoFrame& f = frames[i];
    in.steer = std::clamp(float(f.steer) / 4096.0f, -1.0f, 1.0f);
    in.throttle = std::clamp(float(f.throttle) / 4096.0f, 0.0f, 1.0f);
    in.brake = std::clamp(float(f.brake) / 4096.0f, 0.0f, 1.0f);
    in.shift_up = f.shift > 0;
    in.shift_down = f.shift < 0;
    in.automatic = automatic;
    return in;
}

} // namespace gt3r::race
