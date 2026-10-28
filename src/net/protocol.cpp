#include "net/protocol.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gt3r::net {

void Writer::f32_(float v) {
    u32 b;
    std::memcpy(&b, &v, 4);
    u32_(b);
}

void Writer::f64_(double v) {
    u64 b;
    std::memcpy(&b, &v, 8);
    u32_(u32(b)), u32_(u32(b >> 32));
}

void Writer::str(const std::string& s) {
    size_t n = std::min<size_t>(s.size(), 255);
    u8_(u8(n));
    data.insert(data.end(), s.begin(), s.begin() + ptrdiff_t(n));
}

bool Reader::need(size_t n) {
    if (!ok_ || d_.size() - pos_ < n) ok_ = false;
    return ok_;
}

u8 Reader::u8_() { return need(1) ? d_[pos_++] : 0; }

u16 Reader::u16_() {
    if (!need(2)) return 0;
    u16 v = u16(d_[pos_] | d_[pos_ + 1] << 8);
    pos_ += 2;
    return v;
}

u32 Reader::u32_() {
    u32 lo = u16_(), hi = u16_();
    return lo | hi << 16;
}

float Reader::f32_() {
    u32 b = u32_();
    float v;
    std::memcpy(&v, &b, 4);
    return std::isfinite(v) ? v : 0.0f; // a peer never puts NaN into our simulation
}

double Reader::f64_() {
    u64 lo = u32_(), hi = u32_();
    u64 b = lo | hi << 32;
    double v;
    std::memcpy(&v, &b, 8);
    return std::isfinite(v) ? v : 0.0;
}

std::string Reader::str() {
    size_t n = u8_();
    if (!need(n)) return {};
    std::string s(reinterpret_cast<const char*>(d_.data() + pos_), n);
    pos_ += n;
    return s;
}

ByteSpan Reader::bytes(size_t n) {
    if (!need(n)) return {};
    ByteSpan s = d_.subspan(pos_, n);
    pos_ += n;
    return s;
}

namespace {

// A single float's bits from a peer: exponent 255 (infinity or NaN to the host) is refused.
bool finite_bits(u32 b) { return (b & 0x7F800000u) != 0x7F800000u; }

void s16_(Writer& w, s16 v) { w.u16_(u16(v)); }
s16 s16_(Reader& r) { return s16(r.u16_()); }

} // namespace

const char* flag_text(FlagKind k) {
    switch (k) {
    case FlagKind::Speed: return "faster than its car can go";
    case FlagKind::Teleport: return "jumped along the track";
    case FlagKind::Accel: return "accelerated beyond what a car can";
    case FlagKind::FinishTime: return "finish time others did not see";
    case FlagKind::Motion: return "moves unlike its car";
    }
    return "unusual driving";
}

void write_snapshot(Writer& w, const CarSnapshot& s) {
    w.u8_(s.player);
    w.u32_(s.tick);
    w.u32_(u32(s.clock));
    w.u8_(u8((s.rev_limiter ? 1 : 0) | (s.respawned ? 2 : 0)));
    w.u8_(s.laps);
    const CarEntry& e = s.entry;
    w.u8_(u8((e.pedal_style & 7) | (e.wheel_type & 7) << 3 | (e.asm_on ? 0x40 : 0) | (e.tcs_on ? 0x80 : 0)));
    w.u8_(u8((e.control & 3) | (e.driver_kind & 15) << 2));
    const CarMotion& m = s.motion;
    for (u32 v : m.pos) w.u32_(v);
    for (u32 v : m.vel) w.u32_(v);
    w.u32_(m.heading), w.u32_(m.pitch), w.u32_(m.roll), w.u32_(m.yaw_rate);
    w.u32_(m.omega), w.u32_(m.steer_cmd);
    for (u32 v : m.axle_speed) w.u32_(v);
    s16_(w, m.pitch_rate), s16_(w, m.roll_rate);
    for (s16 v : m.rim_speed) s16_(w, v);
    for (s16 v : m.susp) s16_(w, v);
    w.u8_(m.gear), w.u8_(m.clutch);
    // The frames, newest first; one the same as the frame before it in the list is a single 0xFF.
    const u8 n = std::min<u8>(s.input_count, kInputHistory);
    w.u8_(n);
    for (u8 i = 0; i < n; ++i) {
        const PadInput& f = s.inputs[i];
        if (i > 0 && f == s.inputs[i - 1]) {
            w.u8_(0xFF);
            continue;
        }
        w.u8_(u8(f.flags & 0x1F));
        const u8 shift = f.shift > 0 ? 1 : f.shift < 0 ? 3 : 0;
        w.u8_(u8((f.kind & 0x0F) | shift << 4 | (f.back ? 0x40 : 0) | (f.third ? 0x80 : 0)));
        s16_(w, f.steer);
        w.u16_(f.throttle), w.u16_(f.brake);
        if (f.back) w.u16_(f.back);
        if (f.third) w.u16_(f.third);
    }
}

bool read_snapshot(Reader& r, CarSnapshot& s) {
    s = CarSnapshot{};
    s.player = r.u8_();
    s.tick = r.u32_();
    s.clock = s32(r.u32_());
    const u8 flags = r.u8_();
    s.rev_limiter = flags & 1, s.respawned = flags & 2;
    s.laps = r.u8_();
    const u8 e = r.u8_(), e2 = r.u8_();
    s.entry = {u8(e & 7), u8((e >> 3) & 7), (e & 0x40) != 0, (e & 0x80) != 0, u8(e2 & 3), u8((e2 >> 2) & 15)};
    CarMotion& m = s.motion;
    bool finite = true;
    auto f32 = [&] {
        const u32 b = r.u32_();
        finite = finite && finite_bits(b);
        return b;
    };
    for (u32& v : m.pos) v = f32();
    for (u32& v : m.vel) v = f32();
    m.heading = f32(), m.pitch = f32(), m.roll = f32(), m.yaw_rate = f32();
    m.omega = f32(), m.steer_cmd = f32();
    for (u32& v : m.axle_speed) v = f32();
    m.pitch_rate = s16_(r), m.roll_rate = s16_(r);
    for (s16& v : m.rim_speed) v = s16_(r);
    for (s16& v : m.susp) v = s16_(r);
    m.gear = r.u8_(), m.clutch = r.u8_();
    const u8 n = r.u8_();
    if (n > kInputHistory) return false;
    for (u8 i = 0; i < n; ++i) {
        PadInput& f = s.inputs[i];
        const u8 fl = r.u8_();
        if (fl == 0xFF) {
            if (i == 0) return false;
            f = s.inputs[i - 1];
            continue;
        }
        if (fl & ~0x1Fu) return false;
        const u8 b = r.u8_();
        f.flags = fl;
        f.kind = u8(b & 0x0F);
        const u8 sh = (b >> 4) & 3;
        f.shift = s8(sh == 3 ? -1 : sh == 2 ? -2 : sh);
        f.steer = s16_(r);
        f.throttle = r.u16_(), f.brake = r.u16_();
        if (b & 0x40) f.back = r.u16_();
        if (b & 0x80) f.third = r.u16_();
    }
    s.input_count = n;
    s.brake = n ? std::min(1.0f, float(s.inputs[0].brake) / 4096.0f) : 0.0f;
    return r.ok() && finite;
}

} // namespace gt3r::net
