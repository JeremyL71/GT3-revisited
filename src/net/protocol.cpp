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

s32 q(float v, float unit, s32 lim) { return s32(std::clamp<long>(std::lround(double(v) / unit), -lim, lim)); }
void s16q(Writer& w, float v, float unit) { w.u16_(u16(s16(q(v, unit, 32767)))); }
float s16q(Reader& r, float unit) { return float(s16(r.u16_())) * unit; }
void s8q(Writer& w, float v, float unit) { w.u8_(u8(s8(q(v, unit, 127)))); }
float s8q(Reader& r, float unit) { return float(s8(r.u8_())) * unit; }
void s24q(Writer& w, float v, float unit) {
    u32 x = u32(q(v, unit, 0x7FFFFF));
    w.u8_(u8(x)), w.u8_(u8(x >> 8)), w.u8_(u8(x >> 16));
}
float s24q(Reader& r, float unit) {
    u32 x = r.u8_();
    x |= u32(r.u8_()) << 8;
    x |= u32(r.u8_()) << 16;
    if (x & 0x800000) x |= 0xFF000000u;
    return float(s32(x)) * unit;
}
u8 unit8(float v) { return u8(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f)); }

constexpr float kPos = 0.001f;      // m (±8388 m)
constexpr float kVel = 0.01f;       // m/s (±327 m/s)
constexpr float kAng = 0.001f;      // rad/s (±32 rad/s)
constexpr float kHeading = 6.2831853f / 65536.0f;
constexpr float kSteer = 0.005f;    // rad (±0.635)
constexpr float kOmega = 0.1f;      // rad/s (±3276)
constexpr float kSusp = 0.002f;     // m (±0.254)
constexpr float kSlipRatio = 0.01f;
constexpr float kSlipAngle = 0.005f; // rad

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

u32 pack_quat(const sim::Quat& in) {
    float c[4] = {in.w, in.x, in.y, in.z};
    float n = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2] + c[3] * c[3]);
    if (!(n > 1e-6f)) return 0; // identity (largest = w, rest 0)
    int big = 0;
    for (int i = 1; i < 4; ++i)
        if (std::fabs(c[i]) > std::fabs(c[big])) big = i;
    float sign = c[big] < 0 ? -1.0f : 1.0f; // q and -q are the same rotation: make the largest positive
    u32 v = u32(big);
    int shift = 2;
    for (int i = 0; i < 4; ++i) {
        if (i == big) continue;
        float x = c[i] * sign / n * 1.41421356f; // -1..1
        u32 k = u32(std::clamp<long>(std::lround((x + 1.0f) * 511.5f), 0, 1023));
        v |= k << shift;
        shift += 10;
    }
    return v;
}

sim::Quat unpack_quat(u32 v) {
    int big = int(v & 3);
    float c[4] = {};
    float sum = 0;
    int shift = 2;
    for (int i = 0; i < 4; ++i) {
        if (i == big) continue;
        float x = (float((v >> shift) & 1023) / 511.5f - 1.0f) / 1.41421356f;
        c[i] = x;
        sum += x * x;
        shift += 10;
    }
    c[big] = std::sqrt(std::max(0.0f, 1.0f - sum));
    return {c[0], c[1], c[2], c[3]};
}

InputFrame pack_input(const sim::ControlInputs& in, bool reverse) {
    InputFrame f;
    f.steer = s8(std::clamp<long>(std::lround(std::clamp(in.steer, -1.0f, 1.0f) * 127.0f), -127, 127));
    f.throttle = unit8(in.throttle);
    f.brake = unit8(in.brake);
    f.flags = u8((in.handbrake ? 1 : 0) | (in.shift_up ? 2 : 0) | (in.shift_down ? 4 : 0) | (in.automatic ? 8 : 0) |
                 (reverse ? 16 : 0) | ((in.controller_kind & 3) << 5));
    return f;
}

sim::ControlInputs unpack_input(const InputFrame& f) {
    sim::ControlInputs in;
    in.steer = float(f.steer) / 127.0f;
    in.throttle = float(f.throttle) / 255.0f;
    in.brake = float(f.brake) / 255.0f;
    in.handbrake = f.flags & 1;
    in.shift_up = f.flags & 2;
    in.shift_down = f.flags & 4;
    in.automatic = f.flags & 8;
    in.controller_kind = u8((f.flags >> 5) & 3);
    return in;
}

void write_snapshot(Writer& w, const CarSnapshot& s) {
    const sim::VehicleState& st = s.state;
    w.u8_(s.player);
    w.u32_(s.tick);
    s24q(w, st.position.x, kPos), s24q(w, st.position.y, kPos), s24q(w, st.position.z, kPos);
    w.u32_(pack_quat(st.orientation));
    s16q(w, st.velocity.x, kVel), s16q(w, st.velocity.y, kVel), s16q(w, st.velocity.z, kVel);
    s16q(w, st.angular_velocity.x, kAng), s16q(w, st.angular_velocity.y, kAng), s16q(w, st.angular_velocity.z, kAng);
    float h = std::remainder(s.heading, 6.2831853f);
    w.u16_(u16(std::lround((h < 0 ? h + 6.2831853f : h) / kHeading) & 0xFFFF));
    s16q(w, s.yaw_rate, kAng);
    w.u16_(u16(std::clamp(std::lround(st.engine_rpm), 0L, 65535L)));
    w.u8_(u8(s8(std::clamp(st.gear, -1, 127))));
    w.u8_(u8((st.rev_limiter ? 1 : 0) | (s.respawned ? 2 : 0)));
    w.u8_(s.laps);
    w.u8_(unit8(st.throttle));
    for (const sim::WheelState& wh : st.wheels) s8q(w, wh.steer_angle, kSteer);
    for (const sim::WheelState& wh : st.wheels) s16q(w, wh.angular_velocity, kOmega);
    for (const sim::WheelState& wh : st.wheels) s8q(w, wh.suspension, kSusp);
    for (const sim::WheelState& wh : st.wheels) s8q(w, wh.slip_ratio, kSlipRatio);
    for (const sim::WheelState& wh : st.wheels) s8q(w, wh.slip_angle, kSlipAngle);
    for (const sim::WheelState& wh : st.wheels) w.u16_(u16(wh.surface));
    u8 grounded = 0;
    for (size_t i = 0; i < 4; ++i) grounded |= u8(st.wheels[i].grounded ? 1 << i : 0);
    w.u8_(grounded);
    u8 n = std::min<u8>(s.input_count, kInputHistory);
    w.u8_(n);
    for (u8 i = 0; i < n; ++i) {
        const InputFrame& f = s.inputs[i];
        w.u8_(u8(f.steer)), w.u8_(f.throttle), w.u8_(f.brake), w.u8_(f.flags);
    }
}

bool read_snapshot(Reader& r, CarSnapshot& s) {
    s = CarSnapshot{};
    sim::VehicleState& st = s.state;
    s.player = r.u8_();
    s.tick = r.u32_();
    st.position.x = s24q(r, kPos), st.position.y = s24q(r, kPos), st.position.z = s24q(r, kPos);
    st.orientation = unpack_quat(r.u32_());
    st.velocity.x = s16q(r, kVel), st.velocity.y = s16q(r, kVel), st.velocity.z = s16q(r, kVel);
    st.angular_velocity.x = s16q(r, kAng), st.angular_velocity.y = s16q(r, kAng), st.angular_velocity.z = s16q(r, kAng);
    s.heading = float(r.u16_()) * kHeading;
    s.yaw_rate = s16q(r, kAng);
    st.engine_rpm = float(r.u16_());
    st.gear = s8(r.u8_());
    u8 sflags = r.u8_();
    st.rev_limiter = sflags & 1;
    s.respawned = sflags & 2;
    s.laps = r.u8_();
    st.throttle = float(r.u8_()) / 255.0f;
    for (sim::WheelState& wh : st.wheels) wh.steer_angle = s8q(r, kSteer);
    for (sim::WheelState& wh : st.wheels) wh.angular_velocity = s16q(r, kOmega);
    for (sim::WheelState& wh : st.wheels) wh.suspension = s8q(r, kSusp);
    for (sim::WheelState& wh : st.wheels) wh.slip_ratio = s8q(r, kSlipRatio);
    for (sim::WheelState& wh : st.wheels) wh.slip_angle = s8q(r, kSlipAngle);
    for (sim::WheelState& wh : st.wheels) wh.surface = r.u16_();
    u8 grounded = r.u8_();
    for (size_t i = 0; i < 4; ++i) st.wheels[i].grounded = grounded & (1 << i);
    u8 n = r.u8_();
    if (n > kInputHistory) return false;
    for (u8 i = 0; i < n; ++i) {
        InputFrame& f = s.inputs[i];
        f.steer = s8(r.u8_()), f.throttle = r.u8_(), f.brake = r.u8_(), f.flags = r.u8_();
    }
    s.input_count = n;
    s.brake = n ? float(s.inputs[0].brake) / 255.0f : 0.0f;
    st.speed_kmh = st.velocity.length() * 3.6f;
    return r.ok();
}

} // namespace gt3r::net
