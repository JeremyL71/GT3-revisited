#include "sim/models.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gt3r::sim::model {

// --- generic -----------------------------------------------------------------

float table_lookup(const float* x, const float* y, int n, float v) {
    if (n <= 0) return 0;
    if (v <= x[0]) return y[0];
    if (v >= x[n - 1]) return y[n - 1];
    int i = 1;
    while (i < n - 1 && v > x[i]) ++i;
    float span = x[i] - x[i - 1];
    float t = span > 0 ? (v - x[i - 1]) / span : 1.0f;
    return y[i - 1] + (y[i] - y[i - 1]) * t;
}

void euler_axes(float a, float b, float c, float forward[3], float lateral[3], float up[3]) {
    float sa = std::sin(a), ca = std::cos(a), sb = std::sin(b), cb = std::cos(b), sc = std::sin(c), cc = std::cos(c);
    forward[0] = cc * sb * sa - sc * ca;
    forward[1] = sc * sb * sa + cc * ca;
    forward[2] = cb * sa;
    lateral[0] = cc * cb;
    lateral[1] = sc * cb;
    lateral[2] = -sb;
    up[0] = cc * sb * ca + sc * sa;
    up[1] = sc * sb * ca - cc * sa;
    up[2] = cb * ca;
}

// --- engine ------------------------------------------------------------------

float curve_torque(const EngineSpec& e, float rpm) {
    return table_lookup(e.curve_rpm.data(), e.curve_torque.data(),
                        int(std::min(e.curve_rpm.size(), e.curve_torque.size())), rpm);
}

int build_torque_curve(const u16* torque, const u8* rpm_x100, int n, u16 out_factor, u16 hi_permille, u8 lo_percent,
                       float* out_rpm, float* out_nm, const TurboSpec* turbo) {
    n = std::clamp(n, 0, 16);
    if (n == 0) return 0;
    float f_out = out_factor ? out_factor * 1e-5f : 0.01f;
    float s_lo = lo_percent ? lo_percent * 0.01f : 1.0f;
    float s_hi = hi_permille ? hi_permille * 0.001f : 1.0f;
    float w0 = rpm_x100[0] * 100.0f, w1 = rpm_x100[n - 1] * 100.0f;
    for (int i = 0; i < n; ++i) {
        float w = rpm_x100[i] * 100.0f;
        float t = w1 > w0 ? (w - w0) / (w1 - w0) : 0.0f;
        out_rpm[i] = w;
        out_nm[i] = torque[i] * 9.8f * f_out * (s_lo + (s_hi - s_lo) * t);
        if (turbo && turbo->on) {
            float b = turbo_boost_raw(*turbo, w, w);
            if (b > 0) out_nm[i] *= std::min(b, turbo->cap) / b;
        }
    }
    return n;
}

float friction_torque(const EngineSpec& e, float rpm) {
    return e.friction_coeff * std::max(rpm, 0.0f) * kRpmToRadS;
}

EngineOut engine_torque(const EngineSpec& e, float rpm, float throttle, bool limiter_was, float condition) {
    EngineOut out;
    float x = std::clamp(throttle, 0.0f, 1.0f);
    out.limiter = rpm >= e.rev_limit_rpm || (limiter_was && rpm >= e.rev_limit_rpm - e.limiter_hysteresis_rpm);
    if (out.limiter) x = 0;
    out.throttle = x;
    out.wot = curve_torque(e, rpm) * condition;
    float drag = friction_torque(e, rpm);
    if (e.electric) { // lift-off regen: constant torque, power-limited, fading out below ~150 motor rpm
        float w = std::max(rpm, 0.0f) * kRpmToRadS;
        float regen = e.regen_torque;
        if (e.regen_power > 0 && w > 0) regen = std::min(regen, e.regen_power / w);
        drag += regen * std::clamp(rpm / 150.0f, 0.0f, 1.0f); // the hold brake takes over (Vehicle::step)
    }
    out.torque = x * out.wot - (1 - x) * drag;
    return out;
}

float turbo_stage(const TurboSpec::Stage& st, float n) {
    if (!st.active || st.knee_rpm <= 0) return 0;
    float N = st.knee_rpm, A = st.peak;
    if (n < N) {
        float u = (N - n) / N;
        return A * (1 - u * u);
    }
    if (n <= 3 * N) {
        float u = (n - N) / N;
        return A - 0.25f * A * u * u;
    }
    return 0;
}

float turbo_boost_raw(const TurboSpec& t, float n0, float n1) {
    return 1 + turbo_stage(t.stage[0], n0) + turbo_stage(t.stage[1], n1);
}

float turbo_boost(const TurboSpec& t, float n0, float n1) { return std::min(turbo_boost_raw(t, n0, n1), t.cap); }

float turbo_step(const TurboSpec& t, TurboState& s, float x, int rpm, float dt) {
    if (!t.on) return x;
    float n = float(rpm);
    float bn = turbo_boost(t, n, n);
    float target = bn > 0 ? s.load * n / bn : 0;
    for (int i = 0; i < 2; ++i) {
        if (!t.stage[size_t(i)].active) continue;
        int& lag = s.lag[size_t(i)];
        lag += int((target - float(lag)) * t.stage[size_t(i)].rate * dt);
        if (lag < 0) lag = 0;
    }
    s.load = x * turbo_boost(t, float(s.lag[0]), float(s.lag[1]));
    return bn > 0 ? std::min(1.0f, s.load / bn) : x;
}

float clutch_slip_torque(float wot, float t, float dw) {
    float d = std::fabs(dw);
    float s = std::min(1.0f + std::max(d - 52.36f, 0.0f) / 837.76f, 1.125f);
    float mag = 0.05f * wot + (t > 0 ? s * t : 0.0f);
    return dw >= 0 ? mag : -mag;
}

// --- gearbox -------------------------------------------------------------------

float total_ratio(const GearboxSpec& g, int gear) {
    if (gear < 0) return -g.reverse * g.final_drive;
    if (gear == 0 || gear > int(g.ratios.size())) return 0;
    return g.ratios[size_t(gear - 1)] * g.final_drive;
}

float upshift_rpm(const EngineSpec& e, const GearboxSpec& g, int gear) {
    float cap = e.rev_limit_rpm - 100;
    if (gear < 1 || gear >= int(g.ratios.size())) return cap;
    float r0 = g.ratios[size_t(gear - 1)], r1 = g.ratios[size_t(gear)];
    if (r0 <= 0 || r1 <= 0) return cap;
    float k = r1 / r0;
    for (float rpm = e.idle_rpm * 2; rpm < cap; rpm += 25) {
        if (curve_torque(e, rpm * k) * r1 >= curve_torque(e, rpm) * r0) return rpm;
    }
    return cap;
}

int auto_shift(const EngineSpec& e, const GearboxSpec& g, const AutoShiftIn& in) {
    int n = int(g.ratios.size());
    if (in.gear < 1 || n == 0) return in.gear;
    float t = std::clamp(in.throttle * 1.4f, 0.0f, 1.0f);
    auto up_at = [&](int gear) { return upshift_rpm(e, g, gear) * (0.65f + 0.35f * t); };
    if (in.gear < n && in.rpm > up_at(in.gear) && in.brake < 0.1f) return in.gear + 1;
    if (in.gear > 1) {
        float lower = in.road_rpm * g.ratios[size_t(in.gear - 2)] / g.ratios[size_t(in.gear - 1)];
        if (in.throttle > 0.5f) {
            if (in.road_rpm < e.idle_rpm * 1.6f && lower < up_at(in.gear - 1) * 0.9f) return in.gear - 1;
        } else {
            float limit = up_at(in.gear - 1) * (in.brake > 0.1f ? 0.80f : 0.70f);
            if (lower < limit || in.road_rpm < e.idle_rpm * 1.3f) return in.gear - 1;
        }
    }
    return in.gear;
}

int gear_for_speed(const GearboxSpec& g, float speed, float r, float up) {
    int n = int(g.ratios.size());
    if (speed < 0) return 0;
    float w_lim = (up - 500) * kRpmToRadS;
    for (int k = 1; k < n; ++k) {
        float per = total_ratio(g, k) / std::max(r, 0.05f); // engine rad/s per m/s
        if (speed < w_lim / per) return k;
    }
    return n;
}

int shift_logic(const EngineSpec& e, const GearboxSpec& g, const ShiftIn& in) {
    int n = int(g.ratios.size());
    float kmh = in.speed * 3.6f;
    // Reverse handling.
    if (in.reverse_request) return -1;
    // Pedal pressed below +1 km/h (also while reversing) -> first gear (the launch flag).
    if (in.automatic && in.throttle > 0 && kmh < 1) return 1;
    if (in.gear < 0) return kmh > 1 ? 1 : in.gear;
    // Rolling backwards in a forward gear -> reverse. Deviation: only with the
    // throttle released (the spec rule is unconditional, which reverses a car
    // that bounces off a wall with the throttle held).
    if (kmh < -1 && in.gear >= 1 && in.throttle < 0.05f) return -1;
    float up = e.upshift_rpm > 0 ? e.upshift_rpm : e.rev_limit_rpm - 500;
    auto rpm_at = [&](int gear, float v) { return v / in.driven_radius * total_ratio(g, gear) * kRadSToRpm; };
    // Over-rev protection.
    if (in.gear >= 1 && in.gear < n && rpm_at(in.gear, in.speed) > e.rev_limit_rpm + 500) return in.gear + 1;
    if (!in.automatic) {
        int to = std::clamp(in.gear + in.request, 1, std::max(n, 1));
        return in.request ? to : in.gear;
    }
    if (!in.clutch_locked || in.gear < 1) return in.gear;
    float down = 0;
    if (in.gear >= 2) {
        int k = in.gear - 1; // down-shift k+1 -> k
        float per10 = rpm_at(in.gear, 10.0f / 3.6f);
        float launch = std::max(e.launch_rpm, e.idle_rpm + 500); // [physics §2.4] launch rpm
        down = std::max(up * g.ratios[size_t(in.gear - 1)] / g.ratios[size_t(k - 1)] - per10, launch + 500);
        if (in.throttle > 0.25f) down -= 500; // B3 = 5
    }
    if (in.gear >= 2 && in.rpm < down) return in.gear - 1;
    if (in.gear < n && in.rpm > up && !in.traction_veto) return in.gear + 1;
    return in.gear;
}

namespace {

// PS2 EE FPU single precision [physics §6.7]: mul/add/sub round towards
// zero, no denormals (flushed to zero), overflow saturates to the largest
// magnitude, and add/sub first drop the smaller operand's mantissa bits beyond
// one guard position; div.s rounds to nearest-even. Values are carried as
// floats; the helpers work on their bit patterns.
u32 bits(float f) {
    u32 b;
    std::memcpy(&b, &f, 4);
    return b;
}
float from_bits(u32 b) {
    float f;
    std::memcpy(&f, &b, 4);
    return f;
}
double ps2_value(float f) {
    u32 b = bits(f);
    if (((b >> 23) & 0xFF) == 0) return (b & 0x80000000u) ? -0.0 : 0.0;
    int e = int((b >> 23) & 0xFF);
    double v = std::ldexp(double((b & 0x7FFFFFu) | 0x800000u), e - 127 - 23);
    return (b & 0x80000000u) ? -v : v;
}
// Double -> PS2 float, truncating (or rounding to nearest) the mantissa.
float ps2_round(double v, bool nearest) {
    u32 sign = std::signbit(v) ? 0x80000000u : 0;
    if (v == 0) return from_bits(sign);
    int e;
    double m = std::frexp(std::fabs(v), &e);
    double mm_d = std::ldexp(m, 24);
    u64 mm = u64(nearest ? std::nearbyint(mm_d) : std::trunc(mm_d));
    if (mm >= (u64(1) << 24)) {
        mm >>= 1;
        ++e;
    }
    int field = e + 126;
    if (field <= 0) return from_bits(sign);
    if (field > 255) return from_bits(sign | 0x7FFFFFFFu);
    return from_bits(sign | u32(field) << 23 | (u32(mm) & 0x7FFFFFu));
}
float chop(double v) { return ps2_round(v, false); }
float mul(float a, float b) {
    double p = ps2_value(a) * ps2_value(b);
    return p == 0 ? from_bits((bits(a) ^ bits(b)) & 0x80000000u) : chop(p);
}
float add(float a, float b) {
    u32 x = bits(a), y = bits(b);
    if (((x >> 23) & 0xFF) == 0) x &= 0x80000000u;
    if (((y >> 23) & 0xFF) == 0) y &= 0x80000000u;
    int d = int((x >> 23) & 0xFF) - int((y >> 23) & 0xFF);
    if (d >= 25) y &= 0x80000000u;
    else if (d > 1) y &= 0xFFFFFFFFu << (d - 1);
    else if (d <= -25) x &= 0x80000000u;
    else if (d < -1) x &= 0xFFFFFFFFu << (-d - 1);
    double s = ps2_value(from_bits(x)) + ps2_value(from_bits(y));
    return s == 0 ? 0.0f : chop(s);
}
float sub(float a, float b) { return add(a, from_bits(bits(b) ^ 0x80000000u)); }
float div(float a, float b) {
    u32 sign = (bits(a) ^ bits(b)) & 0x80000000u;
    if (((bits(b) >> 23) & 0xFF) == 0) return from_bits(sign | 0x7FFFFFFFu);
    double q = ps2_value(a) / ps2_value(b);
    return q == 0 ? from_bits(sign) : ps2_round(q, true);
}
int32_t to_int(float f) {
    double v = std::trunc(ps2_value(f));
    return int32_t(std::clamp(v, -2147483648.0, 2147483647.0));
}

float word_float(u32 w) {
    float f;
    std::memcpy(&f, &w, sizeof f);
    return f;
}

} // namespace

bool clutch_anti_stall(float brake, float w, float idle_rpm) {
    float k = word_float(0x3DD67750);               // 2pi/60 (this exact float)
    float lim = mul(add(idle_rpm, 500.0f), k);      // chopped PS2 multiply
    return brake > 0.75f && w < lim;
}

void generate_gear_ratios(int n, int rev_byte, int final_word, int rim, int wc, int ac, int slider, s16 out[8], int drive,
                          int awd_system) {
    static const u8 kSteps[3][6] = {{155, 165, 188, 190, 190, 200}, {165, 175, 190, 192, 190, 200},
                                    {170, 180, 192, 194, 190, 200}};
    const u8* b = kSteps[std::clamp(n, 5, 7) - 5];
    const float k_rpm = word_float(0x3DD67750); // 0.10472
    const float k_kmh = word_float(0x3E8E38E4); // 0.277778
    float t = div(float(10 * slider - 150), 250.0f);
    float v = mul(float(10 * slider), k_kmh);
    float rpm = float(100 * rev_byte);
    float F = float(double(final_word) * 0.001); // double, then rounded to float
    // Driven tyre [physics §6.7]: front for FF and for AWD system 0, rear otherwise. A
    // width code 0 gives no sidewall on the front path and 10 mm on the rear one.
    bool front = drive == 1 || (drive == 2 && awd_system == 0);
    const float c_mm = word_float(0x3A83126F), c_in = word_float(0x3CD013A9), c_sw = word_float(0x3D4CCCCD);
    float W = mul(float(wc ? 5 * wc : (front ? 0 : 10)), c_mm); // rear path: width 0 -> 10 mm
    float R = add(mul(mul(float(rim), c_in), 0.5f), mul(mul(W, float(ac)), c_sw));
    float r = div(div(mul(mul(rpm, k_rpm), R), v), F);
    auto q = [&](int hi, int lo) {
        float a = div(200.0f, float(b[lo])), c = div(200.0f, float(b[hi]));
        return add(a, mul(t, sub(c, a)));
    };
    float q1 = q(0, 1), q2 = q(2, 3), q3 = q(4, 5);
    for (int i = std::min(n, 7); i >= 1; --i) {
        if (i == 1) r = mul(r, q3);
        // cvt.w.s truncates and saturates; the store keeps the low 16 bits.
        out[i] = s16(u16(u32(to_int(mul(r, 1000.0f)))));
        r = mul(r, q1);
        q1 = mul(q1, q2);
    }
    out[0] = s16((3 * int(out[1]) + int(out[2])) / 4);
}

// --- surfaces ------------------------------------------------------------------------

int surface_index(u32 raw) {
    int m = int(raw & 0xFF), f = int((raw >> 8) & 0xFF);
    if (f & 2) {
        if (m == 0 || m == 7) return 12;
        if (m == 1) return 13;
        if (m == 9) return 14;
    }
    return std::clamp(m, 0, 15);
}

namespace {
struct SurfaceRow {
    float amp, rr, wavelength;
    bool speed_scaled;
};
// [physics §3.6] static per-surface table.
constexpr SurfaceRow kSurface[16] = {
    {0.000f, 0.10f, 0.5f, false}, {0.010f, 0.10f, 0.5f, false}, {0.010f, 0.20f, 2.5f, true},
    {0.024f, 25.5f, 5.0f, true},  {0.020f, 0.30f, 5.0f, true},  {0.010f, 0.20f, 0.5f, true},
    {0.000f, 0.10f, 1.0f, true},  {0.000f, 0.10f, 1.0f, false}, {0.000f, 0.10f, 1.0f, false},
    {0.000f, 0.10f, 1.0f, false}, {0.000f, 0.10f, 1.0f, false}, {0.000f, 0.10f, 1.0f, false},
    {0.000f, 0.10f, 1.0f, false}, {0.000f, 0.10f, 1.0f, false}, {0.000f, 0.10f, 1.0f, false},
    {0.000f, 0.10f, 1.0f, false},
};
// [placeholder] grip per surface; materials per the track workstream: 0 tarmac,
// 1 kerb, 2 grass, 3 gravel, 4 sand, 5 dirt, 7 paving, 8 loose, 9 rough,
// 12/13/14 wet tarmac/kerb/rough.
constexpr float kSurfaceMu[16] = {1.00f, 0.95f, 0.60f, 0.55f, 0.50f, 0.80f, 0.90f, 0.97f,
                                  0.75f, 0.95f, 1.00f, 1.00f, 0.75f, 0.70f, 0.72f, 0.75f};
} // namespace

float surface_rolling(int s) { return kSurface[std::clamp(s, 0, 15)].rr; }
float surface_mu(int s) { return kSurfaceMu[std::clamp(s, 0, 15)]; }

float road_bump(int s, float x, float z, float hub_speed) {
    const SurfaceRow& r = kSurface[std::clamp(s, 0, 15)];
    if (r.amp == 0) return 0;
    float f = 2 * kPiF / r.wavelength;
    float w = std::sin(f * x) + std::cos(f * z); // [physics §3.6]
    w = w * std::fabs(w);                           // wave mode 1: signed square
    float scale = r.speed_scaled ? std::min(1.0f, 0.02f * std::fabs(hub_speed)) : 1.0f;
    return r.amp * w * scale;
}

// --- tyres ---------------------------------------------------------------------

float patch_slip_angle(float vx, float vy, float& fade) {
    float speed = std::sqrt(vx * vx + vy * vy);
    const float lo = 1.0f / 3.6f, hi = 10.0f / 3.6f;
    fade = speed <= lo ? 0.0f : speed >= hi ? 1.0f : (speed - lo) / (hi - lo);
    return fade > 0 ? std::atan2(-vy, vx) : 0.0f;
}

float slip_gain(float u);

float slip_ratio(float x, float w, float u) {
    const float eps = 1.0f / 57.6f;
    auto gain = [&] { return slip_gain(u); };
    float raw;
    int state;
    if (std::fabs(w) < eps) {
        state = -1;
        raw = x != w ? 1.0f : 0.0f;
    } else if (w > 0) { // outside the dead band the branch follows the sign of w
        if (x <= w) { raw = (x - w) * gain() / w; state = 1; }
        else { raw = std::fabs(x) < eps ? 0.0f : (x - w) * gain() / x; state = -1; }
    } else {
        if (x >= w) { raw = (x - w) * gain() / w; state = 1; }
        else { raw = (x - w) * gain() / x; state = -1; }
    }
    raw = std::clamp(raw, -1.0f, 1.0f);
    float r = state < 0 ? x : w;
    const float v0 = 20 / 3.6f; // byte x (1/3.6): 20 km/h
    if (std::fabs(r) < v0) raw *= std::fabs(r) / v0;
    return raw;
}

float tire_peak_force(const TireSpec& t, float fz, int surface, float cam) {
    if (fz <= 0) return 0;
    int si = std::clamp(surface, 0, 15);
    bool table = false;
    for (float m : t.surface_mu) table = table || m != 0;
    float mu = table ? t.surface_mu[size_t(si)] : t.mu * surface_mu(si);
    float l = 1.0f;
    if (!t.load.x.empty()) l = lookup(t.load, t.width > 0 ? fz / t.width : fz / std::max(t.fz_ref, 1.0f));
    return fz * l * t.wear_grip * mu * cam;
}

TireOut tire_force(const TireSpec& t, const TireIn& in) {
    TireOut out;
    if (in.peak <= 0) return out;
    // Folded slip angle (physics §3.2): raw = atan2(-vy, vx); +pi when rolling backwards.
    float speed = std::sqrt(in.vx * in.vx + in.vy * in.vy);
    float u1 = 0, u2 = 0;
    if (speed > 1e-4f) {
        float beta = std::atan2(-in.vy, std::fabs(in.vx)); // (-pi/2, pi/2): backwards rolling folded
        float v = std::fabs(beta);
        float sg = beta >= 0 ? 1.0f : -1.0f;
        u1 = sg * lookup(t.slip_a, v) * in.scale;
        u2 = lookup(t.slip_b, v) * in.scale;
        float lim = t.limit_d.x.empty() ? 1.0f : lookup(t.limit_d, in.slip_ratio);
        float mag = std::sqrt(u1 * u1 + u2 * u2);
        out.grip_used = lim > 0 ? mag / lim : 0;
        if (mag > lim && mag > 0) {
            u1 *= lim / mag;
            u2 *= lim / mag;
        }
        out.slip_angle = sg * v;
        // F1 perpendicular to the patch velocity, against the slide; F2 along
        // it, against the motion. In the wheel frame (x heading, y right):
        float dx = in.vx / speed, dy = in.vy / speed;
        // Perpendicular that opposes the lateral slide: u1 > 0 when beta > 0, i.e. vy < 0
        // (sliding left), so F1 pushes right = (-dy, dx) rotated towards +y when moving forwards.
        float px = -dy, py = dx;
        if (in.vx < 0) { px = dy; py = -dx; }
        float f1 = u1 * in.peak, f2 = u2 * in.peak;
        out.fx += px * f1 - dx * f2;
        out.fy += py * f1 - dy * f2;
    }
    // Longitudinal (physics §3.3): Fx = -sign(s) C(|s|) F_peak along the heading.
    float s = in.slip_ratio;
    if (s != 0) {
        float dir = in.rel != 0 ? (in.rel > 0 ? -1.0f : 1.0f) : (s > 0 ? -1.0f : 1.0f);
        out.longitudinal = dir * lookup(t.long_c, s) * in.peak;
        out.fx += out.longitudinal;
    }
    return out;
}

// --- suspension / aero / steering ---------------------------------------------------

float suspension_force(const AxleSpec& a, float x, float v) {
    if (x <= 0) return 0;
    // Spec physics §4.2 structure: compression saturates at the bump-stop limit, a
    // progressive term k2 (x - knee)^2 past the knee, damper velocity clamped to
    // +-1 m/s. Rates/curves are placeholders (linear damper).
    float limit = a.travel + 0.05f;
    float xc = std::min(x, limit);
    float f = a.spring_rate * xc;
    if (xc > a.travel) f += 1.568e6f * (xc - a.travel) * (xc - a.travel);
    f += a.damper * std::clamp(v, -1.0f, 1.0f);
    return std::max(f, 0.0f);
}

SuspensionSpec::Damper fit_damper(float v1, float f1, float v2, float f2) {
    SuspensionSpec::Damper d;
    if (!(v1 > 0 && v2 > v1 && f1 > 0 && f2 > f1)) return d;
    // r(u) = (sqrt(v1 + u^2) - u) / (sqrt(v2 + u^2) - u), u = sqrt(-t), runs
    // monotonically from sqrt(v1/v2) (u = 0) to v1/v2 (u -> inf); bisection on u.
    float target = f1 / f2;
    auto r = [&](double u) { return (std::sqrt(v1 + u * u) - u) / (std::sqrt(v2 + u * u) - u); };
    double lo = 0, hi = 1e3; // larger |t| (near-linear points) counts as invalid -> zero damper
    double rlo = r(lo), rhi = r(hi);
    if (!((target - rlo) * (target - rhi) <= 0)) return d;
    for (int i = 0; i < 100; ++i) {
        double mid = 0.5 * (lo + hi);
        if ((r(mid) - target) * (rlo - target) > 0) {
            lo = mid;
            rlo = r(mid);
        } else {
            hi = mid;
        }
    }
    double u = 0.5 * (lo + hi);
    double g = f2 / (std::sqrt(v2 + u * u) - u);
    d.t = float(-u * u);
    d.g = float(g);
    d.o = float(-g * u);
    return d;
}

float damper_force(const SuspensionSpec& s, float v) {
    v = std::clamp(v, -1.0f, 1.0f);
    if (v < 0) return s.bump.g > 0 ? s.bump.o + s.bump.g * std::sqrt(-v - s.bump.t) : 0.0f;
    return s.rebound.g > 0 ? -(s.rebound.o + s.rebound.g * std::sqrt(v - s.rebound.t)) : 0.0f;
}

SuspensionLimits suspension_limits(const SuspensionSpec& s, float xs) {
    SuspensionLimits l;
    float preload = s.unsprung * 9.8f;
    l.droop = std::min(xs + s.droop, s.k > 0 ? preload / s.k : 0.0f);
    float q = s.ride_height;
    l.limit = xs - q + std::min(s.bump_room, q);
    l.knee = std::max(xs - s.knee_gap, l.limit);
    return l;
}

float static_position(const SuspensionSpec& s, float mass, float share, float unsprung_front, float unsprung_rear) {
    if (s.k <= 0) return 0;
    const double sprung = double(mass) - 2.0 * (double(unsprung_front) + unsprung_rear);
    return float(-0.5 * sprung * share * kGravity / s.k);
}

SuspensionOut suspension_wheel(const SuspensionSpec& s, float x, float v, float xs) {
    SuspensionOut o;
    float preload = s.unsprung * 9.8f;
    const SuspensionLimits lim = suspension_limits(s, xs);
    float droop = lim.droop;
    if (x > droop) { // wheel in the air [physics §10.5]: x_prev := droop, spring force -preload (the body feels it)
        o.x = droop;
        o.spring = -preload;
        return o;
    }
    float limit = lim.limit;
    float knee = lim.knee;
    o.grounded = true;
    o.x = std::max(x, limit);
    o.spring = -s.k * o.x + (o.x < knee ? s.k2 * (o.x - knee) * (o.x - knee) : 0.0f);
    o.damper = damper_force(s, v);
    if (o.x < knee && v < 0) o.damper += s.bump_damping * std::fabs(std::clamp(v, -1.0f, 1.0f));
    o.load = std::max(0.0f, o.spring + o.damper + preload);
    return o;
}

float anti_roll(const SuspensionSpec& s, float xl, float xr) {
    float d = xl - xr; // < 0: left more compressed
    return -s.arb_gain * d * std::fabs(d);
}

float tcs_lateral(float x, float a, float b) {
    float ax = std::fabs(x);
    return ax <= b ? 1.0f : std::max(0.0f, 1.0f - (ax - b) * a);
}

float tcs_factor(float gain, float g, float s_min, float s_opt, float lat, float scale) {
    if (gain <= 0) return 1;
    return std::clamp(1.0f + gain * g * (s_min - s_opt * lat * scale), 0.0f, 1.0f);
}

float understeer_indicator(float v, float steer, float L, float r, float yaw_acc, float lat_ratio) {
    if (v <= 0 || L <= 0) return 0;
    if (lat_ratio <= 0.2f) return 0;           // lateral force not significant
    if (r * steer < 0) return 0;               // yawing against the steer
    if (yaw_acc * r > 0) r += 0.5f * yaw_acc;  // + half the yaw acceleration when they share a sign
    float rt = v * std::sin(std::fabs(steer)) / L;
    float ra = std::fabs(r);
    if (rt <= 0 || ra >= rt) return 0;
    return std::clamp(1.0f - ra / rt, 0.0f, 1.0f);
}

float oversteer_alt(float s, float r, float lat, float tyre_sum) {
    if (tyre_sum <= 0 || s * r * lat >= 0) return 0;
    return 2.0f * std::max(0.0f, std::fabs(s * r) - std::fabs(lat)) / tyre_sum;
}

float oversteer_indicator(float v, float m) {
    if (v <= 0) return 0;
    const float d2r = kPiF / 180.0f;
    float q = std::min(1.0f, 0.018f * v);
    float th = (20.0f + (5.0f - 20.0f) * q) * d2r;
    float g = 3.0f + (14.0f - 3.0f) * q;
    return std::clamp(g * (std::fabs(m) - th) / (90.0f * d2r - th), 0.0f, 1.0f);
}

float abs_pressure(float p, float f_peak, float radius, float brake_max, float gain, float slip_gain, float slip,
                   float slip_opt) {
    if (p <= 0) return 0;
    float e = brake_max > 0 ? radius / brake_max : 0;
    float k = f_peak * e * (1 - gain * slip_gain * (slip - slip_opt));
    return k > 0 ? p * std::min(k, 1.0f) : 0.001f; // a non-positive factor gives 0.001, not 0
}

float slipstream_factor(float x, float y, float v_leader, float h) {
    if (v_leader > 27.78f && x > -v_leader && x < 0 && y > -h && y < h) return (v_leader + x) / v_leader;
    return 0;
}

AeroOut aero_step(const AeroSpec& a, float v, float target, float prev, float dt) {
    AeroOut o;
    float y = prev;
    if (target != 0) y = std::min(1.0f, y + target * a.slip_k_up * dt);
    else y = std::max(0.0f, y - a.slip_k_down * dt);
    o.slip = y;
    float v2 = v * v;
    float sgn = v > 0 ? 1.0f : v < 0 ? -1.0f : 0.0f;
    o.drag = -sgn * a.c_drag * (a.slip_a + (1 - a.slip_a) * (1 - y)) * v2;
    float lift = (a.slip_b + (1 - a.slip_b) * (1 - y)) * v2;
    o.lift_front = a.c_front * lift;
    o.lift_rear = a.c_rear * lift;
    return o;
}

float rolling_curve(float speed) {
    // speed axis in exact km/h / 3.6 (0, 4, 20, 58, 116, 200, 308, 400 km/h)
    static const float x[] = {0, 4 / 3.6f, 20 / 3.6f, 58 / 3.6f, 116 / 3.6f, 200 / 3.6f, 308 / 3.6f, 400 / 3.6f};
    static const float y[] = {0, 0, 0.010f, 0.015f, 0.032f, 0.072f, 0.144f, 0.255f};
    return table_lookup(x, y, 8, std::fabs(speed));
}

PitchTerms pitch_terms(const float cd[4], const float fx[4], const float width[2], float y, float m, float a, float b) {
    const float tp = kTwoPiF;
    float kf = 0.5f * tp / (y + m * a * a), kr = 0.5f * tp / (y + m * b * b);
    float q[4];
    for (int i = 0; i < 4; ++i) q[i] = cd[i] * fx[i] * width[i / 2];
    PitchTerms t;
    t.a_term = (q[0] + q[1]) * kf + (q[2] + q[3]) * kr;
    t.b_term = (a > 0 ? (q[0] + q[1]) / a : 0.0f) - (b > 0 ? (q[2] + q[3]) / b : 0.0f);
    return t;
}

float attitude_step(const float f[4], const float d[4], float lf, float lr, float rf, float rr, PitchTerms pt, float dt,
                    float& pitch, float& prate, float& roll, float& rrate) {
    const float half_pi = 1.5707963f, quarter_pi = 0.78539816f;
    float pacc = ((f[0] + f[1]) + d[0] + d[1]) * lf - ((f[2] + f[3]) + d[2] + d[3]) * lr + pt.a_term;
    prate = std::clamp(prate + pacc * dt, -half_pi, half_pi);
    pitch += prate * dt;
    if (std::fabs(pitch) > quarter_pi) {
        pitch = std::copysign(quarter_pi, pitch);
        prate = 0;
    }
    float racc = ((f[0] - f[1]) + d[0] - d[1]) * rf + ((f[2] - f[3]) + d[2] - d[3]) * rr;
    rrate = std::clamp(rrate + racc * dt, -half_pi, half_pi);
    roll += rrate * dt;
    if (std::fabs(roll) > quarter_pi) {
        roll = std::copysign(quarter_pi, roll);
        rrate = 0;
    }
    return f[0] + f[1] + f[2] + f[3] + d[0] + d[1] + d[2] + d[3] + pt.b_term;
}

GroundFrame ground_frame(const float g[4], float w, float wheelbase, float mean_track) {
    GroundFrame f;
    float u = g[0] + w * (g[2] - g[0]), v = g[1] + w * (g[3] - g[1]);
    f.height = 0.5f * (u + v);
    f.pitch = std::atan2(0.5f * ((g[0] + g[1]) - (g[2] + g[3])), wheelbase);
    f.roll = std::atan2(u - v, mean_track);
    return f;
}

float spawn_height_offset(int n) {
    if (n < 30) return 0;
    if (n < 270) return 0.3f;
    return (300 - n) * 0.01f;
}

bool bump_nudge(const float p[4], float w, float mean_track, float wheelbase, Nudge& out) {
    out = {};
    if (p[0] <= 0 && p[1] <= 0 && p[2] <= 0 && p[3] <= 0) return false;
    float l = p[0] + w * (p[2] - p[0]), r = p[1] + w * (p[3] - p[1]);
    float fm = 0.5f * (p[0] + p[1]), rm = 0.5f * (p[2] + p[3]);
    if (l != r) out.roll = std::atan2(l - r, mean_track);
    if (fm != rm) out.pitch = std::atan2(fm - rm, wheelbase);
    out.height = 0.5f * std::min(l, r) + 0.5f * std::min(fm, rm);
    return true;
}

float bump_limit(const SuspensionSpec& s, float xs) {
    return xs - s.ride_height + std::min(s.bump_room, s.ride_height);
}

void ackermann_ratios(float L, float tf, float& inner, float& outer) {
    const float quarter_pi = 0.78539816f;
    inner = std::atan2(L, L - tf) / quarter_pi;
    outer = std::atan2(L, L + tf) / quarter_pi;
}

float toe_angle(int byte, float r) { return std::atan2((byte - 128) * 0.0005f, r); }

float steer_target(const SteeringSpec& s, float input, float speed, float velocity_angle) {
    float v = std::max(speed, 0.0f);
    // Pad curve clamped to the car's full lock [physics §4.4].
    float lock = s.pad_curve.x.empty() ? s.max_angle / (1.0f + v / std::max(s.speed_reduction, 0.1f))
                                       : std::min(lookup(s.pad_curve, v), s.max_angle);
    // [physics §4.4] the bound on the side of the mean front velocity-direction angle is extended to
    // it, then both are clamped to the full lock; the input scales the bound on its own side
    float pos = std::min(std::max(lock, velocity_angle), s.max_angle);
    float neg = std::min(std::max(lock, -velocity_angle), s.max_angle);
    float x = std::clamp(input, -1.0f, 1.0f);
    return x * (x >= 0 ? pos : neg);
}

float pedal_shape(float x) { return x > 0 ? x + 0.1f * (1 - x) : x; }

void steer_follow(const SteeringSpec& s, float target, float& angle, float& rate, float dt, bool digital) {
    // Moving back towards zero (including a target across zero): centring term
    // and the separate return rate limit [physics §4.4].
    bool back = (angle > 0 && target < angle) || (angle < 0 && target > angle);
    float acc = s.follow_k1 * (target - angle) - s.follow_k2 * rate;
    if (back) acc -= s.follow_k3 * angle;
    float wmax = back ? s.follow_return_rate_max : digital ? s.follow_rate_digital : s.follow_rate_max;
    rate = std::clamp(rate + acc * dt, -wmax, wmax);
    float next = angle + rate * dt;
    if ((angle - target) * (next - target) <= 0) {
        next = target;
        rate = 0;
    }
    angle = next;
}

void ackermann(const SteeringSpec& s, float angle, float wheelbase, float track, float& left, float& right) {
    left = right = angle;
    if (std::fabs(angle) < 1e-5f || s.ackermann <= 0) return;
    float r = wheelbase / std::tan(std::fabs(angle));
    float inner = std::atan(wheelbase / std::max(r - track * 0.5f, 0.05f));
    float outer = std::atan(wheelbase / (r + track * 0.5f));
    float a_in = std::fabs(angle) + (inner - std::fabs(angle)) * s.ackermann;
    float a_out = std::fabs(angle) + (outer - std::fabs(angle)) * s.ackermann;
    if (angle > 0) {
        right = a_in;
        left = a_out;
    } else {
        left = -a_in;
        right = -a_out;
    }
}

// --- differential --------------------------------------------------------------------

float lsd_clutch_capacity(int initial, int accel, int decel, float t) {
    float ratio = (t >= 0 ? accel : decel) * 0.01f;
    return std::max(initial * 9.8f, ratio * std::fabs(t));
}

float lsd_lock(int type, int initial, int accel, int decel, float t, float dw, float sync) {
    switch (type) {
    case 1: return sync; // r: locked (wheels at the axle base)
    case 3:
    case 4: { // m, h: slipping clutch pack transfers exactly C from the faster wheel
        float c = lsd_clutch_capacity(initial, accel, decel, t);
        return dw > 0 ? c : dw < 0 ? -c : 0.0f;
    }
    case 5: { // v: viscous, gain x min(dw^2, cap) x sign
        float cap = float(initial * initial);
        return std::copysign(accel * 0.98f * std::min(dw * dw, cap), dw);
    }
    default: return 0; // f, n, y, t: open
    }
}

void diff_step(int type, int initial, int accel, int decel, float t, float inertia, float dt, float acc_l, float acc_r,
               float& wl, float& wr, DiffState& st) {
    double I = std::max(double(inertia), 1e-3);
    double dw = (double(wl) - wr) - (double(acc_l) - acc_r) * dt / I; // previous speeds, reconstructed
    if (std::fabs(dw) <= 2e-6 * (std::fabs(double(wl)) + std::fabs(double(wr)))) dw = 0; // float reconstruction noise
    diff_step_prev(type, initial, accel, decel, t, inertia, dt, acc_l, acc_r, float(dw), wl, wr, st);
}

void diff_step_prev(int type, int initial, int accel, int decel, float t, float inertia, float dt, float acc_l,
                    float acc_r, float dw_prev, float& wl, float& wr, DiffState& st, float brake_l, float brake_r, float wl_prev,
                    float wr_prev) {
    // Behaviour [physics §2.7], all quantities before the step:
    // dw = previous wheel-speed difference; clutch packs stay locked while flagged
    // and |accL - accR| <= 2C, else transfer C (s = sign dw); viscous transfers
    // gain min(dw^2, cap); then d = dw/2 + (accL - accR) dt / (2 I); a sign change
    // against dw gives d = 0 (and sets the clutch-pack flag).
    if (type <= 0 || type > 7) { // f (and unknown): wheels integrate alone
        // [physics §2.7] with brake torque on a wheel, a zero crossing (or an exact 0) clamps it to 0
        double Ii = std::max(double(inertia), 1e-3);
        auto clamp_wheel = [&](float& w, float acc, float brake, float given) {
            if (brake == 0) return;
            float prev = std::isfinite(given) ? given : float(double(w) - double(acc) * dt / Ii);
            if (prev == 0 || (prev > 0) != (w > 0) || w == 0) w = 0;
        };
        clamp_wheel(wl, acc_l, brake_l, wl_prev);
        clamp_wheel(wr, acc_r, brake_r, wr_prev);
        st.locked = false;
        st.half = 0.5f * (wl - wr);
        return;
    }
    double I = std::max(double(inertia), 1e-3), h = dt;
    double base = 0.5 * (double(wl) + wr);
    double dw = dw_prev;
    if (type == 1) { // r
        wl = wr = float(base);
        st.half = 0;
        return;
    }
    double al = acc_l, ar = acc_r;
    double sg = dw > 0 ? 1 : dw < 0 ? -1 : 0;
    bool locked = false;
    if (type == 3 || type == 4) {
        double c = lsd_clutch_capacity(initial, accel, decel, t);
        if (st.locked && std::fabs(al - ar) <= 2 * c) {
            locked = true;
        } else {
            st.locked = false;
            al -= sg * c;
            ar += sg * c;
        }
    } else if (type == 5) {
        double x = double(accel) * 0.98 * std::min(dw * dw, double(initial) * initial);
        al -= sg * x;
        ar += sg * x;
    }
    double d = locked ? 0.0 : dw * 0.5 + (al - ar) / I * 0.5 * h;
    if (dw * d < 0) {
        d = 0;
        if (type == 3 || type == 4) st.locked = true;
    }
    st.half = float(d);
    wl = float(base + d);
    wr = float(base - d);
}

int driveline_class(int drive, int awd) {
    if (drive == 1) return 1;
    if (drive == 2) return awd == 0 ? 1 : awd == 3 ? 3 : 2;
    return 0;
}

float low_speed_hold(float peak, float w, float u_h, float v_h_body, float cos_d, float sin_d) {
    float v_h = v_h_body * cos_d + u_h * sin_d;
    if (peak == 0 || std::fabs(w) > 1.38889f || std::fabs(u_h - w) > 0.555556f || std::fabs(v_h) > 1.11111f) return 0;
    return peak * std::sin(2.82743f * v_h) * std::cos(1.13097f * w);
}

// ---- tyre wear [physics §8] ----

WearParams wear_params(const u8 e[7], bool enabled) {
    WearParams p;
    if (!enabled) return p;
    // block bytes: 0x2C max, 0x2D loss max, 0x2E worn, 0x2F fresh, 0x30 loss fresh,
    // 0x31 = 4, 0x32 = 0 (forced), 0x33 knee 2, 0x34 loss 2
    int b2c = e[0], b2d = e[1], b2e = e[2], b2f = e[3], b30 = e[4], b31 = 4, b32 = 0, b33 = e[5], b34 = e[6];
    if (b2c == 0) return p;
    if (!(b33 < b2c && b34 < b2d && b31 < b33 && b32 < b34 && b2e < b2d)) return p;
    p.fresh = -float(b2f) * 10000;
    p.loss_fresh = float(b30) * 0.01f;
    p.knee1 = float(b31) * 10000;
    p.loss1 = float(b32) * 0.01f;
    p.knee2 = float(b33) * 10000;
    p.loss2 = float(b34) * 0.01f;
    p.max = float(b2c) * 10000;
    p.loss_max = float(b2d) * 0.01f;
    p.worn = float(100 - b2e) * 0.01f;
    p.slip_cap = 1.0f; // static byte 100 / 100
    return p;
}

WearParams shipped_wear_params() {
    const u8 b[7] = {21, 30, 9, 2, 6, 15, 5};
    return wear_params(b, true);
}

float wear_grip(const WearParams& p, float& w, float floor, int* level) {
    double g;
    int lv;
    if (w >= p.max) {
        w = p.max;
        g = 1 - p.loss_max;
        lv = 127;
    } else if (w >= p.knee2) {
        double f = (w - p.knee2) / (p.max - p.knee2);
        g = (1.0 - p.loss2) + ((1.0 - p.loss_max) - (1.0 - p.loss2)) * f;
        lv = 64 + int(63 * f);
    } else if (w >= p.knee1) {
        double f = (w - p.knee1) / (p.knee2 - p.knee1);
        g = (1.0 - p.loss1) + ((1.0 - p.loss2) - (1.0 - p.loss1)) * f;
        lv = 32 + int(32 * f);
    } else if (w >= 0) {
        g = 1 - p.loss1 * w / p.knee1;
        lv = int(31 * w / p.knee1);
    } else if (w >= p.fresh) {
        g = 1 - p.loss_fresh * w / p.fresh;
        lv = -int(float(w / p.fresh) * 63.9f); // truncation toward zero (cvt.w.s)
    } else {
        g = 1;
        lv = -63;
    }
    if (level) *level = lv;
    return std::max(float(g), floor);
}

namespace {
struct Pt {
    float x, y;
};
float pw_lookup(const Pt* p, int n, float x) {
    if (x <= p[0].x) return p[0].y;
    for (int i = 1; i < n; ++i)
        if (x <= p[i].x) return p[i - 1].y + (p[i].y - p[i - 1].y) * (x - p[i - 1].x) / (p[i].x - p[i - 1].x);
    return p[n - 1].y;
}
struct SmokeClass {
    Pt lat[4];
    Pt slip[8];
    float r0, g, b;
};
const SmokeClass kSmoke[3] = {
    {{{1.111f, 0}, {9.556f, 1.192f}, {12.22f, 1.6f}, {15.0f, 2.0f}},
     {{-1, 2.388f}, {-0.580f, 2.388f}, {-0.314f, 1.176f}, {-0.0353f, 0}, {0.0353f, 0}, {0.314f, 0.784f}, {0.580f, 1.592f}, {1, 1.592f}},
     0.01667f, 0, 0},
    {{{1.556f, 0}, {3.333f, 0.0627f}, {13.11f, 0.3639f}, {14.11f, 0.4f}},
     {{-0.502f, 0.2f}, {-0.439f, 0.1757f}, {-0.1255f, 0.0251f}, {-0.0471f, 0}, {0.0471f, 0}, {0.1098f, 0.0408f}, {0.2353f, 0.1757f},
      {0.2667f, 0.2f}},
     0.005556f, 0.5455f, 0.07f},
    {{{1.556f, 0}, {3.333f, 0.0471f}, {13.11f, 0.2729f}, {14.11f, 0.3f}},
     {{-0.502f, 0.4f}, {-0.439f, 0.3514f}, {-0.1255f, 0.0502f}, {-0.0471f, 0}, {0.0471f, 0}, {0.1098f, 0.0612f}, {0.2353f, 0.2635f},
      {0.2667f, 0.3f}},
     0.05556f, 0.24f, 0},
};
float clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }
} // namespace

float slip_intensity(int surface, float lat, float slip, float rim_speed, float fz, float ramp) {
    if (surface == 1 || surface == 13)
        return clamp01((std::fabs(rim_speed) - 1.389f) * 0.144f) * clamp01(fz / 2450 - 0.25f) * 0.5f;
    int cls = (surface >= 2 && surface <= 5) ? 1 : (surface == 6 || (surface >= 12 && surface <= 14)) ? 2 : 0;
    const SmokeClass& c = kSmoke[cls];
    float a = pw_lookup(c.lat, 4, std::fabs(lat)), b = pw_lookup(c.slip, 8, slip);
    float m = std::sqrt(a * a + b * b);
    if (ramp > c.r0) m += (ramp - c.r0) * c.g;
    return m != 0 ? m + c.b : 0.0f;
}

namespace {
// [physics §8.3] Second curve set (the skid pitch): lateral speed and slip ratio, like the first.
const SmokeClass kSkidPitch[3] = {
    {{{0.444444478f, 0}, {3.33333349f, 0.109803922f}, {6.0f, 0.227450982f}, {9.22222233f, 0.350980401f}},
     {{-1, 0.461176455f}, {-0.65882355f, 0.288627446f}, {-0.250980407f, 0.112941168f}, {-0.0156862754f, 0}, {0, 0},
      {0.203921571f, 0.087843135f}, {0.721568644f, 0.326274484f}, {1, 0.461176455f}},
     0.0166666657f, 0, 0.74999994f},
    {{{1.11111116f, 0}, {6.88888931f, 0.470588237f}, {18.0f, 1.53725493f}, {28.333334f, 1.59215689f}},
     {{-0.345098048f, 0.799999952f}, {-0.298039228f, 0.70274508f}, {-0.109803922f, 0.100392155f}, {-0.0470588244f, 0},
      {0.0470588244f, 0}, {0.0784313753f, 0.163137242f}, {0.188235298f, 0.70274508f}, {0.282352954f, 0.799999952f}},
     0.0555555522f, 0.200000003f, 0.49999997f},
    {{{1.55555558f, 0}, {5.11111116f, 0.150588229f}, {11.7777786f, 0.301176459f}, {28.333334f, 0.293333322f}},
     {{-0.501960814f, 0.299999982f}, {-0.43921569f, 0.26352939f}, {-0.125490203f, 0.0376470573f}, {-0.0470588244f, 0},
      {0.0470588244f, 0}, {0.109803922f, 0.0611764677f}, {0.235294119f, 0.26352939f}, {0.266666681f, 0.299999982f}},
     0.166666642f, 0.385714293f, 0.49999997f},
};
} // namespace

float skid_pitch(int surface, float lat, float slip, float rim_speed, float ramp) {
    if (surface == 1 || surface == 13) {
        // Kerbs: a ramp on the rim speed (2.78 m/s -> 0.5, 55.6 m/s -> 4.3).
        constexpr float x0 = 2.77777791f, y0 = 0.5f, x1 = 55.5555573f, y1 = 4.29999971f;
        float v = std::fabs(rim_speed);
        if (v <= x0) return y0;
        if (x1 <= v) return y1;
        return y0 + (v - x0) * (y1 - y0) / (x1 - x0);
    }
    int cls = (surface >= 2 && surface <= 5) ? 1 : (surface == 6 || (surface >= 12 && surface <= 14)) ? 2 : 0;
    const SmokeClass& c = kSkidPitch[cls];
    float a = pw_lookup(c.lat, 4, std::fabs(lat)), b = pw_lookup(c.slip, 8, slip);
    float p = c.b + std::sqrt(a * a + b * b);
    if (ramp > c.r0) p += (ramp - c.r0) * c.g;
    return p;
}

namespace {
// [physics §8.3b] Third curve set (smoke emission): lateral speed and signed slip speed (m/s).
const SmokeClass kSmokeEmit[3] = {
    {{{11.555556f, 0}, {12.888889f, 0.04705882f}, {13.777778f, 0.1505882f}, {16.333333f, 0.177647f}},
     {{-10.0f, 0.1f}, {-8.888889f, 0.08784313f}, {-8.0f, 0.01254902f}, {-7.555556f, 0}, {8.0f, 0}, {8.444445f, 0.02039216f},
      {9.333334f, 0.08784313f}, {10.444444f, 0.1f}},
     0, 0, 0.3f},
    {{{3.111111f, 0}, {6.666667f, 0.07843138f}, {26.222222f, 0.454902f}, {28.333333f, 0.5f}},
     {{-7.111112f, 0.5f}, {-6.222222f, 0.4392157f}, {-1.777778f, 0.0627451f}, {-0.6666667f, 0}, {0.6666667f, 0}, {1.555556f, 0.04078431f},
      {3.333333f, 0.1756863f}, {3.777778f, 0.2f}},
     0.005555555f, 0.7272727f, 0},
    {{{5.333333f, 0}, {9.777778f, 0.01568627f}, {26.222222f, 0.09098039f}, {28.333333f, 0.1f}},
     {{-9.111112f, 0.1f}, {-7.555556f, 0.08784313f}, {-6.0f, 0.01254902f}, {-4.666667f, 0}, {3.111111f, 0}, {4.222222f, 0.02039216f},
      {5.555556f, 0.09098039f}, {7.777778f, 0.1f}},
     0.05555555f, 0.2f, 0.2f},
};
} // namespace

float smoke_emission(int surface, float lat, float slip_speed, float ramp) {
    int cls = (surface >= 2 && surface <= 5) ? 1 : (surface == 6 || (surface >= 12 && surface <= 14)) ? 2 : 0;
    const SmokeClass& c = kSmokeEmit[cls];
    float a = pw_lookup(c.lat, 4, std::fabs(lat)), b = pw_lookup(c.slip, 8, slip_speed);
    float m = std::sqrt(a * a + b * b);
    if (ramp > c.r0) m += (ramp - c.r0) * c.g;
    return m != 0 ? std::min(1.0f, m + c.b) : 0.0f;
}

float wear_increment(float smoke, float slip_cap, float rate, float fz, float width, float dt) {
    if (fz == 0 || width <= 0) return 0;
    float u = std::max(0.0f, 0.4f * fz / width - 1960.0f);
    float sm = std::min(slip_cap, smoke);
    return ((sm * rate) * u) * dt;
}

float brake_heat_step(float y, float brake_torque, float radius, float pedal, float rim_speed, float dt) {
    float tb = brake_torque * radius, v = std::fabs(rim_speed);
    y += dt * (tb * pedal * v * (1.0f / 68600.0f) - (v + 27.78f) * 0.005f * (y + 0.05f));
    return std::clamp(y, 0.0f, 1.0f);
}

void flap_step(float& x, float& v, float a_x, float u, float fz, float rim_speed, bool rough, float dt,
               Vu0Random& rng) {
    float a = a_x * 5.0f;
    const float room = u >= 0 ? 1.0f - x : x + 1.0f;
    if (rng.next() * room < 0.02f) a += (1.0f - x) * 5.0f * u;
    if (rough && u > 0 && fz > 0) {
        const float left = 1.0f - x;
        if (rng.next() * left < 0.04f) a += left * 20.0f * rim_speed;
    }
    a -= x * 100.0f;
    a -= v * 5.0f;
    v += a * dt;
    x += v * dt;
    if (x > 1.0f) x = 1.0f, v = 0;
    else if (x < -1.0f) x = -1.0f, v = 0;
}


// ---- axle driveline [physics §2.7] ----

AwdLayout awd_layout(int drive, int sys, int b4, int b5) {
    AwdLayout l;
    if (drive == 1) {
        l.layout = 1;
        l.driven_axle = 0;
        return l;
    }
    if (drive != 2) return l; // FR, MR, RR
    switch (sys) {
    case 0: l = {2, 0, b4 * 0.01f, 0}; break;
    case 1: l = {3, 1, b4 * 0.01f, 0}; break;
    case 2: l = {4, 1, b4 >= 50 ? 1.0f : b4 * 0.01f, float(b5 ? b5 : 8) * 98}; break;
    case 3: l = {5, 1, b4 * 0.01f * 0.36f, float(b5 ? b5 : 15) * 98}; break;
    case 4: l = {6, 1, b4 ? 1.0f / (b4 * kPiF / 180) : 0.0f, float(b5 ? b5 : 15) * 98}; break;
    case 7: l = {7, 1, b4 * 0.01f, 0}; break;
    default: l = {0, 1, 0, 0}; break; // 5, 6 (and unknown): plain rear drive
    }
    return l;
}

AwdQ awd5_gain(const AwdIn& in) {
    float a = std::fabs(in.g_over_mu);
    float f = a <= 0.25f ? 1.0f : a >= 1 ? 0.25f : 1.0f - (a - 0.25f) / 0.75f * 0.75f;
    if (in.clutch == 0 || (in.reverse && in.clutch == 1)) return {true, 0};
    bool same_sign = (in.tg > 0 && in.tg + in.fr > 0) || (in.tg < 0 && in.tg + in.fr < 0); // 0 has no sign
    if (f == 1 && (in.clutch == 2 || in.brake > 0.75f || same_sign)) return {true, 0};
    float u = std::fabs(in.wr - in.wf) * in.k * f;
    return u < 1 ? AwdQ{false, u} : AwdQ{true, 0};
}

AwdQ awd_front_torque(const AwdIn& in) {
    switch (in.layout) {
    case 4:
        if (in.k == 1) return {true, 0};
        if (in.handbrake > 0.2f) return {false, 0};
        if (in.tg <= 0 || (in.reverse && in.clutch == 1)) return {true, 0};
        return {false, in.k * in.tg};
    case 5: {
        AwdQ g = awd5_gain(in);
        return g.full ? g : AwdQ{false, 0.5f * g.q * in.tg};
    }
    case 6: {
        if (in.handbrake > 0.2f) return {false, 0};
        if (in.reverse && in.clutch == 1) return {true, 0};
        float g = std::max(0.0f, 1 - std::fabs(in.steer) * in.k);
        return g >= 0.99f ? AwdQ{true, 0} : AwdQ{false, 0.5f * g * in.tg};
    }
    case 7: return {false, in.k * in.tg};
    default: return {false, 0};
    }
}

float awd_centre_clutch(const AwdIn& in, const AwdQ& q, float capacity, bool& engaged) {
    // Engaged: locked when Q is full, else while |(Ff - Fr) - T_g| <= capacity (else the flag
    // clears) [physics §2.7].
    if (engaged) {
        if (q.full || std::fabs((in.ff - in.fr) - in.tg) <= capacity) return 0; // locked: common speed
        engaged = false;
    }
    if (!q.full) return q.q;
    return in.wr >= in.wf ? capacity : -capacity;
}

void torque_vectoring(int type, int initial, float steer_left, float speed, float wl, float wr, float& tl, float& tr) {
    if ((type != 6 && type != 7) || speed < 10 / 3.6f || speed > 180 / 3.6f) return; // 10..180 km/h
    float t = std::min(std::fabs(steer_left) / (15 * kPiF / 180), 1.0f) * initial * (type == 6 ? 9.8f : 0.01f);
    float mean = 0.5f * (wl + wr);
    float theta = steer_left > 0 ? 1.125f * mean : steer_left < 0 ? 0.875f * mean : mean;
    if (wr < theta) {
        tr += 0.4375f * t;
        tl -= 0.5625f * t;
    } else if (wr > theta) { // the odd wheel loses 0.5625 T, the even gains 0.4375 T
        tr -= 0.5625f * t;
        tl += 0.4375f * t;
    }
}


AxleStep axle_drive(const AwdLayout& lay, const AwdIn& in, bool& engaged, float jf, float jr, float jref, float& jd_f,
                    float& jd_r) {
    AxleStep o;
    jd_f = jf;
    jd_r = jr;
    double tg = in.tg;
    switch (lay.layout) {
    case 0: o.drive_r = float(tg); jd_r += jref; break;
    case 1: o.drive_f = float(tg); jd_f += jref; break;
    case 2:
        o.drive_r = lay.k * (in.wf - in.wr);
        o.drive_f = float(tg - o.drive_r);
        jd_f += jref;
        break;
    case 3: o.drive_f = o.drive_r = float(0.5 * tg); break;
    default: {
        AwdQ q = awd_front_torque(in);
        float qf = awd_centre_clutch(in, q, lay.capacity, engaged);
        if (lay.layout <= 6 && engaged) {
            o.common = true;
            o.drive_f = o.drive_r = float(0.5 * tg);
        } else {
            o.drive_f = qf;
            o.drive_r = float(tg - qf);
            jd_r += jref;
        }
    }
    }
    return o;
}

AxleStep axle_step(const AwdLayout& lay, const AwdIn& in, bool& engaged, float jf, float jr, float jref, bool brake_f,
                   bool brake_r, bool f_front, bool f_rear, float dt) {
    float jd_f, jd_r;
    AxleStep o = axle_drive(lay, in, engaged, jf, jr, jref, jd_f, jd_r);
    const double wf0 = in.wf, wr0 = in.wr, h = dt, Ff = in.ff, Fr = in.fr;
    // zero-crossing clamp only with brake torque: w := 0 if it changes sign or was 0
    auto clampz = [](double old, double nw, bool braked) { return braked && (old == 0 || (old > 0) != (nw > 0)) ? 0.0 : nw; };
    double wf = wf0, wr = wr0;
    if (o.common) {
        double w0 = lay.driven_axle == 0 ? wf0 : wr0;
        double w = w0 + (double(in.tg) + Ff + Fr) / (double(jf) + jr + jref) * h;
        wf = wr = clampz(w0, w, brake_f || brake_r);
    } else if (lay.layout == 3) {
        double mean0 = 0.5 * (wf0 + wr0), d0 = 0.5 * (wf0 - wr0);
        double mean = clampz(mean0, mean0 + (double(in.tg) + Ff + Fr) / (double(jf) + jr + jref) * h, brake_f || brake_r);
        double dd = d0 + (Ff - Fr - lay.k * (wf0 - wr0)) * h / (double(jf) + jr);
        if (dd * d0 < 0) dd = 0;
        wf = mean + dd;
        wr = mean - dd;
    } else {
        if (!f_front && jd_f != 0) wf = clampz(wf0, wf0 + (o.drive_f + Ff) / jd_f * h, brake_f);
        if (!f_rear && jd_r != 0) wr = clampz(wr0, wr0 + (o.drive_r + Fr) / jd_r * h, brake_r);
    }
    // centre-clutch crossing: the non-driven axle snaps to the driven one
    bool coupled = lay.layout == 2 || (lay.layout >= 4 && !o.common && o.drive_f != 0);
    if (coupled && ((wf0 - wr0) * (wf - wr) < 0 || (wf - wr == 0 && wf0 - wr0 != 0))) { // a new 0 counts
        if (lay.driven_axle == 0) wr = wf;
        else wf = wr;
        if (lay.layout >= 4 && lay.layout <= 6) engaged = true;
    }
    o.wf = float(wf);
    o.wr = float(wr);
    return o;
}


bool clutch_request(int state, float rpm, float idle_rpm, float brake, float handbrake, bool rear_driven, bool counter) {
    if (counter) return false;
    if (rear_driven && handbrake > 0.8f) return false;
    return state != 0 ? rpm >= idle_rpm : brake < 0.25f;
}

int clutch_engage(int state, bool request, int gear, float speed, float throttle, float rpm, float launch_byte_rpm,
                  float idle_rpm) {
    if (gear == 0) return 0; // neutral (ours only; the original has none)
    if (!request) return state == 1 ? 0 : state; // request 0 opens only a locked clutch
    if (state != 0) return state;
    float launch = std::max(launch_byte_rpm, idle_rpm + 500);
    bool moving = gear > 0 ? speed > 10 / 3.6f : speed < -10 / 3.6f;
    return (moving ? throttle > 0.25f : rpm > launch) ? 2 : 0;
}

int clutch_gear_change(int state, int from, int to) {
    if (state == 0 || from == to || from == 0 || to == 0) return state;
    return (from > 0 && to > 0) ? 3 : 2;
}

int clutch_after(int state, bool crossed, float brake, float engine_w, float idle_rpm, float speed, float wheel_side_w) {
    if ((state == 2 || state == 3) && crossed) return 1;
    if (state == 2 && brake > 0.75f && (clutch_anti_stall(brake, engine_w, idle_rpm) || std::fabs(speed) < 10 / 3.6f)) return 0;
    if (state == 3 && wheel_side_w <= idle_rpm * kRpmToRadS) return 0;
    return state;
}


float engine_condition_factor(float E, float O) {
    const float E1 = -300000, c1 = 0.03f, E2 = 500000, E3 = 600000, c2 = 0.02f, O1 = -100000, cO = 0.05f;
    float fe = E < E1 ? 1 - c1 : E < 0 ? 1 - c1 * E / E1 : E <= E2 ? 1.0f : E < E3 ? 1 - c2 * (E - E2) / (E3 - E2) : 1 - c2;
    float fo = O < O1 ? 1 + cO : O < 0 ? 1 + cO * O / O1 : 1.0f;
    return fe * fo;
}

void engine_condition_step(float& E, float& O, float d) {
    E = std::min(E + d, 600000.0f);
    if (O < 0) {
        float o = O + d;
        O = o >= 0 ? 300000.0f : o;
    } else {
        O = std::min(O + d, 300000.0f);
    }
}

float odometer_step(u32& odometer, float x) {
    if (x <= 0) return 0;
    const s32 n = s32(x) / 2; // truncated metres, halved towards zero
    const u32 w = odometer + u32(n);
    if (w >= kOdometerMax) {
        odometer = kOdometerMax;
        return 0;
    }
    odometer = w;
    return x - float(2 * n);
}

bool oil_worn(float O) { return O >= 300000.0f; }

float dirt_front_brake_scale(float steer) { return 1 - std::min(1.0f, std::fabs(steer) / (15 * kPiF / 180)); }


// ---- planar motion [physics §4.1] ----

ForwardStep forward_step(const PlanarWheel w[4], float u, float mass, float rolling, float ext, float drag, float dt) {
    float fx = 0, meff = 0, ft = 0;
    for (int i = 0; i < 4; ++i) {
        fx += w[i].k * w[i].drive_part;
        if (!w[i].flagged) meff += w[i].ia_r2;
        ft += w[i].k * w[i].brake_part - std::sin(w[i].a) * w[i].slip_grip - std::cos(w[i].a) * w[i].slip_drag;
    }
    fx += rolling;
    float minv = meff > 0 ? 1.0f / (meff + mass) : 1.0f / mass;
    float a0 = (fx + ext + drag) * minv;
    float u1 = u + a0 * dt;
    float at = ft * minv;
    float u2 = u1 + at * dt;
    ForwardStep o;
    if (u1 * u2 > 0) {
        o.u = u2;
        o.ax = a0 + at;
    } else {
        o.u = 0;
        o.ax = a0 - u1 / dt;
    }
    return o;
}

LateralYaw lateral_yaw(const PlanarWheel w[4], const float X[4], const float Y[4], float mass, float yaw_param, float ext_lat,
                       const float grip[4]) {
    LateralYaw o;
    float fa_sum = 0, n = 0, gsum = 0;
    for (int i = 0; i < 4; ++i) {
        float g = w[i].drive_part + w[i].brake_part;
        float ca = std::cos(w[i].a), sa = std::sin(w[i].a);
        float fa = -ca * w[i].slip_grip + sa * w[i].slip_drag - w[i].k * w[i].hold - w[i].s * g;
        float fb = -sa * w[i].slip_grip - ca * w[i].slip_drag - w[i].s * w[i].hold + w[i].k * g;
        fa_sum += fa;
        n += X[i] * fb - Y[i] * fa;
        gsum += grip[i];
    }
    o.yaw_acc = yaw_param > 0 ? n / yaw_param : 0;
    o.ay = (fa_sum - ext_lat) / mass;
    o.g_over_mu = gsum > 0 ? 4 * o.ay / gsum : 0;
    return o;
}

bool stop_hold(float u, const float fade[4], bool any_brake, bool frozen) {
    if (frozen) return true;
    bool fades_zero = fade[0] == 0 && fade[1] == 0 && fade[2] == 0 && fade[3] == 0;
    return std::fabs(u) < 1 / 3.6f && fades_zero && any_brake;
}


// ---- per-wheel force outputs [physics §3.8] ----

TyreCurves tyre_curves(const TireSpec& t, float x) {
    TyreCurves o;
    o.c = t.long_c.x.empty() ? 0.0f : lookup(t.long_c, x);
    o.d = t.limit_d.x.empty() ? 1.0f : lookup(t.limit_d, x);
    return o;
}

TyreLong tyre_long_force(float c, float fp, int st, int sg, float g) {
    TyreLong o;
    float f = c * fp;
    if (sg < 0) f = -f;
    float gf = g * f;
    if (st > 0) o.drive_part = f, o.drive_gained = gf;
    else o.brake_part = f, o.brake_gained = gf;
    return o;
}

namespace {
float wrap_pi(float a) { // into [-pi, pi)
    const float tp = kTwoPiF, p = kPiF;
    a = std::fmod(a + p, tp);
    if (a < 0) a += tp;
    return a - p;
}
} // namespace

TyreLat tyre_lateral(const TireSpec& t, float steer, bool hub_backwards, float phi, float k, float fp, float d) {
    TyreLat o;
    if (k == 0) return o;
    const float p = kPiF, hp = kPiF / 2;
    float a = steer + (hub_backwards ? p : 0.0f);
    float beta = wrap_pi(wrap_pi(a) - wrap_pi(phi));
    o.beta = beta;
    float v, sg;
    if (beta < -hp) v = beta + p, sg = 1;
    else if (beta < 0) v = -beta, sg = -1;
    else if (beta <= hp) v = beta, sg = 1;
    else v = p - beta, sg = -1;
    float u1 = (sg * lookup(t.slip_a, v)) * k, u2 = lookup(t.slip_b, v) * k;
    float m = std::sqrt(u1 * u1 + u2 * u2);
    if (d < m) {
        u1 *= d / m;
        u2 *= d / m;
    }
    o.slip_grip = u1 * fp;
    o.slip_drag = u2 * fp;
    return o;
}

int slip_state(float x, float w) {
    const float eps = 1.0f / 57.6f;
    if (std::fabs(w) < eps) return -1;
    if (w > 0) return x <= w ? 1 : -1;
    return x >= w ? 1 : -1;
}

int slip_sign(float x, float w) {
    const float eps = 1.0f / 57.6f;
    if (std::fabs(w) < eps) return w > x ? 1 : w < x ? -1 : 0;
    if (x == w) return w > 0 ? 1 : -1;
    return w > x ? 1 : -1;
}

float slip_gain(float u) { return 0.5f + 0.5f * std::min(std::fabs(u) * 3.6f / 100, 1.0f); }


float ascc_pressure(const AidsSpec& a, int axle, bool outside, float pedal, float u, float o, float brake, float throttle,
                    float* cap_scale) {
    float p = pedal, cap = 1;
    auto flag = [&](int bit) { return (a.ascc_flags >> bit) & 1; };
    if (brake == 0) {
        if (outside && flag(0 + axle) && a.ascc_a_gain > 0 && o > a.ascc_a_thr) p += (o - a.ascc_a_thr) * a.ascc_a_gain;
        if (!outside && throttle == 0 && flag(4 + axle) && a.ascc_c_gain > 0 && u > a.ascc_c_thr) p += (u - a.ascc_c_thr) * a.ascc_c_gain;
    } else {
        // [physics §4.6] D / E scale both the pressure and the wheel's ABS target slip cap
        if (!outside && flag(2 + axle) && a.ascc_d_gain > 0) {
            float f = std::clamp(1 - (o - 0.1f) * a.ascc_d_gain, 0.0f, 1.0f);
            p *= f, cap *= f;
        }
        if (outside && flag(6 + axle) && a.ascc_e_gain > 0) {
            float f = std::clamp(1 - (u - 0.1f) * a.ascc_e_gain, 0.0f, 1.0f);
            p *= f, cap *= f;
        }
    }
    if (cap_scale) *cap_scale = cap;
    return std::clamp(p, 0.0f, 1.0f);
}

bool asm_lamp_lit(const AidsSpec& a) {
    return a.ascc_a_gain > 0 || a.ascc_c_gain > 0 || a.ascc_d_gain > 0 || a.ascc_e_gain > 0;
}

} // namespace gt3r::sim::model
