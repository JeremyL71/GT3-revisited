// AI driver parameters, event difficulty and rubber band.

#include "ai/ai_driver.h"

#include "game/data/car_database.h"
#include "sim/car_spec.h"

#include <algorithm>

namespace gt3r::ai {

namespace {

constexpr float kKmh = 1.0f / 3.6f;

// The three built-in parameter sets (spec §9.1), as the raw bytes the game
// converts. Only the bytes the driver uses are listed; the aid bytes (9..13,
// 19..24) feed the AI's own stability-control block, which our physics does
// not model separately.
struct RawSet {
    u8 brake_pct, speed_pct, speed_offset, yaw_gain, sat_throttle, sat_brake, look_speed, look_min, look_max,
        grip_weight, bonus_kmh, steer_rate, bonus_ticks, pit_kmh;
};
constexpr RawSet kSets[3] = {
    {100, 100, 100, 24, 100, 0, 90, 15, 40, 100, 20, 9, 0, 80},
    {100, 100, 100, 24, 100, 0, 90, 15, 40, 100, 20, 9, 0, 80},
    {100, 100, 100, 24, 100, 0, 90, 30, 60, 100, 20, 9, 0, 80},
};

u8 byte_at(const data::Event& ev, size_t off) { return off < ev.bytes.size() ? ev.bytes[off] : 0; }

} // namespace

DriverParams DriverParams::builtin(int set) {
    const RawSet& r = kSets[set == 1 ? 1 : set == 2 ? 2 : 0];
    DriverParams p;
    p.brake_factor = 100.0f / float(std::max<u8>(r.brake_pct, 1));
    p.speed_scale = r.speed_pct * 0.01f;
    p.speed_offset = (float(r.speed_offset) - 100.0f) * kKmh;
    p.yaw_gain = float(r.yaw_gain) * 5.0f * 0.01f;
    p.sat_throttle_cut = r.sat_throttle * 0.01f;
    p.sat_brake = r.sat_brake * 0.01f;
    p.lookahead_full_speed = float(r.look_speed) * 2.0f * kKmh;
    p.lookahead_min = r.look_min;
    p.lookahead_max = r.look_max;
    p.grip_front_weight = r.grip_weight * 0.01f;
    p.corner_bonus = float(r.bonus_kmh) * kKmh;
    p.steer_rate = float(r.steer_rate) * 10.0f * 0.0174533f;
    p.corner_bonus_ticks = r.bonus_ticks * 10;
    p.pit_speed = float(r.pit_kmh ? r.pit_kmh : 80) * kKmh;
    return p;
}

void apply_event(DriverParams& p, const data::Event& ev, int chassis_class) {
    size_t c = size_t(std::clamp(chassis_class, 0, 3));
    int throttle = byte_at(ev, 0x1A4 + c), brake = byte_at(ev, 0x1A8 + c), speed = byte_at(ev, 0x1AC + c);
    int offset = s8(byte_at(ev, 0x1B0 + c));
    if (throttle == 0 || throttle > 100) throttle = 100;
    if (brake == 0) brake = 100;
    if (speed == 0) speed = 100;
    if (throttle != 100) p.throttle_scale = throttle * 0.01f;
    if (brake != 100) p.brake_factor *= 100.0f / float(brake);
    if (speed != 100) p.speed_scale *= speed * 0.01f;
    if (offset) p.speed_offset += float(offset) * kKmh;
}

RubberBand RubberBand::from_event(const data::Event& ev) {
    RubberBand r;
    int g = byte_at(ev, 0x1B4);
    r.global = (g ? g : 100) * 0.01f;
    r.slow_amount = byte_at(ev, 0x1B7) * 0.01f;
    r.slow_start = byte_at(ev, 0x1B8);
    r.slow_full = byte_at(ev, 0x1B9) * 10.0f;
    r.fast_amount = byte_at(ev, 0x1BC) * 0.01f;
    r.fast_start = byte_at(ev, 0x1BA);
    r.fast_full = byte_at(ev, 0x1BB) * 10.0f;
    return r;
}

void adapt_spec_for_ai(sim::CarSpec& spec) {
    // spec §6: the AI clamps its angle at 45 degrees; physics clamps
    // steer_angle to the car's full lock, so AI cars get at least that much.
    spec.steering.max_angle = std::max(spec.steering.max_angle, 0.7854f);
}

float RubberBand::scale(double gap) const { return factor(gap) * global; }

std::vector<float> RubberBand::chain(const std::vector<double>& distance, int reference,
                                     const std::vector<bool>& no_speedup) const {
    size_t n = distance.size();
    std::vector<float> out(n, global);
    if (reference < 0 || size_t(reference) >= n) return out;
    std::vector<int> order(n);
    for (size_t i = 0; i < n; ++i) order[i] = int(i);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return distance[size_t(a)] > distance[size_t(b)]; });
    size_t r = size_t(std::find(order.begin(), order.end(), reference) - order.begin());
    std::vector<float> f(n, 1.0f);
    for (size_t k = r; k-- > 0;) { // ahead of R: slowed against the car behind it
        int c = order[k], toward = order[k + 1];
        f[size_t(c)] = f[size_t(toward)] * factor(distance[size_t(toward)] - distance[size_t(c)]);
    }
    for (size_t k = r + 1; k < n; ++k) { // behind R: sped up against the car ahead of it
        int c = order[k], toward = order[k - 1];
        bool skip = size_t(c) < no_speedup.size() && no_speedup[size_t(c)];
        f[size_t(c)] = f[size_t(toward)] * (skip ? 1.0f : factor(distance[size_t(toward)] - distance[size_t(c)]));
    }
    for (size_t i = 0; i < n; ++i) out[i] = int(i) == reference ? 1.0f : f[i] * global;
    return out;
}

float RubberBand::factor(double gap) const {
    auto ramp = [](double x, float a, float b) -> float {
        if (x <= a) return 0;
        if (x >= b || b <= a) return 1;
        return float((x - a) / (b - a));
    };
    float s = 1;
    if (gap > 0) {
        if (fast_amount > 0) s = 1 + fast_amount * ramp(gap, fast_start, fast_full);
    } else if (slow_amount > 0) {
        s = 1 - slow_amount * ramp(-gap, slow_start, slow_full);
    }
    return s;
}

} // namespace gt3r::ai
