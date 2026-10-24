// AI driver parameters, event difficulty and rubber band.

#include "ai/ai_driver.h"

#include "game/data/car_database.h"
#include "sim/car_spec.h"

#include <algorithm>

namespace gt3r::ai {

namespace {

constexpr float kKmh = 1.0f / 3.6f;

// The three built-in parameter sets (ai §9.1), as stored (bytes 25-31 unused).
constexpr ParamBytes kSets[3] = {
    {100, 100, 100, 24, 100, 0, 90, 15, 40, 15, 5, 10, 0, 60, 100, 20, 9, 0, 80},
    {100, 100, 100, 24, 100, 0, 90, 15, 40, 15, 5, 10, 0, 60, 100, 20, 9, 0, 80},
    {100, 100, 100, 24, 100, 0, 90, 30, 60, 0, 0, 10, 0, 60, 100, 20, 9, 0, 80},
};

u8 byte_at(const data::Event& ev, size_t off) { return off < ev.bytes.size() ? ev.bytes[off] : 0; }

} // namespace

DriverParams DriverParams::from_bytes(const ParamBytes& b) {
    DriverParams p;
    p.brake_factor = 100.0f / float(std::max<u8>(b[0], 1));
    p.speed_scale = b[1] * 0.01f;
    p.speed_offset = (float(b[2]) - 100.0f) * kKmh;
    p.yaw_gain = float(b[3]) * 5.0f * 0.01f;
    p.sat_throttle_cut = b[4] * 0.01f;
    p.sat_brake = b[5] * 0.01f;
    p.lookahead_full_speed = float(b[6]) * 2.0f * kKmh;
    p.lookahead_min = b[7];
    p.lookahead_max = b[8];
    p.grip_front_weight = b[14] * 0.01f;
    p.corner_bonus = float(b[15]) * kKmh;
    p.steer_rate = float(b[16]) * 10.0f * 0.0174533f;
    p.corner_bonus_ticks = b[17] * 10;
    p.pit_speed = float(b[18] ? b[18] : 80) * kKmh;
    // The AI aid block (ai §9.1): ASCC A gain / threshold, D, C gain / threshold, E (x 0.01) with
    // the channel flags; TCS lateral a (x 0.1), b (x 0.01), gain (x 0.1), slip scale (x 0.01; 0
    // and 100 give 1).
    sim::ScriptedAids& a = p.aids;
    a.ascc_a_gain = b[9] * 0.01f;
    a.ascc_a_thr = b[10] * 0.01f;
    a.tcs_lat_a = b[11] * 0.1f;
    a.tcs_lat_b = b[12] * 0.01f;
    a.tcs_gain = b[13] * 0.1f;
    a.ascc_flags = b[19];
    a.ascc_d_gain = b[20] * 0.01f;
    a.ascc_c_gain = b[21] * 0.01f;
    a.ascc_c_thr = b[22] * 0.01f;
    a.ascc_e_gain = b[23] * 0.01f;
    a.tcs_scale = b[24] == 0 || b[24] == 100 ? 1.0f : b[24] * 0.01f;
    return p;
}

const ParamBytes& DriverParams::builtin_bytes(int set) { return kSets[set == 1 ? 1 : set == 2 ? 2 : 0]; }

DriverParams DriverParams::builtin(int set) { return from_bytes(builtin_bytes(set)); }

int driveline_class(const sim::CarSpec& spec) {
    const sim::DrivetrainSpec& d = spec.drivetrain;
    if (d.drive == sim::Drive::FF) return 1;
    if (d.drive == sim::Drive::AWD) return d.awd_system == 0 ? 1 : d.awd_system == 3 ? 3 : 2;
    return 0;
}

int builtin_set(int course_category) { return course_category == 1 || course_category == 2 ? course_category : 0; }

DriverParams course_params(const course::DriverSets& sets, int course_category, int driveline_class) {
    if (const course::DriverSets::Set* s = sets.for_class(driveline_class)) return DriverParams::from_bytes(*s);
    return DriverParams::builtin(builtin_set(course_category));
}

void apply_event(DriverParams& p, const data::Event& ev, int driveline_class) {
    size_t c = size_t(std::clamp(driveline_class, 0, 3));
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

DriverParams after_finish(DriverParams p) {
    p.speed_scale *= 0.8f;
    p.throttle_scale = 0.5f;
    return p;
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
    // ai §6: the AI clamps its angle at 45 degrees. The physics clamps a CPU car's
    // steer_angle to +-1 rad, not to the lock; the raised lock only changes steer_angle / lock,
    // the steering fraction of the car-to-car stuck-steer nudge (sim::World, physics §5.3).
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
