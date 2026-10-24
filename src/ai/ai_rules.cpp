// The AI's decision rules as small pure functions (ai_rules.h).

#include "ai/ai_rules.h"

#include "ai/ai_driver.h"
#include "sim/car_spec.h"
#include "sim/models.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace gt3r::ai {

namespace {

constexpr float kKmh = 1.0f / 3.6f;
constexpr float kDeg = std::numbers::pi_v<float> / 180.0f;
constexpr float kGravity = 9.8f; // the tyre records' grip scale (physics §6)

} // namespace

float record_grip(const sim::CarSpec& spec, int axle, int surface) {
    const int s = std::clamp(surface, 0, 15);
    int slot = spec.surface_slot[size_t(s)];
    if (slot < 0 || slot > 2) slot = 0;
    const size_t a = axle ? 1 : 0;
    const sim::TireSpec* rec = &spec.tire_slots[a][size_t(slot)];
    if (rec->slip_a.x.empty()) rec = &spec.tire[a]; // a hand-built spec before its slots are filled
    float mu = rec->surface_mu[size_t(s)] != 0 ? rec->surface_mu[size_t(s)] : rec->mu * sim::model::surface_mu(s);
    return kGravity * mu;
}

float corner_grip(const sim::CarSpec& spec, int surface, float front_weight, const std::array<float, 4>& wear_grip,
                  bool live_wear) {
    const float front = front_weight * record_grip(spec, 0, surface);
    const float rear = (1 - front_weight) * record_grip(spec, 1, surface);
    if (!live_wear) return front + rear;
    // Each wheel's wear grip factor times its axle's grip, halved: the mean per axle.
    return 0.5f * ((wear_grip[0] + wear_grip[1]) * front + (wear_grip[2] + wear_grip[3]) * rear);
}

float braking_grip(const sim::CarSpec& spec, int surface, const std::array<float, 4>& wear_grip, bool live_wear) {
    const float front = record_grip(spec, 0, surface);
    return live_wear ? front * 0.5f * (wear_grip[0] + wear_grip[1]) : front;
}

float corner_speed(float mu, float radius, float bank, int speed_adjust_kmh, const DriverParams& p, float k_down,
                   float v_top) {
    float a = mu * std::fabs(radius);
    // [placeholder floor 0.2] a bank leaning into the arc allows more speed
    if (bank > 0) a /= std::max(1.0f - std::sin(bank), 0.2f);
    float v = 0;
    for (int k = 0; k < 3; ++k) v = std::sqrt(a * (1 + k_down * v * v));
    v = v * p.speed_scale + p.speed_offset + 2.7778f * float(p.speed_trim) + float(speed_adjust_kmh) * kKmh;
    return std::clamp(v, 20.0f * kKmh, std::max(v_top, 20.0f * kKmh));
}

Pedals speed_controller(float v, float target, float gain) {
    Pedals o;
    float e = std::clamp(v - (target - 2 * kKmh), -10 * kKmh, 10 * kKmh);
    o.throttle = std::clamp(0.75f + gain * e, 0.0f, 1.0f);
    o.brake = (o.throttle == 0 && v > target) ? 1.0f : 0.0f;
    return o;
}

std::vector<float> pit_markers(const AiCourse& course) {
    std::vector<float> out;
    const AiCourse::Line* l = course.line(AiCourse::kPitStop);
    if (!l) return out;
    const float lap = course.length();
    bool later = false; // from the first point in the first half of the lap on, one lap is added
    for (const AiCourse::Point& p : l->points) {
        later = later || p.progress < 0.5f * lap;
        out.push_back(later ? p.progress + lap : p.progress);
    }
    return out;
}

int pit_lane_section(const AiCourse& course, float s) {
    const std::vector<float> m = pit_markers(course);
    const int n = int(m.size());
    if (n < 5) return 0;
    const float lap = course.length();
    const float f = s < 0.5f * lap ? s + lap : s;
    if (f < m[1]) return 0;
    if (f < m[2]) return 1;
    if (f < m[size_t(n - 3)]) return 2;
    return 0;
}

SpeedPlan plan_speed(int line, int state, int mode, float v_cur, float v_next, float pit_speed, int section) {
    SpeedPlan sp;
    sp.mode = mode, sp.target = v_cur, sp.next = v_next;
    if (line == AiCourse::kPitStop) { // the stop line: 20 km/h, braking from a straight (gain -2)
        sp.mode = 3;
        sp.target = sp.next = 20 * kKmh;
        sp.pit_limit = true;
    } else if (line == AiCourse::kPitLane && section == 1) { // the braking modes aim at the pit speed
        sp.next = pit_speed + 2 * kKmh;
    } else if (line == AiCourse::kPitLane && section == 2) { // held at the pit speed (arc mode, gain -0.5)
        sp.mode = 1;
        sp.target = sp.next = pit_speed + 2 * kKmh;
        sp.pit_limit = true;
    } else if (state == 2) {
        sp.target = 50 * kKmh;
    }
    sp.turning = state == 2;
    return sp;
}

float arc_target(float target, int arc_ticks, const DriverParams& p, bool on_pit_surface) {
    return !on_pit_surface && arc_ticks >= p.corner_bonus_ticks ? target + p.corner_bonus : target;
}

float recovery_threshold(int state, float v) {
    if (state != 1) return 60 * kDeg;
    if (v >= 0) return 10 * kDeg;
    if (v > -30 * kKmh) return 10 * kDeg - v * (50 * kDeg) / (30 * kKmh);
    return 60 * kDeg;
}

int drive_decision(int state, float abs_eps, float theta, float v) {
    if (abs_eps < theta || state == 2) return v > -2 * kKmh ? kDriveForward : kDriveBrake;
    return v > 2 * kKmh ? kDriveBrake : kDriveReverse;
}

int recovery_next(int state, float abs_eps, float theta, float v, unsigned hit_sides) {
    const bool front = hit_sides & 1u, rear = hit_sides & 2u; // collide::kFrontHit / kRearHit
    switch (state) {
    case 1:
        if (abs_eps < theta) return v > 3 * kKmh ? 0 : 2;
        return rear ? 2 : 1;
    case 2:
        if (abs_eps < 40 * kDeg && v > 30 * kKmh) return 0;
        return front && abs_eps > 60 * kDeg ? 1 : 2;
    default:
        return abs_eps > 80 * kDeg || (rear && abs_eps > 45 * kDeg) ? 2 : 0;
    }
}

EndOfRun end_of_run(RunEnd why, int licence_test_kind, bool on_pit_surface) {
    EndOfRun e;
    switch (why) {
    case RunEnd::RaceFinish:
    case RunEnd::TimeTrialFinish:
    case RunEnd::MachineTest:
        e.kind = kKindAi, e.slow = true, e.main_line = true;
        break;
    case RunEnd::LicenceFinish:
        if (licence_test_kind == 1 || licence_test_kind == 2) e.kind = kKindStopLine, e.main_line = true;
        else e.kind = kKindAi, e.slow = true, e.main_line = true;
        break;
    case RunEnd::LicenceFail:
    case RunEnd::TimeTrialFail:
        e.kind = kKindStopStraight;
        break;
    case RunEnd::FreeRunEnd:
        if (on_pit_surface) e.kind = kKindStopStraight;
        else e.kind = kKindAi, e.main_line = true;
        break;
    }
    return e;
}

bool parked(const sim::VehicleState& st) {
    return st.velocity.length() < 0.0694444f && st.angular_velocity.length() < 0.0523599f;
}

} // namespace gt3r::ai
