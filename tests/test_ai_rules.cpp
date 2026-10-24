// The AI's rules one by one, without game data (src/ai/ai_rules.h, ai_params.cpp,
// safety_respawn.h; the course file's driver parameter sets; the collision sides the recovery
// reads; the AI aid block's ASCC in the physics). Every course, car, set and event value here
// is INVENTED.

#include "test_framework.h"
#include "course_fixtures.h"
#include "sim_fixtures.h"

#include "ai/ai_driver.h"
#include "ai/ai_rules.h"
#include "ai/safety_respawn.h"
#include "formats/course/course_file.h"
#include "formats/course/driver_sets.h"
#include "game/data/car_database.h"
#include "sim/collision.h"
#include "sim/track_walls.h"
#include "sim/vehicle.h"
#include "sim/world.h"
#include "track/track.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>

using namespace gt3r;
namespace st = gt3r::test::stadium;

namespace {

constexpr float kKmh = 1 / 3.6f;
constexpr float kDeg = 3.14159265f / 180.0f;
constexpr double kDt = 1.0 / sim::kDefaultTickHz;

bool near(float a, float b, float tol = 1e-5f) { return std::fabs(a - b) <= tol; }

void put32(Bytes& b, size_t o, u32 v) {
    for (int k = 0; k < 4; ++k) b[o + size_t(k)] = u8(v >> (8 * k));
}

} // namespace

TEST(ai_course_sets_decode) {
    // A course file with only an offset table and the sets: slots 37, 38 and 40 present (0x40
    // bytes each), slot 39 empty (offset 0). Each set's first byte tells it apart.
    Bytes f(0xC0 + 3 * 0x40, 0);
    const u32 off[4] = {0xC0, 0x100, 0, 0x140};
    for (int c = 0; c < 4; ++c) {
        put32(f, size_t(37 + c) * 4, off[c]);
        if (off[c])
            for (int k = 0; k < 0x40; ++k) f[off[c] + size_t(k)] = u8(k < 25 ? 10 * (c + 1) + k : 0);
    }
    course::CourseFile file = course::CourseFile::parse(f);
    course::DriverSets sets = course::DriverSets::parse(f, file);
    CHECK(!sets.empty());
    CHECK(sets.for_class(0) && (*sets.for_class(0))[0] == 10 && (*sets.for_class(0))[24] == 34);
    CHECK(sets.for_class(1) && (*sets.for_class(1))[0] == 20);
    CHECK(sets.for_class(2) == nullptr); // the empty slot
    CHECK(sets.for_class(3) && (*sets.for_class(3))[0] == 40 && (*sets.for_class(3))[31] == 0);
    CHECK(sets.for_class(4) == sets.for_class(0) && sets.for_class(-1) == sets.for_class(0)); // other classes: slot 37
    course::DriverSets none;
    CHECK(none.empty() && none.for_class(0) == nullptr);
}

TEST(ai_params_from_bytes) {
    // Every byte's conversion (ai §9.1), on an invented set.
    const ai::ParamBytes b = {125, 90, 110, 30, 80, 20, 100, 12, 45, 40, 10, 15, 25, 50, 80, 15, 12, 3, 0,
                              0x31, 60, 120, 15, 70, 45};
    ai::DriverParams p = ai::DriverParams::from_bytes(b);
    CHECK(near(p.brake_factor, 0.8f) && near(p.speed_scale, 0.9f) && near(p.speed_offset, 10 * kKmh, 1e-5f));
    CHECK(near(p.yaw_gain, 1.5f) && near(p.sat_throttle_cut, 0.8f) && near(p.sat_brake, 0.2f));
    CHECK(near(p.lookahead_full_speed, 200 * kKmh, 1e-4f) && p.lookahead_min == 12 && p.lookahead_max == 45);
    CHECK(near(p.grip_front_weight, 0.8f) && near(p.corner_bonus, 15 * kKmh) && p.corner_bonus_ticks == 30);
    CHECK(near(p.steer_rate, 120 * kDeg, 1e-4f));
    CHECK(near(p.pit_speed, 80 * kKmh)); // 0 -> 80 km/h
    const sim::ScriptedAids& a = p.aids;
    CHECK(near(a.ascc_a_gain, 0.4f) && near(a.ascc_a_thr, 0.1f) && near(a.tcs_lat_a, 1.5f) && near(a.tcs_lat_b, 0.25f));
    CHECK(near(a.tcs_gain, 5.0f) && a.ascc_flags == 0x31 && near(a.ascc_d_gain, 0.6f) && near(a.ascc_c_gain, 1.2f));
    CHECK(near(a.ascc_c_thr, 0.15f) && near(a.ascc_e_gain, 0.7f) && near(a.tcs_scale, 0.45f));
    ai::ParamBytes b2 = b;
    b2[18] = 70, b2[24] = 100;
    ai::DriverParams p2 = ai::DriverParams::from_bytes(b2);
    CHECK(near(p2.pit_speed, 70 * kKmh) && near(p2.aids.tcs_scale, 1.0f)); // slip scale 100 -> 1
    b2[24] = 0;
    CHECK(near(ai::DriverParams::from_bytes(b2).aids.tcs_scale, 1.0f));   // and 0 -> 1
    // The built-in sets: TCS 6.0 / 1.0 / 0 / 1.0, no ASCC channel enabled.
    for (int set = 0; set < 3; ++set) {
        ai::DriverParams q = ai::DriverParams::builtin(set);
        CHECK(near(q.aids.tcs_gain, 6.0f) && near(q.aids.tcs_lat_a, 1.0f) && q.aids.tcs_lat_b == 0 &&
              q.aids.tcs_scale == 1 && q.aids.ascc_flags == 0);
    }
}

TEST(ai_course_params_select) {
    // ai §9.1: the course's set for the car's driveline class; the built-in set of the course
    // category where the slot is empty; the event's overrides for that class on top (§9.2).
    course::DriverSets sets;
    sets.by_class[1] = course::DriverSets::Set{100, 90, 100, 24, 100, 0, 90, 20, 50, 0, 0, 10, 0, 0, 100, 20, 9, 0, 70};
    ai::DriverParams p = ai::course_params(sets, 0, 1);
    CHECK(near(p.speed_scale, 0.9f) && p.lookahead_min == 20 && near(p.pit_speed, 70 * kKmh) && p.aids.tcs_gain == 0);
    // Empty slots: the category's built-in set (1 -> 1, 2 -> 2, else 0).
    CHECK(ai::course_params(sets, 2, 0).lookahead_min == 30);
    CHECK(ai::course_params(sets, 0, 2).lookahead_min == 15 && ai::course_params(sets, 3, 3).lookahead_min == 15);
    CHECK(ai::builtin_set(1) == 1 && ai::builtin_set(2) == 2 && ai::builtin_set(5) == 0);
    // Event overrides per class: class 1 throttle 90 %, brake 80 %, corner speed 50 %, offset -5 km/h.
    Bytes ev_bytes(0x200, 0);
    ev_bytes[0x1A5] = 90, ev_bytes[0x1A9] = 80, ev_bytes[0x1AD] = 50, ev_bytes[0x1B1] = u8(s8(-5));
    data::Event ev;
    ev.bytes = ev_bytes;
    ai::apply_event(p, ev, 1);
    CHECK(near(p.throttle_scale, 0.9f) && near(p.brake_factor, 1.25f) && near(p.speed_scale, 0.45f) &&
          near(p.speed_offset, -5 * kKmh));
    // After the finish (ai §1.1): corner speed x 0.8, throttle 0.5.
    ai::DriverParams f = ai::after_finish(p);
    CHECK(near(f.speed_scale, 0.36f) && near(f.throttle_scale, 0.5f) && near(f.brake_factor, p.brake_factor));
}

TEST(ai_driveline_class) {
    sim::CarSpec s = test::synthetic_spec_car(sim::Drive::FR);
    CHECK(ai::driveline_class(s) == 0);
    s.drivetrain.drive = sim::Drive::MR;
    CHECK(ai::driveline_class(s) == 0);
    s.drivetrain.drive = sim::Drive::FF;
    CHECK(ai::driveline_class(s) == 1);
    s.drivetrain.drive = sim::Drive::AWD;
    for (auto [system, cls] : {std::pair{0, 1}, {1, 2}, {2, 2}, {3, 3}}) {
        s.drivetrain.awd_system = system;
        CHECK(ai::driveline_class(s) == cls);
    }
}

TEST(ai_corner_grip_formula) {
    // ai §4.2: 9.8 x the tyre record's mu for the point's surface (the record of the compound slot
    // the surface uses); w-weighted front / rear; the wear grip factors per axle when live.
    // The synthetic car: road records mu 1.5 (100 % on tarmac), loose records mu 1.3 (60 % on
    // grass, which uses compound slot 1).
    const sim::CarSpec s = test::synthetic_spec_car();
    CHECK(near(ai::record_grip(s, 0, 0), 9.8f * 1.5f, 1e-4f));
    CHECK(near(ai::record_grip(s, 1, 2), 9.8f * 1.3f * 0.6f, 1e-4f));
    const std::array<float, 4> fresh{1, 1, 1, 1}, worn{0.9f, 0.8f, 0.7f, 0.6f};
    CHECK(near(ai::corner_grip(s, 0, 1.0f, fresh, true), 14.7f, 1e-4f));
    CHECK(near(ai::corner_grip(s, 0, 1.0f, worn, true), 14.7f * 0.85f, 1e-4f));        // front mean 0.85
    CHECK(near(ai::corner_grip(s, 0, 0.5f, worn, true), 7.35f * 0.85f + 7.35f * 0.65f, 1e-4f));
    CHECK(near(ai::corner_grip(s, 0, 0.5f, worn, false), 14.7f, 1e-4f));               // pit surface: no wear
    CHECK(near(ai::braking_grip(s, 0, worn, true), 14.7f * 0.85f, 1e-4f));
    // The point's surface matters, the car's grip curves and loads do not.
    sim::CarSpec t = s;
    for (auto& a : t.tire_slots)
        for (auto& r : a) r.slip_a.y = {0, 0.1f, 0.2f, 0.3f, 0.3f, 0.3f, 0.3f};
    CHECK(near(ai::corner_grip(t, 0, 1.0f, fresh, true), 14.7f, 1e-4f));
}

TEST(ai_corner_speed_formula) {
    ai::DriverParams p;
    // v = sqrt(a (1 + k v^2)) three times from 0: sqrt(a), then twice more (ai §4.2).
    const float mu = 10, r = 100, k = 2e-4f, a = mu * r;
    float v1 = std::sqrt(a), v2 = std::sqrt(a * (1 + k * v1 * v1)), v3 = std::sqrt(a * (1 + k * v2 * v2));
    CHECK(near(ai::corner_speed(mu, r, 0, 0, p, k, 100), v3, 1e-4f));
    CHECK(near(ai::corner_speed(mu, -r, 0, 0, p, 0, 100), std::sqrt(a), 1e-4f)); // the radius' sign does not matter
    // Scale, offset, trim (10 km/h steps) and the point's km/h adjustment.
    p.speed_scale = 0.8f, p.speed_offset = 2, p.speed_trim = 1;
    CHECK(near(ai::corner_speed(mu, r, 0, 7, p, 0, 100), std::sqrt(a) * 0.8f + 2 + 2.7778f + 7 * kKmh, 1e-4f));
    // Clamped to [20 km/h, top speed].
    CHECK(near(ai::corner_speed(0.1f, 1, 0, 0, ai::DriverParams{}, 0, 100), 20 * kKmh));
    CHECK(near(ai::corner_speed(mu, r, 0, 0, ai::DriverParams{}, 0, 25), 25));
}

TEST(ai_speed_controller_rule) {
    // throttle = clamp(0.75 + G clamp(v - (target - 2 km/h), +-10 km/h), 0, 1); brake only with the throttle
    // shut and v above the target (ai §4.4).
    ai::Pedals at = ai::speed_controller(20 - 2 * kKmh, 20, -0.5f);
    CHECK(near(at.throttle, 0.75f) && at.brake == 0);
    ai::Pedals over = ai::speed_controller(30, 20, -2.0f);
    CHECK(over.throttle == 0 && over.brake == 1);
    ai::Pedals under = ai::speed_controller(19, 20, -0.5f); // the error is clamped to 10 km/h
    CHECK(near(under.throttle, 0.75f + 0.5f * (1 - 2 * kKmh), 1e-5f) && under.brake == 0);
    CHECK(ai::speed_controller(10, 20, -0.5f).throttle == 1);
    ai::Pedals lift = ai::speed_controller(20.1f, 20, -0.5f); // over the target, throttle not yet shut: no brake
    CHECK(lift.throttle > 0 && lift.throttle < 0.75f && lift.brake == 0);
}

TEST(ai_pit_lane_target) {
    // The pit lines (ai §4.4 pit speed; corner bonus ai §5): the stop line (4) at 20 km/h braking (gain -2);
    // on the pit lane line (3) the braking modes aim at pit speed + 2 km/h from its point 1 and the
    // car is held at it (arc mode, gain -0.5) from point 2 to point n - 3; otherwise the normal
    // targets; turning round (state 2) aims at 50 km/h.
    const float pit = 60 * kKmh;
    ai::SpeedPlan stop = ai::plan_speed(4, 0, 0, 30, 25, pit, 0);
    CHECK(stop.mode == 3 && near(stop.next, 20 * kKmh) && stop.pit_limit && !stop.turning);
    ai::SpeedPlan in1 = ai::plan_speed(3, 0, 3, 30, 25, pit, 1);
    CHECK(in1.mode == 3 && near(in1.target, 30) && near(in1.next, pit + 2 * kKmh) && !in1.pit_limit);
    ai::SpeedPlan in2 = ai::plan_speed(3, 0, 0, 30, 25, pit, 2);
    CHECK(in2.mode == 1 && near(in2.target, pit + 2 * kKmh) && in2.pit_limit);
    ai::SpeedPlan out = ai::plan_speed(3, 0, 1, 30, 25, pit, 0);
    CHECK(out.mode == 1 && near(out.target, 30) && near(out.next, 25) && !out.pit_limit);
    ai::SpeedPlan turn = ai::plan_speed(6, 2, 1, 30, 25, pit, 0);
    CHECK(turn.turning && near(turn.target, 50 * kKmh));
    // No corner bonus on the pit surface; elsewhere after the bonus delay.
    ai::DriverParams p;
    p.corner_bonus = 5, p.corner_bonus_ticks = 30;
    CHECK(near(ai::arc_target(20, 30, p, false), 25) && near(ai::arc_target(20, 29, p, false), 20));
    CHECK(near(ai::arc_target(20, 300, p, true), 20));

    // The sections on a course with a pit lane line (game line 3, the centre line) and a pit
    // stop line (game line 4) of 12 points, 6 before the start line (620-710 m) and 6 after it
    // (5-80 m): the markers from the first point after the line on get a lap added, and so does
    // the car's distance in the first half of the lap.
    course::AiLines lines = st::make_ai_lines();
    lines.lines[2] = st::make_line(0);
    course::AiLine stop_line;
    for (float d : {620.0f, 640.0f, 660.0f, 680.0f, 700.0f, 710.0f, 5.0f, 20.0f, 35.0f, 50.0f, 65.0f, 80.0f}) {
        course::AiPoint q;
        sim::Vec3 w = st::at(d, 5);
        q.pos = {w.x, 0, w.z};
        stop_line.points.push_back(q);
    }
    lines.lines[3] = stop_line;
    track::Track t(st::make_road(), lines, "stadium-pit");
    ai::AiCourse c(t);
    CHECK(c.line(3) && c.line(4) && c.line(4)->points.size() == 12);
    std::vector<float> m = ai::pit_markers(c);
    const float L = c.length();
    std::printf("  pit markers: point 1 %.0f m, point 2 %.0f m, point 6 %.0f m (lap %.0f m), point n-3 %.0f m\n", m[1], m[2],
                m[6], L, m[9]);
    CHECK(m.size() == 12 && near(m[5], 710, 0.5f) && near(m[6], L + 5, 0.5f) && near(m[9], L + 50, 0.5f));
    CHECK(ai::pit_lane_section(c, 630) == 0 && ai::pit_lane_section(c, 650) == 1 && ai::pit_lane_section(c, 670) == 2);
    CHECK(ai::pit_lane_section(c, 10) == 2 && ai::pit_lane_section(c, 45) == 2);  // past the line
    CHECK(ai::pit_lane_section(c, 55) == 0 && ai::pit_lane_section(c, 300) == 0); // past point n - 3
    ai::AiCourse plain(*st::make_track()); // no pit lane line: never in a section
    CHECK(ai::pit_lane_section(plain, 100) == 0);
}

TEST(ai_recovery_transitions) {
    // ai §7, every rule with its thresholds (heading error |eps|, forward speed v; front / rear =
    // the collision sides of the last step).
    const unsigned F = sim::collide::kFrontHit, R = sim::collide::kRearHit;
    // Threshold theta.
    CHECK(near(ai::recovery_threshold(0, -5), 60 * kDeg) && near(ai::recovery_threshold(2, 0), 60 * kDeg));
    CHECK(near(ai::recovery_threshold(1, 0), 10 * kDeg) && near(ai::recovery_threshold(1, -15 * kKmh), 35 * kDeg, 1e-4f));
    CHECK(near(ai::recovery_threshold(1, -40 * kKmh), 60 * kDeg));
    // Drive decision.
    CHECK(ai::drive_decision(0, 30 * kDeg, 60 * kDeg, 10) == ai::kDriveForward);
    CHECK(ai::drive_decision(0, 30 * kDeg, 60 * kDeg, -1) == ai::kDriveBrake);        // rolling back > 2 km/h
    CHECK(ai::drive_decision(0, 70 * kDeg, 60 * kDeg, 5) == ai::kDriveBrake);         // wrong way, still moving
    CHECK(ai::drive_decision(0, 70 * kDeg, 60 * kDeg, 0) == ai::kDriveReverse);
    CHECK(ai::drive_decision(2, 170 * kDeg, 60 * kDeg, 0) == ai::kDriveForward);      // turning round drives on
    // 0 -> 2.
    CHECK(ai::recovery_next(0, 81 * kDeg, 0, 10, 0) == 2 && ai::recovery_next(0, 79 * kDeg, 0, 10, 0) == 0);
    CHECK(ai::recovery_next(0, 46 * kDeg, 0, 10, R) == 2 && ai::recovery_next(0, 44 * kDeg, 0, 10, R) == 0);
    CHECK(ai::recovery_next(0, 70 * kDeg, 0, 10, F) == 0); // a front hit does not
    // 2 -> 0 and 2 -> 1.
    CHECK(ai::recovery_next(2, 39 * kDeg, 0, 31 * kKmh, 0) == 0 && ai::recovery_next(2, 39 * kDeg, 0, 29 * kKmh, 0) == 2);
    CHECK(ai::recovery_next(2, 61 * kDeg, 0, 0, F) == 1 && ai::recovery_next(2, 59 * kDeg, 0, 0, F) == 2);
    CHECK(ai::recovery_next(2, 120 * kDeg, 0, 0, R) == 2);
    CHECK(ai::recovery_next(2, 39 * kDeg, 0, 10 * kKmh, F) == 2); // under 40 degrees: no reversing
    // 1 -> 0, 1 -> 2 (both rules), 1 stays.
    const float th = ai::recovery_threshold(1, 5 * kKmh);
    CHECK(ai::recovery_next(1, 5 * kDeg, th, 4 * kKmh, 0) == 0);
    CHECK(ai::recovery_next(1, 5 * kDeg, th, 2 * kKmh, 0) == 2);
    CHECK(ai::recovery_next(1, 50 * kDeg, th, -5 * kKmh, R) == 2);
    CHECK(ai::recovery_next(1, 50 * kDeg, th, -5 * kKmh, F) == 1 && ai::recovery_next(1, 50 * kDeg, th, -5 * kKmh, 0) == 1);
}

TEST(ai_collision_sides) {
    // The recovery's flags (ai §7): a wall impulse at octagon points 0-3 is a front hit, 4-7 a rear
    // hit; car-car, the hitting vertex's end for its car and the hit edge's (0-2 front, 4-6 rear,
    // the sides 3 and 7 neither) for the other.
    namespace cl = sim::collide;
    for (int k = 0; k < 8; ++k) CHECK(cl::point_side(k) == (k < 4 ? unsigned(cl::kFrontHit) : unsigned(cl::kRearHit)));
    CHECK(cl::edge_side(0) == cl::kFrontHit && cl::edge_side(2) == cl::kFrontHit && cl::edge_side(3) == 0);
    CHECK(cl::edge_side(4) == cl::kRearHit && cl::edge_side(6) == cl::kRearHit && cl::edge_side(7) == 0);
    // A body driven nose first into a wall across its path: a front point takes the impulse.
    cl::Body b;
    b.local = cl::octagon(2.0, 2.0, 1.8);
    b.radius = cl::bounding_radius(b.local);
    b.inv_mass = 1.0 / 1200, b.inv_inertia = 1.0 / 20000;
    b.heading = 0; // forward (-sin c, -cos c) = -Z
    b.vel = {0, -10};
    b.pos = {0, -3.05};
    std::vector<cl::Segment> walls = {{{10, -5}, {-10, -5}}}; // drivable side +Z
    std::vector<cl::WallHit> hits = cl::collide_walls(b, {0, -2.0}, 0, walls, 0.105, 2);
    CHECK(!hits.empty() && hits[0].body_point >= 0 && cl::point_side(hits[0].body_point) == cl::kFrontHit);
    // Car to car: A runs into B's tail.
    cl::Body a = b, t = b;
    a.pos = {0, 0}, a.vel = {0, -15};
    t.pos = {0, -4.05}, t.vel = {0, -5};
    unsigned sides[2] = {0, 0};
    double j = cl::collide_pair_swept(a, {0, 0}, 0, t, {0, -4.1}, 0, 1.0 / 60, 2, nullptr, nullptr, nullptr, nullptr, sides);
    std::printf("  car-car nose to tail: impulse %.0f N s, sides %u / %u\n", j, sides[0], sides[1]);
    CHECK(j > 0 && sides[0] == cl::kFrontHit && sides[1] == cl::kRearHit);
}

TEST(ai_end_of_run_kinds) {
    // ai §1.1: what drives the player's car after the run.
    using ai::RunEnd;
    for (RunEnd w : {RunEnd::RaceFinish, RunEnd::TimeTrialFinish, RunEnd::MachineTest}) {
        ai::EndOfRun e = ai::end_of_run(w);
        CHECK(e.kind == ai::kKindAi && e.slow && e.main_line);
    }
    CHECK(ai::end_of_run(RunEnd::LicenceFinish, 1).kind == ai::kKindStopLine);
    CHECK(ai::end_of_run(RunEnd::LicenceFinish, 2).kind == ai::kKindStopLine);
    ai::EndOfRun lap = ai::end_of_run(RunEnd::LicenceFinish, 3);
    CHECK(lap.kind == ai::kKindAi && lap.slow);
    CHECK(ai::end_of_run(RunEnd::LicenceFail).kind == ai::kKindStopStraight);
    CHECK(ai::end_of_run(RunEnd::TimeTrialFail).kind == ai::kKindStopStraight);
    CHECK(ai::end_of_run(RunEnd::FreeRunEnd, 0, true).kind == ai::kKindStopStraight);
    ai::EndOfRun free = ai::end_of_run(RunEnd::FreeRunEnd, 0, false);
    CHECK(free.kind == ai::kKindAi && !free.slow);
    // Parked: 3-D speed under 0.0694 m/s and angular speed under 0.0524 rad/s.
    sim::VehicleState s;
    CHECK(ai::parked(s));
    s.velocity = {0, 0.07f, 0};
    CHECK(!ai::parked(s));
    s.velocity = {0.05f, 0, 0.04f}, s.angular_velocity = {0, 0.05f, 0};
    CHECK(ai::parked(s));
    s.angular_velocity = {0.031f, 0.031f, 0.031f};
    CHECK(!ai::parked(s));
}

TEST(ai_stop_driver_parks) {
    // The stop driver (ai §1.1): kind 7 stops with the wheels straight, kind 6 steering along its
    // line; throttle 0 and the brake on until the car stands still, then kind 8 (parked): the
    // brake comes off and the physics holds the clutch open.
    for (int kind : {ai::kKindStopStraight, ai::kKindStopLine}) {
        auto trk = st::make_track();
        auto course = std::make_shared<ai::AiCourse>(*trk);
        sim::TrackWorld walls(*trk);
        sim::World world(*trk, &walls);
        sim::CarSpec spec = test::synthetic_spec_car();
        ai::adapt_spec_for_ai(spec);
        auto v = sim::Vehicle::create(spec);
        sim::Vec3 pos;
        sim::Quat q;
        st::pose(20, 0, pos, q, 0.05f); // slightly off the line's heading
        v->spawn_moving(pos, q, 60 * kKmh);
        int i = world.add(std::move(v));
        ai::AiDriver drv(course, ai::course_params(trk->driver_sets(), 0, 0));
        drv.reset(world.vehicle(i).state());
        drv.set_kind(kind);
        world.vehicle(i).set_driver_kind(kind);
        std::vector<ai::CarSnapshot> snaps(1);
        bool braked = true, no_throttle = true;
        double parked_at = -1;
        float steer_at_rest = 1, brake_after = 1;
        for (int k = 0; k < 600; ++k) {
            sim::ControlInputs in = drv.update(world.vehicle(i), {double(k) * kDt, snaps, -1});
            if (drv.kind() == ai::kKindParked && parked_at < 0) parked_at = k * kDt, brake_after = in.brake;
            if (drv.kind() != ai::kKindParked) braked = braked && in.brake == 1, no_throttle = no_throttle && in.throttle == 0;
            if (drv.kind() == ai::kKindParked) steer_at_rest = std::fabs(in.steer_angle);
            world.set_inputs(i, in);
            world.vehicle(i).set_driver_kind(drv.kind());
            world.step(kDt);
        }
        std::printf("  stop driver kind %d: parked after %.2f s, brake then %.0f, wheel angle at rest %.3f rad\n", kind,
                    parked_at, brake_after, steer_at_rest);
        CHECK(braked && no_throttle && parked_at > 0.5 && parked_at < 6 && brake_after == 0);
        CHECK(drv.kind() == ai::kKindParked && ai::parked(world.vehicle(i).state()));
        if (kind == ai::kKindStopStraight) CHECK(steer_at_rest < 0.01f);
    }
}

TEST(ai_safety_respawn_rule) {
    // ai §12 (not in the original): one rule for race::Field and ai::AiRace.
    auto trk = st::make_track();
    sim::VehicleState on_road;
    on_road.position = st::at(50);
    for (auto& w : on_road.wheels) w.contact = on_road.position;
    sim::VehicleState off_road = on_road;
    off_road.position = {500, 0, 500};
    for (auto& w : off_road.wheels) w.contact = off_road.position;
    {   // off the road mesh for more than 30 ticks
        ai::SafetyRespawn r;
        bool lost = false;
        for (int k = 0; k < 30; ++k) lost = lost || r.lost(*trk, off_road, 1 + k * kDt);
        CHECK(!lost && r.lost(*trk, off_road, 2));
        r.respawned(2);
        CHECK(!r.lost(*trk, off_road, 2.1)); // the count starts again
    }
    {   // no 10 m of progress for 12 s, counted from GO; the pits do not count
        ai::SafetyRespawn r;
        CHECK(!r.lost(*trk, on_road, -5) && !r.lost(*trk, on_road, 12.0));
        CHECK(r.lost(*trk, on_road, 12.02));
        r.progress(9, 5); // under 10 m: no progress
        CHECK(r.last_progress() == 0);
        r.progress(11, 5);
        CHECK(r.last_progress() == 5 && !r.lost(*trk, on_road, 16.9) && r.lost(*trk, on_road, 17.1));
        r.hold(16);
        CHECK(!r.lost(*trk, on_road, 27.9) && r.lost(*trk, on_road, 28.1));
    }
    {   // a position that is not finite
        ai::SafetyRespawn r;
        sim::VehicleState bad = on_road;
        bad.position.x = std::nanf("");
        CHECK(r.lost(*trk, bad, 1));
    }
}

TEST(sim_scripted_ascc_block) {
    // The AI aid block's ASCC (physics §4.6, ai §9.1): a CPU car uses the block's channels where the
    // set enables them, and not the car's own; a human car uses only its own. Lifting off with the
    // wheels turned at speed, the oversteer (A) and understeer (C) channels brake wheels without
    // the pedal. Invented block values.
    test::FlatGround g;
    auto run = [&](bool cpu, bool car_ascc, u8 block_flags) {
        sim::CarSpec s = test::synthetic_spec_car(sim::Drive::FR, true);
        if (!car_ascc) s.aids.ascc_flags = 0;
        auto c = sim::Vehicle::create(s);
        c->set_cpu_driver(cpu);
        sim::ScriptedAids a;
        a.ascc_flags = block_flags;
        a.ascc_a_gain = 0.5f, a.ascc_a_thr = 0.1f, a.ascc_c_gain = 1.0f, a.ascc_c_thr = 0.05f;
        a.ascc_d_gain = 0.3f, a.ascc_e_gain = 0.3f;
        c->set_scripted_aids(a);
        c->spawn_moving({}, {}, 110 * kKmh);
        sim::ControlInputs k;
        k.steer = 0.5f;
        if (cpu) k.steer_angle = 0.12f;
        float pressure = 0;
        for (int i = 0; i < 90; ++i) {
            c->step(k, g, kDt);
            for (size_t w = 0; w < 4; ++w) pressure = std::max(pressure, c->telemetry().brake_pressure[w]);
        }
        return pressure;
    };
    const float cpu_block = run(true, false, 0x33), cpu_none = run(true, true, 0), human_own = run(false, true, 0x33),
                human_none = run(false, false, 0x33);
    std::printf("  lift-off pressure without the pedal: CPU block A+C %.2f, CPU block off (car's own on) %.2f, human "
                "own %.2f, human own off (block on) %.2f\n",
                cpu_block, cpu_none, human_own, human_none);
    CHECK(cpu_block > 0.05f && cpu_none == 0);
    CHECK(human_own > 0.05f && human_none == 0);
}
