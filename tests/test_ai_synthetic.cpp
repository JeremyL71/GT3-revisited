// ai::AiDriver without game data: the stadium course of course_fixtures.h (invented road and AI
// lines) and the spec-path car of sim_fixtures.h, stepped in a sim::World the way ai::AiRace and
// race::Field do. Behaviour tests (line following, braking for a corner, traffic, recovery,
// the countdown), then a characterization trace of AiDriver::update on scripted car states.

#include "test_framework.h"
#include "course_fixtures.h"
#include "sim_fixtures.h"

#include "ai/ai_driver.h"
#include "sim/collision.h"
#include "sim/track_walls.h"
#include "sim/vehicle.h"
#include "sim/world.h"
#include "track/track.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

using namespace gt3r;
namespace st = gt3r::test::stadium;

namespace {

constexpr double kDt = 1.0 / sim::kDefaultTickHz;
constexpr float kKmh = 1 / 3.6f;

sim::Vec3 rotate(const sim::Quat& q, const sim::Vec3& v) {
    sim::Vec3 u{q.x, q.y, q.z};
    sim::Vec3 t = u.cross(v) * 2.0f;
    return v + t * q.w + u.cross(t);
}

// Driver parameters for the synthetic car: the stadium course's own (invented) set, as a race
// picks it (ai §9.1; course_fixtures.h make_driver_sets: corner speed 70 %, no corner bonus).
ai::DriverParams tuned() { return ai::course_params(st::make_driver_sets(), 0, 0); }

// AI cars on the stadium, stepped like ai::AiRace (snapshots at the CoG, the driver's reverse
// request passed on, car-car contact in the World).
struct AiField {
    std::unique_ptr<track::Track> track = st::make_track();
    std::shared_ptr<ai::AiCourse> course = std::make_shared<ai::AiCourse>(*track);
    sim::TrackWorld walls{*track};
    sim::World world{*track, &walls};
    struct Car {
        std::unique_ptr<ai::AiDriver> driver;
        std::unique_ptr<track::LapTimer> timer;
    };
    std::vector<Car> cars;
    std::vector<ai::CarSnapshot> snaps;
    double time = 0;
    bool countdown = false; // hold every car (clutch and race hold) while time < 0

    int add(float s, float lateral, ai::DriverParams p = ai::DriverParams::builtin(0), float kmh = 0,
            sim::Drive drive = sim::Drive::FR, float extra_yaw = 0) {
        sim::CarSpec spec = test::synthetic_spec_car(drive);
        ai::adapt_spec_for_ai(spec);
        auto v = sim::Vehicle::create(spec);
        v->set_cpu_driver(true);
        v->set_scripted_aids(p.aids);
        sim::Vec3 pos;
        sim::Quat q;
        st::pose(s, lateral, pos, q, extra_yaw);
        if (kmh > 0) v->spawn_moving(pos, q, kmh * kKmh);
        else v->reset(pos, q);
        int i = world.add(std::move(v));
        Car c;
        c.driver = std::make_unique<ai::AiDriver>(course, p, i);
        c.driver->reset(world.vehicle(i).state());
        c.timer = std::make_unique<track::LapTimer>(*track);
        c.timer->set_lap1_from_go(true);
        c.timer->update(pos, 0);
        cars.push_back(std::move(c));
        snaps.resize(cars.size());
        return i;
    }
    const sim::Vehicle& vehicle(int i) const { return world.vehicle(i); }
    const ai::AiDebug& debug(int i) const { return cars[size_t(i)].driver->debug(); }
    double distance(int i) const { return cars[size_t(i)].timer->race_distance(); }
    float lateral(int i) const { return track->locate(vehicle(i).state().position, cars[size_t(i)].timer->distance()).lateral; }

    void step() {
        for (size_t i = 0; i < cars.size(); ++i) {
            const sim::Vehicle& v = world.vehicle(int(i));
            ai::CarSnapshot& s = snaps[i];
            s.position = v.state().position + rotate(v.state().orientation, v.setup().cog_offset);
            s.orientation = v.state().orientation;
            s.velocity = v.state().velocity;
            s.half_length = v.setup().length * 0.5f;
            s.half_width = v.setup().width * 0.5f;
            s.front = v.setup().cog_front_extent, s.rear = v.setup().cog_rear_extent;
        }
        for (size_t i = 0; i < cars.size(); ++i) {
            ai::AiContext ctx{time, snaps, int(i)};
            ctx.rank = int(i);
            sim::Vehicle& v = world.vehicle(int(i));
            v.set_clutch_hold(countdown && time < 0);
            v.set_race_hold(countdown && time < 0);
            world.set_inputs(int(i), cars[i].driver->update(v, ctx, kDt));
            v.set_reverse_request(cars[i].driver->reverse_request());
        }
        world.step(kDt);
        time += kDt;
        for (size_t i = 0; i < cars.size(); ++i)
            cars[i].timer->update(world.vehicle(int(i)).state().position, std::max(time, 0.0));
    }
};

} // namespace

TEST(ai_synthetic_course) {
    // The invented lines become lap lines with the arcs the control points describe; the main
    // line evaluates onto the centre of the road and the lanes 3.5 m either side of it.
    auto t = st::make_track();
    ai::AiCourse c(*t);
    CHECK(std::fabs(t->length() - st::kLength) < 1e-2f);
    for (int l : {1, 2, 6}) CHECK(c.line(l) != nullptr && c.line(l)->lap);
    CHECK(c.line(3) == nullptr && c.resolve(ai::AiCourse::kMain) == 6 && c.resolve(7) == 6 && c.resolve(3) == -1);
    float worst[3] = {};
    for (float s = 0; s < t->length(); s += 2.5f) {
        const float want[3] = {-st::kLane, st::kLane, 0};
        const int lines[3] = {1, 2, 6};
        for (int k = 0; k < 3; ++k) {
            sim::Vec3 p = c.evaluate(lines[k], s), q = st::at(s, want[k]);
            worst[k] = std::max(worst[k], std::hypot(p.x - q.x, p.z - q.z));
        }
    }
    std::printf("  stadium: %.1f m lap, lines 1/2/6 within %.3f / %.3f / %.3f m of their lanes\n", t->length(), worst[0],
                worst[1], worst[2]);
    CHECK(worst[0] < 0.1f && worst[1] < 0.1f && worst[2] < 0.1f); // (progress from the 5 m checkpoint polyline)
    const ai::AiCourse::Line* m = c.line(6);
    int arcs = 0;
    for (const auto& p : m->points) arcs += p.arc && std::fabs(p.rr - st::kRadius) < 1e-3f;
    CHECK(arcs == 4);
}

TEST(ai_synthetic_follows_line) {
    // One car, two laps: it stays on the main line (the centre of the road here), never touches
    // a wall, never needs the recovery states or reverse. With the built-in parameters it still
    // laps, running wide in the arcs.
    AiField f;
    int a = f.add(10, 0, tuned());
    float worst = 0;
    int max_state = 0, walls = 0, other_line = 0;
    bool reverse = false;
    while (f.time < 75 && f.cars[0].timer->laps_completed() < 2) {
        f.step();
        if (f.time > 5) worst = std::max(worst, std::fabs(f.lateral(a)));
        max_state = std::max(max_state, f.debug(a).state);
        walls += f.vehicle(a).telemetry().wall_impulse > 0;
        other_line += f.debug(a).line != 6;
        reverse = reverse || f.cars[0].driver->reverse_request();
    }
    const track::LapTimer& tm = *f.cars[0].timer;
    std::printf("  follows line: %d laps in %.1f s (best %.2f s), worst %.2f m off the line\n", tm.laps_completed(), f.time,
                tm.best_lap_time(), worst);
    CHECK(tm.laps_completed() == 2 && tm.best_lap_time() > 25 && tm.best_lap_time() < 40);
    CHECK(worst < 2.5f && walls == 0 && max_state == 0 && other_line == 0 && !reverse);

    AiField b;
    int c = b.add(10, 0);
    int b_state = 0;
    while (b.time < 75 && b.cars[0].timer->laps_completed() < 2) {
        b.step();
        b_state = std::max(b_state, b.debug(c).state);
    }
    std::printf("  built-in parameters: %d laps in %.1f s\n", b.cars[0].timer->laps_completed(), b.time);
    CHECK(b.cars[0].timer->laps_completed() == 2 && b_state == 0);
}

TEST(ai_synthetic_brakes_for_corner) {
    // ai §4.3: approaching an arc faster than its corner speed the driver brakes (mode 3/4,
    // brake pedal) once the braking distance reaches the next point, and enters the arc near the
    // corner speed; on the straight before, it was well above it on full throttle.
    AiField f;
    int a = f.add(10, 0, tuned());
    float top_b = 0, entry = -1, v_corner = 0, brake_from = -1;
    bool braked = false, full_throttle = false;
    while (f.time < 60) {
        f.step();
        const ai::AiDebug& g = f.debug(a);
        const float kmh = f.vehicle(a).state().speed_kmh;
        float s = g.progress;
        bool lap2 = f.cars[0].timer->laps_completed() == 1;
        if (!lap2) continue;
        if (s > st::kStraightB + 20 && s < st::kArc2 - 80) {
            top_b = std::max(top_b, kmh);
            full_throttle = full_throttle || g.throttle == 1;
        }
        if (s > st::kStraightB + 20 && s < st::kArc2 && (g.mode == 3 || g.mode == 4) && g.brake > 0) {
            if (!braked) brake_from = st::kArc2 - s, v_corner = g.next_speed / kKmh;
            braked = true;
        }
        if (entry < 0 && s >= st::kArc2 && s < st::kArc2 + 30) entry = kmh;
        if (entry >= 0) break;
    }
    std::printf("  corner: %.0f km/h on straight B, braking from %.0f m before the arc, %.0f km/h at the arc "
                "(corner speed %.0f)\n",
                top_b, brake_from, entry, v_corner);
    CHECK(full_throttle && braked && entry > 0);
    CHECK(top_b > v_corner + 20 && entry < v_corner + 8 && entry > v_corner - 12);
    CHECK(brake_from > 10 && brake_from < 120);
}

TEST(ai_synthetic_overtakes_slower_car) {
    // ai §8: a faster car closing on a slower one on the same line sees the contact zone ahead,
    // moves to a free lane (lines 1 / 2) and goes past instead of running into it. The slower car
    // starts 115 m ahead, so the faster one closes on straight B: in an arc the lane change is
    // barred while the status says "braking point near" or "partial throttle" (ai §14.1).
    AiField f;
    ai::DriverParams slow = tuned();
    slow.throttle_scale = 0.25f;
    slow.speed_scale *= 0.8f;
    int fast = f.add(5, 0, tuned(), 60), lead = f.add(120, 0, slow, 60);
    int contacts = 0, lane_ticks = 0;
    double passed = -1, first_lane = -1, first_contact = -1;
    float closing = 0;
    while (f.time < 40) {
        f.step();
        contacts += int(f.world.contacts().size());
        if (first_contact < 0 && !f.world.contacts().empty()) first_contact = f.time;
        bool lane = f.debug(fast).line == 1 || f.debug(fast).line == 2;
        if (first_lane < 0 && lane) first_lane = f.time;
        lane_ticks += lane;
        closing = std::max(closing, f.debug(fast).contact);
        if (passed < 0 && f.distance(fast) > f.distance(lead) + 5) passed = f.time;
    }
    std::printf("  overtake: contact intensity %.2f, lane change at %.2f s, first car contact at %.2f s (%d ticks), passed after "
                "%.1f s, %.0f m ahead at 40 s\n",
                closing, first_lane, first_contact, contacts, passed, f.distance(fast) - f.distance(lead));
    // It pulls out before touching; pulling out close behind, it may still brush the slower car's tail.
    CHECK(closing > 0 && lane_ticks > 30 && first_lane > 0 && (first_contact < 0 || first_lane < first_contact));
    CHECK(contacts < 20);
    CHECK(passed > 15 && passed < 30 && f.distance(fast) > f.distance(lead) + 50);
    CHECK(f.debug(fast).line == 6); // back on the main line with the road clear
}

TEST(ai_synthetic_recovers) {
    // ai §7: facing the wrong way the driver turns round (state 2) and drives on along the lap;
    // driven nose-first into the outer wall, the wall's impulse on the car's front (the front
    // flag of that collision step) sends it from turning round to reversing (state 1): it backs
    // out (reverse request), turns and drives on. No timers: the flags of each collision step
    // and the heading error alone decide.
    {
        AiField f;
        int a = f.add(50, 0, tuned(), 0, sim::Drive::FR, st::kPi);
        double d0 = f.distance(a);
        bool turned = false;
        while (f.time < 20) {
            f.step();
            turned = turned || f.debug(a).state == 2;
        }
        std::printf("  recover (backwards): state 2 %d, %.0f m along the lap after 20 s, state %d\n", int(turned),
                    f.distance(a) - d0, f.debug(a).state);
        CHECK(turned && f.debug(a).state == 0 && f.distance(a) > d0 + 100);
    }
    struct Wall {
        bool hit = false, reversed = false, front_to_reverse = false;
        int states = 0, final_state = 0;
        unsigned sides = 0;
        double moved = 0;
    };
    auto into_wall = [](float yaw) { // driven at 20 km/h into the outer wall, `yaw` from the lap direction
        AiField f;
        int a = f.add(50, -4, tuned(), 20, sim::Drive::FR, yaw);
        double d0 = f.distance(a);
        Wall w;
        int last = 0;
        while (f.time < 20) {
            unsigned hits = f.vehicle(a).telemetry().hit_sides; // the step the driver reads next
            f.step();
            w.reversed = w.reversed || f.cars[0].driver->reverse_request();
            w.hit = w.hit || f.vehicle(a).telemetry().wall_impulse > 0;
            w.sides |= f.vehicle(a).telemetry().hit_sides;
            w.states |= 1 << f.debug(a).state;
            if (last == 2 && f.debug(a).state == 1) w.front_to_reverse = w.front_to_reverse || (hits & 1u);
            last = f.debug(a).state;
        }
        w.final_state = f.debug(a).state;
        w.moved = f.distance(a) - d0;
        return w;
    };
    Wall back = into_wall(-2.0f), square = into_wall(-st::kPi / 2);
    for (const auto& [name, w] : {std::pair<const char*, const Wall&>{"115 deg", back}, {"90 deg", square}}) {
        std::printf("  recover (wall, %s): states %x, collision sides %x, front hit -> reversing %d, reversed %d, "
                    "%.0f m along the lap after 20 s\n",
                    name, w.states, w.sides, int(w.front_to_reverse), int(w.reversed), w.moved);
        CHECK(w.hit && (w.sides & 1u) && w.front_to_reverse && w.reversed);
        CHECK(w.states == 7 && w.final_state == 0 && w.moved > 100);
    }
}

TEST(ai_synthetic_countdown) {
    // ai §10: during the countdown the driver neither brakes nor steers and blips the throttle
    // around the reference rpm with the clutch held open (the car stays on its slot); at GO it
    // launches.
    AiField f;
    f.countdown = true;
    f.time = -3;
    int a = f.add(st::kLength - 10, 0, tuned());
    sim::Vec3 p0 = f.vehicle(a).state().position;
    const float up = f.vehicle(a).spec().engine.upshift_rpm;
    float rpm_lo = 1e9f, rpm_hi = 0, thr_lo = 1, thr_hi = 0;
    bool calm = true;
    while (f.time < -kDt / 2) {
        f.step();
        calm = calm && f.debug(a).brake == 0 && f.debug(a).steer_angle == 0;
        if (f.time > -2) {
            float rpm = f.vehicle(a).state().engine_rpm;
            rpm_lo = std::min(rpm_lo, rpm), rpm_hi = std::max(rpm_hi, rpm);
            thr_lo = std::min(thr_lo, f.debug(a).throttle), thr_hi = std::max(thr_hi, f.debug(a).throttle);
        }
    }
    sim::Vec3 d = f.vehicle(a).state().position - p0;
    std::printf("  countdown: rpm %.0f-%.0f (reference %.0f), throttle %.2f-%.2f, moved %.3f m\n", rpm_lo, rpm_hi, up, thr_lo,
                thr_hi, std::hypot(d.x, d.z));
    CHECK(calm && std::hypot(d.x, d.z) < 0.01f);
    // (on above 3/4 of the reference, off from 15/16 of it; the engine overshoots towards the limiter)
    CHECK(rpm_lo > 0.7f * up && rpm_hi <= f.vehicle(a).spec().engine.rev_limit_rpm + 10 && thr_lo < 0.2f && thr_hi > 0.8f);
    for (int i = 0; i < 120; ++i) f.step();
    CHECK(f.vehicle(a).state().speed_kmh > 20);
}

TEST(ai_synthetic_deterministic) {
    // update() depends only on its arguments and its own state: two identical fields stay
    // bit-identical.
    auto run = [] {
        auto f = std::make_unique<AiField>();
        ai::DriverParams slow = tuned();
        slow.throttle_scale = 0.6f;
        f->add(5, 0, tuned(), 50);
        f->add(30, 3, slow, 50);
        f->add(60, -3, ai::DriverParams::builtin(0), 50);
        for (int i = 0; i < 60 * 30; ++i) f->step();
        return f;
    };
    auto a = run(), b = run();
    bool same = true;
    for (int i = 0; i < 3; ++i) {
        const sim::VehicleState &x = a->vehicle(i).state(), &y = b->vehicle(i).state();
        same = same && x.position.x == y.position.x && x.position.z == y.position.z && x.velocity.x == y.velocity.x &&
               x.engine_rpm == y.engine_rpm && a->debug(i).steer_angle == b->debug(i).steer_angle;
    }
    CHECK(same);
}

// --- golden trace ------------------------------------------------------------------
//
// A characterization of AiDriver::update alone: the car is not simulated but posed every tick
// from a scripted state (Vehicle::sync_to), so the trace has no physics feedback that could
// amplify float noise. The script covers each part of update(): the countdown blip (race time
// < 0, rpm swept through the hysteresis), the start (look-ahead extension), lane following with
// a weave and a heading offset (target point, yaw-rate steering, saturation), a slower car
// closing in ahead after the traffic logic switches on (zones, contact intensity, the lane change,
// its blend and the return to the main line), the approach to arc 2 above its corner speed
// (braking point, mode 3, the arc mode and its throttle slew) and a spin to a stop against the
// outer wall (heading error past 80 degrees: the turn-round state; a front hit sends it to
// reversing with the reverse request, a rear hit back to turning round; the wheels straight at a
// standstill). The car is posed, so the collision steps' front / rear sides are injected. Each
// checkpoint holds the outputs and decision words at that tick plus sums of the pedals and
// the steering over the 20 ticks before it.
//
// Regenerate after a deliberate behaviour change with
//     GT3R_GOLDEN_PRINT=1 ./gt3r_tests ai_synthetic_golden
// and paste the table over kAiGolden. Float noise across compilers moves nothing here by more
// than a few 1e-6 (GT3R_GOLDEN_SENSITIVITY=1 nudges the scripted path by 1e-5 m and prints the
// worst deviation as a fraction of the tolerance), so the tolerances are tight; discrete
// words (line, mode, state, drive, zones, next point, contact level, reverse) must match exactly.

namespace {

enum AiGoldenField : int {
    kASteer, kAThrottle, kABrake, kAReverse, kALine, kAMode, kAState, kADrive, kATarget, kANext, kABrakeDist,
    kALook, kAContact, kAZones, kANextPoint, kAHeadingErr, kASumSteer, kASumThrottle, kASumBrake, kAFieldCount
};
using AiRow = std::array<double, kAFieldCount>;
struct AiTol {
    const char* name;
    double abs;
};
constexpr AiTol kAiTol[kAFieldCount] = {
    {"steer", 2e-4},    {"throttle", 2e-3},   {"brake", 2e-3},     {"reverse", 0},     {"line", 0},
    {"mode", 0},        {"state", 0},         {"drive", 0},        {"target", 1e-3},   {"next", 1e-3},
    {"brake_dist", 0.02}, {"lookahead", 0.01}, {"contact", 0},     {"zones", 0},       {"next_point", 0},
    {"heading_err", 2e-4}, {"sum_steer", 4e-3}, {"sum_throttle", 0.04}, {"sum_brake", 0.04},
};

// The scripted run: tick i, `nudge` metres added to the lateral offset (sensitivity check).
//   t 0-1 s    countdown, parked 3 m left of the line in arc 1 (rpm swept)
//   t 1-6 s    the start: 0 to 108 km/h out of arc 1, weaving 1.5 m about the line
//   t 6-9 s    108 km/h on straight B; a car 12 m/s slower on the main line from 6.2 s, 30 m ahead
//              closing to 6 m
//   t 9-11.7 s 108 km/h on towards arc 2 (past its braking point), then down to 65 km/h by 12.7 s
//   t 12.7-13 s into the arc
//   t 13-19 s  the spin: turned 150 degrees right, sliding to a stop against the outer wall
//              (turn-round state; a front hit at 13.9 s: reversing; a rear hit at 17 s: turning
//              round again)
std::vector<AiRow> scripted_ai_run(float nudge = 0) {
    auto trk = st::make_track();
    auto course = std::make_shared<ai::AiCourse>(*trk);
    sim::CarSpec spec = test::synthetic_spec_car();
    ai::adapt_spec_for_ai(spec);
    auto car = sim::Vehicle::create(spec);
    car->set_cpu_driver(true);
    ai::AiDriver drv(course, tuned(), 0);
    const int ticks = 1140, every = 20;
    float s = 200, prev_heading = 0; // the car's distance along the lap, integrated from the scripted speed
    std::vector<AiRow> rows;
    AiRow sums{};
    std::vector<ai::CarSnapshot> snaps(2);
    for (int i = 0; i < ticks; ++i) {
        const float t = float(i) / 60.0f;
        const double race_time = double(i) / 60.0 - 1.0;
        float v = 0; // m/s
        if (t >= 1 && t < 6) v = (t - 1) / 5 * 30;
        else if (t >= 6 && t < 11.7f) v = 30;
        else if (t >= 11.7f && t < 13) v = 30 - std::min(1.0f, t - 11.7f) * 12;
        else if (t >= 13) v = std::max(0.0f, 18 - (t - 13) * 20);
        s += v / 60.0f;
        float lat = (t >= 1 ? 1.5f * std::sin(1.1f * (t - 1)) : 3.0f) + nudge; // left +
        float off = t >= 1 ? 0.03f * std::sin(1.7f * (t - 1)) : 0;             // heading offset, left +
        if (t >= 13) {
            lat = std::max(-7.0f, 1.5f * std::sin(1.1f * 12) - (t - 13) * 6);
            off = -std::min(1.0f, (t - 13) * 2) * 2.618f;
        }
        sim::Vec3 pos, c, d;
        sim::Quat q;
        st::pose(s, lat, pos, q, off);
        st::centre(s, c, d);
        float heading = std::atan2(-d.x, -d.z) + off;
        float yaw_rate = i == 0 ? 0 : std::remainder(heading - prev_heading, 2 * st::kPi) * 60.0f;
        prev_heading = heading;
        sim::VehicleState vs;
        vs.position = pos;
        vs.orientation = q;
        vs.velocity = d * v; // sliding along the lap direction whatever the heading
        vs.angular_velocity = {0, yaw_rate, 0};
        vs.gear = t < 1 ? 1 : 3;
        vs.engine_rpm = t < 1 ? 5000 + 2500 * std::sin(6 * t) : 3000 + v * 100;
        for (sim::WheelState& w : vs.wheels) w.grounded = true;
        car->sync_to(vs, yaw_rate);
        if (t >= 13.9f && t < 14.0f) car->add_hit_sides(sim::collide::kFrontHit); // the nose on the wall
        if (t >= 17.0f && t < 17.05f) car->add_hit_sides(sim::collide::kRearHit); // backing into it
        const sim::VehicleState& me = car->state();
        snaps[0].position = me.position + rotate(me.orientation, car->setup().cog_offset);
        snaps[0].orientation = me.orientation;
        snaps[0].velocity = me.velocity;
        snaps[0].front = car->setup().cog_front_extent, snaps[0].rear = car->setup().cog_rear_extent;
        snaps[0].half_width = car->setup().width * 0.5f;
        float so = s + std::max(6.0f, 30 - (t - 6.2f) * 12);
        sim::Vec3 op, od;
        sim::Quat oq;
        st::pose(so, 0, op, oq);
        st::centre(so, op, od);
        snaps[1].position = op;
        snaps[1].orientation = oq;
        snaps[1].velocity = od * std::max(0.0f, v - 12);
        snaps[1].front = snaps[0].front, snaps[1].rear = snaps[0].rear, snaps[1].half_width = snaps[0].half_width;
        snaps[1].active = t >= 6.2f && t < 9;
        ai::AiContext ctx{race_time, snaps, 0};
        sim::ControlInputs in = drv.update(*car, ctx);
        const ai::AiDebug& g = drv.debug();
        sums[kASumSteer] += in.steer_angle;
        sums[kASumThrottle] += in.throttle;
        sums[kASumBrake] += in.brake;
        if (i % every == every - 1) {
            AiRow r = sums;
            r[kASteer] = in.steer_angle;
            r[kAThrottle] = in.throttle;
            r[kABrake] = in.brake;
            r[kAReverse] = drv.reverse_request();
            r[kALine] = g.line;
            r[kAMode] = g.mode;
            r[kAState] = g.state;
            r[kADrive] = g.drive;
            r[kATarget] = g.target_speed;
            r[kANext] = g.next_speed;
            r[kABrakeDist] = g.brake_distance;
            r[kALook] = g.lookahead;
            r[kAContact] = g.contact;
            r[kAZones] = g.zones;
            r[kANextPoint] = g.next_point;
            r[kAHeadingErr] = g.heading_error;
            rows.push_back(r);
            sums = {};
        }
    }
    return rows;
}

// Generated with GT3R_GOLDEN_PRINT=1 from the code at the time the tests were written.
const std::vector<AiRow> kAiGolden = {
    {0, 0, 0, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 55, 0, 0,
     4, -0.5592176, 0, 3.083333, 0},
    {0, 0.7499999, 0, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 55, 0, 0,
     4, -0.5592176, 0, 3.75, 0},
    {0, 1, 0, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 55, 0, 0,
     4, -0.5592176, 0, 19.75, 0},
    {-0.04080261, 0.3333334, 0, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 49.61655, 0, 0,
     4, -0.5386313, -0.8277459, 3.5, 0},
    {-0.00705246, 0.6666667, 0, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 43.9493, 0, 0,
     4, -0.4690385, -0.4063341, 10.16667, 0},
    {-0.02804197, 0.9999997, 0, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 38.28202, 0, 0,
     4, -0.4100232, -0.2342978, 16.83333, 0},
    {-0.08040327, 1, 0, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 32.61557, 0, 0,
     4, -0.3654097, -1.085664, 20, 0},
    {-0.1480211, 1, 0, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 26.94976, 0, 0,
     4, -0.3375676, -2.28833, 20, 0},
    {-0.1670211, 0.6710985, 0, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 21.2832, 0, 0,
     4, -0.329712, -3.169754, 16.58863, 0},
    {-0.1836771, 0.3377652, 0, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 21.9484, 0, 0,
     4, -0.3651738, -3.521772, 9.921971, 0},
    {-0.1940323, 0.1758692, 0, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 22.94658, 0, 0,
     4, -0.3988812, -3.796235, 4.418977, 0},
    {-0.1949255, 0.0975349, 0, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 23.94647, 0, 0,
     4, -0.4235338, -3.906825, 2.927693, 0},
    {-0.1862656, 0, 1, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 24.94838, 0, 0,
     4, -0.4364503, -3.821855, 0.6116475, 6},
    {-0.1709743, 0, 1, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 25.94997, 0, 0,
     4, -0.407841, -3.57205, 0, 20},
    {-0.068323, 0, 1, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 26.94881, 0, 0,
     4, -0.3188627, -2.810473, 0, 20},
    {-0.001197373, 0, 1, 0, 2, 1, 0,
     1, 18.45041, 75.59345, 0, 27.94559, 0, 0,
     4, -0.1694284, -0.2453204, 0, 20},
    {-0.1095774, 0, 0, 0, 2, 0, 0,
     1, 75.59345, 75.59345, 0, 28.94374, 0, 0,
     5, -0.1369976, -1.810062, 0.4096448, 2},
    {-0.1139116, 0, 0, 0, 2, 0, 0,
     1, 75.59345, 75.59345, 0, 29.94553, 0, 0,
     5, -0.1272216, -2.221402, 0, 0},
    {-0.09353186, 1, 0, 0, 6, 0, 0,
     1, 75.59345, 75.59345, 0, 29.99904, 0, 0,
     5, -0.009754268, -2.075657, 12, 0},
    {-0.07173005, 1, 0, 0, 6, 0, 0,
     1, 75.59345, 75.59345, 0, 29.99978, 0, 0,
     5, -0.008784108, -1.677957, 20, 0},
    {-0.01873823, 1, 0, 0, 6, 0, 0,
     1, 75.59345, 75.59345, 0, 29.99689, 0, 0,
     5, -0.005676687, -0.4642122, 20, 0},
    {-0.05010569, 1, 0, 0, 2, 0, 0,
     1, 75.59345, 75.59345, 0, 29.99369, 1, 5,
     6, -0.03121005, -0.5315472, 20, 0},
    {-0.08104854, 0, 0, 0, 2, 0, 0,
     1, 75.59345, 75.59345, 0, 29.99388, 1, 5,
     6, -0.05760577, -1.32282, 12, 0},
    {-0.1216596, 0, 0, 0, 2, 0, 0,
     1, 75.59345, 75.59345, 0, 29.99723, 1, 5,
     6, -0.08070636, -2.020013, 0, 0},
    {-0.1069611, 0, 0, 0, 2, 0, 0,
     1, 75.59345, 75.59345, 0, 29.99989, 1, 5,
     6, -0.07133614, -2.348011, 0, 0},
    {-0.1109284, 0, 0, 0, 2, 0, 0,
     1, 75.59345, 75.59345, 0, 29.99878, 1, 5,
     6, -0.06145635, -2.123047, 0, 0},
    {-0.1155069, 0, 0, 0, 2, 0, 0,
     1, 75.59345, 75.59345, 0, 29.99519, 1, 5,
     7, -0.06115021, -2.359008, 0, 0},
    {0.06352227, 1, 0, 0, 6, 0, 0,
     1, 75.59345, 75.59345, 0, 29.99325, 0, 0,
     7, 0.04364917, 0.8066507, 19.73791, 0},
    {-0.01653085, 1, 0, 0, 6, 0, 0,
     1, 75.59345, 75.59345, 0, 29.9952, 0, 0,
     7, 0.0207867, 0.5315024, 20, 0},
    {-0.09333789, 1, 0, 0, 6, 0, 0,
     1, 75.59345, 75.59345, 0, 29.99879, 0, 0,
     7, -0.009377664, -1.157326, 20, 0},
    {-0.1413313, 0.935027, 0, 0, 6, 0, 0,
     1, 75.59345, 75.59345, 0, 29.99989, 0, 0,
     7, -0.04056253, -2.477491, 19.76395, 0},
    {-0.1571704, 0.8635306, 0, 0, 6, 0, 0,
     1, 75.59345, 19.14396, 20.15023, 29.99722, 0, 0,
     8, -0.06565289, -2.998246, 17.57512, 0},
    {-0.1669552, 1, 0, 0, 6, 0, 0,
     1, 75.59345, 19.14396, 20.13504, 29.99387, 0, 0,
     8, -0.07856134, -3.264099, 18.4233, 0},
    {-0.1691579, 0.9663612, 0, 0, 6, 0, 0,
     1, 75.59345, 19.14396, 20.13425, 29.99369, 0, 0,
     8, -0.09543731, -3.220676, 19.95771, 0},
    {-0.161559, 0, 1, 0, 6, 3, 0,
     1, 75.59345, 19.14396, 0, 29.9969, 0, 0,
     8, -0.1616983, -3.317561, 3.562479, 16},
    {-0.147381, 0, 1, 0, 6, 3, 0,
     1, 75.59345, 19.14396, 0, 28.29981, 0, 0,
     8, -0.2585246, -3.089805, 0, 20},
    {-0.0173587, 0, 1, 0, 6, 1, 0,
     1, 19.14396, 19.14396, 5.449839, 26.29927, 0, 0,
     9, -0.2581731, -1.551629, 0, 20},
    {-0.003605307, 0.1333333, 0, 0, 6, 1, 0,
     1, 19.14396, 19.14396, 0, 24.29721, 0, 0,
     9, -0.2035201, -0.1946281, 0.6, 12},
    {0.003731101, 0.4666668, 0, 0, 6, 1, 0,
     1, 19.14396, 19.14396, 0, 23.99596, 0, 0,
     9, -0.1743025, 0.01869576, 6.166668, 0},
    {-0.3104283, 0.2666667, 1, 0, 6, 1, 2,
     0, 19.14396, 19.14396, 0, 40, 0, 0,
     9, -2.081747, -3.328771, 6.300001, 6},
    {0.2131707, 0, 1, 0, 6, 1, 2,
     0, 19.14396, 19.14396, 0, 40, 0, 0,
     9, -3.087048, -0.7107762, 2, 20},
    {0.3403392, 0.1166667, 0, 1, 6, 1, 1,
     -1, 19.14396, 19.14396, 0, 10, 0, 0,
     9, 3.121173, 8.052105, 0.4666667, 13},
    {0, 0.4500002, 0, 1, 6, 1, 1,
     -1, 19.14396, 19.14396, 0, 10, 0, 0,
     9, 2.987375, 2.042035, 5.833334, 0},
    {0, 0.7833332, 0, 1, 6, 1, 1,
     -1, 19.14396, 19.14396, 0, 10, 0, 0,
     9, 2.987375, 0, 12.5, 0},
    {0, 1, 0, 1, 6, 1, 1,
     -1, 19.14396, 19.14396, 0, 10, 0, 0,
     9, 2.987375, 0, 18.7, 0},
    {0, 1, 0, 1, 6, 1, 1,
     -1, 19.14396, 19.14396, 0, 10, 0, 0,
     9, 2.987375, 0, 20, 0},
    {0, 1, 0, 1, 6, 1, 1,
     -1, 19.14396, 19.14396, 0, 10, 0, 0,
     9, 2.987375, 0, 20, 0},
    {0, 1, 0, 1, 6, 1, 1,
     -1, 19.14396, 19.14396, 0, 10, 0, 0,
     9, 2.987375, 0, 20, 0},
    {0, 1, 0, 1, 6, 1, 1,
     -1, 19.14396, 19.14396, 0, 10, 0, 0,
     9, 2.987375, 0, 20, 0},
    {0, 1, 0, 1, 6, 1, 1,
     -1, 19.14396, 19.14396, 0, 10, 0, 0,
     9, 2.987375, 0, 20, 0},
    {0, 1, 0, 1, 6, 1, 1,
     -1, 19.14396, 19.14396, 0, 10, 0, 0,
     9, 2.987375, 0, 20, 0},
    {0, 1, 0, 0, 6, 1, 2,
     1, 19.14396, 19.14396, 0, 40, 0, 0,
     9, 3.111669, 0, 20, 0},
    {0, 1, 0, 0, 6, 1, 2,
     1, 19.14396, 19.14396, 0, 40, 0, 0,
     9, 3.111669, 0, 20, 0},
    {0, 1, 0, 0, 6, 1, 2,
     1, 19.14396, 19.14396, 0, 40, 0, 0,
     9, 3.111669, 0, 20, 0},
    {0, 1, 0, 0, 6, 1, 2,
     1, 19.14396, 19.14396, 0, 40, 0, 0,
     9, 3.111669, 0, 20, 0},
    {0, 1, 0, 0, 6, 1, 2,
     1, 19.14396, 19.14396, 0, 40, 0, 0,
     9, 3.111669, 0, 20, 0},
    {0, 1, 0, 0, 6, 1, 2,
     1, 19.14396, 19.14396, 0, 40, 0, 0,
     9, 3.111669, 0, 20, 0},
};

double ai_tolerance(int f, double expected) { return kAiTol[f].abs + 1e-4 * std::fabs(expected); }

} // namespace

TEST(ai_synthetic_golden) {
    std::vector<AiRow> rows = scripted_ai_run();
    if (std::getenv("GT3R_GOLDEN_PRINT")) {
        std::printf("const std::vector<AiRow> kAiGolden = {\n");
        for (const AiRow& r : rows)
            for (int f = 0; f < kAFieldCount; ++f)
                std::printf("%s%.7g%s", f == 0 ? "    {" : f % 7 == 0 ? "     " : "", r[size_t(f)],
                            f + 1 == kAFieldCount ? "},\n" : f % 7 == 6 ? ",\n" : ", ");
        std::printf("};\n");
    }
    if (std::getenv("GT3R_GOLDEN_SENSITIVITY")) {
        for (float nudge : {1e-5f, -1e-5f}) {
            std::vector<AiRow> p = scripted_ai_run(nudge);
            double worst = 0;
            int wf = 0, wk = 0;
            for (size_t k = 0; k < rows.size(); ++k)
                for (int f = 0; f < kAFieldCount; ++f) {
                    double d = std::fabs(p[k][size_t(f)] - rows[k][size_t(f)]);
                    double r = kAiTol[f].abs == 0 ? (d == 0 ? 0 : 1e9) : d / ai_tolerance(f, rows[k][size_t(f)]);
                    if (r > worst) worst = r, wf = f, wk = int(k + 1) * 20;
                }
            std::printf("  ai sensitivity (lateral %+.0e m): worst %.3f of the tolerance (%s, tick %d)\n", double(nudge), worst,
                        kAiTol[wf].name, wk);
        }
    }
    CHECK(rows.size() == 57);
    if (kAiGolden.empty()) {
        std::printf("  no golden values yet\n");
        CHECK(false);
        return;
    }
    CHECK(rows.size() == kAiGolden.size());
    int bad = 0;
    for (size_t k = 0; k < rows.size() && k < kAiGolden.size(); ++k)
        for (int f = 0; f < kAFieldCount; ++f)
            if (std::fabs(rows[k][size_t(f)] - kAiGolden[k][size_t(f)]) > ai_tolerance(f, kAiGolden[k][size_t(f)])) {
                if (++bad <= 10)
                    std::printf("  FAIL ai golden tick %d %s: %.9g, golden %.9g\n", int(k + 1) * 20, kAiTol[f].name,
                                rows[k][size_t(f)], kAiGolden[k][size_t(f)]);
            }
    CHECK(bad == 0);
}
