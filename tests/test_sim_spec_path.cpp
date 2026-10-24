// sim::Vehicle on the spec path without game data: the car of sim_fixtures.h carries both
// suspension blocks, so Vehicle::tick runs what every disc car runs (spec planar step,
// attitude, phase-12 suspension order, spec driveline), unlike CarSpec::test_car, which
// takes the placeholder rigid body. Property tests first, then characterization ("golden
// trace") tests that pin the current behaviour through scripted inputs, so a
// behaviour-preserving restructuring of Vehicle::tick can be checked in CI.

#include "test_framework.h"
#include "sim_fixtures.h"

#include "sim/models.h"
#include "sim/vehicle.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

using namespace gt3r;
using namespace gt3r::sim;
using test::EdgeGround;
using test::FlatGround;
using test::Slope;
using test::synthetic_spec_car;

namespace {

constexpr double kDt = 1.0 / kDefaultTickHz;
constexpr float kG = model::kGravity;

Quat yaw_quat(float yaw) { return {std::cos(yaw / 2), 0, std::sin(yaw / 2), 0}; } // + = nose towards -X (left)

Vec3 rotate(const Quat& q, const Vec3& v) {
    Vec3 u{q.x, q.y, q.z};
    Vec3 t = u.cross(v) * 2.0f;
    return v + t * q.w + u.cross(t);
}

Vec3 forward(const VehicleState& s) { return rotate(s.orientation, {0, 0, -1}); }
float forward_speed(const VehicleState& s) { return s.velocity.dot(forward(s)); }
float heading(const VehicleState& s) { // yaw angle: 0 = nose along -Z, + = turned left
    Vec3 f = forward(s);
    return std::atan2(-f.x, -f.z);
}
float load_sum(const VehicleState& s) { return s.wheels[0].load + s.wheels[1].load + s.wheels[2].load + s.wheels[3].load; }
int grounded_count(const VehicleState& s) {
    int n = 0;
    for (const WheelState& w : s.wheels) n += w.grounded;
    return n;
}

bool finite_state(const VehicleState& s) {
    auto f = [](const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); };
    bool ok = f(s.position) && f(s.velocity) && f(s.angular_velocity) && std::isfinite(s.engine_rpm);
    for (const auto& w : s.wheels) ok = ok && std::isfinite(w.load) && std::isfinite(w.slip_ratio) && f(w.contact);
    return ok;
}

// Flat ground with a grass verge (material 2) beyond x > grass_x and wet tarmac (flag 2) for
// z < wet_z.
struct PatchGround : TrackQuery {
    float grass_x = 1e9f, wet_z = -1e9f;
    u32 surface = 0; // everywhere else
    SurfaceHit probe(float x, float z, float y_max) const override {
        SurfaceHit h;
        h.hit = y_max >= 0;
        h.surface = x > grass_x ? 2u : z < wet_z ? 0x200u : surface;
        return h;
    }
};

// A kicker ramp rising along -Z from z = start to z = end (height `top`), then a drop back to
// the flat ground.
struct Kicker : TrackQuery {
    float start = -20, end = -26, top = 1.0f;
    SurfaceHit probe(float, float z, float y_max) const override {
        SurfaceHit h;
        if (z <= start && z > end) {
            float grade = top / (start - end);
            h.height = (start - z) * grade;
            h.normal = Vec3{0, 1, grade} * (1.0f / std::sqrt(1 + grade * grade));
        }
        h.hit = y_max >= h.height;
        return h;
    }
};

// Wall segments for the spec's swept octagon (track on the LEFT of a -> b).
struct SegmentWalls : WorldQuery {
    std::vector<collide::Segment> segs;
    bool wall_contact(const Vec3&, float, Vec3&, float&) const override { return false; }
    bool wall_segments(const Vec3&, float, std::vector<collide::Segment>& out) const override {
        out.insert(out.end(), segs.begin(), segs.end());
        return true;
    }
    void add(float ax, float az, float bx, float bz) {
        collide::Segment s;
        s.a = {ax, az};
        s.b = {bx, bz};
        segs.push_back(s);
    }
};

// The sphere fallback (a world without wall segments): a wall across the road at z = wall_z.
struct SphereWall : WorldQuery {
    float wall_z = -40;
    bool wall_contact(const Vec3& c, float r, Vec3& n, float& pen) const override {
        float d = c.z - wall_z;
        if (d >= r) return false;
        n = {0, 0, 1};
        pen = r - d;
        return true;
    }
};

bool same_state(const VehicleState& x, const VehicleState& y) {
    auto v = [](const Vec3& p, const Vec3& q) { return p.x == q.x && p.y == q.y && p.z == q.z; };
    bool ok = v(x.position, y.position) && v(x.velocity, y.velocity) && v(x.angular_velocity, y.angular_velocity) &&
              x.orientation.w == y.orientation.w && x.orientation.x == y.orientation.x &&
              x.orientation.y == y.orientation.y && x.orientation.z == y.orientation.z && x.engine_rpm == y.engine_rpm &&
              x.gear == y.gear && x.throttle == y.throttle && x.speed_kmh == y.speed_kmh && x.rev_limiter == y.rev_limiter;
    for (size_t i = 0; i < 4; ++i) {
        const WheelState &p = x.wheels[i], &q = y.wheels[i];
        ok = ok && v(p.contact, q.contact) && p.steer_angle == q.steer_angle && p.spin_angle == q.spin_angle &&
             p.angular_velocity == q.angular_velocity && p.suspension == q.suspension && p.slip_ratio == q.slip_ratio &&
             p.slip_angle == q.slip_angle && p.load == q.load && p.grounded == q.grounded && p.surface == q.surface;
    }
    return ok;
}

bool same_telemetry(const VehicleTelemetry& a, const VehicleTelemetry& b) {
    bool ok = a.engine_torque == b.engine_torque && a.clutch_state == b.clutch_state && a.pitch == b.pitch &&
              a.roll == b.roll && a.pitch_rate == b.pitch_rate && a.roll_rate == b.roll_rate && a.ground_u == b.ground_u &&
              a.steer_angle == b.steer_angle && a.g_over_mu == b.g_over_mu && a.yaw_acc == b.yaw_acc &&
              a.boost == b.boost && a.turbo_load == b.turbo_load && a.turbo_lag == b.turbo_lag &&
              a.understeer == b.understeer && a.oversteer == b.oversteer && a.tcs_factor == b.tcs_factor &&
              a.diff_locked == b.diff_locked && a.diff_half == b.diff_half && a.wall_impulse == b.wall_impulse;
    for (size_t i = 0; i < 4; ++i)
        ok = ok && a.fx[i] == b.fx[i] && a.fy[i] == b.fy[i] && a.long_force[i] == b.long_force[i] &&
             a.peak_force[i] == b.peak_force[i] && a.brake_pressure[i] == b.brake_pressure[i] && a.beta[i] == b.beta[i] &&
             a.smoke[i] == b.smoke[i] && a.brake_heat[i] == b.brake_heat[i];
    return ok;
}

} // namespace

// --- which path ------------------------------------------------------------------

TEST(sim_spec_path_taken) {
    // Observed without a hook: only the spec planar step writes the per-wheel tyre words
    // (telemetry peak_force / slip_gain), and VehicleSetup reports a ride height only for an
    // axle with the spec suspension block. The hand-built test car shows neither.
    FlatGround g;
    for (Drive d : {Drive::FR, Drive::FF, Drive::MR, Drive::RR, Drive::AWD}) {
        CarSpec s = synthetic_spec_car(d);
        CHECK(s.axle[0].susp.valid && s.axle[1].susp.valid);
        auto car = Vehicle::create(s);
        car->reset({}, {});
        car->step({}, g, kDt);
        for (size_t i = 0; i < 4; ++i) {
            CHECK(car->setup().wheels[i].ride_height > 0);
            CHECK(car->telemetry().peak_force[i] > 0 && car->telemetry().slip_gain[i] > 0);
        }
    }
    auto placeholder = Vehicle::create(CarSpec::test_car());
    placeholder->reset({}, {});
    placeholder->step({}, g, kDt);
    for (size_t i = 0; i < 4; ++i)
        CHECK(placeholder->setup().wheels[i].ride_height == 0 && placeholder->telemetry().peak_force[i] == 0);
}

// --- properties --------------------------------------------------------------------

TEST(sim_spec_rest_loads_equal_weight) {
    // Standing start, no inputs: the car stays at its static pose; the tyre loads carry the
    // weight with the axle split of front_weight, and each wheel sits at x_s (physics §4.2).
    for (Drive d : {Drive::FR, Drive::FF, Drive::AWD}) {
        CarSpec s = synthetic_spec_car(d);
        auto car = Vehicle::create(s);
        FlatGround g;
        car->reset({3, 0, -7}, yaw_quat(0.4f));
        for (int i = 0; i < 180; ++i) car->step({}, g, kDt);
        const VehicleState& st = car->state();
        // the weight split applies to the sprung mass; each wheel adds its own unsprung weight
        const float uf = s.axle[0].susp.unsprung, ur = s.axle[1].susp.unsprung, sprung = s.mass - 2 * (uf + ur);
        float w = s.mass * kG, total = load_sum(st), front = st.wheels[0].load + st.wheels[1].load;
        CHECK(std::fabs(total - w) < 1e-3f * w);
        CHECK(std::fabs(front - (sprung * s.front_weight + 2 * uf) * kG) < 1e-3f * w);
        CHECK(std::fabs(st.wheels[0].load - st.wheels[1].load) < 1 && std::fabs(st.wheels[2].load - st.wheels[3].load) < 1);
        for (size_t i = 0; i < 4; ++i) {
            CHECK(st.wheels[i].grounded);
            CHECK(std::fabs(st.wheels[i].suspension - car->setup().wheels[i].static_suspension) < 1e-4f);
        }
        CHECK(std::fabs(st.position.x - 3) < 1e-3f && std::fabs(st.position.y) < 1e-3f && std::fabs(st.position.z + 7) < 1e-3f);
        CHECK(st.speed_kmh < 0.01f && std::fabs(heading(st) - 0.4f) < 1e-4f);
        CHECK(std::fabs(st.engine_rpm - s.engine.idle_rpm) < 1);
    }
    // On a 15 % grade the brakes hold the car (the stop hold zeroes the planar velocity before
    // the move; the published velocity keeps the heave along the tilted body, so the check is
    // on the position).
    auto car = Vehicle::create(synthetic_spec_car());
    Slope slope;
    car->reset({0, 0, 0}, {});
    ControlInputs in;
    in.brake = 1;
    for (int i = 0; i < 120; ++i) car->step(in, slope, kDt);
    Vec3 p0 = car->state().position;
    for (int i = 0; i < 120; ++i) car->step(in, slope, kDt);
    Vec3 d = car->state().position - p0;
    std::printf("  rest: slope hold moved %.2g m along the slope\n", std::hypot(d.x, d.z));
    CHECK(std::hypot(d.x, d.z) < 1e-4f && std::fabs(d.y) < 1e-3f);
}

TEST(sim_spec_accelerates_and_upshifts) {
    // Full throttle from rest: the clutch launches (state 2) and locks, every upshift goes
    // through the shift clutch (state 3) and locks again, gears only go up, the engine
    // never runs past the limiter on a locked clutch, and the turbo builds boost.
    CarSpec s = synthetic_spec_car(Drive::FR);
    auto car = Vehicle::create(s);
    FlatGround g;
    car->reset({}, {});
    ControlInputs in;
    in.throttle = 1;
    int gear = 1, upshifts = 0, through_shift_clutch = 0;
    bool launched = false, monotonic = true, grounded = true, rpm_ok = true;
    float t100 = -1, max_boost = 0;
    for (int i = 0; i < 60 * 25; ++i) {
        car->step(in, g, kDt);
        const VehicleState& st = car->state();
        const VehicleTelemetry& t = car->telemetry();
        launched = launched || t.clutch_state == 2;
        if (st.gear != gear) {
            monotonic = monotonic && st.gear == gear + 1;
            through_shift_clutch += t.clutch_state == 3;
            ++upshifts;
            gear = st.gear;
        }
        if (t.clutch_locked) rpm_ok = rpm_ok && st.engine_rpm <= s.engine.rev_limit_rpm + 10.5f;
        grounded = grounded && grounded_count(st) == 4;
        max_boost = std::max(max_boost, t.boost);
        if (t100 < 0 && st.speed_kmh >= 100) t100 = float(i + 1) / 60.0f;
    }
    const VehicleState& st = car->state();
    std::printf("  accel: 0-100 %.2f s, %d upshifts, gear %d, %.0f km/h after 25 s, boost max %.2f\n", t100, upshifts, st.gear,
                st.speed_kmh, max_boost);
    CHECK(launched && monotonic && upshifts >= 3 && through_shift_clutch == upshifts);
    CHECK(t100 > 5 && t100 < 14);
    CHECK(st.speed_kmh > 150);
    CHECK(rpm_ok && grounded && max_boost > 0.3f && max_boost <= s.engine.turbo.cap - 1 + 1e-5f);
    CHECK(std::fabs(st.position.x) < 0.05f && st.position.z < -500); // straight ahead (-Z)
}

TEST(sim_spec_brakes_to_stop_and_holds) {
    // Full brake from 100 km/h: ABS acts, the car stops in a plausible distance, and with the
    // brake held the stop hold keeps it exactly still; released, it does not creep.
    CarSpec s = synthetic_spec_car(Drive::FR);
    auto car = Vehicle::create(s);
    FlatGround g;
    car->spawn_moving({}, {}, 100 / 3.6f);
    ControlInputs in;
    in.brake = 1;
    bool abs = false;
    int ticks = 0;
    for (; ticks < 600 && forward_speed(car->state()) > 0.01f; ++ticks) {
        car->step(in, g, kDt);
        abs = abs || car->telemetry().abs_active;
    }
    float dist = -car->state().position.z;
    std::printf("  brake: 100-0 km/h in %.1f m, %d ticks, ABS %d\n", dist, ticks, int(abs));
    CHECK(ticks < 600 && abs);
    CHECK(dist > 25 && dist < 60);
    // The CoG (the collision body's position) stays put; the published origin still moves with
    // the pitch while the body settles.
    collide::V2 p0 = car->collision_body().pos;
    for (int i = 0; i < 120; ++i) car->step(in, g, kDt);
    collide::V2 p1 = car->collision_body().pos;
    in.brake = 0;
    for (int i = 0; i < 120; ++i) car->step(in, g, kDt);
    collide::V2 p2 = car->collision_body().pos;
    double held = std::hypot(p1.x - p0.x, p1.z - p0.z), released = std::hypot(p2.x - p1.x, p2.z - p1.z);
    std::printf("  brake: CoG moved %.2g m held, %.2g m released\n", held, released);
    CHECK(held < 1e-4 && released < 1e-4);
}

TEST(sim_spec_cornering_direction) {
    // Steer right: clockwise yaw seen from above (negative about +Y), lateral acceleration to
    // the right (+X), the car moves to +X and the left (outer) wheels carry more load (spec
    // physics §4.2 transfer). Steering left mirrors it.
    struct Run {
        float yaw_rate = 0, a_lat = 0, x = 0, left = 0, right = 0, kmh = 0;
    };
    auto run = [](float steer) {
        auto car = Vehicle::create(synthetic_spec_car(Drive::FR));
        FlatGround g;
        car->spawn_moving({}, {}, 60 / 3.6f);
        ControlInputs in;
        in.steer = steer;
        in.throttle = 0.3f;
        for (int i = 0; i < 60 * 4; ++i) car->step(in, g, kDt);
        const VehicleState& st = car->state();
        Run r;
        r.yaw_rate = st.angular_velocity.y;
        r.a_lat = car->telemetry().accel_local.x;
        r.x = st.position.x;
        r.left = st.wheels[0].load + st.wheels[2].load;
        r.right = st.wheels[1].load + st.wheels[3].load;
        r.kmh = st.speed_kmh;
        return r;
    };
    Run right = run(0.5f), left = run(-0.5f);
    std::printf("  corner: right yaw %.3f rad/s a_y %.2f m/s2 loads L/R %.0f/%.0f; left yaw %.3f a_y %.2f, %.1f km/h\n",
                right.yaw_rate, right.a_lat, right.left, right.right, left.yaw_rate, left.a_lat, right.kmh);
    CHECK(right.yaw_rate < -0.1f && right.a_lat > 2 && right.x > 5 && right.left > right.right * 1.3f);
    CHECK(left.yaw_rate > 0.1f && left.a_lat < -2 && left.x < -5 && left.right > left.left * 1.3f);
    // mirror symmetry (toe, camber and the driveline are symmetric)
    CHECK(std::fabs(right.yaw_rate + left.yaw_rate) < 0.01f * std::fabs(right.yaw_rate));
    CHECK(std::fabs(right.a_lat + left.a_lat) < 0.01f * std::fabs(right.a_lat));
    CHECK(std::fabs(right.x + left.x) < 0.01f * std::fabs(right.x));
}

TEST(sim_spec_airborne_and_landing) {
    // Off a kicker at speed: all four wheels leave the ground (no load), the yaw rate is
    // frozen and the car falls under gravity (physics §10.5/physics §10.6); it lands on four wheels and
    // settles back to its static ride height.
    CarSpec s = synthetic_spec_car(Drive::FR);
    auto car = Vehicle::create(s);
    Kicker k;
    car->spawn_moving({0.5f, 0, 0}, {}, 80 / 3.6f);
    ControlInputs in;
    in.throttle = 0.3f;
    in.steer = 0.05f; // a small yaw rate to see it frozen in the air
    int air = 0, first_air = -1, landed = -1;
    bool no_tyre_force = true, yaw_frozen = true, falling = true;
    double yaw_air = 0;
    float vy_prev = 0;
    for (int i = 0; i < 60 * 4; ++i) {
        car->step(in, k, kDt);
        const VehicleState& st = car->state();
        const VehicleTelemetry& t = car->telemetry();
        // (VehicleState::angular_velocity also carries the pitch / roll rates; the yaw rate itself is the
        // collision body's)
        double yaw = car->collision_body().yaw_rate;
        if (grounded_count(st) == 0) {
            if (air > 0) { // (the tyre phase reads the grounded flags the last tick's phase 12 left)
                yaw_frozen = yaw_frozen && yaw == yaw_air;
                falling = falling && st.velocity.y < vy_prev;
                for (size_t w = 0; w < 4; ++w) no_tyre_force = no_tyre_force && t.fx[w] == 0 && t.fy[w] == 0;
            }
            if (first_air < 0) first_air = i;
            ++air;
            no_tyre_force = no_tyre_force && load_sum(st) == 0;
            yaw_air = yaw;
            vy_prev = st.velocity.y;
        } else if (air > 0 && landed < 0) {
            landed = i;
        }
    }
    const VehicleState& st = car->state();
    std::printf("  airborne: %d ticks in the air from tick %d, landed tick %d, end y %.4f, %.1f km/h\n", air, first_air, landed,
                st.position.y, st.speed_kmh);
    CHECK(air > 10 && landed > first_air);
    CHECK(no_tyre_force);
    CHECK(yaw_frozen);
    CHECK(falling);
    CHECK(grounded_count(st) == 4 && std::fabs(st.position.y) < 0.01f && finite_state(st));
    CHECK(std::fabs(load_sum(st) - s.mass * kG) < 0.05f * s.mass * kG);
}

TEST(sim_spec_ground_miss_keeps_previous_height) {
    // physics §1.3/physics §10.5: off the road mesh the wheels keep the last ground height and stay
    // grounded; the car drives on (disc-free twin of sim_ground_miss_keeps_previous_height).
    auto car = Vehicle::create(synthetic_spec_car());
    EdgeGround g;
    car->spawn_moving({0, 0, 0}, {1, 0, 0, 0}, 50 / 3.6f);
    ControlInputs in;
    in.throttle = 0.6f;
    for (int i = 0; i < 180; ++i) car->step(in, g, kDt);
    const VehicleState& st = car->state();
    std::printf("  off-mesh: z %.1f, y %.4f, %.0f km/h, %d wheels grounded\n", st.position.z, st.position.y, st.speed_kmh,
                grounded_count(st));
    CHECK(st.position.z < -40 && grounded_count(st) == 4 && std::fabs(st.position.y) < 0.01f && st.speed_kmh > 45);
}

TEST(sim_spec_wall_contact) {
    // physics §5.3: the swept octagon stops the car at a wall at speed (no tunnelling) and a
    // glancing hit turns it along the wall; a world without segments uses the sphere fallback.
    for (float yaw : {0.0f, 0.5f}) {
        auto car = Vehicle::create(synthetic_spec_car(Drive::AWD));
        FlatGround g;
        SegmentWalls w;
        w.add(100, -40, -100, -40); // track on the +Z side, where the car starts
        car->reset({0, 0, 20}, yaw_quat(yaw));
        ControlInputs in;
        in.throttle = 1;
        float min_nose = 1e9f, impulse = 0;
        for (int i = 0; i < 60 * 8; ++i) {
            car->step(in, g, kDt, &w);
            const VehicleState& st = car->state();
            min_nose = std::min(min_nose, (st.position + forward(st) * 2.2f).z);
            impulse += car->telemetry().wall_impulse;
        }
        const VehicleState& st = car->state();
        std::printf("  wall (yaw %.1f): nose min z %.2f, impulse %.0f N*s, heading %.2f, %.1f km/h\n", yaw, min_nose, impulse,
                    heading(st), st.speed_kmh);
        CHECK(finite_state(st) && min_nose > -40.6f && impulse > 1000);
        if (yaw == 0) CHECK(std::fabs(forward_speed(st)) < 3);
        else CHECK(heading(st) > 1.0f); // turned towards the wall line (heading pi/2 = along -X)
    }
    auto car = Vehicle::create(synthetic_spec_car());
    FlatGround g;
    SphereWall w;
    car->reset({}, {});
    ControlInputs in;
    in.throttle = 1;
    float min_z = 0, impulse = 0;
    for (int i = 0; i < 60 * 8; ++i) {
        car->step(in, g, kDt, &w);
        min_z = std::min(min_z, car->state().position.z);
        impulse += car->telemetry().wall_impulse;
    }
    std::printf("  sphere wall: closest origin z %.2f, impulse %.0f N*s\n", min_z, impulse);
    CHECK(min_z > w.wall_z + 1.0f && impulse > 1000 && std::fabs(forward_speed(car->state())) < 3);
}

TEST(sim_spec_wall_stuck_assist) {
    // physics §5.3: pinned against a wall (planar speed under 1 m/s), the steering input turns the
    // car directly (yaw rate += -steer x 20 x dt), on both wall paths.
    auto turn = [](bool segments, float steer) {
        auto car = Vehicle::create(synthetic_spec_car(Drive::FR));
        FlatGround g;
        SegmentWalls seg;
        seg.add(100, -40, -100, -40);
        SphereWall sphere;
        const WorldQuery* w = segments ? static_cast<const WorldQuery*>(&seg) : &sphere;
        car->reset({0, 0, -34}, {});
        ControlInputs in;
        in.throttle = 0.5f;
        for (int i = 0; i < 120; ++i) car->step(in, g, kDt, w); // drive up to the wall and press on it
        float h0 = heading(car->state());
        in.steer = steer;
        for (int i = 0; i < 30; ++i) car->step(in, g, kDt, w);
        return heading(car->state()) - h0;
    };
    float seg_r = turn(true, 1), seg_l = turn(true, -1), sph_r = turn(false, 1), sph_l = turn(false, -1);
    std::printf("  stuck assist: heading change in 0.5 s: segments %+.3f / %+.3f, spheres %+.3f / %+.3f rad "
                "(steer right / left)\n",
                seg_r, seg_l, sph_r, sph_l);
    // (the tyres alone turn the car by less than 0.05 rad in that time at a standstill)
    CHECK(seg_r < -0.3f && seg_l > 0.3f && sph_r < -0.3f && sph_l > 0.3f);
}

TEST(sim_spec_surface_change) {
    // physics §3.6/physics §6.5: the tyre record follows the surface class (slot 1 on grass, slot 2 on
    // wet tarmac) with that record's mu[s]: at equal load the peak force scales exactly with
    // it, and braking distances grow on grass and wet tarmac.
    const CarSpec s = synthetic_spec_car();
    auto peak = [&](u32 surface) {
        auto car = Vehicle::create(s);
        PatchGround g;
        g.surface = surface;
        car->reset({}, {});
        for (int i = 0; i < 5; ++i) car->step({}, g, kDt);
        return car->telemetry().peak_force[0];
    };
    auto mu = [&](int slot, int idx) { return s.tire_slots[0][size_t(slot)].surface_mu[size_t(idx)]; };
    float tarmac = peak(0), grass = peak(2), wet = peak(0x200);
    std::printf("  surface: peak force tarmac %.0f grass %.0f wet %.0f N\n", tarmac, grass, wet);
    CHECK(std::fabs(grass / tarmac - mu(1, 2) / mu(0, 0)) < 1e-4f);
    CHECK(std::fabs(wet / tarmac - mu(2, 12) / mu(0, 0)) < 1e-4f);
    auto stop = [&](u32 surface) {
        auto car = Vehicle::create(s);
        PatchGround g;
        g.surface = surface;
        car->spawn_moving({}, {}, 80 / 3.6f);
        ControlInputs in;
        in.brake = 1;
        for (int i = 0; i < 900 && car->state().speed_kmh > 0; ++i) car->step(in, g, kDt);
        return -car->state().position.z;
    };
    float d_tarmac = stop(0), d_grass = stop(2), d_wet = stop(0x200);
    std::printf("  surface: 80-0 km/h tarmac %.1f m, grass %.1f m, wet %.1f m\n", d_tarmac, d_grass, d_wet);
    CHECK(d_grass > 1.5f * d_tarmac && d_wet > 1.15f * d_tarmac);
    // Driving across a boundary: the front wheels pick up the new surface before the rear.
    auto car = Vehicle::create(s);
    PatchGround g;
    g.wet_z = -30;
    car->spawn_moving({}, {}, 60 / 3.6f);
    ControlInputs in;
    in.throttle = 0.4f;
    bool front_first = false;
    for (int i = 0; i < 180; ++i) {
        car->step(in, g, kDt);
        const VehicleState& st = car->state();
        front_first = front_first || (st.wheels[0].surface == 0x200 && st.wheels[2].surface == 0);
    }
    CHECK(front_first && car->state().wheels[3].surface == 0x200);
}

TEST(sim_spec_slipstream) {
    // physics §5.2: the slipstream factor filters in and cuts the drag: coasting from 150 km/h in
    // a tow, the car keeps more speed than in clean air.
    auto coast = [](float factor) {
        auto car = Vehicle::create(synthetic_spec_car());
        FlatGround g;
        car->spawn_moving({}, {}, 150 / 3.6f);
        car->set_slipstream(factor);
        float drag = 0;
        for (int i = 0; i < 180; ++i) {
            car->step({}, g, kDt);
            if (i == 0) drag = car->telemetry().drag_force;
        }
        return std::array<float, 3>{car->state().speed_kmh, car->telemetry().slipstream, drag};
    };
    auto clean = coast(0), tow = coast(1);
    std::printf("  slipstream: after 3 s coasting %.2f km/h clean, %.2f km/h in a tow (factor %.2f)\n", clean[0], tow[0], tow[1]);
    CHECK(clean[1] == 0 && tow[1] > 0.9f && tow[1] <= 1);
    CHECK(tow[0] > clean[0] + 1 && tow[2] < clean[2]);
}

TEST(sim_spec_dirt_brake_cut) {
    // physics §10.4: on a dirt course the front brake pressure falls with the steering angle;
    // braking in a straight line is unchanged (the runs stay bit-identical).
    auto brake = [](bool dirt, float steer) {
        auto car = Vehicle::create(synthetic_spec_car());
        FlatGround g;
        car->set_dirt_course(dirt);
        car->spawn_moving({}, {}, 60 / 3.6f);
        ControlInputs in;
        in.steer = steer;
        in.brake = 0.3f;
        for (int i = 0; i < 20; ++i) car->step(in, g, kDt);
        return std::pair<float, VehicleState>(car->telemetry().brake_pressure[0], car->state());
    };
    auto [straight_dirt, s_dirt] = brake(true, 0);
    auto [straight, s_tarmac] = brake(false, 0);
    auto [turn_dirt, t_dirt] = brake(true, 0.4f);
    auto [turn, t_tarmac] = brake(false, 0.4f);
    std::printf("  dirt: front brake pressure %.3f / %.3f straight, %.3f / %.3f turning (dirt / tarmac)\n", straight_dirt,
                straight, turn_dirt, turn);
    CHECK(straight_dirt == straight && same_state(s_dirt, s_tarmac));
    CHECK(turn > 0.2f && turn_dirt < 0.8f * turn);
}

TEST(sim_spec_engine_condition) {
    // physics §10.3: every tick the full-throttle torque uses the current factor f(E, O), and E,
    // O advance by |u| dt and the odometer (2 m units) by the forward distance. A worn engine
    // (E at its cap) seeded at the start makes 2 % less torque from the first tick and loses
    // ground all run; the words read back are the advanced ones.
    struct Run {
        float dist = 0, torque = 0;
        EngineCondition end;
        bool per_tick = true; // E grew by |u| dt every tick
        VehicleState state;
    };
    auto run = [](const EngineCondition* seed) {
        Run r;
        auto car = Vehicle::create(synthetic_spec_car());
        FlatGround g;
        if (seed) car->set_engine_condition(*seed);
        car->spawn_moving({}, {}, 100 / 3.6f);
        ControlInputs in;
        in.throttle = 1;
        for (int i = 0; i < 300; ++i) {
            const float e0 = car->engine_condition().wear_m;
            const float u = car->state().speed_kmh / 3.6f;
            car->step(in, g, kDt);
            if (i == 0) r.torque = car->telemetry().engine_torque;
            const float de = car->engine_condition().wear_m - e0;
            if (car->engine_condition().wear_m < 600000) r.per_tick = r.per_tick && std::fabs(de - u * kDt) < 0.02f * u * kDt + 1e-3f;
        }
        r.dist = -car->state().position.z;
        r.end = car->engine_condition();
        r.state = car->state();
        return r;
    };
    const EngineCondition zeros{}, worn{5000, 600000, 0}, aged{5000, 450000, 0};
    Run fresh = run(nullptr), fresh0 = run(&zeros), w = run(&worn), a = run(&aged);
    std::printf("  engine condition: torque %.1f / %.1f N*m (fresh / worn), %.1f / %.1f m; fresh counters %.1f / %.1f, "
                "odometer %u units\n",
                fresh.torque, w.torque, fresh.dist, w.dist, fresh.end.wear_m, fresh.end.oil_m, fresh.end.odometer);
    // A fresh engine (all zero, seeded or by default) is unchanged.
    CHECK(fresh0.dist == fresh.dist && fresh0.torque == fresh.torque && same_state(fresh0.state, fresh.state));
    CHECK(fresh.per_tick && std::fabs(fresh.end.wear_m - fresh.dist) < 0.02f * fresh.dist && fresh.end.oil_m == fresh.end.wear_m);
    CHECK(std::fabs(2.0f * float(fresh.end.odometer) - fresh.dist) < 0.02f * fresh.dist + 2);
    // Worn: 98 % torque at the same state, slower all run; E stays at its cap, the odometer runs on.
    CHECK(w.end.wear_m == 600000 && std::fabs(w.torque / fresh.torque - 0.98f) < 1e-4f && w.dist < fresh.dist);
    CHECK(w.end.odometer > 5000 && std::fabs(2.0f * float(w.end.odometer - 5000) - w.dist) < 0.02f * w.dist + 2);
    // Inside the flat part (100 %), a car with miles on it drives exactly like a fresh one.
    CHECK(a.dist == fresh.dist && a.torque == fresh.torque && a.end.wear_m > 450000);
}

// physics §10.3: the odometer word (2 m units) from x = carry + d metres; the carry keeps the
// part below 2 m, x <= 0 counts nothing, the word saturates.
TEST(sim_odometer_step) {
    u32 w = 10;
    CHECK(model::odometer_step(w, 5.5f) == 1.5f && w == 12);
    CHECK(model::odometer_step(w, 1.5f) == 1.5f && w == 12);
    CHECK(model::odometer_step(w, 0.0f) == 0.0f && model::odometer_step(w, -3.0f) == 0.0f && w == 12);
    w = model::kOdometerMax - 2;
    CHECK(model::odometer_step(w, 5.0f) == 0.0f && w == model::kOdometerMax);
    CHECK(model::oil_worn(300000) && !model::oil_worn(299999) && !model::oil_worn(-300000));
}

TEST(sim_spec_electric) {
    // Mod EVs: direct drive (the clutch is locked whenever the driver asks for drive, the motor
    // turns with the wheels), lift-off regeneration slows the car harder than coasting, and
    // with no pedal near walking pace the friction brakes hold it at a standstill.
    CarSpec ev = test::synthetic_spec_ev();
    auto car = Vehicle::create(ev);
    FlatGround g;
    car->reset({}, {});
    ControlInputs in;
    in.throttle = 1;
    bool direct = true;
    for (int i = 0; i < 300; ++i) {
        car->step(in, g, kDt);
        direct = direct && car->telemetry().clutch_state == 1;
    }
    float v5 = car->state().speed_kmh;
    in.throttle = 0;
    for (int i = 0; i < 60; ++i) car->step(in, g, kDt);
    float regen_drop = v5 - car->state().speed_kmh;
    for (int i = 0; i < 60 * 30 && forward_speed(car->state()) > 0.01f; ++i) car->step(in, g, kDt);
    collide::V2 p0 = car->collision_body().pos;
    for (int i = 0; i < 120; ++i) car->step(in, g, kDt);
    collide::V2 p1 = car->collision_body().pos;
    // the same chassis with the turbo engine, coasting in gear from the same speed
    auto ice = Vehicle::create(synthetic_spec_car(Drive::RR));
    ice->spawn_moving({}, {}, v5 / 3.6f);
    for (int i = 0; i < 60; ++i) ice->step({}, g, kDt);
    float coast_drop = v5 - ice->state().speed_kmh;
    std::printf("  electric: %.1f km/h after 5 s, lift-off -%.1f km/h in 1 s (engine car -%.1f), held %.2g m\n", v5,
                regen_drop, coast_drop, std::hypot(p1.x - p0.x, p1.z - p0.z));
    CHECK(direct && v5 > 60 && car->state().gear == 1);
    CHECK(regen_drop > coast_drop + 1);
    CHECK(std::hypot(p1.x - p0.x, p1.z - p0.z) < 1e-3);
}

TEST(sim_spec_reverse) {
    FlatGround g;
    {
        // Automatic: shift down at a standstill selects reverse; the brake pedal drives it
        // (AT reverse pedal swap).
        auto car = Vehicle::create(synthetic_spec_car(Drive::FR));
        car->reset({}, {});
        ControlInputs in;
        for (int i = 0; i < 12; ++i) car->step(in, g, kDt); // standing start: shift lockout 12
        in.shift_down = true;
        car->step(in, g, kDt);
        CHECK(car->state().gear == -1);
        in.shift_down = false;
        in.brake = 0.6f;
        for (int i = 0; i < 120; ++i) car->step(in, g, kDt);
        std::printf("  reverse (AT): %.2f m/s after 2 s\n", forward_speed(car->state()));
        CHECK(car->state().gear == -1 && forward_speed(car->state()) < -1.5f && car->state().position.z > 1);
    }
    {
        // Manual: 1 -> N -> R with the 1-step shift lockout; the accelerator drives backwards.
        auto car = Vehicle::create(synthetic_spec_car(Drive::AWD));
        car->reset({}, {});
        ControlInputs in;
        in.automatic = false;
        for (int i = 0; i < 12; ++i) car->step(in, g, kDt);
        in.shift_down = true;
        car->step(in, g, kDt);
        CHECK(car->state().gear == 0);
        in.shift_down = false;
        car->step(in, g, kDt);
        in.shift_down = true;
        car->step(in, g, kDt);
        CHECK(car->state().gear == -1);
        in.shift_down = false;
        in.throttle = 0.5f;
        for (int i = 0; i < 120; ++i) car->step(in, g, kDt);
        std::printf("  reverse (manual): %.2f m/s after 2 s\n", forward_speed(car->state()));
        CHECK(forward_speed(car->state()) < -2.0f);
    }
    {
        // CPU car: the reverse request holds reverse with the accelerator (no pedal swap).
        auto car = Vehicle::create(synthetic_spec_car(Drive::RR));
        car->set_cpu_driver(true);
        car->reset({}, {});
        car->set_reverse_request(true);
        ControlInputs in;
        in.throttle = 1;
        in.steer_angle = 0;
        for (int i = 0; i < 180; ++i) car->step(in, g, kDt);
        std::printf("  reverse (CPU): gear %d, %.2f m/s after 3 s\n", car->state().gear, forward_speed(car->state()));
        CHECK(car->state().gear == -1 && forward_speed(car->state()) < -2.0f);
    }
}

TEST(sim_spec_deterministic_and_reset) {
    // The same inputs give bit-identical states and telemetry; reset() leaves no per-run
    // state behind: a car that has driven, then is reset, behaves exactly like a new one.
    const CarSpec s = synthetic_spec_car(Drive::AWD, true);
    auto inputs = [](int i) {
        ControlInputs in;
        in.throttle = float((i * 37) % 100) / 100.0f;
        in.brake = (i / 90) % 4 == 3 ? 0.6f : 0;
        in.steer = std::sin(float(i) * 0.013f);
        in.handbrake = (i / 50) % 13 == 7;
        in.shift_up = i % 200 == 0;
        return in;
    };
    PatchGround g;
    g.grass_x = 25;
    auto a = Vehicle::create(s), b = Vehicle::create(s), used = Vehicle::create(s);
    for (int i = 0; i < 600; ++i) used->step(inputs(i * 3 + 1), g, kDt);
    a->reset({1, 0, 2}, yaw_quat(0.3f));
    b->reset({1, 0, 2}, yaw_quat(0.3f));
    used->reset({1, 0, 2}, yaw_quat(0.3f));
    bool same = true, same_used = true;
    float top = 0;
    for (int i = 0; i < 60 * 30; ++i) {
        a->step(inputs(i), g, kDt);
        b->step(inputs(i), g, kDt);
        used->step(inputs(i), g, kDt);
        same = same && same_state(a->state(), b->state()) && same_telemetry(a->telemetry(), b->telemetry());
        same_used = same_used && same_state(a->state(), used->state()) && same_telemetry(a->telemetry(), used->telemetry());
        top = std::max(top, a->state().speed_kmh);
    }
    CHECK(same);
    CHECK(same_used);
    CHECK(top > 20 && finite_state(a->state())); // the run actually drove
}

TEST(sim_spec_spawn_hold_and_pit_stop) {
    // physics §4.3: 120 ticks held at ground + 0.3 m (on the ground for the last 30), flat, gear 1,
    // heading easing to the grid heading; physics §8.6: the pit stop holds 300 ticks and fits fresh
    // tyres at counter 60 (disc-free twin of sim_spawn_hold).
    auto car = Vehicle::create(synthetic_spec_car());
    FlatGround g;
    car->spawn({0, 0, 0}, yaw_quat(0.5f));
    ControlInputs in;
    in.throttle = 1;
    for (int i = 0; i < 60; ++i) car->step(in, g, kDt);
    std::printf("  spawn: after 1 s y %.3f m, %.2f km/h, gear %d, held %d\n", car->state().position.y,
                car->state().speed_kmh, car->state().gear, int(car->spawn_hold_active()));
    CHECK(car->spawn_hold_active() && std::fabs(car->state().position.y - 0.3f) < 0.01f);
    CHECK(car->state().speed_kmh < 0.1f && car->state().gear == 1 && std::fabs(car->telemetry().pitch) < 1e-6f);
    for (int i = 0; i < 70; ++i) car->step(in, g, kDt);
    CHECK(!car->spawn_hold_active());
    CHECK(std::fabs(heading(car->state()) - 0.5f) < 0.01f);
    for (int i = 0; i < 120; ++i) car->step(in, g, kDt);
    CHECK(car->state().speed_kmh > 20 && finite_state(car->state()));

    car->set_tire_wear(model::shipped_wear_params());
    in.handbrake = true; // wear the rear tyres in a few spins
    in.steer = 1;
    for (int i = 0; i < 180; ++i) car->step(in, g, kDt);
    float worn = car->telemetry().tire_wear[2];
    CHECK(worn > -20000);
    // the box heading within the 4 s x 30 deg/s the hold eases through
    const float box = heading(car->state()) + 1.0f;
    car->begin_pit_stop(yaw_quat(box));
    CHECK(car->spawn_hold_active() && car->state().speed_kmh < 0.1f);
    ControlInputs idle;
    for (int i = 0; i < 239; ++i) car->step(idle, g, kDt); // counter 300 -> 62
    CHECK(car->spawn_hold_active() && car->telemetry().tire_wear[2] == worn);
    for (int i = 0; i < 3; ++i) car->step(idle, g, kDt); // through counter 60
    CHECK(car->telemetry().tire_wear[2] == -20000);
    for (int i = 0; i < 60; ++i) car->step(idle, g, kDt);
    CHECK(!car->spawn_hold_active() && std::fabs(std::remainder(heading(car->state()) - box, 6.2831853f)) < 0.01f);
    CHECK(finite_state(car->state()));
}

TEST(sim_spec_race_hold) {
    // physics §9.3: the countdown hold keeps the car in place on a slope with the throttle open
    // (planar velocity zeroed, clutch request 0); released, the car drives off.
    auto car = Vehicle::create(synthetic_spec_car(Drive::AWD));
    Slope slope;
    slope.grade = 0.1f;
    car->reset({0, 0, 0}, {});
    car->set_race_hold(true);
    ControlInputs in;
    in.throttle = 1;
    car->step(in, slope, kDt); // the first tick fits the body to the slope (moves the origin)
    Vec3 p0 = car->state().position;
    for (int i = 0; i < 180; ++i) car->step(in, slope, kDt);
    Vec3 d = car->state().position - p0;
    std::printf("  race hold: moved %.3g m, %.0f rpm, clutch %d\n", std::hypot(d.x, d.z), car->state().engine_rpm,
                car->telemetry().clutch_state);
    CHECK(std::hypot(d.x, d.z) < 1e-4f);
    CHECK(car->telemetry().clutch_state == 0 && car->state().engine_rpm > 3000);
    car->set_race_hold(false);
    for (int i = 0; i < 120; ++i) car->step(in, slope, kDt);
    CHECK(car->state().position.z < -3 && finite_state(car->state()));
}

TEST(sim_spec_parked_clutch) {
    // physics §2.4: driver kind 8 (parked) keeps the clutch request at 0, for the player's car
    // and CPU cars alike: a locked clutch opens on the first tick and stays open, even with the
    // throttle down (the engine revs freely while the car is held). The stop driver's kinds
    // (5-7) keep the AI rule: a locked clutch stays engaged while the engine is at or above idle.
    FlatGround g;
    for (bool cpu : {false, true}) {
        auto parked = Vehicle::create(synthetic_spec_car(Drive::RR));
        parked->set_cpu_driver(cpu);
        parked->spawn_moving({}, {}, 80 / 3.6f); // clutch locked, engine well above idle
        parked->set_driver_kind(8);
        ControlInputs in;
        in.throttle = 1;
        parked->step(in, g, kDt);
        const int first = parked->telemetry().clutch_state;
        bool open = true;
        for (int i = 0; i < 120; ++i) {
            parked->step(in, g, kDt);
            open = open && parked->telemetry().clutch_state == 0;
        }
        const float idle = parked->spec().engine.idle_rpm, up = parked->spec().engine.upshift_rpm;
        auto stopping = Vehicle::create(synthetic_spec_car(Drive::RR));
        stopping->set_cpu_driver(cpu);
        stopping->spawn_moving({}, {}, 80 / 3.6f);
        stopping->set_driver_kind(7);
        stopping->step({}, g, kDt);
        std::printf("  parked (%s): clutch %d after a tick, open for 2 s at %.0f rpm; kind 7: clutch %d\n",
                    cpu ? "CPU car" : "player", first, parked->state().engine_rpm,
                    stopping->telemetry().clutch_state);
        CHECK(first == 0 && open && parked->state().engine_rpm > 0.75f * up && parked->state().engine_rpm > idle);
        CHECK(stopping->telemetry().clutch_state == 1 && stopping->state().engine_rpm >= idle);
    }
}

TEST(sim_spec_aids) {
    // physics §4.5/physics §4.6: TCS cuts the throttle on a wheelspin launch (less rear slip than without
    // it); ABS holds the braking slip near the optimum; the ASCC indicators respond in a
    // slide; a CPU car reads the scripted aid block (TCS) whatever the car's own.
    FlatGround g;
    auto launch = [&](bool aids, bool cpu) {
        auto car = Vehicle::create(synthetic_spec_car(Drive::FR, aids));
        car->set_cpu_driver(cpu);
        car->reset({}, {});
        ControlInputs in;
        in.throttle = 1;
        if (cpu) in.steer_angle = 0;
        float mean = 0, min_factor = 1;
        for (int i = 0; i < 120; ++i) {
            car->step(in, g, kDt);
            mean += car->state().wheels[2].slip_ratio / 120;
            min_factor = std::min(min_factor, car->telemetry().tcs_factor);
        }
        return std::pair<float, float>(mean, min_factor);
    };
    auto [slip_off, f_off] = launch(false, false);
    auto [slip_on, f_on] = launch(true, false);
    auto [slip_cpu, f_cpu] = launch(false, true);
    std::printf("  aids: launch mean rear slip %.3f without TCS, %.3f with (factor min %.2f), CPU %.3f (factor min %.2f)\n",
                slip_off, slip_on, f_on, slip_cpu, f_cpu);
    CHECK(f_off == 1 && f_on < 0.9f && slip_on > slip_off);
    CHECK(f_cpu < 1);
    // ABS: braking from speed, the slip stays near the record's optimum.
    auto car = Vehicle::create(synthetic_spec_car(Drive::FR, true));
    car->spawn_moving({}, {}, 120 / 3.6f);
    ControlInputs in;
    in.brake = 1;
    float max_slip = 0;
    for (int i = 0; i < 120; ++i) {
        car->step(in, g, kDt);
        if (i > 10) max_slip = std::max(max_slip, car->state().wheels[0].slip_ratio);
    }
    std::printf("  aids: ABS max front braking slip %.3f\n", max_slip);
    CHECK(max_slip > 0.02f && max_slip < 0.5f);
    // Handbrake turn: the indicators see the slide.
    in = {};
    in.handbrake = true;
    in.steer = 1;
    car->spawn_moving({}, {}, 80 / 3.6f);
    float under = 0, over = 0;
    for (int i = 0; i < 90; ++i) {
        car->step(in, g, kDt);
        under = std::max(under, car->telemetry().understeer);
        over = std::max(over, car->telemetry().oversteer);
    }
    std::printf("  aids: slide indicators understeer %.2f oversteer %.2f\n", under, over);
    CHECK(over > 0);
    // ASCC against the same car with its ASCC block off. Lifting with the steering held at speed,
    // the oversteer channel (A) brakes the outside wheels without the pedal; braking into a tight
    // turn, the understeer channel (D) relieves the pedal and scales the ABS target slip.
    struct Ascc {
        float pressure_no_pedal = 0, min_target = 1;
    };
    auto ascc = [&](bool on, bool braking) {
        CarSpec s = synthetic_spec_car(Drive::FR, true);
        if (!on) s.aids.ascc_flags = 0;
        auto c = Vehicle::create(s);
        c->spawn_moving({}, {}, 110 / 3.6f);
        ControlInputs k;
        k.steer = braking ? 0.6f : 0.5f;
        k.brake = braking ? 0.5f : 0;
        Ascc r;
        for (int i = 0; i < (braking ? 50 : 90); ++i) {
            c->step(k, g, kDt);
            for (size_t w = 0; w < 4; ++w) {
                if (!braking) r.pressure_no_pedal = std::max(r.pressure_no_pedal, c->telemetry().brake_pressure[w]);
                r.min_target = std::min(r.min_target, c->telemetry().abs_target[w]);
            }
        }
        return r;
    };
    Ascc lift_on = ascc(true, false), lift_off = ascc(false, false), brake_on = ascc(true, true), brake_off = ascc(false, true);
    const float opt = synthetic_spec_car().tire[0].opt_brake;
    std::printf("  aids: ASCC lift-off pressure %.2f (off %.2f); braking turn ABS target %.3f (off %.3f, optimum %.3f)\n",
                lift_on.pressure_no_pedal, lift_off.pressure_no_pedal, brake_on.min_target, brake_off.min_target, opt);
    CHECK(lift_on.pressure_no_pedal > 0.1f && lift_off.pressure_no_pedal == 0);
    CHECK(brake_on.min_target < 0.95f * opt && brake_off.min_target == opt);
}

TEST(sim_options_asm_tcs_off) {
    // The Options menu's ASM / TCS off, applied as the race applies them (apply_aid_options: arcade
    // single race, free run, 2P battle), switch the player's own aids off in the physics, and the
    // dash lamps follow what the physics does (physics §4.6). Each run drives the same car three
    // ways: lifting off with the steering held at speed (ASCC A / C brake wheels without the
    // pedal), braking into a turn (D / E relieve the pedal and the ABS target slip), and a full
    // throttle launch (TCS cuts the throttle).
    FlatGround g;
    struct Run {
        bool asm_lamp = false, tcs_lamp = false;
        float lift_pressure = 0, min_target = 1, min_tcs = 1;
    };
    auto run = [&](bool asm_on, bool tcs_on) {
        CarSpec s = synthetic_spec_car(Drive::FR, true);
        apply_aid_options(s.aids, asm_on, tcs_on);
        Run r;
        r.asm_lamp = model::asm_lamp_lit(s.aids);
        r.tcs_lamp = s.aids.tcs_gain > 0;
        auto lift = Vehicle::create(s);
        lift->spawn_moving({}, {}, 110 / 3.6f);
        ControlInputs k;
        k.steer = 0.5f;
        for (int i = 0; i < 90; ++i) {
            lift->step(k, g, kDt);
            for (size_t w = 0; w < 4; ++w)
                r.lift_pressure = std::max(r.lift_pressure, lift->telemetry().brake_pressure[w]);
        }
        auto brake = Vehicle::create(s);
        brake->spawn_moving({}, {}, 110 / 3.6f);
        k.steer = 0.6f;
        k.brake = 0.5f;
        for (int i = 0; i < 50; ++i) {
            brake->step(k, g, kDt);
            for (size_t w = 0; w < 4; ++w) r.min_target = std::min(r.min_target, brake->telemetry().abs_target[w]);
        }
        auto launch = Vehicle::create(s);
        launch->reset({}, {});
        ControlInputs t;
        t.throttle = 1;
        for (int i = 0; i < 120; ++i) {
            launch->step(t, g, kDt);
            r.min_tcs = std::min(r.min_tcs, launch->telemetry().tcs_factor);
        }
        return r;
    };
    const float opt = synthetic_spec_car().tire[0].opt_brake;
    const Run on = run(true, true), asm_off = run(false, true), tcs_off = run(true, false), off = run(false, false);
    for (const auto& [name, r] : {std::pair<const char*, const Run&>{"both on", on}, {"ASM off", asm_off},
                                  {"TCS off", tcs_off}, {"both off", off}})
        std::printf("  %-8s lamps ASM %d TCS %d; lift-off pressure %.2f, braking ABS target %.3f (optimum %.3f), "
                    "TCS factor min %.2f\n",
                    name, r.asm_lamp, r.tcs_lamp, r.lift_pressure, r.min_target, opt, r.min_tcs);
    CHECK(on.asm_lamp && on.tcs_lamp && on.lift_pressure > 0.1f && on.min_target < 0.95f * opt && on.min_tcs < 0.9f);
    // ASM off: no ASCC channel acts (no pressure without the pedal, the ABS target untouched), TCS still on.
    CHECK(!asm_off.asm_lamp && asm_off.lift_pressure == 0 && asm_off.min_target == opt);
    CHECK(asm_off.tcs_lamp && asm_off.min_tcs < 0.9f);
    // TCS off: no throttle cut, ASCC still on.
    CHECK(!tcs_off.tcs_lamp && tcs_off.min_tcs == 1);
    CHECK(tcs_off.asm_lamp && tcs_off.lift_pressure > 0.1f && tcs_off.min_target < 0.95f * opt);
    CHECK(!off.asm_lamp && !off.tcs_lamp && off.lift_pressure == 0 && off.min_target == opt && off.min_tcs == 1);
}

// --- golden traces -----------------------------------------------------------------
//
// Characterization tests: each scenario drives the synthetic car through scripted inputs
// that reach every phase of Vehicle::tick (gear selection, steering, ground query and
// suspension, aero, tyres, engine / clutch / turbo, driveline and differentials, brakes and
// ABS, TCS / ASCC, planar step and load transfer, attitude and bump nudge, walls), and
// compares a set of state values at checkpoints with the values the current code produced.
// Regenerate after a deliberate behaviour change with
//     GT3R_GOLDEN_PRINT=1 ./gt3r_tests sim_spec_golden
// and paste the printed tables over kGolden* below.
//
// Tolerances. CI builds with GCC (Linux, x86-64), Apple clang (macOS, arm64) and MSVC
// (Windows, x86-64): libm's sin / cos / atan2 can differ in the last bit and clang on arm64
// contracts a * b + c into fused multiply-adds, so the runs agree only to float noise, which
// the dynamics then amplify. Bit equality would flake; the tolerances (kGoldenTol, plus 1e-4
// relative) were set against measured noise instead:
//   - GT3R_GOLDEN_SENSITIVITY=1 reruns each scenario with the start heading nudged by +-1e-6 rad
//     and the ground by +-1e-5 m (perturbations far larger than one ulp) and prints the worst
//     deviation as a fraction of the tolerance (=2 prints every checkpoint): at most 0.07 on
//     the spec-path scenarios and 0.12 on the placeholder one (its summed wall impulse) when
//     the tables were made;
//   - a build with -ffp-contract=off (no FMAs, as on x86-64) against the default arm64 build:
//     at most 0.04 on the spec path, 0.12 on the placeholder scenario.
// A change in the order or content of a tick phase moves these values by many tolerances
// within a few dozen ticks (checked by deliberately breaking vehicle.cpp: load transfer from
// the last tick's a_y, no free-wheel lock, drag skipped, TCS slip gain, camber, toe, ASCC,
// downforce, bump nudge, the CPU clutch rule... each fails at least one test). Integer states
// (gear, clutch state, grounded wheels) must match exactly. The scenarios avoid the regimes
// where the trajectory itself is ill-conditioned (near-standstill, where the low-speed hold
// switches on and off; long slides; road-bump surfaces; braking along a wall) and checkpoints
// that fall on a discrete event, and they stay within a few hundred ticks, well inside our own
// chaos horizon (a 1e-5 m nudge separates runs by 1 cm only after 900-1700 ticks,
// docs/dev/PHYSICS.md). If a scenario starts failing on one platform only, rerun the
// sensitivity check before widening a tolerance.

namespace {

enum GoldenField : int {
    kGx, kGy, kGz, kGHeading, kGPitch, kGRoll, kGSpeed, kGYawRate, kGRpm, kGGear, kGClutch, kGGrounded,
    kGLoadFront, kGLoadLeft, kGSusp0, kGSusp3, kGWheel0, kGWheel3, kGBoost, kGSteer, kGWall, kGSmoke, kGBrakeHeat,
    kGWear, kGFieldCount
};
using GoldenRow = std::array<double, kGFieldCount>;

struct FieldTol {
    const char* name;
    double abs;
};
// Absolute tolerance per field (plus 1e-4 relative, for the large values).
constexpr FieldTol kGoldenTol[kGFieldCount] = {
    {"x", 0.01},       {"y", 0.002},     {"z", 0.01},          {"heading", 0.002}, {"pitch", 0.0005}, {"roll", 0.0005},
    {"km/h", 0.05},    {"yaw_rate", 0.002}, {"rpm", 5},        {"gear", 0},        {"clutch", 0},     {"grounded", 0},
    {"load_front", 10}, {"load_left", 10}, {"susp0", 0.0005}, {"susp3", 0.0005},  {"wheel0", 0.05},  {"wheel3", 0.05},
    {"boost", 0.005},  {"steer", 0.0005}, {"wall_impulse", 5}, {"smoke", 0.01},     {"brake_heat", 0.002}, {"wear", 5},
};

GoldenRow golden_row(const Vehicle& car, double wall_impulse) {
    const VehicleState& st = car.state();
    const VehicleTelemetry& t = car.telemetry();
    GoldenRow r{};
    r[kGx] = st.position.x;
    r[kGy] = st.position.y;
    r[kGz] = st.position.z;
    r[kGHeading] = heading(st);
    r[kGPitch] = t.pitch;
    r[kGRoll] = t.roll;
    r[kGSpeed] = st.speed_kmh;
    r[kGYawRate] = st.angular_velocity.y;
    r[kGRpm] = st.engine_rpm;
    r[kGGear] = st.gear;
    r[kGClutch] = t.clutch_state;
    r[kGGrounded] = grounded_count(st);
    r[kGLoadFront] = st.wheels[0].load + st.wheels[1].load;
    r[kGLoadLeft] = st.wheels[0].load + st.wheels[2].load;
    r[kGSusp0] = st.wheels[0].suspension;
    r[kGSusp3] = st.wheels[3].suspension;
    r[kGWheel0] = st.wheels[0].angular_velocity;
    r[kGWheel3] = st.wheels[3].angular_velocity;
    r[kGBoost] = t.boost;
    r[kGSteer] = t.steer_angle;
    r[kGWall] = wall_impulse;
    for (size_t i = 0; i < 4; ++i) {
        r[kGSmoke] += t.smoke[i];
        r[kGBrakeHeat] += t.brake_heat[i];
    }
    r[kGWear] = t.tire_wear[2];
    return r;
}

// Steps `car` through `ticks` ticks, sampling every `every` ticks.
template <class Inputs>
void drive(Vehicle& car, const TrackQuery& g, const WorldQuery* w, int ticks, int every, Inputs inputs,
           std::vector<GoldenRow>& rows) {
    double wall = 0;
    for (int i = 0; i < ticks; ++i) {
        car.step(inputs(i), g, kDt, w);
        wall += car.telemetry().wall_impulse;
        if (i % every == every - 1) rows.push_back(golden_row(car, wall));
    }
}

// A scenario: the car, the start, the world and the inputs of each tick. `hn` / `gn` perturb
// the start heading (rad) and the ground height (m) for the sensitivity check.
struct Scenario {
    const char* name = "";
    const char* table = ""; // the kGolden* table it is checked against
    int ticks = 0, every = 30;
    virtual ~Scenario() = default;
    virtual void run(float hn, float gn, std::vector<GoldenRow>& rows) const = 0;
    std::vector<GoldenRow> rows(float hn = 0, float gn = 0) const {
        std::vector<GoldenRow> r;
        run(hn, gn, r);
        return r;
    }
};

// Standing start with TCS / ASCC (FR, turbo) and tyre wear on: clutch launch, TCS, turbo lag,
// the 1-2 shift clutch, then braking while turning (ABS, brake heat), then a steering reversal
// on part throttle.
struct LaunchScenario : Scenario {
    LaunchScenario() { name = "launch", table = "kGoldenLaunch", ticks = 450; }
    void run(float hn, float gn, std::vector<GoldenRow>& rows) const override {
        auto car = Vehicle::create(synthetic_spec_car(Drive::FR, true));
        FlatGround g;
        g.height = gn;
        car->reset({0, gn, 0}, yaw_quat(hn));
        car->set_tire_wear(model::shipped_wear_params());
        drive(*car, g, nullptr, ticks, every, [](int i) {
            ControlInputs in;
            if (i < 270) {
                in.throttle = 1;
            } else if (i < 315) {
                in.brake = 1;
                in.steer = 0.3f;
            } else {
                in.throttle = 0.5f;
                in.steer = -0.3f;
            }
            return in;
        }, rows);
    }
};

// AWD (L5 coupling, rear clutch-pack LSD), moving start: a right-hand corner onto a loose
// verge (material 8: lower mu[s]), then a left correction onto wet tarmac (surface class 2:
// the slot-2 tyre record). Both surfaces have no road-bump wave: grass and dirt excite the
// suspension enough that the trajectory depends on the position to the micrometre, which
// is real behaviour but not reproducible across compilers (the property tests cover them).
struct CornerScenario : Scenario {
    // (sampled every 35 ticks: the state at tick 150 is ~100x more sensitive to float noise than its
    // neighbours, a short transient of the clutch-pack / centre-coupling states)
    CornerScenario() { name = "corner", table = "kGoldenCorner", ticks = 420, every = 35; }
    void run(float hn, float gn, std::vector<GoldenRow>& rows) const override {
        struct G : TrackQuery {
            float h = 0;
            SurfaceHit probe(float x, float z, float y_max) const override {
                SurfaceHit s;
                s.height = h;
                s.hit = y_max >= h;
                s.surface = z < -85 ? 0x200u : x > 12 ? 8u : 0u;
                return s;
            }
        } g;
        g.h = gn;
        auto car = Vehicle::create(synthetic_spec_car(Drive::AWD));
        car->spawn_moving({0, gn, 0}, yaw_quat(hn), 70 / 3.6f);
        drive(*car, g, nullptr, ticks, every, [](int i) {
            ControlInputs in;
            in.throttle = 0.35f;
            if (i >= 60 && i < 240) in.steer = 0.2f;
            else if (i >= 240) in.steer = -0.15f;
            return in;
        }, rows);
    }
};

// FF, moving start over a kicker (airborne, the all-airborne rules, landing), then a
// glancing hit on a wall at a shallow angle (swept octagon, impulse with friction, held
// contact points), steering away from it. (Braking or pressing along the wall afterwards makes
// the outcome depend on float noise, so the scenario leaves the wall.)
struct JumpWallScenario : Scenario {
    JumpWallScenario() { name = "jump_wall", table = "kGoldenJumpWall", ticks = 360; }
    void run(float hn, float gn, std::vector<GoldenRow>& rows) const override {
        struct G : Kicker {
            float h = 0;
            SurfaceHit probe(float x, float z, float y_max) const override {
                SurfaceHit s = Kicker::probe(x, z, y_max - h);
                s.height += h;
                return s;
            }
        } g;
        g.h = gn;
        SegmentWalls w;
        w.add(5, -60, 15, -200); // the track (left of a -> b) is the -X side, where the car is
        auto car = Vehicle::create(synthetic_spec_car(Drive::FF));
        car->spawn_moving({0, gn, 0}, yaw_quat(hn), 75 / 3.6f);
        drive(*car, g, &w, ticks, every, [](int i) {
            ControlInputs in;
            in.throttle = i < 250 ? 0.7f : 0.3f;
            in.steer = i < 120 ? 0 : i < 255 ? 0.15f : -0.3f;
            return in;
        }, rows);
    }
};

// CPU car (MR) on a gentle grade: direct road-wheel angles, the CPU clutch rule (engage above
// 3/4 of the upshift rpm), the scripted aid block (TCS), braking in a turn, power again.
struct CpuScenario : Scenario {
    CpuScenario() { name = "cpu", table = "kGoldenCpu", ticks = 360; }
    void run(float hn, float gn, std::vector<GoldenRow>& rows) const override {
        CarSpec s = synthetic_spec_car(Drive::MR);
        s.steering.max_angle = 0.7854f; // as ai::adapt_spec_for_ai
        struct G : Slope {
            float h = 0;
            SurfaceHit probe(float x, float z, float y_max) const override {
                SurfaceHit r = Slope::probe(x, z, y_max - h);
                r.height += h;
                return r;
            }
        } g;
        g.grade = 0.05f;
        g.h = gn;
        auto car = Vehicle::create(s);
        car->set_cpu_driver(true);
        car->reset({0, gn, 0}, yaw_quat(hn));
        drive(*car, g, nullptr, ticks, every, [](int i) {
            ControlInputs in;
            in.steer_angle = i < 150 ? 0.02f * std::sin(float(i) * 0.05f) : i < 270 ? 0.06f : -0.04f;
            in.throttle = i < 250 ? 1.0f : i < 280 ? 0 : 0.6f;
            in.brake = i >= 250 && i < 280 ? 0.3f : 0;
            return in;
        }, rows);
    }
};

// FR with TCS / ASCC, moving start: braking into a tight turn (the ASCC understeer channel
// relieves the brake pressure and scales the ABS target slip, ABS limits the pressure), then
// a lift with the steering held (the oversteer channel brakes the outside wheels without the
// pedal), straightened before the slide grows.
struct AsccScenario : Scenario {
    AsccScenario() { name = "ascc", table = "kGoldenAscc", ticks = 240; }
    void run(float hn, float gn, std::vector<GoldenRow>& rows) const override {
        auto car = Vehicle::create(synthetic_spec_car(Drive::FR, true));
        FlatGround g;
        g.height = gn;
        car->spawn_moving({0, gn, 0}, yaw_quat(hn), 110 / 3.6f);
        drive(*car, g, nullptr, ticks, every, [](int i) {
            ControlInputs in;
            if (i < 50) {
                in.steer = 0.6f;
                in.brake = 0.5f;
            } else if (i < 120) {
                in.steer = 0.6f;
            } else {
                in.throttle = 0.4f;
            }
            return in;
        }, rows);
    }
};

// The placeholder rigid body (CarSpec::test_car, no suspension block: start-of-tick suspension,
// forces at the contact points, hull contact): the other branch of Vehicle::tick, which a
// restructuring has to keep as well. AWD (L3 coupling) on a gentle slope: standing start,
// braking in a turn, then part throttle into a glancing wall contact and away from it.
struct PlaceholderScenario : Scenario {
    PlaceholderScenario() { name = "placeholder", table = "kGoldenPlaceholder", ticks = 360; }
    void run(float hn, float gn, std::vector<GoldenRow>& rows) const override {
        struct G : Slope {
            float h = 0;
            SurfaceHit probe(float x, float z, float y_max) const override {
                SurfaceHit r = Slope::probe(x, z, y_max - h);
                r.height += h;
                return r;
            }
        } g;
        g.grade = 0.02f;
        g.h = gn;
        SegmentWalls w;
        w.add(4, 0, 14, -150); // the track (left of a -> b) is the -X side, where the car is
        auto car = Vehicle::create(CarSpec::test_car(Drive::AWD));
        car->reset({0, gn, 0}, yaw_quat(hn));
        drive(*car, g, &w, ticks, every, [](int i) {
            ControlInputs in;
            in.throttle = i < 150 ? 1.0f : i < 190 ? 0 : 0.6f;
            in.brake = i >= 150 && i < 190 ? 0.6f : 0;
            in.steer = i < 120 ? 0 : i < 215 ? 0.25f : -0.5f;
            return in;
        }, rows);
    }
};

// Generated with GT3R_GOLDEN_PRINT=1 from the code at the time the tests were written.
const std::vector<GoldenRow> kGoldenLaunch = {
    {0, -0.009917275, -0.2856806, -0, 0.02156155, 0, 7.801647,
     0, 2186.895, 1, 2, 4, 4734.198, 7509.907,
     0.06760331, 0.09878144, 6.990686, 10.71911, 0.2314751, 0, 0,
     0.6668593, 0, -19624.58},
    {0, -0.007113587, -2.55491, -0, 0.02542121, 0, 24.16828,
     0, 3585.8, 1, 1, 4, 4822.294, 7311.875,
     0.05909885, 0.10174, 21.65605, 26.93717, 0.4085619, 0, 0,
     0.7111487, 0, -19246.91},
    {0, -0.00635576, -7.054629, -0, 0.02594258, 0, 40.07428,
     0, 5618.239, 1, 1, 4, 4739.953, 7219.956,
     0.05764294, 0.10174, 35.90866, 42.20521, 0.5, 0, 0,
     0.6027263, 0, -18737.56},
    {0, -0.004455535, -13.71595, -0, 0.01507053, 0, 52.97644,
     0, 6814.164, 2, 3, 4, 7376.417, 6086.552,
     0.06635714, 0.08997545, 47.46962, 51.79822, 0.5, 0, 0,
     0, 0, -18461.94},
    {0, -0.001731689, -21.08073, -0, 0.0003239742, 0, 52.8333,
     0, 5765.385, 2, 3, 4, 6403.984, 6639.122,
     0.08529907, 0.06499757, 47.34141, 49.91946, 0.3882985, 0, 0,
     0.3075428, 0, -18423.49},
    {0, -0.001844424, -28.42845, -0, -9.504982e-05, 0, 52.6913,
     0, 4878.026, 2, 3, 4, 6376.177, 6628.104,
     0.08601288, 0.06449679, 47.21417, 49.80241, 0.1659169, 0, 0,
     0.3102353, 0, -18381.56},
    {0, -0.001849756, -35.75688, -0, -0.0001084936, 0, 52.5523,
     0, 4127.241, 2, 3, 4, 6374.708, 6627.452,
     0.08603714, 0.06448287, 47.08962, 49.67487, 0.06596553, 0, 0,
     0.3110586, 0, -18339.34},
    {0, -0.007903566, -43.68521, -0, 0.01801747, 0, 62.32475,
     0, 4879.186, 2, 1, 4, 5480.433, 7116.304,
     0.0695663, 0.09289832, 55.84626, 59.34346, 0.5, 0, 0,
     0.09813333, 0, -18299.22},
    {0, -0.006949747, -53.13133, -0, 0.01828252, 0, 73.0891,
     0, 5655.597, 2, 1, 4, 5678.997, 7058.16,
     0.06804098, 0.09256718, 65.49168, 68.78661, 0.5, 0, 0,
     0.05350018, 0, -18254.08},
    {0.3516765, -0.002632353, -61.74173, -0.0613609, -0.02910081, -0.006859506, 50.8679,
     -0.2522306, 3527.52, 1, 1, 4, 12713.32, 8937.342,
     0.1248925, 0.02159765, 28.22308, 26.20822, 0.5, 0.08996719, 0,
     2.975903, 0.09887765, -18161.77},
    {1.222704, -0.008033378, -67.57362, -0.1917528, 0.007187515, 0.004310397, 42.13794,
     -0.01816168, 5580.006, 1, 1, 4, 5312.431, 6711.2,
     0.08176439, 0.08077031, 37.62901, 41.918, 0.4486254, -0.1026298, 0,
     0.009360733, 0.1005217, -18076.44},
    {1.971194, -0.004609141, -73.94484, -0.07994807, 0.009058607, 0.02827643, 48.99899,
     0.3316458, 6880.026, 1, 1, 4, 5781.836, 4679.301,
     0.05604113, 0.1002219, 43.08092, 51.68397, 0.5, -0.09516945, 0,
     0.4832308, 0.07396417, -17921.47},
    {1.759128, -0.002427728, -81.06676, 0.1040458, 0.004796476, 0.04114908, 53.09914,
     0.3880993, 7497.375, 1, 1, 4, 5783.68, 3827.468,
     0.04929807, 0.10174, 46.59946, 56.32161, 0.5, -0.09079844, 0,
     0.6507868, 0.0544098, -17803.2},
    {0.138076, -0.0010996, -88.49716, 0.3032673, 0.00295774, 0.0464137, 56.09888,
     0.4043174, 7438.359, 1, 1, 4, 5736.443, 3420.303,
     0.04638523, 0.10174, 49.22662, 55.87828, 0.5, -0.08763227, 0,
     0.688978, 0.0386598, -17742.74},
    {-3.022439, 0.0004088208, -95.72223, 0.5209507, 0.000307057, 0.05369074, 56.97632,
     0.4965728, 6628.578, 2, 3, 4, 7565.336, 2442.494,
     0.04496091, 0.1012023, 49.7618, 48.55322, 0.3933592, -0.08694449, 0,
     0.5084963, 0.0243353, -17723.91},
};
const std::vector<GoldenRow> kGoldenCorner = {
    {0, -0.006995497, -11.66116, -0, 0.005206663, 0, 73.51027,
     0, 5985.445, 2, 1, 4, 4971.955, 6963.828,
     0.08308979, 0.07798766, 72.79842, 72.79842, 0.5, 0, 0,
     0.06787448, 0, 0},
    {0.01743586, -0.007298311, -23.94714, -0.003128437, 0.00663153, -0.008281768, 77.94909,
     -0.05413226, 5455.724, 2, 1, 4, 8284.525, 7925.16,
     0.09058928, 0.07023899, 63.22055, 66.35564, 0.5, 0.04121152, 0,
     0.6597277, 0, 0},
    {0.6490791, -0.001901609, -36.78758, -0.100055, 0.004792437, -0.03239607, 80.75204,
     -0.2251121, 6323.959, 2, 1, 4, 5254.107, 9172.938,
     0.1031663, 0.04776482, 76.91563, 70.81312, 0.5, 0.04150189, 0,
     0.203803, 0, 0},
    {2.835935, -0.0006142143, -49.978, -0.2465243, 0.00481054, -0.0452813, 84.29086,
     -0.2662213, 6068.471, 2, 1, 4, 7651.07, 9486.049,
     0.1145382, 0.03340474, 66.95406, 73.80823, 0.5, 0.04045879, 0,
     0.519037, 0, 0},
    {7.06533, 0.0005459441, -63.16843, -0.3990051, 0.003256079, -0.04731849, 86.77352,
     -0.2563903, 6779.758, 2, 1, 4, 5295.487, 9612.963,
     0.1146075, 0.03160765, 82.4593, 76.86823, 0.5, 0.03942214, 0,
     0.3863167, 0, 0},
    {13.41536, 0.0004136392, -75.99559, -0.5500373, 0.004438841, -0.04606812, 89.90643,
     -0.2542317, 6527.511, 2, 1, 4, 7496.325, 9156.062,
     0.1145887, 0.0313606, 74.96574, 79.39133, 0.5, 0.03847629, 0,
     0.4625379, 0, 0},
    {21.65426, -0.001433903, -88.16216, -0.6907147, 0.0005147349, -0.04386425, 90.77972,
     -0.2332916, 6404.755, 3, 3, 4, 6216.057, 9354.273,
     0.1182029, 0.03187365, 84.33523, 79.30627, 0.4416586, 0.009109649, 0,
     0.4565799, 0, 0},
    {31.01822, -0.003937437, -99.48904, -0.736248, 0.0007113911, -0.01647506, 90.68549,
     0.05300079, 5270.125, 3, 3, 4, 8208.317, 6931.277,
     0.1022451, 0.05205502, 76.66443, 77.5778, 0.1676649, -0.02862163, 0,
     0.4282067, 0, 0},
    {40.51434, -0.002423872, -110.7172, -0.6555575, 0.003021582, 0.02354987, 91.38891,
     0.2119153, 4968.411, 3, 1, 4, 6021.619, 4449.417,
     0.06590158, 0.08583012, 81.36044, 87.68341, 0.3038361, -0.02838587, 0,
     0.435161, 0, 0},
    {49.14677, -0.0008126069, -122.9623, -0.4980675, 0.003863997, 0.04719668, 93.63493,
     0.3139251, 4850.254, 3, 1, 4, 7540.132, 3065.977,
     0.04668075, 0.10174, 82.5882, 82.5882, 0.4268866, -0.02790767, 0,
     0.4929314, 0, 0},
    {56.09361, 0.002164003, -136.6001, -0.293591, 0.002489751, 0.05267793, 95.42185,
     0.3863965, 5192.426, 3, 1, 4, 5519.707, 2770.149,
     0.039347, 0.10174, 83.11893, 94.01266, 0.4731977, -0.02744153, 0,
     0.6130242, 0, 0},
    {60.89769, 0.002768157, -151.457, -0.04131898, 0.003807932, 0.05274741, 97.31531,
     0.473976, 4963.785, 3, 1, 4, 7228.617, 2567.36,
     0.03868505, 0.10174, 84.52136, 84.52136, 0.4928601, -0.02734408, 0,
     0.8591267, 0, 0},
};
const std::vector<GoldenRow> kGoldenJumpWall = {
    {0, 0.005978399, -10.88494, -0, 0.0128048, 0, 81.45415,
     0, 6208.77, 2, 1, 4, 5659.956, 5882.722,
     0.07004987, 0.06548648, 75.51462, 72.98758, 0.5, 0, 0,
     0, 0, 0},
    {0, 0.3901118, -22.60887, -0, 0.1669942, 0, 84.85567,
     0, 7222.767, 3, 3, 4, 23113.98, 22079.18,
     0.13261, 0.09488, 67.46553, 76.01785, 0.5, 0, 0,
     1.882908, 0, 0},
    {0, 1.708288, -34.05818, -0, -0.03774249, 0, 82.81281,
     0, 6111.1, 3, 3, 0, 0, 0,
     -0.00245, -0.002177778, 68.70496, 74.19153, 0.3858474, 0, 0,
     0, 0, 0},
    {0, 0.5900602, -45.49361, -0, -0.27172, 0, 84.63744,
     0, 5170.531, 3, 3, 0, 0, 0,
     -0.00245, -0.002177778, 68.70496, 74.19153, 0.1650417, 0, 0,
     0, 0, 0},
    {0.1554496, 0.01060774, -57.36641, -0.02892441, 0.02128929, -0.01036695, 87.02856,
     -0.1141538, 4719.879, 3, 1, 4, 5146.486, 6531.331,
     0.06040991, 0.06646015, 80.36823, 77.71082, 0.5, 0.02961442, 0,
     0, 0, 0},
    {0.9354371, 0.004106994, -69.74944, -0.1094169, 0.01044093, -0.03167174, 91.57906,
     -0.1926393, 4886.179, 3, 1, 4, 6432.474, 8332.842,
     0.09858459, 0.04081328, 83.19991, 81.59059, 0.5, 0.02844392, 0,
     0.02633667, 0, 0},
    {2.79354, 0.003521268, -82.61232, -0.1889269, 0.009804996, -0.03382123, 95.76467,
     -0.1354574, 5247.369, 3, 1, 4, 6798.221, 8062.106,
     0.102507, 0.03789856, 82.94345, 85.47815, 0.5, 0.02736491, 0,
     0.4954471, 0, 0},
    {5.691061, 0.003105233, -95.8876, -0.246414, 0.009665836, -0.02695259, 99.93842,
     -0.1045906, 5345.184, 3, 1, 4, 6850.976, 7686.791,
     0.09786512, 0.04331541, 86.2085, 89.29509, 0.5, 0.02626768, 0,
     0.3072276, 0, 0},
    {7.39818, 0.00902706, -108.3848, -0.1817838, 0.002480651, -0.06271335, 85.51594,
     0.3853827, 5280.594, 3, 1, 4, 5621.485, 10485.99,
     0.1274192, 0.003021813, 89.91586, 77.47752, 0.5, -0.0597944, 10511.04,
     0.7786279, 0, 0},
    {8.042678, -0.0009081961, -120.2537, 0.05024179, 0.004026166, 0.04831615, 85.87162,
     0.44078, 4461.036, 3, 1, 4, 8116.194, 2262.95,
     0.054182, 0.09488, 87.45712, 77.99728, 0.5, -0.05982917, 12024.95,
     0.3588065, 0, 0},
    {6.731424, 0.005734508, -132.1985, 0.2108036, 0.003480691, 0.05623784, 87.59736,
     0.2256436, 4638.751, 3, 1, 4, 7467.898, 2335.116,
     0.04135798, 0.09477059, 89.48461, 79.02208, 0.4783456, -0.05893371, 12024.95,
     0.4689774, 0, 0},
    {3.686693, 0.002832437, -144.0831, 0.3131348, 0.004487927, 0.04802658, 89.34337,
     0.2223628, 4586.798, 3, 1, 4, 7725.292, 3241.361,
     0.04862086, 0.09328412, 84.81567, 80.59893, 0.4602749, -0.05800554, 12024.95,
     0.2094035, 0, 0},
};
const std::vector<GoldenRow> kGoldenCpu = {
    {-0.0004076724, -0.002609462, 0.02902888, 0.000267163, 0.05496773, -4.03965e-07, 0.4531821,
     0.0009453637, 5354.235, 1, 2, 4, 4097.514, 6738.072,
     0.06674372, 0.08026879, 0.3288801, 11.48225, 0.4167989, 0.01985426, 0,
     1.680964, 0, 0},
    {0.01122239, 0.0649118, -1.439958, -0.005384673, 0.07726532, 0.0002374777, 20.32354,
     -0.014859, 4271.87, 1, 2, 4, 3242.772, 8369.074,
     0.04145201, 0.11546, 18.2469, 24.96074, 0.5, 0.003808452, 0,
     0.7668961, 0, 0},
    {0.02033969, 0.2844349, -5.799452, 0.0004967639, 0.07833894, 0.0007110374, 41.54204,
     0.04488517, 5477.124, 1, 1, 4, 3317.312, 8065.19,
     0.03747368, 0.11546, 37.1191, 40.62379, 0.5, -0.01931546, 0,
     0.2895464, 0, 0},
    {-0.109095, 0.6313381, -12.70769, 0.02788972, 0.0584893, 0.001950591, 53.27288,
     0.0547604, 6675.554, 2, 3, 4, 4944.233, 5949.537,
     0.05661574, 0.09500957, 47.60489, 52.88425, 0.5, -0.006541091, 0,
     0.3921128, 0, 0},
    {-0.3420304, 1.000113, -20.05129, 0.0341848, 0.04984765, -0.003600911, 52.30485,
     -0.04887693, 5648.109, 2, 3, 4, 4551.327, 6710.06,
     0.07150911, 0.07732204, 46.98303, 48.23935, 0.3882985, 0.01839007, 0,
     0.5511924, 0, 0},
    {-0.152185, 1.35963, -27.23526, -0.07758657, 0.0490637, -0.02522051, 51.09766,
     -0.3172775, 4778.8, 2, 3, 4, 4952.392, 8631.113,
     0.09288129, 0.05556015, 46.56613, 44.62719, 0.1659169, 0.06, 0,
     0.04319537, 0, 0},
    {1.048326, 1.706995, -34.15734, -0.2562453, 0.0479247, -0.02786579, 50.09229,
     -0.3747782, 4052.826, 2, 1, 4, 5024.018, 9110.283,
     0.1004137, 0.04575561, 45.80899, 41.7336, 0.06596553, 0.06, 0,
     0.006441351, 0, 0},
    {3.526815, 2.046479, -41.03981, -0.4020168, 0.05672466, -0.009512552, 55.76879,
     -0.2392537, 4404.922, 2, 1, 4, 5914.314, 9213.468,
     0.08434592, 0.07131281, 50.55751, 58.99581, 0.4434149, 0.06, 0,
     0.84561, 0, 0},
    {7.049644, 2.393603, -47.88936, -0.5467346, 0.02987166, -0.00704267, 51.50505,
     -0.3881254, 5478.451, 1, 3, 4, 5837.705, 8711.238,
     0.1090085, 0.03698978, 51.9781, 40.4621, 0.4856433, 0.06, 0,
     0.8932161, 0.020935, 0},
    {10.73993, 2.674676, -53.73188, -0.5874195, 0.05590778, 0.02503396, 52.63877,
     0.1488839, 6276.185, 1, 1, 4, 5252.958, 6101.364,
     0.06770335, 0.1005779, 46.81547, 46.12494, 0.5, -0.04, 0,
     0.3482189, 0.01495947, 0},
    {14.43833, 2.99547, -59.96978, -0.4888773, 0.04406668, 0.04987757, 51.60542,
     0.2399816, 6055.459, 2, 3, 4, 4830.411, 4509.976,
     0.05132599, 0.09864227, 45.66636, 57.47913, 0.3079169, -0.04, 0,
     0.1863576, 0.003255418, 0},
    {17.36271, 3.319614, -66.45084, -0.3636204, 0.046407, 0.04935447, 50.62355,
     0.2550353, 5123.454, 2, 3, 4, 4879.377, 4385.283,
     0.04753337, 0.1023471, 44.74766, 57.56979, 0.1271781, -0.04, 0,
     0.07632163, 0, 0},
};
const std::vector<GoldenRow> kGoldenAscc = {
    {0.5756532, 0.001498918, -14.24456, -0.1120558, -0.01395391, -0.03764948, 96.11163,
     -0.440579, 4811.289, 3, 1, 4, 10017.79, 10015.52,
     0.1248925, 0.01485417, 76.62858, 81.92473, 0.5, 0.1071511, 0,
     0.715099, 0.1499838, 0},
    {2.746654, 0.007396367, -26.54667, -0.3661016, -0.0005380051, -0.06408467, 86.46011,
     -0.6073908, 5717.477, 2, 1, 4, 5894.143, 11793.25,
     0.1248925, 0.006936612, 77.57451, 54.46875, 0.405841, 0.118792, 0,
     1.884735, 0.194903, 0},
    {6.957619, 0.01134985, -37.58183, -0.7250386, -0.000120145, -0.06806356, 83.39326,
     -0.8125146, 5222.785, 2, 1, 4, 5354.159, 10420.4,
     0.1244807, 0.0004445499, 73.01841, 49.7359, 0.1754873, 0.1239903, 0,
     3.735793, 0.1509531, 0},
    {12.56809, 0.01050709, -47.04907, -1.124746, -0.004601036, -0.06056298, 74.33508,
     -0.7711217, 4582.618, 2, 1, 4, 6445.68, 10207.32,
     0.1248925, 0.001102773, 54.83098, 49.5088, 0.07016218, 0.1541599, 0,
     6.001486, 0.1612355, 0},
    {18.88879, 0.007204338, -54.58945, -1.523612, 0.001295697, -0.06169684, 67.31855,
     -0.8132914, 4125.474, 2, 1, 4, 5766.174, 10172.78,
     0.1222335, 0.01073391, 41.83891, 43.04645, 0.2625, 0, 0,
     6.56145, 0.1749314, 0},
    {25.78236, 0.006519014, -60.16275, -1.937337, 0.00531527, -0.05482674, 60.44712,
     -0.8175045, 5063.231, 1, 1, 4, 5198.104, 10280.29,
     0.1123817, 0.02196349, 33.29652, 30.73317, 0.3782016, 0, 0,
     7.228177, 0.1776381, 0},
    {33.01479, 0.006053104, -63.61699, -2.314583, 0.008311931, -0.04835367, 54.65413,
     -0.6764058, 3957.434, 1, 1, 4, 4090.42, 9745.858,
     0.1033201, 0.03193225, 25.2069, 23.73518, 0.3714808, 0, 0,
     7.09432, 0.1724975, 0},
    {40.07104, 0.005942112, -64.93232, -2.616134, 0.008999959, -0.04671567, 48.54473,
     -0.537362, 3270.687, 1, 1, 4, 4211.948, 9526.589,
     0.1015022, 0.03388695, 20.41968, 21.37069, 0.3283579, 0, 0,
     6.36891, 0.1642127, 0},
};
const std::vector<GoldenRow> kGoldenPlaceholder = {
    {0, 0.005670152, -0.415101, -0, 0.03991538, 0, 7.984896,
     0, 2110.771, 1, 2, 4, 6992.54, 8237.934,
     0.07263053, 0.1267935, 20.93087, 6.802161, 0, 0, 0,
     2.943202, 0, 0},
    {0, 0.04426669, -2.319768, -0, 0.03638383, 0, 18.71437,
     0, 2062.125, 1, 1, 4, 7250.035, 8231.293,
     0.07725011, 0.1210395, 15.91728, 15.91728, 0, 0, 0,
     0, 0, 0},
    {0, 0.1120786, -5.747957, -0, 0.04215109, 0, 30.81086,
     0, 4171.626, 1, 1, 4, 6538.269, 8232.958,
     0.07050258, 0.1293306, 38.67436, 25.72615, 0, 0, 0,
     1.194416, 0, 0},
    {0, 0.2162329, -10.93881, -0, 0.03954497, 0, 43.23001,
     0, 7707.994, 1, 1, 4, 6869.551, 8231.898,
     0.07350695, 0.125644, 82.06613, 36.92793, 0, 0, 0,
     3.11986, 0, 0},
    {0.220943, 0.3526603, -17.729, -0.05915949, 0.03482138, -0.01082945, 53.48866,
     -0.235632, 7760.478, 1, 1, 4, 7153.287, 10122.86,
     0.08722242, 0.1107591, 47.00445, 46.03379, 0, 0.09102305, 0,
     1.816424, 0, 0},
    {1.067802, 0.487144, -24.22544, -0.1939243, 0.0003846056, -0.01000726, 41.15669,
     -0.2890776, 4532.968, 1, 1, 4, 11172.55, 10358.68,
     0.1304396, 0.05833558, 35.46145, 34.52047, 0, 0.09968426, 0,
     0, 0.08546674, 0},
    {2.636633, 0.5882947, -29.52277, -0.3487012, 0.03547731, -0.00632742, 42.94663,
     -0.3275067, 5269.453, 1, 1, 4, 7412.517, 10284.82,
     0.08566906, 0.1128292, 37.30733, 35.81791, 0, 0.09913441, 0,
     0.8125231, 0.08638787, 0},
    {4.86218, 0.7070687, -35.47034, -0.417232, 0.03535239, -0.0009656256, 40.57269,
     0.1914831, 4391.685, 1, 1, 4, 6773.537, 9960.282,
     0.08135279, 0.1169908, 34.52604, 33.45225, 0, -0.1916233, 7960.631,
     0.7787672, 0.05841516, 0},
    {5.515985, 0.8220823, -41.20235, -0.2495837, 0.03454763, -0.0001965099, 43.83549,
     0.5381054, 5981.903, 1, 1, 4, 7390.463, 8880.165,
     0.08356734, 0.1143664, 36.59401, 38.5054, 0, -0.1973357, 11203.88,
     1.376296, 0.03659226, 0},
    {5.97266, 0.9556648, -47.8577, 0.06201961, 0.03172608, 0.02090592, 51.54072,
     0.5989468, 7108.757, 1, 1, 4, 7709.275, 4789.001,
     0.06688993, 0.1303475, 87.27818, 45.13634, 0, -0.1850847, 11203.88,
     1.696184, 0.01796713, 0},
    {4.88358, 1.104284, -55.24848, 0.3307484, 0.02485838, 0.02123621, 55.6748,
     0.476936, 7464.838, 1, 1, 4, 8376.162, 3972.264,
     0.06924658, 0.1263689, 88.63531, 48.29256, 0, -0.1794963, 11203.88,
     1.939228, 0.006352229, 0},
    {1.964968, 1.253015, -62.7082, 0.5370077, 0.02631383, 0.01345058, 59.71422,
     0.3517628, 7668.705, 1, 1, 4, 7816.535, 4684.495,
     0.0689355, 0.1274894, 84.53283, 51.56713, 0, -0.1742256, 11203.88,
     1.712978, 0, 0},
};

double tolerance(int field, double expected) { return kGoldenTol[field].abs + 1e-4 * std::fabs(expected); }

void check_golden(const Scenario& sc, const std::vector<GoldenRow>& expected) {
    std::vector<GoldenRow> rows = sc.rows();
    if (std::getenv("GT3R_GOLDEN_PRINT")) {
        std::printf("const std::vector<GoldenRow> %s = {\n", sc.table);
        for (const GoldenRow& r : rows)
            for (int f = 0; f < kGFieldCount; ++f)
                std::printf("%s%.7g%s", f == 0 ? "    {" : f % 7 == 0 ? "     " : "", r[size_t(f)],
                            f + 1 == kGFieldCount ? "},\n" : f % 7 == 6 ? ",\n" : ", ");
        std::printf("};\n");
    }
    if (std::getenv("GT3R_GOLDEN_SENSITIVITY")) {
        // Worst |nominal - perturbed| / tolerance over the checkpoints, per perturbation
        // (positions and heading are compared after removing the nudge itself).
        const std::pair<float, float> nudges[] = {{1e-6f, 0}, {-1e-6f, 0}, {0, 1e-5f}, {0, -1e-5f}};
        for (auto [hn, gn] : nudges) {
            std::vector<GoldenRow> p = sc.rows(hn, gn);
            double worst = 0;
            int wf = 0, wt = 0;
            for (size_t k = 0; k < rows.size() && k < p.size(); ++k) {
                // undo the nudge: rotate the perturbed position back by -hn about the origin, lower it by gn
                double c = std::cos(double(hn)), sn = std::sin(double(hn));
                GoldenRow q = p[k];
                q[kGx] = c * p[k][kGx] - sn * p[k][kGz];
                q[kGz] = sn * p[k][kGx] + c * p[k][kGz];
                q[kGy] = p[k][kGy] - gn;
                q[kGHeading] = p[k][kGHeading] - hn;
                double row_worst = 0;
                int rf = 0;
                for (int f = 0; f < kGFieldCount; ++f) {
                    double ratio = std::fabs(q[size_t(f)] - rows[k][size_t(f)]) / tolerance(f, rows[k][size_t(f)]);
                    if (kGoldenTol[f].abs == 0) ratio = q[size_t(f)] == rows[k][size_t(f)] ? 0 : 1e9;
                    if (ratio > worst) worst = ratio, wf = f, wt = int(k + 1) * sc.every;
                    if (ratio > row_worst) row_worst = ratio, rf = f;
                }
                if (std::string(std::getenv("GT3R_GOLDEN_SENSITIVITY")) == "2")
                    std::printf("    tick %d: %.3g (%s)\n", int(k + 1) * sc.every, row_worst, kGoldenTol[rf].name);
            }
            std::printf("  %s sensitivity (heading %+.0e rad, ground %+.0e m): worst %.3f of the tolerance (%s, tick %d)\n",
                        sc.name, double(hn), double(gn), worst, kGoldenTol[wf].name, wt);
        }
    }
    CHECK(rows.size() == size_t(sc.ticks / sc.every));
    if (expected.empty()) {
        std::printf("  %s: no golden values yet\n", sc.name);
        CHECK(false);
        return;
    }
    CHECK(rows.size() == expected.size());
    int bad = 0;
    for (size_t k = 0; k < rows.size() && k < expected.size(); ++k)
        for (int f = 0; f < kGFieldCount; ++f)
            if (std::fabs(rows[k][size_t(f)] - expected[k][size_t(f)]) > tolerance(f, expected[k][size_t(f)])) {
                if (++bad <= 10)
                    std::printf("  FAIL %s tick %d %s: %.9g, golden %.9g\n", sc.name, int(k + 1) * sc.every, kGoldenTol[f].name,
                                rows[k][size_t(f)], expected[k][size_t(f)]);
            }
    CHECK(bad == 0);
}

} // namespace

TEST(sim_spec_golden_launch) { check_golden(LaunchScenario(), kGoldenLaunch); }
TEST(sim_spec_golden_corner) { check_golden(CornerScenario(), kGoldenCorner); }
TEST(sim_spec_golden_jump_wall) { check_golden(JumpWallScenario(), kGoldenJumpWall); }
TEST(sim_spec_golden_cpu) { check_golden(CpuScenario(), kGoldenCpu); }
TEST(sim_spec_golden_ascc) { check_golden(AsccScenario(), kGoldenAscc); }
TEST(sim_spec_golden_placeholder) { check_golden(PlaceholderScenario(), kGoldenPlaceholder); }
