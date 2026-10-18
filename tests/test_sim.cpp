// Vehicle simulation tests on a flat infinite plane (no game data needed),
// plus sanity checks against real cars from the decoded database (need
// GT3R_DISC).

#include "test_framework.h"

#include "game/data/car_database.h"
#include "sim/models.h"
#include "sim/track_walls.h" // compile check of the track adapter
#include "sim/vehicle.h"
#include "sim/world.h"
#include "track/track.h"
#include "disc/volume.h"
#include "vfs/vfs.h"
#include "ai/ai_driver.h"
#include "race/demo_replay.h"
#include "race/field.h"
#include "race/license.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

using namespace gt3r;
using namespace gt3r::sim;

namespace {

struct FlatGround : TrackQuery {
    float height = 0;
    SurfaceHit probe(float, float, float y_max) const override {
        SurfaceHit h;
        h.hit = y_max >= height;
        h.height = height;
        return h;
    }
};

constexpr double kDt = 1.0 / kDefaultTickHz;

Vec3 rotate_fwd(const Quat& q) {
    Vec3 u{q.x, q.y, q.z}, f{0, 0, -1};
    Vec3 t = u.cross(f) * 2.0f;
    return f + t * q.w + u.cross(t);
}

bool finite_state(const VehicleState& s) {
    auto f = [](const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); };
    bool ok = f(s.position) && f(s.velocity) && f(s.angular_velocity) && std::isfinite(s.engine_rpm);
    for (const auto& w : s.wheels) ok = ok && std::isfinite(w.load) && std::isfinite(w.slip_ratio) && f(w.contact);
    return ok;
}

float forward_speed(const Vehicle& v) {
    const Quat& q = v.state().orientation;
    // car -Z in world: rotate (0,0,-1)
    Vec3 u{q.x, q.y, q.z};
    Vec3 f{0, 0, -1};
    Vec3 t = u.cross(f) * 2.0f;
    Vec3 fw = f + t * q.w + u.cross(t);
    return v.state().velocity.dot(fw);
}

struct AccelResult {
    double t100 = -1;     // 0-100 km/h, s
    double t400 = -1;     // standing 400 m, s
    float top_kmh = 0;
    bool finite = true;
};

AccelResult accelerate(const CarSpec& spec, double seconds) {
    auto car = Vehicle::create(spec);
    FlatGround g;
    car->reset({}, {});
    ControlInputs in;
    in.throttle = 1;
    AccelResult r;
    Vec3 start = car->state().position;
    for (int i = 0; i < int(seconds / kDt); ++i) {
        car->step(in, g, kDt);
        double t = (i + 1) * kDt;
        float kmh = forward_speed(*car) * 3.6f;
        if (r.t100 < 0 && kmh >= 100) r.t100 = t;
        if (r.t400 < 0 && (car->state().position - start).length() >= 402.3f) r.t400 = t;
        r.top_kmh = std::max(r.top_kmh, kmh);
        if (!finite_state(car->state())) {
            r.finite = false;
            break;
        }
    }
    return r;
}

data::CarDatabase* database() {
    static data::CarDatabase db;
    static bool tried = false, ok = false;
    if (!tried) {
        tried = true;
        if (Disc* d = test::test_disc()) ok = db.load(d->volume());
    }
    return ok ? &db : nullptr;
}

} // namespace

// --- pure sub-models -------------------------------------------------------------

TEST(sim_curve_torque_interpolates) {
    EngineSpec e;
    e.curve_rpm = {1000, 2000, 4000};
    e.curve_torque = {100, 200, 100};
    CHECK(std::fabs(model::curve_torque(e, 1500) - 150) < 1e-3f);
    CHECK(std::fabs(model::curve_torque(e, 3000) - 150) < 1e-3f);
    CHECK(std::fabs(model::curve_torque(e, 9000) - 100) < 1e-3f); // clamped
    CHECK(std::fabs(model::curve_torque(e, 500) - 100) < 1e-3f);  // clamped below the axis (spec 2.1)
}

TEST(sim_engine_limiter_hysteresis) {
    EngineSpec e;
    e.curve_rpm = {1000, 7000};
    e.curve_torque = {200, 200};
    e.rev_limit_rpm = 7000;
    auto a = model::engine_torque(e, 7010, 1, false);
    CHECK(a.limiter && a.torque < 0);
    auto b = model::engine_torque(e, 6950, 1, true); // within hysteresis
    CHECK(b.limiter);
    auto c = model::engine_torque(e, 6400, 1, true); // clears 500 rpm below the limit (spec 2.2)
    CHECK(!c.limiter && c.torque > 150);
    auto idle = model::engine_torque(e, 600, 0, false); // closed throttle: -k_f * omega
    CHECK(idle.throttle == 0 && idle.torque < 0);
    auto coast = model::engine_torque(e, 5000, 0, false);
    CHECK(coast.torque < 0);
}

TEST(sim_tire_force_shape) {
    TireSpec t;
    t.default_curves();
    t.fz_ref = 4000;
    float peak = model::tire_peak_force(t, 4000, 0, 1);
    CHECK(std::fabs(peak - 4000) < 1);
    CHECK(model::tire_peak_force(t, 8000, 0, 1) < 2 * peak); // load sensitivity
    model::TireIn in;
    in.peak = peak;
    in.vx = 30;
    // Longitudinal: braking slip (s > 0) gives a rearward force; drive slip forward.
    in.slip_ratio = 0.1f;
    float brake = model::tire_force(t, in).fx;
    in.slip_ratio = -0.1f;
    float drive = model::tire_force(t, in).fx;
    in.slip_ratio = -1.0f;
    float spin = model::tire_force(t, in).fx;
    CHECK(brake < -0.9f * peak && drive > 0.9f * peak && spin > 0.5f * peak && spin < drive);
    // Lateral: sliding right (vy > 0) gives a leftward force; slip ratio shrinks it (friction circle).
    in.slip_ratio = 0;
    in.vy = 30 * std::tan(0.14f);
    float fy = model::tire_force(t, in).fy;
    CHECK(fy < -0.8f * peak);
    in.slip_ratio = 0.2f;
    CHECK(std::fabs(model::tire_force(t, in).fy) < std::fabs(fy));
    // Reversing and sliding right also pushes left.
    in.slip_ratio = 0;
    in.vx = -30;
    CHECK(model::tire_force(t, in).fy < 0);
    in.peak = 0;
    CHECK(model::tire_force(t, in).fx == 0);
}

TEST(sim_spec_helpers) {
    // table lookup clamps [spec §2.1]
    const float x[] = {1, 2, 4}, y[] = {10, 20, 0};
    CHECK(model::table_lookup(x, y, 3, 0) == 10 && model::table_lookup(x, y, 3, 9) == 0);
    CHECK(std::fabs(model::table_lookup(x, y, 3, 3) - 10) < 1e-5f);
    // Euler axes at zero and a pure heading [spec §1.3]
    float f[3], l[3], u[3];
    model::euler_axes(0, 0, 0, f, l, u);
    CHECK(f[1] == 1 && l[0] == 1 && u[2] == 1);
    model::euler_axes(0, 0, 1.5707963f, f, l, u);
    CHECK(std::fabs(f[0] + 1) < 1e-5f); // heading +90 deg turns the nose to phys -x (left)
    Vec3 fw = model::phys_to_world(f);
    CHECK(std::fabs(fw.x + 1) < 1e-5f);
    // slip angle fade [spec §3.2]: 1..10 km/h
    float fade;
    CHECK(model::patch_slip_angle(0.2f, 0.1f, fade) == 0 && fade == 0);
    model::patch_slip_angle(10, -1, fade);
    CHECK(fade == 1);
    CHECK(std::fabs(model::patch_slip_angle(10, -10, fade) - 0.7853982f) < 1e-5f);
    // slip ratio (spec 3.4): locked wheel at speed = +1 x gain, dead band, spin at standstill
    CHECK(std::fabs(model::slip_ratio(30, 0, 30) - (0.5f + 0.5f * 30 / 27.78f > 1 ? 1.0f : 0.5f + 0.5f * 30 / 27.78f)) < 1e-4f);
    CHECK(model::slip_ratio(0.001f, 0.001f, 0.001f) == 0);
    CHECK(std::fabs(model::slip_ratio(0, 5, 0) + 0.5f * 5 / (20 * 0.27778f)) < 1e-5f); // low-speed fade
    // surface index (spec 3.6)
    CHECK(model::surface_index(0) == 0 && model::surface_index(0x200) == 12 && model::surface_index(0x201) == 13);
    CHECK(model::surface_index(0x209) == 14 && model::surface_index(0x205) == 5 && model::surface_index(3) == 3);
    CHECK(model::surface_rolling(3) == 25.5f);
    // slipstream and filter/drag [spec §5.2]
    CHECK(model::slipstream_factor(-10, 0, 40, 0.9f) == 0.75f);
    CHECK(model::slipstream_factor(-10, 0, 20, 0.9f) == 0);
    CHECK(model::slipstream_factor(-10, 1.0f, 40, 0.9f) == 0);
    AeroSpec a;
    a.c_drag = 0.4f;
    auto o = model::aero_step(a, 50, 0, 0, 1 / 60.0f);
    CHECK(std::fabs(o.drag + 0.4f * 2500) < 1e-2f && o.slip == 0);
    float slip = 0;
    for (int i = 0; i < 60; ++i) slip = model::aero_step(a, 50, 1, slip, 1 / 60.0f).slip;
    CHECK(slip == 1.0f);
    o = model::aero_step(a, 50, 1, slip, 1 / 60.0f);
    CHECK(std::fabs(o.drag + 0.5f * 0.4f * 2500) < 1e-2f); // full slipstream halves drag
    // torque curve build [spec §2.1]: factor 1180 -> x1.18; tilt between lo% and hi per mille
    const u16 tq[] = {2000, 4000};
    const u8 ax[] = {10, 70};
    float rpm[2], nm[2];
    CHECK(model::build_torque_curve(tq, ax, 2, 1180, 1000, 100, rpm, nm) == 2);
    CHECK(rpm[0] == 1000 && std::fabs(nm[1] - 40 * 9.8f * 1.18f) < 1e-3f);
    model::build_torque_curve(tq, ax, 2, 1000, 1200, 100, rpm, nm);
    CHECK(std::fabs(nm[0] - 20 * 9.8f) < 1e-3f && std::fabs(nm[1] - 40 * 9.8f * 1.2f) < 1e-3f);
}

TEST(sim_turbo_and_suspension_models) {
    // Turbo (spec 2.3, R34 stock bytes: cap 1+0.1*c, stages 0.2/3500/12.5, 0.6/5500/10.5).
    TurboSpec t;
    t.on = true;
    t.cap = 1.6f;
    t.stage[0] = {true, 0.2f, 3500, 12.5f};
    t.stage[1] = {true, 0.6f, 5500, 10.5f};
    CHECK(std::fabs(model::turbo_stage(t.stage[0], 3500) - 0.2f) < 1e-6f);
    CHECK(model::turbo_stage(t.stage[0], 11000) == 0);
    CHECK(model::turbo_boost(t, 5500, 5500) <= t.cap);
    model::TurboState st;
    float first = model::turbo_step(t, st, 1, 6000, 1 / 60.0f);
    float x = first;
    for (int i = 0; i < 600; ++i) x = model::turbo_step(t, st, 1, 6000, 1 / 60.0f);
    CHECK(first < 0.9f);                   // lag: full throttle is not full boost at once
    CHECK(x > 0.99f && st.lag[0] > 5000);  // spools up to the engine rpm
    TurboSpec na;
    CHECK(model::turbo_step(na, st, 0.4f, 6000, 1 / 60.0f) == 0.4f);
    // Damper fit through two points (spec 4.2): F(v) = g (sqrt(v - t) - sqrt(-t)).
    auto d = model::fit_damper(0.1f, 500, 0.5f, 1500);
    CHECK(d.t < 0 && d.g > 0);
    CHECK(std::fabs(d.o + d.g * std::sqrt(-d.t)) < 1e-2f);
    CHECK(std::fabs(d.o + d.g * std::sqrt(0.1f - d.t) - 500) < 1);
    CHECK(std::fabs(d.o + d.g * std::sqrt(0.5f - d.t) - 1500) < 1);
    SuspensionSpec sp;
    sp.k = 40000;
    sp.k2 = 1.5e6f;
    sp.unsprung = 10;
    sp.bump = sp.rebound = d;
    sp.ride_height = 0.13f;
    sp.bump_room = 0.08f;
    sp.droop = 0.1f;
    sp.knee_gap = 0.06f;
    float xs = -0.1f;
    auto rest = model::suspension_wheel(sp, xs, 0, xs);
    CHECK(rest.grounded && std::fabs(rest.load - (4000 + 98)) < 1);
    CHECK(!model::suspension_wheel(sp, 0.05f, 0, xs).grounded);           // beyond max droop
    CHECK(model::suspension_wheel(sp, xs, -0.3f, xs).load > rest.load);  // compressing adds damping
    auto deep = model::suspension_wheel(sp, -0.5f, 0, xs);               // clamped at the bump-stop limit
    CHECK(std::fabs(deep.x - (xs - 0.13f + 0.08f)) < 1e-5f);
    CHECK(model::driveline_class(1, 0xFE) == 1 && model::driveline_class(2, 3) == 3 &&
          model::driveline_class(2, 4) == 2 && model::driveline_class(2, 0) == 1 && model::driveline_class(0, 0xFE) == 0);
}

TEST(sim_gearbox_ratios_and_autoshift) {
    CarSpec s = CarSpec::test_car();
    CHECK(model::total_ratio(s.gearbox, 0) == 0);
    CHECK(model::total_ratio(s.gearbox, -1) < 0);
    CHECK(model::total_ratio(s.gearbox, 1) > model::total_ratio(s.gearbox, 2));
    float up1 = model::upshift_rpm(s.engine, s.gearbox, 1);
    CHECK(up1 > 5000 && up1 < s.engine.rev_limit_rpm);
    model::AutoShiftIn a;
    a.gear = 2;
    a.throttle = 1;
    a.rpm = a.road_rpm = s.engine.rev_limit_rpm - 50;
    CHECK(model::auto_shift(s.engine, s.gearbox, a) == 3);
    a.rpm = a.road_rpm = 5000;
    CHECK(model::auto_shift(s.engine, s.gearbox, a) == 2);
    // Wheelspin: engine high, road speed low -> no down-shift under power.
    a.rpm = 7000;
    a.road_rpm = 2500;
    CHECK(model::auto_shift(s.engine, s.gearbox, a) == 2);
    a.rpm = a.road_rpm = 1200; // bogging
    CHECK(model::auto_shift(s.engine, s.gearbox, a) == 1);
    a.throttle = 0; // coasting keeps the engine in its range
    a.rpm = a.road_rpm = 2500;
    CHECK(model::auto_shift(s.engine, s.gearbox, a) == 2);
    a.rpm = a.road_rpm = 1700;
    CHECK(model::auto_shift(s.engine, s.gearbox, a) == 1);
}

// --- vehicle ---------------------------------------------------------------------

TEST(sim_rests_at_static_ride_height) {
    CarSpec s = CarSpec::test_car();
    auto car = Vehicle::create(s);
    FlatGround g;
    car->reset({0, 0, 0}, {});
    ControlInputs in;
    in.brake = 1;
    for (int i = 0; i < 180; ++i) car->step(in, g, kDt);
    const VehicleState& st = car->state();
    CHECK(std::fabs(st.position.y) < 0.02f);
    CHECK(st.velocity.length() < 0.05f);
    float total = 0;
    for (const auto& w : st.wheels) {
        CHECK(w.grounded);
        total += w.load;
    }
    CHECK(std::fabs(total - s.mass * model::kGravity) < 0.02f * s.mass * model::kGravity);
    float front = st.wheels[0].load + st.wheels[1].load;
    CHECK(std::fabs(front / total - s.front_weight) < 0.03f);
    CHECK(std::fabs(st.engine_rpm - s.engine.idle_rpm) < 250);
}

TEST(sim_test_car_acceleration) {
    AccelResult r = accelerate(CarSpec::test_car(), 60);
    std::printf("  test car (AWD): 0-100 %.2f s, 400 m %.2f s, top %.0f km/h\n", r.t100, r.t400, r.top_kmh);
    CHECK(r.finite);
    // Upshifts go through the spec's state-3 shift clutch (no torque until the engine
    // falls to the new ratio), so each shift costs drive time (spec 2.4, draft 18).
    CHECK(r.t100 > 3.8 && r.t100 < 8.5);
    CHECK(r.top_kmh > 230 && r.top_kmh < 330);
}

TEST(sim_braking_and_abs) {
    CarSpec s = CarSpec::test_car();
    auto car = Vehicle::create(s);
    FlatGround g;
    car->reset({}, {});
    ControlInputs in;
    in.throttle = 1;
    for (int i = 0; i < 60 * 30 && forward_speed(*car) < 100 / 3.6f; ++i) car->step(in, g, kDt);
    in.throttle = 0;
    in.brake = 1;
    Vec3 p0 = car->state().position;
    int ticks = 0;
    while (forward_speed(*car) > 0.1f && ticks < 600) {
        car->step(in, g, kDt);
        ++ticks;
    }
    float dist = (car->state().position - p0).length();
    std::printf("  100-0 km/h: %.1f m in %.2f s\n", dist, ticks * kDt);
    CHECK(dist > 28 && dist < 55); // the clutch stays locked under braking (spec 2.4, draft 18)
}

TEST(sim_steady_cornering) {
    CarSpec s = CarSpec::test_car();
    auto car = Vehicle::create(s);
    FlatGround g;
    car->reset({}, {});
    ControlInputs in;
    in.throttle = 0.35f;
    in.steer = 0.4f;
    float max_lat = 0;
    double yaw_sign = 0;
    for (int i = 0; i < 60 * 15; ++i) {
        car->step(in, g, kDt);
        if (i > 60 * 8) {
            max_lat = std::max(max_lat, std::fabs(car->telemetry().accel_local.x));
            yaw_sign += car->state().angular_velocity.y;
        }
    }
    std::printf("  steady corner: lateral %.2f g, %.0f km/h\n", max_lat / 9.81f, car->state().speed_kmh);
    CHECK(finite_state(car->state()));
    CHECK(max_lat > 2.0f && max_lat < 13.0f);
    // Steering right (positive) turns clockwise seen from above: negative yaw about +Y.
    CHECK(yaw_sign < 0);
}

TEST(sim_drive_layouts_distinct) {
    // Same car, different layouts: AWD launches best; FF puts power down
    // through the front wheels, FR/MR through the rear.
    double t[5] = {};
    for (Drive d : {Drive::FR, Drive::FF, Drive::AWD, Drive::MR}) {
        CarSpec s = CarSpec::test_car(d);
        if (d == Drive::MR) s.front_weight = 0.42f;
        if (d == Drive::FF) s.front_weight = 0.62f;
        s.finalize();
        auto car = Vehicle::create(s);
        FlatGround g;
        car->reset({}, {});
        ControlInputs in;
        in.throttle = 1;
        double front_drive = 0, rear_drive = 0;
        for (int i = 0; i < 60 * 3; ++i) {
            car->step(in, g, kDt);
            const auto& tel = car->telemetry();
            front_drive += std::fabs(tel.drive_torque[0]) + std::fabs(tel.drive_torque[1]);
            rear_drive += std::fabs(tel.drive_torque[2]) + std::fabs(tel.drive_torque[3]);
        }
        t[int(d)] = accelerate(s, 15).t100;
        std::printf("  %s: 0-100 %.2f s, front/rear drive torque %.0f / %.0f\n",
                    d == Drive::FR ? "FR" : d == Drive::FF ? "FF" : d == Drive::AWD ? "AWD" : "MR", t[int(d)],
                    front_drive, rear_drive);
        if (d == Drive::FF) CHECK(rear_drive == 0 && front_drive > 0);
        if (d == Drive::FR || d == Drive::MR) CHECK(front_drive == 0 && rear_drive > 0);
        if (d == Drive::AWD) CHECK(front_drive > 0 && rear_drive > 0);
    }
    CHECK(t[int(Drive::AWD)] > 0 && t[int(Drive::AWD)] < t[int(Drive::FR)]);
    CHECK(t[int(Drive::AWD)] < t[int(Drive::FF)]);
    CHECK(t[int(Drive::MR)] < t[int(Drive::FF)]);
}

TEST(sim_deterministic) {
    CarSpec s = CarSpec::test_car(Drive::FR);
    auto a = Vehicle::create(s), b = Vehicle::create(s);
    FlatGround g;
    a->reset({1, 0, 2}, {});
    b->reset({1, 0, 2}, {});
    for (int i = 0; i < 60 * 30; ++i) {
        ControlInputs in;
        in.throttle = float((i * 37) % 100) / 100.0f;
        in.brake = (i / 90) % 4 == 3 ? 0.6f : 0;
        in.steer = std::sin(i * 0.013f);
        in.shift_up = i % 200 == 0;
        a->step(in, g, kDt);
        b->step(in, g, kDt);
    }
    // Field-wise (memcmp would also compare padding bytes).
    auto same = [](const VehicleState& x, const VehicleState& y) {
        auto v = [](const Vec3& p, const Vec3& q) { return p.x == q.x && p.y == q.y && p.z == q.z; };
        bool ok = v(x.position, y.position) && v(x.velocity, y.velocity) && v(x.angular_velocity, y.angular_velocity) &&
                  x.orientation.w == y.orientation.w && x.orientation.x == y.orientation.x &&
                  x.orientation.y == y.orientation.y && x.orientation.z == y.orientation.z &&
                  x.engine_rpm == y.engine_rpm && x.gear == y.gear && x.throttle == y.throttle &&
                  x.speed_kmh == y.speed_kmh && x.rev_limiter == y.rev_limiter;
        for (size_t i = 0; i < 4; ++i) {
            const WheelState &p = x.wheels[i], &q = y.wheels[i];
            ok = ok && v(p.contact, q.contact) && p.steer_angle == q.steer_angle && p.spin_angle == q.spin_angle &&
                 p.angular_velocity == q.angular_velocity && p.suspension == q.suspension &&
                 p.slip_ratio == q.slip_ratio && p.slip_angle == q.slip_angle && p.load == q.load &&
                 p.grounded == q.grounded && p.surface == q.surface;
        }
        return ok;
    };
    CHECK(same(a->state(), b->state()));
    CHECK(a->state().speed_kmh > 1); // the run actually moved
}

TEST(sim_no_nans_long_abuse) {
    // Ten minutes of violent random inputs including handbrake, manual
    // shifting into reverse and slopes; state must stay finite and bounded.
    CarSpec s = CarSpec::test_car(Drive::FR);
    auto car = Vehicle::create(s);
    FlatGround g;
    car->reset({}, {});
    u32 rng = 12345;
    auto rnd = [&] {
        rng = rng * 1664525u + 1013904223u;
        return float(rng >> 8) / float(1u << 24);
    };
    ControlInputs in;
    in.automatic = false;
    bool ok = true;
    for (int i = 0; i < 60 * 600; ++i) {
        if (i % 30 == 0) {
            in.throttle = rnd() < 0.7f ? rnd() : 0;
            in.brake = rnd() < 0.2f ? rnd() : 0;
            in.steer = rnd() * 2 - 1;
            in.handbrake = rnd() < 0.05f;
        }
        in.shift_up = rnd() < 0.01f;
        in.shift_down = rnd() < 0.008f;
        car->step(in, g, kDt);
        ok = ok && finite_state(car->state()) && car->state().velocity.length() < 150 &&
             car->state().engine_rpm < 20000;
    }
    CHECK(ok);
}

TEST(sim_manual_reverse) {
    CarSpec s = CarSpec::test_car(Drive::FR);
    auto car = Vehicle::create(s);
    FlatGround g;
    car->reset({}, {});
    ControlInputs in;
    in.automatic = false;
    for (int i = 0; i < 12; ++i) car->step(in, g, kDt); // standing start: shift lockout 12 (spec 9.3)
    in.shift_down = true; // 1 -> N
    car->step(in, g, kDt);
    CHECK(car->state().gear == 0);
    in.shift_down = false;
    car->step(in, g, kDt); // 1-step shift lockout (spec 2.6)
    in.shift_down = true;
    car->step(in, g, kDt); // N -> R
    CHECK(car->state().gear == -1);
    in.shift_down = false;
    in.throttle = 0.5f;
    for (int i = 0; i < 120; ++i) car->step(in, g, kDt);
    CHECK(forward_speed(*car) < -2.0f);
}

namespace {

// A vertical wall across the road at z = wall_z (the car drives towards -Z),
// plus a configurable surface grip.
struct WallWorld : WorldQuery {
    float wall_z = -40;
    float mu = 1;
    bool wall_contact(const Vec3& c, float r, Vec3& n, float& pen) const override {
        float d = c.z - wall_z;
        if (d >= r) return false;
        n = {0, 0, 1};
        pen = r - d;
        return true;
    }
    Grip surface_grip(u32) const override { return {mu, 0}; }
};

struct Slope : TrackQuery {
    float grade = 0.15f; // rise per metre along -Z
    SurfaceHit probe(float, float z, float y_max) const override {
        SurfaceHit h;
        h.height = -z * grade;
        h.hit = y_max >= h.height;
        h.normal = Vec3{0, 1, grade} * (1.0f / std::sqrt(1 + grade * grade));
        return h;
    }
};

} // namespace

TEST(sim_wall_stops_car) {
    auto car = Vehicle::create(CarSpec::test_car());
    FlatGround g;
    WallWorld w;
    car->reset({}, {});
    ControlInputs in;
    in.throttle = 1;
    float min_z = 0;
    for (int i = 0; i < 60 * 8; ++i) {
        car->step(in, g, kDt, &w);
        min_z = std::min(min_z, car->state().position.z);
    }
    std::printf("  wall: closest car origin z %.2f (wall at %.0f), final speed %.1f m/s\n", min_z, w.wall_z,
                forward_speed(*car));
    CHECK(finite_state(car->state()));
    CHECK(min_z > w.wall_z + 1.0f); // nose (half length ~2.3 m) never far through
    CHECK(std::fabs(forward_speed(*car)) < 3.0f);
}

TEST(sim_wall_swept_octagon) {
    // Same wall as segments (spec 5.3 swept octagon): the car must stop at it
    // even at speed (no tunnelling), at an angle it glances off.
    struct SegWorld : WorldQuery {
        bool wall_contact(const Vec3&, float, Vec3&, float&) const override { return false; }
        bool wall_segments(const Vec3&, float, std::vector<collide::Segment>& out) const override {
            collide::Segment sg;
            sg.a = {100, -40}; // track on the left of a->b: the +z side, where the car starts
            sg.b = {-100, -40};
            out.push_back(sg);
            return true;
        }
    };
    for (float yaw : {0.0f, 0.5f}) {
        auto car = Vehicle::create(CarSpec::test_car());
        FlatGround g;
        SegWorld w;
        car->reset({0, 0, 20}, {std::cos(yaw / 2), 0, std::sin(yaw / 2), 0});
        ControlInputs in;
        in.throttle = 1;
        float min_nose = 1e9f, impulse = 0;
        for (int i = 0; i < 60 * 8; ++i) {
            car->step(in, g, kDt, &w);
            const auto& st = car->state();
            Vec3 fwd = rotate_fwd(st.orientation);
            min_nose = std::min(min_nose, (st.position + fwd * 2.3f).z);
            impulse += car->telemetry().wall_impulse;
        }
        std::printf("  swept wall (yaw %.1f): nose min z %.2f, total impulse %.0f N*s, final %.1f km/h\n", yaw, min_nose,
                    impulse, car->state().speed_kmh);
        CHECK(finite_state(car->state()));
        CHECK(min_nose > -40.6f);
        CHECK(impulse > 1000);
    }
}

TEST(sim_direct_steer_angle) {
    // ControlInputs::steer_angle (CPU cars) sets the road-wheel angle directly.
    auto car = Vehicle::create(CarSpec::test_car());
    FlatGround g;
    car->reset({}, {});
    ControlInputs in;
    in.steer_angle = 0.2f;
    car->step(in, g, kDt);
    CHECK(std::fabs(car->telemetry().steer_angle - 0.2f) < 1e-6f);
    in.steer_angle = 0.75f; // beyond this car's 0.61 rad lock: CPU angles bypass it (ai.md)
    car->step(in, g, kDt);
    CHECK(std::fabs(car->telemetry().steer_angle - 0.75f) < 1e-6f);
    in.steer_angle = std::numeric_limits<float>::quiet_NaN(); // back to the pad path
    in.steer = 0;
    for (int i = 0; i < 60; ++i) car->step(in, g, kDt);
    CHECK(std::fabs(car->telemetry().steer_angle) < 0.01f);
}

TEST(sim_spawn_hold) {
    // spec 4.3: 120 ticks held 0.3 m up (last 30 at ground), flat, gear 1, clutch open.
    data::CarDatabase* db = database();
    if (!db) SKIP("needs GT3R_DISC (spec suspension block)");
    auto car = Vehicle::create(*db, "r34_gtr_v_99");
    FlatGround g;
    car->spawn({0, 0, 0}, {std::cos(0.25f), 0, std::sin(0.25f), 0}); // yaw 0.5 rad
    ControlInputs in;
    in.throttle = 1;
    for (int i = 0; i < 60; ++i) car->step(in, g, kDt);
    std::printf("  spawn: after 1 s y %.3f m, %.2f km/h, gear %d, held %d\n", car->state().position.y,
                car->state().speed_kmh, car->state().gear, int(car->spawn_hold_active()));
    CHECK(car->spawn_hold_active() && std::fabs(car->state().position.y - 0.3f) < 0.02f);
    CHECK(car->state().speed_kmh < 1 && car->state().gear == 1);
    for (int i = 0; i < 180; ++i) car->step(in, g, kDt);
    CHECK(!car->spawn_hold_active() && car->state().speed_kmh > 20 && finite_state(car->state()));
}

TEST(sim_spawn_moving) {
    // Rolling start at 80 km/h: moving at once, gear above 1st, clutch locked (engine
    // at the road-speed rpm), no spawn hold; holds its speed under part throttle.
    auto car = Vehicle::create(CarSpec::test_car(Drive::FR));
    FlatGround g;
    const float v0 = 80 / 3.6f;
    car->spawn_moving({0, 0, 0}, {1, 0, 0, 0}, v0);
    CHECK(!car->spawn_hold_active());
    CHECK(std::fabs(car->state().speed_kmh - 80) < 0.5f && car->state().gear >= 2);
    ControlInputs in;
    in.throttle = 0.4f;
    in.automatic = true;
    for (int i = 0; i < 30; ++i) car->step(in, g, kDt);
    const VehicleState& st = car->state();
    std::printf("  spawn_moving: after 0.5 s %.1f km/h, gear %d, %.0f rpm, y %.3f\n", st.speed_kmh, st.gear, st.engine_rpm,
                st.position.y);
    CHECK(st.speed_kmh > 70 && st.speed_kmh < 95 && st.gear >= 2 && finite_state(st));
    CHECK(st.engine_rpm > CarSpec::test_car(Drive::FR).engine.idle_rpm + 500);
    CHECK(std::fabs(st.position.z + v0 * 0.5f) < 2.0f); // moved ~11 m along -Z
}

TEST(sim_swept_car_contact_no_tunnelling) {
    // A body crossing 3 m in one tick through a 1.8 m wide one: the end poses do
    // not overlap, the swept test (spec 5.3) still finds the hit.
    collide::Body a, b;
    a.local = b.local = collide::octagon(2.3, 2.3, 1.8);
    a.radius = b.radius = collide::bounding_radius(a.local);
    a.inv_mass = b.inv_mass = 1 / 1500.0;
    a.inv_inertia = b.inv_inertia = 1 / 95000.0;
    b.pos = {0, 0};
    collide::V2 a_start{-3.0, 0};
    a.heading = b.heading = 1.5707963; // both noses towards -x: a slides through b side-on
    a.vel = {180, 0};
    a.pos = a_start + a.vel * (1 / 60.0);
    collide::Body ea = a, eb = b;
    double end_overlap_check = collide::collide_pair(ea, eb);
    collide::Body a2 = a, b2 = b;
    a2.pos = a_start + a.vel * (1 / 60.0);
    double j = collide::collide_pair_swept(a2, a_start, a.heading, b2, b.pos, b.heading, 1 / 60.0);
    std::printf("  swept car-car: discrete end-pose contact %.0f, swept impulse %.0f N*s, a.vx %.1f -> %.1f\n",
                end_overlap_check, j, a.vel.x, a2.vel.x);
    CHECK(j > 1000);
    CHECK(a2.vel.x < a.vel.x && b2.vel.x > 0);
    CHECK(a2.pos.x < b2.pos.x); // a stays on its side of b
}

TEST(sim_wear_models) {
    // spec 8.1/8.4: the one shipped curve and its grip factor.
    model::WearParams p = model::shipped_wear_params();
    CHECK(p.on() && p.fresh == -20000 && p.knee1 == 40000 && p.knee2 == 150000 && p.max == 210000);
    CHECK(std::fabs(p.loss_fresh - 0.06f) < 1e-6f && std::fabs(p.loss_max - 0.30f) < 1e-6f && std::fabs(p.worn - 0.91f) < 1e-6f);
    const u8 bad[7] = {21, 30, 9, 2, 6, 25, 5}; // knee 2 above max: ordering check fails
    CHECK(!model::wear_params(bad, true).on() && !model::wear_params(bad, false).on());
    auto grip = [&](float w, float floor = 0.5f) { return model::wear_grip(p, w, floor); };
    CHECK(std::fabs(grip(-20000) - 0.94f) < 1e-6f && std::fabs(grip(0) - 1) < 1e-6f && std::fabs(grip(40000) - 1) < 1e-6f);
    CHECK(std::fabs(grip(150000) - 0.95f) < 1e-6f && std::fabs(grip(210000) - 0.70f) < 1e-6f);
    CHECK(std::fabs(grip(-30000) - 1) < 1e-6f && std::fabs(grip(95000) - 0.975f) < 1e-5f);
    float w = 300000;
    int lv = 0;
    CHECK(std::fabs(model::wear_grip(p, w, 0.5f, &lv) - 0.70f) < 1e-6f && w == 210000 && lv == 127);
    CHECK(grip(210000, 1.0f) == 1.0f); // control compound floor
    // spec 8.3: tarmac smoke ~1 at 8.2 m/s lateral or slip +0.385 / -0.272; none when gripping.
    CHECK(std::fabs(model::slip_intensity(0, 8.2f, 0, 20, 4000) - 1) < 0.02f);
    CHECK(std::fabs(model::slip_intensity(0, 0, 0.385f, 20, 4000) - 1) < 0.02f);
    CHECK(std::fabs(model::slip_intensity(0, 0, -0.272f, 20, 4000) - 1) < 0.02f);
    CHECK(model::slip_intensity(0, 0.5f, 0.02f, 20, 4000) == 0 && model::slip_intensity(3, 0.5f, 0.02f, 20, 4000) == 0);
    CHECK(model::slip_intensity(1, 0, 0, 20, 4000) > 0.4f); // kerb: rim speed and load
    // spec 8.2 example: R34 normal tyres, 6000 N in a full slide ~3100 units/s.
    float rate = model::wear_increment(1.5f, p.slip_cap, 0.40f, 6000, 0.245f, 1.0f);
    std::printf("  wear: 6000 N full slide %.0f units/s\n", rate);
    CHECK(rate > 3000 && rate < 3200);
    CHECK(model::wear_increment(1, 1, 0.4f, 1000, 0.245f, 1) == 0); // light load: below the 1960 offset
    // spec 8.5: brake heat stays in [0, 1].
    float y = 0;
    for (int i = 0; i < 600; ++i) y = model::brake_heat_step(y, 3000, 0.32f, 1, 40, float(kDt));
    CHECK(y > 0.5f && y <= 1);
    for (int i = 0; i < 3000; ++i) y = model::brake_heat_step(y, 3000, 0.32f, 0, 20, float(kDt));
    CHECK(y < 0.01f);
}

namespace {
struct PitGround : FlatGround {
    SurfaceHit probe(float x, float z, float y_max) const override {
        SurfaceHit h = FlatGround::probe(x, z, y_max);
        h.surface = 0x100; // tarmac, pit-lane flag
        return h;
    }
};
} // namespace

TEST(sim_tire_wear_and_pit_stop) {
    // Wheelspin launches wear the driven tyres (spec 8.2); the pit surface freezes
    // wear; a pit stop holds the car 300 steps and fits fresh tyres (spec 8.6).
    CarSpec spec = CarSpec::test_car(Drive::FR);
    auto car = Vehicle::create(spec);
    FlatGround g;
    car->reset({0, 0, 0}, {1, 0, 0, 0});
    car->set_tire_wear(model::shipped_wear_params());
    CHECK(car->telemetry().tire_wear[2] == 0 || car->telemetry().tire_wear[2] == -20000);
    ControlInputs in;
    in.throttle = 1;
    in.automatic = true;
    for (int i = 0; i < 180; ++i) car->step(in, g, kDt);
    const VehicleTelemetry& t = car->telemetry();
    std::printf("  wear after 3 s launch: front %.0f %.0f rear %.0f %.0f, grip %.3f, smoke rear %.2f, %.0f km/h\n", t.tire_wear[0],
                t.tire_wear[1], t.tire_wear[2], t.tire_wear[3], t.wear_grip[2], t.smoke[2], car->state().speed_kmh);
    CHECK(t.tire_wear[2] > -20000 && t.tire_wear[2] >= t.tire_wear[0] && !t.tires_worn);
    // Pit lane: wear frozen.
    PitGround pg;
    float before = t.tire_wear[2];
    car->spawn_moving({0, 0, 0}, {1, 0, 0, 0}, 3);
    for (int i = 0; i < 60; ++i) car->step(in, pg, kDt);
    CHECK(car->telemetry().on_pit_surface && car->telemetry().tire_wear[2] == before);
    // The stop: velocities zeroed, held for 300 steps, fresh set at counter 60.
    car->begin_pit_stop({1, 0, 0, 0});
    CHECK(car->spawn_hold_active() && car->state().speed_kmh < 0.1f);
    ControlInputs idle;
    for (int i = 0; i < 239; ++i) car->step(idle, pg, kDt); // counter 300 -> 62
    CHECK(car->spawn_hold_active() && car->telemetry().tire_wear[2] == before);
    for (int i = 0; i < 3; ++i) car->step(idle, pg, kDt); // through counter 60
    CHECK(car->telemetry().tire_wear[2] == -20000);
    for (int i = 0; i < 60; ++i) car->step(idle, pg, kDt);
    CHECK(!car->spawn_hold_active() && finite_state(car->state()));
}

TEST(sim_stuck_steer_nudge) {
    // [spec §5.3] a nearly stopped body (|vx|, |vy| < 1 m/s) whose last
    // hit was early in the step (t < 0.1) gets yaw rate += -steer x 20 x dt.
    auto run = [](float steer_a) {
        collide::Body a, b;
        a.local = b.local = collide::octagon(2.0, 2.0, 1.8);
        a.radius = b.radius = collide::bounding_radius(a.local);
        a.inv_mass = b.inv_mass = 1 / 1200.0;
        a.inv_inertia = b.inv_inertia = 1 / 20000.0;
        collide::V2 a0{0.2, 4.001}, b0{0, 0}; // a's nose 1 mm behind b's tail, slightly offset
        a.vel = {0, -0.9};                   // creeping forward (-z) into b
        a.pos = a0 + a.vel * (1 / 60.0);
        b.pos = b0;
        float steer[2] = {steer_a, 0};
        double j = collide::collide_pair_swept(a, a0, 0, b, b0, 0, 1 / 60.0, 2, nullptr, nullptr, steer);
        return std::pair<double, double>(j, a.yaw_rate);
    };
    auto [j0, w0] = run(0);
    auto [j1, w1] = run(1);   // full right
    auto [j2, w2] = run(-0.5f); // half left
    std::printf("  stuck nudge: j %.0f, yaw %.4f (no steer) %.4f (right) %.4f (half left)\n", j0, w0, w1, w2);
    CHECK(j0 > 0 && j1 == j0);
    CHECK(std::fabs((w1 - w0) - (-20.0 / 60.0)) < 1e-9);
    CHECK(std::fabs((w2 - w0) - (10.0 / 60.0)) < 1e-9);
}

TEST(sim_world_car_contact) {
    // Two cars driving head-on through sim::World collide and separate.
    FlatGround g;
    World world(g);
    int a = world.add(Vehicle::create(CarSpec::test_car()));
    int b = world.add(Vehicle::create(CarSpec::test_car()));
    world.vehicle(a).reset({0, 0, 0}, {});
    world.vehicle(b).reset({0.5f, 0, -40}, {0, 0, 1, 0}); // facing +Z
    ControlInputs in;
    in.throttle = 0.7f;
    world.set_inputs(a, in);
    world.set_inputs(b, in);
    int contacts = 0;
    float min_gap = 1e9f;
    for (int i = 0; i < 60 * 5; ++i) {
        world.step(kDt);
        contacts += int(world.contacts().size());
        Vec3 d = world.vehicle(b).state().position - world.vehicle(a).state().position;
        min_gap = std::min(min_gap, std::sqrt(d.x * d.x + d.z * d.z));
    }
    std::printf("  car-car: %d contact ticks, min centre distance %.2f m\n", contacts, min_gap);
    CHECK(contacts > 0);
    CHECK(min_gap > 3.0f); // octagons (length ~4.6 m) never pass through each other
    CHECK(finite_state(world.vehicle(a).state()) && finite_state(world.vehicle(b).state()));
}

TEST(sim_world_kinematic_remote_car) {
    // Online play: a remote car (kinematic, posed from the network) rolls into a parked
    // local car. The local car is pushed; the remote car keeps exactly the pose it was given.
    FlatGround g;
    World world(g);
    int a = world.add(Vehicle::create(CarSpec::test_car()));
    int b = world.add(Vehicle::create(CarSpec::test_car()));
    world.vehicle(a).reset({0, 0, 0}, {});
    world.vehicle(b).reset({0.3f, 0, -12}, {0, 0, 1, 0}); // facing +Z, towards a's nose
    world.set_kinematic(b, true);
    collide::Body body = world.vehicle(b).collision_body();
    VehicleState st = world.vehicle(b).state();
    const float v = 8; // m/s towards +Z
    body.vel = {0, v};
    st.velocity = {0, 0, v};
    int contacts = 0;
    bool pose_kept = true;
    for (int i = 0; i < 120; ++i) {
        body.pos.z += v * kDt; // end of the coming tick
        st.position.z += v * float(kDt);
        world.vehicle(b).set_remote_pose(st, body);
        world.step(kDt);
        contacts += int(world.contacts().size());
        collide::Body now = world.vehicle(b).collision_body();
        pose_kept = pose_kept && now.pos.z == body.pos.z && now.vel.z == body.vel.z &&
                    world.vehicle(b).state().position.z == st.position.z;
    }
    float pushed = world.vehicle(a).state().position.z;
    std::printf("  kinematic remote car: %d contact ticks, local car pushed to z %.2f m\n", contacts, pushed);
    CHECK(contacts > 0);
    CHECK(pose_kept);
    CHECK(pushed > 0.5f && world.vehicle(a).state().velocity.z > 0);
    CHECK(finite_state(world.vehicle(a).state()));
}

TEST(sim_surface_grip_scales_braking) {
    // Surface index from the raw material byte (spec 3.6): 0 tarmac, 2 grass.
    struct Surface : TrackQuery {
        u32 raw = 0;
        SurfaceHit probe(float, float, float y_max) const override {
            SurfaceHit h;
            h.hit = y_max >= 0;
            h.surface = raw;
            return h;
        }
    };
    auto stop_distance = [](u32 raw) {
        auto car = Vehicle::create(CarSpec::test_car());
        Surface tarmac, g;
        g.raw = raw;
        car->reset({}, {});
        ControlInputs in;
        in.throttle = 1;
        for (int i = 0; i < 60 * 30 && forward_speed(*car) < 80 / 3.6f; ++i) car->step(in, tarmac, kDt);
        in.throttle = 0;
        in.brake = 1;
        Vec3 p0 = car->state().position;
        for (int i = 0; i < 900 && forward_speed(*car) > 0.1f; ++i) car->step(in, g, kDt);
        return (car->state().position - p0).length();
    };
    float dry = stop_distance(0), grass = stop_distance(2), wet = stop_distance(0x200);
    std::printf("  80-0 km/h: tarmac %.1f m, wet tarmac %.1f m, grass %.1f m\n", dry, wet, grass);
    CHECK(grass > dry * 1.3f && wet > dry * 1.15f);
}

TEST(sim_slope_hold_and_roll) {
    Slope s;
    auto car = Vehicle::create(CarSpec::test_car(Drive::FR));
    car->reset({0, 0, 0}, {});
    ControlInputs in;
    in.brake = 1;
    for (int i = 0; i < 180; ++i) car->step(in, s, kDt);
    CHECK(car->state().velocity.length() < 0.1f); // brakes hold on a 15 % grade
    in.brake = 0;
    for (int i = 0; i < 180; ++i) car->step(in, s, kDt);
    CHECK(forward_speed(*car) < -1.0f); // rolls back down (+Z) with brakes off
    CHECK(finite_state(car->state()));
}

// --- real cars -------------------------------------------------------------------

TEST(sim_database_r34_gtr) {
    data::CarDatabase* db = database();
    if (!db) SKIP("needs GT3R_DISC");
    std::string err;
    auto spec = build_car_spec(*db, "r34_gtr_v_99", &err);
    CHECK(spec.has_value());
    if (!spec) {
        std::printf("  %s\n", err.c_str());
        return;
    }
    std::printf("  %s: %.0f kg, fw %.2f, wb %.3f, %s, peak %.0f Nm, %.0f kW @ %.0f rpm, limit %.0f, %zu gears "
                "final %.3f, tyres r %.3f/%.3f %s, front share %.2f\n",
                spec->name.c_str(), spec->mass, spec->front_weight, spec->wheelbase,
                spec->drivetrain.drive == Drive::AWD ? "4WD" : "?", spec->peak_torque(), spec->peak_power_kw(),
                spec->peak_power_rpm(), spec->engine.rev_limit_rpm, spec->gearbox.ratios.size(),
                spec->gearbox.final_drive, spec->axle[0].tire_radius, spec->axle[1].tire_radius,
                spec->tire[0].compound.c_str(), spec->drivetrain.front_share);
    std::printf("  idle %.0f, launch %.0f, UP %.0f, J_e %.3f, k_f %.4f, brakes %.0f/%.0f/%.0f N*m, C_d %.3f, yaw I %.0f, "
                "lock %.1f deg, pad curve %zu pts\n",
                spec->engine.idle_rpm, spec->engine.launch_rpm, spec->engine.upshift_rpm, spec->engine.inertia,
                spec->engine.friction_coeff, spec->axle[0].max_brake_torque, spec->axle[1].max_brake_torque,
                spec->aids.handbrake_torque, spec->aero.c_drag, spec->yaw_inertia, spec->steering.max_angle * 57.2958f,
                spec->steering.pad_curve.x.size());
    {
        const TireSpec& t = spec->tire[0];
        float amax = 0, aat = 0;
        for (size_t i = 0; i < t.slip_a.x.size(); ++i)
            if (t.slip_a.y[i] > amax) amax = t.slip_a.y[i], aat = t.slip_a.x[i];
        std::printf("  tyre %s: mu %.2f, mu[grass] %.2f, A peak %.2f @ %.1f deg, opt drive %.3f brake %.3f, C pts %zu, "
                    "L(4 kN) %.2f\n",
                    t.compound.c_str(), t.mu, t.surface_mu[2], amax, aat * 57.2958f, t.opt_drive, t.opt_brake,
                    t.long_c.x.size(), model::lookup(t.load, 4000 / t.width));
        CHECK(std::fabs(t.mu - 1.45f) < 1e-4f); // spec 6.5: "normal" compound
        CHECK(std::fabs(t.wear_rate - 0.40f) < 1e-6f); // spec 8.2: normal compound wear rate
        std::printf("  R34 driveline inertia words: spin %.2f / %.2f, shaft %.2f / %.2f, engine %.3f, final %.3f\n",
                    spec->axle[0].spin_inertia, spec->axle[1].spin_inertia, spec->axle[0].shaft_inertia, spec->axle[1].shaft_inertia,
                    spec->engine.inertia, spec->gearbox.final_drive);
        CHECK(t.long_c.x.size() == 11 && t.slip_a.x.size() == 8 && t.opt_drive < 0 && t.opt_brake > 0);
    }
    CHECK(spec->drivetrain.drive == Drive::AWD);
    CHECK(std::fabs(spec->axle[0].max_brake_torque - 3332) < 1 && std::fabs(spec->axle[1].max_brake_torque - 2254) < 1);
    CHECK(std::fabs(spec->yaw_inertia - 9500) < 1);
    // spec 4.7 R34: drag 0.511, downforce 0.348 / 0.564
    CHECK(std::fabs(spec->aero.c_drag - 0.511f) < 0.002f);
    CHECK(std::fabs(spec->aero.c_front - 0.348f) < 0.002f && std::fabs(spec->aero.c_rear - 0.564f) < 0.002f);
    // spec 4.2 R34 suspension blocks
    const SuspensionSpec &sf = spec->axle[0].susp, &sr = spec->axle[1].susp;
    std::printf("  susp k %.0f/%.0f, arb %.0f/%.0f, k2 %.3g, unsprung %.1f kg, ride %.3f m, bump damper g %.0f t %.4f\n",
                sf.k, sr.k, sf.arb_gain, sr.arb_gain, sf.k2, sf.unsprung, sf.ride_height, sf.bump.g, sf.bump.t);
    CHECK(sf.valid && sr.valid);
    CHECK(std::fabs(sf.k - 37240) < 1 && std::fabs(sr.k - 46550) < 1);
    CHECK(std::fabs(sf.arb_gain - 4900) < 1 && std::fabs(sr.arb_gain - 3430) < 1);
    CHECK(std::fabs(sf.k2 - 1.568e6f) < 10 && std::fabs(sf.unsprung - 10) < 1e-3f);
    CHECK(spec->mass > 1570 && spec->mass < 1770); // 1560 kg + 120 kg [spec 6.4]
    CHECK(spec->gearbox.ratios.size() == 6);
    {
        // Explicit load transfer (spec 4.2): steady right-hand corner loads the left wheels.
        auto car = Vehicle::create(*spec);
        FlatGround g;
        car->reset({}, {});
        ControlInputs in;
        in.throttle = 0.4f;
        in.steer = 0.5f;
        for (int i = 0; i < 60 * 10; ++i) car->step(in, g, kDt);
        const auto& w = car->state().wheels;
        float left = w[0].load + w[2].load, right = w[1].load + w[3].load;
        std::printf("  R34 corner: %.0f km/h, lateral %.2f g, left/right load %.0f / %.0f N, TCS %d\n",
                    car->state().speed_kmh, car->telemetry().accel_local.x / 9.8f, left, right,
                    int(car->telemetry().tcs_active));
        CHECK(car->telemetry().accel_local.x > 3 && left > right * 1.05f); // GT3 X parameter is small
    }
    {
        auto car = Vehicle::create(*spec);
        FlatGround g;
        car->reset({}, {});
        ControlInputs in;
        in.throttle = 1;
        float max_boost = 0;
        for (int i = 0; i < 600; ++i) {
            car->step(in, g, kDt);
            max_boost = std::max(max_boost, car->telemetry().boost);
        }
        std::printf("  R34 boost: max %.2f (cap %.2f)\n", max_boost, car->setup().max_boost);
        CHECK(car->setup().has_turbo && max_boost > 0.1f && max_boost <= car->setup().max_boost + 1e-5f);
    }
    AccelResult r = accelerate(*spec, 70);
    std::printf("  R34: 0-100 %.2f s, 400 m %.2f s, top %.0f km/h\n", r.t100, r.t400, r.top_kmh);
    CHECK(r.finite);
    CHECK(r.t100 > 4.0 && r.t100 < 6.5);
    CHECK(r.top_kmh > 230 && r.top_kmh < 330);
}

TEST(sim_car_settings) {
    // User settings (spec 6.6) feed the spec with the spec's conversions; the
    // stock settings reproduce the default build exactly.
    data::CarDatabase* db = database();
    if (!db) SKIP("needs GT3R_DISC");
    const data::Car* car = db->car("r34_gtr_v_99");
    CHECK(car != nullptr);
    if (!car) return;
    CarSettings st = stock_settings(*db, car->label, car->parts);
    CHECK(st.spring[0] == 40 && st.spring[1] == 50 && st.motion_ratio[0] == 95); // spec 6.6 R34 values
    CHECK(st.ride_height[0] == 130 && st.downforce_front == 29 && st.downforce_rear == 47 && st.final_drive == 3545);
    auto a = build_car_spec(*db, car->label, car->parts);
    auto b = build_car_spec(*db, car->label, car->parts, st);
    CHECK(a && b);
    if (!a || !b) return;
    CHECK(a->axle[0].susp.k == b->axle[0].susp.k && a->gearbox.ratios == b->gearbox.ratios &&
          a->aero.c_drag == b->aero.c_drag && a->aids.tcs_level == b->aids.tcs_level);

    CarSettings t = st;
    t.spring = {60, 70};
    t.motion_ratio = {100, 90};
    t.ride_height = {110, 120};
    t.downforce_front = 50;
    t.downforce_rear = 0;
    t.final_drive = 4000;
    t.gears[1] = 3000;
    t.front_torque = 40;
    t.lsd = {10, 20, 30, 40, 50, 60};
    t.arb = st.arb;
    auto c = build_car_spec(*db, car->label, car->parts, t);
    CHECK(c.has_value());
    if (!c) return;
    CHECK(std::fabs(c->axle[0].susp.k - 60 * 980.0f) < 1 && std::fabs(c->axle[1].susp.k - 70 * 980 * 0.9f) < 1);
    // [spec §4.2] both suspension blocks read the front ride height (the rear ride-height byte is ignored)
    CHECK(std::fabs(c->axle[0].susp.ride_height - 0.110f) < 1e-6f && std::fabs(c->axle[1].susp.ride_height - 0.110f) < 1e-6f);
    float af = 0.5f * 1.2f * 2.0f; // R34 frontal area 2.0 m^2
    CHECK(std::fabs(c->aero.c_front - af * 0.5f) < 1e-4f && c->aero.c_rear == 0);
    CHECK(std::fabs(c->aero.c_drag - (a->aero.c_drag - 0.1f * (a->aero.c_front + a->aero.c_rear) + 0.1f * af * 0.5f)) < 1e-4f);
    CHECK(std::fabs(c->gearbox.final_drive - 4.0f) < 1e-6f && std::fabs(c->gearbox.ratios[0] - 3.0f) < 1e-6f);
    for (int ax = 0; ax < 2; ++ax) { // axles whose LSD part defines a type
        if (c->drivetrain.lsd_type[size_t(ax)] < 0) continue;
        CHECK(c->drivetrain.lsd_initial[size_t(ax)] == 10 + 10 * ax && c->drivetrain.lsd_accel[size_t(ax)] == 30 + 10 * ax &&
              c->drivetrain.lsd_decel[size_t(ax)] == 50 + 10 * ax);
    }
    CHECK(c->drivetrain.lsd_type[1] >= 0);
    // A stiffer spring setting stiffens the static position (x_s ~ -1/k) and still drives.
    auto car2 = Vehicle::create(*c);
    FlatGround g;
    car2->reset({}, {});
    ControlInputs in;
    in.throttle = 1;
    for (int i = 0; i < 300; ++i) car2->step(in, g, kDt);
    CHECK(finite_state(car2->state()) && car2->state().speed_kmh > 60);
    // TCS index -> level; index 1 vs the stock index changes it when the TCS part has steps.
    CarSettings u = st;
    u.tcs = 1;
    auto d = build_car_spec(*db, car->label, car->parts, u);
    CHECK(d.has_value());
    std::printf("  settings: stock TCS index %d level %.2f -> index 1 level %.2f; ABS gain %.2f\n", st.tcs,
                a->aids.tcs_level, d ? d->aids.tcs_level : -1.0f, a->aids.abs_gain);
}

TEST(sim_computed_gearbox_slider) {
    // Computed gearboxes regenerate from the slider (spec 6.7): a higher design
    // speed gives a taller (smaller) top gear.
    data::CarDatabase* db = database();
    if (!db) SKIP("needs GT3R_DISC");
    const data::Car* car = db->car("zz2_00");
    if (!car) SKIP("zz2_00 not in this database");
    CarSettings st = stock_settings(*db, car->label, car->parts);
    auto a = build_car_spec(*db, car->label, car->parts, st);
    st.gear_slider += 3;
    CHECK(apply_gear_slider(*db, car->parts, st));
    auto b = build_car_spec(*db, car->label, car->parts, st);
    CHECK(a && b);
    if (!a || !b) return;
    std::printf("  zz2 slider: top gear %.3f -> %.3f, first %.3f -> %.3f\n", a->gearbox.ratios.back(), b->gearbox.ratios.back(),
                a->gearbox.ratios[0], b->gearbox.ratios[0]);
    CHECK(std::fabs(a->gearbox.ratios[0] - 3.194f) < 1e-4f); // spec 6.7 check value
    CHECK(b->gearbox.ratios.back() < a->gearbox.ratios.back());
}

TEST(sim_showroom_top_speed) {
    // The arcade car pages' speed gauge (menus spec §9): km/h as read off the
    // original's gauges (reference captures, about +-2 km/h).
    data::CarDatabase* db = database();
    if (!db) SKIP("needs GT3R_DISC");
    const struct {
        const char* car;
        float kmh;
    } kRef[] = {{"nsx_s_zero_97", 297}, {"rx7_rz_00", 285}, {"r32_gtr_v2_00", 277}, {"r34_gtr_v2_00", 270},
                {"lancer_evo7_gsr_01", 275}, {"xsara_wrc_99", 250}};
    for (const auto& r : kRef) {
        auto spec = build_car_spec(*db, r.car);
        CHECK(spec.has_value());
        if (!spec) continue;
        float kmh = std::floor(showroom_top_speed(*spec) * 3.6f);
        std::printf("  %s: %.0f km/h (reference %.0f)\n", r.car, kmh, r.kmh);
        CHECK(std::fabs(kmh - r.kmh) <= 4);
    }
}

TEST(sim_database_all_cars_build_and_run) {
    data::CarDatabase* db = database();
    if (!db) SKIP("needs GT3R_DISC");
    int built = 0, total = 0, sane = 0;
    for (const auto& c : db->cars()) {
        ++total;
        std::string err;
        auto spec = build_car_spec(*db, c.label, &err);
        if (!spec) {
            std::printf("  %s\n", err.c_str());
            continue;
        }
        ++built;
        AccelResult r = accelerate(*spec, 20);
        if (r.finite && r.top_kmh > 60) ++sane;
        else std::printf("  %s: top %.0f km/h in 20 s finite=%d\n", c.label.c_str(), r.top_kmh, r.finite);
    }
    std::printf("  cars: %d built / %d, %d drive sanely\n", built, total, sane);
    CHECK(built == total);
    CHECK(sane == built);
}


TEST(sim_drives_trial_mountain) {
    // Integration with the track runtime: a simple line-following driver laps
    // part of Trial Mountain with walls and surfaces from track::Track.
    data::CarDatabase* db = database();
    Disc* disc = test::test_disc();
    if (!db || !disc) SKIP("needs GT3R_DISC");
    const Volume::Entry* e = disc->volume().find("courses/US/trialmt");
    CHECK(e != nullptr);
    if (!e) return;
    Bytes data = disc->volume().read(*e);
    auto t = track::Track::from_course_file(data, "trialmt");
    TrackWorld world(*t);
    auto car = Vehicle::create(*db, "r34_gtr_v_99");
    CHECK(car != nullptr);
    if (!car) return;
    track::Pose start = t->grid_pose(0);
    car->reset(start.position, start.orientation);
    float s0 = t->locate(start.position).distance, hint = s0, travelled = 0, prev = s0;
    float min_y = 1e9f, max_y = -1e9f;
    int walls = 0, grounded_ticks = 0;
    const int ticks = 60 * 60;
    for (int i = 0; i < ticks; ++i) {
        const VehicleState& st = car->state();
        track::TrackPosition tp = t->locate(st.position, hint);
        hint = tp.distance;
        float ds = tp.distance - prev;
        if (ds < -t->length() * 0.5f) ds += t->length();
        if (ds > t->length() * 0.5f) ds -= t->length();
        travelled += ds;
        prev = tp.distance;
        // Aim at a centre-line point ahead; steer by the heading error.
        float look = 12 + st.speed_kmh * 0.25f;
        Vec3 target = t->point_at(tp.distance + look, 0);
        Vec3 to = target - st.position;
        Vec3 fw = rotate_fwd(st.orientation);
        // Signed heading error about +Y (positive = target to the left).
        float err = std::atan2(fw.z * to.x - fw.x * to.z, fw.x * to.x + fw.z * to.z);
        ControlInputs in;
        in.steer = std::clamp(-err * 2.5f, -1.0f, 1.0f); // steer > 0 = right
        float want = 110 - 60 * std::min(1.0f, std::fabs(err) * 3);
        in.throttle = st.speed_kmh < want ? 0.8f : 0.0f;
        in.brake = st.speed_kmh > want + 15 ? 0.6f : 0.0f;
        Vec3 n;
        float pen;
        for (const auto& w : st.wheels) grounded_ticks += w.grounded;
        walls += world.wall_contact(st.position + Vec3{0, 0.5f, 0}, 1.2f, n, pen);
        car->step(in, *t, kDt, &world);
        if (std::getenv("SIM_TRACE") && i % 30 == 0) std::printf("   t=%.1f s=%.1f v=%.1f err=%.2f steer=%.2f g=%d rpm=%.0f pos %.1f %.1f %.1f\n", i * kDt, tp.distance, st.speed_kmh, err, in.steer, st.gear, st.engine_rpm, st.position.x, st.position.y, st.position.z);
        min_y = std::min(min_y, car->state().position.y);
        max_y = std::max(max_y, car->state().position.y);
    }
    std::printf("  trialmt: %.0f m along the lap in 60 s, y %.1f..%.1f, wheels grounded %.0f%%, near-wall ticks %d\n",
                travelled, min_y, max_y, 100.0 * grounded_ticks / (4.0 * ticks), walls);
    CHECK(finite_state(car->state()));
    CHECK(travelled > 800);
    CHECK(grounded_ticks > 4 * ticks * 0.9);
}

namespace {

// Flying-start pose (spec §9.2): on AI line 6 at race progress -metres_before,
// heading along the line over the next 10 m (the grid slot without line 6).
track::Pose demo_flying_pose(const track::Track& t, float metres_before) {
    track::Pose g = t.grid_pose(0);
    static const int kLine = std::getenv("SIM_DEMO_LINE") ? std::atoi(std::getenv("SIM_DEMO_LINE")) : 6; // [spec §9.2] line 6
    std::vector<Vec3> line = t.racing_line(kLine, 1.0f);
    if (line.size() < 20) return g;
    float len = t.length(), target = std::fmod(len - std::max(0.0f, metres_before) + len, len);
    size_t best = 0;
    float bd = 1e30f;
    for (size_t i = 0; i < line.size(); ++i) {
        float dd = std::fabs(std::remainder(t.locate(line[i]).distance - target, len));
        if (dd < bd) bd = dd, best = i;
    }
    size_t ahead = best;
    float acc = 0;
    while (acc < 10.0f) {
        size_t nx = (ahead + 1) % line.size();
        acc += std::hypot(line[nx].x - line[ahead].x, line[nx].z - line[ahead].z);
        ahead = nx;
        if (ahead == best) break;
    }
    float dx = line[ahead].x - line[best].x, dz = line[ahead].z - line[best].z, yaw = std::atan2(-dx, -dz);
    track::Pose p;
    p.position = line[best];
    SurfaceHit h = t.probe(p.position.x, p.position.z, p.position.y + 5.0f);
    if (h.hit) p.position.y = h.height;
    p.orientation = {std::cos(yaw * 0.5f), 0, std::sin(yaw * 0.5f), 0};
    return p;
}

struct DemoResult {
    bool ran = false, completed = false;
    double first_off_s = -1; // first step any wheel touches grass/gravel/sand: the earliest visible divergence
    double finish_s = -1;    // licence run time when our run finishes the test (vs the recorded time)
    std::string fail;        // licence failure ("course_out", "wall", ...)
    double out_s = -1;     // time of the course-out (all four wheels on grass/gravel/sand), -1 = none
    float metres = 0;      // distance along the lap driven until then (or the end)
    double recorded_s = 0; // the demo's recorded run time
    std::string why;
};

// Replays a licence demo (disc demo/<code>) open-loop through sim::Vehicle on its course.
DemoResult run_license_demo(const std::string& code) {
    DemoResult r;
    data::CarDatabase* db = database();
    Disc* disc = test::test_disc();
    if (!db || !disc) return r;
    Vfs vfs(&disc->volume());
    const data::Event* ev = db->event(code);
    std::optional<Bytes> demo_bytes = vfs.read("demo/" + code);
    if (!ev || ev->entries.empty() || !demo_bytes) return r;
    std::optional<race::DemoReplay> demo = race::DemoReplay::parse(*demo_bytes);
    std::optional<CarSpec> spec = build_car_spec(*db, ev->entries[0]);
    std::optional<Bytes> course;
    for (const char* region : {"US", "JP", "UK", "EU"})
        if (!course) course = vfs.read(std::string("courses/") + region + "/" + ev->course);
    if (!demo || !spec || !course) return r;
    auto t = track::Track::from_course_file(*course, ev->course);
    TrackWorld world(*t);
    auto car = Vehicle::create(*spec);
    if (std::getenv("SIM_DEMO_TRACE") && code == std::getenv("SIM_DEMO_TRACE"))
        std::printf("   car %s drive %d awd %d diff %d/%d course %s\n", ev->entries[0].c_str(), int(spec->drivetrain.drive),
                    int(spec->drivetrain.awd_system), spec->drivetrain.lsd_type[0], spec->drivetrain.lsd_type[1], ev->course.c_str());
    race::EventStart es = race::event_start(*ev);
    int hold = 0;
    if (std::getenv("SIM_DEMO_TRACE") && code == std::getenv("SIM_DEMO_TRACE"))
        std::printf("   start type %d speed %.0f flying %.0f m, length %.1f, start_distance %.1f, grid0 s=%.1f\n", es.type, es.initial_kmh,
                    es.flying_m, t->length(), t->road().start_distance, t->locate(t->grid_pose(0).position).distance);
    // [spec draft 23b, §9.4] the flying-start run-up is the CPU AI driver on line 6 (kind 1)
    auto ai_course = std::make_shared<ai::AiCourse>(*t);
    // [ai.md §9.1] the player's run-up uses the built-in set by course category (1 -> 1, dirt 2 -> 2,
    // else 0), without the CPU event overrides
    int course_category = 0;
    for (const data::Course& c : db->courses())
        if (c.label == ev->course) course_category = c.surface;
    ai::AiDriver run_up(ai_course, ai::DriverParams::builtin(course_category == 1 || course_category == 2 ? course_category : 0));
    if (es.flying()) {
        track::Pose p = demo_flying_pose(*t, es.flying_m);
        car->spawn_moving(p.position, p.orientation, es.initial_kmh / 3.6f);
        run_up.reset(car->state());
    } else {
        track::Pose p = t->grid_pose(0);
        car->reset(race::grid_back(p.position, p.orientation, *spec), p.orientation);
        if (es.type == 0 || es.type == 1) hold = int(std::lround(es.countdown * 60));
    }
    r.ran = true;
    r.recorded_s = demo->time_ms * 0.001;
    // The licence judge as the race scene runs it (stop box / section goal / laps; failures).
    race::LicenseTimes lt = race::license_times(*ev);
    race::LicenseRun judge;
    race::LicenseRun::Rules rules;
    rules.kind = lt.kind;
    rules.laps = std::max<int>(1, ev->laps);
    rules.goal_m = t->road().start_distance;
    judge.begin(rules);
    bool started = false;
    double start_t = 0;
    int laps = 0;
    float progress = 0;
    float hint = t->locate(car->state().position).distance, prev = hint;
    size_t n = demo->size() + 600; // the recording, then zero frames (10 s) to let the run end
    for (size_t i = 0; i < n; ++i) {
        // After the last frame the original feeds an all-zero frame (digital, steer 0) [spec §4.4].
        ControlInputs in = i < demo->size() ? demo->inputs(i) : ControlInputs{};
        in.automatic = demo->automatic;
        in.steer = -in.steer; // the disc's steering is positive to the left
        in.controller_kind = i < demo->size() ? demo->frames[i].kind : 3;
        if (es.flying() && !started) {
            // Flying start (spec §9.4, draft 23b): the AI drives until the first start-line crossing;
            // the recorded pad input is ignored until then. Steering is the road-wheel angle.
            const VehicleState& s0 = car->state();
            ai::CarSnapshot snap;
            snap.position = s0.position;
            snap.orientation = s0.orientation;
            snap.velocity = s0.velocity;
            snap.half_length = car->setup().length * 0.5f;
            snap.half_width = car->setup().width * 0.5f;
            ai::AiContext ctx{(int(i) - hold) * kDt, std::span<const ai::CarSnapshot>(&snap, 1), 0};
            ControlInputs ai = run_up.update(*car, ctx, kDt);
            ai.automatic = true;
            car->set_reverse_request(run_up.reverse_request());
            in = ai;
        }
        if (const char* pk = std::getenv("SIM_DEMO_INPUTS"); pk && code == pk && demo->frames[std::min(i, demo->size() - 1)].steer != 0)
            std::printf("   in t=%6.3f steer=%6d kind=%d\n", (int(i) - hold) * kDt, demo->frames[std::min(i, demo->size() - 1)].steer,
                        demo->frames[std::min(i, demo->size() - 1)].kind);
        car->set_race_hold(int(i) < hold); // standing start: the race hold (as the race scene, spec §9.3)
        // [spec draft 23a] the flying-start run-up is driver kind 1 until the first line crossing
        car->set_driver_kind(es.flying() && !started ? 1 : 0);
        car->step(in, *t, kDt, &world);
        const VehicleState& st = car->state();
        track::TrackPosition tp = t->locate(st.position, hint);
        hint = tp.distance;
        float ds = tp.distance - prev;
        if (ds < -t->length() * 0.5f) ds += t->length();
        if (ds > t->length() * 0.5f) ds -= t->length();
        r.metres += ds;
        // start-line crossings (forward): the clock starts at the first, laps count after it
        if (ds > 0 && prev > t->length() * 0.5f && tp.distance < t->length() * 0.5f) {
            // the clock of a flying start runs from the crossing; a standing start's from the signal
            if (!started) started = true, start_t = es.flying() ? (int(i) + 1 - hold) * kDt : 0.0, progress = 0;
            else ++laps;
        }
        float progress_prev = progress;
        if (started) progress += ds;
        prev = tp.distance;
        {
            race::LicenseRun::Step ls;
            ls.started = started;
            ls.elapsed = started ? (int(i) + 1 - hold) * kDt - start_t : 0;
            ls.dt = kDt;
            ls.progress_prev = progress_prev;
            ls.progress = progress;
            ls.laps_completed = laps;
            ls.laps_time = ls.elapsed;
            ls.speed_mps = st.speed_kmh / 3.6f;
            ls.wall_dv = car->telemetry().wall_impulse / std::max(1.0f, spec->mass);
            for (int k = 0; k < 4; ++k) ls.grounded[k] = st.wheels[size_t(k)].grounded, ls.surface[k] = st.wheels[size_t(k)].surface;
            race::LicenseRun::State js = judge.step(ls);
            if (js == race::LicenseRun::State::Finished && r.finish_s < 0) r.finish_s = judge.time();
            if (js == race::LicenseRun::State::Failed && r.fail.empty()) r.fail = judge.failure();
        }
        if (const char* tr = std::getenv("SIM_DEMO_TRACE"); tr && code == tr && i % 15 == 0) {
            const VehicleTelemetry& tl = car->telemetry();
            Vec3 fw = rotate_fwd(st.orientation);
            float yaw = std::atan2(-fw.x, -fw.z) * 57.29578f;
            float road = std::atan2(-tp.direction.x, -tp.direction.z) * 57.29578f;
            std::printf("   road=%6.1f t=%6.2f s=%7.1f lat=%6.2f v=%5.1f g=%d rpm=%5.0f cl=%d | st=%5.2f th=%4.2f br=%4.2f sh=%d | yaw=%7.1f r=%5.2f "
                        "aF=%5.1f aR=%5.1f kR=%5.2f surf=%x%x%x%x\n",
                        road, (int(i) - hold) * kDt, tp.distance, tp.lateral, st.speed_kmh, st.gear, st.engine_rpm, int(tl.clutch_locked),
                        in.steer, in.throttle, in.brake, int(in.shift_up) - int(in.shift_down), yaw, st.angular_velocity.y,
                        st.wheels[0].slip_angle * 57.3f, st.wheels[2].slip_angle * 57.3f, st.wheels[2].slip_ratio,
                        st.wheels[0].surface & 0xF, st.wheels[1].surface & 0xF, st.wheels[2].surface & 0xF, st.wheels[3].surface & 0xF);
        }
        if (const char* tr = std::getenv("SIM_DEMO_ENGINE"); tr && code == tr && i % std::max(1, std::atoi(std::getenv("SIM_DEMO_EVERY") ? std::getenv("SIM_DEMO_EVERY") : "6")) == 0) {
            const VehicleTelemetry& tl = car->telemetry();
            std::printf("   E tick %4d rpm %5.0f g %d cl %d lk %d thr %.3f tcs %.3f proc %.3f | w %6.2f %6.2f %6.2f %6.2f | k %6.3f %6.3f %6.3f %6.3f\n",
                        int(i) - hold, st.engine_rpm, st.gear, int(tl.clutch * 100), int(tl.clutch_locked), in.throttle, tl.tcs_factor,
                        tl.throttle_processed, st.wheels[0].angular_velocity, st.wheels[1].angular_velocity, st.wheels[2].angular_velocity,
                        st.wheels[3].angular_velocity, st.wheels[0].slip_ratio, st.wheels[1].slip_ratio, st.wheels[2].slip_ratio,
                        st.wheels[3].slip_ratio);
        }
        if (const char* tr = std::getenv("SIM_DEMO_WHEELS"); tr && code == tr && i % std::max(1, std::atoi(std::getenv("SIM_DEMO_EVERY") ? std::getenv("SIM_DEMO_EVERY") : "6")) == 0) {
            std::printf("   W t=%6.2f y=%6.2f wall=%6.0f x=%7.1f z=%7.1f", (int(i) - hold) * kDt, st.position.y, car->telemetry().wall_impulse, st.position.x, st.position.z);
            for (const WheelState& w : st.wheels)
                std::printf(" | L%5.0f d%5.2f k%5.2f a%5.1f g%d s%x", w.load, w.steer_angle * 57.3f, w.slip_ratio, w.slip_angle * 57.3f, int(w.grounded),
                            w.surface);
            std::printf("\n");
        }
        int off = 0;
        for (const WheelState& w : st.wheels) {
            u8 m = track::surface_material(w.surface);
            off += w.grounded && (m == 2 || m == 3 || m == 4 || m == 6);
        }
        bool any_off = false;
        for (const WheelState& w : st.wheels) {
            u8 m = track::surface_material(w.surface);
            any_off = any_off || (w.grounded && (m == 2 || m == 3 || m == 4 || m == 6));
        }
        if (any_off && r.first_off_s < 0) r.first_off_s = (int(i) - hold) * kDt;
        if (off == 4) {
            r.out_s = (int(i) - hold) * kDt;
            r.why = "course out";
            return r;
        }
        if (!finite_state(st)) {
            r.why = "non-finite";
            return r;
        }
    }
    r.completed = true;
    return r;
}

} // namespace

TEST(sim_license_demos) {
    // Fidelity benchmark: the disc's licence demonstrations (controller input recorded
    // in the original) replayed open-loop through our physics. The further each gets
    // before leaving the road, the closer our car is to the original's.
    if (!database() || !test::test_disc()) SKIP("needs GT3R_DISC");
    const char* codes[] = {"L0B0003", "L0A0004", "L0B0001", "L0A0006", "LIB0003"};
    const char* names[] = {"B-3", "A-4", "B-1", "A-6", "IB-3"};
    int ran = 0;
    for (size_t k = 0; k < 5; ++k) {
        DemoResult r = run_license_demo(codes[k]);
        if (!r.ran) {
            std::printf("  demo %-4s (%s): not run\n", names[k], codes[k]);
            continue;
        }
        ++ran;
        char fin[96] = "not finished";
        if (!r.fail.empty()) std::snprintf(fin, sizeof fin, "not finished (judge: %s)", r.fail.c_str());
        if (r.finish_s >= 0) std::snprintf(fin, sizeof fin, "finished %.2f s (%+.2f)", r.finish_s, r.finish_s - r.recorded_s);
        char off[48] = "never";
        if (r.first_off_s >= 0) std::snprintf(off, sizeof off, "%.2f s", r.first_off_s);
        if (r.completed)
            std::printf("  demo %-4s (%s): recorded %.2f s | %s | first wheel off %s | input ran out, %.0f m\n", names[k], codes[k],
                        r.recorded_s, fin, off, r.metres);
        else
            std::printf("  demo %-4s (%s): recorded %.2f s | %s | first wheel off %s | %s at %.2f s, %.0f m\n", names[k], codes[k],
                        r.recorded_s, fin, off, r.why.c_str(), r.out_s, r.metres);
        CHECK(r.why != "non-finite");
    }
    CHECK(ran == 5);
}

TEST(sim_r34_skidpad_brake) {
    // Steady-state cornering (60 m skidpad, max sustained lateral g), 100-0 km/h braking and
    // 0-200 km/h (diagnostic for AI lap times).
    data::CarDatabase* db = database();
    if (!db) SKIP("needs GT3R_DISC");
    // constant-steer ramp: full speed sweep at a fixed steering input; the max sustained lateral g
    float best_g = 0, best_v = 0;
    {
        auto car = Vehicle::create(*db, "r34_gtr_v_99");
        FlatGround g;
        car->reset({}, {1, 0, 0, 0});
        float target = 20 / 3.6f, lat_avg = 0;
        for (int i = 0; i < 60 * 120; ++i) {
            const VehicleState& st = car->state();
            ControlInputs in;
            in.steer = -0.35f;
            if (i % 30 == 0) target += 0.05f;
            in.throttle = st.speed_kmh / 3.6f < target ? 0.5f : 0.0f;
            car->step(in, g, kDt);
            lat_avg += (std::fabs(car->telemetry().accel_local.x) / 9.8f - lat_avg) * 0.05f;
            if (i > 300 && lat_avg > best_g) best_g = lat_avg, best_v = st.speed_kmh;
        }
    }
    // braking 100-0
    float brake_m = 0, t200 = 0;
    {
        auto car = Vehicle::create(*db, "r34_gtr_v_99");
        FlatGround g;
        car->reset({}, {1, 0, 0, 0});
        ControlInputs in;
        in.throttle = 1;
        int i = 0;
        for (; i < 60 * 60 && car->state().speed_kmh < 200; ++i) car->step(in, g, kDt);
        t200 = i * float(kDt);
        in.throttle = 1;
        for (int k = 0; k < 600 && car->state().speed_kmh > 100; ++k) {
            in.throttle = 0, in.brake = 1;
            car->step(in, g, kDt);
        }
        Vec3 p0 = car->state().position;
        in.throttle = 0, in.brake = 1;
        for (int k = 0; k < 600 && car->state().speed_kmh > 0.5f; ++k) car->step(in, g, kDt);
        brake_m = (car->state().position - p0).length();
    }
    std::printf("  R34: constant-steer max %.2f g at %.0f km/h, 0-200 %.2f s, 100-0 %.1f m\n", best_g, best_v, t200, brake_m);
    CHECK(best_g > 0.3f);
}

namespace {
// Road only for z > -20; beyond it the ground query misses (off the road mesh).
struct EdgeGround : TrackQuery {
    SurfaceHit probe(float, float z, float y_max) const override {
        SurfaceHit h;
        h.hit = z > -20 && y_max >= 0;
        return h;
    }
};
} // namespace

TEST(sim_ground_miss_keeps_previous_height) {
    // spec §1.3/§5.3: a ground-query miss keeps the previous ground height; the car is not
    // stopped (it keeps driving on that height, the wheels stay grounded and the body is not
    // held up above airborne wheels).
    data::CarDatabase* db = database();
    if (!db) SKIP("needs GT3R_DISC (spec suspension block)");
    auto car = Vehicle::create(*db, "r34_gtr_v_99");
    EdgeGround g;
    car->spawn_moving({0, 0, 0}, {1, 0, 0, 0}, 50 / 3.6f);
    ControlInputs in;
    in.throttle = 0.6f;
    for (int i = 0; i < 180; ++i) car->step(in, g, kDt);
    const VehicleState& st = car->state();
    int grounded = 0;
    for (const WheelState& w : st.wheels) grounded += w.grounded;
    std::printf("  off-mesh: z %.1f, y %.3f, %.0f km/h, %d wheels grounded\n", st.position.z, st.position.y, st.speed_kmh, grounded);
    CHECK(st.position.z < -40 && grounded == 4 && std::fabs(st.position.y) < 0.1f && st.speed_kmh > 45);
}


TEST(sim_wall_contact_held) {
    // [spec §5.3] a point flagged on a wall (by an impulse or a push-out of at most
    // 0.5 m) is held on the wall line while the car keeps pressing in: the body moves so the
    // point is on the line and the velocity into the wall is removed, with no impulse. The flag
    // drops once the car moves away.
    using namespace collide;
    std::vector<Segment> walls(2);
    walls[0].a = {100, 0}, walls[0].b = {0, 0}; // track on the left of a->b: the +z side
    walls[1].a = {0, 0}, walls[1].b = {-100, 0};
    Body b;
    b.local = octagon(2.2, 2.2, 1.8);
    b.radius = bounding_radius(b.local);
    b.inv_mass = 1.0 / 1400;
    b.inv_inertia = 1.0 / 60000;
    const double dt = 1.0 / 60, c = std::atan2(1.0, 0.15); // moving to -x, nose slightly towards -z
    // The lowest point 3 cm behind the wall, flagged on segment 0 (it crosses to segment 1).
    int k = 0;
    for (int i = 1; i < 8; ++i)
        if (world_point({0, 0}, c, b.local[size_t(i)]).z < world_point({0, 0}, c, b.local[size_t(k)]).z) k = i;
    V2 p{0.2, -world_point({0, 0}, c, b.local[size_t(k)]).z - 0.03};
    WallContacts wc;
    wc.on[size_t(k)] = true;
    wc.seg[size_t(k)] = walls[0];
    wc.tangent = {-1, 0};
    b.heading = c;
    b.vel = {-30, -2};
    b.pos = p + b.vel * dt;
    std::vector<WallHit> hits = collide_walls(b, p, c, walls, dt, 2, &wc);
    V2 q = world_point(b.pos - b.vel * dt, b.heading - b.yaw_rate * dt, b.local[size_t(k)]);
    std::printf("  held wall contact: point z %.4f, vz %.3f, yaw rate %.4f, %zu hit(s)\n", q.z, b.vel.z, b.yaw_rate, hits.size());
    CHECK(hits.empty() && wc.on[size_t(k)] && b.vel.z == 0 && b.vel.x == -30);
    CHECK(std::fabs(q.z) < 1e-9);                           // held on the wall line
    CHECK(b.yaw_rate != 0);                                 // the impulse acts at the point
    CHECK(wc.seg[size_t(k)].a.x == 0 && wc.seg[size_t(k)].b.x == -100); // walked to the next segment
    // Moving away: the flag drops and the velocity is kept.
    p = b.pos;
    b.vel = {-30, 3};
    b.yaw_rate = 0;
    b.pos = p + b.vel * dt;
    collide_walls(b, p, b.heading, walls, dt, 2, &wc);
    CHECK(!wc.on[size_t(k)] && b.vel.z == 3);
}
