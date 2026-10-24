#pragma once

// Disc-free fixtures for the vehicle simulation tests: synthetic grounds and a
// hand-built car spec that takes the real spec path of sim::Vehicle::tick.
//
// Every number below is INVENTED (round values chosen for these tests). None is
// taken from the disc or from a car database record; they only need to describe
// a plausible, stable car.

#include "sim/car_spec.h"
#include "sim/models.h"
#include "sim/sim_types.h"

#include <algorithm>
#include <cmath>

namespace gt3r::test {

// An infinite flat plane at `height`.
struct FlatGround : sim::TrackQuery {
    float height = 0;
    sim::SurfaceHit probe(float, float, float y_max) const override {
        sim::SurfaceHit h;
        h.hit = y_max >= height;
        h.height = height;
        return h;
    }
};

// A plane rising along -Z.
struct Slope : sim::TrackQuery {
    float grade = 0.15f; // rise per metre along -Z
    sim::SurfaceHit probe(float, float z, float y_max) const override {
        sim::SurfaceHit h;
        h.height = -z * grade;
        h.hit = y_max >= h.height;
        h.normal = sim::Vec3{0, 1, grade} * (1.0f / std::sqrt(1 + grade * grade));
        return h;
    }
};

// Road only for z > -20; beyond it the ground query misses (off the road mesh).
struct EdgeGround : sim::TrackQuery {
    sim::SurfaceHit probe(float, float z, float y_max) const override {
        sim::SurfaceHit h;
        h.hit = z > -20 && y_max >= 0;
        return h;
    }
};

// A hand-built car with every block the spec path needs: both suspension blocks
// (so Vehicle::tick runs the spec planar step, attitude and phase-12 order, like
// every disc car), tyre records per surface class with a section width, a turbo
// engine, a 5-speed box, the spec driveline (axle inertia words, differentials),
// stock downforce, the pad steering curve, brakes and (optionally) the TCS and ASCC
// blocks. `drive` picks the layout: FR (rear viscous LSD), FF (front clutch-pack LSD),
// MR, RR, or AWD (system 3 = layout L5 with the lateral-g centre coupling).
inline sim::CarSpec synthetic_spec_car(sim::Drive drive = sim::Drive::FR, bool aids = false) {
    using namespace sim;
    constexpr float kDeg = 3.14159265f / 180.0f;
    CarSpec s;
    s.label = "synthetic_spec_car";
    s.name = "Synthetic spec-path car";
    s.mass = 1300;
    s.front_weight = drive == Drive::FF ? 0.60f : drive == Drive::MR ? 0.45f : drive == Drive::RR ? 0.40f : 0.55f;
    s.wheelbase = 2.6f;
    s.length = 4.4f;
    s.width = 1.75f;
    s.height = 1.3f;
    s.front_extent = 2.2f;
    s.rear_extent = 2.2f;
    s.cg_height = 0.30f; // the spec CoG-height parameter X
    s.yaw_inertia = 8000;
    s.power_ps = 250;
    for (size_t a = 0; a < 2; ++a) {
        AxleSpec& ax = s.axle[a];
        ax.track = 1.50f;
        ax.tire_width = 0.225f;
        ax.tire_radius = 0.31f;
        ax.wheel_inertia = 1.0f;
        ax.spin_inertia = 1.0f;  // per wheel
        ax.shaft_inertia = 0.3f; // x final drive
        ax.max_brake_torque = a == 0 ? 2500.0f : 1500.0f;
        SuspensionSpec& p = ax.susp;
        p.valid = true;
        p.k = a == 0 ? 40000.0f : 45000.0f;
        p.arb_gain = a == 0 ? 6000.0f : 4000.0f;
        p.k2 = 1.0e6f;
        p.unsprung = 10;
        p.bump_damping = 2000;
        // Two points per direction, close enough to linear that the sqrt law's slope at zero
        // speed (about critical damping here) keeps the explicit attitude step stable.
        p.bump = model::fit_damper(0.1f, 600.0f, 0.5f, 2400.0f);
        p.rebound = model::fit_damper(0.1f, 800.0f, 0.5f, 3000.0f);
        p.ride_height = 0.12f;
        p.bump_room = 0.08f;
        p.droop = 0.10f;
        p.knee_gap = 0.03f;
        ax.spring_rate = p.k;
        ax.camber = {{-0.08f, -0.02f, 0.02f, 0.08f}, {0.92f, 1.0f, 1.0f, 0.95f}};
        ax.axle_factor = 1.0f;
        ax.camber_offset = -1.0f * kDeg;
        ax.toe_byte = a == 0 ? 130 : 128;
    }
    // Tyre records: road (slot 0), loose surfaces (slot 1), wet (slot 2).
    auto tyre = [&](float mu, const char* name) {
        TireSpec t;
        t.default_curves();
        t.width = 0.225f; // the load curve is in N per metre of section width
        t.load = {{0, 10000, 20000, 40000}, {1.10f, 1.00f, 0.92f, 0.80f}};
        t.mu = mu;
        // mu[s] per surface index (tarmac, kerb, grass, gravel, sand, dirt, 6, paving, loose, rough, 10, 11, wet...)
        static const float kPct[16] = {1.00f, 0.95f, 0.60f, 0.50f, 0.45f, 0.80f, 0.90f, 0.95f,
                                       0.75f, 0.90f, 1.00f, 1.00f, 0.75f, 0.70f, 0.70f, 0.75f};
        for (size_t i = 0; i < 16; ++i) t.surface_mu[i] = mu * kPct[i];
        t.surface_mu[11] = std::min(1.0f, 1.25f / mu); // the wear grip floor
        t.wear_rate = 0.4f;
        t.compound = name;
        return t;
    };
    // (base mu above 1.25 so that tyre wear can lower the grip: its floor is min(1, 1.25 / mu))
    for (size_t a = 0; a < 2; ++a) {
        s.tire_slots[a][0] = tyre(1.50f, "synthetic_road");
        s.tire_slots[a][1] = tyre(1.30f, "synthetic_loose");
        s.tire_slots[a][2] = tyre(1.35f, "synthetic_wet");
        s.tire[a] = s.tire_slots[a][0];
    }
    s.engine.curve_rpm = {1000, 2000, 3000, 4000, 5000, 6000, 7000};
    s.engine.curve_torque = {150, 210, 250, 270, 270, 250, 220};
    s.engine.idle_rpm = 800;
    s.engine.rev_limit_rpm = 7500;
    s.engine.upshift_rpm = 7000;
    s.engine.launch_rpm = 2500;
    s.engine.inertia = 0.15f;
    s.engine.friction_coeff = 0.05f;
    s.engine.turbo.on = true;
    s.engine.turbo.cap = 1.5f;
    s.engine.turbo.stage[0] = {true, 0.6f, 3500.0f, 2.0f};
    s.gearbox.ratios = {3.4f, 2.1f, 1.5f, 1.15f, 0.92f};
    s.gearbox.final_drive = 4.1f;
    s.gearbox.reverse = 3.3f;
    DrivetrainSpec& d = s.drivetrain;
    d.drive = drive;
    // differentials: type index in "frnmhvyt" (2 open, 3 / 4 clutch packs, 5 viscous), raw setting bytes
    auto lsd = [&](int axle, int type) {
        d.lsd_type[size_t(axle)] = type;
        d.lsd_initial[size_t(axle)] = 10;
        d.lsd_accel[size_t(axle)] = 20;
        d.lsd_decel[size_t(axle)] = 10;
    };
    lsd(0, 2);
    lsd(1, 2);
    if (drive == Drive::FF) lsd(0, 4);
    else if (drive == Drive::AWD) lsd(1, 3);
    else lsd(1, 5);
    if (drive == Drive::AWD) {
        d.front_share = 0.4f;
        d.awd_system = 3;
        d.awd_share_byte = 50;
        d.awd_coupling = 0.5f;
    }
    s.aero.air_density = 1.2f;
    s.aero.c_drag = 0.40f;
    s.aero.c_front = 0.10f;
    s.aero.c_rear = 0.20f;
    s.steering.max_angle = 35 * kDeg;
    for (float kmh : {0.0f, 40.0f, 80.0f, 120.0f, 160.0f, 200.0f}) s.steering.pad_curve.x.push_back(kmh / 3.6f);
    for (float deg : {30.0f, 20.0f, 12.0f, 8.0f, 6.0f, 5.0f}) s.steering.pad_curve.y.push_back(deg * kDeg);
    s.aids.abs = true;
    s.aids.abs_gain = 2.0f;
    s.aids.abs_gain_rear = 2.0f;
    s.aids.handbrake_torque = 2000;
    if (aids) {
        s.aids.tcs = true;
        s.aids.tcs_level = 0.5f;
        s.aids.tcs_scale = 0.5f;
        s.aids.tcs_gain = 5.0f;
        s.aids.tcs_lat_a = 1.0f;
        s.aids.tcs_lat_b = 0.2f;
        s.aids.ascc_flags = 0xFF; // every channel on both axles
        s.aids.ascc_a_gain = 0.5f;
        s.aids.ascc_a_thr = 0.3f;
        s.aids.ascc_d_gain = 0.3f;
        s.aids.ascc_c_gain = 0.3f;
        s.aids.ascc_c_thr = 0.3f;
        s.aids.ascc_e_gain = 0.3f;
    }
    s.finalize();
    return s;
}

// The same chassis as a mod-style electric car (sim::EngineSpec::electric): one reduction
// ratio, no idle, lift-off regeneration (the electric branches of Vehicle::tick).
inline sim::CarSpec synthetic_spec_ev(sim::Drive drive = sim::Drive::RR) {
    using namespace sim;
    CarSpec s = synthetic_spec_car(drive);
    s.label = "synthetic_spec_ev";
    s.name = "Synthetic spec-path EV";
    s.engine = {};
    s.engine.electric = true;
    s.engine.curve_rpm = {0, 4000, 8000, 12000};
    s.engine.curve_torque = {300, 300, 150, 100};
    s.engine.idle_rpm = 0;
    s.engine.launch_rpm = 0;
    s.engine.rev_limit_rpm = 12000;
    s.engine.inertia = 0.05f;
    s.engine.friction_coeff = 0.01f;
    s.engine.regen_torque = 150;
    s.engine.regen_power = 50000;
    s.gearbox.ratios = {1.0f};
    s.gearbox.final_drive = 8.0f;
    s.drivetrain.lsd_type = {2, 2};
    s.finalize();
    return s;
}

} // namespace gt3r::test
