#pragma once

// Physical description of one car, in SI units, as consumed by sim::Vehicle.
// Built from the decoded parameter database (build_car_spec) or by hand for
// tests (CarSpec::test_car). Every field documents where its value comes
// from: [db] = decoded database field, [spec] = the original's documented behaviour,
// [placeholder] = conventional value until that behaviour is pinned down.
// See docs/dev/PHYSICS.md.

#include "base/bytes.h"

#include <algorithm>
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace gt3r::data {
class CarDatabase;
}

namespace gt3r::sim {

// Piecewise-linear table (x ascending), looked up clamped (model::lookup).
struct Curve {
    std::vector<float> x, y;
};

enum class Drive : u8 { FR = 0, FF = 1, AWD = 2, MR = 3, RR = 4 }; // = data::DriveType

// [physics §2.3] Two-stage turbo (verified on all 113 turbo cars).
struct TurboSpec {
    bool on = false;           // the cap and stage-0 peak, knee, rate bytes all non-zero
    float cap = 1;             // 1 + 0.1 x turbo part +0x11
    struct Stage {
        bool active = false;   // stage 1: its peak, knee, rate bytes all non-zero
        float peak = 0;        // A = byte x 0.1 (turbo +0x12 / +0x15)
        float knee_rpm = 0;    // N = byte x 100 (+0x13 / +0x16)
        float rate = 0;        // R = byte x 0.1 per second (+0x14 / +0x17)
    };
    std::array<Stage, 2> stage;
};

struct EngineSpec {
    std::vector<float> curve_rpm;      // [db] torque curve axis (rpm, ascending)
    std::vector<float> curve_torque;   // [db] N*m at full throttle, after power factors
    float idle_rpm = 900;              // [physics §2.1] engine +0x44 x10 rpm
    float rev_limit_rpm = 7000;        // [physics §2.1] engine +0x45 x100 (+ parts), >= +0x46 x100 + 200
    float limiter_hysteresis_rpm = 500; // [physics §2.2] fuel cut clears this far below the limit
    float inertia = 0.20f;             // [physics §2.2] drivetrain +0x20 x 0.001 kg*m^2 (x flywheel/clutch %)
    float friction_coeff = 0.06f;      // [physics §2.2] k_f, N*m per rad/s: drivetrain +0x16 x 0.98 / omega_last (rad/s)
    float launch_rpm = 2500;           // [physics §2.4] engine +0x43 x10: clutch starts to bite above this
    float upshift_rpm = 0;             // [physics §2.6] UP = engine +0x46 x100 (0 = rev limit - 500)
    TurboSpec turbo;                   // [physics §2.3]
    // Electric motor (mod cars only; GT3 has no EVs): direct drive with no idle
    // and no clutch slip, and lift-off regenerative braking instead of engine
    // braking: min(regen_torque, regen_power / omega), faded out at low speed.
    bool electric = false;
    float regen_torque = 0;            // N*m at the motor
    float regen_power = 0;             // W
};

struct GearboxSpec {
    std::vector<float> ratios;   // [db] forward gears, 1st first
    float reverse = 3.3f;        // [db] (stored positive)
    float final_drive = 4.0f;    // [db]
    float shift_time = 0.0f;     // [physics §2.6] shifts are instantaneous (1-step lockout)
    float efficiency = 0.90f;    // unused: the spec driveline has no efficiency (kept for the replay spec layout)
};

struct DrivetrainSpec {
    Drive drive = Drive::FR;          // [db] drivetrain +0x14
    float front_share = 0.0f;         // [db/placeholder] front torque share for AWD (0..1)
    u8 awd_system = 0xFE;             // [db] drivetrain +0x15 (driveline class, physics §7 #21)
    // Unused since the physics §2.7 driveline (kept for the replay spec layout, car_spec_io.h):
    float center_lock = 250;
    float lsd_preload = 40;
    float lsd_lock = 0.35f;
    float lsd_viscous = 15;
    // [physics §2.7] per-axle differential from the LSD part (0 front, 1 rear): type
    // index in "frnmhvyt" (-1 = not from the database: open, as n),
    // initial / accel / decel settings (raw bytes).
    std::array<int, 2> lsd_type{-1, -1};
    std::array<int, 2> lsd_initial{}, lsd_accel{}, lsd_decel{};
    float shift_clutch_torque = 0;    // [physics §2.4/6.2] state-3 shift clutch: the shift-clutch torque byte x 9.8 N*m (never written: 0)
    int awd_share_byte = 0;    // [physics §2.5] B4 = the AWD share byte (drivetrain +0x13)
    int awd_strength_byte = 0; // [physics §2.5] B5 = the AWD strength byte [source not in the spec yet: 0 = the table defaults]
    float awd_coupling = 0;           // [physics §7 #59] AWD class 2: k = B4 x 0.01 (N*m per rad/s); informative (model::awd_layout reads the bytes)
    float clutch_max_torque = 0;      // unused since the physics §2.4 clutch (kept for API stability)
    float launch_rpm = 0;             // unused (see EngineSpec::launch_rpm)
};

// [physics §4.2/physics §6.6] Suspension block of one axle, in the spec's suspension
// coordinate x (0 = spring force zero, negative = compressed).
struct SuspensionSpec {
    bool valid = false;        // false: use the placeholder AxleSpec spring/damper
    float k = 0;               // spring rate, N/m (b[0] x 980 x c / 100)
    float arb_gain = 0;        // N/m^2 (b[1] x 98): F = gain d |d|
    float k2 = 0;              // progressive gain below the knee, N/m^2 (b[2] x 98000)
    float unsprung = 0;        // kg per wheel (b[3] x 0.25); its weight is the tyre preload
    float bump_damping = 0;    // N per m/s below the knee while compressing (b[13] x 98)
    struct Damper {
        float t = 0, o = 0, g = 0; // F(v) = o + g sqrt(v - t), o = -g sqrt(-t)
    };
    Damper bump, rebound;
    float ride_height = 0.13f; // q (m)
    float bump_room = 0;       // bump-room byte x 0.001 (m)
    float droop = 0;           // droop byte x 0.001 (m)
    float knee_gap = 0;        // g1 = knee-gap byte x 0.001 (m)
};

struct AxleSpec {
    float track = 1.5f;          // [db] m, wheel centreline to centreline
    float tire_radius = 0.32f;   // [db] m, from tire size
    float tire_width = 0.225f;   // [db] m
    float wheel_inertia = 1.2f;  // [placeholder] kg*m^2 per wheel (tyre + rim + brake)
    // [physics §2.7/6.2] driveline inertia words (0 = not from the database):
    // spin = 0.01 x the spin word (per wheel), shaft = 0.01 x the shaft word (x final drive).
    float spin_inertia = 0, shaft_inertia = 0;
    float spring_rate = 0;       // [placeholder] N/m per wheel; 0 = derived from ride frequency
    float ride_frequency = 1.6f; // [placeholder] Hz, used when spring_rate == 0
    float damping_ratio = 0.35f; // [placeholder] fraction of critical
    float damper = 0;            // [placeholder] N*s/m per wheel; 0 = derived from damping_ratio
    float rest_length = 0.30f;   // [placeholder] spring length at full droop (m)
    float travel = 0.16f;        // [placeholder] usable compression before the bump stop (m)
    float arb_rate = 0;          // [placeholder] N/m per metre of left/right difference; 0 = derived
    float max_brake_torque = 0;  // [physics §4.5] brake +0x11/+0x12 x 98 N*m per wheel; 0 = derived placeholder
    SuspensionSpec susp;         // [physics §4.2] from the database (stock settings physics §6.6)
    Curve camber;                // [physics §3.1/6.4] Cam(angle) from suspension +0x60.. codes; empty = 1
    float axle_factor = 1.0f;    // [physics §3.1] (chassis byte + 900) x 0.001
    float camber_offset = 0;     // [physics §3.1] -(camber setting x 0.1 deg), rad
    int toe_byte = 128;          // [physics §4.4/6.6] toe setting (128 = neutral)
    float toe = 0;               // [physics §4.4] atan2((toe byte - 128) x 0.0005, tyre radius), rad (finalize)
};

struct TireSpec {
    // [physics §3, physics §6.5] one runtime tyre record. build_car_spec decodes the real
    // curves from the database (compound table 27 + force-vol table 26);
    // default_curves() is the placeholder for hand-built specs.
    Curve load;        // L(Fz / section width): x in N per metre of width
    Curve slip_a;      // A(|beta|): lateral force fraction vs folded slip angle (rad)
    Curve slip_b;      // B(|beta|): force along the patch velocity vs slip angle
    Curve limit_d;     // D(s): friction-circle limit vs signed slip ratio
    Curve long_c;      // C(s): longitudinal force fraction vs signed slip ratio
    std::array<float, 16> surface_mu{}; // mu[s] (base mu x force-vol %); all 0 = mu x model::surface_mu(s)
    float width = 0.225f;   // section width (m), for L
    float fz_ref = 4000;    // [placeholder] reference load when `load` is in load-ratio units (width == 0)
    float mu = 1.0f;        // base mu (compound +0x08 / 100)
    float wear_grip = 1.0f; // [physics §3.7] 1 when wear is off
    float wear_rate = 0.4f; // [physics §8.2] compound byte +0x52 / 100 (0.4 = normal compound)
    float opt_drive = -0.10f; // optimum drive slip (first max of C on the wheelspin side)
    float opt_brake = 0.10f;  // optimum braking slip (first max of C on the braking side)
    std::string compound;   // [db] table 27 label

    // Placeholder curves shaped like a road tyre (peak slip ~8 deg / 10 %).
    void default_curves();
};

struct AeroSpec {
    // [physics §5.2] drag = -sign(v) C_d (A + (1-A)(1-y)) v^2, lift_j = C_j (B + (1-B)(1-y)) v^2.
    float c_drag = 0;          // N / (m/s)^2 [placeholder value: 0 = 0.5 rho CdA from drag_area]
    float c_front = 0, c_rear = 0; // downforce coefficients [placeholder 0]
    float slip_a = 0.5f, slip_b = 1.0f;       // [physics §5.2] A, B
    float slip_k_up = 2.0f, slip_k_down = 0.5f; // [physics §5.2] 1/s
    float drag_area = 0.70f;   // [placeholder] Cd*A used to derive c_drag
    float air_density = 1.225f;
};

struct SteeringSpec {
    Curve pad_curve;               // [physics §4.4/6.4] curve B: forward speed (m/s) -> max angle (rad)
    float follow_k1 = 100, follow_k2 = 5, follow_k3 = 50; // [physics §4.4, values I]
    // [physics §4.4 input path] follower rate limits: analog pad (O[3] = 2) 120 deg/s
    // both ways; digital (O[3] = 1) 30 deg/s away from centre, 120 deg/s towards it.
    float follow_rate_max = 2.0944f;        // rad/s away from centre (analog)
    float follow_return_rate_max = 2.0944f; // rad/s towards the centre
    float follow_rate_digital = 0.5236f;    // rad/s away from centre, digital steering
    float max_angle = 0.55f;       // [physics §6.4] steer +0x11 degrees (full lock); fallback curve
    float speed_reduction = 22.0f; // [placeholder] m/s: lock = max / (1 + v / this)
    float rate = 3.5f;             // [placeholder] rad/s steering slew at the wheel
    float ackermann = 0.6f;        // [placeholder] 0 = parallel, 1 = full Ackermann (when the ratios are unset)
    float ack_inner = 0, ack_outer = 0; // [physics §4.4] atan2(L, L -+ tf)/(pi/4); 0 = use `ackermann`
};

struct AidsSpec {
    bool abs = true;   // [physics §4.5] the pressure limit is always on
    float abs_gain = 0.5f;   // [physics §4.5/6.6] G = brake-controller setting x 0.1 (front)
    float abs_gain_rear = 0.5f; // the rear setting x 0.1
    float slip_opt = 0.10f;  // [placeholder] tyre optimum braking/drive slip (record value unknown)
    float handbrake_torque = 0; // [physics §4.5] brake +0x13 x 98 N*m on each rear wheel (0 = placeholder)
    bool tcs = false;  // [db] on when the car's TCS part (table 22) is not "off"; level [placeholder]
    float tcs_slip = 0.12f;
    float tcs_level = 0;     // [physics §4.6] TCS slip-scale byte x 0.01
    // [physics §4.6] TCS gain (byte x 0.1, 0 = off), lateral a / b (bytes x 0.1 /
    // x 0.01), slip scale (byte x 0.01, 1 when 0 or 100).
    float tcs_gain = 0, tcs_lat_a = 0, tcs_lat_b = 0, tcs_scale = 1;
    // [physics §4.6] ASCC channels A (oversteer brake), C (understeer brake, throttle
    // closed), D / E (brake-pressure relief); flag byte bits 0..7 =
    // A-front, A-rear, D-front, D-rear, C-front, C-rear, E-front, E-rear.
    float ascc_a_gain = 0, ascc_a_thr = 0, ascc_d_gain = 0, ascc_c_gain = 0, ascc_c_thr = 0, ascc_e_gain = 0;
    u8 ascc_flags = 0;
};

struct CarSpec {
    std::string label;          // database label ("r34_gtr_v_99")
    std::string name;           // display name
    float mass = 1400;          // [physics §6.4] kg: chassis weight x lightweight x body factors + 120 kg
    float front_weight = 0.55f; // [db] static front axle share
    float wheelbase = 2.6f;     // [db] m
    float length = 4.5f, width = 1.75f, height = 1.35f; // [db] m (body box)
    float cg_height = 0.50f;    // [placeholder] m above ground
    float front_extent = 0, rear_extent = 0; // [physics §5.3/6.2] CoG->front/rear (m), body f32 +0x20/+0x24 mm; 0 = length/2
    float yaw_inertia = 0;      // [physics §6.4] chassis +0x1C x lightweight x body % x 100 (game units; 6000 if 0)
    std::array<u8, 16> surface_slot{0, 0, 1, 1, 1, 1, 2, 0, 0, 0, 0, 0, 2, 2, 2, 2}; // [physics §3.6]
    float brake_balance = 0.65f; // [placeholder] front share of brake torque
    float brake_decel = 1.35f;   // [placeholder] total brake torque, in g at nominal tyre radius
    float power_ps = 0;          // [db] catalogue power (informative)
    std::array<AxleSpec, 2> axle;  // 0 front, 1 rear
    std::array<TireSpec, 2> tire;  // 0 front, 1 rear: compound slot 0 (also the fallback)
    // [physics §6.5] records per axle and compound slot (surface class
    // surface_slot[s]); empty curves = use tire[axle].
    std::array<std::array<TireSpec, 3>, 2> tire_slots;
    EngineSpec engine;
    GearboxSpec gearbox;
    DrivetrainSpec drivetrain;
    AeroSpec aero;
    SteeringSpec steering;
    AidsSpec aids;

    // A self-contained R34-GT-R-like car for tests and the no-disc fallback.
    static CarSpec test_car(Drive drive = Drive::AWD);
    // Fills derived [placeholder] values (spring rates, brake torques, clutch,
    // launch rpm...). Idempotent; called by build_car_spec and Vehicle::create.
    void finalize();

    float peak_torque() const;            // N*m
    float peak_power_rpm() const;         // rpm of max power on the curve
    float peak_power_kw() const;
};

// User settings of a car (physics §6.6: the setting words 0xF0..0x1C8 of the saved garage car),
// raw as the game stores them. stock_settings() gives the defaults; garage
// tuning edits a copy and passes it to build_car_spec.
struct CarSettings {
    std::array<int, 8> gears{};       // 0xF0 reverse, 0xF4.. gears 1-7 (x1000; -999 = none)
    int final_drive = 0;              // 0x110, x1000
    int gear_slider = 0;              // 0x114: computed gearboxes, design top speed / 10 km/h (see apply_gear_slider)
    int front_torque = 0;             // 0x118: drivetrain split (stored; the game reads drivetrain +0x13)
    int brake_front = 1, brake_rear = 1; // 0x11C / 0x120: brake-controller indices (1-based)
    int downforce_front = 0, downforce_rear = 0; // 0x124 / 0x128: percent of A_f
    std::array<int, 6> turbo{};       // 0x12C..0x140: stage 0 peak, knee, rate, stage 1 peak, knee, rate (raw bytes)
    std::array<int, 2> camber{};      // 0x144 / 0x148 -> the camber bytes (front, rear)
    std::array<int, 2> ride_height{}; // 0x14C / 0x150, mm
    std::array<int, 2> toe{};         // 0x154 / 0x158 (128 = neutral)
    std::array<int, 2> spring{};      // 0x15C / 0x160, 0.1 kgf/mm
    std::array<int, 2> motion_ratio{}; // 0x164 / 0x168, percent
    std::array<int, 8> damper{};      // 0x16C..0x188 indices: front bump 1, bump 2, rebound 1, rebound 2 (unused:
                                      // the front second stage reuses 0x16C/0x170), rear bump 1, bump 2, rebound 1, rebound 2
    std::array<int, 2> arb{};         // 0x18C / 0x190 indices
    std::array<int, 6> lsd{};         // 0x194..0x1A8: front initial, rear initial, front accel, rear accel, front decel, rear decel
    int tcs = 0;                      // 0x1B0 TCS index
    std::array<int, 4> ascc{};        // 0x1BC..0x1C8 ASCC indices (A, D, C, E)
};

// [menus §9.3] The top speed the menus show (m/s): from
// 0 km/h to the rev limit in top gear, in steps of (that km/h) / 500 + 1, shifting
// up past the upshift rpm, the last speed where the full-throttle drive force beats
// the tarmac rolling resistance and the aero drag.
float showroom_top_speed(const CarSpec& spec);

// Regenerates the gear words of a computed (adjustable) gearbox from
// settings.gear_slider and final_drive [physics §6.7] - what the game does when the
// slider moves; build_car_spec itself uses the gear words as given. Returns
// false for ordinary gearboxes.
bool apply_gear_slider(const data::CarDatabase& db, const std::array<u64, 27>& parts, CarSettings& settings);

// Stock (default) settings of a car with its installed parts [physics §6.6]
// (computed gearboxes get their generated ratios).
CarSettings stock_settings(const data::CarDatabase& db, const std::string& car_label, const std::array<u64, 27>& parts);

// Builds the spec of a car (table 30 label, e.g. "r34_gtr_v_99") or a car
// variant (tables 31/35, e.g. an opponent car) with its installed parts.
// Returns nullopt with *error set when the label or a required part is missing.
std::optional<CarSpec> build_car_spec(const data::CarDatabase& db, const std::string& label,
                                      std::string* error = nullptr);
// Same, for an explicit part list (Car::parts order, 0 = none / stock).
std::optional<CarSpec> build_car_spec(const data::CarDatabase& db, const std::string& car_label,
                                      const std::array<u64, 27>& parts, std::string* error = nullptr);
// With explicit user settings (the overloads above use stock_settings()).
std::optional<CarSpec> build_car_spec(const data::CarDatabase& db, const std::string& car_label,
                                      const std::array<u64, 27>& parts, const CarSettings& settings,
                                      std::string* error = nullptr);

// The Options menu's ASM / TCS (arcade single race, free run, 2P battle). Off switches the car's
// own aid off: ASM clears every ASCC channel's flag and gain (so the ASM lamp, lit by any gain,
// goes out with it), TCS zeroes the TCS gain. On keeps what the car's parts give it.
void apply_aid_options(AidsSpec& aids, bool asm_on, bool tcs_on);

} // namespace gt3r::sim
