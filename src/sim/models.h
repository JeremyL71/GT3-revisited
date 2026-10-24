#pragma once

// Pure sub-models of the vehicle simulation. Each is a free function of
// plain inputs with no hidden state, so it can be unit-tested and checked
// one-to-one against the original game's behaviour, and swapped
// independently.
// Units: SI (N, m, s, rad, rad/s) except rpm where named.
//
// Tags: [physics §n] follows docs/behaviour/physics.md section n; [placeholder] is a
// conventional model until the behaviour is pinned down.

#include "base/game_random.h"
#include "sim/car_spec.h"
#include "sim/sim_types.h"

#include <numbers>

namespace gt3r::sim::model {

// The sim's pi: double code uses kPi / kTwoPi, float code kPiF / kTwoPiF (the nearest floats, 3.14159265f and
// 6.2831853f).
constexpr double kPi = std::numbers::pi;
constexpr double kTwoPi = 2 * kPi;
constexpr float kPiF = float(kPi);
constexpr float kTwoPiF = float(kTwoPi);
constexpr float kRpmToRadS = 0.10471976f; // 2*pi/60
constexpr float kRadSToRpm = 9.5492966f;
constexpr float kGravity = 9.8f;          // [physics §0] the game's g
constexpr float kKgfmToNm = 9.8f;
constexpr double kTickDt = 1.0 / 60.0;    // [physics §1.1]

// --- generic -----------------------------------------------------------------

// [physics §2.1] Piecewise-linear lookup, clamped to the first/last y
// outside the axis. n <= 0 returns 0.
float table_lookup(const float* x, const float* y, int n, float v);
inline float lookup(const Curve& c, float v) {
    return table_lookup(c.x.data(), c.y.data(), int(std::min(c.x.size(), c.y.size())), v);
}

// [physics §1.3] Body axes from Euler angles in the game's Z-up physics
// frame: R = Rz(heading) * Ry(pitch) * Rx(roll); outputs the forward, lateral
// and up columns (forward = (0,1,0) at zero angles).
void euler_axes(float roll, float pitch, float heading, float forward[3], float lateral[3], float up[3]);
// Physics frame (Z up) <-> course/world frame (sim_types.h, Y up) [physics §1.3]:
// GTRW = (phys.x, phys.z, -phys.y).
inline Vec3 phys_to_world(const float p[3]) { return {p[0], p[2], -p[1]}; }

// --- engine ------------------------------------------------------------------

// [physics §2.1] Full-throttle torque (N*m) at `rpm`: table_lookup on the curve
// (clamped outside the axis).
float curve_torque(const EngineSpec& e, float rpm);

// [physics §2.1] Runtime curve from raw spec values: torque points
// (0.01 kgf*m), rpm axis (x100 rpm), output factor (1000 = x1),
// high-rpm factor (per mille) and low-rpm factor (percent).
// Writes rpm/N*m pairs; returns the point count.
int build_torque_curve(const u16* torque_centi_kgfm, const u8* rpm_x100, int n, u16 out_factor, u16 hi_permille,
                       u8 lo_percent, float* out_rpm, float* out_nm, const TurboSpec* turbo = nullptr);
// (with a turbo, each point is multiplied by min(B, cap) / B, B = uncapped
// boost at that rpm with both stages at the axis rpm [physics §2.1])

// [physics §2.2] Engine braking F(omega) = k_f * omega (N*m, >= 0 for omega >= 0).
float friction_torque(const EngineSpec& e, float rpm);

struct EngineOut {
    float torque = 0;        // net crank torque, N*m (negative = engine braking)
    float throttle = 0;      // effective load x' after the fuel cut
    bool limiter = false;    // fuel cut active (state for the next call)
    float wot = 0;           // full-throttle torque at this speed
};
// [physics §2.2] T = x' T_wot - (1 - x') F; the fuel cut sets at
// rpm >= rev limit and clears below rev limit - 500 rpm (`limiter_was` =
// previous state); while cut, T = -F. x' = `throttle`: for a turbo engine the caller
// passes turbo_step's lagged throttle (physics §2.3); the curve holds steady-state boost.
// `condition` (physics §10.3) scales the full-throttle torque T_wot (not the friction).
EngineOut engine_torque(const EngineSpec& e, float rpm, float throttle, bool limiter_was, float condition = 1.0f);

// --- turbo [physics §2.3] ----------------------------------------------------

// One stage's boost at lagged rpm n (shape type 0): below the knee
// A (1 - ((N - n)/N)^2), from N to 3N A - 0.25 A ((n - N)/N)^2, above 3N 0.
float turbo_stage(const TurboSpec::Stage& st, float rpm);
// Uncapped boost 1 + shape0(n0) + shape1(n1) (inactive stages contribute 0).
float turbo_boost_raw(const TurboSpec& t, float n0, float n1);
// Capped boost Bcap = min(boost_raw, cap).
float turbo_boost(const TurboSpec& t, float n0, float n1);

struct TurboState {
    std::array<int, 2> lag{};  // s16 lagged rpm per stage
    float load = 0;            // previous load x * Bcap(lag)
};
// One step: target = load_prev * n / Bcap(n, n); each active stage's lag +=
// trunc((target - lag) * rate * dt), floored at 0; load = x * Bcap(lag0, lag1);
// returns the effective throttle x' = min(1, load / Bcap(n, n)). `rpm` is the
// integer engine rpm of physics §2.2. Without a turbo returns x.
float turbo_step(const TurboSpec& t, TurboState& s, float throttle, int rpm, float dt);

// [physics §2.4] Anti-stall of a slipping clutch (checked only when the
// speeds did not cross this step): opens when the brake pedal is
// > 0.75 and the new engine speed < (idle_rpm + 500) x 2pi/60 (product
// chopped), both strict.
bool clutch_anti_stall(float brake, float engine_speed, float idle_rpm);

// [physics §2.4] Torque transmitted by a slipping clutch: magnitude
// 0.05 T_wot + (T > 0 ? s(|dw|) T : 0), s(d) = min(1 + max(d - 52.36, 0) / 837.76, 1.125),
// signed by (engine speed - wheel-side speed). dw in rad/s.
float clutch_slip_torque(float wot, float engine_torque, float dw);

// --- gearbox [physics §2.5-2.6] -------------------------------------------------------

float total_ratio(const GearboxSpec& g, int gear); // gear * final (-1 = reverse, 0 = neutral -> 0)
float upshift_rpm(const EngineSpec& e, const GearboxSpec& g, int gear); // [placeholder helper] torque crossover

struct ShiftIn {
    int gear = 1;            // -1 reverse, 1..N forward (our numbering)
    float rpm = 0;           // engine rpm
    float speed = 0;         // forward speed, m/s
    float throttle = 0, brake = 0;
    bool clutch_locked = true;
    bool automatic = true;
    int request = 0;         // manual +1 / -1
    bool reverse_request = false;
    bool traction_veto = false; // a front/driven wheel is airborne or spinning past its optimum
    float driven_radius = 0.3f;
};
// [physics §2.6] Gear decision for one step (lockout/anti-hunt
// counters are kept by the caller). UP = spec up-shift rpm (engine +0x46 x100),
// down table k+1 -> k = max(UP * r[k+1]/r[k] - rpm per 10 km/h in k+1, launch + 500),
// -500 rpm when throttle > 0.25 (B3 = 5). Returns the gear to select.
int shift_logic(const EngineSpec& e, const GearboxSpec& g, const ShiftIn& in);

// [physics §6.7] Ratio generator for computed (adjustable racing)
// gearboxes. n = forward gears, rev_byte = the rev byte (rev limit / 100),
// final_word = the final-drive word (final drive x 1000), driven tyre size codes (rim
// inches, width / 5 mm, aspect / 5 %), slider = design top speed / 10 km/h.
// out[0] = reverse, out[1..n] = gears, all x 1000 (s16). PS2 float
// multiplies/adds chop towards zero; the final drive is computed in double.
// `drive` / `awd_system` = drive type / AWD system B3: the caller passes the codes of the
// driven tyre, which is the front one for FF and for AWD system 0 (rear otherwise).
void generate_gear_ratios(int n, int rev_byte, int final_word, int rim_in, int width_code, int aspect_code, int slider,
                          s16 out[8], int drive = 0, int awd_system = 0xFE);

// [physics §7 #24, verified] Gear for a road speed (m/s): 0 if speed < 0,
// else the lowest k in 1..N-1 with speed < (UP - 500) rpm in gear k, else N.
int gear_for_speed(const GearboxSpec& g, float speed, float driven_radius, float up_rpm);

// [placeholder] previous automatic logic, kept for reference/tests.
struct AutoShiftIn {
    int gear = 1;
    float rpm = 0;
    float road_rpm = 0;
    float throttle = 0;
    float brake = 0;
    float speed = 0;
};
int auto_shift(const EngineSpec& e, const GearboxSpec& g, const AutoShiftIn& in);

// --- surfaces [physics §3.6] -----------------------------------------------------------

// Physics surface index 0..15 from a raw SurfaceHit::surface value
// (material | flags << 8): with flags & 2, material 0/7 -> 12, 1 -> 13, 9 -> 14.
int surface_index(u32 raw);
// Rolling-resistance coefficient rr[s] (static per-surface table) [physics §3.6].
float surface_rolling(int s);
// [placeholder] Per-surface friction factor for tyres without a mu[s] table (hand-built specs use
// mu x surface_mu(s)); the disc's tyre records carry their own mu[s] (physics §6.5).
float surface_mu(int s);
// Road bump height added to the suspension input at world (x, z) for hub speed v.
float road_bump(int s, float x, float z, float hub_speed);

// --- tyres [physics §3] -------------------------------------------------------------

// [physics §3.2] Raw slip angle atan2(-vy, vx) and low-speed fade:
// 0 at <= 1 km/h, 1 at >= 10 km/h, linear between. The angle is 0 when the
// fade is 0.
float patch_slip_angle(float vx, float vy, float& fade);

// [physics §3.4] Slip ratio from x = hub speed along the wheel
// heading, w = wheel peripheral speed and u = hub forward speed: signed
// denominators (w when driving, x when braking), gain 0.5 + 0.5 min(|u|/27.78, 1),
// clamp to [-1, 1], low-speed fade |r| / 5.56 m/s. Forward: < 0 wheelspin, > 0 braking.
float slip_ratio(float x, float w, float u);

// [physics §3.1] Peak force F_peak = Fz * L(Fz / width) * wear * mu[s] * Cam(camber)
// (axle factor applied by the caller). Records without a per-surface table use
// mu * surface_mu(s); without a width, L is evaluated at Fz / fz_ref.
float tire_peak_force(const TireSpec& t, float fz, int surface, float cam);

struct TireIn {
    float vx = 0, vy = 0;    // contact-patch velocity in the wheel frame (x along heading, y right), m/s
    float slip_ratio = 0;    // slip_ratio() result
    float rel = 0;           // x - w (hub minus wheel speed): the longitudinal force opposes it
    float peak = 0;          // tire_peak_force()
    float scale = 1;         // per-wheel enable/scale k (contact)
};
struct TireOut {
    float fx = 0, fy = 0;    // force on the car in the wheel frame (x along heading, y right), N
    float slip_angle = 0;    // folded |beta| with sign (diagnostic)
    float grip_used = 0;     // |(u1, u2)| / limit (> 1 = clipped by the friction circle)
    float longitudinal = 0;  // the C(s) part along the heading (included in fx)
};
// [physics §3.2-3.3] Lateral pair u1 = sg*A(v)*k, u2 = B(v)*k from the folded slip
// angle, scaled down to the friction-circle limit D(slip ratio); F1 acts
// perpendicular to the patch velocity (against the slide), F2 along it
// (against the motion). D and C are looked up at the signed slip ratio (physics §6.5
// axis -1..1); longitudinal |Fx| = C(s) * F_peak along the heading,
// directed against the hub/wheel relative motion `rel` (or -sign(s) when rel = 0).
TireOut tire_force(const TireSpec& t, const TireIn& in);

// --- suspension / aero / brakes ---------------------------------------------------

// [physics §4.6, physics §7 #12] TCS lateral factor: 1 if |x| <= b, else max(0, 1 - (|x| - b) a).
float tcs_lateral(float x, float a, float b);
// [physics §4.6] Throttle factor clamp(1 + gain g (s_min - s_opt lat scale), 0, 1).
float tcs_factor(float gain, float slip_gain, float s_min, float s_opt, float lat, float scale);
// [physics §4.6] Under/oversteer indicators in [0, 1] (forward motion only).
// understeer: target yaw r_t = v sin(steer) / L [interpretation of "v cos(delta)/L"];
// oversteer: rear slip angle m (rad) vs threshold 20 deg -> 5 deg over q = min(1, 0.018 v).
// Gated on lat_ratio = |lateral force| / sum of tyre peak forces > 0.2; adds half
// the yaw acceleration when it shares the yaw rate's sign; 0 when the yaw rate
// opposes the steer.
float understeer_indicator(float speed, float steer, float wheelbase, float yaw_rate, float yaw_acc = 0,
                           float lat_ratio = 1);
// `speed` = total planar speed sqrt(vx^2 + vy^2).
float oversteer_indicator(float speed, float rear_slip_angle);
// Alternative oversteer (driver modes 1/2): 2 max(0, |s r| - |lat|) / tyre_sum if s r lat < 0, else 0.
float oversteer_alt(float s, float r, float lat, float tyre_sum);

// [physics §4.5] ABS pressure limit: p * clamp(F_peak E (1 - G g (s - s*)), 0, 1)
// with E = tire radius / B (B = max brake torque of the axle), s = braking slip.
float abs_pressure(float p, float f_peak, float radius, float brake_max, float gain, float slip_gain, float slip,
                   float slip_opt);

// [placeholder] Spring + damper force (N, >= 0) for compression x (m) and
// compression velocity v (m/s), with bump stop (cars without a spec block).
float suspension_force(const AxleSpec& a, float x, float v);

// [physics §4.2] Fits F(v) = g (sqrt(v - t) - sqrt(-t)) through two
// points (v1, F1), (v2, F2), t < 0. Invalid points give a zero damper.
SuspensionSpec::Damper fit_damper(float v1, float f1, float v2, float f2);
// [physics §4.2] Damper force at suspension velocity v (clamped to
// +-1 m/s; v < 0 = compressing gives a positive force).
float damper_force(const SuspensionSpec& s, float v);

struct SuspensionOut {
    bool grounded = false;
    float x = 0;          // suspension coordinate after the bump-stop clamp
    float spring = 0;     // Fs
    float damper = 0;     // Fd
    float load = 0;       // tyre load Fz = max(0, Fs + Fd + preload)
};
// [physics §4.2/physics §10.5] One wheel: x = suspension coordinate, v = its rate
// (from the unclamped x), x_static = the axle's static position x_s. Airborne
// (x > droop): x = droop, spring = -preload, damper = load = 0. The caller stores
// the returned x (clamped) as the next x_prev; the body force is spring + damper.
SuspensionOut suspension_wheel(const SuspensionSpec& s, float x, float v, float x_static);
// [physics §4.2] The per-axle block positions derived from the static position x_s:
// bump-stop limit x_s - q + min(q, bump room), knee max(x_s - g1, limit), max droop min(x_s + droop, preload / k).
struct SuspensionLimits {
    float knee = 0, limit = 0, droop = 0;
};
SuspensionLimits suspension_limits(const SuspensionSpec& s, float x_static);
// [physics §4.2] x_s = -(half the axle's share of the sprung mass) x g / k, sprung mass = total - 2 (front + rear
// unsprung), share = front weight fraction (front) or its complement (rear).
float static_position(const SuspensionSpec& s, float mass, float share, float unsprung_front, float unsprung_rear);
// [physics §4.2] Anti-roll force on the left wheel (the right gets the negative),
// from x_left - x_right; physically resisting roll.
float anti_roll(const SuspensionSpec& s, float x_left, float x_right);

// [physics §5.2] Slipstream factor of a follower at (x forward, y
// lateral) in the leader's frame; leader forward speed v_leader, half-width h.
float slipstream_factor(float x, float y, float v_leader, float half_width);

// [physics §5.2] One step of the slipstream filter and the aero forces.
struct AeroOut {
    float drag = 0;          // signed force along the forward axis (opposes v)
    float lift_front = 0;    // C_front * (B + (1-B)(1-y)) * v^2
    float lift_rear = 0;
    float slip = 0;          // filtered slipstream factor y after this step
};
AeroOut aero_step(const AeroSpec& a, float v, float slip_target, float slip_prev, float dt);

// Rolling-resistance speed curve RR(|v|) [physics §3.5].
float rolling_curve(float speed);

// [physics §4.3] Longitudinal pitch terms: q_i = cos(delta_i) x
// longitudinal tyre force_i x section width_k; A = sum q_i kP_k (rad/s^2) with
// kP_k = 0.5 x 2pi / (Y + m a^2) front, (Y + m b^2) rear; B = (q0 + q1)/a - (q2 + q3)/b (N).
struct PitchTerms {
    float a_term = 0, b_term = 0;
};
PitchTerms pitch_terms(const float cos_delta[4], const float fx[4], const float width[2], float y_param, float mass,
                       float a, float b);

// [physics §4.3] One attitude step. F = per-wheel body forces (spring +
// anti-roll + transfer - downforce share), D = damper forces; lever factors
// lf = a 2pi/Y, lr = b 2pi/Y, rf/rr = half track 2pi/Z. Rates clamp to +-pi/2,
// angles to +-pi/4 (rate zeroed when clamped). Returns the heave force
// sum F + sum D + B (N).
float attitude_step(const float f[4], const float d[4], float lf, float lr, float rf, float rr, PitchTerms pt, float dt,
                    float& pitch, float& pitch_rate, float& roll, float& roll_rate);

// [physics §4.3] Bump-stop attitude nudge from the penetrations d_i
// (= max(0, bump limit - x_i)), w = 1 - front weight fraction: adds to roll,
// pitch and height (no clamps). Returns false when no wheel is at a bump stop.
struct Nudge {
    float roll = 0, pitch = 0, height = 0;
};
bool bump_nudge(const float pen[4], float w, float mean_track, float wheelbase, Nudge& out);

// [physics §4.3] Ground frame from the ground heights under the
// four hubs (FL, FR, RL, RR), w = 1 - front weight fraction: height under the
// CoG = (U + V)/2 with U = g0 + w (g2 - g0), V = g1 + w (g3 - g1); pitch =
// atan2(((g0 + g1) - (g2 + g3))/2, wheelbase) (nose up +); roll = atan2(U - V,
// mean track) (right side down +). The ground frame is the Euler matrix of
// (pitch, roll, heading); terrain factor s = its up.z / body up.z.
struct GroundFrame {
    float height = 0, pitch = 0, roll = 0;
};
GroundFrame ground_frame(const float g[4], float w, float wheelbase, float mean_track);

// [physics §4.3] Spawn-hold height offset for counter value n: 0 for
// n < 30, 0.3 m for 30 <= n < 270, (300 - n) x 0.01 m above.
float spawn_height_offset(int n);

// [physics §4.2] Bump-stop limit of an axle at static position x_s.
float bump_limit(const SuspensionSpec& s, float x_static);

// [physics §4.4] Ackermann ratios atan2(L, L -+ tf) / (pi/4) (tf = half
// front track) and the toe angle atan2((byte - 128) x 0.0005, tyre radius).
void ackermann_ratios(float wheelbase, float half_track, float& inner, float& outer);
float toe_angle(int toe_byte, float tyre_radius);

// [physics §4.4] Max steer angle at forward speed v from the pad curve (curve B),
// falling back to the placeholder lock / (1 + v / speed_reduction) when absent.
// `velocity_angle` (physics §4.4): the mean velocity-direction angle of the two front wheels, in the
// input's sign convention. Only the bound on its own side is extended to it (m > lock raises the
// bound on that side, the other stays at the lock); both bounds are clamped to the full lock, and the
// input scales the bound on its side (0 = no extension).
float steer_target(const SteeringSpec& s, float input, float speed, float velocity_angle = 0);
// [physics §4.4] Second-order steering follower: acc = k1 (t - d) - k2 w (- k3 d when
// returning towards 0), |w| <= w_max, snap to t when crossing it.
// `digital` (pad state O[3] = 1): the slow rate limit away from centre.
void steer_follow(const SteeringSpec& s, float target, float& angle, float& rate, float dt, bool digital = false);
// [physics §4.4] Pedal shaping of the pad path: x + 0.1 (1 - x) once x > 0.
float pedal_shape(float x);
void ackermann(const SteeringSpec& s, float angle, float wheelbase, float track, float& left, float& right);

// --- differential ---------------------------------------------------------------------

// [physics §2.7] Torque moved from the faster to the slower wheel of an axle
// differential of the given type (index in "frnmhvyt"): open (f, n, y, t) 0;
// locked (r) the full sync torque; clutch pack (m, h) up to
// C = max(initial x 9.8, (T >= 0 ? accel : decel) x 0.01 x |T|); viscous (v)
// gain min(dw^2, cap), gain = accel x 0.98, cap = initial^2. dw = w_left -
// w_right (rad/s), sync = torque that equalises the speeds in one step.
// Returns the locking torque L (left gets T/2 - L, right T/2 + L).
// Slipping transfer only: clutch packs give exactly +-C (no sync
// limit), viscous gain x min(dw^2, cap) x sign; r returns `sync`. The lock flag
// with torque hysteresis for m/h is kept by the vehicle.
float lsd_lock(int type, int initial, int accel, int decel, float axle_torque, float dw, float sync);
// [physics §2.7] One step of an
// axle differential. Inputs: the wheel speeds after an open (equal-split)
// integration step, that step's net wheel torques (drive/2 - tyre force x radius
// - brake) and the wheel inertia. With dw = the previous speed difference:
//   f (0): wheels integrate alone; a wheel with brake torque != 0 whose speed
//     crosses zero (or was or becomes exactly 0) is set to 0;
//   r (1): both wheels at the axle base;
//   m, h (3, 4): stay locked (d = 0) while flagged and |acc_l - acc_r| <= 2C,
//     else unlock and move C between the wheels' torques (towards the slower);
//   v (5): move gain min(dw^2, cap) likewise; n, y, t: open;
//   then d = dw/2 + (acc_l - acc_r) dt / (2 I); a sign change against dw gives
//   d = 0 for every type (and sets the clutch-pack lock flag).
struct DiffState {
    bool locked = false;
    float half = 0; // half-difference after the last step
};
void diff_step(int type, int initial, int accel, int decel, float axle_torque, float inertia, float dt, float acc_left,
               float acc_right, float& w_left, float& w_right, DiffState& state);
// Same with the previous step's speed difference given exactly (diff_step
// reconstructs it from the torques, which loses precision near dw = 0).
// `brake_left/right` (type f only): the wheels' brake torques; `w_left/right_prev`
// the exact previous speeds (NaN = reconstructed as w - acc dt / I, which cannot
// recover an exact 0).
void diff_step_prev(int type, int initial, int accel, int decel, float axle_torque, float inertia, float dt, float acc_left,
                    float acc_right, float dw_prev, float& w_left, float& w_right, DiffState& state, float brake_left = 0,
                    float brake_right = 0, float w_left_prev = std::numeric_limits<float>::quiet_NaN(),
                    float w_right_prev = std::numeric_limits<float>::quiet_NaN());

// Clutch-pack capacity C = max(initial x 9.8, (T >= 0 ? accel : decel) x 0.01 x |T|).
float lsd_clutch_capacity(int initial, int accel, int decel, float axle_torque);

// [physics §7 #21, verified] Driveline class: FF -> 1; AWD -> 1 (system 0),
// 3 (system 3), 2 (other systems); every other layout -> 0.
int driveline_class(int drive_type, int awd_system);

// [physics §4.1] Low-speed hold of a wheel: v_h = v_h' cos d +
// u_h sin d (hub speed across the wheel plane; (u_h, v_h') = hub velocity in the
// body frame), w = wheel speed (m/s). 0 when peak = 0, |w| > 1.38889, |u_h - w| >
// 0.555556 or |v_h| > 1.11111 (strict); else peak sin(0.9 pi v_h) cos(0.36 pi w).
// Acts on the lateral force and yaw only (Vehicle subtracts it from each wheel's
// lateral force).
float low_speed_hold(float peak, float wheel_speed, float u_h, float v_h_body, float cos_d, float sin_d);

// ---- tyre wear, slip intensity, brake heat [physics §8] ----

// [physics §8.1] Wear parameters (cp block) of a race. `max`
// = 0 means wear is off.
struct WearParams {
    float fresh = 0;      // wear of a fresh set (scrub-in start), < 0
    float loss_fresh = 0; // grip loss of a fresh set
    float knee1 = 0, loss1 = 0, knee2 = 0, loss2 = 0;
    float max = 0, loss_max = 0;
    float worn = 0;     // "tires worn" threshold on the grip factor
    float slip_cap = 0; // cap on the slip intensity input
    bool on() const { return max != 0; }
};
// From the event record bytes 0x1BD..0x1C3 (block bytes 0x2C, 0x2D, 0x2E, 0x2F,
// 0x30, 0x33, 0x34; 0x31 forced 4 and 0x32 forced 0). Off (all 0) when
// `enabled` is false (mode / dirt course), the maximum byte is 0, or an
// ordering check fails. The shipped bytes are 21, 30, 9, 2, 6, 15, 5.
WearParams wear_params(const u8 event_bytes[7], bool enabled);
// The one shipped wear curve, enabled.
WearParams shipped_wear_params();

// [physics §8.4] Grip factor of a wheel's wear (clamping wear to max),
// floored at `floor` = min(1, 1.25 / mu) (tyre record mu[11]). `hud_level`
// (optional): -63..127 tyre gauge level (127 = worn out), truncated toward 0.
float wear_grip(const WearParams& p, float& wear, float floor, int* hud_level = nullptr);

// [physics §8.3] Slip intensity ("smoke", skid sound) of a grounded
// wheel on surface index `surface`: kerbs (1, 13) from the wheel's rim speed
// |w r| and load; other surfaces by class from the lateral wheel speed and
// the slip ratio, plus the class term on the wheel speed ramp
// min(1, 0.02 |hub speed|) (the same ramp as the road bump).
float slip_intensity(int surface, float lateral_speed, float slip_ratio, float rim_speed, float fz, float ramp = 0);

// [physics §8.3, second curve set] Skid pitch: the pitch factor of the tyre loops (sound §7.6),
// computed beside the slip intensity from the same inputs. Kerbs (1, 13): a ramp on the rim
// speed |w r| from 0.5 at 10 km/h to 4.3 at 200 km/h. Other surfaces by class:
// sqrt(A(lat)^2 + B(slip)^2) + b, plus the class term on the speed ramp; unlike the slip
// intensity, b is added even when the curves give 0 (tarmac idles at 0.75).
float skid_pitch(int surface, float lateral_speed, float slip_ratio, float rim_speed, float ramp = 0);

// [physics §8.3b, third curve set] Smoke-emission intensity: the
// value race smoke / dust puffs and car dirt read, not the slip intensity above. By
// class (kerbs 1 -> class 0, 13 -> class 2) from the lateral wheel speed and the slip
// speed s = sgn(hub) (hub - rim) in m/s, plus the class ramp term; min(1, m + b), 0 when
// m = 0. Tarmac emits only past 7.6 m/s of wheelspin / 8 m/s of lock-up or 11.6 m/s of
// sideways speed, and then at most about 0.5.
float smoke_emission(int surface, float lateral_speed, float slip_speed, float ramp = 0);

// [physics §8.2] Wear added in one step: ((min(cap, smoke) x rate) x
// max(0, 0.4 Fz / width - 1960)) x dt. rate = compound byte / 100, width = the
// axle's tyre width in metres.
float wear_increment(float smoke, float slip_cap, float rate, float fz, float width, float dt);

// [physics §8.5] Brake-disc heat (visual only): y += dt (T_b pedal |w r|
// / 68600 - (|w r| + 27.78) 0.005 (y + 0.05)), clamped to [0, 1]; T_b = the
// axle's max brake torque x tyre radius, `pedal` = that wheel's brake pressure
// after ABS and the aids.
float brake_heat_step(float y, float brake_torque, float tire_radius, float pedal, float rim_speed, float dt);

// [physics §8.8] A wheel's flap pendulum (visual only: the car models' mud flaps and aerials, morph
// phase 0.5 + 0.5 x): a damped spring, x in [-1, 1], kicked by the body's longitudinal acceleration
// a_x (the forward step's) and at random (two draws of `rng` at most): with the room left to
// swing, as the wheel rolls (hub forward speed u), and on a rough surface (`rough`: the surface's
// tyre record byte is not 0) with the rim speed w, while the wheel is loaded and rolling forward.
// Then v += a dt, x += v dt; at +-1 it stops (v = 0).
void flap_step(float& x, float& v, float a_x, float u, float fz, float rim_speed, bool rough, float dt,
               Vu0Random& rng);


// ---- axle driveline [physics §2.7] ----

// [physics §7 #59] Layout code L, driven axle (0 front, 1 rear), coupling k
// and centre-clutch capacity (N*m) from the drive type (0 FR,
// 1 FF, 2 4WD, 3 MR, 4 RR), AWD system B3, share B4 and strength B5.
struct AwdLayout {
    int layout = 0;
    int driven_axle = 1;
    float k = 0, capacity = 0;
};
AwdLayout awd_layout(int drive_type, int awd_system, int b4, int b5);

// AWD front torque Q (L4..7) before the centre clutch; `full` = the "full
// transfer" sentinel X.
struct AwdQ {
    bool full = false;
    float q = 0;
};
struct AwdIn {
    int layout = 0;
    float k = 0;
    float tg = 0;            // axle input torque T_c x R_g
    float ff = 0, fr = 0;    // summed wheel net torques per axle
    float wf = 0, wr = 0;    // axle speeds (rad/s)
    float handbrake = 0, brake = 0;
    float steer = 0;         // wheel steer angle (rad, any sign)
    float g_over_mu = 0;     // lateral g / mu
    int clutch = 0;          // clutch state 0..3
    bool reverse = false;    // reverse gear
};
// [physics §2.7] L5 coupling gain: u = |w_r - w_f| k f with f = 1 for |g/mu| <= 0.25,
// 0.25 for >= 1, linear between; full when the clutch is open, in reverse while
// locked, or (f = 1) when slipping, braking > 0.75 or T_g and T_g + Fr share a
// sign; else u if u < 1, else full.
AwdQ awd5_gain(const AwdIn& in);
AwdQ awd_front_torque(const AwdIn& in);
// Centre clutch of L4..7: returns the front torque Q_f; `engaged` (in/out) is
// the centre-clutch engaged flag; `engaged` true on return = locked (a common
// axle speed, returns 0). Engaged: locked when Q is full, else while |(Ff - Fr)
// - T_g| <= capacity (else the flag clears). Not locked: Q full -> +capacity if
// w_r >= w_f else -capacity; else Q.
float awd_centre_clutch(const AwdIn& in, const AwdQ& q, float capacity, bool& engaged);

// [physics §2.7] Torque vectoring of an axle with diff type y (6) or t (7): for 2.78
// <= v <= 50 m/s, T = min(|steer| / 15 deg, 1) x initial (x 9.8 for y, x 0.01
// for t); theta = 1.125 m (steer_left > 0) or 0.875 m (< 0), m = mean wheel
// speed; odd (right) wheel below theta: right +0.4375 T, left -0.5625 T; above:
// right -0.5625 T, left +0.4375 T. Adds to t_left / t_right. steer_left = steer
// angle, + = left.
void torque_vectoring(int type, int initial, float steer_left, float speed, float w_left, float w_right, float& t_left,
                      float& t_right);


// [physics §2.7] One axle-speed step. `in` carries the
// layout k, T_g, the summed wheel net torques Ff/Fr, the axle speeds before the
// step and the inputs of the L4..7 front torque; jf/jr = axle inertias, jref =
// reflected engine inertia (J_e |R_g| while locked, else 0); brake_* = the axle
// has brake torque; f_* = the axle's diff is type 'f' (not integrated).
// `engaged` is the L4..6 centre-clutch flag (in/out). Returns the new axle
// speeds and the drive torque per axle (T_g/2 each when the centre clutch is
// locked or for L3).
struct AxleStep {
    float wf = 0, wr = 0;
    float drive_f = 0, drive_r = 0;
    bool common = false; // one common axle speed (centre clutch locked)
};
AxleStep axle_step(const AwdLayout& lay, const AwdIn& in, bool& engaged, float jf, float jr, float jref, bool brake_f,
                   bool brake_r, bool f_front, bool f_rear, float dt);
// The drive torque per axle and the effective inertias for a layout (no
// integration): used by the vehicle for the tyre solve's prediction.
AxleStep axle_drive(const AwdLayout& lay, const AwdIn& in, bool& engaged, float jf, float jr, float jref, float& jd_f,
                    float& jd_r);


// ---- clutch link transitions [physics §2.4] (states 0 open, 1 locked,
// 2 slipping, 3 shift) ----
// Player clutch request (automatic-style pedals): 0 during the
// spawn/stop counter or with the handbrake (> 0.8) on a rear-driven car;
// engaged (state != 0): 1 unless rpm < idle; open: 1 while brake < 0.25.
bool clutch_request(int state, float rpm, float idle_rpm, float brake, float handbrake, bool rear_driven, bool counter);
// Before the step: request 0 opens a locked clutch (1 -> 0; slipping and shift
// states stay); 0 -> 2 when moving (> 10 km/h in
// the gear's direction) on processed throttle > 0.25, else when rpm > launch =
// max(engine +0x43 x 10, idle + 500). `gear` > 0 forward, < 0 reverse, 0 neutral
// (opens).
int clutch_engage(int state, bool request, int gear, float speed, float throttle, float rpm, float launch_byte_rpm,
                  float idle_rpm);
// Gear change [physics §2.4] with the clutch engaged: to or from reverse -> 2,
// forward -> forward -> 3. Gears as above.
int clutch_gear_change(int state, int from, int to);
// After the step: 2 or 3 -> 1 when the engine and wheel-side speeds crossed
// (`crossed`); else 2 -> 0 when brake > 0.75 and (w_e < (idle + 500) 2pi/60
// (clutch_anti_stall) or |v| < 10 km/h); 3 -> 0 when the wheel side <= idle.
int clutch_after(int state, bool crossed, float brake, float engine_w, float idle_rpm, float speed, float wheel_side_w);


// [physics §10.3] Engine-condition torque factor f_E(E) x f_O(O) from the
// running-in/wear counter E and the oil counter O (metres; zero counters give
// exactly 1). E1 = -300000, c1 = 0.03, E2 = 500000, E3 = 600000, c2 = 0.02,
// O1 = -100000, cO = 0.05.
float engine_condition_factor(float E, float O);
// [physics §10.3] Counters after driving d metres: E = min(E +
// d, 600000); O < 0: O + d, jumping to 300000 when it reaches 0; O >= 0: min(O +
// d, 300000).
void engine_condition_step(float& E, float& O, float d);
// [physics §10.3] Odometer step: x = carry + d metres; the odometer (2 m units) gains
// trunc(trunc(x) / 2) and saturates at kOdometerMax (carry 0 then); returns the new carry
// x - 2 * gained (0 when x <= 0). The vehicle skips it while reversing.
constexpr u32 kOdometerMax = 0xEE6B27A6u;
float odometer_step(u32& odometer, float x);
// [physics §10.3] The race HUD's oil lamp: the oil counter at its worn value (O >= 300000).
bool oil_worn(float O);
// [physics §10.4] Dirt brake cut: s = min(1, |steer command| / 15 deg); front brake
// pressure x (1 - s), rear unchanged (only on dirt courses, category 2).
float dirt_front_brake_scale(float steer_command);


// ---- planar motion [physics §4.1] ----

// One wheel's inputs to the planar step, in the spec's body frame (forward, right).
struct PlanarWheel {
    float k = 1, s = 0;        // cos / sin of the wheel steer angle (spec sign, + = left)
    float a = 0;               // velocity-direction angle a_i (rad, physics §3.2)
    float slip_grip = 0, slip_drag = 0;    // tyre force outputs F1, F2 (physics §3.3)
    float drive_part = 0, brake_part = 0;    // drive / brake force parts along the wheel heading (G = drive_part + brake_part)
    float hold = 0;            // low-speed hold force (physics §4.1)
    bool flagged = true;       // the wheel flag byte != 0 (airborne, braked, coupled or driven)
    float ia_r2 = 0;           // the axle's Ia / r^2: effective mass of a free wheel
};
// Forward step: F_x = sum k drive_part + rolling resistance; m^-1 = 1/(M_eff + mass)
// with M_eff = sum of ia_r2 over unflagged wheels; a0 = (F_x + ext + drag) m^-1,
// u1 = u + a0 dt; F_t = sum (k brake_part - sin a slip_grip - cos a slip_drag), a_t = F_t m^-1,
// u2 = u1 + a_t dt; u1 u2 > 0: u = u2, a_x = a0 + a_t; else u = 0, a_x = a0 - u1/dt.
struct ForwardStep {
    float u = 0, ax = 0;
};
ForwardStep forward_step(const PlanarWheel w[4], float u, float mass, float rolling, float ext, float drag, float dt);
// Lateral force and yaw (not airborne): Fa = -cos a slip_grip + sin a slip_drag - k hold - s G,
// Fb = -sin a slip_grip - cos a slip_drag - s hold + k G; levers X = (-tf, +tf, -tr, +tr), Y = (a, a, -b,
// -b); yaw torque N = sum (X Fb - Y Fa); yaw acceleration = N / yaw_param; a_y = (sum Fa -
// ext) / mass; g_over_mu = 4 a_y / sum grip_i (grip_i = record base grip x mu[s_i]).
struct LateralYaw {
    float ay = 0, yaw_acc = 0, g_over_mu = 0;
};
LateralYaw lateral_yaw(const PlanarWheel w[4], const float X[4], const float Y[4], float mass, float yaw_param, float ext_lat,
                       const float grip[4]);
// Stop clamp hold: frozen race, or |u| < 1 km/h (strict) with every fade weight 0
// and some wheel braking.
bool stop_hold(float u, const float fade[4], bool any_brake, bool frozen);


// ---- per-wheel force outputs, exact [physics §3.8] ----

// [physics §3.8] C(x) (longitudinal force fraction) and D(x) (friction-circle radius) at slip ratio x.
struct TyreCurves {
    float c = 0, d = 0;
};
TyreCurves tyre_curves(const TireSpec& t, float slip_ratio);
// [physics §3.8] F = C Fp, negated if sg < 0; drive_part = F (st > 0) / brake_part = F (st <= 0): the force on
// the car (not gained); drive_gained / brake_gained = g F likewise: the wheel reaction for the spin dynamics.
struct TyreLong {
    float drive_part = 0, brake_part = 0, drive_gained = 0, brake_gained = 0;
};
TyreLong tyre_long_force(float c, float fp, int st, int sg, float g);
// [physics §3.8] Slip-angle force: a = steer (spec sign, + left) + pi if the hub forward speed < 0;
// beta = wrap(a) - wrap(phi) re-wrapped into [-pi, pi); folded to v with sign sg_a; u1 = sg_a
// A(v) k, u2 = B(v) k, scaled to the circle radius d when longer; slip_grip = u1 Fp (lateral grip),
// slip_drag = u2 Fp (drag along the velocity, >= 0). k = fade weight (0: no force). Force on the car
// in the body frame (forward, right) = -slip_drag (cos phi, -sin phi) - slip_grip (sin phi, cos phi).
struct TyreLat {
    float slip_grip = 0, slip_drag = 0, beta = 0;
};
TyreLat tyre_lateral(const TireSpec& t, float steer, bool hub_backwards, float phi, float k, float fp, float d);
// Slip-ratio state bytes [physics §3.4]: st = +1 in the wheelspin branch (wheel faster than the
// ground), -1 braking or a near-stopped wheel.
int slip_state(float hub_along_heading, float wheel_speed);
// The slip sign byte sg [physics §3.4]: sign(w - x); in the dead band 0 when equal; outside it an
// exact tie gives sign(w).
int slip_sign(float hub_along_heading, float wheel_speed);
// [physics §3.4] Slip gain g = 0.5 + 0.5 min(|u| x 3.6 / 100, 1) (100/3.6 m/s, shipped data).
float slip_gain(float hub_forward_speed);

// [physics §4.6] One wheel's brake pressure after ASCC, from
// the pedal pressure (no accumulation: the wheel pressure is reset to the pedal every tick). Without brake
// (pedal exactly 0): A adds (o - thr_A) gain_A on the outside wheels; C, with the throttle exactly
// 0, adds (u - thr_C) gain_C on the inside wheels. With brake: D scales the inside wheels by
// clamp(1 - (o - 0.1) gain_D), E the outside ones by clamp(1 - (u - 0.1) gain_E). Channel and axle
// enable from the flag bits; the result is clamped to [0, 1].
// cap_scale (optional): the factor the D / E channels also apply to the wheel's ABS target slip (physics §4.6).
float ascc_pressure(const AidsSpec& a, int axle, bool outside, float pedal, float understeer, float oversteer, float brake,
                    float throttle, float* cap_scale = nullptr);
// [presentation §D3] The ASM lamp is lit when the car has any ASCC gain.
bool asm_lamp_lit(const AidsSpec& a);
} // namespace gt3r::sim::model
