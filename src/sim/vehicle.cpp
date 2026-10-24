#include "sim/vehicle.h"

#include "sim/models.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

// State model and step structure follow the original: one
// explicit (semi-implicit) Euler step of 1/60 s per tick, planar motion
// (horizontal velocity + heading/yaw rate) integrated from the tyre forces
// and a separate heave / pitch / roll attitude driven by the suspension,
// orientation rebuilt every step from three Euler angles (R = Rz Ry Rx in the
// game's Z-up frame, model::euler_axes). The sub-models are the free
// functions of models.h; the few still conventional are tagged [placeholder].
//
// Vehicle::tick is a sequence of phase functions (members of Vehicle::Impl, defined below it in tick order);
// what one phase hands to the next are the per-tick structs below. Two body models share the tick: the spec
// body (both axles carry the spec suspension block: every disc car) and a placeholder rigid body for
// hand-built specs without the block. Where they differ, the placeholder's steps are the placeholder_*
// functions.

namespace gt3r::sim {

namespace {

struct D3 {
    double x = 0, y = 0, z = 0;
    D3() = default;
    D3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    explicit D3(const Vec3& v) : x(v.x), y(v.y), z(v.z) {}
    D3 operator+(const D3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    D3 operator-(const D3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    D3 operator*(double s) const { return {x * s, y * s, z * s}; }
    D3& operator+=(const D3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    double dot(const D3& o) const { return x * o.x + y * o.y + z * o.z; }
    D3 cross(const D3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    double length() const { return std::sqrt(dot(*this)); }
    D3 normalized() const {
        double l = length();
        return l > 1e-12 ? *this * (1.0 / l) : D3{0, 1, 0};
    }
    Vec3 f() const { return {float(x), float(y), float(z)}; }
    bool finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};

double clampd(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }

// World-frame body axes (car +X = lateral/right, +Y = up, -Z = forward) from
// the Euler angles: ax = rotation about the lateral axis (nose up > 0), ay =
// about the forward axis (right side down > 0), heading about up (left > 0).
struct Axes {
    D3 fwd, lat, up;
    D3 to_world(const D3& car) const { return lat * car.x + up * car.y - fwd * car.z; }
    D3 to_car(const D3& w) const { return {w.dot(lat), w.dot(up), -w.dot(fwd)}; }
};

Axes make_axes(double ax, double ay, double heading) {
    float f[3], l[3], u[3];
    model::euler_axes(float(ax), float(ay), float(heading), f, l, u);
    Axes a;
    a.fwd = D3(model::phys_to_world(f));
    a.lat = D3(model::phys_to_world(l));
    a.up = D3(model::phys_to_world(u));
    return a;
}

Quat axes_to_quat(const Axes& a) {
    // Rotation matrix columns: car X -> lat, Y -> up, Z -> -fwd.
    double m00 = a.lat.x, m10 = a.lat.y, m20 = a.lat.z;
    double m01 = a.up.x, m11 = a.up.y, m21 = a.up.z;
    double m02 = -a.fwd.x, m12 = -a.fwd.y, m22 = -a.fwd.z;
    double tr = m00 + m11 + m22, w, x, y, z;
    if (tr > 0) {
        double s = std::sqrt(tr + 1.0) * 2;
        w = 0.25 * s; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        double s = std::sqrt(1.0 + m00 - m11 - m22) * 2;
        w = (m21 - m12) / s; x = 0.25 * s; y = (m01 + m10) / s; z = (m02 + m20) / s;
    } else if (m11 > m22) {
        double s = std::sqrt(1.0 + m11 - m00 - m22) * 2;
        w = (m02 - m20) / s; x = (m01 + m10) / s; y = 0.25 * s; z = (m12 + m21) / s;
    } else {
        double s = std::sqrt(1.0 + m22 - m00 - m11) * 2;
        w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = 0.25 * s;
    }
    return {float(w), float(x), float(y), float(z)};
}

D3 rotate(const Quat& q, const D3& v) {
    D3 u{q.x, q.y, q.z};
    D3 t = u.cross(v) * 2.0;
    return v + t * q.w + u.cross(t);
}

constexpr double kMaxAttitude = 0.6; // rad, clamp on pitch/roll [physics §1.3: "clamped to small values"; value placeholder]

// Euler angles of an orientation (world axes -> physics frame -> angles), pitch and roll clamped.
struct Euler {
    double ax = 0, ay = 0, heading = 0;
};
Euler euler_angles(const Quat& q) {
    D3 fw = rotate(q, {0, 0, -1}), lt = rotate(q, {1, 0, 0}), upw = rotate(q, {0, 1, 0});
    auto to_phys = [](const D3& w) { return D3{w.x, -w.z, w.y}; };
    D3 f = to_phys(fw), l = to_phys(lt), u = to_phys(upw);
    Euler e;
    e.ay = clampd(-std::asin(clampd(l.z, -1, 1)), -kMaxAttitude, kMaxAttitude);
    e.heading = std::atan2(l.y, l.x);
    e.ax = clampd(std::atan2(f.z, u.z), -kMaxAttitude, kMaxAttitude);
    return e;
}

enum class ClutchState { Open, Locked, Slipping, Shift }; // [physics §2.4] states 0, 1, 2, 3

// Half of an axle's share of the sprung mass, kg [physics §4.2]: share x (total - 2 (front + rear unsprung)) / 2.
double sprung_corner(const CarSpec& s, int wheel) {
    const double ms = s.mass - 2.0 * (double(s.axle[0].susp.unsprung) + s.axle[1].susp.unsprung);
    return 0.5 * ms * (wheel < 2 ? s.front_weight : 1 - s.front_weight);
}

bool driven(Drive d, int wheel) {
    switch (d) {
    case Drive::FF: return wheel < 2;
    case Drive::AWD: return true;
    default: return wheel >= 2;
    }
}

// Mean tyre radius of the driven wheels.
double driven_radius(const CarSpec& s) {
    double r = 0;
    int n = 0;
    for (int i = 0; i < 4; ++i)
        if (driven(s.drivetrain.drive, i)) {
            r += s.axle[size_t(i / 2)].tire_radius;
            ++n;
        }
    r /= std::max(n, 1);
    return r;
}

// [physics §4.1-4.3] Both axles carry the spec suspension block (every disc car): the spec planar step, attitude
// and the phase-12 order. Hand-built specs without it take the placeholder rigid body.
bool has_spec_body(const CarSpec& s) { return s.axle[0].susp.valid && s.axle[1].susp.valid; }

// Hub speed along the wheel heading (our steer + = right): cos d uh - sin d lh with d = -steer [physics §3.4, physics §10.2].
double along_heading(double uh, double lh, double steer) { return uh * std::cos(steer) + lh * std::sin(steer); }

// Wheel speed w moved towards 0 by dw, stopping at 0 (a brake).
double coulomb(double w, double dw) { return std::fabs(w) <= dw ? 0.0 : w - std::copysign(dw, w); }

// [physics §2.7] the engine-side axle speed of a driveline layout.
double layout_axle_speed(const model::AwdLayout& lay, double wf, double wr) {
    if (lay.layout == 3) return 0.5 * (wf + wr);
    if (lay.layout == 7) return wr + lay.k * (wf - wr);
    return lay.driven_axle == 0 ? wf : wr;
}

// Hull contact with the ground (bottoming out) [placeholder]. The original has none (physics §11): the ground is
// touched only through the four wheel probes, the bump stops and the physics §4.3 nudge. Off for the spec body;
// GT3R_HULL_CONTACT=1 turns it back on for comparison.
bool hull_contact_on() {
    static const bool on = [] {
        const char* e = std::getenv("GT3R_HULL_CONTACT");
        return e && e[0] == '1';
    }();
    return on;
}

// --- per-tick values: what the phases of Vehicle::tick read and hand on ---------------------

// Who drives the car (Vehicle's driver flags; fixed for the tick).
struct Driver {
    bool cpu = false;             // set_cpu_driver
    int kind = 0;                 // set_driver_kind
    bool clutch_hold = false;     // set_clutch_hold
    bool race_hold = false;       // set_race_hold
    bool reverse_request = false; // set_reverse_request
    Vehicle::ScriptedAids aids;   // set_scripted_aids
    // [physics §2.6, physics §4.4] CPU cars and a non-zero driver kind take the scripted path (automatic, the AI clutch rule).
    bool scripted() const { return cpu || kind != 0; }
};

// What every phase of a tick reads.
struct TickContext {
    const CarSpec& s;
    ControlInputs in;         // the inputs as the tick reads them (condition_inputs)
    const ControlInputs& raw; // as given: the shift logic reads the accelerator itself
    double h;                 // tick length dt_car [physics §1.1]
    Driver drv;
    bool spec_body;           // has_spec_body(s)
};

// The body at the start of the tick.
struct Frame {
    Axes body; // from the Euler angles
    // [physics §10.1] the ground frame: with the spec suspension the one the last tick's phase 12 left (the body
    // axes until there is one, and for the placeholder body).
    Axes ground;
    D3 fwd_h; // horizontal forward direction
    // [physics §10.1] u = ground-frame forward tangent . world velocity (the vertical velocity enters along the
    // ground slope, not along the body pitch): what aero, steering, clutch, aids, shifts and the engine counters
    // read.
    double v_fwd = 0;
    D3 omega_w; // angular velocity in the world (Euler rates; small-angle composition)
};

// Ground query at the hubs and the suspension step [physics §1.3, physics §4.2, physics §4.3].
struct Ground {
    std::array<SurfaceHit, 4> hits;
    std::array<D3, 4> hub_w, normal; // query points, ground normals
    std::array<int, 4> sidx{};       // surface index [physics §3.6]
    model::GroundFrame gf;           // ground height, pitch and roll from the four hub ground heights
    Axes axes;                       // the ground frame (gf pitch and roll, body heading)
    std::array<double, 4> body_f{}, damper_f{}; // per-wheel body force (spring + damper + anti-roll), its damper part
};

// Wheel speeds, slips and peak forces of the tyre phase [physics §3.1-3.4, physics §10.1].
struct WheelMotion {
    std::array<double, 4> radius{};
    std::array<D3, 4> vcs;                 // hub velocity (world)
    std::array<double, 4> vxs{}, vys{};    // hub speed along / across the wheel heading
    std::array<double, 4> vhub{}, fades{}; // |hub velocity|, slip-angle fade weight (physics §3.2)
    std::array<double, 4> peaks{};         // peak force Fp (physics §3.1)
    std::array<D3, 4> wf, wr;              // [placeholder] wheel forward / right on the ground plane
};

// Engine, clutch and turbo [physics §2.2-2.4].
struct Powertrain {
    double G = 0;            // total ratio of the gear at the start of the tick
    model::AwdLayout lay;    // [physics §2.7] driveline layout
    double J = 0;            // engine inertia
    double w_cap = 0;        // locked engine speed cap: rev limit + 10 rpm
    model::EngineOut eo;
    double shaft_torque = 0; // T_g = T_c R_g (no driveline efficiency)
    double slip_dw = 0;      // engine - wheel-side speed of a slipping clutch
};

// [physics §2.7] axle inertias J = shaft x final + 2 spin per axle, and the reflected engine inertia
// J_e |R_g| (only while locked).
struct AxleInertia {
    std::array<double, 2> ia{}, j_ax{}; // wheel spin inertia, axle inertia
    double j_ref = 0;
};

// [placeholder] per-wheel drive torque and effective inertia predicted for the velocity-limited tyre force.
struct WheelDrive {
    std::array<double, 4> tdrive{}, iw{};
};

// Brake torque and pressure after ASCC and ABS [physics §4.5, physics §4.6].
struct Brakes {
    std::array<double, 4> torque{}, pressure{};
    bool abs_any = false;
};

// The tyre phase's output [physics §3.1-3.3, physics §3.8].
struct TyreForces {
    std::array<double, 4> fx{}, fy{}; // in the wheel frame (telemetry; the placeholder body is pushed by them)
    std::array<double, 4> reaction{}; // the longitudinal force the wheel spin reacts to (spec: the gained copy)
    // The spec planar step's inputs [physics §3.8]: drive_part + brake_part, the low-speed hold, the lateral outputs
    // (grip and drag along the velocity direction phi) and the two longitudinal parts.
    std::array<double, 4> long_part{}, hold{}, slip_grip{}, slip_drag{}, drive_part{}, brake_part{}, phi{};
};

// The driveline step's output [physics §2.7].
struct DriveStep {
    std::array<double, 2> drive_ax{}; // drive torque per axle
    double w_side_after = 0;          // engine-side axle speed
};

// [physics §4.1] the spec planar step's accelerations: forward and lateral (0 with all wheels airborne).
struct PlanarAccel {
    double ax = 0, ay = 0;
};

// Forces on the body about the CoG (world). The placeholder body sums gravity, suspension, aero, tyre and hull
// forces here; the spec body moves by its planar and attitude steps instead (only gravity and the
// GT3R_HULL_CONTACT comparison contact end up here).
struct BodyForces {
    D3 force, torque;
    void apply(const D3& f, const D3& at, const D3& cog) {
        force += f;
        torque += (at - cog).cross(f);
    }
};

// The inputs as the tick reads them: the automatic for scripted drivers, the pad pedal curve, the AT reverse
// pedal swap and the EV one-pedal hold.
ControlInputs condition_inputs(const ControlInputs& raw, const Driver& drv, int gear, bool electric, float speed_kmh) {
    ControlInputs in = raw;
    // [physics §2.6] the shift dispatcher uses mode 0 (automatic) for CPU cars and for a
    // non-zero driver kind (run-up, pit lane, stop driver).
    if (drv.scripted()) in.automatic = true;
    if (!drv.cpu && !std::isfinite(in.steer_angle)) { // pad path [physics §4.4]: x + 0.1 (1 - x) once pressed
        in.throttle = model::pedal_shape(std::clamp(in.throttle, 0.0f, 1.0f));
        in.brake = model::pedal_shape(std::clamp(in.brake, 0.0f, 1.0f));
    }
    // Automatic in reverse: the brake pedal drives and the accelerator brakes, since
    // pressing the accelerator below 1 km/h selects first gear [physics §2.6;
    // pedal swap is an interpretation of GT3's AT reverse].
    // Player AT reverse: the brake pedal drives [interpretation]; CPU drivers request reverse
    // explicitly and drive it with the accelerator.
    if (in.automatic && gear < 0 && !drv.cpu) std::swap(in.throttle, in.brake);
    // Mod EVs, one-pedal driving: regen slows the car (models.cpp engine_torque); near walking pace
    // the friction brakes blend in and hold it at a standstill while neither pedal is pressed.
    if (electric && !drv.cpu && drv.kind == 0 && in.throttle < 0.02f) {
        float v = std::fabs(speed_kmh);
        if (v < 5.0f) in.brake = std::max(in.brake, 0.35f * (1.0f - v / 5.0f) + 0.05f);
    }
    return in;
}

AxleInertia axle_inertia(const CarSpec& s, bool locked, double J, double G) {
    AxleInertia r;
    for (int a = 0; a < 2; ++a) {
        const AxleSpec& axs = s.axle[size_t(a)];
        r.ia[size_t(a)] = axs.spin_inertia > 0 ? axs.spin_inertia : axs.wheel_inertia;
        r.j_ax[size_t(a)] = axs.shaft_inertia * s.gearbox.final_drive + 2 * r.ia[size_t(a)];
    }
    r.j_ref = locked ? J * std::fabs(G) : 0.0;
    return r;
}

} // namespace

struct Vehicle::Impl {
    // Planar + attitude state [physics §1.3].
    D3 pos;               // CG, world
    D3 vel;               // world (x/z planar, y heave rate)
    double heading = 0, ax = 0, ay = 0;          // Euler angles
    double yaw_rate = 0, ax_rate = 0, ay_rate = 0;
    D3 cg_local;          // CG in the car frame
    double i_yaw = 2500, i_pitch = 2500, i_roll = 1250;

    std::array<D3, 4> hub{};        // hub (wheel centre) at static ride height, car frame
    std::array<double, 4> x{}, x_prev{}, omega{}, spin{}, steer{};
    std::array<double, 4> kappa{}, alpha{}, load{};
    std::array<bool, 4> grounded{};
    std::array<u32, 4> surface{};
    std::array<D3, 4> contact{};
    // [physics §1.2 phase 12] spec body: the suspension/ground step runs at the end of the tick
    // (after the collision step moved the body); the next tick's tyre phase reads what it left.
    bool late_init = false;
    std::array<D3, 4> late_normal{};
    std::array<SurfaceHit, 4> late_hits{};
    model::GroundFrame late_gf{};
    std::array<int, 4> axle{0, 0, 1, 1};
    std::array<double, 4> rest_x{};
    std::array<double, 4> xs{}; // spec static suspension coordinate x_s (negative)

    double omega_e = 0;
    bool limiter = false;
    int gear = 1;
    ClutchState clutch = ClutchState::Open;
    int shift_lockout = 0;
    double steer_avg = 0, steer_rate = 0;
    bool settle = false; // place on the ground at the first tick after reset
    double tcs_scale = 1;
    double slip = 0, slip_target = 0; // slipstream [physics §5.2]
    double lat_g = 0; // lateral acceleration of the last tick, g (TCS / ASCC input)
    std::array<model::DiffState, 2> diff{}; // spec driveline differential state per axle
    bool cpu_clutch = false;                // CPU binary clutch request state
    std::array<double, 4> ground_prev{};    // last ground height under each hub
    int spawn = 0;                          // spawn-hold counter [physics §4.3]
    double spawn_heading = 0;               // grid heading the hold eases to
    // tyre wear [physics §8]
    model::WearParams wear_p;
    std::array<float, 4> wear{}, wear_grip{1, 1, 1, 1}, smoke{}, smoke_emit{}, skid_pitch{1, 1, 1, 1}, brake_heat{};
    std::array<float, 4> flap_x{}, flap_v{}; // [physics §8.8] flap pendulums (visual only)
    float long_accel = 0;                    // [physics §4.1] the forward step's a_x (airborne too)
    std::array<int, 4> air_steps{}, wear_level{};
    bool pit_surface = false;
    // axle driveline [physics §2.7]
    bool centre_engaged = false;           // L4..6 centre-clutch engaged flag
    float cond_e = 0, cond_o = 0;          // engine condition counters [physics §10.3]
    u32 odometer = 0;                      // 2 m units [physics §10.3]
    float odometer_carry = 0;              // metres not yet counted (< 2)
    bool dirt = false;                     // dirt course [physics §10.4]
    int pad_o3 = 0;                        // pad state O[3]: 1 digital, 2 analog steering [physics §4.4]
    std::array<double, 2> axle_torque{};   // last tick's summed wheel net torques (front, rear)
    double g_over_mu = 0;                  // lateral g / mu (last tick)
    double yaw_acc = 0;                    // last tick's yaw acceleration (ASCC understeer input)
    std::array<double, 4> slip_g{1, 1, 1, 1}; // slip gain from the wheel's own hub forward speed (physics §3.4)
    double rear_beta_prev = 0;             // largest rear |beta| of the last tyre phase (ASCC oversteer input)
    double long_force_sum = 0;             // sum k (drive_part + brake_part) of the last tyre phase
    std::array<bool, 4> free_roll{};       // physics §10.2 free-rolling wheels of the last tick


    model::TurboState turbo;
    std::vector<D3> hull, wall_pts;
    std::array<collide::V2, 8> octagon{};
    double oct_radius = 0;
    std::vector<collide::Segment> segs; // scratch
    collide::WallContacts wall_contacts; // [physics §5.3] points held on a wall
    double wall_radius = 0.3;

    void fit_fresh_tires();
    collide::Body collision_body(double mass) const;
    void set_collision_body(const collide::Body& b);

    // --- the phases of Vehicle::tick, in tick order (defined below it) ---
    void settle_on_ground(const CarSpec& s, const TrackQuery& track);
    Axes ground_frame(bool spec_body) const;
    Frame frame(bool spec_body) const;
    void set_gear(int to);
    void steering(const TickContext& t, const Frame& fr, int& input_kind);
    void wheel_steer_angles(const CarSpec& s);
    // ground and suspension
    D3 hub_point(const CarSpec& s, size_t i, const Axes& A) const;
    void hub_velocity(const CarSpec& s, size_t i, const Axes& Ag, double& uh, double& lh) const;
    const TireSpec& tire_for(const CarSpec& s, int wheel, int surf) const;
    Ground start_ground(const TickContext& t, const TrackQuery& track, const Frame& fr);
    Ground suspension_step(const TickContext& t, const TrackQuery& track, const Axes& A, const D3& omega_w);
    void spec_wheel_contact(const AxleSpec& a, size_t i, double ground, double terrain, Ground& g);
    void placeholder_wheel_contact(const AxleSpec& a, size_t i, double ground, double terrain, const Axes& A,
                                   Ground& g);
    void spec_wheel_suspension(const AxleSpec& a, size_t i, double h, const Axes& A, Ground& g);
    void placeholder_wheel_suspension(const AxleSpec& a, size_t i, double h, Ground& g);
    void front_block_load_transfer(const CarSpec& s, Ground& g) const;
    void placeholder_suspension_forces(const Ground& g, const Axes& A, BodyForces& body) const;
    // aero, tyre slips and peak forces
    model::AeroOut aero(const TickContext& t, const Frame& fr, VehicleTelemetry& tel);
    void placeholder_aero_forces(const CarSpec& s, const Axes& A, const model::AeroOut& aero, BodyForces& body) const;
    WheelMotion wheel_motion(const TickContext& t, const Frame& fr, const Ground& g,
                             const std::array<double, 4>& tyre_load, VehicleTelemetry& tel);
    void spec_wheel_velocity(const CarSpec& s, size_t i, const Axes& Ag, WheelMotion& wm) const;
    void placeholder_wheel_velocity(size_t i, const Axes& Ag, WheelMotion& wm);
    // engine and clutch, brakes and aids
    bool scripted_aids(const Driver& drv) const;
    void traction_control(const TickContext& t, const Ground& g);
    void clutch_link(const TickContext& t, double G, double w_in, double v_fwd, double throttle);
    Powertrain engine_and_clutch(const TickContext& t, const Frame& fr, const Ground& g, VehicleTelemetry& tel);
    model::AwdIn awd_input(const TickContext& t, const Powertrain& p) const;
    WheelDrive placeholder_drive_prediction(const TickContext& t, const Powertrain& p,
                                            const AxleInertia& inertia) const;
    void steer_indicators(const TickContext& t, const Frame& fr, const Ground& g, double& ind_u, double& ind_o) const;
    Brakes brakes(const TickContext& t, const Frame& fr, const Ground& g, const WheelMotion& wm,
                  VehicleTelemetry& tel) const;
    // tyre forces and driveline
    TyreForces tyre_forces(const TickContext& t, const Ground& g, const WheelMotion& wm, VehicleTelemetry& tel);
    TyreForces placeholder_tyre_forces(const TickContext& t, const Frame& fr, const Ground& g, const WheelMotion& wm,
                                       const Powertrain& p, const AxleInertia& inertia, const Brakes& br,
                                       BodyForces& body, VehicleTelemetry& tel) const;
    DriveStep wheel_spin(const TickContext& t, const Frame& fr, const WheelMotion& wm, const TyreForces& tyre,
                         const Brakes& br, const Powertrain& p, const AxleInertia& inertia, VehicleTelemetry& tel);
    void flag_free_wheels(const model::AwdLayout& lay, double drive_f, const std::array<double, 4>& tyre_load,
                          const std::array<double, 4>& tbrake);
    void tyre_wear_and_heat(const TickContext& t, const Ground& g, const WheelMotion& wm, const Brakes& br,
                            VehicleTelemetry& tel);
    void clutch_after_driveline(const TickContext& t, const Frame& fr, const Powertrain& p, double w_side_after,
                                VehicleTelemetry& tel);
    void placeholder_hull_contact(const TickContext& t, const TrackQuery& track, const Frame& fr,
                                  BodyForces& body) const;
    // integration
    PlanarAccel planar_step(const TickContext& t, const Frame& fr, const Ground& g, const WheelMotion& wm,
                            const TyreForces& tyre, const Brakes& br, const AxleInertia& inertia,
                            const model::AeroOut& aero, const std::array<double, 4>& tyre_load, BodyForces& body);
    Ground advance_and_query_ground(const TickContext& t, const TrackQuery& track);
    void load_transfer(const CarSpec& s, double a_y, Ground& late) const;
    void attitude_step(const TickContext& t, const Frame& fr, const model::AeroOut& aero, const TyreForces& tyre,
                       const Ground& late, bool moved);
    void placeholder_integrate(const TickContext& t, const Frame& fr, const Ground& g, const WheelMotion& wm,
                               const Brakes& br, const BodyForces& body);
    // end of the tick
    void walls(const TickContext& t, const WorldQuery& world, const D3& start_pos, double start_heading,
               VehicleTelemetry& tel);
    void record_acceleration(const TickContext& t, const Frame& fr, const BodyForces& body, const PlanarAccel& planar,
                             VehicleTelemetry& tel);
    void lock_free_wheels(const CarSpec& s);
    void select_gear(const TickContext& t, double v_fwd_sel);
    bool state_finite() const;
    void end_telemetry(bool spec_body, VehicleTelemetry& tel) const;
};

Vehicle::Vehicle(const CarSpec& spec) : spec_(spec), impl_(std::make_unique<Impl>()) {
    spec_.finalize();
    Impl& m = *impl_;
    const CarSpec& s = spec_;
    m.cg_local = {0, s.cg_height, s.wheelbase * (0.5 - s.front_weight)};
    // [physics §4.1/4.3/6.4] yaw: r += N / I_z dt with I_z = chassis inertia byte x 100
    // (R34 9500); pitch and roll use I/(2 pi) with Y = I_z and Z = I_z / 2.
    const double L = s.length, W = s.width;
    m.i_yaw = s.yaw_inertia;
    m.i_pitch = s.yaw_inertia / model::kTwoPi;
    m.i_roll = s.yaw_inertia * 0.5 / model::kTwoPi;
    setup_.wheelbase = s.wheelbase;
    setup_.steer_lock = s.steering.max_angle;
    setup_.length = s.length;
    setup_.width = s.width;
    setup_.height = s.height;
    setup_.forward_gears = int(s.gearbox.ratios.size());
    setup_.idle_rpm = s.engine.idle_rpm;
    setup_.rev_limit_rpm = s.engine.rev_limit_rpm;
    setup_.max_rpm = std::ceil((s.engine.rev_limit_rpm + 800) / 1000.0f) * 1000.0f;
    setup_.upshift_rpm = model::upshift_rpm(s.engine, s.gearbox, 1);
    setup_.has_turbo = s.engine.turbo.on;
    setup_.max_boost = s.engine.turbo.on ? s.engine.turbo.cap - 1 : 0;
    for (int i = 0; i < 4; ++i) {
        const AxleSpec& a = s.axle[size_t(m.axle[size_t(i)])];
        double side = (i % 2 == 0) ? -1 : 1;
        double z = i < 2 ? -s.wheelbase * 0.5 : s.wheelbase * 0.5;
        double corner = s.mass * (i < 2 ? s.front_weight : 1 - s.front_weight) * 0.5;
        m.rest_x[size_t(i)] = corner * model::kGravity / a.spring_rate;
        // x_s = -(half the axle's sprung weight) / k [physics §4.2]: the weight split applies to the SPRUNG
        // mass (total - 2 (front + rear unsprung)); each wheel adds its own unsprung weight (preload) on top
        // [verified exact on every licence car].
        if (a.susp.valid)
            m.xs[size_t(i)] = model::static_position(a.susp, s.mass, i < 2 ? s.front_weight : 1 - s.front_weight, s.axle[0].susp.unsprung,
                                                     s.axle[1].susp.unsprung);
        m.hub[size_t(i)] = {side * a.track * 0.5, a.tire_radius, z};
        setup_.wheels[size_t(i)].rest_center = m.hub[size_t(i)].f();
        // the publish() convention: spec suspension -x, placeholder +x (compression)
        setup_.wheels[size_t(i)].static_suspension = float(a.susp.valid ? -m.xs[size_t(i)] : m.rest_x[size_t(i)]);
        setup_.wheels[size_t(i)].ride_height = a.susp.valid ? a.susp.ride_height : 0.0f;
        setup_.wheels[size_t(i)].radius = a.tire_radius;
        setup_.wheels[size_t(i)].width = a.tire_width;
    }
    double H = s.height;
    double hx = W * 0.47, hz = L * 0.47;
    for (double y : {0.15, H * 0.95})
        for (double sx : {-1.0, 1.0})
            for (double sz : {-1.0, 1.0}) m.hull.push_back({sx * hx, y, sz * hz});
    m.wall_radius = std::min(0.35, W * 0.2);
    // [physics §5.3] the octagon is CoG-relative: front = spec front extent - L (wf - 0.5),
    // rear = spec rear extent + L (wf - 0.5) (the spec extents are measured from the wheelbase midpoint).
    const double cog_fwd = s.wheelbase * (s.front_weight - 0.5);
    m.octagon = collide::octagon(float(s.front_extent - cog_fwd), float(s.rear_extent + cog_fwd), W);
    setup_.cog_front_extent = float(s.front_extent - cog_fwd);
    setup_.cog_rear_extent = float(s.rear_extent + cog_fwd);
    setup_.cog_offset = m.cg_local.f();
    m.oct_radius = collide::bounding_radius(m.octagon);
    double r = m.wall_radius, wy = std::min(0.55, H * 0.4);
    for (double sx : {-1.0, 1.0})
        for (double sz : {-1.0, 0.0, 1.0}) m.wall_pts.push_back({sx * (W * 0.5 - r), wy, sz * (L * 0.5 - r)});
    reset({}, {});
}

Vehicle::~Vehicle() = default;

WorldQuery::Grip WorldQuery::surface_grip(u32 surface) const {
    int s = model::surface_index(surface);
    return {model::surface_mu(s), model::surface_rolling(s)};
}

collide::Body Vehicle::Impl::collision_body(double mass) const {
    collide::Body b;
    b.pos = {pos.x, pos.z};
    b.heading = heading;
    b.vel = {vel.x, vel.z};
    b.vy = vel.y;
    b.yaw_rate = yaw_rate;
    b.inv_mass = 1.0 / mass;
    b.inv_inertia = 1.0 / (10.0 * i_yaw); // [physics §5.3]
    b.local = octagon;
    b.radius = oct_radius;
    return b;
}

void Vehicle::Impl::set_collision_body(const collide::Body& b) {
    pos.x = b.pos.x;
    pos.z = b.pos.z;
    heading = b.heading;
    vel.x = b.vel.x;
    vel.z = b.vel.z;
    vel.y = b.vy;
    yaw_rate = b.yaw_rate;
}

collide::Body Vehicle::collision_body() const { return impl_->collision_body(spec_.mass); }

void Vehicle::set_collision_body(const collide::Body& b) { impl_->set_collision_body(b); }

void Vehicle::apply_collision_body(const collide::Body& b) {
    set_collision_body(b);
    publish();
}

void Vehicle::set_remote_pose(const VehicleState& st, const collide::Body& b) {
    set_collision_body(b);
    state_ = st;
}

std::unique_ptr<Vehicle> Vehicle::create(const CarSpec& spec) { return std::unique_ptr<Vehicle>(new Vehicle(spec)); }

std::unique_ptr<Vehicle> Vehicle::create(const data::CarDatabase& db, const std::string& label, std::string* error) {
    auto spec = build_car_spec(db, label, error);
    if (!spec) return nullptr;
    return create(*spec);
}

void Vehicle::set_slipstream(float factor) { impl_->slip_target = std::clamp(factor, 0.0f, 1.0f); }

bool Vehicle::spawn_hold_active() const { return impl_->spawn > 0; }

void Vehicle::set_tire_wear(const model::WearParams& params) {
    impl_->wear_p = params;
    fit_fresh_tires();
}

const model::WearParams& Vehicle::tire_wear_params() const { return impl_->wear_p; }

void Vehicle::fit_fresh_tires() { impl_->fit_fresh_tires(); }

void Vehicle::Impl::fit_fresh_tires() {
    for (size_t i = 0; i < 4; ++i) {
        wear[i] = wear_p.on() ? wear_p.fresh : 0;
        wear_grip[i] = wear_p.on() ? 1 - wear_p.loss_fresh : 1;
        wear_level[i] = wear_p.on() ? -63 : 0;
        brake_heat[i] = 0;
    }
}

void Vehicle::set_engine_condition(const EngineCondition& c) {
    impl_->cond_e = c.wear_m;
    impl_->cond_o = c.oil_m;
    impl_->odometer = c.odometer;
    impl_->odometer_carry = 0;
}
EngineCondition Vehicle::engine_condition() const { return {impl_->odometer, impl_->cond_e, impl_->cond_o}; }
void Vehicle::set_dirt_course(bool dirt) { impl_->dirt = dirt; }

void Vehicle::begin_pit_stop(const Quat& box_orientation) {
    Impl& m = *impl_;
    // [physics §8.6] velocities and dynamic states zeroed.
    m.vel = {};
    m.yaw_rate = m.ax_rate = m.ay_rate = 0;
    for (size_t i = 0; i < 4; ++i) m.omega[i] = m.kappa[i] = m.alpha[i] = 0;
    m.omega_e = 0; // [physics §9.3] engine speed 0, floored to idle on the first step
    m.diff = {};
    m.turbo = {};
    m.steer_avg = m.steer_rate = 0;
    m.flap_x = m.flap_v = {};
    m.wall_contacts.clear();
    m.spawn = 300; // [physics §8.6] 5 s stop, the scripted pose of physics §4.3
    m.spawn_heading = euler_angles(box_orientation).heading; // the box heading as reset() derives it
    publish();
}

void Vehicle::spawn(const Vec3& position, const Quat& orientation) {
    reset(position, orientation);
    impl_->spawn = 120; // [physics §4.3] spawn-hold counter
    impl_->spawn_heading = impl_->heading;
}

void Vehicle::spawn_moving(const Vec3& position, const Quat& orientation, float speed_mps) {
    reset(position, orientation);
    if (!(speed_mps > 0) || !std::isfinite(speed_mps)) return;
    Impl& m = *impl_;
    const CarSpec& s = spec_;
    Axes a = make_axes(m.ax, m.ay, m.heading);
    m.vel = a.fwd * double(speed_mps);
    for (size_t i = 0; i < 4; ++i) m.omega[i] = double(speed_mps) / s.axle[size_t(m.axle[i])].tire_radius;
    const double r_driven = driven_radius(s);
    float up = s.engine.upshift_rpm > 0 ? s.engine.upshift_rpm : s.engine.rev_limit_rpm - 500;
    int nf = int(s.gearbox.ratios.size());
    m.gear = std::clamp(model::gear_for_speed(s.gearbox, speed_mps, float(r_driven), up), 1, std::max(nf, 1));
    double w_in = double(speed_mps) / r_driven * model::total_ratio(s.gearbox, m.gear);
    double w_idle = s.engine.idle_rpm * model::kRpmToRadS, w_lim = s.engine.rev_limit_rpm * model::kRpmToRadS;
    // [physics §9.3] engine = clamp(v ratio, idle, rev limit), rpm truncated; clutch locked
    // (request 1); turbo lag states at the current rpm.
    int rpm = int(std::clamp(w_in, w_idle, w_lim) * model::kRadSToRpm);
    m.omega_e = rpm * model::kRpmToRadS;
    m.clutch = ClutchState::Locked;
    if (s.engine.turbo.on) m.turbo.lag = {rpm, rpm};
    state_.velocity = m.vel.f();
    state_.speed_kmh = speed_mps * 3.6f;
    state_.gear = m.gear;
    state_.engine_rpm = float(m.omega_e * model::kRadSToRpm);
}

void Vehicle::apply_impulse(const collide::V2& point, const collide::V2& J) {
    if (!std::isfinite(J.x) || !std::isfinite(J.z)) return;
    collide::Body b = collision_body();
    collide::V2 r = point - b.pos;
    b.vel = b.vel + J * b.inv_mass;
    b.yaw_rate += b.inv_inertia * r.cross(J);
    apply_collision_body(b);
}

void Vehicle::sync_to(const VehicleState& st, float yaw_rate) {
    Impl& m = *impl_;
    telemetry_.hit_sides = 0; // a published state carries no collision step of ours
    const bool fresh = m.settle; // never stepped since reset(): no internal history to keep
    const Euler e = euler_angles(st.orientation); // as reset() derives them
    m.ay = e.ay;
    m.heading = e.heading;
    m.ax = e.ax;
    Axes a = make_axes(m.ax, m.ay, m.heading);
    m.pos = D3(st.position) + a.to_world(m.cg_local);
    m.vel = D3(st.velocity);
    m.yaw_rate = std::isfinite(yaw_rate) ? yaw_rate : 0.0;
    D3 rest = D3(st.angular_velocity) - D3{0, m.yaw_rate, 0}; // = lat * ax_rate + fwd * ay_rate
    m.ax_rate = rest.dot(a.lat);
    m.ay_rate = rest.dot(a.fwd);
    for (size_t i = 0; i < 4; ++i) {
        const WheelState& w = st.wheels[i];
        bool spec_susp = spec_.axle[size_t(m.axle[i])].susp.valid;
        m.x[i] = m.x_prev[i] = spec_susp ? -w.suspension : w.suspension;
        m.omega[i] = w.angular_velocity;
        m.spin[i] = w.spin_angle;
        m.steer[i] = w.steer_angle;
        m.grounded[i] = w.grounded;
        m.surface[i] = w.surface;
        if (w.load > 0) m.load[i] = w.load;
        m.contact[i] = D3(w.contact);
    }
    m.omega_e = std::max(0.0f, st.engine_rpm) * model::kRpmToRadS;
    int nf = int(spec_.gearbox.ratios.size());
    m.gear = std::clamp(st.gear, -1, std::max(nf, 1));
    if (fresh) { // a new shadow: the drivetrain as if it had been driving (as spawn_moving does)
        m.clutch = m.gear != 0 && m.vel.length() > 1 ? ClutchState::Locked : ClutchState::Open;
        m.shift_lockout = 0;
        int rpm = int(st.engine_rpm);
        if (spec_.engine.turbo.on) m.turbo.lag = {rpm, rpm};
    }
    m.settle = false;
    m.spawn = 0;
    m.late_init = false; // the ground under the new pose is probed at the next tick
    publish();
}

void Vehicle::reset(const Vec3& position, const Quat& orientation) {
    Impl& m = *impl_;
    m.wall_contacts.clear();
    const Euler e = euler_angles(orientation);
    m.ay = e.ay;
    m.heading = e.heading;
    m.ax = e.ax;
    Axes a = make_axes(m.ax, m.ay, m.heading);
    m.pos = D3(position) + a.to_world(m.cg_local);
    m.vel = {};
    m.yaw_rate = m.ax_rate = m.ay_rate = 0;
    for (size_t i = 0; i < 4; ++i) {
        m.x[i] = m.x_prev[i] = spec_.axle[size_t(m.axle[i])].susp.valid ? m.xs[i] : m.rest_x[i];
        m.omega[i] = m.spin[i] = m.steer[i] = m.kappa[i] = m.alpha[i] = 0;
        // static corner load (at x = x_s: -k x_s + preload = corner weight), the wheel load the first
        // tick's prep reads [physics §3.1/physics §4.2]
        m.load[i] = spec_.axle[size_t(m.axle[i])].susp.valid
                        ? (sprung_corner(spec_, int(i)) + spec_.axle[size_t(m.axle[i])].susp.unsprung) * model::kGravity
                        : 0.0;
        m.grounded[i] = true;
        m.surface[i] = 0;
        m.contact[i] = D3(position) + a.to_world(m.hub[i] - D3{0, m.hub[i].y, 0});
        m.ground_prev[i] = position.y;
    }
    m.omega_e = 0; // [physics §9.3] engine speed 0, floored to idle on the first step
    m.limiter = false;
    m.gear = 1;
    m.clutch = ClutchState::Open;
    m.shift_lockout = 12; // [physics §9.3] standing start: shift lockout 12
    m.steer_avg = m.steer_rate = 0;
    m.flap_x = m.flap_v = {};
    m.long_accel = 0;
    m.settle = true;
    m.tcs_scale = 1;
    m.slip = 0;
    m.turbo = {};
    m.centre_engaged = true; // [interpretation: the original game's centre clutch holds at rest]
    m.axle_torque = {};
    m.g_over_mu = 0;
    m.long_force_sum = 0;
    m.yaw_acc = 0;
    m.rear_beta_prev = 0;
    m.free_roll = {};
    m.lat_g = 0;
    m.diff = {};
    m.cpu_clutch = false;
    m.pad_o3 = 0;
    m.spawn = 0;
    // Everything else that is per-run state, so a re-placed vehicle behaves exactly like a new one
    // (tyre wear, wear grip and engine condition persist on purpose).
    m.late_init = false; // phase-12 ground state left by the last tick
    m.late_normal = {};
    m.late_hits = {};
    m.late_gf = {};
    m.slip_g = {1, 1, 1, 1};
    m.slip_target = 0;
    m.smoke = {};
    m.smoke_emit = {};
    m.skid_pitch = {1, 1, 1, 1};
    m.brake_heat = {};
    m.air_steps = {};
    m.pit_surface = false;
    m.spawn_heading = m.heading;
    input_kind_ = 0;
    reverse_request_ = false;
    driver_kind_ = 0;
    clutch_hold_ = false;
    race_hold_ = false;
    telemetry_ = {};
    state_ = {};
    state_.position = position;
    state_.orientation = axes_to_quat(a);
    state_.gear = 1;
    state_.engine_rpm = spec_.engine.idle_rpm;
    for (size_t i = 0; i < 4; ++i) {
        state_.wheels[i].contact = m.contact[i].f();
        state_.wheels[i].suspension = float(m.x[i]);
        state_.wheels[i].grounded = true;
    }
}

void Vehicle::step(const ControlInputs& in, const TrackQuery& track, double dt, const WorldQuery* world) {
    if (!(dt > 0) || !std::isfinite(dt)) return;
    // One spec tick per 1/60 s; longer frames are split into equal ticks.
    int ticks = std::max(1, int(std::ceil(dt * kTickHz - 1e-6)));
    ControlInputs first = in, rest = in;
    rest.shift_up = rest.shift_down = false;
    for (int t = 0; t < ticks; ++t) tick(t == 0 ? first : rest, track, dt / ticks, world);
}

// One tick [physics §1.2], the phases in the original's order. The spec body runs its suspension step as phase 12
// at the end of the tick (after phase 10 moved the body); the next tick's tyre phase reads what it left.
void Vehicle::tick(const ControlInputs& in_raw, const TrackQuery& track, double h, const WorldQuery* world) {
    Impl& m = *impl_;
    const Driver drv{cpu_driver_, driver_kind_, clutch_hold_, race_hold_, reverse_request_, scripted_aids_};
    const TickContext t{spec_, condition_inputs(in_raw, drv, m.gear, spec_.engine.electric, state_.speed_kmh), in_raw,
                        h, drv, has_spec_body(spec_)};
    const Impl saved = m; // the NaN guard's restore point
    VehicleTelemetry tel;
    tel.substeps = 1;

    // --- start: ground fit after a reset, body frame, steering ---
    if (m.settle) m.settle_on_ground(spec_, track);
    const Frame fr = m.frame(t.spec_body);
    if (t.in.automatic && m.gear == 0) m.set_gear(1);
    m.steering(t, fr, input_kind_);
    m.wheel_steer_angles(spec_);

    // --- ground and suspension, aero, tyre slips and peak forces ---
    // [physics §3.1, phase 2] the spec body's tyres read the wheel loads the previous tick's phase 12 left (the tyres
    // lag the loads by a tick); the placeholder body's read this tick's suspension step.
    const std::array<double, 4> load_prev = m.load;
    const Ground g = m.start_ground(t, track, fr);
    const std::array<double, 4> tyre_load = t.spec_body ? load_prev : m.load;
    BodyForces body{{0, -double(spec_.mass) * model::kGravity, 0}, {}};
    if (!t.spec_body) m.placeholder_suspension_forces(g, fr.body, body);
    const model::AeroOut aero = m.aero(t, fr, tel);
    if (!t.spec_body) m.placeholder_aero_forces(spec_, fr.body, aero, body);
    const WheelMotion wm = m.wheel_motion(t, fr, g, tyre_load, tel);

    // --- engine and clutch, brakes and aids, tyre forces, driveline ---
    const Powertrain pt = m.engine_and_clutch(t, fr, g, tel);
    const AxleInertia inertia = axle_inertia(spec_, m.clutch == ClutchState::Locked, pt.J, pt.G);
    const Brakes br = m.brakes(t, fr, g, wm, tel);
    const TyreForces tyre = t.spec_body ? m.tyre_forces(t, g, wm, tel)
                                        : m.placeholder_tyre_forces(t, fr, g, wm, pt, inertia, br, body, tel);
    const DriveStep ds = m.wheel_spin(t, fr, wm, tyre, br, pt, inertia, tel);
    m.flag_free_wheels(pt.lay, ds.drive_ax[0], tyre_load, br.torque);
    // [physics §10.3] odometer (forward only), then E and O by |u| dt
    if (fr.v_fwd >= 0) m.odometer_carry = model::odometer_step(m.odometer, m.odometer_carry + float(fr.v_fwd) * float(h));
    model::engine_condition_step(m.cond_e, m.cond_o, float(std::fabs(fr.v_fwd) * h));
    m.tyre_wear_and_heat(t, g, wm, br, tel);
    m.clutch_after_driveline(t, fr, pt, ds.w_side_after, tel);
    if (!t.spec_body || hull_contact_on()) m.placeholder_hull_contact(t, track, fr, body);

    // --- integration [physics §1.1: explicit Euler, rate then value] ---
    const D3 start_pos = m.pos;
    const double start_heading = m.heading;
    PlanarAccel planar;
    if (t.spec_body) {
        planar = m.planar_step(t, fr, g, wm, tyre, br, inertia, aero, tyre_load, body);
        // [physics §1.2, physics §4.1 "Position and heading"] phase 10 advances the body by velocity x dt and the
        // heading by yaw rate x dt; phase 12 then queries the ground at the moved hubs. (The spawn hold
        // keeps the old order: its scripted pose moves the body itself.)
        const bool moved = m.spawn == 0;
        Ground late = moved ? m.advance_and_query_ground(t, track) : m.suspension_step(t, track, fr.body, fr.omega_w);
        m.load_transfer(spec_, planar.ay, late);
        m.attitude_step(t, fr, aero, tyre, late, moved);
    } else {
        m.placeholder_integrate(t, fr, g, wm, br, body);
    }
    m.heading = std::remainder(m.heading, model::kTwoPi);
    // [physics §1.2 phase 13] the flap pendulums, from the loads and surfaces phase 12 left
    for (size_t i = 0; i < 4; ++i)
        model::flap_step(m.flap_x[i], m.flap_v[i], m.long_accel, float(wm.vxs[i]), float(m.load[i]),
                         float(m.omega[i] * wm.radius[i]),
                         spec_.surface_slot[size_t(std::clamp(model::surface_index(m.surface[i]), 0, 15))] != 0, float(h), vu0_random());

    // --- walls, end-of-tick wheel and gear states ---
    if (world) m.walls(t, *world, start_pos, start_heading, tel);
    m.record_acceleration(t, fr, body, planar, tel);
    state_.throttle = pt.eo.throttle;
    if (t.spec_body) m.lock_free_wheels(spec_);
    // [physics §2.6] end-of-tick gear change (see select_gear), from the ground-frame forward speed.
    m.select_gear(t, m.vel.dot(m.ground_frame(t.spec_body).fwd));

    if (!m.state_finite()) { // NaN guard: back to the start of the tick, at rest
        m = saved;
        m.vel = {};
        m.yaw_rate = m.ax_rate = m.ay_rate = 0;
        for (auto& w : m.omega) w = 0;
    }
    m.end_telemetry(t.spec_body, tel);
    telemetry_ = tel;
    publish();
}

// --- start of the tick --------------------------------------------------------------------

// Fits the body to the ground under the four hubs (pitch, roll, height) at the first tick after a reset, so a
// reset on a slope or banking starts at static ride height.
void Vehicle::Impl::settle_on_ground(const CarSpec& s, const TrackQuery& track) {
    settle = false;
    for (int pass = 0; pass < 3; ++pass) {
        Axes A0 = make_axes(ax, ay, heading);
        double gh[4];
        bool ok = true;
        for (size_t i = 0; i < 4; ++i) {
            D3 hw = pos + A0.to_world(hub[i] - cg_local);
            SurfaceHit hit = track.probe(float(hw.x), float(hw.z), float(hw.y + 2.0));
            ok = ok && hit.hit;
            gh[i] = hit.height;
        }
        if (!ok) break;
        double tf = 0.5 * (s.axle[0].track + s.axle[1].track);
        double pitch = std::atan2(0.5 * (gh[0] + gh[1]) - 0.5 * (gh[2] + gh[3]), s.wheelbase); // nose up > 0
        double roll = std::atan2(0.5 * (gh[0] + gh[2]) - 0.5 * (gh[1] + gh[3]), tf);           // left high > 0
        ax = clampd(pitch, -kMaxAttitude, kMaxAttitude);
        ay = clampd(roll, -kMaxAttitude, kMaxAttitude);
        Axes A1 = make_axes(ax, ay, heading);
        D3 origin{0, 0, 0};
        for (size_t i = 0; i < 4; ++i) origin.y += gh[i] * 0.25;
        D3 o = pos - A1.to_world(cg_local); // current origin; keep x/z, set ground height
        o.y = origin.y;
        pos = o + A1.to_world(cg_local);
    }
}

// [physics §10.1] The ground frame the wheel speeds, u and the gear choice read: with the spec suspension the one
// the last phase 12 left; the body axes before the first phase 12 and for the placeholder body.
Axes Vehicle::Impl::ground_frame(bool spec_body) const {
    return spec_body && late_init ? make_axes(late_gf.pitch, late_gf.roll, heading) : make_axes(ax, ay, heading);
}

Frame Vehicle::Impl::frame(bool spec_body) const {
    Frame f;
    f.body = make_axes(ax, ay, heading);
    f.fwd_h = D3{f.body.fwd.x, 0, f.body.fwd.z}.normalized();
    f.ground = ground_frame(spec_body);
    f.v_fwd = vel.dot(f.ground.fwd);
    f.omega_w = D3{0, yaw_rate, 0} + f.body.lat * ax_rate + f.body.fwd * ay_rate;
    return f;
}

// [physics §2.6] instantaneous shifts, 1-step lockout.
void Vehicle::Impl::set_gear(int to) {
    if (to == gear) return;
    int from = gear;
    gear = to;
    shift_lockout = 1;
    // [physics §2.4] with the clutch engaged, a change to or from
    // reverse gives the launch slip (2), a forward -> forward change the fixed-torque
    // shift clutch (3) until the speeds cross. A locked clutch never just opens.
    clutch = ClutchState(model::clutch_gear_change(int(clutch), from, to));
}

// --- steering [physics §4.4]: pad curve B, second-order follower; Ackermann [placeholder] ---
void Vehicle::Impl::steering(const TickContext& t, const Frame& fr, int& input_kind) {
    const CarSpec& s = t.s;
    const ControlInputs& in = t.in;
    if (std::isfinite(in.steer_angle)) {
        // CPU cars write the road-wheel angle directly, bypassing the pad follower
        // and the car's lock (the AI clamps itself at 45 deg).
        steer_avg = clampd(in.steer_angle, -1.0, 1.0);
        steer_rate = 0;
        return;
    }
    // [physics §4.4] controller kind (0 = keep the previous one); a step with nonzero steer sets
    // the pad state O[3]: kind 3 -> digital (30 deg/s turn-in), kind 2 -> analog (120 deg/s).
    if (in.controller_kind != 0) input_kind = in.controller_kind;
    if (in.steer != 0 && (input_kind == 2 || input_kind == 3)) pad_o3 = input_kind == 3 ? 1 : 2;
    // [physics §3.2/physics §4.4] mean velocity-direction angle phi = atan2(-l_h, u_h) of the two front hubs (0 while the
    // low-speed fade is 0), from the hub velocities the last tick left (ground-frame tangents, physics §10.1), in our
    // sign (+ = right) for the lock extension
    double va = 0;
    for (size_t i = 0; i < 2; ++i) {
        D3 hv = vel + D3{0, yaw_rate, 0}.cross(contact[i] - pos);
        float fade = 0;
        float phi = model::patch_slip_angle(float(hv.dot(fr.ground.fwd)), float(hv.dot(fr.ground.lat)), fade);
        va -= 0.5 * double(phi);
    }
    float target = model::steer_target(s.steering, in.steer, float(fr.v_fwd), float(va));
    float angle = float(steer_avg), rate = float(steer_rate);
    model::steer_follow(s.steering, target, angle, rate, float(t.h), pad_o3 == 1);
    steer_avg = angle;
    steer_rate = rate;
}

// [physics §4.4] front wheels: delta x Ackermann ratio (the inner wheel,
// on the side the car turns to, gets the larger one); toe-in on all wheels
// (our steer sign is + right, so toe-in is + on the left wheel, - on the right).
void Vehicle::Impl::wheel_steer_angles(const CarSpec& s) {
    double dlt = steer_avg;
    double r_left = dlt >= 0 ? s.steering.ack_outer : s.steering.ack_inner;
    double r_right = dlt >= 0 ? s.steering.ack_inner : s.steering.ack_outer;
    if (s.steering.ack_inner <= 0) {
        float sl, sr;
        model::ackermann(s.steering, float(dlt), s.wheelbase, s.axle[0].track, sl, sr);
        steer[0] = sl;
        steer[1] = sr;
    } else {
        steer[0] = dlt * r_left;
        steer[1] = dlt * r_right;
    }
    steer[0] += s.axle[0].toe;
    steer[1] -= s.axle[0].toe;
    steer[2] = s.axle[1].toe;
    steer[3] = -s.axle[1].toe;
}

// --- ground query at the hubs [physics §1.3] and suspension [placeholder physics §4] ---

// [physics §1.3/physics §4.3] the wheel query point of a spec-suspension axle is the hub offset in the CoG plane,
// (x_long, y_lat, 0) rotated by the attitude: pitch and roll move it along the ground only by lever x cos, not
// by the hub's height above the CoG [verified: 4 mm at 1 deg of pitch, which decides kerbs and road edges].
// Hand-built axles keep the hub at its true height.
D3 Vehicle::Impl::hub_point(const CarSpec& s, size_t i, const Axes& A) const {
    D3 off = hub[i] - cg_local;
    if (s.axle[size_t(axle[i])].susp.valid) off.y = 0;
    return pos + A.to_world(off);
}

// [physics §10.1] hub velocity of wheel i in the ground frame: u + X r (forward) and l - Y r (right),
// u / l = world velocity on the ground-frame tangents, X = (-tf, +tf, -tr, +tr), Y = (a, a, -b, -b).
void Vehicle::Impl::hub_velocity(const CarSpec& s, size_t i, const Axes& Ag, double& uh, double& lh) const {
    const double a_len = s.wheelbase * (1 - s.front_weight), b_len = s.wheelbase * s.front_weight;
    const double X = (i % 2 == 0 ? -0.5 : 0.5) * s.axle[i / 2].track, Y = i < 2 ? a_len : -b_len;
    uh = vel.dot(Ag.fwd) + X * yaw_rate;
    lh = vel.dot(Ag.lat) - Y * yaw_rate;
}

// Tyre record for a wheel on surface `surf` [physics §6.5]: compound slot = class of the surface.
const TireSpec& Vehicle::Impl::tire_for(const CarSpec& s, int wheel, int surf) const {
    int slot = s.surface_slot[size_t(std::clamp(surf, 0, 15))];
    if (slot > 2) slot = 0;
    return s.tire_slots[size_t(axle[size_t(wheel)])][size_t(slot)];
}

// The ground the tyre phase reads. The spec body's is what the previous tick's phase 12 left (this tick's query
// at the first tick after a reset); the placeholder body queries the ground and steps the suspension here.
Ground Vehicle::Impl::start_ground(const TickContext& t, const TrackQuery& track, const Frame& fr) {
    if (t.spec_body && late_init) {
        Ground g;
        g.normal = late_normal;
        g.hits = late_hits;
        g.gf = late_gf;
        g.axes = fr.ground;
        for (size_t i = 0; i < 4; ++i) g.sidx[i] = model::surface_index(surface[i]);
        return g;
    }
    Ground g = suspension_step(t, track, fr.body, fr.omega_w);
    if (t.spec_body) late_init = true;
    return g;
}

// Ground query at the hubs and the suspension step for the body pose (axes A, angular velocity omega_w): phase 12
// of the spec body [physics §1.2], the start of the tick for the placeholder body. Each axle takes its own
// suspension (the spec block or the placeholder spring).
Ground Vehicle::Impl::suspension_step(const TickContext& t, const TrackQuery& track, const Axes& A, const D3& omega_w) {
    const CarSpec& s = t.s;
    Ground g;
    std::array<double, 4> ground_h{};
    for (size_t i = 0; i < 4; ++i) {
        g.hub_w[i] = hub_point(s, i, A);
        g.hits[i] = track.probe(float(g.hub_w[i].x), float(g.hub_w[i].z), float(g.hub_w[i].y + 1.0));
        if (g.hits[i].hit) {
            g.sidx[i] = model::surface_index(g.hits[i].surface);
            D3 hub_vel = vel + omega_w.cross(g.hub_w[i] - pos);
            ground_h[i] = g.hits[i].height + model::road_bump(g.sidx[i], float(g.hub_w[i].x), float(g.hub_w[i].z),
                                                              float(hub_vel.length()));
        } else {
            ground_h[i] = ground_prev[i]; // a miss keeps the previous ground height (physics §1.3)
        }
        ground_prev[i] = ground_h[i];
    }
    // [physics §4.3] ground frame and terrain factor s = ground up.z / body up.z.
    {
        float gh[4] = {float(ground_h[0]), float(ground_h[1]), float(ground_h[2]), float(ground_h[3])};
        g.gf = model::ground_frame(gh, 1 - s.front_weight, s.wheelbase, 0.5f * (s.axle[0].track + s.axle[1].track));
    }
    g.axes = make_axes(g.gf.pitch, g.gf.roll, heading);
    const double terrain = g.axes.up.y / std::max(A.up.y, 0.2);
    for (size_t i = 0; i < 4; ++i) {
        const AxleSpec& a = s.axle[size_t(axle[i])];
        if (a.susp.valid) spec_wheel_contact(a, i, ground_h[i], terrain, g);
        else placeholder_wheel_contact(a, i, ground_h[i], terrain, A, g);
    }
    for (size_t i = 0; i < 4; ++i) {
        const AxleSpec& a = s.axle[size_t(axle[i])];
        if (a.susp.valid) spec_wheel_suspension(a, i, t.h, A, g);
        else placeholder_wheel_suspension(a, i, t.h, g);
    }
    if (s.axle[0].susp.valid && !t.spec_body) front_block_load_transfer(s, g);
    late_normal = g.normal;
    late_hits = g.hits;
    late_gf = g.gf;
    return g;
}

// [physics §4.2] spec suspension coordinate of wheel i: x = (body height + hub offset - ground height - bump) x att
// + const, the plain ground height with no slope correction (the triangle normal does not enter the spring
// coordinate). With the query point in the CoG plane: our body sits cg_height above the ground where the spec's
// body height sits q above it, so the gap is measured from ground + cg_height - q and x = x_s - q - gap att
// (equal to x_s at the static pose).
void Vehicle::Impl::spec_wheel_contact(const AxleSpec& a, size_t i, double ground, double terrain, Ground& g) {
    const SurfaceHit& hit = g.hits[i];
    // [physics §1.3/physics §10.5] a ground-query miss keeps the previous ground height (and surface): the
    // wheel stands on that height, the car is not stopped.
    D3 n = hit.hit ? D3(hit.normal).normalized() : D3{0, 1, 0};
    double gap = (ground + (cg_local.y - a.susp.ride_height)) - g.hub_w[i].y;
    double x_gap = rest_x[i] + gap * terrain;
    g.normal[i] = n;
    if (hit.hit) surface[i] = hit.surface;
    else g.sidx[i] = model::surface_index(surface[i]);
    contact[i] = {g.hub_w[i].x, ground, g.hub_w[i].z};
    // Spec suspension coordinate: x = x_s - (contact height - hub height) [physics §4.2].
    grounded[i] = true; // the suspension step decides (droop = airborne)
    x[i] = xs[i] - a.susp.ride_height - (x_gap - rest_x[i]);
}

// [placeholder] compression of a hand-built axle's spring: the gap between the hub and its contact height (the
// tyre radius along the ground normal), scaled by the terrain factor; no ground under the hub = airborne.
void Vehicle::Impl::placeholder_wheel_contact(const AxleSpec& a, size_t i, double ground, double terrain, const Axes& A,
                                              Ground& g) {
    const SurfaceHit& hit = g.hits[i];
    g.normal[i] = {0, 1, 0};
    grounded[i] = false;
    double x_gap = 0;
    if (hit.hit) {
        D3 n = D3(hit.normal).normalized();
        double gap = (ground + a.tire_radius / std::max(n.y, 0.5)) - g.hub_w[i].y;
        x_gap = rest_x[i] + gap * terrain;
        g.normal[i] = n;
        surface[i] = hit.surface;
        contact[i] = {g.hub_w[i].x, ground, g.hub_w[i].z};
    }
    if (x_gap > 0) {
        grounded[i] = true;
    } else {
        x_gap = 0;
        contact[i] = g.hub_w[i] - A.up * a.tire_radius;
    }
    x[i] = std::min(x_gap, double(a.travel) * 2.0);
}

// [physics §4.2] spring, damper, bump stop and anti-roll bar of a spec-suspension wheel; the tyre load
// Fz = max(0, Fs + Fd + preload).
void Vehicle::Impl::spec_wheel_suspension(const AxleSpec& a, size_t i, double h, const Axes& A, Ground& g) {
    double v = (x[i] - x_prev[i]) / h;
    model::SuspensionOut so{};
    if (grounded[i]) {
        so = model::suspension_wheel(a.susp, float(x[i]), float(v), float(xs[i]));
    } else { // no ground under the hub: airborne (physics §10.5)
        so = model::suspension_wheel(a.susp, 1e9f, 0, float(xs[i]));
    }
    grounded[i] = so.grounded;
    load[i] = so.load;
    g.body_f[i] = so.spring + so.damper; // airborne: -preload [physics §10.5]
    g.damper_f[i] = so.grounded ? so.damper : 0.0;
    if (so.grounded) {
        size_t l = i & ~size_t(1), r = l + 1;
        double arb = model::anti_roll(a.susp, float(x[l]), float(x[r]));
        g.body_f[i] += (i == l) ? arb : -arb;
    } else {
        contact[i] = pos + A.to_world(hub[i] - cg_local) - A.up * a.tire_radius;
    }
    x_prev[i] = double(so.x); // [physics §10.5] the clamped x'
}

// [placeholder] linear spring and damper with a linear anti-roll bar; the body force is the tyre load.
void Vehicle::Impl::placeholder_wheel_suspension(const AxleSpec& a, size_t i, double h, Ground& g) {
    double v = (x[i] - x_prev[i]) / h;
    double f = model::suspension_force(a, float(x[i]), float(v));
    if (grounded[i]) f += a.arb_rate * (x[i] - x[i ^ 1]);
    load[i] = grounded[i] ? std::max(f, 0.0) : 0.0;
    g.body_f[i] = load[i];
    x_prev[i] = x[i];
}

// Explicit load transfer [physics §4.2] for a spec with the suspension block on the front axle only (a placeholder
// body): GT3 has no tyre-force moments about the CoG; the last tick's accelerations shift body force between the
// wheels: longitudinal m_s X a / L (front <-> rear), lateral m_s X a / track (left <-> right), with X = spec
// CoG-height parameter (CarSpec::cg_height). The spec body adds the terms after the lateral step instead
// (load_transfer(): phase 12 runs after phase 7 and reads this tick's lateral acceleration and force sum).
void Vehicle::Impl::front_block_load_transfer(const CarSpec& s, Ground& g) const {
    const double M = s.mass;
    double ms = M - 2.0 * (s.axle[0].susp.unsprung + s.axle[1].susp.unsprung);
    double X = s.cg_height;
    double t_mean = 0.5 * (s.axle[0].track + s.axle[1].track);
    // [physics §4.2] the longitudinal factor x the force sum in full per wheel, with the factor =
    // (sprung fraction) X / L and the force sum = sum of the longitudinal tyre forces x cos steer
    // (previous tick's tyre phase).
    double d_long = 2.0 * (ms / M) * X / s.wheelbase * long_force_sum;
    double d_lat = ms * X * lat_g * model::kGravity / t_mean; // right accel loads the left
    for (size_t i = 0; i < 4; ++i) {
        if (!grounded[i]) continue;
        // An inertial moment: the loaded side's support force drops so the body
        // sinks there and its springs (and tyre loads) take the transfer.
        // the lateral term (lateral factor x -a_y) in full on each wheel (left -, right +)
        g.body_f[i] += (i < 2 ? 0.5 : -0.5) * d_long + (i % 2 == 0 ? -1.0 : 1.0) * d_lat;
    }
}

// [placeholder] the suspension pushes the rigid body at the hubs.
void Vehicle::Impl::placeholder_suspension_forces(const Ground& g, const Axes& A, BodyForces& body) const {
    for (size_t i = 0; i < 4; ++i)
        if (g.body_f[i] != 0) body.apply(A.up * g.body_f[i], g.hub_w[i], pos);
}

// --- aero and slipstream [physics §5.2] ---
model::AeroOut Vehicle::Impl::aero(const TickContext& t, const Frame& fr, VehicleTelemetry& tel) {
    model::AeroOut out = model::aero_step(t.s.aero, float(fr.v_fwd), float(slip_target), float(slip), float(t.h));
    slip = out.slip;
    tel.slipstream = float(slip);
    tel.drag_force = std::fabs(out.drag);
    tel.downforce = out.lift_front + out.lift_rear;
    return out;
}

// [placeholder] drag along the body and the downforce at the axles. (The spec body takes the drag in the forward
// step and the downforce in the attitude step.)
void Vehicle::Impl::placeholder_aero_forces(const CarSpec& s, const Axes& A, const model::AeroOut& aero,
                                            BodyForces& body) const {
    body.force += A.fwd * double(aero.drag);
    D3 front_axle = pos + A.to_world(D3{0, 0, -s.wheelbase * 0.5} - cg_local);
    D3 rear_axle = pos + A.to_world(D3{0, 0, s.wheelbase * 0.5} - cg_local);
    body.apply(A.up * -double(aero.lift_front), front_axle, pos);
    body.apply(A.up * -double(aero.lift_rear), rear_axle, pos);
}

// --- tyre geometry and slips [physics §3.2, physics §3.4]; peak forces [physics §3.1] ---
WheelMotion Vehicle::Impl::wheel_motion(const TickContext& t, const Frame& fr, const Ground& g,
                                        const std::array<double, 4>& tyre_load, VehicleTelemetry& tel) {
    const CarSpec& s = t.s;
    // [physics §8.6] pit-lane surface: triangle flag bit 0 under any wheel.
    pit_surface = false;
    for (size_t i = 0; i < 4; ++i)
        if (g.hits[i].hit && ((g.hits[i].surface >> 8) & 1)) pit_surface = true;
    tel.on_pit_surface = pit_surface;
    // [physics §8.4] wear -> grip (phase 2), not on the pit surface.
    std::array<double, 4> wear_mul{1, 1, 1, 1};
    if (wear_p.on() && !pit_surface)
        for (size_t i = 0; i < 4; ++i) {
            const TireSpec& tw = tire_for(s, int(i), g.sidx[i]);
            wear_grip[i] = model::wear_grip(wear_p, wear[i], tw.surface_mu[11], &wear_level[i]);
            wear_mul[i] = wear_grip[i];
        }
    WheelMotion wm;
    for (size_t i = 0; i < 4; ++i) {
        wm.radius[i] = s.axle[size_t(axle[i])].tire_radius;
        if (t.spec_body) spec_wheel_velocity(s, i, g.axes, wm);
        else placeholder_wheel_velocity(i, g.axes, wm);
        wm.vhub[i] = wm.vcs[i].length();
        float fade = 0;
        alpha[i] = model::patch_slip_angle(float(wm.vxs[i]), float(wm.vys[i]), fade);
        wm.fades[i] = fade;
        // the hub's forward speed: along the ground frame (the placeholder body: the horizontal forward direction)
        const double hub_fwd = t.spec_body ? wm.vcs[i].dot(g.axes.fwd) : wm.vcs[i].dot(fr.fwd_h);
        kappa[i] = model::slip_ratio(float(wm.vxs[i]), float(omega[i] * wm.radius[i]), float(hub_fwd));
        slip_g[i] = model::slip_gain(float(hub_fwd)); // [physics §3.4] from the hub forward speed
        if (!grounded[i]) continue;
        // Peak force [physics §3.1]: record by surface class (physics §6.5), load, mu[s], camber.
        const AxleSpec& as = s.axle[size_t(axle[i])];
        const TireSpec& ts = tire_for(s, int(i), g.sidx[i]);
        // [physics §3.1] alpha = sigma (body roll - ground roll) + camber offset,
        // sigma = -1 left, +1 right; roll + = right side down.
        double sigma = (i % 2 == 0) ? -1.0 : 1.0;
        double roll_rel = -std::asin(clampd(fr.body.lat.dot(g.axes.up), -1, 1));
        float cam = as.camber.x.empty() ? 1.0f : model::lookup(as.camber, float(sigma * roll_rel + as.camber_offset));
        wm.peaks[i] = model::tire_peak_force(ts, float(tyre_load[i]), g.sidx[i], cam) * as.axle_factor * wear_mul[i];
    }
    return wm;
}

// [physics §10.1] wheel speeds are measured along the GROUND-frame tangents (ground pitch/roll from the four wheel
// heights, physics §4.3), common to all wheels, and the hub velocity carries only the yaw rate (u + X r, l - Y r):
// pitch/roll rates and the per-triangle normals do not enter.
void Vehicle::Impl::spec_wheel_velocity(const CarSpec& s, size_t i, const Axes& Ag, WheelMotion& wm) const {
    // the hub velocity (u + X r, l - Y r) [physics §10.1] and the speeds along / across the wheel heading
    double uh, lh;
    hub_velocity(s, i, Ag, uh, lh);
    wm.vcs[i] = Ag.fwd * uh + Ag.lat * lh;
    wm.vxs[i] = along_heading(uh, lh, steer[i]);
    wm.vys[i] = -uh * std::sin(steer[i]) + lh * std::cos(steer[i]);
}

// [placeholder] the contact point's velocity (yaw rate only) on the wheel's heading and right axes in the ground
// plane; the previous tick's free wheels are locked to it here [physics §10.2] (the spec body locks them at the end
// of that tick instead).
void Vehicle::Impl::placeholder_wheel_velocity(size_t i, const Axes& Ag, WheelMotion& wm) {
    const D3 n = Ag.up;
    D3 f = Ag.to_world(D3{std::sin(steer[i]), 0, -std::cos(steer[i])});
    f = (f - n * n.dot(f)).normalized();
    wm.wf[i] = f;
    wm.wr[i] = f.cross(n);
    wm.vcs[i] = vel + D3{0, yaw_rate, 0}.cross(contact[i] - pos);
    wm.vxs[i] = wm.vcs[i].dot(wm.wf[i]);
    wm.vys[i] = wm.vcs[i].dot(wm.wr[i]);
    if (free_roll[i]) omega[i] = wm.vxs[i] / wm.radius[i];
}

// --- engine and clutch [physics §2.2, physics §2.4] ---

// [physics §4.6] CPU cars (CPU control source) and a scripted driver kind (!= 0, outside
// the spawn counter) read the AI's driver aid block (Driver::aids): its TCS and ASCC channels.
bool Vehicle::Impl::scripted_aids(const Driver& drv) const { return drv.cpu || (drv.kind != 0 && spawn == 0); }

namespace {

// The ASCC channels of the AI's aid block in the car block's form (physics §4.6; ai §9.1).
AidsSpec ascc_block(const ScriptedAids& a) {
    AidsSpec s;
    s.ascc_a_gain = a.ascc_a_gain, s.ascc_a_thr = a.ascc_a_thr, s.ascc_d_gain = a.ascc_d_gain;
    s.ascc_c_gain = a.ascc_c_gain, s.ascc_c_thr = a.ascc_c_thr, s.ascc_e_gain = a.ascc_e_gain;
    s.ascc_flags = a.ascc_flags;
    return s;
}

} // namespace

// Traction control [physics §4.6]: throttle x clamp(1 + gain g (s_min - s* lat scale)).
void Vehicle::Impl::traction_control(const TickContext& t, const Ground& g) {
    const CarSpec& s = t.s;
    tcs_scale = 1;
    // [ai §9.1] the scripted path reads the AI's block (TCS and ASCC from the driver parameters);
    // the human path the car's block. The handbrake only disables ASCC (physics §4.6).
    const bool scripted = scripted_aids(t.drv);
    const float tcs_gain = scripted ? t.drv.aids.tcs_gain : (s.aids.tcs ? s.aids.tcs_gain : 0.0f);
    const float tcs_a = scripted ? t.drv.aids.tcs_lat_a : s.aids.tcs_lat_a;
    const float tcs_b = scripted ? t.drv.aids.tcs_lat_b : s.aids.tcs_lat_b;
    const float tcs_sc = scripted ? t.drv.aids.tcs_scale : s.aids.tcs_scale;
    if (!(tcs_gain > 0)) return;
    double s_min = 0;
    int wmin = -1;
    for (int i = 0; i < 4; ++i)
        if (driven(s.drivetrain.drive, i) && grounded[size_t(i)] && kappa[size_t(i)] < s_min) {
            s_min = kappa[size_t(i)];
            wmin = i;
        }
    if (wmin < 0) return;
    double gain = slip_g[size_t(wmin)]; // [physics §4.6] g of the lowest-slip wheel
    // [physics §4.1/4.6] lateral input |g / mu| = |4 a_y / sum(base grip x mu[s])|, the
    // records' base grip being 9.8 for every car: lateral g over the mean surface mu.
    double mu_sum = 0;
    for (int k = 0; k < 4; ++k) {
        const TireSpec& tk = tire_for(s, k, g.sidx[size_t(k)]);
        int sk = std::clamp(g.sidx[size_t(k)], 0, 15);
        mu_sum += tk.surface_mu[size_t(sk)] != 0 ? tk.surface_mu[size_t(sk)] : tk.mu * model::surface_mu(sk);
    }
    double g_mu = mu_sum > 0 ? std::fabs(4 * lat_g / mu_sum) : 0;
    g_over_mu = mu_sum > 0 ? 4 * lat_g / mu_sum : 0;
    float lat = model::tcs_lateral(float(g_mu), tcs_a, tcs_b);
    tcs_scale = model::tcs_factor(tcs_gain, float(gain), float(s_min), s.tire[size_t(wmin / 2)].opt_drive, lat, tcs_sc);
}

// Clutch-link transitions [physics §2.4]. The clutch request is set by the driver layer every tick: 0
// during the spawn/stop counter; 0 for a rear-driven car with the handbrake on; engaged: 1 unless rpm < idle;
// open: 1 while the brake pedal < 0.25.
void Vehicle::Impl::clutch_link(const TickContext& t, double G, double w_in, double v_fwd, double throttle) {
    const CarSpec& s = t.s;
    const ControlInputs& in = t.in;
    const Driver& drv = t.drv;
    if (G == 0) clutch = ClutchState::Open; // neutral
    double rpm_now = omega_e * model::kRadSToRpm;
    // [physics §2.4] the request is 0 while the race is frozen (race hold), during the spawn/stop counter and in driver kind 8
    bool clutch_request = model::clutch_request(int(clutch), float(rpm_now), s.engine.idle_rpm, in.brake,
                                                in.handbrake ? 1.0f : 0.0f, s.drivetrain.drive != Drive::FF,
                                                spawn > 0 || drv.race_hold || drv.kind == 8);
    // [physics §2.4] driver kind 8 (parked) keeps the request at 0 above: the AI rule does not
    // run for it, for the player's car and CPU cars alike, so the clutch opens and stays open.
    const bool ai_rule = drv.scripted() && drv.kind != 8;
    if (spawn > 0) { // the spawn hold zeroes the clutch request and holds gear 1
        gear = 1;
    } else if (ai_rule) { // [physics §4.4] the scripted path (kinds 1-7, CPU cars) uses the AI rule
        // [physics §4.4 AI note] binary clutch request: from open, engage when rpm > 0.75 x
        // upshift rpm; once closed, stay engaged while rpm >= idle.
        double rpm = omega_e * model::kRadSToRpm;
        double up = s.engine.upshift_rpm > 0 ? s.engine.upshift_rpm : s.engine.rev_limit_rpm - 500;
        // [ai §1] "while open" / "engaged" refer to the clutch link state S, not to
        // the previous request (a moving spawn starts locked and stays engaged).
        cpu_clutch = clutch != ClutchState::Open ? rpm >= s.engine.idle_rpm : rpm > 0.75 * up;
        if (drv.clutch_hold) cpu_clutch = false; // [ai §10] countdown: request 0
        clutch_request = cpu_clutch;
    }
    // request 0 opens (1 -> 0 only this way); 0 -> 2 on the launch rules
    clutch = ClutchState(model::clutch_engage(int(clutch), clutch_request, G == 0 ? 0 : gear, float(v_fwd),
                                              float(throttle), float(rpm_now), s.engine.launch_rpm, s.engine.idle_rpm));
    if (s.engine.electric) {
        // Direct drive (mod EVs): connected whenever the driver layer asks for drive at all.
        // The AI rule above waits for revs an unloaded motor never has, so it is replaced by
        // "not held" here.
        bool on = ai_rule ? (spawn == 0 && !drv.clutch_hold) : clutch_request;
        clutch = on && G != 0 ? ClutchState::Locked : ClutchState::Open;
        // connected straight away: the motor turns with the wheels
        if (clutch == ClutchState::Locked) omega_e = std::max(0.0, w_in);
    }
}

// Throttle (TCS, shift cut / blip, turbo lag), engine torque and the engine speed through the clutch link
// [physics §2.2-2.4].
Powertrain Vehicle::Impl::engine_and_clutch(const TickContext& t, const Frame& fr, const Ground& g,
                                            VehicleTelemetry& tel) {
    const CarSpec& s = t.s;
    const DrivetrainSpec& d = s.drivetrain;
    const double h = t.h;
    Powertrain p;
    p.G = model::total_ratio(s.gearbox, gear);
    // [physics §2.7] layout and the engine-side axle speed per layout.
    p.lay = model::awd_layout(int(d.drive), d.awd_system, d.awd_share_byte, d.awd_strength_byte);
    const double w_shaft = layout_axle_speed(p.lay, 0.5 * (omega[0] + omega[1]), 0.5 * (omega[2] + omega[3]));
    const double w_in = w_shaft * p.G; // engine-side speed of the gearbox input
    const double w_idle = s.engine.idle_rpm * model::kRpmToRadS;
    p.w_cap = (s.engine.rev_limit_rpm + 10) * model::kRpmToRadS;
    p.J = s.engine.inertia;

    traction_control(t, g);
    double throttle = std::clamp(double(t.in.throttle), 0.0, 1.0) * tcs_scale;
    tel.tcs_factor = float(tcs_scale);
    tel.tcs_active = tcs_scale < 1;
    tel.throttle_processed = float(throttle);

    clutch_link(t, p.G, w_in, fr.v_fwd, throttle);
    // [physics §2.2/physics §2.4] no re-sync to the wheel side here: a locked clutch set the engine speed at the end of
    // the last tick (axle speed x R_g, capped at rev limit + 10 rpm), and T_wot / friction (engine_torque below) and
    // the limiter's rpm word are read from that capped value [verified on the first limiter tick].
    // Shift clutch (3): the processed throttle is cut (wheel side slower) or blipped.
    if (clutch == ClutchState::Shift) {
        throttle = w_in < omega_e ? 0.0 : 1.0;
        tel.throttle_processed = float(throttle); // the processed throttle shows the cut / blip
    }
    {
        // [physics §2.2/2.3] turbo lag inside the engine-torque call: the load follows the
        // processed throttle x after TCS and the shift cut / blip, so boost decays
        // during a shift. The clutch engage rule above reads x itself, not the lagged x'.
        int n_rpm = std::max(int(omega_e * 60.0 / model::kTwoPi), int(s.engine.idle_rpm) - 1);
        throttle = model::turbo_step(s.engine.turbo, turbo, float(throttle), n_rpm, float(h));
        if (s.engine.turbo.on)
            tel.boost = model::turbo_boost(s.engine.turbo, float(turbo.lag[0]), float(turbo.lag[1])) - 1;
        tel.turbo_load = turbo.load;
        tel.turbo_lag = turbo.lag;
    }
    double rpm_e = omega_e * model::kRadSToRpm;
    p.eo = model::engine_torque(s.engine, float(rpm_e), float(throttle), limiter,
                                model::engine_condition_factor(cond_e, cond_o));
    limiter = p.eo.limiter;
    // [physics §2.4] T_g = T_c R_g: no driveline efficiency
    double clutch_share = 0;
    if (clutch == ClutchState::Locked) {
        p.shaft_torque = p.eo.torque * p.G;
        clutch_share = 1;
    } else if (clutch == ClutchState::Slipping || clutch == ClutchState::Shift) {
        double dw = omega_e - w_in;
        // state 3 transmits the fixed +-shift-clutch torque byte x 9.8 N*m (never written: 0) [physics §2.4, physics §6.2]
        double tc = clutch == ClutchState::Shift ? (dw > 0 ? 1 : dw < 0 ? -1 : 0) * s.drivetrain.shift_clutch_torque
                                                 : double(model::clutch_slip_torque(p.eo.wot, p.eo.torque, float(dw)));
        double next = omega_e + h * (p.eo.torque - tc) / p.J;
        p.slip_dw = dw;
        omega_e = std::max(next, 0.0); // no idle floor while slipping
        p.shaft_torque = tc * p.G;
        clutch_share = p.eo.wot > 0 ? std::min(1.0, std::fabs(tc) / p.eo.wot) : 0;
    } else if (s.engine.electric) {
        omega_e = 0; // neutral: the motor is not driven
    } else {
        omega_e = std::max(omega_e + h * p.eo.torque / p.J, w_idle); // open: no rev cap
    }
    tel.engine_torque = p.eo.torque;
    tel.clutch = float(clutch_share);
    tel.clutch_locked = clutch == ClutchState::Locked;
    tel.shifting = shift_lockout > 0;
    return p;
}

// [physics §2.7] the axle driveline's inputs at this point of the tick (the last tick's wheel net torques
// and the current wheel speeds; the driveline step replaces both).
model::AwdIn Vehicle::Impl::awd_input(const TickContext& t, const Powertrain& p) const {
    model::AwdIn awd;
    awd.layout = p.lay.layout;
    awd.k = p.lay.k;
    awd.tg = float(p.shaft_torque);
    awd.ff = float(axle_torque[0]);
    awd.fr = float(axle_torque[1]);
    awd.wf = float(0.5 * (omega[0] + omega[1]));
    awd.wr = float(0.5 * (omega[2] + omega[3]));
    awd.handbrake = t.in.handbrake ? 1.0f : 0.0f;
    awd.brake = t.in.brake;
    awd.steer = float(0.5 * (steer[0] + steer[1]));
    awd.g_over_mu = float(g_over_mu);
    awd.clutch = clutch == ClutchState::Open      ? 0
                 : clutch == ClutchState::Locked   ? 1
                 : clutch == ClutchState::Slipping ? 2
                                                   : 3;
    awd.reverse = gear < 0;
    return awd;
}

// [placeholder] prediction for the velocity-limited tyre force: per-wheel drive torque and effective inertia.
WheelDrive Vehicle::Impl::placeholder_drive_prediction(const TickContext& t, const Powertrain& p,
                                                       const AxleInertia& inertia) const {
    const model::AwdIn awd = awd_input(t, p);
    bool eng = centre_engaged;
    float jdf, jdr;
    model::AxleStep pr = model::axle_drive(p.lay, awd, eng, float(inertia.j_ax[0]), float(inertia.j_ax[1]),
                                           float(inertia.j_ref), jdf, jdr);
    const double drive_ax[2] = {pr.drive_f, pr.drive_r};
    const double jd_ax[2] = {jdf, jdr};
    const double jall = inertia.j_ax[0] + inertia.j_ax[1] + inertia.j_ref;
    WheelDrive wd;
    for (size_t i = 0; i < 4; ++i) {
        size_t a = i / 2;
        wd.iw[i] = (pr.common || p.lay.layout == 3) ? jall * 0.25 : jd_ax[a] * 0.5;
        wd.tdrive[i] = drive_ax[a] * 0.5;
    }
    return wd;
}

// --- brakes [physics §4.5]: pressure x per-axle torque, ABS pressure limit, handbrake ---

// Under/oversteer indicators for ASCC [physics §4.6].
// [physics §4.6] the indicators are computed for every car moving forward (u > 0);
// only the ASCC that reads them is gated (handbrake, enable).
void Vehicle::Impl::steer_indicators(const TickContext& t, const Frame& fr, const Ground& g, double& ind_u,
                                     double& ind_o) const {
    const double v_fwd = fr.v_fwd;
    if (!(v_fwd > 0)) return;
    // spec sign: steer + = left, the same sense as the yaw rate (+ = CCW); ours is + right.
    // a_y of the last lateral step (spec + = right, = our lat_g x g); the gates divide
    // by the sum of the four wheels' tyre-record base grip, 9.8 for every car (physics §6 quirk).
    constexpr double kBaseGripSum = 4 * 9.8;
    const double a_y = lat_g * model::kGravity;
    double lat_ratio = std::fabs(a_y) / kBaseGripSum;
    ind_u = model::understeer_indicator(float(v_fwd), float(-steer_avg), t.s.wheelbase, float(yaw_rate),
                                        float(yaw_acc), float(lat_ratio));
    // [physics §4.6] planar speed and the rear slip angle beta stored by the previous
    // tick's tyre phase (the aids run before this tick's tyre forces)
    double v_pl = std::hypot(vel.dot(g.axes.fwd), vel.dot(g.axes.lat));
    if (t.drv.scripted()) {
        // [physics §4.6] control source CPU car or driver kind != 0 (pit
        // auto-drive): o = 2 max(0, |s r| - |a_y|) / sum base grip when s r a_y < 0, unclamped.
        double v_tot = v_pl;
        ind_o = model::oversteer_alt(float(v_tot), float(yaw_rate), float(a_y), float(kBaseGripSum));
    } else if (t.spec_body) {
        ind_o = model::oversteer_indicator(float(v_pl), float(rear_beta_prev));
    } else { // [placeholder] the forward speed and this tick's rear patch slip angles
        double rear_b = std::max(std::fabs(alpha[2]), std::fabs(alpha[3]));
        ind_o = model::oversteer_indicator(float(v_fwd), float(rear_b));
    }
}

Brakes Vehicle::Impl::brakes(const TickContext& t, const Frame& fr, const Ground& g, const WheelMotion& wm,
                             VehicleTelemetry& tel) const {
    const CarSpec& s = t.s;
    const ControlInputs& in = t.in;
    // [physics §4.6, ai §9.1] the scripted path reads the ASCC channels of the AI's aid block (only
    // where its channel flags and gains enable them), the human path the car's own.
    const bool scripted = scripted_aids(t.drv);
    const AidsSpec block = scripted ? ascc_block(t.drv.aids) : AidsSpec{};
    const AidsSpec& ascc = scripted ? block : s.aids;
    const bool ascc_on = !in.handbrake && fr.v_fwd > 0 && (!scripted || t.drv.aids.ascc_flags != 0);
    double ind_u = 0, ind_o = 0;
    steer_indicators(t, fr, g, ind_u, ind_o);
    tel.understeer = float(ind_u);
    tel.oversteer = float(ind_o);
    Brakes b;
    for (size_t i = 0; i < 4; ++i) {
        int a = axle[i];
        const AxleSpec& as = s.axle[size_t(a)];
        double p = in.brake;
        if (dirt && a == 0) p *= model::dirt_front_brake_scale(float(steer_avg)); // [physics §10.4]
        float cap_scale = 1;
        if (ascc_on) {
            // [physics §4.6] ASCC: A / C add brake pressure without the pedal; D / E relieve it with the pedal
            // and scale the wheel's ABS target slip by the same factor.
            bool outside = (yaw_rate > 0) == (i % 2 == 1); // left turn: right wheels are outside
            p = model::ascc_pressure(ascc, a, outside, float(p), float(ind_u), float(ind_o), in.brake, in.throttle,
                                     &cap_scale);
        }
        // [physics §4.5] ABS on every wheel with p > 0 (an unloaded wheel, Fp = 0, gets the 0.001 floor; the
        // placeholder body: grounded wheels only)
        if (s.aids.abs && p > 0 && (t.spec_body || grounded[i])) {
            double gain = slip_g[i]; // [physics §4.5] the wheel's slip gain
            double lim = model::abs_pressure(float(p), float(wm.peaks[i]), as.tire_radius, as.max_brake_torque,
                                             a == 0 ? s.aids.abs_gain : s.aids.abs_gain_rear, float(gain),
                                             float(kappa[i]), tire_for(s, int(i), g.sidx[i]).opt_brake * cap_scale);
            b.abs_any |= lim < p - 1e-6;
            p = lim;
        }
        double tb = p * as.max_brake_torque;
        b.pressure[i] = p;
        tel.brake_pressure[i] = float(p);
        tel.abs_target[i] = tire_for(s, int(i), g.sidx[i]).opt_brake * cap_scale;
        if (in.handbrake && a == 1) tb += s.aids.handbrake_torque;
        b.torque[i] = tb;
        tel.brake_torque[i] = float(tb);
    }
    tel.abs_active = b.abs_any;
    return b;
}

// --- tyre forces [physics §3.1-3.3] ---

// [physics §3.8] the spec body's tyre outputs from this tick's speeds: the longitudinal parts along the heading, the
// lateral grip / drag along the velocity direction phi and the low-speed hold.
TyreForces Vehicle::Impl::tyre_forces(const TickContext& t, const Ground& g, const WheelMotion& wm,
                                      VehicleTelemetry& tel) {
    TyreForces f;
    double rear_beta_now = 0;
    for (size_t i = 0; i < 4; ++i) {
        if (!grounded[i]) continue;
        const TireSpec& ts = tire_for(t.s, int(i), g.sidx[i]);
        float peak = float(wm.peaks[i]);
        // Body-frame hub velocity (ground tangents, physics §10.1): forward uh, right lh.
        const D3 gfw = g.axes.fwd, grt = g.axes.lat;
        double uh = wm.vcs[i].dot(gfw), lh = wm.vcs[i].dot(grt);
        double phi = wm.fades[i] == 0 ? 0.0 : std::atan2(-lh, uh); // velocity direction (physics §3.2)
        double wrim = omega[i] * wm.radius[i];
        model::TyreCurves cv = model::tyre_curves(ts, float(kappa[i]));
        int st = model::slip_state(float(wm.vxs[i]), float(wrim));
        // slip sign: the force opposes the hub/wheel relative motion [interpretation of sg]
        int sg = model::slip_sign(float(wm.vxs[i]), float(wrim));
        double gain = model::slip_gain(float(uh));
        model::TyreLong tl = model::tyre_long_force(cv.c, peak, st, sg, float(gain));
        model::TyreLat tla =
            model::tyre_lateral(ts, float(-steer[i]), uh < 0, float(phi), float(wm.fades[i]), peak, cv.d);
        double hold_p = model::low_speed_hold(peak, float(wrim), float(wm.vxs[i]), float(wm.vys[i]), 1.0f, 0.0f);
        f.slip_grip[i] = tla.slip_grip;
        f.slip_drag[i] = tla.slip_drag;
        f.drive_part[i] = tl.drive_part;
        f.brake_part[i] = tl.brake_part;
        f.phi[i] = phi;
        if (i >= 2) rear_beta_now = std::max(rear_beta_now, double(std::fabs(tla.beta)));
        tel.beta[i] = tla.beta;
        f.hold[i] = hold_p;
        f.long_part[i] = double(tl.drive_part) + tl.brake_part;
        f.reaction[i] = double(tl.drive_gained) + tl.brake_gained;
        // telemetry: the total in the wheel frame
        double bfx = -tla.slip_drag * std::cos(phi) - tla.slip_grip * std::sin(phi), bfy = tla.slip_drag * std::sin(phi) - tla.slip_grip * std::cos(phi);
        double ch = std::cos(steer[i]), sh = std::sin(steer[i]); // body -> wheel (steer + right)
        f.fx[i] = bfx * ch + bfy * sh + f.long_part[i];
        f.fy[i] = -bfx * sh + bfy * ch - hold_p;
        tel.fx[i] = float(f.fx[i]);
        tel.fy[i] = float(f.fy[i]);
        tel.long_force[i] = float(f.long_part[i]);
        tel.slip_grip[i] = tla.slip_grip;
        tel.slip_drag[i] = tla.slip_drag;
        tel.peak_force[i] = float(peak);
        tel.slip_gain[i] = float(slip_g[i]);
    }
    rear_beta_prev = rear_beta_now;
    return f;
}

// [placeholder] Velocity-limited tyre forces at the contact points, applied to the rigid body (keeps the 60 Hz
// step stable at low speed): the slip ratio is evaluated at the end-of-tick wheel and road speeds predicted from
// drive, brakes and the non-tyre forces, and the force never exceeds what stops that relative motion in one tick.
TyreForces Vehicle::Impl::placeholder_tyre_forces(const TickContext& t, const Frame& fr, const Ground& g,
                                                  const WheelMotion& wm, const Powertrain& p, const AxleInertia& inertia,
                                                  const Brakes& br, BodyForces& body, VehicleTelemetry& tel) const {
    const CarSpec& s = t.s;
    const double h = t.h, M = s.mass;
    const double m_eff = M * 0.25;
    const WheelDrive wd = placeholder_drive_prediction(t, p, inertia);
    // Non-tyre acceleration so far: predicts the contact velocities.
    const D3 a0 = body.force * (1.0 / M);
    TyreForces f;
    for (size_t i = 0; i < 4; ++i) {
        if (!grounded[i]) continue;
        const TireSpec& ts = tire_for(s, int(i), g.sidx[i]);
        float peak = float(wm.peaks[i]);
        D3 vp = wm.vcs[i] + a0 * h;
        double w_pred = coulomb(omega[i] + h * wd.tdrive[i] / wd.iw[i], h * br.torque[i] / wd.iw[i]);
        double v_long = vp.dot(wm.wf[i]);
        double rel_long = v_long - w_pred * wm.radius[i];
        model::TireIn ti;
        ti.vx = float(wm.vxs[i]);
        ti.vy = float(wm.vys[i]);
        ti.slip_ratio = model::slip_ratio(float(v_long), float(w_pred * wm.radius[i]), float(vp.dot(fr.fwd_h)));
        ti.rel = float(rel_long);
        ti.peak = peak;
        ti.scale = float(wm.fades[i]);
        model::TireOut to = model::tire_force(ts, ti);
        double cap_long = std::fabs(rel_long) / (h * (1.0 / m_eff + wm.radius[i] * wm.radius[i] / wd.iw[i]));
        if (w_pred == 0 && br.torque[i] > 0) {
            // Wheel held by the brake: it can react up to the unused brake torque.
            double spare = std::max(0.0, br.torque[i] - std::fabs(wd.tdrive[i]) - std::fabs(omega[i]) * wd.iw[i] / h);
            cap_long = std::max(cap_long, std::min(m_eff * std::fabs(rel_long) / h, spare / wm.radius[i]));
        }
        f.fx[i] = clampd(to.fx, -cap_long, cap_long);
        // [physics §4.1] lateral: the tyre force (faded out below 10 km/h,
        // physics §3.2) plus the low-speed hold, a sideways damper below 5 km/h; no velocity cap.
        // The wheel frame stands in for the body frame of the hold's window tests.
        double hold =
            model::low_speed_hold(peak, float(omega[i] * wm.radius[i]), float(wm.vxs[i]), float(wm.vys[i]), 1.0f, 0.0f);
        f.fy[i] = double(to.fy) - hold;
        const D3 fw = wm.wf[i] * f.fx[i] + wm.wr[i] * f.fy[i];
        if (s.axle[0].susp.valid) { // front-axle spec block only
            // [physics §4.1/4.2] tyre forces drive the planar motion and yaw only.
            body.force += fw;
            D3 arm = contact[i] - pos;
            body.torque.y += arm.z * fw.x - arm.x * fw.z;
        } else {
            body.apply(fw, contact[i], pos);
        }
        tel.fx[i] = float(f.fx[i]);
        tel.fy[i] = float(f.fy[i]);
    }
    f.reaction = f.fx;
    return f;
}

// --- wheel spin: drive, tyre reaction, Coulomb brakes [physics §2.7] ---
DriveStep Vehicle::Impl::wheel_spin(const TickContext& t, const Frame& fr, const WheelMotion& wm,
                                    const TyreForces& tyre, const Brakes& br, const Powertrain& p,
                                    const AxleInertia& inertia, VehicleTelemetry& tel) {
    const CarSpec& s = t.s;
    const DrivetrainSpec& d = s.drivetrain;
    const double h = t.h;
    const std::array<double, 4>& tbrake = br.torque;
    const std::array<double, 4> w_old = omega;
    // Wheel net torque: -r Fx - sgn(w) brake (none at w == 0 exactly), plus torque vectoring.
    std::array<double, 4> tn{};
    for (size_t i = 0; i < 4; ++i)
        tn[i] = -tyre.reaction[i] * wm.radius[i] - (omega[i] == 0 ? 0.0 : std::copysign(tbrake[i], omega[i]));
    for (int a = 0; a < 2; ++a) {
        int type = d.lsd_type[size_t(a)];
        if (type != 6 && type != 7) continue;
        size_t l = size_t(2 * a), r = l + 1;
        float tl = float(tn[l]), tr = float(tn[r]);
        model::torque_vectoring(type, d.lsd_initial[size_t(a)], float(-steer_avg), float(fr.v_fwd),
                                float(omega[l] * wm.radius[l]), float(omega[r] * wm.radius[r]), tl, tr);
        tn[l] = tl;
        tn[r] = tr;
    }
    const double Ff = tn[0] + tn[1], Fr = tn[2] + tn[3];
    model::AwdIn awd = awd_input(t, p);
    awd.ff = float(Ff);
    awd.fr = float(Fr);
    awd.wf = float(0.5 * (w_old[0] + w_old[1]));
    awd.wr = float(0.5 * (w_old[2] + w_old[3]));
    model::AxleStep st = model::axle_step(p.lay, awd, centre_engaged, float(inertia.j_ax[0]), float(inertia.j_ax[1]),
                                          float(inertia.j_ref), tbrake[0] + tbrake[1] > 0, tbrake[2] + tbrake[3] > 0,
                                          d.lsd_type[0] == 0, d.lsd_type[1] == 0, float(h));
    DriveStep out;
    out.drive_ax = {st.drive_f, st.drive_r};
    const double w_front = st.wf, w_rear = st.wr;
    // wheels per axle
    const double w_axle[2] = {w_front, w_rear};
    for (int a = 0; a < 2; ++a) {
        size_t l = size_t(2 * a), r = l + 1;
        int type = d.lsd_type[size_t(a)] < 0 ? 2 : d.lsd_type[size_t(a)];
        float wl, wrr;
        if (type == 0) { // f: each wheel alone, clamped at zero when braked
            wl = float(w_old[l] + tn[l] / inertia.ia[size_t(a)] * h);
            wrr = float(w_old[r] + tn[r] / inertia.ia[size_t(a)] * h);
        } else {
            wl = wrr = float(w_axle[a]);
        }
        // [physics §2.7] the differential works on the peripheral speeds (m/s) with r / Ia: the
        // viscous transfer gain x min(dw^2, initial^2) takes dw in m/s [verified on an airborne
        // front wheel: in rad/s the transfer came out 1/r^2 (about 11x) too strong]
        const double rr_ax = s.axle[size_t(a)].tire_radius;
        float wl_m = float(double(wl) * rr_ax), wr_m = float(double(wrr) * rr_ax);
        model::diff_step_prev(type, d.lsd_initial[size_t(a)], d.lsd_accel[size_t(a)], d.lsd_decel[size_t(a)],
                              float(out.drive_ax[size_t(a)]), float(inertia.ia[size_t(a)] / rr_ax), float(h),
                              float(tn[l]), float(tn[r]), float((w_old[l] - w_old[r]) * rr_ax), wl_m, wr_m,
                              diff[size_t(a)], float(tbrake[l]), float(tbrake[r]), float(w_old[l] * rr_ax),
                              float(w_old[r] * rr_ax));
        wl = float(double(wl_m) / rr_ax);
        wrr = float(double(wr_m) / rr_ax);
        omega[l] = wl;
        omega[r] = wrr;
        tel.diff_locked[size_t(a)] = diff[size_t(a)].locked;
        tel.diff_half[size_t(a)] = diff[size_t(a)].half;
    }
    axle_torque = {Ff, Fr};
    out.w_side_after = layout_axle_speed(p.lay, w_front, w_rear);
    for (size_t i = 0; i < 4; ++i) tel.drive_torque[i] = float(out.drive_ax[i / 2] * 0.5);
    for (size_t i = 0; i < 4; ++i) spin[i] = std::fmod(spin[i] + omega[i] * h, model::kTwoPi);
    return out;
}

// [physics §10.2] a grounded, unbraked, undriven (and uncoupled) wheel is locked to
// the ground: peripheral speed = the hub speed along its heading.
// The flags (physics §3.8: load 0, brake, driven/coupled bits only while the clutch is engaged) are this tick's; the
// overwrite uses the hub speeds at the end of the tick (lock_free_wheels; the placeholder body applies it at the
// start of the next tick, before the slip ratio).
void Vehicle::Impl::flag_free_wheels(const model::AwdLayout& lay, double drive_f,
                                     const std::array<double, 4>& tyre_load, const std::array<double, 4>& tbrake) {
    const bool engaged_now = clutch != ClutchState::Open;
    for (size_t i = 0; i < 4; ++i) {
        const int a = int(i / 2);
        bool both = lay.layout == 2 || lay.layout == 3 || lay.layout == 7;
        bool driven_w = engaged_now && (both || a == (lay.layout == 1 ? 0 : 1));
        if (lay.layout >= 4 && a == 0 && (centre_engaged || drive_f != 0)) driven_w = true;
        free_roll[i] = tyre_load[i] != 0 && tbrake[i] == 0 && !driven_w;
    }
}

// --- tyre smoke, wear and brake heat [physics §8.2-8.5, physics §8.7] ---
void Vehicle::Impl::tyre_wear_and_heat(const TickContext& t, const Ground& g, const WheelMotion& wm, const Brakes& br,
                                       VehicleTelemetry& tel) {
    const CarSpec& s = t.s;
    const double h = t.h;
    // [physics §8.3] slip intensity, after wheel_spin (physics §1.2); airborne wheels keep it 5 more steps.
    for (size_t i = 0; i < 4; ++i) {
        if (!grounded[i]) {
            if (++air_steps[i] > 5) {
                smoke[i] = smoke_emit[i] = 0;
                skid_pitch[i] = 1;
            }
            continue;
        }
        air_steps[i] = 0;
        float ramp = std::min(1.0f, 0.02f * float(wm.vhub[i])); // wheel speed ramp (physics §3.6 / physics §8.3)
        smoke[i] = model::slip_intensity(g.sidx[i], float(wm.vys[i]), float(kappa[i]), float(omega[i] * wm.radius[i]),
                                         float(load[i]), ramp);
        // [physics §8.3] the second curve set from the same inputs: the tyre sounds' pitch (sound §7.6).
        skid_pitch[i] =
            model::skid_pitch(g.sidx[i], float(wm.vys[i]), float(kappa[i]), float(omega[i] * wm.radius[i]), ramp);
        // [physics §8.3b] smoke-emission intensity: slip speed = sgn(hub) (hub - rim).
        double slip_v = wm.vxs[i] - omega[i] * wm.radius[i];
        smoke_emit[i] =
            model::smoke_emission(g.sidx[i], float(wm.vys[i]), float(wm.vxs[i] < 0 ? -slip_v : slip_v), ramp);
    }
    // [physics §8.2] wear accumulation from that slip intensity: not on the pit surface, not on kerb surface 1.
    if (wear_p.on() && !pit_surface)
        for (size_t i = 0; i < 4; ++i) {
            if (load[i] == 0 || g.sidx[i] == 1) continue;
            const TireSpec& tw = tire_for(s, int(i), g.sidx[i]);
            wear[i] += model::wear_increment(smoke[i], wear_p.slip_cap, tw.wear_rate, float(load[i]),
                                             s.axle[size_t(axle[i])].tire_width, float(h));
        }
    // [physics §8.5] brake heat (visual only).
    for (size_t i = 0; i < 4; ++i) {
        const AxleSpec& as = s.axle[size_t(axle[i])];
        brake_heat[i] = model::brake_heat_step(brake_heat[i], as.max_brake_torque, as.tire_radius,
                                               float(br.pressure[i]), float(omega[i] * wm.radius[i]), float(h));
    }
    // [physics §8.7] tyres worn flag.
    tel.tires_worn = false;
    for (size_t i = 0; i < 4; ++i) {
        if (wear_p.on() && wear[i] >= 0 && wear_grip[i] < wear_p.worn) tel.tires_worn = true;
        tel.tire_wear[i] = wear[i];
        tel.wear_grip[i] = wear_grip[i];
        tel.wear_level[i] = wear_level[i];
        tel.smoke[i] = smoke[i];
        tel.smoke_emit[i] = smoke_emit[i];
        tel.skid_pitch[i] = skid_pitch[i];
        tel.brake_heat[i] = brake_heat[i];
    }
}

// Slipping clutch: engine and wheel-side speeds crossed -> locked, held 12 steps [physics §2.4]
// (in either direction, regardless of idle). A locked clutch takes the engine speed from the axles.
void Vehicle::Impl::clutch_after_driveline(const TickContext& t, const Frame& fr, const Powertrain& p,
                                           double w_side_after, VehicleTelemetry& tel) {
    const CarSpec& s = t.s;
    double w_in_after = w_side_after * p.G;
    bool slipping = clutch == ClutchState::Slipping || clutch == ClutchState::Shift;
    bool crossed = slipping && p.slip_dw * (omega_e - w_in_after) <= 0;
    const ClutchState before = clutch;
    clutch = ClutchState(model::clutch_after(int(clutch), crossed, t.in.brake, float(omega_e), s.engine.idle_rpm,
                                             float(fr.v_fwd), float(w_in_after)));
    if (s.engine.electric) clutch = before, crossed = false; // direct drive never slips
    if (crossed) shift_lockout = 12; // 2/3 -> 1 starts the 12-tick shift lockout
    tel.clutch_locked = clutch == ClutchState::Locked; // link state at the end of the clutch step
    if (clutch == ClutchState::Locked) omega_e = std::clamp(w_in_after, 0.0, p.w_cap);
}

// [placeholder] penalty contact of the hull points with the ground (see hull_contact_on()).
void Vehicle::Impl::placeholder_hull_contact(const TickContext& t, const TrackQuery& track, const Frame& fr,
                                             BodyForces& body) const {
    const double M = t.s.mass, h = t.h;
    const double m_eff = M * 0.25;
    const Axes& A = fr.body;
    double k = 80.0 * M, c = 2 * 0.7 * std::sqrt(k * M / 4);
    for (const D3& p_local : hull) {
        D3 p = pos + A.to_world(p_local - cg_local);
        SurfaceHit hit = track.probe(float(p.x), float(p.z), float(p.y + 0.5));
        if (!hit.hit) continue;
        D3 n = D3(hit.normal).normalized();
        double depth = n.dot(D3{p.x, hit.height, p.z} - p);
        if (depth <= 0) continue;
        D3 vp = vel + fr.omega_w.cross(p - pos);
        double vn = vp.dot(n);
        double fn = std::max(0.0, k * depth - c * vn);
        D3 vt = vp - n * vn;
        double vtl = vt.length();
        D3 ff = vtl > 1e-4 ? vt * (-std::min(0.6 * fn, m_eff * vtl / h) / vtl) : D3{};
        body.apply(n * fn + ff, p, pos);
    }
}

// --- integration [physics §1.1: explicit Euler, rate then value] ---

// [physics §4.1] the spec body's planar motion (phases 7-9): forward step, lateral force and yaw, stop hold.
// Gravity and heave are the attitude step's; of `body` only the GT3R_HULL_CONTACT comparison contact acts here.
PlanarAccel Vehicle::Impl::planar_step(const TickContext& t, const Frame& fr, const Ground& g, const WheelMotion& wm,
                                       const TyreForces& tyre, const Brakes& br, const AxleInertia& inertia,
                                       const model::AeroOut& aero, const std::array<double, 4>& tyre_load,
                                       BodyForces& body) {
    const CarSpec& s = t.s;
    const double M = s.mass, h = t.h;
    body.force.y += M * model::kGravity;
    vel += body.force * (h / M);
    const double m_yaw = body.torque.y;
    // [physics §10.6] all four wheels airborne: no drag (nor engine/brake force) acts in the air, yaw is frozen.
    const bool all_airborne = !grounded[0] && !grounded[1] && !grounded[2] && !grounded[3];
    // Body frame (forward, right); the spec's steer sign is + = left.
    model::PlanarWheel pw[4];
    float grip[4], fade4[4];
    bool any_brake = false;
    for (size_t i = 0; i < 4; ++i) {
        model::PlanarWheel& w = pw[i];
        double d_spec = -steer[i];
        w.k = float(std::cos(d_spec));
        w.s = float(std::sin(d_spec));
        const int a = axle[i];
        // [physics §3.8] wheel flag byte: airborne, braked, driven / coupled (the last only while the clutch is
        // engaged). [physics §4.1] the forward step runs before the driveline adds the driven /
        // coupled bits: only airborne or braked wheels leave the effective mass.
        w.flagged = tyre_load[i] == 0 || br.torque[i] > 0; // the tyre phase's load (previous tick's)
        w.ia_r2 = float(inertia.ia[size_t(a)] / (wm.radius[i] * wm.radius[i]));
        any_brake = any_brake || br.torque[i] > 0;
        fade4[i] = float(wm.fades[i]);
        const TireSpec& tg = tire_for(s, int(i), g.sidx[i]);
        int sg = std::clamp(g.sidx[i], 0, 15);
        grip[i] = 9.8f * (tg.surface_mu[size_t(sg)] != 0 ? tg.surface_mu[size_t(sg)] : tg.mu * model::surface_mu(sg));
        if (!grounded[i]) continue;
        // [physics §3.8] slip_grip/slip_drag in the velocity-direction frame, drive_part/brake_part along the heading
        w.a = float(tyre.phi[i]);
        w.slip_grip = float(tyre.slip_grip[i]);
        w.slip_drag = float(tyre.slip_drag[i]);
        w.drive_part = float(tyre.drive_part[i]);
        w.brake_part = float(tyre.brake_part[i]);
        w.hold = float(tyre.hold[i]);
    }
    double u = vel.dot(g.axes.fwd); // physics §10.1 ground-frame forward speed
    double rr_sum = 0;
    for (size_t i = 0; i < 4; ++i)
        if (grounded[i]) rr_sum += tyre_load[i] * model::surface_rolling(g.sidx[i]); // the load lags a tick
    double rolling = -(u > 0 ? 1 : u < 0 ? -1 : 0) * model::rolling_curve(float(std::fabs(u))) * rr_sum;
    double tf = 0.5 * s.axle[0].track, tr = 0.5 * s.axle[1].track;
    double a_len = s.wheelbase * (1 - s.front_weight), b_len = s.wheelbase * s.front_weight;
    float X[4] = {float(-tf), float(tf), float(-tr), float(tr)}, Y[4] = {float(a_len), float(a_len), float(-b_len), float(-b_len)};
    const Axes& A = fr.body;
    D3 fh{A.fwd.x, 0, A.fwd.z}, lh{A.lat.x, 0, A.lat.z};
    PlanarAccel out;
    // [physics §4.1] the forward step runs airborne too; only its a_x is read then (the flaps).
    const model::ForwardStep fs = model::forward_step(pw, float(u), float(M), float(rolling), 0.0f, float(aero.drag), float(h));
    long_accel = fs.ax;
    if (!all_airborne) {
        // the forward step's longitudinal sum: rolling + sum cos d drive_part plus the tyre
        // reaction sum (k brake_part - sin a slip_grip - cos a slip_drag)
        double lsum = rolling;
        for (const model::PlanarWheel& w : pw)
            lsum += double(w.k) * (double(w.drive_part) + w.brake_part) - std::sin(double(w.a)) * w.slip_grip - std::cos(double(w.a)) * w.slip_drag;
        long_force_sum = lsum;
        model::LateralYaw ly = model::lateral_yaw(pw, X, Y, float(M), float(i_yaw), 0.0f, grip);
        out.ax = fs.ax;
        out.ay = ly.ay;
        g_over_mu = ly.g_over_mu;
        yaw_acc = double(ly.yaw_acc) + m_yaw / i_yaw;
        yaw_rate = clampd(yaw_rate + yaw_acc * h, -model::kTwoPi, model::kTwoPi);
        // [physics §4.1/physics §9.3] the race hold (the start countdown) or driver kind 8 hold whatever
        // the speed. A hold zeroes the world x/y velocity, the yaw rate and the longitudinal force sum
        // the load transfer reads; the vertical velocity is kept.
        if (model::stop_hold(float(u), fade4, any_brake, t.drv.race_hold || t.drv.kind == 8)) {
            vel.x = vel.z = 0;
            yaw_rate = 0;
            long_force_sum = 0;
        } else {
            vel += fh * (double(fs.ax) * h) + lh * (double(ly.ay) * h);
        }
    }
    if (spawn > 0) { // the spawn/stop counter zeroes the world x/y velocity and the yaw rate
        vel.x = vel.z = 0;
        yaw_rate = 0;
    }
    return out;
}

// [physics §1.2] phase 10 (position and heading by velocity and yaw rate x dt), then phase 12 at the moved hubs.
Ground Vehicle::Impl::advance_and_query_ground(const TickContext& t, const TrackQuery& track) {
    const double h = t.h;
    pos += vel * h;
    heading += yaw_rate * h;
    Axes Am = make_axes(ax, ay, heading);
    D3 omega_m = D3{0, yaw_rate, 0} + Am.lat * ax_rate + Am.fwd * ay_rate;
    return suspension_step(t, track, Am, omega_m);
}

// [physics §4.2] body force += lateral factor x (-a_y) on the left wheels, -= on the right; += longitudinal factor x
// (force sum - external force, always 0) on the front, -= on the rear; the full value on each grounded wheel.
// a_y and the force sum are THIS tick's lateral step / tyre phase outputs.
void Vehicle::Impl::load_transfer(const CarSpec& s, double a_y, Ground& late) const {
    const double M = s.mass;
    double ms = M - 2.0 * (s.axle[0].susp.unsprung + s.axle[1].susp.unsprung);
    double X = s.cg_height;
    double t_mean = 0.5 * (s.axle[0].track + s.axle[1].track);
    double d_long = 2.0 * (ms / M) * X / s.wheelbase * long_force_sum;
    double d_lat = ms * X * a_y / t_mean; // right accel loads the left
    for (size_t i = 0; i < 4; ++i) {
        if (!grounded[i]) continue;
        late.body_f[i] += (i < 2 ? 0.5 : -0.5) * d_long + (i % 2 == 0 ? -1.0 : 1.0) * d_lat;
    }
}

// [physics §4.3] pitch/roll from the per-wheel body forces and dampers, the longitudinal A/B terms; heave
// along the body up axis, gravity in world Y; the spawn hold's scripted pose and the bump-stop nudge. `moved`:
// phase 10 already moved the body.
void Vehicle::Impl::attitude_step(const TickContext& t, const Frame& fr, const model::AeroOut& aero,
                                  const TyreForces& tyre, const Ground& late, bool moved) {
    const CarSpec& s = t.s;
    const double M = s.mass, h = t.h;
    const Axes& A = fr.body;
    float f[4], dd[4], cd[4], fxl[4];
    for (size_t i = 0; i < 4; ++i) {
        double lift = i < 2 ? aero.lift_front : aero.lift_rear;
        f[i] = float(late.body_f[i] - late.damper_f[i] - 0.5 * lift);
        dd[i] = float(late.damper_f[i]);
        cd[i] = float(std::cos(steer[i]));
        // [physics §4.3] q_i = cos d x (drive + brake parts G = drive_part + brake_part) x width: the lateral outputs
        // slip_grip / slip_drag do not enter, even projected on a steered wheel's axis
        fxl[i] = float(tyre.long_part[i]);
    }
    const double tp = model::kTwoPi;
    double a_len = s.wheelbase * (1 - s.front_weight), b_len = s.wheelbase * s.front_weight;
    double Y = s.yaw_inertia, Z = s.yaw_inertia * 0.5;
    float width[2] = {s.axle[0].tire_width, s.axle[1].tire_width};
    model::PitchTerms pt = model::pitch_terms(cd, fxl, width, float(Y), float(M), float(a_len), float(b_len));
    // Spawn hold [physics §4.3]: while the counter runs, a car at or below
    // ground + f(n) snaps there flat and at rest vertically, its heading eases to
    // the grid heading (<= 30 deg/s), and the attitude update is skipped.
    if (spawn > 0) {
        int n = spawn--;
        if (n == 60 && wear_p.on()) fit_fresh_tires(); // [physics §8.6] tyres replaced at counter 60
        double target = late.gf.height + model::spawn_height_offset(n);
        D3 origin = pos - A.to_world(cg_local);
        if (origin.y <= target) {
            ax = ay = ax_rate = ay_rate = 0;
            Axes Af = make_axes(0, 0, heading);
            D3 o{origin.x, target, origin.z};
            pos = o + Af.to_world(cg_local);
            vel.y = 0;
            if (n >= 30 && n < 270) {
                double dh = std::remainder(spawn_heading - heading, model::kTwoPi);
                double step = 30.0 * model::kPi / 180.0 * h;
                heading += std::fabs(dh) <= step ? dh : std::copysign(step, dh);
            }
            pos.x += vel.x * h;
            pos.z += vel.z * h;
            heading += yaw_rate * h;
            return; // attitude update skipped this step
        }
    }
    float pitch = float(ax), prate = float(ax_rate), roll = float(ay), rrate = float(ay_rate);
    float heave = model::attitude_step(f, dd, float(a_len * tp / Y), float(b_len * tp / Y),
                                       float(0.5 * s.axle[0].track * tp / Z), float(0.5 * s.axle[1].track * tp / Z),
                                       pt, float(h), pitch, prate, roll, rrate);
    ax = pitch;
    ax_rate = prate;
    ay = roll;
    ay_rate = rrate;
    double ms = M - 2.0 * (s.axle[0].susp.unsprung + s.axle[1].susp.unsprung);
    Axes An = make_axes(ax, ay, heading);
    vel += An.up * (heave / ms * h);
    vel.y -= model::kGravity * h;
    if (!moved) { // [physics §4.3] heave changes the velocity only; position moves in phase 10
        pos += vel * h;
        heading += yaw_rate * h;
    }
    // Bump-stop attitude nudge [physics §4.3].
    float pen[4];
    for (size_t i = 0; i < 4; ++i) {
        const SuspensionSpec& sp = s.axle[size_t(axle[i])].susp;
        pen[i] = grounded[i] ? std::max(0.0f, model::bump_limit(sp, float(xs[i])) - float(x[i])) : 0.0f;
    }
    model::Nudge nd;
    if (model::bump_nudge(pen, 1 - s.front_weight, 0.5f * (s.axle[0].track + s.axle[1].track), s.wheelbase, nd)) {
        ay += nd.roll;
        ax += nd.pitch;
        pos.y += nd.height;
        // Downward speed limited to the velocity's component along the ground plane.
        D3 n = late.normal[0];
        D3 vg = vel - n * vel.dot(n);
        vel.y = std::max(vel.y, std::min(0.0, vg.y));
    }
}

// [placeholder] the rigid body: forces and torques about the CoG, rolling resistance and the stop clamp, yaw and
// a pitch / roll attitude from the torques; the spawn / stop counter runs without the scripted pose.
void Vehicle::Impl::placeholder_integrate(const TickContext& t, const Frame& fr, const Ground& g, const WheelMotion& wm,
                                          const Brakes& br, const BodyForces& body) {
    const double M = t.s.mass, h = t.h;
    const D3& fwd_h = fr.fwd_h;
    const Axes& A = fr.body;
    double u_before = vel.dot(fwd_h);
    vel += body.force * (h / M);
    // Rolling resistance on the forward speed, with static hold [physics §3.5].
    {
        double sum = 0;
        for (size_t i = 0; i < 4; ++i)
            if (grounded[i]) sum += load[i] * model::surface_rolling(g.sidx[i]);
        double frr = model::rolling_curve(float(u_before)) * sum;
        double u = vel.dot(fwd_h);
        double du = -std::copysign(frr, u) / M * h;
        if (u != 0 && (u + du) * u < 0) du = -u;
        vel += fwd_h * du;
    }
    // Stop clamp [physics §4.1, condition interpreted]: below 1 km/h with the brake
    // applied and no drive, the planar velocities and yaw rate are zeroed.
    {
        // [physics §4.1] |v| < 1 km/h, every wheel's fade weight 0 and a
        // non-zero brake torque on at least one wheel.
        D3 v_planar{vel.x, 0, vel.z};
        bool fades_zero = true, braking = false;
        for (size_t i = 0; i < 4; ++i) {
            fades_zero = fades_zero && wm.fades[i] == 0;
            braking = braking || br.torque[i] > 0;
        }
        if (v_planar.length() < 1 / 3.6 && fades_zero && braking) {
            vel.x = vel.z = 0;
            yaw_rate = 0;
        }
    }
    // Planar yaw and attitude (pitch about the lateral axis, roll about the forward axis).
    double m_yaw = body.torque.y;
    double m_pitch = body.torque.dot(A.lat), m_roll = body.torque.dot(A.fwd);
    yaw_rate = clampd(yaw_rate + m_yaw / i_yaw * h, -model::kTwoPi, model::kTwoPi);
    if (spawn > 0) {
        // Cars without the spec suspension block: the counter (gear 1, clutch open)
        // runs without the scripted pose.
        int n = spawn--;
        if (n == 60 && wear_p.on()) fit_fresh_tires();
    }
    ax_rate += m_pitch / i_pitch * h;
    ay_rate += m_roll / i_roll * h;
    pos += vel * h;
    heading += yaw_rate * h;
    ax += ax_rate * h;
    ay += ay_rate * h;
    if (std::fabs(ax) > kMaxAttitude) { ax = std::copysign(kMaxAttitude, ax); ax_rate = 0; }
    if (std::fabs(ay) > kMaxAttitude) { ay = std::copysign(kMaxAttitude, ay); ay_rate = 0; }
}

// --- end of the tick ------------------------------------------------------------------------

// Walls [physics §5.3]: the swept octagon against the wall segments (2 iterations, impulse e = 0.05, friction
// mu = 0.5 without torque, collision yaw inertia 10 x I_z, push-out) and the stuck assist; without segments,
// spheres around the body against WorldQuery::wall_contact [placeholder geometry].
void Vehicle::Impl::walls(const TickContext& t, const WorldQuery& world, const D3& start_pos, double start_heading,
                          VehicleTelemetry& tel) {
    const double h = t.h;
    double reach = oct_radius + std::sqrt(vel.x * vel.x + vel.z * vel.z) * h + 1.0;
    segs.clear();
    if (world.wall_segments(pos.f(), float(reach + 8.0), segs)) {
        // [physics §5.3] swept octagon vs wall segments, 2 iterations, push-out.
        collide::Body b = collision_body(t.s.mass);
        std::vector<collide::WallHit> wall_hits =
            collide::collide_walls(b, {start_pos.x, start_pos.z}, start_heading, segs, h, 2, &wall_contacts);
        bool hit_any = false;
        for (const auto& w : wall_hits) {
            hit_any |= !w.sensor;
            if (!w.sensor) tel.hit_sides |= collide::point_side(w.body_point); // [ai §7] the AI's recovery flags
        }
        set_collision_body(b);
        if (hit_any) {
            // Stuck assist: pinned against a wall, the steering still turns the car.
            if (std::fabs(vel.x) < 1 && std::fabs(vel.z) < 1) yaw_rate += -t.in.steer * 20.0 * h;
            tel.wall_impulse = 0;
            for (const auto& w : wall_hits) tel.wall_impulse += float(w.impulse);
        }
    } else {
        // Fallback: spheres around the body against wall_contact() [placeholder geometry].
        wall_contacts.clear();
        Axes A2 = make_axes(ax, ay, heading);
        const double i_coll = 10.0 * i_yaw;
        bool hit_any = false;
        for (const D3& p_local : wall_pts) {
            D3 p = pos + A2.to_world(p_local - cg_local);
            Vec3 n_f;
            float pen = 0;
            if (!world.wall_contact(p.f(), float(wall_radius), n_f, pen) || pen <= 0) continue;
            hit_any = true;
            D3 n = D3{n_f.x, 0, n_f.z}.normalized();
            pos += n * double(pen);
            D3 r = p - n * wall_radius - pos;
            collide::Body b = collision_body(t.s.mass);
            b.inv_inertia = 1.0 / i_coll;
            tel.wall_impulse += float(collide::wall_impulse(b, {r.x, r.z}, {n.x, n.z}));
            set_collision_body(b);
        }
        if (hit_any && std::fabs(vel.x) < 1 && std::fabs(vel.z) < 1) yaw_rate += -t.in.steer * 20.0 * h;
    }
    yaw_rate = clampd(yaw_rate, -62.83, 62.83);
}

// Body-frame acceleration for telemetry, and the lateral g the next tick's TCS / ASCC read.
void Vehicle::Impl::record_acceleration(const TickContext& t, const Frame& fr, const BodyForces& body,
                                        const PlanarAccel& planar, VehicleTelemetry& tel) {
    const double M = t.s.mass;
    tel.accel_local = fr.body.to_car(body.force * (1.0 / M) + D3{0, model::kGravity, 0}).f();
    if (t.spec_body) { // the planar step's accelerations (the tyre forces are not in `force`)
        // [physics §4.1] a_y of the lateral step alone, (sum Fa - ext) / mass (0 when airborne).
        // The body's rolled lateral axis would add g sin(roll) from `force` (about +3% in a steady
        // corner); the original game's value, and the load transfer / TCS / indicators reading it, have none.
        tel.accel_local.x = float(planar.ay);
        tel.accel_local.z -= float(planar.ax);
    }
    lat_g = tel.accel_local.x / model::kGravity;
}

// [physics §10.1/physics §10.2, end of phase 12] a free wheel (grounded, unbraked, not driven or
// coupled this tick) is locked to the ground: wheel speed = cos d (u + X r) - sin d (l - Y r), with u / l the
// world velocity on the ground-frame tangents, half-track levers X = (-tf, +tf, -tr, +tr), axle
// distances Y = (a, a, -b, -b) and d = this tick's wheel steer angle (+ = left). The next tick's slip
// ratio reads it.
void Vehicle::Impl::lock_free_wheels(const CarSpec& s) {
    const Axes Ag = make_axes(late_gf.pitch, late_gf.roll, heading);
    for (size_t i = 0; i < 4; ++i) {
        if (!free_roll[i]) continue;
        double uh, lh;
        hub_velocity(s, i, Ag, uh, lh); // the same expression the next tick's slip ratio uses, so it reads exactly 0
        omega[i] = along_heading(uh, lh, steer[i]) / s.axle[size_t(axle[i])].tire_radius;
    }
}

// [physics §2.4/physics §2.6] the shift decision uses this tick's final state and the gear change
// takes effect at the END of the tick, after the driveline step: the shift tick itself
// still transmits through the old gear / clutch state (called before the NaN guard).
void Vehicle::Impl::select_gear(const TickContext& t, double v_fwd_sel) {
    const CarSpec& s = t.s;
    const DrivetrainSpec& d = s.drivetrain;
    const ControlInputs& in = t.in;
    const int nf = int(s.gearbox.ratios.size());
    bool slow = std::fabs(v_fwd_sel) < 1.5;
    if (shift_lockout > 0) {
        --shift_lockout;
        return;
    }
    // Manual requests (and R/N selection at a standstill in both modes).
    if (in.shift_up) {
        if (gear < 0) set_gear(in.automatic ? 1 : 0);
        else if (gear == 0) set_gear(1);
        else if (!in.automatic && gear < nf) set_gear(gear + 1);
    } else if (in.shift_down) {
        if (gear == 1 && slow) set_gear(in.automatic ? -1 : 0);
        else if (gear == 0 && slow) set_gear(-1);
        else if (!in.automatic && gear > 1) set_gear(gear - 1);
    } else if (gear != 0) {
        bool veto = false;
        // [physics §2.6] the driven axle only: L1/L2 front, L0/L4-6 rear, L3/L7 all four
        const model::AwdLayout vl = model::awd_layout(int(d.drive), d.awd_system, d.awd_share_byte, d.awd_strength_byte);
        for (int i = 0; i < 4; ++i) {
            bool veto_axle = vl.layout == 3 || vl.layout == 7 || (i < 2) == (vl.layout == 1 || vl.layout == 2);
            // [physics §2.6] optimum x the wheel's slip gain
            if (veto_axle &&
                (!grounded[size_t(i)] || kappa[size_t(i)] < s.tire[size_t(i / 2)].opt_drive * slip_g[size_t(i)]))
                veto = true;
        }
        model::ShiftIn si;
        si.gear = gear;
        // rpm s16 = max(int(w 60 / 2 pi), idle - 1) of this tick's final engine speed [physics §2.4/2.6]
        si.rpm = float(std::max(int(omega_e * model::kRadSToRpm), int(s.engine.idle_rpm) - 1));
        si.speed = float(v_fwd_sel);
        si.throttle = t.raw.throttle; // the accelerator (in AT reverse `in` has the pedals swapped)
        si.brake = t.raw.brake;
        si.clutch_locked = clutch == ClutchState::Locked;
        si.automatic = in.automatic;
        si.reverse_request = t.drv.reverse_request;
        si.traction_veto = veto;
        si.driven_radius = float(driven_radius(s));
        // [physics §2.6] automatic shifts only while the clutch is locked
        // (shift_logic); open or slipping, only the reverse/first/over-rev rules act.
        int to = model::shift_logic(s.engine, s.gearbox, si);
        // Reversing into a forward gear only when asked (keeps manual reverse usable).
        if (to == -1 && gear >= 1 && !in.automatic) to = gear;
        if (to == 1 && gear == -1 && !in.automatic) to = gear;
        set_gear(to);
    }
}

bool Vehicle::Impl::state_finite() const {
    bool ok = pos.finite() && vel.finite() && std::isfinite(heading) && std::isfinite(ax) && std::isfinite(ay) &&
              std::isfinite(yaw_rate) && std::isfinite(omega_e);
    for (size_t i = 0; i < 4; ++i) ok = ok && std::isfinite(omega[i]);
    return ok;
}

// The telemetry read from the state at the end of the tick.
void Vehicle::Impl::end_telemetry(bool spec_body, VehicleTelemetry& tel) const {
    tel.clutch_state = int(clutch);
    tel.g_over_mu = float(g_over_mu);
    tel.yaw_acc = float(yaw_acc);
    tel.pitch = float(ax);
    tel.roll = float(ay);
    tel.pitch_rate = float(ax_rate);
    tel.roll_rate = float(ay_rate);
    const Axes Ag = ground_frame(spec_body);
    tel.ground_u = float(vel.dot(Ag.fwd));
    tel.ground_l = float(vel.dot(Ag.lat));
    tel.steer_angle = float(steer_avg);
    tel.steer_rate = float(steer_rate);
}

void Vehicle::publish() {
    Impl& m = *impl_;
    Axes Af = make_axes(m.ax, m.ay, m.heading);
    state_.position = (m.pos - Af.to_world(m.cg_local)).f();
    state_.orientation = axes_to_quat(Af);
    state_.velocity = m.vel.f();
    state_.angular_velocity = (D3{0, m.yaw_rate, 0} + Af.lat * m.ax_rate + Af.fwd * m.ay_rate).f();
    state_.engine_rpm = float(m.omega_e * model::kRadSToRpm);
    state_.gear = m.gear;
    state_.speed_kmh = float(m.vel.length() * 3.6);
    state_.rev_limiter = m.limiter;
    for (size_t i = 0; i < 4; ++i) {
        WheelState& w = state_.wheels[i];
        w.contact = m.contact[i].f();
        // Airborne: the point on the ground under the wheel (renderer: the car's shadow stays on
        // the road). The sim's contact.y is not read for a wheel off the ground.
        if (!m.grounded[i]) w.contact.y = float(m.ground_prev[i]);
        w.steer_angle = float(m.steer[i]);
        w.spin_angle = float(m.spin[i]);
        w.angular_velocity = float(m.omega[i]);
        // compression below the spring's zero-force point (spec coordinate) or droop (placeholder).
        // Spec path: the coordinate after the droop / bump-stop clamp (x_prev, physics §10.5): the raw x
        // follows the ground's distance, which drew airborne wheels on the road under the car.
        const bool spec_susp = spec_.axle[size_t(m.axle[i])].susp.valid;
        w.suspension = float(spec_susp ? -m.x_prev[i] : m.x[i]);
        w.slip_ratio = float(m.kappa[i]);
        w.slip_angle = float(m.alpha[i]);
        w.load = float(m.load[i]);
        w.grounded = m.grounded[i];
        w.surface = m.surface[i];
        w.flap = m.flap_x[i];
    }
    state_.steer = float(m.steer_avg);
}

} // namespace gt3r::sim
