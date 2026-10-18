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
// game's Z-up frame, model::euler_axes). Sub-models not yet in the spec
// (suspension, steering, brakes, clutch/gearbox/differentials) are the
// placeholder functions in models.cpp.

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

constexpr double kMaxAttitude = 0.6; // rad, clamp on pitch/roll [spec §1.3: "clamped to small values"; value placeholder]

enum class ClutchState { Open, Locked, Slipping, Shift }; // [spec §2.4] states 0, 1, 2, 3

// Half of an axle's share of the sprung mass, kg [spec §4.2]: share x (total - 2 (front + rear unsprung)) / 2.
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

} // namespace

struct Vehicle::Impl {
    // Planar + attitude state [spec §1.3].
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
    // [spec §1.2 phase 12] spec body: the suspension/ground step runs at the end of the tick
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
    double slip = 0, slip_target = 0; // slipstream [spec §5.2]
    double lat_g = 0; // lateral acceleration of the last tick, g (TCS / ASCC input)
    std::array<model::DiffState, 2> diff{}; // spec driveline differential state per axle
    bool cpu_clutch = false;                // CPU binary clutch request state
    std::array<double, 4> ground_prev{};    // last ground height under each hub
    int spawn = 0;                          // spawn-hold counter [spec §4.3]
    double spawn_heading = 0;               // grid heading the hold eases to
    // tyre wear [spec §8]
    model::WearParams wear_p;
    std::array<float, 4> wear{}, wear_grip{1, 1, 1, 1}, smoke{}, smoke_emit{}, brake_heat{};
    std::array<int, 4> air_steps{}, wear_level{};
    bool pit_surface = false;
    // axle driveline [spec §2.7, draft 18]
    bool centre_engaged = false;           // L4..6 centre-clutch engaged flag
    float cond_e = 0, cond_o = 0;          // engine condition counters [spec §10.3]
    bool dirt = false;                     // dirt course [spec §10.4]
    int pad_o3 = 0;                        // pad state O[3]: 1 digital, 2 analog steering [spec §4.4]
    std::array<double, 2> axle_torque{};   // last tick's summed wheel net torques (front, rear)
    double g_over_mu = 0;                  // lateral g / mu (last tick)
    double yaw_acc = 0;                    // last tick's yaw acceleration (ASCC understeer input)
    std::array<double, 4> slip_g{1, 1, 1, 1}; // slip gain from the wheel's own hub forward speed (§3.4)
    double rear_beta_prev = 0;             // largest rear |beta| of the last tyre phase (ASCC oversteer input)
    double long_force_sum = 0;             // sum k (drive_part + brake_part) of the last tyre phase
    std::array<bool, 4> free_roll{};       // §10.2 free-rolling wheels of the last tick


    model::TurboState turbo;
    std::vector<D3> hull, wall_pts;
    std::array<collide::V2, 8> octagon{};
    double oct_radius = 0;
    std::vector<collide::Segment> segs; // scratch
    collide::WallContacts wall_contacts; // [spec §5.3] points held on a wall
    double wall_radius = 0.3;
};

Vehicle::Vehicle(const CarSpec& spec) : spec_(spec), impl_(std::make_unique<Impl>()) {
    spec_.finalize();
    Impl& m = *impl_;
    const CarSpec& s = spec_;
    m.cg_local = {0, s.cg_height, s.wheelbase * (0.5 - s.front_weight)};
    // [spec §4.1/4.3/6.4] yaw: r += N / I_z dt with I_z = chassis inertia byte x 100
    // (R34 9500); pitch and roll use I/(2 pi) with Y = I_z and Z = I_z / 2.
    const double L = s.length, W = s.width;
    m.i_yaw = s.yaw_inertia;
    m.i_pitch = s.yaw_inertia / 6.283185307179586;
    m.i_roll = s.yaw_inertia * 0.5 / 6.283185307179586;
    setup_.wheelbase = s.wheelbase;
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
        // x_s = -(half the axle's sprung weight) / k [spec §4.2]: the weight split applies to the SPRUNG
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
    // [spec §5.3] the octagon is CoG-relative: front = spec front extent - L (wf - 0.5),
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

collide::Body Vehicle::collision_body() const {
    const Impl& m = *impl_;
    collide::Body b;
    b.pos = {m.pos.x, m.pos.z};
    b.heading = m.heading;
    b.vel = {m.vel.x, m.vel.z};
    b.vy = m.vel.y;
    b.yaw_rate = m.yaw_rate;
    b.inv_mass = 1.0 / spec_.mass;
    b.inv_inertia = 1.0 / (10.0 * m.i_yaw); // [spec §5.3]
    b.local = m.octagon;
    b.radius = m.oct_radius;
    return b;
}

void Vehicle::set_collision_body(const collide::Body& b) {
    Impl& m = *impl_;
    m.pos.x = b.pos.x;
    m.pos.z = b.pos.z;
    m.heading = b.heading;
    m.vel.x = b.vel.x;
    m.vel.z = b.vel.z;
    m.vel.y = b.vy;
    m.yaw_rate = b.yaw_rate;
}

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

void Vehicle::fit_fresh_tires() {
    Impl& m = *impl_;
    for (size_t i = 0; i < 4; ++i) {
        m.wear[i] = m.wear_p.on() ? m.wear_p.fresh : 0;
        m.wear_grip[i] = m.wear_p.on() ? 1 - m.wear_p.loss_fresh : 1;
        m.wear_level[i] = m.wear_p.on() ? -63 : 0;
        m.brake_heat[i] = 0;
    }
}

void Vehicle::set_engine_condition(float e, float o) {
    impl_->cond_e = e;
    impl_->cond_o = o;
}
void Vehicle::engine_condition(float& e, float& o) const {
    e = impl_->cond_e;
    o = impl_->cond_o;
}
void Vehicle::set_dirt_course(bool dirt) { impl_->dirt = dirt; }

void Vehicle::begin_pit_stop(const Quat& box_orientation) {
    Impl& m = *impl_;
    D3 lt = rotate(box_orientation, {1, 0, 0});
    double box_heading = std::atan2(-lt.z, lt.x); // as reset(): lateral axis in the physics frame (x, -z)
    // [spec §8.6] velocities and dynamic states zeroed.
    m.vel = {};
    m.yaw_rate = m.ax_rate = m.ay_rate = 0;
    for (size_t i = 0; i < 4; ++i) m.omega[i] = m.kappa[i] = m.alpha[i] = 0;
    m.omega_e = 0; // [spec §9.3] engine speed 0, floored to idle on the first step
    m.diff = {};
    m.turbo = {};
    m.steer_avg = m.steer_rate = 0;
    m.wall_contacts.clear();
    m.spawn = 300; // [spec §8.6] 5 s stop, the scripted pose of §4.3
    m.spawn_heading = box_heading;
    publish();
}

void Vehicle::spawn(const Vec3& position, const Quat& orientation) {
    reset(position, orientation);
    impl_->spawn = 120; // [spec §4.3] spawn-hold counter
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
    double r_driven = 0;
    int n = 0;
    for (int i = 0; i < 4; ++i)
        if (driven(s.drivetrain.drive, i)) {
            r_driven += s.axle[size_t(i / 2)].tire_radius;
            ++n;
        }
    r_driven /= std::max(n, 1);
    float up = s.engine.upshift_rpm > 0 ? s.engine.upshift_rpm : s.engine.rev_limit_rpm - 500;
    int nf = int(s.gearbox.ratios.size());
    m.gear = std::clamp(model::gear_for_speed(s.gearbox, speed_mps, float(r_driven), up), 1, std::max(nf, 1));
    double w_in = double(speed_mps) / r_driven * model::total_ratio(s.gearbox, m.gear);
    double w_idle = s.engine.idle_rpm * model::kRpmToRadS, w_lim = s.engine.rev_limit_rpm * model::kRpmToRadS;
    // [spec §9.3] engine = clamp(v ratio, idle, rev limit), rpm truncated; clutch locked
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
    const bool fresh = m.settle; // never stepped since reset(): no internal history to keep
    // Euler angles as reset() derives them.
    D3 fw = rotate(st.orientation, {0, 0, -1}), lt = rotate(st.orientation, {1, 0, 0}), upw = rotate(st.orientation, {0, 1, 0});
    auto to_phys = [](const D3& w) { return D3{w.x, -w.z, w.y}; };
    D3 f = to_phys(fw), l = to_phys(lt), u = to_phys(upw);
    m.ay = clampd(-std::asin(clampd(l.z, -1, 1)), -kMaxAttitude, kMaxAttitude);
    m.heading = std::atan2(l.y, l.x);
    m.ax = clampd(std::atan2(f.z, u.z), -kMaxAttitude, kMaxAttitude);
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
    // Euler angles from the quaternion (world axes -> physics frame -> angles).
    D3 fw = rotate(orientation, {0, 0, -1}), lt = rotate(orientation, {1, 0, 0}), upw = rotate(orientation, {0, 1, 0});
    auto to_phys = [](const D3& w) { return D3{w.x, -w.z, w.y}; };
    D3 f = to_phys(fw), l = to_phys(lt), u = to_phys(upw);
    m.ay = -std::asin(clampd(l.z, -1, 1));
    m.heading = std::atan2(l.y, l.x);
    m.ax = std::atan2(f.z, u.z);
    m.ax = clampd(m.ax, -kMaxAttitude, kMaxAttitude);
    m.ay = clampd(m.ay, -kMaxAttitude, kMaxAttitude);
    Axes a = make_axes(m.ax, m.ay, m.heading);
    m.pos = D3(position) + a.to_world(m.cg_local);
    m.vel = {};
    m.yaw_rate = m.ax_rate = m.ay_rate = 0;
    for (size_t i = 0; i < 4; ++i) {
        m.x[i] = m.x_prev[i] = spec_.axle[size_t(m.axle[i])].susp.valid ? m.xs[i] : m.rest_x[i];
        m.omega[i] = m.spin[i] = m.steer[i] = m.kappa[i] = m.alpha[i] = 0;
        // static corner load (at x = x_s: -k x_s + preload = corner weight), the wheel load the first
        // tick's prep reads [spec §3.1/§4.2]
        m.load[i] = spec_.axle[size_t(m.axle[i])].susp.valid
                        ? (sprung_corner(spec_, int(i)) + spec_.axle[size_t(m.axle[i])].susp.unsprung) * model::kGravity
                        : 0.0;
        m.grounded[i] = true;
        m.surface[i] = 0;
        m.contact[i] = D3(position) + a.to_world(m.hub[i] - D3{0, m.hub[i].y, 0});
        m.ground_prev[i] = position.y;
    }
    m.omega_e = 0; // [spec §9.3] engine speed 0, floored to idle on the first step
    m.limiter = false;
    m.gear = 1;
    m.clutch = ClutchState::Open;
    m.shift_lockout = 12; // [spec §9.3] standing start: shift lockout 12
    m.steer_avg = m.steer_rate = 0;
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

void Vehicle::tick(const ControlInputs& in_raw, const TrackQuery& track, double h, const WorldQuery* world) {
    Impl& m = *impl_;
    // Automatic in reverse: the brake pedal drives and the accelerator brakes, since
    // pressing the accelerator below 1 km/h selects first gear [spec §2.6;
    // pedal swap is an interpretation of GT3's AT reverse].
    ControlInputs in = in_raw;
    // [spec §2.6] the shift dispatcher uses mode 0 (automatic) for CPU cars and for a
    // non-zero driver kind (run-up, pit lane, stop driver).
    if (cpu_driver_ || driver_kind_ != 0) in.automatic = true;
    if (!cpu_driver_ && !std::isfinite(in.steer_angle)) { // pad path [spec §4.4]: x + 0.1 (1 - x) once pressed
        in.throttle = model::pedal_shape(std::clamp(in.throttle, 0.0f, 1.0f));
        in.brake = model::pedal_shape(std::clamp(in.brake, 0.0f, 1.0f));
    }
    // Player AT reverse: the brake pedal drives [interpretation]; CPU drivers request reverse
    // explicitly and drive it with the accelerator.
    if (in.automatic && m.gear < 0 && !cpu_driver_) std::swap(in.throttle, in.brake);
    // Mod EVs, one-pedal driving: regen slows the car (models.cpp engine_torque); near walking pace
    // the friction brakes blend in and hold it at a standstill while neither pedal is pressed.
    if (spec_.engine.electric && !cpu_driver_ && driver_kind_ == 0 && in.throttle < 0.02f) {
        float v = std::fabs(state_.speed_kmh);
        if (v < 5.0f) in.brake = std::max(in.brake, 0.35f * (1.0f - v / 5.0f) + 0.05f);
    }
    const CarSpec& s = spec_;
    const DrivetrainSpec& d = s.drivetrain;
    Impl saved = m;
    const double M = s.mass;
    const int nf = int(s.gearbox.ratios.size());
    VehicleTelemetry tel;
    tel.substeps = 1;
    // Tyre record for a wheel on surface s [spec §6.5]: compound slot = class of s.
    auto tire_for = [&](int wheel, int surf) -> const TireSpec& {
        int slot = s.surface_slot[size_t(std::clamp(surf, 0, 15))];
        if (slot > 2) slot = 0;
        return s.tire_slots[size_t(m.axle[size_t(wheel)])][size_t(slot)];
    };

    if (m.settle) {
        // Fit the body to the ground under the four hubs (pitch, roll, height) so
        // a reset on a slope or banking starts at static ride height.
        m.settle = false;
        for (int pass = 0; pass < 3; ++pass) {
            Axes A0 = make_axes(m.ax, m.ay, m.heading);
            double gh[4];
            bool ok = true;
            for (size_t i = 0; i < 4; ++i) {
                D3 hw = m.pos + A0.to_world(m.hub[i] - m.cg_local);
                SurfaceHit hit = track.probe(float(hw.x), float(hw.z), float(hw.y + 2.0));
                ok = ok && hit.hit;
                gh[i] = hit.height;
            }
            if (!ok) break;
            double tf = 0.5 * (s.axle[0].track + s.axle[1].track);
            double pitch = std::atan2(0.5 * (gh[0] + gh[1]) - 0.5 * (gh[2] + gh[3]), s.wheelbase); // nose up > 0
            double roll = std::atan2(0.5 * (gh[0] + gh[2]) - 0.5 * (gh[1] + gh[3]), tf);           // left high > 0
            m.ax = clampd(pitch, -kMaxAttitude, kMaxAttitude);
            m.ay = clampd(roll, -kMaxAttitude, kMaxAttitude);
            Axes A1 = make_axes(m.ax, m.ay, m.heading);
            D3 origin{0, 0, 0};
            for (size_t i = 0; i < 4; ++i) origin.y += gh[i] * 0.25;
            D3 o = m.pos - A1.to_world(m.cg_local); // current origin; keep x/z, set ground height
            o.y = origin.y;
            m.pos = o + A1.to_world(m.cg_local);
        }
    }

    Axes A = make_axes(m.ax, m.ay, m.heading);
    D3 fwd_h = D3{A.fwd.x, 0, A.fwd.z}.normalized();
    // [spec §10.1] u = ground-frame forward tangent . world velocity (the vertical velocity enters
    // along the ground slope, not along the body pitch): what aero, steering, clutch, aids, shifts and the
    // engine counters read. With the spec suspension the ground frame is the one the last tick's phase 12 left.
    // [spec §4.1-4.3] spec_body: both axles carry the spec suspension block (every disc car): planar step,
    // attitude and the phase-12 order. Hand-built specs without it use a placeholder rigid body.
    const bool spec_body = spec_.axle[0].susp.valid && spec_.axle[1].susp.valid;
    const bool ground_u = spec_body && m.late_init;
    double v_fwd = m.vel.dot(ground_u ? make_axes(m.late_gf.pitch, m.late_gf.roll, m.heading).fwd : A.fwd);
    // Angular velocity in the world (Euler rates; small-angle composition).
    D3 omega_w = D3{0, m.yaw_rate, 0} + A.lat * m.ax_rate + A.fwd * m.ay_rate;

    // --- gear selection [spec §2.6]: instantaneous shifts, 1-step lockout ---
    double r_driven = 0;
    {
        int n = 0;
        for (int i = 0; i < 4; ++i)
            if (driven(d.drive, i)) { r_driven += s.axle[size_t(i / 2)].tire_radius; ++n; }
        r_driven /= std::max(n, 1);
    }
    auto set_gear = [&](int to) {
        if (to == m.gear) return;
        int from = m.gear;
        m.gear = to;
        m.shift_lockout = 1;
        // [spec §2.4, draft 18] with the clutch engaged, a change to or from
        // reverse gives the launch slip (2), a forward -> forward change the fixed-torque
        // shift clutch (3) until the speeds cross. A locked clutch never just opens.
        m.clutch = ClutchState(model::clutch_gear_change(int(m.clutch), from, to));
    };
    // [spec §2.4/§2.6] the shift decision uses this tick's final state and the gear change
    // takes effect at the END of the tick, after the driveline step: the shift tick itself
    // still transmits through the old gear / clutch state (called before the NaN guard).
    auto select_gear = [&](double v_fwd_sel) {
        bool slow = std::fabs(v_fwd_sel) < 1.5;
        if (m.shift_lockout > 0) {
            --m.shift_lockout;
        } else {
            // Manual requests (and R/N selection at a standstill in both modes).
            if (in.shift_up) {
                if (m.gear < 0) set_gear(in.automatic ? 1 : 0);
                else if (m.gear == 0) set_gear(1);
                else if (!in.automatic && m.gear < nf) set_gear(m.gear + 1);
            } else if (in.shift_down) {
                if (m.gear == 1 && slow) set_gear(in.automatic ? -1 : 0);
                else if (m.gear == 0 && slow) set_gear(-1);
                else if (!in.automatic && m.gear > 1) set_gear(m.gear - 1);
            } else if (m.gear != 0) {
                bool veto = false;
                // [spec §2.6, draft 23] the driven axle only: L1/L2 front, L0/L4-6 rear, L3/L7 all four
                const model::AwdLayout vl = model::awd_layout(int(d.drive), d.awd_system, d.awd_share_byte, d.awd_strength_byte);
                for (int i = 0; i < 4; ++i) {
                    bool veto_axle = vl.layout == 3 || vl.layout == 7 || (i < 2) == (vl.layout == 1 || vl.layout == 2);
                    if (veto_axle &&
                        (!m.grounded[size_t(i)] ||
                         m.kappa[size_t(i)] < s.tire[size_t(i / 2)].opt_drive * m.slip_g[size_t(i)])) // [spec §2.6] optimum x the wheel's slip gain
                        veto = true;
                }
                model::ShiftIn si;
                si.gear = m.gear;
                // rpm s16 = max(int(w 60 / 2 pi), idle - 1) of this tick's final engine speed [spec §2.4/2.6]
                si.rpm = float(std::max(int(m.omega_e * model::kRadSToRpm), int(s.engine.idle_rpm) - 1));
                si.speed = float(v_fwd_sel);
                si.throttle = in_raw.throttle; // the accelerator (in AT reverse `in` has the pedals swapped)
                si.brake = in_raw.brake;
                si.clutch_locked = m.clutch == ClutchState::Locked;
                si.automatic = in.automatic;
                si.reverse_request = reverse_request_;
                si.traction_veto = veto;
                si.driven_radius = float(r_driven);
                // [spec §2.6, draft 18] automatic shifts only while the clutch is locked
                // (shift_logic); open or slipping, only the reverse/first/over-rev rules act.
                int to = model::shift_logic(s.engine, s.gearbox, si);
                // Reversing into a forward gear only when asked (keeps manual reverse usable).
                if (to == -1 && m.gear >= 1 && !in.automatic) to = m.gear;
                if (to == 1 && m.gear == -1 && !in.automatic) to = m.gear;
                set_gear(to);
            }
        }
    };
    if (in.automatic && m.gear == 0) set_gear(1);

    // --- steering [spec §4.4]: pad curve B, second-order follower; Ackermann [placeholder] ---
    if (std::isfinite(in.steer_angle)) {
        // CPU cars write the road-wheel angle directly, bypassing the pad follower
        // and the car's lock (the AI clamps itself at 45 deg).
        m.steer_avg = clampd(in.steer_angle, -1.0, 1.0);
        m.steer_rate = 0;
    } else {
        // [spec §4.4] controller kind (0 = keep the previous one); a step with nonzero steer sets
        // the pad state O[3]: kind 3 -> digital (30 deg/s turn-in), kind 2 -> analog (120 deg/s).
        if (in.controller_kind != 0) input_kind_ = in.controller_kind;
        if (in.steer != 0 && (input_kind_ == 2 || input_kind_ == 3)) m.pad_o3 = input_kind_ == 3 ? 1 : 2;
        // [spec §3.2/§4.4] mean velocity-direction angle phi = atan2(-l_h, u_h) of the two front hubs (0 while the
        // low-speed fade is 0), from the hub velocities the last tick left (ground-frame tangents, §10.1), in our
        // sign (+ = right) for the lock extension
        double va = 0;
        {
            const Axes Ag = ground_u ? make_axes(m.late_gf.pitch, m.late_gf.roll, m.heading) : A;
            for (size_t i = 0; i < 2; ++i) {
                D3 hv = m.vel + D3{0, m.yaw_rate, 0}.cross(m.contact[i] - m.pos);
                float fade = 0;
                float phi = model::patch_slip_angle(float(hv.dot(Ag.fwd)), float(hv.dot(Ag.lat)), fade);
                va -= 0.5 * double(phi);
            }
        }
        float target = model::steer_target(s.steering, in.steer, float(v_fwd), float(va));
        float angle = float(m.steer_avg), rate = float(m.steer_rate);
        model::steer_follow(s.steering, target, angle, rate, float(h), m.pad_o3 == 1);
        m.steer_avg = angle;
        m.steer_rate = rate;
    }
    {
        // [spec §4.4, draft 16] front wheels: delta x Ackermann ratio (the inner wheel,
        // on the side the car turns to, gets the larger one); toe-in on all wheels
        // (our steer sign is + right, so toe-in is + on the left wheel, - on the right).
        double dlt = m.steer_avg;
        double r_left = dlt >= 0 ? s.steering.ack_outer : s.steering.ack_inner;
        double r_right = dlt >= 0 ? s.steering.ack_inner : s.steering.ack_outer;
        if (s.steering.ack_inner <= 0) {
            float sl, sr;
            model::ackermann(s.steering, float(dlt), s.wheelbase, s.axle[0].track, sl, sr);
            m.steer[0] = sl;
            m.steer[1] = sr;
        } else {
            m.steer[0] = dlt * r_left;
            m.steer[1] = dlt * r_right;
        }
        m.steer[0] += s.axle[0].toe;
        m.steer[1] -= s.axle[0].toe;
        m.steer[2] = s.axle[1].toe;
        m.steer[3] = -s.axle[1].toe;
    }

    // --- ground query at the hubs [spec §1.3] and suspension [placeholder §4] ---
    D3 force{0, -M * model::kGravity, 0};
    D3 torque{};
    auto apply = [&](const D3& f, const D3& at) {
        force += f;
        torque += (at - m.pos).cross(f);
    };
    std::array<D3, 4> hub_w, normal;
    // [spec §3.1, phase 2] the peak force is computed in the prep phase from the wheel load the
    // previous tick's suspension step (phase 12) left in the wheel load: the tyres lag the loads by a tick.
    const std::array<double, 4> load_prev = m.load;
    std::array<int, 4> sidx{};
    std::array<SurfaceHit, 4> hits;
    std::array<double, 4> ground_h{};
    model::GroundFrame gf;
    Axes ground_axes = make_axes(0, 0, m.heading);
    std::array<double, 4> body_f{}, damper_f{};
    std::array<float, 4> so_x{};
    // [spec §1.2] phase 12 order with the spec suspension: ground query + suspension at the end of the tick.
    // Ground query at the hubs and the suspension step, for the body pose (axes A, angular velocity omega_w).
    // [spec §1.3/§4.3] the wheel query point of a spec-suspension car is the hub offset in the CoG
    // plane, (x_long, y_lat, 0) rotated by the attitude: pitch and roll move it along the ground only by
    // lever x cos, not by the hub's height above the CoG [verified: 4 mm at 1 deg of pitch,
    // which decides kerbs and road edges]. Hand-built specs keep the hub at its true height.
    auto hub_point = [&](size_t i, const Axes& Ah) {
        D3 off = m.hub[i] - m.cg_local;
        if (s.axle[size_t(m.axle[i])].susp.valid) off.y = 0;
        return m.pos + Ah.to_world(off);
    };
    // [spec §10.1] hub velocity of wheel i in the ground frame: u + X r (forward) and l - Y r (right),
    // u / l = world velocity on the ground-frame tangents, X = (-tf, +tf, -tr, +tr), Y = (a, a, -b, -b).
    auto hub_velocity = [&](size_t i, const Axes& Ag, double& uh, double& lh) {
        const double a_len = s.wheelbase * (1 - s.front_weight), b_len = s.wheelbase * s.front_weight;
        const double X = (i % 2 == 0 ? -0.5 : 0.5) * s.axle[i / 2].track, Y = i < 2 ? a_len : -b_len;
        uh = m.vel.dot(Ag.fwd) + X * m.yaw_rate;
        lh = m.vel.dot(Ag.lat) - Y * m.yaw_rate;
    };
    // hub speed along the wheel heading (our steer + = right): cos d uh - sin d lh with d = -steer [spec §3.4, §10.2]
    auto along_heading = [](double uh, double lh, double steer) { return uh * std::cos(steer) + lh * std::sin(steer); };
    auto suspension_phase = [&](const Axes& A, const D3& omega_w) {
        for (size_t i = 0; i < 4; ++i) {
            hub_w[i] = hub_point(i, A);
            hits[i] = track.probe(float(hub_w[i].x), float(hub_w[i].z), float(hub_w[i].y + 1.0));
            if (hits[i].hit) {
                sidx[i] = model::surface_index(hits[i].surface);
                D3 hub_vel = m.vel + omega_w.cross(hub_w[i] - m.pos);
                ground_h[i] = hits[i].height + model::road_bump(sidx[i], float(hub_w[i].x), float(hub_w[i].z),
                                                                float(hub_vel.length()));
            } else {
                ground_h[i] = m.ground_prev[i]; // a miss keeps the previous ground height (§1.3)
            }
            m.ground_prev[i] = ground_h[i];
        }
        // [spec §4.3, draft 16] ground frame and terrain factor s = ground up.z / body up.z.
        {
            float g[4] = {float(ground_h[0]), float(ground_h[1]), float(ground_h[2]), float(ground_h[3])};
            gf = model::ground_frame(g, 1 - s.front_weight, s.wheelbase, 0.5f * (s.axle[0].track + s.axle[1].track));
        }
        ground_axes = make_axes(gf.pitch, gf.roll, m.heading);
        const double terrain = ground_axes.up.y / std::max(A.up.y, 0.2);
        for (size_t i = 0; i < 4; ++i) {
            const AxleSpec& a = s.axle[size_t(m.axle[i])];
            normal[i] = {0, 1, 0};
            m.grounded[i] = false;
            double x = 0;
            const SurfaceHit& hit = hits[i];
            // [spec §1.3/§5.3] a ground-query miss keeps the previous ground height (and surface): the
            // wheel stands on that height, the car is not stopped (spec suspension path).
            const bool virtual_ground = !hit.hit && a.susp.valid;
            if (hit.hit || virtual_ground) {
                D3 n = hit.hit ? D3(hit.normal).normalized() : D3{0, 1, 0};
                double ground = ground_h[i];
                // Gap between the hub and its contact height, scaled by the terrain factor.
                // [spec §4.2] x = (body height + hub offset - ground height - bump) x att + const: the plain
                // ground height, no slope correction (the triangle normal does not enter the spring coordinate).
                // Spec suspension: x = (body height + hub offset - ground) att + (x_s - q) with the query point in the CoG plane;
                // our body sits cg_height above the ground where the spec's body height sits q above it, so the gap is
                // measured from ground + cg_height - q and x = x_s - q - gap att (equal to x_s at the static pose).
                double gap = (ground + (a.susp.valid ? m.cg_local.y - a.susp.ride_height : a.tire_radius / std::max(n.y, 0.5))) - hub_w[i].y;
                x = m.rest_x[i] + gap * terrain;
                normal[i] = n;
                if (hit.hit) m.surface[i] = hit.surface;
                else sidx[i] = model::surface_index(m.surface[i]);
                m.contact[i] = {hub_w[i].x, ground, hub_w[i].z};
            }
            if (a.susp.valid) {
                // Spec suspension coordinate: x = x_s - (contact height - hub height) [spec §4.2].
                m.grounded[i] = true; // the suspension step decides (droop = airborne)
                m.x[i] = m.xs[i] - a.susp.ride_height - (x - m.rest_x[i]);
                continue;
            }
            if (x > 0) {
                m.grounded[i] = true;
            } else {
                x = 0;
                m.contact[i] = hub_w[i] - A.up * a.tire_radius;
            }
            m.x[i] = std::min(x, double(a.travel) * 2.0);
        }
        body_f = {};
        damper_f = {};
        so_x = {};
        for (size_t i = 0; i < 4; ++i) {
            const AxleSpec& a = s.axle[size_t(m.axle[i])];
            double v = (m.x[i] - m.x_prev[i]) / h;
            if (a.susp.valid) {
                model::SuspensionOut so{};
                if (m.grounded[i]) {
                    so = model::suspension_wheel(a.susp, float(m.x[i]), float(v), float(m.xs[i]));
                } else { // no ground under the hub: airborne (§10.5)
                    so = model::suspension_wheel(a.susp, 1e9f, 0, float(m.xs[i]));
                }
                m.grounded[i] = so.grounded;
                m.load[i] = so.load;
                so_x[i] = so.x;
                body_f[i] = so.spring + so.damper; // airborne: -preload [spec §10.5]
                damper_f[i] = so.grounded ? so.damper : 0.0;
                if (so.grounded) {
                    size_t l = i & ~size_t(1), r = l + 1;
                    double arb = model::anti_roll(a.susp, float(m.x[l]), float(m.x[r]));
                    body_f[i] += (i == l) ? arb : -arb;
                } else {
                    m.contact[i] = m.pos + A.to_world(m.hub[i] - m.cg_local) - A.up * a.tire_radius;
                }
            } else {
                double f = model::suspension_force(a, float(m.x[i]), float(v));
                if (m.grounded[i]) f += a.arb_rate * (m.x[i] - m.x[i ^ 1]);
                m.load[i] = m.grounded[i] ? std::max(f, 0.0) : 0.0;
                body_f[i] = m.load[i];
            }
            m.x_prev[i] = a.susp.valid ? double(so_x[i]) : m.x[i]; // [spec §10.5] the clamped x'
        }
        // Explicit load transfer [spec §4.2]: GT3 has no tyre-force moments about the
        // CoG; the last tick's accelerations shift body force between the wheels:
        // longitudinal m_s X a / L (front <-> rear), lateral m_s X a / track (left <-> right),
        // with X = spec CoG-height parameter (CarSpec::cg_height).
        // [spec §4.2] with the spec suspension on both axles the transfer terms are added after the lateral
        // step instead (phase 12 runs after phase 7 and reads this tick's lateral acceleration / longitudinal force sum);
        // a spec with the block on the front axle only adds them here from the last tick's values.
        if (spec_.axle[0].susp.valid && !spec_body) {
            double ms = M - 2.0 * (s.axle[0].susp.unsprung + s.axle[1].susp.unsprung);
            double X = s.cg_height;
            double t_mean = 0.5 * (s.axle[0].track + s.axle[1].track);
            // [spec §4.2] the longitudinal factor x the force sum in full per wheel, with the factor =
            // (sprung fraction) X / L and the force sum = sum of the longitudinal tyre forces x cos steer
            // (previous tick's tyre phase).
            double d_long = 2.0 * (ms / M) * X / s.wheelbase * m.long_force_sum;
            double d_lat = ms * X * m.lat_g * model::kGravity / t_mean; // right accel loads the left
            for (size_t i = 0; i < 4; ++i) {
                if (!m.grounded[i]) continue;
                // An inertial moment: the loaded side's support force drops so the body
                // sinks there and its springs (and tyre loads) take the transfer.
                // the lateral term (lateral factor x -a_y) in full on each wheel (left -, right +)
                body_f[i] += (i < 2 ? 0.5 : -0.5) * d_long + (i % 2 == 0 ? -1.0 : 1.0) * d_lat;
            }
        }
        m.late_normal = normal;
        m.late_hits = hits;
        m.late_gf = gf;
    };
    if (!spec_body || !m.late_init) {
        suspension_phase(A, omega_w);
        if (spec_body) m.late_init = true;
    } else {
        // the previous tick's phase 12 left the ground state; this tick's tyre phase reads it
        normal = m.late_normal;
        hits = m.late_hits;
        gf = m.late_gf;
        ground_axes = make_axes(gf.pitch, gf.roll, m.heading);
        for (size_t i = 0; i < 4; ++i) {
            hub_w[i] = hub_point(i, A);
            sidx[i] = model::surface_index(m.surface[i]);
        }
    }
    if (!spec_body)
        for (size_t i = 0; i < 4; ++i)
            if (body_f[i] != 0) apply(A.up * body_f[i], hub_w[i]);

    // --- aero and slipstream [spec §5.2] ---
    model::AeroOut aero = model::aero_step(s.aero, float(v_fwd), float(m.slip_target), float(m.slip), float(h));
    m.slip = aero.slip;
    tel.slipstream = float(m.slip);
    // [spec §10.6] all four wheels airborne: no drag (nor engine/brake force) acts in the air, yaw is frozen.
    const bool all_airborne = spec_body && !m.grounded[0] && !m.grounded[1] && !m.grounded[2] && !m.grounded[3];
    // [spec §4.1] the spec body moves by the forward step + lateral/yaw sums (drag enters the forward step)
    double planar_ax = 0, planar_ay = 0;
    if (!spec_body) {
        force += A.fwd * double(aero.drag);
        D3 front_axle = m.pos + A.to_world(D3{0, 0, -s.wheelbase * 0.5} - m.cg_local);
        D3 rear_axle = m.pos + A.to_world(D3{0, 0, s.wheelbase * 0.5} - m.cg_local);
        apply(A.up * -double(aero.lift_front), front_axle);
        apply(A.up * -double(aero.lift_rear), rear_axle);
    }
    tel.drag_force = std::fabs(aero.drag);
    tel.downforce = aero.lift_front + aero.lift_rear;

    // Non-tyre acceleration so far: used to predict contact velocities for the
    // velocity-limited tyre force (keeps standstill stable at 60 Hz) [placeholder].
    D3 a0 = force * (1.0 / M);

    // --- tyre geometry and slips [spec §3.2, §3.4] ---
    // [spec §8.6] pit-lane surface: triangle flag bit 0 under any wheel.
    m.pit_surface = false;
    for (size_t i = 0; i < 4; ++i)
        if (hits[i].hit && ((hits[i].surface >> 8) & 1)) m.pit_surface = true;
    tel.on_pit_surface = m.pit_surface;
    // [spec §8.4] wear -> grip (phase 2), not on the pit surface.
    std::array<double, 4> wear_mul{1, 1, 1, 1};
    if (m.wear_p.on() && !m.pit_surface)
        for (size_t i = 0; i < 4; ++i) {
            const TireSpec& tw = tire_for(int(i), sidx[i]);
            m.wear_grip[i] = model::wear_grip(m.wear_p, m.wear[i], tw.surface_mu[11], &m.wear_level[i]);
            wear_mul[i] = m.wear_grip[i];
        }
    std::array<double, 4> fx{}, fy{}, radius{}, vxs{}, vys{}, vhub{}, fades{}, peaks{};
    std::array<D3, 4> wf, wr, vcs;
    const double m_eff = M * 0.25;
    for (size_t i = 0; i < 4; ++i) {
        radius[i] = s.axle[size_t(m.axle[i])].tire_radius;
        // [spec §10.1] wheel speeds are measured along the GROUND-frame
        // tangents (ground pitch/roll from the four wheel heights, §4.3), common to all wheels, and
        // the hub velocity carries only the yaw rate (u + X r, l - Y r): pitch/roll rates and the
        // per-triangle normals do not enter.
        const D3 n = ground_axes.up;
        D3 f = ground_axes.to_world(D3{std::sin(m.steer[i]), 0, -std::cos(m.steer[i])});
        f = (f - n * n.dot(f)).normalized();
        wf[i] = f;
        wr[i] = f.cross(n);
        if (spec_body) {
            // the hub velocity (u + X r, l - Y r) [spec §10.1] and the speeds along / across the wheel heading
            double uh, lh;
            hub_velocity(i, ground_axes, uh, lh);
            vcs[i] = ground_axes.fwd * uh + ground_axes.lat * lh;
            vxs[i] = along_heading(uh, lh, m.steer[i]);
            vys[i] = -uh * std::sin(m.steer[i]) + lh * std::cos(m.steer[i]);
        } else {
            vcs[i] = m.vel + D3{0, m.yaw_rate, 0}.cross(m.contact[i] - m.pos);
            vxs[i] = vcs[i].dot(wf[i]);
            vys[i] = vcs[i].dot(wr[i]);
        }
        // [spec §10.2] previous tick's free wheels (the spec body sets them at the end of that tick instead)
        if (m.free_roll[i] && !spec_body) m.omega[i] = vxs[i] / radius[i];
        vhub[i] = vcs[i].length();
        float fade = 0;
        m.alpha[i] = model::patch_slip_angle(float(vxs[i]), float(vys[i]), fade);
        fades[i] = fade;
        const double hub_fwd = spec_body ? vcs[i].dot(ground_axes.fwd) : vcs[i].dot(fwd_h);
        m.kappa[i] = model::slip_ratio(float(vxs[i]), float(m.omega[i] * radius[i]), float(hub_fwd));
        m.slip_g[i] = model::slip_gain(float(hub_fwd)); // [spec §3.4] from the hub forward speed
        if (!m.grounded[i]) continue;
        // Peak force [spec §3.1]: record by surface class (§6.5), load, mu[s], camber.
        const AxleSpec& as = s.axle[size_t(m.axle[i])];
        const TireSpec& ts = tire_for(int(i), sidx[i]);
        // [spec §3.1, draft 16] alpha = sigma (body roll - ground roll) + camber offset,
        // sigma = -1 left, +1 right; roll + = right side down.
        double sigma = (i % 2 == 0) ? -1.0 : 1.0;
        double roll_rel = -std::asin(clampd(A.lat.dot(n), -1, 1));
        float cam = as.camber.x.empty() ? 1.0f : model::lookup(as.camber, float(sigma * roll_rel + as.camber_offset));
        peaks[i] = model::tire_peak_force(ts, float(spec_body ? load_prev[i] : m.load[i]), sidx[i], cam) * as.axle_factor * wear_mul[i];
    }

    // --- engine and clutch [spec §2.2, §2.4] ---
    double G = model::total_ratio(s.gearbox, m.gear);
    // [spec §2.7, draft 18] layout and the engine-side axle speed per layout.
    const model::AwdLayout lay = model::awd_layout(int(d.drive), d.awd_system, d.awd_share_byte, d.awd_strength_byte);
    auto layout_axle_speed = [&](double wf, double wr) {
        if (lay.layout == 3) return 0.5 * (wf + wr);
        if (lay.layout == 7) return wr + lay.k * (wf - wr);
        return lay.driven_axle == 0 ? wf : wr;
    };
    const double w_shaft = layout_axle_speed(0.5 * (m.omega[0] + m.omega[1]), 0.5 * (m.omega[2] + m.omega[3]));
    double w_in = w_shaft * G; // engine-side speed of the gearbox input
    const double w_idle = s.engine.idle_rpm * model::kRpmToRadS;
    const double w_cap = (s.engine.rev_limit_rpm + 10) * model::kRpmToRadS;
    const double J = s.engine.inertia;

    // Traction control [spec §4.6]: throttle x clamp(1 + gain g (s_min - s* lat scale)).
    m.tcs_scale = 1;
    // [spec §4.6] CPU cars (CPU control source) and a scripted driver kind (!= 0, outside
    // the spawn counter) read the driver aid block, which is all zero: no TCS, no ASCC.
    const bool scripted_aids = cpu_driver_ || (driver_kind_ != 0 && m.spawn == 0);
    // [spec draft 24] the scripted path reads the AI's block (TCS from the driver parameters, no ASCC);
    // the human path the car's block. The handbrake only disables ASCC (§4.6).
    const float tcs_gain = scripted_aids ? scripted_aids_.tcs_gain : (s.aids.tcs ? s.aids.tcs_gain : 0.0f);
    const float tcs_a = scripted_aids ? scripted_aids_.tcs_lat_a : s.aids.tcs_lat_a;
    const float tcs_b = scripted_aids ? scripted_aids_.tcs_lat_b : s.aids.tcs_lat_b;
    const float tcs_sc = scripted_aids ? scripted_aids_.tcs_scale : s.aids.tcs_scale;
    if (tcs_gain > 0) {
        double s_min = 0;
        int wmin = -1;
        for (int i = 0; i < 4; ++i)
            if (driven(d.drive, i) && m.grounded[size_t(i)] && m.kappa[size_t(i)] < s_min) {
                s_min = m.kappa[size_t(i)];
                wmin = i;
            }
        if (wmin >= 0) {
            double g = m.slip_g[size_t(wmin)]; // [spec §4.6] g of the lowest-slip wheel
            // [spec §4.1/4.6, draft 18] lateral input |g / mu| = |4 a_y / sum(base grip x mu[s])|, the
            // records' base grip being 9.8 for every car: lateral g over the mean surface mu.
            double mu_sum = 0;
            for (int k = 0; k < 4; ++k) {
                const TireSpec& tk = tire_for(k, sidx[size_t(k)]);
                int sk = std::clamp(sidx[size_t(k)], 0, 15);
                mu_sum += tk.surface_mu[size_t(sk)] != 0 ? tk.surface_mu[size_t(sk)] : tk.mu * model::surface_mu(sk);
            }
            double g_mu = mu_sum > 0 ? std::fabs(4 * m.lat_g / mu_sum) : 0;
            m.g_over_mu = mu_sum > 0 ? 4 * m.lat_g / mu_sum : 0;
            float lat = model::tcs_lateral(float(g_mu), tcs_a, tcs_b);
            m.tcs_scale = model::tcs_factor(tcs_gain, float(g), float(s_min), s.tire[size_t(wmin / 2)].opt_drive, lat,
                                            tcs_sc);
        }
    }
    double throttle = std::clamp(double(in.throttle), 0.0, 1.0) * m.tcs_scale;
    tel.tcs_factor = float(m.tcs_scale);
    tel.throttle_processed = float(throttle);

    // Clutch-link transitions [spec §2.4, draft 18].
    if (G == 0) m.clutch = ClutchState::Open; // neutral
    // Clutch request, set by the driver layer every tick: 0 during the
    // spawn/stop counter; 0 for a rear-driven car with the handbrake on; engaged: 1
    // unless rpm < idle; open: 1 while the brake pedal < 0.25.
    double rpm_now = m.omega_e * model::kRadSToRpm;
    // [spec §2.4] the request is 0 while the race is frozen (race hold), during the spawn/stop counter and in driver kind 8
    bool clutch_request = model::clutch_request(int(m.clutch), float(rpm_now), s.engine.idle_rpm, in.brake,
                                                in.handbrake ? 1.0f : 0.0f, d.drive != Drive::FF,
                                                m.spawn > 0 || race_hold_ || driver_kind_ == 8);
    if (m.spawn > 0) { // the spawn hold zeroes the clutch request and holds gear 1
        m.gear = 1;
    } else if (cpu_driver_ || driver_kind_ != 0) { // [spec draft 23b] the scripted path (kind != 0) uses the AI rule
        // [spec §4.4 AI note] binary clutch request: from open, engage when rpm > 0.75 x
        // upshift rpm; once closed, stay engaged while rpm >= idle.
        double rpm = m.omega_e * model::kRadSToRpm;
        double up = s.engine.upshift_rpm > 0 ? s.engine.upshift_rpm : s.engine.rev_limit_rpm - 500;
        // [ai.md §10] "while open" / "engaged" refer to the clutch link state S, not to
        // the previous request (a moving spawn starts locked and stays engaged).
        m.cpu_clutch = m.clutch != ClutchState::Open ? rpm >= s.engine.idle_rpm : rpm > 0.75 * up;
        if (clutch_hold_) m.cpu_clutch = false; // [ai.md §10] countdown: request 0
        clutch_request = m.cpu_clutch;
    }
    // request 0 opens (1 -> 0 only this way); 0 -> 2 on the launch rules
    m.clutch = ClutchState(model::clutch_engage(int(m.clutch), clutch_request, G == 0 ? 0 : m.gear, float(v_fwd), float(throttle),
                                                float(rpm_now), s.engine.launch_rpm, s.engine.idle_rpm));
    if (s.engine.electric) {
        // Direct drive (mod EVs): connected whenever the driver layer asks for drive at all.
        // The AI rule above waits for revs an unloaded motor never has, so it is replaced by
        // "not held" here.
        bool on = (cpu_driver_ || driver_kind_ != 0) ? (m.spawn == 0 && !clutch_hold_) : clutch_request;
        m.clutch = on && G != 0 ? ClutchState::Locked : ClutchState::Open;
        if (m.clutch == ClutchState::Locked) m.omega_e = std::max(0.0, w_in); // connected straight away: the motor turns with the wheels
    }
    // [spec §2.2/§2.4] no re-sync to the wheel side here: a locked clutch set the engine speed at the end of
    // the last tick (axle speed x R_g, capped at rev limit + 10 rpm), and T_wot / friction (phase 2) and the
    // limiter's rpm word are read from that capped value [verified on the first limiter tick].
    // Shift clutch (3): the processed throttle is cut (wheel side slower) or blipped.
    if (m.clutch == ClutchState::Shift) {
        throttle = w_in < m.omega_e ? 0.0 : 1.0;
        tel.throttle_processed = float(throttle); // the processed throttle shows the cut / blip
    }

    {
        // [spec §2.2/2.3] turbo lag inside the engine-torque call: the load follows the
        // processed throttle x after TCS and the shift cut / blip, so boost decays
        // during a shift. The clutch engage rule above reads x itself, not the lagged x'.
        int n_rpm = std::max(int(m.omega_e * 60.0 / 6.283185307179586), int(s.engine.idle_rpm) - 1);
        throttle = model::turbo_step(s.engine.turbo, m.turbo, float(throttle), n_rpm, float(h));
        if (s.engine.turbo.on)
            tel.boost = model::turbo_boost(s.engine.turbo, float(m.turbo.lag[0]), float(m.turbo.lag[1])) - 1;
        tel.turbo_load = m.turbo.load;
        tel.turbo_lag = m.turbo.lag;
    }
    double rpm_e = m.omega_e * model::kRadSToRpm;
    model::EngineOut eo = model::engine_torque(s.engine, float(rpm_e), float(throttle), m.limiter,
                                               model::engine_condition_factor(m.cond_e, m.cond_o));
    m.limiter = eo.limiter;
    // [spec §2.4] T_g = T_c R_g: no driveline efficiency
    double shaft_torque = 0, clutch = 0, slip_dw = 0;
    if (m.clutch == ClutchState::Locked) {
        shaft_torque = eo.torque * G;
        clutch = 1;
    } else if (m.clutch == ClutchState::Slipping || m.clutch == ClutchState::Shift) {
        double dw = m.omega_e - w_in;
        // state 3 transmits the fixed +-shift-clutch torque byte x 9.8 N*m (never written: 0) [spec §2.4, §6.2]
        double tc = m.clutch == ClutchState::Shift ? (dw > 0 ? 1 : dw < 0 ? -1 : 0) * s.drivetrain.shift_clutch_torque
                                                   : double(model::clutch_slip_torque(eo.wot, eo.torque, float(dw)));
        double next = m.omega_e + h * (eo.torque - tc) / J;
        slip_dw = dw;
        m.omega_e = std::max(next, 0.0); // no idle floor while slipping (draft 15e)
        shaft_torque = tc * G;
        clutch = eo.wot > 0 ? std::min(1.0, std::fabs(tc) / eo.wot) : 0;
    } else if (s.engine.electric) {
        m.omega_e = 0; // neutral: the motor is not driven
    } else {
        m.omega_e = std::max(m.omega_e + h * eo.torque / J, w_idle); // open: no rev cap (draft 18)
    }
    tel.engine_torque = eo.torque;
    tel.clutch = float(clutch);
    tel.clutch_locked = m.clutch == ClutchState::Locked;
    tel.shifting = m.shift_lockout > 0;

    std::array<double, 4> tdrive{}, iw{};
    // [spec §2.7, draft 18] axle driveline: inertias J = shaft x final + 2 spin per axle,
    // reflected engine inertia J_e |R_g| only while locked, drive torque per layout.
    double ia[2], j_ax[2];
    for (int a = 0; a < 2; ++a) {
        const AxleSpec& axs = s.axle[size_t(a)];
        ia[a] = axs.spin_inertia > 0 ? axs.spin_inertia : axs.wheel_inertia;
        j_ax[a] = axs.shaft_inertia * s.gearbox.final_drive + 2 * ia[a];
    }
    const double j_ref = m.clutch == ClutchState::Locked ? J * std::fabs(G) : 0.0;
    model::AwdIn awd;
    awd.layout = lay.layout;
    awd.k = lay.k;
    awd.tg = float(shaft_torque);
    awd.ff = float(m.axle_torque[0]);
    awd.fr = float(m.axle_torque[1]);
    awd.wf = float(0.5 * (m.omega[0] + m.omega[1]));
    awd.wr = float(0.5 * (m.omega[2] + m.omega[3]));
    awd.handbrake = in.handbrake ? 1.0f : 0.0f;
    awd.brake = in.brake;
    awd.steer = float(0.5 * (m.steer[0] + m.steer[1]));
    awd.g_over_mu = float(m.g_over_mu);
    awd.clutch = m.clutch == ClutchState::Open ? 0 : m.clutch == ClutchState::Locked ? 1 : m.clutch == ClutchState::Slipping ? 2 : 3;
    awd.reverse = m.gear < 0;
    double drive_ax[2] = {0, 0};
    {
        // prediction for the tyre phase: per-wheel drive torque and effective inertia
        bool eng = m.centre_engaged;
        float jdf, jdr;
        model::AxleStep pr = model::axle_drive(lay, awd, eng, float(j_ax[0]), float(j_ax[1]), float(j_ref), jdf, jdr);
        drive_ax[0] = pr.drive_f;
        drive_ax[1] = pr.drive_r;
        const double jd_ax[2] = {jdf, jdr};
        const double jall = j_ax[0] + j_ax[1] + j_ref;
        for (size_t i = 0; i < 4; ++i) {
            size_t a = i / 2;
            iw[i] = (pr.common || lay.layout == 3) ? jall * 0.25 : jd_ax[a] * 0.5;
            tdrive[i] = drive_ax[a] * 0.5;
        }
    }

    // --- brakes [spec §4.5]: pressure x per-axle torque, ABS pressure limit, handbrake ---
    std::array<double, 4> tbrake{}, press{}; // brake torque, pressure after ABS/aids
    bool abs_any = false;
    // Under/oversteer indicators for ASCC [spec §4.6].
    const bool ascc_on = !in.handbrake && v_fwd > 0 && !scripted_aids;
    double ind_u = 0, ind_o = 0;
    // [spec §4.6] the indicators are computed for every car moving forward (u > 0);
    // only the ASCC that reads them is gated (handbrake, enable).
    if (v_fwd > 0) {
        // spec sign: steer + = left, the same sense as the yaw rate (+ = CCW); ours is + right.
        // a_y of the last lateral step (spec + = right, = our lat_g x g); the gates divide
        // by the sum of the four wheels' tyre-record base grip, 9.8 for every car (§6 quirk).
        constexpr double kBaseGripSum = 4 * 9.8;
        const double a_y = m.lat_g * model::kGravity;
        double lat_ratio = std::fabs(a_y) / kBaseGripSum;
        ind_u = model::understeer_indicator(float(v_fwd), float(-m.steer_avg), s.wheelbase, float(m.yaw_rate),
                                            float(m.yaw_acc), float(lat_ratio));
        // [spec §4.6] planar speed and the rear slip angle beta stored by the previous
        // tick's tyre phase (the aids run before this tick's tyre forces)
        double v_pl = std::hypot(m.vel.dot(ground_axes.fwd), m.vel.dot(ground_axes.lat));
        if (cpu_driver_ || driver_kind_ != 0) {
            // [draft 15d] control source CPU car or driver kind != 0 (pit
            // auto-drive): o = 2 max(0, |s r| - |a_y|) / sum base grip when s r a_y < 0, unclamped.
            double v_tot = v_pl;
            ind_o = model::oversteer_alt(float(v_tot), float(m.yaw_rate), float(a_y), float(kBaseGripSum));
        } else {
            double rear_b = spec_body ? m.rear_beta_prev : std::max(std::fabs(m.alpha[2]), std::fabs(m.alpha[3]));
            ind_o = model::oversteer_indicator(float(spec_body ? v_pl : v_fwd), float(rear_b));
        }
    }
    tel.understeer = float(ind_u);
    tel.oversteer = float(ind_o);
    for (size_t i = 0; i < 4; ++i) {
        int ax = m.axle[i];
        const AxleSpec& as = s.axle[size_t(ax)];
        double p = in.brake;
        if (m.dirt && ax == 0) p *= model::dirt_front_brake_scale(float(m.steer_avg)); // [spec §10.4]
        float cap_scale = 1;
        if (ascc_on) {
            // [spec §4.6] ASCC: A / C add brake pressure without the pedal; D / E relieve it with the pedal
            // and scale the wheel's ABS target slip by the same factor.
            bool outside = (m.yaw_rate > 0) == (i % 2 == 1); // left turn: right wheels are outside
            p = model::ascc_pressure(s.aids, ax, outside, float(p), float(ind_u), float(ind_o), in.brake, in.throttle,
                                     &cap_scale);
        }
        // [spec §4.5] ABS on every wheel with p > 0 (an unloaded wheel, Fp = 0, gets the 0.001 floor)
        if (s.aids.abs && p > 0 && (spec_body || m.grounded[i])) {
            double g = m.slip_g[i]; // [spec §4.5] the wheel's slip gain
            double lim = model::abs_pressure(float(p), float(peaks[i]), as.tire_radius, as.max_brake_torque,
                                             ax == 0 ? s.aids.abs_gain : s.aids.abs_gain_rear, float(g), float(m.kappa[i]),
                                             tire_for(int(i), sidx[i]).opt_brake * cap_scale);
            abs_any |= lim < p - 1e-6;
            p = lim;
        }
        double tb = p * as.max_brake_torque;
        press[i] = p;
        tel.brake_pressure[i] = float(p);
        tel.abs_target[i] = tire_for(int(i), sidx[i]).opt_brake * cap_scale;
        if (in.handbrake && ax == 1) tb += s.aids.handbrake_torque;
        tbrake[i] = tb;
    }

    // --- tyre forces [spec §3.1-3.3] ---
    std::array<double, 4> long_part{}, hold_f{}, long_gained{}; // spec body: C(s) part, hold, gained copy
    std::array<int, 4> slip_state{};
    std::array<double, 4> pl_slip_grip{}, pl_slip_drag{}, pl_drive_part{}, pl_brake_part{}, pl_phi{}; // spec body: §3.8 outputs
    double rear_beta_now = 0;
    auto coulomb = [](double w, double dw) { return std::fabs(w) <= dw ? 0.0 : w - std::copysign(dw, w); };
    for (size_t i = 0; i < 4; ++i) {
        if (!m.grounded[i]) continue;
        const TireSpec& ts = tire_for(int(i), sidx[i]);
        float peak = float(peaks[i]);
        // Velocity-limited friction [placeholder, keeps the 60 Hz step stable at
        // low speed]: the slip ratio is evaluated at the end-of-tick wheel and
        // road speeds predicted from drive, brakes and the non-tyre forces, and
        // the force never exceeds what stops that relative motion in one tick.
        D3 vp = vcs[i] + a0 * h;
        double w_pred = coulomb(m.omega[i] + h * tdrive[i] / iw[i], h * tbrake[i] / iw[i]);
        double v_long = vp.dot(wf[i]);
        double rel_long = v_long - w_pred * radius[i];
        model::TireIn ti;
        ti.vx = float(vxs[i]);
        ti.vy = float(vys[i]);
        ti.slip_ratio = model::slip_ratio(float(v_long), float(w_pred * radius[i]), float(vp.dot(fwd_h)));
        ti.rel = float(rel_long);
        ti.peak = peak;
        ti.scale = float(fades[i]);
        if (spec_body) {
            // [spec §3.8] the exact outputs from this tick's speeds.
            // Body-frame hub velocity (ground tangents, §10.1): forward uh, right lh.
            const D3 gfw = ground_axes.fwd, grt = ground_axes.lat;
            double uh = vcs[i].dot(gfw), lh = vcs[i].dot(grt);
            double phi = fades[i] == 0 ? 0.0 : std::atan2(-lh, uh); // velocity direction (§3.2)
            double wrim = m.omega[i] * radius[i];
            model::TyreCurves cv = model::tyre_curves(ts, float(m.kappa[i]));
            int st = model::slip_state(float(vxs[i]), float(wrim));
            // slip sign: the force opposes the hub/wheel relative motion [interpretation of sg]
            int sg = model::slip_sign(float(vxs[i]), float(wrim));
            double g = model::slip_gain(float(uh));
            model::TyreLong tl = model::tyre_long_force(cv.c, peak, st, sg, float(g));
            model::TyreLat tla = model::tyre_lateral(ts, float(-m.steer[i]), uh < 0, float(phi), float(fades[i]), peak, cv.d);
            double hold_p = model::low_speed_hold(peak, float(wrim), float(vxs[i]), float(vys[i]), 1.0f, 0.0f);
            pl_slip_grip[i] = tla.slip_grip;
            pl_slip_drag[i] = tla.slip_drag;
            pl_drive_part[i] = tl.drive_part;
            pl_brake_part[i] = tl.brake_part;
            pl_phi[i] = phi;
            if (i >= 2) rear_beta_now = std::max(rear_beta_now, double(std::fabs(tla.beta)));
            tel.beta[i] = tla.beta;
            hold_f[i] = hold_p;
            long_part[i] = double(tl.drive_part) + tl.brake_part;
            long_gained[i] = double(tl.drive_gained) + tl.brake_gained;
            slip_state[i] = st;
            // telemetry: the total in the wheel frame
            double bfx = -tla.slip_drag * std::cos(phi) - tla.slip_grip * std::sin(phi), bfy = tla.slip_drag * std::sin(phi) - tla.slip_grip * std::cos(phi);
            double ch = std::cos(m.steer[i]), sh = std::sin(m.steer[i]); // body -> wheel (steer + right)
            fx[i] = bfx * ch + bfy * sh + long_part[i];
            fy[i] = -bfx * sh + bfy * ch - hold_p;
            tel.fx[i] = float(fx[i]);
            tel.fy[i] = float(fy[i]);
            tel.long_force[i] = float(long_part[i]);
            tel.slip_grip[i] = tla.slip_grip;
            tel.slip_drag[i] = tla.slip_drag;
            tel.peak_force[i] = float(peak);
            tel.slip_gain[i] = float(m.slip_g[i]);
            continue;
        }
        model::TireOut to = model::tire_force(ts, ti);
        double cap_long = std::fabs(rel_long) / (h * (1.0 / m_eff + radius[i] * radius[i] / iw[i]));
        if (w_pred == 0 && tbrake[i] > 0) {
            // Wheel held by the brake: it can react up to the unused brake torque.
            double spare = std::max(0.0, tbrake[i] - std::fabs(tdrive[i]) - std::fabs(m.omega[i]) * iw[i] / h);
            cap_long = std::max(cap_long, std::min(m_eff * std::fabs(rel_long) / h, spare / radius[i]));
        }
        fx[i] = clampd(to.fx, -cap_long, cap_long);
        // [spec §4.1, draft 18] lateral: the tyre force (faded out below 10 km/h,
        // §3.2) plus the low-speed hold, a sideways damper below 5 km/h; no velocity cap.
        // The wheel frame stands in for the body frame of the hold's window tests.
        double hold = model::low_speed_hold(peak, float(m.omega[i] * radius[i]), float(vxs[i]), float(vys[i]), 1.0f, 0.0f);
        fy[i] = double(to.fy) - hold;
        if (spec_.axle[0].susp.valid) { // front-axle spec block only
            // [spec §4.1/4.2] tyre forces drive the planar motion and yaw only.
            D3 f = wf[i] * fx[i] + wr[i] * fy[i];
            force += f;
            D3 arm = m.contact[i] - m.pos;
            torque.y += arm.z * f.x - arm.x * f.z;
        } else {
            apply(wf[i] * fx[i] + wr[i] * fy[i], m.contact[i]);
        }
        tel.fx[i] = float(fx[i]);
        tel.fy[i] = float(fy[i]);
    }

    if (spec_body) m.rear_beta_prev = rear_beta_now;
    // --- wheel spin: drive, tyre reaction, Coulomb brakes ---
    const std::array<double, 4> w_old = m.omega;
    double w_side_after = 0; // engine-side axle speed
    {
        // [spec §2.7, draft 18]
        // Wheel net torque: -r Fx - sgn(w) brake (none at w == 0 exactly), plus torque vectoring.
        std::array<double, 4> tn{};
        for (size_t i = 0; i < 4; ++i)
            tn[i] = -(spec_body ? long_gained[i] : fx[i]) * radius[i] - (m.omega[i] == 0 ? 0.0 : std::copysign(tbrake[i], m.omega[i]));
        for (int ax = 0; ax < 2; ++ax) {
            int type = d.lsd_type[size_t(ax)];
            if (type != 6 && type != 7) continue;
            size_t l = size_t(2 * ax), r = l + 1;
            float tl = float(tn[l]), tr = float(tn[r]);
            model::torque_vectoring(type, d.lsd_initial[size_t(ax)], float(-m.steer_avg), float(v_fwd),
                                    float(m.omega[l] * radius[l]), float(m.omega[r] * radius[r]), tl, tr);
            tn[l] = tl;
            tn[r] = tr;
        }
        const double Ff = tn[0] + tn[1], Fr = tn[2] + tn[3];
        awd.ff = float(Ff);
        awd.fr = float(Fr);
        awd.wf = float(0.5 * (w_old[0] + w_old[1]));
        awd.wr = float(0.5 * (w_old[2] + w_old[3]));
        model::AxleStep st = model::axle_step(lay, awd, m.centre_engaged, float(j_ax[0]), float(j_ax[1]), float(j_ref),
                                              tbrake[0] + tbrake[1] > 0, tbrake[2] + tbrake[3] > 0, d.lsd_type[0] == 0,
                                              d.lsd_type[1] == 0, float(h));
        drive_ax[0] = st.drive_f;
        drive_ax[1] = st.drive_r;
        const double w_front = st.wf, w_rear = st.wr;
        // wheels per axle
        const double w_axle[2] = {w_front, w_rear};
        for (int ax = 0; ax < 2; ++ax) {
            size_t l = size_t(2 * ax), r = l + 1;
            int type = d.lsd_type[size_t(ax)] < 0 ? 2 : d.lsd_type[size_t(ax)];
            float wl, wrr;
            if (type == 0) { // f: each wheel alone, clamped at zero when braked
                wl = float(w_old[l] + tn[l] / ia[ax] * h);
                wrr = float(w_old[r] + tn[r] / ia[ax] * h);
            } else {
                wl = wrr = float(w_axle[ax]);
            }
            // [spec §2.7] the differential works on the peripheral speeds (m/s) with r / Ia: the
            // viscous transfer gain x min(dw^2, initial^2) takes dw in m/s [verified on an airborne
            // front wheel: in rad/s the transfer came out 1/r^2 (about 11x) too strong]
            const double rr_ax = s.axle[size_t(ax)].tire_radius;
            float wl_m = float(double(wl) * rr_ax), wr_m = float(double(wrr) * rr_ax);
            model::diff_step_prev(type, d.lsd_initial[size_t(ax)], d.lsd_accel[size_t(ax)], d.lsd_decel[size_t(ax)],
                                  float(drive_ax[ax]), float(ia[ax] / rr_ax), float(h), float(tn[l]), float(tn[r]),
                                  float((w_old[l] - w_old[r]) * rr_ax), wl_m, wr_m, m.diff[size_t(ax)], float(tbrake[l]), float(tbrake[r]),
                                  float(w_old[l] * rr_ax), float(w_old[r] * rr_ax));
            wl = float(double(wl_m) / rr_ax);
            wrr = float(double(wr_m) / rr_ax);
            m.omega[l] = wl;
            m.omega[r] = wrr;
            tel.diff_locked[size_t(ax)] = m.diff[size_t(ax)].locked;
            tel.diff_half[size_t(ax)] = m.diff[size_t(ax)].half;
        }
        m.axle_torque = {Ff, Fr};
        w_side_after = layout_axle_speed(w_front, w_rear);
        for (size_t i = 0; i < 4; ++i) tdrive[i] = drive_ax[i / 2] * 0.5;
    }
    {
        // [spec §10.2] a grounded, unbraked, undriven (and uncoupled) wheel is locked to
        // the ground: peripheral speed = the hub speed along its heading.
        // The flags (§3.8: load 0, brake, driven/coupled bits only while the clutch is engaged) are
        // this tick's; the overwrite uses the hub speeds at the end of the tick, so it is applied at
        // the start of the next tick, before the slip ratio (m.free_roll).
        const bool engaged_now = m.clutch != ClutchState::Open;
        for (size_t i = 0; i < 4; ++i) {
            const int ax = int(i / 2);
            bool both = lay.layout == 2 || lay.layout == 3 || lay.layout == 7;
            bool driven_w = engaged_now && (both || ax == (lay.layout == 1 ? 0 : 1));
            if (lay.layout >= 4 && ax == 0 && (m.centre_engaged || drive_ax[0] != 0)) driven_w = true;
            m.free_roll[i] = (spec_body ? load_prev[i] : m.load[i]) != 0 && tbrake[i] == 0 && !driven_w;
        }
    }
    model::engine_condition_step(m.cond_e, m.cond_o, float(std::fabs(v_fwd) * h)); // [spec §10.3]
    for (size_t i = 0; i < 4; ++i) {
        tel.drive_torque[i] = float(tdrive[i]);
        tel.brake_torque[i] = float(tbrake[i]);
    }
    for (size_t i = 0; i < 4; ++i) m.spin[i] = std::fmod(m.spin[i] + m.omega[i] * h, 6.283185307179586);
    // [spec §8.3] slip intensity (phase 12); airborne wheels keep it 5 more steps.
    for (size_t i = 0; i < 4; ++i) {
        if (!m.grounded[i]) {
            if (++m.air_steps[i] > 5) m.smoke[i] = m.smoke_emit[i] = 0;
            continue;
        }
        m.air_steps[i] = 0;
        float ramp = std::min(1.0f, 0.02f * float(vhub[i])); // wheel speed ramp (spec §3.6 / §8.3)
        m.smoke[i] = model::slip_intensity(sidx[i], float(vys[i]), float(m.kappa[i]), float(m.omega[i] * radius[i]), float(m.load[i]),
                                           ramp);
        // [spec §8.3b] smoke-emission intensity: slip speed = sgn(hub) (hub - rim).
        double slip_v = vxs[i] - m.omega[i] * radius[i];
        m.smoke_emit[i] = model::smoke_emission(sidx[i], float(vys[i]), float(vxs[i] < 0 ? -slip_v : slip_v), ramp);
    }
    // [spec §8.2] wear accumulation (phase 13): not on the pit surface, not on kerb surface 1.
    if (m.wear_p.on() && !m.pit_surface)
        for (size_t i = 0; i < 4; ++i) {
            if (m.load[i] == 0 || sidx[i] == 1) continue;
            const TireSpec& tw = tire_for(int(i), sidx[i]);
            m.wear[i] += model::wear_increment(m.smoke[i], m.wear_p.slip_cap, tw.wear_rate, float(m.load[i]),
                                               s.axle[size_t(m.axle[i])].tire_width, float(h));
        }
    // [spec §8.5] brake heat (visual only).
    for (size_t i = 0; i < 4; ++i) {
        const AxleSpec& as = s.axle[size_t(m.axle[i])];
        m.brake_heat[i] = model::brake_heat_step(m.brake_heat[i], as.max_brake_torque, as.tire_radius, float(press[i]),
                                                 float(m.omega[i] * radius[i]), float(h));
    }
    // [spec §8.7] tyres worn flag.
    tel.tires_worn = false;
    for (size_t i = 0; i < 4; ++i) {
        if (m.wear_p.on() && m.wear[i] >= 0 && m.wear_grip[i] < m.wear_p.worn) tel.tires_worn = true;
        tel.tire_wear[i] = m.wear[i];
        tel.wear_grip[i] = m.wear_grip[i];
        tel.wear_level[i] = m.wear_level[i];
        tel.smoke[i] = m.smoke[i];
        tel.smoke_emit[i] = m.smoke_emit[i];
        tel.brake_heat[i] = m.brake_heat[i];
    }
    tel.abs_active = abs_any;
    tel.tcs_active = m.tcs_scale < 1;
    {
        double w_in_after = w_side_after * G;
        // Slipping clutch: engine and wheel-side speeds crossed -> locked, held 12 steps [spec §2.4].
        // (in either direction, regardless of idle: draft 15e).
        bool slipping = m.clutch == ClutchState::Slipping || m.clutch == ClutchState::Shift;
        bool crossed = slipping && slip_dw * (m.omega_e - w_in_after) <= 0;
        const ClutchState before = m.clutch;
        m.clutch = ClutchState(model::clutch_after(int(m.clutch), crossed, in.brake, float(m.omega_e), s.engine.idle_rpm,
                                                   float(v_fwd), float(w_in_after)));
        if (s.engine.electric) m.clutch = before, crossed = false; // direct drive never slips
        if (crossed) m.shift_lockout = 12; // 2/3 -> 1 starts the 12-tick shift lockout
        tel.clutch_locked = m.clutch == ClutchState::Locked; // link state at the end of the clutch step
        if (m.clutch == ClutchState::Locked) m.omega_e = std::clamp(w_in_after, 0.0, w_cap);
    }

    // Hull contact with the ground (bottoming out) [placeholder]. The original has none (spec
    // §11): the ground is touched only through the four wheel probes, the bump stops and the
    // §4.3 nudge. Off for the spec body; GT3R_HULL_CONTACT=1 turns it back on for comparison.
    static const bool hull_on = [] {
        const char* e = std::getenv("GT3R_HULL_CONTACT");
        return e && e[0] == '1';
    }();
    if (!spec_body || hull_on) {
        double k = 80.0 * M, c = 2 * 0.7 * std::sqrt(k * M / 4);
        for (const D3& p_local : m.hull) {
            D3 p = m.pos + A.to_world(p_local - m.cg_local);
            SurfaceHit hit = track.probe(float(p.x), float(p.z), float(p.y + 0.5));
            if (!hit.hit) continue;
            D3 n = D3(hit.normal).normalized();
            double depth = n.dot(D3{p.x, hit.height, p.z} - p);
            if (depth <= 0) continue;
            D3 vp = m.vel + omega_w.cross(p - m.pos);
            double vn = vp.dot(n);
            double fn = std::max(0.0, k * depth - c * vn);
            D3 vt = vp - n * vn;
            double vtl = vt.length();
            D3 ff = vtl > 1e-4 ? vt * (-std::min(0.6 * fn, m_eff * vtl / h) / vtl) : D3{};
            apply(n * fn + ff, p);
        }
    }

    // --- integration [spec §1.1: explicit Euler, rate then value] ---
    const D3 start_pos = m.pos;
    const double start_heading = m.heading;
    double u_before = m.vel.dot(fwd_h);
    if (spec_body) force.y += M * model::kGravity; // gravity and heave are handled by the attitude step
    m.vel += force * (h / M);
    // Rolling resistance on the forward speed, with static hold [spec §3.5].
    if (!spec_body) {
        double sum = 0;
        for (size_t i = 0; i < 4; ++i)
            if (m.grounded[i]) sum += m.load[i] * model::surface_rolling(sidx[i]);
        double frr = model::rolling_curve(float(u_before)) * sum;
        double u = m.vel.dot(fwd_h);
        double du = -std::copysign(frr, u) / M * h;
        if (u != 0 && (u + du) * u < 0) du = -u;
        m.vel += fwd_h * du;
    }
    // Stop clamp [spec §4.1, condition interpreted]: below 1 km/h with the brake
    // applied and no drive, the planar velocities and yaw rate are zeroed.
    {
        // [spec §4.1, draft 14b] |v| < 1 km/h, every wheel's fade weight 0 and a
        // non-zero brake torque on at least one wheel.
        D3 v_planar{m.vel.x, 0, m.vel.z};
        bool fades_zero = true, braking = false;
        for (size_t i = 0; i < 4; ++i) {
            fades_zero = fades_zero && fades[i] == 0;
            braking = braking || tbrake[i] > 0;
        }
        if (!spec_body && v_planar.length() < 1 / 3.6 && fades_zero && braking) {
            m.vel.x = m.vel.z = 0;
            m.yaw_rate = 0;
        }
    }
    // Planar yaw and attitude (pitch about the lateral axis, roll about the forward axis).
    double m_yaw = torque.y;
    double m_pitch = torque.dot(A.lat), m_roll = torque.dot(A.fwd);
    if (!spec_body) m.yaw_rate = clampd(m.yaw_rate + m_yaw / m.i_yaw * h, -6.283185307179586, 6.283185307179586);
    if (spec_body) {
        // [spec §4.1] forward step, lateral force and yaw, stop clamp.
        // Body frame (forward, right); the spec's steer sign is + = left.
        model::PlanarWheel pw[4];
        float grip[4], fade4[4];
        bool any_brake = false;
        const bool engaged = m.clutch != ClutchState::Open;
        for (size_t i = 0; i < 4; ++i) {
            model::PlanarWheel& w = pw[i];
            double d_spec = -m.steer[i];
            w.k = float(std::cos(d_spec));
            w.s = float(std::sin(d_spec));
            const int ax = m.axle[i];
            // wheel flag byte: airborne, braked, driven / coupled (only while the clutch is engaged)
            // [spec §3.8] bits 2/3 while the clutch is engaged: L0, L4-6 rear; L1 front; L2, L3, L7 both;
            // L4-7 front also while the centre clutch is locked or carries torque.
            bool both = lay.layout == 2 || lay.layout == 3 || lay.layout == 7;
            bool driven_w = engaged && (both || ax == (lay.layout == 1 ? 0 : 1));
            if (lay.layout >= 4 && ax == 0 && (m.centre_engaged || drive_ax[0] != 0)) driven_w = true;
            // [spec §4.1, draft 21c] the forward step runs before the driveline adds the driven /
            // coupled bits: only airborne or braked wheels leave the effective mass.
            (void)driven_w;
            w.flagged = load_prev[i] == 0 || tbrake[i] > 0; // the tyre phase's load (previous tick's)
            w.ia_r2 = float(ia[ax] / (radius[i] * radius[i]));
            any_brake = any_brake || tbrake[i] > 0;
            fade4[i] = float(fades[i]);
            const TireSpec& tg = tire_for(int(i), sidx[i]);
            int sg = std::clamp(sidx[i], 0, 15);
            grip[i] = 9.8f * (tg.surface_mu[size_t(sg)] != 0 ? tg.surface_mu[size_t(sg)] : tg.mu * model::surface_mu(sg));
            if (!m.grounded[i]) continue;
            // [spec §3.8] slip_grip/slip_drag in the velocity-direction frame, drive_part/brake_part along the heading
            w.a = float(pl_phi[i]);
            w.slip_grip = float(pl_slip_grip[i]);
            w.slip_drag = float(pl_slip_drag[i]);
            w.drive_part = float(pl_drive_part[i]);
            w.brake_part = float(pl_brake_part[i]);
            w.hold = float(hold_f[i]);
        }
        double u = m.vel.dot(ground_axes.fwd); // §10.1 ground-frame forward speed
        double rr_sum = 0;
        for (size_t i = 0; i < 4; ++i)
            if (m.grounded[i]) rr_sum += load_prev[i] * model::surface_rolling(sidx[i]); // the load lags a tick
        double rolling = -(u > 0 ? 1 : u < 0 ? -1 : 0) * model::rolling_curve(float(std::fabs(u))) * rr_sum;
        double tf = 0.5 * s.axle[0].track, tr = 0.5 * s.axle[1].track;
        double a_len = s.wheelbase * (1 - s.front_weight), b_len = s.wheelbase * s.front_weight;
        float X[4] = {float(-tf), float(tf), float(-tr), float(tr)}, Y[4] = {float(a_len), float(a_len), float(-b_len), float(-b_len)};
        D3 fh{A.fwd.x, 0, A.fwd.z}, lh{A.lat.x, 0, A.lat.z};
        if (!all_airborne) {
            model::ForwardStep fs = model::forward_step(pw, float(u), float(M), float(rolling), 0.0f, float(aero.drag), float(h));
            // the forward step's longitudinal sum: rolling + sum cos d drive_part plus the tyre
            // reaction sum (k brake_part - sin a slip_grip - cos a slip_drag)
            double lsum = rolling;
            for (const model::PlanarWheel& w : pw)
                lsum += double(w.k) * (double(w.drive_part) + w.brake_part) - std::sin(double(w.a)) * w.slip_grip - std::cos(double(w.a)) * w.slip_drag;
            m.long_force_sum = lsum;
            model::LateralYaw ly = model::lateral_yaw(pw, X, Y, float(M), float(m.i_yaw), 0.0f, grip);
            planar_ax = fs.ax;
            planar_ay = ly.ay;
            m.g_over_mu = ly.g_over_mu;
            m.yaw_acc = double(ly.yaw_acc) + m_yaw / m.i_yaw;
            m.yaw_rate = clampd(m.yaw_rate + m.yaw_acc * h, -6.283185307179586, 6.283185307179586);
            // [spec §4.1/§9.3] the race hold (the start countdown) or driver kind 8 hold whatever
            // the speed. A hold zeroes the world x/y velocity, the yaw rate and the longitudinal force sum
            // the load transfer reads; the vertical velocity is kept.
            if (model::stop_hold(float(u), fade4, any_brake, race_hold_ || driver_kind_ == 8)) {
                m.vel.x = m.vel.z = 0;
                m.yaw_rate = 0;
                m.long_force_sum = 0;
            } else {
                m.vel += fh * (double(fs.ax) * h) + lh * (double(ly.ay) * h);
            }
        }
        if (m.spawn > 0) { // the spawn/stop counter zeroes the world x/y velocity and the yaw rate
            m.vel.x = m.vel.z = 0;
            m.yaw_rate = 0;
        }
    }
    if (!spec_body && m.spawn > 0) {
        // Cars without the spec suspension block: the counter (gear 1, clutch open)
        // runs without the scripted pose.
        int n = m.spawn--;
        if (n == 60 && m.wear_p.on()) fit_fresh_tires();
    }
    // [spec §1.2, §4.1 "Position and heading"] phase 10 advances the body by velocity x dt and the
    // heading by yaw rate x dt; phase 12 then queries the ground at the moved hubs. (The spawn hold
    // keeps the old order: its scripted pose moves the body itself.)
    const bool pre_moved = spec_body && m.spawn == 0;
    if (pre_moved) {
        m.pos += m.vel * h;
        m.heading += m.yaw_rate * h;
        Axes Am = make_axes(m.ax, m.ay, m.heading);
        D3 omega_m = D3{0, m.yaw_rate, 0} + Am.lat * m.ax_rate + Am.fwd * m.ay_rate;
        suspension_phase(Am, omega_m);
    } else if (spec_body) {
        suspension_phase(A, omega_w);
    }
    if (spec_body) {
        // [spec §4.2] body force += lateral factor x (-a_y) on the left wheels,
        // -= on the right; += longitudinal factor x (force sum - external force, always 0) on the front, -= on the rear; the full value on
        // each grounded wheel. a_y and the force sum are THIS tick's lateral step / tyre phase outputs.
        double ms = M - 2.0 * (s.axle[0].susp.unsprung + s.axle[1].susp.unsprung);
        double X = s.cg_height;
        double t_mean = 0.5 * (s.axle[0].track + s.axle[1].track);
        double d_long = 2.0 * (ms / M) * X / s.wheelbase * m.long_force_sum;
        double d_lat = ms * X * planar_ay / t_mean; // right accel loads the left
        for (size_t i = 0; i < 4; ++i) {
            if (!m.grounded[i]) continue;
            body_f[i] += (i < 2 ? 0.5 : -0.5) * d_long + (i % 2 == 0 ? -1.0 : 1.0) * d_lat;
        }
    }
    if (spec_body) {
        // [spec §4.3, draft 16] pitch/roll from the per-wheel body forces and dampers,
        // the longitudinal A/B terms; heave along the body up axis, gravity in world Y.
        float f[4], dd[4], cd[4], fxl[4];
        for (size_t i = 0; i < 4; ++i) {
            double lift = i < 2 ? aero.lift_front : aero.lift_rear;
            f[i] = float(body_f[i] - damper_f[i] - 0.5 * lift);
            dd[i] = float(damper_f[i]);
            cd[i] = float(std::cos(m.steer[i]));
            // [spec §4.3] q_i = cos d x (drive + brake parts G = drive_part + brake_part) x width: the lateral outputs
            // slip_grip / slip_drag do not enter, even projected on a steered wheel's axis
            fxl[i] = float(long_part[i]);
        }
        const double tp = 6.283185307179586;
        double a_len = s.wheelbase * (1 - s.front_weight), b_len = s.wheelbase * s.front_weight;
        double Y = s.yaw_inertia, Z = s.yaw_inertia * 0.5;
        float width[2] = {s.axle[0].tire_width, s.axle[1].tire_width};
        model::PitchTerms pt = model::pitch_terms(cd, fxl, width, float(Y), float(M), float(a_len), float(b_len));
        // Spawn hold [spec §4.3]: while the counter runs, a car at or below
        // ground + f(n) snaps there flat and at rest vertically, its heading eases to
        // the grid heading (<= 30 deg/s), and the attitude update is skipped.
        bool held = false;
        if (m.spawn > 0) {
            int n = m.spawn--;
            if (n == 60 && m.wear_p.on()) fit_fresh_tires(); // [spec §8.6] tyres replaced at counter 60
            double target = gf.height + model::spawn_height_offset(n);
            D3 origin = m.pos - A.to_world(m.cg_local);
            if (origin.y <= target) {
                held = true;
                m.ax = m.ay = m.ax_rate = m.ay_rate = 0;
                Axes Af = make_axes(0, 0, m.heading);
                D3 o{origin.x, target, origin.z};
                m.pos = o + Af.to_world(m.cg_local);
                m.vel.y = 0;
                if (n >= 30 && n < 270) {
                    double dh = std::remainder(m.spawn_heading - m.heading, 6.283185307179586);
                    double step = 30.0 * 3.14159265358979 / 180.0 * h;
                    m.heading += std::fabs(dh) <= step ? dh : std::copysign(step, dh);
                }
                m.pos.x += m.vel.x * h;
                m.pos.z += m.vel.z * h;
                m.heading += m.yaw_rate * h;
            }
        }
        if (held) {
            // attitude update skipped this step
        } else {
        float pitch = float(m.ax), prate = float(m.ax_rate), roll = float(m.ay), rrate = float(m.ay_rate);
        float heave = model::attitude_step(f, dd, float(a_len * tp / Y), float(b_len * tp / Y),
                                           float(0.5 * s.axle[0].track * tp / Z), float(0.5 * s.axle[1].track * tp / Z),
                                           pt, float(h), pitch, prate, roll, rrate);
        m.ax = pitch;
        m.ax_rate = prate;
        m.ay = roll;
        m.ay_rate = rrate;
        double ms = M - 2.0 * (s.axle[0].susp.unsprung + s.axle[1].susp.unsprung);
        Axes An = make_axes(m.ax, m.ay, m.heading);
        m.vel += An.up * (heave / ms * h);
        m.vel.y -= model::kGravity * h;
        if (!pre_moved) { // [spec §4.3 step 5] heave changes the velocity only; position moves in phase 10
            m.pos += m.vel * h;
            m.heading += m.yaw_rate * h;
        }
        // Bump-stop attitude nudge [spec §4.3].
        float pen[4];
        for (size_t i = 0; i < 4; ++i) {
            const SuspensionSpec& sp = s.axle[size_t(m.axle[i])].susp;
            pen[i] = m.grounded[i] ? std::max(0.0f, model::bump_limit(sp, float(m.xs[i])) - float(m.x[i])) : 0.0f;
        }
        model::Nudge nd;
        if (model::bump_nudge(pen, 1 - s.front_weight, 0.5f * (s.axle[0].track + s.axle[1].track), s.wheelbase, nd)) {
            m.ay += nd.roll;
            m.ax += nd.pitch;
            m.pos.y += nd.height;
            // Downward speed limited to the velocity's component along the ground plane.
            D3 n = normal[0];
            D3 vg = m.vel - n * m.vel.dot(n);
            m.vel.y = std::max(m.vel.y, std::min(0.0, vg.y));
        }
        }
    } else {
        m.ax_rate += m_pitch / m.i_pitch * h;
        m.ay_rate += m_roll / m.i_roll * h;
        m.pos += m.vel * h;
        m.heading += m.yaw_rate * h;
        m.ax += m.ax_rate * h;
        m.ay += m.ay_rate * h;
        if (std::fabs(m.ax) > kMaxAttitude) { m.ax = std::copysign(kMaxAttitude, m.ax); m.ax_rate = 0; }
        if (std::fabs(m.ay) > kMaxAttitude) { m.ay = std::copysign(kMaxAttitude, m.ay); m.ay_rate = 0; }
    }
    m.heading = std::remainder(m.heading, 6.283185307179586);

    // --- walls [spec §5.3]: impulse e = 0.05, friction mu = 0.5 (no torque),
    // collision yaw inertia 10 x I_z, push-out, stuck assist. Contact geometry
    // comes from the track's wall_contact (spheres around the body), not the
    // game's swept octagon [placeholder].
    if (world) {
        double reach = m.oct_radius + std::sqrt(m.vel.x * m.vel.x + m.vel.z * m.vel.z) * h + 1.0;
        m.segs.clear();
        if (world->wall_segments(m.pos.f(), float(reach + 8.0), m.segs)) {
            // [spec §5.3] swept octagon vs wall segments, 2 iterations, push-out.
            collide::Body b = collision_body();
            std::vector<collide::WallHit> wall_hits =
                collide::collide_walls(b, {start_pos.x, start_pos.z}, start_heading, m.segs, h, 2, &m.wall_contacts);
            bool hit_any = false;
            for (const auto& w : wall_hits) hit_any |= !w.sensor;
            set_collision_body(b);
            if (hit_any) {
                // Stuck assist: pinned against a wall, the steering still turns the car.
                if (std::fabs(m.vel.x) < 1 && std::fabs(m.vel.z) < 1) m.yaw_rate += -in.steer * 20.0 * h;
                tel.wall_impulse = 0;
                for (const auto& w : wall_hits) tel.wall_impulse += float(w.impulse);
            }
        } else {
            // Fallback: spheres around the body against wall_contact() [placeholder geometry].
            m.wall_contacts.clear();
            Axes A2 = make_axes(m.ax, m.ay, m.heading);
            const double i_coll = 10.0 * m.i_yaw;
            bool hit_any = false;
            for (const D3& p_local : m.wall_pts) {
                D3 p = m.pos + A2.to_world(p_local - m.cg_local);
                Vec3 n_f;
                float pen = 0;
                if (!world->wall_contact(p.f(), float(m.wall_radius), n_f, pen) || pen <= 0) continue;
                hit_any = true;
                D3 n = D3{n_f.x, 0, n_f.z}.normalized();
                m.pos += n * double(pen);
                D3 r = p - n * m.wall_radius - m.pos;
                collide::Body b = collision_body();
                b.inv_inertia = 1.0 / i_coll;
                tel.wall_impulse += float(collide::wall_impulse(b, {r.x, r.z}, {n.x, n.z}));
                set_collision_body(b);
            }
            if (hit_any && std::fabs(m.vel.x) < 1 && std::fabs(m.vel.z) < 1) m.yaw_rate += -in.steer * 20.0 * h;
        }
        m.yaw_rate = clampd(m.yaw_rate, -62.83, 62.83);
    }

    tel.accel_local = A.to_car(force * (1.0 / M) + D3{0, model::kGravity, 0}).f();
    if (spec_body) { // the planar step's accelerations (the tyre forces are not in `force`)
        // [spec §4.1] a_y of the lateral step alone, (sum Fa - ext) / mass (0 when airborne).
        // The body's rolled lateral axis would add g sin(roll) from `force` (about +3% in a steady
        // corner); the original game's value, and the load transfer / TCS / indicators reading it, have none.
        tel.accel_local.x = float(planar_ay);
        tel.accel_local.z -= float(planar_ax);
    }
    m.lat_g = tel.accel_local.x / model::kGravity;
    state_.throttle = eo.throttle;

    if (spec_body) {
        // [spec §10.1/§10.2, end of phase 12] a free wheel (grounded, unbraked, not driven or
        // coupled this tick) is locked to the ground: wheel speed = cos d (u + X r) - sin d (l - Y r), with u / l the
        // world velocity on the ground-frame tangents, half-track levers X = (-tf, +tf, -tr, +tr), axle
        // distances Y = (a, a, -b, -b) and d = this tick's wheel steer angle (+ = left). The next tick's slip
        // ratio reads it.
        const Axes Ag = make_axes(m.late_gf.pitch, m.late_gf.roll, m.heading);
        for (size_t i = 0; i < 4; ++i) {
            if (!m.free_roll[i]) continue;
            double uh, lh;
            hub_velocity(i, Ag, uh, lh); // the same expression the next tick's slip ratio uses, so it reads exactly 0
            m.omega[i] = along_heading(uh, lh, m.steer[i]) / s.axle[size_t(m.axle[i])].tire_radius;
        }
    }

    // [spec §2.4] end-of-tick gear change (see select_gear).
    select_gear(m.vel.dot(ground_u || (spec_body && m.late_init) ? make_axes(m.late_gf.pitch, m.late_gf.roll, m.heading).fwd
                                                                : make_axes(m.ax, m.ay, m.heading).fwd));

    // NaN guard.
    bool ok = m.pos.finite() && m.vel.finite() && std::isfinite(m.heading) && std::isfinite(m.ax) &&
              std::isfinite(m.ay) && std::isfinite(m.yaw_rate) && std::isfinite(m.omega_e);
    for (size_t i = 0; i < 4; ++i) ok = ok && std::isfinite(m.omega[i]);
    if (!ok) {
        m = saved;
        m.vel = {};
        m.yaw_rate = m.ax_rate = m.ay_rate = 0;
        for (auto& w : m.omega) w = 0;
    }

    tel.clutch_state = int(m.clutch);
    tel.g_over_mu = float(m.g_over_mu);
    tel.yaw_acc = float(m.yaw_acc);
    tel.pitch = float(m.ax);
    tel.roll = float(m.ay);
    tel.pitch_rate = float(m.ax_rate);
    tel.roll_rate = float(m.ay_rate);
    {
        const Axes Ag = spec_body && m.late_init ? make_axes(m.late_gf.pitch, m.late_gf.roll, m.heading) : make_axes(m.ax, m.ay, m.heading);
        tel.ground_u = float(m.vel.dot(Ag.fwd));
        tel.ground_l = float(m.vel.dot(Ag.lat));
    }
    telemetry_ = tel;
    telemetry_.steer_angle = float(m.steer_avg);
    telemetry_.steer_rate = float(m.steer_rate);
    publish();
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
        // Spec path: the coordinate after the droop / bump-stop clamp (x_prev, §10.5): the raw x
        // follows the ground's distance, which drew airborne wheels on the road under the car.
        const bool spec_susp = spec_.axle[size_t(m.axle[i])].susp.valid;
        w.suspension = float(spec_susp ? -m.x_prev[i] : m.x[i]);
        w.slip_ratio = float(m.kappa[i]);
        w.slip_angle = float(m.alpha[i]);
        w.load = float(m.load[i]);
        w.grounded = m.grounded[i];
        w.surface = m.surface[i];
    }
}

} // namespace gt3r::sim
