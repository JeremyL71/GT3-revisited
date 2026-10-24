// AI driver decision logic. Section numbers in the comments refer to the
// behaviour notes the code follows; values marked [placeholder] are ours.

#include "ai/ai_driver.h"

#include "ai/ai_rules.h"
#include "sim/collision.h"
#include "sim/models.h"
#include "sim/vehicle.h"
#include "track/track.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <numbers>

namespace gt3r::ai {

namespace {

using sim::Vec3;
constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kDeg = kPi / 180.0f;
constexpr float kKmh = 1.0f / 3.6f;
constexpr float kG = 9.8f;

Vec3 rotate(const sim::Quat& q, const Vec3& v) {
    Vec3 u{q.x, q.y, q.z};
    Vec3 t = u.cross(v) * 2.0f;
    return v + t * q.w + u.cross(t);
}

Vec3 flat(Vec3 v) {
    v.y = 0;
    float l = v.length();
    return l > 1e-6f ? v * (1.0f / l) : Vec3{0, 0, -1};
}

// Zone bytes (ai §8.1), as bits.
enum Zone : u8 { kFront = 1, kFrontLeft = 2, kFrontRight = 4, kLeft = 8, kRight = 16, kRearLeft = 32, kRearRight = 64, kRear = 128 };

} // namespace

struct AiDriver::Impl {
    // Countdown blipper (ai §10): pedal and hysteresis state.
    float countdown_throttle = 0;
    bool blip_up = false;
    // Car-derived constants (first update).
    bool car_ready = false;
    float alpha_peak = 0.14f;          // front tyre peak slip angle (rad)
    float k_down = 0;                  // downforce per mass, relative to g, per (m/s)^2
    float brake_floor = 0.04f;         // s^2/m: minimum braking-distance coefficient
    float v_top = 80;                  // m/s
    float front_arm = -1.3f;           // car-frame z of the front axle

    // Line state.
    bool placed = false;
    int line = AiCourse::kMain, prev_line = AiCourse::kMain;
    float blend = 0;
    int next = 0;                  // next control point index
    int mode = 0, next_kind = 0;   // current segment mode, kind of the segment at `next`
    float v_cur = 0, v_next = 0;   // targets for the current and the next segment
    int status = 0;                // braking/throttle status used by the traffic logic
    int arc_ticks = 0;
    int cooldown = 0;
    int state = 0;                 // recovery state machine (ai §7)
    int kind = 0;                  // driver kind (ai §1)
    bool pit_limit = false;        // on the stop line or in the pit lane's limit section (last tick)
    float steer = 0;               // road-wheel angle command (rad)
    float throttle = 0;            // applied throttle (slew in arcs)
    float brake = 0;               // last brake output (the stop driver's kind 5 keeps it)
    bool reverse = false;          // reverse-gear request
};

AiDriver::AiDriver(std::shared_ptr<const AiCourse> course, DriverParams params, int start_offset)
    : course_(std::move(course)), p_(params), start_offset_(start_offset), m_(std::make_unique<Impl>()) {}

AiDriver::~AiDriver() = default;

void AiDriver::reset(const sim::VehicleState& st) {
    Impl& m = *m_;
    const track::Track& t = course_->track();
    progress_ = t.locate(st.position).distance;
    m.state = 0;
    m.steer = 0;
    m.throttle = 0;
    m.brake = 0;
    m.reverse = false;
    m.countdown_throttle = 0, m.blip_up = false;
    m.cooldown = 0;
    m.blend = 0;
    m.pit_limit = false;
    m.placed = true;
    // Initial line: nearest of 6, 2, 1 at the car's progress within 100 m.
    int best = course_->resolve(AiCourse::kMain);
    float best_d = 100;
    for (int cand : {6, 2, 1}) {
        int l = course_->resolve(cand);
        if (l != cand) continue;
        Vec3 p = course_->evaluate(l, progress_);
        float d = std::hypot(p.x - st.position.x, p.z - st.position.z);
        if (d < best_d) best_d = d, best = l;
    }
    m.line = m.prev_line = best;
    m.next = -1; // re-initialised on the first update (needs the car)
}

bool AiDriver::reverse_request() const { return m_->reverse; }

void AiDriver::set_line(int game_line, bool blend) {
    int l = course_->resolve(game_line);
    if (l < 0) return;
    m_->prev_line = blend ? m_->line : l;
    m_->line = l;
    m_->blend = blend ? 1.0f : 0.0f;
    m_->next = -1;
}

void AiDriver::reset_recovery() { m_->state = 0; }

void AiDriver::set_kind(int kind) { m_->kind = kind; }

int AiDriver::kind() const { return m_->kind; }

sim::ControlInputs AiDriver::update(const sim::Vehicle& car, const AiContext& ctx, double dt_d) {
    Impl& m = *m_;
    const float dt = float(dt_d);
    const sim::VehicleState& st = car.state();
    const sim::CarSpec& spec = car.spec();
    const sim::VehicleTelemetry& tel = car.telemetry();
    const AiCourse& course = *course_;
    const track::Track& trk = course.track();
    if (!m.placed) reset(st);

    if (!m.car_ready) {
        m.car_ready = true;
        // Peak slip angle: argmax over whole degrees 0..45 (ai §6.1), <= 0.8 rad.
        const sim::TireSpec& tf = spec.tire[0];
        float best = -1;
        for (int d = 0; d <= 45; ++d) {
            float v = sim::model::lookup(tf.slip_a, d * kDeg);
            if (v > best) best = v, m.alpha_peak = std::min(d * kDeg, 0.8f);
        }
        m.alpha_peak = std::max(m.alpha_peak, 2 * kDeg);
        m.k_down = (spec.aero.c_front + spec.aero.c_rear) / std::max(spec.mass * kG, 1.0f);
        m.brake_floor = 1.0f / (2.0f * std::max(spec.brake_decel, 0.3f) * kG);
        float cd = spec.aero.c_drag > 0 ? spec.aero.c_drag : 0.5f * spec.aero.air_density * spec.aero.drag_area;
        m.v_top = std::clamp(std::cbrt(std::max(spec.peak_power_kw(), 30.0f) * 1000.0f / std::max(cd, 0.1f)), 30.0f, 120.0f);
        m.front_arm = car.setup().wheels[0].rest_center.z;
    }

    // --- own state ------------------------------------------------------------
    Vec3 fwd = flat(rotate(st.orientation, {0, 0, -1}));
    Vec3 right{-fwd.z, 0, fwd.x};
    float v = st.velocity.dot(fwd);                  // speed along the flattened heading
    float speed = std::hypot(st.velocity.x, st.velocity.z);
    float yaw_r = -st.angular_velocity.y;            // yaw rate, + = turning right
    progress_ = trk.locate(st.position, progress_).distance;
    const float s = progress_;
    // The tyre wear factors scale the planned grip except on the pit surface (ai §4.2, ai §13).
    const bool live_wear = !tel.on_pit_surface;

    // Front-wheel velocity angle and the steering slew (ai §6), shared with the stop driver.
    auto slew_steer = [&](float target) {
        float max_step = p_.steer_rate * dt;
        m.steer += std::clamp(target - m.steer, -max_step, max_step);
    };
    auto outputs = [&](float thr, float brk) {
        sim::ControlInputs in;
        in.automatic = true;
        in.throttle = std::clamp(thr, 0.0f, 1.0f);
        in.brake = brk;
        float lock = sim::model::steer_target(spec.steering, 1.0f, std::fabs(v));
        in.steer = lock > 1e-4f ? std::clamp(m.steer / lock, -1.0f, 1.0f) : 0.0f; // for displays
        in.steer_angle = m.steer; // CPU cars set the road-wheel angle directly (ai §1)
        m.brake = brk;
        dbg_.steer_angle = m.steer;
        dbg_.throttle = in.throttle;
        dbg_.brake = in.brake;
        dbg_.kind = m.kind;
        dbg_.state = m.state;
        dbg_.line = m.line;
        dbg_.progress = s;
        return in;
    };
    // Parked (kind 8, ai §1.1): wheels back to straight, no pedals (the physics opens the clutch).
    if (m.kind == kKindParked && ctx.race_time >= 0) {
        m.blend = std::max(0.0f, m.blend - dt);
        slew_steer(0);
        m.throttle = 0, m.reverse = false;
        return outputs(0, 0);
    }
    // The stop driver (kinds 5, 6, 7; ai §1.1): no throttle, the brake on (kind 5 keeps the last
    // brake), the wheels steered to `target`; once the car stands still it is parked (kind 8) and
    // the brake comes off.
    auto stop_driver = [&](float target) {
        slew_steer(target);
        m.throttle = 0, m.reverse = false;
        float brk = m.kind == kKindStopFree ? m.brake : 1.0f;
        if (parked(st)) m.kind = kKindParked, brk = 0;
        return outputs(0, brk);
    };
    if (m.kind == kKindStopStraight && ctx.race_time >= 0) {
        m.blend = std::max(0.0f, m.blend - dt);
        return stop_driver(0);
    }

    // Corner speed of the arc starting at a point (ai §4.2): the record grip of its surface with
    // the live wear factors, the downforce fixed point, the set's scale, offset and the point's
    // km/h adjustment.
    const AiCourse::Line* L = course.line(m.line);
    auto segment_target = [&](int idx) {
        const auto& pt = L->points[size_t(idx)];
        if (!pt.kind) return m.v_top;
        float mu = corner_grip(spec, pt.surface, p_.grip_front_weight, tel.wear_grip, live_wear);
        dbg_.grip = mu;
        return corner_speed(mu, pt.radius, pt.bank, pt.speed_adjust, p_, m.k_down, m.v_top);
    };
    // Point index change (ai §4.1).
    auto shift_points = [&]() {
        int n = int(L->points.size());
        m.v_cur = m.v_next;
        m.mode = m.next_kind;
        int idx = m.next >= n ? n - 1 : m.next;
        m.next_kind = L->points[size_t(idx)].kind;
        m.v_next = segment_target(idx);
        m.arc_ticks = 0;
    };
    auto init_points = [&]() {
        int n = int(L->points.size());
        m.next = course.next_point(m.line, s);
        int cur = (m.next + n - 1) % n;
        m.next_kind = L->points[size_t(cur)].kind;
        m.v_next = segment_target(cur);
        shift_points();
    };
    auto switch_line = [&](int wanted) {
        int l = course.resolve(wanted);
        if (l < 0) return false;
        m.prev_line = m.line;
        m.line = l;
        m.blend = 1;
        L = course.line(l);
        init_points();
        return true;
    };
    if (!L) {
        // No AI line at all: coast.
        sim::ControlInputs none;
        none.brake = 1;
        return none;
    }
    if (m.next < 0) init_points();
    {
        int n = int(L->points.size());
        int nn = course.next_point(m.line, s);
        if (nn != m.next && !(L->lap && nn == 0 && m.next == n)) {
            m.next = nn;
            shift_points();
        }
    }
    m.blend = std::max(0.0f, m.blend - dt);

    // --- traffic inputs (ai §8.1) ----------------------------------------------
    u8 zones = 0;
    float contact = 0;
    if (ctx.self >= 0 && size_t(ctx.self) < ctx.cars.size()) {
        const CarSnapshot& me = ctx.cars[size_t(ctx.self)];
        for (size_t j = 0; j < ctx.cars.size(); ++j) {
            if (int(j) == ctx.self || !ctx.cars[j].active) continue;
            const CarSnapshot& o = ctx.cars[j];
            Vec3 d = o.position - me.position; // CoG to CoG (the snapshots hold the CoG)
            float d2 = d.dot(d);
            if (d2 >= 400 || std::fabs(d.y) > 5) continue; // only pairs within 20 m (centre distance, ai §14)
            float u = d.dot(fwd), lat = d.dot(right);
            // Combined extents: ahead = own front + other's rear, behind = own rear + other's front.
            float lf = me.front + o.rear, lr = me.rear + o.front, W = me.half_width + o.half_width;
            {
                if (u > lf) {
                    if (lat < -W) zones |= kFrontLeft;
                    else if (lat > W) zones |= kFrontRight;
                    else zones |= kFront | (lat > 0 ? kFrontRight : kFrontLeft);
                } else if (u < -lr) {
                    if (lat < -W) zones |= kRearLeft;
                    else if (lat > W) zones |= kRearRight;
                    else zones |= kRear | (lat > 0 ? kRearRight : kRearLeft);
                } else if (lat < 0) {
                    zones |= kLeft | (u > 0 ? kFrontLeft : kRearLeft);
                } else {
                    zones |= kRight | (u > 0 ? kFrontRight : kRearRight);
                }
            }
            // Contact intensity: closing on a car ahead within the combined half widths, bumper to
            // bumper gap p, forward closing speed q (ai §8.1).
            float gap = u - lf;
            float closing = (st.velocity * me.time_scale - o.velocity * o.time_scale).dot(fwd);
            if (gap > 0 && std::fabs(lat) <= W && closing > 0) {
                float c = 0;
                if (closing > 10) c = 1;
                else if (closing > 0.2778f && gap / closing < 3) c = 0.5f;
                if (c == 0 && gap < 2) c = 0.25f;
                contact = std::max(contact, c);
            }
        }
    }

    // --- braking point (ai §4.3) --------------------------------------------------
    float throttle_mult = 1;
    m.status = 0;
    int n_pts = int(L->points.size());
    int npt = std::min(m.next, n_pts - 1);
    const AiCourse::Point& np = L->points[size_t(npt)];
    float dist_np = std::hypot(np.pos.x - st.position.x, np.pos.z - st.position.z);
    float brake_d = 0;
    if (m.line != AiCourse::kPitStop && m.mode != 3 && m.mode != 4 && m.next_kind != 0 && v > m.v_next) {
        float mu_f = braking_grip(spec, np.surface, tel.wear_grip, live_wear);
        float coef = std::max(m.brake_floor, 1.0f / (2.0f * std::max(mu_f, 1.0f)));
        brake_d = (v * v - m.v_next * m.v_next) * coef * (1 - std::sin(np.slope)) *
                  std::max(0.1f, 1 + 0.01f * float(np.brake_adjust));
        if (p_.brake_factor < 1) brake_d *= p_.brake_factor;
        if (m.cooldown == 0 && contact > 0) brake_d += 20 * contact;
        if (brake_d >= dist_np) {
            m.mode = m.mode != 0 ? 4 : 3;
            m.status = 2;
        } else {
            float soft = p_.brake_factor > 1 ? brake_d * (p_.brake_factor - 1) : 0;
            float margin = dist_np - brake_d;
            if (margin < soft + 50) m.status = 1;
            if (soft > 0 && margin < soft) throttle_mult = margin / soft;
        }
    }
    if (m.line != AiCourse::kPitLane) throttle_mult *= p_.throttle_scale;

    // --- target point (ai §3) ----------------------------------------------------
    float look;
    if (m.kind == kKindPitStop) look = 5;                        // on the stop line
    else if (m.kind == kKindPitLeave && m.pit_limit) look = 20;  // leaving, in the pit-lane limit
    else if (m.state == 1) look = 10;
    else if (m.state == 2) look = 40;
    else {
        float f = v <= 0 ? 0 : std::min(1.0f, v / std::max(p_.lookahead_full_speed, 1.0f));
        look = p_.lookahead_min + (p_.lookahead_max - p_.lookahead_min) * f;
        double ms = ctx.race_time * 1000.0;
        if (ms < 2000) look += float(40.0 * (2000.0 - std::max(ms, 0.0)) / 2000.0);
    }
    Vec3 target;
    for (int k = 0; k < 10; ++k) {
        target = course.evaluate(m.line, s + look);
        if (m.blend > 0) {
            Vec3 old = course.evaluate(m.prev_line, s + look);
            target = target + (old - target) * m.blend;
        }
        if (std::hypot(target.x - st.position.x, target.z - st.position.z) >= 5) break;
        look += 1;
    }
    Vec3 pv = target - st.position;
    pv.y = 0;
    float eps = std::atan2(pv.dot(right), pv.dot(fwd)); // + = target to the right
    float aeps = std::fabs(eps);

    // --- drive decision and recovery (ai §7) -------------------------------------
    // The collision sides of the last step (the front and rear impulse flags) drive the state
    // machine; there are no timers.
    const float theta = recovery_threshold(m.state, v);
    const int drive = drive_decision(m.state, aeps, theta, v);
    const bool brake_flag = drive == kDriveBrake;
    m.state = recovery_next(m.state, aeps, theta, v, tel.hit_sides);
    // Below 2 km/h either way the wheels are steered straight.
    const int moving = v > 2 * kKmh ? 1 : v < -2 * kKmh ? -1 : 0;

    // --- steering (ai §6) ------------------------------------------------------------
    float steer_t = 0, saturation = 0, r_des = 0, alpha = 0;
    if (moving != 0) {
        if (drive == kDriveForward) {
            if (m.state != 0 && aeps > 80 * kDeg) {
                alpha = eps > 0 ? m.alpha_peak : -m.alpha_peak;
            } else {
                Vec3 vd = speed < 5 * kKmh ? fwd : flat(st.velocity);
                Vec3 vr{-vd.z, 0, vd.x};
                float y = pv.dot(vr), d2 = std::max(pv.dot(pv), 1.0f);
                r_des = 2 * (moving > 0 ? speed : -speed) * y / d2;
                if (r_des > 0 && yaw_r < r_des) alpha = p_.yaw_gain * (r_des - yaw_r);
                else if (r_des < 0 && yaw_r > r_des) alpha = -p_.yaw_gain * (yaw_r - r_des);
                else if (p_.counter_steer) alpha = p_.yaw_gain * (r_des - yaw_r);
                if (std::fabs(alpha) > m.alpha_peak) {
                    saturation = std::fabs(alpha) / m.alpha_peak;
                    alpha = alpha > 0 ? m.alpha_peak : -m.alpha_peak;
                }
            }
            // Velocity direction at the front axle (zero-slip steer angle).
            Vec3 arm = rotate(st.orientation, {0, 0, m.front_arm});
            Vec3 vw = st.velocity + st.angular_velocity.cross(arm);
            float vf = vw.dot(fwd);
            float beta = std::fabs(vf) > 0.5f ? std::atan2(vw.dot(right), std::fabs(vf)) : 0.0f;
            steer_t = std::clamp(alpha + beta, -45 * kDeg, 45 * kDeg);
        } else {
            steer_t = eps < -20 * kDeg ? -30 * kDeg : eps > 20 * kDeg ? 30 * kDeg : eps * 1.5f;
            if (moving < 0) steer_t = -steer_t;
        }
    }
    // The stop driver along the line (kind 6; kind 5 without its brake): this steering, then stop.
    if ((m.kind == kKindStopLine || m.kind == kKindStopFree) && ctx.race_time >= 0) {
        dbg_.heading_error = eps;
        dbg_.target = target;
        return stop_driver(steer_t);
    }
    slew_steer(steer_t);

    // --- throttle and brake (ai §4.4, ai §5) ----------------------------------------
    float thr = 0, brk = 0;
    if (drive == kDriveForward) {
        const SpeedPlan plan = plan_speed(m.line, m.state, m.mode, m.v_cur, m.v_next, p_.pit_speed,
                                          m.line == AiCourse::kPitLane ? pit_lane_section(course, s) : 0);
        m.pit_limit = plan.pit_limit;
        Pedals pd;
        if (plan.turning) { // turning round (state 2): 50 km/h, gain -0.5 (ai §7)
            pd = speed_controller(v, plan.target, -0.5f);
            m.arc_ticks = 0;
        } else {
            switch (plan.mode) {
            case 3: pd = speed_controller(v, plan.next, -2.0f); m.arc_ticks = 0; break;
            case 4: // braking inside an arc: the ticks in the arc keep counting
                pd = speed_controller(v, plan.next, -1.0f);
                m.arc_ticks = std::min(m.arc_ticks + 1, 6000);
                break;
            case 1:
            case 2: {
                pd = speed_controller(v, arc_target(plan.target, m.arc_ticks, p_, tel.on_pit_surface), -0.5f);
                if (m.status == 0) m.status = pd.throttle == 1.0f ? (plan.mode == 1 ? 4 : 3) : 5;
                m.arc_ticks = std::min(m.arc_ticks + 1, 6000);
                break;
            }
            default: pd.throttle = 1; m.arc_ticks = 0; break;
            }
        }
        thr = pd.throttle * throttle_mult;
        brk = pd.brake;
    } else {
        thr = drive == kDriveReverse ? 1.0f : 0.0f;
        brk = brake_flag ? 1.0f : 0.0f;
        m.arc_ticks = 0;
    }

    // --- traffic decisions (ai §8.2) ---------------------------------------------
    if (m.cooldown > 0) --m.cooldown;
    bool traffic_on = ctx.race_time * 1000.0 > double(start_offset_ + 1) * 100.0 + 5000.0; // 5 s + 0.1 s (index + 1)
    if (drive == kDriveForward && traffic_on && m.line != AiCourse::kPitStop && m.cooldown == 0) {
        bool allowed = !(m.status == 1 || m.status == 2 || m.status == 5);
        if (m.line == AiCourse::kPitLane) {
            if (((zones & kLeft) && (zones & kFrontLeft)) || ((zones & kRight) && (zones & kFrontRight))) thr *= 0.5f;
            allowed = false;
        }
        bool swapped = course.side_swapped(s);
        auto to_b = [&](int cur) { return (cur == 1 || cur == 6) && course.line(2) ? 2 : -1; };
        auto to_a = [&](int cur) { return (cur == 2 || cur == 6) && course.line(1) ? 1 : -1; };
        auto left_line = [&](int cur) { return swapped ? to_a(cur) : to_b(cur); };
        auto right_line = [&](int cur) { return swapped ? to_b(cur) : to_a(cur); };
        int chosen = -1;
        if (allowed && contact > 0) {
            int s1 = (zones & kFrontLeft) ? -1 : left_line(m.line);
            int s2 = (zones & kFrontRight) ? -1 : right_line(m.line);
            auto fallback = [&]() { return s2 >= 0 && m.status != 4 ? s2 : -1; };
            if (m.next_kind == 1) chosen = s1 >= 0 ? s1 : fallback();
            else chosen = s1 >= 0 && m.status != 3 ? s1 : fallback();
            if (chosen >= 0 && switch_line(chosen)) m.cooldown = 30;
            else chosen = -1;
        }
        if (chosen < 0) {
            thr *= 1 - contact;
            // Statuses 1, 2, 5 and the pit lane stop here (ai §14.1 step 6); else the
            // alongside / back-to-main moves (step 9).
            if (allowed) {
                u8 near = zones & (kFront | kRight | kFrontRight | kFrontLeft | kLeft);
                bool lft = zones & kLeft, rgt = zones & kRight;
                if (!near) {
                    if (m.line != AiCourse::kMain) switch_line(AiCourse::kMain), m.blend = 0;
                } else if (lft != rgt) {
                    int l = lft ? (m.status != 4 ? right_line(m.line) : -1) : (m.status != 3 ? left_line(m.line) : -1);
                    if (l >= 0) switch_line(l);
                }
            }
        }
    }

    // Steering saturation (ai §5).
    if (saturation > 0 && st.gear >= 2) {
        thr *= std::max(0.0f, 1 - p_.sat_throttle_cut * (saturation - 1));
        brk = std::min(1.0f, brk + p_.sat_brake * (saturation - 1));
    }
    // No following brake or speed matching (ai §14): only throttle x (1 - c) above and the
    // braking distance + 20 c.
    dbg_.follow = 0;

    // Applied throttle: slew-limited in arcs (ai §1).
    if (m.mode == 1 || m.mode == 2) {
        m.throttle += std::clamp(thr - m.throttle, -dt, dt);
    } else {
        m.throttle = thr;
    }

    // Countdown (ai §10): no brake, no steer, the clutch held open by the
    // caller (Vehicle::set_clutch_hold); the throttle blips around the reference rpm R
    // (the upshift rpm UP) with hysteresis, rank-dependent rates, and holds proportionally
    // in the last 12 frames. At GO the speed controller's pedal applies at once (no ramp).
    float out_thr = m.throttle, out_brk = brk;
    if (ctx.race_time < 0) {
        const float R = spec.engine.upshift_rpm > 0 ? spec.engine.upshift_rpm : spec.engine.rev_limit_rpm - 500;
        const float rpm = st.engine_rpm, r = float(ctx.rank);
        long long n = std::llround(-ctx.race_time * 60.0); // frames of countdown left
        float& t = m.countdown_throttle;
        if (n >= 13) {
            if (rpm < 0.75f * R) m.blip_up = true;
            else if (rpm >= 15.0f / 16.0f * R) m.blip_up = false;
            t = m.blip_up ? std::min(1.0f, t + dt * (5.0f - 0.2f * r)) : std::max(0.0f, t - dt * (10.0f - 0.4f * r));
        } else {
            t = std::clamp(0.625f - 0.002f * r - 2.0f * (rpm - R) / std::max(1.0f, spec.engine.rev_limit_rpm), 0.0f, 1.0f);
        }
        out_brk = 0;
        out_thr = t;
        m.throttle = 0;
        m.steer = 0;
    }
    // Gears: automatic (ai §1). The reverse request (ai §7) is read by the caller through reverse_request().
    m.reverse = drive == kDriveReverse;
    if (drive == kDriveReverse && st.gear >= 1 && std::fabs(v) > 1.0f) out_thr = 0; // stop before reversing
    dbg_.gear_want = m.reverse ? -1 : std::max(st.gear, 1);

    // Debug.
    dbg_.prev_line = m.prev_line;
    dbg_.blend = m.blend;
    dbg_.next_point = m.next;
    dbg_.mode = m.mode;
    dbg_.drive = drive;
    dbg_.hits = tel.hit_sides;
    dbg_.status = m.status;
    dbg_.lookahead = look;
    dbg_.target_speed = m.v_cur;
    dbg_.next_speed = m.v_next;
    dbg_.brake_distance = brake_d;
    dbg_.distance_to_point = dist_np;
    dbg_.heading_error = eps;
    dbg_.desired_yaw = r_des;
    dbg_.slip_wanted = alpha;
    dbg_.saturation = saturation;
    dbg_.contact = contact;
    dbg_.alpha_peak = m.alpha_peak;
    dbg_.zones = zones;
    dbg_.target = target;
    return outputs(out_thr, out_brk);
}

} // namespace gt3r::ai
