// Phase 3, the AI driver's step, decisions and traffic; see tick.h.

#include "base/ee_math.h"
#include "sim/exact/ai_course.h"
#include "sim/exact/tick.h"

namespace gt3r::sim::exact {

namespace {

const F kZero{}, kOne = F::raw(0x3F800000), kHalf = F::raw(0x3F000000);
const F kMinusHalf = F::raw(0xBF000000), kMinusOne = F::raw(0xBF800000), kMinusTwo = F::raw(0xC0000000);
// The look-ahead: the stop line, leaving the pit (kind 4), reversing out, turning round; the start's long look ahead.
const F kLookStop = F::raw(0x40A00000), kLookLeave = F::raw(0x41A00000), kLookReverse = F::raw(0x41200000);
const F kLookTurn = F::raw(0x42200000), kStartLook = F::raw(0x42200000), kStartMs = F::raw(0x44FA0000);
const F kParkSpeed = F::raw(0x3D8E38E4), kParkTurn = F::raw(0x3D567750);       // 0.25 km/h, 3 deg/s
const F kTheta = F::raw(0x3F860A92), kThetaRev = F::raw(0x3E32B8C2);           // 60, 10 deg
const F kThetaSpeed = F::raw(0xC1055556), kThetaSlope = F::raw(0x3F5F66F3);    // -30 km/h, 50 deg
const F kThetaSpan = F::raw(0x41055556), kThetaBase = F::raw(0x3E32B8C2);      // 30 km/h, 10 deg
const F kBack = F::raw(0xBF0E38E4), kRoll = F::raw(0x3F0E38E4);                // -2, 2 km/h
const F kOut = F::raw(0x3F555556);                                             // 3 km/h
const F kRound = F::raw(0x3F32B8C2), kRoundSpeed = F::raw(0x41055556);         // 40 deg, 30 km/h
const F kReverseOut = F::raw(0x3F860A92), kSpin = F::raw(0x3FB2B8C2);          // 60, 80 deg
const F kRearHit = F::raw(0x3F490FDB);                                         // 45 deg
const F kMoving = F::raw(0x3F0E38E4), kMovingBack = F::raw(0xBF0E38E4);        // 2, -2 km/h
const F kWrong = F::raw(0x3FB2B8C2), kWrongNeg = F::raw(0xBFB2B8C2);           // 80, -80 deg
const F kSlow = F::raw(0x3FB1C71D);                                            // 5 km/h
const F kLock = F::raw(0x3F490FDB), kLockNeg = F::raw(0xBF490FDB);             // 45 deg
const F kTurnNeg = F::raw(0xBEB2B8C2), kTurnFull = F::raw(0xBF060A92);         // -20, -30 deg
const F kTurn = F::raw(0x3EB2B8C2), kTurnFullPos = F::raw(0x3F060A92);         // 20, 30 deg
const F kTurnGain = F::raw(0x3FC00001);                                        // 1.5
const F kStopSpeed = F::raw(0x40B1C71D), kTurnSpeed = F::raw(0x415E38E4);      // 20, 50 km/h
const F kPitMargin = F::raw(0x3F0E38E4), kPitMargin2 = F::raw(0x3F0E38E4);     // 2 km/h
const F kBlipUp = F::raw(0x3E4CCCCD), kBlipDown = F::raw(0x3ECCCCCD), kHoldRank = F::raw(0x3B03126F);
const F kBlipUpRate = F::raw(0x40A00000), kBlipDownRate = F::raw(0x41200000), kHoldBase = F::raw(0x3F200000);

} // namespace

void ai_steer(Car& c, F target) { slew(c.ai.steer, target, c.ai.steer_rate * c.dt); }

void ai_park(Car& c) {
    ai_begin(c);
    ai_steer(c, kZero);
    c.ai.brake = kZero;
    c.ai.throttle = kZero;
}

void ai_stop(Car& c, F target) {
    Driver& d = c.ai;
    ai_begin(c);
    ai_steer(c, target);
    d.throttle = kZero;
    const F v = sqrt(c.vel.x * c.vel.x + c.vel.y * c.vel.y + c.vel.z * c.vel.z);
    const F w = sqrt(c.yaw_rate * c.yaw_rate + c.roll_rate * c.roll_rate + c.pitch_rate * c.pitch_rate);
    if (v < kParkSpeed && w < kParkTurn) {
        d.brake = kZero;
        c.driver_kind = 8;
        return;
    }
    if (c.driver_kind != 5) d.brake = kOne;
}

void ai_hold(Car& c, s32 n) {
    Driver& d = c.ai;
    const F r = ee::int_to_f(c.rank);
    c.clutch_req = kZero;
    const s32 up = s32(c.p.d.up_rpm), rpm = c.rpm;
    if (n < (pal_rate(c) ? 11 : 13)) {
        const F q = ee::int_to_f(rpm - up) / ee::int_to_f(s32(c.p.e.rev_limit));
        F x = (kHoldBase - r * kHoldRank) - (q + q);
        if (kOne < x) x = kOne;
        else if (x < kZero) x = kZero;
        d.throttle = x;
        return;
    }
    bool rise;
    if (c.blip == 0) {
        rise = rpm < 3 * up / 4;
        if (rise) c.blip = 1;
    } else {
        rise = rpm < 15 * up / 16;
        if (!rise) c.blip = 0;
    }
    const F dt = c.dt;
    if (rise) {
        F x = d.throttle + dt * (kBlipUpRate - r * kBlipUp);
        d.throttle = kOne < x ? kOne : x;
    } else {
        F x = d.throttle - dt * (kBlipDownRate - r * kBlipDown);
        d.throttle = x < kZero ? kZero : x;
    }
}

void ai_traffic(Car& c, AiPlan& plan) {
    Driver& d = c.ai;
    if (d.cooldown > 0) d.cooldown = s16(d.cooldown - 1);
    const u8 line = d.line;
    if (line == 4) return;
    bool lanes = true;
    const auto z = [&](int k) { return s8(d.zone[size_t(k)]); }; // front, front-left, front-right, left, right, ...
    if (line == 3) {
        lanes = false;
        if ((z(3) != 0 && z(1) != 0) || (z(4) != 0 && z(2) != 0)) d.throttle = d.throttle * kHalf;
    }
    const s8 st = d.status;
    if (st > 0 && (st < 3 || st == 5)) lanes = false;
    const F contact = d.contact;
    if (d.cooldown > 0) return;
    if (lanes && kZero < contact) {
        const s32 left = z(1) != 0 ? -1 : plan.lane_left(d.line, d.next, c.f3e4);
        const s32 right = z(2) != 0 ? -1 : plan.lane_right(d.line, d.next, c.f3e4);
        const s32 right_ok = right >= 0 && d.status != 4 ? right : -1;
        s32 pick;
        if (d.next_kind == 1) pick = left >= 0 ? left : right_ok;
        else if (d.next_kind == 2 && right >= 0) pick = right;
        else pick = left >= 0 && d.status != 3 ? left : right_ok;
        if (pick >= 0) {
            plan.request_line(c, pick, false);
            d.cooldown = pal_rate(c) ? 25 : 30;
            return;
        }
    }
    d.throttle = d.throttle * (kOne - contact);
    if (!lanes) return;
    const s8 l = z(3), r = z(4);
    if (s32(z(0)) + s32(r) + (s32(z(2)) + s32(z(1))) + s32(l) == 0) {
        plan.request_line(c, 6, false);
        return;
    }
    if ((l != 0) == (r != 0)) return;
    if (l != 0) {
        if (d.status == 4) return;
        const s32 to = plan.lane_right(d.line, d.next, c.f3e4);
        if (to != -1) plan.request_line(c, to, false);
    } else {
        if (d.status == 3) return;
        const s32 to = plan.lane_left(d.line, d.next, c.f3e4);
        if (to != -1) plan.request_line(c, to, false);
    }
}

void ai_decide(Car& c, const AiInputs& in, AiPlan& plan) {
    Driver& d = c.ai;
    ai_begin(c);
    if (c.driver_kind == 7) {
        ai_stop(c, kZero);
        return;
    }
    const F scale = plan.braking(c);
    F q = kZero; // the steering saturation ratio
    s32 drive = 0; // 1 forward, -1 reverse, 0 brake
    bool brake = false;
    F nsin = -ee::sin(c.heading), cosh = ee::cos(c.heading);

    // The look-ahead and the aim point (blended with the old line's after a change), the heading error.
    F look;
    if (c.driver_kind == 3) look = kLookStop;
    else if (c.driver_kind == 4 && d.pit_limit != 0) look = kLookLeave;
    else if (d.state != 0) look = d.state == 1 ? kLookReverse : kLookTurn;
    else {
        look = ai_lookahead(d, c.speed);
        const s32 left = s32(2000u - in.clock / 3u);
        if (left > 0) look = look + ee::int_to_f(left) * kStartLook / kStartMs;
    }
    F x, y;
    look = plan.target(c, d.line, d.next, look, x, y);
    if (kZero < d.blend) {
        F xo, yo;
        plan.target(c, d.prev_line, d.next, look, xo, yo);
        const F w = d.blend;
        x = x + (xo - x) * w;
        y = y + (yo - y) * w;
    }
    const F eps = angle_diff(ee::atan2(-x, y), c.heading, c.behaviour.rule(Rule::AngleWrap));
    F aeps = eps;
    if (eps < kZero) aeps = -eps;

    // The drive decision with the heading threshold, then the recovery state.
    const u8 st = d.state;
    F v = c.speed;
    F theta = kTheta;
    if (st == 1) {
        if (kZero <= v) theta = kThetaRev;
        else if (kThetaSpeed < v) theta = kThetaBase - v * kThetaSlope / kThetaSpan;
    }
    if (aeps < theta || st == 2) {
        if (kBack < v) drive = 1;
        else brake = true;
    } else if (kRoll < v) brake = true;
    else drive = -1;
    if (st == 1) {
        if (aeps < theta) d.state = kOut < v ? 0 : 2;
        else if (in.hit_rear != 0) d.state = 2;
    } else if (st == 2) {
        if (aeps < kRound && kRoundSpeed < v) d.state = 0;
        else if (in.hit_front != 0 && (in.hit_x != 0 || kReverseOut < aeps)) d.state = 1;
    } else {
        if (kSpin < aeps) d.state = 2;
        else if (in.hit_x != 0) d.state = 1;
        else if (in.hit_rear != 0 && kRearHit < aeps) d.state = 2;
    }
    v = c.speed;
    s32 moving = 0;
    if (kMoving < v) moving = 1;
    else if (v < kMovingBack) moving = -1;

    // The steering target.
    F target = kZero;
    if (moving != 0) {
        if (drive == 1) {
            const F amax = (tire_record(c.p, 0, c.w[0].surface).peak_slip + tire_record(c.p, 1, c.w[1].surface).peak_slip) * kHalf;
            if (d.state != 0) {
                if (kWrong < eps) target = amax;
                else if (eps < kWrongNeg) target = -amax;
            }
            if (target == kZero) {
                // Pure pursuit: the yaw rate of the arc to the aim point, in the velocity's direction (the heading's when slow).
                const F p2 = x * x + y * y;
                const F vx = c.vel.x, vy = c.vel.y;
                const F vxy = sqrt(vx * vx + vy * vy);
                F cs = cosh, ns = nsin;
                if (!(vxy < kSlow)) {
                    const F a = ee::atan2(-vx, vy);
                    ns = -ee::sin(a);
                    cs = ee::cos(a);
                }
                F rd = vxy * kMinusTwo * (cs * x - ns * y) / p2;
                if (c.speed < kZero) rd = -rd;
                const F r = c.yaw_rate;
                if (kZero < rd) {
                    if (r < rd) {
                        target = target + (rd - r) * d.yaw_gain;
                        if (amax < target) {
                            q = target / amax;
                            if (!dirt_course(in.course_category)) target = amax;
                        }
                    }
                } else if (rd < kZero && rd < r) {
                    const F neg = -amax;
                    target = target - (r - rd) * d.yaw_gain;
                    if (target < neg) {
                        q = -target / amax;
                        if (!dirt_course(in.course_category)) target = neg;
                    }
                }
            }
            target = target + (c.w[0].slip_dir + c.w[1].slip_dir) * kHalf;
            if (kLock < target) target = kLock;
            else if (target < kLockNeg) target = kLockNeg;
        } else if (eps < kTurnNeg) target = kTurnFull;
        else if (kTurn < eps) target = kTurnFullPos;
        else target = eps * kTurnGain;
        if (moving == -1) target = -target;
    }
    const u8 kind = c.driver_kind;
    if (u8(kind - 5) < 2) {
        ai_stop(c, target);
        return;
    }
    if (u8(u8(c.control) - 1) < 2 || kind != 0) ai_steer(c, target);

    // Throttle and brake.
    if (drive == 1) {
        const u8 line = d.line;
        d.pit_limit = 0;
        F aim = d.v_cur, next = d.v_next;
        u8 mode = d.mode;
        if (line == 4) { // the pit stop line: 20 km/h, braking from a straight
            aim = next = kStopSpeed;
            mode = 3;
            d.pit_limit = 1;
        } else if (line == 3) { // the pit lane, by the stop line's points
            const F lap = plan.lap_length();
            F s = c.f3e4;
            if (s < lap * kHalf) s = s + lap;
            if (s < plan.stop_distance(1)) {
            } else if (s < plan.stop_distance(2)) next = d.pit_speed + kPitMargin;
            else if (s < plan.stop_distance(plan.stop_count() - 3)) {
                aim = next = d.pit_speed + kPitMargin2;
                mode = 1;
                d.pit_limit = 1;
            }
        } else if (d.state == 2) aim = kTurnSpeed;
        F thr;
        bool skip_clamp = false;
        if (d.state == 2) {
            plan.speed_control(c, aim, kMinusHalf);
            d.arc_ticks = 0;
            thr = d.throttle;
            skip_clamp = true;
        } else if (mode >= 5) thr = d.throttle;
        else if (mode == 0) {
            d.throttle = kOne;
            d.arc_ticks = 0;
            thr = kOne;
        } else if (mode == 3) {
            plan.speed_control(c, next, kMinusTwo);
            d.arc_ticks = 0;
            thr = d.throttle;
        } else if (mode == 4) {
            plan.speed_control(c, next, kMinusOne);
            thr = d.throttle;
            d.arc_ticks = u16(d.arc_ticks + 1);
        } else { // the arc modes 1 / 2: the corner bonus after its delay (not on the pit surface)
            if (!(c.x535 & 0x10) && !(s32(d.arc_ticks) < d.bonus_ticks)) aim = aim + d.corner_bonus;
            plan.speed_control(c, aim, kMinusHalf);
            thr = d.throttle;
            if (d.status == 0) d.status = !(thr == kOne) ? 5 : mode == 1 ? 4 : 3;
            d.arc_ticks = u16(d.arc_ticks + 1);
        }
        if (!skip_clamp && d.arc_ticks >= (pal_rate(c) ? 5001 : 6001)) d.arc_ticks = pal_rate(c) ? 5000 : 6000; // 100 s
        d.throttle = thr * scale;
        if (s32(5000 + 100 * s32(c.rank)) < s32(in.clock / 3u)) ai_traffic(c, plan);
    } else {
        if (drive != -1) d.throttle = kZero;
        else {
            d.reverse = 1, d.xa9 = 0;
            d.throttle = kOne;
        }
        d.brake = brake ? kOne : kZero;
        d.arc_ticks = 0;
    }

    // The steering saturation: the throttle cut and the brake for a wanted slip angle beyond the tyres' peak.
    if (kZero < q && !(c.gear < 2)) {
        const F e = q - kOne;
        F k = kOne - d.sat_throttle * e;
        if (k < kZero) k = kZero;
        d.throttle = d.throttle * k;
        const F b = d.brake + e * d.sat_brake;
        d.brake = kOne < b ? kOne : b;
    }
}

void ai_step(Car& c, const AiInputs& in, AiPlan& plan) {
    c.handbrake = kZero;
    c.reverse_req = 0;
    c.shift_req = 0;
    if (c.race_hold != 0 || c.hold != 0) {
        ai_hold(c, c.race_hold != 0 ? in.start_count : s32(c.hold));
        set_brake_pedal(c, kZero);
        c.steer_cmd = kZero;
        return;
    }
    if (c.driver_kind == 8) {
        c.clutch_req = kZero;
        ai_park(c);
        return;
    }
    if (c.clutch == 0) c.clutch_req = 3 * s32(c.p.d.up_rpm) / 4 < s32(c.rpm) ? kOne : kZero;
    else c.clutch_req = s32(c.rpm) < s32(c.p.e.idle) ? kZero : kOne;
    ai_decide(c, in, plan);
}

void ai_gear_hint_clear(Driver& d) {
    d.gear_braking = 0;
    d.gear_hint = -1;
}

void ai_gear_hint(Car& c) {
    Driver& d = c.ai;
    ai_gear_hint_clear(d);
    F v = d.v_next;
    if (d.mode == 4 && !(d.v_next < d.v_cur)) v = d.v_cur;
    if (c.speed < v) return;
    if (d.mode == 3 || d.mode == 4) d.gear_braking = 1;
    d.gear_hint = s8(gear_for_speed(c, v));
}

void ai_update(Car& c, AiPlan& plan) {
    plan.braking(c);
    ai_gear_hint(c);
}

void driver_input(Car& c, const PadConstants& k, const PadFrame& f, u8 course_kind, const AiInputs& in, AiPlan& plan) {
    c.full_throttle = 1;
    const s8 src = c.control;
    if (src == 0) {
        pad_hud(c, f);
        if (c.driver_kind == 0 || c.hold != 0) {
            pad_input(c, k, f, course_kind);
            ai_update(c, plan);
        } else {
            ai_step(c, in, plan);
            ai_gear_hint(c);
            ai_controls(c);
        }
    } else if (src == 1) {
        pad_hud(c, f);
        ai_step(c, in, plan);
        ai_controls(c);
    } else if (src == 2) {
        ai_step(c, in, plan);
        ai_controls(c);
    }
}

F CoursePlan::braking(Car& c) { return ai_braking(c, k_, cp_); }
F CoursePlan::target(const Car& c, int line, int hint, F look, F& dx, F& dy) { return ai_target(c, k_, line, hint, look, dx, dy); }
void CoursePlan::speed_control(Car& c, F target, F gain) { ai_speed_control(c, target, gain); }
bool CoursePlan::request_line(Car& c, int line, bool at_once) { return ai_request_line(c, k_, cp_, line, at_once); }
void CoursePlan::replan(Car& c) { ai_replan(c, k_, cp_); }
s32 CoursePlan::lane_left(int line, int hint, F s) { return k_.lane_left(line, hint, s); }
s32 CoursePlan::lane_right(int line, int hint, F s) { return k_.lane_right(line, hint, s); }
F CoursePlan::lap_length() { return k_.lap_length; }
s32 CoursePlan::stop_count() { return s32(k_.lines[4].size()); }
F CoursePlan::stop_distance(s32 k) { // the program reads past the line for an index outside it (a line of fewer than 3 points)
    return k >= 0 && size_t(k) < k_.lines[4].size() ? k_.lines[4][size_t(k)].dist : F();
}

} // namespace gt3r::sim::exact
