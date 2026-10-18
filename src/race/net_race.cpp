#include "race/net_race.h"

#include "race/interp.h"
#include "sim/vehicle.h"

#include <algorithm>
#include <cmath>

namespace gt3r::race {

namespace {

// Rotation by the world-frame rotation vector w (axis * angle).
sim::Quat quat_from_rotvec(const sim::Vec3& w) {
    float a = w.length();
    if (a < 1e-9f) return {};
    float s = std::sin(a * 0.5f) / a;
    return {std::cos(a * 0.5f), w.x * s, w.y * s, w.z * s};
}

sim::Quat conj(const sim::Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }

} // namespace

RemoteCar::Pose RemoteCar::raw(double t) const {
    Pose p;
    p.state = snap_.state;
    p.body_x = snap_.body_x, p.body_z = snap_.body_z;
    p.heading = snap_.heading, p.yaw_rate = snap_.yaw_rate;
    p.brake = snap_.brake;
    float age = float(std::clamp(t - double(snap_.race_time), 0.0, kMaxExtrapolation));
    if (age <= 0) return p;
    sim::VehicleState& s = p.state;
    sim::Vec3 d = s.velocity * age;
    s.position += d;
    s.orientation = normalize(quat_mul(quat_from_rotvec(s.angular_velocity * age), s.orientation));
    for (sim::WheelState& w : s.wheels) {
        w.contact += d;
        w.spin_angle += w.angular_velocity * age;
    }
    p.body_x += s.velocity.x * age;
    p.body_z += s.velocity.z * age;
    p.heading += p.yaw_rate * age;
    return p;
}

void RemoteCar::attach(std::unique_ptr<sim::Vehicle> shadow, const sim::TrackQuery* track, const sim::WorldQuery* world) {
    shadow_ = std::move(shadow);
    track_ = track;
    world_ = world;
    if (!track_) shadow_.reset();
}

void RemoteCar::reset() {
    std::shared_ptr<sim::Vehicle> sh = shadow_;
    const sim::TrackQuery* t = track_;
    const sim::WorldQuery* w = world_;
    *this = RemoteCar{};
    shadow_ = sh, track_ = t, world_ = w;
}

RemoteCar::Pose RemoteCar::shadow_pose() const {
    Pose p;
    p.state = shadow_->state();
    sim::collide::Body b = shadow_->collision_body();
    p.body_x = float(b.pos.x), p.body_z = float(b.pos.z);
    p.heading = float(b.heading), p.yaw_rate = float(b.yaw_rate);
    p.brake = input_.brake;
    return p;
}

void RemoteCar::step_shadow() {
    sim::ControlInputs in = input_;
    in.shift_up = in.shift_down = false; // edges belong to the owner's tick; the gear comes with each state
    shadow_->set_reverse_request(reverse_);
    shadow_->step_tick(in, *track_, 1.0 / sim::kDefaultTickHz, world_);
    sim_time_ += 1.0 / sim::kDefaultTickHz;
    ++sim_tick_;
    history_[sim_tick_ % history_.size()] = shadow_->state().position;
}

void RemoteCar::receive(const net::CarSnapshot& s, double local_race_time) {
    if (have_ && s.tick <= snap_.tick) return; // late or duplicate
    double t = local_race_time == kNoTime ? double(s.race_time) : local_race_time;
    net::CarSnapshot n = s;
    if (have_) { // wheel spin angles are not sent: carry them on with the mean wheel speed
        float dt = float(double(s.tick - snap_.tick) / sim::kDefaultTickHz);
        for (size_t i = 0; i < 4; ++i)
            n.state.wheels[i].spin_angle = snap_.state.wheels[i].spin_angle +
                                           0.5f * (snap_.state.wheels[i].angular_velocity + s.state.wheels[i].angular_velocity) * dt;
    }
    if (!have_) {
        snap_ = n;
        have_ = true;
        off_pos_ = {};
        off_rot_ = {};
        corr_time_ = t;
        if (shadow_) {
            shadow_->sync_to(n.state, n.yaw_rate);
            sim_time_ = n.race_time, sim_tick_ = n.tick;
        }
        return;
    }
    Pose shown = predict(t);
    // How good was the prediction: the shadow's position at this state's tick vs the state.
    if (shadow_ && n.tick <= sim_tick_ && sim_tick_ - n.tick < history_.size()) {
        last_error_ = (history_[n.tick % history_.size()] - n.state.position).length();
        last_horizon_ = double(n.tick - snap_.tick) / sim::kDefaultTickHz;
    }
    snap_ = n;
    Pose fresh;
    if (shadow_) {
        input_ = n.input_count ? net::unpack_input(n.inputs[0]) : sim::ControlInputs{};
        reverse_ = n.input_count && (n.inputs[0].flags & 16);
        shadow_->sync_to(n.state, n.yaw_rate);
        sim_time_ = n.race_time, sim_tick_ = n.tick;
        history_[sim_tick_ % history_.size()] = n.state.position;
        int catch_up = int(std::lround(std::clamp(t - double(n.race_time), 0.0, kMaxExtrapolation) * sim::kDefaultTickHz));
        for (int i = 0; i < catch_up; ++i) step_shadow();
        fresh = shadow_pose();
    } else {
        fresh = raw(t);
    }
    off_pos_ = shown.state.position - fresh.state.position;
    off_rot_ = normalize(quat_mul(shown.state.orientation, conj(fresh.state.orientation)));
    if (off_pos_.length() > kSnapDistance) off_pos_ = {}, off_rot_ = {};
    corr_time_ = t;
}

RemoteCar::Pose RemoteCar::predict(double t) {
    Pose p;
    if (shadow_ && have_) {
        // One tick per call in play (t = the end of the coming tick); held beyond the horizon.
        while (sim_time_ + 0.5 / sim::kDefaultTickHz < t && sim_time_ - double(snap_.race_time) < kMaxExtrapolation)
            step_shadow();
        p = shadow_pose();
    } else {
        p = raw(t);
    }
    if (!have_) return p;
    float k = float(std::exp(-std::max(0.0, t - corr_time_) / kBlend));
    if (k < 1e-3f) return p;
    sim::Vec3 o = off_pos_ * k;
    p.state.position += o;
    for (sim::WheelState& w : p.state.wheels) w.contact += o;
    p.state.orientation = normalize(quat_mul(slerp(sim::Quat{}, off_rot_, k), p.state.orientation));
    return p;
}

std::optional<Plausibility::Flag> Plausibility::fire(net::FlagKind k, float v) {
    unsigned bit = 1u << unsigned(k);
    if (fired_ & bit) return std::nullopt;
    fired_ |= bit;
    return Flag{k, v};
}

std::optional<Plausibility::Flag> Plausibility::on_state(const net::CarSnapshot& s, bool contact) {
    std::optional<Flag> out;
    auto horiz = [](const sim::Vec3& v) { return std::hypot(v.x, v.z); };
    float v = horiz(s.state.velocity);
    if (have_ && s.tick > prev_.tick) {
        double dt = double(s.tick - prev_.tick) / sim::kDefaultTickHz;
        // Faster than its gearing allows, for half a second (a shove can spike it briefly).
        if (v > top_ * kSpeedMargin + kSpeedSlack) over_ += dt;
        else over_ = 0;
        if (over_ > kSpeedTime) out = fire(net::FlagKind::Speed, v * 3.6f);
        // A jump its own motion does not explain (a respawn says so).
        if (!out && !s.respawned && !prev_.respawned && dt <= 0.5) {
            sim::Vec3 expect = (prev_.state.velocity + s.state.velocity) * float(0.5 * dt);
            sim::Vec3 moved = s.state.position - prev_.state.position;
            float jump = horiz(moved - expect);
            if (jump > kTeleport) out = fire(net::FlagKind::Teleport, jump);
        }
        // Speed gained over the last half second, in both its quarters: engines ramp, while a shove
        // from another car is one step (walls and brakes only take speed away).
        speeds_[nspeeds_++ % speeds_.size()] = {s.race_time, v};
        if (!out && !contact && !s.respawned) {
            auto speed_at = [&](double ago, float& vout) {
                for (size_t k = 0; k < std::min(nspeeds_, speeds_.size()); ++k) {
                    double span = s.race_time - speeds_[k].first;
                    if (std::fabs(span - ago) <= 0.03) {
                        vout = speeds_[k].second;
                        return true;
                    }
                }
                return false;
            };
            float v25 = 0, v50 = 0;
            if (speed_at(0.25, v25) && speed_at(0.5, v50)) {
                float g1 = float((v - v25) / 0.25 / 9.81), g2 = float((v25 - v50) / 0.25 / 9.81);
                if (std::min(g1, g2) > kAccelG) out = fire(net::FlagKind::Accel, 0.5f * (g1 + g2));
            }
        }
    } else if (!have_) {
        speeds_[nspeeds_++ % speeds_.size()] = {s.race_time, v};
    }
    prev_ = s;
    have_ = true;
    return out;
}

std::optional<Plausibility::Flag> Plausibility::on_prediction_error(float m, double horizon, bool contact) {
    if (contact || !(horizon > 0)) {
        motion_ = 0;
        return std::nullopt;
    }
    if (m > kMotionError) motion_ += horizon;
    else motion_ = std::max(0.0, motion_ - horizon);
    if (motion_ > kMotionTime) return fire(net::FlagKind::Motion, m);
    return std::nullopt;
}

std::optional<Plausibility::Flag> Plausibility::on_finish(double claimed, double observed) {
    double off = std::fabs(claimed - observed);
    if (off > kFinishSlack) return fire(net::FlagKind::FinishTime, float(off));
    return std::nullopt;
}

net::CarSnapshot make_snapshot(const sim::Vehicle& v, u32 tick, int laps, const net::InputFrame* inputs, int n,
                               bool respawned) {
    net::CarSnapshot s;
    s.respawned = respawned;
    s.tick = tick;
    s.state = v.state();
    sim::collide::Body b = v.collision_body();
    s.body_x = float(b.pos.x), s.body_z = float(b.pos.z);
    s.heading = float(b.heading), s.yaw_rate = float(b.yaw_rate);
    s.laps = u8(std::clamp(laps, 0, 255));
    s.input_count = u8(std::clamp(n, 0, net::kInputHistory));
    for (int i = 0; i < s.input_count; ++i) s.inputs[i] = inputs[i];
    s.brake = s.input_count ? float(s.inputs[0].brake) / 255.0f : 0.0f;
    return s;
}

void complete_snapshot(net::CarSnapshot& s, const sim::VehicleSetup& setup, float mass_kg, double countdown) {
    sim::VehicleState& st = s.state;
    s.race_time = float(double(s.tick) / sim::kDefaultTickHz - countdown);
    sim::Vec3 cog = st.position + rotate(st.orientation, setup.cog_offset);
    s.body_x = cog.x, s.body_z = cog.z;
    int grounded = 0;
    for (const sim::WheelState& w : st.wheels) grounded += w.grounded ? 1 : 0;
    for (size_t i = 0; i < 4; ++i) {
        sim::WheelState& w = st.wheels[i];
        const sim::VehicleSetup::Wheel& ws = setup.wheels[i];
        // Wheel centre in the car frame (VehicleSetup::Wheel), then down by the radius.
        sim::Vec3 local = ws.rest_center;
        local.y += w.suspension - ws.static_suspension - ws.radius;
        w.contact = st.position + rotate(st.orientation, local);
        w.load = w.grounded && grounded ? mass_kg * 9.81f / float(grounded) : 0.0f;
    }
    st.speed_kmh = st.velocity.length() * 3.6f;
}

} // namespace gt3r::race
