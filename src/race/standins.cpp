#include "race/standins.h"

#include "base/log.h"
#include "race/interp.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gt3r::race {

namespace {

constexpr float kG = 9.81f;
constexpr float kPi = 3.14159265f;

// --- RoadTrack ---------------------------------------------------------------

class RoadQuery : public sim::TrackQuery {
public:
    explicit RoadQuery(const course::Road& r) : road_(r) {}
    sim::SurfaceHit probe(float x, float z, float y_max) const override {
        sim::SurfaceHit h;
        u32 surface = 0;
        std::optional<float> y = road_.height_at(x, z, y_max, &surface);
        if (!y) return h;
        h.hit = true;
        h.height = *y;
        h.surface = surface;
        // Normal from neighbouring samples (the road API has no per-triangle normal).
        constexpr float d = 0.5f;
        std::optional<float> yx = road_.height_at(x + d, z, *y + 1.0f), yz = road_.height_at(x, z + d, *y + 1.0f);
        float sx = yx ? (*yx - *y) / d : 0, sz = yz ? (*yz - *y) / d : 0;
        sim::Vec3 n{-sx, 1, -sz};
        h.normal = n * (1.0f / n.length());
        return h;
    }

private:
    const course::Road& road_;
};

class RoadWorld : public sim::WorldQuery {
public:
    explicit RoadWorld(const course::Road& r) : road_(r) {}
    bool wall_contact(const sim::Vec3& c, float radius, sim::Vec3& normal, float& penetration) const override {
        float best = 0;
        bool hit = false;
        for (const std::vector<course::Vec3>& wall : road_.walls)
            for (size_t i = 0; i + 1 < wall.size(); ++i) {
                const course::Vec3 &a = wall[i], &b = wall[i + 1];
                if (c.y < std::min(a.y, b.y) - 4 || c.y > std::max(a.y, b.y) + 6) continue;
                float ex = b.x - a.x, ez = b.z - a.z, l2 = ex * ex + ez * ez;
                if (l2 < 1e-8f) continue;
                float t = std::clamp(((c.x - a.x) * ex + (c.z - a.z) * ez) / l2, 0.0f, 1.0f);
                float px = a.x + ex * t, pz = a.z + ez * t;
                float dx = c.x - px, dz = c.z - pz, d = std::sqrt(dx * dx + dz * dz);
                float pen = radius - d;
                if (pen > best && d > 1e-5f) {
                    best = pen;
                    normal = {dx / d, 0, dz / d};
                    hit = true;
                }
            }
        penetration = best;
        return hit;
    }
    Grip surface_grip(u32 surface) const override {
        if (surface == 0) return {1.0f, 0.0f};
        return {0.85f, 0.02f};
    }

private:
    const course::Road& road_;
};

class RoadTrack : public RaceTrack {
public:
    RoadTrack(course::Road road, std::string label)
        : road_(std::move(road)), label_(std::move(label)), query_(road_), world_(road_) {
        cps_ = road_.checkpoints;
        std::sort(cps_.begin(), cps_.end(), [](const auto& a, const auto& b) { return a.distance < b.distance; });
        length_ = road_.length > 0 ? road_.length : (cps_.empty() ? 1000.0f : cps_.back().distance + 15.0f);
    }

    const char* backend_name() const override { return "stand-in (GTRW road)"; }
    const sim::TrackQuery& query() const override { return query_; }
    const sim::WorldQuery& world() const override { return world_; }
    float length() const override { return length_; }

    int grid_count() const override { return std::max<int>(1, int(road_.grid.size())); }

    TrackPose grid_pose(int slot) const override {
        sim::Vec3 p;
        if (road_.grid.empty()) {
            if (cps_.empty()) return {};
            p = mid(0);
        } else {
            const course::GridSlot& g = road_.grid[size_t(std::clamp(slot, 0, int(road_.grid.size()) - 1))];
            p = {g.pos.x, g.pos.y, g.pos.z};
        }
        return pose_at(p, nearest_cp(p, -1));
    }

    TrackPose respawn_pose(const sim::Vec3& p) const override {
        int i = nearest_cp(p, -1);
        if (i < 0) return {p, {}};
        return pose_at(mid(i), i);
    }

    sim::Vec3 point_ahead(const sim::Vec3& p, float metres) const override {
        int i = nearest_cp(p, idx_);
        if (i < 0) return p;
        int n = int(cps_.size());
        float target = cps_[size_t(i)].distance + metres;
        int k = i;
        for (int step = 0; step < n; ++step) {
            int j = (k + 1) % n;
            float dj = cps_[size_t(j)].distance + (j < i ? length_ : 0.0f);
            if (dj > target) break;
            k = j;
        }
        return mid(k);
    }
    sim::Vec3 point_at(float s, float lateral) const override {
        if (cps_.empty()) return {};
        s = std::fmod(s, length_);
        if (s < 0) s += length_;
        size_t i = 0;
        while (i + 1 < cps_.size() && cps_[i + 1].distance <= s) ++i;
        sim::Vec3 a = mid(int(i)), b = mid(int((i + 1) % cps_.size()));
        float span = (i + 1 < cps_.size() ? cps_[i + 1].distance : length_) - cps_[i].distance;
        float t = span > 0 ? std::clamp((s - cps_[i].distance) / span, 0.0f, 1.0f) : 0.0f;
        sim::Vec3 p = a + (b - a) * t;
        float dx = b.x - a.x, dz = b.z - a.z, l = std::max(1e-3f, std::hypot(dx, dz));
        // Left of the travel direction (dx, dz) is (dz, -dx) in this right-handed, Y-up frame.
        return p + sim::Vec3{dz / l, 0, -dx / l} * lateral;
    }
    float distance_of(const sim::Vec3& p) const override {
        int i = nearest_cp(p, -1);
        return i < 0 ? 0.0f : cps_[size_t(i)].distance;
    }
    bool is_kerb(u32 surface) const override { return surface == 1 || surface == 2; }
    bool is_offroad(const sim::SurfaceHit& hit) const override { return !hit.hit || hit.surface >= 8; }

    void lap_reset(double) override {
        idx_ = -1;
        started_ = false;
        laps_ = 0;
        last_ = best_ = -1;
        passed_ = 0;
        back_ = 0;
    }
    void lap_teleport(const sim::Vec3& p) override { idx_ = nearest_cp(p, -1); }

    bool lap_update(const sim::Vec3& p, double t) override {
        if (cps_.empty()) return false;
        int n = int(cps_.size());
        int i = nearest_cp(p, idx_);
        bool lap = false;
        if (idx_ >= 0 && i != idx_) {
            int step = i - idx_;
            if (step > n / 2) step -= n;
            if (step < -n / 2) step += n;
            if (step > 0) {
                passed_ += step;
                back_ = 0;
            } else {
                back_ += float(-step);
            }
            // Crossing the start line forwards: index wraps from the end to the start.
            if (step > 0 && i < idx_) {
                if (started_ && passed_ >= n / 2) {
                    double lt = t - lap_start_;
                    last_ = lt;
                    if (best_ < 0 || lt < best_) best_ = lt;
                    ++laps_;
                    lap = true;
                }
                if (!started_ || lap) {
                    started_ = true;
                    lap_start_ = t;
                    passed_ = 0;
                }
            }
        }
        idx_ = i;
        return lap;
    }
    int laps_completed() const override { return laps_; }
    bool lap_started() const override { return started_; }
    double current_lap_time(double now) const override { return started_ ? now - lap_start_ : 0; }
    double last_lap_time() const override { return last_; }
    double best_lap_time() const override { return best_; }
    float lap_progress() const override {
        if (idx_ < 0 || cps_.empty()) return 0;
        return std::clamp(cps_[size_t(idx_)].distance / length_, 0.0f, 1.0f);
    }
    bool wrong_way() const override { return back_ >= 3; }
    double race_distance() const override {
        return double(laps_) * length_ + (idx_ >= 0 && !cps_.empty() ? cps_[size_t(idx_)].distance : 0.0f);
    }

private:
    sim::Vec3 mid(int i) const {
        const course::Checkpoint& c = cps_[size_t(i)];
        float x = (c.x0 + c.x1) * 0.5f, z = (c.z0 + c.z1) * 0.5f;
        float y = road_.height_at(x, z).value_or(0.0f);
        return {x, y, z};
    }
    // Nearest checkpoint midpoint; with a hint, only within +-12 checkpoints of it.
    int nearest_cp(const sim::Vec3& p, int hint) const {
        int n = int(cps_.size()), best = -1;
        float bd = std::numeric_limits<float>::max();
        auto test = [&](int i) {
            const course::Checkpoint& c = cps_[size_t(i)];
            float x = (c.x0 + c.x1) * 0.5f - p.x, z = (c.z0 + c.z1) * 0.5f - p.z;
            float d = x * x + z * z;
            if (d < bd) bd = d, best = i;
        };
        if (hint >= 0 && n > 30) {
            for (int k = -12; k <= 12; ++k) test(((hint + k) % n + n) % n);
            if (bd < 60.0f * 60.0f) return best;
        }
        for (int i = 0; i < n; ++i) test(i);
        return best;
    }
    TrackPose pose_at(sim::Vec3 p, int cp) const {
        TrackPose pose;
        if (std::optional<float> y = road_.height_at(p.x, p.z, p.y + 100.0f)) p.y = *y;
        pose.position = p;
        if (cp >= 0 && cps_.size() > 1) {
            sim::Vec3 a = mid(cp), b = mid((cp + 1) % int(cps_.size()));
            float dx = b.x - a.x, dz = b.z - a.z;
            pose.orientation = quat_from_yaw(std::atan2(-dx, -dz)); // nose (-Z) along the travel direction
        }
        return pose;
    }

    course::Road road_;
    std::string label_;
    RoadQuery query_;
    RoadWorld world_;
    std::vector<course::Checkpoint> cps_;
    float length_ = 0;
    int idx_ = -1, laps_ = 0, passed_ = 0;
    float back_ = 0;
    bool started_ = false;
    double lap_start_ = 0, last_ = -1, best_ = -1;
};

// --- KinematicVehicle ----------------------------------------------------------

class KinematicVehicle : public RaceVehicle {
public:
    KinematicVehicle(const CarSetup& car, const RaceTrack& track) : car_(car), track_(track) {
        if (car_.ratios.empty()) car_.ratios = {3.2f, 2.0f, 1.4f, 1.1f, 0.9f};
        const WheelSpec& w = car_.wheels;
        for (int i = 0; i < 4; ++i) {
            int axle = i >> 1;
            sim::VehicleSetup::Wheel& wh = setup_.wheels[size_t(i)];
            wh.radius = w.radius[axle];
            wh.width = w.width[axle];
            wh.rest_center = {(i & 1 ? 0.5f : -0.5f) * w.track[axle], w.radius[axle], (axle ? 0.5f : -0.5f) * w.wheelbase};
        }
        setup_.wheelbase = w.wheelbase;
        setup_.width = std::max(w.track[0], w.track[1]) + 0.2f;
        setup_.length = w.wheelbase + 1.8f;
        setup_.forward_gears = int(car_.ratios.size());
        setup_.idle_rpm = car_.idle_rpm;
        setup_.rev_limit_rpm = car_.limit_rpm;
        setup_.max_rpm = std::ceil((car_.limit_rpm + 500) / 1000.0f) * 1000.0f;
        setup_.upshift_rpm = car_.redline_rpm;
    }

    const char* backend_name() const override { return "stand-in (kinematic)"; }
    const sim::VehicleState& state() const override { return st_; }
    const sim::VehicleSetup& setup() const override { return setup_; }
    float last_impact() const override { return impact_; }

    void reset(const TrackPose& pose) override {
        st_ = {};
        st_.position = pose.position;
        st_.orientation = pose.orientation;
        // Heading from the orientation (rotation about Y only is assumed).
        sim::Vec3 f = rotate(pose.orientation, {0, 0, -1});
        yaw_ = std::atan2(-f.x, -f.z);
        v_ = 0;
        steer_ = 0;
        gear_ = 1;
        rpm_ = car_.idle_rpm;
        reverse_hold_ = 0;
        for (float& s : spin_) s = 0;
        place_on_ground();
        publish(0, 0);
    }

    void step(const sim::ControlInputs& in, double dtd) override {
        float dt = float(dtd);
        impact_ = 0;
        int n = int(car_.ratios.size());
        float r = car_.wheels.radius[1];
        float aspeed = std::fabs(v_);

        // Gear selection.
        if (in.automatic) {
            if (gear_ >= 1) {
                if (aspeed < 0.5f && in.brake > 0.3f && in.throttle < 0.05f) reverse_hold_ += dt;
                else reverse_hold_ = 0;
                if (reverse_hold_ > 0.6f) gear_ = -1, reverse_hold_ = 0;
                else if (rpm_ > car_.redline_rpm * 0.97f && gear_ < n) gear_++;
                else if (gear_ > 1 && rpm_ < car_.redline_rpm * 0.5f) gear_--;
            } else if (in.throttle > 0.3f && aspeed < 0.5f) {
                gear_ = 1;
            }
        } else {
            if (in.shift_up) gear_ = gear_ < 0 ? 0 : std::min(n, gear_ + 1);
            if (in.shift_down) gear_ = gear_ > 0 ? gear_ - 1 : (aspeed < 1.0f ? -1 : gear_);
        }
        // In automatic reverse the pedals swap roles (brake drives backwards).
        float throttle = in.throttle, brake = in.brake;
        if (in.automatic && gear_ < 0) std::swap(throttle, brake);

        float ratio = gear_ > 0 ? car_.ratios[size_t(gear_ - 1)] : gear_ < 0 ? car_.reverse_ratio : 0;
        float wheel_rpm = aspeed / r * 60.0f / (2 * kPi);
        float target = gear_ == 0 ? car_.idle_rpm + throttle * (car_.limit_rpm - car_.idle_rpm)
                                  : std::max(car_.idle_rpm + throttle * 2200.0f * std::max(0.0f, 1 - aspeed / 6),
                                             wheel_rpm * ratio * car_.final_drive);
        rpm_ = gear_ == 0 ? rpm_ + (target - rpm_) * std::min(1.0f, dt * 6) : target;
        rpm_ = std::clamp(rpm_, car_.idle_rpm, car_.limit_rpm + 100);
        bool limiter = rpm_ >= car_.limit_rpm;

        sim::SurfaceHit ground = track_.query().probe(st_.position.x, st_.position.z, st_.position.y + 2.0f);
        sim::WorldQuery::Grip grip = ground.hit ? track_.world().surface_grip(ground.surface) : sim::WorldQuery::Grip{0.6f, 0.08f};
        float mu = 1.05f * grip.mu;
        float m = car_.mass_kg;

        float drive = 0;
        float slip = 0;
        if (gear_ != 0 && !limiter) {
            drive = torque_at(rpm_) * throttle * ratio * car_.final_drive * 0.85f / r;
            float traction = mu * m * kG * (car_.drive_type == 2 ? 1.0f : 0.55f);
            if (drive > traction) slip = std::min(1.0f, 0.15f + (drive - traction) / traction), drive = traction;
            if (gear_ < 0) drive = -drive;
        }
        float dir = v_ > 0 ? 1.0f : v_ < 0 ? -1.0f : 0.0f;
        float brake_f = (brake * 1.1f + (in.handbrake ? 0.5f : 0.0f)) * mu * m * kG;
        float drag = 0.5f * 1.2f * 0.75f * v_ * aspeed + (0.015f + grip.drag) * m * kG * dir;
        sim::Vec3 fwd = rotate(st_.orientation, {0, 0, -1});
        float slope = -m * kG * fwd.y;
        float a = (drive - drag + slope) / m;
        float v_new = v_ + a * dt;
        // Brakes stop the car, they never push it backwards.
        float bdv = brake_f / m * dt;
        if (std::fabs(v_new) <= bdv) v_new = 0;
        else v_new -= bdv * (v_new > 0 ? 1.0f : -1.0f);
        if (aspeed < 0.05f && std::fabs(drive) < 1e-3f) v_new = 0;
        v_ = v_new;

        // Steering: lock shrinks with speed; yaw rate limited by lateral grip.
        float lock = 0.6f / (1.0f + aspeed / 22.0f);
        steer_ += (in.steer * lock - steer_) * std::min(1.0f, dt * 12.0f);
        float yaw_rate = v_ * std::tan(steer_) / car_.wheels.wheelbase;
        float max_rate = mu * kG / std::max(aspeed, 1.0f);
        float slide = 0;
        if (std::fabs(yaw_rate) > max_rate) {
            slide = std::fabs(yaw_rate) / max_rate - 1;
            yaw_rate = std::copysign(max_rate, yaw_rate);
            v_ *= 1.0f - std::min(0.5f, slide) * dt; // scrub
        }
        yaw_ -= yaw_rate * dt; // positive steer (right) turns clockwise seen from above
        sim::Vec3 heading{-std::sin(yaw_), 0, -std::cos(yaw_)};
        st_.position += heading * (v_ * dt);

        // Walls: three spheres along the body.
        float rad = setup_.width * 0.5f;
        for (float off : {-0.5f * setup_.length + rad, 0.0f, 0.5f * setup_.length - rad}) {
            sim::Vec3 c = st_.position + heading * (-off);
            c.y += 0.5f;
            sim::Vec3 nrm;
            float pen = 0;
            if (!track_.world().wall_contact(c, rad, nrm, pen)) continue;
            st_.position += nrm * pen;
            sim::Vec3 vel = heading * v_;
            float vn = vel.dot(nrm);
            if (vn < 0) {
                impact_ = std::max(impact_, -vn);
                vel = vel - nrm * (vn * 1.2f);
                v_ = vel.dot(heading) * 0.85f;
            }
        }
        place_on_ground();
        for (int i = 0; i < 4; ++i) spin_[i] += v_ / setup_.wheels[size_t(i)].radius * dt * (i >= 2 && slip > 0 ? 1.3f : 1.0f);
        st_.rev_limiter = limiter;
        st_.throttle = limiter ? 0 : throttle;
        publish(slip, slide);
        st_.angular_velocity = {0, -yaw_rate, 0};
    }

private:
    float torque_at(float rpm) const {
        const auto& x = car_.curve_rpm;
        const auto& y = car_.curve_torque_nm;
        if (x.empty() || x.size() != y.size()) return 250;
        if (rpm <= x.front()) return y.front() * std::max(0.3f, rpm / x.front());
        for (size_t i = 1; i < x.size(); ++i)
            if (rpm <= x[i]) return y[i - 1] + (y[i] - y[i - 1]) * (rpm - x[i - 1]) / std::max(1.0f, x[i] - x[i - 1]);
        return y.back();
    }

    void place_on_ground() {
        sim::Quat qy = quat_from_yaw(yaw_);
        float h[4];
        int hits = 0;
        float sum = 0;
        for (int i = 0; i < 4; ++i) {
            sim::Vec3 rc = setup_.wheels[size_t(i)].rest_center;
            sim::Vec3 p = st_.position + rotate(qy, {rc.x, 0, rc.z});
            ground_[i] = track_.query().probe(p.x, p.z, st_.position.y + 2.5f);
            h[i] = ground_[i].hit ? ground_[i].height : st_.position.y;
            if (ground_[i].hit) ++hits, sum += h[i];
        }
        if (hits) st_.position.y = sum / float(hits);
        else st_.position.y -= 0.0f; // off the road mesh: keep the height
        if (hits < 4)
            for (float& v : h) v = hits ? std::clamp(v, st_.position.y - 0.5f, st_.position.y + 0.5f) : st_.position.y;
        float front = (h[0] + h[1]) * 0.5f, rear = (h[2] + h[3]) * 0.5f;
        float left = (h[0] + h[2]) * 0.5f, right = (h[1] + h[3]) * 0.5f;
        float pitch = std::atan2(front - rear, car_.wheels.wheelbase);
        float roll = std::atan2(right - left, std::max(car_.wheels.track[0], 1.0f));
        sim::Quat qp{std::cos(pitch * 0.5f), std::sin(pitch * 0.5f), 0, 0};
        sim::Quat qr{std::cos(roll * 0.5f), 0, 0, std::sin(roll * 0.5f)};
        st_.orientation = normalize(quat_mul(qy, quat_mul(qp, qr)));
    }

    void publish(float slip, float slide) {
        sim::Vec3 heading = rotate(st_.orientation, {0, 0, -1});
        st_.velocity = heading * v_;
        st_.speed_kmh = std::fabs(v_) * 3.6f;
        st_.engine_rpm = rpm_;
        st_.gear = gear_;
        for (int i = 0; i < 4; ++i) {
            sim::WheelState& w = st_.wheels[size_t(i)];
            sim::Vec3 rc = setup_.wheels[size_t(i)].rest_center;
            w.contact = st_.position + rotate(st_.orientation, {rc.x, 0, rc.z});
            w.steer_angle = i < 2 ? steer_ : 0;
            w.spin_angle = spin_[i];
            w.angular_velocity = v_ / setup_.wheels[size_t(i)].radius;
            w.grounded = ground_[i].hit;
            w.surface = ground_[i].surface;
            bool driven = car_.drive_type == 1 ? i < 2 : car_.drive_type == 2 ? true : i >= 2;
            w.slip_ratio = driven ? -slip : 0; // sim convention: < 0 = wheelspin
            w.slip_angle = slide * 0.2f;
            w.load = car_.mass_kg * kG * 0.25f;
        }
    }

    CarSetup car_;
    const RaceTrack& track_;
    sim::VehicleSetup setup_;
    sim::VehicleState st_;
    sim::SurfaceHit ground_[4];
    float yaw_ = 0, v_ = 0, steer_ = 0, rpm_ = 900, reverse_hold_ = 0, impact_ = 0;
    float spin_[4] = {};
    int gear_ = 1;
};

} // namespace

std::unique_ptr<RaceTrack> make_road_track(course::Road road, const std::string& label) {
    return std::make_unique<RoadTrack>(std::move(road), label);
}

std::unique_ptr<RaceVehicle> make_kinematic_vehicle(const CarSetup& car, const RaceTrack& track) {
    return std::make_unique<KinematicVehicle>(car, track);
}

} // namespace gt3r::race
