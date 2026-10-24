#include "race/pit.h"

#include "base/log.h"
#include "race/interp.h"
#include "track/track.h"

#include <algorithm>
#include <cmath>

namespace gt3r::race {


bool PitLane::load(const track::Track& t, int car_index) {
    t_ = &t;
    ok_ = false;
    reset();
    const course::AiLines& ai = t.ai_lines();
    if (ai.lines.size() < 4) return false;
    const course::AiLine& lane = ai.lines[2], &stop = ai.lines[3]; // game lines 3 and 4
    int n = int(stop.points.size());
    if (lane.points.size() < 4 || n < 6 + car_index) return false;
    lane_ = t.racing_line(2, 2.0f); // game line 3: a lap through the pit lane
    if (lane_.size() < 4) return false;
    length_ = t.length();
    auto at = [&](int i) {
        const course::Vec3& p = stop.points[size_t(i)].pos;
        return sim::Vec3{p.x, p.y, p.z};
    };
    entry0_ = distance(at(0));
    entry_ = distance(at(1));
    sim::Vec3 b = at(n - 4 - car_index);
    box_ = distance(b);
    exit_ = distance(at(n - 2));
    // Box heading: the stop line's direction at the box point.
    sim::Vec3 nb = at(std::min(n - 1, n - 3 - car_index));
    float dx = nb.x - b.x, dz = nb.z - b.z;
    float yaw = std::atan2(-dx, -dz);
    box_q_ = {std::cos(yaw * 0.5f), 0, std::sin(yaw * 0.5f), 0};
    ok_ = true;
    if (car_index == 0) LOG_INFO("Pit lane: entry %.0f m, box %.0f m, exit %.0f m (lap %.0f m)", entry_, box_, exit_, length_);
    return true;
}

float PitLane::distance(const sim::Vec3& p) const { return t_->locate(p).distance; }

float PitLane::to_entry(const sim::Vec3& p) const {
    if (!ok_) return 1e9f;
    float d = entry_ - distance(p);
    return d < 0 ? d + length_ : d;
}

// prev -> cur passes `mark` forwards (wrapping over the line).
bool PitLane::crossed(float prev, float cur, float mark) const {
    if (prev < 0) return false;
    float d = cur - prev;
    if (d < -length_ * 0.5f) d += length_;
    if (d <= 0 || d > 50) return false;
    float m = mark - prev;
    if (m < 0) m += length_;
    return m > 0 && m <= d;
}

sim::ControlInputs PitLane::follow(const std::vector<sim::Vec3>& line, const sim::VehicleState& s, float target_kmh) const {
    sim::ControlInputs in;
    size_t best = 0;
    float bd = 1e30f;
    for (size_t i = 0; i < line.size(); ++i) {
        float dx = line[i].x - s.position.x, dz = line[i].z - s.position.z, d = dx * dx + dz * dz;
        if (d < bd) bd = d, best = i;
    }
    float look = 6.0f + s.speed_kmh / 3.6f * 0.3f;
    size_t k = best;
    float acc = 0;
    while (k + 1 < line.size() && acc < look) {
        acc += std::hypot(line[k + 1].x - line[k].x, line[k + 1].z - line[k].z);
        ++k;
    }
    sim::Vec3 fwd = rotate(s.orientation, {0, 0, -1}), right = rotate(s.orientation, {1, 0, 0});
    sim::Vec3 d = line[k] - s.position;
    float ang = std::atan2(d.dot(right), d.dot(fwd));
    in.steer = std::clamp(ang * 1.7f, -1.0f, 1.0f);
    in.automatic = true;
    if (s.speed_kmh < target_kmh) in.throttle = std::min(1.0f, (target_kmh - s.speed_kmh) / 15.0f + 0.15f);
    else if (s.speed_kmh > target_kmh + 2) in.brake = std::min(1.0f, (s.speed_kmh - target_kmh) / 20.0f);
    return in;
}

PitLane::Out PitLane::update(const In& in) {
    Out o;
    if (!ok_) return o;
    const sim::VehicleState& s = in.state;
    float dist = distance(s.position);
    float prev = prev_dist_;
    prev_dist_ = dist;
    float to_box = box_ - dist;
    if (to_box < -length_ * 0.5f) to_box += length_;
    if (to_box > length_ * 0.5f) to_box -= length_;
    auto go = [&](State st, int line, bool blend = false) {
        state_ = st;
        o.line = line;
        o.blend = blend;
    };
    switch (state_) {
    case Normal:
        if ((in.on_pit_surface && crossed(prev, dist, entry_)) || (in.armed && !in.race_over && crossed(prev, dist, entry0_))) {
            go(Lane, 3); // onto the pit lane line at once
            stopped_ = false;
            o.entered = true;
        }
        break;
    case Lane:
        // Decision 25 m before the box: stop, or drive through on the pit lane line.
        if (to_box <= 25.0f && to_box > 0) {
            if (!in.race_over && in.want_service && s.speed_kmh < 100.0f) go(Stop, 4);
            else go(Leaving, 3);
        } else if (to_box < -0.5f && crossed(prev, dist, exit_)) { // (entered past the decision point)
            go(Normal, 6, true);
        }
        break;
    case Stop:
        if (!stopped_ && crossed(prev, dist, box_)) {
            stopped_ = true;
            o.begin_stop = true;
            o.box = box_q_;
        } else if (stopped_ && !in.stopping && to_box < -0.5f) {
            go(Leaving, 3);
        }
        break;
    case Leaving:
        if (crossed(prev, dist, exit_)) go(Normal, 6, true);
        break;
    }
    o.exited = o.line == 6;
    o.kind = int(state_);
    o.auto_drive = state_ != Normal;
    o.hold = o.auto_drive && in.stopping;
    return o;
}

} // namespace gt3r::race
