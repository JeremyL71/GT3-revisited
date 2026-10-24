#include "ai/safety_respawn.h"

#include "ai/ai_driver.h"
#include "sim/vehicle.h"
#include "track/track.h"

#include <algorithm>
#include <cmath>

namespace gt3r::ai {

bool SafetyRespawn::lost(const track::Track& t, const sim::VehicleState& st, double race_time) {
    // Off the road mesh: no road below a point 3 m above, or the road more than 5 m above the
    // point. All five probes, since one can fall through a hairline crack between triangles.
    auto off = [&](const sim::Vec3& p) {
        sim::SurfaceHit h = t.probe(p.x, p.z, p.y + 3.0f);
        return !h.hit || p.y < h.height - 5.0f;
    };
    bool all_off = off(st.position);
    for (const sim::WheelState& w : st.wheels) all_off = all_off && off(w.contact);
    off_mesh_ticks_ = all_off ? off_mesh_ticks_ + 1 : 0;
    bool lost = off_mesh_ticks_ > kOffMeshTicks || !std::isfinite(st.position.x);
    if (race_time > 0 && std::max(race_time, 0.0) - last_progress_ > kStallSeconds) lost = true;
    return lost;
}

void SafetyRespawn::progress(double race_distance, double race_time) {
    if (race_distance > best_ + kProgressMetres) best_ = race_distance, last_progress_ = std::max(race_time, 0.0);
}

void SafetyRespawn::hold(double race_time) { last_progress_ = std::max(race_time, 0.0); }

void SafetyRespawn::respawned(double race_time) {
    last_progress_ = std::max(race_time, 0.0);
    off_mesh_ticks_ = 0;
}

void respawn_car(sim::Vehicle& car, track::LapTimer& timer, AiDriver& driver, const track::Track& t) {
    track::Pose p = t.respawn_pose(timer.distance());
    car.spawn(p.position, p.orientation);
    timer.teleport(p.position);
    driver.reset(car.state());
}

} // namespace gt3r::ai
