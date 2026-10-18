#pragma once

// Best-lap ghost for the time trial: the vehicle states of a lap, one per
// simulation tick, replayed in step with the current lap's clock. Recorded
// live while driving, or re-simulated from a replay's best lap
// (ghost_from_replay).

#include "race/backend.h"
#include "race/replay.h"
#include "sim/sim_types.h"

#include <vector>

namespace gt3r::race {

class GhostLap {
public:
    void clear() { frames_.clear(); }
    void add(const sim::VehicleState& s) { frames_.push_back(s); }
    bool empty() const { return frames_.empty(); }
    size_t size() const { return frames_.size(); }
    // State `seconds` after the lap started (interpolated); false when past the end.
    bool state_at(double seconds, double tick_hz, sim::VehicleState& out) const;
    std::vector<sim::VehicleState>& frames() { return frames_; }

private:
    std::vector<sim::VehicleState> frames_;
};

// Re-simulates `r` with a fresh vehicle from `make` and returns the states of
// its best lap (empty if the replay has none). Also reports whether the run
// reproduced the recorded final position exactly.
template <class MakeVehicle>
GhostLap ghost_from_replay(const Replay& r, MakeVehicle&& make, bool* reproduced = nullptr);

} // namespace gt3r::race

#include "race/interp.h"

namespace gt3r::race {

inline bool GhostLap::state_at(double seconds, double tick_hz, sim::VehicleState& out) const {
    if (frames_.empty() || seconds < 0) return false;
    double f = seconds * tick_hz;
    size_t i = size_t(f);
    if (i + 1 >= frames_.size()) return false;
    out = interpolate(frames_[i], frames_[i + 1], float(f - double(i)));
    return true;
}

template <class MakeVehicle>
GhostLap ghost_from_replay(const Replay& r, MakeVehicle&& make, bool* reproduced) {
    GhostLap g;
    auto v = make();
    if (!v) return g;
    v->reset(r.start);
    ReplayPlayer p(&r);
    double dt = 1.0 / r.tick_hz;
    while (!p.done()) {
        while (const ReplayEvent* e = p.pending_event())
            if (e->kind == ReplayEvent::Respawn) v->reset(e->pose);
        u32 t = p.tick();
        if (r.best_lap >= 0 && t >= r.best_lap_begin && t < r.best_lap_end) g.add(v->state());
        v->step(p.next_inputs(), dt);
    }
    if (reproduced) {
        const sim::Vec3& a = v->state().position;
        *reproduced = a.x == r.final_position.x && a.y == r.final_position.y && a.z == r.final_position.z;
    }
    return g;
}

} // namespace gt3r::race
