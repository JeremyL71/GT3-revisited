#pragma once

// Ghosts (presentation §G11): the original's rules for which run (lap) becomes the ghost, and GhostLap, a car's states
// over a lap, one per race step (the Analyzer's REPLAY draws track B's lap from them in step with track A's). The ghosts of
// the live runs are cars of their race (race/exact_field.h: ExactFieldOptions::ghost, the free run's ghost car).

#include "base/bytes.h"
#include "sim/sim_types.h"

#include <vector>

namespace gt3r::race {

// The original's ghost rules (presentation §G11). Licence tests, time trials and machine
// tests: the ghost is the session's best run, the whole run from its first tick, kept when a
// run sets the session record (at or under the session's best). Free runs: the session's best
// lap. A run's (a free run's lap's) recording is kept only while it is under 36,000 ticks (10
// minutes): a longer one sets no record and no ghost.
constexpr u32 kGhostRecordTicks = 36000;
constexpr bool ghost_recording_kept(u32 ticks) { return ticks < kGhostRecordTicks; }
// After a live run (a free run: a lap that became the best lap): the ghost is replaced by the
// best run (lap) when the run has a time (whole ms; a FAIL, an EXIT or the top-speed test has
// none), its recording was kept, and there is no ghost yet (ghost_ms < 0) or the time is
// strictly under the ghost's.
constexpr bool replaces_ghost(long long time_ms, bool recording_kept, long long ghost_ms) {
    return time_ms >= 0 && recording_kept && (ghost_ms < 0 || time_ms < ghost_ms);
}
// A free run's lap becomes the session's best lap when its recording was kept and it is
// strictly under the best lap so far (best_ms < 0: none yet).
constexpr bool takes_best_lap(long long lap_ms, bool recording_kept, long long best_ms) {
    return lap_ms >= 0 && recording_kept && (best_ms < 0 || lap_ms < best_ms);
}

class GhostLap {
public:
    void clear() { frames_.clear(), shown_.clear(); }
    // `shown` false: the ghost car is not drawn on this frame.
    void add(const sim::VehicleState& s, bool shown = true) { frames_.push_back(s), shown_.push_back(shown); }
    bool empty() const { return frames_.empty(); }
    size_t size() const { return frames_.size(); }
    // State `seconds` after the lap started (interpolated); false when past the end or hidden.
    bool state_at(double seconds, double tick_hz, sim::VehicleState& out) const;
    std::vector<sim::VehicleState>& frames() { return frames_; } // (edits drop the shown flags)
    bool shown(size_t i) const { return shown_.size() != frames_.size() || shown_[i]; }
    void set_shown(size_t i, bool on) {
        if (i < shown_.size()) shown_[i] = on;
    }

private:
    std::vector<sim::VehicleState> frames_;
    std::vector<bool> shown_;
};

} // namespace gt3r::race

#include "race/interp.h"

namespace gt3r::race {

inline bool GhostLap::state_at(double seconds, double tick_hz, sim::VehicleState& out) const {
    if (frames_.empty() || seconds < 0) return false;
    double f = seconds * tick_hz;
    size_t i = size_t(f);
    if (i + 1 >= frames_.size() || !shown(i)) return false;
    out = interpolate(frames_[i], frames_[i + 1], float(f - double(i)));
    return true;
}

} // namespace gt3r::race
