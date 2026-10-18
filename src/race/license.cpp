#include "race/license.h"

#include "game/data/car_database.h"
#include "track/track.h"

#include <algorithm>
#include <cmath>

namespace gt3r::race {

LicenseTimes license_times(ByteSpan b) {
    LicenseTimes t;
    if (b.size() < 0x1D0) return t;
    for (int i = 0; i < 3; ++i) {
        size_t o = 0x1C4 + 4 * size_t(i);
        int m = b[o], s = b[o + 1], ms = rd16(b, o + 2);
        int v = m * 60000 + s * 1000 + ms;
        t.ms[size_t(i)] = v > 0 ? v : -1;
    }
    t.kind = b[0x19A];
    return t;
}

LicenseTimes license_times(const data::Event& ev) { return license_times(ev.bytes); }

int license_medal(const LicenseTimes& t, double seconds) {
    if (seconds < 0) return -1;
    long long ms = std::llround(seconds * 1000.0);
    for (int i = 0; i < 3; ++i)
        if (t.ms[size_t(i)] > 0 && ms <= t.ms[size_t(i)]) return i;
    return -1;
}

const char* course_out(const LicenseRun::Step& s) {
    int off = 0;
    for (int i = 0; i < 4; ++i) {
        if (!s.grounded[i]) continue;
        u8 m = track::surface_material(s.surface[i]);
        if (m == 2 || m == 3 || m == 4 || m == 6) ++off;
    }
    if (off == 4) return "course_out";
    if (s.wall_dv > 10.0f) return "wall";
    return nullptr;
}

long long LicenseRun::stop_time_ms(long long clock_ticks, float speed3d_mps) {
    float f = speed3d_mps / kStopSpeed; // 0..1 when stopped
    long long units = std::max(0LL, clock_ticks) * 50 + (long long)(50.0f * f);
    return units / 3;
}

long long LicenseRun::event_ms(long long clock_ticks, float f) {
    long long units = std::max(0LL, clock_ticks) * 50 + (long long)(50.0f * std::clamp(f, 0.0f, 1.0f));
    return units / 3;
}

LicenseRun::State LicenseRun::step(const Step& s) {
    if (state_ != State::Running) return state_;
    // Failures first, as the game checks them before the stop test.
    int off = 0, pit = 0, stop = 0;
    for (int i = 0; i < 4; ++i) {
        // The stop box: the hub's ground-query triangle, whether the tyre touches it or not.
        if (track::surface_flags(s.surface[i]) & track::kSurfaceFlag4) ++stop;
        if (!s.grounded[i]) continue;
        u8 m = track::surface_material(s.surface[i]), f = track::surface_flags(s.surface[i]);
        if (m == 2 || m == 3 || m == 4 || m == 6) ++off;
        if (f & track::kSurfacePitLane) ++pit;
    }
    if (off == 4 || pit == 4 || s.wall_dv > 10.0f || s.pylon) {
        failure_ = off == 4 ? "course_out" : pit == 4 ? "pit_lane" : s.wall_dv > 10.0f ? "wall" : "pylon";
        return state_ = State::Failed;
    }
    // Kind 1 runs from the first tick (no line-crossing gate; the start area has no 0x04
    // triangles), timed from GO.
    if (rules_.kind == 1) {
        if (s.lap_length <= 0 && s.clock_ticks < 0) { // legacy callers (no clock): time from the line
            if (s.started && stop == 4 && s.speed_mps <= kStopSpeed) time_ = s.elapsed, state_ = State::Finished;
            return state_;
        }
        if (stop == 4 && s.clock_ticks >= 0 && s.speed3d_mps <= kStopSpeed) {
            time_ = double(stop_time_ms(s.clock_ticks, s.speed3d_mps) - start_ms_) / 1000.0;
            state_ = State::Finished;
        }
        return state_;
    }
    if (s.lap_length > 0) {
        // The original's progress events: no events while held.
        if (s.clock_ticks < 0) return state_;
        double L = s.lap_length;
        if (counter_ < 0) counter_ = s.dist_prev >= 0 ? int(std::floor(s.dist_prev / L)) + 1 : 0;
        double line = double(counter_) * L; // the next forward line crossing
        if (s.dist_prev < line && s.dist >= line) {
            float f = float((line - s.dist_prev) / (s.dist - s.dist_prev));
            long long ms = event_ms(s.clock_ticks, f);
            int old = counter_++;
            if (old == 0) {
                if (!rules_.standing) start_ms_ = ms; // flying tests: timed from the line
            } else if (rules_.kind == 3 && old == std::max(1, rules_.laps)) {
                time_ = double(ms - start_ms_) / 1000.0;
                state_ = State::Finished;
            }
            return state_;
        }
        if (rules_.kind == 2 && counter_ >= 1 && rules_.goal_m > 0) {
            double base = double(counter_ - 1) * L, p0 = s.dist_prev - base, p1 = s.dist - base;
            if (p0 < rules_.goal_m && p1 >= rules_.goal_m) {
                float f = float((rules_.goal_m - p0) / (p1 - p0));
                time_ = double(event_ms(s.clock_ticks, f) - start_ms_) / 1000.0;
                state_ = State::Finished;
            }
        }
        return state_;
    }
    if (!s.started) return state_;
    switch (rules_.kind) {
    case 2:
        if (rules_.goal_m > 0 && s.progress_prev < rules_.goal_m && s.progress >= rules_.goal_m) {
            double f = s.progress > s.progress_prev ? double(s.progress - rules_.goal_m) / double(s.progress - s.progress_prev) : 0;
            time_ = std::max(0.0, s.elapsed - f * s.dt); // the crossing inside the step
            state_ = State::Finished;
        }
        break;
    default:
        if (s.laps_completed >= std::max(1, rules_.laps)) {
            time_ = s.laps_time;
            state_ = State::Finished;
        }
        break;
    }
    return state_;
}

} // namespace gt3r::race
