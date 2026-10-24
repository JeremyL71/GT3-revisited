#pragma once

// Licence tests (docs/dev/RACE.md "Licence tests"): the event's medal times and
// the judgement GT mode uses too (GameResult::license_medal).
//
// Event record (database table 32): gold / silver / bronze at +0x1C4 / +0x1C8 /
// +0x1CC, each {u8 minutes, u8 seconds, u16 milliseconds}; +0x19A test kind
// (1 stop test, 2 section, 3 full lap(s)).

#include "base/bytes.h"

#include <array>
#include <string>

namespace gt3r::data {
struct Event;
} // namespace gt3r::data

namespace gt3r::race {

struct LicenseTimes {
    std::array<int, 3> ms{-1, -1, -1}; // gold, silver, bronze (ms); -1 = none
    int kind = 0;                      // +0x19A
    bool valid() const { return ms[2] > 0; }
};
LicenseTimes license_times(const data::Event& ev);
LicenseTimes license_times(ByteSpan event_record);

// One licence run, judged every simulation tick (docs/dev/RACE.md "Licence tests"):
//   kind 1 stop test: finished on the first tick all four wheels' hub ground queries are on
//          stop-zone triangles (road flag 0x04, grounded or not) and the 3-D body speed is
//          <= 1 km/h (no hold time, no line-crossing gate); timed from GO with the
//          original's sub-tick term from the leftover speed; partial stops keep waiting;
//   kind 2 section: finished when the lap's track progress crosses the course's goal
//          distance (GTRW header +0x1C), once the lap counter is >= 1;
//   kind 3 laps: finished on the line crossing whose old lap counter equals the laps
//          (+0x199): one clock difference, not a sum of lap times.
// Times (as the game's licence tests keep them): the race clock in 1/3000 s (+50 per tick,
// running from GO; checks see the value before this tick's advance), an event inside a
// tick adds trunc(50 f) with f its fraction of the tick (progress interpolated, or the
// leftover speed for kind 1), ms = floor(units / 3) - start. Standing tests start at GO
// (start 0); flying tests at the first line crossing's interpolated time. The lap counter
// starts at 0 behind the line, 1 past it, and counts forward line crossings.
// Failures end the run at once: all four wheels on grass / gravel / sand (materials
// 2, 3, 4, 6), all four wheels on the pit lane (flag 0x01), or a wall hit changing
// the car's speed by more than 10 m/s. Wrong way, car contact, stopping and time
// never fail a test.
class LicenseRun {
public:
    enum class State { Running, Finished, Failed };
    struct Rules {
        int kind = 3;
        int laps = 1;
        float goal_m = 0; // section finish distance (kind 2)
        bool standing = true; // standing start (V0 = 0): timed from GO
    };
    struct Step {
        bool started = false;        // the clock runs (after the signal / the start line)
        double elapsed = 0;          // seconds since the start
        double dt = 1.0 / 60;
        float progress_prev = 0, progress = 0; // metres along the lap since the start line
        int laps_completed = 0;
        double laps_time = 0;        // sum of the completed laps (kind 3)
        float speed_mps = 0;
        float wall_dv = 0;           // speed change from wall hits this step (m/s)
        bool pylon = false;          // the car touched a pylon (any touch fails, no threshold)
        bool grounded[4] = {};
        u32 surface[4] = {};         // raw road surface under each wheel (its hub's ground query)
        // The original's race clock: whole ticks since the start signal (GO) completed before
        // this tick, < 0 while the car is held; and the body's 3-D speed (incl. vertical).
        long long clock_ticks = -1;
        float speed3d_mps = 0;
        // Unwrapped track distance from the start line (m, negative behind it) before and after
        // this tick, and the lap length. lap_length > 0 selects the original's clock timing for
        // kinds 2 / 3; otherwise the legacy fields above (started / elapsed / progress / laps).
        double dist_prev = 0, dist = 0;
        float lap_length = 0;
    };
    static constexpr float kStopSpeed = 0.27777779f; // 1 km/h (f32, as the original)
    // Kind 1 recorded time in ms: floor((clock + trunc(50 |v| / 1 km/h)) / 3), the clock in
    // 1/3000 s before this tick's advance (+50 per tick), from GO (standing start).
    static long long stop_time_ms(long long clock_ticks, float speed3d_mps);
    // An event at fraction f (0..1) of the tick: floor((50 clock_ticks + trunc(50 f)) / 3) ms.
    static long long event_ms(long long clock_ticks, float f);
    void begin(const Rules& r) {
        rules_ = r, state_ = State::Running, time_ = -1, failure_.clear();
        counter_ = -1, start_ms_ = 0;
    }
    int lap_counter() const { return counter_; } // -1 until the first clocked step
    long long start_ms() const { return start_ms_; } // the clock origin (0 = GO)
    State step(const Step& s);
    State state() const { return state_; }
    // Tests (GT3R_RACE lic_finish=<s>): end a running run as finished with this time.
    void force_finish(double seconds) {
        if (state_ == State::Running) time_ = seconds, state_ = State::Finished;
    }
    double time() const { return time_; }               // seconds (finished runs)
    const std::string& failure() const { return failure_; } // "course_out", "pit_lane", "wall"
    const Rules& rules() const { return rules_; }

private:
    Rules rules_;
    State state_ = State::Running;
    double time_ = -1;
    std::string failure_;
    int counter_ = -1;        // the game's lap counter (docs/dev/RACE.md "Licence tests")
    long long start_ms_ = 0;  // 0, or the time of the first crossing (flying)
};

// Course-out (physics §11, Time Trial and License): all four wheels on grass / gravel / sand
// (materials 2, 3, 4, 6) or a wall hit changing the speed by more than 10 m/s; car-car
// hits never count. Returns "course_out" / "wall", or nullptr.
const char* course_out(const LicenseRun::Step& s);

// Medal for a finished run: 0 gold, 1 silver, 2 bronze, -1 none. The time is
// rounded to the millisecond; limits are inclusive.
int license_medal(const LicenseTimes& t, double seconds);

} // namespace gt3r::race
