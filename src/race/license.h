#pragma once

// Licence tests (docs/dev/RACE.md "Licence tests"): the event's medal times and
// the medal GT mode uses too (GameResult::license_medal).
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

// A licence run as the race screen keeps it: the test's rules (its kind and laps, a standing or a flying start) and, once
// the race flow has judged it (the licence conductor, sim/exact/race_flow.h), its result: finished with its time, or failed.
// The test (docs/dev/RACE.md "Licence tests"): kind 1 a stop test, 2 a section, 3 full lap(s); a standing test is timed
// from GO, a flying one from its first line crossing.
class LicenseRun {
public:
    enum class State { Running, Finished, Failed };
    struct Rules {
        int kind = 3;
        int laps = 1;
        bool standing = true; // standing start (V0 = 0): timed from GO
    };
    void begin(const Rules& r) { rules_ = r, state_ = State::Running, time_ = -1, failure_.clear(); }
    // The run's result (a finished run's time in seconds; `failure`: why a failed run failed).
    void end(State s, double time, std::string failure = {}) {
        state_ = s, time_ = s == State::Finished ? time : -1, failure_ = std::move(failure);
    }
    State state() const { return state_; }
    double time() const { return time_; }               // seconds (finished runs)
    const std::string& failure() const { return failure_; }
    const Rules& rules() const { return rules_; }

private:
    Rules rules_;
    State state_ = State::Running;
    double time_ = -1;
    std::string failure_;
};

// Medal for a finished run: 0 gold, 1 silver, 2 bronze, -1 none. The time is
// rounded to the millisecond; limits are inclusive.
int license_medal(const LicenseTimes& t, double seconds);

// A judged licence run as GT3 presents it, from the race flow's result words (race info + 0x40 / + 0x44; the licence
// conductor's judging, sim/exact/race_flow.h licence_end / licence_out) and whether it posted the failed event (9):
//   - result 2..4: finished with a medal (0 gold .. 2 bronze), the result's time;
//   - result 0 with the failed event: slower than bronze: finished, no medal, the result's time: the result screen's
//     TIME EXPIRED with the time (presentation §A4);
//   - result -1: out of the course: failed, no time: FAIL;
//   - result 0 without it (a test with no judging times): finished, no medal, `best` (s).
struct LicenseOutcome {
    LicenseRun::State state = LicenseRun::State::Finished;
    double time = -1; // seconds (-1 none)
    int medal = -1;
    std::string failure;
};
LicenseOutcome license_outcome(int result, u32 result_time_ms, bool failed_event, double best);

// Whether a licence run's end plays the fail jingle (music §3.1): GT3's judging posts its failed event (9) for a course-out
// and for a time slower than bronze, live and in the run's replay or demonstration alike (those post it too: the race-flow
// byte keeps only the result from being stored), and the fail jingle answers it; a medal plays its medal jingle live and the
// finish jingle (event 5) in a replay or demonstration. So: every outcome without a medal.
inline bool license_fail_jingle(LicenseRun::State s, int medal) { return s != LicenseRun::State::Finished || medal < 0; }
inline bool license_fail_jingle(const LicenseOutcome& o) { return license_fail_jingle(o.state, o.medal); }

} // namespace gt3r::race
