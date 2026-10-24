#pragma once

// Pit stops (docs/dev/RACE.md "Pit stops and tyre wear"; physics §8.6, ai §13). The course needs AI
// lines 3 (pit lane, file line 2) and 4 (pit stop line, file line 3); the stop line's points' track
// distances are the markers.
//
// This is the pit state machine only; the car is driven by its ai::AiDriver on the line the state
// asks for (Out::line): the pit lane line at the set's pit speed, the stop line at 20 km/h.
// State (the physics driver kind): 0 normal; 2 pit lane (line 3, from the entry); 3 the stop
// (line 4); 4 leaving (line 3 again). Entry: in state 0, on the pit-lane surface, crossing
// stop-line point 1 forwards (or, for a CPU car with the pit request, point 0). 25 m before the
// box: drive through (state 4) when the race is over, no tyre is used (or wear is off) or at
// >= 100 km/h, otherwise state 3. Crossing the box (stop-line point N - 4 - n for car n):
// sim::Vehicle::begin_pit_stop (5 s, tyres at 1 s). 0.5 m past the box -> state 4; at stop-line
// point N - 2 -> state 0 and back to the main line (blended). The player has no speed limiter of
// their own; the pit lane is always driven by the computer.

#include "sim/sim_types.h"

#include <vector>

namespace gt3r::track {
class Track;
}

namespace gt3r::race {

class PitLane {
public:
    enum State : int { Normal = 0, Lane = 2, Stop = 3, Leaving = 4 };

    // car_index = n (0 for the player). False when the course has no pit lines.
    bool load(const track::Track& t, int car_index = 0);
    bool available() const { return ok_; }
    void reset() { state_ = Normal, prev_dist_ = -1, stopped_ = false; }

    struct In {
        sim::VehicleState state;
        bool on_pit_surface = false;
        bool want_service = false; // some tyre used (wear on)
        bool race_over = false;
        bool stopping = false;     // the 5 s stop is running (spawn hold active)
        bool armed = false;        // CPU car with the pit request (tyres worn): enters at stop-line point 0
    };
    struct Out {
        bool auto_drive = false;   // the pit lane is driving the car (state != 0)
        int kind = 0;              // the driver kind of the state (0, 2, 3, 4)
        // Switch the car's AI driver to this game line now (0 = no change): 3 at the entry and when
        // leaving or driving through, 4 for the stop, 6 at the exit. `blend`: only the exit blends.
        int line = 0;
        bool blend = false;
        bool hold = false;         // the 5 s stop is running: hold the car with the brake
        bool begin_stop = false;   // call Vehicle::begin_pit_stop(box) now
        sim::Quat box;             // box heading
        bool entered = false;      // the car entered the pit lane this tick ("Pitting in")
        bool exited = false;       // back to normal this tick (reset the AI's recovery state)
    };
    Out update(const In& in);
    State state() const { return state_; }
    // For a driver heading for the pits (test autopilot): metres to the entry marker
    // along the lap, and inputs following the pit lane line at `kmh`.
    float to_entry(const sim::Vec3& p) const;
    sim::ControlInputs lane_inputs(const sim::VehicleState& s, float kmh) const { return follow(lane_, s, kmh); }
    bool driving() const { return state_ != Normal; }

private:
    float distance(const sim::Vec3& p) const;
    bool crossed(float prev, float cur, float mark) const;
    // [placeholder, test autopilot only] a simple follower of a sampled line at a fixed speed.
    sim::ControlInputs follow(const std::vector<sim::Vec3>& line, const sim::VehicleState& s, float target_kmh) const;

    const track::Track* t_ = nullptr;
    bool ok_ = false;
    State state_ = Normal;
    std::vector<sim::Vec3> lane_; // sampled pit lane line (test autopilot)
    float entry0_ = 0, entry_ = 0, box_ = 0, exit_ = 0, length_ = 0;
    sim::Quat box_q_;
    float prev_dist_ = -1;
    bool stopped_ = false;
};

} // namespace gt3r::race
