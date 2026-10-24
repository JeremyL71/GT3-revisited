#pragma once

// The AI's decision rules as small pure functions (used by AiDriver::update and the race; tested
// one by one in tests/test_ai_rules.cpp). Section numbers are docs/behaviour/ai.md's.

#include "base/bytes.h"
#include "sim/sim_types.h"

#include <array>
#include <vector>

namespace gt3r::sim {
struct CarSpec;
}

namespace gt3r::ai {

struct DriverParams;
class AiCourse;

// --- grip (ai §4.2, ai §4.3) ----------------------------------------------------------------

// 9.8 x the tyre record's mu for surface `surface` on axle `axle` (0 front, 1 rear): the record of
// the compound slot that surface uses (sim::CarSpec::surface_slot). That mu is the compound's
// base mu x the surface's grip percentage (physics §6.5); records without a per-surface table
// use base mu x sim::model::surface_mu.
float record_grip(const sim::CarSpec& spec, int axle, int surface);

// The grip acceleration (m/s^2) the corner speed of a point with `surface` is planned with:
// w x front + (1 - w) x rear record grip, w = the parameter set's front weight. With the tyre
// wear factors live (`live_wear`: off on the pit surface), each axle's grip is scaled by the mean
// of its two wheels' wear grip factors.
float corner_grip(const sim::CarSpec& spec, int surface, float front_weight, const std::array<float, 4>& wear_grip,
                  bool live_wear);

// The front grip the braking distance is planned with (ai §4.3): the front record grip on the
// next point's surface, scaled by the mean front wear grip factor when live.
float braking_grip(const sim::CarSpec& spec, int surface, const std::array<float, 4>& wear_grip, bool live_wear);

// Corner speed (m/s) of an arc (ai §4.2): lateral acceleration a = mu |r| (more on a bank leaning
// into the arc), three downforce iterations v <- sqrt(a (1 + k v^2)) from v = 0, then the set's
// scale and offset, the trim and the point's km/h adjustment, clamped to [20 km/h, v_top].
float corner_speed(float mu, float radius, float bank, int speed_adjust_kmh, const DriverParams& p, float k_down,
                   float v_top);

// --- speed control (ai §4.4, ai §5) ---------------------------------------------------------

struct Pedals {
    float throttle = 0, brake = 0;
};
// throttle = clamp(0.75 + gain x clamp(v - (target - 2 km/h), +-10 km/h), 0, 1); full brake only
// with the throttle shut and v above the target.
Pedals speed_controller(float v, float target, float gain);

// Where a car on the pit lane line (game line 3) is (ai §4.4 pit speed), by the pit STOP line's
// (game line 4) points, the pit markers: 0 outside the limit, 1 from stop-line point 1 (the pit
// entry) to point 2 (the braking modes aim at the pit speed), 2 from point 2 to point n - 3
// (held at the pit speed). Distances are compared lap-unwrapped: the stop line's
// points from the first one in the first half of the lap on, and the car's distance when in the
// first half, have a lap length added. `s` is the car's track distance.
int pit_lane_section(const AiCourse& course, float s);
// The stop line's point distances with that lap added (pit_lane_section).
std::vector<float> pit_markers(const AiCourse& course);

// What the speed controller aims at this tick (ai §4.4).
struct SpeedPlan {
    int mode = 0;         // 0 straight (full throttle), 1/2 arc, 3/4 braking
    float target = 0;     // the arc modes' target (before the corner bonus) or the state-2 target
    float next = 0;       // the braking modes' target
    bool turning = false; // recovery state 2: target 50 km/h, gain -0.5, no mode
    bool pit_limit = false; // on the pit stop line or in the pit lane's limit section
};
// line: the driver's game line; state: the recovery state; mode, v_cur, v_next: the point
// bookkeeping's; pit_speed: the set's pit-lane speed (m/s); section: pit_lane_section().
SpeedPlan plan_speed(int line, int state, int mode, float v_cur, float v_next, float pit_speed, int section);

// The arc modes' target: plus the set's corner bonus once the car has been in the arc for the
// bonus delay, except on the pit surface.
float arc_target(float target, int arc_ticks, const DriverParams& p, bool on_pit_surface);

// --- recovery (ai §7) -------------------------------------------------------------------------

// Heading threshold theta: 60 degrees; in state 1, 10 degrees moving forward, opening linearly to
// 60 degrees at 30 km/h in reverse.
float recovery_threshold(int state, float v);
enum Drive : int { kDriveBrake = 0, kDriveForward = 1, kDriveReverse = -1 };
// |eps| < theta or state 2: forward unless rolling back faster than 2 km/h (then brake);
// otherwise brake while faster than 2 km/h, else reverse.
int drive_decision(int state, float abs_eps, float theta, float v);
// The state after this tick (0 racing, 1 reversing, 2 turning round). hit_sides: the
// collide::HitSide bits of the last collision step (a front and / or a rear impulse).
//   0 -> 2: |eps| > 80, or rear hit and |eps| > 45 degrees;
//   1 -> 0: |eps| < theta and v > 3 km/h;  1 -> 2: |eps| < theta and v <= 3 km/h, or
//           |eps| >= theta and a rear hit;
//   2 -> 0: |eps| < 40 degrees and v > 30 km/h;  2 -> 1: otherwise, a front hit and |eps| > 60.
int recovery_next(int state, float abs_eps, float theta, float v, unsigned hit_sides);

// --- end of a run (ai §1.1) ----------------------------------------------------------------------

// Driver kinds (sim::Vehicle::set_driver_kind).
enum DriverKind : int { kKindNormal = 0, kKindAi = 1, kKindPitLane = 2, kKindPitStop = 3, kKindPitLeave = 4,
                        kKindStopFree = 5, kKindStopLine = 6, kKindStopStraight = 7, kKindParked = 8 };
enum class RunEnd { RaceFinish, TimeTrialFinish, MachineTest, LicenceFinish, LicenceFail, TimeTrialFail, FreeRunEnd };
struct EndOfRun {
    int kind = kKindAi;
    bool slow = false; // the run-on driver's corner speed x 0.8 and throttle 0.5 (after_finish)
    bool main_line = false; // the player's car switches to the main line (6) first
};
// What drives the player's car once its run is over: kind 1 (the AI drives on along the main line)
// after a race, time trial or machine test, slowed; kind 7 (stop, wheels straight) after a
// failed licence test or time trial and at the end of a free run on the pit surface; kind 6
// (stop along the main line) after licence tests of kind 1 (stop) or 2 (section); kind 1 slowed
// after the other licence tests; kind 1 at the end of a free run elsewhere.
EndOfRun end_of_run(RunEnd why, int licence_test_kind = 0, bool on_pit_surface = false);
// The stop driver's test for "parked" (kind 8): 3-D speed < 0.0694 m/s (0.25 km/h) and angular
// speed < 0.0524 rad/s (3 degrees/s).
bool parked(const sim::VehicleState& st);

} // namespace gt3r::ai
