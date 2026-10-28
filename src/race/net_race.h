#pragma once

// Online battle helpers for the race scene (docs/dev/ONLINE.md).
//
// Each player runs only its own car on the exact core; the others arrive as net::CarSnapshot (their records' motion and
// their pad frames). RemoteCar turns the newest snapshot into the car "now" on the local race clock: a prediction shadow
// (the remote car alone on a one-car exact race of our course, race::exact_shadow) is synced from each snapshot and run on
// with the owner's newest frame; without one, dead reckoning along the velocity (capped). On each new snapshot the jump
// between what was shown and the new prediction is blended out over ~0.1 s so cars glide instead of popping. Collisions use
// the unblended prediction (the best guess of where the car really is).
//
// All players start the countdown at the same wall-clock instant (net::Event::Go), so the owner's race clock in a snapshot and
// the local race clock are the same timeline: age = local race time - snapshot race time.

#include "net/protocol.h"
#include "sim/sim_types.h"

#include <array>
#include <deque>
#include <memory>
#include <optional>

namespace gt3r::sim {
struct VehicleSetup;
}
namespace gt3r::sim::exact {
struct Car;
class ExactRace;
} // namespace gt3r::sim::exact

namespace gt3r::race {

class RemoteCar {
public:
    static constexpr double kMaxExtrapolation = 0.25; // s; older data is held, not flung onward
    static constexpr double kBlend = 0.1;             // s, time constant of visual corrections
    static constexpr float kSnapDistance = 8.0f;      // m: larger jumps (respawns) are not blended

    RemoteCar();
    ~RemoteCar();
    RemoteCar(RemoteCar&&) noexcept;
    RemoteCar& operator=(RemoteCar&&) noexcept;

    // The prediction shadow: a one-car race of the remote car (its car, entry and grid slot) on our course. `physics`: it
    // runs on from each state with the owner's newest frame (braking and turn-in come out right, where dead reckoning keeps
    // the car going straight at speed); false: it is only synced, and carried on along its velocity (dead reckoning).
    void attach(std::unique_ptr<sim::exact::ExactRace> shadow, const net::CarEntry& entry, bool physics = true);
    bool attached() const { return shadow_ != nullptr; }
    bool physics() const { return shadow_ && physics_; }
    // The entry the shadow was built for (a shadow is rebuilt when the owner's differs; pedal style and driver kind follow
    // the states).
    const net::CarEntry& shadow_entry() const { return entry_; }

    // A completed snapshot (complete_snapshot). `local_race_time`: the local race clock when it arrived (kNoTime = its own).
    void receive(const net::CarSnapshot& s, double local_race_time = kNoTime);
    bool has() const { return have_; }
    const net::CarSnapshot& latest() const { return newest_; } // the newest received (one ahead of our race waits)

    struct Pose {
        sim::VehicleState state;                  // shown (blended)
        float body_x = 0, body_z = 0;             // collision CoG x / z (unblended, course frame)
        float heading = 0, yaw_rate = 0;
        float brake = 0;
        const sim::exact::Car* car = nullptr;     // the predicted record (unblended; valid until the next call)
    };
    // The car at local race time t (with a shadow: steps it up to t, one step per call in play).
    Pose predict(double local_race_time);
    // How far the prediction was from the owner's state when it arrived (metres, after the last receive; the horizon is the
    // time between the two states). Plausibility checks.
    float last_error() const { return last_error_; }
    double last_error_horizon() const { return last_horizon_; }
    void reset();

    static constexpr double kNoTime = -1e300;

private:
    static constexpr size_t kPending = 16;
    void apply(const net::CarSnapshot& s, double t);
    Pose predict_from_applied(double t);
    Pose raw(double t) const; // dead reckoning from snap_ (no shadow)
    Pose shadow_pose(double t);
    void step_shadow();
    void sync(const net::CarSnapshot& s);
    net::CarSnapshot snap_;   // the state the prediction runs from
    net::CarSnapshot newest_; // the newest received
    std::deque<net::CarSnapshot> pending_; // received states ahead of our race clock, oldest first
    bool have_ = false;
    sim::Vec3 off_pos_;   // shown - raw at corr_time_
    sim::Quat off_rot_;   // shown = off_rot * raw at corr_time_
    double corr_time_ = 0;
    // The shadow.
    std::unique_ptr<sim::exact::ExactRace> shadow_;
    std::unique_ptr<sim::exact::Car> moved_; // dead reckoning: the synced record carried on to now
    bool physics_ = true;
    net::CarEntry entry_;
    std::unique_ptr<sim::VehicleSetup> setup_; // the shadow car's geometry
    double hz_ = 60;           // the shadow's steps per second
    double sim_time_ = 0;      // race time the shadow has reached
    net::PadInput input_;      // the owner's newest frame
    std::array<std::pair<double, sim::Vec3>, 64> history_{}; // shadow CoG by race time (ring), for last_error()
    size_t nhistory_ = 0;
    float last_error_ = 0;
    double last_horizon_ = 0;
};

// Plausibility checks on another player's car (docs/dev/ONLINE.md "Cheat warnings"): what a
// car of its spec can physically do. Owner authority means a modified game could report
// anything; these catch the blatant cases and tell the host. Each kind fires once per race.
class Plausibility {
public:
    struct Flag {
        net::FlagKind kind;
        float value;
    };
    void set_car(float geared_top_speed_mps) { top_ = geared_top_speed_mps; }
    void reset() { *this = Plausibility{top_}; }
    // A new state of the car; `contact`: our car touched it within the last second.
    std::optional<Flag> on_state(const net::CarSnapshot& s, bool contact);
    // How far physics predicted it from where it said it was (RemoteCar::last_error()).
    std::optional<Flag> on_prediction_error(float metres, double horizon, bool contact);
    // Its claimed finish against when we saw it cross the line.
    std::optional<Flag> on_finish(double claimed, double observed);

    static constexpr float kSpeedMargin = 1.15f, kSpeedSlack = 4.0f;   // x top speed, + m/s
    static constexpr double kSpeedTime = 0.5;                          // s above it
    static constexpr float kTeleport = 6.0f;                           // m beyond its motion
    static constexpr float kAccelG = 2.5f;                             // speed gain over 0.5 s
    static constexpr float kMotionError = 5.0f;                        // m off its own physics...
    static constexpr double kMotionTime = 2.0;                         // ...for this long
    static constexpr double kFinishSlack = 2.0;                        // s

private:
    explicit Plausibility(float top) : top_(top) {}
    std::optional<Flag> fire(net::FlagKind k, float v);
    float top_ = 150;
    bool have_ = false;
    net::CarSnapshot prev_;
    double over_ = 0, motion_ = 0;
    std::array<std::pair<double, float>, 64> speeds_{}; // (race time, speed) ring
    size_t nspeeds_ = 0;
    unsigned fired_ = 0;

public:
    Plausibility() = default;
};

// The record fields a snapshot carries (net::CarMotion) from a car, and back into a car (the shadow's sync): the motion bit
// for bit, the rates, wheel speeds and suspension in their fixed units; the wheels' ground points moved with the car; then
// the body axes and hub offsets from the attitude.
net::CarMotion car_motion(const sim::exact::Car& c);
void sync_motion(sim::exact::Car& c, const net::CarMotion& m);
// The race entry flags of a car for its snapshots (asm / tcs: the race entry's Options).
net::CarEntry car_entry(const sim::exact::Car& c, bool asm_on, bool tcs_on);

// The local car's snapshot after a step. race_time: the race clock (s, negative in the countdown); `inputs`: this step's
// frame first, then older ones (up to net::kInputHistory are sent).
net::CarSnapshot make_snapshot(const sim::exact::Car& c, const net::CarEntry& entry, u32 tick, double race_time, int laps,
                               const net::PadInput* inputs, int n, bool respawned = false);
// Fills in what the wire leaves out: the race time, the CoG and velocity in the course frame (the heading's rotation as the
// orientation), the speed, the collision body's pose, the brake lamp.
void complete_snapshot(net::CarSnapshot& s);

} // namespace gt3r::race
