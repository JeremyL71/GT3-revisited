#pragma once

// Online battle helpers for the race scene (docs/dev/ONLINE.md).
//
// Each player simulates only its own car; the others arrive as net::CarSnapshot.
// RemoteCar turns the newest snapshot into the car's pose "now" on the local race
// clock: dead reckoning along its velocity and spin (capped), and on each new
// snapshot the jump between what was shown and the new prediction is blended out
// over ~0.1 s so cars glide instead of popping. Collisions use the unblended
// prediction (the best guess of where the car really is).
//
// All players start the countdown at the same wall-clock instant (net::Event::Go), so
// the owner's race_time in a snapshot and the local race clock are the same timeline:
// age = local race time - snapshot race time.

#include "net/protocol.h"
#include "sim/sim_types.h"

#include <array>
#include <memory>
#include <optional>

namespace gt3r::sim {
class Vehicle;
struct VehicleSetup;
class TrackQuery;
class WorldQuery;
}

namespace gt3r::race {

class RemoteCar {
public:
    static constexpr double kMaxExtrapolation = 0.25; // s; older data is held, not flung onward
    static constexpr double kBlend = 0.1;             // s, time constant of visual corrections
    static constexpr float kSnapDistance = 8.0f;      // m: larger jumps (respawns) are not blended

    // Physics-based prediction: a vehicle built from the remote car's spec runs on from each
    // new state with the owner's newest input (braking and turn-in come out right, where dead
    // reckoning keeps the car going straight at speed). Without it, dead reckoning.
    void attach(std::unique_ptr<sim::Vehicle> shadow, const sim::TrackQuery* track, const sim::WorldQuery* world);
    bool physics() const { return shadow_ != nullptr; }

    // `local_race_time`: the local race clock when it arrived (NaN = its own race_time).
    void receive(const net::CarSnapshot& s, double local_race_time = kNoTime);
    bool has() const { return have_; }
    const net::CarSnapshot& latest() const { return snap_; }

    struct Pose {
        sim::VehicleState state;               // shown (blended)
        float body_x = 0, body_z = 0;          // collision CoG x/z (unblended)
        float heading = 0, yaw_rate = 0;
        float brake = 0;
    };
    // The car at local race time t (with physics: steps the shadow up to t).
    Pose predict(double local_race_time);
    // How far the prediction was from the owner's state when it arrived (metres, after the
    // last receive; the horizon is the time between the two states). Plausibility checks.
    float last_error() const { return last_error_; }
    double last_error_horizon() const { return last_horizon_; }
    void reset();

    static constexpr double kNoTime = -1e300;

private:
    Pose raw(double t) const;            // dead reckoning from snap_
    Pose shadow_pose() const;
    void step_shadow();
    net::CarSnapshot snap_;
    bool have_ = false;
    sim::Vec3 off_pos_;   // shown - raw at corr_time_
    sim::Quat off_rot_;   // shown = off_rot * raw at corr_time_
    double corr_time_ = 0;
    // Physics.
    std::shared_ptr<sim::Vehicle> shadow_;
    const sim::TrackQuery* track_ = nullptr;
    const sim::WorldQuery* world_ = nullptr;
    double sim_time_ = 0;      // race time the shadow has reached
    u32 sim_tick_ = 0;         // owner tick the shadow has reached
    sim::ControlInputs input_; // the owner's newest input
    bool reverse_ = false;
    std::array<sim::Vec3, 64> history_{}; // shadow position by tick (ring), for last_error()
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

// The local car's snapshot after a tick. `inputs`: this tick's input first, then older
// ones (up to net::kInputHistory are sent).
net::CarSnapshot make_snapshot(const sim::Vehicle& v, u32 tick, int laps, const net::InputFrame* inputs, int n,
                               bool respawned = false);

// Fills in what the wire leaves out, from the remote car's own setup: the race clock
// (tick / 60 - countdown), the CoG for collisions, wheel contact points (from the
// suspension travel), loads (grounded wheels share the weight) and speed.
void complete_snapshot(net::CarSnapshot& s, const sim::VehicleSetup& setup, float mass_kg, double countdown);

} // namespace gt3r::race
