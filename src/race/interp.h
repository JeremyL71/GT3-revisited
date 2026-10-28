#pragma once

// Fixed-tick simulation helpers: an accumulator that turns variable frame
// times into whole simulation ticks, and interpolation of vehicle states
// between the last two ticks for rendering.

#include "sim/sim_types.h"

namespace gt3r::race {

class FixedStep {
public:
    explicit FixedStep(double hz = sim::kDefaultTickHz) : step_(1.0 / hz) {}

    // Adds frame time; returns the number of ticks to run now (at most
    // `max_ticks`: after a long stall the excess time is dropped instead of
    // making the game spiral).
    int advance(double frame_dt, int max_ticks = 8);
    // Ticks paced by a shared clock instead (online races): `clock_ticks` ticks are due by now
    // and `done` have run. Returns the number to run now (at most `max_ticks`) and counts them
    // in `done`; alpha() is then the clock's way into the tick after them (0..1). Frame times
    // and the clock's ticks fall in different places: the blend has to come from the clock too.
    int follow(double clock_ticks, long long& done, int max_ticks);
    // Fraction (0..1) of a tick elapsed since the last tick: render the
    // state interpolated between previous and current by this amount.
    float alpha() const { return float(accum_ / step_); }
    double step() const { return step_; }
    void reset() { accum_ = 0; }

private:
    double step_;
    double accum_ = 0;
};

sim::Quat normalize(const sim::Quat& q);
// Shortest-arc spherical interpolation.
sim::Quat slerp(const sim::Quat& a, const sim::Quat& b, float t);
// Rotates a car-frame vector into the world frame.
sim::Vec3 rotate(const sim::Quat& q, const sim::Vec3& v);
sim::Quat quat_from_yaw(float yaw); // rotation about +Y
sim::Quat quat_mul(const sim::Quat& a, const sim::Quat& b);

// The wheel's turn from a to b: b's spin_step when the angles are that far apart (by whole turns; b a tick after a),
// else the shorter way round.
float spin_change(const sim::WheelState& a, const sim::WheelState& b);
// Interpolates everything rendering, camera, HUD and audio read. The wheels turn by
// spin_change (their spin angles wrap); gear and flags come from `b`.
sim::VehicleState interpolate(const sim::VehicleState& a, const sim::VehicleState& b, float t);

} // namespace gt3r::race
