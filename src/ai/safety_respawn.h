#pragma once

// The engine's safety respawn for CPU cars (not in the original game, which never moves a car:
// ai §12). One rule shared by race::Field and ai::AiRace.
//
// A car is lost when its centre and all four wheel contact points have been off the road mesh
// (no road below a point 3 m above them, or more than 5 m below the road found) for more than
// 30 ticks, when its position is not finite, or, after GO, when its race distance has not grown
// by 10 m for 12 s (time in the pits does not count). A lost car is put on the centre line at
// its lap distance, facing along the lap, with the physics' spawn hold (sim::Vehicle::spawn); its
// lap timer moves with it and its driver is reset.

namespace gt3r::sim {
class Vehicle;
struct VehicleState;
}
namespace gt3r::track {
class Track;
class LapTimer;
}

namespace gt3r::ai {

class AiDriver;

class SafetyRespawn {
public:
    static constexpr double kStallSeconds = 12.0;
    static constexpr double kProgressMetres = 10.0;
    static constexpr int kOffMeshTicks = 30;

    // After the car's step, before its lap timer update: whether the car is lost. race_time =
    // seconds since GO (< 0 during the countdown).
    bool lost(const track::Track& t, const sim::VehicleState& st, double race_time);
    // After the lap timer update: the car's race distance (laps included).
    void progress(double race_distance, double race_time);
    // The car is in the pits: not lost for lack of progress.
    void hold(double race_time);
    // The car was put back on the track.
    void respawned(double race_time);
    // The race time the race distance last grew by 10 m (0 = GO).
    double last_progress() const { return last_progress_; }

private:
    int off_mesh_ticks_ = 0;
    double last_progress_ = 0, best_ = 0;
};

// Puts a lost car back on the track: the centre line at its timer's lap distance, facing along
// the lap, with the spawn hold; the timer teleported, the driver reset onto its starting line.
void respawn_car(sim::Vehicle& car, track::LapTimer& timer, AiDriver& driver, const track::Track& t);

} // namespace gt3r::ai
