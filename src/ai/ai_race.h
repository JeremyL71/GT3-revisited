#pragma once

// A minimal multi-car race loop driving every car with an AiDriver: used by
// the AI test screen and the tests, and a reference for how the race scene
// wires opponents (snapshots, rubber band, lap timing). Cars are stepped
// together in a sim::World (car-to-car contact, slipstream, per-car time
// scale).

#include "ai/ai_driver.h"
#include "sim/track_walls.h"
#include "sim/vehicle.h"
#include "sim/world.h"
#include "track/track.h"

#include <memory>
#include <string>
#include <vector>

namespace gt3r::data {
class CarDatabase;
}

namespace gt3r::ai {

class AiRace {
public:
    struct Entry {
        std::string label;   // car or car-variant label
        DriverParams params; // driver parameters (event already applied)
    };
    // Cars start on grid slots 0..n-1. `reference` is the car the rubber band
    // measures gaps to (-1: none).
    AiRace(const track::Track& track, const data::CarDatabase& db, const std::vector<Entry>& entries,
           RubberBand rubber = {}, int reference = -1);
    ~AiRace();

    // One 1/60 s tick for every car.
    void step();
    double time() const { return time_; }

    struct Car {
        std::string label;
        const sim::Vehicle* vehicle = nullptr; // owned by the world
        std::unique_ptr<AiDriver> driver;
        std::unique_ptr<track::LapTimer> timer;
        float time_scale = 1;
        int respawns = 0;     // safety respawns: left the road mesh, or no progress for 12 s
        double last_progress_time = 0; // last time the race distance grew by 10 m
        double best_distance = 0;
        int off_mesh_ticks = 0; // consecutive steps with the car and all its wheels off the road mesh
    };
    int count() const { return int(cars_.size()); }
    const Car& car(int i) const { return cars_[size_t(i)]; }
    const sim::World& world() const { return *sim_; }
    int contacts_inline() const { return contacts_inline_; } // of those, nose-to-tail
    int overtakes() const { return overtakes_; } // ticks on which the running order changed (after 10 s)
    int contacts() const { return contacts_; } // car-car contact events so far (pair starts touching)
    // Car indices ordered by race distance (leader first).
    std::vector<int> ranking() const;
    const std::string& error() const { return error_; }

private:
    const track::Track& track_;
    std::shared_ptr<AiCourse> course_;
    sim::TrackWorld world_;
    RubberBand rubber_;
    int reference_;
    std::unique_ptr<sim::World> sim_;
    std::vector<Car> cars_;
    int contacts_ = 0, contacts_inline_ = 0, overtakes_ = 0;
    std::vector<int> last_rank_;
    std::vector<std::pair<int, int>> touching_;
    std::vector<CarSnapshot> snaps_;
    double time_ = 0;
    std::string error_;
};

} // namespace gt3r::ai
