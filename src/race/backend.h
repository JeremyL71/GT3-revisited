#pragma once

// The race scene talks to the vehicle simulation and the track runtime only
// through these two small interfaces, so it can run on the real
// implementations (sim::Vehicle from the physics workstream, track::Track
// from the track workstream) or on the simple stand-ins in this directory
// (KinematicVehicle, RoadTrack) while those are not available.
//
// make_track()/make_vehicle() pick the real implementation when it exists
// and fall back to the stand-ins otherwise (see backend.cpp).

#include "formats/model/car_model.h"
#include "sim/sim_types.h"
#include "sim/vehicle.h"

#include <memory>
#include <string>
#include <vector>

namespace gt3r {
class Vfs;
namespace data {
class CarDatabase;
}
namespace track {
class Track;
}
} // namespace gt3r

namespace gt3r::sim {
class Vehicle;
} // namespace gt3r::sim

namespace gt3r::race {

struct TrackPose {
    sim::Vec3 position;
    sim::Quat orientation;
};

// A split-point or line crossing of a car's lap timer (track::LapTimer::Crossing;
// presentation §D6).
struct LapCrossing {
    int point = 0;          // split point index; at the line, the points passed this lap
    bool line = false;      // a completed lap
    int lap = 0;            // laps completed before it
    long long ms = 0;       // race clock (ms)
    long long split_ms = 0; // since the lap's start (at the line: the lap time)
};

// Track runtime as seen by the race scene.
class RaceTrack {
public:
    virtual ~RaceTrack() = default;
    virtual const char* backend_name() const = 0;
    virtual const sim::TrackQuery& query() const = 0;
    virtual float length() const = 0;

    virtual int grid_count() const = 0;
    virtual TrackPose grid_pose(int slot) const = 0;
    // Pose on the centre line nearest to `p` (respawn after leaving the track).
    virtual TrackPose respawn_pose(const sim::Vec3& p) const = 0;
    // Centre-line point `metres` further along the lap than the point nearest
    // to `p` (auto-drive / demo steering).
    virtual sim::Vec3 point_ahead(const sim::Vec3& p, float metres) const = 0;
    // Centre-line point at track distance s (wrapped), `lateral` metres to the left.
    virtual sim::Vec3 point_at(float s, float lateral = 0) const = 0;
    // Track distance of the centre-line point nearest to p.
    virtual float distance_of(const sim::Vec3& p) const = 0;
    // The real track runtime when this is backed by it (opponents need it), else nullptr.
    virtual const track::Track* runtime() const { return nullptr; }
    // Player's race distance (laps * length + distance), for positions.
    virtual double race_distance() const = 0;

    // Walls and surface grip for the vehicle simulation.
    virtual const sim::WorldQuery& world() const = 0;
    // Surface classes for effects (rumble, sounds).
    virtual bool is_kerb(u32 surface) const = 0;
    virtual bool is_offroad(const sim::SurfaceHit& hit) const = 0;

    // Lap timing for the player's car.
    virtual void lap_reset(double time) = 0;
    virtual void lap_teleport(const sim::Vec3& p) = 0;
    virtual bool lap_update(const sim::Vec3& p, double time) = 0; // true when a lap was completed
    virtual int laps_completed() const = 0;
    virtual bool lap_started() const = 0;
    virtual double current_lap_time(double now) const = 0;
    virtual double last_lap_time() const = 0; // < 0 when none
    virtual double best_lap_time() const = 0; // < 0 when none
    virtual float lap_progress() const = 0;   // 0..1
    virtual bool wrong_way() const = 0;
    // track::LapTimer::set_lap1_from_go (races: lap 1 from GO) and freeze (after the finish).
    virtual void lap_set_lap1_from_go(bool from_go, bool before_line) { (void)from_go, (void)before_line; }
    virtual void lap_freeze() {}
    // The player's split-point / line crossings of the last lap_update().
    virtual std::vector<LapCrossing> lap_crossings() const { return {}; }
    // The course's split points (track distances), as the lap timer uses them.
    virtual std::vector<float> split_points() const { return {}; }
};

// What the race scene knows about the chosen car; filled from the database.
struct CarSetup {
    std::string label;      // database car label, e.g. "r34_gtr_v_99"
    std::string name;       // display name
    std::string model;      // car model id, e.g. "ni0137"
    float mass_kg = 1300;
    std::vector<float> curve_rpm, curve_torque_nm; // engine torque curve
    float idle_rpm = 900, redline_rpm = 7000, limit_rpm = 7500;
    std::vector<float> ratios; // forward gears
    float reverse_ratio = 3.5f, final_drive = 4.0f;
    int drive_type = 0;        // database DriveType (FR, FF, ...)
    WheelSpec wheels;          // placement and tyre size
    CarBodyMod body_mod;       // mod cars: stretched shell or a body of their own (paint 0)
    u16 sound_id = 0;          // engine sound bank
    u8 sound_variant = 0;
    // A fully built spec (e.g. a GT-mode garage car with parts and settings);
    // when set the vehicle is created from it instead of from `label`.
    std::shared_ptr<const sim::CarSpec> spec;
};

// Fills a CarSetup from the database (defaults where something is missing).
CarSetup car_setup_from_database(const data::CarDatabase& db, const std::string& car_label);

class RaceVehicle {
public:
    virtual ~RaceVehicle() = default;
    virtual const char* backend_name() const = 0;
    // Respawns (the player's car, replayed respawn events). Real vehicles use
    // the spawn hold: held 0.3 m up for 2 s, then dropped (sim::Vehicle::spawn).
    virtual void reset(const TrackPose& pose) = 0;
    // Grid start (physics §9): placed at rest without the spawn hold.
    virtual void place(const TrackPose& pose) { reset(pose); }
    // Dirt course (course table 34 +0x08 = 2): the physics §10.4 front-brake cut with steering.
    // In a field this applies to every car.
    virtual void set_dirt_course(bool dirt) { (void)dirt; }
    // Rolling / flying starts: placed moving straight ahead at speed (m/s), no spawn hold.
    virtual void reset_moving(const TrackPose& pose, float speed_mps) { reset(pose); (void)speed_mps; }
    virtual void step(const sim::ControlInputs& in, double dt) = 0;
    virtual const sim::VehicleState& state() const = 0;
    // Wheel placement, gear count and rpm range (tach) in the car frame.
    virtual const sim::VehicleSetup& setup() const = 0;
    // Velocity change from wall hits during the last step (m/s), for rumble.
    virtual float last_impact() const = 0;
    // Turbo boost (sim::VehicleTelemetry::boost; 0 for NA cars and the stand-in).
    virtual float boost() const { return 0; }
    // The simulation vehicle behind this one (spec, telemetry), or nullptr (stand-ins).
    virtual const sim::Vehicle* sim_vehicle() const { return nullptr; }
    // Player reverse (GT3's "back" action): requests reverse gear (sim::Vehicle::set_reverse_request).
    virtual void set_reverse_request(bool on) { (void)on; }
    // Scripted driving (sim::Vehicle::set_driver_kind; 0 = the driver): the pit lane's
    // auto drive sets its state (2 lane, 3 box line, 4 leaving): aids off like a CPU car.
    virtual void set_driver_kind(int kind) { (void)kind; }
    // The AI aid block the physics reads while the computer drives this car (sim::Vehicle::
    // set_scripted_aids; the player's AI parameter set, ai §9.1).
    virtual void set_scripted_aids(const sim::ScriptedAids& a) { (void)a; }
    // The start-countdown hold (sim::Vehicle::set_race_hold, physics §9.3): the car is held in place, the
    // brakes stay off and the engine can rev. Returns false when the backend has no such hold (the
    // caller then holds the car with the brakes).
    virtual bool set_race_hold(bool on) { (void)on; return false; }
    // The pit-box stop (sim::Vehicle::begin_pit_stop): 5 s, tyres changed at 1 s.
    virtual void begin_pit_stop(const sim::Quat& box) { (void)box; }
    // Engine condition (sim::Vehicle::set_engine_condition, physics §10.3): what the run starts
    // with and, read back, what it hands on. The stand-in has none (a fresh engine).
    virtual void set_engine_condition(const sim::EngineCondition& c) { (void)c; }
    virtual sim::EngineCondition engine_condition() const { return {}; }
    // Wall impulse of the last step (N*s; sim::VehicleTelemetry::wall_impulse), for sounds.
    virtual float wall_impulse() const { return 0; }
    // Strongest car-to-car contact impulse of the last step involving this car (N*s).
    virtual float car_impulse() const { return 0; }
};

std::unique_ptr<RaceTrack> make_track(const Vfs& vfs, ByteSpan course_file, const std::string& label);
std::unique_ptr<RaceVehicle> make_vehicle(const CarSetup& car, const data::CarDatabase* db, const RaceTrack& track);

} // namespace gt3r::race
