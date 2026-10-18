#pragma once

// Several cars stepped together in the order the spec gives
// (§1.2: each phase runs for all cars before the
// next): slipstream factors for every car (§5.2), then each car's tick
// (with its own walls, §5.3), then swept car-to-car contact (§5.3: up to 2
// iterations, impulse with restitution 0.5, no friction, overlap split half
// and half), only when
// enabled (the game enables it in modes with opponents).
//
//   sim::World world(*track, &walls);              // walls: sim::TrackWorld, optional
//   int me = world.add(sim::Vehicle::create(db, "r34_gtr_v_99"));
//   world.vehicle(me).reset(pose.position, pose.orientation);
//   every tick: world.set_inputs(me, pad); world.set_inputs(ai, ai_inputs); world.step(1.0 / 60);
//   world.contacts() -> car-car contacts of the last step (sound, HUD, AI)
//
// Deterministic: cars are processed in index order. A single car stepped
// through World behaves exactly like Vehicle::step (plus slipstream = 0).

#include "sim/vehicle.h"

#include <memory>
#include <vector>

namespace gt3r::sim {

class World {
public:
    explicit World(const TrackQuery& track, const WorldQuery* world = nullptr) : track_(track), world_(world) {}

    // Takes ownership; returns the car index (0, 1, ...).
    int add(std::unique_ptr<Vehicle> v);
    int size() const { return int(cars_.size()); }
    Vehicle& vehicle(int i) { return *cars_[size_t(i)].vehicle; }
    const Vehicle& vehicle(int i) const { return *cars_[size_t(i)].vehicle; }

    // Inputs used by the next step() (persist until changed; shift requests
    // are consumed by the step).
    void set_inputs(int i, const ControlInputs& in) { cars_[size_t(i)].inputs = in; }
    // A car can be excluded (ghost / retired): it is not stepped and not collided.
    void set_active(int i, bool active) { cars_[size_t(i)].active = active; }
    // Online play: a remote player's car. Not stepped (its pose is set from the network
    // before each step, as at the end of the coming tick: Vehicle::set_remote_pose), but
    // it collides: a contact is solved with both bodies as usual and only the other car
    // keeps the result (the remote peer solves the same contact for its own car).
    void set_kinematic(int i, bool on) { cars_[size_t(i)].kinematic = on; }
    bool kinematic(int i) const { return cars_[size_t(i)].kinematic; }

    // Per-car time scale (spec §1.1 dt_car = scale / 60; GT3's rubber band
    // writes it): 1 = normal.
    void set_time_scale(int i, float scale) { cars_[size_t(i)].time_scale = scale; }
    float time_scale(int i) const { return cars_[size_t(i)].time_scale; }

    void set_car_collisions(bool on) { car_collisions_ = on; }
    void set_slipstream(bool on) { slipstream_ = on; }

    // Steps all cars by dt (ceil(dt * 60) ticks of 1/60 s).
    void step(double dt);

    struct Contact {
        int a = -1, b = -1;   // car indices
        Vec3 point;           // world contact point
        Vec3 normal;          // horizontal unit normal, from a towards b
        float impulse = 0;    // N*s (|j|; spec: |j| / 1e5 is the HUD/sound strength)
    };
    const std::vector<Contact>& contacts() const { return contacts_; }

private:
    struct Car {
        std::unique_ptr<Vehicle> vehicle;
        ControlInputs inputs;
        bool active = true;
        bool kinematic = false;
        float time_scale = 1;
    };
    void tick(double h, bool first);

    const TrackQuery& track_;
    const WorldQuery* world_;
    std::vector<Car> cars_;
    bool car_collisions_ = true;
    bool slipstream_ = true;
    std::vector<Contact> contacts_;
};

} // namespace gt3r::sim
