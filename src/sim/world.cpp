#include "sim/world.h"

#include "sim/models.h"

#include <algorithm>
#include <cmath>

namespace gt3r::sim {

int World::add(std::unique_ptr<Vehicle> v) {
    Car c;
    c.vehicle = std::move(v);
    cars_.push_back(std::move(c));
    return int(cars_.size()) - 1;
}

void World::step(double dt) {
    if (!(dt > 0) || !std::isfinite(dt)) return;
    contacts_.clear();
    int ticks = std::max(1, int(std::ceil(dt * Vehicle::kTickHz - 1e-6)));
    for (int t = 0; t < ticks; ++t) tick(dt / ticks, t == 0);
    for (Car& c : cars_) c.inputs.shift_up = c.inputs.shift_down = false;
}

void World::tick(double h, bool first) {
    const int n = int(cars_.size());
    // Phase: slipstream factors for every car from every leader [spec §5.2].
    if (slipstream_) {
        for (int f = 0; f < n; ++f) {
            if (!cars_[size_t(f)].active || cars_[size_t(f)].kinematic) continue;
            float sum = 0;
            const VehicleState& fs = cars_[size_t(f)].vehicle->state();
            for (int l = 0; l < n; ++l) {
                if (l == f || !cars_[size_t(l)].active) continue;
                const Vehicle& lv = *cars_[size_t(l)].vehicle;
                collide::Body lb = lv.collision_body();
                // Follower position in the leader's frame (x forward, y lateral).
                collide::V2 d{fs.position.x - lb.pos.x, fs.position.z - lb.pos.z};
                collide::V2 fwd = collide::to_world_dir(lb.heading, {0, 1});
                collide::V2 lat = collide::to_world_dir(lb.heading, {1, 0});
                float v_l = float(lb.vel.dot(fwd));
                sum += model::slipstream_factor(float(d.dot(fwd)), float(d.dot(lat)), v_l,
                                                float(lv.spec().width * 0.5));
            }
            cars_[size_t(f)].vehicle->set_slipstream(std::min(sum, 1.0f));
        }
    }
    // Start poses for the swept car-car test.
    std::vector<collide::Body> start(cars_.size());
    for (size_t i = 0; i < cars_.size(); ++i) {
        start[i] = cars_[i].vehicle->collision_body();
        if (cars_[i].kinematic) { // its pose is already the end of this tick: back along its motion
            collide::Body& b = start[i];
            b.pos = {b.pos.x - b.vel.x * h, b.pos.z - b.vel.z * h};
            b.heading -= b.yaw_rate * h;
        }
    }
    // Phase: each car's own tick (tyres, engine, suspension, walls).
    for (Car& c : cars_) {
        if (!c.active || c.kinematic) continue;
        ControlInputs in = c.inputs;
        if (!first) in.shift_up = in.shift_down = false;
        c.vehicle->step_tick(in, track_, h * double(c.time_scale), world_);
    }
    // Phase: car-car contact [spec §5.3].
    if (!car_collisions_) return;
    // The steering input as a fraction of full lock (-1 left .. +1 right), for the stuck assist.
    auto steer_fraction = [](const Car& c) {
        const ControlInputs& in = c.inputs;
        float lock = c.vehicle->spec().steering.max_angle;
        float f = std::isfinite(in.steer_angle) ? (lock > 0 ? in.steer_angle / lock : 0.0f) : in.steer;
        return std::clamp(f, -1.0f, 1.0f);
    };
    for (int a = 0; a < n; ++a) {
        if (!cars_[size_t(a)].active) continue;
        for (int b = a + 1; b < n; ++b) {
            if (!cars_[size_t(b)].active) continue;
            const bool ka = cars_[size_t(a)].kinematic, kb = cars_[size_t(b)].kinematic;
            if (ka && kb) continue; // two remote cars: their own peers solve it
            Vehicle& va = *cars_[size_t(a)].vehicle;
            Vehicle& vb = *cars_[size_t(b)].vehicle;
            // Same height band only (bridges): half-heights + 5 m [spec §5.1 broad phase].
            float dy = std::fabs(va.state().position.y - vb.state().position.y);
            if (dy > 0.5f * (va.spec().height + vb.spec().height) + 5.0f) continue;
            collide::Body ba = va.collision_body(), bb = vb.collision_body();
            collide::V2 p, nrm;
            float steer[2] = {steer_fraction(cars_[size_t(a)]), steer_fraction(cars_[size_t(b)])};
            double j = collide::collide_pair_swept(ba, start[size_t(a)].pos, start[size_t(a)].heading, bb,
                                                   start[size_t(b)].pos, start[size_t(b)].heading, h, 2, &p, &nrm, steer);
            if (j <= 0) continue;
            if (!ka) va.apply_collision_body(ba);
            if (!kb) vb.apply_collision_body(bb);
            Contact c;
            c.a = a;
            c.b = b;
            c.point = {float(p.x), va.state().position.y + 0.5f, float(p.z)};
            c.normal = {float(nrm.x), 0, float(nrm.z)};
            c.impulse = float(j);
            contacts_.push_back(c);
        }
    }
}

} // namespace gt3r::sim
