#include "ai/ai_race.h"

#include "game/data/car_database.h"
#include "sim/car_spec.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace gt3r::ai {

AiRace::AiRace(const track::Track& track, const data::CarDatabase& db, const std::vector<Entry>& entries,
               RubberBand rubber, int reference)
    : track_(track), course_(std::make_shared<AiCourse>(track)), world_(track), rubber_(rubber), reference_(reference) {
    sim_ = std::make_unique<sim::World>(track_, &world_);
    int slots = std::max(track.grid_count(), 1);
    // Dirt courses (course table 34 +0x08 = 2): the physics' front-brake cut (spec §10.4).
    bool dirt = false;
    for (const data::Course& c : db.courses())
        if (c.label == track.label()) dirt = c.surface == 2;
    for (size_t i = 0; i < entries.size(); ++i) {
        Car c;
        c.label = entries[i].label;
        std::string err;
        std::unique_ptr<sim::Vehicle> v;
        if (auto spec = sim::build_car_spec(db, entries[i].label, &err)) {
            adapt_spec_for_ai(*spec);
            v = sim::Vehicle::create(*spec);
        }
        if (!v) {
            error_ += entries[i].label + ": " + err + "; ";
            continue;
        }
        int slot = int(cars_.size()) % slots;
        track::Pose pose = track.grid_pose(slot);
        // Extra cars beyond the grid start further back along the lap.
        if (int(cars_.size()) >= slots) pose = track.respawn_pose(track.length() - 10.0f * float(cars_.size()));
        // The grid slot marks the nose: move the car back by its front extent (as race::grid_back).
        if (int(cars_.size()) < slots) {
            float ext = v->setup().cog_front_extent; // the CoG moved back from the slot (§9.2)
            const sim::Quat& q = pose.orientation;
            sim::Vec3 u{q.x, q.y, q.z};
            auto rot = [&](const sim::Vec3& a) {
                sim::Vec3 t = u.cross(a) * 2.0f;
                return a + t * q.w + u.cross(t);
            };
            sim::Vec3 fwd = rot({0, 0, -1}), cog = rot(v->setup().cog_offset);
            pose.position = {pose.position.x - fwd.x * ext - cog.x, pose.position.y, pose.position.z - fwd.z * ext - cog.z};
        }
        v->set_cpu_driver(true);
        v->set_dirt_course(dirt);
        v->reset(pose.position, pose.orientation);
        c.vehicle = v.get();
        sim_->add(std::move(v));
        c.driver = std::make_unique<AiDriver>(course_, entries[i].params, int(cars_.size()));
        c.driver->reset(c.vehicle->state());
        c.timer = std::make_unique<track::LapTimer>(track);
        c.timer->set_lap1_from_go(true); // a race: lap 1 from GO
        c.timer->update(pose.position, 0);
        cars_.push_back(std::move(c));
    }
    snaps_.resize(cars_.size());
}

AiRace::~AiRace() = default;

std::vector<int> AiRace::ranking() const {
    std::vector<int> r(cars_.size());
    std::iota(r.begin(), r.end(), 0);
    std::stable_sort(r.begin(), r.end(), [&](int a, int b) {
        return cars_[size_t(a)].timer->race_distance() > cars_[size_t(b)].timer->race_distance();
    });
    return r;
}

void AiRace::step() {
    const double dt = 1.0 / sim::kDefaultTickHz;
    for (size_t i = 0; i < cars_.size(); ++i) {
        const auto& v = *cars_[i].vehicle;
        CarSnapshot& s = snaps_[i];
        s.position = v.state().position;
        s.orientation = v.state().orientation;
        s.velocity = v.state().velocity;
        s.half_length = v.setup().length * 0.5f;
        s.half_width = v.setup().width * 0.5f;
        s.front = v.setup().cog_front_extent, s.rear = v.setup().cog_rear_extent; // front / rear extents from the CoG
        {
                const sim::Quat& q = v.state().orientation;
                sim::Vec3 u{q.x, q.y, q.z}, c = v.setup().cog_offset;
                sim::Vec3 t = u.cross(c) * 2.0f;
                s.position = v.state().position + c + t * q.w + u.cross(t); // the CoG (the car's reference point in the physics spec)
            }
        s.time_scale = cars_[i].time_scale;
    }
    // Rubber band chain from the reference car (spec §14).
    std::vector<float> band;
    if (reference_ >= 0 && reference_ < int(cars_.size()) && rubber_.enabled()) {
        std::vector<double> dist;
        for (const Car& c : cars_) dist.push_back(c.timer->race_distance());
        band = rubber_.chain(dist, reference_);
    }
    for (size_t i = 0; i < cars_.size(); ++i) {
        Car& c = cars_[i];
        AiContext ctx{time_, snaps_, int(i)};
        ctx.rank = int(i);
        for (const auto& k : sim_->contacts())
            if ((k.a == int(i) && k.b >= 0) || (k.b == int(i) && k.a >= 0)) ctx.body_contact = true;
        sim_->set_inputs(int(i), c.driver->update(*c.vehicle, ctx, dt));
        sim_->vehicle(int(i)).set_reverse_request(c.driver->reverse_request());
        c.time_scale = i < band.size() ? band[i] : 1.0f;
        sim_->set_time_scale(int(i), c.time_scale);
    }
    sim_->step(dt);
    // Count contact events: a pair touching now that did not touch last tick.
    {
        std::vector<std::pair<int, int>> now;
        for (const auto& k : sim_->contacts()) now.push_back({std::min(k.a, k.b), std::max(k.a, k.b)});
        std::sort(now.begin(), now.end());
        now.erase(std::unique(now.begin(), now.end()), now.end());
        for (const auto& pr : now)
            if (!std::binary_search(touching_.begin(), touching_.end(), pr)) {
                ++contacts_;
                for (const auto& k : sim_->contacts())
                    if (std::min(k.a, k.b) == pr.first && std::max(k.a, k.b) == pr.second) {
                        const auto& q = sim_->vehicle(k.a).state().orientation;
                        sim::Vec3 u{q.x, q.y, q.z}, f{0, 0, -1};
                        sim::Vec3 t = u.cross(f) * 2.0f;
                        sim::Vec3 fwd = f + t * q.w + u.cross(t);
                        if (std::fabs(fwd.dot(k.normal)) > 0.7f) ++contacts_inline_;
                        break;
                    }
            }
        touching_ = std::move(now);
    }
    time_ += dt;
    // Overtakes: position changes after the first 10 s (the start shuffles the grid).
    {
        std::vector<int> r = ranking();
        if (time_ > 10 && r.size() == last_rank_.size())
            for (size_t k = 0; k < r.size(); ++k)
                if (r[k] != last_rank_[k]) {
                    ++overtakes_;
                    break;
                }
        last_rank_ = std::move(r);
    }
    for (size_t i = 0; i < cars_.size(); ++i) {
        Car& c = cars_[i];
        const sim::VehicleState& st = c.vehicle->state();
        // Safety respawn when a car left the road mesh (through a wall or a gap): the centre and
        // all four wheels off the mesh (or 5 m below it) for half a second. A single probe can
        // fall through a hairline crack between road triangles at exact float coordinates.
        auto off = [&](const sim::Vec3& p) {
            sim::SurfaceHit h = track_.probe(p.x, p.z, p.y + 3.0f);
            return !h.hit || p.y < h.height - 5.0f;
        };
        bool all_off = off(st.position);
        for (const sim::WheelState& w : st.wheels) all_off = all_off && off(w.contact);
        c.off_mesh_ticks = all_off ? c.off_mesh_ticks + 1 : 0;
        bool lost = c.off_mesh_ticks > 30;
        // Stuck for 12 s (no 10 m of progress): the race scene's respawn rule.
        if (time_ - c.last_progress_time > 12.0) lost = true;
        if (!std::isfinite(st.position.x)) lost = true;
        if (lost) {
            track::Pose p = track_.respawn_pose(c.timer->distance());
            sim_->vehicle(int(i)).reset(p.position, p.orientation);
            c.timer->teleport(p.position);
            c.driver->reset(c.vehicle->state());
            ++c.respawns;
            c.off_mesh_ticks = 0;
            c.last_progress_time = time_;
        }
        c.timer->update(c.vehicle->state().position, time_);
        double d = c.timer->race_distance();
        if (d > c.best_distance + 10) {
            c.best_distance = d;
            c.last_progress_time = time_;
        }
    }
}

} // namespace gt3r::ai
