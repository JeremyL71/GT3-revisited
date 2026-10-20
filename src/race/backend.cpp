#include "race/backend.h"

#include "base/log.h"
#include "formats/course/course_file.h"
#include "formats/model/wheel_spec.h"
#include "game/data/car_database.h"
#include "race/standins.h"

#include <algorithm>
#include <cmath>

// The real track runtime and vehicle simulation are used as soon as their
// implementations are part of the tree; until then the stand-ins run.
#if __has_include("track/track.cpp")
#define GT3R_RACE_REAL_TRACK 1
#include "sim/track_walls.h"
#include "track/track.h"
#else
#define GT3R_RACE_REAL_TRACK 0
#endif
#if __has_include("sim/vehicle.cpp") && GT3R_RACE_REAL_TRACK
#define GT3R_RACE_REAL_VEHICLE 1
#else
#define GT3R_RACE_REAL_VEHICLE 0
#endif

namespace gt3r::race {

namespace {

u64 part_id(const data::Car& car, data::TableId table) {
    for (size_t i = 0; i < data::kCarPartTables.size(); ++i)
        if (data::kCarPartTables[i] == int(table)) return car.parts[i];
    return 0;
}

#if GT3R_RACE_REAL_TRACK
// Physics only probes the road mesh; off it, give it the ground estimate so
// cars that leave the road keep rolling instead of falling.
class GroundQuery : public sim::TrackQuery {
public:
    explicit GroundQuery(const track::Track& t) : t_(t) {}
    sim::SurfaceHit probe(float x, float z, float y_max) const override {
        sim::SurfaceHit h = t_.probe_or_ground(x, z, y_max);
        h.hit = true; // probe_or_ground reports misses with hit = false but a usable height
        return h;
    }

private:
    const track::Track& t_;
};

class TrackAdapter : public RaceTrack {
public:
    explicit TrackAdapter(std::unique_ptr<track::Track> t) : t_(std::move(t)), ground_(*t_), world_(*t_), lap_(*t_) {}
    const char* backend_name() const override { return "track::Track"; }
    const sim::TrackQuery& query() const override { return ground_; }
    const sim::WorldQuery& world() const override { return world_; }
    float length() const override { return t_->length(); }
    int grid_count() const override { return std::max(1, t_->grid_count()); }
    TrackPose grid_pose(int slot) const override {
        track::Pose p = t_->grid_pose(slot);
        return {p.position, p.orientation};
    }
    TrackPose respawn_pose(const sim::Vec3& pos) const override {
        track::TrackPosition tp = t_->locate(pos, lap_.distance());
        track::Pose p = t_->respawn_pose(tp.distance);
        return {p.position, p.orientation};
    }
    sim::Vec3 point_ahead(const sim::Vec3& pos, float metres) const override {
        track::TrackPosition tp = t_->locate(pos, lap_.distance());
        return t_->point_at(tp.distance + metres);
    }
    sim::Vec3 point_at(float s, float lateral) const override { return t_->point_at(s, lateral); }
    float distance_of(const sim::Vec3& p) const override { return t_->locate(p).distance; }
    bool is_kerb(u32 surface) const override { return track::surface_type(surface) == track::Surface::Kerb; }
    bool is_offroad(const sim::SurfaceHit& hit) const override {
        track::Surface s = track::surface_type(hit.surface);
        return hit.segment < 0 || s == track::Surface::Grass || s == track::Surface::Gravel ||
               s == track::Surface::OffRoad;
    }
    void lap_reset(double t) override { lap_.reset(t); }
    void lap_teleport(const sim::Vec3& p) override { lap_.teleport(p); }
    bool lap_update(const sim::Vec3& p, double t) override { return lap_.update(p, t); }
    int laps_completed() const override { return lap_.laps_completed(); }
    bool lap_started() const override { return lap_.lap_started(); }
    double current_lap_time(double now) const override { return lap_.current_lap_time(now); }
    double last_lap_time() const override { return lap_.last_lap_time(); }
    double best_lap_time() const override { return lap_.best_lap_time(); }
    float lap_progress() const override { return lap_.lap_progress(); }
    bool wrong_way() const override { return lap_.wrong_way(); }
    void lap_set_lap1_from_go(bool from_go, bool before_line) override { lap_.set_lap1_from_go(from_go, before_line); }
    void lap_freeze() override { lap_.freeze(); }
    const track::Track& track() const { return *t_; }
    const track::Track* runtime() const override { return t_.get(); }
    double race_distance() const override { return lap_.race_distance(); }

private:
    std::unique_ptr<track::Track> t_;
    GroundQuery ground_;
    sim::TrackWorld world_;
    track::LapTimer lap_;
};
#endif

#if GT3R_RACE_REAL_VEHICLE
class VehicleAdapter : public RaceVehicle {
public:
    VehicleAdapter(std::unique_ptr<sim::Vehicle> v, const RaceTrack& t) : v_(std::move(v)), t_(t) {}
    const char* backend_name() const override { return "sim::Vehicle"; }
    // Grid starts and respawns use the spawn hold (spec §4.3): 2 s held above the ground.
    void reset(const TrackPose& p) override { v_->spawn(p.position, p.orientation); }
    void place(const TrackPose& p) override { v_->reset(p.position, p.orientation); }
    void set_dirt_course(bool dirt) override { v_->set_dirt_course(dirt); }
    void reset_moving(const TrackPose& p, float v) override { v_->spawn_moving(p.position, p.orientation, v); }
    void step(const sim::ControlInputs& in, double dt) override {
        sim::Vec3 before = v_->state().velocity;
        v_->step(in, t_.query(), dt, &t_.world());
        // Impacts: horizontal velocity change well beyond what tyres can do in one tick.
        sim::Vec3 dv = v_->state().velocity - before;
        dv.y = 0;
        float tyre_limit = 2.0f * 9.81f * float(dt);
        impact_ = std::max(0.0f, dv.length() - tyre_limit);
    }
    const sim::VehicleState& state() const override { return v_->state(); }
    const sim::VehicleSetup& setup() const override { return v_->setup(); }
    float last_impact() const override { return impact_; }
    float boost() const override { return v_->telemetry().boost; }
    const sim::Vehicle* sim_vehicle() const override { return v_.get(); }
    void begin_pit_stop(const sim::Quat& box) override { v_->begin_pit_stop(box); }
    void set_reverse_request(bool on) override { v_->set_reverse_request(on); }
    void set_driver_kind(int kind) override { v_->set_driver_kind(kind); }
    bool set_race_hold(bool on) override {
        v_->set_race_hold(on);
        return true;
    }
    float wall_impulse() const override { return v_->telemetry().wall_impulse; }

private:
    std::unique_ptr<sim::Vehicle> v_;
    const RaceTrack& t_;
    float impact_ = 0;
};
#endif

} // namespace

CarSetup car_setup_from_database(const data::CarDatabase& db, const std::string& label) {
    CarSetup s;
    s.label = label;
    const data::Car* car = db.car(label);
    if (!car) return s;
    s.name = car->name;
    s.model = db.model_of(*car);
    if (auto ch = db.chassis(part_id(*car, data::TableId::Chassis)); ch && ch->weight_kg) s.mass_kg = ch->weight_kg;
    if (auto en = db.engine(part_id(*car, data::TableId::Engine))) {
        for (size_t i = 0; i < en->rpm.size() && i < en->torque.size(); ++i) {
            s.curve_rpm.push_back(float(en->rpm[i]));
            s.curve_torque_nm.push_back(en->torque[i] * 9.80665f);
        }
        if (en->rpm_limit_x100) s.limit_rpm = en->rpm_limit_x100 * 100.0f;
        else if (!s.curve_rpm.empty()) s.limit_rpm = s.curve_rpm.back();
        s.redline_rpm = s.limit_rpm - 300;
    }
    if (auto gb = db.gearbox(part_id(*car, data::TableId::Gear))) {
        for (size_t i = 0; i < gb->gears && i < gb->ratios.size(); ++i)
            if (gb->ratios[i] > 0) s.ratios.push_back(gb->ratios[i]);
        if (gb->reverse > 0) s.reverse_ratio = gb->reverse;
        if (gb->final_drive > 0) s.final_drive = gb->final_drive;
    }
    if (auto dt = db.drivetrain(part_id(*car, data::TableId::Drivetrain))) s.drive_type = dt->drive_type;
    if (auto ws = wheel_spec_for_car(db, label)) s.wheels = *ws;
    s.body_mod = body_mod_for_car(db, label);
    data::CarDatabase::EngineSound es = db.engine_sound(*car);
    s.sound_id = es.id;
    s.sound_variant = es.variant;
    return s;
}

std::unique_ptr<RaceTrack> make_track(const Vfs& vfs, ByteSpan course_file, const std::string& label) {
    (void)vfs;
#if GT3R_RACE_REAL_TRACK
    try {
        return std::make_unique<TrackAdapter>(track::Track::from_course_file(course_file, label));
    } catch (const std::exception& e) {
        LOG_WARN("Race: track runtime failed (%s); using the stand-in", e.what());
    }
#endif
    course::CourseFile file = course::CourseFile::parse(course_file);
    ByteSpan road = file.section(course_file, course::kSlotRoad);
    if (road.empty()) throw FormatError("course has no road (GTRW) section");
    return make_road_track(course::Road::parse(road), label);
}

std::unique_ptr<RaceVehicle> make_vehicle(const CarSetup& car, const data::CarDatabase* db, const RaceTrack& track) {
#if GT3R_RACE_REAL_VEHICLE
    if (db && dynamic_cast<const TrackAdapter*>(&track)) {
        std::string err;
        std::unique_ptr<sim::Vehicle> v = car.spec ? sim::Vehicle::create(*car.spec) : sim::Vehicle::create(*db, car.label, &err);
        if (v) return std::make_unique<VehicleAdapter>(std::move(v), track);
        LOG_WARN("Race: sim::Vehicle::create(%s) failed: %s; using the stand-in", car.label.c_str(), err.c_str());
    }
#else
    (void)db;
#endif
    return make_kinematic_vehicle(car, track);
}

} // namespace gt3r::race
