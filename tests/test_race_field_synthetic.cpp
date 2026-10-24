// race::Field (and ai::AiRace) without game data: the stadium course of course_fixtures.h, a
// RaceTrack adapter over it, and a data::CarDatabase loaded from an in-memory archive whose
// only cars are INVENTED records (the container layouts are docs/formats/database.md's;
// every value in the records is a round number chosen here).
//
// Field builds each opponent with sim::build_car_spec(db, label), so it needs a database; a
// tiny one with a single car is enough. The car's engine is too weak to move it, which makes
// it the "stuck" car both race loops' safety respawn is for.

#include "test_framework.h"
#include "course_fixtures.h"
#include "sim_fixtures.h"

#include "ai/ai_driver.h"
#include "ai/ai_race.h"
#include "formats/db/gt_db.h"
#include "game/data/car_database.h"
#include "race/field.h"
#include "sim/track_walls.h"
#include "track/track.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <optional>
#include <string>

using namespace gt3r;
namespace st = gt3r::test::stadium;

namespace {

constexpr double kDt = 1.0 / sim::kDefaultTickHz;

void put16(Bytes& b, size_t o, u16 v) { b[o] = u8(v), b[o + 1] = u8(v >> 8); }
void put32(Bytes& b, size_t o, u32 v) { put16(b, o, u16(v)), put16(b, o + 2, u16(v >> 16)); }
void put64(Bytes& b, size_t o, u64 v) { put32(b, o, u32(v)), put32(b, o + 4, u32(v >> 32)); }

// GTAR of 36 empty GTDT tables (record sizes large enough for every record decoded here), and
// empty STDB string tables.
Bytes empty_paramdb() {
    const u32 n = u32(data::TableId::Count);
    Bytes out(16 + 4 * size_t(n + 1));
    std::memcpy(out.data(), "GTAR", 4);
    put32(out, 4, n);
    put32(out, 8, u32(out.size())); // data offset
    put32(out, 12, 7);
    for (u32 i = 0; i <= n; ++i) put32(out, 16 + 4 * size_t(i), 16 * i);
    for (u32 i = 0; i < n; ++i) {
        Bytes t(16);
        std::memcpy(t.data(), "GTDT", 4);
        put16(t, 6, u16(i));
        put16(t, 10, i == u32(data::TableId::Car) ? 0x110 : 0x200); // record size; no records
        put32(t, 12, 16);
        out.insert(out.end(), t.begin(), t.end());
    }
    return out;
}

Bytes empty_stdb() {
    Bytes b(16);
    std::memcpy(b.data(), "STDB", 4);
    put32(b, 12, 16);
    return b;
}

// A database holding one car, `label`: chassis, engine and suspension records (everything else
// takes build_car_spec's defaults). With the suspension block it is a spec-path car, as every
// disc car is. `torque` is the engine's torque word at every axis point.
std::unique_ptr<data::CarDatabase> synthetic_database(const std::string& label, u16 torque) {
    auto db = std::make_unique<data::CarDatabase>();
    const Bytes paramdb = empty_paramdb(), stdb = empty_stdb();
    bool ok = db->load([&](const std::string& path) -> std::optional<Bytes> {
        if (path == "database/paramdb_us.db") return paramdb;
        if (path == "database/paramstr_us.db" || path == "database/paramunistr_us.db") return stdb;
        return std::nullopt;
    });
    if (!ok) return nullptr;
    const std::string ch = "ch_" + label, en = "en_" + label, su = "su_" + label;
    for (const std::string& l : {label, ch, en, su}) db->add_label(l);
    Bytes c(0x40);
    put64(c, 0, db::id_hash(ch));
    put64(c, 8, db::id_hash(label));
    c[0x10] = 55;            // front weight %
    put16(c, 0x14, 4400);    // length mm
    put16(c, 0x16, 1300);    // height mm
    put16(c, 0x18, 2600);    // wheelbase mm
    put16(c, 0x1A, 1180);    // weight kg (+120 kg in the spec)
    c[0x1C] = 80;            // yaw inertia / 100
    c[0x1D] = 200;
    db->put_record(data::TableId::Chassis, c);
    Bytes e(0x60);
    put64(e, 0, db::id_hash(en));
    put64(e, 8, db::id_hash(label));
    const u8 axis[4] = {10, 30, 50, 70}; // x100 rpm
    for (int i = 0; i < 4; ++i) {
        put16(e, size_t(0x18 + 2 * i), torque);
        e[size_t(0x47 + i)] = axis[i];
    }
    e[0x44] = 80; // idle x10 rpm
    e[0x45] = 75; // rev limit x100
    e[0x46] = 70; // upshift x100
    e[0x57] = 4;  // axis points
    db->put_record(data::TableId::Engine, e);
    // Suspension (stock settings and the block bytes build_car_spec converts; one-step setting
    // ranges, so each setting reads its minimum byte): springs 40 / 45 x 0.1 kgf/mm at a 100 %
    // motion ratio, ride height 120 mm, dampers through (0.1 m/s, 12 x 49 N) and (0.5 m/s, 49 x 49 N)
    // in bump, (0.1, 16 x 49) and (0.5, 61 x 49) in rebound, anti-roll bytes 60 / 40, progressive
    // byte 10, unsprung 40 x 0.25 kg, bump damping 20 x 98, knee gap 30 mm, bump room 80 mm,
    // droop 100 mm.
    Bytes s(0x80);
    put64(s, 0, db::id_hash(su));
    put64(s, 8, db::id_hash(label));
    s[0x1D] = s[0x1E] = 80;   // bump room
    s[0x1F] = s[0x20] = 100;  // droop
    s[0x23] = 40, s[0x26] = 45; // springs
    s[0x27] = s[0x28] = 100;  // motion ratios
    s[0x29] = s[0x2A] = 10;   // progressive gain
    for (size_t o : {0x2B, 0x32, 0x39, 0x40, 0x47, 0x4B}) s[o] = 1; // damper / anti-roll setting steps
    s[0x2C] = s[0x3A] = 12, s[0x2F] = s[0x3D] = 49; // bump forces (x 49 N)
    s[0x33] = s[0x41] = 16, s[0x36] = s[0x44] = 61; // rebound forces
    s[0x48] = 60, s[0x4C] = 40; // anti-roll
    put16(s, 0x54, 120), put16(s, 0x5A, 120); // ride heights (mm)
    s[0x5C] = s[0x5D] = 40;   // unsprung
    for (size_t o : {0x70, 0x72, 0x74, 0x76}) s[o] = 10; // damper speeds (x 0.01 m/s)
    for (size_t o : {0x71, 0x73, 0x75, 0x77}) s[o] = 50;
    s[0x78] = s[0x79] = 30;   // knee gap
    s[0x7A] = s[0x7B] = 20;   // bump damping
    db->put_record(data::TableId::Suspension, s);
    Bytes car(0x110);
    put64(car, 0, db::id_hash(label));
    put64(car, 8 + 8 * 3, db::id_hash(ch));  // part slot 3: chassis
    put64(car, 8 + 8 * 6, db::id_hash(en));  // part slot 6: engine
    put64(car, 8 + 8 * 19, db::id_hash(su)); // part slot 19: suspension
    db->put_record(data::TableId::Car, car);
    db->rebuild();
    return db;
}

// race::RaceTrack over a track::Track, as the app's adapter (src/race/backend.cpp) does.
class StadiumRaceTrack : public race::RaceTrack {
public:
    StadiumRaceTrack() : t_(st::make_track()), ground_(*t_), walls_(*t_), lap_(*t_) {}
    const char* backend_name() const override { return "stadium"; }
    const sim::TrackQuery& query() const override { return ground_; }
    const sim::WorldQuery& world() const override { return walls_; }
    float length() const override { return t_->length(); }
    int grid_count() const override { return t_->grid_count(); }
    race::TrackPose grid_pose(int slot) const override {
        track::Pose p = t_->grid_pose(slot);
        return {p.position, p.orientation};
    }
    race::TrackPose respawn_pose(const sim::Vec3& p) const override {
        track::Pose r = t_->respawn_pose(t_->locate(p).distance);
        return {r.position, r.orientation};
    }
    sim::Vec3 point_ahead(const sim::Vec3& p, float m) const override { return t_->point_at(t_->locate(p).distance + m); }
    sim::Vec3 point_at(float s, float lateral) const override { return t_->point_at(s, lateral); }
    float distance_of(const sim::Vec3& p) const override { return t_->locate(p).distance; }
    const track::Track* runtime() const override { return t_.get(); }
    double race_distance() const override { return lap_.race_distance(); }
    bool is_kerb(u32) const override { return false; }
    bool is_offroad(const sim::SurfaceHit& h) const override { return h.segment < 0; }
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
    const track::Track& track() const { return *t_; }

private:
    struct Ground : sim::TrackQuery { // never misses, as the app's GroundQuery
        explicit Ground(const track::Track& t) : t(t) {}
        sim::SurfaceHit probe(float x, float z, float y_max) const override {
            sim::SurfaceHit h = t.probe_or_ground(x, z, y_max);
            h.hit = true;
            return h;
        }
        const track::Track& t;
    };
    std::unique_ptr<track::Track> t_;
    Ground ground_;
    sim::TrackWorld walls_;
    track::LapTimer lap_;
};

struct FieldRun {
    double first_respawn = -1, second_respawn = -1; // race time
    sim::Vec3 grid, respawned;
    float lateral = 0, height = 0, moved_before = 0; // lateral and height half a second into the hold
    bool player_held = true;
};

FieldRun run_field(const data::CarDatabase& db, StadiumRaceTrack& rt, bool cpu_respawn, double until) {
    race::FieldConfig cfg;
    cfg.player_spec = std::make_shared<sim::CarSpec>(test::synthetic_spec_car());
    cfg.opponents = {"synth_stall"};
    cfg.countdown = 3;
    cfg.cpu_respawn = cpu_respawn;
    std::string err;
    std::unique_ptr<race::RaceField> f = race::RaceField::create(rt, db, cfg, &err);
    FieldRun r;
    CHECK(f != nullptr && err.empty());
    if (!f) {
        std::printf("  field: %s\n", err.c_str());
        return r;
    }
    CHECK(f->opponents() == 1 && f->labels().size() == 1);
    race::TrackPose p = f->player_grid();
    f->player().place({race::grid_back(p.position, p.orientation, f->player().setup()), p.orientation});
    const sim::Vehicle* opp = f->opponent_vehicle(0);
    r.grid = opp->state().position;
    sim::ControlInputs hold;
    hold.brake = 1;
    sim::Vec3 player0 = f->player().state().position;
    while (f->race_time() < until) {
        f->player().step(hold, kDt);
        const sim::VehicleState& o = f->opponent_state(0);
        if (r.first_respawn >= 0 && std::fabs(f->race_time() - (r.first_respawn + 0.5)) < kDt / 2) {
            track::TrackPosition tp = rt.track().locate(o.position);
            r.lateral = tp.lateral;
            r.height = o.position.y - tp.centre.y;
        }
        if (r.first_respawn < 0) {
            if (opp->spawn_hold_active()) {
                r.first_respawn = f->race_time();
                r.respawned = o.position;
            } else {
                sim::Vec3 d = o.position - r.grid;
                r.moved_before = std::max(r.moved_before, std::hypot(d.x, d.z));
            }
        } else if (r.second_respawn < 0 && opp->spawn_hold_active() && f->race_time() > r.first_respawn + 2.5) {
            r.second_respawn = f->race_time();
        }
    }
    sim::Vec3 d = f->player().state().position - player0;
    r.player_held = std::hypot(d.x, d.z) < 0.05f; // the player is never respawned
    return r;
}

} // namespace

TEST(race_field_synthetic_database) {
    // The in-memory database decodes the invented car and build_car_spec turns it into a spec.
    auto db = synthetic_database("synth_stall", 10);
    CHECK(db != nullptr);
    if (!db) return;
    CHECK(db->cars().size() == 1 && db->car("synth_stall") != nullptr);
    std::string err;
    auto spec = sim::build_car_spec(*db, "synth_stall", &err);
    CHECK(spec.has_value());
    if (!spec) {
        std::printf("  %s\n", err.c_str());
        return;
    }
    std::printf("  synthetic database car: %.0f kg, wheelbase %.2f m, peak %.1f N*m, idle %.0f, limit %.0f rpm\n", spec->mass,
                spec->wheelbase, spec->peak_torque(), spec->engine.idle_rpm, spec->engine.rev_limit_rpm);
    CHECK(spec->mass == 1300 && std::fabs(spec->wheelbase - 2.6f) < 1e-6f && spec->engine.rev_limit_rpm == 7500);
    CHECK(spec->peak_torque() < 2); // too weak to overcome its own friction: a car that cannot move
    CHECK(spec->axle[0].susp.valid && spec->axle[1].susp.valid && std::fabs(spec->axle[0].susp.k - 39200) < 1);
    CHECK(spec->axle[0].susp.bump.g > 0 && spec->axle[0].susp.rebound.g > 0);
    CHECK(std::fabs(spec->axle[1].susp.ride_height - 0.12f) < 1e-6f);
}

TEST(race_field_stuck_respawn) {
    // race::Field's safety respawn ([race] respawn, not in the original): a CPU car that has not
    // made 10 m of progress for 12 s of RACE time (the clock starts at GO, not during the
    // countdown) is put back on the centre line at its lap distance with the 120-tick spawn hold
    // (Vehicle::spawn: 0.3 m up), and again 12 s later if it is still stuck. The player's car is
    // never respawned. With cpu_respawn off nothing happens.
    auto db = synthetic_database("synth_stall", 10);
    if (!db) {
        CHECK(false);
        return;
    }
    StadiumRaceTrack rt;
    FieldRun on = run_field(*db, rt, true, 26);
    std::printf("  field: stuck car respawned at race time %.3f s and %.3f s, %.2f m from its slot, lateral %.3f m, "
                "%.2f m above the road\n",
                on.first_respawn, on.second_respawn, std::hypot(on.respawned.x - on.grid.x, on.respawned.z - on.grid.z),
                on.lateral, on.height);
    CHECK(on.moved_before < 0.05f && on.player_held);
    CHECK(std::fabs(on.first_respawn - 12.0) < 1.5 * kDt); // (the race clock is a running sum of 1/60)
    CHECK(std::fabs(on.second_respawn - on.first_respawn - 12.0) < 1.5 * kDt);
    CHECK(std::fabs(on.lateral) < 0.05f && std::fabs(on.height - 0.3f) < 0.02f); // centre line, spawn height
    FieldRun off = run_field(*db, rt, false, 26);
    CHECK(off.first_respawn < 0 && off.moved_before < 0.05f);
}

TEST(ai_race_stuck_respawn) {
    // ai::AiRace runs the same safety respawn as race::Field (ai::SafetyRespawn, ai §12): its race
    // clock starts at 0 (no countdown), so the stuck car is put back at 12 s, on the centre line
    // with the spawn hold (0.3 m up); set_respawn(false) turns it off as [race] respawn = off does.
    auto db = synthetic_database("synth_stall", 10);
    if (!db) {
        CHECK(false);
        return;
    }
    auto t = st::make_track();
    for (bool on : {true, false}) {
        ai::AiRace race(*t, *db, {{"synth_stall", ai::course_params(t->driver_sets(), 0, 0)}});
        race.set_respawn(on);
        CHECK(race.count() == 1 && race.error().empty());
        if (race.count() != 1) return;
        double respawn = -1;
        bool held = false;
        float lateral = 1, height = 0;
        while (race.time() < 13) {
            race.step();
            if (respawn < 0 && race.car(0).respawns > 0) respawn = race.time(), held = race.car(0).vehicle->spawn_hold_active();
            if (respawn >= 0 && std::fabs(race.time() - (respawn + 0.5)) < kDt / 2) { // half a second into the hold
                track::TrackPosition tp = t->locate(race.car(0).vehicle->state().position);
                lateral = tp.lateral;
                height = race.car(0).vehicle->state().position.y - tp.centre.y;
            }
        }
        std::printf("  AiRace (respawn %s): stuck car respawned at %.3f s (spawn hold %d, lateral %.3f m, %.2f m up), "
                    "%d respawn(s)\n",
                    on ? "on" : "off", respawn, int(held), lateral, height, race.car(0).respawns);
        if (on) {
            CHECK(std::fabs(respawn - 12.0) < 1.5 * kDt && held && race.car(0).respawns == 1);
            CHECK(std::fabs(lateral) < 0.05f && std::fabs(height - 0.3f) < 0.02f);
        } else {
            CHECK(respawn < 0 && race.car(0).respawns == 0);
        }
    }
}
