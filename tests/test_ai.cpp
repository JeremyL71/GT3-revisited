// AI driver tests (src/ai/, docs/dev/AI.md).

#include "test_framework.h"

#include "ai/ai_driver.h"
#include "ai/ai_race.h"
#include "game/data/car_database.h"
#include "race/backend.h"
#include "race/field.h"
#include "sim/models.h"
#include "track/track.h"
#include "vfs/vfs.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <atomic>
#include <thread>

using namespace gt3r;

namespace {

data::CarDatabase* ai_database() {
    static data::CarDatabase db;
    static bool tried = false, ok = false;
    if (!tried) {
        tried = true;
        if (Disc* d = test::test_disc()) ok = db.load(d->volume());
    }
    return ok ? &db : nullptr;
}

std::unique_ptr<track::Track> load_course(const std::string& name) {
    Disc* d = test::test_disc();
    if (!d) return nullptr;
    const Volume::Entry* e = d->volume().find("courses/US/" + name);
    if (!e) return nullptr;
    Bytes data = d->volume().read(*e);
    return track::Track::from_course_file(data, name);
}

struct LapRun {
    int laps = 0;
    double best = -1, first = -1;
    int respawns = 0;
    double max_stall = 0; // longest time without 10 m of progress
};

LapRun run_laps(const track::Track& t, const data::CarDatabase& db, const std::string& car, int laps, double limit,
                ai::DriverParams p) {
    if (std::getenv("GT3R_AI_CS")) p.counter_steer = true;
    if (const char* g = std::getenv("GT3R_AI_SLEW")) p.steer_rate = float(std::atof(g));
    ai::AiRace race(t, db, {{car, p}});
    LapRun r;
    if (race.count() != 1) return r;
    while (race.time() < limit) {
        race.step();
        const auto& c = race.car(0);
        r.max_stall = std::max(r.max_stall, race.time() - c.last_progress_time());
        if (std::getenv("GT3R_AI_TRACE") && int(race.time() * 60) == 1) {
            const auto& g = c.driver->debug();
            std::printf("   grip %.2f m/s2 peak slip %.3f\n", g.grip, g.alpha_peak);
            ai::AiCourse co(t);
            const auto* l = co.line(co.resolve(6));
            for (size_t k = 0; k < l->points.size() && k < 12; ++k)
                std::printf("   pt%zu s%.0f kind %d r %.0f b%d v%d slope %.3f bank %.3f surface %d\n", k, l->points[k].progress,
                            l->points[k].kind, l->points[k].radius, l->points[k].brake_adjust, l->points[k].speed_adjust,
                            l->points[k].slope, l->points[k].bank, l->points[k].surface);
        }
        static const double from = std::getenv("GT3R_AI_FROM") ? std::atof(std::getenv("GT3R_AI_FROM")) : -1;
        bool dense = from >= 0 && race.time() >= from && race.time() < from + 1.5;
        if (std::getenv("GT3R_AI_TRACE") && (dense || int(race.time() * 60) % 30 == 0)) {
            const auto& g = c.driver->debug();
            const auto& st = c.vehicle->state();
            std::printf("   t%6.1f s%6.0f v%5.1f g%d line%d pt%d mode%d st%d dr%d vc%5.1f vn%5.1f bd%5.0f/%4.0f thr%.2f brk%.0f "
                        "eps%+5.2f ry%+5.2f a%+5.2f sat%.1f hit%u steer%+5.2f sa%+5.2f/%+5.2f yaw%+5.2f lat%5.1f pos(%.1f %.1f) wi%.0f\n",
                        race.time(), g.progress, st.speed_kmh, st.gear, g.line, g.next_point, g.mode, g.state, g.drive,
                        g.target_speed * 3.6, g.next_speed * 3.6, g.brake_distance, g.distance_to_point, g.throttle,
                        g.brake, g.heading_error, g.desired_yaw, g.slip_wanted, g.saturation, g.hits, g.steer_angle, st.wheels[0].slip_angle, st.wheels[1].slip_angle, -st.angular_velocity.y, c.vehicle->telemetry().accel_local.x, st.position.x, st.position.z,
                        c.vehicle->telemetry().wall_impulse);
            if (std::getenv("GT3R_AI_TRACE_ENGINE"))
                std::printf("     y %.2f susp %.2f rpm %.0f clutch %.2f locked %d torque %.0f wheels %.1f %.1f %.1f %.1f grounded %d%d%d%d surf %x %x %x %x\n",
                            st.position.y, st.wheels[0].suspension, st.engine_rpm, c.vehicle->telemetry().clutch, int(c.vehicle->telemetry().clutch_locked),
                            c.vehicle->telemetry().engine_torque, st.wheels[0].angular_velocity, st.wheels[1].angular_velocity,
                            st.wheels[2].angular_velocity, st.wheels[3].angular_velocity, int(st.wheels[0].grounded),
                            int(st.wheels[1].grounded), int(st.wheels[2].grounded), int(st.wheels[3].grounded),
                            st.wheels[0].surface, st.wheels[1].surface, st.wheels[2].surface, st.wheels[3].surface);
        }
        if (c.timer->laps_completed() >= laps) break;
    }
    const auto& c = race.car(0);
    r.laps = c.timer->laps_completed();
    r.best = c.timer->best_lap_time();
    r.respawns = c.respawns;
    return r;
}

std::string fmt_time(double t) {
    if (t < 0) return "--";
    char b[32];
    std::snprintf(b, sizeof b, "%d:%06.3f", int(t) / 60, std::fmod(t, 60.0));
    return b;
}

} // namespace

TEST(ai_params_builtin) {
    ai::DriverParams p = ai::DriverParams::builtin(0);
    CHECK(std::fabs(p.yaw_gain - 1.2f) < 1e-5f);
    CHECK(std::fabs(p.lookahead_full_speed - 50.0f) < 1e-3f);
    CHECK(p.lookahead_min == 15 && p.lookahead_max == 40);
    CHECK(std::fabs(p.steer_rate - 1.5708f) < 1e-3f);
    CHECK(std::fabs(p.corner_bonus - 20 / 3.6f) < 1e-4f);
    CHECK(std::fabs(p.pit_speed - 80 / 3.6f) < 1e-4f);
    ai::DriverParams p2 = ai::DriverParams::builtin(2);
    CHECK(p2.lookahead_min == 30 && p2.lookahead_max == 60);
}

TEST(ai_rubber_band_curve) {
    ai::RubberBand rb;
    rb.slow_amount = 0.2f, rb.slow_start = 20, rb.slow_full = 80;
    rb.fast_amount = 0.1f, rb.fast_start = 50, rb.fast_full = 100;
    CHECK(rb.scale(0) == 1.0f);
    CHECK(rb.scale(-10) == 1.0f);          // ahead, inside the dead zone
    CHECK(std::fabs(rb.scale(-50) - 0.9f) < 1e-5f);
    CHECK(std::fabs(rb.scale(-500) - 0.8f) < 1e-5f);
    CHECK(rb.scale(40) == 1.0f);            // behind, inside the dead zone
    CHECK(std::fabs(rb.scale(75) - 1.05f) < 1e-5f);
    CHECK(std::fabs(rb.scale(5000) - 1.1f) < 1e-5f);
}

TEST(ai_event_overrides) {
    data::CarDatabase* db = ai_database();
    if (!db) SKIP("needs GT3R_DISC");
    const data::Event* ev = db->event("Gsunday_e_0001");
    CHECK(ev != nullptr);
    if (!ev) return;
    ai::DriverParams p = ai::DriverParams::builtin(0);
    ai::apply_event(p, *ev, 0);
    CHECK(std::fabs(p.throttle_scale - 0.85f) < 1e-5f);
    CHECK(std::fabs(p.speed_scale - 0.80f) < 1e-5f);
    CHECK(std::fabs(p.brake_factor - 1.0f) < 1e-5f);
    ai::RubberBand rb = ai::RubberBand::from_event(*ev);
    CHECK(std::fabs(rb.slow_amount - 0.2f) < 1e-5f && rb.slow_start == 20 && rb.slow_full == 80);
    CHECK(std::fabs(rb.fast_amount - 0.1f) < 1e-5f && rb.fast_start == 50 && rb.fast_full == 100);
}

TEST(ai_line_roles) {
    Disc* d = test::test_disc();
    if (!d) SKIP("needs GT3R_DISC");
    // Main line evaluated every 5 m (ai §2.3) lies on the road mesh.
    size_t on = 0, n = 0, worst_n = 0;
    float worst = 1;
    std::string worst_name;
    for (const auto& e : d->volume().entries()) {
        const std::string prefix = "courses/US/";
        if (e.path.rfind(prefix, 0) != 0 || e.path.back() == '/') continue;
        Bytes data = d->volume().read(e);
        auto t = track::Track::from_course_file(data, e.path.substr(prefix.size()));
        ai::AiCourse c(*t);
        int l = c.resolve(ai::AiCourse::kMain);
        if (l < 0 || !c.line(l)->lap) continue;
        size_t con = 0, cn = 0;
        for (float s = 0; s < t->length(); s += 5, ++cn) {
            sim::Vec3 p = c.evaluate(l, s);
            con += t->probe(p.x, p.z, 1e30f).hit;
        }
        on += con, n += cn;
        float f = float(con) / float(std::max<size_t>(cn, 1));
        if (f < worst) worst = f, worst_name = t->label(), worst_n = cn - con;
    }
    std::printf("  main line on the road: %zu/%zu samples; worst %s (%zu off)\n", on, n, worst_name.c_str(), worst_n);
    CHECK(n > 0 && on >= n * 995 / 1000);
}

// The course files' AI driver parameter sets (slots 37-40, ai §9.1): decoded on every US course;
// a set per driveline class on all but the course whose slots are empty, which takes the
// built-in set of its category.
TEST(ai_course_param_sets_disc) {
    Disc* d = test::test_disc();
    if (!d) SKIP("needs GT3R_DISC");
    int files = 0, with_sets = 0, partial = 0, pit_other = 0;
    std::string without;
    for (const auto& e : d->volume().entries()) {
        const std::string prefix = "courses/US/";
        if (e.path.rfind(prefix, 0) != 0 || e.path.back() == '/') continue;
        Bytes data = d->volume().read(e);
        auto t = track::Track::from_course_file(data, e.path.substr(prefix.size()));
        ++files;
        int n = 0;
        for (int c = 0; c < 4; ++c) n += t->driver_sets().for_class(c) != nullptr;
        if (n == 4) ++with_sets;
        else if (n == 0) without += (without.empty() ? "" : " ") + t->label();
        else ++partial;
        for (int c = 0; c < 4; ++c)
            if (const auto* s = t->driver_sets().for_class(c)) {
                ai::DriverParams p = ai::DriverParams::from_bytes(*s);
                pit_other += std::fabs(p.pit_speed * 3.6f - 80) > 2.5f; // 78 or 80 km/h on the disc
            }
    }
    std::printf("  %d course files: %d with four sets, %d partial, none on: %s; pit speeds other than 78-80: %d\n",
                files, with_sets, partial, without.c_str(), pit_other);
    CHECK(files == 80 && with_sets == 79 && partial == 0 && without == "L_circle60_wet1" && pit_other == 0);
}

// One car, two laps on every lap course of the US set.
TEST(ai_laps_all_courses) {
    data::CarDatabase* db = ai_database();
    if (!db) SKIP("needs GT3R_DISC");
    Disc* d = test::test_disc();
    const char* only = std::getenv("GT3R_AI_COURSE");
    const std::string car = std::getenv("GT3R_AI_CAR") ? std::getenv("GT3R_AI_CAR") : "r34_gtr_v_99";
    int courses = 0, ok = 0;
    std::printf("  reference car %s\n", car.c_str());
    // Read every course first (disc access is serial), then drive them in parallel.
    struct Job {
        std::string name;
        std::unique_ptr<track::Track> track;
        LapRun result;
    };
    std::vector<Job> jobs;
    for (const auto& e : d->volume().entries()) {
        const std::string prefix = "courses/US/";
        if (e.path.rfind(prefix, 0) != 0 || e.path.back() == '/') continue;
        std::string name = e.path.substr(prefix.size());
        if (name.rfind("L_", 0) == 0) continue; // licence sections
        if (only && name != only) continue;
        Bytes data = d->volume().read(e);
        auto t = track::Track::from_course_file(data, name);
        ai::AiCourse c(*t);
        int main = c.resolve(ai::AiCourse::kMain);
        if (main < 0 || !c.line(main)->lap) continue; // no lap line (none among the race courses)
        jobs.push_back({name, std::move(t), {}});
    }
    std::atomic<size_t> next{0};
    auto worker = [&] {
        for (size_t k; (k = next++) < jobs.size();)
        {
            // ai §9.1: the course's set for the car's driveline class (no event overrides).
            jobs[k].result = run_laps(*jobs[k].track, *db, car, 2, 30.0 + jobs[k].track->length() / 8.0,
                                      ai::race_params(*db, car, *jobs[k].track));
        }
    };
    std::vector<std::thread> pool;
    for (unsigned i = 0; i < std::max(1u, std::min(8u, std::thread::hardware_concurrency())); ++i) pool.emplace_back(worker);
    for (auto& th : pool) th.join();
    for (const Job& j : jobs) {
        const LapRun& r = j.result;
        ++courses;
        bool good = r.laps >= 2 && r.max_stall < 20 && r.respawns <= 2;
        ok += good;
        std::printf("  %-16s %6.0f m  laps %d  best %s  respawns %d  stall %.1fs%s\n", j.name.c_str(), j.track->length(),
                    r.laps, fmt_time(r.best).c_str(), r.respawns, r.max_stall, good ? "" : "  <--");
    }
    std::printf("  %d/%d lap courses: two laps, at most two safety respawns\n", ok, courses);
    CHECK(courses > 0);
    CHECK(ok == courses);
}

// Six cars of beginner events on their courses: everyone finishes, no
// permanent pile-up (cars collide through sim::World).
TEST(ai_event_races) {
    data::CarDatabase* db = ai_database();
    if (!db) SKIP("needs GT3R_DISC");
    for (const char* label : {"Gsunday_e_0001", "Gsunday_e_0002", "Gsunday_e_0003", "Gclubman_e_0003"}) {
        const data::Event* ev = db->event(label);
        CHECK(ev != nullptr);
        if (!ev) continue;
        auto t = load_course(ev->course);
        CHECK(t != nullptr);
        if (!t) continue;
        std::vector<ai::AiRace::Entry> entries;
        for (const auto& name : ev->entries) {
            if (name.empty() || name == "-") continue;
            // The course's set for each car's driveline class and the event's overrides for it.
            entries.push_back({name, ai::race_params(*db, name, *t, ev)});
            if (entries.size() == 6) break;
        }
        // Car 0 stands in for the player (rubber-band reference).
        ai::AiRace race(*t, *db, entries, ai::RubberBand::from_event(*ev), 0);
        CHECK(race.error().empty());
        double max_stall = 0;
        const int laps = 2;
        const char* tr = std::getenv("GT3R_AI_EVTRACE"); // "<event>:<car>"
        int trace_car = -1;
        if (tr && std::string(tr).rfind(label, 0) == 0) trace_car = std::atoi(std::strchr(tr, ':') + 1);
        while (race.time() < 900) {
            race.step();
            if (trace_car >= 0 && int(race.time() * 60) % 30 == 0) {
                const auto& c = race.car(trace_car);
                const auto& g = c.driver->debug();
                const auto& st = c.vehicle->state();
                std::printf("   t%6.1f s%6.0f v%5.1f g%d line%d pt%d mode%d st%d dr%d vc%5.1f vn%5.1f thr%.2f brk%.0f eps%+5.2f "
                            "a%+5.2f sat%.1f hit%u steer%+5.2f c%.2f z%02X resp%d pos(%.0f %.0f) gw%d rpm%.0f\n",
                            race.time(), g.progress, st.speed_kmh, st.gear, g.line, g.next_point, g.mode, g.state, g.drive,
                            g.target_speed * 3.6, g.next_speed * 3.6, g.throttle, g.brake, g.heading_error, g.slip_wanted,
                            g.saturation, g.hits, g.steer_angle, g.contact, g.zones, c.respawns, st.position.x,
                            st.position.z, g.gear_want, st.engine_rpm);
            }
            bool all = true;
            for (int i = 0; i < race.count(); ++i) {
                max_stall = std::max(max_stall, race.time() - race.car(i).last_progress_time());
                all = all && race.car(i).timer->laps_completed() >= laps;
            }
            if (all) break;
        }
        std::printf("  %s on %s, %d cars, %.1f s, longest stall %.1f s, %d car-car contacts (%d nose-to-tail), %d position changes\n",
                    ev->label.c_str(), ev->course.c_str(), race.count(), race.time(), max_stall, race.contacts(),
                    race.contacts_inline(), race.overtakes());
        int pos = 1;
        for (int i : race.ranking()) {
            const auto& c = race.car(i);
            std::printf("   P%d %-28s laps %d best %s respawns %d\n", pos++, c.label.c_str(), c.timer->laps_completed(),
                        fmt_time(c.timer->best_lap_time()).c_str(), c.respawns);
            CHECK(c.timer->laps_completed() >= laps);
            CHECK(c.respawns <= 2);
        }
        CHECK(max_stall < 20);
    }
}

TEST(ai_deterministic) {
    data::CarDatabase* db = ai_database();
    if (!db) SKIP("needs GT3R_DISC");
    auto t = load_course("trialmt");
    if (!t) SKIP("no trialmt");
    const ai::DriverParams p = ai::race_params(*db, "r34_gtr_v_99", *t);
    std::vector<ai::AiRace::Entry> e = {{"r34_gtr_v_99", p}, {"r34_gtr_v_99", p}, {"r34_gtr_v_99", p}};
    ai::AiRace a(*t, *db, e), b(*t, *db, e);
    for (int i = 0; i < 60 * 40; ++i) {
        a.step();
        b.step();
    }
    bool same = true;
    for (int i = 0; i < a.count(); ++i) {
        const auto &pa = a.car(i).vehicle->state().position, &pb = b.car(i).vehicle->state().position;
        same = same && std::memcmp(&pa, &pb, sizeof pa) == 0;
    }
    CHECK(same);
}

// Diagnostic (prints only): wall clearance along the main line, GT3R_AI_WALLS=<course>:<s0>:<s1>.
TEST(ai_debug_line_walls) {
    const char* w = std::getenv("GT3R_AI_WALLS");
    if (!w || !test::test_disc()) SKIP("diagnostic");
    std::string spec = w;
    std::string name = spec.substr(0, spec.find(':'));
    float s0 = std::stof(spec.substr(spec.find(':') + 1)), s1 = std::stof(spec.substr(spec.rfind(':') + 1));
    auto t = load_course(name);
    ai::AiCourse c(*t);
    int l = c.resolve(6);
    if (const char* pt = std::getenv("GT3R_AI_WALLPT")) { // "x:z"
        float x = std::stof(pt), z = std::stof(std::strchr(pt, ':') + 1);
        sim::SurfaceHit h = t->probe(x, z, 1e30f);
        track::WallContact wc = t->wall_contact({x, h.height + 0.5f, z}, 4.0f);
        sim::SurfaceHit low = t->probe(x, z, h.height - 3.0f);
        std::printf("  point (%.1f %.1f) road %d top %.1f below %s%.1f lateral %.1f wall %d edge %d dist %.2f\n", x, z,
                    h.hit, h.height, low.hit ? "" : "-", low.hit ? low.height : 0.0f, t->locate({x, h.height, z}).lateral,
                    wc.wall, wc.edge, wc.hit ? 4.0f - wc.penetration : -1.0f);
    }
    for (size_t k = 0; k < c.line(l)->points.size(); ++k) {
        const auto& q = c.line(l)->points[k];
        if (c.ahead(s0 - 150, q.progress) > s1 - s0 + 300) continue;
        sim::SurfaceHit h = t->probe(q.pos.x, q.pos.z, 1e30f);
        std::printf("  pt%zu s %.0f pos(%.1f %.1f) on_road %d kind %d r %.1f arc %d centre(%.1f %.1f) a0 %.2f da %.2f hdg %.3f chord_dir %.3f\n", k,
                    q.progress, q.pos.x, q.pos.z, h.hit, q.kind, q.radius, q.arc, q.cx, q.cz, q.a0, q.da, q.heading,
                    k + 1 < c.line(l)->points.size() ? std::atan2(c.line(l)->points[k + 1].pos.x - q.pos.x, c.line(l)->points[k + 1].pos.z - q.pos.z) : 0.0);
    }
    for (float s = s0; s <= s1; s += 2) {
        sim::Vec3 p = c.evaluate(l, s);
        track::TrackPosition tp = t->locate(p);
        track::WallContact wc = t->wall_contact(p + sim::Vec3{0, 0.5f, 0}, 6.0f);
        sim::SurfaceHit top = t->probe(p.x, p.z, 1e30f), low = t->probe(p.x, p.z, top.height - 3.0f);
        std::printf("  s %.0f pos(%.1f %.1f %.1f) lat %.1f centre_y %.1f road top %.1f below %s%.1f wall %d dist %.2f\n", s,
                    p.x, p.y, p.z, tp.lateral, tp.centre.y, top.height, low.hit ? "" : "-", low.hit ? low.height : 0.0f,
                    wc.wall, wc.hit ? 6.0f - wc.penetration : -1.0f);
    }
}

// A CPU pit stop driven by the AI driver itself (ai §13): with a wear curve that arms the car at
// once (invented: fresh wear 0, "worn" below 1.01), the opponent enters at the stop line's point
// 0, drives the pit lane line at the set's pit speed (driver kind 2), the stop line at 20 km/h
// (3), stops in its box, leaves (4) and returns to racing (0).
TEST(ai_cpu_pit_stop_disc) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Vfs vfs(&disc->volume());
    data::CarDatabase db;
    CHECK(db.load(vfs));
    Bytes course = disc->volume().read(*disc->volume().find("courses/US/trialmt"));
    std::unique_ptr<race::RaceTrack> track = race::make_track(vfs, course, "trialmt");
    CHECK(track->runtime() != nullptr);
    race::FieldConfig cfg;
    cfg.player = "r34_gtr_v_99";
    cfg.opponents = {"r34_gtr_v_99"};
    cfg.countdown = 0;
    cfg.wear = sim::model::shipped_wear_params();
    cfg.wear.fresh = 0;
    cfg.wear.worn = 1.01f;
    std::string err;
    std::unique_ptr<race::RaceField> f = race::RaceField::create(*track, db, cfg, &err);
    CHECK(f != nullptr);
    if (!f) return;
    f->set_laps(10);
    const ai::DriverParams p = ai::course_params(track->runtime()->driver_sets(), 0,
                                                 ai::driveline_class(f->opponent_vehicle(0)->spec()));
    double entered = -1, stopped = -1, left = -1, back = -1;
    float at_decision = -1, at_box = 0, last_kmh = 0, surface_top = 0;
    int kinds = 0, last_kind = 0;
    while (f->race_time() < 240 && back < 0) {
        f->player().step(sim::ControlInputs{}, 1.0 / 60);
        for (int i : f->take_pit_entries())
            if (i == 0 && entered < 0) entered = f->race_time();
        const sim::Vehicle& v = *f->opponent_vehicle(0);
        int k = v.driver_kind();
        kinds |= 1 << k;
        if (k == 3 && last_kind == 2) at_decision = v.state().speed_kmh; // 25 m before the box
        if (k == 3 && v.telemetry().on_pit_surface && at_decision >= 0) surface_top = std::max(surface_top, v.state().speed_kmh);
        if (stopped < 0 && v.spawn_hold_active()) stopped = f->race_time(), at_box = last_kmh;
        last_kmh = v.state().speed_kmh, last_kind = k;
        if (left < 0 && stopped >= 0 && k == 4) left = f->race_time();
        if (left >= 0 && k == 0 && !f->opponent_in_pits(0)) back = f->race_time();
    }
    std::printf("  CPU pit stop on trialmt: entered %.1f s, stopped %.1f s, leaving %.1f s, racing %.1f s; "
                "%.1f km/h at the stop decision (set's pit speed %.0f + 2), at most %.1f km/h on the stop line, "
                "%.1f km/h into the box; driver kinds seen %x\n",
                entered, stopped, left, back, at_decision, p.pit_speed * 3.6f, surface_top, at_box, kinds);
    CHECK(entered > 0 && stopped > entered && left > stopped && back > left);
    CHECK(kinds == 0x1D); // 0, 2, 3, 4
    CHECK(at_decision > 0 && at_decision < 100 && at_box < 30);
}
