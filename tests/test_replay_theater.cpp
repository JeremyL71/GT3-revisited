// Replay Theater, REPLAY OPTIONS and the Analyzer (docs/dev/RACE.md "Replay Theater"):
// replay format v7 with the car spec, the replay files the theater lists, the theater's file
// list page, the replay options' state and the Analyzer's lap analysis.
#include "test_framework.h"
#include "base/config.h"
#include "base/file.h"
#include "game/replay_theater.h"
#include "menu/flow/theater_page.h"
#include "race/analyzer.h"
#include "race/backend.h"
#include "race/replay.h"
#include "race/replay_library.h"
#include "race/replay_options.h"
#include "sim/car_spec_io.h"
#include "sim/vehicle.h"
#include "vfs/vfs.h"

#include <SDL3/SDL.h>

#include <cmath>
#include <cstring>
#include <filesystem>

using namespace gt3r;
using namespace gt3r::race;

namespace {

struct FlatRoad : sim::TrackQuery {
    sim::SurfaceHit probe(float, float, float y_max) const override {
        sim::SurfaceHit h;
        h.hit = y_max >= 0;
        return h;
    }
};

class FlatVehicle : public RaceVehicle {
public:
    explicit FlatVehicle(const sim::CarSpec& spec = sim::CarSpec::test_car(sim::Drive::FR)) : v_(sim::Vehicle::create(spec)) {}
    const char* backend_name() const override { return "test"; }
    void reset(const TrackPose& p) override { v_->reset(p.position, p.orientation); }
    void step(const sim::ControlInputs& in, double dt) override { v_->step(in, road_, dt); }
    const sim::VehicleState& state() const override { return v_->state(); }
    const sim::VehicleSetup& setup() const override { return v_->setup(); }
    float last_impact() const override { return 0; }

private:
    FlatRoad road_;
    std::unique_ptr<sim::Vehicle> v_;
};

std::string temp_dir(const char* name) {
    std::filesystem::path p = std::filesystem::temp_directory_path() / name;
    std::filesystem::remove_all(p);
    std::filesystem::create_directories(p);
    return p.string();
}

using menu::flow::key_bit;
using menu::flow::MenuKey;

} // namespace

// Every field of a spec survives the bytes exactly (re-serialising gives the same bytes),
// and a vehicle built from the copy runs bit-identically.
TEST(car_spec_io_roundtrip) {
    sim::CarSpec s = sim::CarSpec::test_car(sim::Drive::AWD);
    s.label = "test_car", s.name = "Test Car";
    s.tire[0].compound = "racing_hard";
    s.engine.turbo.on = true, s.engine.turbo.stage[1].peak = 1.25f;
    s.drivetrain.lsd_accel = {12, 34};
    s.aids.ascc_flags = 5;
    Bytes b = sim::serialize_car_spec(s);
    std::optional<sim::CarSpec> q = sim::parse_car_spec(b);
    CHECK(q.has_value());
    if (!q) return;
    CHECK(sim::serialize_car_spec(*q) == b);
    CHECK(q->name == "Test Car" && q->tire[0].compound == "racing_hard" && q->drivetrain.lsd_accel[1] == 34);
    CHECK(q->engine.curve_torque == s.engine.curve_torque && q->aids.ascc_flags == 5);
    FlatVehicle a(s), c(*q);
    TrackPose p{{0, 0, 0}, {}};
    a.reset(p), c.reset(p);
    sim::ControlInputs in;
    in.throttle = 1;
    for (int i = 0; i < 300; ++i) a.step(in, 1.0 / 60), c.step(in, 1.0 / 60);
    CHECK(a.state().position.z == c.state().position.z && a.state().engine_rpm == c.state().engine_rpm);
    // Malformed input is refused.
    Bytes cut(b.begin(), b.begin() + long(b.size() / 2));
    CHECK(!sim::parse_car_spec(cut));
    CHECK(!sim::parse_car_spec({}));
}

// Replay format v7: what the theater shows, and the run's car.
TEST(replay_v7_theater_fields) {
    Replay r;
    r.course = "trialmt", r.car = "r34_gtr_v_99";
    for (int i = 0; i < 20; ++i) r.inputs.push_back(quantize(sim::ControlInputs{}));
    r.comment = "My best";
    r.kind = ReplayKind::GtRace;
    r.laps = 3;
    r.total_time = 301.5;
    r.power_ps = 330;
    r.car_model = "ni0137", r.wheel = "rays_01";
    r.saved = 1760000000;
    r.spec = std::make_shared<const sim::CarSpec>(sim::CarSpec::test_car());
    r.condition = {4321, -12345.5f, 300000}; // v8: the engine condition the run started with
    Replay q = Replay::parse(r.serialize());
    CHECK(q.condition == r.condition);
    CHECK(q.comment == "My best" && q.kind == ReplayKind::GtRace && q.laps == 3 && q.total_time == 301.5);
    CHECK(q.power_ps == 330 && q.car_model == "ni0137" && q.wheel == "rays_01" && q.saved == 1760000000);
    CHECK(q.spec && sim::serialize_car_spec(*q.spec) == sim::serialize_car_spec(*r.spec));
    CHECK(q.ticks() == 20);
    CHECK(std::string(replay_kind_name(ReplayKind::License)) == "License");
    r.spec.reset();
    CHECK(!Replay::parse(r.serialize()).spec); // no spec: the database car
}

// Replay format v9: the field's rules as the run had them (a GT race's grid by power-to-weight
// is rebuilt from these in the theater); older files take the grid from their kind.
TEST(replay_v9_field_rules) {
    Replay r;
    r.course = "trialmt", r.car = "r34_gtr_v_99", r.event = "Gsunday_e_0001";
    r.kind = ReplayKind::ArcadeRace;
    r.grid_by_power = true, r.cpu_respawn = false;
    Replay q = Replay::parse(r.serialize());
    CHECK(q.grid_by_power == std::optional<bool>(true) && q.cpu_respawn == std::optional<bool>(false));
    // Not recorded stays not recorded (an old replay saved again from the theater).
    r.grid_by_power.reset(), r.cpu_respawn.reset();
    q = Replay::parse(r.serialize());
    CHECK(!q.grid_by_power && !q.cpu_respawn);
    // A v8 file (no rules byte; with no inputs it is the last byte): the grid from the kind.
    auto as_v8 = [](const Replay& rp) {
        Bytes b = rp.serialize();
        b.pop_back();
        const u32 v8 = 8;
        std::memcpy(b.data() + 8, &v8, 4);
        return Replay::parse(b);
    };
    r.kind = ReplayKind::GtRace;
    q = as_v8(r);
    CHECK(q.grid_by_power == std::optional<bool>(true) && !q.cpu_respawn);
    r.kind = ReplayKind::Practice; // Quick race: the arcade single race's grid
    CHECK(as_v8(r).grid_by_power == std::optional<bool>(false));

    // 2P battle: the Options menu's settings and player 2's car as the run had them.
    Replay p2;
    p2.course = "trialmt", p2.car = "r34_gtr_v_99", p2.car2 = "supra_rz_97";
    for (int i = 0; i < 3; ++i) p2.inputs.push_back(quantize(sim::ControlInputs{})), p2.inputs2.push_back(p2.inputs.back());
    ReplayBattle b;
    b.laps = 5;
    b.boost_amount = 0.2f, b.boost_start = 10, b.boost_full = 200;
    b.handicap_m = -35;
    b.tyre_damage = true;
    b.model2 = "to0050", b.wheel2 = "rays_02", b.paint2 = 4;
    sim::CarSpec spec2 = sim::CarSpec::test_car();
    spec2.aids.tcs_gain = 0;
    b.spec2 = std::make_shared<const sim::CarSpec>(spec2);
    p2.battle = b;
    q = Replay::parse(p2.serialize());
    CHECK(q.battle.has_value() && q.inputs2.size() == 3);
    if (q.battle) {
        const ReplayBattle& qb = *q.battle;
        CHECK(qb.laps == 5 && qb.boost_amount == 0.2f && qb.boost_start == 10 && qb.boost_full == 200);
        CHECK(qb.handicap_m == -35 && qb.tyre_damage && qb.model2 == "to0050" && qb.wheel2 == "rays_02" && qb.paint2 == 4);
        CHECK(qb.spec2 && sim::serialize_car_spec(*qb.spec2) == sim::serialize_car_spec(spec2));
    }
    p2.battle->spec2.reset(); // the database car
    q = Replay::parse(p2.serialize());
    CHECK(q.battle && !q.battle->spec2);
    // Not recorded (an older 2P replay saved again) stays not recorded.
    p2.battle.reset();
    q = Replay::parse(p2.serialize());
    CHECK(!q.battle && q.car2 == "supra_rz_97" && q.inputs2.size() == 3);
}

// The theater's storage: one folder, newest first, delete, Edit file name, info texts.
TEST(replay_library_files) {
    const std::string data = temp_dir("gt3r_test_theater");
    const std::string dir = replay_dir(data);
    CHECK(dir == data + "/replays" || dir == data + "replays");
    Replay r;
    r.course = "trialmt", r.car = "r34_gtr_v_99";
    for (int i = 0; i < 60; ++i) r.inputs.push_back(quantize(sim::ControlInputs{}));
    std::optional<std::string> p1 = save_replay(r, dir);
    std::optional<std::string> p2 = save_replay(r, dir); // same second: the next stamp
    CHECK(p1 && p2 && *p1 != *p2);
    std::vector<ReplayEntry> l = list_replay_entries(dir);
    CHECK(l.size() == 2);
    if (l.size() != 2) return;
    CHECK(l[0].header.saved > 0 && l[0].ticks == 60 && l[0].header.inputs.empty() && l[0].size > 0);
    CHECK(replay_date(l[0]).size() == 10);
    CHECK(replay_display_name(l[0]).find(':') != std::string::npos); // unnamed: the save time
    CHECK(rename_replay_file(l[0].path, "Fast lap"));
    std::vector<ReplayEntry> l2 = list_replay_entries(dir);
    bool named = false;
    for (const ReplayEntry& e : l2) named = named || replay_display_name(e) == "Fast lap";
    CHECK(named);
    CHECK(delete_replay_file(l[1].path));
    CHECK(list_replay_entries(dir).size() == 1);
    CHECK(replay_size_text(1) == "1KB" && replay_size_text(2048) == "2KB");
    CHECK(replay_time_text(83.4561) == "1:23.456" && replay_time_text(-1) == "-:--.---");
    // The theater backend's entries: replays with their info texts (no disc: labels).
    std::vector<menu::flow::TheaterEntry> e = game::replay_theater_entries(0, dir, nullptr);
    CHECK(e.size() == 1 && e[0].deletable && e[0].text("course_id") == "trialmt" && e[0].text("comment") == "Fast lap");
    std::filesystem::remove_all(data);
}

// The demonstrations in the original's order and names (the DEMONSTRATION page).
TEST(theater_demonstration_names) {
    std::vector<std::string> c = game::demonstration_codes();
    CHECK(c.size() == 58 && c[0] == "L0B0001" && c[8] == "L0A0001" && c[16] == "LIB0001" && c[48] == "AT010001" &&
          c[57] == "AT100001");
    CHECK(game::demonstration_mode("L0B0001") == "License B-1" && game::demonstration_file("L0B0001") == "B1_demo");
    CHECK(game::demonstration_mode("LIA0008") == "License IA-8" && game::demonstration_file("LIA0008") == "IA8_demo");
    CHECK(game::demonstration_mode("AT010001") == "Time Trial 1" && game::demonstration_file("AT100001") == "TT10_demo");
}

// The file list page: 8 to a row, confirm plays at once, Start deletes after one OK.
TEST(theater_page_flow) {
    using menu::flow::TheaterEntry;
    using menu::flow::TheaterPage;
    std::vector<TheaterEntry> entries(19);
    for (size_t i = 0; i < entries.size(); ++i) entries[i].id = "f" + std::to_string(i), entries[i].deletable = true;
    std::vector<std::string> removed;
    menu::flow::TheaterBackend b;
    b.remove = [&](const TheaterEntry& e) {
        removed.push_back(e.id);
        return true;
    };
    menu::flow::set_theater_backend(b);
    TheaterPage p;
    std::vector<std::string> se;
    p.open(TheaterPage::Mode::LoadReplay, entries);
    auto press = [&](MenuKey k) {
        p.tick(0, se);
        int r = p.tick(key_bit(k), se);
        p.tick(0, se);
        return r;
    };
    p.tick(0, se);
    CHECK(press(MenuKey::Right) == 0 && p.cursor() == 1);
    CHECK(press(MenuKey::Down) == 0 && p.cursor() == 9);
    CHECK(press(MenuKey::Down) == 0 && p.cursor() == 17);
    CHECK(press(MenuKey::Down) == 0 && p.cursor() == 17); // no row below
    CHECK(press(MenuKey::Right) == 0 && p.cursor() == 18);
    CHECK(press(MenuKey::Right) == 0 && p.cursor() == 18); // the last file
    CHECK(p.top_row() == 0);
    // Delete: Start asks (CANCEL focused), OK deletes.
    CHECK(press(MenuKey::Start) == 0 && p.confirming());
    CHECK(press(MenuKey::Confirm) == 0 && !p.confirming() && removed.empty()); // CANCEL
    press(MenuKey::Start);
    press(MenuKey::Left); // OK
    CHECK(press(MenuKey::Confirm) == 0 && removed.size() == 1 && removed[0] == "f18" && p.entries().size() == 18);
    CHECK(p.cursor() == 17);
    CHECK(press(MenuKey::Confirm) == 1 && p.chosen() && p.chosen()->id == "f17");
    CHECK(press(MenuKey::Cancel) == -1);
    // An empty list refuses confirm.
    p.open(TheaterPage::Mode::Demonstration, {});
    p.tick(0, se);
    CHECK(press(MenuKey::Confirm) == 0);
    menu::flow::set_theater_backend({});
}

// REPLAY OPTIONS: the mode table, the rows, toggles, storage.
TEST(replay_options_state) {
    CHECK(ReplaySettings::target_allowed(0, 0) && !ReplaySettings::target_allowed(0, 1) && !ReplaySettings::target_allowed(0, 2));
    CHECK(!ReplaySettings::target_allowed(1, 0) && ReplaySettings::target_allowed(1, 2));
    CHECK(ReplaySettings::target_allowed(2, 0) && ReplaySettings::target_allowed(2, 1) && ReplaySettings::target_allowed(2, 2));
    CHECK(ReplaySettings::default_target(1) == 2 && ReplaySettings::default_target(3) == 0);
    ReplaySettings s;
    CHECK(s.mode == 0 && s.current_target() == 0 && s.items == 0 && s.bgm && s.se);
    s.mode = 1, s.bgm = false;
    CHECK(s.bgm_on()); // avant-garde forces the BGM

    ReplayOptions o;
    o.open(ReplaySettings{}, false);
    o.tick(0);
    auto press = [&](MenuKey k) {
        o.tick(key_bit(k));
        return o.tick(0);
    };
    CHECK(o.rows() == 5 && o.row() == ReplayOptions::RowMode);
    press(MenuKey::Right), press(MenuKey::Right); // broadcast
    press(MenuKey::Confirm);
    CHECK(o.settings().mode == 2 && o.settings().current_target() == 2 && o.changed());
    press(MenuKey::Down); // TARGET: the cursor on AUTO
    CHECK(o.column() == 2);
    press(MenuKey::Left), press(MenuKey::Confirm); // TOP
    CHECK(o.settings().target[2] == 1 && o.settings().current_target() == 1);
    press(MenuKey::Up), press(MenuKey::Left), press(MenuKey::Left), press(MenuKey::Confirm); // traditional
    CHECK(o.settings().mode == 0 && o.settings().current_target() == 0);
    press(MenuKey::Down);
    CHECK(o.cell(ReplayOptions::RowTarget, 1) == 2 && o.cell(ReplayOptions::RowTarget, 0) == 1);
    press(MenuKey::Right); // TOP / AUTO are not allowed: the cursor stays
    CHECK(o.column() == 0);
    press(MenuKey::Down), press(MenuKey::Right), press(MenuKey::Right), press(MenuKey::Confirm); // ITEM time
    CHECK(o.settings().items == ReplaySettings::Time);
    press(MenuKey::Down); // the second row keeps the column
    CHECK(o.row() == ReplayOptions::RowItem2 && o.column() == 2);
    press(MenuKey::Confirm); // tyres
    CHECK(o.settings().items == (ReplaySettings::Time | ReplaySettings::Tire));
    press(MenuKey::Down), press(MenuKey::Right), press(MenuKey::Confirm); // SE off
    CHECK(!o.settings().se && o.settings().bgm);
    CHECK(!o.tick(key_bit(MenuKey::Cancel))); // closes
    // Applying the avant-garde mode stores BGM on (it stays on in the other modes).
    ReplaySettings quiet;
    quiet.bgm = false;
    o.open(quiet, false);
    o.tick(0);
    press(MenuKey::Right), press(MenuKey::Confirm);
    CHECK(o.settings().mode == 1 && o.settings().bgm);
    press(MenuKey::Left), press(MenuKey::Confirm);
    CHECK(o.settings().mode == 0 && o.settings().bgm);
    // 2P: the SCREEN row.
    o.open(ReplaySettings{}, true);
    CHECK(o.rows() == 6 && o.cell(ReplayOptions::RowScreen, 1) == 1);
    // [options] storage.
    IniDocument cfg;
    ReplaySettings w;
    w.mode = 2, w.target = {0, 2, 1, 0}, w.items = 9, w.se = false, w.split = false;
    w.save(cfg);
    ReplaySettings rd = ReplaySettings::load(cfg);
    CHECK(rd.mode == 2 && rd.target[2] == 1 && rd.items == 9 && !rd.se && rd.bgm && !rd.split);
}

// The Analyzer: a replay's lap sampled tick by tick into a trace (distances from the line), sectors.
TEST(analyzer_replay_lap) {
    FlatVehicle live;
    ReplayRecorder rec;
    TrackPose start{{0, 0, 0}, {}};
    rec.begin("test", "test_car", true, 0, start, 60.0);
    live.reset(start);
    sim::ControlInputs in;
    in.throttle = 1;
    for (int i = 0; i < 600; ++i) live.step(rec.record(in), 1.0 / 60);
    rec.lap(5.0, 120, 420);
    Replay r = rec.replay();
    // Distance along +z on a 400 m "lap"; the run played back as the race screen plays it.
    const float L = 400;
    auto dist = [&](const sim::Vec3& p) { return std::fmod(std::fabs(p.z) + 1000 * L, L); };
    auto sample = [&](const Replay& rp) {
        const AnalyzerLap lap = analyzer_lap(rp);
        LapSampler s(rp.tick_hz, L, lap.whole);
        FlatVehicle v;
        v.place(rp.start);
        ReplayPlayer p(&rp);
        while (!p.done() && p.tick() < lap.end) {
            const u32 k = p.tick();
            sim::ControlInputs pin = p.next_inputs();
            if (k >= lap.begin) s.add(v.state(), pin, dist(v.state().position));
            v.step(pin, 1.0 / rp.tick_hz);
        }
        return s.take(lap.whole ? -1 : rp.best_lap);
    };
    const AnalyzerLap lap = analyzer_lap(r);
    CHECK(!lap.whole && lap.begin == 120 && lap.end == 420);
    AnalyzerTrack t = sample(r);
    CHECK(t.trace.size() == 300 && t.ghost.size() == 300 && t.trace.time == 5.0);
    bool rising = true;
    for (size_t i = 1; i < t.trace.size(); ++i) rising = rising && t.trace.dist[i] >= t.trace.dist[i - 1];
    CHECK(rising);
    CHECK(t.index_at_distance(t.trace.dist[150]) <= 150 && t.trace.speed_kmh[299] > t.trace.speed_kmh[0]);
    CHECK(t.trace.throttle[10] == 1.0f && t.trace.x.size() == 300);
    // A replay without a lap: the whole run from 0.
    Replay whole = r;
    whole.best_lap = -1;
    CHECK(analyzer_lap(whole).whole && analyzer_lap(whole).end == 600);
    AnalyzerTrack w = sample(whole);
    CHECK(w.trace.size() == 600 && w.trace.dist[0] == 0.0f && std::fabs(w.trace.time - 10.0) < 1e-9);
    // A lap whose first sample is just short of the line counts from 0, not from the lap length.
    LapSampler ls(60.0, L, false);
    sim::VehicleState s0;
    ls.add(s0, in, L - 1.0f), ls.add(s0, in, 2.0f), ls.add(s0, in, 6.0f);
    AnalyzerTrack short_start = ls.take(1.0);
    CHECK(short_start.trace.dist[0] == 0.0f && short_start.trace.dist[1] == 2.0f && short_start.trace.dist[2] == 6.0f);

    // Sectors from the logger map's points, sorted, deduplicated, starting at 0.
    LoggerMap m;
    m.points = {{0, 0, 250}, {0, 0, 100}, {0, 0, 102}, {0, 0, 399.5f}};
    std::vector<float> s = logger_sectors(m, [](float, float, float z) { return z; }, L);
    CHECK(s.size() == 3 && s[0] == 0 && s[1] == 100 && s[2] == 250);
    Analyzer a;
    a.set_course(nullptr, std::nullopt, s, L);
    CHECK(a.sector_at(0) == 0 && a.sector_at(99) == 0 && a.sector_at(100) == 1 && a.sector_at(399) == 2);
    // Keys: R1 jumps to the next sector, L1 back; Triangle leaves.
    a.set_tracks(t, {});
    a.open(Analyzer::View::Graph);
    a.tick(0);
    a.tick(key_bit(MenuKey::R1)), a.tick(0);
    CHECK(a.cursor() == 100);
    a.tick(key_bit(MenuKey::R1)), a.tick(0);
    CHECK(a.cursor() == 250);
    a.tick(key_bit(MenuKey::L1)), a.tick(0);
    CHECK(a.cursor() == 100);
    CHECK(!a.tick(key_bit(MenuKey::Cancel)));
}

// LGM0: the map's transform and sector points.
TEST(analyzer_logger_map_parse) {
    Bytes b(0x38 + 2 * 16, 0);
    std::memcpy(b.data(), "LGM0", 4);
    auto f32 = [&](size_t o, float v) { std::memcpy(b.data() + o, &v, 4); };
    auto u32w = [&](size_t o, u32 v) { std::memcpy(b.data() + o, &v, 4); };
    u32w(0x10, 192), u32w(0x14, 128);
    f32(0x18, 10), f32(0x1C, 20), f32(0x20, 0.5f), f32(0x24, 0);
    u32w(0x30, 0), u32w(0x34, 2);
    f32(0x38, 1), f32(0x3C, 2), f32(0x40, 3);
    f32(0x48, 4), f32(0x4C, 5), f32(0x50, 6);
    std::optional<LoggerMap> m = LoggerMap::parse(b);
    CHECK(m && m->points.size() == 2 && m->points[1][2] == 6 && m->scale == 0.5f);
    if (m) {
        m->width = 192, m->height = 128;
        auto [px, py] = m->project(10, 20);
        CHECK(px == 96 && py == 64);
        // Turned maps: the origin comes off after the rotation (the picture's centre is the
        // world point that rotates onto (ox, oz)).
        m->rot = 3.14159265f;
        auto [qx, qy] = m->project(-10, -20);
        CHECK(std::fabs(qx - 96) < 1e-3f && std::fabs(qy - 64) < 1e-3f);
    }
    b[0] = 'X';
    CHECK(!LoggerMap::parse(b));
}

// LGM0 on the disc: every map's own sector points (positions along the lap) land on its
// outline through LoggerMap::project, the turned courses too (trialmt -184.5 degrees).
TEST(analyzer_logger_maps_disc) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Vfs vfs(&disc->volume());
    int maps = 0, points = 0, on_outline = 0, trialmt = -1;
    for (const std::string& f : vfs.list_dir("logger")) {
        std::optional<Bytes> b = vfs.read("logger/" + f);
        std::optional<LoggerMap> m = b ? LoggerMap::parse(*b) : std::nullopt;
        if (!m || m->width <= 0) continue;
        ++maps;
        // An outline pixel within 2 px.
        auto near_outline = [&](float u, float v) {
            for (int dy = -2; dy <= 2; ++dy)
                for (int dx = -2; dx <= 2; ++dx) {
                    int x = int(std::lround(u)) + dx, y = int(std::lround(v)) + dy;
                    if (x < 0 || y < 0 || x >= m->width || y >= m->height) continue;
                    u32 c = m->rgba[size_t(y) * size_t(m->width) + size_t(x)];
                    if ((c >> 24) > 64 && (c & 0xFF) + ((c >> 8) & 0xFF) + ((c >> 16) & 0xFF) > 60) return true;
                }
            return false;
        };
        int hits = 0;
        for (const auto& p : m->points) {
            auto [u, v] = m->project(p[0], p[2]);
            hits += near_outline(u, v);
        }
        points += int(m->points.size()), on_outline += hits;
        if (f == "trialmt") trialmt = hits;
    }
    CHECK(maps == 81);
    CHECK(on_outline * 100 >= points * 95); // 1347 of 1411 (the old GTCM-style formula: 1109)
    CHECK(trialmt == 19);
}
