// Replay Theater, REPLAY OPTIONS and the Analyzer (docs/dev/RACE.md "Replay Theater"):
// the replay format's theater fields, the replay files the theater lists, the theater's file
// list page, the replay options' state and the Analyzer's sectors and keys.
#include "test_framework.h"
#include "replay_fixtures.h"
#include "base/config.h"
#include "base/file.h"
#include "game/replay_theater.h"
#include "menu/flow/theater_page.h"
#include "race/analyzer.h"
#include "race/backend.h"
#include "race/replay.h"
#include "race/replay_library.h"
#include "race/replay_options.h"
#include "sim/exact/race_record.h"
#include "vfs/vfs.h"

#include <SDL3/SDL.h>

#include <cmath>
#include <cstring>
#include <filesystem>

using namespace gt3r;
using namespace gt3r::race;

namespace {

std::shared_ptr<const sim::exact::RaceRecord> record() { return test::race_record_stub(); }

using menu::flow::key_bit;
using menu::flow::MenuKey;

} // namespace

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
    r.condition = {4321, -12345.5f, 300000}; // v8: the engine condition the run started with
    r.exact = record();
    Replay q = Replay::parse(r.serialize());
    CHECK(q.condition == r.condition);
    CHECK(q.comment == "My best" && q.kind == ReplayKind::GtRace && q.laps == 3 && q.total_time == 301.5);
    CHECK(q.power_ps == 330 && q.car_model == "ni0137" && q.wheel == "rays_01" && q.saved == 1760000000);
    CHECK(q.ticks() == 20 && q.exact != nullptr);
    CHECK(std::string(replay_kind_name(ReplayKind::License)) == "License");
    // A version 11 file without a race record (a run before every race ran on the exact core) is refused.
    r.exact.reset();
    bool too_old = false;
    try {
        Replay::parse(r.serialize());
    } catch (const ReplayTooOld&) {
        too_old = true;
    }
    CHECK(too_old);
}

// Replay format v9: the field's rules as the run had them (a GT race's grid by power-to-weight
// is rebuilt from these in the theater).
TEST(replay_v9_field_rules) {
    Replay r;
    r.course = "trialmt", r.car = "r34_gtr_v_99", r.event = "Gsunday_e_0001";
    r.kind = ReplayKind::ArcadeRace;
    r.grid_by_power = true;
    r.exact = record();
    Replay q = Replay::parse(r.serialize());
    CHECK(q.grid_by_power == std::optional<bool>(true));
    // Not recorded stays not recorded (a replay saved again from the theater).
    r.grid_by_power.reset();
    q = Replay::parse(r.serialize());
    CHECK(!q.grid_by_power);
    // A file of a version before 11 is refused (not played approximately).
    {
        Bytes b = r.serialize();
        const u32 v8 = 8;
        std::memcpy(b.data() + 8, &v8, 4);
        bool too_old = false;
        try {
            Replay::parse(b);
        } catch (const ReplayTooOld&) {
            too_old = true;
        }
        CHECK(too_old);
    }

    // 2P battle: the Options menu's settings and player 2's car as the run had them.
    Replay p2;
    p2.course = "trialmt", p2.car = "r34_gtr_v_99", p2.car2 = "supra_rz_97";
    p2.exact = record();
    for (int i = 0; i < 3; ++i) p2.inputs.push_back(quantize(sim::ControlInputs{})), p2.inputs2.push_back(p2.inputs.back());
    ReplayBattle b;
    b.laps = 5;
    b.catchup = 2;
    b.handicap_m = -35;
    b.tyre_damage = true;
    b.model2 = "to0050", b.wheel2 = "rays_02", b.paint2 = 4;
    p2.battle = b;
    q = Replay::parse(p2.serialize());
    CHECK(q.battle.has_value() && q.inputs2.size() == 3);
    if (q.battle) {
        const ReplayBattle& qb = *q.battle;
        CHECK(qb.laps == 5 && qb.catchup == 2);
        CHECK(qb.handicap_m == -35 && qb.tyre_damage && qb.model2 == "to0050" && qb.wheel2 == "rays_02" && qb.paint2 == 4);
    }
    p2.battle->catchup = 1;
    q = Replay::parse(p2.serialize());
    CHECK(q.battle && q.battle->catchup == 1);
    // Not recorded (an older 2P replay saved again) stays not recorded.
    p2.battle.reset();
    q = Replay::parse(p2.serialize());
    CHECK(!q.battle && q.car2 == "supra_rz_97" && q.inputs2.size() == 3);
}

// The theater's storage: one folder, newest first, delete, Edit file name, info texts.
TEST(replay_library_files) {
    const std::string data = test::temp_dir("theater").string();
    const std::string dir = replay_dir(data);
    CHECK(dir == data + "/replays" || dir == data + "replays");
    Replay r;
    r.course = "trialmt", r.car = "r34_gtr_v_99";
    for (int i = 0; i < 60; ++i) r.inputs.push_back(quantize(sim::ControlInputs{}));
    r.exact = record();
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
    CHECK(e.size() == 1 && e[0].deletable && e[0].playable && e[0].text("course_id") == "trialmt" && e[0].text("comment") == "Fast lap");
    // A file of an older version: listed with the reason, not playable, deletable.
    {
        Bytes old = r.serialize();
        const u32 v10 = 10;
        std::memcpy(old.data() + 8, &v10, 4);
        CHECK(write_host_file(dir + "/trialmt_r34_gtr_v_99_20250101-120000.gt3replay", old));
        int too_old = 0;
        for (const ReplayEntry& le : list_replay_entries(dir)) too_old += le.too_old;
        CHECK(too_old == 1);
        bool listed = false;
        for (const menu::flow::TheaterEntry& t : game::replay_theater_entries(0, dir, nullptr))
            if (!t.playable) listed = t.deletable && t.text("comment") == "Recorded with an older version";
        CHECK(listed);
    }
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
TEST(analyzer_sectors_and_keys) {
    const float L = 400;
    // Sectors from the logger map's points, sorted, deduplicated, starting at 0.
    LoggerMap m;
    m.points = {{0, 0, 250}, {0, 0, 100}, {0, 0, 102}, {0, 0, 399.5f}};
    std::vector<float> s = logger_sectors(m, [](float, float, float z) { return z; }, L);
    CHECK(s.size() == 3 && s[0] == 0 && s[1] == 100 && s[2] == 250);
    Analyzer a;
    a.set_course(nullptr, std::nullopt, s, L);
    CHECK(a.sector_at(0) == 0 && a.sector_at(99) == 0 && a.sector_at(100) == 1 && a.sector_at(399) == 2);
    // Keys: R1 jumps to the next sector, L1 back; Triangle leaves.
    AnalyzerTrack t;
    for (int i = 0; i < 300; ++i) {
        t.trace.dist.push_back(float(i) * L / 300), t.trace.speed_kmh.push_back(100), t.trace.throttle.push_back(1);
        t.trace.brake.push_back(0), t.trace.x.push_back(0), t.trace.z.push_back(float(i));
    }
    t.trace.time = 5.0;
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

// The data logger's bins (GT3's Analyzer pass, LoggerBinner): a car at 10 m/s, 1 m a step, half throttle, from 3 m before
// the line on lap 0: bins of half a metre (the lap / 16383 is less) from the line on, each the speed x 40 and the throttle
// x 255 rounded, the drawn position at its end; the first sample counts from at most 0 m; the smoothing keeps a constant.
TEST(analyzer_logger_bins) {
    namespace ex = sim::exact;
    using ee::F;
    LoggerBinner b(F(1000.0f));
    CHECK(b.width() == F(0.5f));
    ex::Car c;
    c.laps = 0;
    for (int k = 0; k < 30; ++k) {
        const float d = float(k) - 3.0f; // the distance into lap 1
        c.laps = d < 0 ? 0 : 1;
        c.f3e4 = F(d < 0 ? d + 1000.0f : d);
        c.speed_abs = F(10.0f);
        c.throttle_pedal = F(0.5f), c.brake_pedal = F(0.0f);
        c.gear = 3;
        c.flow = F(0.0f);
        c.draw_pos_prev = c.draw_pos;
        c.draw_pos = {F(d), F(0.0f), F(2.0f * d)};
        b.add(c, double(k) / 60.0, u32(k + 1));
    }
    b.finish();
    const std::vector<LoggerBin>& bins = b.bins();
    CHECK(bins.size() == 52); // (26 m from the line in half metres)
    if (bins.size() == 52) {
        CHECK(bins[10].gear() == 3 && (bins[10].word >> 3) == 400 && bins[10].throttle == 128 && bins[10].brake == 0);
        CHECK(bins[10].x == 5 && bins[10].z == 11); // (the end of bin 10: 5.5 m, z 11 m, whole metres)
        CHECK(b.start_frame() == 4); // (the step that reached the line)
    }
    const AnalyzerTrack t = logger_track(b, nullptr, 60.0);
    CHECK(t.trace.size() == 52 && std::fabs(t.trace.speed_kmh[20] - 36.0f) < 0.01f);
    CHECK(std::fabs(t.times[1] - 1.0 / 60.0) < 1e-6); // (1 m from the line: one step on)
    CHECK(t.frame_at_distance(2.0f, 0) == 6);
}
