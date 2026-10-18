// GT3's quick menus around a run (pre-race and result: layout per mode, entry list,
// Top 3, licence page) and their item actions, the free run before a GT race, and our
// panels opened from them (options, settings, records, the session's data logger).
// (Race screen code map: docs/dev/RACE.md.)

#include "app/screen_race_impl.h"

namespace gt3r::race_screen {

// Free run before a GT race: the course alone (no opponents, no laps); leaving it
// ("Exit" in the pause menu) returns to the pre-race menu with the field rebuilt.
void RaceScreen::start_free_run(App& app) {
    free_run_ = true;
    quick_ = prerace_ = false;
    race_mode_ = false;
    int laps = race_laps_;
    build_vehicle(app);
    race_laps_ = 0;
    free_run_laps_ = laps;
    restart();
}

void RaceScreen::end_free_run(App& app) {
    free_run_ = false;
    race_mode_ = true;
    build_vehicle(app);
    race_laps_ = free_run_laps_;
    if (field_) field_->set_laps(race_laps_);
    restart();
    open_prerace_menu(app);
}

// The quick-menu layout of this run, as the original picks it per mode (arcade
// race, free run, time trial, machine test, licence, GT race, championship, 2P;
// menus spec §11).
// `before`: the menu before the first run (licence, machine test, GT race, time trial,
// free run); the arcade race and 2P battle run first.
std::string RaceScreen::quick_menu_layout(bool before) const {
    const menu::flow::GameRequest& rq = launch_.request;
    if (!license_event_.empty()) return "qm_gt_license1.pmb";
    if (two_player_) return "qm_arc_2p_result1.pmb";
    if (from_menu_ && rq.mode == "go_machine_test")
        return rq.race_code == "M1000" ? "qm_gt_mt_10001.pmb" : rq.race_code == "MMAX" ? "qm_gt_mt_max1.pmb"
                                                                                        : "qm_gt_mt_4001.pmb";
    if (!field_) return race_laps_ > 0 ? "qm_arc_tt_ltd1.pmb" : "qm_arc_tt_free1.pmb";
    if (before) return championship() ? "qm_gt_event_champ1.pmb" : "qm_gt_event1.pmb";
    if (championship()) return "qm_gt_result_champ1.pmb";
    if ((from_menu_ && rq.mode.rfind("go_", 0) == 0) || preset_.gt_result) return "qm_gt_result1.pmb";
    return "qm_arc_result1.pmb";
}

// Whether this run opens with its quick menu (quick_menu_layout(true)).
bool RaceScreen::menu_before_run() const {
    const std::string& m = launch_.request.mode;
    return m == "go_race" || m == "go_machine_test" || m == "go_license" || m == "go_free_run" ||
           m == "time_trial" || m == "free_run";
}

// Licence quick menu: the test's name, target times and description panel, and the
// preview box (the disc's demonstration before a run, the run after it).
void RaceScreen::fill_license_menu(App& app) {
    start_preview();
    quick_menu_.set_text("qm_license_name", license_title(license_event_));
    std::string page = license_event_;
    for (char& c : page) c = char(std::tolower(static_cast<unsigned char>(c)));
    quick_menu_.add_page_layout(*app.vfs(), app.renderer(), page); // the test's description panel
    static const char* const kW[] = {"qm_license_time_gold", "qm_license_time_silver", "qm_license_time_bronz"};
    for (int i = 0; i < 3; ++i)
        if (license_times_.ms[size_t(i)] > 0) quick_menu_.set_text(kW[i], Hud::format_time(license_times_.ms[size_t(i)] / 1000.0));
}

// The GT Mode ranking this run's records go to (licence, machine test, arcade time trial),
// or "" (free run, Quick race practice, races).
std::string RaceScreen::records_key() const {
    const menu::flow::GameRequest& rq = launch_.request;
    if (!from_menu_ || rq.practice) return {};
    if (!license_event_.empty()) return "lic:" + license_event_;
    if (rq.mode == "go_machine_test") return "mt:" + rq.race_code;
    if (rq.mode == "time_trial" && !rq.race_code.empty()) return "at:" + rq.race_code;
    return {};
}

// Time trial / machine test quick menus: "Top 3" = entries 0..2 of the ranking (name and
// time, the top speed for MMAX), before and after every run (time trial and machine test
// read the record again before each quick menu).
void RaceScreen::fill_top3() {
    const menu::flow::GameRequest& rq = launch_.request;
    if (!license_event_.empty() || (rq.mode != "go_machine_test" && rq.mode != "time_trial")) return;
    std::string key = records_key();
    game::GtModeSession* gs = game::gt_mode_session();
    if (key.empty() || !gs) return;
    const game::GtMode& gt = gs->gt();
    std::vector<game::RankEntry> top = gt.ranking_top(key, 3);
    for (size_t k = 0; k < 3; ++k) {
        const std::string n = std::to_string(k + 1);
        std::string name, value;
        if (k < top.size()) {
            name = gt.ranking_name(top[k]);
            char b[32];
            if (key == "mt:MMAX") std::snprintf(b, sizeof b, "%.1f km/h", top[k].value);
            value = key == "mt:MMAX" ? std::string(b) : Hud::format_time(top[k].value);
        }
        quick_menu_.set_class_text("qm_rank_name" + n, name);
        quick_menu_.set_class_text("qm_list_car_name" + n, name);
        quick_menu_.set_class_text("qm_rank_time" + n, value);
        quick_menu_.set_class_text("qm_rank_speed" + n, value);
    }
}

void RaceScreen::open_prerace_menu(App& app) {
    quick_ = true;
    prerace_ = true;
    silence(app);
    std::string pmb = quick_menu_layout(true);
    if (!quick_menu_.load(*app.vfs(), app.renderer(), hud_.fonts(), pmb)) {
        quick_ = prerace_ = false; // no layout: straight to the race
        return;
    }
    if (!license_event_.empty()) {
        fill_license_menu(app);
        return;
    }
    // The entry list (the capture of GT3's RACE MENU): every car, no times yet.
    if (field_) {
        std::vector<ResultRow> rows = result_rows();
        for (ResultRow& r : rows) r.time = "--'--.---";
        quick_menu_.set_rows(std::move(rows));
    }
    if (race_laps_ > 0) quick_menu_.set_text("laps", std::to_string(race_laps_) + (race_laps_ == 1 ? " Lap" : " Laps"));
    fill_top3();
}

void RaceScreen::open_quick_menu(App& app) {
    prerace_ = false;
    finish_menu_shown_ = true;
    quick_ = true;
    silence(app);
    std::string pmb = quick_menu_layout(false);
    if (!quick_menu_.load(*app.vfs(), app.renderer(), hud_.fonts(), pmb)) quick_menu_.load(*app.vfs(), app.renderer(), hud_.fonts(), "qm_arc_result1.pmb");
    {
        // The result quick menu lists every finished car's own race time (the reference
        // capture), not the gaps the overlay shows.
        std::vector<ResultRow> rows = result_rows();
        if (field_) {
            std::vector<RaceField::Standing> st = field_->standings();
            for (size_t i = 0; i < rows.size() && i < st.size(); ++i)
                if (st[i].finished) rows[i].time = Hud::format_time(st[i].time);
        }
        quick_menu_.set_rows(std::move(rows));
    }
    if (!license_event_.empty()) fill_license_menu(app); // the test's name and target times
    fill_top3();
    for (const ResultRow& r : result_rows()) LOG_INFO("Race result: %-40s %s", r.name.c_str(), r.time.c_str());
}

void RaceScreen::update_quick_menu(App& app, double dt) {
    if (panel_ != Panel::None) {
        update_panel(app);
        return;
    }
    if (!quick_sounds_tried_) {
        quick_sounds_tried_ = true;
        quick_sounds_.load(*app.vfs());
        quick_sounds_.set_enabled(menu::flow::option_value(&app.config(), "menu_se") != "off");
    }
    QuickMenu::Choice choice = quick_menu_.update(menu_keys(app.input().pad(0)), dt);
    for (const std::string& se : quick_menu_.take_sounds()) quick_sounds_.play(app.audio().voices(), se);
    if (!preset_.qm.empty() && quick_menu_.focus(preset_.qm)) // scripted (screenshots)
        choice = {preset_.qm, "root", quick_menu_.action(preset_.qm)};
    preset_.qm.clear();
    if (choice.empty()) return;
    const std::string& a = choice.item;
    const QmAction act = quick_menu_action(choice.action);
    if (preview_ && choice.page == "root" && act != QmAction::Option && act != QmAction::Records &&
        act != QmAction::Logger)
        stop_preview(app);
    if (choice.page != "root") { // yes/no dialogs (the popup has closed itself)
        if (choice.page == "exit_sure" && act == QmAction::Exit) { // "Retire from championship?" Yes
            abandoned_ = true;
            quick_ = false;
            leave(app);
        } else if (choice.page == "ghost_dlg" && act == QmAction::Yes) {
            load_saved_ghost();
            toast(ghost_.empty() ? "No saved best lap" : "Ghost: " + Hud::format_time(ghost_time_));
        }
        return;
    }
    switch (act) {
    case QmAction::Start:
        quick_ = false;
        prerace_ = false;
        restart();
        break;
    case QmAction::FreeRun: start_free_run(app); break;
    case QmAction::Replay:
        quick_ = false;
        watch_replay();
        replay_from_quick_ = replaying_;
        if (!replaying_) quick_ = true;
        break;
    case QmAction::Demo: start_demo(); break;
    case QmAction::SaveReplay:
    case QmAction::SaveGhost: save_current_replay(); break;
    case QmAction::LoadGhost:
        if (!ghost_.empty() && quick_menu_.open_page("ghost_dlg")) return; // "Update Ghost?"
        load_saved_ghost();
        toast(ghost_.empty() ? "No saved best lap" : "Ghost: " + Hud::format_time(ghost_time_));
        break;
    case QmAction::Records: {
        // The game's records (GT mode keeps them from the results) for licences, machine
        // tests and arcade time trials; else the best laps of the saved replays.
        std::string key = records_key(); // (Quick race practice: not the GT Mode save's)
        if (!key.empty()) {
            app.push(make_records_screen(key));
            return;
        }
        open_records(app);
        break;
    }
    case QmAction::Logger:
        open_session_analyzer(app);
        panel_ = Panel::Logger;
        break;
    case QmAction::Option:
        panel_ = Panel::Options;
        panel_sel_ = 0;
        break;
    case QmAction::Setting:
        if (from_menu_ && launch_.request.mode.rfind("go_", 0) == 0 && launch_spec_ && !launch_.request.practice) {
            // GT mode: the game's car settings menu for the ride car; rebuilt when it closes.
            app.push(make_gt_setting_screen(launch_.request.event.empty() ? launch_.request.race_code
                                                                          : launch_.request.event));
            setting_pending_ = true;
            return;
        }
        panel_ = Panel::Settings;
        panel_sel_ = 0;
        break;
    case QmAction::NextSession:
        quick_ = false;
        leave(app); // GT mode chains the next round
        break;
    case QmAction::SaveGame:
        // The save page (Save game); the save includes this round (the original
        // stores the championship's progress around its save).
        if (from_menu_ && championship() && !launch_.request.practice) {
            app.push(make_game_save_screen(result_latch_.for_leave(build_result())));
            return;
        }
        break;
    case QmAction::Exit: {
        std::string link = quick_menu_.link(a);
        if (!link.empty() && quick_menu_.open_page(link)) return; // "Retire from championship?"
        quick_ = false;
        leave(app);
        break;
    }
    default:
        // An item whose .mbl link opens a page of the layout (championship Exit ->
        // "exit_sure" when chosen by a script).
        if (std::string link = quick_menu_.link(a); !link.empty() && quick_menu_.open_page(link)) return;
        LOG_WARN("Quick menu: item %s has no action (%s)", a.c_str(), choice.action.c_str());
        break;
    }
}

std::vector<OptionRow> RaceScreen::option_rows() const {
    std::vector<OptionRow> rows;
    rows.push_back({"Transmission", controls_.config().automatic ? "AT" : "MT"});
    if (panel_ == Panel::Settings) return rows;
    rows.push_back({"Camera", camera_mode_name(drive_camera_)});
    rows.push_back({"Music", music_on_ ? "ON" : "OFF"});
    rows.push_back({"Vibration", rumble_on_ ? "ON" : "OFF"});
    rows.push_back({"Display", preset_.hud ? "ON" : "OFF"});
    if (two_player_) rows.push_back({"2P split", split_horizontal(*app_) ? "Horizontal" : "Vertical"});
    return rows;
}

void RaceScreen::change_option(App& app, int row, int dir) {
    switch (row) {
    case 0: controls_.config().automatic = !controls_.config().automatic; break;
    case 1: {
        int n = int(CameraMode::Tv); // drive cameras: Chase .. Orbit
        drive_camera_ = CameraMode((int(drive_camera_) + dir + n) % n);
        if (!replaying_) camera_.set_mode(drive_camera_);
        break;
    }
    case 2:
        music_on_ = !music_on_; // music_cues() starts / stops the music
        break;
    case 3: rumble_on_ = !rumble_on_; break;
    case 4: preset_.hud = !preset_.hud; break;
    case 5: preset_.split = split_horizontal(app) ? "vertical" : "horizontal"; break;
    default: break;
    }
}

void RaceScreen::update_panel(App& app) {
    const MenuInput& m = app.menu();
    if (panel_ == Panel::Logger) { // the Analyzer (GRAPH) over the session's laps
        bool open = analyzer_.tick(menu_keys(app.input().pad(0)));
        for (const std::string& se : analyzer_.take_sounds()) quick_sounds_.play(app.audio().voices(), se);
        if (!open) panel_ = Panel::None;
        return;
    }
    if (m.pressed(PadButton::Circle) || m.pressed(PadButton::Triangle) ||
        (panel_ == Panel::Records && m.pressed(PadButton::Cross))) {
        panel_ = Panel::None;
        return;
    }
    if (panel_ == Panel::Options || panel_ == Panel::Settings) {
        int n = int(option_rows().size());
        panel_sel_ = std::clamp(panel_sel_ + m.vertical(), 0, n - 1);
        int h = m.horizontal();
        if (h || m.pressed(PadButton::Cross)) change_option(app, panel_sel_, h ? h : 1);
    }
}

void RaceScreen::open_records(App& app) {
    records_ = course_records(replay_dir(app), courses_[size_t(course_sel_)].label);
    record_names_.clear();
    for (const RecordEntry& r : records_) record_names_.push_back(car_display_name(r.car));
    if (!records_loaded_) records_loaded_ = records_panel_.load(*app.vfs(), app.renderer(), hud_.fonts()) || true;
    panel_ = Panel::Records;
}

void RaceScreen::draw_panel(App& app, const SDL_FRect& dst) {
    SDL_Renderer* r = app.renderer();
    switch (panel_) {
    case Panel::Records:
        records_panel_.draw(r, hud_.fonts(), records_, record_names_, ascii(course_name_), dst);
        break;
    case Panel::Logger:
        analyzer_.draw(r, dst);
        break;
    case Panel::Options:
    case Panel::Settings:
        draw_option_list(r, hud_.fonts(), panel_ == Panel::Options ? "OPTIONS" : "SETTINGS", option_rows(), panel_sel_,
                         "Up/Down select   Left/Right change   Circle back", dst);
        break;
    case Panel::None: break;
    }
}

} // namespace gt3r::race_screen
