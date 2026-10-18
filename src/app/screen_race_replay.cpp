// Replays: the run's recording, playback (watch / restart / stop / save), the replay view
// mode and REPLAY OPTIONS (items, followed car, mode indicator) and the director's
// per-shot colour filter.
// (Race screen code map: docs/dev/RACE.md.)

#include "app/screen_race_impl.h"

namespace gt3r::race_screen {

Replay RaceScreen::finished_recording() const {
    Replay r = recorder_.replay();
    r.final_position = vehicle_->state().position;
    // What the theater's info panels show, and the car itself (replay format v6).
    r.kind = replay_kind();
    r.laps = int(lap_times_.size());
    double sum = 0;
    for (double t : lap_times_) sum += t;
    r.total_time = !license_event_.empty() ? (license_.state() == LicenseRun::State::Finished ? license_.time() : -1)
                   : lap_times_.empty()    ? -1
                   : field_                ? sum
                                           : lap_times_.back();
    r.spec = car_.spec;
    r.car_model = launch_model_;
    if (from_menu_) r.wheel = launch_.request.car_wheel;
    if (car_.spec) r.power_ps = game::spec_power_ps(*car_.spec);
    else if (std::optional<sim::CarSpec> sp = sim::build_car_spec(db_, car_.label)) r.power_ps = game::spec_power_ps(*sp);
    return r;
}

ReplayKind RaceScreen::replay_kind() const {
    if (online_) return ReplayKind::Online;
    if (two_player_) return ReplayKind::TwoPlayer;
    if (!license_event_.empty()) return ReplayKind::License;
    if (!from_menu_) return field_ ? ReplayKind::ArcadeRace : ReplayKind::TimeTrial;
    const std::string& m = launch_.request.mode;
    if (launch_.request.practice || m == "go_free_run" || m == "go_machine_test") return ReplayKind::Practice;
    if (m.rfind("go_", 0) == 0) return ReplayKind::GtRace;
    return m == "single_race" ? ReplayKind::ArcadeRace : ReplayKind::TimeTrial;
}

// Leaves a replay: back to the quick menu when it was opened from there (or is the 2P
// post-race replay), else drive again.
void RaceScreen::end_replay(App& app) {
    if (replay_from_quick_) {
        replay_from_quick_ = false;
        post_replay_ = false;
        replaying_ = false;
        camera_.set_mode(drive_camera_);
        silence(app);
        if (!finish_menu_shown_) open_quick_menu(app);
        else quick_ = true;
    } else {
        stop_playback();
    }
}

void RaceScreen::watch_replay() {
    if (replaying_ || recorder_.tick() == 0) return;
    if (finished_) latch_result(); // the replay re-runs the field: keep the live run's result
    start_playback(finished_recording());
}

void RaceScreen::start_playback(const Replay& r) {
    replay_ = r;
    player_ = ReplayPlayer(&replay_);
    replaying_ = true;
    if (race_mode_ && app_) { // replays of races re-run the whole field from the grid
        race_event_ = replay_.event;
        race_opponents_ = replay_.opponents;
        paint_seed_override_ = replay_.paint_seed;
        build_vehicle(*app_);
    }
    // As the live start: moving starts, else placed at rest without the spawn hold (spec §9).
    if (replay_.start_speed > 0) vehicle_->reset_moving(replay_.start, replay_.start_speed);
    else vehicle_->place(replay_.start);
    reset_run_state();
    drive_camera_ = camera_.mode();
    camera_.set_mode(CameraMode::Tv);
    director_.reset();
    LOG_INFO("Replay: %s / %s, %u ticks (%.1f s)", replay_.course.c_str(), replay_.car.c_str(), replay_.ticks(),
             replay_.duration());
}

void RaceScreen::stop_playback() {
    replaying_ = false;
    camera_.set_mode(drive_camera_);
    restart();
}

void RaceScreen::save_current_replay() {
    if (!app_) return;
    Replay r = replaying_ ? replay_ : finished_recording();
    if (r.ticks() == 0) return;
    std::optional<std::string> path = save_replay(r, replay_dir(*app_));
    toast(path ? "Replay saved" : "Could not save the replay");
}

// REPLAY OPTIONS ITEM: the HUD parts shown during replays (default none). Easy meter, time,
// map and tyres are the race HUD's own parts; car name, steering and accel / brake are
// drawn here as simple native gauges (the original's steering and accel / brake
// monitor art is not reproduced).
void RaceScreen::draw_replay_items(Ui& ui, const SDL_FRect& rect) {
    const u8 it = replay_settings_.items;
    if (!it) return;
    HudData d = hud_data(0);
    d.show_meters = it & ReplaySettings::EasyMeter;
    d.show_times = it & ReplaySettings::Time;
    d.show_map = d.show_map && (it & ReplaySettings::Map);
    d.show_tires = d.show_tires && (it & ReplaySettings::Tire);
    d.show_rank = false;
    d.wrong_way = d.auto_drive = false;
    d.countdown = d.finish_t = -1;
    d.event_text.clear(), d.intro_title.clear();
    if (d.show_meters || d.show_times || d.show_map || d.show_tires) {
        if (original_res_) {
            d.four_three = true;
            hud_.draw(ui.renderer(), d, &rect);
        } else {
            hud_.draw(ui.renderer(), d);
        }
    }
    const float x0 = original_res_ ? rect.x + 16 : 16, y0 = float(Ui::kHeight) - 60;
    if (it & ReplaySettings::CarName) ui.text(x0, original_res_ ? rect.y + 12 : 12, d.car_name, colors::kText);
    if (it & ReplaySettings::Steering) { // a centred bar: the steering angle
        const float w = 120, cx = x0 + w / 2, y = y0;
        ui.fill(x0, y, w, 6, Color{0, 0, 0, 140});
        float st = std::clamp(last_in_.steer, -1.0f, 1.0f) * w / 2;
        ui.fill(st < 0 ? cx + st : cx, y + 1, std::fabs(st), 4, Color{255, 200, 0, 230});
    }
    if (it & ReplaySettings::Accel) { // throttle (green) and brake (red) bars
        const float w = 120, y = y0 + 10;
        ui.fill(x0, y, w, 4, Color{0, 0, 0, 140});
        ui.fill(x0, y, w * std::clamp(last_in_.throttle, 0.0f, 1.0f), 4, Color{80, 220, 80, 230});
        ui.fill(x0, y + 6, w, 4, Color{0, 0, 0, 140});
        ui.fill(x0, y + 6, w * std::clamp(last_in_.brake, 0.0f, 1.0f), 4, Color{230, 60, 50, 230});
    }
}

// REPLAY OPTIONS changed (or L1 / R1): store them and apply sound and the view mode.
void RaceScreen::apply_replay_settings(App& app, const ReplaySettings& rs) {
    const bool mode_changed = rs.mode != replay_settings_.mode;
    replay_settings_ = rs;
    replay_settings_.save(app.config());
    app.save_config();
    replay_se_ = rs.se;
    if (mode_changed) {
        director_.reset();
        last_shot_ = -2;
        shot_filter_ = {};
        replay_focus_shot_ = -2;
    }
}

// TARGET: MANUAL keeps the car chosen with Left / Right, TOP follows the leader, AUTO picks
// at every new shot the car closest behind another (a battle), else the leader (M).
void RaceScreen::update_replay_focus() {
    const int target = replay_settings_.current_target();
    const int n = field_->opponents();
    replay_focus_ = std::clamp(replay_focus_, -1, n - 1);
    if (target == 0) return;
    std::vector<RaceField::Standing> st = field_->standings();
    auto leader = [&] {
        for (const RaceField::Standing& x : st)
            if (x.car >= -1) return x.car;
        return -1;
    };
    if (target == 1) {
        replay_focus_ = leader();
        return;
    }
    int shot = camera_.replay_record() >= 0 ? camera_.replay_record() : 1000 + camera_.tv_index();
    if (shot == replay_focus_shot_) return;
    replay_focus_shot_ = shot;
    int best = leader();
    double gap = 1e9;
    for (size_t i = 1; i < st.size(); ++i)
        if (st[i].car >= -1 && st[i - 1].distance - st[i].distance < gap) gap = st[i - 1].distance - st[i].distance, best = st[i].car;
    replay_focus_ = gap < 60.0 ? best : leader();
}

// The replay view mode indicator: L2 icon, the mode's icon, R2 icon side by side (1 px
// apart), alpha min(1, 4 t^2) while its timer runs out (as in GT3; its place
// on screen is M: bottom left).
void RaceScreen::draw_mode_display(Ui& ui) {
    if (mode_display_t_ <= 0) return;
    static const char* const kNames[4] = {"replay_mode_traditional", "replay_mode_avantgarde", "replay_mode_broadcast",
                                          "replay_mode_crsline"};
    float a = float(std::min(1.0, 4.0 * mode_display_t_ * mode_display_t_));
    hud_.draw_images_row(ui.renderer(), {"replay_mode_l2", kNames[std::clamp(replay_settings_.mode, 0, 3)], "replay_mode_r2"},
                         28, 397, a, original_res_ ? &frame_rect_ : nullptr);
}

void RaceScreen::update_shot_filter() {
    // Only the attract demo: GT3 starts a replay in the view mode stored in the options
    // (default 0, the GTRC trackside cameras with no grade rolls); the demo (replay mode 2)
    // picks the mode at random, and only mode 1's director rolls the filter (presentation.md §G).
    bool tv = (attract_ || ((replaying_ || (theater_ && demo_)) && replay_settings_.mode == 1)) &&
              camera_.mode() == CameraMode::Tv;
    if (!tv) {
        shot_filter_ = {};
        last_shot_ = -2;
        return;
    }
    int shot = camera_.replay_record() >= 0 ? camera_.replay_record() : 1000 + camera_.tv_index();
    if (shot == last_shot_) return;
    last_shot_ = shot;
    auto r = [this] { return filter_rng_.rand01(); };
    ShotFilter& f = shot_filter_;
    if (f.on) f.on = r() < 0.8f;
    else if (r() < 0.1f) {
        static const float kPreset[5][3] = {{1, 0, 0}, {0, 0, 1}, {0, 1, 1}, {0.3f, 0.3f, 0.3f}, {1, 1, 1}};
        int p = int(5 * r()) % 5;
        for (int i = 0; i < 3; ++i) f.curve[i] = kPreset[p][i];
        float q = r();
        f.saturation = q < 0.7f ? 1.0f : q < 0.85f ? r() : 0.0f;
        f.on = true;
    }
    f.levels = r() < 0.25f;
    if (debug_) LOG_INFO("shot filter: shot %d on %d levels %d", shot, int(f.on), int(f.levels));
    if (f.levels) {
        float a = r(), b = r();
        f.black = a * 0.175f, f.white = 0.2f + 0.8f * b;
        if (r() < 0.05f) std::swap(f.black, f.white); // 0.25 x 0.05 = 1.25 % negative
    } else {
        f.black = 0, f.white = 1;
    }
}

void RaceScreen::apply_shot_filter(render::DrawOptions& o) const {
    const ShotFilter& f = shot_filter_;
    if (!f.on && !f.levels) return;
    const SunParams& e = sun_.params();
    for (int i = 0; i < 3; ++i) o.grade[i] = f.on ? f.curve[i] : (e.grade ? e.grade_g[size_t(i)] : 0.5f);
    o.grade[3] = std::max(f.on ? f.saturation : (e.grade ? e.saturation : 1.0f), 1e-3f);
    o.grade_black = f.black, o.grade_white = f.white;
    o.grade_saturation_first = true;
}

} // namespace gt3r::race_screen
