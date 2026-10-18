// Licence tests and demonstrations: the test's judge (LicenseRun) every tick, the medal,
// and the disc's demonstration runs (quick menu DEMO, the licence menu's preview box).
// (Race screen code map: docs/dev/RACE.md.)

#include "app/screen_race_impl.h"

namespace gt3r::race_screen {

sim::ControlInputs RaceScreen::demo_inputs(size_t i) const {
    sim::ControlInputs in = disc_demo_->inputs(i);
    in.steer = kDemoSteerSign * in.steer;
    return in;
}

void RaceScreen::start_demo() {
    if (!track_ || !vehicle_) return;
    pre_demo_run_ = finished_recording();
    quick_ = false;
    restart();
    demo_ = true;
    demo_tick_ = 0;
    if (disc_demo_) demo_judge_.begin(license_.rules());
    demo_off_ = false;
    drive_camera_ = camera_.mode() == CameraMode::Tv ? drive_camera_ : camera_.mode();
    camera_.set_mode(CameraMode::Tv);
    director_.reset();
}

// Licence quick menu: the demonstration runs live in the preview box behind the menu.
void RaceScreen::start_preview() {
    if (!track_ || !vehicle_ || preview_) return;
    // After a run, GT3's licence menu replays that run in the box, looping
    // (presentation.md A7); with no run, the disc's demonstration.
    if (finish_menu_shown_ && recorder_.tick() > 0) {
        pre_demo_run_ = finished_recording();
        preview_ = run_preview_ = true;
        start_playback(*pre_demo_run_);
        quick_ = finish_menu_shown_ = true; // (the playback resets the run state)
        return;
    }
    if (!disc_demo_) return;
    pre_demo_run_ = finished_recording();
    restart();
    demo_ = true, preview_ = true, quick_ = true;
    demo_tick_ = 0;
    demo_judge_.begin(license_.rules());
    demo_off_ = false;
    drive_camera_ = camera_.mode() == CameraMode::Tv ? drive_camera_ : camera_.mode();
    camera_.set_mode(CameraMode::Tv);
    director_.reset();
}

void RaceScreen::stop_preview(App& app) {
    if (!preview_) return;
    if (run_preview_) {
        run_preview_ = preview_ = false;
        replaying_ = false;
        bool q = quick_;
        restart();
        quick_ = q;
        camera_.set_mode(drive_camera_);
        if (pre_demo_run_) recorder_.assign(std::move(*pre_demo_run_));
        pre_demo_run_.reset();
        silence(app);
        return;
    }
    preview_ = false;
    demo_ = false;
    bool q = quick_;
    restart();
    quick_ = q;
    camera_.set_mode(drive_camera_);
    if (pre_demo_run_) recorder_.assign(std::move(*pre_demo_run_));
    pre_demo_run_.reset();
    silence(app);
}

void RaceScreen::end_demo(App& app) {
    demo_ = false;
    restart();
    camera_.set_mode(drive_camera_);
    if (pre_demo_run_) recorder_.assign(std::move(*pre_demo_run_));
    pre_demo_run_.reset();
    silence(app);
    quick_ = true;
}

// "L0B0003" -> "B-3", "LIA0007" -> "IA-7".
std::string RaceScreen::license_title(const std::string& code) {
    if (code.size() < 7) return code;
    std::string cls = code.substr(1, 2);
    if (cls[0] == '0') cls = cls.substr(1);
    return cls + "-" + std::to_string(std::atoi(code.substr(3).c_str()));
}

void RaceScreen::begin_license() {
    license_times_ = {};
    const std::string& demo_code = license_event_.empty() ? theater_demo_ : license_event_;
    if (!demo_code.empty() && !disc_demo_tried_ && app_ && app_->vfs()) { // the disc's demonstration run
        disc_demo_tried_ = true;
        std::string err;
        if (std::optional<Bytes> b = app_->vfs()->read("demo/" + demo_code)) disc_demo_ = DemoReplay::parse(*b, &err);
        if (disc_demo_) LOG_INFO("Demo %s: %zu frames, %.3f s", demo_code.c_str(), disc_demo_->size(),
                                 disc_demo_->time_ms / 1000.0);
        else LOG_WARN("Demo %s: no demo run (%s)", demo_code.c_str(), err.c_str());
    }
    if (license_event_.empty()) return;
    const data::Event* ev = db_.event(license_event_);
    if (!ev) return;
    license_times_ = license_times(*ev);
    LicenseRun::Rules r;
    r.kind = license_times_.kind;
    r.laps = std::max<int>(1, ev->laps);
    if (const track::Track* t = track_ ? track_->runtime() : nullptr) r.goal_m = t->road().start_distance;
    r.standing = !tt_start_.flying();
    license_.begin(r);
    license_progress_ = 0;
    license_dist_valid_ = false;
}

void RaceScreen::update_license(const sim::VehicleState& s, double dt) {
    LicenseRun::Step st;
    st.started = track_->lap_started();
    st.dt = dt;
    st.elapsed = st.started ? track_->current_lap_time(race_time_) : 0;
    float prog = st.started && track_->laps_completed() == 0 ? track_->lap_progress() * track_->length() : license_progress_;
    st.progress_prev = license_progress_;
    st.progress = prog;
    license_progress_ = prog;
    st.laps_completed = track_->laps_completed();
    for (double t : lap_times_) st.laps_time += t;
    st.speed_mps = s.speed_kmh / 3.6f;
    // The original's clock: ticks since GO completed before this one (its stop check runs
    // before the clock's per-tick advance); the body's 3-D speed.
    st.clock_ticks = tt_hold_ > 0 ? -1 : std::llround(race_time_ * 60.0) - 1;
    st.speed3d_mps = std::sqrt(s.velocity.x * s.velocity.x + s.velocity.y * s.velocity.y + s.velocity.z * s.velocity.z);
    // Unwrapped distance from the line (negative behind it) for the progress events.
    st.lap_length = track_->length();
    st.dist = track_->race_distance();
    st.dist_prev = license_dist_valid_ ? license_dist_ : st.dist;
    license_dist_ = st.dist, license_dist_valid_ = tt_hold_ <= 0;
    st.wall_dv = vehicle_->last_impact(); // speed change this step (no other cars here)
    st.pylon = pylon_hit_; // any touch fails (the licence test's failure check, failure bit 0)
    for (int i = 0; i < 4; ++i) st.grounded[i] = s.wheels[size_t(i)].grounded, st.surface[i] = s.wheels[size_t(i)].surface;
    if (preset_.lic_finish > 0 && st.started && st.elapsed >= preset_.lic_finish) license_.force_finish(st.elapsed);
    LicenseRun::State r = license_.step(st);
    if (r == LicenseRun::State::Running) return;
    finished_ = true;
    finish_position_ = r == LicenseRun::State::Finished ? 1 : 0;
    music_license_end_s_ = race_time_, music_license_failed_ = r != LicenseRun::State::Finished; // replay's finish cue
    if (r == LicenseRun::State::Finished) {
        int medal = license_medal(license_times_, license_.time());
        LOG_INFO("Licence %s: finished in %.3f s, medal %d", license_event_.c_str(), license_.time(), medal);
        // The session's best judged run is what GT mode records (a retry or a replay
        // afterwards must not lose a medal already earned).
        // (Not the demonstration: its run is judged through the same code, swapped in.)
        auto rank = [](int m) { return m >= 0 ? m : 3; };
        if (!demo_ && (!best_license_ || rank(medal) < rank(best_license_->medal) ||
                       (rank(medal) == rank(best_license_->medal) && license_.time() < best_license_->time)))
            best_license_ = BestLicense{medal, license_.time()};
        // GT mode: the medal goes into the save at once (also kept if the game is closed
        // before returning to the menus).
        if (!demo_ && from_menu_ && launch_.request.mode == "go_license" && !launch_.request.practice && medal >= 0)
            game::save_license_run(license_event_, medal, license_.time());
        if (preset_.prize >= 0) medal = std::min(preset_.prize, 2); // screenshots: show this medal
        shown_medal_ = medal;
        // The medal model (race/gold|silver|bronze.prm) flies in on the result screen.
        static const char* const kPrm[] = {"race/gold.prm", "race/silver.prm", "race/bronze.prm"};
        prize_.reset();
        if (medal >= 0 && app_ && app_->gpu() && app_->vfs())
            if (std::optional<Bytes> b = app_->vfs()->read(kPrm[medal])) {
                prize_ = std::make_unique<render::PrizeView>(*app_->gpu());
                if (!prize_->load(*b, kPrm[medal])) prize_.reset();
            }
    }
    else LOG_INFO("Licence %s: FAIL (%s)", license_event_.c_str(), license_.failure().c_str());
}

} // namespace gt3r::race_screen
