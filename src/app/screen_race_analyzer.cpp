// The Analyzer (data logger, race/analyzer.h): the quick menu's DATA LOGGER over the
// session's laps, and the Replay Theater's logger mode (two replays re-simulated, REPLAY
// with the second as the ghost).
// (Race screen code map: docs/dev/RACE.md.)

#include "app/screen_race_impl.h"

namespace gt3r::race_screen {

void RaceScreen::load_analyzer(App& app) {
    if (!analyzer_loaded_) analyzer_loaded_ = true, analyzer_.load(*app.vfs(), app.renderer(), hud_.fonts());
    analyzer_.imperial = imperial_;
    // The course's data-logger map (picture, sector points).
    std::optional<LoggerMap> map;
    const std::string label = courses_[size_t(course_sel_)].label;
    if (std::optional<Bytes> b = app.vfs()->read("logger/" + label)) map = LoggerMap::parse(*b);
    const float len = track_ ? track_->length() : 0.0f;
    std::vector<float> sectors;
    if (map && track_) sectors = logger_sectors(*map, [this](float x, float y, float z) { return track_->distance_of({x, y, z}); }, len);
    analyzer_.set_course(app.renderer(), std::move(map), std::move(sectors), len);
}

// Quick menu DATA LOGGER: A the session's last lap, B its best (GRAPH).
void RaceScreen::open_session_analyzer(App& app) {
    load_analyzer(app);
    AnalyzerTrack a, b;
    a.trace = logger_.last(), b.trace = logger_.best();
    a.hz = b.hz = 1.0 / step_.step();
    a.title = b.title = ascii(car_.name);
    analyzer_.set_tracks(std::move(a), std::move(b));
    analyzer_.open(Analyzer::View::Graph);
}

// The theater's Analyzer: re-simulate track A (the replay being played) and track B (the
// second file, same course) to get their laps; REPLAY plays A's lap with B as the ghost.
void RaceScreen::setup_logger(App& app) {
    load_analyzer(app);
    auto dist = [this](const sim::Vec3& p) { return track_->distance_of(p); };
    const float len = track_->length();
    AnalyzerTrack a = analyze_replay(replay_, [&] { return make_vehicle(car_, &db_, *track_); }, dist, len);
    a.title = ascii(car_.name);
    AnalyzerTrack b;
    if (!launch_.request.replay_file2.empty())
        if (std::optional<Replay> rb = read_replay_file(launch_.request.replay_file2); rb && rb->course == replay_.course) {
            CarSetup cb = car_setup_from_database(db_, rb->car);
            if (rb->spec) cb.spec = rb->spec;
            b = analyze_replay(*rb, [&] { return make_vehicle(cb, &db_, *track_); }, dist, len);
            b.title = car_display_name(rb->car);
        }
    const bool whole = replay_.best_lap < 0 || replay_.best_lap_end <= replay_.best_lap_begin;
    logger_begin_ = whole ? 0 : replay_.best_lap_begin;
    logger_end_ = whole ? replay_.ticks() : replay_.best_lap_end;
    ghost_ = b.ghost;
    LOG_INFO("Analyzer: track A %zu samples (%.3f s), track B %zu samples", a.trace.size(), a.trace.time, b.trace.size());
    analyzer_.set_tracks(std::move(a), std::move(b));
    analyzer_.open(launch_.request.logger_view == 1 ? Analyzer::View::Graph : Analyzer::View::Replay);
    logger_seek(app, std::max(0.0f, preset_.lcursor));
    if (preset_.lcursor >= 0) analyzer_.set_cursor(preset_.lcursor);
}

// REPLAY: track A's replay from its lap's start to the sample at distance `d`.
void RaceScreen::logger_seek(App& app, float d) {
    const AnalyzerTrack& a = analyzer_.track(0);
    u32 target = logger_begin_ + (a.valid() ? u32(a.index_at_distance(d)) : 0);
    start_playback(Replay(replay_));
    while (player_.tick() < target && !player_.done()) tick(app, PadState{});
    silence(app);
    render_state_ = prev_ = vehicle_->state();
    camera_.snap(render_state_);
}

// The theater's Analyzer, every frame: true while it runs (the race loop does nothing else).
bool RaceScreen::update_logger(App& app, double dt) {
    logger_acc_ += dt;
    bool open = true;
    while (open && logger_acc_ >= 1.0 / 60.0) {
        logger_acc_ -= 1.0 / 60.0;
        open = analyzer_.tick(menu_keys(app.input().pad(0)));
        if (!quick_sounds_tried_) {
            quick_sounds_tried_ = true;
            quick_sounds_.load(*app.vfs());
        }
        for (const std::string& se : analyzer_.take_sounds()) quick_sounds_.play(app.audio().voices(), se);
    }
    if (!open) {
        leave(app); // back to the Analyzer's top page
        return true;
    }
    if (analyzer_.view() == Analyzer::View::Graph) {
        silence(app);
        return true;
    }
    if (std::optional<float> s = analyzer_.take_seek()) logger_seek(app, *s);
    for (int i = analyzer_.take_camera_presses(); i > 0; --i) next_view(camera_, 0);
    const double rate = analyzer_.playing() ? (analyzer_.slow() ? 0.25 : 1.0) : 0.0;
    int n = rate > 0 ? step_.advance(dt * rate) : 0;
    for (int i = 0; i < n; ++i) {
        if (player_.tick() + 1 >= logger_end_ || player_.done()) { // loop the lap
            logger_seek(app, 0);
            break;
        }
        tick(app, PadState{});
    }
    render_state_ = interpolate(prev_, vehicle_->state(), step_.alpha());
    camera_.set_track_distance(track_->distance_of(render_state_.position));
    camera_.set_lap(track_->laps_completed());
    camera_.update(render_state_, vehicle_->setup(), PadState{}, dt);
    const AnalyzerTrack& a = analyzer_.track(0);
    if (a.valid()) {
        size_t i = std::min<size_t>(player_.tick() > logger_begin_ ? player_.tick() - logger_begin_ : 0, a.trace.size() - 1);
        analyzer_.set_cursor(a.trace.dist[i]);
    }
    if (!analyzer_.playing()) silence(app);
    return true;
}

} // namespace gt3r::race_screen
