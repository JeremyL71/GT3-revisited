// Sound glue: the race music cues, course ambience (CRSS) listeners, the inputs of the
// per-car extra sounds (wind, whine, turbo) and silencing everything.
// (Race screen code map: docs/dev/RACE.md.)

#include "app/screen_race_impl.h"

namespace gt3r::race_screen {

// The race's music cues (music spec §2-3), every frame.
void RaceScreen::music_cues(App& app) {
    RaceMusic::Cues c;
    c.music_on = replaying_ || demo_ || attract_ ? preset_.music && replay_settings_.bgm_on() : music_on_;
    bool lic_replay = !license_event_.empty() && !attract_ && (replaying_ || demo_);
    c.kind = lic_replay                       ? RaceMusic::Kind::LicenseReplay
             : demo_ || attract_              ? RaceMusic::Kind::Demo
             : !license_event_.empty()        ? RaceMusic::Kind::License
             : two_player_                    ? RaceMusic::Kind::TwoPlayer
                                              : RaceMusic::Kind::Race;
    c.paused = paused_;
    c.start_type = field_ ? music_start_type_ : tt_start_.type;
    c.holding = !replaying_ && (field_ ? field_->race_time() < 0 : tt_hold_ > 0);
    c.finished = finished_ || finished2_;
    c.position = 1; // every finish plays the jingle; in 2P the first one is the winner's
    // The fail jingle: a licence FAIL, a licence time slower than bronze (GT3 posts
    // ExamFail for both), a voided time trial.
    c.failed = tt_failed_ || (!license_event_.empty() && (license_.state() == LicenseRun::State::Failed ||
                                                           (license_.state() == LicenseRun::State::Finished && shown_medal_ < 0)));
    c.medal = license_event_.empty() ? -1 : shown_medal_;
    if (lic_replay) { // the replayed / demo run's own end: the judge (demo) or the live run's end time (replay)
        bool demo_end = demo_ && demo_judge_.state() != LicenseRun::State::Running;
        bool replay_end = replaying_ && music_license_end_s_ >= 0 && race_time_ >= music_license_end_s_;
        c.finished = demo_end || replay_end;
        c.failed = demo_end ? demo_judge_.state() == LicenseRun::State::Failed : music_license_failed_;
    }
    c.since_finish = finish_wait_;
    bool results = (field_ || two_player_) && license_event_.empty() && machine_mark_m_ <= 0;
    c.menu = !quick_ ? RaceMusic::Menu::None
             : prerace_ || !results ? RaceMusic::Menu::Short
                                    : RaceMusic::Menu::Result;
    music_.cues(app.audio(), *app.vfs(), c);
}

// CRSS course ambience: one listener per screen, every car's track distance.
void RaceScreen::update_course_ambience(App& app) {
    if (!crss_ok_) return;
    auto listener = [&](const CameraRig& cam, const sim::VehicleState& car) {
        CourseAmbience::Listener l;
        render::Vec3 e = cam.eye();
        sim::Vec3 r = cam_right(cam);
        l.pos = {e.x, e.y, e.z};
        l.velocity = {car.velocity.x, car.velocity.y, car.velocity.z};
        l.right = {r.x, r.y, r.z};
        sim::SurfaceHit h = track_->query().probe(car.position.x, car.position.z, car.position.y + 2.0f);
        l.surface_flags = h.hit ? (h.surface >> 8) : 0;
        return l;
    };
    std::vector<CourseAmbience::Listener> ls{listener(camera_, render_state_)};
    std::vector<float> dist{track_->distance_of(render_state_.position)};
    if (field_) {
        if (field_->has_second()) {
            ls.push_back(listener(camera2_, render_state2_));
            dist.push_back(track_->distance_of(field_->second_state().position));
        }
        for (int i = 0; i < field_->opponents(); ++i) dist.push_back(track_->distance_of(field_->opponent_state(i).position));
    }
    crss_.update(ls, dist);
    crss_player_.update(app.audio().voices(), crss_);
}

RaceScreen::SoundCar RaceScreen::car_sound_facts(const std::string& label) const {
    SoundCar c;
    std::string base = db_.car(label) ? label : "";
    if (base.empty())
        if (const data::CarVariant* v = db_.variant(label)) base = v->car;
    const data::Car* car = base.empty() ? nullptr : db_.car(base);
    if (!car) return c;
    for (size_t i = 0; i < data::kCarPartTables.size(); ++i) {
        int t = data::kCarPartTables[i];
        if (t != int(data::TableId::RacingModify) && t != int(data::TableId::Gear)) continue;
        const ByteSpan* rec = db_.record(data::TableId(t), car->parts[i]);
        if (!rec) continue;
        if (t == int(data::TableId::RacingModify) && rec->size() > 0x39 && (*rec)[0x39])
            c.cd = float((*rec)[0x39]) / 100.0f;
        if (t == int(data::TableId::Gear) && rec->size() > 0x10) c.whine = (*rec)[0x10] != 0;
    }
    return c;
}

// Turbo model inputs for one physics step. Physics does not expose the boost load and
// the stage lag states yet: until VehicleTelemetry has `turbo_load` / `turbo_lag`, the
// load is approximated from the boost (Bcap - 1) over the cap.
CarExtraSounds::Step RaceScreen::extra_step(const sim::Vehicle& v, float throttle) {
    CarExtraSounds::Step st;
    const sim::VehicleTelemetry& t = v.telemetry();
    const sim::TurboSpec& ts = v.spec().engine.turbo;
    st.turbo = ts.on;
    st.throttle = throttle;
    st.torque = t.engine_torque;
    st.load = ts.cap > 1.0f ? std::clamp(t.boost / (ts.cap - 1.0f), 0.0f, 1.0f) : 0.0f;
    st.lag0 = st.lag1 = int(st.load * 6144.0f);
    [&](const auto& tt) {
        if constexpr (requires { tt.turbo_load; }) st.load = tt.turbo_load;
        if constexpr (requires { tt.turbo_lag; }) st.lag0 = int(tt.turbo_lag[0]), st.lag1 = int(tt.turbo_lag[1]);
    }(t);
    return st;
}

CarExtraSounds::Frame RaceScreen::extra_frame(const sim::Vehicle& v, const sim::VehicleState& s, const SoundCar& car,
                                              bool onboard, float x, float z, SoundView view) {
    CarExtraSounds::Frame f;
    const sim::VehicleTelemetry& t = v.telemetry();
    const sim::CarSpec& spec = v.spec();
    f.u = s.speed_kmh / 3.6f;
    [&](const auto& tt) {
        if constexpr (requires { tt.slipstream; }) f.slipstream = tt.slipstream;
    }(t);
    f.cd = car.cd;
    f.whine = car.whine;
    f.clutch = t.clutch > 0.01f ? 1 : 0;
    f.torque = t.engine_torque;
    const std::vector<float>& r = spec.gearbox.ratios;
    if (!r.empty()) {
        f.first_ratio = r[0];
        f.ratio = s.gear >= 1 && size_t(s.gear) <= r.size() ? r[size_t(s.gear - 1)] : s.gear < 0 ? spec.gearbox.reverse : 0.0f;
    }
    // Driven axle speed: rear, front, or the mean of all four (4WD).
    float front = 0.5f * (std::fabs(s.wheels[0].angular_velocity) + std::fabs(s.wheels[1].angular_velocity));
    float rear = 0.5f * (std::fabs(s.wheels[2].angular_velocity) + std::fabs(s.wheels[3].angular_velocity));
    sim::Drive d = spec.drivetrain.drive;
    f.shaft = d == sim::Drive::FF ? front : d == sim::Drive::AWD ? 0.5f * (front + rear) : rear;
    if (onboard) x = 0, z = 1; // the followed car's own sounds sit in front in the in-car view
    f.x = x, f.z = z;
    f.view = view;
    return f;
}

void RaceScreen::silence(App& app) {
    engine_.stop(app.audio().voices());
    engine2_.stop(app.audio().voices());
    opp_engines_.stop(app.audio().voices());
    car_sfx_.stop(sfx_, app.audio().voices());
    car_sfx2_.stop(sfx_, app.audio().voices());
    extra_.stop(sfx_, app.audio().voices());
    extra2_.stop(sfx_, app.audio().voices());
    crss_player_.stop(app.audio().voices());
    sfx_.stop_all(app.audio().voices());
    app.input().set_rumble(1, {});
    app.input().set_trigger_rumble(1, 0, 0);
    app.input().set_rumble(0, {});
    app.input().set_trigger_rumble(0, 0, 0);
}

} // namespace gt3r::race_screen
