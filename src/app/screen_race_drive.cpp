// The race loop: update_drive() (one frame: menus vs driving, pause, replay keys,
// the fixed-step ticks, finish -> results flow, cameras, sound placement, rumble) and
// tick() (one physics step: player / auto / scripted / demo inputs, pit lane, laps,
// machine and time-trial judging, countdown, fail-safe respawns), plus the per-tick
// helpers (autopilot, gear hint, dirt, pylons, tyre smoke, force feedback).
// (Race screen code map: docs/dev/RACE.md.)

#include "app/screen_race_impl.h"

namespace gt3r::race_screen {

sim::ControlInputs RaceScreen::scripted_inputs(double t) const {
    sim::ControlInputs in;
    in.automatic = controls_.config().automatic;
    size_t pos = 0;
    const std::string& sc = preset_.script;
    while (pos < sc.size()) {
        size_t end = sc.find(';', pos);
        if (end == std::string::npos) end = sc.size();
        std::string item = sc.substr(pos, end - pos);
        pos = end + 1;
        size_t c = item.find(':'), at = item.find('@');
        if (c == std::string::npos || at == std::string::npos || at < c) continue;
        if (std::atof(item.substr(at + 1).c_str()) > t) continue;
        std::string f = item.substr(0, c);
        float v = float(std::atof(item.substr(c + 1, at - c - 1).c_str()));
        if (f == "steer") in.steer = v;
        else if (f == "thr") in.throttle = v;
        else if (f == "brk") in.brake = v;
    }
    in.controller_kind = 2;
    return in;
}

// Change view (R1), remembered for the player's next race like GT3 does.
void RaceScreen::next_view(CameraRig& cam, int player) {
    cam.next_mode();
    if (player == 0) drive_camera_ = cam.mode();
    if (cam.mode() == CameraMode::Bumper || cam.mode() == CameraMode::Chase || cam.mode() == CameraMode::ChaseFar)
        g_last_view[player] = {cam.mode(), cam.mirror()};
}

void RaceScreen::respawn() {
    TrackPose p = track_->respawn_pose(vehicle_->state().position);
    apply_respawn(p);
    recorder_.respawn(p);
}

void RaceScreen::apply_respawn(const TrackPose& p) {
    net_.respawn_ticks = 30; // online: the jump is announced (no teleport warning)
    vehicle_->reset(p);
    prev_ = vehicle_->state();
    track_->lap_teleport(p.position);
    camera_.snap(vehicle_->state());
    offroad_time_ = 0;
}

sim::ControlInputs RaceScreen::autodrive(const sim::VehicleState& s) {
    // Test autopilot with pit=1: from 250 m before the pit entry it follows the pit lane
    // line, so it reaches the entry marker on the pit-lane surface like a player would.
    if (preset_.pit && pit_.available() && !pit_.driving() && pit_.to_entry(s.position) < 250.0f) {
        sim::ControlInputs pin = pit_.lane_inputs(s, 60.0f);
        pin.automatic = true;
        return pin;
    }
    sim::ControlInputs in;
    sim::Vec3 ahead = track_->point_ahead(s.position, 9.0f + s.speed_kmh * 0.12f);
    sim::Vec3 fwd = rotate(s.orientation, {0, 0, -1}), right = rotate(s.orientation, {1, 0, 0});
    sim::Vec3 d = ahead - s.position;
    float ang = std::atan2(d.dot(right), d.dot(fwd));
    in.steer = std::clamp(ang * 1.7f, -1.0f, 1.0f);
    // Manual gearbox driven here: the automatic would select reverse after a stop.
    in.automatic = false;
    const sim::VehicleSetup& su = vehicle_->setup();
    if (++auto_shift_ >= 15) { // at most one shift every 1/4 s
        if (s.gear < 1 || (s.engine_rpm > su.rev_limit_rpm * 0.96f && s.gear < su.forward_gears && s.speed_kmh > 15.0f))
            in.shift_up = true; // (not while held on the grid with the engine revving)
        else if (s.gear > 1 && s.engine_rpm < su.rev_limit_rpm * 0.5f) in.shift_down = true;
        if (in.shift_up || in.shift_down) auto_shift_ = 0;
    }
    // Speed limit from the curvature ahead: v = sqrt(a_lat / k) at each sample, reachable
    // from here with a_brake over the distance to it.
    constexpr float kLat = 7.0f, kBrake = 6.0f, kStep = 6.0f;
    float vmax = 70.0f; // m/s
    sim::Vec3 p0 = track_->point_ahead(s.position, 0.0f), p1 = track_->point_ahead(s.position, kStep);
    for (int i = 2; i <= 24; ++i) {
        sim::Vec3 p2 = track_->point_ahead(s.position, kStep * float(i));
        float h1 = std::atan2(p1.x - p0.x, p1.z - p0.z), h2 = std::atan2(p2.x - p1.x, p2.z - p1.z);
        float dh = std::fabs(std::remainder(h2 - h1, 6.2831853f));
        float seg = std::max(1.0f, std::hypot(p2.x - p1.x, p2.z - p1.z));
        float k = dh / seg;
        float v_corner = k > 1e-4f ? std::sqrt(kLat / k) : 70.0f;
        float dist = kStep * float(i - 1);
        vmax = std::min(vmax, std::sqrt(v_corner * v_corner + 2 * kBrake * dist));
        p0 = p1, p1 = p2;
    }
    float target = std::max(40.0f, vmax * 3.6f);
    if (s.speed_kmh < target) in.throttle = std::min(1.0f, (target - s.speed_kmh) / 10.0f + 0.2f);
    else if (s.speed_kmh > target + 3) in.brake = std::min(1.0f, (s.speed_kmh - target) / 15.0f);
    return in;
}

void RaceScreen::tick(App& app, const PadState& pad) {
    double dt = step_.step();
    sim::ControlInputs in;
    int kind = 2; // pad record kind of this step (spec §4.4)
    if (replaying_) {
        if (player_.done()) return;
        while (const ReplayEvent* e = player_.pending_event())
            if (e->kind == ReplayEvent::Respawn) apply_respawn(e->pose);
            else if (e->kind == ReplayEvent::Respawn2 && field_ && field_->has_second()) field_->reset_second(e->pose);
        in = player_.next_inputs();
        kind = in.controller_kind;
    } else {
        // After the finish GT3 drives the player's car (AUTO DRIVE).
        // Rolling start: the computer drives the player's car for the first 5 s.
        // Hand-over after exactly 300 ticks (counted in ticks, so it cannot drift by a frame).
        bool rolling_drive = rolling_ && field_ && std::llround(field_->race_time() * 60.0) < 300;
        // Flying licence / time-trial start: the computer drives up to the start line.
        bool flying_drive = flying_runup();
        bool auto_drive = preset_.autodrive || finished_ || demo_ || rolling_drive || flying_drive;
        sim::ControlInputs want = auto_drive ? autodrive(vehicle_->state()) : controls_.update(pad, dt);
        if (!replaying_) vehicle_->set_reverse_request(!auto_drive && controls_.reverse());
        // Rolling / flying run-up (spec draft 23b): the regular CPU AI driver on its line until
        // the hand-over, with the automatic box (physics runs the scripted aids path, kind 1).
        const sim::Vehicle* rsv = vehicle_->sim_vehicle();
        // (Also in demonstrations: their recorded frames are consumed but ignored until the line.)
        // Demonstrations: our physics is not bit-exact with the original's, so a long open-loop
        // run drifts off the original's line (the fast arcade time trials 3-22 s after the
        // line); once two wheels leave the road or the run would fail (course-out, wall), the
        // same AI driver takes over to the end (ours).
        bool demo_takeover = demo_ && disc_demo_ && (demo_off_ || demo_judge_.state() == LicenseRun::State::Failed);
        if ((rolling_drive || flying_drive || demo_takeover) && !preset_.autodrive && !finished_ && rsv && track_->runtime()) {
            if (!runup_driver_) {
                if (!runup_course_) runup_course_ = std::make_shared<ai::AiCourse>(*track_->runtime());
                // The built-in set of the course category, no event overrides (ai.md §9.1).
                runup_driver_ = std::make_unique<ai::AiDriver>(
                    runup_course_, ai::DriverParams::builtin(driver_param_set(db_, course_label_for_dirt_)));
                runup_driver_->reset(vehicle_->state());
            }
            const sim::VehicleState& vs = vehicle_->state();
            ai::CarSnapshot me{vs.position, vs.orientation, vs.velocity};
            want = runup_driver_->update(*rsv, {race_time_, std::span<const ai::CarSnapshot>(&me, 1), 0});
            want.automatic = true;
            vehicle_->set_reverse_request(runup_driver_->reverse_request());
        } else {
            runup_driver_.reset();
        }
        if (!preset_.script.empty() && !auto_drive) want = scripted_inputs(script_time_);
        script_time_ += dt;
        // Demonstration of a licence test / time attack: the disc's recorded run (until the
        // takeover above).
        kind = auto_drive ? 0 : controls_.kind();
        // GT3's scripted-driver kind (spec ai.md §1, the "driver kind"): 1 for the rolling / flying
        // run-up until the hand-over, 5 for the stop driver after the finish; pit auto drive
        // overrides it below. Demos and the attract mode stay 0.
        scripted_kind_ = finished_ && !demo_ ? 5 : (rolling_drive || flying_drive || demo_takeover) ? 1 : 0;
        if (demo_ && disc_demo_ && !flying_drive && !demo_takeover) {
            want = demo_inputs(demo_tick_);
            kind = disc_demo_->kind(demo_tick_);
        }
        // Tests (GT3R_RACE demo_drive=1): the disc's recorded inputs drive the player's own,
        // judged run (its tick counter runs from the start of the run).
        if (preset_.demo_drive && !demo_ && disc_demo_ && !finished_ && !flying_drive) {
            want = demo_inputs(demo_drive_tick_);
            kind = disc_demo_->kind(demo_drive_tick_);
            ++demo_drive_tick_;
        }
        if (demo_) ++demo_tick_;
        // After a stop test or a failed test the car is held (the game's stop mode).
        if (finished_ && (tt_failed_ || (!license_event_.empty() &&
                                          (license_.state() == LicenseRun::State::Failed || license_.rules().kind == 1))))
            want = sim::ControlInputs{}, want.brake = 1;
        want.controller_kind = u8(kind);
        in = recorder_.record(want);
        recorder_.set_driver_kind(scripted_kind_);
    }
    // Standing start of a licence test / machine test (start type 1 / 0): the car is held by the
    // race hold (spec §9.3: in place, brakes off, the engine can rev) and the clock starts at the signal.
    bool holding = tt_hold_ > 0;
    float lamp_pedal = in.brake; // the brake lamps follow the pedal, not the start hold
    if (!vehicle_->set_race_hold(holding) && holding) in.brake = 1, in.handbrake = true;
    // Pit lane: the computer drives from the entry marker to the exit (live and in
    // replays, so the stop is re-run); the box stop is the vehicle's own 5 s hold.
    // Replays use the recorded scripted-driver kind (it changes the physics: aids, gearbox).
    if (replaying_) scripted_kind_ = player_.driver_kind();
    if (!(pit_.available() && field_)) vehicle_->set_driver_kind(scripted_kind_);
    if (pit_.available() && field_) {
        PitLane::In pi;
        pi.state = vehicle_->state();
        const sim::Vehicle* sv = vehicle_->sim_vehicle();
        if (sv) {
            const sim::VehicleTelemetry& t = sv->telemetry();
            pi.on_pit_surface = t.on_pit_surface;
            for (int i = 0; i < 4; ++i) pi.want_service = pi.want_service || t.tire_wear[size_t(i)] > sv->tire_wear_params().fresh;
            pi.want_service = pi.want_service && sv->tire_wear_params().on();
            pi.stopping = sv->spawn_hold_active();
        }
        pi.race_over = finished_;
        if (preset_.pit && !replaying_) pi.want_service = sv && sv->tire_wear_params().on();
        PitLane::Out po = pit_.update(pi);
        if (po.entered) {
            pit_msg_text_ = ascii(car_.name) + " Pitting in...";
            pit_msg_time_ = 3.0;
            LOG_INFO("Race: %s (t %.1f)", pit_msg_text_.c_str(), race_time_);
        }
        if (po.begin_stop) vehicle_->begin_pit_stop(po.box), LOG_INFO("Race: pit stop (t %.1f)", race_time_);
        vehicle_->set_driver_kind(po.auto_drive ? std::max(2, int(pit_.state())) : scripted_kind_);
        if (po.auto_drive && !replaying_) {
            // The recorded input is what the simulation sees (replays reproduce the stop).
            sim::ControlInputs pin = po.in;
            pin.controller_kind = 0;
            in = recorder_.replace_last(pin);
        }
    }
    prev_ = vehicle_->state();
    last_in_ = in;
    bool p2 = two_player_ && field_ && field_->has_second();
    if (p2) {
        if (replaying_) {
            last_in2_ = player_.second_inputs();
        } else {
            bool ai2 = preset_.autodrive || finished2_;
            sim::ControlInputs w2 = ai2 ? autodrive(field_->second_state()) : controls2_.update(app.input().race_pad(1), dt);
            field_->set_second_reverse(!ai2 && controls2_.reverse());
            w2.controller_kind = u8(ai2 ? 0 : controls2_.kind());
            last_in2_ = recorder_.record_second(w2);
        }
        field_->set_second_inputs(last_in2_);
    }
    (void)kind;
    // Brake lamps (GT3: a 4-frame history of "brake pedal > 0" per car; lamp factor k / 4).
    brake_lamps_.push(0, lamp_pedal);
    if (p2) brake_lamps_.push(1, last_in2_.brake);
    if (field_)
        for (int i = 0; i < field_->opponents(); ++i) brake_lamps_.push(2 + size_t(i), field_->opponent_brake(i));
    if (online_) online_before_step(dt);
    vehicle_->step(in, dt);
    if (online_) online_after_step();
    if (field_)
        for (int i : field_->take_pit_entries()) {
            pit_msg_text_ = car_display_name(field_->opponent_label(i)) + " Pitting in...";
            pit_msg_time_ = 3.0;
            LOG_INFO("Race: %s (t %.1f)", pit_msg_text_.c_str(), race_time_);
        }
    if (const sim::Vehicle* sv = vehicle_->sim_vehicle()) extra_.step(extra_step(*sv, in.throttle));
    if (p2)
        if (const sim::Vehicle* sv2 = field_->second_vehicle()) extra2_.step(extra_step(*sv2, last_in2_.throttle));
    if (p2) {
        const sim::VehicleState& s2 = field_->second_state();
        RumbleModel::Events ev2;
        for (const sim::WheelState& w : s2.wheels)
            if (w.grounded && track_->is_kerb(w.surface)) ev2.kerb = true;
        ev2.offroad = track_->is_offroad(track_->query().probe(s2.position.x, s2.position.z, s2.position.y + 2.0f));
        ev2.impact = field_->second_impact();
        rumble2_.update(s2, ev2, last_in2_, dt);
        if (!finished2_ && field_->second_finish() >= 0) { // the field's finish rule (race::finishes_at_line)
            finished2_ = true;
            finish_position2_ = field_->second_position();
            if (!finished_) p2_won_ = true;
        }
    }
    if (debug_ && (++debug_ticks_ % 30) == 0) {
        const sim::VehicleState& v = vehicle_->state();
        LOG_INFO("t=%.1f pos(%.1f %.1f %.1f) %.1f km/h gear %d rpm %.0f in(st %.2f th %.2f br %.2f) lap %.3f reverb %.3f "
                 "turbo %d amb(air %.2f city %.2f crowd %.2f sea %.2f) pit %d",
                 race_time_, v.position.x, v.position.y, v.position.z, v.speed_kmh, v.gear, v.engine_rpm, in.steer,
                 in.throttle, in.brake, track_->lap_progress(), ambience_.depth(), extra_.turbo().level(),
                 crss_.channel(0).l + crss_.channel(0).r, crss_.channel(1).l + crss_.channel(1).r,
                 crss_.channel(3).l + crss_.channel(3).r, crss_.channel(4).l + crss_.channel(4).r, int(pit_.state()));
    }
    const sim::VehicleState& s = vehicle_->state();
    if (holding) {
        tt_hold_ -= dt;
        if (tt_hold_ <= 1e-9) { // the signal: the lap clock starts here
            tt_hold_ = 0;
            race_time_ = 0;
            track_->lap_reset(0);
            track_->lap_update(s.position, 0);
        }
    } else {
        race_time_ += dt;
    }
    // Races: the field's race clock (0 = GO, the same for every car).
    bool lap_done = !holding && track_->lap_update(s.position, field_ ? std::max(field_->race_time(), 0.0) : race_time_);
    u32 tick_index = replaying_ ? player_.tick() : recorder_.tick(); // ticks simulated so far

    // Lap bookkeeping for the ghost: frame k = state k ticks after the lap started.
    bool started = track_->lap_started();
    if (started && !was_started_) {
        cur_lap_.clear();
        logger_.begin_lap();
        lap_begin_tick_ = tick_index;
        lap_ticks_ = 0;
    }
    was_started_ = started;
    if (started) {
        cur_lap_.add(s);
        ++lap_ticks_;
        if (!replaying_ && !demo_)
            logger_.sample(track_->lap_progress() * track_->length(), s.speed_kmh, in.throttle, in.brake, s.position.x,
                           s.position.z);
    }
    odometer_m_ += double(s.speed_kmh) / 3.6 * dt;
    update_gear_hint(s);
    update_dirt(dt);
    update_smoke(dt);
    update_pylons(dt);
    if (lap_done && tt_failed_) lap_done = false; // a failed time trial records no lap
    if (lap_done) {
        double t = track_->last_lap_time();
        lap_times_.push_back(t);
        LOG_INFO("Race: lap %zu %s %.3f s", lap_times_.size(), car_.label.c_str(), t);
        if (!replaying_) recorder_.lap(t, lap_begin_tick_, tick_index);
        if (!replaying_ && !demo_) logger_.finish_lap(t);
        if (!replaying_ && !demo_ && !race_mode_ && (ghost_time_ < 0 || t < ghost_time_)) {
            ghost_ = cur_lap_;
            ghost_time_ = t;
            LOG_INFO("Race: new best lap %.3f s, ghost updated (%zu frames)", t, ghost_.size());
        }
        cur_lap_.clear();
        cur_lap_.add(s);
        lap_begin_tick_ = tick_index;
        lap_ticks_ = 1;
    }

    if (!replaying_ && !demo_ && !holding) top_speed_kmh_ = std::max(top_speed_kmh_, s.speed_kmh);
    // Machine test 0-400 m / 0-1000 m: distance from the signal, the crossing timed in the step.
    if (machine_mark_m_ > 0 && !holding && !finished_ && !replaying_ && !demo_) {
        double step_m = double(s.speed_kmh) / 3.6 * dt, before = machine_dist_;
        machine_dist_ += step_m;
        if (before < machine_mark_m_ && machine_dist_ >= machine_mark_m_) {
            double f = step_m > 0 ? (machine_dist_ - machine_mark_m_) / step_m : 0;
            machine_time_ = race_time_ - f * dt;
            finished_ = true;
            finish_position_ = 1;
            LOG_INFO("Machine test: %.0f m in %.3f s (top %.1f km/h)", machine_mark_m_, machine_time_, top_speed_kmh_);
        }
    }
    // Time trial course-out (spec §11): the run fails, no lap is recorded, the car stops.
    bool time_trial = !free_run_ && (!from_menu_ || launch_.request.mode == "time_trial"); // not free run
    if (time_trial && !field_ && license_event_.empty() && machine_mark_m_ <= 0 && !finished_ && !replaying_ && !demo_ &&
        !preset_.autodrive && track_->lap_started()) {
        LicenseRun::Step st;
        float mass = vehicle_->sim_vehicle() ? vehicle_->sim_vehicle()->spec().mass : 1200.0f;
        st.wall_dv = vehicle_->last_impact(); // speed change this step (no other cars here)
        (void)mass;
        for (int i = 0; i < 4; ++i) st.grounded[i] = s.wheels[size_t(i)].grounded, st.surface[i] = s.wheels[size_t(i)].surface;
        const char* why = course_out(st);
        if (!why && pylon_hit_) why = "pylon"; // time trial stops on a rigid-body hit too (failure bit 0)
        if (why) {
            tt_failed_ = true;
            finished_ = true;
            finish_position_ = 0;
            LOG_INFO("Time trial: FAIL (%s) at %.2f s, %.0f km/h, wall dv %.1f m/s", why, race_time_, s.speed_kmh, st.wall_dv);
        }
    }
    // Licence tests: judged every tick (stop box, section goal, laps, failures).
    if (!license_event_.empty() && !finished_ && !replaying_ && !demo_) update_license(s, dt);
    if (demo_ && disc_demo_ && demo_judge_.state() == LicenseRun::State::Running) { // the demonstration's own judge
        std::swap(license_, demo_judge_);
        bool fin = finished_;
        update_license(s, dt);
        finished_ = fin;
        std::swap(license_, demo_judge_);
        // (A course-out in the computer-driven run-up of a flying start does not count: the
        // run is the recording's from the line on.)
        if (demo_judge_.state() == LicenseRun::State::Failed && flying_runup()) demo_judge_.begin(license_.rules());
        // (Ours: the takeover comes as soon as two wheels are off the road, before the run
        // would fail, so the demonstration hardly leaves the track.)
        if (!demo_off_ && !flying_runup()) {
            int off = 0;
            for (const sim::WheelState& w : s.wheels) {
                u8 m = track::surface_material(w.surface);
                off += w.grounded && (m == 2 || m == 3 || m == 4 || m == 6);
            }
            if (off >= 2) demo_off_ = true, LOG_INFO("Demo: AI takeover at step %zu (wheels off the road)", demo_tick_);
        }
        if (demo_judge_.state() != LicenseRun::State::Running)
            LOG_INFO("Licence demo: %s %.3f s (recorded %.3f s; step %zu, %s)",
                     demo_judge_.state() == LicenseRun::State::Finished ? "finished" : "FAILED", demo_judge_.time(),
                     disc_demo_->time_ms / 1000.0, demo_tick_, demo_judge_.failure().c_str());
    }
    // Time trials with a lap count (arcade "time trial" events) end after their laps.
    if (license_event_.empty() && lap_done && !field_ && !finished_ && !replaying_ && !demo_ &&
        race_laps_ > 0 && track_->laps_completed() >= race_laps_) {
        finished_ = true;
        finish_position_ = 1;
        finish_record_rank_ = saved_best_ < 0 || track_->best_lap_time() < saved_best_ ? 0 : -1;
    }
    // Races: the field's finish rule (race::finishes_at_line: the race's laps, the leader has
    // finished, or the first crossing after a time limit); the place is the race order's.
    if (field_ && !finished_ && field_->player_finish() >= 0) {
        finished_ = true;
        track_->lap_freeze(); // no more laps after the finish (the HUD's lap time runs on)
        finish_position_ = field_->player_position();
        if (online_) finish_position_ = online_finish_position();
        if (two_player_ && !finished2_) p1_won_ = true;
        LOG_INFO("Race: finished %d of %d after %d laps (%.3f s)", finish_position_, field_->opponents() + 1,
                 race_laps_, race_time_);
    }

    if ((finished_ || finished2_) && !replaying_) finish_wait_ += dt; // seconds since the finish (results timing)
    // The medal's animation runs from the frame its model starts to fade in.
    if (prize_ && finished_ && result_frame() >= LicenseResultOverlay::model_start(shown_medal_)) prize_->tick(dt);

    RumbleModel::Events ev;
    for (const sim::WheelState& w : s.wheels)
        if (w.grounded && track_->is_kerb(w.surface)) ev.kerb = true;
    sim::SurfaceHit h = track_->query().probe(s.position.x, s.position.z, s.position.y + 2.0f);
    ev.offroad = track_->is_offroad(h);
    ev.impact = vehicle_->last_impact();
    rumble_.update(s, ev, in, dt);
    {
        SoundView view = camera_.onboard() ? SoundView::InCarTarget
                         : camera_.mode() == CameraMode::Tv || replaying_ ? SoundView::Other
                                                                           : SoundView::ChaseTarget;
        // The clash one-shots are scaled by 32/d, d = camera to car.
        auto cam_dist = [](const CameraRig& c, const sim::Vec3& p) {
            render::Vec3 e = c.eye();
            return std::sqrt((p.x - e.x) * (p.x - e.x) + (p.y - e.y) * (p.y - e.y) + (p.z - e.z) * (p.z - e.z));
        };
        if (se_on()) car_sfx_.update(sfx_, app.audio().voices(), s, vehicle_->setup(), view,
                        replaying_ ? 0.0f : vehicle_->wall_impulse(), replaying_ ? 0.0f : vehicle_->car_impulse(),
                        cam_dist(camera_, s.position));
        if (field_ && field_->has_second())
            if (se_on()) car_sfx2_.update(sfx_, app.audio().voices(), field_->second_state(), field_->second_setup(),
                             camera2_.onboard() ? SoundView::InCarTarget : SoundView::ChaseTarget,
                             field_->second_wall_impulse(), field_->second_car_impulse(),
                             cam_dist(camera2_, field_->second_state().position));
        // Force feedback is sent once per frame: keep the strongest hits of the frame's steps.
        ffb_wall_ = std::max(ffb_wall_, std::fabs(vehicle_->wall_impulse()));
        ffb_car_ = std::max(ffb_car_, std::fabs(vehicle_->car_impulse()));
        if (field_ && field_->has_second()) {
            ffb_wall2_ = std::max(ffb_wall2_, std::fabs(field_->second_wall_impulse()));
            ffb_car2_ = std::max(ffb_car2_, std::fabs(field_->second_car_impulse()));
        }
    }
    // Start countdown beeps: count_d at 3, 2, 1, count_s at the start signal.
    if ((field_ || tt_start_.countdown > 0) && !replaying_) {
        double t = field_ ? field_->race_time() : (tt_hold_ > 0 ? -tt_hold_ : race_time_ + 1.0);
        int c = t < 0 ? int(std::ceil(-t)) : 0;
        if (rolling_) c = t < kRollingAutoDrive ? 4 : 0; // START (count_s) at the hand-over only
        if (c != last_count_ && c <= 3) {
            if (c > 0) sfx_.play(app.audio().voices(), "count_d");
            else if (last_count_ > 0) sfx_.play(app.audio().voices(), "count_s");
        }
        last_count_ = c;
    }
    ambience_.tick(app.audio().voices(), h.hit ? (h.surface >> 8) : 0);

    if (replaying_) return; // respawns come from the replay's events
    // The original never respawns a car. Non-original fail-safes: falling out of the
    // world (always), and with [race] respawn = all a player car stuck off the road; the
    // test autopilot (auto=1) gives up on a stuck car.
    bool lost = !std::isfinite(s.position.x) || !std::isfinite(s.position.y) || s.position.y < lowest_ - 50;
    bool player_failsafe = (respawn_all_ && !online_) || preset_.autodrive; // never online
    offroad_time_ = h.hit || !player_failsafe ? 0.0 : offroad_time_ + dt;
    // Auto-drive gives up on a stuck car (crashed into a wall) and respawns.
    double progress = track_->laps_completed() + double(track_->lap_progress());
    if (progress > best_progress_ + 1e-4) best_progress_ = progress, stuck_time_ = 0;
    else if (preset_.autodrive && !(field_ && field_->race_time() < 0)) stuck_time_ += dt; // not while held on the grid
    if (pit_.driving()) stuck_time_ = offroad_time_ = 0; // the pit lane drive and the stop
    if (lost || offroad_time_ > 8.0 || stuck_time_ > 4.0) {
        respawn();
        stuck_time_ = 0;
        best_progress_ = -1;
    }
}

void RaceScreen::update_drive(App& app, double dt) {
    const MenuInput& m = app.menu();
    if (online_ && !online_update(app)) return;
    if (logger_mode_ && update_logger(app, dt)) return;
    if (auto_replay_pending_) { // back from the prize page: the race's replay
        auto_replay_pending_ = false;
        finish_menu_shown_ = false;
        watch_replay();
        replay_from_quick_ = replaying_;
        if (!replaying_) open_quick_menu(app);
        return;
    }
    // Race -> quick menu (presentation.md C1): the last race frame stays up (nothing is
    // simulated) for 10 frames, then a black wash at alpha 0.5 per frame, then a cut.
    if (exit_transition_ >= 0) {
        if (++exit_transition_ > kExitHold + kExitWash) {
            exit_transition_ = -1;
            if (online_) { // no quick menu, replay or restart online: back to the lobby
                leave(app);
                return;
            }
            latch_result(); // the race loop ended: its result as GT mode records it
            // GT-mode race finished (as in the original; presentation.md §F): the race prize
            // page (cup with the prize money counting into the credits, or the consolation
            // prize), then the race's replay starts by itself; its end (or EXIT) opens the
            // result quick menu (presentation.md §F).
            if (from_menu_ && launch_.request.mode == "go_race" && field_ && finished_ && !gt_after_race_done_ &&
                !championship() && !launch_.request.practice) {
                gt_after_race_done_ = true;
                int place = finish_position_ > 0 ? finish_position_ - 1 : -1;
                const std::string code = launch_.request.event.empty() ? launch_.request.race_code : launch_.request.event;
                if (game::queue_race_prize(code, place)) {
                    prize_shown_ = true;
                    app.push(make_presentation_screen());
                }
                auto_replay_pending_ = true;
                silence(app);
                return;
            }
            open_quick_menu(app);
            // GT mode licence test: a run with a medal goes into the test's records at once,
            // and one that makes the ranking asks for the name before the quick menu
            // (as the original does).
            if (!license_event_.empty() && from_menu_ && launch_.request.mode == "go_license" &&
                !launch_.request.practice && !run_recorded_ &&
                license_.state() == LicenseRun::State::Finished && shown_medal_ >= 0) {
                run_recorded_ = any_run_recorded_ = true;
                if (game::add_license_record(launch_.request.race_code, license_.time()) >= 0)
                    app.push(make_presentation_screen());
            }
        }
        return;
    }
    if (attract_) { // any button returns to the title; the demo ends a few seconds after the finish
        bool any = false;
        for (PadButton b : {PadButton::Cross, PadButton::Circle, PadButton::Triangle, PadButton::Square,
                            PadButton::Start, PadButton::Select})
            any = any || m.pressed(b);
        // The demo ends by itself after 1800 steps (the original's timer) or at the finish.
        if (any || (finished_ && finish_wait_ > 5.0) || race_time_ > kAttractSeconds) {
            attract_interrupted_ = any;
            leave(app);
            return;
        }
    }
    if (setting_pending_) { // back from GT mode's settings: the ride car as now set up
        setting_pending_ = false;
        if (std::shared_ptr<const sim::CarSpec> sp = gt_ride_car_spec()) {
            launch_spec_ = sp;
            car_.spec = sp;
            if (build_vehicle(app)) restart();
            toast("Settings applied");
        }
    }
    toast_time_ = std::max(0.0, toast_time_ - dt);
    lamps_.tick(dt);
    pit_msg_time_ = std::max(0.0, pit_msg_time_ - dt);
    if (app.vfs()) music_cues(app);
    if (quick_) {
        if (preview_) { // the demonstration runs behind the menu, looping
            int n = step_.advance(dt);
            for (int i = 0; i < n; ++i) tick(app, PadState{});
            render_state_ = interpolate(prev_, vehicle_->state(), step_.alpha());
            camera_.set_track_distance(track_->distance_of(render_state_.position));
            camera_.update(render_state_, vehicle_->setup(), PadState{}, dt);
            if (run_preview_) {
                if (player_.done() && pre_demo_run_) { // loop the run
                    start_playback(*pre_demo_run_);
                    quick_ = finish_menu_shown_ = true;
                }
            } else if (demo_tick_ > disc_demo_->size() + 180) {
                restart();
                quick_ = true;
                demo_tick_ = 0;
                demo_judge_.begin(license_.rules());
                demo_off_ = false;
            }
        }
        update_quick_menu(app, dt);
        return;
    }
    if (theater_ && demo_ && !paused_ && disc_demo_ && demo_tick_ > disc_demo_->size() + 180) {
        leave(app); // the theater's demonstration ran out: back to the theater
        return;
    }
    if (demo_ && !theater_ && (m.pressed(PadButton::Start) || m.pressed(PadButton::Cross) || m.pressed(PadButton::Circle))) {
        end_demo(app);
        return;
    }
    // The race pad (the player's race layout, buttons in GT3's meaning): its presses this frame.
    const PadState pad = app.input().race_pad(0);
    const u16 race_pressed = u16(pad.buttons & ~race_prev_);
    race_prev_ = pad.buttons;
    auto rpressed = [&](PadButton b) { return (race_pressed & (1u << unsigned(b))) != 0; };
    if (!paused_ && (m.pressed(PadButton::Start) || rpressed(PadButton::Start) ||
                     (replaying_ && m.pressed(PadButton::Triangle)))) {
        open_pause(app);
        return;
    } else if (paused_ && !replay_opts_open_ && (m.pressed(PadButton::Start) || rpressed(PadButton::Start))) {
        paused_ = false; // (ours: Start also continues)
        return;
    }
    if (paused_) {
        if (online_) { // the others race on: so does our car (no input) behind the menu
            int n = online_ticks(step_.advance(dt));
            for (int i = 0; i < n; ++i) tick(app, PadState{});
            render_state_ = interpolate(prev_, vehicle_->state(), step_.alpha());
            // ... and the camera keeps following it (offline the frozen view is the pause).
            camera_.set_look_back(false);
            camera_.set_track_distance(track_->distance_of(render_state_.position));
            camera_.set_lap(track_->laps_completed());
            camera_.update(render_state_, vehicle_->setup(), PadState{}, dt);
        }
        update_pause(app, dt);
        return;
    }
    // GT3's buttons: R1 change view, L1 look back (held), Select respawn (ours), R3 our
    // performance overlay; see race/controls.h. Neither is on a controller by default: the
    // presets bind respawn to the keyboard (and the touch overlay) only and leave the
    // overlay (a developer tool) unbound, and under the "gt3" layout the race pad is the
    // menu pad, whose Select / R3 do nothing in a race (bind them in a custom race layout;
    // GT3R_RACE=perf=1 starts with the overlay on).
    bool own_race_pad = !app.input().race_uses_menu_pad(0);
    if (replaying_ || (theater_ && demo_)) {
        // Replays: R1 / L1 cycle the replay view mode (presentation.md §G), shown for a moment
        // by the mode indicator; Left / Right choose the followed car (TARGET MANUAL).
        // (The indicator's art names L2 / R2: they cycle the mode too.)
        const bool prev = m.pressed(PadButton::L1) || m.pressed(PadButton::L2) || rpressed(PadButton::L1);
        const bool next = m.pressed(PadButton::R1) || m.pressed(PadButton::R2) || rpressed(PadButton::R1);
        if (prev || next) {
            ReplaySettings rs = replay_settings_;
            rs.cycle_mode(prev ? -1 : 1);
            apply_replay_settings(app, rs);
            mode_display_t_ = kModeDisplaySeconds;
        }
        if (replay_settings_.current_target() == 0 && field_ && (m.pressed(PadButton::Left) || m.pressed(PadButton::Right))) {
            int n = field_->opponents() + 1; // -1 the player, then the opponents
            replay_focus_ = (replay_focus_ + 1 + (m.pressed(PadButton::Right) ? 1 : n - 1)) % n - 1;
        }
        camera_.set_look_back(false);
    } else {
        if (rpressed(PadButton::R1)) next_view(camera_, 0);
        camera_.set_look_back(pad.down(PadButton::L1));
    }
    // Never online: a respawn there would skip track (respawn is offline only).
    if (!replaying_ && !online_ && own_race_pad && rpressed(PadButton::Select)) respawn();
    if (own_race_pad && rpressed(PadButton::R3)) show_perf_ = !show_perf_;

    Uint64 now = SDL_GetPerformanceCounter();
    if (last_counter_) frame_ms_ = double(now - last_counter_) * 1000.0 / double(SDL_GetPerformanceFrequency());
    last_counter_ = now;

    int n = step_.advance(dt);
    if (online_) n = online_ticks(n);
    Uint64 h_t0 = SDL_GetTicksNS();
    for (int i = 0; i < n; ++i) tick(app, pad);
    hitch::ticks = n, hitch::ticks_ms = double(SDL_GetTicksNS() - h_t0) / 1e6;
    hitch::smoke = int(smoke_.live()), hitch::x = render_state_.position.x, hitch::z = render_state_.position.z;
    hitch::race_t = float(race_time_);
    // After the finish: results overlay, then the quick menu (GT3's flow).
    if ((finished_ || finished2_) && !replaying_ && !finish_menu_shown_) {
        bool all_done = true;
        if (field_)
            for (const RaceField::Standing& st : field_->standings()) all_done = all_done && st.finished;
        if (all_done && all_done_at_ < 0) all_done_at_ = finish_wait_;
        double shown = field_ ? ResultsOverlay::fully_shown_time(size_t(field_->opponents() + 1 + (field_->has_second() ? 1 : 0)))
                              : 5.0;
        bool leave = finish_wait_ > shown && (app.menu().pressed(PadButton::Cross) ||
                                              (field_ ? (all_done && finish_wait_ - all_done_at_ > 15.0)
                                                      : finish_wait_ > 15.0));
        // Licence results and the time-trial FAIL: GT3's presenter timeline (frames from
        // the result); Cross only in its last hold. Leaving takes 2 more frames (event 1).
        if (!field_ && (!license_event_.empty() || tt_failed_)) {
            bool tt = license_event_.empty();
            int medal = tt ? -1 : presented_medal();
            int f = result_frame(), skip = LicenseResultOverlay::skippable_from(medal, tt);
            leave = f >= LicenseResultOverlay::end_frame(medal, tt) + 2 ||
                    (skip >= 0 && f >= skip && app.menu().pressed(PadButton::Cross));
        }
        if (two_player_ && finish_wait_ > shown + 15.0) leave = true;
        if (leave && two_player_ && all_done && !post_replay_done_) {
            // GT3's 2P battle: a dimmed split-screen replay of the race, then the 2P RESULTS menu.
            post_replay_done_ = true;
            watch_replay();
            post_replay_ = replay_from_quick_ = replaying_;
            if (!replaying_) open_quick_menu(app);
        } else if (leave) {
            exit_transition_ = 0; // the race loop ends: GT3's loader transition, then the menu
        }
    }
    if (post_replay_ && replaying_ && !paused_ &&
        (app.menu().pressed(PadButton::Cross) || app.menu().pressed(PadButton::Circle))) {
        end_replay(app); // the post-race replay: any button goes on to the results menu
        return;
    }
    if (replaying_ && player_.done() && !paused_ && replay_from_quick_) {
        post_replay_ = false;
        if (!finish_menu_shown_) {
            replay_from_quick_ = false;
            replaying_ = false;
            camera_.set_mode(drive_camera_);
            silence(app);
            open_quick_menu(app);
            return;
        }
        replay_from_quick_ = false; // back to the quick menu, as after a race
        replaying_ = false;
        camera_.set_mode(drive_camera_);
        silence(app);
        quick_ = true;
    } else if (replaying_ && player_.done() && !paused_ && theater_) {
        leave(app); // the theater's replay ended: back to the theater
        return;
    } else if (replaying_ && player_.done() && !paused_) {
        open_pause(app); // end of the replay: show the menu
        toast("End of replay");
        const sim::Vec3& a = vehicle_->state().position;
        bool exact = a.x == replay_.final_position.x && a.y == replay_.final_position.y && a.z == replay_.final_position.z;
        LOG_INFO("Replay: end, final position %s the recording", exact ? "matches" : "DIFFERS FROM");
    }
    render_state_ = interpolate(prev_, vehicle_->state(), step_.alpha());
    if (two_player_ && field_ && field_->has_second()) {
        PadState pad2 = app.input().race_pad(1);
        auto pressed2 = [&](PadButton b) { return pad2.down(b) && !(pad2_prev_ & (1u << unsigned(b))); };
        if (pressed2(PadButton::R1)) next_view(camera2_, 1);
        camera2_.set_look_back(pad2.down(PadButton::L1));
        if (!replaying_ && !online_ && !app.input().race_uses_menu_pad(1) && pressed2(PadButton::Select)) {
            TrackPose p = track_->respawn_pose(field_->second_state().position);
            field_->reset_second(p);
            recorder_.respawn_second(p);
            camera2_.snap(field_->second_state());
        }
        pad2_prev_ = pad2.buttons;
        render_state2_ = interpolate(field_->second_prev_state(), field_->second_state(), step_.alpha());
        camera2_.set_track_distance(track_->distance_of(render_state2_.position));
        camera2_.set_lap(field_->second_laps().laps);
        camera2_.update(render_state2_, field_->second_setup(), pad2, dt);
        app.input().set_rumble(1, rumble2_on_ && !replaying_ ? rumble2_.rumble() : PadRumble{});
        if (rumble2_on_ && !replaying_) app.input().set_trigger_rumble(1, rumble2_.trigger_left(), rumble2_.trigger_right());
        else app.input().set_trigger_rumble(1, 0, 0);
        if (se_on() && engine2_.loaded()) {
            sim::Vec3 off2 = render_state2_.position - to_sim(camera2_.eye());
            if (camera2_.onboard()) engine2_.set_position(0.0f, 1.0f);
            else engine2_.set_position(off2.dot(cam_right(camera2_)), off2.dot(cam_forward(camera2_)));
            engine2_.set_view(camera2_.onboard() ? EngineSound::View::OnboardOwn : EngineSound::View::ChaseOwn);
            engine2_.update(app.audio().voices(), render_state2_.engine_rpm, render_state2_.throttle, dt);
        }
        if (const sim::Vehicle* sv2 = field_->second_vehicle()) {
            sim::Vec3 o2 = render_state2_.position - to_sim(camera2_.eye());
            if (se_on()) extra2_.update(sfx_, app.audio().voices(),
                           extra_frame(*sv2, render_state2_, sound_car2_, camera2_.onboard(), o2.dot(cam_right(camera2_)),
                                       o2.dot(cam_forward(camera2_)),
                                       camera2_.onboard() ? SoundView::InCarTarget : SoundView::ChaseTarget));
        }
    }
    camera_.set_track_distance(track_->distance_of(render_state_.position));
    camera_.set_lap(track_->laps_completed());
    // GT3's pre-race shot: races with a standing-start hold play start.cam until one second
    // before the signal (start counter C >= 60 frames, t = 1 - (C - 60) / 600).
    {
        bool intro = field_ && race_mode_ && !two_player_ && !replaying_ && !demo_ && field_->race_time() < -1.0;
        double frames = field_ ? -field_->race_time() * 60.0 : 0.0;
        camera_.set_intro(intro, float(1.0 - (frames - 60.0) / 600.0));
    }
    if (replaying_ && field_) { // the replay's followed car (TARGET)
        update_replay_focus();
        if (replay_focus_ >= 0 && replay_focus_ < field_->opponents()) {
            sim::VehicleState fs = interpolate(field_->opponent_prev_state(replay_focus_), field_->opponent_state(replay_focus_),
                                               step_.alpha());
            camera_.set_track_distance(track_->distance_of(fs.position));
            camera_.set_lap(field_->opponent_laps(replay_focus_));
            camera_.update(fs, field_->opponent_setup(replay_focus_), pad, dt);
        } else {
            camera_.update(render_state_, vehicle_->setup(), pad, dt);
        }
    } else {
        camera_.update(render_state_, vehicle_->setup(), pad, dt);
    }
    if (mode_display_t_ > 0) mode_display_t_ = std::max(0.0, mode_display_t_ - dt);
    update_shot_filter();

    bool feel = rumble_on_ && !replaying_ && !demo_;
    app.input().set_rumble(0, feel ? rumble_.rumble() : PadRumble{});
    if (!feel) app.input().set_trigger_rumble(0, 0, 0);
    else app.input().set_trigger_rumble(0, rumble_.trigger_left(), rumble_.trigger_right());
    if (field_ && field_->opponents() > 0) {
        std::vector<const sim::VehicleState*> st;
        for (int i = 0; i < field_->opponents(); ++i) st.push_back(&field_->opponent_state(i));
        render::Vec3 e = camera_.eye();
        opp_engines_.view = camera_.onboard() ? EngineSound::View::OnboardOther
                            : replaying_ || camera_.mode() == CameraMode::Tv ? EngineSound::View::Replay
                                                                             : EngineSound::View::ChaseOther;
        std::vector<float> slip;
        for (int i = 0; i < field_->opponents(); ++i) {
            float y = 0;
            [&](const auto& t) {
                if constexpr (requires { t.slipstream; }) y = t.slipstream;
            }(field_->opponent_vehicle(i)->telemetry());
            slip.push_back(y);
        }
        if (se_on()) opp_engines_.update(app.audio().voices(), {e.x, e.y, e.z}, render_state_.velocity, cam_right(camera_),
                            cam_forward(camera_), st, dt, &slip);
    }
    if (se_on() && engine_.loaded()) {
        // Stereo placement: centred on board, else the car's offset from the camera.
        sim::Vec3 off = render_state_.position - to_sim(camera_.eye());
        if (camera_.onboard()) engine_.set_position(0.0f, 1.0f);
        else engine_.set_position(off.dot(cam_right(camera_)), off.dot(cam_forward(camera_)));
        engine_.set_view(replaying_ ? EngineSound::View::Replay
                         : camera_.onboard() ? EngineSound::View::OnboardOwn
                                             : EngineSound::View::ChaseOwn);
        engine_.update(app.audio().voices(), render_state_.engine_rpm, render_state_.throttle, dt);
    }
    if (const sim::Vehicle* sv = vehicle_->sim_vehicle()) {
        sim::Vec3 o = render_state_.position - to_sim(camera_.eye());
        SoundView view = camera_.onboard() ? SoundView::InCarTarget
                         : camera_.mode() == CameraMode::Tv || replaying_ ? SoundView::Other
                                                                           : SoundView::ChaseTarget;
        if (se_on()) extra_.update(sfx_, app.audio().voices(),
                      extra_frame(*sv, render_state_, sound_car_, camera_.onboard(), o.dot(cam_right(camera_)),
                                  o.dot(cam_forward(camera_)), view));
    }
    update_course_ambience(app);
    // Controller force feedback (triggers, wheels) while someone is driving.
    if (!replaying_ && !demo_ && rumble_on_) {
        if (!finished_)
            if (const sim::Vehicle* sv = vehicle_->sim_vehicle())
                app.input().set_force_feedback(0, feedback(*sv, last_in_, ffb_wall_, ffb_car_));
        if (field_ && field_->has_second() && !finished2_)
            if (const sim::Vehicle* sv2 = field_->second_vehicle())
                app.input().set_force_feedback(
                    1, feedback(*sv2, last_in2_, ffb_wall2_, ffb_car2_));
    }
    ffb_wall_ = ffb_car_ = ffb_wall2_ = ffb_car2_ = 0;
}

// Tyre fy is along the wheel's right axis (forward x up), so in a right-hand corner the
// front tyres push right (fy > 0) and the wheel is pulled left (negative force).
ForceFeedbackState RaceScreen::feedback(const sim::Vehicle& v, const sim::ControlInputs& in, float wall, float car) {
    const sim::VehicleTelemetry& t = v.telemetry();
    ForceFeedbackState f;
    f.speed_kmh = v.state().speed_kmh;
    f.brake = in.brake;
    f.abs_active = t.abs_active;
    float front_load = std::max(1.0f, v.spec().mass * v.spec().front_weight * 9.81f);
    f.steer_force = std::clamp(-(t.fy[0] + t.fy[1]) / front_load, -1.0f, 1.0f);
    f.jolt = std::min(1.0f, std::max(wall, car) / 5000.0f);
    return f;
}

// Recommended-gear hint (GT3: the player's car runs the AI's line bookkeeping passively;
// when faster than the coming section's target speed T, the hint is the gear the automatic
// box would pick for T, blinking in braking zones). T = the next point's speed, or in an
// arc's braking zone min(current, next).
void RaceScreen::update_gear_hint(const sim::VehicleState& s) {
    hint_gear_ = -1, hint_blink_ = false;
    const sim::Vehicle* sv = vehicle_ ? vehicle_->sim_vehicle() : nullptr;
    const track::Track* rt = track_ ? track_->runtime() : nullptr;
    if (!sv || !rt || replaying_) return;
    if (!hint_driver_) {
        if (!hint_course_) hint_course_ = std::make_shared<ai::AiCourse>(*rt);
        hint_driver_ = std::make_unique<ai::AiDriver>(hint_course_, ai::DriverParams::builtin(0));
        hint_driver_->reset(s);
    }
    ai::CarSnapshot me{s.position, s.orientation, s.velocity};
    hint_driver_->update(*sv, {race_time_, std::span<const ai::CarSnapshot>(&me, 1), 0});
    const ai::AiDebug& d = hint_driver_->debug();
    float t = d.mode == 4 ? std::min(d.target_speed, d.next_speed) : d.next_speed;
    float v = s.speed_kmh / 3.6f;
    if (t <= 0 || v < t) return;
    const sim::CarSpec& sp = sv->spec();
    float r = sp.drivetrain.drive == sim::Drive::FF ? sv->setup().wheels[0].radius : sv->setup().wheels[2].radius;
    hint_gear_ = sim::model::gear_for_speed(sp.gearbox, t, r, sv->setup().upshift_rpm);
    hint_blink_ = d.mode == 3 || d.mode == 4;
    static int last = -1;
    if (debug_ && hint_gear_ != last)
        LOG_INFO("gear hint %d%s (v %.1f T %.1f mode %d)", hint_gear_, hint_blink_ ? " blink" : "", v, t, d.mode);
    last = hint_gear_;
}

// Dirt (GT3: the car's condition word, 0..0.8, weakens the reflection passes as 1 - dirt):
// every step it grows by S x (8 on tarmac courses, else 1) x 0.001 x dt, S = the sum over the
// wheels on loose surfaces (indices 2-6, 12-14) of the smoke-emission intensity (physics §8.3b).
void RaceScreen::update_dirt(double dt) {
    const sim::Vehicle* sv = vehicle_ ? vehicle_->sim_vehicle() : nullptr;
    if (!sv || replaying_) return;
    const sim::VehicleState& st = sv->state();
    const sim::VehicleTelemetry& t = sv->telemetry();
    float sum = 0;
    for (size_t i = 0; i < 4; ++i) {
        int si = sim::model::surface_index(st.wheels[i].surface);
        bool loose = (si >= 2 && si <= 6) || (si >= 12 && si <= 14);
        if (loose && st.wheels[i].grounded) sum += t.smoke_emit[i]; // the wheel's smoke emission (already <= 1)
    }
    int category = 0;
    for (const data::Course& c : db_.courses())
        if (c.label == course_label_for_dirt_) category = c.surface;
    car_dirt_ = std::min(0.8, car_dirt_ + double(sum) * (category == 0 ? 8.0 : 1.0) * 0.001 * dt);
    if (category == 1) dirt_dims_ = false; // the wet Route 5: no dimming
}

// Pylons (GT3's rigid-body pylons, every race mode): each car's box knocks them about; a
// touch by the player's car fails a licence.
void RaceScreen::update_pylons(double dt) {
    if (pylons_.count() == 0) return;
    std::vector<Pylons::Car> cars;
    auto add = [&](const sim::Vehicle* v) {
        if (!v) return;
        const sim::VehicleState& st = v->state();
        const sim::CarSpec& sp = v->spec();
        Pylons::Car c;
        c.pos = st.position, c.vel = st.velocity, c.orientation = st.orientation;
        c.yaw_rate = st.angular_velocity.y;
        c.front = sp.front_extent > 0 ? sp.front_extent : sp.length * 0.5f;
        c.rear = sp.rear_extent > 0 ? sp.rear_extent : sp.length * 0.5f;
        c.half_width = sp.width * 0.5f;
        cars.push_back(c);
    };
    add(vehicle_ ? vehicle_->sim_vehicle() : nullptr); // car 0 = the player
    if (field_) {
        if (field_->has_second()) add(field_->second_vehicle());
        for (int i = 0; i < field_->opponents(); ++i) add(field_->opponent_vehicle(i));
    }
    unsigned hits = pylons_.step(track_->query(), cars, dt, &track_->world());
    if ((hits & 1u) && !pylon_hit_) {
        pylon_hit_ = true;
        const sim::Vec3& p = vehicle_->state().position;
        LOG_INFO("Race: pylon touched at (%.1f %.1f %.1f), %.0f km/h", p.x, p.y, p.z, vehicle_->state().speed_kmh);
    }
}

// Tyre smoke / dust: every car's wheels emit each tick (GT3: physics phase 12), then the puffs move.
void RaceScreen::update_smoke(double dt) {
    if (!smoke_.loaded()) return;
    const track::Track* rt = track_ ? track_->runtime() : nullptr;
    auto car = [&](const sim::Vehicle* v) {
        if (!v) return;
        const sim::VehicleState& st = v->state();
        const sim::VehicleTelemetry& t = v->telemetry();
        for (size_t i = 0; i < 4; ++i) {
            const sim::WheelState& w = st.wheels[i];
            if (!w.grounded || t.smoke_emit[i] <= 0.001f) continue; // the wheel's smoke emission (physics §8.3b)
            float sun = 1;
            if (rt) {
                RoadShade rs = road_shade(rt->road(), w.contact.x, w.contact.z, w.contact.y + 1.0f);
                if (rs.hit) sun = rs.a;
            }
            smoke_.emit({w.contact.x, w.contact.y, w.contact.z}, sim::model::surface_index(w.surface), t.smoke_emit[i], sun,
                        (i & 1) != 0);
        }
    };
    car(vehicle_ ? vehicle_->sim_vehicle() : nullptr);
    if (field_) {
        if (field_->has_second()) car(field_->second_vehicle());
        for (int i = 0; i < field_->opponents(); ++i) car(field_->opponent_vehicle(i));
    }
    smoke_.update(dt);
}

} // namespace gt3r::race_screen
