// The solo modes: the time trial, licence test, machine test, free run and free practice as GT3 runs them, one race with
// the mode's conductor (sim::exact: its virtuals judge the run, time its laps, sectors and splits, and the free run's lap
// handlers keep its best lap and drive its ghost car). (Race screen code map: docs/dev/RACE.md.)

#include "app/screen_race_impl.h"
#include "race/exact_demo.h"
#include "race/exact_field.h"
#include "sim/exact/race.h"

namespace gt3r::race_screen {

std::optional<sim::exact::Conductor> RaceScreen::solo_conductor() const {
    using K = sim::exact::Conductor;
    if (race_mode_ || two_player_ || online_) return std::nullopt;
    // As run_kind() tells the runs apart: GT mode's free run before a race is free practice; a test with judging times: the
    // licence tests (L...) under the licence conductor, the arcade time attacks (AT..., ranked by their times too) under
    // the time trial's; the machine tests; a time trial of no set laps the free run (its lap ghost).
    if (free_run_) return K::FreePractice;
    if (!license_event_.empty()) return race::demo_conductor(license_event_);
    if (machine_) return K::MachineTest;
    return race_laps_ > 0 ? K::TimeTrial : K::FreeRun;
}

bool RaceScreen::build_exact_solo(App& app) {
    const std::optional<sim::exact::Conductor> kind = solo_conductor();
    if (!kind) {
        error_ = "This run cannot be set up";
        return false;
    }
    FieldConfig cfg;
    cfg.event = !solo_event_.empty() ? solo_event_ : race_event_;
    cfg.max_opponents = 0;
    // What every race takes from the screen (the program, rules, course, the player, playback and ghost).
    ExactFieldOptions o = exact_options(app);
    o.kind = *kind;
    o.config = preset_.autodrive ? 3 : 0; // the test autopilot: the race's AI drives the player's car (the pad decoded)
    o.laps = race_laps_;
    // The race information's words: the visit's session best, the licence test's judging times (the event's), the speed
    // unit; the free run's ghost car shown with Select's ghost switch.
    o.session = exact_session_;
    if (const data::Event* ev = db_.event(cfg.event)) {
        const LicenseTimes t = license_times(*ev);
        for (size_t k = 0; k < 3; ++k) o.licence_times[k] = u32(std::max(0, t.ms[k]));
    }
    o.speed_unit = imperial_ ? 1 : 0;
    o.ghost_shown = preset_.ghost && ghost_shown_;
    o.ghost_option = ghost_on_; // (a better lap the ghost at once; else "Update Ghost?")
    // The free run's best lap and ghost stay from run to run of the visit, as GT3's race object keeps them.
    if (*kind == sim::exact::Conductor::FreeRun && !replaying_ && !demo_) {
        if (!exact_free_run_) exact_free_run_ = std::make_shared<sim::exact::SoloRecorders>(true);
        o.free_run = exact_free_run_;
    }
    cfg.exact = std::move(o);
    std::string err;
    solo_field_ = RaceField::create(*track_, db_, cfg, &err);
    if (!solo_field_) {
        if (replaying_ || demo_) {
            LOG_ERROR("Replay: the recorded run cannot be built (%s)", err.c_str());
            toast("This replay cannot be played.");
        }
        error_ = "This run cannot be set up: " + err;
        LOG_WARN("Race: the %s cannot be set up (%s)", sim::exact::conductor_name(*kind), err.c_str());
        return false;
    }
    vehicle_ = &solo_field_->player();
    track_->set_lap_source(exact_lap_source(*solo_field_)); // the exact core times the laps itself
    if (1.0 / step_.step() != solo_field_->tick_hz()) step_ = FixedStep(solo_field_->tick_hz()); // (PAL: 50 a second)
    solo_result_done_ = false;
    LOG_INFO("Race: %s, event %s", sim::exact::conductor_name(*kind), cfg.event.c_str());
    return true;
}

// After a step of a solo run: the race's clock and start hold for the screen, the visit's session best,
// and the run's end as the race flow decides it.
void RaceScreen::exact_solo_step() {
    const sim::exact::ExactRace* r = exact_race_of(*solo_field_);
    if (!r) return;
    const double rt = solo_field_->race_time();
    tt_hold_ = std::max(0.0, -rt), race_time_ = std::max(0.0, rt);
    if (replaying_ || demo_) return;
    const sim::exact::RaceFlow& fl = r->flow();
    for (size_t k = 0; k < 4; ++k) exact_session_[k] = fl.session.sector[k];
    exact_session_[4] = fl.session.lap;
    if (solo_result_done_ || finished_) return;
    const sim::exact::Car& c = r->cars()[0];
    if (c.finished == 0) return;
    solo_result_done_ = true;
    using K = sim::exact::Conductor;
    const double best = c.best_lap == sim::exact::kNoTime ? -1.0 : double(c.best_lap) / 1000.0;
    switch (r->kind()) {
    case K::TimeTrial: { // every lap ends the run; out of the course (event 10) fails it
        bool out = false;
        for (const sim::exact::RaceEvent& e : r->events()) out = out || e.type == 10;
        finished_ = true;
        if (out) {
            LOG_INFO("Time trial (exact): FAIL (out of the course) at %.2f s", race_time_);
            if (!license_event_.empty()) return license_end(LicenseRun::State::Failed, -1, -1, "out of the course");
            tt_failed_ = true;
            finish_position_ = 0;
            return;
        }
        track_->lap_freeze();
        LOG_INFO("Time trial (exact): %.3f s", best);
        // A time attack with judging times: its medal as the screen gives it (license_end ranks the run).
        if (!license_event_.empty()) license_end(LicenseRun::State::Finished, best, license_medal(license_times_, best), {});
        else finish_position_ = 1, rank_finish(best, false);
        return;
    }
    case K::MachineTest: { // the split line ends the run: the time to it, or the top speed
        finished_ = true;
        finish_position_ = 1;
        track_->lap_freeze();
        top_speed_kmh_ = c.max_speed.host() * 3.6f;
        if (machine_ && machine_->timed()) {
            machine_time_ = best;
            LOG_INFO("Machine test (exact): %.3f s (top %.1f km/h)", machine_time_, top_speed_kmh_);
            rank_finish(machine_time_, false);
        } else {
            LOG_INFO("Machine test (exact): top speed %.1f km/h", top_speed_kmh_);
            rank_finish(top_speed_kmh_, true);
        }
        return;
    }
    case K::License: { // the licence's judging (the race clock) and its display (race info + 0x40
                       // 0, slower than bronze, shows TIME EXPIRED with + 0x44's time; -1, out of the course, FAIL)
        bool failed_event = false;
        for (const sim::exact::RaceEvent& e : r->events()) failed_event = failed_event || e.type == 9;
        const race::LicenseOutcome o = race::license_outcome(fl.result, fl.result_time, failed_event, best);
        license_end(o.state, o.time, o.medal, o.failure);
        return;
    }
    default: return; // the free run does not end
    }
}

} // namespace gt3r::race_screen
