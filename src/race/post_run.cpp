#include "race/post_run.h"

namespace gt3r::race {

PostRun post_run(RunKind kind, const RunOutcome& o) {
    PostRun p;
    switch (kind) {
    case RunKind::ArcadeRace: // the win's trophy (presentation §F), then the replay
        p.prize = o.finished && o.place == 0;
        p.replay = true;
        break;
    case RunKind::GtRace: // a finish's prize page (presentation §F), then the replay
    case RunKind::Championship:
        p.prize = o.finished && o.place >= 0;
        p.replay = true;
        break;
    case RunKind::TwoPlayer: p.replay = true; break;
    case RunKind::TimeTrial: // the record's name entry, then the replay (presentation §C4)
    case RunKind::FreeRun:
    case RunKind::MachineTest:
        p.name_entry = o.ranked;
        p.replay = true;
        break;
    case RunKind::Licence: // the name entry, no replay (presentation §A8)
        p.name_entry = o.ranked;
        break;
    case RunKind::FreePractice:
    case RunKind::Other: break;
    }
    return p;
}

MenuBackdrop menu_backdrop(RunKind kind, bool before_run) {
    switch (kind) {
    case RunKind::Licence: return MenuBackdrop::Preview;
    case RunKind::TimeTrial:
    case RunKind::FreeRun:
    case RunKind::MachineTest: return MenuBackdrop::Race;
    case RunKind::GtRace:
    case RunKind::Championship:
    case RunKind::FreePractice: return before_run ? MenuBackdrop::Race : MenuBackdrop::None;
    case RunKind::ArcadeRace:
    case RunKind::TwoPlayer:
    case RunKind::Other: return MenuBackdrop::None;
    }
    return MenuBackdrop::None;
}

bool replay_over(PlayMode m, bool recording_done) { return m == PlayMode::Replay && recording_done; }

bool QuickMenuItems::enabled(std::string_view item) const {
    if (item == "replay" || item == "save_replay") return replay;
    if (item == "save_ghost" || item == "data_logger") return session_record;
    if (item == "setting" || item == "save_setting" || item == "load_setting") return !setting_locked;
    if (item == "exit" || item == "save_game") return !exit_locked;
    if (item == "free_session") return !free_run_locked;
    return true;
}

void QuickMenuItems::visit(RunKind kind, bool garage_car, bool dirt_course) {
    setting_locked = kind == RunKind::TimeTrial ||
                     ((kind == RunKind::ArcadeRace || kind == RunKind::FreeRun) && !garage_car);
    free_run_locked = kind == RunKind::GtRace && dirt_course;
}

void QuickMenuItems::after_run(RunKind kind, bool ended, bool record, bool last_round) {
    replay = kind != RunKind::Licence || ended;
    if (record && (kind == RunKind::TimeTrial || kind == RunKind::FreeRun || kind == RunKind::MachineTest ||
                   kind == RunKind::Licence))
        session_record = true;
    if (kind == RunKind::Championship && last_round) exit_locked = true;
}

bool pause_exit_finishes(const PauseExitRun& r) {
    if (r.finished) return false;
    if (r.kind == RunKind::FreeRun) return r.laps_completed >= 1;
    if (r.kind == RunKind::MachineTest) return r.top_speed_test && r.top_speed_mps > 1.0f;
    return false;
}

double exit_finish_overlay_delay(const PauseExitRun& r) {
    return r.kind == RunKind::MachineTest ? kTopSpeedExitOverlayDelay : kRecordOverlayDelay;
}

} // namespace gt3r::race
