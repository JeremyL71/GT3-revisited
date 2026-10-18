#include "race/race_audio.h"

#include "audio/audio_engine.h"
#include "audio/bgm.h"
#include "audio/car_sound_model.h"
#include "race/car_sfx.h"
#include "base/log.h"
#include "formats/bgm_index.h"
#include "vfs/vfs.h"

#include <SDL3/SDL_timer.h>

#include <algorithm>
#include <cmath>

namespace gt3r::race {

float ReverbAmbience::target_for(u32 flags) {
    switch (flags & 0x18) {
    case 0x08: return 0.5f;
    case 0x10:
    case 0x18: return 0.25f;
    default: return 0.125f;
    }
}

void ReverbAmbience::tick(VoiceMixer& mixer, u32 flags) {
    constexpr float kStep = 1.0f / 36.0f;
    depth_ += std::clamp(target_for(flags) - depth_, -kStep, kStep);
    s16 evol = s16(std::lround(depth_ * 32767.0f));
    mixer.set_reverb_depth(evol, evol);
}

void ReverbAmbience::reset(VoiceMixer& mixer) {
    depth_ = 0.125f;
    mixer.set_reverb_depth(0x0FFF, 0x0FFF);
}

void RaceMusic::stop(AudioEngine& audio) {
    if (active_) audio.bgm().stop(audio);
    active_ = false;
    seen_ = false;
}

std::vector<RaceMusic::Action> RaceMusic::decide(const Cues& c, bool music_playing) {
    using Op = Action::Op;
    std::vector<Action> out;
    if (!c.music_on) {
        if (!seen_ || was_on_) out.push_back({Op::Stop});
        seen_ = true;
        was_on_ = false;
        return out;
    }
    bool first = !seen_ || !was_on_;
    if (first) { // a new run (or music switched on): no jingle state carried over
        jingle_ = start_jingle_ = medal_played_ = demo_played_ = false;
        was_finished_ = c.finished;
        was_holding_ = false;
    }
    seen_ = was_on_ = true;
    // Quick menus: their music, looping; closing one stops the BGM.
    if (c.menu != was_menu_ || (first && c.menu != Menu::None)) {
        if (c.menu == Menu::None) {
            out.push_back({Op::Stop});
        } else {
            out.push_back({Op::PlayIfDifferent, c.menu == Menu::Result ? bgm::kResult : bgm::kQuickMenu, true});
        }
        was_menu_ = c.menu;
        jingle_ = start_jingle_ = false;
        was_finished_ = c.finished;
        was_paused_ = c.paused;
        was_holding_ = c.holding;
        medal_played_ = false;
        return out;
    }
    if (c.menu != Menu::None) return out;
    if (c.paused != was_paused_) {
        out.push_back({c.paused ? Op::Pause : Op::Resume});
        was_paused_ = c.paused;
    }
    if (c.paused) return out;
    // The start: standing starts play a jingle while the cars are held (type 1,
    // the 4 s hold: "jg04e"; type 2, the 10 s hold: "jg11"); the race music
    // replaces it at the signal (licence tests: the music stops instead).
    if (c.holding && !was_holding_ && c.kind != Kind::Demo && (c.start_type == 1 || c.start_type == 2)) {
        out.push_back({Op::Play, c.start_type == 1 ? bgm::kStartShort : bgm::kStartLong, false});
        jingle_ = start_jingle_ = true;
    } else if (!c.holding && was_holding_ && start_jingle_) {
        start_jingle_ = jingle_ = false;
        if (c.kind == Kind::License || c.kind == Kind::LicenseReplay) out.push_back({Op::Stop});
        else out.push_back({Op::Play, bgm::kRace, false}), music_playing = true;
    }
    was_holding_ = c.holding;
    if (c.finished && !was_finished_) {
        // The finish: "jg12" for every finish, whatever the place (2P: only the
        // winner's); "fail" for a failed licence test or a voided time trial. A
        // licence pass has no finish jingle, only the medal's (below).
        start_jingle_ = false;
        if (c.failed) out.push_back({Op::Play, bgm::kFail, false}), jingle_ = true;
        else if (c.kind == Kind::License) jingle_ = true; // silence until the medal jingle
        else if (c.kind != Kind::Demo && (c.kind != Kind::TwoPlayer || c.position == 1))
            out.push_back({Op::Play, bgm::kFinish, false}), jingle_ = true;
        medal_played_ = false;
    } else if (!c.finished && was_finished_) {
        jingle_ = false; // restart: race music again
        medal_played_ = false;
    }
    was_finished_ = c.finished;
    if (c.kind == Kind::License && c.finished && !c.failed && c.medal >= 0 && !medal_played_ &&
        c.since_finish >= kMedalDelay) {
        static const int kMedal[3] = {bgm::kLicenseGold, bgm::kLicenseSilver, bgm::kLicenseBronze};
        out.push_back({Op::Play, kMedal[std::min(c.medal, 2)], false});
        medal_played_ = true;
    }
    // Race tracks from the start signal (none in licence tests; the demo track in
    // attract races), one after another.
    if (!jingle_ && !music_playing && !c.holding && c.kind != Kind::License && c.kind != Kind::LicenseReplay) {
        if (c.kind == Kind::Demo) {
            if (!demo_played_) out.push_back({Op::Play, bgm::kDemo, false}), demo_played_ = true;
        } else {
            out.push_back({Op::Play, bgm::kRace, false});
        }
    }
    return out;
}

void RaceMusic::cues(AudioEngine& audio, Vfs& vfs, const Cues& c) {
    active_ = c.music_on;
    for (const Action& a : decide(c, audio.bgm().playing(audio))) {
        switch (a.op) {
        case Action::Op::Play: audio.bgm().play(audio, vfs, a.id, -1, a.loop); break;
        case Action::Op::PlayIfDifferent: audio.bgm().play_if_different(audio, vfs, a.id, 0, a.loop); break;
        case Action::Op::Stop: audio.bgm().stop(audio); break;
        case Action::Op::Pause: audio.pause_music(); break;
        case Action::Op::Resume: audio.resume_music(); break;
        }
    }
}

float OpponentEngines::attenuation(float d) { return std::min(1.0f, 32.0f / std::max(d, 0.25f)); }

float OpponentEngines::doppler(const sim::Vec3& dir, const sim::Vec3& v_src, const sim::Vec3& v_lis) {
    constexpr float c = 331.45f;
    // dir points from the listener to the source: a source moving towards the
    // listener has v_src . dir < 0.
    float vs = std::clamp(v_src.dot(dir), -c * 0.5f, c * 0.5f);
    float vl = std::clamp(v_lis.dot(dir), -c * 0.5f, c * 0.5f);
    return std::clamp((c + vl) / (c + vs), 0.25f, 3.99f);
}

void OpponentEngines::load(Vfs& vfs, const std::vector<Car>& cars) {
    engines_.clear();
    winds_.clear();
    cd_.clear();
    active_.assign(cars.size(), false);
    for (const Car& c : cars) {
        auto e = std::make_unique<EngineSound>();
        if (c.sound_id) e->load(vfs, c.sound_id, c.variant);
        engines_.push_back(std::move(e));
        auto w = std::make_unique<EngineUnit>();
        w->load(vfs, EngineUnit::kWindBank);
        winds_.push_back(std::move(w));
        cd_.push_back(c.cd);
    }
}

void OpponentEngines::update(VoiceMixer& mixer, const sim::Vec3& listener, const sim::Vec3& lv, const sim::Vec3& right,
                             const sim::Vec3& forward, const std::vector<const sim::VehicleState*>& states, double dt,
                             const std::vector<float>* slipstream) {
    size_t n = std::min(states.size(), engines_.size());
    std::vector<std::pair<float, size_t>> by_distance;
    for (size_t i = 0; i < n; ++i) by_distance.push_back({(states[i]->position - listener).length(), i});
    std::sort(by_distance.begin(), by_distance.end());
    std::vector<bool> want(n, false), wind(n, false);
    for (size_t k = 0; k < by_distance.size() && int(k) < max_voiced; ++k) want[by_distance[k].second] = true;
    for (size_t k = 0; k < by_distance.size() && int(k) < max_wind; ++k) wind[by_distance[k].second] = true;
    for (size_t i = 0; i < n; ++i) {
        EngineSound& e = *engines_[i];
        if (!want[i] || !e.loaded()) {
            if (active_[i]) {
                e.stop(mixer);
                if (i < winds_.size()) winds_[i]->stop(mixer);
            }
            active_[i] = false;
            continue;
        }
        const sim::VehicleState& s = *states[i];
        sim::Vec3 d = s.position - listener;
        float dist = d.length();
        sim::Vec3 dir = dist > 1e-3f ? d * (1.0f / dist) : sim::Vec3{0, 0, 1};
        e.set_view(view);
        e.set_position(d.dot(right), d.dot(forward)); // pan + front/back (surround cue)
        e.set_volume(attenuation(std::hypot(d.x, d.z)));
        float dop = doppler(dir, s.velocity, lv);
        e.update(mixer, s.engine_rpm, s.throttle, dt, dop);
        if (i < winds_.size() && !wind[i]) winds_[i]->stop(mixer);
        else if (i < winds_.size() && winds_[i]->loaded()) {
            SoundView sv = view == EngineSound::View::OnboardOther ? SoundView::InCarOther
                           : view == EngineSound::View::Replay      ? SoundView::Other
                                                                    : SoundView::ChaseOther;
            float y = slipstream && i < slipstream->size() ? (*slipstream)[i] : 0.0f;
            car_sound::Wind w = car_sound::wind(s.speed_kmh / 3.6f, y, cd_[i]);
            winds_[i]->update_at(mixer, float(w.rpm), sfx_gain(sv, kChWind) * attenuation(std::hypot(d.x, d.z)) * w.gain,
                                 d.dot(right), d.dot(forward), dop);
        }
        active_[i] = true;
    }
}

void OpponentEngines::stop(VoiceMixer& mixer) {
    for (size_t i = 0; i < engines_.size(); ++i) engines_[i]->stop(mixer);
    for (auto& w : winds_) w->stop(mixer);
    active_.assign(active_.size(), false);
}

} // namespace gt3r::race
