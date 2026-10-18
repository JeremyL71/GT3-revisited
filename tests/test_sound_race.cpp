// Race-side sound models: wind/in-car/whine/turbo maths, the looping effect
// voice, the rn_01.es wind unit, and CRSS course ambience (with GT3R_DISC:
// every course's CRSS section).

#include "test_framework.h"

#include "audio/bgm.h"
#include "audio/car_sound_model.h"
#include "audio/course_ambience.h"
#include "audio/engine_sound.h"
#include "audio/loop_effect.h"
#include "audio/surround_cue.h"
#include "formats/course/course_file.h"
#include "formats/sound/crss.h"
#include "formats/sound/inst.h"
#include "formats/sound/mads.h"
#include "menu/flow/menu_sound.h"
#include "race/race_audio.h"
#include "vfs/vfs.h"

#include <cmath>

using namespace gt3r;

TEST(sound_car_models) {
    using namespace gt3r::car_sound;
    // Wind: C_d 0.35 at 100 km/h -> 0.001 * 0.35 * 27.78^2 = 0.27; rpm on
    // the 10..450 km/h ramp.
    Wind w = wind(100 / 3.6f, 0, 0.35f);
    CHECK(std::abs(w.gain - 0.27f) < 0.01f);
    CHECK(std::abs(w.rpm - int(2500 + (100 - 10) / 440.0 * 5000)) <= 1);
    CHECK(wind(80, 0, 0.35f).gain == 0.5f);         // capped
    CHECK(wind(30, 1, 0.35f).gain == 0.0f);         // full slipstream silences it
    CHECK(wind(1, 0, 0.35f).rpm == 2500 && wind(200, 0, 0.35f).rpm == 7500);
    Loop ic = in_car(320 / 3.6f, 0, 0.35f);
    CHECK(std::abs(ic.pitch - 1.5f) < 1e-4f && ic.gain == 2.0f);
    // Whine: off with a stock gearbox or the clutch open.
    CHECK(whine(false, 1, 400, 3, 3, 500).gain == 0 && whine(true, 0, 400, 3, 3, 500).gain == 0);
    Loop wh = whine(true, 1, 490, 3, 3, 1047.1976f);
    CHECK(std::abs(wh.gain - 1.0f) < 1e-5f && std::abs(wh.pitch - 3.5f) < 1e-4f);
    // Turbo: level from the load above 0.75, scaled 0.2 per frame; blow-off
    // once when the throttle drops under 75 %, re-armed above 90 %.
    Turbo t;
    t.step(true, 1.0f, 1.75f, 1.75f, 600, 3000, 0);
    t.frame();
    CHECK(t.level() == 25 && std::abs(t.spool().pitch - 3000 / 6144.0f) < 1e-5f && t.blow_off() == 0);
    t.step(true, 0.2f, 1.6f, 0.5f, 100, 2000, 0);
    t.frame();
    CHECK(std::abs(t.blow_off() - (1.6f - 0.8f) / 256 * 2.5f) < 1e-6f);
    t.step(true, 0.2f, 0.5f, 0.5f, 100, 1000, 0);
    t.frame();
    CHECK(t.blow_off() == 0); // not again until re-armed
    t.step(true, 0.95f, 0.5f, 0.6f, 100, 1000, 0);
    t.step(true, 0.5f, 1.2f, 0.6f, 100, 1000, 0);
    t.frame();
    CHECK(t.blow_off() > 0);
    t.step(false, 1, 2, 2, 600, 0, 0);
    t.frame();
    CHECK(t.level() == 0);
}

TEST(sound_course_ambience_gates) {
    // One gated emitter 10 m to the listener's right; a car between the
    // gates fades it in at 1/60 per frame.
    sound::CrssData d;
    sound::CrssEmitter e;
    e.gate_a = {0, 0, 0};
    e.gate_b = {100, 0, 0};
    e.points.push_back({10, 0, 0, 1.0f});
    d.channels[CourseAmbience::Crowds].push_back(e);
    CourseAmbience a;
    a.load(d, [](float x, float, float, float* out) { *out = x; return true; }, 1000.0f);
    CourseAmbience::Listener l;
    l.right = {1, 0, 0};
    for (int i = 0; i < 30; ++i) a.update({l}, {50.0f});
    CHECK(std::abs(a.channel(CourseAmbience::Crowds).r - 0.5f) < 0.01f);
    CHECK(a.channel(CourseAmbience::Crowds).l < 1e-3f);
    for (int i = 0; i < 60; ++i) a.update({l}, {500.0f}); // car left the span
    CHECK(a.channel(CourseAmbience::Crowds).r == 0);
    // Doppler: moving towards the source raises the pitch.
    for (int i = 0; i < 60; ++i) a.update({l}, {50.0f});
    l.velocity = {30, 0, 0};
    a.update({l}, {50.0f});
    CHECK(std::abs(a.channel(CourseAmbience::Crowds).pitch - 331.45f / (331.45f - 30)) < 1e-3f);
    // Tunnels follow the surface flags.
    l.surface_flags = 0x08;
    a.update({l}, {});
    CHECK(a.tunnel() == 1 && a.tunnel2() == 0);
}

TEST(sound_loop_effect) {
    auto bank = std::make_shared<sound::InstBank>();
    Bytes adpcm;
    for (int f = 0; f < 2; ++f) {
        adpcm.push_back(0);
        adpcm.push_back(f ? 0x03 : 0x06);
        for (int i = 0; i < 14; ++i) adpcm.push_back(0x11);
    }
    bank->adpcm = std::make_shared<const Bytes>(adpcm);
    sound::InstRegion r;
    r.volume = 0x4000;
    r.center = 60 << 8;
    r.key_lo = r.key_hi = 60;
    r.adsr1 = 0x000F;
    bank->regions.push_back(r);
    LoopEffect fx;
    fx.set(bank, 0);
    VoiceMixer m;
    fx.update(m, 0.5f, 0.25f, 2.0f);
    CHECK(fx.playing(m));
    std::vector<s16> out(2 * 512);
    m.mix(out.data(), 512);
    CHECK(out[2 * 500] > 0 && std::abs(out[2 * 500] - 2 * out[2 * 500 + 1]) < 8);
    fx.update(m, 0, 0);
    m.mix(out.data(), 512);
    CHECK(!fx.playing(m));
}

TEST(sound_crss_on_disc) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Volume& vol = disc->volume();
    size_t files = 0, with = 0, emitters = 0, gated = 0, failures = 0;
    for (const auto& e : vol.entries()) {
        if (e.path.rfind("courses/US/", 0) != 0) continue;
        Bytes d = vol.read(e);
        if (!course::CourseFile::looks_like_course(d)) continue;
        ++files;
        try {
            auto cf = course::CourseFile::parse(d);
            ByteSpan s = cf.section(d, 35);
            if (s.empty()) continue;
            auto c = sound::CrssData::parse(s);
            ++with;
            for (const auto& ch : c.channels)
                for (const auto& em : ch) {
                    ++emitters;
                    gated += em.gated();
                }
        } catch (const std::exception& ex) {
            std::printf("  %s: %s\n", e.path.c_str(), ex.what());
            ++failures;
        }
    }
    std::printf("  CRSS: %zu/%zu courses, %zu emitters (%zu gated)\n", with, files, emitters, gated);
    CHECK(files == 80 && with == 75 && failures == 0);
    CHECK(emitters > 100 && gated > 50);

    // The wind bank plays through EngineUnit.
    Vfs vfs(&disc->volume());
    EngineUnit u;
    CHECK(u.load(vfs, EngineUnit::kWindBank));
    VoiceMixer m;
    u.update(m, 3750, 0.5f);
    CHECK(u.state().layer[0] == 0 && u.state().layer[1] == 1);
    u.stop(m);
}

TEST(sound_surround_cue) {
    SurroundCue c;
    c.update(0.5f, 0.8f); // in front: channels in phase
    CHECK(c.invert_l == c.invert_r);
    c.update(0.5f, -0.8f); // behind on the right: one side inverted
    CHECK(c.invert_l != c.invert_r);
    c.update(-0.5f, -0.8f); // moving across behind keeps them opposite
    CHECK(c.invert_l != c.invert_r);
    c.update(-0.5f, 0.8f); // back in front: in phase again
    CHECK(c.invert_l == c.invert_r);
    VoiceMixer m;
    CHECK(!m.surround_cue());
}

TEST(sound_menu_effects) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    Vfs vfs(&disc->volume());
    menu::flow::MenuSounds s;
    CHECK(s.load(vfs));
    VoiceMixer m;
    s.play(m, "cursor"); // one region: keyed hard left and hard right
    CHECK(m.active_voices() == 2);
    s.play(m, "next"); // a pair: first region only
    CHECK(m.active_voices() == 3);
    s.play(m, "cursor"); // earlier instances keep playing
    CHECK(m.active_voices() == 5);
    m.stop_all();
    s.set_enabled(false);
    s.play(m, "ok");
    CHECK(m.active_voices() == 0);
}

TEST(sound_bgm_ids) {
    CHECK(std::string(bgm::sequence_name(bgm::kGtMenu)) == "gtmenu01");
    CHECK(std::string(bgm::sequence_name(bgm::kWheelShop)) == "wheelshop01");
    CHECK(bgm::is_stream(bgm::kRace) && !bgm::is_stream(0x23) && !bgm::is_sequence(bgm::kFinish));
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC");
    auto m = sound::MadsIndex::parse(disc->volume().read(*disc->volume().find("bgm/ads.inf")));
    CHECK(int(m.groups.size()) == 17);
    CHECK(m.groups[size_t(bgm::kRace - 0x12)].size() == 25);
    CHECK(m.groups[size_t(bgm::kFinish - 0x12)].at(0).file == "str02a.ads");
    CHECK(m.groups[size_t(bgm::kResult - 0x12)].at(0).title == "result");
    CHECK(m.groups[size_t(bgm::kLicenseBronze - 0x12)].at(0).title == "bronze");
    CHECK(m.groups[size_t(bgm::kFail - 0x12)].at(0).title == "fail");
    CHECK(m.groups[size_t(bgm::kCarGet - 0x12)].at(0).title == "getacar");
    CHECK(m.groups[size_t(bgm::kDemo - 0x12)].at(0).file == "demo_a01.ads");
}

// The race's music cue sequence: start jingle,
// race track at the signal, the finish jingle for every place, licence medal
// and fail jingles, the result / quick menus, pause.
TEST(sound_race_music_cues) {
    using race::RaceMusic;
    using Op = RaceMusic::Action::Op;
    using A = RaceMusic::Action;
    using V = std::vector<A>;
    {   // A single race (start type 2), finishing 4th, then the result menu.
        RaceMusic m;
        RaceMusic::Cues c;
        c.start_type = 2;
        c.holding = true;
        CHECK((m.decide(c, false) == V{A{Op::Play, bgm::kStartLong, false}}));
        CHECK(m.decide(c, true).empty());
        CHECK(m.decide(c, false).empty()); // jingle over, still held: silence
        c.holding = false;                 // the signal
        CHECK((m.decide(c, false) == V{A{Op::Play, bgm::kRace, false}}));
        CHECK(m.decide(c, true).empty());
        CHECK((m.decide(c, false) == V{A{Op::Play, bgm::kRace, false}})); // the next track
        c.paused = true;
        CHECK((m.decide(c, true) == V{A{Op::Pause}}));
        c.paused = false;
        CHECK((m.decide(c, true) == V{A{Op::Resume}}));
        c.finished = true;
        c.position = 4;
        CHECK((m.decide(c, true) == V{A{Op::Play, bgm::kFinish, false}}));
        CHECK(m.decide(c, false).empty()); // after the jingle: silence until the menu
        c.menu = RaceMusic::Menu::Result;
        CHECK((m.decide(c, false) == V{A{Op::PlayIfDifferent, bgm::kResult, true}}));
        CHECK(m.decide(c, true).empty());
        c.menu = RaceMusic::Menu::None;
        CHECK((m.decide(c, true) == V{A{Op::Stop}}));
    }
    {   // A licence test from rest (start type 1): a pass plays only the medal jingle.
        RaceMusic m;
        RaceMusic::Cues c;
        c.kind = RaceMusic::Kind::License;
        c.start_type = 1;
        c.holding = true;
        CHECK((m.decide(c, false) == V{A{Op::Play, bgm::kStartShort, false}}));
        c.holding = false;
        CHECK((m.decide(c, true) == V{A{Op::Stop}})); // no race music in licence tests
        CHECK(m.decide(c, false).empty());
        c.finished = true;
        c.position = 1;
        c.medal = 0;
        CHECK(m.decide(c, false).empty()); // no "jg12" on a licence pass
        c.since_finish = RaceMusic::kMedalDelay / 2;
        CHECK(m.decide(c, false).empty());
        c.since_finish = RaceMusic::kMedalDelay;
        CHECK((m.decide(c, true) == V{A{Op::Play, bgm::kLicenseGold, false}}));
        CHECK(m.decide(c, true).empty());
        c.menu = RaceMusic::Menu::Short;
        CHECK((m.decide(c, false) == V{A{Op::PlayIfDifferent, bgm::kQuickMenu, true}}));
    }
    {   // A failed licence test (flying start: no jingle, no hold).
        RaceMusic m;
        RaceMusic::Cues c;
        c.kind = RaceMusic::Kind::License;
        c.start_type = 3;
        CHECK(m.decide(c, false).empty());
        c.finished = c.failed = true;
        CHECK((m.decide(c, false) == V{A{Op::Play, bgm::kFail, false}}));
        c.since_finish = 5;
        CHECK(m.decide(c, true).empty());
    }
    {   // A licence replay (or disc demo): "jg12" at the finish, no race music, no medal.
        RaceMusic m;
        RaceMusic::Cues c;
        c.kind = RaceMusic::Kind::LicenseReplay;
        c.start_type = 3;
        CHECK(m.decide(c, false).empty());
        c.finished = true;
        c.medal = 0;
        CHECK((m.decide(c, false) == V{A{Op::Play, bgm::kFinish, false}}));
        c.since_finish = 5;
        CHECK(m.decide(c, false).empty());
    }
    {   // Music switched off; attract demo track once.
        RaceMusic m;
        RaceMusic::Cues c;
        c.music_on = false;
        CHECK((m.decide(c, false) == V{A{Op::Stop}}));
        CHECK(m.decide(c, false).empty());
        c.music_on = true;
        c.kind = RaceMusic::Kind::Demo;
        c.start_type = 0;
        CHECK((m.decide(c, false) == V{A{Op::Play, bgm::kDemo, false}}));
        CHECK(m.decide(c, false).empty());
    }
}
