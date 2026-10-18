#pragma once

// Race audio beyond the engine note: the reverb "ambience" model and race
// music.
//
// Ambience (from the game's race sound code):
// every frame the reverb depth (EVOL) moves towards a target by at most 1/36:
//   target 0.5   when the surface under the car has attribute bits
//                (flags & 0x18) == 0x08 (tunnels / covered sections),
//          0.25  when (flags & 0x18) is 0x10 or 0x18,
//          0.125 otherwise (= 0x0FFF, the boot setting);
// EVOL = depth * 32767. The flags are the GTRW triangle flags, i.e.
// SurfaceHit::surface >> 8 (see docs/dev/TRACK.md).

#include "audio/engine_sound.h"
#include "base/bytes.h"
#include "sim/sim_types.h"

#include <memory>

#include <random>
#include <string>
#include <vector>

namespace gt3r {
class AudioEngine;
class Vfs;
class VoiceMixer;
} // namespace gt3r

namespace gt3r::race {

class ReverbAmbience {
public:
    static float target_for(u32 surface_flags);
    // One game frame (1/60 s): steps the depth and writes it to the mixer.
    void tick(VoiceMixer& mixer, u32 surface_flags);
    void reset(VoiceMixer& mixer); // back to the boot depth (0.125)
    float depth() const { return depth_; }

private:
    float depth_ = 0.125f;
};

// Race music, as GT3's race BGM object runs it:
// race tracks (bgm id 0x14) played once each, the next one starting when a
// track ends; finish / medal / fail jingles; the quick menus' music.
// start() / update() / stop() keep the old interface; cues() drives the
// event-completion states every frame.
class RaceMusic {
public:
    // Race: every one-player race and solo run; TwoPlayer: 2P battle (the
    // finish jingle only for the winner); License: no race tracks; Demo: attract.
    // LicenseReplay: a licence run replayed (or its disc demo): the finish posts
    // "jg12" as in races, no medal jingle.
    enum class Kind : u8 { Race, TwoPlayer, License, Demo, LicenseReplay };
    // The quick menu on screen: Result (race / 2P results: "result"), Short (pre-race,
    // time trial, licence, machine test: "shortmenu-2").
    enum class Menu : u8 { None, Short, Result };
    struct Cues {
        bool music_on = true;
        Kind kind = Kind::Race;
        bool paused = false;
        int start_type = 2;         // event start type (+0x1F3): 1 / 2 have a start jingle
        bool holding = false;       // the cars are held before the start signal
        bool finished = false;
        int position = 0;           // finishing place (1 = won; used by 2P)
        bool failed = false;        // licence test failed / time trial voided
        int medal = -1;             // licence medal 0 gold .. 2 bronze, -1 none
        double since_finish = 0;    // seconds since the finish
        Menu menu = Menu::None;
    };
    // One BGM request. Op Play restarts, PlayIfDifferent keeps the same piece.
    struct Action {
        enum class Op : u8 { Play, PlayIfDifferent, Stop, Pause, Resume } op;
        int id = -1;
        bool loop = false;
        bool operator==(const Action&) const = default;
    };
    // The pure decision for one frame (also used by the tests).
    // `music_playing`: the current BGM is still sounding.
    std::vector<Action> decide(const Cues& c, bool music_playing);
    // decide() and apply the result through AudioEngine::bgm().
    void cues(AudioEngine& audio, Vfs& vfs, const Cues& c);

    // Seconds after the result at which a licence medal's jingle starts: the end of the
    // presenter's 30-frame dim, 37 frames after the result (presentation.md A4).
    static constexpr double kMedalDelay = 37.0 / 60.0;

    // A new run: the next cues() call starts the music afresh.
    void reset() { seen_ = false; }
    void stop(AudioEngine& audio);

private:
    bool active_ = false;
    // cues() state
    bool seen_ = false, was_on_ = false, was_paused_ = false, was_finished_ = false, jingle_ = false;
    bool was_holding_ = false, start_jingle_ = false, demo_played_ = false;
    bool medal_played_ = false;
    Menu was_menu_ = Menu::None;
};

// Engine notes of the other cars, placed like GT3 does: volume
// min(1, 32 / d) (d = horizontal distance to the camera, >= 0.25 m), Doppler
// (c + v_listener) / (c + v_source) along the camera->car direction
// (c = 331.45 m/s), per-view channel gains ("other car" columns). GT3 sounds
// every car and lets the voice allocator drop quiet voices; we give the
// nearest `max_voiced` cars a note (4 voices each) to stay inside 46 voices.
class OpponentEngines {
public:
    struct Car {
        u16 sound_id = 0;
        u8 variant = 0;
        float cd = 0.35f; // drag coefficient for the wind unit
    };
    void load(Vfs& vfs, const std::vector<Car>& cars);
    // `listener`/`listener_velocity`: camera position and the followed car's velocity.
    // `right`/`forward`: the listener's (camera's) unit axes, for GT3's stereo pan.
    void update(VoiceMixer& mixer, const sim::Vec3& listener, const sim::Vec3& listener_velocity,
                const sim::Vec3& right, const sim::Vec3& forward, const std::vector<const sim::VehicleState*>& states,
                double dt, const std::vector<float>* slipstream = nullptr);
    void stop(VoiceMixer& mixer);

    int max_voiced = 5;    // cars with an engine note at once
    int max_wind = 2;      // of those, the nearest with their wind unit (voice budget)
    EngineSound::View view = EngineSound::View::ChaseOther;
    // GT3: full volume within 32 m, then 32 / d.
    static float attenuation(float horizontal_distance);
    // Doppler factor for a source moving with v_src and a listener with v_lis
    // along the unit direction listener -> source (speed of sound 331.45 m/s).
    static float doppler(const sim::Vec3& dir, const sim::Vec3& v_src, const sim::Vec3& v_lis);

private:
    std::vector<std::unique_ptr<EngineSound>> engines_;
    // The wind / road unit (rn_01.es) of each car: other cars are heard with the
    // "other car" wind gain (in-car / whine / turbo gains are 0 for other cars).
    std::vector<std::unique_ptr<EngineUnit>> winds_;
    std::vector<float> cd_;
    std::vector<bool> active_;
};

} // namespace gt3r::race
