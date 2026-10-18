#pragma once

// Per-car race sound effects, following GT3's race sound code (research
// summary in docs/dev/RACE.md "Race sounds"): tyre squeal / slide / dirt /
// kerb rumble / road roar per wheel and surface, collision one-shots, and the
// per-camera channel gain table.
//
// Pure functions are exposed for tests; CarSfx drives a RaceSfx for one car.

#include "audio/car_sound_model.h"
#include "audio/engine_sound.h"
#include "race/race_sfx.h"
#include "sim/sim_types.h"
#include "sim/vehicle.h"

#include <array>
#include <string>
#include <vector>

namespace gt3r::race {

// GT3's camera views for sound: 0 in-car target, 1 in-car other, 2 chase
// target, 3 chase other, 4 any other camera (replay/TV).
enum class SoundView : u8 { InCarTarget, InCarOther, ChaseTarget, ChaseOther, Other };

// Channel gains (table of the race sound code), by view and channel.
enum SfxChannel : int {
    kChEngine = 0, kChExhaust, kChTurbo, kChBlowOff, kChWhine, kChClash, kChRumble, kChInCar, kChWind,
    kChRoad, kChRain, kChSquealTarmac, kChUnused, kChSlide, kChDirt, kChCount
};
float sfx_gain(SoundView v, SfxChannel ch);

struct TyreSound {
    float vol = 0, pitch = 1;
};
// Squeal / slide (surface class 0 tarmac, 1 loose, 2 wet): v_lat = lateral hub
// speed (m/s), slip = slip ratio (< 0 wheelspin, > 0 braking), hub_speed (m/s).
TyreSound tyre_squeal(int cls, float v_lat, float slip, float hub_speed);
TyreSound kerb_rumble(float load_n, float wheel_speed);
TyreSound road_noise(float tyre_width, float wheel_speed);
// Collision strength 0..1 (< 0.2 ignored): wall |j| / 50000, car-car |j| / 7500.
float wall_strength(float impulse);
float car_strength(float impulse);
// Volume of a collision one-shot: channel gain x strength x 0.6 x 32/d, d = camera to car (m,
// 0.25 at least), capped at 1. In the cockpit d is about a metre, so any audible hit is full.
float clash_volume(SoundView view, float strength, float distance);

class CarSfx {
public:
    explicit CarSfx(std::string slot_prefix = "p1_") : prefix_(std::move(slot_prefix)) {}
    // One game frame for the followed car. `view` selects the gains; tyre_widths per axle;
    // `distance` = camera to the car (m), for the collision one-shots.
    void update(RaceSfx& sfx, VoiceMixer& mixer, const sim::VehicleState& s, const sim::VehicleSetup& setup,
                SoundView view, float wall_impulse, float car_impulse, float distance);
    void stop(RaceSfx& sfx, VoiceMixer& mixer);

private:
    std::string prefix_;
    std::array<int, 4> airborne_{}; // frames of grace after a wheel leaves the ground
};

// The car's sounds besides the engine and tyres (docs/formats/sound.md "Race car
// sounds besides the engine"): the rn_01.es wind unit, the in-car loop, the
// transmission whine and the turbo spool / blow-off. One per voiced car.
class CarExtraSounds {
public:
    // Per physics step (turbo model).
    struct Step {
        bool turbo = false;
        float throttle = 0, load = 0, torque = 0;
        int lag0 = 0, lag1 = 0;
    };
    // Per game frame.
    struct Frame {
        float u = 0;           // forward speed, m/s
        float slipstream = 0;  // filtered slipstream factor 0..1
        float cd = 0.35f;      // drag coefficient C_d (body record +0x39 / 100)
        bool whine = false;    // non-stock gearbox (gear part +0x10 != 0)
        int clutch = 1;        // 0 = disengaged
        float torque = 0;      // N m
        float ratio = 1, first_ratio = 1;
        float shaft = 0;       // driven axle speed, rad/s
        float x = 0, z = 1;    // position in listener space (m: right, forward)
        float dist_gain = 1;   // min(1, 32/d); 1 for the followed car
        float doppler = 1;
        SoundView view = SoundView::ChaseTarget;
    };
    explicit CarExtraSounds(std::string slot_prefix = "p1_") : prefix_(std::move(slot_prefix)) {}
    bool load(Vfs& vfs);
    void step(const Step& s);
    void update(RaceSfx& sfx, VoiceMixer& mixer, const Frame& f);
    void stop(RaceSfx& sfx, VoiceMixer& mixer);
    void reset() { turbo_.reset(), prev_load_ = 0; }
    const car_sound::Turbo& turbo() const { return turbo_; }

private:
    std::string prefix_;
    EngineUnit wind_;
    car_sound::Turbo turbo_;
    float prev_load_ = 0;
};

} // namespace gt3r::race
