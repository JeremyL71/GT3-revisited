#pragma once

// Replays: the per-tick driver inputs of a run plus everything needed to
// re-simulate it (course, car, gearbox setting, start pose, respawns). The
// vehicle simulation is deterministic (same inputs and dt -> bit-identical
// states on the same build), so playing the inputs back through a fresh
// vehicle reproduces the run. Inputs are quantised *before* they are fed to
// the simulation while recording, so the stored values are exactly the ones
// the simulation saw.
//
// File format (little endian), ".gt3replay" in <user data>/replays/:
//   0x00 "GT3RRPL1"
//   u32 version (9; 1..8 are still read), u32 tick count N, f64 tick rate (Hz)
//   string course, string car (u16 length + bytes), u8 automatic, u8 paint
//   pose start (f32 x, y, z, qw, qx, qy, qz)
//   u32 event count; events {u32 tick, u8 kind, pose}
//   f64 best lap time (< 0 = none), u32 best lap first tick, u32 best lap end tick
//   f32 final x, y, z (position after the last tick, for verification)
//   v2: string event (race with opponents; empty = time trial), u16 count + strings opponents
//   v3: string car2 (2P battle; empty = none), then (with car2) N x input for player 2 after
//       player 1's inputs. Respawn events of player 2 have kind 2.
//   v4: f32 start speed (m/s; rolling / flying starts, 0 = from rest), after car2
//   v6: u32 paint seed (the race generator's state before the paint draw), after the start speed
//   N x input {s16 steer * 32767, u16 throttle * 65535, u16 brake * 65535, u8 flags}
//        flags: 1 handbrake, 2 shift up, 4 shift down, 8 automatic,
//        v5: 16 controller kind 2 (analog), 32 kind 3 (digital steering); neither = 0
//   v7 (after the v6 paint seed, before the inputs): string comment (the file name the
//       memory-card pages show and edit), u8 kind (ReplayKind), u16 laps completed, f64 total
//       time (< 0 = none), f32 power (PS), string car model, string wheel, s64 saved (unix
//       seconds, 0 = unknown), u32 n + n bytes car spec (sim/car_spec_io.h; 0 = the database car)
//   v8 (after the v7 car spec): the engine condition the run started with (physics §10.3):
//       u32 odometer (2 m units), f32 running-in / wear, f32 oil (older replays: a fresh engine)
//   v9 (after the v8 condition): u8 the field's rules as the run had them: bits 0-1 the grid
//       by power-to-weight (GT mode), bits 2-3 CPU respawn ([race] respawn); each 0 = not
//       recorded, 1 = off, 2 = on. Then with car2, the 2P battle: u8 recorded (0 = no, nothing
//       follows); u16 laps, f32 boost amount, boost start, boost full (m), f32 handicap (m),
//       u8 tyre damage, string model2, string wheel2, u8 paint2, u32 n + n bytes player 2's
//       car spec (0 = the database car)

#include "base/game_random.h"
#include "race/backend.h"
#include "sim/car_spec.h"
#include "sim/sim_types.h"

#include <memory>

#include <optional>
#include <string>
#include <vector>

namespace gt3r::race {

// The value the simulation sees after a save/load round trip.
sim::ControlInputs quantize(const sim::ControlInputs& in);

struct ReplayEvent {
    enum Kind : u8 { Respawn = 1, Respawn2 = 2 };
    u32 tick = 0; // applied before simulating this tick
    Kind kind = Respawn;
    TrackPose pose;
};

// v9, 2P battle: the battle as its run had it (the Options menu's laps, catch-up boost,
// handicap and tyre damage; player 2's car from its car page). Playback builds the battle with
// these, not with the Options menu's settings at the time it is watched.
struct ReplayBattle {
    int laps = 0;
    float boost_amount = 0, boost_start = 0, boost_full = 1; // FieldConfig's catch-up boost
    float handicap_m = 0;                                    // FieldConfig::handicap_m
    bool tyre_damage = false;                                // shipped wear on both cars
    std::shared_ptr<const sim::CarSpec> spec2; // player 2's car as raced (null = the database car)
    std::string model2, wheel2;                // its model / fitted wheel overrides ("" = its own)
    int paint2 = 1;
};

// What kind of run a replay holds (the memory-card info panel's "Mode").
enum class ReplayKind : u8 {
    Unknown = 0,
    TimeTrial = 1,  // our time trial / arcade time trial or free run
    ArcadeRace = 2, // arcade single race
    TwoPlayer = 3,  // 2 player battle
    GtRace = 4,     // GT mode event / championship round
    License = 5,    // licence test
    Practice = 6,   // GT mode free run / machine test, quick race practice
    Online = 7,
};
const char* replay_kind_name(ReplayKind k); // "Arcade", "GT", "License", ...

struct Replay {
    static constexpr u32 kVersion = 9;
    std::string course, car; // database labels
    bool automatic = true;
    int paint = 0;
    double tick_hz = sim::kDefaultTickHz;
    TrackPose start;
    std::vector<sim::ControlInputs> inputs; // one per tick (already quantised)
    // Player 1's scripted-driver kind per tick (sim::Vehicle::set_driver_kind: 0 the driver,
    // 1 the AI drives, 5 the stop driver (kinds 5-7 act alike in the physics), 8 parked; empty =
    // all 0). Stored in input flag bits 64 (1), 128 (5) and both (8).
    std::vector<u8> driver_kind;
    std::vector<ReplayEvent> events;        // sorted by tick
    double best_lap = -1;
    u32 best_lap_begin = 0, best_lap_end = 0; // tick range [begin, end) of the best lap
    sim::Vec3 final_position;
    std::string event;                  // race with opponents (event label); empty = time trial
    std::vector<std::string> opponents; // opponent car labels, grid order
    std::string car2;                    // 2P battle: player 2's car label
    std::vector<sim::ControlInputs> inputs2; // player 2's inputs (same length as inputs)
    float start_speed = 0;               // m/s: rolling / flying start (0 = from rest)
    u32 paint_seed = GameRandom::kBootSeed; // race_info_random() before the paint draw (older replays: the boot seed)
    // v7: what the theater and the memory-card pages show, and the car the run used.
    std::string comment;                 // file name ("" = none given)
    ReplayKind kind = ReplayKind::Unknown;
    int laps = 0;                        // laps completed by player 1
    double total_time = -1;              // race / run time at the end (s), < 0 = none
    float power_ps = 0;                  // the car's power
    std::string car_model, wheel;        // model / fitted wheel overrides ("" = the car's own)
    s64 saved = 0;                       // when it was saved (unix seconds)
    std::shared_ptr<const sim::CarSpec> spec; // the run's car (null = the database car)
    sim::EngineCondition condition;      // v8: player 1's engine condition at the start (zeros = fresh)
    // v9: the field's rules as the run had them; playback rebuilds a race's field with these
    // (nullopt: not recorded, the playback's own). Before v9 the grid comes from the kind
    // (GT mode's races, ReplayKind::GtRace, were the ones lined up by power-to-weight).
    std::optional<bool> grid_by_power;   // FieldConfig::grid_by_power
    std::optional<bool> cpu_respawn;     // FieldConfig::cpu_respawn
    std::optional<ReplayBattle> battle;  // v9 with car2 (nullopt: not recorded, the playback's own)

    u32 ticks() const { return u32(inputs.size()); }
    double duration() const { return double(inputs.size()) / tick_hz; }

    Bytes serialize() const;
    static Replay parse(ByteSpan data); // throws FormatError
};

// Records a run tick by tick.
class ReplayRecorder {
public:
    void begin(std::string course, std::string car, bool automatic, int paint, const TrackPose& start, double hz,
               std::string event = {}, std::vector<std::string> opponents = {});
    // Returns the quantised inputs to feed to the simulation for this tick.
    sim::ControlInputs record(const sim::ControlInputs& in);
    // Replaces this tick's recorded input (the computer took over after record());
    // returns the quantised value.
    sim::ControlInputs replace_last(const sim::ControlInputs& in) {
        sim::ControlInputs q = quantize(in);
        if (active_ && !replay_.inputs.empty()) replay_.inputs.back() = q;
        return q;
    }
    // 2P: player 2's inputs for the same tick (call before record()).
    sim::ControlInputs record_second(const sim::ControlInputs& in);
    // This tick's scripted-driver kind (after record()); only 0, 1 and 5 are stored.
    void set_driver_kind(int kind);
    void set_second_car(std::string car2) { replay_.car2 = std::move(car2); }
    void set_start_speed(float mps) { replay_.start_speed = mps; }
    void set_paint_seed(u32 seed) { replay_.paint_seed = seed; }
    void set_engine_condition(const sim::EngineCondition& c) { replay_.condition = c; }
    void respawn_second(const TrackPose& pose);
    void respawn(const TrackPose& pose); // before the next record()
    void lap(double time, u32 begin_tick, u32 end_tick); // a completed lap; keeps the best
    void finish(const sim::Vec3& final_position) { replay_.final_position = final_position; }
    // Puts a finished recording back (e.g. after a demonstration run); recording stops.
    void assign(Replay r) { replay_ = std::move(r); active_ = false; }
    u32 tick() const { return replay_.ticks(); }
    const Replay& replay() const { return replay_; }
    bool active() const { return active_; }

private:
    Replay replay_;
    bool active_ = false;
};

// Steps through a replay.
class ReplayPlayer {
public:
    explicit ReplayPlayer(const Replay* r = nullptr) : r_(r) {}
    void reset() { tick_ = 0, event_ = 0; }
    bool done() const { return !r_ || tick_ >= r_->ticks(); }
    u32 tick() const { return tick_; }
    // Events due before the current tick (call before next_inputs()).
    const ReplayEvent* pending_event();
    sim::ControlInputs next_inputs(); // advances one tick
    // 2P: player 2's inputs for the tick the last next_inputs() returned.
    sim::ControlInputs second_inputs() const;
    // Scripted-driver kind for the tick the last next_inputs() returned.
    int driver_kind() const;

private:
    const Replay* r_;
    u32 tick_ = 0;
    size_t event_ = 0;
};

// Saving/loading under <dir> (created on demand). save() returns the file path.
std::optional<std::string> save_replay(const Replay& r, const std::string& dir);
struct ReplayFile {
    std::string path, name; // name = file name without directory
};
std::vector<ReplayFile> list_replays(const std::string& dir); // newest first

} // namespace gt3r::race
