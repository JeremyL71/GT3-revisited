#pragma once

// A race with opponents: the player's car and the AI cars in one sim::World
// (car-to-car contact, slipstream, per-car time scale), the AI driven by
// ai::AiDriver with the event's parameters and rubber band (see docs/dev/AI.md).
//
// The player's car is exposed as a RaceVehicle: stepping it steps the whole
// field (AI inputs first, then World::step), so the race screen's fixed-tick
// loop, replays and HUD work unchanged. Everything is deterministic (AI and
// World), so a replay of the player's inputs reproduces the whole race.
//
// Start: cars are held until the start signal (race_time < 0 during the
// countdown); the player sits on the last grid slot behind the opponents.

#include "race/backend.h"
#include "race/net_race.h"
#include "sim/models.h"

#include <memory>
#include <string>
#include <vector>

namespace gt3r::data {
struct Event;
} // namespace gt3r::data

namespace gt3r::race {

struct FieldConfig {
    std::string player;                 // car (or variant) label of the player
    std::shared_ptr<const sim::CarSpec> player_spec; // when set, used instead of `player`
    std::string event;                  // event label (opponents, AI parameters, rubber band); may be empty
    std::vector<std::string> opponents; // car variant labels; empty = the event's entries
    int max_opponents = 5;
    // Second human (2P battle): car label or spec; empty = none.
    std::string second_player;
    std::shared_ptr<const sim::CarSpec> second_spec;
    double countdown = 3.0;             // seconds before the start signal (0 = rolling start: no hold)
    // Course category (course table 34 +8: 0 tarmac, 1 wet Route 5, 2 dirt): picks the CPU drivers'
    // built-in parameter set (ai.md §9.1) before the event's per-driveline-class overrides.
    int course_category = 0;
    // 2P battle options (the original's options record, docs/dev/RACE.md "Options"):
    // boost = catch-up for the trailing player as a time-step scale 1 + A x (gap - start) /
    // (full - start) (full A from `full` m or a lap down), A = 0 = off (low 0.15 / 20 / 1000,
    // high 0.20 / 10 / 200); handicap metres (> 0: player 2 starts back, < 0: player 1), both
    // cars placed on the line before the start: the leader 10 m back, the other max(|h|, 20) m.
    float boost_amount = 0, boost_start = 0, boost_full = 1;
    float handicap_m = 0;
    // GT-mode grid: opponents ranked by power-to-weight (kg per PS, lightest per PS on pole);
    // otherwise they take the slots in the given order (arcade: the random draw).
    bool grid_by_power = false;
    // Time-limited race (endurance events with 0 laps, event +0x1E4 minutes): a car finishes at
    // its first line crossing at or after it (finishes_at_line).
    double time_limit = 0;
    // Non-original fail-safe ([race] respawn): put a CPU car that is lost or has made no
    // progress for 12 s back on the track. The original never respawns a car.
    bool cpu_respawn = true;
    // Rolling start: every opponent starts moving at this speed (km/h); 0 = from rest.
    float start_kmh = 0;
    // Tyre wear of the race (model::wear_params of event +0x1BD..+0x1C3); off by default.
    sim::model::WearParams wear;
    // Online battle (docs/dev/ONLINE.md): the other players' cars. They take the opponent
    // indices 0..n-1 (no AI): never stepped here, posed from the network before each step
    // (set_remote_pose) and collided as sim::World kinematic cars. `slot` is the grid slot.
    struct Remote {
        std::string car; // car label
        int slot = 0;
        std::shared_ptr<const sim::CarSpec> spec; // their car as raced (garage parts); null = the stock car
    };
    std::vector<Remote> remote;
    int player_slot = -1; // the local player's grid slot (-1: behind the opponents)
};

// How an event starts (event record, database table 32; docs/dev/RACE.md "Starts"):
//   +0x198 u8 initial speed (km/h; 0 for every race), +0x199 u8 laps, +0x1B5 u8 cars
//   (player included), +0x1E4 u8 time limit (minutes), +0x1E6 s16 flying-start
//   distance (metres before the line, negative), +0x1F2 u8 countdown seconds (type 0),
//   +0x1F3 u8 start type: 0 demo / machine test, 1 standing (240 frames), 2 standing
//   (600 frames, every single race), 3 rolling (GT endurance) or flying (time attack,
//   moving licence tests: initial speed > 0 at the flying-start distance).
struct EventStart {
    int type = 2;
    int laps = 0, cars = 0;
    float initial_kmh = 0;
    double time_limit_s = 0;
    float flying_m = 0;    // metres before the line (positive)
    double countdown = 10; // seconds the cars are held (0 = no hold)
    bool rolling() const { return type == 3 && initial_kmh <= 0; }
    bool flying() const { return type == 3 && initial_kmh > 0; }
};
EventStart event_start(const data::Event& ev);
// Rolling start: the computer drives the player's car for this long after the start
// (300 frames), every car starting at kRollingKmh.
constexpr double kRollingAutoDrive = 5.0;
constexpr float kRollingKmh = 80.0f;

// Grid slot point -> car position: the slot marks the nose, so the car is moved back
// along its heading by its front extent (physics spec §9.2).
sim::Vec3 grid_back(const sim::Vec3& slot, const sim::Quat& heading, const sim::VehicleSetup& setup);
// The same from a car spec (builds the vehicle's setup).
sim::Vec3 grid_back(const sim::Vec3& slot, const sim::Quat& heading, const sim::CarSpec& spec);

// kg per PS of a car spec (GT mode's grid order: lowest on pole).
float power_to_weight(const sim::CarSpec& spec);

class RaceField {
public:
    // nullptr with *error set when the track runtime is unavailable or no car could be built.
    static std::unique_ptr<RaceField> create(const RaceTrack& track, const data::CarDatabase& db,
                                             const FieldConfig& cfg, std::string* error = nullptr);
    virtual ~RaceField() = default;

    virtual RaceVehicle& player() = 0;
    // The player's start pose (behind the opponents).
    virtual TrackPose player_grid() const = 0;

    // Opponents (index 0..n-1).
    virtual int opponents() const = 0;
    virtual const std::string& opponent_label(int i) const = 0;
    virtual const std::string& opponent_model(int i) const = 0; // cars/day/<model>
    virtual const sim::VehicleState& opponent_state(int i) const = 0;
    virtual const sim::VehicleState& opponent_prev_state(int i) const = 0; // before the last tick (interpolation)
    virtual const sim::VehicleSetup& opponent_setup(int i) const = 0;
    virtual int opponent_laps(int i) const = 0;

    // Seconds since the start signal (< 0 during the countdown).
    virtual double race_time() const = 0;
    // Player's position (1 = leader) in the race order (standings()), as the HUD shows it.
    virtual int player_position() const = 0;
    // Race time of the player's / second player's finish (finishes_at_line), < 0 = still racing.
    virtual double player_finish() const = 0;
    virtual double second_finish() const = 0;
    virtual int contacts_last_tick() const = 0;
    virtual const std::vector<std::string>& labels() const = 0; // opponent labels as configured

    // Second human player (2P battle), when configured. Its inputs are set
    // before each player().step() and used by that step.
    virtual bool has_second() const = 0;
    virtual void set_second_inputs(const sim::ControlInputs& in) = 0;
    virtual void set_second_reverse(bool on) { (void)on; } // player 2's reverse request
    virtual const sim::VehicleState& second_state() const = 0;
    virtual const sim::VehicleState& second_prev_state() const = 0;
    virtual const sim::VehicleSetup& second_setup() const = 0;
    virtual TrackPose second_grid() const = 0;
    virtual void reset_second(const TrackPose& p) = 0;
    virtual float second_impact() const = 0;
    virtual float second_boost() const = 0;
    virtual const sim::Vehicle* second_vehicle() const = 0;
    virtual const sim::Vehicle* opponent_vehicle(int i) const = 0;
    // The brake pedal the car was given on the last tick (lamp state).
    virtual float opponent_brake(int i) const = 0;
    // Strongest car-car impulse on opponent i in the last step (N s), for its clash sound.
    virtual float opponent_car_impulse(int i) const { (void)i; return 0; }
    virtual float second_brake() const = 0;
    virtual float second_wall_impulse() const = 0;
    virtual float second_car_impulse() const = 0;
    struct LapInfo {
        int laps = 0;
        bool started = false, wrong_way = false;
        double current = 0, last = -1, best = -1, race_distance = 0;
    };
    virtual LapInfo second_laps() const = 0;
    virtual int second_position() const = 0;

    // The race order: every car (player = -1, second player = -2, opponents 0..n-1), ranked
    // by ranks_ahead after every tick (ties keep the previous order).
    struct Standing {
        int car = -1;
        bool finished = false;
        double time = 0; // race time at the finish (s)
        int laps = 0;    // laps completed (frozen at the finish)
        double distance = 0;
    };
    virtual void set_laps(int laps) = 0;
    // Opponents that entered the pit lane since the last call ("%s Pitting in...").
    virtual std::vector<int> take_pit_entries() = 0;
    virtual bool opponent_in_pits(int i) const = 0;
    virtual std::vector<Standing> standings() const = 0;

    // Online battle: remote players (opponents with FieldConfig::remote entries).
    virtual bool opponent_remote(int i) const { (void)i; return false; }
    // The remote car's pose at the end of the coming tick (used by the next step).
    virtual void set_remote_pose(int i, const RemoteCar::Pose& p) { (void)i, (void)p; }
    // Its owner's finish time (the owner is the authority), or that it left (no contact any more).
    virtual void set_remote_finish(int i, double race_time) { (void)i, (void)race_time; }
    virtual void set_remote_retired(int i) { (void)i; }
    virtual bool opponent_retired(int i) const { (void)i; return false; }
    // Contacts of the last step between the player's car and remote cars: the impulse the remote
    // car felt (its owner applies it if its machine missed the hit), and when the player's car last
    // touched each remote car (race time; < -1e9 = never).
    struct RemoteHit {
        int opponent = -1;
        float point_x = 0, point_z = 0; // world
        float jx = 0, jz = 0;           // impulse on the remote car, N*s
    };
    virtual std::vector<RemoteHit> take_remote_hits() { return {}; }
    virtual double last_contact_with(int i) const { (void)i; return -1e18; }
    // When this machine saw a remote car complete the race distance (race time; < 0 = not yet).
    virtual double remote_observed_finish(int i) const { (void)i; return -1; }
    // A hit on the player's car that another player's machine saw (contact message).
    virtual void apply_player_impulse(float point_x, float point_z, float jx, float jz) { (void)point_x, (void)point_z, (void)jx, (void)jz; }
    // A ghosted remote car is drawn translucent and has no contact (lobby ping threshold).
    virtual void set_remote_ghost(int i, bool on) { (void)i, (void)on; }
    virtual bool opponent_ghost(int i) const { (void)i; return false; }
};

// The race rules of the original (presentation spec, "Race order and finish").
// Ranking: more laps completed first; at equal laps two running cars by race distance, a
// running car ahead of a finished one, two finished cars by finish time.
bool ranks_ahead(const RaceField::Standing& a, const RaceField::Standing& b);
// Whether a car finishes at a line crossing (cars are judged in race order, the leader
// first): it has completed the race's laps (race_laps > 0), the car ranked first has
// finished, or the crossing is at or after the time limit (seconds, 0 = none). There is no
// other finish: a car that stops short never finishes.
bool finishes_at_line(int laps_completed, int race_laps, bool leader_finished, double race_time, double time_limit);

} // namespace gt3r::race
