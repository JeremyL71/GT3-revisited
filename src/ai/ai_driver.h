#pragma once

// AI ("CPU") drivers (milestone M3). Behaviour follows the original game's AI; the
// implementation and what is spec-confirmed vs. placeholder are described in
// docs/dev/AI.md.
//
//   auto course = std::make_shared<ai::AiCourse>(*track);         // once per track
//   ai::DriverParams p = ai::DriverParams::builtin(0);
//   ai::apply_event(p, *db.event("Gsunday_e_0001"), 0);            // event difficulty
//   ai::AiDriver drv(course, p, /*start_offset=*/slot);       // car: adapt_spec_for_ai, set_cpu_driver(true)
//   drv.reset(car.state());
//   every tick (1/60 s), for every AI car i (cars live in a sim::World):
//       std::vector<ai::CarSnapshot> cars = ...;                   // all cars, this tick
//       world.set_inputs(i, drv.update(world.vehicle(i), {race_time, cars, i}));
//       world.vehicle(i).set_reverse_request(drv.reverse_request());
//       world.set_time_scale(i, rubber_band.scale(player_distance - car_distance));
//   world.step(1.0 / 60);
// AI cars should be built with adapt_spec_for_ai(spec). ai::AiRace
// (ai/ai_race.h) is a complete example; see docs/dev/AI.md.
//
// The driver is deterministic: update() depends only on its arguments and
// its own state (no clocks, no RNG), so replays stay in sync.
//
// Cars are physically independent here (car-to-car contact belongs to the
// physics workstream); the driver avoids others through its line choice and
// throttle, as GT3's does.

#include "sim/sim_types.h"

#include <memory>
#include <span>
#include <string>
#include <vector>

namespace gt3r::data {
struct Event;
}
namespace gt3r::sim {
class Vehicle;
struct CarSpec;
}
namespace gt3r::track {
class Track;
}

namespace gt3r::ai {

// ---------------------------------------------------------------------------
// Driver parameters (spec §9.1). Units are SI.
struct DriverParams {
    float brake_factor = 1.0f;      // k_brake: braking distance multiplier (100 / brake %)
    float speed_scale = 1.0f;       // corner speed scale
    float speed_offset = 0.0f;      // m/s added to corner speeds
    float yaw_gain = 1.2f;          // K_y: wanted slip angle per rad/s of yaw-rate error
    float sat_throttle_cut = 1.0f;  // K_t: throttle cut per unit of steering over-saturation
    float sat_brake = 0.0f;         // K_b: brake added per unit of steering over-saturation
    float lookahead_full_speed = 50.0f; // v_L, m/s
    float lookahead_min = 15.0f, lookahead_max = 40.0f; // L0, L1 (m)
    float grip_front_weight = 1.0f; // w: front/rear blend of the grip used for corner speeds
    float corner_bonus = 20.0f / 3.6f; // m/s added to the arc target while in an arc
    int corner_bonus_ticks = 0;     // ticks in the arc before the bonus applies
    float steer_rate = 1.5708f;     // rad/s slew of the road-wheel angle
    float pit_speed = 80.0f / 3.6f; // m/s on the pit-lane line
    float throttle_scale = 1.0f;    // event throttle percentage
    int speed_trim = 0;             // +-2 steps of 10 km/h (debug trim)
    // [placeholder, not GT3] scales the tyre grip the planner assumes, to
    // match our physics until its tyre model follows the game.
    float grip_scale = 1.0f;
    // [placeholder, not GT3] scales the additive corner speed boosts (arc
    // bonus, per-point km/h adjustments), which were tuned for GT3's grip.
    float boost_scale = 1.0f;
    // [placeholder, not GT3] also steer against a yaw-rate overshoot (GT3
    // only corrects a deficit and relies on the AI's stability aid).
    bool counter_steer = false;

    // One of the three built-in sets (by car spec byte 0x31: 1 -> 1, 2 -> 2, else 0).
    static DriverParams builtin(int set);
};

// Event difficulty (spec §9.2): per chassis class (0..3) throttle %, brake %,
// corner speed % and corner speed offset from the event record.
void apply_event(DriverParams& p, const data::Event& ev, int chassis_class = 0);

// Rubber band (spec §9.3): per-car simulation time scale from the race
// distance gap to the reference car (the player).
struct RubberBand {
    float slow_amount = 0, slow_start = 0, slow_full = 0; // car ahead of the reference
    float fast_amount = 0, fast_start = 0, fast_full = 0; // car behind the reference
    float global = 1.0f;
    bool enabled() const { return slow_amount > 0 || fast_amount > 0 || global != 1.0f; }

    static RubberBand from_event(const data::Event& ev);
    // gap = reference race distance - car race distance (metres, laps included).
    // Multiply the car's dt by the result.
    float scale(double gap) const;
    // The same without the global scale (one link of the chain below).
    float factor(double gap) const;
    // Event pass mode 0 (every shipped event; spec §14): a chain from the reference car R
    // along the ranking (race distance, laps included). Each car is scaled against its
    // neighbour towards R and its factor multiplies that neighbour's, so the factors
    // compound outward; R stays at 1. `no_speedup` cars (off-course / pit surface) get no
    // speed-up factor of their own. Returns the dt scale per car (global scale included
    // for every car but R).
    std::vector<float> chain(const std::vector<double>& distance, int reference,
                             const std::vector<bool>& no_speedup = {}) const;
};

// Per-car spec adjustments for CPU cars: the steering lock is raised to the
// AI's 45-degree clamp (spec §6), since physics clamps steer_angle to it.
// Also call vehicle.set_cpu_driver(true) (the CPU clutch request rule,
// no aids) and, each tick, vehicle.set_reverse_request(drv.reverse_request()).
void adapt_spec_for_ai(sim::CarSpec& spec);

// ---------------------------------------------------------------------------
// AI lines of one course, prepared for driving (shared by all AI cars).
// Line numbers are the game's: 1..7 = file lines 0..6 (spec §2.1).
class AiCourse {
public:
    explicit AiCourse(const track::Track& track);
    ~AiCourse();
    AiCourse(const AiCourse&) = delete;
    AiCourse& operator=(const AiCourse&) = delete;

    static constexpr int kLineA = 1, kLineB = 2, kPitLane = 3, kPitStop = 4, kSideMap = 5, kMain = 6;

    struct Point {
        sim::Vec3 pos;           // world, y on the road
        u8 kind = 0;             // 0 straight, 1 arc (segment starting here)
        u8 raw_kind = 0;         // flags byte 0 as stored (the side map uses it as a flag)
        s8 brake_adjust = 0;     // percent (spec §2.2 byte 1)
        s8 speed_adjust = 0;     // km/h (byte 2)
        float progress = 0;      // track distance [0, length)
        float radius = 0;        // signed, > 0 turns left; 0 on straights
        float slope = 0;         // rad, uphill positive [placeholder derivation]
        float bank = 0;          // rad, positive leaning into this point's arc [placeholder derivation]
        float grip = 1;          // surface grip multiplier at the point
        float heading = 0;       // radians, world frame, as stored (course::AiPoint::heading)
        // Span to the next point: straight or arc (centre, start angle, sweep).
        bool arc = false;
        float cx = 0, cz = 0, a0 = 0, da = 0, rr = 0;
    };
    struct Line {
        std::vector<Point> points;
        bool lap = false;        // covers the whole lap
        bool empty() const { return points.size() < 2; }
    };

    const track::Track& track() const { return track_; }
    float length() const { return length_; }
    // nullptr when the line is absent; game numbering 1..7.
    const Line* line(int game_line) const;
    // Applies the game's fallback table (spec §2.1); -1 when nothing exists.
    int resolve(int wanted) const;
    // Position on the line at track distance s (spec §2.3).
    sim::Vec3 evaluate(int game_line, float s) const;
    // Index of the first point whose progress is ahead of s (== count past the last point).
    int next_point(int game_line, float s) const;
    // Distance from a to b along the lap (wrapped into [0, length)).
    float ahead(float a, float b) const;
    // Side map (line 5): true when "left" means line B at progress s.
    bool side_swapped(float s) const;

private:
    const track::Track& track_;
    float length_ = 0;
    std::vector<Line> lines_; // index = game line (0 unused)
};

// What the AI knows about every car in the race this tick.
struct CarSnapshot {
    sim::Vec3 position;
    sim::Quat orientation;
    sim::Vec3 velocity;
    float half_length = 2.2f, half_width = 0.9f;
    // Body extents from the reference point (front and rear extents; spec §14) and the
    // car's simulation time scale (its velocity is integrated x scale).
    float front = 2.2f, rear = 2.2f;
    float time_scale = 1;
    bool active = true;
};

struct AiContext {
    double race_time = 0;                // seconds since the start signal (< 0 = countdown)
    std::span<const CarSnapshot> cars;   // all cars including this one
    int self = -1;                       // index of this car in `cars`
    int rank = 0;                        // race rank (0 = first; the grid order at the start)
    bool body_contact = false;           // touched another car in the last collision step (spec §14:
                                         // car-car hits set the same recovery flags as walls)
};

// Live values for debug screens.
struct AiDebug {
    int line = 0, prev_line = 0;
    float blend = 0;
    int next_point = 0;
    int mode = 0;            // 0 straight, 1 arc, 3/4 braking
    int state = 0;           // recovery state machine (spec §7)
    int drive = 1;           // 1 forward, 0 brake, -1 reverse
    float progress = 0, lookahead = 0;
    float target_speed = 0, next_speed = 0, brake_distance = 0, distance_to_point = 0;
    float heading_error = 0, desired_yaw = 0, slip_wanted = 0, saturation = 0;
    float steer_angle = 0, throttle = 0, brake = 0;
    float contact = 0;
    float spin = 0, rear_slip = 0; // unused (the placeholder traction lift was removed)
    float follow = 0;        // deceleration needed behind a slower car, in units of grip (placeholder)
    int gear_want = 0;       // gear the driver is selecting
    float oversteer = 0;     // reserved (always 0; the stability-aid placeholder was removed)
    float grip = 0;          // assumed front grip (m/s^2) and peak slip angle (rad)
    float alpha_peak = 0;
    u8 zones = 0;            // bit i = zone byte i (front, FL, FR, L, R, RL, RR, rear) occupied
    sim::Vec3 target;
};

class AiDriver {
public:
    AiDriver(std::shared_ptr<const AiCourse> course, DriverParams params = {}, int start_offset = 0);
    ~AiDriver();

    // Call once the car is placed (grid). Picks the starting line (spec §2.1).
    void reset(const sim::VehicleState& state);
    // One tick (1/60 s). `car` is this AI's vehicle (state, spec, telemetry).
    sim::ControlInputs update(const sim::Vehicle& car, const AiContext& ctx, double dt = 1.0 / sim::kDefaultTickHz);

    const AiDebug& debug() const { return dbg_; }
    const DriverParams& params() const { return p_; }
    float progress() const { return progress_; }
    // The AI's reverse-gear request (spec §7) after the last update(); pass it
    // to sim::Vehicle::set_reverse_request() before stepping the car.
    bool reverse_request() const;
    // Forces a line (game numbering), e.g. for tests. Blends like a lane change.
    void set_line(int game_line);

private:
    struct Impl;
    std::shared_ptr<const AiCourse> course_;
    DriverParams p_;
    int start_offset_ = 0;
    float progress_ = 0;
    AiDebug dbg_;
    std::unique_ptr<Impl> m_;
};

} // namespace gt3r::ai
