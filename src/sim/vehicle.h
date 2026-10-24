#pragma once

// Vehicle simulation (milestone M2). One Vehicle per car; the race scene
// steps it at a fixed tick with ControlInputs and a TrackQuery and reads
// VehicleState back (see src/sim/sim_types.h for frames and units).
//
//   auto car = sim::Vehicle::create(db, "r34_gtr_v_99");
//   car->reset(grid.position, grid.orientation);       // origin on the road
//   every tick: car->step(inputs, *track, 1.0 / sim::kDefaultTickHz, &world);   // sim::TrackWorld world(*track)
//   render:     car->state(), car->setup().wheels[i]
//
// Timing: one simulation tick per 1/60 s (GT3
// steps its cars once per frame at a fixed dt = 1/60 with explicit Euler, no
// sub-steps). step(dt) runs ceil(dt * 60) equal ticks, so pass 1/60.
//
// Determinism: step() uses only its arguments and internal state (no clocks,
// no global RNG), so the same inputs and dt produce bit-identical states on
// the same build. Replays should record ControlInputs per tick.
//
// Conventions follow the spec: WheelState::slip_ratio > 0 = braking (wheel
// slower than the road), slip_angle = atan2(-vy, vx) of the contact patch.
//
// The model and its provenance (spec vs. placeholder) are described in
// docs/dev/PHYSICS.md.

#include "sim/car_spec.h"
#include "sim/models.h"
#include "sim/collision.h"
#include "sim/sim_types.h"

#include <memory>
#include <string>

namespace gt3r::data {
class CarDatabase;
}

namespace gt3r::sim {

// Optional extra world information beyond sim::TrackQuery: wall collision
// and surface friction. Implemented over track::Track by sim::TrackWorld
// (sim/track_walls.h); pass nullptr for "no walls, every surface is tarmac".
class WorldQuery {
public:
    virtual ~WorldQuery() = default;
    // Deepest contact of a sphere with the walls. Walls are vertical: the
    // normal is horizontal and points from the wall to the sphere centre.
    virtual bool wall_contact(const Vec3& centre, float radius, Vec3& normal, float& penetration) const = 0;
    // Grip multiplier and extra rolling drag coefficient for a raw
    // SurfaceHit::surface value (tarmac = {1, 0}).
    struct Grip {
        float mu = 1;
        float drag = 0;
    };
    // Default: the sim's own per-surface tables (physics §3.6 index; rolling
    // resistance from the spec, mu placeholder). Informative only: Vehicle
    // reads the surface tables directly from SurfaceHit::surface.
    virtual Grip surface_grip(u32 surface) const;
    // Wall segments (course x/z) within `radius` of `centre` (and in its
    // height band), for the spec's swept octagon collision (physics §5.3). Return
    // false when not supported: the vehicle then falls back to wall_contact().
    virtual bool wall_segments(const Vec3& centre, float radius, std::vector<collide::Segment>& out) const {
        (void)centre; (void)radius; (void)out;
        return false;
    }
};

// Static geometry for the renderer/camera, in the car frame (+X right, +Y up,
// nose -Z, origin at the wheelbase midpoint on the ground).
struct VehicleSetup {
    struct Wheel {
        Vec3 rest_center; // wheel centre at static ride height
        // WheelState::suspension at the static ride height. The wheel centre in the car frame is
        // rest_center.y + (WheelState::suspension - static_suspension) (+ = compressed, the wheel
        // rises into the arch; airborne = max droop). Spec suspension: -x_s [physics §4.2].
        float static_suspension = 0;
        float ride_height = 0; // [physics §4.2] q = the ride-height byte x 0.001 m of this axle (0: placeholder suspension)
        float radius = 0.3f;
        float width = 0.2f;
    };
    std::array<Wheel, 4> wheels; // FL, FR, RL, RR
    // The CoG in the car frame, and the body extents measured from it: what the
    // original game uses for the collision octagon, proximity and the grid placement (physics §5.3, physics §9.2).
    Vec3 cog_offset{};
    float cog_front_extent = 2.25f, cog_rear_extent = 2.25f;
    float wheelbase = 2.6f;
    float length = 4.5f, width = 1.75f, height = 1.35f;
    float steer_lock = 0.55f; // full lock (rad) [physics §6.4]: the formula cars' steering morph
    int forward_gears = 5;
    float idle_rpm = 900;
    float rev_limit_rpm = 7000;   // fuel cut (tach redline)
    float max_rpm = 8000;         // sensible tach scale end
    float upshift_rpm = 6800;     // first-gear full-throttle shift point (shift light)
    bool has_turbo = false;
    float max_boost = 0;          // cap - 1 (physics §2.3), same unit as VehicleTelemetry::boost
};

// Extra per-tick values for telemetry/debug screens (not part of the contract).
struct VehicleTelemetry {
    float engine_torque = 0;      // N*m at the crank (after throttle/limiter)
    float clutch = 1;             // 0 open .. 1 closed
    bool clutch_locked = true;
    float pitch = 0, roll = 0, pitch_rate = 0, roll_rate = 0; // [physics §4.3] body attitude (rad, nose up / right side down) and rates
    float ground_u = 0, ground_l = 0; // [physics §10.1] world velocity . ground-frame forward / lateral tangent at the end of the tick
    int clutch_state = 1;         // [physics §2.4] link state at the end of the tick (after a gear change): 0 open, 1 locked, 2 slipping, 3 shift
    bool shifting = false;
    float steer_angle = 0;        // rad, average front road-wheel angle
    float g_over_mu = 0, yaw_acc = 0;  // [physics §4.1] lateral g / mu, yaw acceleration (read by the next tick's aids)
    std::array<float, 4> beta{};       // [physics §3.2] tyre slip angle beta per wheel (spec planar path)
    float steer_rate = 0;         // rad/s, the steering follower's rate (our sign: + = right; 0 on the CPU path)
    std::array<float, 4> fx{}, fy{}; // tyre forces in the wheel frame, N
    // Per-wheel tyre words of the spec planar step (0 for a hand-built spec without the suspension block):
    std::array<float, 4> long_force{}; // drive_part + brake_part: the ungained longitudinal force (N) [physics §3.8]
    std::array<float, 4> slip_grip{}, slip_drag{}; // the lateral outputs: grip and drag along the velocity (N) [physics §3.8]
    std::array<float, 4> peak_force{}; // Fp (N) [physics §3.1]
    std::array<float, 4> slip_gain{};  // g [physics §3.4]
    std::array<float, 4> brake_pressure{}; // brake pressure after ASCC and ABS (0..1) [physics §4.5]
    std::array<float, 4> abs_target{};     // the ABS target slip (record +0x144 x D/E factor) [physics §4.5/4.6]
    std::array<float, 4> drive_torque{}, brake_torque{};
    Vec3 accel_local;             // body-frame acceleration, m/s^2 (x right, y up, z back)
    float drag_force = 0;         // N
    float downforce = 0;          // N
    bool abs_active = false, tcs_active = false;
    float understeer = 0, oversteer = 0; // ASCC indicators [physics §4.6]
    float tcs_factor = 1;             // TCS throttle factor this tick (1 = no cut) [physics §4.6]
    float throttle_processed = 0;     // throttle after the pedal shaping and TCS, before the turbo lag
    // Turbo boost of the lagged stages: Bcap(lag0, lag1) - 1 (physics §2.3), i.e. the
    // torque multiplier above 1. Read as gauge pressure in kgf/cm^2 for the HUD
    // [interpretation: the game's gauge value's relation to the boost model is not in the spec].
    float boost = 0;
    float slipstream = 0;              // filtered slipstream factor y, 0..1 [physics §5.2]
    float turbo_load = 0;              // turbo load after the step, x * Bcap(lag) [physics §2.3]
    std::array<int, 2> turbo_lag{};    // stage lag states, s16 rpm [physics §2.3]
    // Tyre wear [physics §8] (wear 0 / grip 1 when wear is off).
    std::array<float, 4> tire_wear{};      // wear units (fresh set -20000, worn out 210000)
    std::array<float, 4> wear_grip{1, 1, 1, 1}; // grip factor from wear (physics §8.4)
    std::array<int, 4> wear_level{};       // tyre gauge level -63..127 (physics §8.4)
    std::array<float, 4> smoke{};          // slip intensity (skid smoke / squeal, physics §8.3)
    std::array<float, 4> smoke_emit{};     // smoke-emission intensity 0..1 (physics §8.3b): tyre smoke / dust puffs, car dirt
    std::array<float, 4> skid_pitch{1, 1, 1, 1}; // second slip channel (physics §8.3): tyre loops' pitch (sound §7.6)
    std::array<float, 4> brake_heat{};     // brake-disc heat 0..1, visual only (physics §8.5)
    bool tires_worn = false;               // some wheel wear >= 0 and grip < threshold (physics §8.7)
    bool on_pit_surface = false;           // a wheel on a pit-lane triangle (flag bit 0, physics §8.6)
    std::array<bool, 2> diff_locked{}; // clutch-pack lock flag per axle (spec driveline)
    std::array<float, 2> diff_half{};  // half wheel-speed difference per axle, m/s (peripheral, as the original game keeps it)
    float wall_impulse = 0;       // sum of wall impulses this tick, N*s (spec: |j| / 1e5 = strength)
    // [ai §7] collide::HitSide bits of this tick's collision step: a wall hit at an octagon point,
    // and (added by sim::World) car-car responses at a point or edge. The AI's recovery flags.
    unsigned hit_sides = 0;
    int substeps = 0;             // ticks run by the last step() call's tick (always 1 per tick)
};

class Vehicle {
public:
    // Car label (table 30) or variant label (tables 31/35). nullptr on failure
    // with *error set.
    static std::unique_ptr<Vehicle> create(const data::CarDatabase& db, const std::string& label,
                                           std::string* error = nullptr);
    static std::unique_ptr<Vehicle> create(const CarSpec& spec);
    ~Vehicle();

    // Places the car at rest (physics §9.3 standing start): `position` is the
    // car-frame origin (wheelbase midpoint on the ground), `orientation`
    // car->world. Suspension at its static compression, wheels at rest, engine
    // idle, gear 1, clutch open, shift lockout 12. Standing grid starts use this
    // (every car of race::Field, RaceVehicle::place), held by the race hold during
    // the countdown rather than the spawn hold.
    void reset(const Vec3& position, const Quat& orientation);
    // reset() plus the game's spawn hold [physics §4.3]: for 120 ticks the car is held
    // at ground + 0.3 m (dropping onto the ground for the last 30), flat, in
    // gear 1 with the clutch open, its heading easing to `orientation`'s at <= 30
    // deg/s. The original uses it for the free-practice pit start (physics §9.4); this
    // engine uses it for its own respawns (RaceVehicle::reset, race::Field's CPU and
    // second-player respawns, ai::AiRace's; ai::SafetyRespawn), never for grid starts.
    void spawn(const Vec3& position, const Quat& orientation);
    bool spawn_hold_active() const;
    // [physics §8] Tyre wear of this race (model::wear_params / shipped_wear_params;
    // a default WearParams = off). Fits a fresh set. reset()/spawn() keep the wear.
    void set_tire_wear(const model::WearParams& params);
    const model::WearParams& tire_wear_params() const;
    // Fresh set [physics §8.4]: wear = fresh, grip = 1 - loss_fresh, brake heat 0.
    void fit_fresh_tires();
    // [physics §8.6] The stop at the pit box: all velocities and dynamic states
    // zeroed, then the 300-step hold (lift to +0.3 m, heading easing to the
    // box orientation's heading at <= 30 deg/s, down over the last 30 steps);
    // fresh tyres at counter 60 when wear is on. spawn_hold_active() is true
    // during the stop. The pit-lane state machine (lines 3/4, auto drive,
    // the 25 m decision) is the race's.
    void begin_pit_stop(const Quat& box_orientation);
    // [physics §10.3] Engine condition: odometer (2 m units), E running-in/wear and O oil
    // (metres). Every tick the full-throttle torque is scaled by the current factor
    // f(E, O), E and O advance by |forward speed| x dt and the odometer by the forward
    // distance (not while reversing). All zero (the default) is a fresh engine with f = 1.
    // A run is seeded with the garage car's values and hands the advanced ones back.
    // Setting it also clears the odometer's sub-2 m carry, as a newly created car has none.
    void set_engine_condition(const EngineCondition& c);
    EngineCondition engine_condition() const;
    // [physics §10.4] Dirt course (course category 2): front brake cut with steering.
    void set_dirt_course(bool dirt);
    // [physics §9.3] reset() with the car already moving straight ahead at
    // `speed_mps` (rolling and flying starts): world velocity along the heading,
    // every wheel and axle at road speed, the gear-for-speed gear (physics §7 #24, at least
    // 1st), engine = clamp(v ratio, idle, rev limit) with the rpm truncated, clutch
    // locked, turbo lag states at that rpm, no hold. speed <= 0 is reset().
    // Placement (grid slot moved back by the front extent, or AI line 6) and the
    // AI hand-over (300 steps for a rolling start) are the race's (physics §9.2/9.4).
    void spawn_moving(const Vec3& position, const Quat& orientation, float speed_mps);
    void step(const ControlInputs& in, const TrackQuery& track, double dt, const WorldQuery* world = nullptr);
    // Exactly one tick of length dt_car (normally 1/60; sim::World passes
    // time_scale / 60 for the spec's per-car step length dt_car, physics §1.1).
    void step_tick(const ControlInputs& in, const TrackQuery& track, double dt_car, const WorldQuery* world = nullptr) {
        tick(in, track, dt_car, world);
    }
    // Slipstream target for the next ticks: sum over leaders of
    // model::slipstream_factor() (physics §5.2), clamped to 1. 0 = clean air.
    void set_slipstream(float factor);

    const VehicleState& state() const { return state_; }
    const VehicleTelemetry& telemetry() const { return telemetry_; }
    const VehicleSetup& setup() const { return setup_; }
    const CarSpec& spec() const { return spec_; }

    static constexpr double kTickHz = 60.0; // [physics §1.1]

    // CPU-driven car [physics §4.4 AI note]: the original game's binary clutch request
    // replaces the player's, and the AI aid block (ScriptedAids) the car's own.
    void set_cpu_driver(bool on) { cpu_driver_ = on; }
    // [physics §4.6; physics §8.6] driver kind (0 = normal, 1 the AI drives (run-up, after the
    // finish), 2/3/4 pit auto-drive states, 5/6/7 the stop driver, 8 parked; ai §1). A non-zero
    // kind outside the spawn hold takes the scripted aids path like a CPU car: the AI aid block
    // (ScriptedAids) and the alternate oversteer indicator. Kinds 1-7 use the AI clutch rule;
    // kind 8 holds the clutch request at 0 instead (for CPU cars too), so the clutch opens and
    // stays open (physics §2.4), and holds the car in place. The race scene sets it while the
    // computer drives the player's car.
    void set_driver_kind(int kind) { driver_kind_ = kind; }
    // [ai §9.1] The aid block CPU cars and scripted drivers read (sim::ScriptedAids), filled
    // from the AI driver parameter set at car init (ai::DriverParams::aids): its TCS, and its
    // ASCC channels where the set enables them. ABS is not part of the block.
    using ScriptedAids = sim::ScriptedAids;
    void set_scripted_aids(const ScriptedAids& a) { scripted_aids_ = a; }
    // [ai §10] The AI holds its clutch request at 0 during the countdown (it blips the
    // throttle with the clutch open); outside the hold a CPU / scripted car uses the AI request rule
    // (physics §4.4: open -> rpm > 3/4 upshift rpm, engaged -> rpm >= idle). Set by the AI each tick.
    void set_clutch_hold(bool on) { clutch_hold_ = on; }
    // [physics §9.3, physics §4.1] The race hold flag, set for every car while the start sequence counts
    // down: the world x/y velocity, yaw rate and longitudinal force sum are zeroed each tick (the car settles
    // vertically, on slopes too) and the player's clutch request is 0; the brakes are not applied, so the
    // wheels stay free. reset() clears it.
    void set_race_hold(bool on) { race_hold_ = on; }
    bool race_hold() const { return race_hold_; }
    const ScriptedAids& scripted_aids() const { return scripted_aids_; }
    int driver_kind() const { return driver_kind_; }
    bool cpu_driver() const { return cpu_driver_; }
    // Reverse-gear request (the AI's backing-out flag): with the automatic
    // gearbox, reverse holds while set, throttle open or not.
    void set_reverse_request(bool on) { reverse_request_ = on; }

    struct Impl;

    // Internal (sim::World): the car as a 2-D collision body (physics §5.3) and back.
    collide::Body collision_body() const;
    void apply_collision_body(const collide::Body& b); // also refreshes state()
    // Internal (sim::World): car-car responses of this tick's collision step (ai §7).
    void add_hit_sides(unsigned sides) { telemetry_.hit_sides |= sides; }
    // Online play: a remote player's car is never stepped here (sim::World kinematic car);
    // its published state and collision pose (pos, heading, vel, vy, yaw_rate of the
    // remote's collision_body()) come from the network. Mass, inertia and outline stay
    // this car's own.
    void set_remote_pose(const VehicleState& st, const collide::Body& body);
    // Online play, prediction shadows (race::RemoteCar): puts the car into a state its owner
    // published - pose, velocities (yaw rate from its collision body), each wheel's spin,
    // steer angle and suspension, engine speed, gear - so it can be stepped on with the owner's
    // inputs. Internal states the snapshot does not carry (clutch, differentials, turbo, tyre
    // history) are kept, so a shadow synced every few ticks stays smooth. Never used on a car
    // the local player drives. Clears the telemetry's collision sides (hit_sides).
    void sync_to(const VehicleState& st, float yaw_rate);
    // A horizontal impulse J (N*s, world x/z) at a world point, as the car-car contact applies
    // it [physics §5.3]: v += J / m, yaw rate += (r x J) / (10 I_z). Online: a hit another player's
    // machine saw that ours missed.
    void apply_impulse(const collide::V2& point, const collide::V2& J);

private:
    explicit Vehicle(const CarSpec& spec);
    void tick(const ControlInputs& in, const TrackQuery& track, double h, const WorldQuery* world);
    void set_collision_body(const collide::Body& b);
    void publish();

    CarSpec spec_;
    bool cpu_driver_ = false;
    int driver_kind_ = 0;
    ScriptedAids scripted_aids_;
    bool clutch_hold_ = false;
    bool race_hold_ = false;
    int input_kind_ = 0; // last nonzero ControlInputs::controller_kind
    bool reverse_request_ = false;
    VehicleSetup setup_;
    VehicleState state_;
    VehicleTelemetry telemetry_;
    std::unique_ptr<Impl> impl_;
};

} // namespace gt3r::sim
