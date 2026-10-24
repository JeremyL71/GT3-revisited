# Vehicle simulation

This document describes what the driving simulation in `src/sim/` does: the car's
state, the fixed tick and the order of its steps, the engine and driveline, the
tyres, the suspension and body motion, the driver aids, and the interactions
between cars and with walls (§0–§5), then the car's parameters from the database
and the player's settings, tyre wear, pit stops, the visual flap pendulums, race
starts, ground-frame details and leaving the road (§6–§11). Code comments cite its sections as `physics §n`.

Where the code marks a rule **[placeholder]** or as an **interpretation**, this
document says so: that part is a conventional model or a reading of the observed
behaviour, not a statement about the original game.

Code is referenced by file and function, for example `src/sim/models.cpp`,
`model::tire_force`. Most rules live in two places: the pure sub-models in
`src/sim/models.h` / `models.cpp` (each a free function of plain inputs), and the
tick in `src/sim/vehicle.cpp`. `Vehicle::tick` is a short sequence of phase
functions, members of `Vehicle::Impl` defined below it in tick order (§1.2), and
these call the sub-models.

## Overview

**Frames.** The race, renderer and track use the world frame of the decoded
course data: right-handed, +Y up, metres (`src/sim/sim_types.h`). A car's own
frame has +X to the right, +Y up and the nose towards −Z, with its origin at the
wheelbase midpoint on the ground. Internally the simulation tracks the centre of
gravity (CoG) and three Euler angles in a Z-up *physics frame*; see §1.3.

**Units.** SI throughout (m, s, kg, N, N·m, rad, rad/s), except engine speeds
given in rpm where named. Gravity is 9.8 m/s² (§0).

**Time.** One tick of 1/60 s per frame, explicit (semi-implicit) Euler, no
sub-steps (§1.1). The steps of a tick run in a fixed order (§1.2); with several
cars, each phase runs for every car before the next (§1.2, §5).

**State.** A car is a planar rigid body (horizontal velocity, heading, yaw rate)
with a separate heave/pitch/roll attitude driven by four independent suspension
corners, plus the engine, clutch, gearbox, driveline and per-wheel spin and
suspension states (§1.3).

**Two body models.** Every car built from the disc carries a *suspension block*
for both axles (§4.2). Those cars use the model described here ("the spec body").
Hand-built specs without the block (the test car `CarSpec::test_car`, the no-disc
fallback) use an older **[placeholder]** rigid body: start-of-tick suspension,
forces applied at the contact points, a velocity-limited tyre force and a hull
penalty contact. Both models run the same phases of the tick; where the
placeholder body differs, its steps are the `placeholder_*` functions of
`Vehicle::Impl`. This document describes the spec body unless it says otherwise.

| File | Role |
|---|---|
| `src/sim/sim_types.h` | shared types: inputs, wheel and vehicle state, track query |
| `src/sim/vehicle.h`, `vehicle.cpp` | `Vehicle`: state, the tick and its phase functions (`Vehicle::Impl`), integration |
| `src/sim/models.h`, `models.cpp` | pure sub-models (engine, tyres, suspension, aids, driveline) |
| `src/sim/car_spec.h`, `car_spec.cpp` | `CarSpec` (SI parameters) and `build_car_spec` |
| `src/sim/collision.h`, `collision.cpp` | 2-D octagon collision against walls and cars |
| `src/sim/world.h`, `world.cpp` | `World`: several cars stepped together |
| `src/sim/track_walls.h` | `TrackWorld`: the course walls as a `WorldQuery` |

---

## §0 Constants and conventions

The engine uses these constants (`src/sim/models.h`):

| Name | Value | Use |
|---|---|---|
| g | 9.8 m/s² (`model::kGravity`) | weight, preloads, static suspension, lateral g |
| kgf·m → N·m | × 9.8 | torque and force bytes in the database |
| rpm → rad/s | × 2π/60 = 0.10471976 | engine and wheel speeds |
| km/h → m/s | ÷ 3.6 | every speed threshold given in km/h |
| tick | 1/60 s (`model::kTickDt`) | §1.1 |

Sign conventions used throughout:

- **Yaw rate** and **heading**: positive = turning left (counter-clockwise seen from above).
- **Pitch**: positive = nose up. **Roll**: positive = right side down.
- **Steering**: the public inputs and `WheelState::steer_angle` use + = right. Several
  rules use + = left (the yaw-rate sense); the code negates the angle where it
  calls them, and this document says which convention a formula uses.
- **Slip ratio**: > 0 braking (wheel slower than the road), < 0 wheelspin (§3.4).
- **Slip angle**: `WheelState::slip_angle` = atan2(−v_y, v_x) of the contact-patch
  velocity in the wheel frame (x along the wheel heading, y to the right).
- **Suspension coordinate x** (spec body): 0 where the spring force is zero,
  negative = compressed (§4.2). `WheelState::suspension` publishes −x′, the clamped value (§4.2).
- Wheel order: FL, FR, RL, RR (index 0–3); axle 0 = front, 1 = rear.

## §1 State and time step

### §1.1 Time step

The engine steps each car once per 1/60 s (`Vehicle::kTickHz`, `model::kTickDt`)
with explicit Euler integration, "rate then value": each velocity is updated from
its acceleration first, and positions and angles then move with the new velocity.
There are no sub-steps.

`Vehicle::step(inputs, track, dt)` runs ⌈dt·60⌉ equal ticks of dt / ticks each
(a small tolerance stops 1/60 from rounding up to two). Pass dt = 1/60. Edge-triggered
shift requests are applied only on the first of those ticks.

`World` (§5) gives each car its own step length **dt_car = time_scale / 60**
(`World::set_time_scale`). The race's rubber band writes the time scale; 1 is
normal speed. The car then runs one tick of length dt_car (`Vehicle::step_tick`).

The tick is deterministic: it reads only its arguments and the car's state (no
clocks), so the same inputs give bit-identical states on the same build. The one
global random sequence it draws from feeds only the visual flap pendulums (§8.8),
never the driving state. If a tick produces a non-finite value, the car's previous
state is restored with all velocities and wheel speeds zeroed.

### §1.2 Step order

One tick of a spec-body car runs these steps in this order (`Vehicle::tick`).
The code refers to some of them by phase number; those numbers are given in the
left column. The third column names the phase function (a member of
`Vehicle::Impl` unless qualified); what one phase hands to the next is a small
per-tick struct (`TickContext`, `Frame`, `Ground`, `WheelMotion`, `Powertrain`,
`Brakes`, `TyreForces`, ...).

| Phase | Step | Function | Section |
|---|---|---|---|
| | Input shaping: CPU cars and scripted drivers use the automatic; pad pedals shaped; automatic reverse swaps the pedals; electric one-pedal hold | `condition_inputs` | §4.4, §2.6, §2.2 |
| | First tick after a reset: fit pitch, roll and height to the ground under the hubs | `settle_on_ground` | §4.3 |
| | Body axes and forward speed u on the ground frame the last phase 12 left; an automatic left in neutral selects first | `frame`, `set_gear` | §1.3, §2.6 |
| | Steering: target, follower, Ackermann and toe | `steering`, `wheel_steer_angles` | §4.4 |
| | Ground state: reuse what the last phase 12 left (first tick: query now) | `start_ground` | §4.2 |
| | Aero forces and slipstream filter | `aero` | §5.2 |
| 2 | Tyre preparation: pit-surface flag, wear grip, wheel slips, slip gains, peak forces from the loads the previous tick left | `wheel_motion` | §3.1–§3.4, §8.4 |
| | Traction control, clutch request and engagement, turbo lag, engine torque, engine speed | `engine_and_clutch` (`traction_control`, `clutch_link`) | §4.6, §2.2–§2.4 |
| | Under/oversteer indicators, ASCC, ABS, brake torques | `brakes` (`steer_indicators`) | §4.5, §4.6 |
| | Tyre force outputs | `tyre_forces` | §3.8 |
| | Wheel and axle spin step, differentials | `wheel_spin` | §2.7 |
| | Free-rolling wheel flags, odometer and engine condition counters, slip intensity, skid pitch, wear, brake heat | `flag_free_wheels`, `model::odometer_step`, `model::engine_condition_step`, `tyre_wear_and_heat` | §8, §10 |
| | Clutch after the step: crossing → locked, engine speed from the wheels | `clutch_after_driveline` | §2.4 |
| 7 | Planar step: forward step, lateral force and yaw, stop and race holds | `planar_step` | §4.1 |
| 10 | Position and heading advance by velocity × dt and yaw rate × dt | `advance_and_query_ground` | §4.1 |
| 12 | Ground query at the moved hubs and suspension; load transfer; attitude, heave, bump nudge (or the spawn hold) | `suspension_step`, `load_transfer`, `attitude_step` | §4.2, §4.3 |
| 13 | Flap pendulums (visual only) | `model::flap_step` | §8.8 |
| | Walls: swept octagon, impulses, push-out | `walls` | §5.3 |
| | Body acceleration recorded (the lateral g the next tick's aids read) | `record_acceleration` | §4.1, §4.6 |
| | Free wheels locked to the ground | `lock_free_wheels` | §10 |
| | Gear decision; a change takes effect now, after the driveline step | `select_gear` | §2.6 |
| | Non-finite check (restores the previous state at rest) | `state_finite` | §1.1 |

While the spawn/stop counter runs (§4.3), phase 10 is skipped: phase 12 queries
the ground at the pose the tick started with, and the hold's scripted pose (or,
above the target, the attitude step) moves the body.

The placeholder body (hand-built specs) runs the same sequence with its own steps
where it differs: `start_ground` queries the ground and steps the suspension at the
start of the tick (`placeholder_wheel_contact`, `placeholder_wheel_suspension`,
`placeholder_suspension_forces`), `placeholder_aero_forces`,
`placeholder_tyre_forces` (velocity-limited, with a per-wheel drive prediction,
`placeholder_drive_prediction`), `placeholder_hull_contact`, and
`placeholder_integrate` in place of phases 7–12. Inside `suspension_step` each axle
takes its own suspension, so a spec with the block on the front axle only still runs
(as a placeholder body, with `front_block_load_transfer`).

Two consequences of this order matter in practice:

- **The tyres lag the loads by one tick.** Phase 12 runs at the end of the tick, so
  the tyre forces of a tick use the wheel loads and ground state the previous tick
  left (`load_prev` in `Vehicle::tick`).
- **A shift takes effect at the end of the tick.** The tick that decides a shift
  still transmits through the old gear and clutch state.

With several cars (`World::tick`), each phase runs for every car before the next:
first the slipstream factors of every car (§5.2), then each car's tick in index
order (with its own walls), then car-to-car contact (§5.3).

### §1.3 State, frames and the ground query

**Physics frame.** The rules are written in a Z-up frame: x right, y forward
(at heading 0), z up. It maps to the world frame as world = (x, z, −y)
(`model::phys_to_world`). The body axes come from three Euler angles,
R = R_z(heading) · R_y(roll) · R_x(pitch) (`model::euler_axes`); the columns of R
are the body's lateral (right), forward and up axes. At zero angles forward is
(0, 1, 0) in the physics frame, which is world −Z, the car's nose. A heading c
puts the nose along (−sin c, −cos c) in world x/z (`collide::to_world_dir`), so a
positive heading turns the nose left.

**Car state** (`Vehicle::Impl`):

| Group | State |
|---|---|
| Body | CoG position (world), velocity (world; x/z planar, y vertical), heading, pitch, roll, yaw rate, pitch rate, roll rate |
| Per wheel | suspension coordinate x and its clamped previous value x′, spin ω (rad/s), spin angle, steer angle, slip ratio, slip angle, slip gain, load, grounded flag, surface, contact point, last ground height |
| Engine | speed ω_e (rad/s), fuel-cut flag, turbo lag states and load (§2.3), odometer and condition counters (§10.3) |
| Transmission | gear (−1 reverse, 0 neutral, 1…N), clutch state 0–3 (§2.4), shift lockout counter |
| Driveline | differential state per axle (lock flag, half speed difference), centre-clutch engaged flag (§2.7) |
| Steering | follower angle and rate, the pad's steering kind (§4.4) |
| Carried to the next tick | lateral acceleration, lateral g/μ, yaw acceleration, largest rear slip angle, longitudinal force sum, summed wheel torques per axle, free-roll flags, the ground frame, ground hits and normals |
| Other | slipstream factor y (§5.2), spawn/stop counter (§4.3), wall contacts (§5.3), wear, smoke, skid pitch, brake heat and flap pendulums (§8) |

The CoG sits `cg_height` above the ground and L·(w_f − 0.5) ahead of the wheelbase
midpoint, where L is the wheelbase and w_f the front weight fraction. Distances
from the CoG to the axles are **a = L(1 − w_f)** (front) and **b = L·w_f** (rear).

When a car is placed (`Vehicle::reset`), the pitch and roll taken from its
orientation are clamped to ±0.6 rad, a **[placeholder]** value for "small angles";
the placeholder body clamps its attitude there too. (The spec body clamps at ±π/4,
§4.3.)

`Vehicle::state()` publishes the car-frame origin (not the CoG), a quaternion,
world velocities, the per-wheel `WheelState`, engine rpm, gear and speed
(`Vehicle::publish`). An airborne wheel's published contact point is put back on
the last ground height, so its shadow stays on the road.

**Ground query.** Each wheel is probed straight down at its *query point*
(`TrackQuery::probe`, highest surface at or below the point + 1 m). For a
spec-body car the query point is the hub's offset in the CoG plane, (lateral,
longitudinal, 0) from the CoG, rotated by the attitude: pitch and roll move it
along the ground only by lever × cos(angle), not by the hub's height above the
CoG. The query returns the height, normal and raw surface value (§3.6).

A **miss** (off the road mesh) keeps the previous ground height and surface of
that wheel. The wheel stands on that height, so the car is not stopped and its
wheels stay grounded (test `sim_ground_miss_keeps_previous_height`: a car driven
off the mesh keeps four wheels grounded at the old height). For hand-built specs a
miss leaves the wheel unloaded.

The ground under the four hubs also defines the *ground frame* (§4.3), whose
forward and lateral tangents are what speeds are measured on (§10.1).
At the start of a tick the engine takes the forward speed u = velocity · (ground
forward tangent) of the frame the last phase 12 left; aero, steering, clutch,
aids and shifts read this u.

---

## §2 Engine, transmission and driveline

The engine produces a crank torque from a full-throttle curve, a throttle, engine
braking and a fuel cut (§2.1–§2.2); turbo engines lag their boost (§2.3). A
four-state clutch links the engine to the gearbox (§2.4). The gearbox ratio and
the drive layout (§2.5) feed an axle driveline with differentials (§2.7). Shift
decisions are §2.6.

### §2.1 Torque curve

Curves are piecewise-linear tables looked up with clamping: below the first
point the first value, above the last the last value (`model::table_lookup`,
`model::lookup`). Every curve in the simulation uses this lookup.

The full-throttle torque T_wot(rpm) is `model::curve_torque`, a lookup on the
car's torque curve. `model::build_torque_curve` builds the curve from the engine
record (format: `docs/formats/database.md`):

- up to 16 points; torque t_i in 0.01 kgf·m, rpm axis n_i in 100 rpm;
- T_i = t_i × 0.01 × 9.8 × F × s(n_i), with F = output factor / 1000 (engine
  +0x42 × 10, so 1000 = ×1) and a linear tilt s from the low-rpm factor (percent)
  at the first axis point to the high-rpm factor (per mille) at the last;
- with a turbo, each point is multiplied by min(B, cap) / B, where B is the
  uncapped boost at that rpm with both stages at the axis rpm (§2.3), so the curve
  holds the capped full-boost torque;
- the part multipliers are applied in integer percent steps before the build
  (§6).

Idle = engine +0x44 × 10 rpm (900 if 0). Rev limit = engine +0x45 × 100 rpm (plus
part adders; the last axis point if 0), but at least +0x46 × 100 + 200 rpm.

Test `sim_spec_helpers`: a factor of 1180 gives ×1.18; lookups clamp outside the axis.

### §2.2 Engine torque, engine braking and fuel cut

`model::engine_torque` gives the net crank torque at the stored engine speed:

  T = x′ · T_wot(rpm) − (1 − x′) · F(ω),  F(ω) = k_f · ω

- x′ is the effective throttle after traction control, the shift cut/blip (§2.4)
  and the turbo lag (§2.3), clamped to 0…1.
- **Engine braking** F (`model::friction_torque`) is linear in engine speed. k_f =
  drivetrain +0x16 (× part percentages) × 0.98 / ω_last, where ω_last is the last
  torque-curve rpm in rad/s, so engine braking at the top of the curve equals the
  byte × 0.98 N·m. (R34 GT-R: k_f = 0.302 N·m per rad/s.)
- **Fuel cut**: sets when rpm ≥ rev limit and clears when rpm falls below rev
  limit − 500 rpm (`limiter_hysteresis_rpm`). While cut, x′ = 0 and T = −F.
- The engine-condition factor (§10.3) scales T_wot, not F.

The rpm the curve, friction and fuel cut read is the **stored** engine speed:
there is no re-sync to the wheel side at the start of the tick. With a locked
clutch the stored value was set at the end of the previous tick from the wheels,
capped at rev limit + 10 rpm (§2.4).

Engine inertia J_e = drivetrain +0x20 × flywheel % × clutch % × 0.001 kg·m²
(R34: 0.201). The engine speed integrates as described for each clutch state in
§2.4.

Mod cars only (GT3 has no electric cars): an electric motor replaces engine braking
by lift-off regeneration min(regen torque, regen power / ω), faded out below
150 motor rpm, and uses a direct-drive clutch (locked whenever it is asked to drive,
else open with the motor stopped). For the player, below 5 km/h with the accelerator
under 0.02, the brake is raised to at least 0.05 + 0.35 (1 − v / 5 km/h), so the car
holds itself at a standstill.

### §2.3 Turbo

Turbo engines have one or two stages (`TurboSpec`; all 113 turbo cars on the
disc decode this way). From the turbo part and the user's turbo settings:

| Parameter | Value |
|---|---|
| cap | 1 + 0.1 × turbo +0x11 |
| stage peak A | setting byte × 0.1 (stock: turbo +0x12 / +0x15) |
| stage knee N | setting byte × 100 rpm (+0x13 / +0x16) |
| stage rate R | setting byte × 0.1 per second (+0x14 / +0x17) |

The turbo is on when the cap and stage 0's three bytes are all non-zero; stage 1
is active when its three bytes are non-zero.

**Boost shape** of a stage at lagged rpm n (`model::turbo_stage`):

- n < N: A (1 − ((N − n)/N)²)
- N ≤ n ≤ 3N: A − 0.25 A ((n − N)/N)²
- n > 3N: 0

Uncapped boost B(n₀, n₁) = 1 + shape₀(n₀) + shape₁(n₁); capped Bcap = min(B, cap)
(`model::turbo_boost_raw`, `model::turbo_boost`).

**Lag step** (`model::turbo_step`), once per tick with n = the integer engine rpm
(at least idle − 1):

1. target = load_prev · n / Bcap(n, n);
2. each active stage: lag += trunc((target − lag) · R · dt), floored at 0 (lag is
   an integer rpm);
3. load = x · Bcap(lag₀, lag₁), where x is the throttle after TCS and the shift
   cut/blip;
4. x′ = min(1, load / Bcap(n, n)).

So x′ is the throttle scaled by how much of the full boost the lagged stages give;
boost decays during a shift cut. The clutch's engage rule reads x, not x′.
Without a turbo x′ = x.

Telemetry reports the load (`turbo_load`) and the two lag states (`turbo_lag`).
The HUD boost value is Bcap(lag₀, lag₁) − 1 (`VehicleTelemetry::boost`, maximum
cap − 1, `VehicleSetup::max_boost`). Reading it as gauge pressure is an **interpretation**. R34 GT-R: maximum
0.79 against a cap of 0.80 (test `sim_database_r34_gtr`).

### §2.4 Clutch

The clutch link has four states (`ClutchState`; telemetry `clutch_state`):

| State | Meaning | Torque to the gearbox |
|---|---|---|
| 0 open | engine free | none |
| 1 locked | engine turns with the wheels | the engine torque T |
| 2 slipping | launch or reverse change | T_c from the slip rule below |
| 3 shift | forward-to-forward gear change | ± the shift-clutch torque |

The torque into the axle is T_g = T_clutch × R_g, with R_g the total ratio (§2.5).
There is no driveline efficiency.

**Clutch request** (player, `model::clutch_request`), set every tick:
- 0 during the spawn/stop counter (§4.3), during the race hold (the start
  countdown), and for driver kind 8 (parked, §8.6), for the player's car and CPU
  cars alike;
- 0 with the handbrake on a car that is not front-wheel drive;
- with the clutch engaged (state ≠ 0): 1 unless rpm < idle;
- with the clutch open: 1 while the brake pedal < 0.25.

CPU cars and scripted drivers of kinds 1–7 use a different request rule (§4.4,
CPU cars). Kind 8 is not one of them: the AI rule does not run for a parked car,
so its request stays 0 and a locked clutch opens on the first tick and stays
open while the engine revs freely (test `sim_spec_parked_clutch`, both a player
car and a CPU car). Stop-driver kinds 5–7 keep the AI rule, so a locked clutch
stays engaged while the engine is at or above idle. A direct-drive (electric)
car connects under kinds 1–7 whenever it is not held, and stays disconnected under
kind 8.

**Before the step** (`model::clutch_engage`):
- request 0 opens a *locked* clutch (1 → 0). States 2 and 3 are not opened by the
  request. This is the only way out of state 1.
- From open (0), with request 1: → 2 when the car moves faster than 10 km/h in
  the gear's direction and the processed throttle > 0.25; when not moving, → 2 when
  rpm > the launch rpm = max(engine +0x43 × 10, idle + 500) (R34: 2500 rpm).
- Neutral (gear 0) opens the clutch. GT3 has no neutral; this engine adds one for
  the manual gearbox at a standstill (§2.6).

**Gear change with the clutch engaged** (`model::clutch_gear_change`): a change
to or from reverse → 2; a forward → forward change → 3. A locked clutch never just
opens on a shift.

**Engine speed and transmitted torque** in the step:

- **Locked (1)**: T_g = T · R_g. The engine speed is set at the end of the tick to
  the wheel-side speed ω_in = (driven axle speed) × R_g, clamped to 0 … rev limit + 10 rpm.
- **Slipping (2)**: Δω = ω_e − ω_in. The clutch torque (`model::clutch_slip_torque`) is

    |T_c| = 0.05 · T_wot + (T > 0 ? s(|Δω|) · T : 0),  s(d) = min(1 + max(d − 52.36, 0) / 837.76, 1.125)

  signed by Δω (52.36 rad/s = 500 rpm, 837.76 rad/s = 8000 rpm). Then
  ω_e ← max(ω_e + dt (T − T_c) / J_e, 0): no idle floor while slipping. T_g = T_c · R_g.
- **Shift (3)**: T_c = sign(Δω) × the shift-clutch torque (the shift-clutch byte ×
  9.8 N·m; the engine has no source for it, so this is 0). The processed throttle is cut
  to 0 while the wheel side is slower than the engine and blipped to 1 while it is
  faster. The engine speed integrates as in state 2.
- **Open (0)**: ω_e ← max(ω_e + dt · T / J_e, idle). No rev cap.

**After the step** (`model::clutch_after`):
- 2 or 3 → 1 when the engine and wheel-side speeds crossed during the step
  (Δω before and after have opposite signs or the gap closed), in either direction
  and regardless of idle; this starts a 12-tick shift lockout;
- otherwise 2 → 0 when the brake pedal > 0.75 and either the anti-stall test
  holds (`model::clutch_anti_stall`: engine speed < (idle + 500) × 2π/60, both
  comparisons strict, the product rounded towards zero) or |speed| < 10 km/h;
- 3 → 0 when the wheel-side speed ≤ idle.

Measured (R34 GT-R from rest on flat ground, full throttle):
the clutch goes 0 → 2 on tick 11 (0.18 s, the free-revving engine at 2553 rpm), locks
at 0.65 s and 15 km/h; each up-shift spends 0.18–0.35 s in state 3 before re-locking.

### §2.5 Gearbox and drive layout

Gears are numbered −1 (reverse), 0 (neutral), 1…N. The total ratio
(`model::total_ratio`) is R_g = ratio_k × final drive for a forward gear,
−reverse × final drive in reverse and 0 in neutral. Gear words come from the user
settings (×0.001); computed gearboxes are generated from a slider (§6.7).
Shifts are instantaneous (`GearboxSpec::shift_time` = 0).

The **driven radius** used to convert road speed to engine speed is the mean tyre
radius of the driven wheels.

The **layout code L** (`model::awd_layout`) comes from the drive type, the AWD
system byte B3 (drivetrain +0x15), the AWD share byte B4 (drivetrain +0x13) and the
AWD strength byte B5; the table is §7 #59. B5 has no known database source, so it is
0 and the centre-clutch capacities use their defaults. The older driveline fields of `DrivetrainSpec` (centre lock,
LSD preload and lock, clutch maximum torque) are no longer read. The user's front-torque setting is stored but not read; the
share comes from drivetrain +0x13. What each layout does is §2.7.

### §2.6 Shift logic

The gear decision runs at the end of the tick (`Vehicle::Impl::select_gear`)
with that tick's final state; a change takes effect immediately and sets a
1-tick lockout (`shift_lockout`), so the next tick makes no decision. A clutch
re-lock (§2.4) and a standing start set a 12-tick lockout.

The automatic gearbox is used for CPU cars and any non-zero driver kind (run-up,
pit lane, stop driver), whatever the input says.

**Inputs** (`model::ShiftIn`): engine rpm = max(int(ω_e · 60/2π), idle − 1) of the
final engine speed; forward speed u on the ground frame; the accelerator and brake
pedals as pressed; whether the clutch is locked; the reverse request; and a
*traction veto*. The veto is set when a wheel of the driven axle (L1, L2: front;
L0, L4–L6: rear; L3, L7: all four) is airborne or its slip ratio is below the
tyre's optimum drive slip × that wheel's slip gain (spinning past the optimum).

**Rules** (`model::shift_logic`), in order:

1. Reverse request (CPU cars backing out) → reverse.
2. Automatic, accelerator pressed and speed < +1 km/h → first gear (also while
   reversing; this is how a player leaves reverse).
3. In reverse: → first when moving forward faster than 1 km/h, else stay.
4. Rolling backwards faster than 1 km/h in a forward gear → reverse, **only with
   the accelerator released** (< 0.05). This is a deliberate deviation: the
   unconditional rule reversed a car that bounced off a wall with the throttle held.
5. Over-rev: rpm the road speed implies in the current gear > rev limit + 500 →
   up one gear.
6. Manual: the request (+1/−1), clamped to 1…N.
7. Automatic, only while the clutch is locked:
   - **down** k+1 → k when rpm < max(UP · r_{k+1}/r_k − (rpm per 10 km/h in gear k+1),
     launch rpm + 500), lowered by 500 rpm when the throttle > 0.25;
   - **up** when rpm > UP and there is no traction veto.

UP is engine +0x46 × 100 rpm (rev limit − 500 if 0; R34: 8000 rpm). The −10 km/h
term gives the down-shift an anti-hunt band below the speed at which the lower gear
would reach UP.

**Manual buttons** (`select_gear`): shift-up from reverse selects first
(automatic) or neutral (manual); from neutral, first. Shift-down in first below
1.5 m/s selects reverse (automatic) or neutral (manual); from neutral below
1.5 m/s, reverse. In the manual mode the automatic's moves into and out of
reverse are ignored, so manual reverse stays usable. An automatic left in
neutral selects first.

**Automatic reverse** (**interpretation**): for the player in reverse with the
automatic, the brake pedal drives and the accelerator brakes, since pressing the
accelerator below 1 km/h selects first. CPU drivers request reverse explicitly
and drive it with the accelerator.

### §2.7 Axle driveline and differentials

The driveline moves the clutch output T_g to two axles and then to four wheels
(`model::axle_step`, `model::diff_step_prev`, called from `Vehicle::Impl::wheel_spin`).

**Inertias.** Per axle, J_axle = shaft × final drive + 2 · I_a, where shaft =
0.01 × the shaft inertia word and I_a = 0.01 × the spin inertia word of that axle
(drivetrain +0x1C/+0x1E and +0x18/+0x1A, scaled by flywheel, clutch and propeller
shaft parts; R34: spin 8.00/8.00, shaft 0.40/3.00). Without a spin word the wheel
inertia is a **[placeholder]** 12 r² + 0.3 kg·m² (clamped to 0.6…3). The engine's
reflected inertia J_ref = J_e · |R_g| is added to the driven side only while the
clutch is locked.

**Wheel net torque** of wheel i:
  τ_i = −r · F_i^g − sgn(ω_i) · T_brake,i
where F_i^g is the *gained* longitudinal tyre force (§3.8) and there is no brake
term at ω_i = 0 exactly. Torque vectoring (below) adds to τ. Each axle's sum is
F_f = τ₀ + τ₁ and F_r = τ₂ + τ₃.

**Drive torque per axle** (`model::axle_drive`):

| L | Front | Rear | Engine inertia on |
|---|---|---|---|
| 0 | 0 | T_g | rear |
| 1 | T_g | 0 | front |
| 2 | T_g − k(ω_f − ω_r) | k(ω_f − ω_r) | front |
| 3 | T_g / 2 | T_g / 2 | (both, in the mean) |
| 4–7 | Q_f | T_g − Q_f | rear |

For L4–L6 with the centre clutch engaged, both axles turn at one common speed and
each gets T_g / 2.

**Front torque Q of L4–L7** (`model::awd_front_torque`); "full" means the full-transfer
case that locks or saturates the centre clutch:

- **L4**: full when k = 1; 0 with the handbrake; full when T_g ≤ 0 or in reverse
  with the clutch locked; else Q = k · T_g.
- **L5**: from the lateral g/μ value (§4.1): f = 1 for |g/μ| ≤ 0.25, 0.25 for
  |g/μ| ≥ 1, linear between. Full when the clutch is open, or in reverse with it
  locked; when f = 1, also full with the clutch slipping, the brake > 0.75, or T_g
  and T_g + F_r of the same sign; otherwise u = |ω_r − ω_f| · k · f, full if u ≥ 1,
  else Q = ½ · u · T_g (`model::awd5_gain`).
- **L6**: 0 with the handbrake; full in reverse with the clutch locked; g =
  max(0, 1 − |steer| · k), full if g ≥ 0.99, else Q = ½ · g · T_g.
- **L7**: Q = k · T_g; there is no common-speed lock.

The handbrake test is "> 0.2" of a 0/1 input.

**Centre clutch** (`model::awd_centre_clutch`, L4–L7), with the engaged flag:
- engaged: stays locked (common speed) while Q is full or |(F_f − F_r) − T_g| ≤
  capacity; otherwise the flag clears;
- not engaged: a full Q transmits +capacity if ω_r ≥ ω_f, else −capacity; otherwise Q.

The flag starts engaged after a reset (**interpretation**: the centre clutch holds at rest).

**Axle speeds** (`model::axle_step`):
- common speed (L4–L6 engaged): ω += (T_g + F_f + F_r) / (J_f + J_r + J_ref) · dt;
- L3: the mean speed integrates with the total inertia; the half-difference d
  integrates as d + (F_f − F_r − k(ω_f − ω_r)) dt / (J_f + J_r), set to 0 if it changes sign;
- otherwise each axle: ω += (drive + F) / J · dt, except an axle whose
  differential is type `f` (not integrated at the axle).
- A zero-crossing clamp applies only to an axle with brake torque: ω := 0 if it
  changes sign or was 0.
- Coupled layouts (L2, and L4–L7 when not common and Q_f ≠ 0): when the front and
  rear speeds cross (or meet), the non-driven axle snaps to the driven one; for
  L4–L6 the centre clutch becomes engaged.

The engine-side axle speed is the driven axle's speed; for L3 the mean of the two
axles; for L7 ω_r + k(ω_f − ω_r).

**Differentials** (`model::diff_step_prev`), one per axle. The type comes from the
LSD part as an ASCII letter; its index in "frnmhvyt" selects the rule (no LSD
part: `n`, open). The settings are the user's LSD initial / acceleration /
deceleration values. The step works on **peripheral speeds** (m/s) with inertia
I_a / r; in particular Δv below is in m/s.

Starting from both wheels at the new axle speed (type `f`: each wheel integrated
alone, ω += τ / I_a · dt), with Δv = the previous peripheral speed difference
(left − right) and T = the axle's drive torque:

| Type | Rule |
|---|---|
| f | wheels alone; a braked wheel whose speed crosses zero (or was or becomes exactly 0) is set to 0 |
| r | locked: both wheels at the axle speed |
| n, y, t | open |
| m, h | clutch pack, capacity C = max(initial × 9.8, (T ≥ 0 ? accel : decel) × 0.01 × \|T\|): stays locked while flagged and \|τ_l − τ_r\| ≤ 2C; otherwise unlocks and moves C from the faster wheel's torque to the slower's |
| v | viscous: moves accel × 0.98 × min(Δv², initial²) from the faster to the slower |

Then the half-difference d = Δv/2 + (τ_l′ − τ_r′) · dt / (2I) (0 while a clutch
pack is locked); if d has the opposite sign to Δv it is set to 0 (and a clutch pack
locks). Left = axle + d, right = axle − d.

**Torque vectoring** (types `y`, `t`; `model::torque_vectoring`), between 10 and
180 km/h: T_v = min(|steer| / 15°, 1) × initial × (9.8 for `y`, 0.01 for `t`).
With m̄ the mean peripheral speed, θ = 1.125 m̄ when steering left, 0.875 m̄ when
steering right. If the right wheel is slower than θ it gains 0.4375 T_v and the
left loses 0.5625 T_v; if faster, the right loses 0.5625 T_v and the left gains
0.4375 T_v. The steer sign used here (positive = left) is an **interpretation**.

Test `sim_drive_layouts_distinct`: FR, FF, AWD and MR put their drive torque on
the expected axles; the test car's AWD (L3) splits it equally.

---

## §3 Tyres

Each axle has three tyre records (compound slots 0–2, chosen by the surface class,
§3.6), each with curves A, B, C, D and L, a per-surface friction table μ[s] and the
optimum slips (`TireSpec`; decoding: §6.5). Per tick and wheel the
engine computes the slips (§3.2, §3.4), the peak force (§3.1), and the force
outputs (§3.3, §3.8). Forces are computed only for grounded wheels.

| Curve | Input | Output |
|---|---|---|
| L | load per metre of section width, N/m | load factor |
| A | folded slip angle, rad (0…π/2) | lateral force fraction |
| B | folded slip angle, rad | force fraction along the patch velocity (drag) |
| C | signed slip ratio (−1…1) | longitudinal force fraction |
| D | signed slip ratio | friction-circle radius for the lateral pair |

A record whose B curve is empty gets a **[placeholder]** default B curve.

### §3.1 Peak force

The peak force of a wheel (`model::tire_peak_force`, applied in `Vehicle::Impl::wheel_motion`):

  F_p = F_z · L(F_z / width) · μ[s] · Cam(α) · axle factor · wear factor

- **F_z** is the wheel load the previous tick's phase 12 left (§1.2); 0 gives 0.
- **μ[s]** is the record's friction for surface index s (base μ × the force-vol
  percentage). Records without a table use base μ × a **[placeholder]** per-surface
  grip table (`model::surface_mu`).
- **Camber factor** Cam(α): a 4-point curve per axle from the suspension part
  (angle codes → (128 − code) × 0.125°, multiplier codes / 200; used only when every
  multiplier > 0.2, else Cam = 1). The camber angle is
  α = σ · (body roll relative to the ground) + camber offset, with σ = −1 on the
  left wheels and +1 on the right, the relative roll = −asin(body lateral axis ·
  ground-frame up axis, §4.3), and the camber offset = −(camber setting × 0.1°).
- **Axle factor** = (b + 900) × 0.001, with b = (chassis byte + 124) mod 256 (the
  chassis byte at +0x12 front, +0x1E rear).
- **Wear factor**: §3.7.

### §3.2 Slip angle and low-speed fade

The *velocity-direction angle* of a wheel is φ = atan2(−l_h, u_h), where (u_h, l_h)
is the hub velocity, forward and to the right, on the ground frame (positive φ =
moving to the left). The hub velocity carries only the yaw rate r:
u_h = u + X·r, l_h = l − Y·r, with lateral offsets X = (−t_f/2, +t_f/2, −t_r/2,
+t_r/2) and longitudinal offsets Y = (a, a, −b, −b) (§10.1).

The **fade weight** k (`model::patch_slip_angle`) follows the contact-patch speed
along and across the wheel: 0 at ≤ 1 km/h, 1 at ≥ 10 km/h, linear between. When it
is 0, φ is taken as 0 and the wheel makes no lateral force. The fade multiplies the
lateral force pair (§3.3), so lateral grip builds up between 1 and 10 km/h.

The **tyre slip angle** (`model::tyre_lateral`) is β = wrap(a) − wrap(φ),
re-wrapped into [−π, π), with a = the wheel's steer angle (+ left), plus π when the
hub moves backwards. It is folded to v ∈ [0, π/2] with a sign:

| β | v | sign |
|---|---|---|
| < −π/2 | β + π | + |
| −π/2 … 0 | −β | − |
| 0 … π/2 | β | + |
| > π/2 | π − β | − |

`WheelState::slip_angle` reports atan2(−v_y, v_x) of the patch velocity in the
wheel frame; telemetry `beta` reports β.

The mean φ of the two front wheels also bounds the steering (§4.4).

### §3.3 Tyre force

From the folded slip angle v, the fade k, the slip ratio s (§3.4) and F_p
(`model::tyre_lateral`, `model::tyre_curves`, `model::tyre_long_force`):

- **Lateral pair**: u₁ = sign · A(v) · k and u₂ = B(v) · k. If |(u₁, u₂)| exceeds the
  friction-circle radius D(s), both are scaled down to it.
- **Slip grip** F₁ = u₁ · F_p acts perpendicular to the patch velocity, against the
  slide; **slip drag** F₂ = u₂ · F_p ≥ 0 acts along the velocity, against the motion.
  On the car, in the body frame (forward, right):
  F = −F₂ (cos φ, −sin φ) − F₁ (sin φ, cos φ).
- **Longitudinal**: F = C(s) · F_p along the wheel heading, negated when the slip
  sign sg (§3.4) is negative. C and D are looked up at the *signed* slip ratio.

The D limit applies only to the lateral pair: it is the room the slip ratio
leaves for cornering force. The longitudinal force is not reduced by the slip angle.

Test `sim_tire_force_shape` checks the shape. The hand-built-spec path uses
`model::tire_force`, the same law with the patch velocity in the wheel frame and
the longitudinal force directed against the hub/wheel relative motion, plus a
**[placeholder]** cap at what stops that relative motion in one tick.

### §3.4 Slip ratio, slip state and slip gain

With x = the hub speed along the wheel heading, w = ω · r the wheel's peripheral
speed and u = the hub's forward speed, `model::slip_ratio` computes, with a dead
band ε = 1/57.6 m/s:

| Case | Raw slip | State st |
|---|---|---|
| \|w\| < ε | 1 if x ≠ w, else 0 | −1 |
| w > 0, x ≤ w (wheelspin) | (x − w) · g / w | +1 |
| w > 0, x > w (braking) | (x − w) · g / x (0 if \|x\| < ε) | −1 |
| w < 0, x ≥ w | (x − w) · g / w | +1 |
| w < 0, x < w | (x − w) · g / x | −1 |

The raw value is clamped to [−1, 1]. A low-speed fade then multiplies it by
|r_ref| / (20 km/h) when |r_ref| < 20 km/h, with r_ref = x for state −1 and w for
state +1.

The **slip gain** g = 0.5 + 0.5 · min(|u| · 3.6 / 100, 1) (`model::slip_gain`)
rises from 0.5 at standstill to 1 at 100 km/h. It enters the slip ratio, the
gained wheel reaction (§3.8), ABS (§4.5), TCS (§4.6) and the shift veto (§2.6).

The **slip state** st (`model::slip_state`) is +1 in the wheelspin branch and −1
when braking or near-stopped. The **slip sign** sg (`model::slip_sign`) is
sign(w − x); in the dead band an exact tie gives 0, outside it a tie gives sign(w).
Using sg as the direction the force opposes is an **interpretation**.

Free-rolling wheels are locked to the ground at the end of each tick, so their
slip ratio reads exactly 0 (§10.2).

Test `sim_spec_helpers`: a locked wheel at 30 m/s gives +1 (the dead-band case); equal tiny speeds give 0;
a wheel spinning at 5 m/s at standstill gives −0.5 · 5/5.56.

### §3.5 Rolling resistance

Rolling resistance acts on the forward speed u in the forward step (§4.1):

  F_rr = −sgn(u) · RR(|u|) · Σ_grounded F_z,i · rr[s_i]

with F_z the previous tick's loads, rr[s] the surface coefficient (§3.6) and the
speed curve RR (`model::rolling_curve`):

| km/h | 0 | 4 | 20 | 58 | 116 | 200 | 308 | 400 |
|---|---|---|---|---|---|---|---|---|
| RR | 0 | 0 | 0.010 | 0.015 | 0.032 | 0.072 | 0.144 | 0.255 |

On tarmac (rr = 0.1) a 16.5 kN car at 100 km/h feels about 45 N; on surface 3
(rr = 25.5) the same car feels about 11 kN, which stops it quickly.

### §3.6 Surfaces

The raw surface value of a ground hit is the triangle's material in the low byte
and its flags in the next (`docs/formats/course.md`). `model::surface_index` turns
it into an index 0–15: with flag 0x02 (wet) set, materials 0 and 7 become 12, 1
becomes 13 and 9 becomes 14; otherwise the index is the material (clamped to
0–15).

The engine uses this static per-surface table (`kSurface` in `src/sim/models.cpp`;
names from `docs/formats/course.md`):

| s | Surface | Bump amplitude (m) | Wavelength λ (m) | Speed-scaled | rr[s] | Tyre slot |
|---|---|---|---|---|---|---|
| 0 | tarmac | 0 | 0.5 | no | 0.10 | 0 |
| 1 | kerb | 0.010 | 0.5 | no | 0.10 | 0 |
| 2 | grass | 0.010 | 2.5 | yes | 0.20 | 1 |
| 3 | gravel | 0.024 | 5.0 | yes | 25.5 | 1 |
| 4 | sand | 0.020 | 5.0 | yes | 0.30 | 1 |
| 5 | dirt | 0.010 | 0.5 | yes | 0.20 | 1 |
| 6 | – | 0 | 1.0 | yes | 0.10 | 2 |
| 7–11 | paving, others | 0 | 1.0 | no | 0.10 | 0 |
| 12–15 | wet tarmac, wet kerb, wet road, – | 0 | 1.0 | no | 0.10 | 2 |

**Tyre slot**: the compound record a wheel uses on that surface
(`CarSpec::surface_slot`); slot 0 is also the fallback.

**Road bump** (`model::road_bump`), added to the ground height under the wheel
(not on surfaces with amplitude 0): with w = sin(2πx/λ) + cos(2πz/λ) at the hub's
world (x, z), the bump is amplitude × w·|w| × scale, where scale = min(1, 0.02 ·
|hub speed|) on speed-scaled surfaces and 1 otherwise. A kerb therefore moves the
ground by up to ±4 cm in a 0.5 m pattern. The same speed ramp min(1, 0.02 ·
|hub speed|) feeds the slip intensity (§8.3).

The pit-lane flag (0x01) is read by the wear and pit rules (§8.6).

The vehicle reads these tables straight from the raw surface value of each hit;
`WorldQuery::surface_grip` (and so `TrackWorld`) only reports the placeholder μ
and rr[s] of a surface for information.

### §3.7 Wear grip factor

The wear factor in the peak force (§3.1) is 1 when tyre wear is off. With wear on
it is the grip factor of the wheel's wear (`model::wear_grip`, §8.4), and
it is not applied while any wheel is on the pit-lane surface. The record's own
`TireSpec::wear_grip` stays 1.

### §3.8 Per-wheel force outputs

Each grounded wheel hands the planar step (§4.1) and the driveline (§2.7) these
values (`model::tyre_curves`, `model::tyre_long_force`, `model::tyre_lateral`):

| Output | Meaning |
|---|---|
| φ | velocity-direction angle (§3.2) |
| slip grip F₁, slip drag F₂ | the lateral pair (§3.3), in the velocity-direction frame |
| drive part | F = ±C(s) F_p when st = +1 (wheelspin branch), else 0; along the heading |
| brake part | the same F when st = −1, else 0 |
| G = drive part + brake part | the longitudinal force on the car (not gained) |
| gained copies | g · drive part, g · brake part: the reaction on the wheel's spin (§2.7) |
| low-speed hold | §4.1 |

So the car is pushed by the ungained force C · F_p, while the wheel is slowed or
sped up by g · C · F_p, with g the slip gain of §3.4 (from the hub's forward speed).
Telemetry reports G (`long_force`), F₁, F₂, F_p and g per wheel.

Each wheel also has a **flag**: airborne (zero load), braked, or driven/coupled
(§10.2). The forward step (§4.1) reads only the airborne and braked parts.

---

## §4 Body motion, suspension, steering, brakes and aids

The hand-built-spec path of this section is the `placeholder_*` phases of
`Vehicle::Impl` (§1.2): start-of-tick suspension with a linear damper
(`placeholder_wheel_suspension`, `model::suspension_force`) and the rigid-body
integration (`placeholder_integrate`).

### §4.1 Planar motion

The body's horizontal motion is integrated from the tyre outputs (§3.8), not from
forces applied at the contact points (`model::forward_step`, `model::lateral_yaw`,
`model::stop_hold`). It works in the body frame (forward, right), with each wheel's
steer angle d in the + left convention, k = cos d and s = sin d.

**Forward step.** With u the ground-frame forward speed:
- F_x = Σ k · drive part + F_rr (§3.5);
- effective mass m_eff = M + Σ I_a / r² over the wheels that are *not* flagged
  airborne or braked (a free wheel's spin inertia adds to the car's);
- a₀ = (F_x + drag) / m_eff (drag from §5.2), u₁ = u + a₀ dt;
- F_t = Σ (k · brake part − sin φ · F₁ − cos φ · F₂), a_t = F_t / m_eff, u₂ = u₁ + a_t dt;
- if u₁ and u₂ have the same sign, u ← u₂ and a_x = a₀ + a_t; otherwise the tyre
  reactions stop the car: u ← 0 and a_x = a₀ − u₁/dt.

**Lateral force and yaw.** For each wheel:
- F_a (right) = −cos φ · F₁ + sin φ · F₂ − k · hold − s · G
- F_b (forward) = −sin φ · F₁ − cos φ · F₂ − s · hold + k · G

Yaw torque N = Σ (X · F_b − Y · F_a) with X = (−t_f/2, +t_f/2, −t_r/2, +t_r/2) and
Y = (a, a, −b, −b). Yaw acceleration = N / I_z; a_y = Σ F_a / M; the lateral **g/μ**
value = 4 a_y / Σ grip_i with grip_i = 9.8 × μ_i[s_i] (the records' base grip is
9.8 for every car, so this is lateral g over the mean surface μ). The yaw rate is
clamped to ±2π rad/s. These values are kept for the next tick's aids (§4.6) and
the load transfer (§4.2).

**Yaw inertia** I_z = the chassis inertia byte (+0x1C) × lightweight % × body % × 100
(R34 GT-R: 9500; 6000 if 0).

**Velocity update.** The world velocity gains a_x · dt along the horizontal
projection of the body's forward axis and a_y · dt along its lateral axis; the
vertical velocity comes from the attitude step (§4.3).

**Low-speed hold** (`model::low_speed_hold`), a sideways damper for a car almost at
rest: with w the wheel's peripheral speed and (x, y) the hub speed along and across
the wheel, the hold is F_p · sin(0.9π · y) · cos(0.36π · w), and 0 when F_p = 0,
|w| > 5 km/h, |x − w| > 2 km/h or |y| > 4 km/h (strict). It enters F_a and F_b as
above, and so acts on the lateral force and yaw only.

**Holds.** The planar velocity (world x and z) and the yaw rate are zeroed (the
first two also zero the longitudinal force sum):
- while the race hold is on (the start countdown, `Vehicle::set_race_hold`) or for
  driver kind 8, whatever the speed; the vertical velocity is kept, so a car settles
  on slopes; brakes are not applied, so the wheels stay free;
- by the **stop clamp**: |u| < 1 km/h (strict), every wheel's fade weight 0 and
  some wheel with brake torque. That the clamp needs brake torque is an
  **interpretation** of the condition;
- while the spawn/stop counter runs (§4.3).

With all four wheels airborne the planar step is skipped: no drag, no engine or
brake force, and the yaw rate is frozen (§10.6).

**Position and heading (phase 10).** After the planar step the CoG moves by
velocity × dt and the heading by yaw rate × dt; phase 12 then queries the ground at
the moved hubs.

**Longitudinal force sum** (for the load transfer): F_rr + Σ (k · G − sin φ · F₁ −
cos φ · F₂).

### §4.2 Suspension

Each corner is an independent spring, damper, bump stop and droop limit in the
*suspension coordinate* x (0 = spring force zero, negative = compressed), from
the axle's suspension block (`SuspensionSpec`, `model::suspension_wheel`). The
ground query and suspension run in phase 12, at the end of the tick, after the body
moved; the next tick's tyres read the loads they leave.

**Parameters** (from the user settings and the suspension part; §6.6):

| Parameter | Value |
|---|---|
| spring k | spring setting (0.1 kgf/mm) × 980 × c / 100 N/m, c = motion ratio % |
| progressive k₂ | byte × 98000 N/m², below the knee |
| anti-roll gain | ARB setting value × 98 N/m² |
| unsprung mass | byte × 0.25 kg per wheel; its weight is the tyre preload |
| bump damping below the knee | byte × 98 N per m/s |
| dampers | square-root law through two points per direction (below) |
| ride height q | the **front** ride-height setting × 0.001 m, for both axles |
| bump room, droop, knee gap g₁ | bytes × 0.001 m |

Both blocks read the front ride height; the rear setting is not read (test
`sim_car_settings`). R34 GT-R: k = 37240 / 46550 N/m, anti-roll 4900 / 3430,
k₂ = 1.57 × 10⁶, unsprung 10 kg, q = 0.130 m.

**Static position** (`model::static_position`):
  x_s = −½ · share · (M − 2(m_u,f + m_u,r)) · g / k
with share = w_f for the front and 1 − w_f for the rear: the weight split applies
to the sprung mass, and each wheel adds its own unsprung weight (the preload) on top.

**Limits** (`model::suspension_limits`, `model::bump_limit`):
- bump-stop limit = x_s − q + min(bump room, q);
- knee = max(x_s − g₁, bump-stop limit);
- droop = min(x_s + droop, preload / k).

**Suspension coordinate from the ground.** With the query point in the CoG plane
(§1.3), x = x_s − q − gap · s_t, where gap = (ground height + bump + CoG height − q)
− (query point height), and s_t is the terrain factor (§4.3). At the static pose
x = x_s. The triangle's normal does not enter.

**One wheel** (`model::suspension_wheel`), with v = (x − x′) / dt from the
previous clamped value x′:
- **airborne** when x > droop: x′ := droop, spring force = −preload (the body feels
  the hanging wheel), no damper, load 0;
- otherwise x is clamped to the bump-stop limit (x ← max(x, limit)), and
  - spring F_s = −k x + (x < knee ? k₂ (x − knee)² : 0),
  - damper F_d from the fitted law, plus bump damping × |v| when x < knee and compressing,
  - tyre load F_z = max(0, F_s + F_d + preload).
- The clamped x is stored as the next x′ (§10.5).

**Dampers** (`model::fit_damper`, `model::damper_force`): F(v) = o + g √(v − t)
with o = −g √(−t), so F(0) = 0, fitted through two (speed, force) points per
direction (speeds = byte × 0.01 × 100/c m/s, forces = setting value × 49 N). The
speed is clamped to ±1 m/s. Compressing (v < 0) uses the bump curve and gives a
positive force; extending uses the rebound curve, negated. Points that do not fit
give no damping.

**Anti-roll bar** (`model::anti_roll`): with d = x_left − x_right on a grounded
axle, the left body force gets −gain · d · |d| and the right the opposite. It acts
on the body forces, not on the tyre loads.

**Load transfer.** The tyre forces make no moments about the CoG; instead the
body forces are shifted explicitly after the lateral step, using this tick's a_y
and longitudinal force sum (§4.1):
- longitudinal: d_long = 2 (m_s / M) · X / L · (force sum); each grounded front
  wheel +½ d_long, each grounded rear wheel −½ d_long;
- lateral: d_lat = m_s · X · a_y / t̄; each grounded left wheel −d_lat, each right +d_lat;

with m_s = M − 2(m_u,f + m_u,r), t̄ the mean track and X the CoG-height parameter
X = ½ (chassis +0x1D × 0.001 + front ride height). The loaded side's support force
drops, so the body sinks there and its springs and tyre loads take the transfer.
R34 GT-R in a steady 0.82 g right-hander at 49 km/h: summed left/right loads 9880 / 6548 N
(test `sim_database_r34_gtr`).

**Body force** of a wheel = F_s + F_d + anti-roll + transfer; the attitude step
(§4.3) subtracts its share of the downforce.

For the renderer, `VehicleSetup::Wheel` publishes `static_suspension` = −x_s and
`ride_height` = q; `WheelState::suspension` is −x′, the coordinate after the droop
and bump-stop clamps, so an airborne wheel is drawn hanging at full droop.

### §4.3 Attitude, heave and the ground frame

**Ground frame** (`model::ground_frame`), from the ground heights g₀…g₃ (FL, FR,
RL, RR) under the four hubs, with w = 1 − w_f:
- U = g₀ + w (g₂ − g₀), V = g₁ + w (g₃ − g₁) (the left and right heights at the CoG);
- height under the CoG = (U + V) / 2;
- pitch = atan2(((g₀ + g₁) − (g₂ + g₃)) / 2, L), nose up positive;
- roll = atan2(U − V, t̄), right side down positive.

The ground frame is the Euler matrix of (ground pitch, ground roll, heading). The
**terrain factor** s_t = (ground up · world up) / (body up · world up) (the latter
floored at 0.2) scales the suspension coordinate (§4.2).

**Attitude step** (`model::attitude_step`), with F_i = body force − damper force −
½ × that axle's downforce, D_i = damper force, and lever factors
l_f = a · 2π / Y, l_r = b · 2π / Y, r_f = (t_f / 2) · 2π / Z, r_r = (t_r / 2) · 2π / Z,
where Y = I_z and Z = I_z / 2:
- pitch acceleration = (F₀ + F₁ + D₀ + D₁) l_f − (F₂ + F₃ + D₂ + D₃) l_r + A;
- roll acceleration = (F₀ − F₁ + D₀ − D₁) r_f + (F₂ − F₃ + D₂ − D₃) r_r;
- rates are clamped to ±π/2 rad/s, then angles move by rate × dt; an angle beyond
  ±π/4 is clamped there and its rate zeroed;
- heave force H = Σ F + Σ D + B; the velocity gains H / m_s · dt along the body's
  up axis, then −g · dt in world Y.

The **pitch terms** A and B (`model::pitch_terms`) come from the longitudinal
tyre forces: q_i = cos d_i × G_i × the axle's section width, with G the drive +
brake parts only (the lateral outputs do not enter, even on a steered wheel);
A = (q₀ + q₁) k_f + (q₂ + q₃) k_r with k_f = ½ · 2π / (Y + M a²),
k_r = ½ · 2π / (Y + M b²); B = (q₀ + q₁)/a − (q₂ + q₃)/b. Drive force lifts the
nose: on a full-throttle launch the R34 GT-R pitches up 0.014 rad (0.8°) as the clutch
takes up, peaking at 0.018 rad (1.0°) later in first gear.

The heave changes only the velocity; the position moves in phase 10 (§4.1).

**Bump-stop nudge** (`model::bump_nudge`): with penetrations p_i = max(0,
bump-stop limit − x_i) of the grounded wheels (x before the clamp),
l = p₀ + w(p₂ − p₀), r = p₁ + w(p₃ − p₁), front mean f̄ and rear mean r̄:
roll += atan2(l − r, t̄) (if l ≠ r), pitch += atan2(f̄ − r̄, L) (if they differ),
height += ½ min(l, r) + ½ min(f̄, r̄). The downward velocity is then limited to
that of the velocity's component along the plane of the front-left wheel's ground
triangle (it may not sink faster than sliding along that plane would). There is no other hull/ground
contact: the ground is touched only through the four wheel probes, the bump stops
and this nudge (hand-built specs use a **[placeholder]** hull penalty contact;
`GT3R_HULL_CONTACT=1` turns it on for spec-body cars for comparison).

**First tick after a reset.** The body is fitted to the ground under the four hubs
(pitch, roll and height, three passes), so a car placed on a slope or banking
starts at rest at its static ride height.

**Spawn hold** (`Vehicle::spawn`, counter 120; pit stop, counter 300,
§8.6). Each tick with the counter n > 0 (counted down):
- the clutch request is 0 and the gear is 1; the planar velocity and yaw rate are zeroed;
- target height = ground height + offset(n), with offset (`model::spawn_height_offset`)
  0 for n < 30, 0.3 m for 30 ≤ n < 270, (300 − n) × 0.01 m above;
- if the body origin is at or below the target, the car snaps to it, flat and at
  rest vertically, and the attitude step is skipped; for 30 ≤ n < 270 the heading
  eases towards the spawn heading at ≤ 30°/s;
- if the body is above the target (the last 30 ticks), it falls normally.

Measured (R34 GT-R, `spawn`, throttle held): 0.300 m up for 91 ticks, then the drop;
on the ground at about tick 106, released at tick 120; the clutch takes up at once.

In the race, respawns use the spawn hold: the player's car (`VehicleAdapter::reset`
in `src/race/backend.cpp`, or `race::RaceField` in a race), the second player's car, and CPU cars put
back on the track by the safety respawn (`ai::respawn_car`, used by `race::RaceField` and
`ai::AiRace`; §11). A standing grid start uses `Vehicle::reset` without a hold and
the race hold during the countdown (§4.1); rolling and flying starts use
`Vehicle::spawn_moving` (§9.3).

### §4.4 Steering and pad input

**Pedals.** On the pad path (a player car whose steering is not a direct angle),
the accelerator and brake are shaped x ← x + 0.1 (1 − x) once x > 0
(`model::pedal_shape`): any press gives at least 0.1.

**Pad kind.** `ControlInputs::controller_kind` is 2 for the analog stick and 3 for
the digital pad (d-pad); 0 keeps the previous kind. A tick with non-zero steering
sets the car's steering mode: digital for kind 3, analog for kind 2
(`src/race/controls.cpp`, `ControlMapper::update`: a held d-pad with the stick
centred asks for full lock as kind 3; the stick is kind 2). A recorded demo past
its last frame feeds an all-zero, digital input (`DemoReplay::inputs`).

**Target angle** (`model::steer_target`): the lock at forward speed u comes from
the car's pad curve B (6 points: steer +0x1E… km/h, +0x24… × 0.2°), clamped to the
full lock (steer +0x11 degrees). The mean velocity-direction angle of the two front
wheels (§3.2), taken here with + = right like the input, extends the bound on its own
side: bound⁺ = min(max(lock, φ̄), full lock), bound⁻ = min(max(lock, −φ̄), full lock);
the target is input × the bound on
the input's side. So when the car slides, the steering can follow the slide up to
full lock on that side only. Without a pad curve the lock is a **[placeholder]**
full lock / (1 + u / 22 m/s).

R34 GT-R pad curve: 51° at 0 km/h (clamped to its 40° lock), 38.8° at 12 km/h,
18.0° at 36, 12.6° at 53, 10.0° at 78, 3.2° at 255 km/h.

**Follower** (`model::steer_follow`): a second-order follower of the target t:
- acceleration = k₁ (t − δ) − k₂ ω, and additionally − k₃ δ while returning towards
  0 (target on the centre side of the angle); k₁ = 100, k₂ = 5, k₃ = 50;
- the rate is limited to 120°/s towards the centre; away from it 120°/s (analog)
  or 30°/s (digital);
- when the angle would cross the target it snaps to it and the rate is zeroed.

Measured (R34 GT-R at rest, full lock 40°): analog 21 ticks, digital 81 ticks.

**Wheel angles.** The front wheels get δ × Ackermann ratio: inner = atan2(L, L −
t_f/2) / (π/4), outer = atan2(L, L + t_f/2) / (π/4), the inner wheel (on the side
the car turns to) getting the larger (`model::ackermann_ratios`; R34: 1.2035 /
0.8455). Toe = atan2((toe setting − 128) × 0.0005, tyre radius), applied as toe-in
on both axles (`model::toe_angle`). The blended **[placeholder]** Ackermann
(`model::ackermann`) is used only when the ratios are unset.

**CPU cars** (AI note). A finite `ControlInputs::steer_angle` sets the road-wheel
angle directly (clamped to ±1 rad), bypassing the pad curve, the follower and the
lock; the pedals are not shaped. CPU cars (`Vehicle::set_cpu_driver`) and scripted
drivers of kinds 1–7 replace the player's clutch request with a binary rule:
from open, engage when rpm > 0.75 × UP; once engaged (clutch state ≠ 0), stay engaged
while rpm ≥ idle. During the countdown the AI holds the request at 0
(`Vehicle::set_clutch_hold`). Driver kind 8 (parked) skips this rule, for CPU cars
too: its request is 0 (§2.4). CPU cars and all scripted kinds read the AI's aid block instead of the car's
own TCS and ASCC (§4.6).

### §4.5 Brakes and ABS

Per wheel (`Vehicle::Impl::brakes`, `model::abs_pressure`):

1. pressure p = the shaped brake pedal (on dirt courses the front is reduced with
   steering, §10.4), then ASCC (§4.6);
2. **ABS** on every wheel with p > 0 (always on):
     p ← p · clamp(F_p · E · (1 − G · g · (s − s*)), 0, 1),  E = r / T_max
   with F_p this tick's peak force, r the tyre radius, T_max the axle's maximum
   brake torque, G the ABS gain of the axle, g the slip gain, s the slip ratio and
   s* the tyre's optimum braking slip (× the ASCC factor of §4.6). A non-positive
   factor gives 0.001, not 0; an unloaded wheel (F_p = 0) therefore keeps 0.001;
3. brake torque = p · T_max; the handbrake adds its torque on each rear wheel.

| Parameter | Value |
|---|---|
| T_max front / rear | brake part +0x11 / +0x12 × 98 N·m per wheel |
| handbrake | brake part +0x13 × 98 N·m per rear wheel |
| ABS gain G | brake-controller setting × 0.1 (front and rear settings) |

Telemetry reports each wheel's pressure after ASCC and ABS (`brake_pressure`) and
its ABS target slip (`abs_target`).

R34 GT-R: 3332 / 2254 / 4900 N·m. Brake torque opposes the wheel's spin
(§2.7). Hand-built specs use a **[placeholder]** total of 1.35 g at a 65 % front
share and 1.5 × the rear torque for the handbrake.

Measured: R34 GT-R 100–0 km/h in 23.5 m (`sim_r34_skidpad_brake`); with full
pedal from 210 km/h ABS starts cutting the rear pressure after about 0.4 s (0.87 at
172 km/h, slip 0.11) and lowers it to about 0.65 by 100 km/h (slip 0.13), while the
front stays near 1.

### §4.6 Traction control, ASCC and the under/oversteer indicators

**Traction control (TCS)** (`model::tcs_factor`, `model::tcs_lateral`). With gain > 0:
- s_min = the most negative slip ratio among the grounded driven wheels (only if < 0);
- g = that wheel's slip gain, s* = its tyre's optimum drive slip (negative);
- lateral factor λ = 1 if |g/μ| ≤ b, else max(0, 1 − (|g/μ| − b) · a), with g/μ the
  previous tick's lateral value (§4.1);
- throttle × clamp(1 + gain · g · (s_min − s* · λ · scale), 0, 1).

| Source | gain | a | b | scale |
|---|---|---|---|---|
| player car (TCSC part, level ∉ {0, 100}) | byte +0x13 × 0.1 (0 at level 100) | +0x14 × 0.1 | +0x17 × 0.01 | level × 0.01 |
| CPU cars, scripted drivers (aid block) | p13 × 0.1 | p11 × 0.1 | p12 × 0.01 | p24 × 0.01 (0 or 100 → 1) |

The TCS level is the setting interpolated between the part's minimum and maximum
(§6); levels 0 and 100 mean off.

**The AI aid block** (`sim::ScriptedAids`, `Vehicle::set_scripted_aids`). CPU cars
(`Vehicle::set_cpu_driver`) and a non-zero driver kind outside the spawn/stop
counter (§8.6) read this block instead of the car's own TCS and ASCC. It is filled
from the car's AI driver parameter set (bytes p0…p24, `ai::DriverParams::from_bytes`;
AI document §9.1): the TCS terms above, and the ASCC channels of the table below with
gain_A = p9 × 0.01, threshold_A = p10 × 0.01, flags = p19, gain_D = p20 × 0.01,
gain_C = p21 × 0.01, threshold_C = p22 × 0.01, gain_E = p23 × 0.01. Every built-in
set gives TCS 6.0 / 1.0 / 0 / 1.0 and channel flags 0 (no ASCC); a set with
non-zero flags turns the channels on. ABS is not part of the block: it always uses the car's own
gains (§4.5). The race passes the block to every CPU car and to the player's car for
its scripted drivers (run-up, pit lane, end of run; §8.6, §9.4).

**Indicators** (every car moving forward, u > 0), from the previous tick's values:
- **understeer** (`model::understeer_indicator`): 0 unless |a_y| / (4 × 9.8) > 0.2
  and the yaw rate r does not oppose the steer; r gets half the yaw acceleration
  added when they share a sign; target r_t = u · sin|δ| / L; understeer =
  clamp(1 − |r| / r_t, 0, 1). Using sin δ here is an **interpretation**;
- **oversteer**, player (`model::oversteer_indicator`): from the largest rear |β|
  of the previous tyre phase and the planar speed v: q = min(1, 0.018 v), threshold
  θ = 20° → 5° and gain 3 → 14 as q goes 0 → 1; oversteer = clamp(gain · (|β| − θ)
  / (90° − θ), 0, 1);
- **oversteer**, CPU and scripted (`model::oversteer_alt`): 2 · max(0, |v · r| −
  |a_y|) / (4 × 9.8) when v · r · a_y < 0, else 0 (not clamped).

**ASCC** (`model::ascc_pressure`), for cars moving forward without the
handbrake. The outside wheels are the right ones when turning left (r > 0) and the
left ones otherwise. Each channel has a gain, and A and C a threshold; flag bits
enable each channel per axle (bits 0–7: A front/rear, D front/rear, C front/rear,
E front/rear). A player car takes them from its ASCC part and settings (§6); CPU
cars and scripted drivers take them from the AI aid block above, and run no ASCC
while its flags are 0 (the car's own ASCC is never used on that path).

| Channel | When | Wheels | Effect |
|---|---|---|---|
| A | brake pedal exactly 0 | outside | p += (oversteer − threshold_A) · gain_A when above the threshold |
| C | brake pedal and throttle exactly 0 | inside | p += (understeer − threshold_C) · gain_C when above |
| D | brake pressed | inside | p ×= clamp(1 − (oversteer − 0.1) · gain_D, 0, 1) |
| E | brake pressed | outside | p ×= clamp(1 − (understeer − 0.1) · gain_E, 0, 1) |

D and E scale the wheel's ABS target slip by the same factor. The pressure is reset
to the pedal every tick and clamped to [0, 1]. The handbrake disables ASCC only.

Measured (`sim_scripted_ascc_block`, an invented block with A and C on both axles):
lifting off with the wheels turned at 110 km/h, a CPU car's A and C channels brake
wheels without the pedal; with the block's flags 0 a CPU car brakes no wheel even
though the car's own ASCC is on, and a player car uses only its own.

**The Options menu's ASM / TCS** (`sim::apply_aid_options`). In arcade single
race, free run and 2P battle (both players' cars) the race applies the two
switches to the car's own aids when it sets up; other modes ignore them, and a
saved replay keeps the aids its run used. ASM off clears the ASCC flags and all
four channel gains, so no channel acts and the dash ASM lamp (lit while any ASCC
gain is above 0, `model::asm_lamp_lit`; presentation §D3) goes out. TCS off sets
the TCS gain to 0. On keeps what the car's parts give. ABS and the AI aid block
(CPU cars, scripted drivers) are not affected.

Measured (`sim_options_asm_tcs_off`, a test FR car with TCS and ASCC): with both on,
lifting off with the wheels turned at 110 km/h brakes wheels to 0.35, braking into a
turn lowers the ABS target slip to 0.087 (optimum 0.100), and a full-throttle launch
cuts the throttle to 0.09. ASM off gives no pressure without the pedal and the
target stays 0.100, with TCS unchanged; TCS off leaves the throttle factor at 1, with
ASCC unchanged.

Telemetry reports the indicators (`understeer`, `oversteer`) and the TCS throttle
factor (`tcs_factor`, 1 = no cut).

### §4.7 Aerodynamic parameters

From the body part (`build_car_spec`):

| Parameter | Value |
|---|---|
| frontal area A | body f32 +0x44 m², quantised to 0.0001 m² (width × height if not above 0.1 m²) |
| C_d | body +0x39 / 100 (0.33 if 0) |
| air density ρ | 1.2 kg/m³ |
| A_f | ½ ρ A |
| downforce C_front, C_rear | A_f × the front / rear downforce setting % (stock: body +0x13 / +0x3C) |
| drag coefficient C_drag | A_f · C_d + 0.1 (C_front + C_rear) |

R34 GT-R: C_drag = 0.511, C_front = 0.348, C_rear = 0.564 N/(m/s)². The forces
are §5.2. Hand-built specs use a **[placeholder]** C_drag = ½ ρ · C_dA.

---

## §5 Air, contact and collisions

### §5.1 Broad phase

Car pairs are tested only when they are in the same height band, so cars on a
bridge do not touch cars below: |Δy| ≤ ½ (h_a + h_b) + 5 m (`World::tick`). The
swept test has no further broad phase (so fast cars cannot tunnel); its static
separation and `collide::collide_pair` skip pairs whose bounding circles do not overlap.

Walls are taken within reach of the car: the segments of the course's wall lines
within the octagon's bounding radius + the distance moved this tick + 9 m in x/z,
whose height span overlaps the band from 5 m below to 8 m above the car's centre
(`TrackWorld::wall_segments`).

### §5.2 Aero forces and slipstream

**Slipstream factor** of a follower behind one leader (`model::slipstream_factor`),
with the follower at (x forward, y lateral) in the leader's frame, the leader's
forward speed v and half-width h:
- (v + x) / v when v > 100 km/h, −v < x < 0 (within one second of the leader's travel)
  and |y| < h;
- 0 otherwise.

`World` sums the factors from every other active car and clamps the sum to 1
(`Vehicle::set_slipstream`); this phase runs for every car before any car's tick.

**Filter** (`model::aero_step`): y ← min(1, y + target × 2 /s × dt) while the target
is non-zero, else y ← max(0, y − 0.5 /s × dt).

**Forces**, with u the ground-frame forward speed:
- drag = −sgn(u) · C_drag · (A + (1 − A)(1 − y)) · u², A = 0.5;
- downforce_j = C_j · (B + (1 − B)(1 − y)) · u², B = 1 (so the slipstream does not
  change downforce).

Full slipstream halves the drag. The drag enters the forward step (§4.1); each
wheel's body force loses half its axle's downforce (§4.3).

Measured (two R34 GT-Rs in a `World`, 10 m apart from 150 km/h at part throttle):
after 2 s the follower's factor is 1 and its drag 474 N against the leader's 934 N.

### §5.3 Collisions

Collisions are 2-D, in the world x/z plane (`src/sim/collision.h`). A car is an
**octagon** around its CoG (`collide::octagon`): with a = CoG → front, b = CoG →
rear, h = width / 2 and chamfer c = 0.2 × width, the points (x lateral, y forward)
are (h, a − c), (h − c, a), (−h + c, a), (−h, a − c), (−h, −b + c), (−h + c, −b),
(h − c, −b), (h, −b + c). The extents come from the body record measured from the
wheelbase midpoint (§6.2), so a = front extent − L(w_f − 0.5) and b = rear
extent + L(w_f − 0.5) (extents of 0.3 m or less fall back to half the length).
`VehicleSetup` publishes these CoG-relative extents and the CoG offset. The
collision **yaw inertia is 10 · I_z** (`Vehicle::collision_body`). Body-local
points convert to world with `collide::world_point`.

#### Walls

Course walls are polylines whose drivable side is on the **left** of the line
direction (`docs/formats/course.md`). Walls are one-sided: only a point that
starts the tick on that side and crosses to the other is stopped. Edges flagged as
sensors record a crossing without a response (none on the US disc).

The wall step (`collide::collide_walls`) runs after phase 12 from the pose at the
start of the tick to the pose at its end, for up to 2 rounds:

1. **Held contacts** (below) are re-projected first; if that moved the body, the
   end pose is recomputed from the new velocity.
2. **Sweep**: each octagon point that is not held moves on a straight chord from
   its start to its end position (position and heading interpolated linearly); the
   earliest crossing of a wall segment within the segment's span wins.
3. Advance the body to the hit and apply the **impulse** (`collide::wall_impulse`)
   at the point, with the wall normal n pointing to the track side:
     j = −(1 + e)(v_p · n) / K,  K = 1/m + (r × n)² / (10 I_z),  e = 0.05
   (only when approaching). **Friction** min(μ · j, m · |v_t|), μ = 0.5, acts
   against the tangential contact velocity, including the vertical velocity (which
   it never reverses), with no torque.
4. Flag the point as held on that segment; re-sweep the rest of the tick with the
   new velocity.

Then, in two passes, any point still beyond a wall by less than 1.5 m (within the
segment's span) is **pushed out** perpendicular to it; a push-out of at most 0.5 m
also flags the point as held.

**Held contacts** (`collide::WallContacts`, `hold_wall_contacts`): a flagged point
stays on its wall line while the car presses in. Each round, if the point slid past
an end of its segment, it moves on to the neighbouring segment once. The flag drops
when the CoG velocity points away from the wall or the point is on the line or back
on the track side (within 10⁻⁵ m). Otherwise the body moves so the point lies on the
wall line, the velocity is projected on the last hit wall's tangent, and the yaw rate
gets the angular part of that velocity change applied at the point (no strength is
reported). A car that keeps pressing in therefore slides along the wall; once a
flag drops, a new crossing takes a fresh impulse with friction. Placing a car clears
its held contacts.

**Stuck assist**: after a wall hit, if both horizontal velocity components are
below 1 m/s, yaw rate += −steer × 20 × dt (steer = the input, −1 … +1, + right), so
the steering still turns a car pinned against a wall. The yaw rate is then clamped
to ±62.83 rad/s.

`VehicleTelemetry::wall_impulse` is the sum of |j| this tick (|j| / 10⁵ is the hit
strength for sound and HUD). A `WorldQuery` without wall segments falls back to a
**[placeholder]** sphere contact around the body (`WorldQuery::wall_contact`).

Measured (`sim_wall_swept_octagon`): driving straight into a wall at −40 m, the nose
stops at −40.00; at 0.5 rad yaw at −39.73.

#### Car to car

After every car's tick, `World::tick` runs `collide::collide_pair_swept` for each
active pair (a < b, index order) in the same height band (§5.1). Each car's
**start pose** is its collision body before the tick; the **velocities** are those
after the tick. In the step's plane frame (X, Y) = (x, −z), for at most 2 rounds:

1. **Static separation** along the line of centres, both orderings
   (`pb_separate`): for each point of one body inside the other's outline, the
   vector along the centre line to the boundary; the deepest decides, and each
   body moves half of it.
2. **Persistence** of points flagged by an earlier response (`pb_persist`): a point
   that left its edge's span, or lies outside the edge line, drops its flag;
   otherwise both bodies move half-way to put it on the line. In both of the last
   two cases, when the bodies approach through the edge the relative normal velocity
   is removed, half from each body (not mass-weighted).
3. **End pose** = start + v · dt · (1 − t) (heading likewise), with t the time
   already used.
4. **Pair sweep** (`pb_sweep`), both orderings: each unflagged point of one body
   moves on a straight chord in the other body's frame; a hit is a crossing from
   outside to inside an edge with the chord passing between the edge's end points.
   A point starting on an edge line (within 10⁻⁵ m) is treated as already inside (no
   new hit). The earliest hit wins; ties go to body 0.
5. Advance both bodies to the hit. If the hitting point is still outside the edge
   plane, each body moves half the gap along the normal (**touch-up**).
6. **Impulse** at the crossing point with the edge's normal:
   j = −1.5 (v_rel · n) / K, K = 1/m_a + 1/m_b + (r_a × n)²/(10 I_a) + (r_b × n)²/(10 I_b):
   restitution 0.5, no friction, and not skipped for separating contact. Flag the
   hitting point with that edge.

The bodies end at the last end pose computed; an impulse in the last round changes
only velocities. A body that ends nearly stopped (|v_x|, |v_z| < 1 m/s, strict)
with the last hit early in the tick (t < 0.1) gets the stuck-steer nudge (yaw rate
+= −steer × 20 × dt, steer as a fraction of full lock) and a recomputed end pose.
Applying the nudge before the end pose is an **interpretation**. Yaw rates are
clamped to ±20π rad/s.

`World::contacts()` reports each contact (cars, point, normal from a to b, |j|).
In online play a remote car is kinematic: its start pose is taken back along its
motion, the contact is solved with both bodies, and only the local car keeps the
result (`World::set_kinematic`). `Vehicle::apply_impulse` applies an impulse J at
a point as the pair step does: v += J / m, yaw rate += (r × J) / (10 I_z).

`collide::collide_pair` is a static (unswept) version: a separating-axis test of
the two octagons, the deepest vertex of either body inside the other against its
nearest edge, the same impulse, then the overlap split half and half.

Measured: `sim_world_car_contact` (head-on): one contact tick, minimum centre
distance 4.61 m; `sim_swept_car_contact_no_tunnelling`: a body moving 180 m/s,
which would jump past the other within one tick, is caught by the sweep and leaves
the contact at 75 m/s.

#### Front and rear hit flags

Each tick a car reports which end of it the collision responses of that tick acted
on (`VehicleTelemetry::hit_sides`: bit 0 front, `collide::kFrontHit`; bit 1 rear,
`collide::kRearHit`). The AI's recovery reads them (AI document §7). With the octagon
points numbered as listed above (0–3 at the front, 4–7 at the rear) and edge k
running from point k to point k + 1:

- **walls**: every non-sensor hit of the sweep sets the side of the octagon point
  that hit (`collide::point_side`); push-outs and held contacts set nothing;
- **car to car**: every impulse of the pair step sets, on the car whose point hit,
  that point's side, and on the car whose edge was hit, the edge's side
  (`collide::edge_side`): edges 0–2 front, 4–6 rear; the side edges 3 and 7 set
  neither. `World::tick` adds these to the wall bits after the cars' ticks.

The flags start at 0 every tick, and a car synced from the network
(`Vehicle::sync_to`) reports none.

---

## §6 Car parameters from the database and the player's settings

Before a car can drive, the engine turns its database records into one flat
parameter set in SI units, `sim::CarSpec` (`src/sim/car_spec.h`). The builder
is `sim::build_car_spec` in `src/sim/car_spec.cpp`. It reads:

- the car record (table 30) and its list of installed parts, or a car variant
  (table 31, opponent and prize cars; table 35, arcade cars). A variant's part
  slots that are 0 take the base car's part;
- the part records those slots point to (see `docs/formats/database.md`
  for the tables and offsets used below);
- the player's settings for the car (`sim::CarSettings`, §6.6). Without
  explicit settings the builder uses the stock settings of the installed parts.
  Car variants always use the stock settings: their own settings bytes
  (+0xE8 onwards) are not decoded yet.

`CarSpec::finalize` then fills in anything the records left open (§6.3).
Building fails, with an error message, if the label is unknown, the chassis
record has no weight or wheelbase, the engine record is missing or too short,
or the torque curve comes out empty. All 244 cars in the car table of the
test disc build without error.

The vehicle simulation (§0-§5, in the first part of this document) never reads
the database itself. It reads only `CarSpec`. Tests can also build a car by
hand (`CarSpec::test_car`, an R34-like car whose values are a placeholder).

A few of the builder's rules look like quirks. The engine keeps them on
purpose, because they change how cars behave:

- Both suspension blocks read the **front** ride-height setting. The rear
  ride-height setting is stored but has no effect (§6.6).
- The tyre records' "base grip" is 9.8 for every car. So wherever the aids
  divide by the sum of the four wheels' base grip, the divisor is the same
  for every car (`Vehicle::Impl::steer_indicators`, `traction_control`,
  `planar_step`): 4 × 9.8 for the lateral gates of the
  under/oversteer indicators, and 9.8 × the sum of the four wheels' surface
  μ for the lateral g / μ input of TCS.
- The drivetrain-split setting (word 0x118) is stored but never read. The
  4WD share always comes from drivetrain +0x13 (§6.6, §7 #59).
- The fixed torque of the shift clutch (clutch state 3, §2.4) has no source,
  so it is always 0 N·m (§6.2).

### §6.1 Adjustable settings (step, min, max triplets)

Many tuning parts store an adjustable value as three bytes: a step count *n*,
a minimum *a* and a maximum *b*. The player's setting is an index *i* from 1
to *n*. The value the engine uses is

  v = a  if n < 2,  otherwise  v = a + (b − a)·(i − 1) / (n − 1),

computed in integers. The division truncates towards zero. This rule
(`interp_setting` in `src/sim/car_spec.cpp`) gives the damper forces, the
anti-roll bar rates, the TCS level, the four ASCC gains and the front and
rear ABS gains. Each value is then scaled by the factor given for it in
§6.2, §6.6 or §4.

### §6.2 How installed parts change the parameters

Each part changes the parameters through integer percentage or per-mille
multipliers. A factor byte of 0 means "no change". The multiplications are
done in integers in a fixed order, so the order matters for rounding.

**Engine output factors.** Two factors start at P = 1000 (per mille, the
"power factor") and Q = 100 (percent, the second factor). The engine parts
are applied in this order: port polish, engine balance, displacement,
computer, NA tune, turbo kit, **port polish again**, muffler, intercooler.
Port polish runs twice, so its factor applies squared (102 % gives
1000 × 1.02 × 1.02 = 1040 ‰).

| Part | Effect |
|---|---|
| port polish, displacement, computer, muffler, intercooler | P ← P·(+0x11)/100, Q ← Q·(+0x12)/100 |
| engine balance, NA tune | +0x11 added to every torque-curve rpm byte (the curve moves up the rev range); +0x12 added to the rev-limit byte; P ← P·(+0x13)/100, Q ← Q·(+0x14)/100 |
| turbo kit | P ← P·(u16 +0x1C)/100, Q ← Q·(+0x1B)/100; +0x18 added to the rev-limit byte and +0x19 to the up-shift byte; boost cap 1 + 0.1·(+0x11) (the boost stages come from the settings, §6.6) |

The torque curve is then built from the engine record with the output
factor (engine +0x42 × 10), P (capped at 65535) and Q (capped at 255). The
curve itself is described in §2.1. From the engine record and these adders:

| Parameter | Rule |
|---|---|
| idle | engine +0x44 × 10 rpm (900 rpm if 0) |
| clutch bite ("launch") rpm | engine +0x43 × 10 rpm (2500 rpm if 0) |
| up-shift rpm UP | up-shift byte × 100 rpm (engine +0x46 plus the turbo adder) |
| rev limit | max(L, UP + 200 rpm), where L = rev-limit byte (engine +0x45 plus the adders) × 100 rpm, or the last curve point if the byte is 0 |

**Drivetrain** (table 13, scaled by the flywheel, clutch and propeller-shaft
parts):

| Parameter | Rule |
|---|---|
| drive type | +0x14: 0 FR, 1 FF, 2 4WD, 3 MR, 4 RR |
| AWD system B3 | +0x15 (0xFE = none) |
| 4WD share B4 | +0x13 |
| engine inertia J_e | u16 +0x20 × flywheel +0x12 % × clutch +0x12 % × 0.001 kg·m² |
| engine friction word | u16 +0x16 × flywheel, clutch and shaft +0x11 % each; k_f = word × 0.98 / ω_last N·m per rad/s, where ω_last is the last rpm point of the curve in rad/s, so engine braking at the top of the curve is word × 0.98 N·m (§2.2) |
| wheel spin inertia, front / rear | u16 +0x18 / +0x1A × flywheel +0x13 / +0x14 % × clutch +0x13 / +0x14 % × shaft +0x12 / +0x13 %, kept to 16 bits, × 0.01 |
| shaft inertia, front / rear | u16 +0x1C / +0x1E × shaft +0x14 / +0x15 %, kept to 16 bits, × 0.01 |
| shift-clutch torque | no source: always 0 N·m (§2.4) |

**Body and chassis geometry.**

- Collision box: the body record's four floats at +0x20 are the front and
  rear extent (mm, measured from the wheelbase midpoint), width and height,
  each truncated to whole millimetres. A zero front or rear extent means
  half the chassis length (in whole mm). A positive width or height replaces
  the body width and chassis height. These extents give the collision
  octagon (§5.3) and the grid placement (§9.2). For the R34 GT-R the extents
  are 2.262 m front, 2.335 m rear, and the box is 1.779 m wide and 1.244 m high.
- Track width per axle: tread (body +0x32 / +0x34) − 2 × wheel offset
  (signed bytes +0x3D / +0x3E), in mm.

**Brakes** (table 0): front / rear maximum brake torque per wheel =
+0x11 / +0x12 × 98 N·m; handbrake torque on each rear wheel = +0x13 × 98 N·m.

**Aerodynamics** (body record): frontal area A from the float at +0x44,
rounded down to 0.0001 m² (the width × height box if the stored area is
0.1 m² or less). The drag coefficient C_d is +0x39 / 100 (0.33 if 0). Air
density is ρ = 1.2. With A_f = ½·ρ·A:

  C_front = A_f · (front downforce setting %)/100,  C_rear = A_f · (rear setting %)/100,
  C_drag = A_f · C_d + 0.1·(C_front + C_rear).

The R34 GT-R gets C_drag 0.511, C_front 0.348 and C_rear 0.564. How these
coefficients turn into forces is in §4.7 and §5.2.

**Differentials** (LSD part, table 21): the type letter at +0x11 (front) and
+0x1B (rear) becomes its index in the string `frnmhvyt`. A letter that is
not in the string leaves that axle open. The initial, acceleration and
deceleration values come from the settings (§6.6); their effect is in §2.7.

### §6.3 Derived values and fallbacks

`CarSpec::finalize` completes a spec. It is idempotent: running it twice
changes nothing. For every disc car it touches only values the database
does not provide. It and a few fallbacks in `build_car_spec` supply the
following **[placeholder]** conventional values, not the game's:

- mass at least 300 kg; front weight share clamped to 0.3-0.7;
- spring, damper and anti-roll rates for cars without a suspension block,
  from a 1.6 Hz ride frequency and 0.35 damping ratio (no disc car lacks the
  block);
- brake torque if the brake record gives none: 1.35 g at a 65 / 35
  front / rear split; handbrake 1.5 × the rear brake torque;
- the wheel inertia used when the drivetrain has no spin-inertia word:
  clamp(12·r² + 0.3, 0.6, 3.0) kg·m² for tyre radius r;
- a 5-speed gearbox, a final drive of 4.0, a reverse ratio of 3.3 and a
  simple torque curve if the records have none;
- gear ratios stored as "not set" (−999) in a gearbox that is not a computed
  one (§6.7): filled in geometrically between the first and top gear;
- idle clamped to 500-2500 rpm; a rev limit less than 1500 rpm above idle
  is replaced by the last curve point;
- extents of 0.3 m or less become half the length; the centre-of-gravity
  height of a car without a suspension block becomes max(0.35 m, 0.37 ×
  height);
- the tyre B curve, when a compound stores none, is a generic shape (§6.5).

finalize also computes the toe angles and the Ackermann ratios (§4.4) from
the settings and geometry. A yaw-inertia parameter of 0 becomes 6000 (§6.4).

### §6.4 Mass, inertia, steering and camber data

**Mass.** With W = chassis weight (kg, +0x1A), l = lightweight part u16
+0x12 (‰) and k = body u16 +0x30 (‰):

  m = ⌊⌊W·l / 1000⌋·k / 1000⌋ + 120 kg.

A zero factor is skipped. The R34 GT-R V-spec
comes out at 1560 + 120 = 1680 kg.

**Yaw inertia.** With c = chassis +0x1C, the lightweight part's +0x11 (%)
and the body's +0x38 (%):

  I_z = 100 · ⌊⌊c·lw% / 100⌋ · body% / 100⌋,  6000 if the result is 0.

These are game units (the R34 GT-R has 9500). Pitch and roll use I_z / 2π
and I_z / 4π (§4.1, §4.3).

**Other chassis values:** front weight share = chassis +0x10 / 100 (0.55 if
0); wheelbase +0x18 mm; length +0x14 mm; front / rear axle grip factor
((byte + 124) mod 256 + 900) / 1000 from +0x12 / +0x1E (§3.1). The
centre-of-gravity height parameter is ½·(chassis +0x1D mm + front ride
height) (§4.2).

**Steering.** Full lock = steer part +0x11 degrees (the R34 GT-R: 40°). The
speed-dependent pad curve (curve B of §4.4) has six points: speeds +0x1E..
+0x23 in km/h, limits +0x24..+0x29 × 0.2°. The curve is used only if its
first limit is non-zero and the speeds never decrease. Otherwise the
steering falls back to a [placeholder] curve, lock / (1 + v / 22 m/s).

**Camber curves** (§3.1): per axle, four angle codes a (suspension
+0x60.. front, +0x68.. rear) and four multiplier codes m (+0x64.. front,
+0x6C.. rear) give the points (angle (128 − a)·4°/32, factor m/200). The
points are sorted by angle. The curve is used only if every factor is
above 0.2; otherwise camber has no effect on grip.

### §6.5 Tyre records

Each axle's tyre part (tables 28 / 29) points to a tyre size and to three
pairs of a compound record (table 27) and a force-volume record (table 26).
Pair *k* gives the tyre record for **compound slot** *k*. While driving, each
wheel uses the record of the slot that matches the class of the surface
under it (`Vehicle::Impl::tire_for`, `CarSpec::surface_slot`):

| Surface index (§3.6) | Slot |
|---|---|
| 0, 1, 7-11 (tarmac, kerb, paving, ...) | 0 |
| 2-5 (grass, gravel, sand, dirt) | 1 |
| 6, 12-15 (6 and the wet surfaces) | 2 |

Road cars carry the same compound in all three slots. For example, every
slot of the R34 GT-R is "normal".

**Tyre size** (table 25): radius = rim inches × 0.0254 / 2 + section width ×
aspect ratio. The section width is the width code × 5 mm; the aspect ratio
is the aspect code × 5 %.

**Record contents** (`tire_block` and `tire_record` in
`src/sim/car_spec.cpp`; curve offsets are in the compound record):

| Field | Source |
|---|---|
| base μ | compound +0x08 / 100 (1.00 if 0) |
| slip-angle range θ | compound +0x09 degrees (45° if 0), at most 90° |
| A: lateral force fraction vs. \|slip angle\| | 8 points: x = (+0x12..) × θ/255, y = (+0x1A..)/200 |
| C: longitudinal force fraction vs. slip ratio, D: friction-circle limit | one signed 11-point slip axis from −1 to 1. Wheel-spin side: x = −(+0x22..+0x27)/255, C = (+0x28..)/200, D = (+0x2E..)/200. Braking side: x = (+0x34..+0x39)/255, C = (+0x3A..)/200, D = (+0x40..)/200. The two sides share the point at 0, where C = 0 and D takes the braking side's value |
| B: force along the patch velocity vs. slip angle | 6 points: x = (+0x46..) × 90°/255, y = (+0x4C..)/200. If the compound stores no B points, a generic [placeholder] curve is used |
| L: load sensitivity | 4 points: x = (+0x0A..) × 196 N per metre of section width, y = (+0x0E..)/200 |
| μ[s] per surface | μ × force-vol byte % (0 = 100 %): +0x08..+0x11 for surfaces 0-9, +0x12..+0x14 for surfaces 12-14 |
| wear rate | compound +0x52 / 100 (§8.2) |

The first point of curves A, B and C (both sides) is set to (0, 0), and
L's first x to 0; D and L keep their first y. Missing force-volume bytes default
as follows: in slot 0, the wet surfaces 12 / 13 / 14 take the values of
surfaces 0 / 1 / 9; in slots 1 and 2, any zero surface byte takes slot 0's
value. Entry 11 of the μ table is not a surface: it holds min(1, 1.25 / μ),
the floor of the wear grip factor (§8.4).

Two **optimum slips** are read off curve C. Going outwards from 0 on each
side, the engine finds the first maximum. The wheel-spin side gives the
drive optimum (used by TCS and by the shift veto), the braking side gives the
brake optimum (the ABS target, §4.5).

The R34 GT-R's "normal" tyre has μ 1.45, μ on grass 0.87, drive optimum
−0.180 and brake optimum 0.137. The tyre force model itself is §3.

### §6.6 The player's settings

`sim::CarSettings` holds the setting words of a garage car (offsets 0xF0 to
0x1C8 of the saved car record) as raw values. `stock_settings` gives the
defaults of the installed parts. GT mode's Settings pages edit a copy, and
`build_car_spec` converts it.

| Words | Setting | Stock value from | Conversion |
|---|---|---|---|
| 0xF0, 0xF4..0x10C | reverse, gears 1-7 (× 1000; −999 = none) | gear +0x12, +0x14.. | ratio = word × 0.001 (reverse: \|word\| × 0.001); a computed gearbox regenerates them (§6.7) |
| 0x110 | final drive (× 1000) | gear +0x22 | × 0.001 |
| 0x114 | gearbox slider (design top speed / 10 km/h) | gear +0x29 | §6.7 |
| 0x118 | drivetrain split | drivetrain +0x13 | stored, not read |
| 0x11C / 0x120 | brake controller index, front / rear | brake controller +0x14 | ABS gain = §6.1 value × 0.1 (front: triplet +0x11/+0x12/+0x13; rear: +0x15/+0x16/+0x17) |
| 0x124 / 0x128 | downforce front / rear, % | body +0x13 / +0x3C | C_front / C_rear = A_f × % (§6.2) |
| 0x12C..0x140 | turbo: stage-0 peak, knee, rate; stage-1 peak, knee, rate | turbo +0x12..+0x17 | peak × 0.1, knee × 100 rpm, rate × 0.1 /s. The turbo is on if the cap and stage 0's three values are all non-zero; stage 1 is active if its three values are (§2.3) |
| 0x144 / 0x148 | camber, front / rear | suspension +0x13 / +0x16 | camber offset −setting × 0.1° (§3.1) |
| 0x14C / 0x150 | ride height, mm | suspension u16 +0x54 / +0x5A | q = **front** value × 0.001 m on both axles |
| 0x154 / 0x158 | toe (128 = neutral; 0 is treated as 128) | suspension +0x19 / +0x1C | toe angle atan2((byte − 128) × 0.0005, r) (§4.4) |
| 0x15C / 0x160 | spring, 0.1 kgf/mm | suspension +0x23 / +0x26 | k = spring × 980 × c / 100 N/m |
| 0x164 / 0x168 | motion ratio c, % | suspension +0x27 / +0x28 | scales k and the damper reference speeds |
| 0x16C..0x188 | damper indices: front bound 1, 2, rebound 1, 2; rear the same | suspension +0x2E, +0x31, +0x35, +0x38, +0x3C, +0x3F, +0x43, +0x46 | force at each reference speed = §6.1 value × 49 N; reference speed = byte × 0.01 × 100/c m/s |
| 0x18C / 0x190 | anti-roll bar index | suspension +0x4A / +0x4E | gain = §6.1 value × 98 N/m² |
| 0x194..0x1A8 | LSD initial front, rear; accel front, rear; decel front, rear | LSD +0x12, +0x1C, +0x15, +0x1F, +0x18, +0x22 | raw (§2.7) |
| 0x1B0 | TCS index | TCSC +0x11 | see below |
| 0x1BC..0x1C8 | ASCC indices A, D, C, E | ASCC +0x12, +0x1B, +0x21, +0x26 | see below |

The **front** rebound reuses the front bound indices (0x16C for stage 1,
0x170 for stage 2), so the front words 0x174 / 0x178 have no effect. The rear
uses all four of its words. The other suspension values (bump-stop rate,
unsprung mass, travel, knee gap, bump damping) come from the suspension
part, not from the settings. §4.2 describes what they do.

**Traction control** (TCSC part, table 22). The level is the §6.1 value
(steps +0x12, min +0x15, max +0x16, index 0x1B0), in percent:

- TCS is active only if the level is neither 0 nor 100;
- slip scale = level / 100 (1 if the level is 0 or 100);
- gain = +0x13 × 0.1 (0 if the level is 100);
- lateral terms a = +0x14 × 0.1, b = +0x17 × 0.01.

**Stability control** (ASCC part, table 23):

- gains A = §6.1(+0x11, +0x14, +0x15) / 100, D = §6.1(+0x18, +0x19, +0x1A) / 100,
  C = §6.1(+0x1E, +0x1F, +0x20) / 100, E = §6.1(+0x23, +0x24, +0x25) / 100;
- thresholds A = +0x13 / 100, C = +0x22 / 100;
- enable flags: a byte made from the low two bits of +0x17, +0x16, +0x1D
  and +0x1C (in that order, from bit 0). Its bits 0-7 are A-front, A-rear,
  D-front, D-rear, C-front, C-rear, E-front, E-rear.

How these values act is §4.5 and §4.6. As an example, the stock R34 GT-R gets
TCS gain 5.0 and slip scale 0.5, ABS gain 9.0 on both axles, and ASCC flags 0x41
(A-front and E-front).

Passing a car's stock settings explicitly gives the same spec as building
it without settings. This is tested on the R34 GT-R (`sim_car_settings`,
whose stock values are springs 40 / 50, motion ratio 95 %, ride height
130 mm, downforce 29 / 47 % and final drive 3.545).

### §6.7 Computed (adjustable racing) gearboxes

A gear record with +0x28 non-zero describes a computed gearbox. Its ratios
are generated from the gearbox slider *s* (design top speed / 10 km/h)
instead of being stored. `model::generate_gear_ratios` (`src/sim/models.cpp`)
works with:

- n = forward gear count (gear +0x11);
- the rev byte R_b = engine +0x45 + engine balance +0x12 + NA tune +0x12 +
  turbo +0x18;
- the final-drive word F (setting 0x110).

The tyre used is the **driven** one: front for FF and for 4WD system 0,
rear otherwise. Its rolling radius is

  r = rim × 0.0254 / 2 + W × aspect code × 0.05,

where W = 5 × width code mm. A width code of 0 gives no sidewall on the
front path and 10 mm on the rear one.

The steps:

1. t = (10s − 150) / 250; design speed v = 10s km/h in m/s.
2. Top-gear ratio g = (100·R_b rpm in rad/s) · r / v / (F × 0.001). This puts
   the rev limit at the design speed in top gear.
3. Step factors from the table below (b₀..b₅, chosen by n, clamped to 5-7):
   q(h, l) = 200/b_l + t·(200/b_h − 200/b_l); q₁ = q(0, 1), q₂ = q(2, 3),
   q₃ = q(4, 5).

   | n | b₀ | b₁ | b₂ | b₃ | b₄ | b₅ |
   |---|---|---|---|---|---|---|
   | 5 | 155 | 165 | 188 | 190 | 190 | 200 |
   | 6 | 165 | 175 | 190 | 192 | 190 | 200 |
   | 7 | 170 | 180 | 192 | 194 | 190 | 200 |

4. From gear n down to gear 1: gear 1 is first multiplied by q₃. The gear's
   word is the truncated value of 1000·g, kept to 16 bits. Then g ← g·q₁ and
   q₁ ← q₁·q₂. Each lower gear is therefore taller by a growing step.
5. Reverse word = (3 × gear-1 word + gear-2 word) / 4, in integers.

The arithmetic copies the console's single-precision floats bit for bit.
Multiplications, additions and subtractions round towards zero. An addition
first drops the smaller operand's mantissa bits beyond one guard position.
Denormals become zero and overflow saturates. Divisions round to nearest.
The final drive is converted in double precision and then rounded once.
Constants are the exact float words. Conversion to an integer truncates and
saturates. Doing this makes the generated words bit-identical to the game's
(docs/dev/PHYSICS.md: all 234 cars and a random set of 10,564 words, no
mismatch).

When the generator runs: `stock_settings` uses it for a computed gearbox's
default words. `build_car_spec` runs it again from the given slider and
final drive and ignores the given gear words, as the game does. `apply_gear_slider` runs it when the player moves the slider.
Check value: the zz2 car's first gear is 3.194 at its stock slider. Raising
the slider by 3 makes the top gear taller, from 0.758 to 0.698
(`sim_computed_gearbox_slider`).

## §7 Numbered reference rules

A few small rules are cited by number (#n) because several subsystems share
them. Each is a pure function in `src/sim/models.h`.

**#12, TCS lateral factor** (`model::tcs_lateral`). For the lateral input x
(lateral g over the mean surface μ, §4.6) and the TCS terms a, b (§6.6):

  f = 1 if |x| ≤ b, otherwise max(0, 1 − (|x| − b)·a).

**#21, driveline class** (`model::driveline_class`): FF → 1; 4WD with
system 0 → 1, system 3 → 3, any other system → 2; FR, MR, RR → 0. The race
uses the class to choose the course's AI parameter set and an event's per-class
AI overrides (`src/race/field.cpp`; see the AI document).

**#24, gear for a road speed** (`model::gear_for_speed`). For speed v
(m/s), driven tyre radius r and up-shift rpm UP:

- 0 if v < 0;
- otherwise the lowest gear k from 1 to N − 1 in which v is below the speed
  of UP − 500 rpm;
- otherwise N.

The moving start (§9.3) uses it, and never selects less than 1st.

**#59, driveline layout** (`model::awd_layout`). This gives the layout code
L, the driven axle, the coupling k and the centre-clutch capacity used by
the driveline (§2.7). B4 = drivetrain +0x13 and B5 = the AWD strength byte
(no database source known yet, so 0, which selects the defaults below).

| Drive / AWD system | L | Driven axle | k | Capacity |
|---|---|---|---|---|
| FR, MR, RR | 0 | rear | – | – |
| FF | 1 | front | – | – |
| 4WD, system 0 | 2 | front | B4 × 0.01 | – |
| 4WD, system 1 | 3 | both (mean) | B4 × 0.01 | – |
| 4WD, system 2 | 4 | rear | 1 if B4 ≥ 50, else B4 × 0.01 | (B5 or 8) × 98 N·m |
| 4WD, system 3 | 5 | rear | B4 × 0.01 × 0.36 | (B5 or 15) × 98 N·m |
| 4WD, system 4 | 6 | rear | 1 / (B4 degrees in radians), 0 if B4 = 0 | (B5 or 15) × 98 N·m |
| 4WD, system 7 | 7 | rear | B4 × 0.01 | – |
| 4WD, systems 5, 6 or unknown | 0 | rear | – | – |

On the test disc's 244 cars this gives L0 150 times, L1 29, L2 2, L3 8, L4 40,
L5 11 and L6 4.

## §8 Tyre wear, tyre smoke and brake heat

Tyre wear exists only in races that turn it on. Each wheel has a wear value
w (in wear units), a grip factor and a gauge level. In the same tick
(§1.2):

- **before the tyre forces** (phase 2, `Vehicle::Impl::wheel_motion`): the
  pit-surface flag (§8.6), then each wheel's wear grip factor (§8.4), which
  multiplies its peak force (§3.1);
- **after the driveline** and before the planar step
  (`Vehicle::Impl::tyre_wear_and_heat`): the slip intensity (§8.3) and the
  smoke emission (§8.3b) of each grounded wheel, then the wear increment
  (§8.2), the brake heat (§8.5) and the tyres-worn flag (§8.7).

With wear off, w stays 0 and the grip factor stays 1, but the slip
intensity, smoke emission and brake heat are still computed. §8.8 adds the
wheels' flap pendulums, another visual-only state. `reset()` and
`spawn()` keep the wear: a car put back on the track keeps its tyres.

### §8.1 Wear parameters of a race

`model::wear_params` builds the wear curve from seven event bytes,
0x1BD-0x1C3. Two more values are fixed: the first knee is always 4 and its
loss always 0.

| Byte | Parameter | Unit | Shipped value |
|---|---|---|---|
| 0x1BD | maximum wear w_max | × 10000 | 21 → 210000 |
| 0x1BE | grip loss at w_max | % | 30 → 0.30 |
| 0x1BF | "worn" margin; threshold = (100 − byte) / 100 | % | 9 → 0.91 |
| 0x1C0 | fresh set: w_fresh = −byte × 10000 | × 10000 | 2 → −20000 |
| 0x1C1 | grip loss of a fresh set | % | 6 → 0.06 |
| (fixed) | knee 1 | × 10000 | 4 → 40000 |
| (fixed) | loss at knee 1 | % | 0 |
| 0x1C2 | knee 2 | × 10000 | 15 → 150000 |
| 0x1C3 | loss at knee 2 | % | 5 → 0.05 |

The slip-intensity cap of §8.2 is 1. Wear is **off** when:

- the race does not enable it;
- the maximum byte is 0;
- or the ordering check fails: knee 2 < maximum, loss 2 < maximum loss,
  knee 1 < knee 2, loss 1 < loss 2, and margin < maximum loss must all hold.

The disc ships one curve, the values in the table (`shipped_wear_params`).

Where wear is on (`RaceScreen::build_vehicle_inner`, `src/race/field.cpp`):

- single-player races take the event's bytes, but never on dirt courses
  (course category 2). The setting applies to every car of the field;
- in the two-player battle, the "tyre damage" option uses the shipped curve,
  also never on dirt;
- time trials, licence tests and free runs have no wear.

### §8.2 Wear accumulation

For each wheel, each tick:

  Δw = min(cap, S) · k · max(0, 0.4·F_z / b − 1960) · Δt,

where:

- S = the slip intensity (§8.3);
- k = the compound's wear rate (§6.5; "normal" = 0.40);
- F_z = the tyre load in N;
- b = the axle's section width in metres.

The 1960 offset means a lightly loaded tyre does not wear even when it
slides. A wheel adds no wear if it has no load, stands on the kerb surface
(index 1), or the car is on the pit surface. Wear counts up from w_fresh
(−20000) and stops at w_max (§8.4).

Example: a "normal" 245 mm tyre at 6000 N in a full slide wears about 3134
units per second (`sim_wear_models`). A 3 s full-throttle launch of the
rear-driven test car takes its rear tyres from −20000 to −14219 and leaves
the fronts at −20000 (`sim_tire_wear_and_pit_stop`).

### §8.3 Slip intensity

The slip intensity S of a grounded wheel (`model::slip_intensity`) drives
the wear and the skid sound's volume (its pitch is the skid pitch below). It
is computed from:

- the wheel's lateral speed (across its heading, m/s);
- its slip ratio (§3.4);
- its rim speed |ω·r|;
- its load F_z;
- a speed ramp ρ = min(1, 0.02·|hub speed|).

**Kerbs** (surface 1 and the wet kerb 13):

  S = clamp((|ω·r| − 1.389)·0.144, 0, 1) · clamp(F_z / 2450 − 0.25, 0, 1) · 0.5.

**Every other surface** uses one of three curve classes. Class 1 is
surfaces 2-5 (grass, gravel, sand, dirt). Class 2 is surface 6 and the wet
surfaces 12-14. Class 0 is everything else. With a = A_c(|lateral speed|)
and b = B_c(slip ratio), both piecewise linear and clamped at the ends:

  m = √(a² + b²);  m ← m + (ρ − ρ₀)·g if ρ > ρ₀;  S = m + o if m ≠ 0, else 0.

| Class | A_c points (lateral m/s → value) | B_c points (slip ratio → value) | ρ₀ | g | o |
|---|---|---|---|---|---|
| 0 | 1.111→0, 9.556→1.192, 12.22→1.6, 15→2 | −1→2.388, −0.580→2.388, −0.314→1.176, −0.0353→0, 0.0353→0, 0.314→0.784, 0.580→1.592, 1→1.592 | 0.01667 | 0 | 0 |
| 1 | 1.556→0, 3.333→0.0627, 13.11→0.3639, 14.11→0.4 | −0.502→0.2, −0.439→0.1757, −0.1255→0.0251, −0.0471→0, 0.0471→0, 0.1098→0.0408, 0.2353→0.1757, 0.2667→0.2 | 0.005556 | 0.5455 | 0.07 |
| 2 | 1.556→0, 3.333→0.0471, 13.11→0.2729, 14.11→0.3 | −0.502→0.4, −0.439→0.3514, −0.1255→0.0502, −0.0471→0, 0.0471→0, 0.1098→0.0612, 0.2353→0.2635, 0.2667→0.3 | 0.05556 | 0.24 | 0 |

On tarmac, S reaches about 1 at 8.2 m/s of sideways speed, or at a slip
ratio of +0.385 (locking) or −0.272 (wheelspin). A tyre that grips stays at
0. An airborne wheel keeps its last S, skid pitch and smoke emission for 5
more ticks; after that S and the smoke emission are 0 and the skid pitch is 1.

**Skid pitch** (`model::skid_pitch`). A second curve set, computed beside S
from the same inputs, gives each grounded wheel's **skid pitch** P, the pitch
factor of its tyre sound loops (`VehicleTelemetry::skid_pitch`; the sound
document). The load does not enter.

- Kerbs (surfaces 1 and 13): P rises linearly with the rim speed |ω·r| from
  0.5 at 2.78 m/s (10 km/h) to 4.3 at 55.6 m/s (200 km/h), clamped outside.
- Every other surface, by the classes of S: P = o + √(a² + b²), plus
  (ρ − ρ₀)·g when ρ > ρ₀. Unlike S, the offset o is added even when the
  curves give 0.

| Class | A_c points (lateral m/s → value) | B_c points (slip ratio → value) | ρ₀ | g | o |
|---|---|---|---|---|---|
| 0 | 0.444→0, 3.333→0.1098, 6.0→0.2275, 9.222→0.3510 | −1→0.4612, −0.6588→0.2886, −0.2510→0.1129, −0.0157→0, 0→0, 0.2039→0.0878, 0.7216→0.3263, 1→0.4612 | 0.01667 | 0 | 0.75 |
| 1 | 1.111→0, 6.889→0.4706, 18.0→1.5373, 28.33→1.5922 | −0.3451→0.8, −0.2980→0.7027, −0.1098→0.1004, −0.0471→0, 0.0471→0, 0.0784→0.1631, 0.1882→0.7027, 0.2824→0.8 | 0.05556 | 0.2 | 0.5 |
| 2 | 1.556→0, 5.111→0.1506, 11.78→0.3012, 28.33→0.2933 | −0.502→0.3, −0.439→0.2635, −0.1255→0.0376, −0.0471→0, 0.0471→0, 0.1098→0.0612, 0.2353→0.2635, 0.2667→0.3 | 0.1667 | 0.3857 | 0.5 |

So a gripping tyre on tarmac idles at 0.75 (above ρ₀ the class-0 slope g is 0),
and the pitch rises with sliding (above 1 from about 9 m/s sideways); loose and
wet surfaces idle at 0.5 at rest. A placed car starts at P = 1 on every wheel.

### §8.3b Smoke emission

A third set of curves (`model::smoke_emission`) gives the **smoke-emission
intensity** E from 0 to 1. Tyre smoke, dust puffs and car dirt read E, not
S. The structure is the same as §8.3, except that:

- the second input is the signed slip speed sgn(hub)·(hub − rim) in m/s
  (positive = the wheel turns slower than the road, as when braking);
- kerbs have no special rule (surface 1 is class 0, 13 is class 2);
- the result is E = min(1, m + o).

| Class | A_c points (lateral m/s) | B_c points (slip speed m/s) | ρ₀ | g | o |
|---|---|---|---|---|---|
| 0 | 11.56→0, 12.89→0.047, 13.78→0.151, 16.33→0.178 | −10→0.1, −8.89→0.088, −8.0→0.0125, −7.56→0, 8.0→0, 8.44→0.0204, 9.33→0.088, 10.44→0.1 | 0 | 0 | 0.3 |
| 1 | 3.11→0, 6.67→0.078, 26.22→0.455, 28.33→0.5 | −7.11→0.5, −6.22→0.439, −1.78→0.063, −0.67→0, 0.67→0, 1.56→0.041, 3.33→0.176, 3.78→0.2 | 0.00556 | 0.727 | 0 |
| 2 | 5.33→0, 9.78→0.016, 26.22→0.091, 28.33→0.1 | −9.11→0.1, −7.56→0.088, −6.0→0.0125, −4.67→0, 3.11→0, 4.22→0.0204, 5.56→0.091, 7.78→0.1 | 0.0556 | 0.2 | 0.2 |

Tarmac therefore smokes only beyond about 7.6 m/s of wheelspin, 8 m/s of
lock-up or 11.6 m/s of sideways speed, and then at roughly 0.3-0.5. Loose
surfaces throw dust from low speeds up. Readers:

- the race's tyre smoke: a puff from each grounded wheel with E > 0.001
  (`src/race/tyre_smoke.h`, `RaceScreen::update_smoke`);
- the car's dirt level, which grows with the sum of E over wheels on loose
  surfaces (`RaceScreen::update_dirt`).

### §8.4 Grip factor, gauge level and a fresh set

`model::wear_grip` maps a wheel's wear w to a grip factor G and a gauge level
from −63 to 127. The curve is piecewise linear. Wear beyond w_max is clamped
back to w_max.

| Wear range | G | Gauge level |
|---|---|---|
| w < w_fresh | 1 | −63 |
| w_fresh ≤ w < 0 | 1 − loss_fresh · w / w_fresh | −trunc(63.9 · w / w_fresh) |
| 0 ≤ w < knee 1 | 1 − loss₁ · w / knee₁ | trunc(31 · w / knee₁) |
| knee 1 ≤ w < knee 2 | linear from 1 − loss₁ to 1 − loss₂ | 32 + trunc(32 f) |
| knee 2 ≤ w < w_max | linear from 1 − loss₂ to 1 − loss_max | 64 + trunc(63 f) |
| w ≥ w_max | 1 − loss_max | 127 |

Here f is the fraction of the way through the range. The factor never drops
below the record's floor min(1, 1.25 / μ) (§6.5). Compounds with μ ≤ 1.25
therefore never lose grip to wear, and a μ 1.45 tyre (the R34 GT-R's
"normal") stops at 0.862 instead of 0.70.

With the shipped curve:

| w | −20000 | −10000 | 0 | 40000 | 100000 | 150000 | 180000 | 210000 |
|---|---|---|---|---|---|---|---|---|
| G (floor 0.5) | 0.94 | 0.97 | 1.00 | 1.00 | 0.973 | 0.95 | 0.825 | 0.70 |
| level | −63 | −31 | 0 | 32 | 49 | 64 | 95 | 127 |

So a new set is slightly slow until it is scrubbed in (w reaches 0), is at
its best up to knee 1, then fades, slowly at first and then quickly. The
HUD's tyre gauge colours each tyre by its level (docs/dev/RACE.md).

The factor is computed and applied only while the car is **off** the pit
surface. On the pit surface the peak force is not reduced, and the reported
factor keeps its last value.

A **fresh set** (`Vehicle::fit_fresh_tires`) sets w = w_fresh, G = 1 −
loss_fresh (0.94 shipped), level −63 and brake heat 0. `set_tire_wear`
fits one when a race sets its wear parameters; a pit stop fits one (§8.6).

### §8.5 Brake heat

Brake-disc heat y, from 0 to 1, is visual only: it makes the brake discs
glow. For each wheel, each tick (`model::brake_heat_step`):

  y ← clamp(y + Δt·(T·p·|ω r| / 68600 − (|ω r| + 27.78)·0.005·(y + 0.05)), 0, 1),

where:

- T = the axle's maximum brake torque × tyre radius;
- p = the wheel's brake pressure after the aids and ABS (§4.5, §4.6);
- |ω r| = the rim speed.

Hard braking from speed heats the discs. The cooling term grows with speed
and never quite stops, so the discs cool off after braking.

### §8.6 Pit lane and pit stops

**Pit surface.** A wheel whose ground triangle has flag bit 0 set (bit 8 of
the raw surface value) is on the pit lane. If any wheel is, the car is "on
the pit surface" for that tick. The effects are:

- no wear accumulates (§8.2);
- the wear grip factor is not applied (§8.4);
- the race uses the flag to start the pit-lane sequence.

Car-to-car contact ignores the flag (§5.3).

**The stop** (`Vehicle::begin_pit_stop`):

1. All velocities and dynamic states are set to zero: body velocity, yaw,
   pitch and roll rates, wheel speeds and slips, engine speed (it rises back
   to idle on the next tick), differential and turbo states, steering, and
   held wall contacts.
2. A 300-tick counter starts (5 s). While it runs, the car is held in the
   scripted pose of §4.3: lifted to 0.3 m over the first 30 ticks, its
   heading easing towards the box's heading at up to 30°/s, and dropped
   back onto the ground for the last 30 ticks. The
   counter also holds 1st gear, keeps the clutch request at 0 (open) and
   zeroes the planar velocity and yaw rate.
3. When the counter reaches 60, about 4 s into the stop and 1 s before
   release, a fresh set is fitted if wear is on.

A hand-built car without the suspension block runs the counter without the
scripted pose. There is no fuel.

**Driver kinds.** The rest of the pit sequence is the race's
(`src/race/pit.h`, docs/dev/RACE.md "Pit stops and tyre wear"). The
computer drives the car through the lane and sets the vehicle's **driver
kind** (`Vehicle::set_driver_kind`):

| Kind | Meaning |
|---|---|
| 2 | along the pit lane at the AI set's pit speed + 2 km/h (80 + 2 km/h when the set gives none) |
| 3 | on the stop line at 20 km/h towards the box |
| 4 | leaving |
| 0 | back to normal |

While the driver kind is non-zero and no stop counter runs, the car
behaves like a CPU car:

- automatic gearbox;
- the AI's clutch request rule (§4.4 AI note), except in kind 8 (below);
- the AI aid block (§4.6): its TCS, and its ASCC channels where its flags
  enable them (the built-in sets: TCS gain 6.0, lateral a 1.0, b 0, slip
  scale 1.0, no ASCC);
- the alternative oversteer indicator (§4.6).

The other kinds belong to the run-up and the end of a run (§9.4):

| Kind | Meaning |
|---|---|
| 1 | the AI drives the car (run-up, driving on after the run) |
| 5, 6, 7 | the stop driver: no throttle, brakes on (kind 5 keeps its last brake), steering along the line (5, 6) or straight (7) |
| 8 | parked |

Kind 8 also applies the hold of §4.1 (as the race hold, §9.3): every tick
the planar velocity, the yaw rate and the longitudinal force sum are zeroed,
the vertical velocity is kept and the brakes are not applied. Kind 8 also
keeps the clutch request at 0 instead of running the AI clutch rule, for the
player's car and CPU cars alike, so the clutch opens and stays open (§2.4).
The vehicle treats kinds 1–7 alike; they differ only in what the computer
driver asks for. Replays store the player's kind per tick as 0, 1, 5 (for 5–7) or 8.

### §8.7 Tyres-worn flag

While wear is on, the telemetry flag `tires_worn` is set when some wheel has
w ≥ 0 and a grip factor below the "worn" threshold (0.91 with the shipped
curve). A CPU car with the flag set heads for the pits (`src/race/field.cpp`;
see the AI document). A tyre whose floor (§8.4) is above the threshold
never sets the flag.

### §8.8 Flap pendulums

Each wheel carries a pendulum x ∈ [−1, 1] with rate v. It is visual only: the car
model's mud flaps and aerials take the morph phase 0.5 + 0.5 x (`WheelState::flap`).
Each tick, after phase 12 (`model::flap_step`), with a_x the forward step's
acceleration (§4.1; computed even with all wheels airborne), u the hub speed along
the wheel heading, F_z the tyre load and w the rim speed:

- a = 5 a_x;
- one random draw r ∈ [0, 1): if r · room < 0.02, a += 5 u (1 − x), with room =
  1 − x for u ≥ 0, else 1 + x;
- on a surface whose tyre slot is not 0 (§3.6), loaded and rolling forward, a second
  draw: if r · (1 − x) < 0.04, a += 20 w (1 − x);
- a −= 100 x + 5 v; then v += a·dt and x += v·dt; at ±1 the pendulum stops (v = 0).

A steady a_x therefore holds x at 5 a_x / 100 (the flaps lean back under throttle and
forward under braking), and the kicks grow with speed and on rough ground. All cars
draw from one shared sequence, reseeded at every run start. Placing a car and a pit
stop reset its pendulums. R34 GT-R from rest at full throttle: up to 0.64 within 2 s
(`sim_database_r34_gtr`).

## §9 Race starts

How the cars are placed and started depends on the event's start type
(event +0x1F3) and initial speed (+0x198). The table lists what the race
asks the vehicle to do in each case (`RaceScreen::restart`,
`race::RaceField::create`; the event fields are in docs/dev/RACE.md, "Starts and
event fields").

| Start | Events | Placement (§9.2) | Vehicle call (§9.3) | Afterwards |
|---|---|---|---|---|
| standing | type 2 (every single race, 10 s hold), type 1 (licence tests from rest, 4 s), type 0 (demo / machine test, countdown byte +0x1F2) | grid slot, nose on the slot point | `reset` (at rest), race hold during the countdown | the player drives from GO |
| rolling | type 3 with speed 0 (GT endurance) | grid slot; nose on the slot point, except the player's car, whose origin is on it | `spawn_moving` at 80 km/h, every car | the computer drives the player's car for 300 frames (§9.4) |
| flying | type 3 with a speed (time attack, moving licence tests) | on racing line 6, the event's distance before the line | `spawn_moving` at the event's speed | the computer drives until the start line (§9.1, §9.4) |
| demo race | type 0 with a speed | grid slots, as for the rolling start | `spawn_moving` at that speed, every car, no countdown | – |

None of these starts uses the 120-tick spawn hold of §4.3. Replays store
the start speed and re-run the same start (`RaceScreen::start_playback`).

### §9.1 Flying starts

Licence tests and time trials whose event has an initial speed start
flying:

- the car starts at the event's speed (+0x198 km/h), `flying_m` metres
  before the start line (the s16 at +0x1E6, stored negative);
- the computer drives the player's car until its first crossing of the line
  (`RaceScreen::flying_runup`), and the pad is ignored until then;
- the lap clock starts at that crossing.

The run-up driver is the regular AI driver with the course's parameter set
for the car's driveline class (§7 #21), else the built-in set of the course's
category, without the event's overrides (AI document §9.1).

### §9.2 Placement

**Grid slots.** A course's grid slot marks the **nose** of the car, so the
car is moved back along its heading by its front extent measured from the
centre of gravity (`race::grid_back`):

  front extent from CoG = front extent − L·(w_f − 0.5),

where L is the wheelbase and w_f the front weight share. The extent is the
one the collision box uses (§6.2, §5.3). An opponent whose grid index n is
beyond the course's slot count is placed 12·n m before the line.

**Flying starts.** The car sits on the AI's racing line 6:

- at track distance (lap length − flying_m), i.e. that far before the line;
- facing the line point about 10 m further on;
- at the ground height under its own line point.

A course without line 6 uses grid slot 0 (`flying_pose` in the race screen,
`demo_flying_pose` in `tests/test_sim.cpp`).

### §9.3 The car's state at the start

**At rest** (`Vehicle::reset`). The car is placed with:

- its suspension at the static position x_s of each axle, and each wheel's
  load at its static corner load;
- every wheel stopped;
- the engine speed 0. The display shows idle, and the first tick lifts the
  engine to idle, because an open clutch floors the engine at idle (§2.4);
- 1st gear, clutch open;
- a shift lockout of 12 ticks: no gear change during the first 12 ticks.

The aids, turbo, differential, slipstream, smoke and brake-heat states are
cleared. So are the race hold, driver kind and reverse request. Tyre wear
and engine condition are kept. On the first tick the body is fitted to the
ground under the four hubs (pitch, roll and height), so a grid slot on a
slope or banking starts at rest.

**The race hold** (`Vehicle::set_race_hold`) keeps every car in place while
the start sequence counts down. Each tick it:

- zeroes the planar velocity, the yaw rate and the longitudinal force sum
  that feeds the load transfer (§4.2);
- keeps the vertical velocity, so the car settles on its springs, on slopes
  too;
- sets the player's clutch request to 0, so the clutch opens and the engine
  revs freely.

The brakes are **not** applied and the wheels stay free. Measured with the
R34 GT-R: with full throttle during the hold the engine runs on its rev
limiter (about 7900-8560 rpm) and the car does not move; one second after a
2 s hold is released it is at about 33 km/h in 1st gear. CPU
cars also hold their clutch request at 0 during the countdown and blip the
throttle (AI document §10). A car backend without the hold (the stand-ins)
is held with the brakes and handbrake instead.

**Moving** (`Vehicle::spawn_moving`, rolling and flying starts). This is
`reset` followed by:

- the velocity v along the car's heading;
- every wheel spinning at road speed (ω = v / r), and the axles with them;
- the gear from §7 #24 (at least 1st);
- the engine speed = clamp(v / r_d × overall ratio, idle, rev limit),
  truncated to whole rpm, with r_d the mean driven tyre radius;
- the clutch locked;
- the turbo lag states at that rpm (§2.3);
- no hold. The 12-tick shift lockout of `reset` still applies.

The R34 GT-R gets 1st gear at 3305 rpm at 30 km/h, 2nd at 5435 rpm at
80 km/h, and 3rd at 7277 rpm at 150 km/h.

### §9.4 Run-up, hand-over and the pit start

- **Rolling start:** the computer drives the player's car for 300 frames
  (5 s) from the moving start, then hands over at the START signal.
- **Flying start:** the computer drives until the first start-line crossing
  (§9.1).

The hand-over of the rolling start comes after exactly 300 ticks of race
time. During a run-up the vehicle's driver kind is 1, which puts the car on
the scripted path of §8.6:

- the automatic gearbox;
- the AI clutch rule;
- the AI aid block of the run-up driver's parameter set (§4.6).

**End of the run** (`ai::end_of_run`). Once the player's run is over, the
computer drives the car again (unless the pit lane is driving it), with the
AI's parameter set including the event's overrides; the vehicle keeps the aid
block of the run-up's set (without the overrides):

| The run ended with | Driver kind | Driving |
|---|---|---|
| a race finish, a time-trial finish, a machine test (at its goal distance; the top-speed test too) | 1 | on along the main line (6), at 0.8 × the corner speeds and a throttle factor of 0.5 |
| a licence test of kind 1 (stop) or 2 (section) | 6 | stops along the main line |
| any other finished licence test | 1 | as after a race |
| a failed licence test or time trial (course-out, wall) | 7 | stops with the wheels straight |
| the end of a free run on the pit surface | 7 | stops with the wheels straight |
| the end of a free run elsewhere | 1 | on along the main line, at full speed |

A stopping car is **parked** (kind 8, §8.6) once it stands still: 3-D speed
below 0.0694 m/s (0.25 km/h) and angular speed below 0.0524 rad/s (3°/s)
(`ai::parked`); the brake then comes off, the wheels return to straight, and
the clutch opens and stays open (§2.4) while the hold keeps the car in place.
CPU cars that finish drive on with kind 1 and the same slowdown. Measured
(`ai_stop_driver_parks`): kinds 6 and 7 park after about 1.7 s.

The 120-tick **spawn hold** (`Vehicle::spawn`, §4.3) is the free-practice pit
start. This engine also uses it for its own respawns, which the game does not
have (§11).

## §10 Ground frame, free wheels, engine condition, dirt and airborne cars

### §10.1 Speeds in the ground frame

The car's speeds are measured along the **ground**, not along the body. The
ground frame is the one §4.3 builds from the ground heights under the four
hubs, as left by the previous tick's suspension step. It has the car's
heading and the ground's pitch and roll. Its forward and lateral tangents
give:

  u = v · t_fwd,  l = v · t_lat,

where v is the body's world velocity. Because of this, a vertical velocity
contributes along the slope of the road, not along the body's pitch. u is
the forward speed that these read:

- aerodynamics;
- the steering curve;
- the clutch and shift rules;
- the aids;
- the engine-condition counters;
- the planar forward step (§4.1).

Each wheel's **hub velocity** uses only the yaw rate r, through fixed
levers:

  u_i = u + X_i·r,  l_i = l − Y_i·r,

with:

- X = (−t_f/2, +t_f/2, −t_r/2, +t_r/2) for FL, FR, RL, RR, where t_f and t_r
  are the front and rear track widths;
- Y = (a, a, −b, −b), where a = L·(1 − w_f) is the distance from the centre
  of gravity to the front axle and b = L·w_f the distance to the rear axle.

Pitch and roll rates and the individual triangle normals do not enter. For
a wheel turned by δ (positive to the right, with l positive to the right),
the speed along its heading is u_i cos δ + l_i sin δ and the speed across it
is −u_i sin δ + l_i cos δ. The telemetry
reports u and l at the end of the tick (`ground_u`, `ground_l`).

### §10.2 Free-rolling wheels

At the end of each tick, a wheel that is **free** is locked to the ground
(flagged in `Vehicle::Impl::flag_free_wheels` after the driveline step, locked in
`lock_free_wheels`). A free wheel:

- is grounded (its tyre load is non-zero);
- has no brake torque at all;
- is neither driven nor coupled this tick.

Its spin is set so that its rim speed equals its hub speed along its
heading. This uses the end-of-tick hub velocity of §10.1 and the same
expression the next tick's slip ratio uses, so the next slip ratio is
exactly 0.

A wheel counts as driven or coupled while the clutch is not open and its
axle carries drive in the layout (§7 #59):

- rear for L0 and L4-L6;
- front for L1;
- both axles for L2, L3 and L7;

and, for L4-L7, the front axle also whenever the centre clutch is engaged or
carries torque, even with the clutch open.

Why: an undriven, unbraked wheel has nothing to make it slip, and locking it
removes small spurious slip forces. Measured: a coasting front wheel of a
rear-driven car (S2000 under part throttle) shows a slip ratio of exactly 0.
Hand-built cars without the suspension block apply the same lock at the
start of the next tick instead (`placeholder_wheel_velocity`).

### §10.3 Engine condition

Two distance counters in metres describe the engine's condition:

- **E**, running-in and wear;
- **O**, the oil.

They scale the full-throttle torque T_wot (not the engine friction) by
f = f_E(E)·f_O(O) (`model::engine_condition_factor`):

| E | f_E | | O | f_O |
|---|---|---|---|---|
| < −300 km | 0.97 | | < −100 km | 1.05 |
| −300 km … 0 | 1 − 0.03·E / (−300 km) | | −100 km … 0 | 1 + 0.05·O / (−100 km) |
| 0 … 500 km | 1 | | ≥ 0 | 1 |
| 500 … 600 km | 1 − 0.02·(E − 500 km) / 100 km | | | |
| ≥ 600 km | 0.98 | | | |

Driving d metres updates the counters (`model::engine_condition_step`):

- E ← min(E + d, 600 km);
- if O < 0: O ← O + d, and when it reaches 0 it jumps to 300 km;
- if O ≥ 0: O ← min(O + d, 300 km).

Each tick, after the driveline step (§1.2), the vehicle advances E and O by
|u|·Δt, so reversing counts too, and the full-throttle torque of every tick
uses the factor of the current counters (`Vehicle::Impl::engine_and_clutch`).

**Odometer.** The vehicle also keeps an odometer word in 2 m units
(`model::odometer_step`). Each tick with u ≥ 0 (not while reversing) it adds
x = carry + u·Δt metres: the odometer gains trunc(trunc(x) / 2) units and the
rest, x − 2 × gained, is carried to the next tick (an odd metre waits for the
next one). The odometer stops at 0xEE6B27A6 units and the carry is then 0.
The race HUD shows it and lights the oil lamp once O ≥ 300 km, the worn value
(`model::oil_worn`).

The resulting behaviour:

- a new engine (E = −300 km) gives 97 % and frees up over its first 300 km;
- it gives full power until 500 km, then loses 2 % by 600 km;
- fresh oil (O = −300 km after an oil change) gives up to 5 % extra, which
  fades over its last 100 km. After that the oil counts as used (O jumps to
  300 km) and gives no bonus.

**Through a run.** A car's engine condition is the triple odometer, E, O
(`sim::EngineCondition`; all zero is a fresh engine with f = 1). GT mode
stores it per garage car: a new car starts at E = −300 km, O = 0, and an oil
change sets O = −300 km. The car's torque curve itself is built without the
factor (`GtMode::car_spec`); the factor acts only inside the vehicle.

- **Which runs carry it** (`GtMode::carries_engine_condition`): GT-mode races
  and championships, machine tests and free runs, and arcade races and free
  runs. Such a run of a garage car starts with that car's values
  (`GtMode::run_condition`, the odometer in 2 m units); in these modes a car
  that is not a garage car starts with zeros. Licence tests, time trials and
  the two-player battle always start with a fresh engine and hand nothing back.
- **During the run** the race seeds the player's car with the values
  (`Vehicle::set_engine_condition`, which also clears the odometer carry) at
  every start and restart. A restart or retry starts from the values the
  previous live run left, so every live run of the session keeps advancing
  them.
- **After the run** the values the last live run left go back to the garage
  car (`GtMode::end_run`), and the total distance grows by 2 m for each
  odometer unit the runs added. Replays do not advance them.
- **Replays** store the values their run started with (replay version 8; older
  replays: a fresh engine), and re-simulate (playback, ghosts, the analyser)
  from them.

Opponents and the second player always have f = 1.

Measured (`sim_spec_engine_condition`, the invented spec car): at the same state a
worn engine (E = 600 km) gives 259.5 N·m against 264.8 N·m fresh; from zeros, 169.5 m
driven advance E and O by 169.3 m and the odometer by 84 units.

### §10.4 Dirt courses: front brake cut when steering

On a course whose category byte (course table +0x08) is 2 (dirt), every car
of the race has its **front** brake pressure scaled before the aids and ABS
(`model::dirt_front_brake_scale`):

  p_front ← p_front · (1 − min(1, |δ| / 15°)),

where δ is the steering angle (the front road-wheel angle before Ackermann
and toe, §4.4). The rear brakes are unchanged.
Braking while turning therefore loads the rear brakes only, which helps the
car rotate on loose ground; with 15° of lock or more the fronts do not brake
at all. Measured with the R34 GT-R at half brake (pressure 0.55 after pedal
shaping, §4.4): with 16.8° of lock the front pressure is 0 on a dirt course
and 0.55 on tarmac.

Dirt courses also have no tyre wear (§8.1).

### §10.5 A wheel in the air

A wheel is airborne when its suspension coordinate x lies beyond the
maximum droop (x > droop limit, §4.2; `model::suspension_wheel`). Then:

- the stored coordinate (the next tick's x_prev) is the droop limit, not
  the raw value, so the damper rate starts from the limit when the wheel
  lands;
- the spring force on the body is −preload: the wheel's unsprung weight
  hangs on the body;
- damper force, anti-roll force and tyre load are 0;
- the wheel has no tyre forces, and its published contact point is put on the
  last ground height under it (its shadow, §1.3). `WheelState::suspension`
  reports the clamped coordinate, so an airborne wheel is drawn at full droop.

A ground query that misses the course (off the mesh) does **not** make a
wheel airborne: it keeps the previous ground height and surface (§1.3).

### §10.6 All four wheels in the air

If all four wheels were airborne after the previous tick, the planar step
(§4.1) is skipped. No tyre, engine, brake, rolling-resistance or drag force
acts in the plane, and the yaw rate is left unchanged; the race hold, kind 8 and
the stop clamp (§4.1) do not act either, only the spawn/stop counter. Heave, gravity and
the attitude step (§4.3) still run, so the car falls and the airborne
wheels' preload still acts on the body.

The only horizontal effect left is the heave force acting along the tilted
body axis, so the horizontal speed changes only slightly while the car is in
the air.

## §11 Leaving the road

**No hull contact.** For cars with the suspension block, which is every disc
car, the body never touches the ground. The ground acts only through:

- the four wheel probes;
- the bump stops (§4.2);
- the bump-stop attitude nudge (§4.3).

A car that bottoms out rides on its bump stops. Setting `GT3R_HULL_CONTACT=1`
turns on a **[placeholder]** penalty contact, for comparison only. It
always applies to hand-built cars without the suspension block. It uses
eight hull points at 0.15 m and at 0.95 × height, at ±0.47 of the width and
length, with:

- a spring of 80·m N/m;
- 0.7 of critical damping;
- friction of at most 0.6 × the normal force.

**Course-out** (`race::course_out`). In time trials and licence tests only,
the run fails when:

- all four wheels are grounded on materials 2, 3, 4 or 6 (grass, gravel,
  sand, and surface 6); or
- a wall hit changes the car's speed by more than 10 m/s in one step.

Hits on other cars never count. A failed time trial records no lap, shows
FAIL and brakes the car to a stop (`RaceScreen::tick`). A licence test fails
through its judge (docs/dev/RACE.md, "Licence tests"). In a time trial,
touching a pylon also ends the run. Races, machine tests and free runs have no
course-out.

**No respawn in the game.** The game never puts a car back on the track, and
the physics never does so by itself. The race layer's respawns are this
engine's own fail-safe (docs/dev/RACE.md, "Course-out and respawns"; `[race]
respawn = off | cpu | all`, default `cpu`):

- **CPU cars** (`ai::SafetyRespawn`, one rule for `race::RaceField` and
  `ai::AiRace`; AI document §12). A car is *lost* when its centre and all four
  wheel contact points have been off the road mesh for more than 30 ticks (no
  road below a point 3 m above them, or the road more than 5 m above them),
  when its position is not finite, or, after GO, when its race distance has not
  grown by 10 m for 12 s. Time in the pits does not count, and the 12 s count
  from GO. A lost car is put on the course's centre line at its lap distance,
  facing along the lap, with the spawn hold (§4.3; `ai::respawn_car`); its lap
  timer moves with it and its driver is reset. `off` disables it.
- **The player's car**: falling out of the world (a non-finite position, or
  a height below −1050 m) always respawns it; with `all` (never online) it is
  also respawned after 8 s with no road under its centre.
  The player can respawn by hand (Select). Each player respawn uses the spawn
  hold at the track's respawn pose nearest the car and is recorded in the
  replay.

## Verification

Run `TMPDIR=$PWD/build/tmp GT3R_DISC=disc/GT3.iso ./build/gt3r_tests sim_` to run
the simulation tests with the disc; they print the measured values quoted above.
Selected results (this build):

| Test | Result |
|---|---|
| `sim_database_r34_gtr` | R34 GT-R: 0–100 km/h 5.03 s, 400 m 13.50 s, top 265 km/h |
| `sim_r34_skidpad_brake` | R34: constant-steer maximum 0.73 g at 63 km/h, 0–200 20.85 s, 100–0 23.5 m |
| `sim_surface_grip_scales_braking` | 80–0 km/h: tarmac 32.7 m, wet tarmac 43.1 m, grass 53.7 m |
| `sim_drives_trial_mountain` | 1532 m along Trial Mountain in 60 s, wheels grounded 100 % |
| `sim_license_demos` | licence demo B-1 finishes in 34.93 s (recorded 34.94 s) |
| `sim_database_all_cars_build_and_run` | 244 of 244 cars build and drive |

The spec body can also be tested without the disc: `tests/test_sim_spec_path.cpp`
(`sim_spec_*`) drives an invented car with both suspension blocks
(`synthetic_spec_car` in `tests/sim_fixtures.h`), so `Vehicle::tick` takes the
same path as every disc car. These tests check behaviour properties (rest loads,
launch and up-shifts, braking, cornering, jumps, ground misses, walls, surfaces,
slipstream, dirt brake cut, engine condition, reverse, spawn hold and pit stop,
race hold, aids, determinism), and golden traces (`sim_spec_golden_*`; one of them
uses the placeholder body) pin the tick's output for scripted inputs. Disc tests
such as `sim_spawn_hold` and `sim_ground_miss_keeps_previous_height` have
disc-free twins there.

## Open points

- The AWD strength byte B5 has no known source; the L4–L6 centre-clutch
  capacities use the defaults (§2.5).
- Interpretations: the automatic's reverse pedal swap (§2.6), the slip sign as
  force direction (§3.4), sin δ in the understeer target (§4.6), the torque-vectoring
  steer sign (§2.7), the centre clutch engaged at rest (§2.7), the stop-clamp
  condition (§4.1), the stuck-nudge order (§5.3) and the HUD boost value (§2.3).
- Wall contact is approximate: wall corners against car edges are covered only by
  the push-out, and deep penetrations are pushed out perpendicular to the wall.
