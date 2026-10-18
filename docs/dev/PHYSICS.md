# Vehicle physics (src/sim/)

This is the driving simulation. It follows a written description of the
original's vehicle behaviour (the "spec": its state, step order, tyre,
suspension, driveline, aids and car-parameter rules). Every sub-model is a
pure function in `src/sim/models.h`, tagged **[spec §n]** (follows section n
of that description) or **[placeholder]** (a conventional model where the
behaviour is not pinned down yet), so each one can be checked on its own
against the original's observable behaviour and swapped independently.
Section (§n) and draft numbers below refer to that description.

| File | Role |
|---|---|
| `sim/sim_types.h` | shared contract between sim, track and race |
| `sim/vehicle.h/.cpp` | `Vehicle`: state, tick structure, integration |
| `sim/car_spec.h/.cpp` | `CarSpec` (SI parameters) and `build_car_spec()` from `data::CarDatabase` |
| `sim/models.h/.cpp` | pure sub-models |
| `sim/track_walls.h` | `TrackWorld`: `track::Track` walls as `sim::WorldQuery` |
| `app/screen_sim.cpp` | `gt3r --screen sim` ("Physics test"): flat plane plus telemetry |
| `tests/test_sim.cpp` | unit, spec-function, behaviour, determinism and real-data tests |

## API

```cpp
auto car = sim::Vehicle::create(db, "r34_gtr_v_99"); // car (table 30) or variant (31/35) label
car->reset(pose.position, pose.orientation);        // origin = wheelbase midpoint on the ground
sim::TrackWorld world(*track);                      // optional: walls
car->step(inputs, query, 1.0 / 60, &world);         // one tick per 1/60 s
car->set_slipstream(sum_of_model_slipstream_factor); // optional, spec §5.2
car->state(); car->setup(); car->telemetry();
```

`step(dt)` runs `ceil(dt·60)` ticks. There are no sub-steps (spec §1.1). It is
deterministic: the same inputs give bit-identical states. The first tick after
`reset()` fits the body to the ground under the four hubs (pitch, roll and
height), so grid poses on slopes and banking start at rest.

Conventions:
- `WheelState::slip_ratio` follows spec §3.4: > 0 means braking, < 0 means
  wheelspin.
- `slip_angle` = atan2(−vy, vx) of the contact patch.

## User settings (garage tuning)

`sim::CarSettings` holds the spec §6.6 settings (the garage car record's settings words +0xF0..0x1C8) as raw values:
- gear words, final drive, computed-gearbox slider, drivetrain split;
- brake-controller indices, downforce front/rear, turbo bytes;
- camber, ride height, toe, spring, motion ratio, damper indices, anti-roll indices;
- LSD settings, TCS and ASCC indices.

`stock_settings(db, car, parts)` gives the defaults. `build_car_spec(db, car, parts, settings)` applies them through the spec's conversions:
- springs: b0·980·c/100;
- dampers and anti-roll bars by index interpolation;
- downforce: A_f·%;
- final drive: ×0.001;
- LSD initial/accel/decel;
- TCS level, ASCC gains, ABS gain front/rear;
- computed gearboxes regenerated from the slider.

The overloads without settings use the stock settings, so `phys_car_spec` is unchanged.

`apply_gear_slider()` regenerates a computed gearbox's gear words from the slider.
`stock_settings` uses it for the default words, and the converter runs it again
from the given slider and final drive, as the original does.

`front_torque` (settings word +0x118) is stored but not read: the game takes
the 4WD share B4 from drivetrain +0x13 instead (spec §2.7, §6.6).

Verified against the original's car parameters with randomised settings:
0 mismatches.

## Multi-car world

`sim/world.h` (`sim::World`) steps several cars in the spec's phase-major order:
1. slipstream factors (§5.2) for every car;
2. each car's tick (`Vehicle::step_tick`), with a per-car `time_scale` for
   dt_car (§1.1, used by the rubber band);
3. car-to-car contact (§5.3): restitution 0.5, no friction, overlap split half
   and half.

`contacts()` reports each contact. A single car stepped through World behaves
like `Vehicle::step`.

## What follows the spec

| Area | Spec | Implementation |
|---|---|---|
| Tick | §1.1 | dt = 1/60 (dt_car = scale/60 in World), one tick per frame, semi-implicit Euler |
| State | §1.3, §4.1, §4.3 | Planar motion (horizontal velocity, heading, yaw rate) plus heave/pitch/roll. Euler build R = Rz·Ry·Rx (spec §1.3; a = pitch, b = roll per draft 14). Yaw inertia = chassis +0x1C·parts·100 (R34 9500); pitch I/2π, roll I/4π. |
| Torque curve | §2.1 | `build_torque_curve`, including the turbo min(B, cap)/B scaling. Diff-verified on all NA and turbo cars. |
| Engine torque | §2.2 | x'·T_wot − (1−x')·k_f·ω, k_f = drivetrain +0x16·0.98/ω_last. Fuel cut with 500 rpm hysteresis. Diff-verified. |
| Turbo | §2.3 (draft 14c) | `turbo_step`: two stages (turbo +0x12..+0x17, cap +0x11), integer lagged rpm, previous load, x' = min(1, load/Bcap(n)). |
| Clutch | §2.4 (draft 18) | States open / locked / slipping / shift (3). Request rule: 0 during the spawn/stop counter or with the handbrake on a rear-driven car; engaged: 1 unless rpm < idle; open: 1 while brake < 0.25. Locked → open only through the request. 0 → 2 on throttle > 0.25 when moving > 10 km/h, else above the launch rpm max(engine +0x43·10, idle + 500). 2/3 → 1 when the speeds cross (then a 12-tick shift lockout). 2 → 0 with brake > 0.75 and (anti-stall or < 10 km/h); 3 → 0 at wheel side ≤ idle. A forward gear change with the clutch engaged → state 3 (the fixed ± shift-clutch torque byte ·9.8, which is 0; throttle cut or blipped); to/from reverse → 2. No rev cap while open. Engine inertia from drivetrain +0x20. T_wot, friction and the limiter read the stored engine speed: for a locked clutch the end-of-tick value capped at rev limit + 10 rpm, not the wheel side re-read at the start of the tick. |
| Shifts | §2.6 | Instant, 1-step lockout, UP / down table, traction veto (tyre drive optimum), over-rev. Rolling back → reverse only with zero throttle. Pedal pressed below +1 km/h → first gear (automatic). In automatic reverse the brake pedal drives (interpretation). Diff-verified (`model_shift_logic`, `model_shift_manual`). |
| 4WD | §2.7 / §7 #21 | Driveline class: FF → 1, AWD system 0 → 1 (front drive), system 3 → fixed 50:50, others → rear = k·(ω_f − ω_r) with k = B4 × 0.01 (the 4WD share, §2.7). |
| Driveline and differentials | §2.7 (draft 18) | Axle speeds: layout L from `awd_layout` (§2.5: FR/MR/RR 0, FF 1, 4WD systems 0–4 and 7 → L2–L7, 5/6 → 0), inertias J = 0.01·(front/rear axle inertia)·final + 2·0.01·(front/rear wheel inertia) (derived from drivetrain +0x18..+0x1E with the flywheel/clutch/prop percentages, §2.7), reflected engine inertia J_e·\|R_g\| only while locked. Wheel net torque −r·Fx − sgn(w)·brake. Per layout: rear/front drive, L2 coupling k(ω_f − ω_r), L3 mean + half-difference with k, L4–7 front torque (`awd_front_torque`, L5 gain `awd5_gain` on the lateral g / mu value) through the centre clutch (`awd_centre_clutch`, engaged flag, common-speed lock, crossing snap). Zero-crossing clamp only with brake torque; 'f' axles not integrated. Wheels: `diff_step_prev` (f: wheels alone with the braked clamp; r; m/h lock flag; v gain accel·0.98, cap initial²; open), torque vectoring for y/t (`torque_vectoring`). Locked engine speed from the driven axle (L3 mean, L7 ω_r + k(ω_f − ω_r)). The type byte is the ASCII letter; type = its index in "frnmhvyt". The differential step runs on peripheral speeds (m/s) with r/Ia, as in the game (the viscous Δw² is in (m/s)², §2.7). |
| Computed gearboxes | §6.7 (draft 15b) | `generate_gear_ratios`: slider × 10 km/h design speed, step table, driven tyre front only for FF and AWD system 0. PS2 float emulation: mul/add/sub chop with the add alignment quirk, divisions round to nearest-even, exact constant words, saturating truncation. Bit-exact: all 234 real cars, physics-spec's random set (0 / 10,564 words) and `model_gear_generator`. |
| Turbo HUD | §2.3 | `telemetry().boost` = Bcap(lagged stages) − 1, `setup().has_turbo` / `max_boost` (gauge reading is an interpretation) |
| CPU steering | AI spec | `ControlInputs::steer_angle` (when finite) sets the road-wheel angle directly, bypassing the pad follower and the car's lock (the AI clamps itself at 45°) |
| TCS / ASCC | §4.6 (draft 18) | TCS throttle × clamp(1 + gain·g·(s_min − s*·lat·scale)), with gain/a/b/scale from the TCSC part; the lateral input is the lateral g / mu value \|4·a_y / Σ(9.8·mu_i[s_i])\| (lateral g over the mean surface mu). ASCC channels A/C/D/E: gains, thresholds and wheel flags from the ASCC part; under/oversteer indicators; the side follows the yaw-rate sign (> 0 = left turn, 0 = right). |
| Body attitude | §4.3 (draft 16) | `pitch_terms` (A/B), `attitude_step`: pitch/roll from body forces and dampers with lever factors a·2π/Y and half-track·2π/Z; rates clamped to ±π/2 and angles to ±π/4 (rate zeroed); heave along the body up axis over the sprung mass, then gravity in world Y. `bump_nudge` with the downward-speed limit. Downforce subtracted half per wheel from body forces. The A/B terms take the drive + brake force parts G only (§3.8). |
| Tyre wear, smoke, brake heat | §8.1–8.5, 8.7 (draft 17) | `model::wear_params` (event bytes 0x1BD..0x1C3, the one shipped curve `shipped_wear_params()`), `wear_grip` (curve, floor min(1, 1.25/mu), gauge level), `slip_intensity` (kerb / per-class curves), `wear_increment`, `brake_heat_step`. `Vehicle::set_tire_wear()` turns wear on and fits a fresh set (-20000, grip 0.94); `fit_fresh_tires()`. Per tick: pit-surface flag (triangle flag bit 0), wear grip on the peak force (not on the pit surface), smoke (airborne wheels keep it 5 steps), wear (not on the pit surface or kerb 1), brake heat, tyres-worn flag. Telemetry `tire_wear`, `wear_grip`, `wear_level`, `smoke`, `brake_heat`, `tires_worn`, `on_pit_surface`. The compound rate is `TireSpec::wear_rate` (compound byte 0x52 / 100). reset()/spawn() keep the wear. |
| Pit stop | §8.6 | `Vehicle::begin_pit_stop(box orientation)`: velocities and dynamic states zeroed, the 300-step hold of §4.3 easing to the box heading, fresh tyres at counter 60. The pit-lane state machine (lines 3/4, auto drive, the 25 m decision, markers) belongs to race/AI. Car–car contact ignores the pit-surface bit, as the original's car contact does (the bit filters only the race proximity classifier, §8.6 draft 24a). No fuel. |
| Spawn hold / ground frame | §4.3 (draft 16) | `Vehicle::spawn()`: 120-tick counter; the car is held at ground + `spawn_height_offset(n)` (0.3 m, 0 for the last 30), flat, gear 1 with the clutch open, heading easing to the grid heading at ≤ 30°/s. `reset()` is the §9.3 standing start (gear 1, clutch open, shift lockout 12); `set_race_hold()` is the countdown race hold (§9.3; world x/y velocity, yaw rate and the longitudinal force sum zeroed each tick, clutch request 0, brakes off; the race scene and `race::Field` use it for every car); the 120-step `spawn()` is only for the free-practice pit start (§9.4). `spawn_moving(pos, quat, v)` is the §9.3 moving start: velocity along the heading, wheels and axles at road speed, gear from the §7 #24 rule (at least 1st), engine = clamp(v·ratio, idle, rev limit) with the rpm truncated, clutch locked, turbo lag states at that rpm, no hold. Telemetry also exposes `slipstream` (§5.2), `turbo_load` and `turbo_lag` (§2.3). `ground_frame` (§4.3): height, pitch and roll from the four hub ground heights. Terrain factor s = ground up.y / body up.y scales the suspension coordinate. A probe miss keeps the previous ground height. |
| Camber / toe / Ackermann | §3.1, §4.4 (draft 16) | Cam(α), α = σ·(roll − ground roll) − setting·0.1°, σ = −1 left, +1 right; axle factor (byte + 900)·0.001; toe atan2((byte − 128)·0.0005, r), toe-in; Ackermann ratios atan2(L, L ∓ tf)/(π/4) |
| CPU cars | §4.4 AI note | `Vehicle::set_cpu_driver`: binary clutch request (from open, engage at rpm > 0.75·UP; stay engaged at rpm ≥ idle), TCS/ASCC off (AI aid block all zero). `set_reverse_request` for the automatic |
| Load transfer | §4.2 | Explicit only, in phase 12 after the lateral step: per-wheel body force += (lateral transfer coefficient)·(−a_y) (left −, right +) and (longitudinal transfer coefficient)·(longitudinal force sum) (front +, rear −), the full value on each grounded wheel, with this tick's a_y (lateral step output) and longitudinal force sum. Tyre forces drive planar motion and yaw, not pitch/roll; the tyre phase reads the previous phase-12 loads. |
| Tyres | §3, §6.5 | Real per-car records from compound (27) + force-vol (26): curves A, B, C, D (signed 11-point slip axis), L (Fz/width), per-surface mu[s], optimum slips. Three records per axle, chosen by the surface class. `tire_force` diff-verified (§3.3); all curves match the original's tyre data on 244 cars (§6.5). |
| Slip | §3.2, §3.4 | patch slip angle with the 1–10 km/h fade; draft-12 slip ratio (diff-verified) |
| Rolling resistance / surfaces | §3.5, §3.6 | RR(v)·ΣFz·rr[s] with static hold; surface index, bump wave |
| Suspension | §4.2, §6.6 | Per axle, from stock settings: k = b0·980·c/100, sqrt-law damper fitted through two points per direction, progressive k₂ below the knee, bump-stop clamp, max droop, preload = unsprung weight, quadratic anti-roll bar. Tyre load Fz = max(0, Fs + Fd + preload). R34: k 37240 / 46550, ARB 4900 / 3430. Static x_s = −½·share·(total − 2 unsprung)·g/k, ride height q = the front ride height on both axles (`model::static_position`, `suspension_limits`; static position and limits exact on all cars, `phys_car_spec`). Wheel query points are the hub offsets in the CoG plane rotated by the attitude (no hub height), x = x_s − q − (ground + cg_height − q − hub y)·s. |
| Steering | §4.4 | pad curve B, second-order follower, lock = steer +0x11; the bound on the side of the mean front velocity-direction angle φ (§3.2) is extended to it, the other stays (diff `model_steer_target_extension`) |
| Brakes | §4.5 | brake bytes ·98 N·m, handbrake +0x13·98, always-on ABS pressure limit (non-positive factor → 0.001), gain = the brake controller byte·0.1 (§4.5, §6.2), optimum = tyre brake optimum. Diff-verified (`model_abs_pressure`). |
| Aero | §4.7, §5.2, §6.6 | ρ 1.2, area body +0x44, Cd body +0x39, stock downforce (body +0x13 / +0x3C), drag = A_f·Cd + 0.1(C_f + C_r) (R34 0.511 / 0.348 / 0.564); slipstream filter |
| Stop clamp | §4.1 (draft 14b) | below 1 km/h, all fade weights 0 and some brake torque |
| Walls | §5.3 | One-sided walls: track on the LEFT of each edge; closed loops include the closing edge; sensor edges (kWallSensor, none on the US disc) only report. Octagon body (front/rear extents, width, 0.2·width chamfer), swept against wall segments (`WorldQuery::wall_segments`; TrackWorld supplies `track::Track::wall_lines()` with flags and closure), 2 iterations, impulse e = 0.05, μ = 0.5 without torque, collision inertia 10·I, push-out, stuck assist, yaw clamp ±20π. Persistent wall contacts (`collide::WallContacts`, §5.3 re-projection pass): a point flagged by an impulse or a push-out of ≤ 0.5 m is held on the wall line each round while the car presses in (velocity projected on the last hit wall's tangent, the angular part of that impulse at the point, no strength), skipped by the sweep and push-out, and dropped when the centre velocity points away or the point is on the line (within 1e-5 m) or back on the track side; a car still pressing in then takes fresh impulses with friction each crossing, as the original does. Diff `race_wall_hits` (trialmt, 100/180 km/h, 5/12/30°): first hit identical; speeds after 0.5 s within ~1-4 km/h on the right-hand walls, more contact than the original on the left-hand spot and in some 30° hits at 100 km/h (chaotic on both sides) |
| Car-car | §5.3 | `collide_pair_swept`, the spec's step structure: at most 2 rounds of static separation along the line of centres (both orderings, half each), persistence of flagged vertices (half/half push onto the edge line and half/half removal of the approaching normal velocity), end pose = start + v dt (1 - t), analytic pair sweep (each vertex on a straight line in the other body's frame against its edges, both orderings, ties to body 0, flagged and on-line vertices skipped), advance to the hit, impulse e = 0.5 at the crossing vertex with the hit edge's normal (not clamped, no friction), touch-up push, vertex flag. The bodies end at the last end pose computed. Only in `sim::World` with car collisions on. Diff: contact, time, point, normal, impulse and end state match the original within float noise (`model_collide_pair_swept`). |
| Car spec | §6 | mass +120 kg, extents/width/height (§6.2), TCS on iff the TCS level ∉ {0, 100}. All fields except computed gearboxes match the original's car specs on 244 cars (diff `car_spec`). |

## Still placeholder or approximated

- **Load transfer:** the explicit terms use the last tick's accelerations.
- **Cars without a spec suspension block** (hand-built specs such as the test car; every
  disc car has one) use a placeholder rigid body (start-of-tick suspension, velocity-capped
  tyre forces, forces applied at the contact points) and the placeholder wheel inertia.
  There is no switch: the spec planar step and the spec driveline are the only path for
  cars with the block.
- **Low-speed stability:** the tyre force is capped at what stops the
  predicted wheel/road relative motion in one tick (see Fidelity status 1).
- **Hull/ground contact:** penalty force.

## Verification status of the models

Verified against the original's behaviour:
- `table_lookup`, `euler_axes`, `patch_slip_angle`, `slipstream_factor`, `aero_step`, `surface_index`
- `build_torque_curve` / `build_car_spec` (all 244 cars)
- `slip_ratio`, `engine_torque`, `tire_force`, `road_bump`

Ready for comparison:
- `turbo_stage`, `turbo_boost_raw`, `turbo_boost`, `turbo_step` (§2.3)
- `clutch_slip_torque` (#10)
- `shift_logic` (§2.6)
- `fit_damper`, `damper_force` (#17), `suspension_wheel`, `anti_roll` (§4.2)
- `abs_pressure` (§4.5)
- `steer_target`, `steer_follow` (§4.4)
- `driveline_class` (#21)
- `rolling_curve`, `surface_rolling`
- `collide::octagon` (#26), `collide::wall_impulse` (#27), `collide::world_point` (#29), `collide::collide_walls`, `collide::collide_pair`

## Measured behaviour (flat ground unless noted)

| Case | First pass (6-DOF, 16 sub-steps) | Spec tick (draft 13) | Draft 14c | Now (draft 18) |
|---|---|---|---|---|
| R34 GT-R 0–100 km/h / 400 m / top speed | 5.00 s / 13.25 s / 277 km/h | 5.05 / 13.30 / 274 | 4.98 / 13.27 / 262 (real tyres, turbo lag, downforce drag, 50:50 4WD, real diffs) | 5.77 / 13.43 / 259 (shift clutch, ATTESA L5 coupling, axle inertias) |
| R34 steady right-hand corner, 0.51 g | – | – | left/right tyre load 8981 / 7934 N (explicit transfer) | – |
| Test car (4WD) 0–100 / 400 m | 5.17 / 13.43 s | 5.48 / 13.68 | 5.45 / 13.65 | 6.70 / 14.83 |
| Test car braking 100–0 km/h | 41.9 m | 42.8 m | 42.8 m | 50.7 m |
| Braking 80–0: tarmac / wet / grass | 27.0 / – / – m | 28.0 / 36.6 / 45.1 | 28.0 / 36.6 / 45.1 | – |
| Swept wall: straight / 0.5 rad | – | – | nose stops at −39.86 / −39.63 (wall at −40) | – |
| Car-car head-on (World) | – | – | contact, minimum centre distance 4.88 m | – |
| Trial Mountain test driver, 60 s | 1404 m | 1487 m | 1512 m, wheels grounded 99 % | 1526 m |

## Fidelity status (spec draft 24c, residual pass 2026-10-16)

Everything listed under "What follows the spec" is implemented from the spec,
and most of it is verified against the original's behaviour. In the
whole-pipeline comparisons with the original (the original game running next
to `sim::Vehicle` on the same inputs) the
two now run in lock step until **float noise** decides: the original's own
float-noise horizon, measured by re-running it with add/mul rounded to nearest
instead of chopped (`GT3R_DEMO_NEAREST=1`), is of the same length.

### Matches the original tick for tick

| Comparison with the original | Result |
|---|---|
| `phys_steady_cornering` (R34, 60/100 km/h, steer sweep) | load split exact to 1 N; no quantity out of tolerance at the original's best angle; max lateral g 1.326 / 1.390 vs 1.327 / 1.391 |
| `phys_top_end` | 150→220 km/h 16.920 vs 16.921 s, 210→250 km/h 21.263 vs 21.268 s |
| `phys_power_oversteer`, `phys_power_rx7_table` | rear β > 10° on the same tick in all runs (RX-7 player 166/166); RX-7 table: nothing differs by 0.5 % in 240 ticks |
| `phys_launch_sequence`, `phys_power_rx7_clutch` | shift tick, shift clutch (state 3) and re-lock exact |
| licence demo replays (`phys_demo_*`) | lock-step table below |

### Licence demo lock-step (`phys_demo_*`)

Body position of the original (O) against ours (U), ticks after the hold, up to
ours' first wall impulse (the original side has no walls). "O vs O′" is the
original against itself with nearest rounding: the float-noise horizon. The
comparisons assert the U horizons (10–20 % margin).

| Demo | O vs U within 1 cm / 0.3 m | O vs O′ within 1 cm / 0.3 m | Before this pass (0.3 m) | Remaining |
|---|---|---|---|---|
| B-1 (Beetle, standing) | 1538 / whole run; **same finish tick** (38.93 s) | 1441 / whole run | 852 | float noise |
| A-4 (RX-7, standing) | 997 / 1216 | 968 / 1213 | 387 | float noise |
| B-3 (911, standing, slope) | 1052 / 1409 | 1364 / 1674 | 485 | a one-tick landing knife-edge (FR wheel, tick 940, x at the droop limit) |
| A-6 (flying) | 596 / 865 | 866 / 1039 | 812 | the hand-over state copy, then a sqrt-damper amplification near zero damper speed |
| IB-3 (flying) | 408 / 504 (wall) | 741 / 972 | 314 | the hand-over state copy; ours touches a wall at tick 504 (the original side has none; its path is 2.5 cm away) |

Our own sensitivity is of the same size: ours against ours with one ground
height nudged by 1e-5 m (the "chaos horizon" line of each test), or built with
`-ffp-contract=off`, separates by 1 cm after 900–1700 ticks.

`sim_license_demos` (ours alone, the race scene's set-up): B-1 finishes 34.93 s
(recorded 34.94 s), A-4 44.53 s (recorded 43.85 s; the original's own replay
leaves the road at 28.5 s), B-3 course-out at 25.7 s (the original's replay at
25.5 s), A-6 course-out at 29.3 s (the original's replay at 16.9–19.7 s), IB-3
not finished after its wall touch.

### Fixes of the residual pass (each found as a first divergence)

| Fix | Where it showed | Evidence |
|---|---|---|
| Static suspension position x_s = −½·share·(total − 2 unsprung)·g / k (the weight split applies to the **sprung** mass) | static pitch 0.3 mrad off on B-1, 1.8 mm x_s on B-3 | static position exact on 234 cars + 590 random settings (`phys_car_spec*`) |
| Both suspension blocks read the front ride height | rear bump-stop limit 10 mm off on 4 cars | `phys_car_spec`, rear bump-stop limit |
| Engine torque, friction and the limiter read the stored engine speed (for a locked clutch the capped end-of-tick value), no re-sync to the wheel side first | first limiter tick decelerated 1.3 % more (B-1 tick 33) | B-1 finishes on the same tick |
| Pitch/heave terms A/B use the drive + brake parts G only (no projected lateral grip / drag forces) | front loads drifted from the first steering input (A-4 tick 28) | `phys_demo_a4`, `phys_steady_cornering` load split |
| Hub velocities are the game's lever form (u + X r, l − Y r) and the free-wheel lock is applied at the end of the tick with the same expression the next slip ratio uses | −78 N slip forces every third tick at rest on B-3; free wheels lagged a tick | `phys_demo_b3`, `phys_power_rx7_table` |
| The differential works on peripheral speeds (m/s, r/Ia): the viscous transfer gain·min(Δw², initial²) takes Δw in m/s | the split of an airborne front wheel (B-3 tick 930) | `phys_demo_b3` |
| One-sided steering-lock extension (only the bound on the side of the mean front velocity angle) | IB-3 steering after the hand-over | `model_steer_target_extension` (199 of 1000 samples differ with the symmetric rule) |
| Race hold (`Vehicle::set_race_hold`) instead of brakes + handbrake during countdowns; a hold also zeroes the longitudinal force sum | B-3's hold on a 3° slope settled 1.4 mrad nose-down with braked wheels | `phys_stop_hold_words`; B-3 hold loads exact |
| Aero, steering, clutch, aids, shifts and engine counters read the ground-frame u (§10.1), and the query points lie in the CoG plane | small u offsets on slopes; 4 mm hub offset at 1° pitch | `phys_demo_*` |

Comparison set-up: the hand-over copy (flying starts) now also sets the attitude
and its rates directly, the follower angle (not the mean road-wheel angle) and
rate, the hub velocities (§10.1), turbo lag, differential lock flags and the
aids' previous-tick words; the load tolerance uses the mean static corner load
(B-3's "divergence at tick 1" was the first tick's zero front load scaling the
tolerance to 0.07 N).

### Classification of what is left

- **(b) float noise.** B-1 and A-4 reach the original's own nearest-rounding
  horizon; the first differences grow smoothly from the 1e-5 level and nothing
  jumps. Float contraction (FMA) on our side moves our trajectories at the
  same rate but leaves every O–U horizon unchanged (B-3 1041 vs 1052, A-4 1009
  vs 997, A-6 and IB-3 identical), so matching the compiler's float operation
  order is no win: the dominant float-level difference is the PS2's chopped
  single precision against our double precision, which only a full PS2-float
  port of the sim could remove. `-ffp-contract` is left at the default.
- **(b) knife-edges amplified by the model.** B-3's front-right landing at tick
  940 is decided by x at the droop limit; A-6 grows through the sqrt damper law
  (dF/dv is steep near v = 0, a 1e-5 m difference in x moves Fd by 20 N).
- **(c) replay environment.** Flying starts copy our state into the original at
  the line; moving that copy into a run that is otherwise in step
  (`GT3R_DEMO_RETRANSFER=k` on A-4) is harmless at ticks 150/300 but shortens
  the horizon at 450, so some transient state is still not copied. The
  original side has no walls (IB-3 touches one at tick 504), and B-3 leaves the
  road on both sides where the course's walls / race phases are missing.
- **(a) systematic** differences found in this pass are fixed; none is left in
  the five demos above the float-noise level. Wall contact (`race_wall_hits`)
  is still approximate.

### Known remaining deviations

1. **Driver parameters of the run-up.** The original uses the built-in set by
   course category (1 → set 1, dirt 2 → set 2, else 0) without event overrides
   (AI spec §9.1); `sim_license_demos` does the same.
2. **AWD strength byte.** B5 (the AWD strength, §2.7) has no known database source yet, so
   the centre-clutch capacities use the table defaults (8 or 15 × 98 N·m).
   Torque vectoring's steer sign (positive = left in the original) is an
   interpretation.
3. **Pit stop for cars without the spec suspension block** (hand-built specs)
   run the 300-step counter without the scripted pose.
4. **Hand-built test car.** `CarSpec::test_car` is a placeholder (AWD = system
   1 coupling with B4 = 50, wheel inertia placeholder when the spin-inertia
   word is absent).
5. **Rolling/flying start placement and hand-over** (§9.2/9.4: grid slot moved
   back by the front extent, AI line 6, 300-step AI drive) are the race's; physics
   provides `set_driver_kind` (scripted aids path, automatic, AI clutch rule).
6. **Draft 19 (§10) done:** free-rolling wheels locked to the ground (§10.2, at the end of the tick, cos d (u + X r) − sin d (l − Y r), the same expression the next slip ratio reads),
   engine condition factor and counters (§10.3, `set_engine_condition`), dirt
   front brake cut (§10.4, `set_dirt_course`), airborne wheel body force
   −preload and the clamped x_prev (§10.5), no drag and frozen yaw with all
   wheels airborne (§10.6), handbrake disables ASCC only (§4.6).
   A ground-query miss (off the road mesh) keeps the previous ground height and
   surface for that wheel (§1.3/§5.3), so wheels and body agree off the mesh.
   Wheel speeds and slip use the §10.1 ground-frame tangents and a yaw-only
   hub velocity.
   Benchmark: `sim_license_demos` (see "Licence demo lock-step" above).
   `SIM_DEMO_TRACE=<code>`, `SIM_DEMO_WHEELS`, `SIM_DEMO_ENGINE`, `SIM_DEMO_INPUTS`
   print traces.

7. **Wall contact geometry.** Car points against wall edges are swept as
   in the game, but wall corners against car edges are not tested
   separately; the push-out covers them. The push-out is perpendicular for
   penetrations up to 1.5 m; the original moves along the wall beyond 0.5 m
   (§5.3 end-of-step push-out) and pushes wall corners onto car edges, not yet reproduced. The track adapter keeps the wall
   segments within reach of the car rather than the game's grid lookup
   (same result).
8. **Car-car contact details.** The near-stopped end-pose recompute
    (§5.3) uses the spec's condition (draft 16c): both speeds < 1 m/s
    and the last hit at t < 0.1; `sim::World` then also applies the
    stuck-steer yaw nudge (yaw rate += -steer x 20 x dt, steer = input as a
    fraction of full lock; applying it before the end-pose recompute is an
    interpretation). The step is computed in the plane frame (X, Y) in the
    same order of operations as the spec's step. A
    vertex lying on an edge line to float noise is decided by rounding in
    the original; ours treats it as already inside (no new hit), which
    matches the original on 98 % of overlapping starts.
9. **Unknown or undecoded data:**
    - turbo intake smoothing (sound only);
    - camber's "visual-only" gains;
    - power-restricted events (§2.2 power cap);
    - user downforce/suspension settings beyond §6.6;
    - arcade-car settings (table 35 +0xE8..).
10. **Hull/ground contact:** none for spec-suspension cars, as in the original (spec §11): the
    ground is touched only through the wheel probes, bump stops and the §4.3 nudge.
    `GT3R_HULL_CONTACT=1` restores the old penalty contact for comparison; hand-built specs
    still use it.
11. **Turbo boost HUD value:** `boost` = Bcap − 1 is an interpretation of
    the game's boost gauge reading.
