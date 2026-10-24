# AI drivers (`src/ai/`)

CPU opponents that drive through the same vehicle simulation as the player.
The behaviour is described in [docs/behaviour/ai.md](../behaviour/ai.md) ("ai §n"
below names its sections); this page says how it is implemented and which parts
are confirmed against the original and which are ours.

| File | Contents |
|---|---|
| `src/ai/ai_driver.h` | public API: `DriverParams`, `course_params`, `apply_event`, `after_finish`, `RubberBand`, `adapt_spec_for_ai`, `AiCourse`, `CarSnapshot`, `AiContext`, `AiDriver`, `AiDebug` |
| `src/ai/ai_rules.h/.cpp` | the decision rules as small pure functions (grip, corner speed, speed controller, pit-lane sections, recovery, end of run) |
| `src/ai/ai_params.cpp` | parameter sets (stored-byte conversion, built-in sets, the course's set), event overrides, rubber band |
| `src/ai/ai_course.cpp` | AI lines prepared for driving (progress per point, arcs, evaluation, the point's surface) |
| `src/ai/ai_driver.cpp` | the per-tick decision logic |
| `src/ai/safety_respawn.h/.cpp` | the non-original safety respawn shared by `race::Field` and `AiRace` (ai §12) |
| `src/ai/ai_race.h/.cpp` | `AiRace`: N AI cars in a `sim::World` with lap timers (tests, screen, reference for the race scene); `race_params` |
| `src/formats/course/driver_sets.*` | the course file's driver parameter sets (slots 37-40, docs/formats/course.md) |
| `src/app/screen_ai.cpp` | developer screen `ai` ("AI test") |
| `tests/test_ai.cpp` | tests (below) |
| `tests/test_ai_rules.cpp` | disc-free tests of each rule (below) |
| `tests/test_ai_synthetic.cpp` | disc-free tests on the invented stadium course of `tests/course_fixtures.h` (below) |
| `tools/gt3tool/cmd_ai.cpp` | `gt3tool ai-survey [prefix]`: line roles and flag statistics |

## Using it (race workstream)

```cpp
auto course = std::make_shared<ai::AiCourse>(*track);           // once per track
auto spec = *sim::build_car_spec(db, "sun_e_vitz_rs15_00");
ai::adapt_spec_for_ai(spec);                                     // AI-car spec: 45° lock
const int cls = ai::driveline_class(spec);
ai::DriverParams p = ai::course_params(track->driver_sets(), category, cls); // the course's set (ai §9.1)
ai::apply_event(p, *db.event(event_label), cls);                 // event difficulty
auto v = sim::Vehicle::create(spec); v->set_cpu_driver(true);     // CPU clutch rule
v->set_scripted_aids(p.aids);                                     // the set's AI aid block (TCS, ASCC)
int id = world.add(std::move(v));                                 // sim::World
ai::AiDriver drv(course, p, /*start_offset=*/grid_slot);
drv.reset(world.vehicle(id).state());
ai::RubberBand rb = ai::RubberBand::from_event(*db.event(event_label));
// every 1/60 s tick:
//   fill std::vector<ai::CarSnapshot> (position, orientation, velocity, half sizes) for all cars
//   world.set_inputs(id, drv.update(world.vehicle(id), {race_time, snapshots, id}));
//   world.vehicle(id).set_reverse_request(drv.reverse_request());
//   world.vehicle(id).set_driver_kind(drv.kind());               // (the stop driver parks itself)
//   world.set_time_scale(id, rb.scale(player_race_distance - car_race_distance));
//   world.step(1.0 / 60);
```

`race_time` is seconds since the start signal; during the countdown (negative
values) the driver blips the throttle, with no brake or steering (ai §10); the
caller holds the clutch open (`Vehicle::set_clutch_hold`) and the car in place
(`Vehicle::set_race_hold`), as `race::Field` does. `AiRace::step()` is a
complete example. The driver is deterministic (no clocks, no RNG;
`ai_deterministic` checks bit-identical positions of three cars after 40 s).

## What is implemented, and from where

Spec-confirmed (matches the original's behaviour, [C]):

- Line numbering (game line = file line + 1), roles, fallback table and
  initial line choice (ai §2.1); control-point fields used: kind, brake %,
  speed km/h, radius (ai §2.2); evaluation by progress with straight/arc
  interpolation (ai §2.3).
- Parameter sets (ai §9.1): the course file's set for the car's driveline class
  (slots 37-40; 79 of the 80 US course files carry four), the built-in set of
  the course category only where the slot is empty (L_circle60_wet1); event
  overrides per driveline class on top for CPU cars (ai §9.2), none for the
  player's run-up. Every byte is converted, the aid bytes included: the set's
  TCS and ASCC are the aid block the physics reads for CPU and scripted cars
  (`sim::ScriptedAids`, physics §4.6), so e.g. most dirt-course sets run CPU
  cars without TCS.
- Lookahead L₀..L₁ by speed, +40 m decaying over the first 2 s, 5 m
  minimum distance, line blending over 1 s, 5 m on the stop line and 20 m
  leaving the pits in the pit-lane limit (ai §3).
- Segment bookkeeping; corner speed (ai §4.2) from the grip the tyre records
  give: 9.8 × the record's mu for the point's surface (the compound slot the
  surface uses), w-blended front / rear, scaled by the live wear factors except
  on the pit surface, three downforce iterations from 0; braking distance with
  slope/brake-% terms and car floor, braking modes 3/4 (ai §4.3); the speed
  controller with the game's gains and on/off brake (ai §4.4); modes and corner
  bonus (ai §5), the arc status set only when it is 0, mode 4 counting the arc
  ticks; throttle slew in arcs.
- Pit lane (ai §13): the AI driver drives it itself. The race's pit state machine
  (`race::PitLane`: markers, entry, 25 m decision, box, exit) only switches the
  driver's line (3 at entry, 4 for the stop, 3 when driving through or leaving,
  all immediate; 6 blended at the exit) and driver kind (2/3/4). On line 3 the
  sections come from the stop line's points (ai §4.4): the braking modes aim at the
  set's pit speed + 2 km/h from point 1, the arc controller holds it (gain -0.5,
  no corner bonus on the pit surface) from point 2 to n - 3; the stop line runs
  braking mode 3 at 20 km/h. The player's pit lane uses the same driver.
- Steering as a wanted front slip angle from the yaw-rate deficit
  (pure-pursuit desired yaw rate, K_y = 1.2), clamped to the tyre's peak
  slip angle, plus the front-axle velocity angle; saturation feeds the
  throttle cut; 45° clamp; 90°/s slew; wheels straight below 2 km/h (ai §6).
- Recovery (ai §7): the drive decision and the 3-state machine with the game's
  angle and speed thresholds and no timers; the flags are the collision step's
  front / rear impulse sides (`VehicleTelemetry::hit_sides`, set by the wall
  response at octagon points 0-3 / 4-7 and by car-car responses at a vertex or
  edge); turning round aims at 50 km/h; reverse via the automatic gearbox.
- Traffic: zone bytes and contact intensity as the car–car pass computes
  them, lane-change choice through the side map, 0.5 s cooldown, throttle ×
  (1 − c), move away from a car alongside, back to the main line when free,
  5 s + offset start delay (ai §8); no following brake or speed matching (ai §14).
- Countdown: no brake or steering, the throttle blipped around the upshift
  rpm while the caller holds the clutch open (ai §10).
- Driver kinds and the end of a run (ai §1): after a race, time trial or machine
  test the player's car is driven on along the main line by the AI (kind 1,
  corner speed × 0.8, throttle 0.5; CPU finishers get the same slowdown); a
  failed licence test or time trial stops it with the wheels straight (kind 7);
  stop / section licence tests stop along the main line (kind 6), the other licence
  tests drive on slowed (kind 1); a free run's end drives on along the main line
  (kind 1, not slowed), or stops (kind 7) on the pit surface; the stop
  driver parks the car (kind 8: brake released, clutch open) once it stands
  still. `ai::end_of_run` picks the kind; `screen_race_drive.cpp` runs it.
- Rubber band curve (ai §9.3, applied as `sim::World::set_time_scale`); `race::Field`
  and `AiRace` chain it along the running order from the reference car, the player
  (`RubberBand::chain`, ai §14).

Arcs are built the game's way (ai §2.3): centre = control point + radius ×
the left normal of its stored heading, positions at the interpolated angle
around that centre. An earlier chord-based fit (choosing between the two
arcs of the given radius through both points) bulged up to 13 m off the
road where the radius is half the chord (r_akasaka, Monte Carlo); that, not
the walls, caused most of the crashes and respawns. The main line now lies
on the road for 73,601 of 73,765 samples (5 m apart) over all lap courses;
the rest are on L_testcourse1, whose mesh does not cover the whole lap.

CPU cars are driven the original's way: automatic gearbox,
`Vehicle::set_cpu_driver(true)` (the original's CPU clutch request), the
set's aid block (`Vehicle::set_scripted_aids`), the road-wheel angle in
`ControlInputs::steer_angle`, and `Vehicle::set_reverse_request()` from
`AiDriver::reverse_request()` while backing out.

Approximate or ours (marked `[placeholder]` in the code where they are values):

- **Braking grip.** The braking distance uses 9.8 × the front record's mu on the
  next point's surface with the front wear factor; the original also multiplies
  by the load curve at the static per-width load, the braking-side peak of curve
  C and the camber table value (ai §4.3), which we leave out.
- **Steering lock.** `adapt_spec_for_ai` raises the car's full lock to at
  least the AI's 45° clamp. The physics clamps a CPU car's `steer_angle` to
  ±1 rad, not to the lock, so the raised lock only changes the steering
  fraction (angle / lock) of the car-to-car stuck-steer nudge (physics §5.3)
  and the display-only `ControlInputs::steer`.
- **Slope/bank** of a control point from the road normal (the loader's
  derivation was not read), with a 0.2 floor on the bank divisor; the point's
  progress and surface come from our own track projection and ground query.
- **Stop line target.** The original evaluates line 4's target with its own
  function (not read); ours evaluates it like any line. During the 5 s stop
  the car is held with the brake (the original runs the countdown blipper).
- **The player's AI** (run-up, pit lane, end of run) sees no other cars (an own
  snapshot only); the original's player driver gets the zone bytes once its
  kind is non-zero. Player 2 of a 2P battle is still driven by the screen's
  heuristic autopilot after the finish.
- The safety respawn (ai §12) is ours: GT3 never moves a car. One rule
  (`ai::SafetyRespawn`) for `race::Field` and `AiRace`: lost after 0.5 s off the
  road mesh, a non-finite position, or 12 s from GO without 10 m of progress
  (not counting the pits); re-placed on the centre line with the spawn hold;
  `[race] respawn = off` / `AiRace::set_respawn(false)` disable it.

Removed once the arcs were right (each was tested off, singly and together,
in both driveline modes): a stability aid, enabling physics TCS, a manual
gearbox chooser, a corner-bonus cap, a braking margin while saturated, the
throttle cut in 1st gear, lifting the event throttle cap at low speed,
backing out when scraping a wall, and the wall-aware lookahead with a
centre-line fallback. Removed with the original's rules (2026-10-20): the grip
estimate from our own tyre model and its `grip_scale` / `boost_scale` knobs,
the wall-sphere / pinned recovery flag with its dwell times, forced back-out
and pinned steering, the fixed 82 km/h pit-lane follower, and the "kind 5 after
every finish" autopilot.

Not implemented: the recommended-gear output (the HUD hint runs its own passive
driver).

## Tests (`./gt3r_tests ai_`, with `GT3R_DISC` for the disc-based ones)

| test | checks |
|---|---|
| `ai_params_builtin`, `ai_rubber_band_curve` | parameter conversion; rubber-band curve values (no disc) |
| `ai_course_sets_decode`, `ai_params_from_bytes`, `ai_course_params_select`, `ai_driveline_class` (no disc) | the course file's set slots (an invented file: present, empty, other classes), every byte's conversion incl. the aid block, the course set vs the category's built-in set, event overrides per class, `after_finish` |
| `ai_corner_grip_formula`, `ai_corner_speed_formula`, `ai_speed_controller_rule` (no disc) | 9.8 × record mu per surface / compound slot, w blend, wear factors (off on the pit surface); the three downforce iterations, scale / offset / trim / km/h, clamps; the controller |
| `ai_pit_lane_target` (no disc) | the stop line's markers with the lap added across the start line, the pit-lane sections, the speed plans of lines 3 / 4 and state 2, no corner bonus on the pit surface |
| `ai_recovery_transitions`, `ai_collision_sides` (no disc) | every threshold and transition of ai §7; the front / rear sides of wall and car-car responses |
| `ai_end_of_run_kinds`, `ai_stop_driver_parks` (no disc) | the end-of-run kinds; kinds 7 and 6 stop the car (throttle 0, brake 1) and park it (kind 8, brake off, wheels straight) |
| `ai_safety_respawn_rule`, `ai_race_stuck_respawn`, `race_field_stuck_respawn` (no disc) | the shared respawn rule (off-mesh ticks, 12 s from GO, pits, non-finite); `AiRace` and `race::Field` on a car that cannot move: both re-place it at 12 s with the spawn hold; off when disabled |
| `sim_scripted_ascc_block` (no disc) | the AI aid block's ASCC acts for a CPU car where enabled, the car's own block only for a human |
| `ai_event_overrides` | Sunday Cup: throttle 85 %, corner speed 80 %, rubber band 20 %/20–80 m, 10 %/50–100 m |
| `ai_course_param_sets_disc` | 79 of the 80 US course files carry four sets, L_circle60_wet1 none; pit speeds 78–80 km/h |
| `ai_line_roles` | main line on Trial Mountain: 393/393 samples on the road |
| `ai_laps_all_courses` | one car (`r34_gtr_v_99`, the course's set for its class, no event) drives two laps from the grid on all 36 non-licence US courses with a lap line; no stall over 20 s, at most 2 safety respawns. Env: `GT3R_AI_COURSE`, `GT3R_AI_CAR`, `GT3R_AI_TRACE=1` (+`GT3R_AI_FROM=<s>`) for a per-tick trace, `GT3R_AI_TRACE_ENGINE=1` adds body height, per-wheel ground contact / suspension / surface, rpm and clutch |
| `ai_event_races` | 6 cars of Sunday Cup 1–3 and Clubman Cup 3 on their courses in a `sim::World` (car contact, slipstream, rubber band against car 0), each with the course's set for its class and the event's overrides: all finish 2 laps, at most 2 safety respawns per car, no stall over 20 s; prints contact events and position changes. `GT3R_AI_EVTRACE=<event>:<car>` traces one car |
| `ai_cpu_pit_stop_disc` | a CPU car armed at once on Trial Mountain enters, reaches the stop decision at the set's pit speed + 2 (80.6 km/h), the box at about 20 km/h, stops and returns to racing (kinds 2, 3, 4, 0) |
| `ai_deterministic` | two identical 3-car races are bit-identical after 40 s |
| `ai_synthetic_*` (no disc) | the stadium course (two 200 m straights, 50 m arcs, invented AI lines 1 / 2 / 6 and an invented parameter set: corner speed 70 %, no bonus) and the spec-path test car: line following, braking for an arc, overtaking a slower car on a straight, turning round, and backing out of a wall after a front hit (115° and 90°), the countdown blip, determinism, and a golden trace of `AiDriver::update` on scripted car states with injected collision sides (`GT3R_GOLDEN_PRINT=1` regenerates it) |

Current reference (2026-10-20, the original's rules: the course's set for the car's
class, record grip, exact recovery). Best of two laps from a standing start,
`r34_gtr_v_99`, no event; times move whenever the physics changes. Against the table
before (built-in set by category, our tyre-model grip, timer-based recovery): mean
-0.6 s; most tarmac courses 0.3-3.6 s faster (the record grip plans faster corners,
the course sets brake later, e.g. trialmt brake 115 %), r_akasaka +3.4 s, r_slalom80r
+2.2 s; on dirt the course sets turn CPU TCS off: r_smtsouth +8.6 s (a spin on lap 2),
r_tahitimaze +8.4 s, tahitimaze -8.2 s. One safety respawn over the 36 courses
(r_tahitimaze: after a spin the car stood nose to a wall at 34° in state 2, below the
60° that sends it reversing, wheels straight below 2 km/h, and crawled along the wall
for 12 s); longest other stall 8.4 s (tahitimaze).

| course | length | best lap | longest stall | | course | length | best lap | longest stall |
|---|---|---|---|---|---|---|---|---|
| akasaka | 5121 m | 1:58.489 | 2.0 s | | r_smtnorth | 3578 m | 1:28.643 | 2.5 s |
| apricot | 3865 m | 1:35.694 | 2.1 s | | r_smtsouth | 3291 m | 1:44.055 | 5.8 s |
| deepforest | 3577 m | 1:27.482 | 2.1 s | | r_tahitidirt | 3723 m | 1:50.638 | 2.5 s |
| grandvalley | 5022 m | 2:11.750 | 2.5 s | | r_tahitimaze | 3555 m | 2:35.386 | 12.0 s (1 respawn) |
| laguna | 3604 m | 1:35.577 | 3.8 s | | r_trialmt | 3921 m | 1:41.079 | 2.5 s |
| midfield | 3597 m | 1:24.348 | 2.2 s | | rome | 3959 m | 1:41.036 | 2.1 s |
| montecarlo | 3363 m | 1:55.109 | 2.6 s | | route11 | 4561 m | 2:14.019 | 2.0 s |
| r_akasaka | 5121 m | 2:04.142 | 2.1 s | | route5 | 3783 m | 1:47.022 | 2.4 s |
| r_apricot | 3865 m | 1:38.039 | 2.2 s | | route5_dry | 3783 m | 1:37.811 | 2.2 s |
| r_deepforest | 3577 m | 1:27.515 | 2.2 s | | seattle | 3863 m | 1:51.168 | 2.4 s |
| r_grandvalley | 5022 m | 2:14.178 | 2.5 s | | slalom80r | 11235 m | 5:30.039 | 4.3 s |
| r_midfield | 3588 m | 1:27.578 | 2.4 s | | smtnorth | 3578 m | 1:28.441 | 2.5 s |
| r_rome | 3959 m | 1:43.206 | 2.0 s | | smtsouth | 3291 m | 1:37.236 | 4.7 s |
| r_route11 | 4535 m | 2:16.038 | 2.1 s | | super | 2404 m | 0:45.138 | 2.7 s |
| r_route5 | 3783 m | 1:46.237 | 2.3 s | | tahitidirt | 3723 m | 1:51.261 | 2.5 s |
| r_route5_dry | 3783 m | 1:41.139 | 2.1 s | | tahitimaze | 3555 m | 2:31.890 | 8.4 s |
| r_seattle | 3863 m | 1:49.899 | 2.4 s | | testcourse | 10342 m | 2:23.051 | 1.9 s |
| r_slalom80r | 11235 m | 5:40.459 | 4.2 s | | trialmt | 3922 m | 1:40.326 | 2.5 s |

For comparison, a stock R34 GT-R in GT3 laps Trial Mountain in roughly
1:35–1:40, Deep Forest in 1:22–1:26 and Grand Valley East in 1:55–2:00 (rough
figures from memory of the game, not measured).

Event races (6 cars, 2 laps, `sim::World` with contact and rubber band; each car the
course's set for its class plus the event's overrides):

| event | course | lap times | longest stall | contact events (nose-to-tail) | position changes |
|---|---|---|---|---|---|
| Sunday Cup 1 | super | 1:01.9–1:03.6 | 7.2 s | 135 (0) | 12 |
| Sunday Cup 2 | midfield | 1:55.8–1:58.6 | 8.6 s | 115 (78) | 15 |
| Sunday Cup 3 | trialmt | 2:12.9–2:13.7 | 7.8 s | 85 (32) | 16 |
| Clubman Cup 3 | deepforest | 1:40.2–1:41.8 | 6.3 s | 92 (26) | 13 |

(Before, with the built-in set 0 and class 0 overrides for every car: 1:02.0–1:04.3,
175 (127), 11; 1:59.2–2:01.1, 56 (16), 17; 2:13.8–2:16.3, 72 (45), 13; 1:41.2–1:42.7,
73 (34), 21. Against the original running the same races (2026-10-11 measurements),
contact episodes per race: Sunday Cup 1 7, Sunday Cup 2 8, Sunday Cup 3 17-40,
Clubman Cup 3 10-30.)

A contact event is a pair of cars starting to touch (a rising edge per pair, no
debounce); a position change is a tick on which the running order changed after the
first 10 s. No car needs a respawn in these races.

## Screen

`gt3r --screen ai` lists the events with opponents; Cross starts one. The
map shows walls, AI lines (green main, blue A, red B, yellow pit lane),
cars and the focused car's target point; the table shows positions, laps,
best laps and the focused car's driver state (mode, line, targets, braking
distance, heading error, wanted slip, throttle/brake, contact, rubber-band
time scale). L1/R1 change the focused car, Square zooms, Select toggles the
lines, Circle pauses, Start restarts. `GT3R_AI_EVENT=<label>` and
`GT3R_AI_PRERUN=<seconds>` are for screenshots. Car 0 stands in for the
player as the rubber-band reference.
