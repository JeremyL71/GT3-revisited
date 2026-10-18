# AI drivers (`src/ai/`)

CPU opponents that drive through the same vehicle simulation as the player.
The behaviour follows a written description of the original's AI (the
"spec"; section numbers below refer to it); this page says how it is
implemented and which parts are confirmed against the original and which are ours.

| File | Contents |
|---|---|
| `src/ai/ai_driver.h` | public API: `DriverParams`, `apply_event`, `RubberBand`, `adapt_spec_for_ai`, `AiCourse`, `CarSnapshot`, `AiContext`, `AiDriver`, `AiDebug` |
| `src/ai/ai_params.cpp` | built-in parameter sets, event overrides, rubber band |
| `src/ai/ai_course.cpp` | AI lines prepared for driving (progress per point, arcs, evaluation) |
| `src/ai/ai_driver.cpp` | the per-tick decision logic |
| `src/ai/ai_race.h/.cpp` | `AiRace`: N AI cars in a `sim::World` with lap timers (tests, screen, reference for the race scene) |
| `src/app/screen_ai.cpp` | developer screen `ai` ("AI test") |
| `tests/test_ai.cpp` | tests (below) |
| `tools/gt3tool/cmd_ai.cpp` | `gt3tool ai-survey [prefix]`: line roles and flag statistics |

## Using it (race workstream)

```cpp
auto course = std::make_shared<ai::AiCourse>(*track);           // once per track
auto spec = *sim::build_car_spec(db, "sun_e_vitz_rs15_00");
ai::adapt_spec_for_ai(spec);                                     // AI-car spec: 45° lock
auto v = sim::Vehicle::create(spec); v->set_cpu_driver(true);     // CPU clutch, no aids
int id = world.add(std::move(v));                                 // sim::World
ai::DriverParams p = ai::DriverParams::builtin(0);               // set by car spec byte 0x31
ai::apply_event(p, *db.event(event_label), 0);                   // event difficulty
ai::AiDriver drv(course, p, /*start_offset=*/grid_slot);
drv.reset(world.vehicle(id).state());
ai::RubberBand rb = ai::RubberBand::from_event(*db.event(event_label));
// every 1/60 s tick:
//   fill std::vector<ai::CarSnapshot> (position, orientation, velocity, half sizes) for all cars
//   world.set_inputs(id, drv.update(world.vehicle(id), {race_time, snapshots, id}));
//   world.vehicle(id).set_reverse_request(drv.reverse_request());
//   world.set_time_scale(id, rb.scale(player_race_distance - car_race_distance));
//   world.step(1.0 / 60);
```

`race_time` is seconds since the start signal; negative values (countdown)
hold the car on the brakes. `AiRace::step()` is a complete example. The
driver is deterministic (no clocks, no RNG; `ai_deterministic` checks
bit-identical positions of three cars after 40 s).

## What is implemented, and from where

Spec-confirmed (matches the original's behaviour, [C]):

- Line numbering (game line = file line + 1), roles, fallback table and
  initial line choice (§2.1); control-point fields used: kind, brake %,
  speed km/h, radius (§2.2); evaluation by progress with straight/arc
  interpolation (§2.3).
- Lookahead L₀..L₁ by speed, +40 m decaying over the first 2 s, 5 m
  minimum distance, line blending over 1 s (§3).
- Segment bookkeeping, corner speed with the 3-iteration downforce fixed
  point, braking distance with slope/brake-% terms and car floor, braking
  modes 3/4 (§4); the speed controller with the game's gains and on/off
  brake (§4.4); modes and corner bonus (§5); throttle slew in arcs.
- Steering as a wanted front slip angle from the yaw-rate deficit
  (pure-pursuit desired yaw rate, K_y = 1.2), clamped to the tyre's peak
  slip angle, plus the front-axle velocity angle; saturation feeds the
  throttle cut; 45° clamp; 90°/s slew (§6).
- Recovery drive decision and the 3-state machine with the game's angle and
  speed thresholds; reverse via the automatic gearbox (§7).
- Traffic: zone bytes and contact intensity as the car–car pass computes
  them, lane-change choice through the side map, 0.5 s cooldown, throttle ×
  (1 − c), move away from a car alongside, back to the main line when free,
  5 s + offset start delay (§8).
- Parameter sets (§9.1), event overrides per chassis class (§9.2), rubber
  band curve (§9.3, applied as `sim::World::set_time_scale`).

Arcs are built the game's way (§2.3): centre = control point + radius ×
the left normal of its stored heading, positions at the interpolated angle
around that centre. An earlier chord-based fit (choosing between the two
arcs of the given radius through both points) bulged up to 13 m off the
road where the radius is half the chord (r_akasaka, Monte Carlo); that, not
the walls, caused most of the crashes and respawns. The main line now lies
on the road for 73,601 of 73,765 samples (5 m apart) over all lap courses;
the rest are on L_testcourse1, whose mesh does not cover the whole lap.

CPU cars are driven the original's way: automatic gearbox,
`Vehicle::set_cpu_driver(true)` (the original's CPU clutch request, no
driving aids, since the AI aid block is all zero), the road-wheel angle in
`ControlInputs::steer_angle`, and `Vehicle::set_reverse_request()` from
`AiDriver::reverse_request()` while backing out.

Placeholders (ours, marked `[placeholder]` in the code):

- **Grip numbers.** Corner speeds need the game's tyre record μ; we derive it
  from our tyre model (`tire_peak_force` × peak of the lateral curve, × g)
  and the peak slip angle from our lateral curve. `DriverParams::grip_scale`
  and `boost_scale` (default 1) allow tuning.
- **Steering lock.** `adapt_spec_for_ai` raises the car's full lock to the
  AI's 45° clamp, because physics clamps `steer_angle` to the lock.
- **Collision flags.** The game's state machine reads two words of the
  collision body; we use "a wall within reach of the nose, centre or tail"
  or "pinned for 1.5 s", with a dwell (3 s reversing, 1.5 s turning) so the
  states do not flip every tick; when pinned the wheels are steered even
  below 2 km/h, turning round slowly uses full lock, and entering reverse
  from a wall forces 2.5 s of reverse.
- **Following (traffic).** GT3 only lifts (throttle × (1 − c)) within 10 m
  and otherwise relies on contact. With our car-to-car impulses that gave
  100–250 contact events per 2-lap race, mostly nose-to-tail. The driver
  brakes to match a slower car in its path (the path bends with the yaw
  rate) when the deceleration needed to keep a gap of 1.5 m + 0.25 s × v
  exceeds 0.35 of its grip, and lifts from 0.15. Being held up (> 0.15)
  also triggers the spec's passing-line choice, and on a passing line a car
  overlapping by less than 60 % of the combined width is passed rather than
  followed.
- **Slope/bank** of a control point from the road normal (the loader's
  derivation was not read).
- **Countdown**: brake held (the game blips the throttle with the clutch
  open, §10).
- `AiRace` (not the driver) respawns a car on the centre line if it leaves
  the road mesh or makes no 10 m of progress in 12 s, as the race scene
  would. GT3 itself never respawns CPU cars.

Removed once the arcs were right (each was tested off, singly and together,
in both driveline modes): a stability aid, enabling physics TCS, a manual
gearbox chooser, a corner-bonus cap, a braking margin while saturated, the
throttle cut in 1st gear, lifting the event throttle cap at low speed,
backing out when scraping a wall, and the wall-aware lookahead with a
centre-line fallback.

Pit stops (lines 3/4) are driven by the race (docs/dev/RACE.md "Pit stops and
tyre wear"). Not implemented: the AI's own aid parameters, driver kinds 3–8, the recommended-gear output.

## Tests (`./gt3r_tests ai_`, with `GT3R_DISC` for the disc-based ones)

| test | checks |
|---|---|
| `ai_params_builtin`, `ai_rubber_band_curve` | parameter conversion; rubber-band curve values (no disc) |
| `ai_event_overrides` | Sunday Cup: throttle 85 %, corner speed 80 %, rubber band 20 %/20–80 m, 10 %/50–100 m |
| `ai_line_roles` | main line on Trial Mountain: 393/393 samples on the road |
| `ai_laps_all_courses` | one car (`r34_gtr_v_99`, the built-in set of the course category, no event) drives two laps from the grid on all 36 non-licence US courses with a lap line; no stall over 20 s, at most 2 safety respawns. Env: `GT3R_AI_COURSE`, `GT3R_AI_CAR`, `GT3R_AI_TRACE=1` (+`GT3R_AI_FROM=<s>`) for a per-tick trace, `GT3R_AI_TRACE_ENGINE=1` adds body height, per-wheel ground contact / suspension / surface, rpm and clutch |
| `ai_event_races` | 6 cars of Sunday Cup 1–3 and Clubman Cup 3 on their courses in a `sim::World` (car contact, slipstream, rubber band against car 0): all finish 2 laps, at most 2 safety respawns per car, no stall over 20 s; prints contact events and position changes. `GT3R_AI_EVTRACE=<event>:<car>` traces one car |
| `ai_deterministic` | two identical 3-car races are bit-identical after 40 s |

Current reference (spec planar step, dirt-course flag; the off-mesh rule needs the
car and all four wheels off the road mesh for 0.5 s; no AI-side wheelspin control:
the placeholder throttle lift is removed, the CPU aid block's TCS does that job, as in
the original). Best of two laps from a standing start, `r34_gtr_v_99`, the built-in set of
the course category (AI spec §9.1: set 2 on the dirt courses tahitidirt / tahitimaze /
smtnorth / smtsouth and their reverses, set 1 on route5 / r_route5, else 0); times
move whenever the physics changes. CPU cars run the original's AI aid block (TCS gain
6.0, AI spec §9.1) since 2026-10-11. All 36 courses: no safety respawn; longest stall
4.5 s (tahitimaze). Against the table before CPU TCS most courses are 0.1-1.7 s slower
(r_akasaka +3.6 s, its 8.9 s spin gone), r_tahitimaze -2.1 s. Against the previous table (set 0
everywhere): the Smokey Mountain / Swiss Alps dirt courses are 1.5-2.3 s faster with
set 2, the Tahiti Maze pair 5.5-9 s slower (r_tahitidirt +1.2 s, tahitidirt -0.7 s),
route5 -0.6 s; tarmac courses within 0.6 s.

| course | length | best lap | longest stall | | course | length | best lap | longest stall |
|---|---|---|---|---|---|---|---|---|
| akasaka | 5121 m | 1:58.767 | 1.9 s | | r_smtnorth | 3578 m | 1:29.433 | 2.4 s |
| apricot | 3865 m | 1:36.150 | 2.0 s | | r_smtsouth | 3291 m | 1:35.550 | 2.7 s |
| deepforest | 3577 m | 1:30.100 | 2.0 s | | r_tahitidirt | 3723 m | 1:53.483 | 2.4 s |
| grandvalley | 5022 m | 2:15.100 | 2.3 s | | r_tahitimaze | 3555 m | 2:29.433 | 2.2 s |
| laguna | 3604 m | 1:37.433 | 3.6 s | | r_trialmt | 3921 m | 1:43.467 | 2.3 s |
| midfield | 3597 m | 1:26.717 | 2.0 s | | rome | 3959 m | 1:42.683 | 2.0 s |
| montecarlo | 3363 m | 1:56.583 | 2.4 s | | route11 | 4561 m | 2:17.667 | 1.8 s |
| r_akasaka | 5121 m | 2:09.300 | 4.1 s | | route5 | 3783 m | 1:48.567 | 2.2 s |
| r_apricot | 3865 m | 1:37.750 | 2.0 s | | route5_dry | 3783 m | 1:39.400 | 2.0 s |
| r_deepforest | 3577 m | 1:28.183 | 2.0 s | | seattle | 3863 m | 1:52.167 | 2.3 s |
| r_grandvalley | 5022 m | 2:14.683 | 2.3 s | | slalom80r | 11235 m | 5:31.250 | 3.8 s |
| r_midfield | 3588 m | 1:27.217 | 2.3 s | | smtnorth | 3578 m | 1:28.067 | 2.3 s |
| r_rome | 3959 m | 1:42.650 | 1.8 s | | smtsouth | 3291 m | 1:35.150 | 4.4 s |
| r_route11 | 4535 m | 2:19.050 | 1.9 s | | super | 2404 m | 0:45.700 | 2.5 s |
| r_route5 | 3783 m | 1:49.233 | 2.2 s | | tahitidirt | 3723 m | 1:49.667 | 2.4 s |
| r_route5_dry | 3783 m | 1:41.133 | 2.0 s | | tahitimaze | 3555 m | 2:33.100 | 4.5 s |
| r_seattle | 3863 m | 1:50.317 | 2.3 s | | testcourse | 10342 m | 2:23.183 | 1.7 s |
| r_slalom80r | 11235 m | 5:35.233 | 3.0 s | | trialmt | 3922 m | 1:42.583 | 2.3 s |

For comparison, a stock R34 GT-R in GT3 laps Trial Mountain in roughly
1:35–1:40, Deep Forest in 1:22–1:26 and Grand Valley East in 1:55–2:00 (rough
figures from memory of the game, not measured). Against the former rigid-body
physics path (capped tyre forces, since removed) the AI was up to about 4 s
slower on tarmac (midfield 1:26.8 vs 1:22.4, super 0:45.3 vs 0:43.2); scaling
its cornering-grip estimate changes little, so the difference is mostly the
car's acceleration under the spec driveline, not AI tuning. Physics measured
0.72 g peak lateral on the planar step (`sim_r34_skidpad_brake`; the old path
gave 0.78 g; being checked against the original).

Event races (6 cars, 2 laps, `sim::World` with contact and rubber band):

| event | course | lap times | contact events | position changes |
|---|---|---|---|---|
| Sunday Cup 1 | super | 1:01.9–1:04.6 | 22 | 11 |
| Sunday Cup 2 | midfield | 1:59.7–2:01.3 | 26 | 12 |
| Sunday Cup 3 | trialmt | 2:15.8–2:16.7 | 71 | 14 |
| Clubman Cup 3 | deepforest | 1:40.9–1:42.6 | 87 | 15 |

(2026-10-11, AI spec §14 / §14.1 with the corrected 20 m centre-distance gate for the
zone bytes and c (not sqrt 20 m), body extents and time-scaled closing speed, the rubber
band as a chain from car 0, car-car hits setting the recovery flags. Against the original
running the same races, contact episodes per race with
the CoG reference point and CoG extents (VehicleSetup::cog_*; grid slots move the CoG back by
the setup's CoG offset): Sunday Cup 1 3 vs 7, Sunday Cup 2 9 vs 8, Sunday Cup 3 21 vs 26 (original 17-40
over start holds), Clubman Cup 3 19 vs 10 (original 10-30 over start holds 0/1/2/120).)

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
