# AI drivers: behaviour

This page describes what the engine's computer-controlled ("CPU") drivers do: how they read
a course's AI lines, plan speeds, steer, recover from crashes, deal with other cars, start
a race, and how the race puts them on the grid, keeps them on the course, sends them to
the pits and (as a non-original fail-safe) respawns them, and how the same driver takes
over the player's car in the pit lane and after a run, and recommends the player a gear.
Code comments cite its sections as `ai §n`.

| Code | Contents |
|---|---|
| `src/ai/ai_driver.h` | `DriverParams`, `driveline_class`, `course_params`, `apply_event`, `after_finish`, `RubberBand`, `adapt_spec_for_ai`, `AiCourse`, `CarSnapshot`, `AiContext`, `AiDriver` |
| `src/ai/ai_rules.*` | the decision rules as small functions: grip, corner speed, speed controller and pit-lane sections, recovery, end of a run (§1.1, §4, §5, §7) |
| `src/ai/ai_course.cpp` | AI lines prepared for driving (§2) |
| `src/ai/ai_driver.cpp` | the per-tick driver, `AiDriver::update` (§1, §3–§8, §10) |
| `src/ai/ai_params.cpp` | parameter sets, event difficulty, the run-on, rubber band (§1.1, §9, §14) |
| `src/ai/safety_respawn.*` | the safety respawn shared by the race and `AiRace` (§12) |
| `src/ai/ai_race.*` | `AiRace`, a minimal all-CPU race used by tests and the AI screen, and `race_params` (§15) |
| `src/formats/course/ai_lines.*` | the AI-line section of a course file (§2.4) |
| `src/formats/course/driver_sets.*` | the course file's driver parameter sets (§9.1) |
| `src/race/field.cpp` | CPU cars in a race: grid, tick order, rubber band, pits, respawn, finish (§11–§14) |
| `src/race/pit.*` | the pit-lane state machine (§13) |
| `src/app/screen_race_drive.cpp` | the AI on the player's car: run-up, pit lane, end of the run, gear hint (§1, §1.1, §10, §13, §16) |

Implementation notes and measured results live in `docs/dev/AI.md`. Where this page and
that one disagree, this page follows the current code.

Conventions: one tick is 1/60 s; speeds are in m/s unless given in km/h; angles in radians
unless given in degrees; the world is Y-up; "horizontal" means in the X–Z plane. A car's
heading error and yaw rate are positive to the **right**. Values the code marks
`[placeholder]` are the engine's own conventional choices, not a description of the
original game; they are flagged as such below.

---

## §1 The driver and its outputs

One `ai::AiDriver` drives one car. It is created with the shared course
(`ai::AiCourse`, §2), a parameter set (§9) and a *start offset* (the car's grid slot,
used for the traffic start delay, §8.2), and `reset` is called once the car is placed
(§2.1). Every tick the race calls `AiDriver::update` with the car (state, spec, setup, and
from its telemetry the collision sides of the last step, the tyre wear grip factors and
the pit-surface flag), the race time (negative during the countdown), snapshots of every
car (§14), the car's own index and its rank. The driver also has a *driver kind* (below),
set by the caller with `AiDriver::set_kind` and changed by the driver itself only when the
stop driver parks the car (§1.1); the caller passes `AiDriver::kind()` on to the physics
after the update.

The driver is deterministic: `update` depends only on its arguments and its own state
(no clock, no random numbers), so a replay of the player's inputs reproduces the whole
field (the test `ai_deterministic` checks bit-identical positions of three cars after
40 s).

### Step order of one update

1. On the first update, derive the car constants: peak slip angle, downforce, braking
   floor, top speed (§6.1, §4.2).
2. Own state: horizontal forward direction, forward speed v (velocity along it), yaw rate,
   and progress s (the car's track distance, found near the previous value).
3. After the start signal, driver kinds 8 (parked) and 7 (stop with the wheels straight)
   return here (§1.1).
4. Point bookkeeping on the current line (§4.1); the line blend decays (§3).
5. Traffic inputs: zone bits and contact intensity (§8.1, §14).
6. Braking point (§4.3).
7. Target point and heading error (§3).
8. Drive decision and recovery state (§7).
9. Steering (§6). After the start signal, driver kinds 5 and 6 (the stop driver along the
   line) return here with this steering (§1.1).
10. Throttle and brake from the speed controller (§4.4, §5).
11. Traffic decisions: lane changes, lift (§8.2, §14.1).
12. Steering-saturation cut (§5).
13. Throttle slew (below).
14. Countdown override (§10).
15. Outputs.

### Outputs

| Output | Meaning |
|---|---|
| road-wheel angle | `ControlInputs::steer_angle`, radians, + = right. CPU cars set the front-wheel angle directly; the physics uses it in place of the pad steering curve and the car's lock (`sim::Vehicle::tick` clamps it to ±1 rad). The driver itself never asks for more than 45°. |
| steer | the same angle divided by the car's lock at the current speed, clamped to ±1; for displays only |
| throttle, brake | 0..1. The brake is on/off (0 or 1) except where the saturation term adds to it (§5) |
| automatic | always set: CPU cars use the automatic gearbox |
| reverse request | `AiDriver::reverse_request()`, true while the drive decision is "reverse" (§7). The caller passes it to `sim::Vehicle::set_reverse_request` before the step; with the automatic gearbox reverse is held while it is set. When reversing is wanted but the car is still in a forward gear and moving at more than 1 m/s, the throttle is cut so it stops first. |

**Throttle slew.** While the current segment is an arc (modes 1 and 2, §5) the applied
throttle moves towards the wanted value by at most 1 per second (dt per tick); elsewhere it
takes the wanted value at once.

**No AI line.** If the course has no usable line at all the driver returns full brake and
nothing else.

### The CPU car in the physics

The race builds every CPU car with `ai::adapt_spec_for_ai` (the steering lock is raised to
at least 45°, the AI's own clamp), `sim::Vehicle::set_cpu_driver(true)` and
`sim::Vehicle::set_scripted_aids` with its parameter set's aid block (§9.1). A CPU car:

- always uses the automatic gearbox;
- uses the binary CPU clutch request instead of the player's: while the clutch link is
  open, engage once the engine exceeds ¾ of the upshift rpm (rev limit − 500 if the car has
  none); once engaged, stay engaged while the engine is at or above idle. During the
  countdown the request is held at 0 (§10), and at driver kind 8 (parked, below) the rule
  does not run at all;
- reads the AI's driving-aid block (`sim::ScriptedAids`) instead of the car's own: the
  traction control and ASCC channels of its parameter set (§9.1);
- drives reverse with the accelerator (the player's automatic swaps the pedals in reverse;
  a CPU car does not).

**Driver kind.** The physics also knows a *driver kind* (`sim::Vehicle::set_driver_kind`),
set for CPU cars and for the player's car while the computer drives it. A non-zero kind
takes the same automatic gearbox and, outside a spawn or pit-stop hold, the same aid block;
kinds 1–7 also take the CPU clutch rule. Kind 8 instead holds the clutch request at 0, for
the player's car and CPU cars alike, so the clutch opens and stays open, and it holds the
car in place as the countdown's race hold does (§10). The kinds (`ai::DriverKind`):

| Kind | Meaning |
|---|---|
| 0 | normal racing |
| 1 | the AI drives on: the player's rolling / flying start run-up (and a demonstration taken over after it leaves the road), and the run-on after a finish (§1.1) |
| 2, 3, 4 | the pit lane: lane, stop line, leaving (§13) |
| 5, 6, 7 | the stop driver (§1.1): 5 along the line keeping its last brake, 6 along the line, 7 with the wheels straight |
| 8 | parked (§1.1) |

The player's run-up driver is a regular `AiDriver` with the course's set for the car's
driveline class (else the course category's built-in set) and no event overrides (§9.1,
`src/app/screen_race_drive.cpp`).

### §1.1 End of a run and the stop driver

When a run is over the computer drives the player's car (`ai::end_of_run`, run by
`src/app/screen_race_drive.cpp`; not in demonstrations, with the test autopilot, or while
the pit lane drives the car):

| How the run ended | Kind | Driven |
|---|---|---|
| race finished, time trial finished, machine test (any of the three, once it reaches its goal) | 1 | on along the main line, slowed (the run-on) |
| licence test finished, test kind 1 (stop) or 2 (section) | 6 | stopped along the main line |
| other licence test finished | 1 | on along the main line, slowed |
| licence test or time trial failed | 7 | stopped with the wheels straight |
| end of a free run, on the pit surface | 7 | stopped with the wheels straight |
| end of a free run, elsewhere | 1 | on along the main line, not slowed |

On the first tick the player's driver is reset at the car (§2.1) and given the player's
parameters with the event's overrides (§9.2), slowed where the table says so; "along the
main line" switches it to line 6, blended over 1 s (§3). This driver sees only its own car.

**The run-on** (`ai::after_finish`): the corner speed scale × 0.8 and the throttle
multiplier set to 0.5 (in place of the event's throttle, §4.3). Corner speeds already
planned stay until the next point index change (§4.1). A CPU car that finishes a race gets
the same run-on, and kind 1 if its kind was 0.

**The stop driver** (kinds 5, 6, 7; only once the race time is ≥ 0): no throttle, no
reverse request, the brake full (kind 5 keeps its last brake output instead). Kind 7 slews
the wheels to straight; kinds 5 and 6 steer as the normal driver would (§6) towards the
target point. Once the car is *parked* (3-D speed below 0.0694 m/s, 0.25 km/h, and angular
speed below 0.0524 rad/s, 3°/s) the kind becomes 8 and the brake is released on that tick.

**Parked** (kind 8, race time ≥ 0): the wheels slew back to straight and both pedals are 0;
the physics holds the car in place and keeps the clutch request at 0 (not the CPU clutch
rule, §1), so the clutch opens and stays open. No end of run chooses kind 5.

Measured (`ai_stop_driver_parks`, test car at 60 km/h on the invented stadium course):
kinds 7 and 6 brake with the throttle shut and park after 1.67 s and 1.68 s, brake then 0,
wheels straight at rest.

---

## §2 AI lines

Each course file carries seven AI lines (format: `docs/formats/course.md`, "AI lines
(slot 31)"; parser `course::AiLines::parse`). A line is a list of sparse control points
(3–91 per line on the US courses, denser in corners); from each point a straight or a circular arc runs to
the next. `ai::AiCourse` prepares them once per track; all CPU cars share it.

### §2.1 Line numbers, roles, fallback and the starting line

The driver numbers lines 1–7; line k is file line k − 1.

| Line | Role | Notes (from `gt3tool ai-survey` over the US courses) |
|---|---|---|
| 1 | line A, a passing line | full lap; mostly 1–2 m to the right of the main line |
| 2 | line B, a passing line | full lap; mostly 1–2 m to the left of the main line |
| 3 | pit lane | a lap that passes through the pit lane |
| 4 | pit stop line | part of a lap; its points are the pit markers (§13) |
| 5 | side map | never driven; a per-point flag that swaps left and right (§8.2) |
| 6 | main racing line | the line the driver normally follows |
| 7 | optional extra line | present on some courses; the driver never asks for it |

A line counts as present when it has at least two points. When the driver asks for a line,
the first present line in its row of the fallback table is used:

| Wanted | Tried in order |
|---|---|
| 1 | 1, 6, 2 |
| 2 | 2, 6, 1 |
| 3 | 3 |
| 4 | 4 |
| 5 | none |
| 6 | 6, 1, 2 |
| 7 | 7, 6 |

A number below 0 or above 7 is treated as 6; 0, like 5, has no row. If nothing in the row is present the request fails and
the driver stays on its line (`AiCourse::resolve`).

**Starting line.** `AiDriver::reset` (called on the grid, after a safety respawn, and for
the player's car when the pit lane or the end of a run takes it over) locates the car on
the lap and considers lines 6, 2 and 1, each only if it is itself present (not via
fallback). It evaluates each at the car's progress (§2.3) and takes the one whose point is
horizontally nearest the car, if within 100 m; ties go to the earlier of 6, 2, 1. If none is
within 100 m it takes the fallback for 6. Reset also clears the recovery state, the
steering, throttle and brake memory, the reverse request, the countdown pedal, the traffic
cooldown, the line blend and the pit-limit flag (§3); it keeps the driver kind.

**Line switches.** `AiDriver::set_line` switches to a line through the fallback table,
blended over 1 s (§3) or at once; the pit lane's switches are immediate, the return to the
main line at the pit exit and at the end of a run blended (§1.1, §13).

### §2.2 Control-point fields

Of each 40-byte point the driver uses:

| Field | Use |
|---|---|
| x, z | position (the file's X is mirrored: world x = −x / 4096) |
| flags byte 0 | *kind*: non-zero starts an arc, zero a straight. On the side map (line 5) it is the swap flag (§8.2) |
| flags byte 1 (signed) | brake adjustment, percent: the braking distance to this point is scaled by 1 + b/100, at least 0.1 (§4.3) |
| flags byte 2 (signed) | speed adjustment, km/h, added to the corner speed of the arc starting here (§4.2) |
| radius | signed arc radius in metres (> 0 turns left), only when flags bit 0 is set |
| heading | the stored travel direction at the point, used to build the arc (§2.3) |

A point whose radius is 0 is a straight whatever its kind byte says.

When the course is loaded each point also gets:

- **height**: the road surface under it. Where the road passes over itself the point keeps
  to the deck of the previous point (the highest surface no more than 6 m above it).
  Off the road mesh, the height of the nearest road edge.
- **progress**: its track distance, located near the previous point's.
- **surface**: the physics surface index of the road under it (0 off the mesh), which picks
  the tyre-record grip of the arc and braking plans (§4.2, §4.3).
- **slope** (uphill positive) and **bank** (positive when the road leans into this point's
  arc), both from the road normal. `[placeholder]`: a conventional derivation, not the
  original's.

**Lap lines.** A line is a lap line when its points cover more than half the lap and no gap
between consecutive points, including the closing gap from the last back to the first,
reaches half a lap. Other lines are partial (the pit stop line, and a few lines on licence
courses).

### §2.3 Position on a line at a track distance

`AiCourse::evaluate(line, s)` gives the line's position at track distance s.

1. Find the span [Pᵢ, Pᵢ₊₁) whose progress range contains s (cyclic for lap lines).
   On a partial line, a distance before its first point gives the first point, one past
   its end the last point.
2. t = (s − sᵢ) / (sᵢ₊₁ − sᵢ), measured along the lap; the height is interpolated
   linearly.
3. On a straight span the position is Pᵢ + t (Pᵢ₊₁ − Pᵢ).
4. On an arc span the position is the point at angle a₀ + t·Δa on the circle of radius |r|
   around the arc centre.

If the line is absent the centre line of the road is used.

**Building the arcs.** For an arc point Pᵢ with signed radius r and stored heading h
(travel direction (−sin h, cos h), left normal (cos h, sin h)):

- The engine first fits the arc of radius |r| through Pᵢ and Pᵢ₊₁ that leaves Pᵢ closest
  to the track's direction there; if |r| is not more than half the chord this fails and the
  span is a straight.
- It then builds the arc from the stored heading: centre C = Pᵢ + r × (left normal), a₀ the angle
  from C to Pᵢ, the end angle from C to Pᵢ₊₁, at radius |r|. The sweep direction is the
  fitted arc's, or, if the fit failed, the one the heading implies; the sweep is wrapped so
  it runs that way (it can exceed 180° in hairpins). This construction is used, replacing
  the fit, whenever |r| > 1 m and Pᵢ₊₁ lies within 25 % of |r| of the circle, i.e. when
  the stored heading is consistent with the radius.

Why both: the chord fit alone bulged up to 13 m off the road where the radius is about
half the chord (`docs/dev/AI.md`); the heading-based centre keeps the line on the road.
Measured with `ai_line_roles`: evaluated every 5 m, the main line of every lap course lies
on the road mesh for 73,601 of 73,765 samples; all the misses are on L_testcourse1, whose
mesh does not cover the whole lap.

The last point of a partial line starts no span.

### §2.4 Other users of the line data

The AI-line parser also offers, for tools and other parts of the race:

- `course::AiLines::parse` marks a line *closed* when it has at least four points and its
  ends are less than 250 m apart.
- `course::AiLine::sample_arcs` samples the straight/arc chain at a step; of the two arcs
  joining two points it takes the one continuing the previous segment's direction (the
  first span takes the minor arc). `AiLine::sample(road, step)` adds road heights
  (following the previous sample within 3 m where the road overlaps itself) and closes the
  loop when the line is closed, or when its points cover more than 60 % of the lap with a
  closing gap under 40 %.
- `track::Track::racing_line` gives a dense polyline of a line built in (track distance,
  lateral offset) space; the test autopilot's approach to the pit entry
  (`race::PitLane::lane_inputs`) follows line 3 this way. Cars in the pit lane are driven by
  their `AiDriver` on lines 3 and 4 (§13).
- `AiLine::sample(step)` (Catmull-Rom through the points) is only suitable for dense
  lines.

---

## §3 Target point

The driver steers towards a point on its line a *lookahead* L ahead of its own progress.

The first row that applies:

| Situation | L |
|---|---|
| driver kind 3 (on the pit stop line, §13) | 5 m |
| driver kind 4 (leaving the pits) while in the pit-lane limit on the last tick (on the stop line, or in the held section of the pit lane line, §4.4) | 20 m |
| recovery state 1 (reversing, §7) | 10 m |
| recovery state 2 (turning round) | 40 m |
| otherwise | L₀ + (L₁ − L₀) · min(1, v / v_L), with L₀ for v ≤ 0 |

L₀, L₁ and v_L come from the parameter set (§9.1; the built-in sets: 15 m, 40 m, 50 m/s,
and 30 m and 60 m in set 2). In the last row only, for the first 2 s after the start signal
L is lengthened by 40 m × (2 − t)/2 (the full 40 m during the countdown), so cars leaving the
grid aim far down the road.

The target is the line position at s + L. While the line blend b is above zero (after a
lane change, §8.2) the target is the blend of the old line's and the new line's position
at s + L, weighted b towards the old one; b starts at 1 and falls by 1 per second, so a
lane change is blended over 1 s. If the target is less than 5 m away horizontally, L grows
by 1 m and the target is recomputed (at most ten tries in all).

The **heading error** ε is the horizontal angle from the car's forward direction to the
target, positive to the right.

---

## §4 Speed planning

The driver works segment by segment along its line: the *current* segment (the one it is
on) and the *next* one (starting at the next control point). Straights have the target
speed v_top, arcs their corner speed.

### §4.1 Point index change

Each tick the driver finds the next control point ahead of its progress on its line. When
that index changes, the next segment becomes the current one:

- current target ← next target; mode ← the next segment's kind (0 straight, 1 arc);
- the next segment is the one starting at the new next point (the last point once past the
  end of a partial line): its kind and target are loaded;
- the tick count in the arc (for the corner bonus, §5) restarts.

On reset and after a lane change the bookkeeping is rebuilt: the segment containing the
car is loaded as current and the one after it as next.

### §4.2 Corner speed

The corner speed of the arc starting at point P (radius r, surface σ_P, bank β_P, speed
adjustment Δv_P in km/h) is worked out when the point becomes the next one (§4.1)
(`ai::corner_grip`, `ai::corner_speed`):

- **record grip** g_a(σ) of axle a = 9.8 × the tyre record's μ for surface σ: the record
  of the compound slot that surface uses (`sim::CarSpec::surface_slot`), its per-surface μ,
  or its base μ × the surface's grip factor when the record has no per-surface table;
- grip acceleration μ = w · g_f(σ_P) + (1 − w) · g_r(σ_P), w the front weight of the
  parameter set (1: front only). Off the pit surface each axle's term is multiplied by the
  mean wear grip factor of its two wheels (the live tyre wear); on the pit surface it is not;
- lateral acceleration a = μ · |r|, divided by max(1 − sin β_P, 0.2) when the bank is
  positive;
- with downforce, from v = 0, three times v ← √(a (1 + k_d v²)) (the first gives √a),
  where k_d = (front + rear downforce coefficients) / (m g);
- v_c = v · k_speed + v_off + 2.7778 · trim + Δv_P / 3.6;
- clamped to [20 km/h, max(v_top, 20 km/h)].

k_speed and v_off are the parameter set's corner-speed scale and offset (§9.1) after the
event's overrides (§9.2) and the run-on (§1.1); trim is a debug setting (± steps of
10 km/h, normally 0).

The **top speed** v_top = clamp(∛(P / c_d), 30, 120) m/s, with P the car's peak power in
watts (at least 30 kW) and c_d its drag coefficient (from the spec, or ½ρ·A_d), at least
0.1. Straights target v_top, so the driver is flat out on them.

### §4.3 Braking point

The driver checks each tick whether it must brake for the next segment. The check applies
only when the line is not the pit stop line, the car is not already in a braking mode (3 or
4), the next segment is an arc, and v exceeds its corner speed v_n.

Braking distance:

d_b = (v² − v_n²) · max(c_floor, 1 / (2 · max(μ_b, 1))) · (1 − sin θ_P) ·
max(0.1, 1 + b_P / 100)

where θ_P is the slope and b_P the brake adjustment of the next point, μ_b the front record
grip g_f on its surface (§4.2) times the mean front wear grip factor (not on the pit
surface; `ai::braking_grip`), and c_floor = 1 / (2 · max(braking deceleration in g, 0.3) ·
9.8), a floor from the car's brakes. Then:

- if the brake factor k_brake (§9) is below 1, d_b is multiplied by it (later braking);
- if the traffic cooldown is 0, 20 m × c is added (c the contact intensity, §8.1).

D is the horizontal distance from the car to the next point.

- **D ≤ d_b**: braking starts: mode 3 if the current segment was a straight, 4 if it was
  an arc; status 2. The braking mode lasts until the next point index change.
- **otherwise**, with the *soft margin* m_s = d_b · (k_brake − 1) when k_brake > 1 (else
  0) and the free distance D − d_b: status 1 if D − d_b < m_s + 50 m; and if
  D − d_b < m_s the throttle is multiplied by (D − d_b) / m_s, so a timid driver lifts
  gradually before its braking point.

Finally, except on the pit lane line, the throttle multiplier is multiplied by the event's
throttle percentage (§9.2), or by 0.5 during the run-on (§1.1).

### §4.4 Speed controller

Wherever the driver holds a speed it uses one controller. For a target speed v_t and gain K
(per m/s):

e = clamp(v − (v_t − 2 km/h), −10 km/h, +10 km/h)
throttle = clamp(0.75 + K · e, 0, 1)
brake = 1 if the throttle is 0 and v > v_t, else 0

So the car holds about 2 km/h under the target with three-quarter throttle, lifts above
it, and brakes fully only when it is over the target with the throttle already shut.

| Where | Target | K |
|---|---|---|
| braking from a straight (mode 3) | v_n | −2 |
| braking in an arc (mode 4) | v_n | −1 |
| in an arc (modes 1, 2) | current corner speed + corner bonus (§5) | −0.5 |
| recovery state 2 (turning round, §7), whatever the mode | 50 km/h (on the pit lines see below) | −0.5 |

On the pit lines the plan changes (`ai::plan_speed`):

| Where | Plan |
|---|---|
| on the pit stop line (4) | mode 3 at 20 km/h (K = −2) |
| on the pit lane line (3), section 1 | the braking modes 3 and 4 aim at the pit speed + 2 km/h instead of v_n |
| on the pit lane line (3), section 2 | arc mode 1 at the pit speed + 2 km/h (K = −0.5) |
| on the pit lane line elsewhere | as on any line |

The sections come from the pit stop line's points, the pit markers (`ai::pit_lane_section`,
`ai::pit_markers`; none when the stop line has fewer than 5 points): section 1 runs from its
point 1 (the pit entry) to point 2, section 2 from point 2 to point n − 3 (n points).
Distances are compared lap-unwrapped: from the first stop-line point in the first half of
the lap on, the markers have a lap length added, and so does the car's track distance when
it is in the first half. The pit speed is the parameter set's (§9.1). "Pit-lane limit" (§3)
means the stop line or section 2. In state 2 the target is 20 km/h on the stop line, the
current segment's target in section 1 and the pit speed + 2 km/h in section 2, always with
K = −0.5.

The result is multiplied by the throttle multiplier of §4.3.

---

## §5 Modes, corner bonus and steering saturation

| Mode | Meaning | Throttle |
|---|---|---|
| 0 | straight | full (× the throttle multiplier) |
| 1 | arc | speed controller at the corner target |
| 2 | arc (handled like 1; no control point produces it, since kinds are 0 or 1) | as mode 1 |
| 3 | braking, entered from a straight | speed controller at v_n, K = −2 |
| 4 | braking, entered in an arc | speed controller at v_n, K = −1 |

**Corner bonus.** In an arc, once the car has been in it for the parameter set's bonus
delay the target is the corner speed plus the set's corner bonus (20 km/h with no delay in
every built-in set), except on the pit surface, where there is no bonus. The driver
deliberately aims above its planned corner speed and lets steering saturation hold it
back. The ticks in the arc count in modes 1, 2 and 4 (braking inside an arc keeps
counting) and restart at a point index change (§4.1), in modes 0 and 3, while braking or
reversing (§7) and in recovery state 2.

**Status.** The traffic logic (§8.2) reads a status the speed logic sets each tick: 0
nothing; 1 the braking point is near (§4.3); 2 braking; in an arc, when neither 1 nor 2 is
set: 4 at full throttle (mode 1), 3 at full throttle in mode 2 (therefore never seen), 5 not
at full throttle.

**Steering saturation.** When the steering asks for more front slip than the tyre's peak
(§6) the saturation S = |α_wanted| / α_peak exceeds 1. From 2nd gear up:

throttle ← throttle · max(0, 1 − K_t (S − 1)), brake ← min(1, brake + K_b (S − 1))

with K_t = 1 and K_b = 0 in every built-in set: the driver lifts in proportion to how far
it is past the grip limit, and does not brake for it.

---

## §6 Steering

Steering is worked out only while the car moves (|v| > 2 km/h). Below that the wheels are
steered back to straight. The recovery state used here is the one after this tick's
transition (§7).

**Driving forward.**

1. If the car is in a recovery state (1 or 2) and |ε| > 80°, the wanted front slip angle
   is the tyre's peak slip angle α_peak, towards the target.
2. Otherwise the driver wants the yaw rate that would carry it on a circle through the
   target (pure pursuit): r_des = 2 · v_s · y / d², where y is the target's sideways offset
   from the velocity direction (the heading below 5 km/h), d its distance (d² at least
   1 m²) and v_s the horizontal speed, negative when moving backwards. With the actual
   yaw rate r, the wanted slip angle is α = K_y (r_des − r), but only when the car is
   turning too little in the wanted direction (r_des > 0 and r < r_des, or r_des < 0 and
   r > r_des); otherwise α = 0. K_y = 1.2 (rad of slip per rad/s). `[placeholder]` an
   option, off by default, also steers against an overshoot.
3. If |α| > α_peak it is clamped to ±α_peak and the saturation S = |α|/α_peak is passed
   to §5.
4. The road-wheel angle target is α + β, clamped to ±45°, where β is the direction of the
   velocity at the front axle relative to the car (the steer angle that gives zero front
   slip): the velocity plus the yaw-rate contribution at the front axle's distance from the
   reference point; β = 0 when the axle's forward speed is under 0.5 m/s.

**Braking or reversing.** The target is ±30° when |ε| > 20°, otherwise 1.5 ε; when the car
is moving backwards the sign is reversed.

**Slew.** The commanded angle moves towards the target by at most the steering rate times
dt: 90°/s in every built-in set.

### §6.1 Grip and peak slip angle

On its first update the driver derives the **peak slip angle** α_peak from the car's
front tyre: the whole-degree angle from 0° to 45° at which the front tyre's lateral curve
is highest (the first such angle), at most 0.8 rad and at least 2°.

The **grip** is not fixed: each arc's corner speed and each braking distance look up the
tyre records' grip for the point's surface, with the live wear factors, when they are
planned (§4.2, §4.3).

The car's braking floor (§4.3), downforce factor and top speed (§4.2) are fixed on the
first update.

---

## §7 Drive decision and recovery

Each tick the driver decides to drive forward, brake, or reverse, and keeps a three-state
recovery machine: 0 racing, 1 reversing, 2 turning round.

**Heading threshold** θ: 60°, except in state 1, where it is 10° when v ≥ 0 and
10° + 50° · (−v)/(30 km/h) while reversing slower than 30 km/h (so it opens up to 60° as
reverse speed builds).

**Drive decision.**

| Condition | Decision |
|---|---|
| \|ε\| < θ, or state 2 | forward if v > −2 km/h, else brake |
| otherwise | brake if v > 2 km/h, else reverse |

The reverse request (§1) is "decision = reverse".

**Collision flags.** The state machine reads two flags, the *front hit* and the *rear hit*:
the collision sides of the car's last collision step (`sim::VehicleTelemetry::hit_sides`,
`sim::collide::HitSide`). A wall impulse at one of the car's collision octagon points 0–3
is a front hit, at points 4–7 a rear hit (sensor points do not count). In a car-car
response the car whose vertex hit gets that vertex's end, and the other car the end of the
edge that was hit: edges 0–2 front, 4–6 rear, the side edges 3 and 7 neither (§14).

**State changes** (`ai::recovery_next`). There are no timers: each tick the state follows
from the heading error, the forward speed and the two flags.

| From | To | When |
|---|---|---|
| 0 | 2 | \|ε\| > 80°, or a rear hit with \|ε\| > 45° |
| 1 | 0 | \|ε\| < θ and v > 3 km/h |
| 1 | 2 | \|ε\| < θ and v ≤ 3 km/h, or \|ε\| ≥ θ and a rear hit |
| 2 | 0 | \|ε\| < 40° and v > 30 km/h |
| 2 | 1 | otherwise, a front hit with \|ε\| > 60° |

Otherwise the state stays. The drive decision and the lookahead (§3) use the state before
this tick's change, steering (§6) and the speed plan (§4.4) the state after it. In state 1
the lookahead is 10 m, in state 2 40 m (§3); in state 2 the car drives forward at 50 km/h
(§4.4).

Measured (`ai_synthetic_recovers`, invented stadium course): a car placed facing backwards
turns round (state 2) and races on; a car driven nose first into a wall at 115° or 90°
gets a front hit, reverses (state 1) and drives on along the lap (338 m and 350 m in 20 s).

---

## §8 Traffic

### §8.1 Traffic inputs

Every tick the driver compares itself with every other car (the pairing rules are in §14)
and builds two inputs.

**Zone bits.** The other car's centre is expressed in this car's horizontal frame: u
forward, ℓ to the right. With the combined extents l_f = own front + other's rear,
l_r = own rear + other's front and W = the sum of the half widths:

| Where the other car is | Bits set |
|---|---|
| ahead (u > l_f), ℓ < −W | front-left |
| ahead, ℓ > W | front-right |
| ahead, overlapping sideways | front, and front-right if ℓ > 0 else front-left |
| behind (u < −l_r), ℓ < −W | rear-left |
| behind, ℓ > W | rear-right |
| behind, overlapping sideways | rear, and rear-right if ℓ > 0 else rear-left |
| alongside, ℓ < 0 | left, and front-left if u > 0 else rear-left |
| alongside, ℓ ≥ 0 | right, and front-right if u > 0 else rear-right |

The bits are front 1, front-left 2, front-right 4, left 8, right 16, rear-left 32,
rear-right 64, rear 128; bits from all nearby cars are combined. The rear bits are kept but
no decision reads them.

**Contact intensity** c ∈ {0, 0.25, 0.5, 1}: for a car ahead and overlapping sideways
(|ℓ| ≤ W) with a bumper gap p = u − l_f > 0 that this car is closing on (closing speed
q > 0, the difference of the two cars' forward velocities each multiplied by its
simulation time scale, §9.3):

| Condition | c |
|---|---|
| q > 10 m/s | 1 |
| q > 1 km/h and p / q < 3 s | 0.5 |
| otherwise, p < 2 m | 0.25 |

c is the largest over all cars. It adds 20 c metres to the braking distance (§4.3) from the
first tick, and drives the decisions below.

### §8.2 Traffic decisions

Traffic decisions start 5 s + 0.1 s × (start offset + 1) after the start signal, so the
field leaves the grid on its lines and spreads out first; the start offset is the car's
grid slot. They are taken only while the drive decision is "forward", not on the pit stop
line, and not during the cooldown. The exact order is §14.1.

**Left and right lines.** From the main line or line A, *left* is line B; from the main
line or line B, *right* is line A. Where the side map (line 5) flags the current span
(its kind byte non-zero at the car's progress) the two are swapped. There is no left line
from line B and no right line from line A (unswapped).

**Passing.** With contact (c > 0) and lane changes allowed, the driver tries the left line
unless something is front-left, else the right line unless something is front-right.
Before an arc it prefers the left line; on a straight it prefers it too unless status is 3.
The right line is not taken at status 4. A successful change blends to the new line over
1 s (§3) and starts a cooldown of 30 ticks (0.5 s).

**Lift.** If no passing line was taken, the throttle is multiplied by (1 − c).

**Alongside and back.** If nothing is front, front-left, front-right, left or right, a car
on a passing line returns to the main line at once (no blend). If a car is alongside on
exactly one side, the driver moves to the line on the other side (not to the right at
status 4, not to the left at status 3).

**Pit lane.** On the pit lane line the driver never changes lanes; it halves its throttle
when a car is both left and front-left, or both right and front-right.

---

## §9 Driver parameters

### §9.1 Built-in parameter sets

A parameter set is stored as 32 bytes, of which bytes 0–24 are used
(`ai::DriverParams::from_bytes`; layout also in `docs/formats/course.md`, "Driver parameter
sets"). The engine has three built-in sets (`DriverParams::builtin`):

| Byte | Meaning | Conversion | Set 0 | Set 1 | Set 2 |
|---|---|---|---|---|---|
| 0 brake % | braking distance factor | k_brake = 100 / % (0 read as 1) | 100 | 100 | 100 |
| 1 speed % | corner speed scale | k_speed = % / 100 | 100 | 100 | 100 |
| 2 speed offset | corner speed offset | (byte − 100) km/h | 100 | 100 | 100 |
| 3 yaw gain | K_y (§6) | byte × 0.05 | 24 | 24 | 24 |
| 4 saturation throttle | K_t (§5) | % / 100 | 100 | 100 | 100 |
| 5 saturation brake | K_b (§5) | % / 100 | 0 | 0 | 0 |
| 6 lookahead speed | v_L (§3) | byte × 2 km/h | 90 | 90 | 90 |
| 7 lookahead min | L₀ (m) | byte | 15 | 15 | 30 |
| 8 lookahead max | L₁ (m) | byte | 40 | 40 | 60 |
| 9, 10 ASCC A gain / threshold | aid block | × 0.01 | 15, 5 | 15, 5 | 0, 0 |
| 11, 12 TCS lateral a / b | aid block | × 0.1 / × 0.01 | 10, 0 | 10, 0 | 10, 0 |
| 13 TCS gain | aid block | × 0.1 (0 = off) | 60 | 60 | 60 |
| 14 grip weight | w (§4.2) | % / 100 | 100 | 100 | 100 |
| 15 corner bonus | km/h (§5) | byte | 20 | 20 | 20 |
| 16 steering rate | slew (§6) | byte × 10 °/s | 9 | 9 | 9 |
| 17 bonus delay | ticks (§5) | byte × 10 | 0 | 0 | 0 |
| 18 pit speed | km/h (0 → 80) | byte | 80 | 80 | 80 |
| 19 ASCC channel flags | aid block | bits 0–7: A-front, A-rear, D-front, D-rear, C-front, C-rear, E-front, E-rear | 0 | 0 | 0 |
| 20 ASCC D gain | aid block | × 0.01 | 0 | 0 | 0 |
| 21, 22 ASCC C gain / threshold | aid block | × 0.01 | 0, 0 | 0, 0 | 0, 0 |
| 23 ASCC E gain | aid block | × 0.01 | 0 | 0 | 0 |
| 24 TCS slip scale | aid block | × 0.01 (0 and 100 → 1) | 0 | 0 | 0 |

So every built-in set uses K_y = 1.2, v_L = 180 km/h (50 m/s), 90°/s steering, a 20 km/h
corner bonus and traction control with gain 6.0, and no ASCC (no channel flag set); set 2
looks further ahead (30–60 m).

**Which set** (`ai::course_params`). A car gets the course file's set for its *driveline
class* (course file slots 37–40, slot 37 + class; `course::DriverSets`,
`track::Track::driver_sets`). Only where that slot is empty does it get the built-in set of
the course category (course table 34, byte +0x08): 1 (the wet Route 5 courses) → set 1, 2
(dirt) → set 2, anything else → set 0 (`ai::builtin_set`, `race::driver_param_set`;
`FieldConfig::course_category`). A class outside 0–3 reads the class 0 slot.

Driveline class (`ai::driveline_class`): front-engine/mid/rear rear-drive 0; front-drive 1;
four-wheel drive by its system: system 0 → 1, system 3 → 3, others → 2. The same class
picks the event overrides (§9.2).

Who uses it: CPU cars in a race and in `AiRace` (`ai::race_params`) get the course set and
then the event's overrides (§9.2). The player's car gets the course set without overrides
for the rolling / flying run-up and the pit lane, and with the race's or licence test's
overrides at the end of a run (§1.1).

Measured: 79 of the 80 US course files carry a set for all four classes; L_circle60_wet1
has none and falls back to the built-in set; every set's pit speed is 78 or 80 km/h
(`ai_course_param_sets_disc`). Trial Mountain's class 0 set, for example: brake 115 %
(k_brake ≈ 0.87, later braking), lookahead 18–50 m, bonus delay 30 ticks, pit speed 78 km/h,
TCS gain 5.0 with slip scale 0.5, and the ASCC C-front channel on (gain 1.0, threshold
0.1). The category choice alone: Trial Mountain → 0, Route 5 → 1, Tahiti Dirt → 2
(`race_driver_param_set_disc`).

**Aid block.** For CPU cars, and for non-zero driver kinds outside a spawn or pit-stop
hold, the physics reads the AI driving-aid block instead of the car's own: the set's TCS
gain, lateral terms and slip scale, and its ASCC channels, each acting only when its flag
bit is set and its gain is non-zero (`DriverParams::aids` → `sim::Vehicle::set_scripted_aids`,
`sim::ScriptedAids`). The race sets it for each CPU car from that car's parameters, and for
the player's car from the player's course set. ABS is not part of the block.

### §9.2 Event difficulty

An event record (database table 32) overrides the set per *driveline class* c (0–3):

| Bytes | Meaning | Default when | Effect |
|---|---|---|---|
| +0x1A4 + c | throttle % | 0 or > 100 → 100 | throttle_scale = % / 100: every throttle except on the pit lane line is multiplied by it (§4.3) |
| +0x1A8 + c | brake % | 0 → 100 | k_brake × 100 / %: below 100 the braking point stays but the driver lifts gradually before it (§4.3); above 100 it brakes later |
| +0x1AC + c | corner speed % | 0 → 100 | k_speed × % / 100 |
| +0x1B0 + c | corner speed offset, signed km/h | 0 = none | added to v_off |

The overrides apply on top of the car's set (§9.1); the driveline class is
`ai::driveline_class` (§9.1).

Example (`ai_event_overrides`): Sunday Cup race 1 gives throttle 85 %, corner speed 80 %,
brake unchanged. In the start measured in §10 the Sunday Cup cars leave the line at 0.85
throttle.

### §9.3 Rubber band

The event can make CPU cars run their simulation faster or slower depending on how far they
are from the reference car (the leading human). The factor scales the car's simulation step:
the car is integrated with dt × factor each tick (`sim::World::set_time_scale`), so it
literally moves in faster or slower time.

| Bytes | Meaning |
|---|---|
| +0x1B4 | global scale % (0 → 100) |
| +0x1B7 | slow-down amount A_s, % |
| +0x1B8 | slow-down start, m |
| +0x1B9 | slow-down full, × 10 m |
| +0x1BA | speed-up start, m |
| +0x1BB | speed-up full, × 10 m |
| +0x1BC | speed-up amount A_f, % |

For a gap g = (reference's race distance − car's race distance), laps included:

- g > 0 (car behind): factor = 1 + A_f · ramp(g, start_f, full_f)
- g ≤ 0 (car ahead): factor = 1 − A_s · ramp(−g, start_s, full_s)

where ramp(x, a, b) is 0 for x ≤ a, 1 for x ≥ b and linear between. The band is enabled
when either amount is non-zero or the global scale is not 100 %. Sunday Cup: slow-down 20 %
from 20 m to 80 m ahead, speed-up 10 % from 50 m to 100 m behind (so 50 m ahead runs at
0.9, 500 m ahead at 0.8, 75 m behind at 1.05; `ai_rubber_band_curve`). How the factors are
chained across the field is §14.

---

## §10 Countdown and start

During the countdown (race time < 0) every car is held by the physics' race hold: no
horizontal motion, brakes off, the engine free to rev. Every car of the field gets the hold
alike (`src/race/field.cpp`). A CPU car additionally:

- has its clutch request held at 0 (`sim::Vehicle::set_clutch_hold`), so it revs with the
  clutch open;
- steers straight and uses no brake;
- **blips the throttle** around a reference rpm R (the car's upshift rpm, or rev limit −
  500). With n the frames of countdown left and r the car's rank (its index among the
  opponents: the grid order, pole first, unless the GT-mode grid is sorted by
  power-to-weight):
  - while n ≥ 13: a hysteresis switch turns *on* below 0.75 R and *off* at or above
    (15/16) R; while on the pedal rises by (5 − 0.2 r) per second up to 1, while off it
    falls by (10 − 0.4 r) per second down to 0;
  - in the last 12 frames: pedal = clamp(0.625 − 0.002 r − 2 (rpm − R) / rev limit, 0, 1),
    a proportional hold at about R;
- keeps its applied throttle memory at 0, so at GO the speed controller's pedal applies at
  once.

At GO the clutch hold and race hold end; the CPU clutch rule (§1) engages the clutch from
the revs.

Measured (scratch probe, Sunday Cup 3 field on Trial Mountain, 3 s countdown): the CPU cars
stay within 0.01 m of their slots; the pole car's engine swings between about 4300 and
5700 rpm with the pedal switching between 0 and 1, rises from about 5100 to 5800 rpm with the
pedal easing from 0.94 to 0.71 over the last 12 frames, and takes 0.85 (the event's throttle)
the tick after GO, rolling within three ticks.

**Rolling start** (GT endurance events): the field is placed on the grid already moving
at 80 km/h (`FieldConfig::start_kmh`, `race::kRollingKmh`) with no countdown and no hold;
the player's car is driven by a run-up `AiDriver` (driver kind 1) for the first 5 s.
Demonstration races of start type 0 with an initial speed start every car moving at that
speed, also with no countdown.

**Flying start** (time trials and moving licence tests): the player's car is placed on the
main line the event's distance before the start line at its initial speed, and the same
run-up driver (kind 1) drives it until it crosses the line, which starts the run.

---

## §11 CPU cars in the race field

`race::RaceField::create` (`src/race/field.cpp`) builds a race with opponents in one
`sim::World` (car-to-car contact, slipstream, per-car time scale).

**Who races.** The configured opponent labels, or else the event's entries, cut to
`max_opponents` (the event's car count − 1, at most 5). A car whose spec cannot be built is
skipped with a warning.

**Grid.** Opponents take slots in the given order or, in GT mode, by power-to-weight:
lowest kg per PS on pole (PS from the peak of torque × rpm on the full-throttle curve).
The player takes the slot after the opponents (the last slot if the grid is shorter)
unless a slot is configured. Each car's slot point marks its nose: the car is moved back
along its heading by its front extent. An opponent beyond the grid's slots is placed on the
centre line 12 m × its index before the line.

**Per car.** Spec adapted for the AI (§1), CPU driver on, placed at rest without the spawn
hold (standing start) or moving (§10); parameters = the course's set for the car's
driveline class, else the course category's built-in set (§9.1), with the event's
overrides for that class (§9.2), and the set's aid block given to the physics; an
`AiDriver` with the slot as start offset, reset on the grid (§2.1); a lap timer whose lap 1
starts at GO; a pit-lane state machine if tyre wear is on (§13). When the race has tyre
wear every car, CPU or human, starts on a fresh set; car collisions are always on.

**Each tick.**

1. Snapshot every car (§14).
2. After GO, with the rubber band enabled, compute the chain of time scales (§14).
3. For each CPU car, in field order: build its context (race time, snapshots, rank = its
   index); set the clutch hold and race hold while the race time is negative; run the
   driver; pass its reverse request and driver kind; run the pit-lane state machine, which
   can switch the driver's line and kind and hold the car during its stop (§13); set its
   time scale from the rubber band chain (1 when the band is off).
4. The human cars' inputs (held by the race hold during the countdown).
5. Step the world; advance the race time.
6. Lap timers and the safety respawn (§12); finishes, judged in the last race order (a CPU
   car that finishes gets the run-on, §1.1); the race order.

Remote players' cars (online) are posed from the network and have no driver.

---

## §12 Safety respawn (not in the original)

The original game never respawns a CPU car. The engine has a fail-safe, on by default
(`[race] respawn = cpu`, or `all`, which also covers the player's car; `off` disables it;
`FieldConfig::cpu_respawn`, kept in a race replay;
`AiRace::set_respawn` in §15). One rule (`ai::SafetyRespawn`, `ai::respawn_car`) serves
the race field and `AiRace`.

After the car's step, before its lap timer update, a CPU car is *lost* when

- the centre and all four wheel contact points are off the road mesh (no road surface
  below a level 3 m above the point, or the point more than 5 m below the surface found)
  for more than 30 ticks (0.5 s); a
  single probe can fall through a hairline crack between road triangles, hence all five;
- or its position is not finite;
- or, after GO, its race distance has not grown by 10 m for 12 s, counted from GO.

A lost car is put on the centre line at its current lap distance, facing along the lap,
with the physics' spawn hold (`sim::Vehicle::spawn`), its lap timer moved with it, and its
driver reset (§2.1); the off-mesh count and the 12 s start again. After the timer update
the race distance is recorded: a gain of more than 10 m over the best so far restarts the
12 s. Time in the pit lane (while the pit state machine drives, §13) does not count as lack
of progress.

Measured: in `ai_laps_all_courses` (one car, two laps on all 36 lap courses) one car
needed a respawn, on r_tahitimaze (12.0 s without progress); the longest time without 10 m
of progress elsewhere was 8.4 s (tahitimaze). In `ai_event_races` (four 6-car events) no
car needed one; the longest was 8.6 s (Sunday Cup 2). Without the disc,
`race_field_stuck_respawn` checks the progress rule on a car that cannot move: the field
re-places it at race time 12.0 s and again at 24.02 s, 3 m from its grid slot, on the centre
line 0.3 m above the road (the spawn hold); `ai_race_stuck_respawn` gets the same for
`AiRace` at 12.0 s, and no respawn with it turned off.

---

## §13 CPU pit stops

CPU cars pit only in races with tyre wear on and only on courses whose AI lines 3 (pit
lane, at least 4 points) and 4 (pit stop line) exist, the stop line having at least 6 + n
points (n = the opponent's index + 1; the player is 0). The car is driven by its own
`AiDriver` on the lines and driver kinds a state machine (`race::PitLane`) asks for. The
state machine uses the stop line's points as markers, by their track distance:

| Marker | Stop-line point |
|---|---|
| entry for a CPU car with a pit request | 0 |
| entry from the pit surface | 1 |
| box of car n | N − 4 − n |
| exit | N − 2 |

The box heading is the direction from the box point to the next one.

**States** (also the driver kind, §1): 0 normal, 2 pit lane, 3 stopping, 4 leaving. Each
change can switch the AI driver's line: at once, except the return to the main line,
which is blended over 1 s (§2.1). The state machine runs after the driver's update, so a
switch takes effect from the next tick.

1. **Arming.** A car is armed when the physics reports its tyres worn (a wheel's wear
   grip below the worn threshold). An armed car enters when it next passes stop-line point
   0, unless its race is over (it has finished, or is on its last lap). Any car that crosses
   point 1 on the pit surface also enters. Entry (state 2, line 3) is reported ("<car>
   Pitting in...").
2. **Lane** (2, line 3). 25 m before its box the car decides: it stops (state 3, line 4) if
   its race is not over, some tyre has wear, and it is slower than 100 km/h; otherwise it
   drives through (state 4, still line 3). A car that entered past the decision point
   returns to normal at the exit.
3. **Stopping** (3, line 4). The driver runs the stop line at 20 km/h with a 5 m lookahead
   (§3, §4.4); crossing the box starts the physics' pit stop
   (`sim::Vehicle::begin_pit_stop`: a 5 s hold with fresh tyres fitted during it). While
   held, the car's inputs are replaced by full brake and its reverse request is cleared.
4. **Leaving** (4, line 3). Once the stop is over and the car is 0.5 m past the box it
   takes the pit lane line again. At the exit it returns to state 0 (kind 0) and to the main
   line (6, blended), and its recovery state is reset to 0 (`AiDriver::reset_recovery`).

On line 3 the driver holds the parameter set's pit speed + 2 km/h through the pit-lane
sections (§4.4: 78 or 80 km/h + 2 with the disc's course sets), with a 20 m lookahead while
leaving in the limit (§3); on the pit surface it gets no corner bonus (§5) and plans
without the wear factors (§4.2, §4.3). On the pit lane line it does not change lanes (§8.2).

The car is not "lost" while the state machine drives (§12). The race sets the time scale
to 1 then, but sets the chain's value (§14) on the same tick, so the rubber band still
applies; on the pit surface the car only loses its own speed-up link.

**The player's car.** In a race with a pit lane the player's car runs the same state
machine (its box is N − 4) and, while it drives, the player's own `AiDriver` (the course's
set for the car's class, no event overrides, §9.1), reset at the car on entry and seeing
only its own car; its race is over once the player has finished. The driven inputs are
recorded, so a replay re-runs the stop.

Measured (`ai_cpu_pit_stop_disc`, one CPU car on Trial Mountain armed at once): it enters at
91.2 s, reaches the stop decision at 80.6 km/h (the set's pit speed 78 + 2), goes into the
box at 18.9 km/h and stops at 111.4 s, leaves at 117.1 s and is racing again at 124.0 s,
with driver kinds 0, 2, 3 and 4.

---

## §14 Car-to-car interaction and the rubber band chain

**Snapshots.** Each tick, before any driver runs, the race records for every car: its
centre of gravity (the car origin plus the rotated CoG offset; this is the car's reference
point), orientation, velocity, half width, front and rear extents measured from the CoG,
and its current time scale.

**Pairs.** A driver compares itself with another car only when their centres are less than
20 m apart and differ by at most 5 m in height. From each pair it takes the zone bits and
the contact intensity of §8.1, using the extents from the CoG and the time-scaled closing
speed.

**Hits.** A car-car collision response adds its sides to both cars' collision sides for
that step (`sim::Vehicle::add_hit_sides`): the car whose vertex hit gets that vertex's end,
the other the end of the edge that was hit. These are the front and rear hit flags of §7,
the same as a wall's. Measured (`ai_collision_sides`): a car running into another's tail
gets a front hit, the car ahead a rear hit.

**No following.** The engine implements no following brake and no speed matching: a car
caught behind a slower one lifts (throttle × (1 − c)), lengthens its braking distance by
20 c m, looks for a passing line, and otherwise relies on contact.

**Rubber band chain.** The time scales of §9.3 are applied as a chain along the race order
from the reference car R (the code notes this as pass mode 0, used by every event on the
disc; no other mode is implemented):

1. Rank every car by race distance, laps included (ties keep their order).
2. R's own factor is 1.
3. Going forward from R, each car ahead gets factor(gap to the car just behind it) times
   that car's factor.
4. Going back from R, each car behind gets factor(gap to the car just ahead of it) times
   that car's factor; a car on the pit surface gets no speed-up of its own (its link is 1)
   but still inherits its neighbour's.
5. Every car except R is then multiplied by the global scale.

So the factors compound outwards: a car two places behind a slow-down chain is affected
through its neighbours, not by its own distance to R alone. In a race the reference is
player 1, or player 2 when ahead; the chain runs only after GO; the human cars' own scales
are not taken from it.

### §14.1 Order of the traffic decision

Per CPU car per tick, after the speed logic (§4, §5) and steering (§6):

1. Count the cooldown down by one tick.
2. Continue only if the drive decision is "forward", the start delay of §8.2 has passed,
   the car is not on the pit stop line and the cooldown is 0.
3. Lane changes are allowed unless the status is 1, 2 or 5.
4. On the pit lane line: halve the throttle when boxed in on one side (left and
   front-left, or right and front-right); lane changes are not allowed.
5. With contact (c > 0) and lane changes allowed: choose a passing line as in §8.2 (the
   side map decides which line is left); if the change succeeds, start the 0.5 s cooldown
   and stop here.
6. No line chosen: throttle × (1 − c). Statuses 1, 2, 5 and the pit lane stop here.
7. Find the nearby zones: front, front-left, front-right, left, right.
8. Find whether exactly one of left / right is occupied.
9. Moves: nothing nearby → back to the main line at once (if not on it); one side occupied
   → move to the line on the other side (not right at status 4, not left at status 3).

---

## §15 The all-CPU test race (`ai::AiRace`)

`AiRace` runs N CPU cars with no human, for the tests and the developer screen
`gt3r --screen ai`. It uses the same rules as the race field: each entry carries its
parameters, which the tests and the AI screen take from `ai::race_params` (the course's set for the car's driveline class, else the
category's built-in set, then the event's overrides when an event is given, §9.1, §9.2),
the set's aid block goes to the physics, and the safety respawn is the shared rule of §12.
It differs in a few ways:

- cars take grid slots in entry order; extra cars start 10 m × index before the line;
  standing start, no countdown (the race time starts at 0, so the 12 s of §12 count from
  the start);
- the rubber band reference is a chosen car (car 0 in the tests stands in for the player);
- rank = car index, start offset = car index;
- the dirt-course front-brake cut is set from the course category, as for any car;
- no pit stops and no run-on after a finish;
- the safety respawn can be turned off with `AiRace::set_respawn(false)`;
- it counts contact events (a pair starting to touch), nose-to-tail contacts, and
  position changes after the first 10 s.

Measured with the disc (`./build/gt3r_tests ai_`, all pass): one `r34_gtr_v_99` with the
course's set for its class completes two laps on all 36 lap courses, with one respawn (on
r_tahitimaze, §12) (e.g. Trial Mountain 1:40.3, Deep Forest 1:27.5, Grand Valley 2:11.8,
Super Speedway 0:45.1); in the 6-car events, Sunday Cup 1–3 and Clubman Cup 3, every car
finishes two laps with no respawn, with 85–135 car-car contact events and 12–16 position
changes per race. These lap times move whenever the physics changes.

---

## §16 Recommended-gear hint

The HUD's recommended gear (presentation §D2) comes from a second `AiDriver` on the player's
car that only watches: it has the course's set for the car's driveline class without event
overrides (§9.1), sees only its own car, and its outputs are discarded
(`update_gear_hint`, `src/app/screen_race_drive.cpp`; not during replays). Each tick its
point bookkeeping and braking check (§4.1–§4.3) give a target speed T: the next segment's
speed (v_top for a straight), or in braking mode 4 the lower of the current and the next.
When the car is at or above T the hint is the automatic gearbox's gear for T
(`sim::model::gear_for_speed`), blinking while the watcher is in a braking mode (3 or 4);
below T there is no hint.
