# AI drivers

The CPU drivers are part of GT3's race step on the exact core
(docs/dev/PHYSICS.md): the AI driver runs in phase 3 of each car's step, on
the car's own record, in the PS2's arithmetic. The behaviour is described in
[docs/behaviour/ai.md](../behaviour/ai.md) ("ai §n" below names its sections);
this page says where it lives.

| File | Contents |
|---|---|
| `src/sim/exact/tick_ai.cpp` | the AI's step: drive decision and recovery, modes, steering, throttle and brake, traffic and lane changes, the end-of-run drivers, the gear hint (ai §1, §5–§8, §10, §16) |
| `src/sim/exact/tick_ai_plan.cpp` | the plan: next-point bookkeeping, corner speeds, braking point, target point and speed controller (ai §3, §4) |
| `src/sim/exact/ai_course.h/.cpp` | the AI course object: the lines as the race reads them and their evaluation by progress (ai §2) |
| `src/sim/exact/ai_setup.h/.cpp` | the AI course object as the race's set-up loads it from the course file |
| `src/sim/exact/tick_input.cpp` | the AI paths' apply: the driver's outputs into the car's inputs |
| `src/sim/exact/tick_proximity.cpp` | proximity and the zone bytes the traffic rules read (ai §8, §14) |
| `src/sim/exact/tick_race.cpp`, `race.cpp`, `race_flow.*` | the rubber band, the race order, driver kinds at the finish and the end of a run (ai §1.1, §9.3, §11, §14) |
| `src/sim/exact/pit_lane.h`, `tick_pit.cpp` | the pit lane's state machine and the AI's pit switches (ai §13) |
| `src/sim/exact/start.h/.cpp` | the grid, the driver parameter set and the event's overrides, the AI's line and plan at the start, a player's run-up by the AI (ai §9, §10) |
| `src/formats/course/ai_lines.*`, `driver_sets.*` | the course file's AI lines and driver parameter sets (docs/formats/course.md) |
| `tools/gt3tool/cmd_ai.cpp` | `gt3tool ai-survey [prefix]`: line roles and flag statistics |

## The player's car

The race's own AI drives the player's car when the race says so: the run-up of
rolling and flying starts, after the finish, the end of a failed run, the pit
lane, and the license demonstrations. The race screen's autopilot (the
`auto=1` race preset, online's autopilot, the quick-menu demonstrations) asks
the race to let its AI drive the player's car (`RaceVehicle::ai_take_over`,
driver config 3). The HUD's recommended gear is the AI plan's gear for the
speed it aims at (ai §16), read from the player's driver record.

## Tests

| test | checks |
|---|---|
| `exact_game_race_lockstep` | a GT Mode race (Sunday Cup, 6 cars), a rally and a split-screen battle through the race screen's field, every car record compared after every step with the same race set up directly |
| `race_field_event` | a race field on Trial Mountain: opponents from the event, the player last on the grid, everyone held during the countdown, standings by distance |
| `exact_game_solo_modes` | single-car modes: the AI's run-up, a timed lap, the end of the run |
| `exact_demo_records` | the disc's demos replayed on the exact core reach the result their saves record |
| `net_exact_duel_ai`, `net_exact_duel_autopilot` | the race's AI driving the player's car online |

The AI is checked step for step against the game's own race step on the four
GT3 discs, outside this repository (every field of every car record, whole CPU
races on several courses).
