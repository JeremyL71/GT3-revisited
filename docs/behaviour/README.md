# Behaviour

What the engine does, system by system: state, step order, rules, equations,
constants and parameter tables. The implementation notes in [docs/dev/](../dev/)
say how the code is organised and how each system was tested; these pages
describe the behaviour itself.

| Page | Covers | Code |
|---|---|---|
| [physics](physics.md) | the driving simulation: tick and state, engine and driveline, tyres, suspension, steering, brakes and aids, aerodynamics, collisions (§0–§5); car parameters from the database and the player's settings, numbered reference rules, tyre wear, smoke and brake heat, pit stops, race starts, engine condition, airborne cars, leaving the road (§6–§11) | `src/sim/`, parts of `src/race/` |
| [ai](ai.md) | CPU drivers: AI lines, speed planning, steering, recovery, traffic, starts, rubber band, pits, the safety respawn, the AI driving the player's car | `src/ai/`, `src/race/field.cpp`, `src/race/pit.*`, `src/app/screen_race_drive.cpp` |
| [gt_mode](gt_mode.md) | GT Mode's rules: money, dealers, garage, parts and settings, car condition, licences, events and championships, endurance, prizes, records, endings, saves, randomness; the arcade progression | `src/game/` |
| [menus](menus.md) | the menu system and the game flow: pages, focus, input, popups, drawing, transitions, arcade, code-drawn screens, options, race quick menus, save page and name entry, Replay Theater, menu sounds | `src/menu/`, `src/app/`, `src/game/gt_mode_menu/`, `src/race/results.*`, `src/race/post_run.*` |
| [presentation](presentation.md) | how a race is shown: licence tests, start and finish, leaving a race, HUD, pause, prize pages, replays, cameras, lighting, lamps, wet road, effects, which events trigger race sounds | `src/race/`, `src/app/screen_race*` |
| [sound](sound.md) | voices and mixing, reverb, the sequencer, sound effects, the engine sound model, other race sounds, ambience | `src/audio/`, `src/race/race_audio.*`, `src/race/car_sfx.*`, `src/race/race_sfx.*` |
| [music](music.md) | which music plays when, song markers, and how a request becomes sound | `src/audio/`, `src/race/race_audio.*`, `src/race/race_playlist.*`, `src/race/music_markers.*`, `src/menu/flow/` |

Some systems have no page here; their behaviour is described with their
implementation notes: online battle ([docs/dev/ONLINE.md](../dev/ONLINE.md)),
VR headsets ([docs/dev/VR.md](../dev/VR.md)), movies
([docs/dev/MOVIES.md](../dev/MOVIES.md)), controllers and input mapping
([docs/INPUT.md](../INPUT.md)) and mods ([docs/MODDING.md](../MODDING.md)).

## Conventions

- Code comments cite these pages as `<page> §n`: `physics §4.2` is
  [physics.md](physics.md) §4.2. Presentation's sections are lettered (§A–§M).
- One tick is 1/60 s. Units are SI unless a value says otherwise.
- A rule the code tags **[placeholder]** is a conventional model chosen by this
  engine, not a description of the original game; the pages say so where it applies.
- Where the code leaves a question open, the page says so.

When a pull request changes behaviour, it updates the page that describes it.
