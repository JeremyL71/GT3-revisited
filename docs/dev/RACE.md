# Race scene

The race screen runs every race of the game: GT Mode and arcade races,
licence tests, time trials, 2P battles, online battles, replays and the
Replay Theater. On its own (`gt3r <disc> --screen race`, "Time trial" in the
main menu) it is a time trial: choose a course and a car (names from
`data::CarDatabase`), then drive.

Code: `src/race/` (engine side) and `src/app/screen_race*.cpp` (the screen; see
"Race screen code map" below).

| File | What |
|---|---|
| `race/backend.h/.cpp` | `RaceTrack` / `RaceVehicle` interfaces, adapters over `track::Track` + `sim::TrackWorld` and `sim::Vehicle`, `car_setup_from_database()` |
| `race/standins.h/.cpp` | Fallbacks used when the real track runtime / vehicle model is not built: GTRW-based track query and walls, kinematic car |
| `race/controls.h/.cpp` | Pad → `sim::ControlInputs` mapping, rumble model |
| `race/interp.h/.cpp` | Fixed-tick accumulator, quaternion helpers, `VehicleState` interpolation |
| `race/camera_rig.h/.cpp` | Chase, far chase, bumper, roof and free-orbit cameras |
| `race/race_scene.h/.cpp` | GPU scene: course (scenery, backdrop, sky domes) and the car (body LODs + four wheels) |
| `race/hud.h/.cpp` | HUD (lap times, tachometer, gear, speed) using the game's own assets |
| `race/replay.h/.cpp` | Replay format, recorder, player, save/list in `<user data>/replays/` |
| `race/replay_library.h/.cpp` | The Replay Theater's files: list with headers, delete, rename, info texts |
| `race/replay_options.h/.cpp` | REPLAY OPTIONS: replay view mode, target, HUD items, sound; the panel |
| `race/analyzer.h/.cpp` | The Analyzer (data logger): lap analysis of replays, GRAPH / REPLAY screens |
| `sim/car_spec_io.h/.cpp` | CarSpec to bytes and back (replays carry their car) |
| `race/ghost.h` | Best-lap ghost (live, or re-simulated from a saved replay) |
| `race/replay_cameras.h/.cpp` | GT3's replay camera script (GTRC) and the replay director |
| `race/race_audio.h/.cpp` | Reverb ambience model, race music, opponents' engine notes |
| `race/race_sfx.h/.cpp`, `race/car_sfx.h/.cpp` | Named se.inf effects; tyre, kerb, road and collision sounds per car |
| `race/field.h/.cpp` | Player(s) + AI opponents in one `sim::World`; standings |
| `race/results.h/.cpp` | Results overlay, time-trial overlay, post-race quick menu (game .pmb pages) |
| `race/paint.h/.cpp` | carcolor.db and the race-start colour draw |
| `tests/test_race.cpp` | Control mapping, stepping, interpolation, rumble, render-state decoding, car setup |
| `tools/gt3tool/cmd_race.cpp` | Diagnostics: `race-texlod`, `race-blend`, `race-sheen`, `race-carprims` |

## Race screen code map

The screen is one class, `RaceScreen`, declared in the private header
`src/app/screen_race_impl.h` and defined over several files next to the other
screens in `src/app/` (a `screen_race_` prefix rather than a subdirectory: the
app sources are the flat `src/app/*.cpp` glob, and `src/race/` is already the
engine side). The header lists the methods grouped by the file that defines
them, then the state.

| File | What lives there |
|---|---|
| `screen_race.h` | Public: `make_menu_race_screen()` (`make_online_race_screen()` is declared in `screen_online.h`) |
| `screen_race_impl.h` | Private: `RaceScreen`, `DebugPreset` (GT3R_RACE), `MenuLaunch`, small helpers (`BrakeLamps`, `Perf`, `menu_keys`) |
| `screen_race.cpp` | Core: constructors (menu / online / `--screen race`), `update` / `render` dispatch, `init`, the menu request handler, `make_*_race_screen`, the screen registration |
| `screen_race_launch.cpp` | GT3R_RACE parsing (`read_preset`), menu requests → course / car / event / options (`resolve_launch`), online launches (`resolve_online`) |
| `screen_race_setup.cpp` | Loading and (re)starting a run: `start`, `build_vehicle` (alone or a `RaceField`), `restart` / `reset_run_state`, flying starts, warp, TV cameras, the saved-lap ghost |
| `screen_race_drive.cpp` | The race loop: `update_drive` (one frame) and `tick` (one physics step), autopilot and scripted inputs, finish detection, respawns, gear hint, dirt, pylons, tyre smoke, force feedback |
| `screen_race_render.cpp` | `render_drive`: 3D views (1P, 2P split, 4:3 frame), draw items, reflections, mirrors, sun / lamps, `hud_data`, the results / licence / time-trial overlays |
| `screen_race_audio.cpp` | Music cues, course ambience listeners, extra car sound inputs, `silence` |
| `screen_race_select.cpp` | The standalone selection page: course / car picker and saved-replay list |
| `screen_race_quick_menu.cpp` | Pre-race and result quick menus (layout per mode, item actions), free run before a GT race, options / settings / records panels |
| `screen_race_pause.cpp` | The pause window (GT3's PauseMenu, REPLAY OPTIONS, our longer list) |
| `screen_race_results.cpp` | Result rows, the result latched and posted to the menus, `leave`, race display strings |
| `screen_race_replay.cpp` | Recording / playback glue, replay view modes and REPLAY OPTIONS, the director's shot filter |
| `screen_race_analyzer.cpp` | The Analyzer: the quick menu's data logger and the theater's logger mode |
| `screen_race_online.cpp` | Online battle glue (field from the grid, events, synchronised start, prediction, ghosting); its state is `RaceScreen::net_` |
| `screen_race_license.cpp` | Licence tests (the judge every tick, medals) and the disc's demonstration runs (DEMO, preview box) |
| `screen_race_vr.cpp` | VR builds only: the stereo race (`vr_stereo`, `vr_base`, `render_vr_eyes`); see [VR](VR.md) |

## Loop

```
update(dt):
  FixedStep::advance(dt)          -> n ticks of 1/60 s (sim::kDefaultTickHz), at most 8 per frame
  per tick: prev = state; inputs = ControlMapper(pad) (or auto-drive)
            vehicle.step(inputs, 1/60)   (sim::Vehicle with track + sim::TrackWorld walls/grip)
            lap timer, rumble, respawn checks
  render_state = interpolate(prev, state, FixedStep::alpha())
  camera, engine audio and rumble use render_state
render:
  View3D::draw(course items + car items) with frustum culling, draw distance, sorted blending, fog
  HUD over it
```

The backends are chosen at compile time (`__has_include` of `track/track.cpp`
and `sim/vehicle.cpp`) and at run time (falls back to the stand-ins if the
real ones throw). The HUD's debug line (R3) shows which are active.

## Controls

| Pad | Action |
|---|---|
| Cross (pressure) | throttle |
| Square (pressure) | brake |
| Circle | handbrake |
| Left stick X / d-pad | steer (d-pad ramps at 2.8 lock/s, returns at 5/s) |
| R1 / L1 | shift up / down (manual gearbox; toggle AT/MT with Square on the selection screen or in the pause menu) |
| Select | next camera (chase, far chase, bumper, roof, free orbit) |
| Triangle | respawn on the track |
| Start | pause menu: Resume, Restart, Camera, Gearbox, Exit to selection |
| R3 | performance overlay |
| Right stick, L2/R2, mouse | free-orbit camera |

Stick: 6 % deadzone, response curve `x^1.25` (fine control near the centre).
Pedals: 8/255 bottom deadzone. Buttons held without a pressure report give
full travel (keyboards).

Rumble (`InputManager::set_rumble`): wall impacts pulse the large motor
(delta-v / 8 m/s, 0.25 s), off-road surfaces a low constant level,
wheelspin a light buzz; kerbs pulse the small motor at a speed-dependent
rate. Impulse triggers (`set_trigger_rumble`): left = ABS-style chatter when
a wheel locks under braking, right = wheelspin under throttle.

## Rendering

- Course: slot 1 scenery + extras (one mesh per GTM1 node) with frustum and
  distance culling (2.5 km), backdrop (slot 22, never distance-culled), sky
  dome (slot 21 only, as the original's main view; slot 24 is a low-resolution
  dome for a secondary pass) drawn first as camera-centred background without
  depth writes. The frame is cleared to black like the original. Scenery
  packets take the node programs' GS state (`gtm1::Options::gs_state`): depth
  write (ZMSK), culling, alpha test reference and AFAIL (foliage). See
  docs/formats/course.md.
- Car: `cars/day/<model>` with all LODs; body LOD by camera distance
  (< 30 m, < 80 m, beyond). The wheels are split into four models re-centred
  on their hubs and drawn with `car * translate(hub) * rotY(-steer) *
  rotX(-spin)`.
- Car placement (as GT3 draws it): the car model's origin (wheelbase
  midpoint, y = 0 = the body's lowest point) sits at the physics body height,
  which at rest is the ride height q (the front ride height in mm x 0.001 m, both axles; physics spec §4.2) above
  the ground. Our sim's car frame is on the ground, so the body is drawn at
  `car * translate(0, q, 0)` (0.13 m when the car has placeholder
  suspension). Wheel centres are at (±half-track, R + travel, ∓wheelbase/2)
  in the sim frame, travel = `WheelState::suspension -
  VehicleSetup::Wheel::static_suspension` (+ = compressed, the wheel rises
  into the arch; airborne = max droop). In GT3's model frame this is
  y = R - q - (x' - x_s). R34: q 0.130, R 0.327, static hub 0.197 above the
  model origin. Not drawn yet: the visual camber (setting + the camber gain x
  travel) and the swing-arm half-track shrink, both small.
- Car lighting (docs/formats/car_model.md "Materials"): lit packets carry the
  selected paint's material (`scene::Material`, from GTM1 +0x38[paint]); the
  renderer lights them with the course's Env0 car light 0 (direction, ambient
  + global ambient, diffuse, specular), texel x min(Me + La Ma + Ld Md N.L +
  Ls Ms (N.H)^p, 2) (`DrawOptions::gt3_lighting`). Lamp materials (flag bit 0)
  are drawn with the lamps off (no emission). Only light 0 is used (all
  sampled courses enable just that one); night point lights are not.
- Car reflections: GT3's per-vertex Fresnel-like weight 1 - 0.8 cos (C1 =
  0.8) times the environment. GT3 renders a 256x256 sphere map every frame
  from 0.8 m above the focus car of a scene made for it: the slot 23 sky dome
  (dark, about 0.13; rotated RotY(Env0 +0x28) RotX(Env0 +0x48), which turn
  the other way from ours) and the slot 5 scenery (about 0.06 bright), on
  black, unfogged and unlit. We render that scene into a 128 px cube map
  (`View3D::render_env_cube`, within 200 m) and look it up the way GT3 reads
  its view-dependent map (`DrawOptions::env_gt3_lookup`: the angle from the
  toward-viewer direction compressed to 2 asin(0.9 sin(theta / 2)), then tilted
  15 degrees up about the camera's x axis). Courses without slot 5 fall back
  to the drawn course (`[video] car_reflections = off`: a sky / scenery-colour
  gradient instead). The paint's weight is its texel alpha (CLUT 33..127 on
  the R34); the glass reflection's is the interior behind it (1.0).
  Not reproduced: the per-car reflection dimmer K (meaning open) and the
  destination-alpha rule for overlapping reflection chunks (measured on the
  R34: the DATE chunks overlap earlier ones only along strip seams).
- Cameras (GT3's player views are two camera points of the car's GTCI block, +0x68 count / +0x6C
  offset of 0x40-byte records, translate p at +0x00; fallbacks (0, -0.8, 0) / (0, -1.5, -6)):
  the eye sits at -p in the car model's frame (origin at the body height), looking along that frame's
  forward axis, 60 degrees horizontal over the 640x448 frame (44 vertical), near 0.1. In-car (point
  0, R34 1.01 m up, 0.2 m back): the body's attitude, own car not drawn, rear-view mirror. Chase
  (point 1, R34 6 m back, 1.5 m up): heading and pitch lagged by GT3's springs at 60 Hz (yaw 30 e -
  10 w trapezoid, pitch 20 e - 10 w), no roll, no camera lag in position. Change view toggles the
  two (the view point index wraps at 2; GTCI point 2 duplicates point 1 and,
  like points 3..19, only the replay director uses it); the mirror is shown in the in-car view
  only, never while looking back, and has no option. A new game starts in-car and each player's
  last view carries over to the next race (options +0x11C / +0x120). Look back (heading + pi,
  pitch negated, no mirror) is L1 on the race pad. Ours: change view also offers in-car without
  the mirror and a far chase (bumper -> bumper, no mirror -> chase -> far chase); `[race] views =
  gt3` keeps GT3's two. The last view is remembered for the session only (GT3 keeps it in its
  options). In-car vibration (GT3's in-car camera shakers, point 0 only, also
  while looking back): amplitude A = sum over the wheels of |slip ratio| 0.3 + |slip angle| /
  (pi/2), at most 3; speed factor S = speed clamped to 10..120 km/h over 120 km/h; on about half
  the 60 Hz updates each axis gets a target (r - 0.5) A 250 through v += ((u - u_prev) - 12 v)/60,
  x += v/60; the view turns about the eye by 8 S clamp(x) degrees of pitch and 2 S clamp(x) of
  yaw (no roll, no eye movement). GT3's RNG (s = 17 s + 17, out s ^ rotl16(s), seed 0x87654321,
  two steps per draw, including the replay shakers' draws) is reproduced. `[race] camera_shake =
  off` (ours) disables it. Roof and orbit stay as extras (presets).
- Road shade (docs/formats/course.md): the four wheels' road attributes A/B scale the car's light
  colours (per draw item) and the light shadow's alpha F; `race/road_shade.*` interpolates them
  from the road mesh (test `race_road_shade_disc`).
- Point-light courses (route5, route11, circles): `race/point_lights.*` picks each car's up to four
  lamps; the car is drawn with its own light set (`render::ItemLights`, up to four directional
  lights) and node 1 casts one shadow per lamp (shear -A78 (r.x, 0, r.z), alpha = lamp shadow
  alpha x w). The lamps' discs in the reflection map (`ReflectionDiscMesh`): GT3 draws them
  over the sky / scenery map before each car: of the same nine candidates sorted by weight, the
  first six point lamps (kind != 0); each an untextured 10-wedge fan centred on the lamp's
  sphere-map point r / |r + z|, radius (car-light record +0x40) / distance in map units, centre
  colour record +0x70..+0x7C with alpha a sqrt(w), the rim alpha 0, normal alpha blend, no
  depth test, the destination alpha kept, weakest last. We build each disc in the map's space as
  our lookup reads it (axis towards the viewer tilted 15 degrees up, map radius sin(phi / 2)),
  map its vertices back to world directions 1 m from the map's eye and draw them last into the
  cube (the focus car only; GT3 piles every car's discs onto the shared map).
- Headlamp pool (node 5): drawn additively in the ground frame when Env0 +0x58 is non-zero
  (alpha x E58).
- Reflection dimmer: GT3 multiplies the reflection passes by 1 - dirt (the car's dirt 0..0.8,
  zero on the wet Route 5); we use the garage car's dirt (others clean). It builds up during the
  race as in GT3: + S x (8 on tarmac courses, else 1) x 0.001 per second, S = the sum over the wheels
  on loose surfaces (indices 2-6, 12-14) of min(1, slip intensity), capped at 0.8; the result goes
  back to GT mode in `GameResult::car_dirt`.
- Recommended gear ("> n" after the gear digit): the player's car runs a passive AI driver (line
  bookkeeping and braking points only); when the car is faster than the coming target speed T (the
  next point's speed; in an arc's braking zone min(current, next)) the hint is the automatic box's
  gear for T (`sim::model::gear_for_speed`), blinking in braking zones.
- Reverse (GT3's "back" action): Triangle (gt3 layout) / L2 (ours), pressure like the throttle;
  while at least the throttle it requests reverse gear (`RaceVehicle::set_reverse_request`) and
  drives it (through the brake in AT, which the sim swaps in reverse; the throttle in MT). Test
  `race_control_reverse`.
- Pit auto drive sets `sim::Vehicle::set_driver_kind` (pit-lane state, non-zero; 0 otherwise):
  the original's scripted driver path (aids off), for the player and the opponents.
- Pre-race shot (races with a standing-start hold): GT3's starting camera plays `cameras/start.cam`
  (GTRC, one record, kind 0x420, read from the disc: `CameraRig::set_start_shot`) in the car's frame
  (curves x right, y up, z forward) at t = 1 - (C - 60) / 600, C the frames to the signal: three cuts
  in the one record (eye curves with steps at t 0.8 and 0.9, 204 keys): t 0..0.8 a straight move
  from 30 m ahead (3.0, 2.0, 30.4) to 6 m ahead (-2.9, 2.5, 6.3); 0.8..0.9 a low sweep along the
  side towards the rear; 0.9..1 round the front corner to (-1.6, 0.6, 4.0); the target curves aim
  near the car's centre ((0.65, 0.47, 0) -> (0.65, 0.28, 0) at 0.9), zoom 2.414 (45 degrees
  horizontal); the course name in capitals over the scene; cuts to the player view one second
  before the signal (captures of the original: the approach shot from ahead).
- Rolling / flying run-up (spec draft 23b): the regular CPU AI driver (`ai::AiDriver`, builtin set
  0) drives the player's car on its line with the automatic box until the hand-over (first
  start-line crossing; rolling: tick 300, counted in ticks), with driver kind 1; kind 5 after the
  finish (the stop driver); demos and the attract mode stay 0.
- Options (`menu::flow` [options]; the race options of GT3's options block, menu spec §10), wired:
  - 2P boost (none / low / high): the trailing player's time step runs 1 + A x (gap - start) /
    (full - start) faster (A from `full` m or a lap down): low 0.15 / 20 m / 1000 m, high
    0.20 / 10 m / 200 m (`FieldConfig::boost_*`).
  - 2P handicap h (metres): both cars on the line before the start, player 2 max(h, 20) m back
    and player 1 10 m for h > 0, the other way round for h < 0 (`FieldConfig::handicap_m`; we use
    the centre line for GT3's line 6). Test `race_two_player_options_disc`.
  - 2P tyre damage: the shipped wear curve for both cars, not on dirt courses.
  - ASM / TCS off: the player's car (and player 2's) gets its ASCC flags / TCS gain cleared, in
    arcade single race, free run and 2P only; on keeps the car's own systems.
  - gear_auto: the default AT/MT outside arcade single race and 2P (they use the car page).
  - race / replay SE off: no engine, tyre or car one-shot sounds (race SE live, replay SE in
    replays, demos and the attract mode); race BGM off: no music.
  - vibration per player (vib1 / vib2), off in replays.
  - wide mode: the 2P split default (`[race] split = auto`): top / bottom with wide mode off (4:3),
    side by side with it on.
  - ghost: the time-trial ghost is drawn only when on.
  Not wired: GT FORCE force feedback and the steering-assist types (wheel input only; platform's
  force-feedback path), the 1P tyre-damage option (no race reader in the original).
- 2P battle, 4:3 top / bottom halves (matched to captures of the original): ordinal rank top
  left with the map below it, Fastest / Last / Lap (small label, large time) top right, speed then
  a dark panel with the gear digit, a ghost "> 8", AT / MT, the bar tachometer and a red shift
  light; in-car views get a rear-view mirror per half (240 x 55 at (200, 16) of the half) and hide
  their own car; 2P replays draw both halves with no HUD and no mark. No 3-2-1 countdown or
  "PLAYER n WIN!" (the original's 2P battle has neither: the clock runs from the start); a
  finished player's half shows AUTO DRIVE, then the RACE RESULTS panel over that half while the
  other player races on: the full-screen panel from the top of the half, car names only, a car
  picture per row (the car model rendered on the panel's blue, like GT3's car icon maker) and
  "Running..." for a car still racing. After both finish: a dimmed split-screen replay of the race
  (Cross / Circle skip it), then the 2P RESULTS menu.
- Replay pause (GT3): the frame dimmed, CONTINUE / EXIT / REPLAY OPTIONS on a dark strip
  (buttons 124..236 / 244..356 / 364..508 x 192..216); REPLAY OPTIONS opens GT3's REPLAY OPTIONS
  panel (see "Replay Theater").
- Replays start the player's car with `place()` like the live standing start (no spawn hold); the
  2P and field replays now end where the recordings ended.
- Replay determinism: replays (and restarts) reuse the time-trial vehicle, so
  `sim::Vehicle::reset` now clears all per-run state (physics fixed the phase-12 ground state,
  slip gains, smoke, brake heat, ... surviving a reset; test `race_vehicle_reuse_deterministic_disc`).
  The scripted-driver kind (run-up 1, stop driver 5 after a finish or a course-out) changes the
  physics, so replays store it per tick (input flag bits 64 / 128; old files read 0) and play it
  back (test `race_replay_driver_kind`). Time-trial, field, 2P and licence replays end exactly where
  their recordings ended.
- Arcade time trial: "Lap 1/1" with Best / Time (the capture).
- Tyre smoke puffs up to 3 m that cross the ground line are split along it (GT3): the part above
  is drawn as is, the part below fades to alpha 0 one puff size down.
- Attract demo: no HUD, the replay mark.
- Replay colour filter (GT3's replay director view mode 1, rolled at every cut: the attract demo,
  and replays in the avant-garde view mode chosen in REPLAY OPTIONS or with L1 / R1; presentation spec §G): on with p 0.1, kept with p 0.8; curve preset red (1,0,0), blue (0,0,1),
  cyan (0,1,1), (0.3,0.3,0.3) or (1,1,1); saturation 1 / random / 0 (p 0.7 / 0.15 / 0.15);
  separately (p 0.25) black = 0.175 r, white = 0.2 + 0.8 r', swapped (negative) in 1.25 % of
  shots. Applied as the colour-grade pass with saturation first and the levels
  (`DrawOptions::grade_black/white/grade_saturation_first`), replacing the course grade for the shot.
- Tyre smoke and dust (`race/tyre_smoke.*`): a 256-puff pool fed each tick by every car's wheels
  with smoke-emission intensity > 0.001 (`VehicleTelemetry::smoke_emit`, physics
  §8.3b — not the slip intensity `smoke` of §8.3: race used that until now and made
  tarmac launches 5-10x too smoky, since the slip intensity reaches 1 at a slip ratio of 0.39 while the emission intensity stays
  0 below 7.6 m/s of wheelspin and tops out at 0.5 on tarmac) (one puff per wheel per tick, at the contact point, no horizontal
  velocity); smoke class from the surface (tarmac / grass / gravel / sand / dirt / wet spray), per
  course from Env0 +0x4D0 (6 x 0x44: RGB, opacity = enable, size, growth, rise, spin, fade,
  gravity, alphamix, fade spread n) and +0x910 (colour shade floor x the road sun mask); strength =
  intensity x T1[k]; linear growth / fade / rise; camera-facing quads sorted back to front, alpha
  1.99 x strength x opacity x texel, course slot 26 texture (4 frames). GT3's per-puff cull (no
  near clipping exists for puffs): a puff is drawn only while its centre's view depth d is inside
  (near, far) and it is under two screen widths across (P0 x size < 2 d, P0 the projection's x
  scale), so a camera inside or right next to a puff does not see it (test
  `race_tyre_smoke_cull`). No fog, depth test on, no depth write, normal alpha blend. Route 5
  (Env0 +0x914 / +0x91C x / y divisors 2 / 2 on route5 and r_route5 only; each 1 when <= 0 or
  > 64): GT3 draws the puffs over black into a half-resolution buffer (the frame's depth copied
  in), without the ground-line split, and adds that buffer to the frame (Cd + Cs, nearest), so
  the spray never darkens; we add the puffs straight to the frame (`TyreSmoke` additive: the same
  where puffs do not overlap; at full resolution).
  Texture alpha: `Tex1::decode` / `GsMemory::decode_rgba` already map the GS alpha 0..0x80 to
  0..255, so race code must not double it again (the smoke, BLBd billboards, glare sprite and
  HUD map art did, which made every puff's texel alpha up to 2x: opaque cores).
- Replays: no HUD (unless REPLAY OPTIONS ITEM turns parts on); the `gt_logo_mini` mark at the
  bottom right (right edge 620, bottom 421).
- Pause: GT3's windows from `pause1..3.pmb` (`race/pause_menu.*`, presentation spec §E): races
  CONTINUE / EXIT, time trials / licence tests / machine tests CONTINUE / EXIT / TRY AGAIN,
  replays CONTINUE / EXIT / REPLAY OPTIONS; EXIT asks "Exit race?" (OK / CANCEL) and ends the
  race into its quick menu; the selected button blinks (30-frame fade); SE cursor / ok / cancel.
  Select opens our longer list (restart, replays, camera, gearbox); Start also continues (ours).
  `GT3R_RACE="...,pause_sel=2"` opens it with the cursor on a button.
- Quick menus (`QuickMenu`, results.*) run on menu::flow::MenuController (menu spec §11): the
  icon row with the scroll, focus pulse, help layer, confirm blink and the menu SE, the race
  scene behind them; `prerace=1` opens the pre-race menu (screenshots).
- The env map is quantised like GT3's PSMCT16 map (5 bits per channel).
- Arcade cars run with their table-35 parts (`race/arcade_car.*`); the car page's RACING / DRIFT
  picks the tyre records: stage 10 ("_k") / 11 ("_l"), stage 12 ("_m") on dirt courses, + 13 on the
  wet Route 5 (test `race_arcade_drift_tyres_disc`). 2P battle: player 2's car, paint, AT/MT and
  RACING/DRIFT come from `GameRequest::car2`, `car2_color`, `transmission2`, `tire_mode2` (test
  `race_two_player_drift_disc`).
- Buttons: the race reads each player's race pad (`InputManager::race_pad`), whose buttons are
  GT3's defaults: Cross accelerate, Square brake, Circle handbrake, Triangle
  reverse, R2 / L2 shift, R1 change view, L1 look back (held), Start pause; we put respawn on
  Select and the performance overlay on R3. The per-player race layout (`[playerN] race = modern
  | gt3`, custom `[playerN.race]`) maps physical inputs onto them; `gt3` is the menu pad itself.
  docs/INPUT.md "Race controls" (the touch overlay presses race pad buttons).
- Car shadows (nodes 1 and 6, docs/formats/car_model.md): drawn in the
  car's ground frame (plane through the four wheel contacts, the car's
  heading), node 1 sheared by the course's shadow direction (Env0 +0xA70
  elevation, +0xA74 azimuth, +0xA78 length: (sin a, 0, cos a) x length x
  cos e with a = -azimuth, e = -elevation), colour Env0 +0xA60, alpha =
  vertex alpha x Env0 +0xA6C (x the road-shade factor F for node 1, see
  "Road shade"), 2 cm lift and a 2 % pull towards the eye as the
  depth bias. `[video] car_shadows = off` disables them.
- Speed unit: `GtData::imperial` (true for the NTSC-U data set we load; mph
  without a GT-mode session); `[race] units = kmh | mph` overrides. HUD faces
  `speed_120mp` / `160mp` / `200mp` for N = 2 / 3 / 4 (scale 40 + 40 N mph);
  the odometers and the data logger follow.
- Arcade requests: `transmission` 0/1 picks AT/MT; `hud=0` (display off) also
  hides the time-trial FAIL / record overlays.
- Transparent geometry is sorted back to front per mesh.
- Course render state from node programs (see "Course render state" below):
  alpha-only passes are skipped, the road sheen is drawn additively, masked by
  the base pass's vertex alpha. Textures with transparent texels on course
  geometry are alpha-tested.

### Road "noise" (fixed)

The noisy, smeared road came from course passes the decoder drew as normal
opaque/blended geometry although the game draws them with special frame
buffer state:

1. **Road sheen** (chunk flag 4, alpha mode 0x58): a lit second pass over the
   base road vertices with a high-contrast noise texture. The game blends it
   as `Cs * Ad + Cd`, i.e. additively, weighted by the frame buffer alpha that
   the base pass wrote from its vertex alpha (which is 0 or 0x80 per vertex:
   20909/20909 sheen vertices on Trial Mountain coincide with a base vertex;
   40 % have base alpha 0). Drawn opaque it replaced the asphalt with noise.
   Now: `gtm1::Options::road_sheen` copies the base alpha onto the sheen
   vertices and flags them `kAdditive | kEnvironmentMap`; the shader draws a
   grey specular highlight of the texture (additive, depth-equal).
2. **Alpha-only shadow masks** (FBMSK 0x00FFFFFF): red-filled textures that
   write only the frame buffer alpha (tree and bridge shadows that switch the
   sheen off). Drawn as colour they appeared as bright red ribbons (Seattle).
   Now drawn into the frame alpha only (`scene::kAlphaOnly`, see "Known gaps").
3. **Cut-outs**: course textures with transparent texels are alpha-tested
   (foliage rendered as solid green quads before).

Texture LODs were checked and are not involved: the second texture table of
course GTM1s is a quarter-resolution copy of every set (`gt3tool
race-texlod`), the decoder uses the full-resolution one; road UVs stay in
0..1 with REPEAT wrapping (`race-blend ... all`).

### Course render state (GTM1 node program ops)

| Op | Operand | Meaning |
|---|---|---|
| 0x20 | u8 mode, u8 fix | GS ALPHA (`A | B<<2 | C<<4 | D<<6`, FIX). 0x44 = `(Cs-Cd)*As+Cd`; 0x58 = `Cs*Ad+Cd` (sheen passes); 0x5800 also seen |
| 0x24 | u32 (little endian) | GS FBMSK. 0x00FFFFFF = write alpha only (shadow masks); 0xFF000000 = colour only (decals that must not disturb the alpha); 0 = all |

Survey: Seattle 42×0x44, 37×0x58, 6×0x5800; FBMSK 0 ×51, 0xFF000000 ×42,
0x00FFFFFF ×33. Trial Mountain: 35 / 35 alpha changes, FBMSK 0 / 0xFF000000
×61 each. `Draw::alpha_mode` / `Draw::fb_mask` carry these.

## HUD

Reproduces GT3's in-race HUD (`race/hud.*`), from the race workstream's HUD
research (captures of the original, `database/racedetail.db` rows,
`display.imgs`):

- **Frame.** The game lays out a 640x448 frame. Alignment 0 = left/top, 1 =
  right/bottom, 2 = centre; alignments with bit 4 (4/5) are pushed outward by
  106.667 units in the 16:9 option. We draw that 16:9 layout on our 640x360
  UI: `x' = (x + 106.667) * 0.75`, `y' = y * 360/448`.
- **Meter cluster** (racedetail type 3 presets): remote view (chase/TV
  cameras) = `revb_NN` tachometer centred right at (620, 320) 160x140 with the
  digital speed (580, 385) and gear inside; onboard view (bumper/roof) =
  `rev_NN` tachometer at (350, 436) 170x110 + `speed_XXX` speedometer at
  (290, 436), digital speed (350, 410), gear (320, 360).
- **Round meters**: face stretched to the box; pivot = box centre + offset;
  angle = a0 + 90 + clamp(v, 0, F + 8)/F (a1 - a0) degrees, screen y down;
  tapered red needle from n0 r to n1 r (base 0.04, tip 0.02 of its length); a
  segmented band at 0.95..0.98 r (500 rpm steps, 1000 on the 18k face), red
  from the shift-up rpm.

  | meter | offset | r | a0..a1 | needle |
  |---|---|---|---|---|
  | onboard tach (rev_NN, 151x105) | (0, 23) | 75 | 72.5..287.5 | -0.2..0.95 |
  | remote tach (revb_NN, 140x140) | (3, 4) | 73 | 29.85..238.34 | 0.38..0.9 |
  | speedometer (speed_*, 151x105) | (0, 23) | 75 | 72.5..287.5 | -0.1..0.95 |

- **Scales**: tach F = floor((rev limit + 1500)/1000)*1000, face = first of
  06..12, 18 covering it; speed scale = N*80 km/h with N = clamp(int(vtop*1.1
  / 22.22) + 1, 2, 4) (vtop in m/s, here estimated from rev limit and top gear).
  `race/meter_*.img` and `race/disp.imgs` are never loaded by the game.
- **Digital speed**: font `speed`, "%3d", amber (GS 0x801880D0), unit in sw_12.
  **Gear**: font `gearr`, unlit "8" under the digit, amber (R red), shift light
  `:` blinking 3 frames on / 3 off above the shift-up rpm. AT/MT: `meter` font
  glyphs `>` / `=`.
- **Laps/times** (right, R = 615 + 106.667): "Lap n/N" (sw_20) at y 48;
  time trial "Best" (84) and "Time" (112); races "Best", "Last", "Lap" (84,
  112, 140); times `M'SS.mmm` in sw_16, labels in sw_12.
- **Mini-map**: course slot 33 `GTCM`: +0x10 width, +0x14 height, +0x18/+0x1C
  world X/Z of the map centre, +0x20 scale (px per metre), +0x24 rotation
  (degrees), +0x28 offset of a Tex1 (4-bit) image. Map pixel = (W/2, H/2) +
  scale * R(rot) (X - ox, Z - oz); north-up. Drawn at (16, 32) in time trial,
  (16, 80) in races; `ecar_dot` for opponents, `1pcar_dot` for the player.
  Verified on 76 course files (centre line 100 % on the map art).
- **Races**: position "1st".."12th" (one string, nothing outside 1..12; no
  "/N") in font `rank#1`, 3 px glyph spacing, at the raw (40, 64) (no 16:9
  push), black 75 % shadow at +4 first; "Lap 1/N" is shown from the grid.
  Start countdown with the `count3..1` / `start` images.
- **Colours and pens**: the GS colour is the drawn colour (amber 0x801880D0 =
  RGB 208, 128, 24 on screen, red 240, 0, 0). The pen y of the sw_*, odo and
  speed fonts is the baseline.
- **Back panels** (flat polygons, black 50 %): remote view an octagon (3 px cut
  corners) over 484..620 x 400..430 (right-aligned); onboard view the gear
  panel 281..359 x 345..376 (top corners cut 3, straight to 30 % of the
  height, tapering to a 15 % inset).
- **Indicator lamps**: glyphs `!` ASM, `"` TCS, `#` hand brake, `$` oil of font
  `odo#1`, each followed by a space; ASM green (73, 204, 43), TCS (255, 184,
  20), the others red; unlit black 50 %; "acting" blinks (counter + 21 per
  frame, lit while positive as a signed byte). Onboard centred at (435, 430),
  remote pen (620 - 125, 410). We light ASM when the car has ASCC, TCS when
  it has TCS (blinking while it cuts), the hand-brake lamp from the input,
  the oil lamp when the garage car's oil counter reaches 300 km.
- **Odometers** (font `odo#1`, cyan 98, 192, 192, centred, baseline): "%06d"
  total distance of the car (km or miles; GT mode: the garage car's odometer
  plus this race) and "%04d" trip in tenths since the start. Onboard (205,
  428) / (205, 370) in the speedometer face, remote (605, 424) / (525, 424).
- **Gear panel**: the digit at half brightness while the clutch is not
  engaged; a ghost "> 8" after it (GT3 lights it red with the AI line's
  recommended gear when the player is faster than the line's target speed;
  not driven yet).
- **Rear-view mirror** (onboard cameras, not in replays): frame rect (200, 20)
  240x64 with a 1 px black outline; the camera's view with z flipped (looking
  back, mirrored), 1.3x zoom, far 120 m; we leave the player's own car out.

- **Boost meter** (turbo cars): `boost` face 64x64 at (530, 436) onboard /
  (475, 436) chase, needle 0..2 over -45..200.32 degrees; the value is the
  pressure ratio (1.0 = the face's "0 kg/cm2", i.e. 1 + physics' boost).

Not reproduced yet: the recommended-gear hint, the mirror toggle button, the
needle's 8-pass motion blur, the 2P mini-map.

## Audio

`EngineSound` with the car's `CarDatabase::engine_sound()` id and variant
(e.g. R34: 21609, turbo0), updated every frame with the interpolated rpm and
effective throttle; the view gain follows the camera (onboard / chase).
Stopped on pause and when leaving the screen.

## Replays

Every run is recorded: per tick the quantised `ControlInputs` (steer s16,
pedals u16, flags), plus course, car, gearbox, start pose, respawn events and
the best lap's tick range. Inputs are quantised before the simulation sees
them, so playing the file back through a fresh `sim::Vehicle` reproduces the
run bit for bit (test `race_replay_reproduces_run`; a 230 s Trial Mountain
run re-simulates to the identical final position). Format: `replay.h`.

- Format v7 also stores what the theater shows (comment = file name, kind, laps, total
  time, power, save time) and the run's **car spec** (`sim/car_spec_io.h`: every CarSpec field,
  floats bit-exact), so a replay saved from GT mode (garage car with parts and settings), arcade
  (tyre stage, ASM / TCS options), licence or quick race plays back with the same car wherever it
  is loaded; older files play with the database car. The paint is applied too.
- Saving: every race's Save replay (pause list, the quick menus' SAVE / Save replay of GT mode,
  arcade, licence, quick race, 2P) writes at once (a toast, no slot page) to
  `<user data>/replays/<course>_<car>_<stamp>.gt3replay` (two saves in one second get the next
  stamp). That folder is all the theater lists: no slots, no capacity limit.
- Selection screen: Circle lists saved replays; Cross plays one.
- During playback: TV camera by default (Select cycles cameras), Triangle/Start pauses
  (Resume, Restart replay, Camera, Save, Stop replay, Exit).

### Replay Theater

Title > Replay Theater (`replay_theater` page, menu spec "Replay Theater"; captures of the
original's pages: theater, DEMONSTRATION list, Analyzer top, REPLAY OPTIONS):

- **Load Replay**: the saved replays on the original's DEMONSTRATION page art
  (`memorycard.mbl` page `demo_loader`, `menu/flow/theater_page.*`): 8 icons to a row, rows
  scrolling, the focused file's info (Size, Date, Mode, File Name, Car, Power, Course, Record =
  best lap, Laps), title "Load replay". Confirm plays it at once; Start deletes it after one
  "Delete OK?" (the file page's confirm art); cancel goes back. The original goes through the
  memory-card file page (slots, confirm, "working"); ours is deliberately native storage
  (`race/replay_library.*`, app backend `game/replay_theater.*`). The replay plays once
  (`GameRequest` mode `theater_replay`; Start = the replay pause with REPLAY OPTIONS, EXIT or
  its end) and the theater page comes back with focus on its first item (as the original).
- **Demonstration**: the disc's demonstration runs (`demo/<event>`, 48 licence tests then 10
  arcade time trials, the original's order, names and info) on the same page; one plays as its
  licence test / time trial driven by the demo's inputs, TV cameras, the mark only, Start =
  replay pause, then back. The demonstration's car is the demo's own car record (parts and
  settings: the arcade time trials' dirt tyres, stage 12, differ from the event car's); the
  flying start's run-up is driven by the AI driver (physics spec §9.4, also for time trials),
  the recorded frames ignored until the line. Our physics is not bit-exact, so the open-loop
  runs drift off the original's line (the fast arcade ones 3-22 s after the line): once two
  wheels leave the road or the run would fail (course-out, wall), the same AI driver takes over
  to the end (ours; see "Demo runs").
- **Analyzer** (`logger.mbl` page `top`, GameFlow control "logger"): TRACK A / TRACK B panels
  (Car, Power, Course, Time, Date, Mode, File Name) and REPLAY / GRAPH / LOAD A / LOAD B / EXIT;
  LOAD opens the list titled "Load track"; REPLAY / GRAPH need a track (B on another course is
  left out) and start the race screen in mode `logger`. There both replays are re-simulated
  (`race/analyzer.*`, `analyze_replay`: the best lap, or the whole run without one; races are
  re-run without their field, so contacts are not reproduced) into per-tick traces: distance
  from the line, speed, throttle, brake, position. The screen is logger1.pmb page
  `replay_logger`: bottom band with per-track status (SPEED, LAP TIME, SECTOR TIME, SECTOR AVER.
  SPEED at the cursor) and the sector bar (`icon_sector<n>`; sectors = the points of the
  course's `logger/<course>` LGM0 map, formats/course.md). GRAPH: speed / accel / brake traces of
  A (red) and B (blue) in a window round the cursor, the LGM0 picture with both cars, Left /
  Right move the cursor, L1 / R1 (L2 / R2) jump sectors, Circle / Cross open EXPRESSION (SPEED /
  ACCEL / BRAKE on/off, DISPLAY A/B/AB). REPLAY: track A's lap plays (looping) with B as the
  translucent ghost (drawn with A's model), START / STOP / SLOW (x0.25) / CAMERA. Triangle back to
  the top page. The input details and the drawing of traces, dots and the sector bar are ours
  (they are not specified; no captures with data, the PS2 cards were
  empty). `GT3R_RACE="lcursor=<m>"` places the cursor for screenshots.
- **REPLAY OPTIONS** (`race/replay_options.*`, presentation spec §G): MODE (view modes 0..3),
  TARGET (MANUAL: Left / Right pick the followed car in field replays; TOP: the leader; AUTO: at
  every new shot the car closest behind another, else the leader), ITEM (HUD parts shown in
  replays: meters, car name, time block, map, steering, tyres, pedals; steering / pedals / car
  name are simple native gauges), SOUND (replay BGM / SE), 2P SCREEN; stored in `[options]`
  (`replay_mode`, `replay_targets`, `replay_items`, `replay_bgm`, `replay_se`, `replay_screen`).
  Mode 1 (avant-garde) rolls the shot colour filter and forces the BGM on; modes 0 / 2 / 3 run
  the GTRC cameras (the original's handler differences between them are not reproduced). L1 / R1
  (and L2 / R2) during a replay cycle the mode with GT3's indicator (L2, mode icon, R2) for 2 s.
  SCREEN single shows a 2P replay as player 1's view only. `GT3R_RACE="ropts=<row>"` /
  `"rmode=<n>"` for screenshots.

### Replay (TV) cameras

The course's own camera script, GTRC (course slot 34), decoded by the race
workstream (all 80 US courses, 4493 records, no errors):

- Header: +0x14 course id (= GTRW id), +0x18 lap length L (= GTRW length),
  +0x1C script end T, +0x24 record count, +0x28 records (0x80 bytes, sorted),
  +0x2C/+0x30 a second list of fixed start shots, +0x34/+0x38 zones (gate the
  automount camera only).
- Record: +0x00 kind, +0x04/+0x08 start/end track distance D, +0x0C duration,
  +0x18 on-board mount, +0x28/+0x2C zoom min/max, curve pointers +0x34 zoom,
  +0x40/44/48 camera x/y/z (world), +0x58/5C/60 look-at x/y/z.
- Curve: `u32 n; n x {f32 t, f32 v}`, piecewise linear in t with linear
  extrapolation.
- Kind: `&3` mode (0 trackside, 3 on-board, 1 car-local), `&0xC` look at the
  car (+0.4 m) instead of the target curves, `0x10` zoom = distance x zoom(0).
- D = k*L + s with a lap variant k = lap+1 .. nvar (nvar = max(1, int(T/L) - 1
  - int(first.start/L))), so consecutive laps get different cuts;
  t = (D - start)/(end - start). A shot runs while the car is in its range;
  a new one starts when u is in [0, 0.6].
- zoom = 1/tan(half FOV); taken as the horizontal half-angle of the PS2's 4:3
  frame (`fovy = 2 atan(0.75/zoom)`; axis not confirmed).

Courses without a script fall back to placeholder trackside cameras every
90 m. On-board records use the bumper view.

## Races with opponents

`race/field.*`: the player and up to 5 AI opponents (the event's entries,
`ai::AiDriver` with the event's parameters and rubber band) in one
`sim::World` (car-to-car contact, slipstream, time scale). The player's
`RaceVehicle` steps the whole field, so the fixed-tick loop, replays and HUD
are unchanged; the player starts on the last grid slot. A car finishes at a line
crossing (`race::finishes_at_line`, presentation spec "Race order and finish"): after the
event's laps, or once the leader has finished (a lapped car, the player included, finishes at
its next crossing: "+1 lap"), or at its first crossing after a time limit (nothing ends at the
limit itself); its laps stop counting there. The race order (`race::ranks_ahead`, kept every
tick, ties keep their places) is the HUD position and the player's place: more laps first,
then running cars by distance, a running car ahead of a finished one, finished cars by finish
time. Replays of races store the event
and opponents (format v2) and re-run the whole field: a 40 s race replay
reproduces the player's final position exactly. `GT3R_RACE="event=AEAT0001"`
starts one directly.

### Starts and event fields

From the event record (database table 32; `race::event_start`):

| Field | Meaning |
|---|---|
| +0x198 u8 | initial speed, km/h (0 for every race; 50–220 flying starts, 80 demos / free time trial) |
| +0x199 u8 | laps |
| +0x1B5 u8 | cars in the race, player included (opponents = min(cars - 1, 5)) |
| +0x1E4 u8 | time limit, minutes (120 for the two 2-hour endurance races, which have 0 laps) |
| +0x1E6 s16 | flying-start distance, metres before the line (negative) |
| +0x1F2 u8 | countdown seconds (start type 0 only) |
| +0x1F3 u8 | start type: 0 demo / machine test, 1 standing 240 frames (licences from rest), 2 standing 600 frames (every single race), 3 rolling (GT endurance) / flying (time attack, moving licence tests) |

- **Standing** (type 2): all cars held 10 s; `count3..1` and beeps over the
  last 3 s, then START and `count_s`. Type 1 holds 4 s.
- **Rolling** (type 3 races): every car leaves its grid slot at 80 km/h
  (`sim::Vehicle::spawn_moving`: gear and rpm matched, no spawn hold), no
  countdown; the computer drives the player's car for 300 frames (5 s), then
  START and `count_s` at the hand-over.
- **Grid** (physics spec §9): standing starts place every car with
  `Vehicle::reset` (no spawn hold; that is only for respawns and pit
  starts), its nose on the slot point (moved back by the front extent).
  Flying starts sit on the racing line (game line 6) at -distance, heading
  along the line's next 10 m.
- **Grid order**: player last. Arcade: opponents in draw order. GT mode: ranked by
  power-to-weight (kg/PS = mass / peak of torque x rpm), strongest on pole
  (`FieldConfig::grid_by_power`; `GameResult::order` still indexes
  `GameRequest::opponents`).
- **Flying starts** (type 3 with an initial speed: time attack, moving
  licence tests): the car starts at the event's speed, `flying_m` metres
  before the line along the lap (on the start slot's lateral position), and
  the lap clock starts at the line. The computer drives up to the line (AUTO
  DRIVE; the AI driver, spec §9.4) in licence tests and time trials alike
  (`flying_runup()`). `GT3R_RACE="tt_event=L0A0006"` runs one.
- Replays store the start speed (format v4), so rolling and flying runs
  re-simulate exactly.
- Time-limited races show the time remaining ("Remaining h:mm'ss") in place
  of the lap count.

## Results and flags

From the race workstream's research (presentation spec §B, `result1.pmb`,
`qm_*.pmb`):

- **Start**: `count3..1` images 1 s each with `count_d` beeps, `start` image
  and `count_s` at the green light (cars held; physics' spawn hold places them).
- **Finish**: the `finish` image for 4 s, top edge at y 160. After the finish
  the game drives the player's car (auto drive). 2P: "PLAYER n WIN!" (sw_20,
  150 %) on the winner's half only.
- **Wrong way**: red "WRONG WAY" centred at (320, 305) (2P half: 200).
  GT3 has no penalties, no lap invalidation, no "final lap" message in normal
  races (endurance only); `win`/`lose`/`replay` images are unused.
- **Results overlay** (races): `result1.pmb` page "result" elements drawn
  over the running race: title bar 5 s after the finish, rank bars at
  6.0 + 0.12 i s (fade-in, unrolling 0.19 per frame out of the bar above),
  then the "OFFICIAL TIMER SEIKO" bar. Names/times sw_16 at 87 %, the
  player's row 0xFFC800, others 0xCCCCCC. Time column: winner "m'ss.mmm",
  "+gap", "+N lap(s)", "Running...". Leaves on Cross after it is fully shown,
  or 15 s after every car finished (2P: 15 s).
- **Time-trial overlay** (time trial with a lap count): records rank and a
  flickering "NEW RECORD"/"RECORD TIME", "Lap" + best lap at y 270.
- **Quick menu**: the layout per mode follows the original's race drivers
  (`RaceScreen::quick_menu_layout`): arcade race `qm_arc_result1.pmb` and 2P
  `qm_arc_2p_result1.pmb` after the race (the race runs first); arcade time trial
  `qm_arc_tt_ltd1.pmb` and time attack / GT free run `qm_arc_tt_free1.pmb`, machine tests
  `qm_gt_mt_4001/10001/max1.pmb` and licence tests `qm_gt_license1.pmb` **before the
  first run and after every run** (the drivers' loop opens with the menu); GT races
  `qm_gt_event1.pmb` (championship rounds `qm_gt_event_champ1.pmb`) before and
  `qm_gt_result1.pmb` / `qm_gt_result_champ1.pmb` after. Background, title, result list
  widgets filled with the standings, item icons in the band (scrolling past 5), the
  focused item's help layer (item order and help layers from the `.mbl`). Items are
  dispatched on their `.mbl` action string (`race::quick_menu_action`: qm_start /
  qm_try_again, qm_free_run, qm_replay, qm_demo, qm_save_replay, qm_save_ghost,
  qm_load_ghost, qm_records, qm_logger, qm_option, qm_setting, qm_next_session,
  qm_save_game, qm_exit, qm_yes, qm_no; qm_go_race, qm_load_setting, qm_save_setting,
  qm_best_time and qm_dummy are also action names, which no layout uses);
  test `race_quick_menu_actions_disc` checks that every item of the disc's 12 layouts
  has one, or a link to a page of its layout. Championship rounds: Next session (back to
  the menus, which chain the next round), Save game (the save page in Save game mode,
  `make_game_save_screen`, docs/dev/GT_MODE.md "Saves": a new or existing save gets the
  game with this round recorded, the running game and its save untouched, as
  the original stores the championship's progress around its save), and
  Exit through the layout's "Retire from championship?" page (Yes posts
  `GameResult::abandoned`). Every item works:
  - Retry / Start, Replay (returns to the quick menu at its end), Save
    replay / ghost, Load ghost (with "Update Ghost?" when one is loaded).
  - Demonstration: the computer drives the course from the TV cameras;
    Cross/Start/Circle return; the player's run stays the one replay/save use.
  - Records: best laps on the course from the saved replays, fastest first
    (car, time, date from the file name) on `records1.pmb` page `records_free_tt`.
  - Data logger: the Analyzer's GRAPH (see "Replay Theater") over the last (A)
    and best (B) laps of the session.
  - Option: transmission, camera, music, vibration, HUD, 2P split. Setting
    (free time trial): transmission (full settings live in GT mode).
  `GT3R_RACE="...,quick=1,qm=<item>"` chooses an item when the menu opens.

## 2-player battle

Arcade "2 Player Battle" (or `GT3R_RACE="p2=1,car2=<label>"`): both cars in
one `sim::World` with car contact; player 2 uses pad 2 (Select camera,
Triangle respawn), its own camera, rumble and engine note. Layout as GT3's
16:9 option: side by side, each 3D view letterboxed to (x0, H/6, W/2,
8H/11); per-half HUD of racemode 8 (426.667x448 units): rank, lap and
Fastest/Last/Lap lines, digital speed, gear, the `meter_mini` bar
tachometer, countdown and the winner message, and the course map at 70 %
with both players' dots.

`[race] split = auto|horizontal|vertical` (or `GT3R_RACE="split=h"`):
horizontal is the PS2's 4:3 layout, player 1 on top, each half a 640x224
racemode-7 HUD (text at 70 %); auto and vertical are side by side.

2P races are recorded: replay format v3 adds player 2's car and inputs (and
kind-2 respawn events); playback re-runs both cars and the final position
matches.

## Race sounds

From the research into GT3's race sounds (effects, per-view gain
table):

- Channel gains per view (in-car/chase x target/other, other cameras) for
  engine, exhaust, turbo, blow-off, whine, clash, rumble, in-car wind, road
  rush, road noise, rain, squeal, slide, dirt (`sfx_gain`).
- Tyres, per wheel by surface: tarmac/paving `on_skill` squeal + `on_road`
  roar, kerbs `rumble`, grass/gravel/sand `off_skill`, dirt `off_roadnoiz`,
  wet `rain_noiz`. Squeal/slide volume and pitch = sqrt(A(v_lat)^2 +
  B(slip)^2) (+ offset, + speed term) from the shipped curve records (three
  surface classes); kerb from load and wheel speed; road noise from tyre width
  and speed; silent 6 frames after a wheel leaves the ground. We use the
  loudest wheel per effect (one voice per effect).
- Collisions (the followed car only; GT3 reads no other car's hits): the
  strongest hit of the step, wall |j|/50000 -> `clash`, car-car |j|/7500 ->
  `clash2` (both cars of a pair), track objects -> `clash3`; clamped to 1,
  below 0.2 ignored. Volume gain x s x 0.6 x 32/d with d the camera-to-car
  distance (0.25 m at least), capped at full, centred; in the cockpit any
  audible hit is full volume. Keyed again on every step with such a hit (no
  cooldown). A wall hit is audible from about |j| = 10,000 N s: roughly a
  15 degree hit at 100 km/h or 8 degrees at 180 km/h; shallower scrapes are
  silent in GT3 too. Most car-car contact steps are light rubbing below the
  0.2 threshold (4 AI event races: 1037 of 1045 steps under |j| = 1500,
  about 6 km/h of closing speed along the normal).
- Other cars' engines: min(1, 32/d) attenuation (horizontal distance), Doppler
  (c + v_listener)/(c + v_source) with c = 331.45 m/s, "other" view gains; GT3
  sounds every car (voice priority decides); we voice the nearest 5.
- Countdown beeps `count_d` / `count_s`.
- Stereo: every engine note is panned with `EngineSound::pan_for_offset`
  from the car's offset in the viewing camera's frame (own car centred in
  the onboard views); opponents' notes the same way.
- Engine notes (own car, player 2, opponents) are placed with
  `EngineSound::set_position(x, z)` in the viewing camera's frame (x right,
  z forward; the followed car's own sounds sit in front in the in-car view),
  which also drives the optional surround cue.
- Wind / road noise: the shared `sound/rn_01.es` unit per followed car
  (`EngineUnit::update_at`), rpm and gain from `car_sound::wind(u, y, C_d)`;
  C_d = body (racing-modify, table 5) record +0x39 / 100 (R34: 0.35), y =
  `VehicleTelemetry::slipstream`.
- In-car loop (`in_car`), transmission whine (`turbine2`; only with a
  non-stock gearbox: gear part +0x10 != 0, clutch engaged; driven-axle speed
  = rear, front or the 4WD mean of the wheel speeds), turbo spool
  (`turbine`, `car_sound::Turbo` stepped every physics tick from
  `turbo_load` / `turbo_lag`) and blow-off (`b_off`, 32/d unclamped), all
  through the per-view channel gains. Followed cars only (player 1 and 2);
  opponents have their engine note and, for the nearest two, the wind unit
  at the "other car" gain (turbo / whine / in-car are 0 for other cars in
  the in-car and chase views).
- Course ambience: the CRSS emitters (course slot 35) through
  `CourseAmbience` / `CourseAmbiencePlayer`: one listener per screen (camera
  position and right axis, the followed car's velocity and the surface flags
  under it), every car's track distance for the gated crowd emitters, and
  the tunnel loops. Missing on the licence layouts without the section.
- GT-mode garage cars: the whine flag is `GameRequest::racing_gearbox`.

## Paint colours

`database/carcolor.db`: per model (id_hash) a list of colour ids, plus colour
definitions (id -> RGB). A car's paint index is the position of its colour id
in its model's list. Variant records carry no colour; the event record has
colour ids at +0x110 (u32 per slot, 0xFFFFFFFF = choose). Licence tests: slot 0
is the test car (entry 0) and gives its paint (e.g. B-2 Viper GTS 0x101, red
with silver stripes; 31 of the 48 tests set one, all in the model's list; the
rally tests and most race cars are 0xFFFFFFFF). Race reads it for `go_license` and
`tt_event=L…` (test `race_license_car_paint_disc`). Machine tests, arcade time
trials (`AT…`) and the attract demo's player slot are 0xFFFFFFFF. Unset
cars get a colour at race start in entry order (player first): each candidate
weighted 0.984375^j, divided by 1 + similarity to every other car's
candidates (fixed colours: 10000 sim^64), sorted ascending, then picked with
the game's RNG (`x = x*17 + 17; r = x ^ rotl(x, 16)`) on the race module's
state (`race_info_random()`, GT-mode spec §20): seeded 0x87654321 at start
and never reseeded, so it continues from race to race and from the opponent
pool draw of the same race (GT mode, arcade Single Race). A replay stores the
state the paint draw started from (`Replay::paint_seed`, format v6; older
replays use 0x87654321, which is what they were recorded with) and re-runs the
draw from it.

## Ghost

The best lap of the session is replayed as a translucent car
(`DrawItem::alpha` 0.4) in step with the current lap clock. On starting a
time trial, saved replays of the same course and car (newest 10) are scanned
and the fastest lap is re-simulated into the initial ghost.

## Audio ambience and music

Reverb depth (EVOL) follows the game's ambience rule every tick: target 0.5
when the surface flags under the car have `(flags & 0x18) == 0x08` (tunnels,
covered sections), 0.25 for 0x10/0x18, else 0.125; step at most 1/36 per
frame (`ReverbAmbience`). Race music: a random `race_*` track from
`bgm/ads.inf`, the next one when it ends; stopped when leaving the race.

## Menus

`screen_race.cpp` registers a `menu::flow` GameRequest handler: arcade time
trial ("compulsory", course ids `tt01`..`tt10` = events `AT010001`..: course
and car from the event), free run / time attack (`ATAF0001`, the menu's
course and car), single race (`A{E,N,H,P}{C,B,A,S}T0001` events: opponents,
laps, AI parameters) and the GT-mode requests (`go_race` with opponents;
`go_license`, `go_machine_test`, `go_free_run`, `go_preview` as time trials; a
GT-mode garage car comes as a full `sim::CarSpec`). 2P battle runs as a time
trial. Before popping, the screen posts a `menu::flow::GameResult`
(finished, position, cars, total time, best lap, lap times, distance). Menu car ids are database labels without their
leading '_'; arcade variants map to their base car. "Exit to menu" pops the
race screen and the menus resume. Scripted check:

```bash
GT3R_GT3MENU_SCRIPT="cross,wait:90,down,cross,wait:90,cross,wait:90,cross,wait:90,cross,wait:150" \
  gt3r disc.iso --screen gt3menu --shot tt.png     # Arcade > Time Trial > first course/car
```

## Screenshots, scripted runs

With `--shot` the app steps exactly 1/60 s per frame (not wall-clock time), so a
shot of frame N (`GT3R_SHOT_FRAME`) is identical on every run: same simulation
time, AI positions, interpolation and reflection cube.

```bash
GT3R_RACE="course=trialmt,car=r34_gtr_v_99,cam=0,auto=1,warp=20" \
  gt3r disc.iso --screen race --shot out.png
```

| Key | Effect |
|---|---|
| `course`, `car` | course label / car label or model id; skips the selection |
| `cam` | 0 chase, 1 far chase, 2 bumper, 3 roof, 4 free orbit (`oyaw`, `opitch`, `odist`), 5 replay (TV) |
| `auto=1` | auto-drive: steers to the centre line 9 m + 0.12 s·v ahead, speed from the curvature ahead (7 m/s² lateral, 6 m/s² braking), own gear changes, respawns when progress stalls 4 s |
| `warp=N` | simulate N seconds before the first frame |
| `at=M`, `atkmh=V` | time trial: start on the racing line M metres past the start line, moving at V km/h (default 150); for screenshots of a given place |
| `turn=D` | standing start turned D degrees about +Y (e.g. 180: facing back down the straight; lighting comparisons) |
| `mt=1` | manual gearbox |
| `pause=1` | open the pause menu |
| `hud=0`, `perf=1` | hide HUD / show the performance line |
| `sheen=0` | disable the road sheen passes |
| `quit=N` | exit after N seconds (frame-time logging) |
| `watch=1`, `rwarp=N` | after `warp`, play the run back (and skip N s into it) |
| `save=1` | save the run's replay after `warp` |
| `replay=<file>|last` | watch a saved replay |
| `ghost=0`, `music=0` | no saved-replay ghost / no race music |
| `event=<label>` | race against the event's field (start type, laps, cars, time limit from the record) |
| `p2=1`, `car2=<label>`, `split=h\|v` | 2P battle, player 2's car, split layout |
| `laps=N`, `quick=1`, `qm=<item>` | time trial lap count; open the quick menu after the finish; choose an item |
| `demo=1` | start the demonstration (licence: the disc's demo run) |
| `theater=<event>` | the Replay Theater's Demonstration of an event (AT010001 .. AT100001, L0B0001 ...) |
| `script=thr:1@0;steer:-0.3@4` | scripted player input (steer / thr / brk set at times in seconds) |
| `pit=1` | auto-drive takes the pit lane at the next entry (pit stop test) |
| `tt_event=<label>` | time trial on the event's course with its start (flying starts of licence tests / time attack) |
| `gpusync=1` | wait for the GPU after the 3D pass so "scene draw" includes GPU time |

`GT3R_RACE_DEBUG=1` logs vehicle telemetry twice a second.

## Touch controls

On phones the HUD's right-hand blocks (meters, laps/times) and the top-left
rank/map move inside `App::touch().safe_area()` (the part of the 640x360 UI
no visible touch control covers; the whole area when the overlay is hidden,
so desktop output is unchanged). `HudData::safe_area`. When the area is
narrow (16:9 / 16:10 with the right-hand pedal cluster on screen, width
446) the meter cluster is also scaled by clamp(width / 560, 0.7, 1) about
the area's bottom-right corner, so it does not cover the car. Desktop check:
a portable copy of the app with `[touch] mode = on` in its `config.ini`.

## Licence tests

`race/license.*` (rules), `screen_race_license.cpp` (flow). Facts from the original's
behaviour (presentation spec §A):

- Event fields: +0x19A kind (1 stop test, 2 section, 3 lap(s)), +0x199 laps,
  start type +0x1F3 (1 = standing, 240-frame hold; 3 = flying), gold /
  silver / bronze at +0x1C4 / +0x1C8 / +0x1CC ({u8 min, u8 s, u16 ms}). The
  car is the event's entry 0 (a variant with its own parts). 48 tests: 2 stop,
  33 section, 13 lap tests; 16 standing, 32 flying.
- **Standing start**: 4 s hold with 3-2-1, the clock starts at the signal (GO), and
  a standing test of every kind is timed from GO, including the run-up to the line
  (the first line crossing does not restart it). **Flying start**: the computer
  drives (AUTO DRIVE) from the start distance at the event's speed; the clock origin
  is the first line crossing and the player takes over there.
- **Clock** (the original's licence clock): 1/3000 s units, +50 per tick from GO;
  the checks see the value before the tick's advance (the first racing tick sees 0).
  An event inside a tick adds trunc(50 f), f its fraction of the tick, and the time is
  ms = floor(units / 3) - start. Lap counter: 0 when placed behind the line, 1 past it,
  +1 per forward line crossing.
- **Ends**: kind 1 (stop) on the first tick all four wheels' hub ground-query
  triangles carry road flag 0x04 (grounded or not; on L_testcourse1 a 24 m wide box
  1000-1025 m from the line) and the 3-D body speed is <= 0.27777779 m/s (1 km/h,
  equality passes); no hold time, no line-crossing gate; f = |v| / 1 km/h; a partial
  stop keeps waiting (no fail, no timeout). Kind 2 when the lap's progress crosses the
  course's goal distance (GTRW header +0x1C) with the lap counter >= 1, f from the
  progress before / after. Kind 3 on the line crossing whose old lap counter equals the
  laps (+0x199): one clock difference, not a sum of lap times. The HUD time is the same
  clock - start (tests `race_license_run_rules`, `race_license_clock_timing`).
- **Failures** (immediate): all four wheels on grass / gravel / sand
  (materials 2, 3, 4, 6), all four on the pit lane (flag 0x01), a wall hit
  changing the speed by more than 10 m/s, and **any touch of a pylon**
  (failure "pylon"; no threshold, also on a cone at rest). Wrong way, car contact,
  stopping and time never fail. There is no stop-box overrun rule: B-1 / B-2's row
  of 20 cones 1.9 m past the box is the limit. A fail plays the fail cue and shows
  FAIL (no time under it), then the licence quick menu; from the failing tick the HUD
  Time reads the car's recorded time, 359,999,999 ms, which GT3's formatter shows as
  `-'--.---` (it also dashes negative times and clamps at 999'59.999; test
  `race_hud_time_format`). After a finish the HUD shows the recorded time, frozen.
  Time trial stops the car on a pylon hit too (the same failure bit as a wall hit)
  and its HUD Time then shows the run's best lap, or dashes.
- **Pylons** (`race/pylons.*`, docs/formats/course.md "Rigid bodies"): created in every
  race mode on courses that have them and reset on every run / retry; asleep while
  three hull points rest on the ground; awake bodies tumble (torque-free rotation,
  angular velocity x 0.999 per frame and at most 20 rad/s, gravity, the deepest ground
  point bouncing with restitution 0.25 (0 under 0.9 m/s) and friction 0.5). Each car's
  box (front / rear extent, half width, 1 m high, sides leaning in 5 %) pushes a body
  out through its nearest face and hits it (restitution 0.01, inertia x 1.5); the car
  itself feels nothing. A touch by the player's car fails a licence and stops a time
  trial (test
  `race_pylons_disc`). Awake bodies then meet the course walls (GTRW wall points; sensor edges
  skipped; edges within 8 m of the motion): the centre against each edge in x/z with the type's
  radius (+0x90): overlapping (foot strictly inside the edge, distance < r) puts the centre r
  from the edge on its drivable side n = (s.z, 0, -s.x) / |s|; else, when the centre crossed the
  edge this frame, it goes back along its motion onto the wall line; then the ground contact's
  impulse (restitution 0.25, none under 0.9 m/s, friction 0.5) along n at the hull point with the
  least n.p. Every body, asleep too, is drawn after its planar shadow: the type's GTM1 node 1,
  projected along Env0 scene light 0's direction (+0xAD0, not the car shadow's +0xA70..A78)
  onto the plane of the road triangle under the body's centre (probed at placement and every
  awake step; p' = p - L (n.p + d) / (n.L)), no depth writes, 5 mm nearer the camera (GT3 moves
  near / far by 0.005). The pylon draw sets no colour: it inherits the GS colour scale left by
  the car shadows (Env0 +0xA60 colour, vertex alpha x +0xA6C), which we use (the order of the two
  draws was not confirmed). Checked on L_testcourse1 against a capture of B-1: the cones' long
  shadows join into a dark band along the row.
- **Medal**: time in ms (rounded) <= target, gold first; slower than bronze =
  no prize. `GameResult::license_medal` (-1 none / failed, 0 gold, 1 silver,
  2 bronze), `failure` ("course_out", "pit_lane", "wall") and `total_time`.
- **HUD**: Best, the elapsed Time and the three targets with the
  `mark_1st/2nd/3rd` icons at (615, 140 / 168 / 196); no lap or position.
- **Result** (presentation spec A; `LicenseResultOverlay`): the live race keeps
  running behind it (the car under the stop driver, the race camera, the HUD; no FINISH
  message). Frames from the result: wait 6, a black dim 0 -> 1/3 over 30 frames, the medal
  jingle at its end (frame 37, `RaceMusic::kMedalDelay`), a further 180 (gold) / 30
  (silver, bronze) frames, then the `race/gold|silver|bronze.prm` model (composited over
  the race, weight over 60 frames, 2 s fly-in then its loop), the word (GOLD / SILVER /
  BRONZE, sw_20 x 1.25 at (320, 336), black shadow then the colour copy at (+2, -2), alpha
  over ~31 frames) and the time (same colour, (320, 112), over 60 frames); a 300-frame
  hold, then 300 frames that Cross can skip (about 14.2 s gold, 11.7 s silver / bronze).
  No medal: the fail jingle at once, then after 60 frames a (160, 160, 255) wash rising to
  50 % over 30 frames and the text (160, 192, 64) in two copies 30 px above / below y 224
  merging over ~31 frames: TIME EXPIRED with the time at y 140 (slower than bronze) or
  FAIL (the US text table's RACEDISPLAY_ExamTimeOver / ExamFail); 300 + 48 frames. A
  time-trial FAIL (RACEDISPLAY_TimeTrialFail) uses the same wash and text, 300 frames then
  240 Cross-skippable ones. Leaving the race (results, presenter end): the last frame
  stays 10 frames, then a black wash at alpha 0.5 per frame, then a cut to the quick menu
  (presentation spec C1). The licence quick menu (`qm_gt_license1.pmb`, the test name,
  targets and description panel) shows the run just driven replaying in a loop in its
  preview box (the disc's demonstration before a run).
- Machine tests (`go_machine_test`): M400 / M1000 hold 7 s, then the time
  to the 400 / 1000 m mark (interpolated) is `total_time`; MMAX runs the
  event's laps. `GameResult::top_speed_kmh` = the run's best speed (every
  race).
- `GT3R_RACE="tt_event=L0B0003"` runs a test (car, course, start from the
  event).
- **Medal model**: `race/gold|silver|bronze.prm` drawn by `render::PrizeView`
  (shared with GT mode's menus) with the file's animation and camera (fly-in
  over frames 0-120, then one turn per 200 frames; 60 degrees horizontal),
  over a dark backdrop from 0.6 s after the finish, the medal word and time
  on top. No model for NO PRIZE. `GT3R_RACE="...,prize=0"` shows a medal for
  screenshots.
- **Demo runs** (`race/demo_replay.*`): `demo/<event>` on the disc (48
  licence tests, 10 arcade time attacks) are the game's replay saves of the
  demonstration drive: controller input, re-simulated on playback (format in
  `demo_replay.h`; steering +-4096, positive = left; frame 0 = the race's
  first step, countdown included; the car record says AT/MT). The licence
  menu's preview box is no movie: the original runs the demo live behind the
  menu and draws the scene into (32, 58, 280, 210); we do the same (looping,
  TV cameras, silent), and "Demonstration" runs it full screen. Our physics
  is not bit-exact with the original, so the open-loop run drifts (e.g. B-3
  leaves the road after ~30 s); once two wheels are off the road or the run
  would fail the test (not during a flying start's run-up) the AI driver
  takes over (`ai::AiDriver`, the run-up's built-in set; before, the debug
  autopilot, and in the Replay Theater nothing).
  `GT3R_RACE="tt_event=L0B0003,demo=1"` starts it, `theater=<event>` as the
  Replay Theater does. Findings for the arcade time-trial demos (AT01..AT10):
  their start (event speed 100..220 km/h, -100..-399 m on line 6) matches
  the licences'; GT3 drives every flying run-up (licence, time trial, free
  run with a distance) with the CPU AI path until the line (§9.1), which we
  did for licences only, and the theater used the event car (racing tyres on
  the dirt AT08 / AT09). With both fixed the runs still leave the road
  (first failure 3-22 s after the line; changing the frame alignment by
  -16..+4 frames moves it, no offset fixes it): the open-loop drift of our
  physics at 150-250 km/h, so the takeover applies to them too.

## GT-mode flow around a race

- A finished single GT race (not a championship round): RACE RESULTS overlay (car pictures in
  every row) -> the prize page (cup with the prize money counting into the credits, or the
  consolation prize; `game::queue_race_prize`, shown by `make_presentation_screen()` over the
  race) -> the race's replay starts by itself -> its end or EXIT -> the GT result quick menu
  (finished cars with their own race times, `--'--.---` for a car still racing when the race
  ended). `GameResult::prize_shown` tells GT mode not to
  show the page again. The result is latched when the live race ends (`RaceResultLatch`, as the
  original records the place and prize before the replay) and is what every exit posts: quick
  menu EXIT, replay pause EXIT, leaving from a replay; Retry drops it. The result menu's rows
  are kept with it (`latched_rows_`): the replay re-runs the whole field, so the standings
  after it are the replay's, not the race's. A finished GT single race
  is also written to the save at that moment (`game::save_race_run`). Screenshots: `gt3r --screen gtprize` (`GT3R_PRIZE=<event>:<place>`,
  `GT3R_GTMODE_PRIZE_FRAME=<n>`), `GT3R_RACE="...,quick=1,gtq=1"` for the GT result menu.

- GT-mode races and machine tests open with the game's pre-race quick menu
  (`qm_gt_event1.pmb`, championship rounds `qm_gt_event_champ1.pmb`,
  machine tests `qm_gt_mt_4001/10001/max1.pmb`; licence tests their licence menu
  with the disc's demonstration in the preview box; menu launches with `GT3R_RACE`
  set skip it unless `qm=` is given): Start, Free run (the
  course alone without laps; Exit in the pause menu returns to this menu),
  Setting (GT mode's car settings screen; the car is rebuilt from
  `gt_ride_car_spec()` when it closes), Option, Exit.
- Bought wheels (`GameRequest::car_wheel`): the car model is loaded with
  `wheel/<label>` as its wheel.
- Title attract demo (`attract_demo`): all cars computer-driven from the
  event (demo events start every car at their +0x198 speed, no countdown),
  TV cameras, no HUD; any button ends it (finished = false), else it ends
  after 1800 steps (30 s, the attract demo's timer) or 5 s after the finish.

## Pit stops and tyre wear

- **Wear** (physics spec §8): endurance events carry the wear bytes
  (+0x1BD..+0x1C3); single-player races on non-dirt courses turn it on for
  every car (`FieldConfig::wear` -> `Vehicle::set_tire_wear`).
- **Tyre gauge**: `tire_gage` at (20, 435), each tyre body (alpha codes
  5/10/15/20 in GS units) in the colour of its wear level (-63..127: blue,
  green, yellow, orange, red; black <= -64), outlines dark grey. Shown when
  wear is on.
- **Pit lane** (`race/pit.*`, spec §8.6): markers are the points of AI line 4
  (file line 3); entry at its point 1 on the pit surface -> AUTO DRIVE along
  the pit lane line (line 3) at 80 + 2 km/h; 25 m before the box (point N - 4
  for the player) it drives through when the race is over, no tyre is used
  or at >= 100 km/h, otherwise it follows the stop line at 20 km/h and stops
  in the box (`Vehicle::begin_pit_stop`: 5 s, tyres at 1 s). It leaves 0.5 m
  past the box and hands back at point N - 2. The forced inputs are recorded,
  so replays re-run the stop. HUD: "AUTO DRIVE" and "<car> Pitting in..."
  (green, (320, 130), 3 s).
- **CPU cars** (AI spec §13): with wear on, a car is armed when the physics
  sets "tires worn" (a wheel's wear grip below 0.91) and enters at its next
  pass of stop-line point 0 (not on its last lap); then the same lane, stop
  (box point N - 4 - n) and exit, without rubber band, and the AI driver is
  reset onto its line afterwards. "<car> Pitting in..." for every car.
  Laguna Seca 200 km, 30 min: every opponent pits about every 4-7 laps.
- `GT3R_RACE="...,pit=1"`: the test autopilot follows the pit lane line from
  250 m before the entry, so it reaches the entry marker on the pit surface
  like a player would, and stops.

## Course-out and respawns (spec §11)

- The original never respawns or resets a car. **Course-out** is a flag in
  Time Trial and License only: all four (grounded) wheels on grass / gravel /
  sand (materials 2, 3, 4, 6), or a wall hit with a speed change above
  10 m/s (car-car hits never count). The lap is not recorded, FAIL is shown
  and the car is braked to a stop; then the quick menu (Retry). Arcade time
  trials and the race screen's own time trial apply it; free runs do not.
  `race::course_out()` is shared with the licence judge.
- `[race] respawn = off | cpu | all` (default `cpu`) controls the
  non-original fail-safe: `cpu` puts a CPU car that is lost or has made no
  progress for 12 s back on the track (races only); `all` also respawns the
  player's car after 8 s off the road; `off` never. Falling out of the world
  (NaN or 50 m below the course) always resets the car, and the test
  autopilot (`auto=1`) keeps its 4 s stuck respawn. The player's manual
  Triangle respawn and Restart/Retry stay.

## Original resolution and field of view

- `[video] original_res = on` (or `GT3R_ORIGINAL_RES=1`): the 3D scene is
  rendered at the PS2's 640x448 into a 4:3 frame (pillarboxed) and upscaled
  with `[video] original_filter = nearest` (default) or `bilinear`
  (`DrawOptions::fixed_width/height`, `nearest`); the HUD is drawn in the 4:3
  frame with the original layout (no 16:9 widening, `HudData::four_three`).
  2P split screen keeps the normal path.
- Field of view is Hor+: every camera fixes the vertical FOV and the
  horizontal one follows the aspect, so 16:9 shows more at the sides than
  4:3 and nothing is cropped vertically. (The per-camera vertical FOVs are
  our values, not the original's.)

## Sun glare and course effects (Env0 / GTFX / BLBd)

From the race workstream's research (facts for docs/formats/course.md, owned
by the course area):

- **Env0** (slot 32) is the course's environment block, read every frame:
  flags at +0x10 (car lights 0-3, ambient particles 0x20, colour-grade pass
  0x80, sun 0x400, scene lights bits 11-14, car lamp glows 0x8000, GTFX lamps
  0x200000), far clip +0x20 (the sun sits at far/2), texture / car LOD
  thresholds +0x50 / +0x54, fog RGB +0x60, distance A +0x70 and minimum
  visibility b +0x74 (visibility = b + (1 - b) * min(1, A / z)), four car
  lights at +0xD0 (0x80 each), a colour-grade curve +0x980..988 with
  saturation +0x994, the sun at +0x9C0, eight lamp types at +0xC60 and four
  scene lights at +0xAD0.
- **Sun** (+0x9C0..+0x9E8, flag 0x400): RGB, occlusion weight, glare size k
  (angular half-size atan k), disc size / gain, elevation / azimuth (degrees;
  direction (sin a cos e, -sin e, cos a cos e) with e = -elev, a = -az),
  style (0 = `race/glare.img` 512x512 star-burst sprite, 1 procedural
  72-spike burst, 2 glow disc), occlusion box N. The original draws one
  additive screen-aligned billboard at the sun after the scene, coloured RGB
  x visibility from an N x N depth test at the sun pixel; no ghosts, no
  falloff towards the centre, no bloom. A sun disc (14-segment fan) is drawn
  right after the sky dome. Trial Mountain: elev 31, az 92, k 0.40; Seattle
  16.8 / 220.2 / 0.45; Laguna 11.6 / -31.6 / 1.10; Tahiti dirt and Midfield
  have no sun.
- `race::SunGlare` draws the glare billboard (style 0, `race/glare.img`
  for every style). Visibility: the original depth-tests an N x N pixel box
  around the sun; we cast a 3 x 3 grid of rays through the same angular box
  against the course's opaque scenery and backdrop triangles
  (`race::Occluder`, an XZ grid; about 0.1 ms) and use the open fraction.
  The sun disc is not drawn yet.
- **Fog**: the course's Env0 fog replaces our linear placeholder: the model
  shader's `fog_gt3` mode, amount (1 - b)(1 - min(1, A / d)) towards the
  fog colour (`GT3R_NO_COURSE_FOG=1` restores the old fog for comparison).
- **Colour grade** (flag 0x80): applied per fragment in the model shader
  (curve x^2 + 2 g x (1 - x) per channel, then saturation); visible on Monte
  Carlo and reverse Deep Forest only. The original applies it to the frame
  after blending; per fragment differs slightly on additive layers.
- **Lamp glows** (`race::LampGlows`, GTFX + lamp types): per lamp the
  original's cull, attenuation, directional factor (facing . camera->lamp),
  blink, a ray-tested visibility (1 m short of the lamp, so the fixture does
  not hide it), an additive glow and a halo. The flare.bin meshes are drawn
  as an equivalent procedural sprite (radial disc and thin streaks). Seattle
  69 lamps (stadium lights), Route 5 / 11 about 300 (tunnel lamps).
- **Billboards** (`race::Billboards`, BLBd): every sprite as a quad turning
  about the vertical to face the camera (vertices re-uploaded when the
  camera turns, `GpuModel::update_vertices`), alpha-tested, unlit vertex
  colour x 2. Tahiti dirt 596 trees, Seattle 307, Rome 1073 spectators.
- Slot 23 (tiny GTM1) is not used in races.
- **Car lamps** (research notes, not in the repository): every car (player, 2P, opponents; replays
  from the recorded pedal) keeps a 4-frame history of "brake pedal > 0"; k = frames on.
  Lamp materials (GTM1 material flag bit 0) add their emission x k/4
  (`Material::lamp_emission`, `DrawItem::lamp`), and the node programs' op 0x06 switch
  with selector id 0 takes its "brake pressed now" branch (lit lens chunks;
  `gtm1::RunParams::brake`, a second body model `CarVisual::body_brake_`). Car model time
  of day from Env0: flag 0x80000 `cars/eve/`, else 0x8000 `cars/night/`, else
  `cars/day/`. On Env0 flag 0x8000 courses (night and dusk: route5 / route11 families,
  L_circle*, seattle, laguna, smtnorth, r_deepforest, r_grandvalley, r_smtsouth ...) the
  car lamp glow pass (`race::CarLampGlows`) draws the GTCI lists (+0x40 headlamps always,
  +0x50 brake lamps while braking at k/4 or 1, +0x48 tail lamps as a halo only) with the
  Env0 +0x9F0 lamp type (colour x intensity, glow x 8, halo, gain, attenuation c / l / q,
  max distance), the directional factor dot(viewing axis, camera -> lamp)^n per record,
  a 0.5 m pull towards the camera along the viewing axis; stretched (bar) lamps as an
  elliptical halo. No reversing lamps and no headlight switch exist. The 10x10 depth-box
  occlusion test is a ray test here (`race::Occluder`). The car lamps' streaks on wet roads
  belong to the wet-road reflection (below).
- **Wet-road reflection** (`race::WetRoad`; courses with a slot 9
  model: route5, route5_dry, route11 and their reverse layouts, three L_circle tests). After the
  scenery and before the cars: the frame alpha is cleared to 0 and slot 13 (alpha-only packets,
  alpha test off, depth-tested, no depth write) writes the mask, texel x vertex alpha; a small
  buffer (Env0 +0xC24 x +0xC28, 64 x 128 on all of them), colour and depth cleared to 0, gets
  slot 9 (the scenery stored already mirrored below the road; the main view and projection
  unchanged) with its own fog (Env0 +0xC40: RGB, +0xC50 A, +0xC54 b; route5 none, route5_dry a
  constant 27 %, route11 A 4.5 / b 0.17, black), the cars mirrored about the road plane under
  each (Env0 flag 0x10000, wet route5: body and wheels; `CarVisual::add_mirror_items`) and the
  lamp streaks (flag 0x20000); then Env0 +0xC2C blur passes (the buffer onto itself, bilinear,
  -0.5 / +0.5 texel; wet 1, dry 6) and the composite over the frame (bilinear, net -0.5 texel)
  with Env0 +0xC20: 0x54 (Cs - Cd) Ad + Cd on the wet courses, 0x58 Cs Ad + Cd on the dry
  ones. Streaks: per car the headlamps (I 1), brake lamps (k for flag-bit-0 records, else 1 while
  the pedal is down) and tail lamps (0.3) at the lamp's mirror image; c = cos(lamp axis, eye ->
  lamp), skipped when negative or behind the eye; a camera-facing quad 0.5 m nearer the eye,
  half width (E74 + (E70 - E74) c) x lamp size (+0x40), from min(E78, lamp height) above the
  image (the road surface) down to E7C below it, `race/reflection.img`, colour lamp RGB x I x c
  x E60..E68 x E6C, added; no attenuation or occlusion (wet route5: width 0.3, cap 1, length 1.8,
  gain 1; route5_dry 0.5 / 1 / 0.5 / 0.3; route11 0.63..0.03 / 0.6 / 0.2 / 0.07). Ours: the
  buffer and blur through `View3D::render_offscreen`, the frame items (alpha clear quad, mask,
  composite quad with `DrawItem::texture` / `dest_alpha_blend`) just past the near plane, in
  the scenery's draw order; the frame alpha's GS range above 0x80 (weights up to 2) is clamped
  to 1; the main view only (not 2P halves or the mirror). Slots 9 / 13 were drawn as visible
  scenery before.
- **Draw order of translucent scenery**: the PS2 draws a course's packets in file order with
  depth writes on, also for blended ones. Trial Mountain's tree avenue has untextured white
  light-shaft sheets (vertex alpha <= 0x34) late in the file; drawn earlier (our old per-mesh
  distance sort) they blocked the canopy and the road's tree-shadow overlay behind them
  (rectangular holes). The course scenery item is drawn `DrawItem::in_order`: its blended meshes
  in model order, before the distance-sorted blended items (cars, smoke).
- **Lamp glow**: Trial Mountain and Deep Forest have no GTFX lamps (Env0 flag 0x200000 off) and
  no bloom / haze pass exists in races; their tunnel lamps are plain fogged scenery quads whose
  glow is painted in the 32x32 texture times an over-bright vertex colour (up to 1.99).

## Controller force feedback

Once per frame while someone drives (not paused, in replays, menus, after
the finish or with Vibration off): `ForceFeedbackState` with speed, the
applied brake, ABS, the steering force -(fy_FL + fy_FR) / front axle load
(tyre fy points to the wheel's right) and a jolt from wall / car impulses
(/ 5000). Player 2 too in a 2P battle.

## Performance

`Race perf [...]` is logged every 3 s (frame time, 3D pass time, meshes
drawn/culled, draw calls, triangles, active voices). This Mac (Metal,
4× MSAA, 2560×1440, 120 Hz vsync), auto-drive, `gpusync=1`:

| Course | Frame | 3D pass (CPU+GPU) | Meshes drawn / total | Draw calls | Triangles |
|---|---|---|---|---|---|
| Trial Mountain | 8.3 ms (vsync) | 1.3–2.0 ms | 41–119 / 400 | 330–1100 | 11–32 k |
| Seattle | 8.3 ms (vsync) | 1.8–3.7 ms | 46–291 / 400 | 710–6070 | 19–155 k |
| Rome | 8.3 ms (vsync) | 0.3–0.5 ms (no sync) | 113–199 / 812 | 920–1870 | 25–53 k |

## Known gaps

- The sheen colour is a grey approximation (the real one is lit by the
  PS2's type-4 vector-unit lighting pass, which is not specified). Its mask
  is the PS2's: the course scenery draws with the frame-buffer alpha
  (`DrawItem::frame_alpha`: every pass writes its texel x vertex alpha, FBMSK
  0xFF000000 passes keep it, the alpha-only shadow masks write only it), and
  the sheen adds Cs x Ad (`BlendMode::DestAlphaAdd`). Identical to the old
  per-vertex mask where no shadow mask lies (Trial Mountain, Laguna, Rome,
  Deep Forest: pixel-identical frames); Seattle's masks change a few pixels.
- The visibility lists (Vls0) are not used: they only spare the PS2 drawing
  hidden scenery, and frustum / distance culling per node keeps us at the
  vsync (see "Performance"). Course node programs have no LOD switches: run
  on Trial Mountain, Seattle, Route 5 and Rome, the only op 0x07 switches
  executed are the car models' (n = 4).
- Kerb / off-road classification for rumble comes from `track::surface_type`
  (being refined by the track area).
