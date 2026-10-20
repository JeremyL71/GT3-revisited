# Track runtime (`src/track/`)

`track::Track` turns a decoded course (`docs/formats/course.md`, GTRW road and
AI lines) into the queries the simulation and race scene need. `track::LapTimer`
times laps for one car. Header: `src/track/track.h`. Tests:
`tests/test_track.cpp`. Diagnostics: `gt3tool track-survey <iso> [prefix] [bench]`
and `gt3tool track-probe <iso> courses/US/<name> <x> <z> [y_max]`.

Frames are those of `src/sim/sim_types.h`: world right-handed, +Y up, metres;
car frame nose towards -Z. Track distance `s` is metres along the lap from the
start/finish line, in `[0, length())`.

```cpp
auto track = track::Track::load(vfs, "trialmt");              // courses/US/trialmt
sim::SurfaceHit h = track->probe(x, z, wheel_y + 1.0f);        // sim::TrackQuery
track::Pose p = track->grid_pose(0);                           // pole slot
track::WallContact w = track->wall_contact(centre, radius);
track::TrackPosition tp = track->locate(pos, previous_s);      // distance + lateral
track::LapTimer timer(*track);
if (timer.update(pos, race_clock)) { /* lap done: timer.last_lap_time() */ }
```

## Loading

- `Track::load(vfs, label, region = "US")` reads `courses/<region>/<label>` through
  the VFS (so mods apply). `from_course_file(bytes)` takes an already read file.
  The `Track(Road, AiLines)` constructor is for tests.
- Loading builds everything up front (`track-survey` loads all 80 US courses,
  including decompression, in about 1.4 s). After that, every query
  is `const` and allocation-free (`wall_contacts` appends to the caller's vector),
  so you can share one `Track` between threads.

## Surface probe

`probe(x, z, y_max)` returns the highest road triangle that contains `(x, z)`
in the XZ plane with a height at or below `y_max`. It reports the height, the
upward unit normal, the raw surface value and the road segment.

- **Index.** The course's own lookup grid (GTRW +0x48: 8 m cells, a list of
  (segment, triangle) references per cell) is decoded and used directly. At
  load time it is checked against every triangle: the centroid and points just
  inside each corner must be listed in their cells. 79 of the 80 US courses
  pass. In r_route11, one triangle corner lies outside the grid's breakpoint
  range, so for that course a uniform 8 m grid is built from triangle bounding
  boxes instead. `index_source()` says which grid is in use. Triangles store
  precomputed barycentric and plane coefficients, so a probe costs one cell
  lookup plus a few multiply-adds per candidate.
- **Speed.** 18.7 ns per probe on average over all 80 US courses (10.6 to 29.9
  ns per course, measured with `track-survey bench`), and about 22 ns on
  trialmt in `track_trialmt`. That run is on an Apple-silicon desktop with a
  RelWithDebInfo build, using racing-line points and random points.
- **Exactness.** The result equals the brute-force `Road::height_at` at random
  points and along the racing line on all 80 US courses (`track_all_us`).
- **Bridges and overlaps.** The road passes over itself on Midfield (25 m
  above), Special Stage Route 5 (20 m above) and Rome. Pass `y_max` = wheel
  height plus a margin (1 m is plenty) and the lower road is found under a
  bridge. The game itself has no `y_max`. Instead it restricts the search to
  the previous hit's segment and that segment's neighbour list (GTRW segment
  +0x16 / +0x18), and takes the highest triangle among those. A stateless
  `y_max` gives the same answer for a car on the road.
- **Raw surface value.** `SurfaceHit::surface` packs the triangle's material
  byte (GTRW triangle +0x08) into bits 0-7 and its flags byte (triangle +0x0C)
  into bits 8-15. Helpers: `surface_type()` (typed `Surface`),
  `surface_material()`, `surface_flags()`, `surface_physics_index()`.

### Surfaces

| material | `Surface` | where it is (all 80 US courses surveyed) | tyre compound slot (game default map) |
|---|---|---|---|
| 0 | Tarmac | racing surface, pit lane; 61% of all road area | 0 (road) |
| 1 | Kerb | thin strips at corner edges | 0 |
| 2 | Grass | verges beside the road | 1 |
| 3 | Gravel | wide run-off at corner exits (Midfield, Laguna, Slalom) | 1 |
| 4 | Sand | run-off traps, Laguna Seca only | 1 |
| 5 | Dirt | the rally stages (Tahiti, Swiss Alps); only on those 15 courses | 1 |
| 7 | Paving | road sections in Rome, Grand Valley, Deep Forest | 0 |
| 8 | Loose | patches on smtsouth (name is a guess) | 0 |
| 9 | Rough | road sections on Akasaka and Special Stage Route 5 (name is a guess) | 0 |

The meanings come from top-down maps of the material per triangle (pictures in
scratch `track/img/`) and from the game's behaviour (physics spec §3.6). The game's tyre model indexes its
per-surface tables with the material, except on wet triangles (flag 0x02),
where materials 0 and 7 become 12, 1 becomes 13 and 9 becomes 14
(`surface_physics_index`). The physics-spec workstream found that each car
maps these 15 indices to one of three tyre compound records (default map
0,0,1,1,1,1,2,0,0,0,0,0,2,2,2). The other per-surface tables (bump amplitude
and frequency, rolling resistance, skid sound and smoke) are applied by the
tyre and surface models in `src/sim/models.*`.

Flags: 0x01 marks the pit lane. The game counts the wheels on it. 0x02 marks a
wet surface, found on route5 and r_route5 (not the `_dry` files) and the wet
circle tests. 0x08 marks tunnels or covered sections and 0x10 marks stripes
across the road and some zones (both are hypotheses). 0x04 appears only on 80
triangles of L_testcourse1.

## Off the road mesh

The road mesh covers everything drivable: tarmac, kerbs, grass, gravel and
sand run-off. The wall polylines run along its outer edge (see Walls). A car
leaves the mesh only by getting through a wall or at the ends of a licence
course's mesh.

- `probe()` returns `hit = false`, `surface = 0`, `segment = -1` (`Surface::OffRoad`
  for callers that classify a miss).
- `probe_or_ground()` never misses. Off the mesh it returns the height of the
  nearest road vertex within 24 m (one below `y_max` if possible), or else the
  centre-line height, with an up normal. `hit` stays false so callers can still
  apply off-road grip. Physics should treat a miss as `OffRoad` (grass-like grip)
  at that height, and the race scene should respawn a car that stays off the
  mesh.
- For reference, when the game's wheel query misses, the game sets the miss
  flag, uses surface 0 and keeps a fallback height derived from the car body.
  It does not fall back to scenery.

## Start grid

`grid_pose(slot)` puts the car at the GTRW grid position on the road surface,
facing heading `yaw + pi` (the game adds pi in its accessor). `Pose::forward`
is `(-sin h, 0, -cos h)` and `orientation` rotates the car's nose (-Z) onto it.
On all 80 US courses every grid slot lies on the road and faces along the lap
(dot product with the lap direction > 0.7, usually > 0.99). Licence variants
(`L_*`) use only slot 0. Their remaining slots are leftovers, and in
L_smtsouth1 one of them faces backwards.

## Walls

The GTRW wall polylines (2–10 per course, 284 over the 80 US courses) follow
the edge of the road mesh: left edge, right edge, pit wall.

- **Lines and flags.** `wall_lines()` gives each polyline with per-point flags
  and `closed`. `walls()` still returns the plain point lists.
  - **Closed loops.** Every polyline is a closed loop: the last point's tag
    holds -count and links back to the first point.
  - **Flags.** They are the upper half of the point tag, which the game reads
    as the u16 at point +0x0E:
    - `kWallCorner` (0x1): the vertex is also tested against the car's edges.
      1,377 points carry it.
    - `kWallSensor` (0x2): the edge *starting* at that point only records
      crossings. The game supports it, but no US course uses it: 0 of 154,682
      points carry it.
- **One-sided edges.** Like the game, every edge is one-sided, with the
  drivable side on the **left** of the polyline direction (left = up ×
  direction = `(dz, 0, -dx)`). Probing 2 m to each side confirms this: road on
  the left only for 102,473 edges, on the right only for 45, and on both sides
  for 51,868, where the mesh continues behind the wall.
- **Contact.** `wall_contact()` tests a sphere in the XZ plane against every
  solid edge, including closing edges and excluding sensor edges.
  - **In front of a wall** (the left side), it pushes away from the closest
    point when the distance is less than the radius.
  - **Behind a wall**, up to 2 m deep and within the edge's span, it pushes
    along the normal with penetration `radius + depth`. A fast car that went
    slightly through comes back.
  - **Height.** Edges are ignored when the sphere centre is more than 5 m below
    or 8 m above the edge's points. This keeps a bridge's walls from catching
    cars underneath.
  - `wall_contact()` returns the deepest contact. `wall_contacts()` returns all
    of them, for example for the corner spheres of a box.
  - `WallContact::edge` indexes `wall_lines()[wall]` and counts the closing
    edge as the last one.
- **Checked.** On trialmt, no point of the racing line comes within 1 m of a
  wall. Over all 80 courses, about 1.3% of racing-line samples do, mostly on
  the test-course oval, where the line hugs the outside wall.
- **The game's own collision** is swept and polygon-based (physics spec §5.3; `src/sim/collision.*`).
  `wall_contact()` is a simpler, sphere-based stand-in.

## Lap geometry

The checkpoint records (about 15 m apart) are lines across the whole drivable
width at a known track distance. Their stored direction points against the
direction of travel; this was checked on every course.

- **`locate(p, hint)`.** A point between two consecutive gate lines gets a
  track distance interpolated from its signed distances to both lines. This is
  continuous, and it increases monotonically along any forward path, which a
  nearest-point projection is not on the inside of tight corners. The lateral
  offset is measured in the bilinear patch spanned by the two lines, positive
  to the left of travel.
  - **Hint.** Pass the previous distance as `hint` (searched ±150 m) so that
    places where the track passes near itself resolve correctly.
  - **Fallback.** Without a gate match (far off the track) it falls back to the
    nearest point of the centre polyline.
- **Centre line.** The midpoint of a gate line is often not the middle of the
  road, because the lines include run-off and the pit lane. At load time each
  gate's centre moves along its line to the middle of the racing surface around
  it: tarmac, kerb, dirt or paving, but not grass, gravel, sand or pit lane. The
  centre's height follows the previous gate, so it stays on a bridge or under
  one. `point_at(s, lateral)` and `direction_at(s)` evaluate this centre line.
  `respawn_pose(s)` is the centre point, facing along the lap.
  - **Checked.** Respawn poses every 25 m are on the racing surface for
    14,728 of 14,820 positions over all 80 courses. The misses are on parts of
    licence courses that have no road mesh, such as L_testcourse1 and the
    slalom courses.
- **`racing_line(index = -1)`.** Rebuilds an AI line as the game defines it: from
  each control point a straight, or, when flags bit 0 is set, a circular arc
  whose signed radius is the control point's `radius()` (see `AiLine::sample_arcs`
  and `docs/formats/course.md`). Index -1 picks the first non-empty line of 0,
  1, 2, 5, 6, because some licence and long courses leave line 0 empty.
  - **Checked.** 99.6% of samples lie on the road over all courses, and 100%
    on trialmt.

## Lap timing (`LapTimer`)

- **Progress.** Call `update(position, time)` every tick (or every few). The
  timer keeps an unwrapped progress value: track distance plus laps times
  length. Each update re-locates the car near its previous distance.
- **Laps.** A car on the grid sits just behind the line. Its first crossing of
  `s = 0` starts lap 1. Lap k ends when progress reaches k × length. So
  reversing over the line, or crossing it backwards and forwards again, never
  counts a lap. Cutting the infield cannot skip progress either, because of
  the ±150 m search window.
- **Clock.** Times follow the original's race clock (presentation spec "Race
  order and finish"): `time` is the clock after the tick (0 = GO); a crossing is
  timed at the clock before the tick plus trunc(f × the tick's 1/3000 s units),
  f the tick's fraction at the line on progress, ms = floor(units / 3), so every
  time is whole milliseconds (60 Hz: exactly the original's and
  `LicenseRun::event_ms`). `set_lap1_from_go()`: races (every car of a field)
  and standing licence tests time lap 1 from GO instead of the first crossing,
  and a race's current lap time runs from GO before the line.
  `freeze()` stops a finished car's laps (its lap time runs on).
- **Sectors.** By default the lap is split into 3 equal parts of track
  distance; `set_sector_distances()` changes the split. The timer records
  `sector_times()` for the current lap and `last_sector_times()`,
  `last_lap_time()` and `best_lap_time()` for completed laps.
- **Wrong way.** `wrong_way()` turns on after 15 m of net backwards travel. It
  turns off after about 25 m forwards again (backwards travel is capped at 30 m).
- **Respawn.** Call `teleport(position)` after a respawn. It re-locates the car
  without adding progress.
- **Checked.** In `track_all_us`, driving each course's racing line once from
  the pole slot records exactly one lap with monotonic progress on 78 of 78
  lap courses. L_circle60_dry1 and L_circle60_wet1 are excluded: their AI
  "line" is 3 points, not a lap.

## Coverage summary (80/80 US courses, `track_all_us` / `track-survey`)

| check | result |
|---|---|
| loads, decodes the game grid | 80 load; 79 use the game grid, 1 a built grid |
| `probe` == brute-force `height_at` | 80/80 |
| grid slots on road and facing along the lap | 80/80 |
| one lap with monotonic progress along the racing line | 78/78 (2 circle tests have no lap line) |
| respawn poses on the racing surface | 14,728/14,820 |
| probe time | 18.7 ns mean (10.6–29.9 per course) |
