# Courses

Everything here comes from the retail disc (`PBPX-955.03`, v1.10). Values are
little endian. Code: `src/formats/course/` (container, road, AI lines, model
assembly), `src/formats/model/gtm1.*` (GTM1 decoder shared with cars, owned
by the car-models workstream), `tools/gt3tool/cmd_course.cpp`,
`src/app/screen_courses.cpp` (`gt3r --screen courses`), `tests/test_course.cpp`.

Status: container ✅, scenery (GTM1) ✅, road surface/walls/checkpoints/grid
(GTRW) ✅ (a few fields still unknown), AI lines ✅ (straight/arc chain; some
per-point flag bits unknown), logger maps 🔎, visibility lists, replay
cameras, GTFX/BLBd/Env0 ✅ (most fields; see below), sdd ❓ (CRSS: see [sound.md](sound.md)).

Coverage: `gt3tool course-survey` decodes **320/320** course files (container,
road, AI lines, every GTM1 including textures). `test_course` checks all 80
US files.

## Files

| Path | Contents |
|---|---|
| `courses/<US,JP,UK,EU>/<name>` (gzip, 4–14 MB) | One file per course layout. 80 per region. `r_` = reverse layout, `L_` = license-test variants. 58 of the 80 names are byte-identical in every region. The other 22 differ by a few KB, inside the scenery (for example 1,812 bytes in `trialmt` US vs UK, probably regional billboards). |
| `courses/.list` | Plain-text list of region folder and course names. |
| `logger/<name>` (gzip) | `LGM0`: the data-logger course map. It is **not** an AI line (see below). |
| `cameras/*.cam` | `GTRC`: camera tracks for attach and start shots (the same format as the replay camera section). |
| `replay/*.dat`, `demo/*` | Replay and demo inputs (survey only, below). |

The game knows the folders `courses/US`, `courses/JP`, `courses/UK` and
`courses/EU` (names stored in the disc's CORE.GT3) and picks the region folder
by the game's region.

## Coordinate frames

Scenery (GTM1), road (GTRW) and checkpoints share one frame: metres,
**right-handed, Y up**. Billboard text reads correctly with a right-handed
camera.

The AI lines store 20.12 fixed-point X/Z in a frame whose **X is negated**:
`world_x = -x / 4096`, `world_z = z / 4096`. With that mapping, every AI
control point on all 80 US courses lies on the road mesh: the per-line
median distance is ≤ 5.7 m, and the worst single point is 16 m away.

## Container

A table of 48 u32 section offsets (0 means absent). A section ends at the next
present offset. Slot 0 is always 0, and slot 1 always starts at 0xC0.

| Slot | Tag | Present | Contents |
|---|---|---|---|
| 1 | `GTM1` | 320 | Main scenery model |
| 2–4 | `Vls0` | 320 | Visibility lists for slot 1 ❓ |
| 5 | `GTM1` | 320 | Second scenery model: ~100 nodes covering the same area. Probably near-road detail or reflections ❓ |
| 6, 7 | `Vls0` | 320, 112 | Visibility for slot 5 |
| 9, 13 | `GTM1` | 36, 40 | Wet-road reflection: slot 9 the scenery pre-mirrored below the road (drawn into a small buffer), slot 13 its alpha-only mask (`Vls0` in 10 and 14). US: route5 / route11 families, the L_circle tests; see docs/dev/RACE.md "Wet-road reflection" |
| 21 | `GTM1` | 320 | Sky dome (centred on the camera, view rotation only, no fog). The only sky the race's main view draws |
| 22 | `GTM1` | 304 | Distant backdrop (mountains, skyline) |
| 23 | `GTM1` | 320 | Tiny unit-size model; not used by the race (not the sun) |
| 24 | `GTM1` | 304 | Low-resolution sky dome (±50 m; trialmt: one 128×32 T4 texture vs slot 21's 1024×256 T8). Drawn only by a secondary view pass, never in the main view |
| 25 | – | 28 | Small table starting with a u32 1..7 |
| 26 | `Tex1` | 316 | Extra texture set |
| 27 | `GTM1` | 52 | Optional model |
| 28 | `GTRW` | 320 | Road surface, walls, checkpoints, start grid |
| 29 | `GTFX` | 320 | Point-light glows (street / stadium lamps), see below |
| 30 | `BLBd` | 320 | Billboard sprites (trees, spectators) with their Tex1, see below |
| 31 | – | 320 | AI lines |
| 32 | `Env0` | 320 | Environment block: flags, fog, lights, sun, lamp types, colour grade, see below |
| 33 | `GTCM` | 320 | HUD course map: header (`a0 00 55 00` → 160×85?) plus Tex1 |
| 34 | `GTRC` | 320 | Replay camera track (keyframed curves per track-distance range) |
| 35 | `CRSS` | 300 | Course ambience emitters (air, city, crowds, sea; gated crowd points). See [sound.md](sound.md) |
| 36 | – | 256 | Unit-cube vertices followed by `GTM1`-like data ❓ |
| 37–40 | `sdd` | 320 | Four 0x40-byte parameter blocks. Byte patterns such as `73 64 64 18 64 00 5a 12`. Possibly per-surface or sound ❓ |

## GTM1 scenery

GTM1 is the same container as car bodies. The header, chunk and VIF layouts
are in the car-models docs (`docs/formats/car_model.md`). Course-relevant
facts follow.

- **Header** (relative to `GTM1`):
  - `+0x10` u16 node count, `+0x12` u16 chunk count, `+0x16` u16 texture-set count, `+0x18` u16 texture-LOD count (2 in courses).
  - `+0x20` node program offsets, `+0x24` chunk offsets, `+0x2C` Tex1 offsets for texture LOD 0.
  - `+0x30` per-node bounding spheres (16 bytes each).
  - `+0x34` table of per-texture-LOD set tables. Courses carry two parallel lists of sets: full-resolution and small. The game picks the set table per node from the node's bounding-sphere distance.
  - `+0x3C` texture animation list.
- **Node programs** are byte code, run when a node is drawn. Arguments follow the op byte. Multi-byte values are unaligned.
  - 00: end.
  - 01: no-op.
  - 02 `u8` / 03 `u16`: call node.
  - 04 `u8` / 05 `u16`: draw chunk.
  - 06: `s16`, then `u8 n`, then `s16 rel[n]`. A switch on an engine-supplied selector. If the selector ≥ n, the program skips the table; otherwise it jumps by `rel[sel]`, relative to that field.
  - 07: 16-byte sphere, then `u8 n`, then `s16 rel[n]`. A distance LOD switch: branch `min(lod, n-1)`.
  - 08 `s8` / 09 `s16`: relative jump (from the operand).
  - 0a: `u8 n`, `n×(f32 x,y,z)`, `s16 skip`. Bounding-box cull. If the box is fully outside, the program jumps by `skip` (relative to the skip field). Otherwise it continues and enables clipping only when some point is outside.
  - 0b: re-enable clipping.
  - 0c / 0d: push / pop matrix.
  - 0e `u8`.
  - 0f `16×f32`: load matrix (rows, row-vector convention `v' = x·r0 + y·r1 + z·r2 + r3`).
  - 10 `16×f32`: multiply matrix.
  - 11 `3×f32`: translate (`r3 += x·r0 + y·r1 + z·r2`).
  - 12 `3×f32`: scale rows.
  - 13 `4×f32`: axis-angle rotate.
  - 14 / 15 / 16 `f32`: rotate about X / Y / Z, in degrees.
  - 17 / 18, 1a / 1b, 1d / 1e, 2d / 2e: enable / disable render states 5, 4, 3 and 0xF.
  - 19, 1f, 29, 2a, 2f: `u8`.
  - 1c, 20: `u8 u8`.
  - 21, 24 (bit-inverted), 32, 36: `u32`. 27, 34: `f32`.
  - 22, 23, 25, 26, 30: no arguments.
  - 28, 33, 37: 16 bytes. 31: 12 bytes.
  - 2b `u8` / 2c `u16`: select texture set.
  - 35: `u16 base`, `u16 count`. Animated chunk: draws `base + frame`.
- **Course specifics.** Course nodes never call other nodes. Running every node once draws every chunk exactly once (trialmt: 600 nodes, 562/562 chunks). Each node begins with `0a` (8-point box), optionally `22`, then binds a texture set and draws chunks.
- **Chunk flags seen in courses.**
  - 0: pre-lit. V4-8 vertex colours, `0x80` = 1.0.
  - 4: second pass over the same vertices with normals instead of colours. Material high byte `0x0a`. This is the environment-map sheen on the road. `CourseModelOptions::reflections` skips it by default.
  - 0x20: two morph keys.
  - 0x41, 0x100, 0x104: other variants.
- **Packet `mat`.** The low byte is texture index + 1 in the bound set (0 = untextured). The high byte holds flags (0x04, 0x06, 0x08, 0x0a, 0x0c, 0x0e, 0xfe).
- **Prim.** GIF PRIM is a triangle strip with TME and usually IIP/FGE. ABE is set on alpha-blended packets (0x7C / 0x5C / 0x6C).

## GTRW: road surface

Offsets are relative to `GTRW`. Implemented in `road.h`.

```
0x10 u32 ? (9)   u32 course id (same value in GTRC)   f32 track length (m)   f32 ? (0.025)
0x20 u16 flag, u16 marker_count u32 markers     -> {u16 kind, s16 ?, f32 track distance}
0x28 u32 grid_count            u32 grid         -> {f32 x, y, z, yaw}       start grid, 1–8 slots; heading = yaw + pi
0x30 u32 segment_count         u32 segments     -> 0x40-byte records
0x38 u32 wall_point_count      u32 wall_points  -> {f32 x, y, z; u32 tag}
0x40 u32 checkpoint_count      u32 checkpoints  -> 0x20-byte records
0x48 u32 grid_a                u32 grid_b       -> lookup grids (triangles, wall points)
0x50 u32 extra_count (4)       u32 extra        -> 0x20-byte records {0, 0, x, y, z, 0, f32, 0x80000000}
0x58 u32 lamp_count             u32 lamps        -> 0x20-byte lamp records (point-light courses; else 0, end)
```

- **Point-light courses**: header byte +0x21 = 1 (u16 +0x20 = 0x0101) on route5 / route5_dry / r_route5 /
  r_route5_dry (281 lamps), route11 / r_route11 (273) and the L_circle tests (28); every other course has 0.
  Lamp record (0x20 bytes): u32 type n (the Env0 car-light record n, Env0 +0xD0 + 0x80 n, gives the colours:
  +0x10/+0x20/+0x30 ambient/diffuse/specular, +0x40 reflection-disc size, +0x4C spot exponent, +0x50 cutoff
  in degrees (180 = none), +0x54/+0x58/+0x5C attenuation c0/c1/c2, +0x60..+0x68 shadow RGB, +0x6C shadow alpha,
  +0x70..+0x7C reflection-disc colour), vec3 spot axis ((0, 1, 0) on the disc), vec3 position (lamp heads
  11–27 m up), f32 kind (0 directional, else point; 1.0 on the disc). Records run along the track. Each car is
  lit by up to four lamps picked from the nine around the index under its front-left wheel, weighted
  w = 1 / (c0 + c1 d + c2 d²) (× max(r·axis, 0)^exp inside the cutoff), as directional lights along car →
  lamp with the record's colours × w; node 1 of the car model casts one shadow per lamp.
- **Segment** (the road is cut into 20–204 pieces, about 30 m each, 0x40 bytes):
  - `u16 vertex_count`, `u16 triangle_count`, `u32 vertices`, `u32 triangles`, `u32 0`, `u32 wall_refs`, `u16 ?` (+0x14),
    `u16 neighbour_count` (+0x16), then `neighbour_count` × `u16` segment ids (+0x18, room for 20), zeros.
    The neighbour lists are symmetric, never contain the segment itself and hold 2–8 entries (the first two are the
    previous and next segment). The game's ground query uses them: after a hit it only accepts triangles of the
    same segment or its neighbours, which is how it keeps a car on the right level of a bridge. +0x14 is not
    neighbour_count + 1 (that only holds sometimes); meaning unknown.
  - Vertices are `{f32 x, y, z; u16 attrA, u16 attrB}`. The game divides the attributes by 65535, interpolates
    them barycentrically at the contact point (clamped to the triangle's min/max) and hands both to the wheel. Values:
    0xFFFF (most), 0 and small integers 1..~280; 74 of 80 courses have values other than 0xFFFF (the slalom
    courses have none). They are lighting values, not grip: on day courses B is an open-sky mask (0 in tunnels
    and under bridges) and A a direct-sun mask (0 in tunnels, under bridges with a wider soft edge, and along
    shaded stretches). The race averages them over the four wheels and scales the car lights by them: s = P2 +
    (1 - P2) b (1 - (1 - P1)(1 - a)) with (P1, P2) = Env0 +0xAC8/+0xACC (ambient), +0xAB4/+0xABC (diffuse),
    +0xAC0/+0xAC4 (specular); the car's light shadow takes F = (AB8 + (AB0 - AB8) a) b (Env0 +0xAB0/+0xAB8).
    On point-light courses A holds the index of the nearest lamp record and B = A + 1. The query also keeps the
    raw A of the hit triangle's first vertex.
  - Triangles are `{u16 a, b, c, u16 0; u16 material, u16 0; u16 flags, u16 0}` (the zero halves are zero on all
    1,142,387 triangles of the 80 US courses). The wheel query copies the material and flag bytes.
  - The wall_refs block lists u16 indices into the wall points that bound the segment (one 16-byte row in trialmt). It is kept raw.
- **Material** (triangle +0x08), all 80 US courses (triangles / area):

  | value | triangles | area (m²) | meaning (from top-down maps; see `docs/dev/TRACK.md`) |
  |---|---|---|---|
  | 0 | 646,338 | 7.00 M | tarmac (and pit lane) |
  | 1 | 132,503 | 0.16 M | kerbs: thin slivers along corner edges |
  | 2 | 236,806 | 4.87 M | grass verges |
  | 3 | 27,340 | 0.48 M | gravel run-off on corner exits |
  | 4 | 6,153 | 0.08 M | sand traps (Laguna Seca only) |
  | 5 | 80,815 | 0.75 M | dirt: only on the 15 Tahiti and Swiss Alps rally courses |
  | 7 | 9,904 | 0.06 M | paved road sections (Rome, Grand Valley, Deep Forest) |
  | 8 | 552 | 0.01 M | patches on smtsouth (meaning unknown) |
  | 9 | 1,976 | 0.01 M | road sections on Akasaka and Route 5 (meaning unknown) |

  The tyre model (physics spec §3.6) turns the material into a surface index: with flag 0x02 set, materials 0 and 7 become
  12, 1 becomes 13 and 9 becomes 14; otherwise the index is the material.
- **Flags** (triangle +0x0C): 0 (1,004,502), 0x01 (35,685), 0x02 (35,443), 0x10 (29,079), 0x08 (26,202), 0x18
  (7,141), 0x11 (2,635), 0x12 (1,618), 0x04 (80), 0x0A (2).
  - 0x01: pit lane (Trial Mountain map; the physics counts wheels on it).
  - 0x02: wet road (route5 and r_route5 but not the `_dry` files, and the wet circle tests).
  - 0x08: tunnels / covered sections (hypothesis; trialmt).
  - 0x10: stripes across the road and some zones (hypothesis).
  - 0x04: only on L_testcourse1.
- **Wall points** `{f32 x, y, z; u32 tag}` are concatenated polylines. The first point's tag holds the count in the low 16 bits, and the last point's tag holds `-count`. The game follows that -count back to the first point, so every polyline is a **closed loop** (all 284 lines on the 80 US courses). The upper 16 bits are point flags (the game reads the u16 at +0x0E). Bit 0 marks a corner vertex that is also tested against the car's edges (1,377 points). Bit 1 marks a sensor edge, starting at that point, that only records crossings; the game supports it but no US course uses it (0 of 154,682 points). There are 2–10 lines per course. They run along the outer edge of the road mesh (left edge, right edge, pit wall) with the drivable side on the **left** of the polyline direction in the world frame (checked for 102,473 one-sided edges against 45 the other way).
- **Checkpoint**: `{f32 x0, z0, x1, z1, distance, dir_x, dir_z, f32 ?}`. A line across the whole drivable width (including run-off; trialmt: 52 m) at `distance` metres from the start line. `(dir_x, dir_z)` is a unit vector perpendicular to the line pointing **against** the direction of travel (checked on all courses). They are about 15 m apart and stored as a ring that starts anywhere (trialmt: 254, the first one at 1957.8 m). The last float increases with distance on most courses (trialmt: distance − 1663.6); meaning unknown. `Road::centre_line()` joins the midpoints in distance order; the track runtime moves each midpoint to the middle of the racing surface (TRACK.md). On circle and slalom licence courses checkpoints only cover part of the area.
- **Start grid**: `{f32 x, y, z, yaw}`. The car's heading is `yaw + pi` (the game's accessor adds pi). With the car nose along -Z that gives forward = `(-sin h, 0, -cos h)`; this faces along the lap for every slot on all 80 US courses. y is 0 on most courses but carries real heights on some (smtsouth). Licence variants use only slot 0; the other slots are leftovers (one faces backwards in L_smtsouth1).
- **Header +0x1C**: 0 in 26 courses, the lap length in some base files, other values in licence files (L_smtsouth1: 833.8 m with slot 0 at 3246.8 m on a 3290.6 m lap), so possibly the goal distance of a licence test ❓. **+0x20**: 1, or 0x101 on the circle and route5/route11 courses ❓.
- **Grid A (+0x48): triangle lookup grid** (decoded, `Road::grid_index`). `u32 cells_off`, `s16 nx`, `s16 nz`, `f32 xs[nx]`, `f32 zs[nz]` (sorted breakpoints, uniform 8 m on every course), then at `cells_off` `(nx-1)·(nz-1)` u32 offsets, one per cell (index `zi·(nx-1)+xi`). Each points at groups `{u16 n, u16 segment, u16 triangle[n]}` ending with `n = 0`; empty cells share an empty list. Every triangle is listed in every cell it overlaps (verified on 79 of 80 courses; in r_route11 one triangle corner lies outside the breakpoint range). Points outside the breakpoint range have no road.
- **Grid B (+0x4C)**: same layout, but the ids index the wall/obstacle points (+0x3C); every point appears exactly once. The game uses it to find wall points near a car.
- **Markers**: about 20 `(u16 kind 0/1/2, s16 -1/0/1, f32 distance)` records, distances increasing. Possibly corner/section marks ❓.

## AI lines (slot 31)

```
0x00 u32 size   u32 8   u32 0
0x0C u32 line offsets[7]        0 = absent; may point at an empty list
0x28 "build MM/DD/YY hh:mm:ss\n" (tool stamp, e.g. "build 04/14/01 05:34:42")
line: u32 n, n × 40-byte control points
  s32 flags     low byte 0/1, upper bytes small packed values
  s32 x, z      1/4096 m, sim frame (world_x = -x/4096)
  s32 0
  s32 segment   GTRW segment index
  s32 0
  s32 radius    1/4096 m: signed radius of the arc starting here (flags bit 0); 0 on straights
  s32 0, s32 0
  s32 packed    low 16 bits: heading, 4096 = one turn (sim frame)
```

- Words 3, 5, 7 and 8 are zero in every record of every course.
- Lines 0–2 and 5 cover the whole lap and are near-identical variants of the racing line. Lines 3 and 4 cover part of a lap (pit lane?). Line 6 is optional.
- Points are sparse: 4–90 per lap, denser in corners. The line is a chain of **straights and circular arcs**: a point with flags bit 0 set starts an arc to the next point whose signed radius (in metres, > 0 turning left in the world frame) is the `radius` word; otherwise a straight follows. Of the two arcs of that radius joining the points, the one that continues the previous segment's direction is meant (hairpins on the slalom courses exceed 180°). `AiLine::sample_arcs()` rebuilds it: 100% of the samples lie on the road in trialmt, 99.6% over all 80 US courses, and driving it laps every lap course (78/78; the two 60 m circle tests have a 3-point line).
- Line 0 is empty on some courses (several licence files, seattle, slalom80r, r_smtsouth); use the first non-empty of 0, 1, 2, 5, 6. The first and last point can be far apart (a long straight without points), so "closed" is better decided from coverage of the lap.

## logger/: LGM0 data-logger maps

All 81 files were surveyed.

```
0x00 "LGM0"
0x10 u32 192, u32 128     map size in pixels (L_deepforest1/2: 384, 256)
0x18 f32 ox, oz, scale, rot (degrees), 0, 0   world→map transform, as the HUD map's GTCM
0x30 u32 Tex1 offset      0x34 u32 n
0x38 n × {f32 x, y, z, 0} 2..30 points along the lap
Tex1 (one T8 texture, the map image) to EOF
```

The texture is a 192×128 outline of the course (decoded and checked for trialmt), the
size of the Analyzer's map box (logger1.pmb `LoggerMapCourse_GraphMode`). A world point maps
to (w/2 + s(cos r dx - sin r dz), h/2 + s(sin r dx + cos r dz)), dx = x - ox, dz = z - oz,
y down (the GTCM formula; the cars' traces land on the outline). The points are used as the
Analyzer's sector boundaries (their track distances; trialmt 19, the page has 40 sector
labels) (M: the original's use of them is not known).

## Environment, sun, lamps and billboards (slots 32, 29, 30; from the race workstream)

Facts from the race workstream (the race reads these every frame). All values f32
unless noted; offsets from the start of the slot.

### Env0 (slot 32)

A raw dump of the developers' 0xF40-byte environment struct (block size
0x1380, the rest zero); there is no parser.

| Off | Meaning |
|---|---|
| +0x00 | `Env0` |
| +0x10 | u32 flags: bits 0-3 car lights 0-3; 5 ambient particle field; 7 colour-grade pass; 10 sun disc + glare; 11-14 scene lights 0-3; 15 car lamp glows; 16 mirrored cars and 17 lamp streaks in the wet-road reflection; 21 GTFX lamp glows (6, 8, 9, 18, 19, 20, 22: replay / car-model modes, partly unknown) |
| +0x20 | far clip (m); the sun sits at far / 2 |
| +0x50 / +0x54 | scenery texture-LOD threshold / car LOD size threshold |
| +0x60..+0x68 | fog RGB (0..1) |
| +0x70 / +0x74 | fog distance A / minimum visibility b (1.0 = no fog): visibility = b + (1 - b) min(1, A / z), z = view depth (per vertex). The backdrop (slot 22) gets the same fog at true distance |
| +0xB0..+0xC8 | ambient particles: u32 count, box / size / speed |
| +0xD0 + 0x80 i | car light i: direction (towards the light, normalised by the renderer), ambient, diffuse, specular (w lanes 1); scaled per car by its shade (+0xAB0..+0xACC). Day courses enable light 0 (the sun) and often light 1, a light from below with a negative diffuse (rome, apricot, smtnorth/smtsouth, testcourse, super, r_deepforest, ...: direction (0, −0.29, 0) or (0, −0.79, −1), diffuse −0.2..−0.26, ambient and specular 0) that darkens the lower body. Global ambient +0xEF0 (w = 1) |
| +0x980..+0x988, +0x994 | colour grade (flag 7): per channel out = x^2 + 2 g x (1 - x) (g = 0.5 identity), then saturation s around luma (0.30, 0.59, 0.11); no bloom. Only Monte Carlo (g 0.29 0.28 0.27, s 0.97) and reverse Deep Forest (0.5 0.5 0.45) change the picture |
| +0x9C0..+0x9E8 | sun: RGB, occlusion weight W, glare size k (angular half-size atan k), disc size d (angular radius atan d), disc gain, elevation, azimuth (degrees; direction (sin a cos e, -sin e, cos a cos e), e = -elev, a = -az, y up), i32 style (0 `race/glare.img` sprite, 1 72-spike burst, 2 glow disc), occlusion box N (pixels) |
| +0x9F0..+0xA1C | car-lamp glow type (as the lamp types below) |
| +0xA60..+0xA78 | car paint highlight colour, elevation / azimuth, scale |
| +0xAD0 + 0x50 i | scene light i: direction, ambient, diffuse, third colour |
| +0xC60 + 0x40 i | lamp type i (8): RGB, intensity, glow size (x10 = flare mesh scale), halo size, halo gain, directional exponent n, attenuation c / l / q (I = 1 / (c + l z + q z^2)), max distance, occlusion box (x128 px), blink period |
| +0xC20..+0xC54 | wet-road reflection pass: +0xC20 composite ALPHA mode (0x54 / 0x58), +0xC24 / +0xC28 buffer width / height, +0xC2C blur passes, +0xC40 fog block (the +0x60 layout: RGB, +0xC50 A, +0xC54 b) |
| +0xE60..+0xE7C | wet-road lamp streaks (`race/reflection.img`): RGB, gain, half width head-on / side-on (x lamp size), cap above the image, length below |

The sun: a 14-segment disc drawn right after the sky dome (geometry covers
it), and after the scene one additive screen-aligned glare billboard at the
sun, coloured RGB x visibility, where visibility is the passing fraction of
an N x N pixel depth test at the sun pixel. No ghosts, no falloff towards the
screen centre.

| Course | Sun elev / az | k | Fog A / b |
|---|---|---|---|
| trialmt | 31 / 92.05 | 0.40 | 120 / 0.70 |
| seattle | 16.8 / 220.25 | 0.45 | 55 / 0.65 |
| laguna | 11.6 / -31.6 | 1.10 | 4 / 1.0 (none) |
| tahitidirt | off | – | 125 / 0.50 |
| midfield | off | – | 155 / 0.70 |

### GTFX (slot 29)

`GTFX`, +0x0C block size, +0x10 u32 has-table, +0x14 offset of the table:
u32 list count, then list offsets (0 = empty); a list is u32 n then n
32-byte lamps {u32 type (0..7), vec3 position, vec3 facing, f32 blink
phase}. Lists are per visibility cell (slot 2). Per lamp: directional factor
max(0, facing . normalize(camera -> lamp))^n, an occlusion test at the lamp,
an additive glow (`race/flare.bin`: three untextured meshes, a 0.425 m
radial disc with thin streaks) and a halo disc. Seattle 69 lamps, Route 5 /
11 about 300, the circle stadiums, Smokey Mountain North, reverse Grand
Valley; most other courses have one empty list.

### BLBd (slot 30)

Header: +0x10 u16 list count, +0x12 u16 count of 0x50-byte records (+0x20,
always 0), +0x14 u16 sprite-type count, +0x16 u16 mesh count (always 0),
+0x18 offset of the list offsets, +0x1C offset of an embedded Tex1, +0x24
offset of the sprite types {f32 width, f32 height, offset of 4 UVs, offset of
4 RGBA colours (0x80 = 1.0)}. A list (one per visibility cell): u32 count,
then 16-byte instances {u16 type, u16 texture + 1, f32 x, y, z} (bottom
centre). Corner order bottom-left, bottom-right, top-left, top-right; some
types store the image rotated in the texture. Tahiti dirt 596 sprites
(trees), Seattle 307, Rome 1073 (spectators), Laguna 12.

## Rigid bodies: pylons, cones, small boards (GTRW +0x50, slot 36; from the race workstream)

- **Placements**: GTRW header +0x50 = count, +0x54 = offset (from the GTRW start) of 32-byte
  records: +0x00 u32 type index (read as u16), +0x04 u32 0, +0x08 f32 x, y, z (course frame,
  as the triangles; y is replaced at race start, see below), +0x14 f32 Euler angles a, b, c
  (b = yaw; a and c are 0 on the disc). The game's pool holds 100 bodies: records past the
  100th are never created (L_slalom80r3 has 111).
- **Types**: slot 36 (offset-table entry 0x90; 0 on courses without bodies): +0x00 u32 0,
  +0x04 zero-terminated u32 offsets (from the slot start) of type descriptors: +0x00 u32 N hull
  points, +0x04/+0x08/+0x0C f32 principal inertias, +0x10 N x {f32 x, y, z, 1} hull points
  (centre of mass at the origin, y up), +0x90 f32 bounding radius, +0x94 mass, +0x98 1/mass,
  +0xAC offset (from the descriptor) of the type's own GTM1 at +0xB0 (stream 0 = the body,
  stream 1 = a planar shadow drawn projected onto the ground along the light).
- Cone (L_testcourse1 type 0): 5 hull points, base 0.367 m square, 0.789 m high, centre of mass
  0.196 m above the base; inertia (4, 3, 4), radius 1.162, mass 15 (heavier 35 kg cones on
  L_deepforest1/2 and L_grandvalley2, 5 kg on L_slalom80r4); 8-point thin boards (mass 8) on
  the laguna and slalom families.
- Bodies on every L_* course except L_testcourse2/3 (L_slalom80r1-4: 95 / 98 / 111 / 77,
  L_laguna1 52, L_testcourse1 40, ...) and on some normal courses (laguna 47, slalom80r 79,
  tahitidirt 25, rome 17, super 8, trialmt 4, ...). `gadget/pylon.mdl` is not used by the race.
- L_testcourse1 (B-1 / B-2): row 1 of 20 cones at z = -590.7 (about 1026.9 m from the start
  line, 1.9 m past the stop box end), x = -979.3 .. -956.5 every 1.2 m; row 2 (20 cones) 352.9 m
  behind the start line, never reached.

## Other files (survey only)

- `cameras/attach.cam`, `start.cam`: `GTRC`. `u32 size-16` at 0x0C, `u32 4` at 0x10. They contain blocks of 0x80-byte camera entries pointing at 20-byte key records `{u32 tag, 0, 3×f32}`.
- `replay/*.dat`: 3 files of 87,128 bytes. Start `u32 7200, u32 7200` (frames?), with a flag at 0x0C.
- `demo/*`: 67 files. `u32 0x40`, `u32 size-ish`, `u32 0x40000/0x30000/0x10000`, `u32 checksum?`, then `"Base"` at 0x10.
- `demo/thumbnail/*`: 68 files of 96 bytes.

## Tools

```
gt3tool course-info   <iso> courses/US/trialmt
gt3tool course-obj    <iso> courses/US/trialmt <outdir> [road] [lines] [sky] [scenery2]
gt3tool course-shot   <iso> courses/US/trialmt <out.png> [top|persp] [road] [lines] [noscenery] [size=N]
gt3tool course-survey <iso> [courses/] [textures]
gt3r <iso> --screen courses
```

`course-shot` is a small software rasterizer (z-buffer, nearest texture
sampling, alpha test) for checking decodes offline.

## Sky dome (slot 21)

- One node, one chunk, PRIM 0x1C (textured, Gouraud, no fog, no blend), ALPHA test entry state.
  Trial Mountain: a 1024×256 T8 texture (CT32 CLUT, REGION_CLAMP over the whole texture,
  TEX1 bilinear magnify / minify, no mipmaps) on a 100 m dome reaching y = -13.5 below the eye,
  plus one 128×16 T4 cut-out quad (alpha 0 / 0x80) inside the dome. Vertex colours are 0x80
  (1.0) everywhere: no colour scale.
- The race clears the frame to black (RGBA 0) with depth before drawing (colour and depth
  both cleared). Domes that stop at y = 0 (Rome: y 0..40)
  leave black wherever no scenery is drawn below the horizon.

## Render state in GTM1 node programs (from the race workstream)

- Node op `0x20` (u8 mode, u8 fix) sets the GS `ALPHA` register: `0x44` is
  normal blending, `0x58` is additive weighted by framebuffer alpha (used by
  the road "sheen" passes).
- Node op `0x24` (u32) sets the GS framebuffer write mask (`FBMSK`):
  `0x00FFFFFF` writes alpha only (shadow masks that switch the sheen off under
  trees and bridges), `0xFF000000` writes colour only (decals).
- Chunk flag 4 marks the second road "sheen" pass; base road vertex alpha
  (0 or 0x80) is the per-vertex sheen mask it is weighted by.
- Course GTM1 texture table 1 holds quarter-resolution copies of every set.
- Op `0x2F` (u8) is TEST.AFAIL. Course foliage (6,274 packets on the 19 base US courses) is
  drawn with PRIM ABE, ALPHA 0x44, alpha test GREATER 0x7F and AFAIL 3 (RGB_ONLY): fully
  opaque texels (alpha 0x80) pass and write colour and depth; translucent ones fail and
  still blend their colour, but write no depth (and no alpha). The node restores AFAIL 0 and
  AREF 0x20 afterwards. Blending these edges *with* depth writes hides foliage drawn after
  them (sky showing through tree canopies).
- Ops `0x25` / `0x26` (depth write off / on) wrap most blended scenery: decals, ground shadow
  overlays (ALPHA 0x44, ATE off, FBMSK 0xFF000000; 7,562 packets) and the 0x58 sheen passes.
- Ops `0x2D` / `0x2E` turn back-face culling on / off. Culling is on at stream entry; nodes
  turn it off around double-sided parts (animated flags and banners).
- ALPHA 0x58 only applies to packets with PRIM.ABE. Some road base passes (Laguna, Rome,
  Tahiti dirt walls) are opaque packets (PRIM 0x3C) drawn while the register still holds
  0x58; they are ordinary opaque geometry, not sheen passes.
- State histogram per slot: `gt3tool course-states <iso> courses/US/<name> [slot] [tex | cover x z r]`.
