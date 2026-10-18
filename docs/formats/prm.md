# Prize scenes (`.prm`: `PRZ0` / `PRM0`)

Implemented in `src/formats/model/prm_model.*` on top of the GTM1 decoder
(`gtm1.*`, see [car_model.md](car_model.md)). `gt3tool prm` describes a file
and exports its model (optionally posed at a frame); `gt3tool model` and
`model-survey` accept `.prm` files. Status: all 98 `.prm` files on the disc
parse and decode (`gt3tool model-survey`: 2171/2171 model files including the
cars and wheels).

Sources: own analysis of the disc data and of the game's prize
presentations as they appear on screen.

All values little endian. Gzip-compressed in the VOL (the "Z" in `PRZ0` is not
a compression layer: the payload is plain).

## Files

| Files | Content |
|---|---|
| `race/gold.prm`, `silver.prm`, `bronze.prm` | race result trophies: an extruded GT logo on a crystal base, in gold, silver, bronze |
| `race/consolation.prm` | consolation prize (a leather key fob) |
| `race/{a,g}_{easy,normal,hard,special}_{gold,silver,bronze}.prm` | arcade (`a_`) and GT mode (`g_`) cups and trophies |
| `race/unfortunate.prm`, `race/prize.prm` (`PRM0`; also `race/{JP,US,BR}/prize.prm`) | placeholder: an untextured white disc with a camera move, no model track |
| `race/test.prm`, `race/JP/test.prm` | test scenes (the latter has a trivial model) |
| `menu/lic_{b,a,ib,ia,s,d}.prm` | license cards of the license-get screen (B, A, IB, IA; `lic_s` is the S license, a steering-wheel badge; `lic_d` has the same size and layout as `lic_a`/`lic_b`) |
| `menu/champ_00..09.prm` | championship trophies (statues, cups) |
| `menu/unfortunate.prm` | as `race/unfortunate.prm` |

`menu/JP/*` and `race/JP/*` are the Japanese versions: 35 of the 48 files
with a counterpart are byte-identical, the others differ (13). The game picks the files by prize type
(`menu/<name>.prm`, `race/<class><medal>`) and loads
`race/consolation.prm` directly.

## Header (0x40 bytes)

Pointers are file offsets, relocated in place when the file is loaded (the
+0x04 self pointer is 0 on disk and is set by the relocation). A zero pointer
means "absent".

| Offset | Type | Meaning |
|---|---|---|
| 0x00 | char[4] | `PRZ0` (94 files) or `PRM0` (4 files, older layout, see below). The loader does not check it. |
| 0x04 | u32 | 0 (self pointer at run time) |
| 0x08 | u32 | 0 |
| 0x0C | u32 | file size, or 0 (the small placeholder files) |
| 0x10 | f32 | start time in frames (always 0) |
| 0x14 | f32 | last frame (119, 240, 320, 400, 540, 600) |
| 0x18 | f32 | loop frame: when the time passes last frame + 1 it continues from here (60, 120, 180, 240, 319) |
| 0x1C | f32 | key frames per second (60). The loader divides it by 60; each 60 Hz tick adds that step to the time |
| 0x20 | u16 | light count (2 in every file; up to 8 are supported) |
| 0x22 | u16 | key count of the animated track(s) = last frame + 2 |
| 0x24 | u32 | view (camera) matrix track |
| 0x28 | u32 | model matrix track |
| 0x2C | u32 | `GTM1` model |
| 0x30 | u32 | light records |
| 0x34 | u16 | flags: bit 0 = the view track is one static matrix, bit 1 = the model track is one static matrix |
| 0x36 | u16 | 0 |
| 0x38 | f32 | display parameter A: 0 or -1 for race prizes, 14.5..38 for menu cards and trophies |
| 0x3C | f32 | display parameter B: 0..0.6 |

The two display parameters are used only by the menu trophy/championship
presentations, together with a constant 160.0; their meaning (focus distance / blur amount of the multi-pass
anti-aliased menu render is a guess) is open.

The usual layout is header, lights at 0x40, view track at 0xC0, model track at
0x100, `GTM1` after the last key. Files with an animated view track and no
model track (`unfortunate`) put the view track at 0xC0 and the `GTM1` straight
after it.

## Matrix tracks

Each key is a 4x4 float matrix (64 bytes) in the row-vector convention used
throughout GTM1 (`v' = v * M`, rows 0-2 are the basis vectors, row 3 is the
translation). A track holds one matrix when its flag bit is set, otherwise
`key count` matrices, one per frame.

Sampling at time `t` (frames): `i = floor(t)`, `j = i + 1`, except that
`j = loop frame` when `i = last frame`; the two keys are blended by `t - i`.
The game interpolates the rotation through quaternions (normalized basis
vectors) and the translation linearly, falling back to a plain linear blend
for degenerate matrices.

Draw order: the view matrix is applied to the camera, then, for each light, a
shadow pass (below), then the model matrix and the `GTM1` node 0. In effect
`v_camera = v * model(t) * view(t)`.

The projection set up for these scenes is a frustum with a horizontal half
angle of 30 degrees (60 degree field of view), vertical extent 0.7 of the
horizontal (640x448 aspect), near 0.1, far 300; the camera looks down -Z.

Examples: the race trophies fly in from z = -43.7 to z = 50.8 over frames
0..120 under a static view at z = -75.2 and then spin about Y, one turn per
200 frames, looping 120..320. The license cards drop in from above and settle
by frame ~180, looping on the last frame.

## Lights (0x40 bytes each)

| Offset | Meaning |
|---|---|
| 0x00 | vec4 light vector (w = 0; small magnitude, about 0.1-0.5, not normalized) |
| 0x10 | vec4 diffuse colour (w 1) |
| 0x20 | vec4 ambient colour (w 1; 0.1 grey in the race prizes) |
| 0x30 | vec4 specular colour (equal to +0x10 in every file) |

The four vectors define an OpenGL-style light (position, diffuse, ambient,
specular), one per record. Lights beyond the count (up to 8) are off. The lights
are placed after the view matrix and before the model matrix, so they
are fixed in scene space: towards the light = the vector normalised and turned
by the view. The light-model (global) ambient is set to (0, 0, 0, 1) first.
Lit packets then use the GTM1 VU formula of [car_model.md](car_model.md) with
the material (e.g. the GT cups' bowl, material 3: Ma (0.23, 0.16, 0.20),
Md (0.47, 0.21, 0.24), untextured at scale 255: a deep magenta).

Example (`race/g_easy_gold.prm`): light 0 (−0.12, 0.14, 0.45), diffuse 0.42,
ambient 0.1 (a key light from the upper left, towards the camera); light 1
(0.04, −0.21, −0.44), diffuse (0.07, 0.08, 0.13), ambient 0.1 (a faint blue
back light from below).

### Planar shadows

For each light `i` the game draws GTM1 node 1 (when the model has one) through
a projection matrix that flattens it onto the plane `y = -0.1 * (i + 1)` along
the light vector `L`: row 1 is `(-L.x / L.y, 0, -L.z / L.y, 0)`, row 3 has the
plane height in y, the rest is identity. When `|L.y| < 1e-4` the shear terms
are 0 (straight down); when `L.y` is exactly 0 the shadow is skipped. Only the
six license cards have a node 1; the trophies do not.

## `PRM0`

Same header, but +0x30 (lights) is 0: the files have two light records at
0xC0 that nothing points to, two identity matrices at 0x40..0xBF, the view
track at 0x140 and the GTM1 after it. Decoded with the same rules.

## The GTM1 model

Standard GTM1 (node programs, chunks, VIF packets, one Tex1 set). Units are
arbitrary scene units (the race trophies are about 11 x 7.3 x 4.6, the
license card 8.0 x 0.6 x 5.9). Features that differ from car models:

- **Environment maps are in the file.** Most packets are reflection packets
  (pre-lit colours plus a normal in the UV slot); their texture field points
  to a sphere map in the model's own Tex1 set (cars use a runtime slot). The
  decoder (option `gtm1::Options::static_env_maps`, always on for `.prm`)
  keeps that texture on the `kEnvironmentMap` primitive. The renderer
  derives the UVs like the car reflection kernel (reflected view vector in
  view space, `s = 0.5 + 0.5 Rx/m`, `t = 0.5 - 0.5 Ry/m`,
  `m = |R + (0, 0, 1)|`; `prm::apply_sphere_map` is the older CPU
  approximation from the view-space normal) and scales the vertex colour by
  the reflection rows C0 + C1 min(V.N, 0) (ops 0x36/0x37, 0x32/0x33). The
  cups' gold comes from these rows: C0 0.9, C1 (−0.5, −0.36, −0.26) over a
  near-neutral cream sphere map.
- **Blend modes.** Passes use GS ALPHA 0x44 (normal blend), 0x48
  (`Cs*As + Cd`, additive), 0x58 (`Cs*Ad + Cd`, additive weighted by the
  frame buffer alpha), 0x54 (`(Cs - Cd)*Ad + Cd`, blend by the frame buffer
  alpha) and 0x89 (`Cd*As`: the frame buffer darkened by the fragment alpha;
  the GT cups draw white, alpha 0x5A..0x80 rings inside the bowl rim this
  way). 0x48 / 0x58 become `kAdditive | kNoDepthWrite`, 0x89
  `kMultiplyDest | kNoDepthWrite`.
- **Destination-alpha masks.** Alpha-only passes (FBMSK 0x00FFFFFF) write a
  texture's alpha into the frame buffer; a later 0x54 pass then shows its
  colour only there. The license card emblem works this way: the "LICENSE x"
  wreath texture is the mask and a flat env-mapped quad supplies the shine.
  The decoder turns the 0x54 pass into an alpha-blended primitive textured with
  a white copy of the mask (texture name suffix `_mask`, UVs mapped from the
  mask quad's plane) and coloured with the mean colour of the pass's env map.
- Node 1, when present, is the shadow silhouette (`prm::load_prm_shadow`).

Placeholders (`unfortunate.prm`, `prize.prm`) have an empty (48-byte) Tex1
set and only untextured, pre-lit geometry.

## Open questions

- Meaning of header +0x38 / +0x3C (menu display parameters).
- Whether prize reflection packets use exactly the car kernel's sphere-map
  formula (assumed).
- 0x58 passes are weighted by the destination alpha the earlier passes wrote;
  the decoder draws them as plain additive layers (Ad = 1). If the VU lights
  alpha like colour (w lanes), lit passes would leave Ad ≈ 2 (texel alpha ×
  a saturated lit alpha); the captures suggest less for the wooden plinth.
- Morph key frames (second vertex set, as in cars) were not seen in prize
  models but are not checked for.

Materials: the GTM1 material records (Ma, Md, Ms, Me, shininess) are decoded and lit
like the cars' (the GTM1 VU formula). In the GT-mode prizes (`menu/champ_00..09`,
`menu/lic_*`, `race/g_*`) every material has Ms = 0 and shininess 0 except `champ_03`
material 0 (Ms 0.5, shininess 20, exponent 16), so the specular term only shows there.
