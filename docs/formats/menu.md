# Menu assets

Code: `src/formats/menu/` (decoders), `src/menu/` (static scene + SDL
renderer), `src/menu/flow/` (interactive runtime: focus, timings and
transitions as the original runs them; `gt3r --screen gt3menu`),
`src/app/screen_menu.cpp` (viewer, `gt3r --screen menu`),
`tools/gt3tool/cmd_menu.cpp` (`menu-pmb`, `menu-mbl`, `menu-imgs`,
`menu-shot`, `menu-survey`).

Coverage on PBPX-955.03: **960/963 .pmb** (the other 3 are headerless
fragments, see below), **51/51 .mbl**, **23/23 .imgs**; 4,374 pages,
151,461 elements, 31,435 embedded/IMGS images decoded with 0 failures.

## How a menu screen is assembled

- Screen layouts live in `menu/<LANG>/<screen><n>.pmb` (LANG = US, JP, BR,
  FR, GR, IT, SP; there is no UK folder under `menu/`). One file holds many
  **pages** (for example `menu/US/gt_mode2.pmb` holds the GT-mode home
  `hm/hm` and its sub-dialogs). Page names have the form `<group>/<page>`.
- The game loads a pmb by short name (`pause1`) and looks pages up by name
  (`pause_race`), then looks up **layers** (`defaultLayer`) and **elements**
  by name (`m_continue`, and `m_continue` + suffix for the selected look)
  to bind them to C++ objects of the `gt2k_menu` classes.
- `menu/<screen>.mbl` (language independent) describes the flow: for each
  page and each selectable item, the action string for game code and the page
  to go to.
- Images used by layouts are embedded in the pmb (Tex1 sets). IMGS files are
  named image libraries for widgets drawn by code (car pictures, course maps,
  icons, `race/<LANG>/display.imgs` for the HUD).

## .pmb layout

All values little endian; every "pointer" is a byte offset from the start of
the file (the game relocates them in place). Strings are NUL-terminated.

```
0x00 u32 page count
0x04 u32 offset of page pointer table (always 0x10)
0x08 u32 image count
0x0C u32 offset of image table (= file size when there are no images)
```

**Group** (used for both pages and layers):

```
+0x00 char name[N]
+N    u32 count_a; u32 ptr_a -> u32 ptr[count_a]
+N+8  u32 count_b; u32 ptr_b -> u32 ptr[count_b]
```

- Page: list A = page **items** (selectable elements, usually buttons), list
  B = **layers**.
- Layer: list A = **elements**, list B = **widgets** (elements with a widget
  class, see below).

N = 64 in every shipped file. 16 old Japanese files (`gt_mode.1.pmb`,
`default1.pmb`, `memcard1.pmb`, ...) use N = 32 everywhere; the parser tries
64 and falls back to 32. (Those files are not referenced by the game.)

**Element** (N + 0x2C bytes):

```
+0x00   char name[N]
+N+0x00 u16 type
+N+0x02 u16 flags
+N+0x04 f32 x, y, z, w, h
+N+0x18 u32 widget_class  -> u32 -> char[]  class / binding name
+N+0x1C u32 alt           -> Element         (type & 8)
+N+0x20 u32 image         -> ImageRef        (type & 1)
+N+0x24 u32 colors        -> u32[4]          (type & 2)
+N+0x28 u32 text          -> TextStyle       (type & 4)
```

- `type` and `flags` are the low and high halves of one u32 flag word read
  by the game:
  - `type` bits 0-3: parts present (verified for all 151k elements): 1 image,
    2 colored quad, 4 text, 8 alternative state. Bits 4-7: blend mode (0 normal,
    1 / 2 other alpha modes). Bits 8-11: static bevel decoration. Bits 12-15:
    focus style, 0x2000 pulsing highlight (most buttons), 0x1000 static.
  - `flags` bits 0-1: 1 = widget class/name payload present; bit 2: the
    texture entry is gzip compressed; bit 3: sets an item flag (unknown);
    bits 4-7: fade in when created (24 frames; only 0/1 occur); bits 8-11:
    fade out when dismissed (9 frames). So 0x1C = fade-in layer element,
    0x1D widget, 0x1E page item, 0x0F alt state (no fade).
- **Coordinates**: menu space is 640x448 (the NTSC frame buffer, shown at
  4:3) with the **origin at the bottom left, y up**. (x, y) is the bottom-left
  corner of the box, (w, h) its size. Evidence: titles have the largest y,
  alphabetical lists run top to bottom with decreasing y, full-screen
  backgrounds are (0, 0, 640, 448).
- **z** orders drawing; larger is in front. The game sets the selected
  variant of a button to z + 1.
- **Alt**: the element's other look (normally the focused button), with the
  same box. Drawn instead of the element when focused.
- **Widget class**: elements of a layer's list B point to a class/binding
  name such as `connect_status`, `event_status`, `scroll_bar`, `mpeg` (often
  equal to the element name, sometimes empty). Game code draws them (movies,
  car pictures, gauges); their own parts, if any, are drawn normally.

**ImageRef** (0x24 bytes): `u32 image index; f32 uv[4][2]`, in the same corner
order as the colors (top-left, top-right, bottom-right, bottom-left).
Coordinates are normalized to the texture's REGION_CLAMP rectangle (the
`Tex1::decode` crop). 86,540 of 86,649 refs are in 0..1; all are
axis-aligned.

**Colors**: four `u8 r, g, b, a` corner colors, same corner order. Alpha is
0..255 (0xFF opaque; pause dims use 0x7E). When an image is present, the
colors modulate it (0xFF = unchanged). The 4 colors differ (gradient) in
about 6% of elements.

**TextStyle** (0x10 bytes):

```
u16 unk (always 1)
u16 align       low nibble: 3x3 anchor, 0 top-left, 1 top-center, ... 4 center,
                ... 8 bottom-right. Flags 0x10 and 0x20 also occur (meaning
                unknown; 0x10 is seen on texts that look like ids, e.g. "home_doc")
u8  r, g, b, a  (a uses the GS scale: 0x7F/0x80 = opaque)
u32 text -> char[]   (game encoding: ASCII, {0x80+N, c} = ISO-8859-N char,
                     EUC-JP pairs; may be empty: filled by code)
u32 font -> char[]   font spec
```

Font specs: `<name>[#<variant>][*<xscale>[,<yscale>]]`, e.g. `sw_16#1`,
`sw_16#1*90`, `sw_16#1*65,75`, `sq_9#1`, `rank#1*80`. The number in the name
is the pixel size; scales are percent (a single value is assumed to scale the
width only). Some Japanese files keep a Windows font description
(`ＭＳ ゴシック|h-24|c128|...`). Font files are covered by the text
workstream.

**Image table**: `{u32 unpacked_size; u32 offset}` per image. The data at
`offset` is either a raw `Tex1` set or a gzip member containing one (26,878
gzip, 1,699 raw). Each set holds one texture.

### Layers and default visibility

Layer names seen: `defaultLayer`, `hidden` (elements shown by code, e.g. the
pause menu buttons), numbered sub-screens (`s_1`..`s_12`, `m_01`.., `info1`,
`layer1`..). The viewer shows only `defaultLayer` (plus page items) when a page
has one, otherwise every layer except `hidden`; others can be toggled.

## .mbl layout

```
0x00 u32 page count
0x04 u32 offset of page pointer table (8)
...
last u32 = file size
```

Page (0x10C bytes in every file the game loads):

```
+0x000 char name[64]        "group/page", matches the pmb page name
+0x040 s16 pmb number N     the layout is in menu/<LANG>/<set>N.pmb
+0x042 u16 flags            0x1 horizontal list, 0x2 wrap, 0x4 fade from black,
                            0x8 fade to black on leave, 0x30 clear mode,
                            0xC0 confirm feedback, 0x100 cancel allowed
+0x044 char import[64]      page drawn together with this one, from its own
                            pmb (e.g. "00cmn/status", the GT-mode status bar)
+0x084 char back[64]        page to return to on cancel ("return" = the page
                            we came from)
+0x0C4 char back_action[64] command run on cancel, e.g. "commit", "go_top"
+0x104 u32 item count; u32 -> Item*[]
```

Pages are sorted by name (the game binary-searches them). Every page of the
24 sets the game loads exists in its pmb (576/576) and every link, back and
import target exists (2026/2026; two `qm_*` pages keep a stale texture path in
the import field), checked by `menu_flow_disc_targets`.

Item (0x14C bytes, the size the game copies):

```
+0x000 char name[64]        element name in the pmb page (page-level item)
+0x040 char group[64]       pmb layer shown while the item is focused, sliding
                            in with the cursor direction (e.g. "m_01", "s_1")
+0x080 char layer[64]       pmb layer shown while the item is focused
                            (e.g. "layer1", "info1")
+0x0C0 char action[64]      command for game code, run on confirm
+0x100 char link[64]        page to switch to
+0x140 s16 id               only read by control code (-1, 1, 201, 501...)
+0x142 u16 kind             low nibble: 0 go to link, 1 open link as popup,
                            2 finish (control code takes over), 3 back,
                            4 multi container; bit 4 children horizontal,
                            bit 5 choice wraps
+0x144 u32 child count; u32 -> Item*[]   (multi-choice items, e.g. ON/OFF;
                            the children are page items too)
```

19 test files (`mtest3.mbl`, `qm_test01.mbl`, `default.mbl`, ...) use older,
smaller layouts (page header 0x84 or 0x64 bytes, items 0xAC bytes with
`name[64]`, `str40[32]`, `link[64]`); they are parsed and flagged `legacy`.

## IMGS layout

```
0x00 "IMGS"
0x04 u32 0
0x08 u32 entry count
0x0C u32 offset of first Tex1 set (= 0x10 + count * entry size)
0x10 entries: char name[S - 4]; u32 index     S = 0x20, or 0x40 in
     menu/<LANG>/arcade_menu.imgs (long car names)
...  one single-texture Tex1 set per entry, back to back, 16-byte aligned
```

Names are sorted; `index` is the ordinal of the Tex1 set. The first entry is
usually `00no_texture`.

## Known problems / open questions

- `menu/IT/name_entry1.pmb`, `qm_arc_result1.pmb` and `qm_gt_license1.pmb`
  (504, 280 and 592 bytes) have no header: they are fragments (pointers point
  outside the file). Other languages have full files.
- `menu/cars/ho0100` fails to decompress with the current gzip reader (not a
  menu layout; noted for the car workstream).
- Text align flags: 0x10 marquee, 0x20 typewriter (see the spec).
- Animation/transition data is not in the pmb: element fade flags select
  scripts built into the program; focus, confirm and page transitions are
  reimplemented in `src/menu/flow/` (`menu_controller.h`,
  `screen_transition.h` describe them) (`gt3r --screen gt3menu`).
- Language: the US program never changes the region index except from a
  save; `language.mbl`, `launch.mbl`, `default.mbl` are unused leftovers.
- Text is drawn with the game fonts (`src/text/game_font.h`, via
  `menu/menu_text.h`); the 3x3 anchor mapping is inferred from layouts, and
  the align flags 0x10/0x20 are ignored.
