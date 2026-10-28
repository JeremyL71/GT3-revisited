# PSS movies (`stream/*.pss`) ✅

Decoder: `src/formats/pss.*` (demuxer), `src/video/` (MPEG-2 video, playback).
Tools: `gt3tool video-info | video-frames | video-yuv | video-psnr | video-wav | video-ads | video-survey`.

GT3.VOL holds 51 `.pss` files (about 1.4 GB, all stored uncompressed) under
`stream/`, plus `stream/course.ipic` (not a movie; see the end). The values
below are the NTSC-U disc's; the other releases differ as listed under
"Regional releases".

## Container

A PSS file is a plain **MPEG-2 program stream** (ISO/IEC 13818-1):

| Item | GT3 values |
|---|---|
| Pack size | 16384 bytes (every pack starts at a multiple of 0x4000); file size = packs × 16384 + 4 |
| Pack header | MPEG-2 (`00 00 01 BA`, 14 bytes, no stuffing), program_mux_rate 22500 (× 50 bytes/s = 9 Mbit/s) |
| System header | `00 00 01 BB` once, in the first pack |
| Video | stream `E0`, PES packets of up to 4090 bytes, PTS (and DTS on reference pictures) |
| Audio | stream `BD` (private stream 1), one packet every ~5.9 packs, PTS on every packet |
| Padding | stream `BE` at the end of the last pack(s) |
| End | `00 00 01 B9` (the trailing 4 bytes) |

### Audio in private stream 1

Each `BD` payload (after the PES header) starts with a 4-byte sub-header
`FF A1 00 00` (only value seen on the disc; byte 0 is the sub-stream id). The
remaining bytes of all `BD` packets, concatenated, form a **complete `.ads`
file**: `SShd` header (codec 0x10 PS-ADPCM, 44100 Hz, 2 channels, interleave
0x400) then `SSbd` and the body, split at arbitrary byte positions (each
packet carries 4073 audio bytes). The total equals `0x28 + SSbd size`
exactly. Audio decodes with the existing `AdsStream`/`PsAdpcmDecoder`.

The first audio PTS equals the first video PTS, so the two start together;
each packet's PTS is the time of the first sample that starts in it
(4073 bytes ≈ 3563.9 stereo samples ≈ 7273 ticks at 90 kHz, matching the
PTS steps).

Only 3 movies have audio: `opening.pss` (225.7 s), `ending_long_full.pss`
(232.7 s) and `ending_short_full.pss` (90.6 s). Course flyovers, menu and
title loops are silent (the game plays BGM over them).

## Video elementary stream

Encoded with TMPGEnc (user data `encoded by TMPGEnc b12c`). One sequence
header (with custom intra and non-intra matrices) at the start of each file,
then GOPs of 18 pictures (`I B B P B B P ...`, M=3).

| Field | Value |
|---|---|
| Profile/level | Main@Main (0x48), `progressive_sequence` 1, 4:2:0 |
| Size | 640×448 (50 files), 320×112 (`menu_bg2.pss`) |
| Frame rate | 30 fps (code 5) or 29.97 (code 4: `opening.pss`, the endings, `title_full.pss`, ...) |
| Bit rate | 9 Mbit/s (3 Mbit/s for `menu_bg2`), VBV 1835008 bits |
| Pictures | frame pictures only, `frame_pred_frame_dct` 1, `progressive_frame` 1 |
| Coding | `q_scale_type` 1 (non-linear), `intra_vlc_format` 1, zigzag scan, no concealment vectors |
| DC precision | 8, 9 or 10 bits (varies per file) |

**Field-split frames:** in 48 of the 51 files every frame holds two
640×224 fields stacked vertically (first field on top, the second 1/60 s
later below), i.e. 60 Hz interlaced video stored as half-height pictures.
Only `ending_long_full`, `ending_short_full` and `menu_bg2` are ordinary
progressive frames. No header field distinguishes them (see
`src/video/field_layout.h` for the detection).

So only frame motion compensation with frame DCT is needed; no field or
dual-prime prediction, no field pictures.

Coverage: all 51 files decode with zero slice errors, frame counts identical
to FFmpeg (39,307 frames total). See `docs/dev/MOVIES.md` for the accuracy
comparison.

## Regional releases

| | NTSC-U | NTSC-J | PAL |
|---|---|---|---|
| Files | 51 | 51 | 49 (no `arcade_full`, `level_full`, which the game does not play) |
| Size | 640×448 | 640×448 | 640×512 |
| Frame rate | 30 or 29.97 fps (the opening, endings and title 29.97) | 30 fps | 25 fps (code 3) for all |
| Field-split | 2 × 640×224, 1/60 s apart | as NTSC-U | 2 × 640×256, 1/50 s apart, same order |
| `menu_bg2.pss` | 320×112, 360 pictures | as NTSC-U | 320×112, 300 pictures (the same 12 s) |
| Audio | 44100 Hz | 48000 Hz, tracks of their own (opening 184.5 s) | 44100 Hz (opening 216.8 s; endings as long as NTSC-U's) |
| `course.ipic` | 42 stills, 640×448 | none | 42 stills, 640×512 |

PAL's opening runs 219.2 s of pictures (5,479) against its 216.8 s of audio;
the endings have 232.7 s and 90.6 s of audio over 224.3 s and 91.2 s of
pictures, as on NTSC-U (the picture stays on its last frame while the music
ends). All 49 PAL movies decode with zero slice
errors (31,955 pictures). The MPEG headers do not tell the TV system (both
releases mark the pictures square-pixel and NTSC); the frame rate does.

## `stream/course.ipic` ✅ (`src/video/ipu.*`)

42 course preview stills (640×448, progressive) as PS2 IPU pictures.

```
ipua archive:  0x00 "ipua"  0x04 u32 count (42)
               0x08 count × { u32 offset; char name[28] }   (names: akasaka, r_route5_dry, ...)
               pictures at 64-byte aligned offsets (first at 0x580)
ipum picture:  0x00 "ipum"  0x04 u32 data size  0x08 u16 width  0x0A u16 height
               0x0C u32 frame count (1)  0x10 IPU bitstream
```

IPU bitstream (the PS2 IPU's intra decoding, as FFmpeg's `ipu` decoder also
reads it): `u8 flags` — bits 0–1 intra_dc_precision, `0x04` dct_type bit per
macroblock, `0x10` alternate scan, `0x20` intra_vlc_format, `0x40`
q_scale_type, `0x80` MPEG-1 style blocks — then every macroblock in raster
order with no slices: a `1` (address increment) before all but the first,
macroblock type `1` (intra) or `01` (intra + 5-bit quantiser_scale_code),
optional dct_type, then six MPEG-2 intra blocks. DC predictors run across the
whole picture; the default intra matrix applies. All 42 GT3 stills use flags
`0x62` (10-bit DC, non-linear quantiser, intra VLC table one).

Verified: 42/42 decode; against FFmpeg's IPU decoder PSNR ≥ 87.7 dB with
`-idct int` (≥ 65.5 dB default), max sample difference 1.

## Movies referenced by the game program

The disc's CORE.GT3 file holds a 7-entry table of menu movies, played by
the menus' movie widget: records of `{char* path; u32 width; u32 height; u32 flag; u32; u32}`
(0x18 bytes). `flag` is 1 exactly for the 640×448 field-split ones:

| Path | Size | flag |
|---|---|---|
| `stream/arcade_menu_full.pss` | 640×448 | 1 |
| `stream/level_menu_full.pss` | 640×448 | 1 |
| `stream/laguna_full.pss` | 640×448 | 1 |
| `stream/menu_bg2.pss` | 320×112 | 0 |
| `stream/menu_bg.pss` | 640×224 | 0 (not on the disc) |
| `stream/arcade_top.pss` | 320×448 | 0 (not on the disc) |
| `stream/arcade_level.pss` | 320×448 | 0 (not on the disc) |

Other references: `stream/opening.pss` (boot), `stream/title_full.pss`,
the two endings, `<course>_full.pss` built from course names (flyovers),
and `stream/course.ipic`.
