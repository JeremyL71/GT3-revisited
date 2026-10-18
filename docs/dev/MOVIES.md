# Movies (FMV playback)

GT3's FMVs are the 51 `stream/*.pss` files: MPEG-2 program streams with
MPEG-2 video and PS-ADPCM audio (format details: [../formats/pss.md](../formats/pss.md)).
They are played from the user's disc through the VFS, decoded by our own
code, with no extra dependency.

## Decoder choice

**Decision: a compact MPEG-2 video decoder of our own (`src/video/`, ~800
lines plus tables).**

Requirements were: all targets (Windows, macOS, Linux, Android), nothing for
the user to install, licence compatible with an open-source project, and
real-time playback on phones.

| Option | Verdict |
|---|---|
| Own decoder (chosen) | MP@ML with only what the disc uses is small: VLC tables, IDCT, dequantisation, half-pel MC, I/P/B reordering. No build or licence questions, identical on every platform, and fast enough (below). |
| FFmpeg (libavcodec) via FetchContent | LGPL/GPL, autotools build that does not fit CMake FetchContent; cross-compiling for Android/Windows is a project of its own. Far too heavy for 51 files. |
| libmpeg2 | GPL-2 (would force the whole project to GPL), unmaintained since 2008, needs autotools. |
| pl_mpeg (single header, MIT) | Licence and build would be fine, but it decodes MPEG-1 only. |
| Platform decoders (MediaCodec, VideoToolbox, Media Foundation) | MPEG-2 support is optional/absent on many devices (Android drops it), four code paths, and PS2-specific audio still needs our code. |

What is supported (and what throws `FormatError` instead): MPEG-2 Main
Profile 4:2:0 frame pictures, I/P/B, frame motion vectors with half-pel
interpolation, frame and field DCT, linear/non-linear quantiser scale, both
intra VLC tables, zigzag and alternate scan, loaded or default quantiser
matrices (sequence header and quant matrix extension), concealment vectors
(parsed), open/closed GOPs and broken links, PTS tracking and display
reordering. Not supported: MPEG-1 streams, field pictures, field/dual-prime
motion compensation, 4:2:2/4:4:4, scalability. None of these occur on the
disc (surveyed: every picture coding extension of all 51 files).

## Code map

| File | Role |
|---|---|
| `src/formats/pss.*` | Program-stream demuxer: packs, PES headers, PTS/DTS, private stream 1 sub-header; streams via a `ReadFn` |
| `src/video/bit_reader.h` | MSB-first bit reader over zero-padded units |
| `src/video/mpeg2_tables.*` | Annex B VLC tables as direct lookup tables, scans, default matrix, non-linear quantiser scale |
| `src/video/idct.*` | 8×8 IDCT, 32-bit fixed point, Loeffler/Ligtenberg/Moschytz factorisation (13-bit constants, 2 guard bits between passes); double-precision reference for tests |
| `src/video/mpeg2_decoder.*` | Start-code unit assembly, headers, slices, macroblocks, MC, reordering, output frames (YUV 4:2:0) |
| `src/video/movie.*` | Demuxer + decoder + `AdsStream` audio: pull video frames and PCM independently; `open_movie(Vfs&, path)` |
| `src/video/movie_player.*` | Worker thread, SDL audio stream, A/V clock, IYUV texture upload |
| `src/video/field_layout.*` | Detects field-split movies (two fields stacked per frame) |
| `src/video/ipu.*` | IPU stills: `ipum` pictures, `ipua` archive, `CoursePreviews` |
| `src/video/yuv.*` | YUV→RGBA for PNG export |
| `src/app/screen_movies.cpp/.h` | `--screen movies` (movie list, Square: course stills), `--screen boot`, `play_movie()` |
| `tools/gt3tool/cmd_video.cpp` | `video-info`, `video-frames`, `video-yuv`, `video-psnr`, `video-wav`, `video-ads`, `video-survey`, `video-ipu` |
| `tests/test_video.cpp` | Tables, IDCT accuracy, synthetic streams, disc checks |

### Data flow

```
Volume::read_range ──► PssDemuxer ──► E0 payload + PTS ──► Mpeg2Decoder ──► Frame queue (≤ 6) ──► IYUV texture
   (64 KB reads)              └──► BD payload ──► byte FIFO ──► AdsStream ──► SDL_AudioStream (≈0.25 s queued)
```

* **Streaming:** nothing is loaded whole. The demuxer reads 64 KB chunks;
  video packets are queued only until the decoder consumes them; the audio
  FIFO is trimmed as `AdsStream` reads forward. `open_movie` uses
  `Vfs::open_stream`, so movies replaced or added by mods stream too.
* **Decoder input:** `push()` takes ES bytes in any chunking. The decoder
  splits at start codes and decodes each slice as soon as the next start
  code arrives, so it never buffers a whole picture.
* **Timestamps:** a PES PTS is attached to the first picture start code
  that begins in that packet; pictures without one get previous + frame
  period. Frames come out in display order with their PTS.

### Playback and A/V sync (`MoviePlayer`)

A worker thread keeps ≈0.25 s of PCM queued in an SDL audio stream the
player owns (opened on the default device, independent of `AudioEngine`;
gain = music volume) and up to 6 decoded frames ahead. Audio has priority
so it never underruns while video decodes.

On the main thread `update()` advances the clock by wall time and, while
audio plays, steers it toward `audio start offset + (samples put − samples
still queued) / rate` (5% per frame; jumps if more than 100 ms off — the
raw audio position moves in device-buffer steps of 10–20 ms);
silent movies (48 of 51) and the tail after the audio ends use the wall
clock. The newest frame whose PTS is due is uploaded with
`SDL_UpdateYUVTexture` (IYUV, BT.601 limited range, SDL's default for YUV
textures); late frames are dropped. Pause stops the audio device and freezes
the clock. `start_seconds` decodes and discards up to a start time (used for
screenshots; there is no index for real seeking).

### Field-split movies

48 of the 51 movies are *field-split*: each 640×448 MPEG frame is two
640×224 pictures stacked — the first field on top, the second field (1/60 s
later) below. The PS2 sends them to the TV as the two interlaced fields; FFmpeg
shows the stacked halves as-is. Nothing in the stream marks this, so
`FieldLayoutDetector` (`src/video/field_layout.*`) decides per movie from the
first 16 non-flat frames: the luma rows on either side of the middle seam
are discontinuous in field-split frames (ratio of seam difference to
neighbouring row differences > 3). Survey: the 48 field-split movies have
≥ 94% of frames above the threshold, the 3 progressive ones (`ending_long_full`,
`ending_short_full`, `menu_bg2`) ≤ 1.6%. The player then draws the top half
for the first half of each frame period and the bottom half for the second
("bob", 60 pictures/s, each field stretched to the full height).
`MoviePlayer::picture_rects()`/`draw()` place each field with its half-line
offset (first field on even frame lines, second on odd: source rows mapped
with +1/4 resp. −1/4 field line, drawn over frame rows [0, H−1) resp.
[1, H)), so static detail does not bob up and down; the rectangles stay
inside each field's half of the texture, so the halves never bleed into
each other. CORE.GT3's own movie table carries the same distinction as a
per-movie flag (pss.md).

## Game-flow API

| Use | API |
|---|---|
| Full-screen movie, then continue | `play_movie(app, "stream/opening.pss", on_done, options)` / `make_movie_screen(...)` in `src/app/screen_movies.h`. Pops itself at the end, on skip (Cross/Start/Circle unless `skippable = false`) or if the file cannot be opened, then calls `on_done(app)`. `loop = true` for attract loops. |
| Boot | screen id `boot` (`make_boot_screen()`): opening movie, then the `gt3menu` screen |
| Movie inside a menu page | `video::MoviePlayer`: `start_loop([vfs]{ return video::open_movie(*vfs, "stream/menu_bg2.pss"); })`, `update(renderer)` every frame, `draw(renderer, rect)` (handles field-split movies) |
| Course preview stills | `video::CoursePreviews`: `load(vfs)`, `image("akasaka")` → `RgbaImage` (640×448 RGBA, cached) |

`GT3R_SKIP_MOVIES=1` makes game-flow movies end immediately (development).

`gt3tool video-frames` exports the raw stacked frames.

## Verification

### Video vs FFmpeg (all 51 files)

`gt3tool video-psnr` decodes a movie and compares it frame by frame with raw
`yuv420p` from FFmpeg 8 (Homebrew) piped in:

```bash
ffmpeg -v error [-idct int] -i x.pss -f rawvideo -pix_fmt yuv420p - | gt3tool video-psnr <iso> stream/x.pss - q
```

| Reference | Frames | Frame count match | PSNR min | PSNR mean | Max sample diff | Identical frames |
|---|---|---|---|---|---|---|
| FFmpeg `-idct int` (same IDCT family) | 39,307 | 51/51 | 77.6 dB | 86.7 dB | 2 | 364 |
| FFmpeg default (`simple` IDCT) | 39,307 | 51/51 | 58.4 dB | 63.0 dB | 7 | 294 |

Differences come only from IDCT rounding (MPEG-2 allows any IEEE 1180
compliant IDCT, and the error accumulates through P/B chains until the next
I picture). Against FFmpeg's `int` IDCT (libjpeg's integer LLM, the same
algorithm with different special-case arithmetic) PSNR is above 77 dB with
at most ±2 per sample, which shows everything else — VLC parsing,
dequantisation, mismatch control, motion compensation, rounding of
bidirectional prediction, reordering — matches exactly. Full per-file results
come from `gt3tool video-psnr` against FFmpeg's decode of every file.

### Audio

The concatenated `BD` payloads (`gt3tool video-ads`) are recognised by FFmpeg
as an `.ads` file (`adpcm_psx`, 44100 Hz, stereo), with the same sample count
as our decoding (`gt3tool video-wav`) for all 3 movies with audio. The
existing PS-ADPCM decoder rounds `(x + 32) >> 6` where FFmpeg truncates, so
samples differ by about ±24 on average, plus a few hundred samples (of 20 M)
around full-scale clipping in `opening.pss`/`ending_long_full.pss` where the
two decoders saturate history differently (`src/formats/ps_adpcm.*`, not
owned by this workstream).

### Tests

`tests/test_video.cpp` (filter `video`): VLC table integrity, IEEE 1180-style
IDCT accuracy against the double-precision reference, a hand-encoded
synthetic MPEG-2 stream (I/P/B, skipped macroblocks, reordering, PTS, a
half-pel vector), a
synthetic program stream for the demuxer (packs, padding, resync, private
stream 1 sub-header), a synthetic movie with audio, and disc tests (skipped
without `GT3R_DISC`): `menu_bg2.pss` fully (360 frames), `opening.pss` start
with audio, and the first frames of every movie. 9/9 pass with the disc
(6 pass, 3 skip without), ~0.4 s. The IDCT meets every IEEE 1180 limit
(peak error 1, worst per-pixel MSE 0.017 vs 0.06, overall MSE ≤ 0.014 vs
0.02, means ≤ 0.0035 / 0.00026 vs 0.015 / 0.0015).

The Android
target (`build-android`, libmain.so) builds with the movie code.

## Performance

Single-threaded decode, RelWithDebInfo (`-O2`), Apple Silicon Mac,
`gt3tool video-survey` (demux + decode + frame copy, all 51 files, audio
included): **39,307 frames (1,311 s of video) in 29.8 s = 1,320 fps, 44×
real time**, i.e. about 0.75 ms per 640×448 frame (worst single frame ≈ 4 ms).

In the app (scratch harness running `MoviePlayer` at a 60 Hz render loop
for 12 s): `opening.pss` 0 dropped frames, decode avg 1.0–1.4 ms, presented
frame within 3–9 ms of the audio clock; `akasaka_full.pss` (wall clock)
0 dropped.

Mobile: a mid-range ARM64 phone core is roughly 3–5× slower than an M-series
core on scalar integer code, i.e. ≈4–5 ms per frame against a 33 ms budget,
on a worker thread. The IYUV upload is 430 KB per frame (13 MB/s). Further
headroom if ever needed: NEON for MC/IDCT, skipping the IDCT for DC-only
blocks, or decoding into the texture directly.

## Next steps

* Seeking (byte-ratio seek to a pack, resync on the next I picture and the
  audio interleave block whose PTS matches).
