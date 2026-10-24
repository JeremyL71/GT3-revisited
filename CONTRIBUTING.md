# Contributing

Thanks for helping. GT3 Revisited is an independent reimplementation written
from documented behaviour (the disc's file formats and how the game
behaves), verified against the original's observable behaviour. It contains
no original code, data or assets; game content is read from the player's
own disc at runtime. The rules about what may go into the repository keep it
that way, so please read them first.

## Project rules

1. **Never commit original code**, in any form: not the game program or
   parts of it, not listings or translations of it, not code written by
   transcribing it. This applies to code, docs, comments, issues and pull
   requests.
2. **Never commit game data**: no files from the disc, extracted assets,
   textures, sounds, hex dumps of assets, screenshots of game textures used
   as fixtures, or test files cut from the disc. Tests that need game data
   read the user's disc at run time (`test_disc()`, `GT3R_DISC`) and skip
   without it.
3. **Record behaviour and facts, then implement from that description.**
   File layouts, constants, table values, algorithms described in prose and
   observed behaviour are fine to record. File formats go in
   [docs/FORMATS.md](docs/FORMATS.md) and [docs/formats/](docs/formats/);
   how a system behaves goes in [docs/behaviour/](docs/behaviour/), and how
   ours implements it in the notes in [docs/dev/](docs/dev/). Code comments
   cite behaviour sections as `<page> §n` (for example `physics §4.2`).
4. Anything you extract from your own disc while working (`gt3tool extract`,
   screenshots, logs) stays outside the repository.

## Building

Requirements: CMake 3.21 or later, a C++20 compiler and SDL3 (3.4 or later;
fetched and built by CMake if it is not installed). zlib is optional.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
```

Per-platform notes (macOS, Windows, Linux packages, the MinGW cross build,
Android and release packaging) are in [README](README.md#building),
[docs/dev/PLATFORMS.md](docs/dev/PLATFORMS.md) and
[android/README.md](android/README.md).

## Tests

```bash
ctest --test-dir build                                   # unit tests, no game data
GT3R_DISC="/path/to/image.iso" ./build/gt3r_tests        # also the disc tests
GT3R_DISC="/path/to/image.iso" ./build/gt3r_tests ai_    # tests whose name contains "ai_"
```

- The **unit tests** need no game data and run in CI on Linux, Windows and
  macOS. Some build small worlds in the disc's formats from invented values
  (`tests/*_synth.h`, `tests/*_fixtures.h`) to run the physics, the AI and
  the GT Mode menus without a disc.
- The **disc tests** read your own disc image (`GT3R_DISC`) and check the
  decoders and game systems against real data: every file of a format
  parses, every car builds, races run, saves round-trip. They skip when no
  disc is configured. Run them before sending changes to formats, physics,
  AI, menus or GT Mode.
- Online code has fuzz targets (`-DGT3R_FUZZ=ON`, see
  [docs/dev/ONLINE.md](docs/dev/ONLINE.md#fuzzing)).

To check a screen visually: `gt3r <iso> --screen <id> --shot out.png`.

## Code style

- C++20, `namespace gt3r`, 4-space indent, about 120 columns, `snake_case`
  functions, `PascalCase` types, `trailing_underscore_` members. Match the
  code around you.
- The build stays warning-free (`-Wall -Wextra`, `/W4` on MSVC).
- All asset loads go through the `Vfs` (in the app) or `Volume` (in tools),
  so mods keep working.
- New screens register with `GT3R_REGISTER_SCREEN` (`src/app/screen_registry.h`),
  new `gt3tool` commands with `GT3TOOL_COMMAND` (`tools/gt3tool/tool.h`),
  tests with `TEST` (`tests/test_framework.h`).
- Comments explain *why*; header comments document file layouts.

## Pull requests

- One topic per pull request, with a description of what changed and how
  you verified it (tests, disc tests, screenshots of our own screens).
- Add or update tests for behaviour changes, and update the docs that
  describe the area (`docs/behaviour/`, `docs/formats/`, `docs/dev/`, [MODDING](docs/MODDING.md)).
- Say where behaviour deliberately differs from the original, and why.
- CI must pass on all platforms.
- By contributing you agree that your work is released under the
  [MIT License](LICENSE).
