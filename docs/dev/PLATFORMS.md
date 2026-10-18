# Platforms

Status of each target, how to build and test it, and known issues. "Verified"
means built *and* run here; "built" means compiled and unit-tested but not
run interactively; "unverified" means neither.

## Status (2026-10-16)

| Target | Compiler | Status | Evidence |
|---|---|---|---|
| macOS 26 arm64 | Apple clang 21 | ✅ verified | Daily development platform; full game, Metal |
| macOS arm64 | Homebrew GCC 16 (libstdc++) | ✅ built, tests pass | `-Wall -Wextra` clean, `gt3r_tests` 0 failures |
| Linux x86_64/arm64 (Ubuntu 24.04) | GCC 13 | ✅ built, tests pass | Docker `ubuntu:24.04` with the CI apt list; SDL 3.4.12 fetched; warning-free; ctest passes |
| Linux (Ubuntu 24.04) | clang 18 | ✅ built, tests pass | Same container, `CC=clang CXX=clang++` |
| Linux desktop session | | ⚠️ unverified | Not run on a real X11/Wayland desktop (no display in the container) |
| Windows x64 | MinGW-w64 GCC 13 (cross) | ✅ built | All executables link; see below for the wine test run |
| Windows x64 | MSVC 2022 | ✅ verified | CI (windows-2022) and a real PC (Windows 11): all tests pass with the disc, 3D (Vulkan, RTX 3080) and online races verified |
| Android arm64 (API 36 emulator) | NDK 27 clang | ✅ verified | Boot → opening movie → title → Arcade → Time Trial → driving with Vulkan; touch controls, tilt, back, home/resume, rotation, file picker |
| Android x86_64 | NDK 27 clang | ✅ built | Part of the same APK; not run |
| Android real device | | ⚠️ unverified | Emulator only. Frame time must be measured on hardware (see below) |
| iOS | | ❌ not started | SDL supports it; touch controls are already platform-neutral |

## Building and testing

### macOS

```bash
brew install sdl3 cmake
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
ctest --test-dir build
```

Second compiler (catches missing includes that libc++ hides):

```bash
brew install gcc
CC=gcc-16 CXX=g++-16 cmake -S . -B build-gcc -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build-gcc -j -- -k
```

This needs Homebrew's SDL3: GCC cannot build SDL's Objective-C Cocoa
backend, so the FetchContent fallback does not work with GCC on macOS.

### Linux

Install the SDL build dependencies (the list CI uses), then build as usual:

```bash
sudo apt-get install ninja-build pkg-config libasound2-dev libpulse-dev libpipewire-0.3-dev \
  libx11-dev libxext-dev libxrandr-dev libxcursor-dev libxfixes-dev libxi-dev libxss-dev libxtst-dev \
  libwayland-dev libdecor-0-dev libxkbcommon-dev libegl1-mesa-dev libdrm-dev libgbm-dev \
  libudev-dev libdbus-1-dev zlib1g-dev
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build && ctest --test-dir build
```

SDL 3.4 needs `libxtst-dev` (configure fails without it). Without Wayland,
X11 or Vulkan dev packages SDL drops those backends silently.

From macOS, the same build runs in Docker (`colima start`, then
`docker run -v $PWD:/src:ro ubuntu:24.04 ...` with the commands above).

### Windows

Visual Studio 2022: `cmake -S . -B build` then build `gt3r` (or open the
folder in VS). SDL3 and zlib are fetched. The renderer uses Vulkan on
Windows (we ship SPIR-V and MSL only, no DXIL yet; see RENDERING.md), so a
GPU driver with Vulkan support is needed for 3D. Without it the game runs
2D-only.

MinGW cross build (Windows executables from macOS or Linux; `brew install mingw-w64` on
macOS), with the toolchain file in `cmake/mingw-w64.cmake`:

```bash
cmake -S . -B build-mingw -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64.cmake -DCMAKE_BUILD_TYPE=Release \
      -DGT3R_FETCH_SDL=ON -DCMAKE_EXE_LINKER_FLAGS="-static -static-libgcc -static-libstdc++"
cmake --build build-mingw --target gt3r gt3tool gt3r_tests gt3r-relay
```

- `GT3R_FETCH_SDL=ON` is needed: without it CMake finds the host's own SDL3
  (e.g. Homebrew's).
- The static `gt3r.exe` needs only system DLLs (UCRT, built into Windows 10
  and 11).

Running the Windows build on a Mac with CrossOver's Wine (a separate bottle):

```bash
CX=/Applications/CrossOver.app/Contents/SharedSupport/CrossOver/bin
$CX/cxbottle --bottle gt3r-test --create --template win10_64
GT3R_DISC='Z:\path\to\image.iso' $CX/wine --bottle gt3r-test --wait-children "$PWD/build-mingw/gt3r_tests.exe"
```

The full suite passes this way with the disc, including every online test
(Winsock, the receive thread, the relay). The game starts and draws its menus,
but 3D is unavailable: we ship SPIR-V and MSL only, and SDL's GPU backend does
not accept CrossOver's Vulkan (MoltenVK). A D3D12 path (DXIL shaders) would
render there through CrossOver's D3DMetal, and on Windows machines without
Vulkan.

### Android

See [android/README.md](../../android/README.md). Short version:

```bash
export JAVA_HOME=$(/usr/libexec/java_home -v 21)   # JDK 17 or 21
android/fetch-sdl.sh                                 # SDL 3.4.12 into android/third_party/SDL
cd android && ./gradlew assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

Testing on an emulator without the file picker: the app reads extra
command-line arguments from `<user data>/args.txt`, one per line. On a
debug build:

```bash
adb shell "run-as org.gt3revisited.gt3r sh -c 'echo /data/local/tmp/gt3.iso > files/args.txt'"
```

The app can read world-readable files in `/data/local/tmp`. The full image
(3.9 GB) rarely fits on an emulator; a sparse copy that keeps the VOL
header and only the files a test needs works (create it with `truncate` on
the device and `dd ... conv=notrunc` the needed ranges in). Remove it and
uninstall the APK afterwards. `adb shell input keyevent` presses are
shorter than a frame; both keyboard and touch input latch such presses so
injected events register.

Useful checks: `adb emu sensor set acceleration x:y:z` for tilt steering,
`settings put system user_rotation N` (with `accelerometer_rotation 0`)
for orientation, `logcat --pid=$(pidof org.gt3revisited.gt3r)` for the
engine log ("Race perf" lines every 3 s with `[race] perf=1` or after
pressing R3).

## CI

`.github/workflows/build.yml`:

- **desktop** (ubuntu-24.04, windows-2022, macos-14): configure, build
  `gt3r gt3tool gt3r_tests`, run ctest (tests that need a disc skip).
- **android**: JDK 21, installs NDK 27.0.12077973 + CMake 3.22.1 with
  sdkmanager, fetches SDL, `assembleDebug`, uploads the APK.

Validated locally with `actionlint`; the Linux job's commands were run in
Docker, the Android job's on macOS. GitHub Actions itself was not run.

## Release packages

No game data is in any package. `cmake/Packaging.cmake` adds the icon, the
macOS `Info.plist` (`packaging/Info.plist.in`) and install rules;
`GT3R_FETCH_SDL=ON` builds SDL from source so the result doesn't depend on a
system or Homebrew SDL.

| Package | Contents | Build |
|---|---|---|
| macOS (arm64, macOS 12+) | `gt3r.app` (icon, Info.plist, `.iso` document type), `gt3tool`, README | `cmake -S . -B build-rel -DCMAKE_BUILD_TYPE=Release -DGT3R_FETCH_SDL=ON -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0`, build `gt3r gt3tool`, `cmake --install build-rel --prefix dist`, `ditto -c -k --keepParent` |
| Windows x64 | `gt3r.exe` (icon resource), `gt3tool.exe`, `SDL3.dll`, README | same with `-DGT3R_SDL_SHARED=ON`; zip |
| Linux x86_64 | `bin/gt3r`, `bin/gt3tool`, `share/applications/gt3r.desktop`, icon, README | same; `tar czf` (built on Ubuntu 22.04 for an older glibc) |
| Android | `gt3r-<tag>-android.apk`, signed with the project's release key (CI secrets; [android/README.md](../../android/README.md#release-signing)) | `./gradlew assembleRelease` |

CI job `package` / `package-android` builds these on `v*` tags and manual
runs (`workflow_dispatch`) and uploads them as artifacts. The macOS app is
not code-signed or notarized: first launch needs right-click → Open. The
icon is original artwork drawn by `packaging/make_icon.py` (also writes the
Windows `.ico` and the Android launcher icons).

Verified locally: the macOS package (self-contained: `otool -L` shows only
system frameworks; the bundle launches, `plutil -lint` OK); the Linux tarball
and the Windows zip from a MinGW cross build in Docker (see below for what
they contain). The MSVC-built Windows package and an AppImage are not made.

## Windows / MSVC review

Checked by reading the code before the first MSVC build (which then built and
passed every test):

- Disc access goes through `SDL_IOFromFile` with 64-bit offsets, so the
  3.9 GB image is fine; host files use `SDL_LoadFile`/`SDL_SaveFile`
  (UTF-8 paths). No `<filesystem>`, `fstream`, `fseek`/`ftell` in the game.
- No POSIX-only headers or calls, no `M_PI` (code uses `std::numbers`), no
  `__builtin_*`/VLAs; `__attribute__` uses are guarded; `%zu`/`%llu` formats
  are correct for MSVC's UCRT; no `windows.h` includes (so `min`/`max`
  macros and `PadRumble::small` cannot clash).
- `_CRT_SECURE_NO_WARNINGS` is defined for MSVC (fopen/getenv/localtime
  deprecation noise).
- Dev tools only: `gt3tool` opens some paths with
  `fopen` (ANSI code page on Windows, so non-ASCII paths fail there).

## Known issues

- **Android frame time (emulator):** the API 36 arm64 emulator renders the
  race at 5–8 fps (120–200 ms/frame) at 2560×1600 with gfxstream Vulkan.
  CPU scene time is ~11 ms; MSAA 1 vs 4 makes no difference, so the
  emulator's GPU translation is the bottleneck. Not representative; needs a
  real device. `[video] render_scale` (auto ≈ 900 lines on mobile) brought
  the 20:9 phone-shaped emulator from ~150 to ~88 ms per frame.
- **Touch overlay vs HUD:** resolved. Controls use the side bars on wide
  phones, and the race HUD insets (and on 16:9 shrinks) its meters into
  `App::touch().safe_area()`. Menus accept direct taps (tap to focus, tap
  again to confirm) as well as the D-pad overlay.
- **Emulator "gamepad":** the Android emulator exposes its keyboard as a
  gamepad (`qwerty2`), so the overlay starts hidden there and appears on
  the first touch.
- **Portrait on Android 16 tablets:** the activity is landscape-only; when
  the system forces portrait the game is letterboxed (works, no crash).
- **Tilt direction on gamepads — verified on a DualSense (2026-10-11,
  Bluetooth, macOS):** resting on its side with the right grip down (turned
  clockwise as seen by the player), the accelerometer read
  (-8.78, 4.36, 0.97) m/s² and `steering_roll` gave +63°, i.e. steer right,
  as intended. DS4 and Switch Pro are not tried; they share SDL's axis
  convention.
- **Force feedback — partly verified:** on the same DualSense, SDL reported a
  PS5 pad with accelerometer, gyro and rumble, and accepted every command we
  send (`SDL_SendGamepadEffect` returned success for the brake resistance,
  the ABS vibration and the off packet; light bar and rumble also succeeded).
  How the effects *feel* has not been judged by a person yet. Xbox impulse
  triggers and wheel haptics are untested with real devices; the wheel force
  direction may need `[ffb] invert = on` on some wheels.
- Real Linux desktop sessions and real Android devices are unverified (see
  the table).
