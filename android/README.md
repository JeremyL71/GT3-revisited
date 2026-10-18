# Android build

Requirements: Android SDK with NDK 27 and CMake 3.22 or later (install both
from the SDK Manager), and **JDK 17 or 21**. Newer JDKs are not yet supported
by Gradle 8.

```bash
android/fetch-sdl.sh
```
```bash
cd android && ./gradlew assembleDebug
```

The APK is written to `app/build/outputs/apk/debug/`. Install it with
`adb install -r app/build/outputs/apk/debug/app-debug.apk`.

`fetch-sdl.sh` checks out SDL3 into `android/third_party/SDL`. That single
checkout supplies both SDL's Java glue (`org.libsdl.app`) and the native
library, so the two always match. The engine's root `CMakeLists.txt` builds
`libmain.so`, which `GT3Activity` loads.

## Using it

1. Copy your GT3 `.iso` to the device (or keep it on an SD card or USB drive).
2. Launch the app and tap **Open disc image...**. Android's file picker
   opens and the app reads the image through the Storage Access Framework, so
   no storage permission is needed.
3. Bluetooth and USB controllers (DualSense, Xbox, Switch Pro, 8BitDo and
   others) work immediately. The phone's accelerometer is available as the
   `tilt` binding source for steering.

The Android back button acts as Triangle (back). Without a controller,
on-screen touch controls appear: a D-pad and face buttons in menus, and a
steering zone (drag, or tilt with `[touch] steering = tilt`), pressure
pedals and shift buttons while racing. They hide when a controller is used.
See [docs/INPUT.md](../docs/INPUT.md#touch-controls-phones-and-tablets).

Tested on the Pixel emulators (API 36, arm64): Vulkan renderer, boot →
opening movie → title → Arcade → Time Trial → driving with touch controls
and tilt, back button, home and resume (the race pauses), rotation,
and the system file picker. Status and testing tips:
[docs/dev/PLATFORMS.md](../docs/dev/PLATFORMS.md).

## Release signing

Release APKs are signed with the project's own key, so every release can
update an installed copy. Create it once with `android/make-release-key.sh`
(the certificate names the project, not a person), keep the `.jks` and its
password in a password manager, and add two GitHub Actions secrets:

```bash
base64 -i gt3r-release.jks | gh secret set ANDROID_KEYSTORE_BASE64
```
```bash
gh secret set ANDROID_KEYSTORE_PASSWORD
```

Tagged builds (`v*`) then run `assembleRelease` with it. Locally, set
`GT3R_KEYSTORE` (path) and `GT3R_KEYSTORE_PASSWORD` before
`./gradlew assembleRelease`; without them the release APK is unsigned.
Debug builds keep using the local debug key.
