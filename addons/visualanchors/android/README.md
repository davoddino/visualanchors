# VisualAnchors Plugin

This plugin provides QR code detection and pose estimation capabilities for Godot on Android. It streams luma frames from Camera2, detects QR markers with OpenCV 4.x, and solves the pose with IPPE (`SOLVEPNP_IPPE_SQUARE`) using calibrated intrinsics and distortion. The resulting transform is converted to Godot's XR coordinate frame and smoothed with One-Euro + SLERP filters before being emitted to GDScript.

Key features:
- Native OpenCV 4.x backend (C++) compiled via the Android NDK.
- Sub-pixel corner extraction and deterministic TL/TR/BR/BL ordering.
- IPPE pose estimation with cheirality filtering and reprojection gating.
- Smoothing (One-Euro translation, quaternion SLERP) with configurable parameters.
- Strict configuration checks: intrinsics, distortion, QR size, and external camera transform are all required before streaming starts.

## Repository layout

- `bin/visualanchors-*.aar` – prebuilt debug/release binaries shipped with the addon.
- `src/main/main/java/` – Android Java glue (configuration, Camera2 controller, JNI bridge, smoothing).
- `src/main/main/cpp/` – Native OpenCV pipeline (`va_qr_detector`, `va_pose`, `va_jni`).
- `src/main/main/jniLibs/` – Output directory for compiled `.so` artefacts (populated by Gradle/CMake).

To rebuild the AAR locally use the project-level Gradle wrapper:

```
./gradlew assembleRelease assembleDebug -p addons/visualanchors/android/src
```

Make sure the Godot template AARs exist under `android/build/libs/` (they are generated when you install the Android build template from the editor).

### OpenCV dependency

Set `OpenCV_DIR` to point to the Android OpenCV SDK (e.g. `export OpenCV_DIR=/path/to/OpenCV-android-sdk/sdk/native/jni`) before invoking Gradle so CMake can locate the static/shared libraries. Only the `arm64-v8a` ABI is produced by default; adjust `abiFilters` in `build.gradle` if you need additional architectures.

After rebuilding, copy the fresh `visualanchors-*.aar` files into `addons/visualanchors/android/bin/` so the export plugin can pick them up.
