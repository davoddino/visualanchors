# VisualAnchors Plugin

This plugin provides QR code detection and pose estimation capabilities for Godot on Android. It processes camera image buffers (RGBA or LUMA) to detect QR codes, extract their text payload and corner positions, and estimate the 3D pose (rotation and translation) of the QR code relative to the camera using provided camera intrinsics. This enables applications like augmented reality tracking or visual anchoring where QR codes serve as markers.

Key features:
- Detect QR codes from camera frames.
- Retrieve payload text and pixel coordinates of corners.
- Compute 3D pose matrix for positioning in 3D space.
- Configurable QR physical size and camera parameters.

## Repository layout

- `bin/visualanchors-*.aar` – prebuilt debug/release binaries shipped with the addon.
- `src/` – Android library project with the Java sources (Camera2 controller, ZXing wrapper, etc.).
- `src/libs/zxing-core-3.5.2.jar` – bundled ZXing dependency used at build time.

To rebuild the AAR locally use the project-level Gradle wrapper:

```
./gradlew assembleRelease assembleDebug -p addons/visualanchors/android/src
```

Make sure the Godot template AARs exist under `android/build/libs/` (they are generated when you install the Android build template from the editor).

After rebuilding, copy the fresh `visualanchors-*.aar` files into `addons/visualanchors/android/bin/` so the export plugin can pick them up.
