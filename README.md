# VisualAnchors for Godot

A Godot plugin tailored for Meta Quest that uses an Android-side OpenCV backend to detect QR markers, estimate their pose, and project the result into Godot's XR space with configurable One-Euro and SLERP smoothing. It ships with an export plugin that links the native AAR automatically and exposes a `VisualAnchorsManager` GDScript helper so you can orchestrate everything directly from your scenes.

## Contents
- [Key Features](#key-features)
- [Requirements](#requirements)
- [Quick Start](#quick-start)
- [Project Setup](#project-setup)
- [Camera Permission Handling](#camera-permission-handling)
- [VisualAnchorsManager API](#visualanchorsmanager-api)
- [Rebuilding the Android Plugin](#rebuilding-the-android-plugin)
- [Troubleshooting](#troubleshooting)

## Key Features
- QR detection powered by a native OpenCV 4.x (C++) pipeline bridged through JNI.
- IPPE pose estimation (`SOLVEPNP_IPPE_SQUARE`) with cheirality checks plus configurable reprojection/area gating.
- Transforms converted to Godot XR coordinates and filtered with One-Euro (position) and SLERP (rotation).
- Camera2 controller with luma-only extraction, headset intrinsics cache, and minimal allocations.
- Godot export plugin that injects the prebuilt AARs (`visualanchors-debug.aar` / `visualanchors-release.aar`) into Android builds automatically.
- High-level `VisualAnchorsManager` node exposing signals (`qr_pose`, `anchor_updated`, `error`) and automatic binding of 3D nodes to detected anchors.

## Requirements
- Godot 4.2 or later with XR support enabled.
- Android build template installed (Project > Install Android Build Template) with SDK/NDK configured.
- Target hardware: Meta Quest (Quest 2/3/Pro) or another Android-based headset that exposes a front camera.
- QR markers with a known physical size (mandatory to obtain an absolute scale).
- Camera permissions granted to the app (see the dedicated section).

## Quick Start
1. Copy the `addons/visualanchors` folder into `res://addons` (already present in this repo).
2. In Godot, open **Project > Project Settings > Plugins** and enable `VisualAnchors`.
3. In **Project > Export > Android**, make sure the plugin is ticked under *Plugins* (Godot discovers it via `plugin.cfg` and the AARs in `addons/visualanchors/android/bin/`).
4. Rebuild the Android export templates if needed, then export your Meta Quest build using *Custom Build* mode.

## Project Setup

### 1. Add the manager to a scene
- Instantiate a `VisualAnchorsManager` node (registered via `class_name`) and parent it under your XR origin or another suitable node.
- The manager binds the Android singleton in `_ready()` and disconnects in `_exit_tree()`. Use `has_plugin()` to confirm the native module is available (true only when running on Android).

### 2. Configure calibration and marker sizes
Before calling `start()` you must configure:
- `set_intrinsics(fx, fy, cx, cy, dist)` with your camera parameters and distortion vector (minimum 4 coefficients). As a starting point you can feed `VisualAnchorsManager.ESTIMATED_INTRINSICS` and `ESTIMATED_DISTORTION`.
- `set_default_qr_size_m(size)` with the physical size (in meters) of your default QR markers.
- `set_external_camera_from_xr(transform)` with the 4x4 column-major matrix that maps the headset camera into Godot's XR space. If you do not have an external calibration yet, begin with `Transform3D.IDENTITY`.
- Optional: `set_qr_size_map({ payload: size_m })` to override marker sizes on a per-payload basis.

### 3. Tune filters and bindings
- `set_filters({ smoothing = {...}, gating = {...} })` lets you tweak the One-Euro cutoffs/beta values and the gating limits (max reprojection error, minimum area, rotation/translation jumps).
- Use `bind(node3d, payload, { follow = true, offset = Transform3D.IDENTITY, on_pose = callable })` to automatically attach a 3D node to the pose of a marker. With `follow = false` the binding is one-shot and removed after the first pose. `anchor_updated` is emitted whenever a bound node is updated.

### 4. Start detection
Example setup:

```gdscript
@tool
extends XROrigin3D

@onready var anchors: VisualAnchorsManager = $VisualAnchorsManager

func _ready() -> void:
	if !anchors.has_plugin():
		push_warning("VisualAnchors is not available: run on an Android/Quest device.")
		return

	var intr := VisualAnchorsManager.ESTIMATED_INTRINSICS
	anchors.set_intrinsics(intr.fx, intr.fy, intr.cx, intr.cy, VisualAnchorsManager.ESTIMATED_DISTORTION)
	anchors.set_default_qr_size_m(0.1) # 10 cm marker
	anchors.set_external_camera_from_xr(Transform3D.IDENTITY)
	anchors.set_filters({
		"smoothing": { "pos_min_cutoff": 1.0, "pos_beta": 0.02, "rot_min_cutoff_deg": 1.0, "rot_beta": 0.02 }
	})

	anchors.qr_pose.connect(_on_qr_pose)
	anchors.error.connect(_on_anchor_error)

	var anchor_node := Node3D.new()
	add_child(anchor_node)
	anchors.bind(anchor_node, "ANCHOR_PAYLOAD", {
		"follow": true,
		"on_pose": func(payload, node, world_transform, reproj_err, area):
			print("Pose updated:", payload, world_transform.origin)
	})

	if !anchors.start():
		push_warning("VisualAnchors start() returned false; double-check your configuration.")

func _on_qr_pose(payload: String, transform: Transform3D, reproj_err_px: float, area_px: float) -> void:
	print("QR", payload, "error", reproj_err_px, "area", area_px)

func _on_anchor_error(code: int, message: String) -> void:
	push_error("VisualAnchors error %d: %s" % [code, message])
```

### 5. Stop when done
Call `stop()` when you no longer need frames (for example in `_exit_tree()` or when changing scenes) to release the camera.

## Camera Permission Handling
- On first launch Android will request the `CAMERA` permission. Without it, `start()` fails and the logs show `permission not granted`.
- On Meta Quest you often need to grant the headset-level permission too: open **Settings > Privacy > App Permissions > Camera** on the headset and enable your app. If the plugin declares the *Hand Tracking Camera* permission (`com.oculus.permission.HAND_TRACKING_CAMERA`), ensure it is enabled in the same screen.
- If you sideload the app with `adb` and do not see a system prompt, go manually to **Settings > Privacy > App Permissions > [Your App] > Camera** on the Quest and enable access. The plugin will not receive frames until the permission is granted there.

## VisualAnchorsManager API

**Signals**
- `qr_pose(payload: String, transform: Transform3D, reproj_err_px: float, area_px: float)` – emitted for every detected marker with its pose.
- `error(code: int, message: String)` – configuration or runtime error reported by the native plugin.
- `anchor_updated(payload: String, node: Node3D, world_transform: Transform3D)` – fired whenever a bound node is updated with a new pose.

**Key Methods**
- `has_plugin() -> bool` – true if the native `VisualAnchors` singleton is available (Android runtime only).
- `start() -> bool` / `stop()` / `is_running()` – control the camera stream lifecycle.
- `set_intrinsics(fx, fy, cx, cy, dist)` – provide camera intrinsics and distortion coefficients.
- `set_default_qr_size_m(size_m)` and `set_qr_size_map(dict)` – define the physical scale of your markers.
- `set_external_camera_from_xr(transform)` – supply the 4x4 column-major camera-from-XR transform.
- `set_filters(params)` – configure smoothing (`pos_min_cutoff`, `pos_beta`, `rot_min_cutoff_deg`, `rot_beta`) and gating (`max_reproj_err_px`, `min_area_px`, `max_cond_h`, `max_rot_jump_deg`, `max_trans_jump_m`).
- `bind(node, payload, options)` / `unbind(node)` / `unbind_payload(payload)` / `clear_bindings()` – manage automatic anchoring of scene nodes.

Internally the manager asks the native plugin for `listApiMethods()` to confirm available calls and reapplies any cached configuration as soon as the singleton becomes ready.

## Rebuilding the Android Plugin
Prebuilt `arm64-v8a` AARs live in `addons/visualanchors/android/bin/`. To regenerate them:

```bash
cd addons/visualanchors/android/src
export OpenCV_DIR=/path/to/OpenCV-android-sdk/sdk/native/jni
./gradlew assembleRelease assembleDebug
```

After the build finishes, copy `build/outputs/aar/visualanchors-*.aar` into `addons/visualanchors/android/bin/`. If you need extra ABIs, adjust `abiFilters` in `android/src/main/cpp/CMakeLists.txt` and `build.gradle`.

## Troubleshooting
- **`start()` returns `false`**: ensure you call `set_intrinsics`, `set_distortion`, `set_default_qr_size_m`, and `set_external_camera_from_xr` before starting. Missing any of these raises `ConfigurationMissing`.
- **No `qr_pose` signals**: watch `adb logcat -s VisualAnchors` for detection errors or gating that is too strict (`max_reproj_err_px` / `min_area_px`).
- **Camera permission revoked**: if logs show `CAMERA permission not granted`, revisit the Quest privacy settings and re-enable it.
- **Plugin not listed in Godot**: double-check that it is enabled in **Project Settings > Plugins** and that you export with *Custom Build* so the external AARs are included.
- **Performance concerns**: the decoder runs on a dedicated thread; avoid heavy work in signal callbacks. Offload expensive tasks to your own worker threads if needed.

Happy hacking with VisualAnchors! Feel free to open an issue or reach out directly if you have questions or feature requests.
