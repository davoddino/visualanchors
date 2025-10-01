# VisualAnchors Plugin

This plugin provides QR code detection and pose estimation capabilities for Godot on Android. It processes camera image buffers (RGBA or LUMA) to detect QR codes, extract their text payload and corner positions, and estimate the 3D pose (rotation and translation) of the QR code relative to the camera using provided camera intrinsics. This enables applications like augmented reality tracking or visual anchoring where QR codes serve as markers.

Key features:
- Detect QR codes from camera frames.
- Retrieve payload text and pixel coordinates of corners.
- Compute 3D pose matrix for positioning in 3D space.
- Configurable QR physical size and camera parameters.

**Note:** This implementation is not optimized yet. It may perform slowly on lower-end devices due to computational overhead in decoding and pose estimation. Future optimizations could include GPU acceleration, caching, or reduced resolution processing.
