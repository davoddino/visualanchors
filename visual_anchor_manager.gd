extends Node

signal qr_detected(text, corners, pose, width, height)
signal debug_message(message)

var plugin = null
var pixel_format = "RGBA"
var qr_size = 0.1  # 10cm
var debug_enabled = false
var _anchor_node: Node3D = null
var _anchor_placed := false
var _intrinsics_sent := false
var _cfg_loaded := false
var _cfg_fx := 0.0
var _cfg_fy := 0.0
var _cfg_cx := 0.0
var _cfg_cy := 0.0

func _ready():
	if Engine.has_singleton("VisualAnchors"):
		plugin = Engine.get_singleton("VisualAnchors")
		print("VisualAnchors plugin loaded")
		_load_config()
		
		# Connect signals
		if plugin:
			plugin.connect("qr_detected", Callable(self, "_on_qr_detected"))
			plugin.connect("debug_log", Callable(self, "_on_debug_log"))
			
			# Configure default settings
			plugin.set_qr_size_m(qr_size)
			plugin.set_pixel_format(pixel_format)
			# Enable diagnostic frame saving immediately
			enable_debug(true)
			# Run a one-shot self-test (generate and decode a QR)
			print("🔬 self-test:", self_test_generate_and_decode())
	else:
		print("VisualAnchors plugin not found!")

func set_camera_intrinsics(fx: float, fy: float, cx: float, cy: float) -> void:
	if plugin:
		plugin.set_camera_intrinsics(fx, fy, cx, cy)
		print("Camera intrinsics set: fx=", fx, " fy=", fy, " cx=", cx, " cy=", cy)

func set_qr_size(size_meters: float) -> void:
	qr_size = size_meters
	if plugin:
		plugin.set_qr_size_m(size_meters)
		print("QR size set to ", size_meters, " meters")

func set_pixel_format(format: String) -> void:
	pixel_format = format
	if plugin:
		plugin.set_pixel_format(format)
		print("Pixel format set to ", format)

func enable_debug(enabled: bool) -> void:
	debug_enabled = enabled
	if plugin:
		plugin.set_debug_save_frames(enabled)
		print("Debug mode ", "enabled" if enabled else "disabled")

func scan_rgba(rgba: PackedByteArray, width: int, height: int, stride: int) -> bool:
	if plugin:
		return plugin.scan_rgba(rgba, width, height, stride)
	return false

func scan_luma(luma: PackedByteArray, width: int, height: int, stride: int) -> bool:
	if plugin:
		return plugin.scan_luma(luma, width, height, stride)
	return false

func scan_png(png_bytes: PackedByteArray) -> bool:
	if plugin:
		return plugin.scan_png(png_bytes)
	return false

func scan_png_path(path: String) -> bool:
	if plugin:
		return plugin.scan_png_path(path)
	return false

func test_detection() -> bool:
	if plugin:
		return plugin.test_detection()
	return false

func self_test_generate_and_decode() -> bool:
	if plugin and plugin.has_method("self_test_generate_and_decode"):
		return plugin.self_test_generate_and_decode()
	return false

func _on_qr_detected(text, corners, pose, width, height):
    # One-shot anchoring: place a single anchor on first valid detection, don't update afterwards
    if _anchor_placed:
        emit_signal("qr_detected", text, corners, pose, width, height)
        return
    if pose == null:
        _ensure_intrinsics(width, height)
        print("[VA] Pose unavailable (intrinsics set if config present; will anchor on next detection)")
        emit_signal("qr_detected", text, corners, pose, width, height)
        return
    var cam := get_viewport().get_camera_3d()
    if cam == null:
        print("[VA] No active Camera3D; cannot place anchor")
        emit_signal("qr_detected", text, corners, pose, width, height)
        return
    var t_cam_to_marker := _pose_to_transform(pose)
    var t_world_marker := cam.global_transform * t_cam_to_marker
    var t_world_anchor := t_world_marker * Transform3D(Basis(), Vector3(0.0, 0.10, 0.0))
    _place_anchor(t_world_anchor)
    _anchor_placed = true
    print("[VA] One-shot anchor placed for payload:", text)
    emit_signal("qr_detected", text, corners, pose, width, height)

func _on_debug_log(message):
	print("VisualAnchors: ", message)
	emit_signal("debug_message", message)

func _load_config() -> void:
	var path := "res://visual_anchor_markers.json"
	if not FileAccess.file_exists(path):
		return
	var f := FileAccess.open(path, FileAccess.READ)
	if f:
		var txt := f.get_as_text()
		f.close()
		var data := JSON.parse_string(txt)
		if typeof(data) == TYPE_DICTIONARY and data.has("camera_intrinsics"):
			var ci = data["camera_intrinsics"]
			if ci.has("fx"): _cfg_fx = float(ci["fx"]) else: _cfg_fx = 0.0
			if ci.has("fy"): _cfg_fy = float(ci["fy"]) else: _cfg_fy = 0.0
			if ci.has("cx"): _cfg_cx = float(ci["cx"]) else: _cfg_cx = 0.0
			if ci.has("cy"): _cfg_cy = float(ci["cy"]) else: _cfg_cy = 0.0
			_cfg_loaded = (_cfg_fx > 0.0 and _cfg_fy > 0.0 and _cfg_cx > 0.0 and _cfg_cy > 0.0)
		if typeof(data) == TYPE_DICTIONARY and data.has("default_size_m"):
			qr_size = float(data["default_size_m"]) if data["default_size_m"] != null else qr_size

func _ensure_intrinsics(w: int, h: int) -> void:
	if _intrinsics_sent or not _cfg_loaded or plugin == null:
		return
	# Infer base image size from principal point assuming centered intrinsics
	var base_w := int(round(_cfg_cx * 2.0))
	var base_h := int(round(_cfg_cy * 2.0))
	if base_w <= 0 or base_h <= 0:
		return
	var sx := float(w) / float(base_w)
	var sy := float(h) / float(base_h)
	var fx := _cfg_fx * sx
	var fy := _cfg_fy * sy
	var cx := _cfg_cx * sx
	var cy := _cfg_cy * sy
	plugin.set_camera_intrinsics(fx, fy, cx, cy)
	_intrinsics_sent = true
	print("[VA] Intrinsics set: fx=", fx, " fy=", fy, " cx=", cx, " cy=", cy, " (from ", base_w, "x", base_h, "→", w, "x", h, ")")

# ===== Anchoring helpers =====
func _pose_to_transform(p: Array) -> Transform3D:
	# pose is 4x4 column-major [R|t]
	var bx := Vector3(p[0], p[1], p[2])
	var by := Vector3(p[4], p[5], p[6])
	var bz := Vector3(p[8], p[9], p[10])
	var origin := Vector3(p[12], p[13], p[14])
	var basis := Basis(bx, by, bz)
	return Transform3D(basis, origin)

func _place_anchor(world_xform: Transform3D) -> void:
    if _anchor_node == null:
        _anchor_node = Node3D.new()
        _anchor_node.name = "va_anchor"
        add_child(_anchor_node)
        # Visual cube (5 cm)
        var cube := MeshInstance3D.new()
        var box := BoxMesh.new()
        box.size = Vector3(0.05, 0.05, 0.05)
        cube.mesh = box
        _anchor_node.add_child(cube)
        print("[VA] Anchor created")
    _anchor_node.global_transform = world_xform
