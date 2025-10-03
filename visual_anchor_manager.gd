extends Node
class_name VisualAnchorsManager

signal qr_detected(payload, corners, pose, frame_width, frame_height, timestamp_ns)
signal anchor_updated(payload, node, world_transform)

const _MODE_FOLLOW := 0
const _MODE_ONE_SHOT := 1

var _plugin: Object
var _bindings: Dictionary = {}
var _camera_ref: WeakRef
var _running := false
var _cached_qr_size: float = 0.1
var _cached_intrinsics: Array = []
var _plugin_connected := false
var _polling_enabled := false
var _plugin_methods: PackedStringArray = PackedStringArray()

func _ready():
	set_process(true)
	_ensure_plugin()

func _exit_tree():
	stop()
	if _plugin and _plugin_connected:
		_plugin.disconnect("qr_detected", Callable(self, "_on_plugin_detected"))
		_plugin_connected = false
	_plugin = null
	_polling_enabled = false

func start(config: Dictionary = {}) -> bool:
	if not _ensure_plugin():
		push_warning("VisualAnchors Android plugin not available on this platform")
		return false
	if config.has("camera_node"):
		set_camera(config["camera_node"])
	elif config.has("camera_path"):
		var node := get_node_or_null(config["camera_path"])
		if node:
			set_camera(node)
	if config.has("qr_size_m"):
		set_qr_size(config["qr_size_m"])
	if config.has("intrinsics") and config["intrinsics"] is Dictionary:
		var intr: Dictionary = config["intrinsics"]
		if intr.has("fx") and intr.has("fy") and intr.has("cx") and intr.has("cy"):
			set_camera_intrinsics(float(intr.fx), float(intr.fy), float(intr.cx), float(intr.cy))
	var plugin_cfg := {}
	for key in ["camera_id", "target_fps", "min_interval_ms", "qr_size_m", "intrinsics"]:
		if config.has(key):
			plugin_cfg[key] = config[key]
	var ok := false
	if _plugin:
		if not _plugin_supports("start"):
			push_warning("VisualAnchors plugin missing start(); attempting anyway")
		var call_result := _plugin.callv("start", [plugin_cfg])
		ok = bool(call_result)
		if not ok:
			print("[VA Manager] plugin.start returned", call_result)
	else:
		print("[VA Manager] plugin reference missing during start")
	_running = ok
	if ok:
		_polling_enabled = not _plugin_connected and _plugin_supports("poll_detection")
	return ok

func stop() -> void:
	if _plugin and _plugin_supports("stop"):
		_plugin.callv("stop", [])
	_running = false
	_polling_enabled = false

func is_running() -> bool:
	return _running

func set_camera(node: Node) -> void:
	if node is Node3D:
		_camera_ref = weakref(node)
	else:
		push_warning("set_camera expects a Node3D, got %s" % [typeof(node)])

func bind(node: Node3D, payload: String, options: Dictionary = {}) -> void:
	if node == null:
		push_warning("bind: node is null for payload %s" % payload)
		return
	var binding := {
		"target": weakref(node),
		"payload": payload,
		"offset": options.get("offset", Transform3D.IDENTITY),
		"mode": _MODE_FOLLOW if options.get("follow", true) else _MODE_ONE_SHOT,
		"smoothing": clamp(options.get("smoothing", 0.0), 0.0, 1.0),
		"pending": true,
		"callback": options.get("on_detect", Callable()),
		"size_m": options.get("size_m", 0.0),
		"last_timestamp_ns": 0,
		"smoothed_transform": null,
		"lock_after": max(float(options.get("lock_after", 0.0)), 0.0),
		"lock_position_epsilon": max(float(options.get("lock_position_epsilon", 0.025)), 0.0),
		"lock_rotation_epsilon_deg": max(float(options.get("lock_rotation_epsilon_deg", 4.0)), 0.0),
		"lock_timer": 0.0,
		"locked": false,
		"locked_transform": Transform3D.IDENTITY,
		"lock_stick": bool(options.get("lock_stick", false))
	}
	if not _bindings.has(payload):
		_bindings[payload] = []
	_bindings[payload].append(binding)
	var size_override := float(binding["size_m"])
	if _plugin and size_override > 0.0 and _plugin_supports("set_payload_size"):
		_plugin.callv("set_payload_size", [payload, size_override])

func unbind(node: Node3D) -> void:
	for payload in _bindings.keys():
		_bindings[payload] = _bindings[payload].filter(func(b):
			var target := (b["target"] as WeakRef).get_ref()
			return target != null and target != node
		)

func unbind_payload(payload: String) -> void:
	_bindings.erase(payload)
	if _plugin and _plugin_supports("clear_payload_size"):
		_plugin.callv("clear_payload_size", [payload])

func clear_bindings() -> void:
	_bindings.clear()
	if _plugin and _plugin_supports("clear_all_payload_sizes"):
		_plugin.callv("clear_all_payload_sizes", [])

func set_qr_size(size_m: float) -> void:
	_cached_qr_size = size_m
	if _plugin and _plugin_supports("set_qr_size_m"):
		_plugin.callv("set_qr_size_m", [size_m])

func set_camera_intrinsics(fx: float, fy: float, cx: float, cy: float) -> void:
	_cached_intrinsics = [fx, fy, cx, cy]
	print("[VA Manager] set_camera_intrinsics fx=", fx, " fy=", fy, " cx=", cx, " cy=", cy)
	if _plugin and _plugin_supports("set_camera_intrinsics"):
		_plugin.callv("set_camera_intrinsics", [fx, fy, cx, cy])

func ensure_intrinsics(fx: float, fy: float, cx: float, cy: float) -> void:
	if _cached_intrinsics.size() == 4:
		return
	set_camera_intrinsics(fx, fy, cx, cy)

func set_pixel_format(format: String) -> void:
	if _plugin and _plugin_supports("set_pixel_format"):
		_plugin.callv("set_pixel_format", [format])

func scan_rgba(image, width: int, height: int, stride: int) -> bool:
	if not _plugin or not _plugin_supports("scan_rgba"):
		return false
	return bool(_plugin.callv("scan_rgba", [image, width, height, stride]))

func scan_luma(image, width: int, height: int, stride: int) -> bool:
	if not _plugin or not _plugin_supports("scan_luma"):
		return false
	return bool(_plugin.callv("scan_luma", [image, width, height, stride]))

func has_plugin() -> bool:
	return _ensure_plugin()

func _ensure_plugin() -> bool:
	if _plugin:
		return true
	if Engine.has_singleton("VisualAnchors"):
		var singleton := Engine.get_singleton("VisualAnchors")
		if singleton:
			_bind_plugin(singleton)
			return true
	return false

func _refresh_plugin_methods() -> void:
	_plugin_methods = PackedStringArray()
	if _plugin == null:
		return
	var reported := _plugin.callv("list_api_methods", [])
	if reported is PackedStringArray:
		_plugin_methods = (reported as PackedStringArray).duplicate()
	elif reported is Array:
		_plugin_methods = PackedStringArray(reported)

func _plugin_supports(method: String) -> bool:
	if method == "":
		return false
	if _plugin_methods.is_empty():
		return true
	return _plugin_methods.has(method)

func _bind_plugin(singleton: Object) -> void:
	if _plugin == singleton:
		return
	if _plugin and _plugin_connected:
		_plugin.disconnect("qr_detected", Callable(self, "_on_plugin_detected"))
	_plugin = singleton
	_plugin_connected = false
	_refresh_plugin_methods()
	if _plugin:
		var method_names: Array = []
		for info in _plugin.get_method_list():
			if info is Dictionary and info.has("name"):
				method_names.append(info["name"])
		print("[VA Manager] plugin methods=", method_names)
		if not _plugin_methods.is_empty():
			print("[VA Manager] plugin self-reported API=", _plugin_methods)
		var err := _plugin.connect("qr_detected", Callable(self, "_on_plugin_detected"))
		if err != OK:
			push_warning("VisualAnchors: connect qr_detected failed (" + str(err) + ")")
			_plugin_connected = false
			_polling_enabled = _plugin_supports("poll_detection")
		else:
			_plugin_connected = true
			_polling_enabled = false
	if _cached_qr_size > 0.0:
		if _plugin_supports("set_qr_size_m"):
			_plugin.callv("set_qr_size_m", [_cached_qr_size])
	if _cached_intrinsics.size() == 4:
		if _plugin_supports("set_camera_intrinsics"):
			_plugin.callv("set_camera_intrinsics", _cached_intrinsics)

func _on_plugin_detected(payload: String, corners_raw, pose_raw, width: int, height: int, timestamp_ns: int) -> void:
	var corners := PackedFloat32Array(corners_raw)
	var pose := PackedFloat32Array(pose_raw)
	_handle_detection(payload, corners, pose, width, height, timestamp_ns)

func _process(_delta: float) -> void:
	if not _polling_enabled:
		return
	_poll_pending_detection()

func _poll_pending_detection() -> void:
	if _plugin == null or not _plugin_supports("poll_detection"):
		return
	var result = _plugin.poll_detection()
	if typeof(result) != TYPE_DICTIONARY:
		return
	var dict: Dictionary = result
	if not dict.has("payload"):
		return
	var payload := String(dict.get("payload", ""))
	var width := int(dict.get("width", 0))
	var height := int(dict.get("height", 0))
	var timestamp_ns := int(dict.get("timestamp_ns", 0))
	var corners := _to_packed_float32(dict.get("corners", []))
	if corners.is_empty() and dict.has("corner_tl_x"):
		corners = PackedFloat32Array([
			float(dict.get("corner_tl_x", 0.0)),
			float(dict.get("corner_tl_y", 0.0)),
			float(dict.get("corner_tr_x", 0.0)),
			float(dict.get("corner_tr_y", 0.0)),
			float(dict.get("corner_br_x", 0.0)),
			float(dict.get("corner_br_y", 0.0)),
			float(dict.get("corner_bl_x", 0.0)),
			float(dict.get("corner_bl_y", 0.0)),
		])
	var pose := _to_packed_float32(dict.get("pose", []))
	_handle_detection(payload, corners, pose, width, height, timestamp_ns)

func _to_packed_float32(values) -> PackedFloat32Array:
	if values is PackedFloat32Array:
		return values.duplicate()
	if values is PackedFloat64Array:
		var out := PackedFloat32Array()
		out.resize(values.size())
		for i in range(values.size()):
			out[i] = float(values[i])
		return out
	if values is Array:
		return PackedFloat32Array(values)
	return PackedFloat32Array()

func _handle_detection(payload: String, corners: PackedFloat32Array, pose: PackedFloat32Array, width: int, height: int, timestamp_ns: int) -> void:
	emit_signal("qr_detected", payload, corners, pose, width, height, timestamp_ns)
	_update_bindings(payload, pose, width, height, timestamp_ns)

func _update_bindings(payload: String, pose: PackedFloat32Array, frame_width: int, frame_height: int, timestamp_ns: int) -> void:
	if not _bindings.has(payload):
		return
	var list: Array = _bindings[payload]
	var to_remove: Array = []
	for binding in list:
		var target_ref: WeakRef = binding["target"]
		var node := target_ref.get_ref()
		if node == null:
			to_remove.append(binding)
			continue
		var camera := _get_camera()
		if camera == null:
			continue
		var marker_transform := _pose_to_transform(pose)
		var offset_xform: Transform3D = binding.get("offset", Transform3D.IDENTITY)
		var raw_transform: Transform3D = camera.global_transform * marker_transform * offset_xform
		var dt := 0.0
		var last_ts := int(binding.get("last_timestamp_ns", 0))
		if timestamp_ns > 0 and last_ts > 0:
			dt = max(0.0, float(timestamp_ns - last_ts) * 1e-9)
		binding["last_timestamp_ns"] = timestamp_ns
		var prev_variant: Variant = binding.get("smoothed_transform", null)
		var has_prev: bool = prev_variant is Transform3D and not binding["pending"]
		var prev_transform: Transform3D = raw_transform
		if has_prev:
			prev_transform = prev_variant
		var smoothing_weight := float(binding.get("smoothing", 0.0))
		var smoothed_transform := raw_transform
		if smoothing_weight > 0.0 and has_prev and node is Node3D:
			var lambda := lerp(6.0, 18.0, clamp(smoothing_weight, 0.0, 1.0))
			var alpha := clamp(smoothing_weight, 0.0, 1.0)
			if dt > 0.0:
				alpha = clamp(1.0 - exp(-lambda * dt), 0.0, 1.0)
			smoothed_transform = _smooth_transform(prev_transform, raw_transform, alpha)
		binding["pending"] = false
		binding["smoothed_transform"] = smoothed_transform

		var lock_after := float(binding.get("lock_after", 0.0))
		if lock_after > 0.0:
			var lock_stick := bool(binding.get("lock_stick", false))
			var locked := bool(binding.get("locked", false))
			var pos_epsilon := float(binding.get("lock_position_epsilon", 0.025))
			var rot_epsilon_rad := deg_to_rad(float(binding.get("lock_rotation_epsilon_deg", 4.0)))
			if locked:
				if lock_stick:
					smoothed_transform = binding.get("locked_transform", smoothed_transform)
				else:
					var locked_transform: Transform3D = binding.get("locked_transform", smoothed_transform)
					var pos_delta_locked := locked_transform.origin.distance_to(smoothed_transform.origin)
					var rot_delta_locked := _rotation_difference_rad(locked_transform, smoothed_transform)
					if pos_delta_locked > pos_epsilon * 2.0 or rot_delta_locked > rot_epsilon_rad * 2.0:
						binding["locked"] = false
						binding["lock_timer"] = 0.0
					else:
						smoothed_transform = locked_transform
			else:
				var reference_transform := smoothed_transform
				if has_prev:
					reference_transform = prev_transform
				var pos_delta := reference_transform.origin.distance_to(smoothed_transform.origin)
				var rot_delta := _rotation_difference_rad(reference_transform, smoothed_transform)
				if pos_delta <= pos_epsilon and rot_delta <= rot_epsilon_rad:
					binding["lock_timer"] = float(binding.get("lock_timer", 0.0)) + dt
					if binding["lock_timer"] >= lock_after:
						binding["locked"] = true
						binding["locked_transform"] = smoothed_transform
				else:
					binding["lock_timer"] = 0.0

		node.global_transform = smoothed_transform
		emit_signal("anchor_updated", payload, node, smoothed_transform)
		var cb: Callable = binding["callback"]
		if cb.is_valid():
			cb.call(payload, node, smoothed_transform, frame_width, frame_height)
		if binding["mode"] == _MODE_ONE_SHOT:
			to_remove.append(binding)

	if to_remove.size() > 0:
		for item in to_remove:
			list.erase(item)
		if list.is_empty():
			_bindings.erase(payload)

func _get_camera() -> Node3D:
	if _camera_ref:
		var node := _camera_ref.get_ref()
		if node:
			return node
	var viewport := get_viewport()
	if viewport:
		var cam := viewport.get_camera_3d()
		if cam:
			_camera_ref = weakref(cam)
			return cam
	return null

func _pose_to_transform(pose: PackedFloat32Array) -> Transform3D:
	if pose == null or pose.size() < 16:
		return Transform3D.IDENTITY
	var basis := Basis(
		Vector3(pose[0], pose[1], pose[2]),
		Vector3(pose[4], pose[5], pose[6]),
		Vector3(pose[8], pose[9], pose[10])
	)
	var origin := Vector3(pose[12], pose[13], pose[14])
	var ortho_basis := basis.orthonormalized()
	return Transform3D(ortho_basis, origin)

func _smooth_transform(current: Transform3D, target: Transform3D, weight: float) -> Transform3D:
	weight = clamp(weight, 0.0, 1.0)
	var lerp_origin := current.origin.lerp(target.origin, weight)
	var from_quat := current.basis.orthonormalized().get_rotation_quaternion()
	var to_quat := target.basis.orthonormalized().get_rotation_quaternion()
	var blended_quat := from_quat.slerp(to_quat, weight)
	var basis := Basis(blended_quat)
	return Transform3D(basis, lerp_origin)

func _rotation_difference_rad(a: Transform3D, b: Transform3D) -> float:
	var qa := a.basis.orthonormalized().get_rotation_quaternion()
	var qb := b.basis.orthonormalized().get_rotation_quaternion()
	return qa.angle_to(qb)
