extends Node
class_name VisualAnchorsManager

signal qr_pose(payload: String, transform: Transform3D, reproj_err_px: float, area_px: float)
signal error(code: int, message: String)
signal anchor_updated(payload: String, node: Node3D, world_transform: Transform3D)

const _MODE_FOLLOW := 0
const _MODE_ONE_SHOT := 1

const ESTIMATED_INTRINSICS := {
	"fx": 880.0,
	"fy": 880.0,
	"cx": 640.0,
	"cy": 480.0,
}

const ESTIMATED_DISTORTION := [0.0, 0.0, 0.0, 0.0, 0.0]

const LOG_VERBOSE := false

func _log_msg(message: String) -> void:
	if LOG_VERBOSE:
		print(message)

var _plugin: Object
var _plugin_connected: bool = false
var _plugin_methods: PackedStringArray = PackedStringArray()
var _running: bool = false

var _bindings: Dictionary = {}

var _cached_intrinsics: Dictionary = {}
var _cached_distortion: PackedFloat64Array = PackedFloat64Array()
var _cached_default_size: float = 0.0
var _cached_size_map: Dictionary = {}
var _cached_external_transform: Transform3D = Transform3D.IDENTITY
var _cached_smoothing: Dictionary = {
	"pos_min_cutoff": 1.0,
	"pos_beta": 0.02,
	"rot_min_cutoff_deg": 1.0,
	"rot_beta": 0.02,
}
var _cached_gating: Dictionary = {
	"max_reproj_err_px": 6.0,
	"min_area_px": 64.0,
	"max_cond_h": 1.0e6,
	"max_rot_jump_deg": 20.0,
	"max_trans_jump_m": 0.15,
}

func _ready() -> void:
	_ensure_plugin()

func _exit_tree() -> void:
	stop()
	_disconnect_plugin()

func has_plugin() -> bool:
	return _ensure_plugin()

func set_intrinsics(fx: float, fy: float, cx: float, cy: float, dist: PackedFloat64Array) -> void:
	_cached_intrinsics = {
		"fx": fx,
		"fy": fy,
		"cx": cx,
		"cy": cy,
	}
	_cached_distortion = dist.duplicate()
	_apply_intrinsics()

func set_default_qr_size_m(size_m: float) -> void:
	_cached_default_size = size_m
	_apply_default_size()

func set_qr_size_map(dict_id_to_size: Dictionary) -> void:
	_cached_size_map = dict_id_to_size.duplicate(true)
	_apply_size_map()

func set_external_camera_from_xr(xform: Transform3D) -> void:
	_cached_external_transform = xform
	_apply_external_transform()

func set_filters(params: Dictionary) -> void:
	var smoothing_source: Dictionary = params
	if params.has("smoothing") and params["smoothing"] is Dictionary:
		smoothing_source = params["smoothing"]
	var gating_source: Dictionary = params
	if params.has("gating") and params["gating"] is Dictionary:
		gating_source = params["gating"]
	for key in _cached_smoothing.keys():
		if smoothing_source.has(key):
			_cached_smoothing[key] = float(smoothing_source[key])
		elif params.has(key):
			_cached_smoothing[key] = float(params[key])
	for key in _cached_gating.keys():
		if gating_source.has(key):
			_cached_gating[key] = float(gating_source[key])
		elif params.has(key):
			_cached_gating[key] = float(params[key])
	_apply_filters()

func start() -> bool:
	if not _ensure_plugin():
		push_warning("VisualAnchors Android plugin not available on this platform")
		_log_msg("[VA Manager] start aborted: plugin missing")
		return false
	if _running:
		_log_msg("[VA Manager] start ignored: already running")
		return true
	var success: bool = true
	var call_result: Variant = _plugin.call("start")
	if typeof(call_result) == TYPE_BOOL:
		success = bool(call_result)
	if success:
		_running = true
		_log_msg("[VA Manager] start succeeded")
	else:
		_running = false
		_log_msg("[VA Manager] start failed")
	return success

func stop() -> void:
	if _plugin and _plugin_connected:
		if _plugin_has("stop"):
			_plugin.call("stop")
			_log_msg("[VA Manager] stop invoked on plugin")
	_running = false

func is_running() -> bool:
	return _running

func bind(node: Node3D, payload: String, options: Dictionary = {}) -> void:
	if node == null:
		push_warning("bind: node is null for payload %s" % payload)
		return
	var binding: Dictionary = {
		"target": weakref(node),
		"payload": payload,
		"offset": options.get("offset", Transform3D.IDENTITY),
		"mode": _MODE_FOLLOW if options.get("follow", true) else _MODE_ONE_SHOT,
		"callback": options.get("on_pose", Callable()),
	}
	if not _bindings.has(payload):
		_bindings[payload] = []
	_bindings[payload].append(binding)

func unbind(node: Node3D) -> void:
	for payload in _bindings.keys():
		_bindings[payload] = _bindings[payload].filter(func(b):
			var target: Node3D = (b["target"] as WeakRef).get_ref() as Node3D
			return target != null and target != node
		)

func unbind_payload(payload: String) -> void:
	_bindings.erase(payload)

func clear_bindings() -> void:
	_bindings.clear()

func _ensure_plugin() -> bool:
	if _plugin:
		return true
	if Engine.has_singleton("VisualAnchors"):
		var singleton: Object = Engine.get_singleton("VisualAnchors")
		if singleton:
			_bind_plugin(singleton)
			return true
		_log_msg("[VA Manager] Engine singleton VisualAnchors missing")
	return false

func _bind_plugin(singleton: Object) -> void:
	if _plugin and _plugin_connected:
		_disconnect_plugin()
	_plugin = singleton
	if _plugin:
		_log_msg("[VA Manager] binding plugin")
		_refresh_plugin_methods()
		var err_pose: int = _plugin.connect("qr_pose", Callable(self, "_on_plugin_pose"))
		if err_pose != OK:
			push_warning("VisualAnchors: connect qr_pose failed (%d)" % err_pose)
			_log_msg("[VA Manager] connect qr_pose failed code=%d" % err_pose)
		var err_err: int = _plugin.connect("error", Callable(self, "_on_plugin_error"))
		if err_err != OK:
			push_warning("VisualAnchors: connect error failed (%d)" % err_err)
			_log_msg("[VA Manager] connect error failed code=%d" % err_err)
		_plugin_connected = true
		_apply_cached_configuration()
		_log_msg("[VA Manager] plugin bound, methods=%s" % [_plugin_methods])

func _disconnect_plugin() -> void:
	if _plugin and _plugin_connected:
		if _plugin.is_connected("qr_pose", Callable(self, "_on_plugin_pose")):
			_plugin.disconnect("qr_pose", Callable(self, "_on_plugin_pose"))
		if _plugin.is_connected("error", Callable(self, "_on_plugin_error")):
			_plugin.disconnect("error", Callable(self, "_on_plugin_error"))
		_log_msg("[VA Manager] plugin disconnected")
	_plugin_connected = false
	_plugin = null

func _refresh_plugin_methods() -> void:
	_plugin_methods = PackedStringArray()
	if _plugin == null:
		return
	var reported: Variant = _plugin.call("listApiMethods")
	if reported is PackedStringArray:
		_plugin_methods = reported.duplicate()
	elif reported is Array:
		_plugin_methods = PackedStringArray(reported)

func _plugin_has(method: String) -> bool:
	if method == "":
		return false
	if _plugin_methods.is_empty():
		return true
	return _plugin_methods.has(method)

func _apply_cached_configuration() -> void:
	_apply_intrinsics()
	_apply_default_size()
	_apply_size_map()
	_apply_external_transform()
	_apply_filters()

func _apply_intrinsics() -> void:
	if _plugin == null:
		return
	if _cached_intrinsics.is_empty():
		return
	if _plugin_has("setCameraIntrinsics"):
		_plugin.call("setCameraIntrinsics",
			_cached_intrinsics.get("fx", 0.0),
			_cached_intrinsics.get("fy", 0.0),
			_cached_intrinsics.get("cx", 0.0),
			_cached_intrinsics.get("cy", 0.0)
		)
		_log_msg("[VA Manager] setCameraIntrinsics fx=%f fy=%f cx=%f cy=%f" % [
			_cached_intrinsics.get("fx", 0.0),
			_cached_intrinsics.get("fy", 0.0),
			_cached_intrinsics.get("cx", 0.0),
			_cached_intrinsics.get("cy", 0.0)
		])
	if _plugin_has("setDistortion") and _cached_distortion.size() >= 4:
		_plugin.call("setDistortion", _cached_distortion)
		_log_msg("[VA Manager] setDistortion len=%d" % _cached_distortion.size())

func _apply_default_size() -> void:
	if _plugin == null:
		return
	if _cached_default_size <= 0.0:
		return
	if _plugin_has("setDefaultQrSizeMeters"):
		_plugin.call("setDefaultQrSizeMeters", _cached_default_size)
		_log_msg("[VA Manager] setDefaultQrSizeMeters %.4f" % _cached_default_size)

func _apply_size_map() -> void:
	if _plugin == null:
		return
	if not _plugin_has("clearQrSizeOverrides"):
		return
	_plugin.call("clearQrSizeOverrides")
	_log_msg("[VA Manager] clearQrSizeOverrides")
	if _cached_size_map.is_empty():
		return
	for payload in _cached_size_map.keys():
		var size_m: float = float(_cached_size_map[payload])
		if size_m > 0.0 and _plugin_has("setQrSizeForId"):
			_plugin.call("setQrSizeForId", str(payload), size_m)
			_log_msg("[VA Manager] setQrSizeForId payload=%s size=%.4f" % [payload, size_m])

func _apply_external_transform() -> void:
	if _plugin == null:
		return
	if _plugin_has("setExternalCameraFromXR"):
		_plugin.call("setExternalCameraFromXR", _transform_to_float_array(_cached_external_transform))
		_log_msg("[VA Manager] setExternalCameraFromXR origin=%s" % _cached_external_transform.origin)

func _apply_filters() -> void:
	if _plugin == null:
		return
	if _plugin_has("setSmoothingParams"):
		_plugin.call("setSmoothingParams",
			_cached_smoothing["pos_min_cutoff"],
			_cached_smoothing["pos_beta"],
			_cached_smoothing["rot_min_cutoff_deg"],
			_cached_smoothing["rot_beta"]
		)
		_log_msg("[VA Manager] setSmoothingParams %s" % [_cached_smoothing])
	if _plugin_has("setGatingParams"):
		_plugin.call("setGatingParams",
			_cached_gating["max_reproj_err_px"],
			_cached_gating["min_area_px"],
			_cached_gating["max_cond_h"],
			_cached_gating["max_rot_jump_deg"],
			_cached_gating["max_trans_jump_m"]
		)
		_log_msg("[VA Manager] setGatingParams %s" % [_cached_gating])

func _on_plugin_pose(payload: String, transform_array, reproj_err_px: float, area_px: float) -> void:
	var transform: Transform3D = _array_to_transform(transform_array)
	emit_signal("qr_pose", payload, transform, reproj_err_px, area_px)
	_update_bindings(payload, transform, reproj_err_px, area_px)

func _on_plugin_error(code: int, message: String) -> void:
	emit_signal("error", code, message)

func _array_to_transform(data) -> Transform3D:
	var arr: PackedFloat32Array = PackedFloat32Array()
	if data is PackedFloat32Array:
		arr = data
	elif data is PackedFloat64Array:
		arr.resize(data.size())
		for i in range(data.size()):
			arr[i] = float(data[i])
	elif data is Array:
		arr = PackedFloat32Array(data)
	if arr.size() < 16:
		return Transform3D.IDENTITY
	var basis: Basis = Basis(
		Vector3(arr[0], arr[1], arr[2]),
		Vector3(arr[4], arr[5], arr[6]),
		Vector3(arr[8], arr[9], arr[10])
	).orthonormalized()
	var origin: Vector3 = Vector3(arr[12], arr[13], arr[14])
	return Transform3D(basis, origin)

func _transform_to_float_array(xform: Transform3D) -> PackedFloat32Array:
	var arr: PackedFloat32Array = PackedFloat32Array()
	arr.resize(16)
	var basis: Basis = xform.basis
	arr[0] = basis.x.x
	arr[1] = basis.x.y
	arr[2] = basis.x.z
	arr[3] = 0.0

	arr[4] = basis.y.x
	arr[5] = basis.y.y
	arr[6] = basis.y.z
	arr[7] = 0.0

	arr[8] = basis.z.x
	arr[9] = basis.z.y
	arr[10] = basis.z.z
	arr[11] = 0.0

	arr[12] = xform.origin.x
	arr[13] = xform.origin.y
	arr[14] = xform.origin.z
	arr[15] = 1.0
	return arr

func _update_bindings(payload: String, marker_transform: Transform3D, reproj_err_px: float, area_px: float) -> void:
	if not _bindings.has(payload):
		return
	var list: Array = _bindings[payload]
	var to_remove: Array = []
	for binding in list:
		var target_ref: WeakRef = binding["target"]
		var node: Node3D = target_ref.get_ref() as Node3D
		if node == null:
			to_remove.append(binding)
			continue
		var offset: Transform3D = binding.get("offset", Transform3D.IDENTITY)
		var world_transform: Transform3D = marker_transform * offset
		node.global_transform = world_transform
		emit_signal("anchor_updated", payload, node, world_transform)
		var cb: Callable = binding.get("callback", Callable())
		if cb.is_valid():
			cb.call(payload, node, world_transform, reproj_err_px, area_px)
		if binding["mode"] == _MODE_ONE_SHOT:
			to_remove.append(binding)
	if to_remove.size() > 0:
		for item in to_remove:
			list.erase(item)
		if list.is_empty():
			_bindings.erase(payload)
