extends EditorExportPlugin

func _supports_platform(platform: EditorExportPlatform) -> bool:
	if platform is EditorExportPlatformAndroid:
		return true
	return false

func _get_android_libraries(platform: EditorExportPlatform, debug: bool) -> PackedStringArray:
	var libs := PackedStringArray()
	var base := "res://addons/visualanchors/android/bin/"
	var file_name := "visualanchors-debug.aar" if debug else "visualanchors-release.aar"
	libs.append(base + file_name)
	return libs

func _get_android_dependencies(platform: EditorExportPlatform, debug: bool) -> PackedStringArray:
	return PackedStringArray([])

func _get_name():
	return "VisualAnchors"
