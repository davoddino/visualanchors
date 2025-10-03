package org.example.visualanchors;

import android.app.Activity;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import org.godotengine.godot.Dictionary;
import org.godotengine.godot.Godot;
import org.godotengine.godot.plugin.GodotPlugin;
import org.godotengine.godot.plugin.SignalInfo;
import org.godotengine.godot.plugin.UsedByGodot;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.Collections;
import java.util.LinkedHashSet;

/**
 * VisualAnchors Godot plugin: QR detection + pose estimation with optional internal camera stream.
 */
public class VisualAnchorsPlugin extends GodotPlugin implements CameraController.FrameListener {
    private static final String TAG = "VisualAnchors";
    private static final List<String> EXPORTED_METHODS = Collections.unmodifiableList(Arrays.asList(
            "set_qr_size_m",
            "set_camera_intrinsics",
            "set_pixel_format",
            "scan_rgba",
            "scan_luma",
            "poll_detection",
            "start",
            "stop",
            "set_payload_size",
            "clear_payload_size",
            "clear_all_payload_sizes",
            "list_api_methods",
            "has_api_method"
    ));
    private static final Set<String> EXPORTED_METHOD_SET = new LinkedHashSet<>(EXPORTED_METHODS);
    private static final float[] IDENTITY_POSE = new float[]{
            1f, 0f, 0f, 0f,
            0f, 1f, 0f, 0f,
            0f, 0f, 1f, 0f,
            0f, 0f, 0f, 1f
    };

    private final QrDecoder qr = new QrDecoder();
    private final PoseEstimator poseEstimator = new PoseEstimator();
    private final Handler mainHandler = new Handler(Looper.getMainLooper());

    private float physicalSizeMeters = 0.10f; // lato QR (default 10 cm)
    private boolean intrinsicsReady = false;
    private float fx = 0.0f, fy = 0.0f, cx = 0.0f, cy = 0.0f;
    private final Map<String, Float> payloadSizes = new ConcurrentHashMap<>();

    private final Object detectionLock = new Object();
    private Detection pendingDetection = null;

    private CameraController cameraController;
    private final ExecutorService decoderExecutor = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "VA-Decoder");
        t.setPriority(Thread.NORM_PRIORITY + 1);
        t.setDaemon(true);
        return t;
    });
    private final AtomicBoolean frameInFlight = new AtomicBoolean(false);
    private long minFrameIntervalNs = 33_000_000L; // ~30 FPS default cap
    private volatile long lastDecodeMonoNs = 0L;
    private boolean streamingRequested = false;
    private final StartOptions startOptions = new StartOptions();

    private static final class Detection {
        final String text;
        final float[] corners;
        final float[] pose;
        final int width;
        final int height;
        final long timestampNs;

        Detection(String text, float[] corners, float[] pose, int width, int height, long timestampNs) {
            this.text = text;
            this.corners = corners;
            this.pose = pose;
            this.width = width;
            this.height = height;
            this.timestampNs = timestampNs;
        }
    }

    private static final class StartOptions {
        String cameraId = null;
    }

    public VisualAnchorsPlugin(Godot godot) {
        super(godot);
        Log.i(TAG, "VisualAnchorsPlugin initialized");
    }

    @Override public String getPluginName() { return "VisualAnchors"; }

    @Override
    public List<String> getPluginMethods() {
        return EXPORTED_METHODS;
    }

    @Override
    public Set<SignalInfo> getPluginSignals() {
        HashSet<SignalInfo> signals = new HashSet<>();
        signals.add(new SignalInfo("qr_detected", String.class, float[].class, float[].class, Integer.class, Integer.class, Long.class));
        return signals;
    }

    /* ===================== Config ===================== */

    @UsedByGodot
    public void set_qr_size_m(float size) {
        physicalSizeMeters = size;
        Log.d(TAG, "QR physical size set to " + size + " m");
    }

    @UsedByGodot
    public void set_camera_intrinsics(float fx, float fy, float cx, float cy) {
        if (fx <= 0 || fy <= 0) {
            Log.w(TAG, "Invalid intrinsics: fx=" + fx + " fy=" + fy);
            intrinsicsReady = false;
            return;
        }
        this.fx = fx; this.fy = fy; this.cx = cx; this.cy = cy;
        intrinsicsReady = true;
        Log.i(TAG, "Camera intrinsics set fx=" + fx + " fy=" + fy + " cx=" + cx + " cy=" + cy);
    }

    /** Imposta il formato del buffer 4BPP (RGBA/BGRA/ARGB/ABGR) usato da scan_rgba. */
    @UsedByGodot
    public void set_pixel_format(String fmt) {
        qr.setPixelFormat(fmt);
        Log.i(TAG, "Pixel format set to " + fmt);
    }

    @UsedByGodot
    public void set_payload_size(String payload, float sizeMeters) {
        if (payload == null || payload.isEmpty()) {
            Log.w(TAG, "set_payload_size: payload is null/empty");
            return;
        }
        if (!(sizeMeters > 0f)) {
            payloadSizes.remove(payload);
            Log.i(TAG, "set_payload_size: cleared override for payload=" + payload);
            return;
        }
        payloadSizes.put(payload, sizeMeters);
        Log.i(TAG, "set_payload_size: payload=" + payload + " size=" + sizeMeters);
    }

    @UsedByGodot
    public void clear_payload_size(String payload) {
        if (payload == null || payload.isEmpty()) {
            return;
        }
        payloadSizes.remove(payload);
        Log.i(TAG, "clear_payload_size: payload=" + payload);
    }

    @UsedByGodot
    public void clear_all_payload_sizes() {
        payloadSizes.clear();
        Log.i(TAG, "clear_all_payload_sizes: cleared overrides");
    }

    @UsedByGodot
    public String[] list_api_methods() {
        return EXPORTED_METHODS.toArray(new String[0]);
    }

    @UsedByGodot
    public boolean has_api_method(String name) {
        return name != null && EXPORTED_METHOD_SET.contains(name);
    }

    /* ===================== Streaming control ===================== */

    @UsedByGodot
    public boolean start(Dictionary config) {
        applyStartConfig(config);
        streamingRequested = true;
        Log.i(TAG, "start: requested with config=" + (config != null ? config.toString() : "{}"));
        boolean ok = startCameraController();
        if (!ok) {
            streamingRequested = false;
        } else {
            applyAutomaticIntrinsics();
        }
        return ok;
    }

    @UsedByGodot
    public void stop() {
        streamingRequested = false;
        stopCameraController();
        Log.i(TAG, "stop: streaming disabled by caller");
    }

    private void applyStartConfig(Dictionary config) {
        if (config == null) {
            return;
        }
        Object cameraIdValue = config.get("camera_id");
        if (cameraIdValue != null) {
            startOptions.cameraId = cameraIdValue.toString();
        }
        Object qrSizeValue = config.get("qr_size_m");
        if (qrSizeValue instanceof Number) {
            set_qr_size_m(((Number) qrSizeValue).floatValue());
        }
        Object minIntervalMs = config.get("min_interval_ms");
        if (minIntervalMs instanceof Number) {
            double ms = ((Number) minIntervalMs).doubleValue();
            if (ms >= 0.0) {
                minFrameIntervalNs = (long) (ms * 1_000_000.0);
            }
        }
        Object targetFps = config.get("target_fps");
        if (targetFps instanceof Number) {
            double fps = ((Number) targetFps).doubleValue();
            if (fps > 0.0) {
                minFrameIntervalNs = (long) (1_000_000_000.0 / fps);
            }
        }
        Object intrinsicsDict = config.get("intrinsics");
        if (intrinsicsDict instanceof Dictionary) {
            Dictionary intr = (Dictionary) intrinsicsDict;
            Object fxVal = intr.get("fx");
            Object fyVal = intr.get("fy");
            Object cxVal = intr.get("cx");
            Object cyVal = intr.get("cy");
            if (fxVal instanceof Number && fyVal instanceof Number && cxVal instanceof Number && cyVal instanceof Number) {
                set_camera_intrinsics(
                        ((Number) fxVal).floatValue(),
                        ((Number) fyVal).floatValue(),
                        ((Number) cxVal).floatValue(),
                        ((Number) cyVal).floatValue());
            }
        }
    }

    private boolean startCameraController() {
        if (cameraController != null) {
            Log.i(TAG, "startCameraController: controller already running");
            return true;
        }
        Godot godot = getGodot();
        Activity activity = godot != null ? godot.getActivity() : null;
        if (activity == null) {
            Log.w(TAG, "startCameraController: activity unavailable");
            return false;
        }
        cameraController = new CameraController(activity, this, startOptions.cameraId);
        boolean started = cameraController.start();
        if (!started) {
            cameraController = null;
            Log.e(TAG, "startCameraController: failed to start camera controller");
        } else {
            Log.i(TAG, "startCameraController: camera pipeline started");
        }
        return started;
    }

    private void stopCameraController() {
        CameraController ctrl = cameraController;
        cameraController = null;
        if (ctrl != null) {
            ctrl.stop();
            Log.i(TAG, "stopCameraController: camera pipeline stopped");
        }
        frameInFlight.set(false);
    }

    /* ===================== FrameListener ===================== */

    @Override
    public void onFrame(CameraController.FrameData frame) {
        if (!streamingRequested) {
            return;
        }
        long now = System.nanoTime();
        if (minFrameIntervalNs > 0 && (now - lastDecodeMonoNs) < minFrameIntervalNs) {
            return;
        }
        if (!frameInFlight.compareAndSet(false, true)) {
            return; // decoder busy, drop frame
        }
        lastDecodeMonoNs = now;
        final byte[] luma = frame.luma;
        final int width = frame.width;
        final int height = frame.height;
        final int stride = frame.stride;
        final long timestamp = frame.timestampNs;
        decoderExecutor.execute(() -> {
            try {
                processFrame(luma, width, height, stride, timestamp);
            } finally {
                frameInFlight.set(false);
            }
        });
    }

    private void processFrame(byte[] luma, int width, int height, int stride, long timestampNs) {
        Log.v(TAG, "processFrame: decoding frame " + width + "x" + height + " stride=" + stride);
        QrDecoder.ResultData res = qr.decodeLuma(luma, width, height, stride);
        handleResult(res, width, height, "STREAM", timestampNs);
    }

    /* ===================== Scan APIs ===================== */

    /** Buffer 4BPP (coerente con CameraTexture Godot). */
    @UsedByGodot
    public boolean scan_rgba(byte[] rgba, int width, int height, int stride) {
        if (rgba == null || width <= 0 || height <= 0 || stride <= 0) {
            Log.w(TAG, "scan_rgba: invalid args");
            return false;
        }
        QrDecoder.ResultData res = qr.decodeRGBA(rgba, width, height, stride);
        return handleResult(res, width, height, "RGBA", System.nanoTime());
    }

    /** Solo piano Y (grayscale) con stride. */
    @UsedByGodot
    public boolean scan_luma(byte[] luma, int width, int height, int stride) {
        if (luma == null || width <= 0 || height <= 0 || stride <= 0) {
            Log.w(TAG, "scan_luma: invalid args");
            return false;
        }
        QrDecoder.ResultData res = qr.decodeLuma(luma, width, height, stride);
        return handleResult(res, width, height, "LUMA", System.nanoTime());
    }

    /* ===================== Helpers ===================== */

    private boolean handleResult(QrDecoder.ResultData res, int width, int height, String srcTag, long timestampNs) {
        if (res == null) {
            return false;
        }

        final String payload = res.text;
        float sizeForPayload = physicalSizeMeters;
        if (payload != null) {
            Float override = payloadSizes.get(payload);
            if (override != null && override.floatValue() > 0f) {
                sizeForPayload = override.floatValue();
            }
        }
        float[] pose = computePose(res, sizeForPayload);
        final float[] cornersCopy = Arrays.copyOf(res.corners, res.corners.length);
        final float[] poseCopy = pose != null ? Arrays.copyOf(pose, pose.length) : Arrays.copyOf(IDENTITY_POSE, IDENTITY_POSE.length);

        Detection detection = new Detection(payload, cornersCopy, poseCopy, width, height, timestampNs);
        synchronized (detectionLock) {
            pendingDetection = detection;
        }
        Log.i(TAG, "QR detected: " + payload + " via " + srcTag);
        emitDetection(detection);
        return true;
    }

    private void emitDetection(Detection det) {
        Runnable emitTask = () -> {
            float[] cornersForSignal = copyFloatArray(det.corners);
            float[] poseForSignal = copyFloatArray(det.pose);
            emitSignal("qr_detected", det.text, cornersForSignal, poseForSignal, det.width, det.height, det.timestampNs);
        };
        if (Looper.myLooper() == Looper.getMainLooper()) {
            emitTask.run();
        } else {
            mainHandler.post(emitTask);
        }
    }

    private float[] computePose(QrDecoder.ResultData res, float sizeMeters) {
        if (!intrinsicsReady) {
            // OK non avere la pose: il testo e i corner sono già utili
            return null;
        }
        float[] pose = poseEstimator.computePose(res.corners, sizeMeters, fx, fy, cx, cy);
        if (pose == null) {
            Log.w(TAG, "Pose estimation failed for payload " + res.text);
        }
        return pose;
    }

    @UsedByGodot
    public Dictionary poll_detection() {
        Detection det;
        synchronized (detectionLock) {
            det = pendingDetection;
            pendingDetection = null;
        }
        if (det == null) {
            Log.v(TAG, "poll_detection: no pending detection");
            return null;
        }
        Log.v(TAG, "poll_detection: returning detection width=" + det.width + " height=" + det.height);
        Dictionary dict = new Dictionary();
        dict.put("payload", det.text);
        dict.put("width", det.width);
        dict.put("height", det.height);
        dict.put("timestamp_ns", det.timestampNs);
        putCorners(dict, det.corners);
        putPose(dict, det.pose);
        return dict;
    }

    private void putCorners(Dictionary dict, float[] c) {
        if (c == null || c.length < 8) {
            return;
        }
        dict.put("corner_tl_x", (double) c[0]);
        dict.put("corner_tl_y", (double) c[1]);
        dict.put("corner_tr_x", (double) c[2]);
        dict.put("corner_tr_y", (double) c[3]);
        dict.put("corner_br_x", (double) c[4]);
        dict.put("corner_br_y", (double) c[5]);
        dict.put("corner_bl_x", (double) c[6]);
        dict.put("corner_bl_y", (double) c[7]);
        dict.put("corners", copyFloatArray(c));
    }

    private void putPose(Dictionary dict, float[] pose) {
        float[] m = pose;
        if (pose == null || pose.length < 16) {
            m = Arrays.copyOf(IDENTITY_POSE, IDENTITY_POSE.length);
        }
        dict.put("pose_r00", (double) m[0]);
        dict.put("pose_r10", (double) m[1]);
        dict.put("pose_r20", (double) m[2]);

        dict.put("pose_r01", (double) m[4]);
        dict.put("pose_r11", (double) m[5]);
        dict.put("pose_r21", (double) m[6]);

        dict.put("pose_r02", (double) m[8]);
        dict.put("pose_r12", (double) m[9]);
        dict.put("pose_r22", (double) m[10]);

        dict.put("pose_tx", (double) m[12]);
        dict.put("pose_ty", (double) m[13]);
        dict.put("pose_tz", (double) m[14]);
        dict.put("pose", copyFloatArray(m));
    }

    private float[] copyFloatArray(float[] src) {
        if (src == null) {
            return new float[0];
        }
        return Arrays.copyOf(src, src.length);
    }

    /* ===================== Lifecycle ===================== */

    @Override
    public void onMainDestroy() {
        super.onMainDestroy();
        streamingRequested = false;
        stopCameraController();
        decoderExecutor.shutdownNow();
    }

    private void applyAutomaticIntrinsics() {
        if (intrinsicsReady || cameraController == null) {
            return;
        }
        CameraController.IntrinsicData data = cameraController.getCachedIntrinsics();
        if (data == null) {
            return;
        }
        Log.i(TAG, "Auto intrinsics from camera: fx=" + data.fx + " fy=" + data.fy + " cx=" + data.cx + " cy=" + data.cy);
        set_camera_intrinsics(data.fx, data.fy, data.cx, data.cy);
    }
}
