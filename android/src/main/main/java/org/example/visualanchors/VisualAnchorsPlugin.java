package org.example.visualanchors;

import android.app.Activity;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import org.godotengine.godot.Godot;
import org.godotengine.godot.plugin.GodotPlugin;
import org.godotengine.godot.plugin.SignalInfo;
import org.godotengine.godot.plugin.UsedByGodot;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Visual Anchors plugin (OpenCV backend + JNI).
 */
public final class VisualAnchorsPlugin extends GodotPlugin implements CameraController.FrameListener {
    private static final String TAG = "VisualAnchors";

    private static final int ERROR_CONFIG = 1;
    private static final int ERROR_RUNTIME = 2;
    private static final int ERROR_INPUT = 3;

    private static final List<String> EXPORTED_METHODS = Collections.unmodifiableList(Arrays.asList(
            "setCameraIntrinsics",
            "setDistortion",
            "setDefaultQrSizeMeters",
            "setQrSizeForId",
            "clearQrSizeOverrides",
            "setExternalCameraFromXR",
            "setSmoothingParams",
            "setGatingParams",
            "start",
            "stop",
            "listApiMethods",
            "hasApiMethod"
    ));

    private final Config config = new Config();
    private final PoseEstimator poseEstimator = new PoseEstimator();
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final ExecutorService decoderExecutor = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "VA-Decoder");
        t.setPriority(Thread.NORM_PRIORITY + 1);
        t.setDaemon(true);
        return t;
    });
    private final AtomicBoolean frameInFlight = new AtomicBoolean(false);

    private CameraController cameraController;
    private boolean streamingRequested = false;
    private volatile boolean firstFrameLogged = false;

    public VisualAnchorsPlugin(Godot godot) {
        super(godot);
        Log.i(TAG, "VisualAnchorsPlugin (OpenCV) initialised");
    }

    @Override
    public String getPluginName() {
        return "VisualAnchors";
    }

    @Override
    public List<String> getPluginMethods() {
        return EXPORTED_METHODS;
    }

    @Override
    public Set<SignalInfo> getPluginSignals() {
        Set<SignalInfo> signals = new HashSet<>();
        signals.add(new SignalInfo("qr_pose", String.class, float[].class, Double.class, Double.class));
        signals.add(new SignalInfo("error", Integer.class, String.class));
        return signals;
    }

    /* ============================ Public API ============================ */

    @UsedByGodot
    public void setCameraIntrinsics(double fx, double fy, double cx, double cy) {
        try {
            config.setIntrinsics(fx, fy, cx, cy);
            poseEstimator.reset();
        } catch (RuntimeException ex) {
            emitError(ERROR_INPUT, ex.getMessage());
            throw ex;
        }
    }

    @UsedByGodot
    public void setDistortion(double[] distCoeffs) {
        try {
            config.setDistCoeffs(distCoeffs);
            poseEstimator.reset();
        } catch (RuntimeException ex) {
            emitError(ERROR_INPUT, ex.getMessage());
            throw ex;
        }
    }

    @UsedByGodot
    public void setDefaultQrSizeMeters(double sizeMeters) {
        try {
            config.setDefaultQrSizeMeters(sizeMeters);
            poseEstimator.reset();
        } catch (RuntimeException ex) {
            emitError(ERROR_INPUT, ex.getMessage());
            throw ex;
        }
    }

    @UsedByGodot
    public void setQrSizeForId(String id, double sizeMeters) {
        try {
            config.setQrSizeForId(id, sizeMeters);
            poseEstimator.reset();
        } catch (RuntimeException ex) {
            emitError(ERROR_INPUT, ex.getMessage());
            throw ex;
        }
    }

    @UsedByGodot
    public void clearQrSizeOverrides() {
        config.clearQrSizeOverrides();
        poseEstimator.reset();
    }

    @UsedByGodot
    public void setExternalCameraFromXR(float[] mat4x4ColumnMajor) {
        try {
            config.setExternalCameraFromXR(mat4x4ColumnMajor);
            poseEstimator.reset();
        } catch (RuntimeException ex) {
            emitError(ERROR_INPUT, ex.getMessage());
            throw ex;
        }
    }

    @UsedByGodot
    public void setSmoothingParams(double posMinCutoff, double posBeta,
                                   double rotMinCutoffDeg, double rotBeta) {
        try {
            config.setSmoothingParams(posMinCutoff, posBeta, rotMinCutoffDeg, rotBeta);
            poseEstimator.reset();
        } catch (RuntimeException ex) {
            emitError(ERROR_INPUT, ex.getMessage());
            throw ex;
        }
    }

    @UsedByGodot
    public void setGatingParams(double maxReprojErrPx, double minAreaPx,
                                double maxCondH, double maxRotJumpDeg, double maxTransJumpM) {
        try {
            config.setGatingParams(maxReprojErrPx, minAreaPx, maxCondH, maxRotJumpDeg, maxTransJumpM);
            poseEstimator.reset();
        } catch (RuntimeException ex) {
            emitError(ERROR_INPUT, ex.getMessage());
            throw ex;
        }
    }

    @UsedByGodot
    public boolean start() {
        try {
            config.ensureReady();
            Log.i(TAG, "start(): config OK -> " + config.debugSummary());
        } catch (Errors.ConfigurationMissing ex) {
            emitError(ERROR_CONFIG, ex.getMessage());
            Log.e(TAG, "start(): configuration missing", ex);
            return false;
        } catch (RuntimeException ex) {
            emitError(ERROR_INPUT, ex.getMessage());
            Log.e(TAG, "start(): invalid input", ex);
            return false;
        }
        poseEstimator.reset();
        streamingRequested = true;
        boolean ok = startCameraController();
        if (!ok) {
            streamingRequested = false;
            emitError(ERROR_RUNTIME, "Failed to start camera controller");
            Log.e(TAG, "start(): camera controller failed to start");
            return false;
        }
        Log.i(TAG, "start(): camera controller started");
        return true;
    }

    @UsedByGodot
    public void stop() {
        streamingRequested = false;
        stopCameraController();
        poseEstimator.reset();
    }

    @UsedByGodot
    public String[] listApiMethods() {
        return EXPORTED_METHODS.toArray(new String[0]);
    }

    @UsedByGodot
    public boolean hasApiMethod(String name) {
        return name != null && EXPORTED_METHODS.contains(name);
    }

    /* ============================ Camera Control ============================ */

    private boolean startCameraController() {
        if (cameraController != null) {
            return true;
        }
        Godot godot = getGodot();
        Activity activity = godot != null ? godot.getActivity() : null;
        if (activity == null) {
            emitError(ERROR_CONFIG, "Activity unavailable");
            return false;
        }
        cameraController = new CameraController(activity, this, null);
        boolean started = cameraController.start();
        if (!started) {
            emitError(ERROR_RUNTIME, "CameraController failed to start");
            cameraController = null;
        } else {
            Log.i(TAG, "CameraController started");
        }
        return started;
    }

    private void stopCameraController() {
        CameraController ctrl = cameraController;
        cameraController = null;
        if (ctrl != null) {
            ctrl.stop();
            Log.i(TAG, "CameraController stopped");
        }
        frameInFlight.set(false);
    }

    /* ============================ FrameListener ============================ */

    @Override
    public void onFrame(CameraController.FrameData frame) {
        if (!streamingRequested) {
            return;
        }
        if (!frameInFlight.compareAndSet(false, true)) {
            return;
        }
        Log.v(TAG, "onFrame: scheduling decode timestamp=" + (frame != null ? frame.timestampNs : -1));
        decoderExecutor.execute(() -> {
            try {
                processFrame(frame);
            } catch (RuntimeException ex) {
                Log.e(TAG, "processFrame failed", ex);
                emitError(ERROR_RUNTIME, ex.getMessage());
            } finally {
                frameInFlight.set(false);
            }
        });
    }

    private void processFrame(CameraController.FrameData frame) {
        if (frame == null) {
            return;
        }
        if (!firstFrameLogged) {
            Log.i(TAG, "processFrame: first frame received width=" + frame.width + " height=" + frame.height + " stride=" + frame.stride);
            firstFrameLogged = true;
        }
        PoseEstimator.PoseOutput[] outputs;
        try {
            outputs = poseEstimator.processFrame(
                    frame.luma, frame.width, frame.height, frame.stride, frame.timestampNs, config);
        } catch (Errors.ConfigurationMissing ex) {
            emitError(ERROR_CONFIG, ex.getMessage());
            Log.e(TAG, "processFrame: configuration missing", ex);
            throw ex;
        } catch (RuntimeException ex) {
            Log.e(TAG, "processFrame: unexpected runtime exception", ex);
            throw ex;
        }
        if (outputs.length == 0) {
            if (Log.isLoggable(TAG, Log.VERBOSE)) {
                Log.v(TAG, "processFrame: no detections for frame timestamp=" + frame.timestampNs);
            }
            return;
        }
        Log.i(TAG, "processFrame: detections=" + outputs.length + " timestamp=" + frame.timestampNs);
        mainHandler.post(() -> emitDetections(outputs));
    }

    private void emitDetections(PoseEstimator.PoseOutput[] outputs) {
        for (PoseEstimator.PoseOutput out : outputs) {
            if (out == null) {
                continue;
            }
            Log.i(TAG, "emitDetections: payload=" + out.payload + " reproj=" + out.reprojectionErrorPx + " area=" + out.areaPx);
            emitSignal("qr_pose", out.payload, out.transform, out.reprojectionErrorPx, out.areaPx);
        }
    }

    private void emitError(int code, String message) {
        Log.e(TAG, "emitError: code=" + code + " message=" + message);
        mainHandler.post(() -> emitSignal("error", code, message != null ? message : ""));
    }

    @Override
    public void onMainDestroy() {
        super.onMainDestroy();
        streamingRequested = false;
        stopCameraController();
        decoderExecutor.shutdownNow();
    }
}
