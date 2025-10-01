package org.example.visualanchors;

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

public class VisualAnchorsPlugin extends GodotPlugin {
    private static final String TAG = "VisualAnchors";

    private final QrDecoder qr = new QrDecoder();
    private final PoseEstimator poseEstimator = new PoseEstimator();

    private float physicalSizeMeters = 0.10f; // lato QR (default 10 cm)
    private boolean intrinsicsReady = false;
    private float fx = 0.0f, fy = 0.0f, cx = 0.0f, cy = 0.0f;
    private final Object detectionLock = new Object();
    private Detection pendingDetection = null;

    private static final class Detection {
        final String text;
        final float[] corners;
        final float[] pose;
        final int width;
        final int height;

        Detection(String text, float[] corners, float[] pose, int width, int height) {
            this.text = text;
            this.corners = corners;
            this.pose = pose;
            this.width = width;
            this.height = height;
        }
    }

    public VisualAnchorsPlugin(Godot godot) {
        super(godot);
        Log.i(TAG, "VisualAnchorsPlugin initialized");
    }

    @Override public String getPluginName() { return "VisualAnchors"; }

    @Override
    public List<String> getPluginMethods() {
        return Arrays.asList(
                "set_qr_size_m",
                "set_camera_intrinsics",
                "set_pixel_format",
                "scan_rgba",
                "scan_luma",
                "poll_detection"
        );
    }

    @Override
    public Set<SignalInfo> getPluginSignals() {
        return new HashSet<>();
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

    /* ===================== Scan APIs ===================== */

    /** Buffer 4BPP (coerente con CameraTexture Godot). */
    @UsedByGodot
    public boolean scan_rgba(byte[] rgba, int width, int height, int stride) {
        if (rgba == null || width <= 0 || height <= 0 || stride <= 0) {
            Log.w(TAG, "scan_rgba: invalid args");
            return false;
        }
        QrDecoder.ResultData res = qr.decodeRGBA(rgba, width, height, stride);
        return handleResult(res, width, height, "RGBA");
    }

    /** Solo piano Y (grayscale) con stride. */
    @UsedByGodot
    public boolean scan_luma(byte[] luma, int width, int height, int stride) {
        if (luma == null || width <= 0 || height <= 0 || stride <= 0) {
            Log.w(TAG, "scan_luma: invalid args");
            return false;
        }
        QrDecoder.ResultData res = qr.decodeLuma(luma, width, height, stride);
        return handleResult(res, width, height, "LUMA");
    }
    /* ===================== Helpers ===================== */

    private boolean handleResult(QrDecoder.ResultData res, int width, int height, String srcTag) {
        if (res == null) {
            return false;
        }

        float[] pose = computePose(res);
        final String payload = res.text;
        final float[] cornersCopy = Arrays.copyOf(res.corners, res.corners.length);
        final float[] poseCopy;
        if (pose != null) {
            poseCopy = Arrays.copyOf(pose, pose.length);
        } else {
            poseCopy = new float[]{1f,0f,0f,0f,
                                   0f,1f,0f,0f,
                                   0f,0f,1f,0f,
                                   0f,0f,0f,1f};
        }

        Log.i(TAG, "QR detected: " + payload + " via " + srcTag);
        synchronized (detectionLock) {
            pendingDetection = new Detection(payload, cornersCopy, poseCopy, width, height);
        }
        return true;
    }

    private float[] computePose(QrDecoder.ResultData res) {
        if (!intrinsicsReady) {
            // OK non avere la pose: il testo e i corner sono già utili
            return null;
        }
        float[] pose = poseEstimator.computePose(res.corners, physicalSizeMeters, fx, fy, cx, cy);
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
        putCorners(dict, det.corners);
        putPose(dict, det.pose);
        if (det.corners != null && det.corners.length >= 8) {
            Log.v(TAG, "Corners TL(" + det.corners[0] + "," + det.corners[1] + ") TR(" + det.corners[2] + "," + det.corners[3] + ") BR(" + det.corners[4] + "," + det.corners[5] + ") BL(" + det.corners[6] + "," + det.corners[7] + ")");
        } else {
            Log.v(TAG, "Corners unavailable for " + det.text);
        }
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

        Object[] arr = new Object[8];
        for (int i = 0; i < 8; i++) {
            arr[i] = (double) c[i];
        }
        dict.put("corners", arr);
    }

    private void putPose(Dictionary dict, float[] pose) {
        float[] m;
        if (pose == null || pose.length < 16) {
            m = new float[]{
                    1f,0f,0f,0f,
                    0f,1f,0f,0f,
                    0f,0f,1f,0f,
                    0f,0f,0f,1f
            };
        } else {
            m = pose;
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

        Object[] arr = new Object[16];
        for (int i = 0; i < 16; i++) {
            arr[i] = (double) m[i];
        }
        dict.put("pose", arr);
    }
}
