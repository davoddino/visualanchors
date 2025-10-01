package org.example.visualanchors;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

import org.godotengine.godot.Godot;
import org.godotengine.godot.plugin.GodotPlugin;
import org.godotengine.godot.plugin.SignalInfo;
import org.godotengine.godot.plugin.UsedByGodot;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
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

    private int failureCount = 0;

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
                "scan_png",
                "scan_png_path"
        );
    }

    @Override
    public Set<SignalInfo> getPluginSignals() {
        Set<SignalInfo> s = new HashSet<>();
        s.add(new SignalInfo("qr_detected", String.class, float[].class, float[].class, int.class, int.class));
        s.add(new SignalInfo("debug_log", String.class));
        return s;
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
        Log.d(TAG, "Camera intrinsics set fx=" + fx + " fy=" + fy + " cx=" + cx + " cy=" + cy);
    }

    /** Imposta il formato del buffer 4BPP (RGBA/BGRA/ARGB/ABGR) usato da scan_rgba. */
    @UsedByGodot
    public void set_pixel_format(String fmt) {
        qr.setPixelFormat(fmt);
        Log.d(TAG, "Pixel format set to " + fmt);
        emitSignal("debug_log", "Pixel format set to: " + fmt);
    }

    /* ===================== Scan APIs ===================== */

    /** Buffer 4BPP (coerente con CameraTexture Godot). */
    @UsedByGodot
    public boolean scan_rgba(byte[] rgba, int width, int height, int stride) {
        if (rgba == null || width <= 0 || height <= 0 || stride <= 0) {
            Log.w(TAG, "scan_rgba: invalid args");
            emitSignal("debug_log", "scan_rgba: invalid args");
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
            emitSignal("debug_log", "scan_luma: invalid args");
            return false;
        }
        QrDecoder.ResultData res = qr.decodeLuma(luma, width, height, stride);
        return handleResult(res, width, height, "LUMA");
    }

    /** PNG già in memoria (quello che salvi dal frame Godot). */
    @UsedByGodot
    public boolean scan_png(byte[] pngBytes) {
        if (pngBytes == null || pngBytes.length == 0) {
            Log.w(TAG, "scan_png: empty buffer");
            emitSignal("debug_log", "scan_png: empty buffer");
            return false;
        }

        // Leggi dimensioni (per completezza del segnale)
        int pngW, pngH;
        try {
            BitmapFactory.Options o = new BitmapFactory.Options();
            o.inJustDecodeBounds = true;
            BitmapFactory.decodeByteArray(pngBytes, 0, pngBytes.length, o);
            pngW = o.outWidth; pngH = o.outHeight;
            if (pngW <= 0 || pngH <= 0) {
                Log.w(TAG, "scan_png: invalid PNG bounds");
                emitSignal("debug_log", "scan_png: invalid PNG bounds");
                return false;
            }
        } catch (Throwable t) {
            Log.w(TAG, "scan_png: bounds error", t);
            emitSignal("debug_log", "scan_png: bounds error");
            return false;
        }

        BitmapFactory.Options decodeOpts = new BitmapFactory.Options();
        decodeOpts.inPreferredConfig = Bitmap.Config.ARGB_8888;
        decodeOpts.inScaled = false;
        Bitmap bmp = BitmapFactory.decodeByteArray(pngBytes, 0, pngBytes.length, decodeOpts);
        if (bmp == null) {
            Log.w(TAG, "scan_png: BitmapFactory.decodeByteArray null");
            emitSignal("debug_log", "scan_png: decodeByteArray null");
            return false;
        }

        QrDecoder.ResultData res;
        try {
            res = qr.decodeBitmap(bmp);
        } finally {
            bmp.recycle();
        }
        return handleResult(res, pngW, pngH, "PNG");
    }

    /** Carica un file e lo tratta come PNG (supporta user:// e file://). */
    @UsedByGodot
    public boolean scan_png_path(String path) {
        if (path == null || path.isEmpty()) {
            Log.w(TAG, "scan_png_path: empty path");
            emitSignal("debug_log", "scan_png_path: empty path");
            return false;
        }

        File file = resolvePath(path);
        if (file == null || !file.exists()) {
            Log.w(TAG, "scan_png_path: file not found");
            emitSignal("debug_log", "scan_png_path: not found");
            return false;
        }

        try {
            byte[] data = readAllBytes(file);
            return scan_png(data);
        } catch (IOException e) {
            Log.w(TAG, "scan_png_path: IO error", e);
            emitSignal("debug_log", "scan_png_path: IO error");
            return false;
        }
    }

    /* ===================== Helpers ===================== */

    private boolean handleResult(QrDecoder.ResultData res, int width, int height, String srcTag) {
        if (res == null) {
            failureCount++;
            if (failureCount == 1 || failureCount % 10 == 0) {
                Log.w(TAG, "QR decode failed on " + srcTag + " (attempts=" + failureCount + ")");
                emitSignal("debug_log", "QR decode failed (" + failureCount + " attempts)");
            }
            return false;
        }
        failureCount = 0;

        float[] pose = computePose(res);
        Log.i(TAG, "QR detected: " + res.text + " via " + srcTag);
        emitSignal("debug_log", "QR detected: " + res.text);
        emitSignal("qr_detected", res.text, res.corners, pose, width, height);
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

    private File resolvePath(String path) {
        if (path == null) return null;

        if (path.startsWith("user://")) {
            String rel = path.substring("user://".length());
            File base = getGodot().getActivity().getExternalFilesDir(null);
            if (base == null) base = getGodot().getActivity().getFilesDir();
            return new File(base, rel);
        }
        if (path.startsWith("file://")) {
            return new File(path.substring("file://".length()));
        }
        // res:// non è accessibile come file su Android
        return new File(path);
    }

    private byte[] readAllBytes(File file) throws IOException {
        try (FileInputStream fis = new FileInputStream(file);
             ByteArrayOutputStream bos = new ByteArrayOutputStream(Math.max(1024, (int) Math.min(Integer.MAX_VALUE, file.length())))) {
            byte[] buffer = new byte[8192];
            int read;
            while ((read = fis.read(buffer)) != -1) {
                bos.write(buffer, 0, read);
            }
            return bos.toByteArray();
        }
    }
}
