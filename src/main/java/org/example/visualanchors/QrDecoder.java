package org.example.visualanchors;

import android.util.Log;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Result;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.GlobalHistogramBinarizer;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.PlanarYUVLuminanceSource;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/** ZXing wrapper minimale: PNG/Bitmap in → testo + 4 corner (TL,TR,BR,BL) out. */
final class QrDecoder {
    private static final String TAG = "VisualAnchorsQR";

    enum PixelFormat { RGBA, BGRA, ARGB, ABGR }

    private PixelFormat pixelFormat = PixelFormat.RGBA;
    private final MultiFormatReader reader = new MultiFormatReader();
    private byte[] lumaBuffer = new byte[0];
    private byte[] rotatedBuffer = new byte[0];
    private int[] argbBuffer = new int[0];

    static final class ResultData {
        final String text;
        final float[] corners; // TL,TR,BR,BL (x0,y0,...)
        ResultData(String text, float[] corners) { this.text = text; this.corners = corners; }
    }

    void setPixelFormat(String fmt) {
        if (fmt == null) { pixelFormat = PixelFormat.RGBA; return; }
        try { pixelFormat = PixelFormat.valueOf(fmt.toUpperCase(Locale.ROOT)); }
        catch (IllegalArgumentException ex) { pixelFormat = PixelFormat.RGBA; }
    }

    private static final Map<DecodeHintType, Object> HINTS = new EnumMap<>(DecodeHintType.class);
    static {
        HINTS.put(DecodeHintType.TRY_HARDER, Boolean.TRUE);
        HINTS.put(DecodeHintType.POSSIBLE_FORMATS, Collections.singletonList(BarcodeFormat.QR_CODE));
        HINTS.put(DecodeHintType.CHARACTER_SET, "UTF-8");
    }

    QrDecoder() {
        reader.setHints(HINTS);
    }

    /* ======================= Public API ======================= */

    /** Decodifica da buffer RGBA-like. */
    ResultData decodeRGBA(byte[] rgba, int width, int height, int stride) {
        if (rgba == null || width <= 0 || height <= 0 || stride <= 0) {
            return null;
        }
        int strideBytes = stride;
        if (strideBytes < width * 4) strideBytes *= 4; // stride may be in pixels
        if (strideBytes < width * 4) strideBytes = width * 4;

        int pixelCount = width * height;
        byte[] yPlane = ensureLumaCapacity(pixelCount);
        int[] argb = ensureArgbCapacity(pixelCount);

        int idx = 0;
        for (int y = 0; y < height; y++) {
            int row = y * strideBytes;
            for (int x = 0; x < width; x++, idx++) {
                int base = row + x * 4;
                if (base + 3 >= rgba.length) return null;
                int r, g, b;
                switch (pixelFormat) {
                    case RGBA:
                        r = rgba[base] & 0xFF; g = rgba[base + 1] & 0xFF; b = rgba[base + 2] & 0xFF; break;
                    case BGRA:
                        b = rgba[base] & 0xFF; g = rgba[base + 1] & 0xFF; r = rgba[base + 2] & 0xFF; break;
                    case ARGB:
                        r = rgba[base + 1] & 0xFF; g = rgba[base + 2] & 0xFF; b = rgba[base + 3] & 0xFF; break;
                    case ABGR:
                        r = rgba[base + 3] & 0xFF; g = rgba[base + 2] & 0xFF; b = rgba[base + 1] & 0xFF; break;
                    default:
                        r = rgba[base] & 0xFF; g = rgba[base + 1] & 0xFF; b = rgba[base + 2] & 0xFF; break;
                }
                argb[idx] = 0xFF000000 | (r << 16) | (g << 8) | b;
                yPlane[idx] = (byte) fastLuma(r, g, b);
            }
        }

        ResultData res = decodeSource(new PlanarYUVLuminanceSource(yPlane, width, height, 0, 0, width, height, false));
        if (res != null) {
            return res;
        }

        byte[] rot = ensureRotatedCapacity(pixelCount);
        for (int i = 0, j = pixelCount - 1; i < pixelCount; i++, j--) {
            rot[i] = yPlane[j];
        }
        res = decodeSource(new PlanarYUVLuminanceSource(rot, width, height, 0, 0, width, height, false));
        if (res != null) {
            return res;
        }

        LuminanceSource source = new RGBLuminanceSource(width, height, argb);
        return decodeSource(source);
    }

    /** Decodifica da piano Y (luma). */
    ResultData decodeLuma(byte[] y, int width, int height, int stride) {
        if (y == null || width <= 0 || height <= 0 || stride <= 0) {
            return null;
        }
        int pixelCount = width * height;
        byte[] packed;
        if (stride == width) {
            packed = y;
        } else {
            packed = ensureLumaCapacity(pixelCount);
            for (int row = 0; row < height; row++) {
                System.arraycopy(y, row * stride, packed, row * width, width);
            }
        }
        ResultData r = decodeSource(new PlanarYUVLuminanceSource(packed, width, height, 0, 0, width, height, false));
        if (r != null) {
            return r;
        }
        byte[] rot180 = ensureRotatedCapacity(pixelCount);
        for (int i = 0, j = pixelCount - 1; i < pixelCount; i++, j--) {
            rot180[i] = packed[j];
        }
        return decodeSource(new PlanarYUVLuminanceSource(rot180, width, height, 0, 0, width, height, false));
    }

    /* ======================= Core ======================= */

    private ResultData decodeSource(LuminanceSource source) {
        if (source == null) return null;

        ResultData r = tryDecode(new BinaryBitmap(new HybridBinarizer(source)));
        if (r != null) {
            return r;
        }

        r = tryDecode(new BinaryBitmap(new GlobalHistogramBinarizer(source)));
        if (r != null) {
            return r;
        }

        LuminanceSource inverted = source.invert();
        r = tryDecode(new BinaryBitmap(new HybridBinarizer(inverted)));
        if (r != null) {
            return r;
        }

        r = tryDecode(new BinaryBitmap(new GlobalHistogramBinarizer(inverted)));
        if (r != null) {
            return r;
        }

        return null;
    }

    private ResultData tryDecode(BinaryBitmap bb) {
        try {
            reader.setHints(HINTS);
            Result result = reader.decodeWithState(bb);
            if (result == null) return null;

            ResultPoint[] pts = result.getResultPoints();
            if (pts == null || pts.length < 3) return null;

            float[] corners = buildCorners(pts);
            if (corners == null) return null;
            enforceClockwise(corners);

            return new ResultData(result.getText(), corners);
        } catch (NotFoundException ignore) {
            return null;
        } catch (Exception e) {
            Log.e(TAG, "tryDecode: unexpected", e);
            return null;
        } finally {
            reader.reset();
        }
    }

    private static int fastLuma(int r, int g, int b) {
        int y = (r * 38 + g * 75 + b * 15) >> 7;
        if (y < 0) return 0;
        return y > 255 ? 255 : y;
    }

    private byte[] ensureLumaCapacity(int size) {
        if (lumaBuffer.length < size) {
            lumaBuffer = new byte[size];
        }
        return lumaBuffer;
    }

    private byte[] ensureRotatedCapacity(int size) {
        if (rotatedBuffer.length < size) {
            rotatedBuffer = new byte[size];
        }
        return rotatedBuffer;
    }

    private int[] ensureArgbCapacity(int size) {
        if (argbBuffer.length < size) {
            argbBuffer = new int[size];
        }
        return argbBuffer;
    }

    /* ======================= Corners helpers ======================= */

    private float[] buildCorners(ResultPoint[] pts) {
        List<float[]> raw = new ArrayList<>();
        for (ResultPoint p : pts) if (p != null) raw.add(new float[]{p.getX(), p.getY()});
        if (raw.size() < 3) return null;

        // Se ZXing fornisce 3 punti (finder pattern), ricava il 4° per parallelogramma
        if (raw.size() == 3) {
            float[] a = raw.get(0), b = raw.get(1), c = raw.get(2);
            raw.add(new float[]{a[0] + c[0] - b[0], a[1] + c[1] - b[1]});
        }

        // Se >4, prendi i 4 più lontani dal baricentro
        if (raw.size() > 4) {
            float cx = 0, cy = 0; for (float[] p : raw) { cx += p[0]; cy += p[1]; }
            cx /= raw.size(); cy /= raw.size();
            final float Cx = cx, Cy = cy;
            raw.sort((p1, p2) -> Double.compare(dist2(p2, Cx, Cy), dist2(p1, Cx, Cy)));
            raw = new ArrayList<>(raw.subList(0, 4));
        }

        // Ordina per angolo e ruota per iniziare dal top-left
        float cx = 0, cy = 0; for (float[] p : raw) { cx += p[0]; cy += p[1]; }
        cx /= raw.size(); cy /= raw.size();
        final float Cx2 = cx, Cy2 = cy;
        raw.sort((p1, p2) -> {
            double a1 = Math.atan2(p1[1] - Cy2, p1[0] - Cx2);
            double a2 = Math.atan2(p2[1] - Cy2, p2[0] - Cx2);
            return Double.compare(a1, a2);
        });
        int tl = 0; double best = Double.POSITIVE_INFINITY;
        for (int i = 0; i < raw.size(); i++) {
            double s = raw.get(i)[0] + raw.get(i)[1];
            if (s < best) { best = s; tl = i; }
        }
        Collections.rotate(raw, -tl);

        float[] out = new float[8];
        for (int i = 0; i < 4; i++) { out[2*i] = raw.get(i)[0]; out[2*i+1] = raw.get(i)[1]; }
        return out;
    }

    private void enforceClockwise(float[] q) {
        if (q == null || q.length < 8) return;
        double area2 = 0.0;
        for (int i = 0; i < 4; i++) {
            int j = (i + 1) & 3;
            area2 += q[2*i] * q[2*j+1] - q[2*j] * q[2*i+1];
        }
        if (area2 < 0.0) { swap(q, 2, 6); swap(q, 3, 7); }
    }

    private void swap(float[] a, int i, int j) { float t = a[i]; a[i] = a[j]; a[j] = t; }
    private double dist2(float[] p, float cx, float cy) { double dx = p[0]-cx, dy = p[1]-cy; return dx*dx+dy*dy; }
}
