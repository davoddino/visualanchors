package org.example.visualanchors;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
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

    static final class ResultData {
        final String text;
        final float[] corners; // TL,TR,BR,BL (x0,y0,...)
        ResultData(String text, float[] corners) { this.text = text; this.corners = corners; }
    }

    private boolean debug = false;
    void setDebug(boolean enabled) { debug = enabled; }

    void setPixelFormat(String fmt) {
        if (fmt == null) { pixelFormat = PixelFormat.RGBA; return; }
        try { pixelFormat = PixelFormat.valueOf(fmt.toUpperCase(Locale.ROOT)); }
        catch (IllegalArgumentException ex) { pixelFormat = PixelFormat.RGBA; }
        if (debug) Log.d(TAG, "Pixel format set to: " + pixelFormat);
    }

    private static final Map<DecodeHintType, Object> HINTS = new EnumMap<>(DecodeHintType.class);
    static {
        HINTS.put(DecodeHintType.TRY_HARDER, Boolean.TRUE);
        HINTS.put(DecodeHintType.POSSIBLE_FORMATS, Collections.singletonList(BarcodeFormat.QR_CODE));
        HINTS.put(DecodeHintType.CHARACTER_SET, "UTF-8");
    }

    /* ======================= Public API ======================= */

    /** Decodifica da bytes PNG. */
    ResultData decodePNG(byte[] pngBytes) {
        if (pngBytes == null || pngBytes.length == 0) {
            Log.w(TAG, "decodePNG: empty input");
            return null;
        }
        final BitmapFactory.Options opts = new BitmapFactory.Options();
        opts.inPreferredConfig = Bitmap.Config.ARGB_8888; // evita RGB_565
        opts.inScaled = false;                            // no DPI scaling
        Bitmap bmp = BitmapFactory.decodeByteArray(pngBytes, 0, pngBytes.length, opts);
        if (bmp == null) {
            Log.w(TAG, "decodePNG: BitmapFactory failed");
            return null;
        }
        try { return decodeBitmap(bmp); }
        finally { bmp.recycle(); }
    }

    /** Decodifica da Bitmap (assunta ARGB_8888). */
    ResultData decodeBitmap(Bitmap bmp) {
        if (bmp == null) return null;
        int w = bmp.getWidth(), h = bmp.getHeight();
        if (w <= 0 || h <= 0) return null;

        int[] argb = new int[w * h];
        bmp.getPixels(argb, 0, w, 0, 0, w, h);
        if (debug) {
            StringBuilder sb = new StringBuilder("pixels[0..4]: ");
            for (int i = 0; i < Math.min(5, argb.length); i++)
                sb.append(String.format(Locale.ROOT, "0x%08X ", argb[i]));
            Log.d(TAG, "decodeBitmap " + w + "x" + h + " " + sb);
        }

        // LuminanceSource direttamente da ARGB (ZXing gestisce rotazioni/deskew internamente)
        LuminanceSource source = new RGBLuminanceSource(w, h, argb);
        return decodeSource(source);
    }

    /** Decodifica da buffer RGBA-like. */
    ResultData decodeRGBA(byte[] rgba, int width, int height, int stride) {
        if (rgba == null || width <= 0 || height <= 0 || stride <= 0) return null;
        int strideBytes = stride;
        if (strideBytes < width * 4) strideBytes *= 4; // stride may be in pixels
        if (strideBytes < width * 4) strideBytes = width * 4;

        int[] argb = new int[width * height];
        int idx = 0;
        for (int y = 0; y < height; y++) {
            int row = y * strideBytes;
            for (int x = 0; x < width; x++, idx++) {
                int base = row + x * 4;
                if (base + 3 >= rgba.length) return null;
                int r, g, b;
                switch (pixelFormat) {
                    case RGBA:
                        r = rgba[base] & 0xFF; g = rgba[base+1] & 0xFF; b = rgba[base+2] & 0xFF; break;
                    case BGRA:
                        b = rgba[base] & 0xFF; g = rgba[base+1] & 0xFF; r = rgba[base+2] & 0xFF; break;
                    case ARGB:
                        r = rgba[base+1] & 0xFF; g = rgba[base+2] & 0xFF; b = rgba[base+3] & 0xFF; break;
                    case ABGR:
                        r = rgba[base+3] & 0xFF; g = rgba[base+2] & 0xFF; b = rgba[base+1] & 0xFF; break;
                    default:
                        r = rgba[base] & 0xFF; g = rgba[base+1] & 0xFF; b = rgba[base+2] & 0xFF; break;
                }
                argb[idx] = 0xFF000000 | (r << 16) | (g << 8) | b;
            }
        }
        LuminanceSource source = new RGBLuminanceSource(width, height, argb);
        return decodeSource(source);
    }

    /** Decodifica da piano Y (luma). */
    ResultData decodeLuma(byte[] y, int width, int height, int stride) {
        if (y == null || width <= 0 || height <= 0 || stride <= 0) return null;
        // Pack to tight if needed
        byte[] packed = y;
        if (stride != width) {
            packed = new byte[width * height];
            for (int row = 0; row < height; row++) {
                System.arraycopy(y, row * stride, packed, row * width, width);
            }
        }
        ResultData r = decodeSource(new PlanarYUVLuminanceSource(packed, width, height, 0, 0, width, height, false));
        if (r != null) return r;
        // Try 180 as cheap fallback
        byte[] rot180 = new byte[width * height];
        for (int i = 0, j = rot180.length - 1; i < rot180.length; i++, j--) rot180[i] = packed[j];
        return decodeSource(new PlanarYUVLuminanceSource(rot180, width, height, 0, 0, width, height, false));
    }

    /* ======================= Core ======================= */

    private ResultData decodeSource(LuminanceSource source) {
        if (source == null) return null;

        // 1) Hybrid
        ResultData r = tryDecode(new BinaryBitmap(new HybridBinarizer(source)));
        if (r != null) return r;

        // 2) GlobalHistogram
        r = tryDecode(new BinaryBitmap(new GlobalHistogramBinarizer(source)));
        if (r != null) return r;

        // 3) Inverted + Hybrid
        r = tryDecode(new BinaryBitmap(new HybridBinarizer(source.invert())));
        if (r != null) return r;

        // 4) Inverted + GlobalHistogram
        r = tryDecode(new BinaryBitmap(new GlobalHistogramBinarizer(source.invert())));
        if (r != null) return r;

        if (debug) Log.w(TAG, "decodeSource: all attempts failed");
        return null;
    }

    private ResultData tryDecode(BinaryBitmap bb) {
        MultiFormatReader reader = new MultiFormatReader();
        try {
            reader.setHints(HINTS);
            Result result = reader.decode(bb);
            if (result == null) return null;

            ResultPoint[] pts = result.getResultPoints();
            if (pts == null || pts.length < 3) return null;

            float[] corners = buildCorners(pts);
            if (corners == null) return null;
            enforceClockwise(corners);

            if (debug) Log.i(TAG, "QR: \"" + result.getText() + "\" points=" + pts.length);
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
