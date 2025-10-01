package org.example.visualanchors;

import android.graphics.Matrix;
import android.util.Log;

/** Stima posa da omografia H = K [r1 r2 t], con normalizzazione robusta e re-ortogonalizzazione. */
class PoseEstimator {
    private static final String TAG = "VisualAnchorsPose";

    /**
     * Calcola la posa del marker nel frame della camera (trasformazione camera→marker) in formato 4x4 column-major.
     * Richiede i 4 corner in ordine TL,TR,BR,BL (pixel), lato fisico del QR in metri e intrinseci (fx,fy,cx,cy)
     * riferiti alla risoluzione del frame passato al decoder.
     */
    float[] computePose(float[] corners, float sizeMeters, float fx, float fy, float cx, float cy) {
        if (corners == null || corners.length < 8) {
            Log.w(TAG, "Invalid corners array");
            return null;
        }
        if (sizeMeters <= 0.0f) {
            Log.w(TAG, "Invalid physical size (<=0)");
            return null;
        }
        if (fx <= 0 || fy <= 0) {
            Log.w(TAG, "Invalid intrinsics (fx/fy <= 0)");
            return null;
        }

        // Modello piano Z=0 nel frame marker: quadrato centrato sull'origine
        final float half = sizeMeters * 0.5f;
        final float[] src = new float[]{ -half,  half,   // TL
                                          half,  half,   // TR
                                          half, -half,   // BR
                                         -half, -half }; // BL

        // Corner immagine (pixel) in ordine TL,TR,BR,BL
        final float[] dst = new float[]{
                corners[0], corners[1],
                corners[2], corners[3],
                corners[4], corners[5],
                corners[6], corners[7]
        };

        // Omografia da piano marker → immagine
        final Matrix m = new Matrix();
        if (!m.setPolyToPoly(src, 0, dst, 0, 4)) {
            Log.w(TAG, "setPolyToPoly failed");
            return null;
        }
        final float[] v = new float[9];
        m.getValues(v);
        // Matrix.getValues fornisce una 3x3 in ordine:
        // [ a, c, tx ]
        // [ b, d, ty ]
        // [ 0, 0,  1 ]
        final double[][] H = new double[][]{
                { v[0], v[1], v[2] }, // a, c, tx
                { v[3], v[4], v[5] }, // b, d, ty
                { v[6], v[7], 1.0 }   // p, q, 1
        };

        // K^{-1}
        final double[][] Kinv = new double[][]{
                { 1.0 / fx, 0.0,     -cx / fx },
                { 0.0,      1.0 / fy, -cy / fy },
                { 0.0,      0.0,       1.0     }
        };

        // Hnorm = K^{-1} H = [r1 r2 t] (a scala)
        final double[][] Hn = multiply(Kinv, H);

        double[] h1 = new double[]{ Hn[0][0], Hn[1][0], Hn[2][0] };
        double[] h2 = new double[]{ Hn[0][1], Hn[1][1], Hn[2][1] };
        double[] h3 = new double[]{ Hn[0][2], Hn[1][2], Hn[2][2] };

        // Scala robusta: usa media delle norme di h1 e h2
        final double n1 = Math.sqrt(dot(h1, h1));
        final double n2 = Math.sqrt(dot(h2, h2));
        final double lambda = (n1 + n2) * 0.5;
        if (!(lambda > 0.0)) {
            Log.w(TAG, "Normalization failed (lambda <= 0)");
            return null;
        }

        // R colonne e traslazione nel frame camera (world=marker)
        double[] r1 = scale(h1, 1.0 / lambda);
        double[] r2 = scale(h2, 1.0 / lambda);
        double[] t  = scale(h3, 1.0 / lambda);

        // r3 = r1 × r2 e re-ortogonalizzazione
        double[] r3 = cross(r1, r2);

        // Correggi eventuale riflessione: imponi det(R) > 0
        double det = det3(r1, r2, r3);
        if (det < 0.0) {
            r3 = scale(r3, -1.0);
        }

        // Re-ortogonalizza (Gram-Schmidt leggero)
        normalizeInPlace(r1);
        // r2' = r3 × r1 per garantire ortogonalità e mano destra
        r2 = cross(r3, r1);
        normalizeInPlace(r2);
        // r3' = r1 × r2
        r3 = cross(r1, r2);
        normalizeInPlace(r3);

        // [R|t] mappa marker→camera. Per avere trasform camera→marker, invertiamo:
        // T_cm = [R t; 0 1],  T_mc = [Rᵀ  -Rᵀ t; 0 1]
        final double[][] Rinv = transpose(new double[][]{
                { r1[0], r2[0], r3[0] },
                { r1[1], r2[1], r3[1] },
                { r1[2], r2[2], r3[2] }
        });
        final double[] tInv = multiplyVec(Rinv, new double[]{ -t[0], -t[1], -t[2] });

        // Matrice 4x4 column-major per Godot (camera→marker)
        float[] pose = new float[16];
        pose[0] = (float) Rinv[0][0];
        pose[1] = (float) Rinv[1][0];
        pose[2] = (float) Rinv[2][0];
        pose[3] = 0f;

        pose[4] = (float) Rinv[0][1];
        pose[5] = (float) Rinv[1][1];
        pose[6] = (float) Rinv[2][1];
        pose[7] = 0f;

        pose[8]  = (float) Rinv[0][2];
        pose[9]  = (float) Rinv[1][2];
        pose[10] = (float) Rinv[2][2];
        pose[11] = 0f;

        pose[12] = (float) tInv[0];
        pose[13] = (float) tInv[1];
        pose[14] = (float) tInv[2];
        pose[15] = 1f;

        return pose;
    }

    /* ===================== Algebra helpers ===================== */

    private static double dot(double[] a, double[] b) {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[]{
                a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0]
        };
    }

    private static void normalizeInPlace(double[] v) {
        double len = Math.sqrt(dot(v, v));
        if (len > 0.0) {
            v[0] /= len; v[1] /= len; v[2] /= len;
        }
    }

    private static double[] scale(double[] v, double s) {
        return new double[]{ v[0]*s, v[1]*s, v[2]*s };
    }

    private static double det3(double[] c1, double[] c2, double[] c3) {
        // det([c1 c2 c3]) con c* colonne
        return  c1[0]*(c2[1]*c3[2] - c2[2]*c3[1])
              - c2[0]*(c1[1]*c3[2] - c1[2]*c3[1])
              + c3[0]*(c1[1]*c2[2] - c1[2]*c2[1]);
    }

    private static double[][] multiply(double[][] A, double[][] B) {
        double[][] R = new double[3][3];
        for (int i = 0; i < 3; i++) {
            R[i][0] = A[i][0]*B[0][0] + A[i][1]*B[1][0] + A[i][2]*B[2][0];
            R[i][1] = A[i][0]*B[0][1] + A[i][1]*B[1][1] + A[i][2]*B[2][1];
            R[i][2] = A[i][0]*B[0][2] + A[i][1]*B[1][2] + A[i][2]*B[2][2];
        }
        return R;
    }

    private static double[] multiplyVec(double[][] A, double[] v) {
        return new double[]{
                A[0][0]*v[0] + A[0][1]*v[1] + A[0][2]*v[2],
                A[1][0]*v[0] + A[1][1]*v[1] + A[1][2]*v[2],
                A[2][0]*v[0] + A[2][1]*v[1] + A[2][2]*v[2]
        };
    }

    private static double[][] transpose(double[][] M) {
        return new double[][]{
                { M[0][0], M[1][0], M[2][0] },
                { M[0][1], M[1][1], M[2][1] },
                { M[0][2], M[1][2], M[2][2] }
        };
    }
}
