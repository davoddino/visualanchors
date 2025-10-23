package org.example.visualanchors;

/**
 * Smooths orientation using SLERP with a One-Euro style adaptive blend.
 */
final class QuatSmoother {
    private final double minCutoffDeg;
    private final double beta;
    private float[] prevQuat;

    QuatSmoother(double minCutoffDeg, double beta) {
        if (!(minCutoffDeg > 0.0)) {
            throw new Errors.InvalidInput("Quaternion smoother minCutoff must be > 0");
        }
        if (beta < 0.0) {
            throw new Errors.InvalidInput("Quaternion smoother beta must be >= 0");
        }
        this.minCutoffDeg = minCutoffDeg;
        this.beta = beta;
    }

    float[] reset(float[] quat) {
        prevQuat = quat != null ? normalize(quat) : null;
        return prevQuat;
    }

    float[] filter(float[] quat, double dtSeconds) {
        if (quat == null || quat.length != 4) {
            throw new Errors.InvalidInput("Quaternion smoother expects 4D quaternion input");
        }
        float[] q = normalize(quat);
        if (prevQuat == null) {
            prevQuat = q;
            return q.clone();
        }
        if (!(dtSeconds > 0.0)) {
            prevQuat = q;
            return q.clone();
        }
        double freq = 1.0 / dtSeconds;
        double angVelDeg = relativeAngleDeg(prevQuat, q) * freq;
        double cutoffDeg = minCutoffDeg + beta * Math.abs(angVelDeg);
        double alpha = alpha(freq, cutoffDeg);
        float[] blended = slerp(prevQuat, q, (float) clamp(alpha, 0.0, 1.0));
        prevQuat = blended;
        return blended.clone();
    }

    private static double alpha(double freq, double cutoffDeg) {
        double cutoff = Math.max(1e-3, cutoffDeg); // treat as pseudo-Hz
        double tau = 1.0 / (2.0 * Math.PI * cutoff);
        return 1.0 / (1.0 + tau * freq);
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private static float[] normalize(float[] q) {
        double len = Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (len == 0.0) {
            throw new Errors.InvalidInput("Quaternion length is zero");
        }
        float inv = (float) (1.0 / len);
        return new float[]{q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv};
    }

    private static double relativeAngleDeg(float[] a, float[] b) {
        float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
        dot = Math.max(-1.0f, Math.min(1.0f, dot));
        return Math.toDegrees(2.0 * Math.acos(Math.abs(dot)));
    }

    private static float[] slerp(float[] a, float[] b, float t) {
        float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
        float[] bb = b.clone();
        if (dot < 0.0f) {
            dot = -dot;
            bb[0] = -bb[0];
            bb[1] = -bb[1];
            bb[2] = -bb[2];
            bb[3] = -bb[3];
        }
        if (dot > 0.9995f) {
            float[] result = new float[4];
            for (int i = 0; i < 4; i++) {
                result[i] = a[i] + t * (bb[i] - a[i]);
            }
            return normalize(result);
        }
        double theta0 = Math.acos(dot);
        double sinTheta0 = Math.sin(theta0);
        double theta = theta0 * t;
        double sinTheta = Math.sin(theta);
        double s0 = Math.cos(theta) - dot * sinTheta / sinTheta0;
        double s1 = sinTheta / sinTheta0;
        float[] out = new float[4];
        for (int i = 0; i < 4; i++) {
            out[i] = (float) (s0 * a[i] + s1 * bb[i]);
        }
        return normalize(out);
    }
}
