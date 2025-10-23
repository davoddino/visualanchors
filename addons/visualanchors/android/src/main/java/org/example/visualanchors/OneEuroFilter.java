package org.example.visualanchors;

import java.util.Arrays;

/**
 * One-Euro filter for 3D translation vectors.
 */
final class OneEuroFilter {
    private final double minCutoff;
    private final double beta;
    private final double dCutoff;
    private double[] prevValue;
    private double[] prevDerivative;

    OneEuroFilter(double minCutoff, double beta) {
        this(minCutoff, beta, 1.0);
    }

    OneEuroFilter(double minCutoff, double beta, double dCutoff) {
        if (!(minCutoff > 0.0)) {
            throw new Errors.InvalidInput("OneEuro minCutoff must be > 0");
        }
        if (beta < 0.0) {
            throw new Errors.InvalidInput("OneEuro beta must be >= 0");
        }
        this.minCutoff = minCutoff;
        this.beta = beta;
        this.dCutoff = dCutoff;
    }

    double[] reset(double[] initial) {
        prevValue = initial != null ? initial.clone() : null;
        prevDerivative = null;
        return prevValue;
    }

    double[] filter(double[] value, double dtSeconds) {
        if (value == null || value.length != 3) {
            throw new Errors.InvalidInput("OneEuroFilter expects 3D input");
        }
        if (!(dtSeconds > 0.0)) {
            // fallback to returning original value with reset
            prevValue = value.clone();
            prevDerivative = new double[]{0.0, 0.0, 0.0};
            return prevValue;
        }
        double freq = 1.0 / dtSeconds;
        if (prevValue == null) {
            prevValue = value.clone();
            prevDerivative = new double[]{0.0, 0.0, 0.0};
            return prevValue;
        }
        double alphaDerivative = alpha(freq, dCutoff);
        double[] derivative = new double[3];
        for (int i = 0; i < 3; i++) {
            double dx = (value[i] - prevValue[i]) * freq;
            double prevDx = prevDerivative != null ? prevDerivative[i] : 0.0;
            derivative[i] = lerp(prevDx, dx, alphaDerivative);
        }
        double[] cutoff = new double[3];
        for (int i = 0; i < 3; i++) {
            cutoff[i] = minCutoff + beta * Math.abs(derivative[i]);
        }
        double[] filtered = new double[3];
        for (int i = 0; i < 3; i++) {
            double alpha = alpha(freq, cutoff[i]);
            filtered[i] = lerp(prevValue[i], value[i], alpha);
        }
        prevValue = filtered;
        prevDerivative = derivative;
        return Arrays.copyOf(filtered, 3);
    }

    private static double alpha(double freq, double cutoff) {
        double tau = 1.0 / (2.0 * Math.PI * cutoff);
        return 1.0 / (1.0 + tau * freq);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
