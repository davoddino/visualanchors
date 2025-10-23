package org.example.visualanchors;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Locale;

/**
 * Mutable configuration holder with validation helpers.
 */
final class Config {
    private boolean intrinsicsSet = false;
    private double fx, fy, cx, cy;
    private double[] distCoeffs;
    private boolean defaultSizeSet = false;
    private double defaultQrSizeMeters;
    private final Map<String, Double> payloadSizes = new HashMap<>();
    private boolean externalTransformSet = false;
    private float[] externalCameraFromXR; // column-major 4x4

    private double smoothingPosMinCutoff = 1.0;
    private double smoothingPosBeta = 0.02;
    private double smoothingRotMinCutoffDeg = 1.0;
    private double smoothingRotBeta = 0.02;

    private double gatingMaxReprojErrPx = 6.0;
    private double gatingMinAreaPx = 64.0;
    private double gatingMaxCondH = 1.0e6;
    private double gatingMaxRotJumpDeg = 60.0;
    private double gatingMaxTransJumpM = 0.5;

    void setIntrinsics(double fx, double fy, double cx, double cy) {
        if (!(fx > 0.0) || !(fy > 0.0)) {
            throw new Errors.InvalidInput("Intrinsics must have fx, fy > 0");
        }
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        intrinsicsSet = true;
    }

    void setDistCoeffs(double[] dist) {
        if (dist == null || dist.length < 4) {
            throw new Errors.InvalidInput("Distortion coefficients must have length >= 4");
        }
        distCoeffs = dist.clone();
    }

    void setDefaultQrSizeMeters(double size) {
        if (!(size > 0.0)) {
            throw new Errors.InvalidInput("Default QR size must be > 0");
        }
        defaultQrSizeMeters = size;
        defaultSizeSet = true;
    }

    void setQrSizeForId(String id, double size) {
        if (id == null || id.isEmpty()) {
            throw new Errors.InvalidInput("Payload id must be non-empty");
        }
        if (!(size > 0.0)) {
            throw new Errors.InvalidInput("QR size override must be > 0");
        }
        payloadSizes.put(id, size);
    }

    void clearQrSizeOverrides() {
        payloadSizes.clear();
    }

    void setExternalCameraFromXR(float[] mat4x4ColumnMajor) {
        if (mat4x4ColumnMajor == null || mat4x4ColumnMajor.length != 16) {
            throw new Errors.InvalidInput("externalCamera_from_XR must be a 4x4 column-major matrix");
        }
        externalCameraFromXR = mat4x4ColumnMajor.clone();
        externalTransformSet = true;
    }

    void setSmoothingParams(double posMinCutoff, double posBeta,
                            double rotMinCutoffDeg, double rotBeta) {
        if (!(posMinCutoff > 0.0) || !(rotMinCutoffDeg > 0.0)) {
            throw new Errors.InvalidInput("Smoothing cutoffs must be > 0");
        }
        if (posBeta < 0.0 || rotBeta < 0.0) {
            throw new Errors.InvalidInput("Smoothing beta must be >= 0");
        }
        this.smoothingPosMinCutoff = posMinCutoff;
        this.smoothingPosBeta = posBeta;
        this.smoothingRotMinCutoffDeg = rotMinCutoffDeg;
        this.smoothingRotBeta = rotBeta;
    }

    void setGatingParams(double maxReprojErrPx, double minAreaPx,
                         double maxCondH, double maxRotJumpDeg, double maxTransJumpM) {
        if (!(maxReprojErrPx > 0.0) || !(minAreaPx > 0.0)) {
            throw new Errors.InvalidInput("Gating reprojection and min area must be > 0");
        }
        this.gatingMaxReprojErrPx = maxReprojErrPx;
        this.gatingMinAreaPx = minAreaPx;
        this.gatingMaxCondH = maxCondH;
        this.gatingMaxRotJumpDeg = maxRotJumpDeg;
        this.gatingMaxTransJumpM = maxTransJumpM;
    }

    void ensureReady() {
        if (!intrinsicsSet) {
            throw new Errors.ConfigurationMissing("Camera intrinsics not set (fx, fy, cx, cy required)");
        }
        if (distCoeffs == null || distCoeffs.length < 4) {
            throw new Errors.ConfigurationMissing("Distortion coefficients missing (min length 4)");
        }
        if (!defaultSizeSet) {
            throw new Errors.ConfigurationMissing("Default QR physical size (m) not set");
        }
        if (!externalTransformSet) {
            throw new Errors.ConfigurationMissing("externalCamera_from_XR transform not provided");
        }
    }

    double getFx() { return fx; }
    double getFy() { return fy; }
    double getCx() { return cx; }
    double getCy() { return cy; }
    double[] getDistCoeffs() { return distCoeffs; }
    double getDefaultQrSizeMeters() { return defaultQrSizeMeters; }
    float[] getExternalCameraFromXR() { return externalCameraFromXR; }

    Map<String, Double> getPayloadSizes() {
        return Collections.unmodifiableMap(payloadSizes);
    }

    double getSmoothingPosMinCutoff() { return smoothingPosMinCutoff; }
    double getSmoothingPosBeta() { return smoothingPosBeta; }
    double getSmoothingRotMinCutoffDeg() { return smoothingRotMinCutoffDeg; }
    double getSmoothingRotBeta() { return smoothingRotBeta; }
    double getGatingMaxReprojErrPx() { return gatingMaxReprojErrPx; }
    double getGatingMinAreaPx() { return gatingMinAreaPx; }
    double getGatingMaxCondH() { return gatingMaxCondH; }
    double getGatingMaxRotJumpDeg() { return gatingMaxRotJumpDeg; }
    double getGatingMaxTransJumpM() { return gatingMaxTransJumpM; }

    String debugSummary() {
        return String.format(Locale.US,
                "intrinsicsSet=%b fx=%.2f fy=%.2f cx=%.2f cy=%.2f distLen=%d defaultSize=%.4f externalSet=%b overrides=%d " +
                        "smooth(posCut=%.3f,posBeta=%.3f,rotCut=%.3f,rotBeta=%.3f) gate(reproj=%.3f,area=%.1f,cond=%.1e,rot=%.1f,trans=%.3f)",
                intrinsicsSet, fx, fy, cx, cy,
                distCoeffs != null ? distCoeffs.length : 0,
                defaultQrSizeMeters, externalTransformSet, payloadSizes.size(),
                smoothingPosMinCutoff, smoothingPosBeta, smoothingRotMinCutoffDeg, smoothingRotBeta,
                gatingMaxReprojErrPx, gatingMinAreaPx, gatingMaxCondH, gatingMaxRotJumpDeg, gatingMaxTransJumpM);
    }
}
