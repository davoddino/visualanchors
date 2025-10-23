package org.example.visualanchors;

import java.util.Map;

/**
 * JNI-backed adapter that feeds Quest camera frames to the native OpenCV pipeline.
 */
final class QrDetectorOpenCV {
    static {
        System.loadLibrary("visualanchors");
    }

    static final class DetectionResult {
        final String payload;
        final float[] corners;  // length 8 (TL,TR,BR,BL)
        final float[] transform; // column-major 4x4
        final double reprojectionErrorPx;
        final double areaPx;

        DetectionResult(String payload, float[] corners, float[] transform,
                        double reprojectionErrorPx, double areaPx) {
            this.payload = payload;
            this.corners = corners;
            this.transform = transform;
            this.reprojectionErrorPx = reprojectionErrorPx;
            this.areaPx = areaPx;
        }
    }

    DetectionResult[] detect(byte[] yPlane,
                             int width,
                             int height,
                             int rowStride,
                             long timestampNs,
                             Config config) {
        if (config == null) {
            throw new Errors.InvalidInput("Config must not be null");
        }
        Map<String, Double> overrides = config.getPayloadSizes();
        String[] ids = overrides.keySet().toArray(new String[0]);
        double[] sizes = new double[ids.length];
        for (int i = 0; i < ids.length; i++) {
            sizes[i] = overrides.get(ids[i]);
        }
        return nativeDetectAndEstimate(
                yPlane,
                width,
                height,
                rowStride,
                timestampNs,
                config.getFx(),
                config.getFy(),
                config.getCx(),
                config.getCy(),
                config.getDistCoeffs(),
                config.getDefaultQrSizeMeters(),
                ids,
                sizes,
                config.getExternalCameraFromXR(),
                config.getGatingMaxReprojErrPx(),
                config.getGatingMinAreaPx(),
                config.getGatingMaxCondH()
        );
    }

    private static native DetectionResult[] nativeDetectAndEstimate(
            byte[] yPlane,
            int width,
            int height,
            int rowStride,
            long timestampNs,
            double fx,
            double fy,
            double cx,
            double cy,
            double[] distCoeffs,
            double defaultQrSizeMeters,
            String[] payloadIds,
            double[] payloadSizes,
            float[] externalCameraFromXR,
            double gatingMaxReprojErrPx,
            double gatingMinAreaPx,
            double gatingMaxCondH);
}
