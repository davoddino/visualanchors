package org.example.visualanchors;

import android.util.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Orchestrates JNI-backed detection results and applies post-filters (smoothing + gating).
 */
final class PoseEstimator {
    private static final String TAG = "VisualAnchorsTrack";

    private final QrDetectorOpenCV detector = new QrDetectorOpenCV();
    private final Map<String, TrackState> tracks = new HashMap<>();

    static final class PoseOutput {
        final String payload;
        final float[] transform; // column-major 4x4 (XR frame)
        final double reprojectionErrorPx;
        final double areaPx;
        final long timestampNs;

        PoseOutput(String payload, float[] transform, double reprojectionErrorPx, double areaPx, long timestampNs) {
            this.payload = payload;
            this.transform = transform;
            this.reprojectionErrorPx = reprojectionErrorPx;
            this.areaPx = areaPx;
            this.timestampNs = timestampNs;
        }
    }

    PoseOutput[] processFrame(byte[] yPlane,
                              int width,
                              int height,
                              int rowStride,
                              long timestampNs,
                              Config config) {
        QrDetectorOpenCV.DetectionResult[] detections = detector.detect(
                yPlane, width, height, rowStride, timestampNs, config);
        if (detections == null || detections.length == 0) {
            return new PoseOutput[0];
        }
        List<PoseOutput> outputs = new ArrayList<>(detections.length);
        for (QrDetectorOpenCV.DetectionResult det : detections) {
            if (det == null || det.payload == null || det.payload.isEmpty()) {
                continue;
            }
            TrackState track = tracks.computeIfAbsent(det.payload, key ->
                    new TrackState(
                            config.getSmoothingPosMinCutoff(),
                            config.getSmoothingPosBeta(),
                            config.getSmoothingRotMinCutoffDeg(),
                            config.getSmoothingRotBeta()
                    )
            );
            PoseOutput filtered = track.apply(det, config, timestampNs);
            if (filtered != null) {
                outputs.add(filtered);
            }
        }
        return outputs.toArray(new PoseOutput[0]);
    }

    void reset() {
        tracks.clear();
    }

    private static final class TrackState {
        private final OneEuroFilter translationFilter;
        private final QuatSmoother rotationFilter;
        private float[] lastFilteredMatrix;
        private long lastTimestampNs = 0L;

        TrackState(double posMinCutoff, double posBeta, double rotMinCutoffDeg, double rotBeta) {
            translationFilter = new OneEuroFilter(posMinCutoff, posBeta);
            rotationFilter = new QuatSmoother(rotMinCutoffDeg, rotBeta);
        }

        PoseOutput apply(QrDetectorOpenCV.DetectionResult det, Config config, long timestampNs) {
            double[] translation = extractTranslation(det.transform);
            float[] quaternion = extractQuaternion(det.transform);
            if (lastFilteredMatrix != null && lastTimestampNs > 0L) {
                double rawTransJump = distance(translation, extractTranslation(lastFilteredMatrix));
                double rawRotJump = Math.abs(relativeAngleDeg(quaternion, extractQuaternion(lastFilteredMatrix)));
                boolean transExceeded = config.getGatingMaxTransJumpM() > 0.0 && rawTransJump > config.getGatingMaxTransJumpM();
                boolean rotExceeded = config.getGatingMaxRotJumpDeg() > 0.0 && rawRotJump > config.getGatingMaxRotJumpDeg();
                if (transExceeded || rotExceeded) {
                    Log.d(TAG, "Gating raw jump payload=" + det.payload
                            + " rawTransJump=" + String.format(Locale.US, "%.4f", rawTransJump)
                            + " rawRotJumpDeg=" + String.format(Locale.US, "%.2f", rawRotJump)
                            + " maxTrans=" + config.getGatingMaxTransJumpM()
                            + " maxRot=" + config.getGatingMaxRotJumpDeg());
                    return null;
                }
            }
            double dt = 0.0;
            if (lastTimestampNs > 0L && timestampNs > lastTimestampNs) {
                dt = (timestampNs - lastTimestampNs) * 1e-9;
            }
            double[] filteredT = translationFilter.filter(translation, dt > 0.0 ? dt : 1.0 / 60.0);
            float[] filteredQ = rotationFilter.filter(quaternion, dt > 0.0 ? dt : 1.0 / 60.0);

            if (lastFilteredMatrix != null && dt > 0.0) {
                double transJump = distance(filteredT, extractTranslation(lastFilteredMatrix));
                double rotJump = Math.abs(relativeAngleDeg(filteredQ, extractQuaternion(lastFilteredMatrix)));
                boolean transExceeded = config.getGatingMaxTransJumpM() > 0.0 && transJump > config.getGatingMaxTransJumpM();
                boolean rotExceeded = config.getGatingMaxRotJumpDeg() > 0.0 && rotJump > config.getGatingMaxRotJumpDeg();
                if (transExceeded || rotExceeded) {
                    Log.d(TAG, "Gating filtered jump payload=" + det.payload
                            + " transJump=" + String.format(Locale.US, "%.4f", transJump)
                            + " rotJumpDeg=" + String.format(Locale.US, "%.2f", rotJump)
                            + " maxTrans=" + config.getGatingMaxTransJumpM()
                            + " maxRot=" + config.getGatingMaxRotJumpDeg());
                    return null;
                }
            }

            float[] filteredMatrix = composeMatrix(filteredQ, filteredT);
            lastFilteredMatrix = filteredMatrix;
            lastTimestampNs = timestampNs;
            if (Log.isLoggable(TAG, Log.VERBOSE)) {
                Log.v(TAG, "Pose accepted payload=" + det.payload
                        + " t=" + formatVec(filteredT)
                        + " dt=" + String.format(Locale.US, "%.4f", dt)
                        + " reproj=" + String.format(Locale.US, "%.3f", det.reprojectionErrorPx));
            }
            return new PoseOutput(det.payload, filteredMatrix, det.reprojectionErrorPx, det.areaPx, timestampNs);
        }

        private static double[] extractTranslation(float[] mat) {
            return new double[]{mat[12], mat[13], mat[14]};
        }

        private static float[] extractQuaternion(float[] mat) {
            float m00 = mat[0]; float m10 = mat[1]; float m20 = mat[2];
            float m01 = mat[4]; float m11 = mat[5]; float m21 = mat[6];
            float m02 = mat[8]; float m12 = mat[9]; float m22 = mat[10];
            float trace = m00 + m11 + m22;
            float x, y, z, w;
            if (trace > 0f) {
                float s = (float) Math.sqrt(trace + 1.0f) * 2f;
                w = 0.25f * s;
                x = (m21 - m12) / s;
                y = (m02 - m20) / s;
                z = (m10 - m01) / s;
            } else if ((m00 > m11) && (m00 > m22)) {
                float s = (float) Math.sqrt(1.0f + m00 - m11 - m22) * 2f;
                w = (m21 - m12) / s;
                x = 0.25f * s;
                y = (m01 + m10) / s;
                z = (m02 + m20) / s;
            } else if (m11 > m22) {
                float s = (float) Math.sqrt(1.0f + m11 - m00 - m22) * 2f;
                w = (m02 - m20) / s;
                x = (m01 + m10) / s;
                y = 0.25f * s;
                z = (m12 + m21) / s;
            } else {
                float s = (float) Math.sqrt(1.0f + m22 - m00 - m11) * 2f;
                w = (m10 - m01) / s;
                x = (m02 + m20) / s;
                y = (m12 + m21) / s;
                z = 0.25f * s;
            }
            return new float[]{x, y, z, w};
        }

        private static float[] composeMatrix(float[] quat, double[] translation) {
            float[] q = quat.clone();
            double len = Math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
            if (!(len > 0.0)) {
                throw new Errors.InvalidInput("Quaternion has zero length");
            }
            float inv = (float) (1.0 / len);
            q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
            float x = q[0], y = q[1], z = q[2], w = q[3];
            float xx = x * x;
            float yy = y * y;
            float zz = z * z;
            float xy = x * y;
            float xz = x * z;
            float yz = y * z;
            float wx = w * x;
            float wy = w * y;
            float wz = w * z;
            float[] m = new float[16];
            m[0] = 1.0f - 2.0f * (yy + zz);
            m[1] = 2.0f * (xy + wz);
            m[2] = 2.0f * (xz - wy);
            m[3] = 0.0f;

            m[4] = 2.0f * (xy - wz);
            m[5] = 1.0f - 2.0f * (xx + zz);
            m[6] = 2.0f * (yz + wx);
            m[7] = 0.0f;

            m[8] = 2.0f * (xz + wy);
            m[9] = 2.0f * (yz - wx);
            m[10] = 1.0f - 2.0f * (xx + yy);
            m[11] = 0.0f;

            m[12] = (float) translation[0];
            m[13] = (float) translation[1];
            m[14] = (float) translation[2];
            m[15] = 1.0f;
            return m;
        }

        private static double distance(double[] a, double[] b) {
            double dx = a[0] - b[0];
            double dy = a[1] - b[1];
            double dz = a[2] - b[2];
            return Math.sqrt(dx * dx + dy * dy + dz * dz);
        }

        private static String formatVec(double[] v) {
            if (v == null || v.length < 3) {
                return "( NaN, NaN, NaN )";
            }
            return String.format(Locale.US, "(%.4f, %.4f, %.4f)", v[0], v[1], v[2]);
        }

        private static double relativeAngleDeg(float[] q1, float[] q2) {
            float dot = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
            dot = Math.max(-1.0f, Math.min(1.0f, dot));
            return Math.toDegrees(2.0 * Math.acos(Math.abs(dot)));
        }
    }
}
