package org.example.visualanchors;

import android.app.Activity;
import android.content.pm.PackageManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.util.Log;
import android.util.Size;
import android.graphics.Rect;


import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import static android.Manifest.permission.CAMERA;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * Minimal Camera2 controller delivering luma frames via callback. Designed for single-consumer use
 * and keeps allocations to a minimum by reusing internal row buffers.
 */
final class CameraController implements AutoCloseable {
    private static final String TAG = "VA/CameraController";

    interface FrameListener {
        void onFrame(FrameData frame);
    }

    static final class FrameData {
        final byte[] luma;
        final int width;
        final int height;
        final int stride;
        final long timestampNs;

        FrameData(byte[] luma, int width, int height, int stride, long timestampNs) {
            this.luma = luma;
            this.width = width;
            this.height = height;
            this.stride = stride;
            this.timestampNs = timestampNs;
        }
    }

    static final class IntrinsicData {
        final float fx;
        final float fy;
        final float cx;
        final float cy;
        final float skew;
        final float[] distortion;
        final int width;
        final int height;

        IntrinsicData(float fx, float fy, float cx, float cy, float skew, float[] distortion, int width, int height) {
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            this.skew = skew;
            this.distortion = distortion;
            this.width = width;
            this.height = height;
        }
    }

    private final Activity activity;
    private final FrameListener listener;
    private final String preferredCameraId;

    private HandlerThread cameraThread;
    private Handler cameraHandler;
    private CameraDevice cameraDevice;
    private CameraCaptureSession captureSession;
    private ImageReader imageReader;
    private Size captureSize;
    private IntrinsicData cachedIntrinsics;

    private final AtomicBoolean startRequested = new AtomicBoolean(false);
    private byte[] rowBuffer = new byte[0];

    CameraController(Activity activity, FrameListener listener, String preferredCameraId) {
        this.activity = activity;
        this.listener = listener;
        this.preferredCameraId = preferredCameraId;
    }

    boolean start() {
        if (startRequested.getAndSet(true)) {
            Log.d(TAG, "start: already running");
            return true;
        }
        if (activity == null) {
            Log.w(TAG, "start: activity null");
            startRequested.set(false);
            return false;
        }
        if (!hasPermission(CAMERA)) {
            Log.w(TAG, "start: CAMERA permission not granted");
            startRequested.set(false);
            return false;
        }
        // Meta Quest specific camera permission; ignore if not declared.
        final String headsetPermission = "com.oculus.permission.HAND_TRACKING_CAMERA";
        try {
            if (activity.getPackageManager().getPermissionInfo(headsetPermission, 0) != null) {
                if (!hasPermission(headsetPermission)) {
                    Log.w(TAG, "start: headset camera permission not granted");
                    startRequested.set(false);
                    return false;
                }
            }
        } catch (PackageManager.NameNotFoundException ignore) {
            // Permission not declared; proceed.
        }
        final CameraManager manager = (CameraManager) activity.getSystemService(Activity.CAMERA_SERVICE);
        if (manager == null) {
            Log.w(TAG, "start: CameraManager unavailable");
            startRequested.set(false);
            return false;
        }
        try {
            String cameraId = selectCamera(manager);
            if (cameraId == null) {
                Log.w(TAG, "start: no suitable camera id found");
                startRequested.set(false);
                return false;
            }
            setupBackgroundThread();
            configureImageReader(manager, cameraId);
            if (imageReader == null) {
                Log.w(TAG, "start: imageReader not configured");
                teardownBackgroundThread();
                startRequested.set(false);
                return false;
            }
            final CountDownLatch openLatch = new CountDownLatch(1);
            final String logCameraId = cameraId;
            Runnable openTask = () -> {
                try {
                    Log.i(TAG, "start: opening camera " + logCameraId + " at " + captureSize);
                    manager.openCamera(logCameraId, new CameraDevice.StateCallback() {
                        @Override
                        public void onOpened(CameraDevice camera) {
                            Log.i(TAG, "camera onOpened " + logCameraId);
                            stateCallback.onOpened(camera);
                            openLatch.countDown();
                        }

                        @Override
                        public void onDisconnected(CameraDevice camera) {
                            stateCallback.onDisconnected(camera);
                            openLatch.countDown();
                        }

                        @Override
                        public void onError(CameraDevice camera, int error) {
                            stateCallback.onError(camera, error);
                            openLatch.countDown();
                        }
                    }, cameraHandler);
                } catch (CameraAccessException ex) {
                    Log.e(TAG, "start: CameraAccessException while opening camera", ex);
                    openLatch.countDown();
                }
            };
            if (Looper.myLooper() == Looper.getMainLooper()) {
                openTask.run();
            } else {
                activity.runOnUiThread(openTask);
            }
            try {
                if (!openLatch.await(2, TimeUnit.SECONDS)) {
                    Log.w(TAG, "start: camera open timed out");
                }
            } catch (InterruptedException ie) {
                Thread.currentThread().interrupt();
                Log.w(TAG, "start: camera open wait interrupted", ie);
            }
            return true;
        } catch (CameraAccessException ex) {
            Log.e(TAG, "start: CameraAccessException", ex);
        } catch (SecurityException se) {
            Log.e(TAG, "start: SecurityException", se);
        }
        startRequested.set(false);
        release();
        return false;
    }

    void stop() {
        if (!startRequested.getAndSet(false)) {
            return;
        }
        Log.i(TAG, "stop: stopping camera");
        release();
    }

    private void release() {
        closeQuietly(captureSession);
        captureSession = null;
        closeQuietly(cameraDevice);
        cameraDevice = null;
        if (imageReader != null) {
            imageReader.close();
            imageReader = null;
        }
        teardownBackgroundThread();
    }

    @Override
    public void close() {
        stop();
    }

    private void setupBackgroundThread() {
        cameraThread = new HandlerThread("VA-Camera");
        cameraThread.start();
        cameraHandler = new Handler(cameraThread.getLooper());
    }

    private void teardownBackgroundThread() {
        if (cameraThread != null) {
            cameraThread.quitSafely();
            try {
                cameraThread.join();
            } catch (InterruptedException ignore) {
            }
            cameraThread = null;
        }
        cameraHandler = null;
    }

    private boolean hasPermission(String permission) {
        return activity.checkSelfPermission(permission) == PackageManager.PERMISSION_GRANTED;
    }

    private String selectCamera(CameraManager manager) throws CameraAccessException {
        List<String> ids = Arrays.asList(manager.getCameraIdList());
        if (ids.isEmpty()) {
            return null;
        }
        if (preferredCameraId != null && ids.contains(preferredCameraId)) {
            return preferredCameraId;
        }
        String fallback = null;
        for (String id : ids) {
            CameraCharacteristics characteristics = manager.getCameraCharacteristics(id);
            Integer facing = characteristics.get(CameraCharacteristics.LENS_FACING);
            if (facing != null && facing == CameraCharacteristics.LENS_FACING_BACK) {
                return id;
            }
            fallback = id;
        }
        return fallback != null ? fallback : ids.get(0);
    }

    private void configureImageReader(CameraManager manager, String cameraId) throws CameraAccessException {
        CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);
        StreamConfigurationMap map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
        if (map == null) {
            Log.w(TAG, "configureImageReader: no StreamConfigurationMap");
            captureSize = new Size(1280, 720);
        } else {
            Size[] choices = map.getOutputSizes(ImageReader.class);
            if (choices == null || choices.length == 0) {
                choices = map.getOutputSizes(android.graphics.ImageFormat.YUV_420_888);
            }
            captureSize = chooseBestSize(choices);
        }
        if (captureSize == null) {
            captureSize = new Size(1280, 720);
        }
        imageReader = ImageReader.newInstance(captureSize.getWidth(), captureSize.getHeight(),
                android.graphics.ImageFormat.YUV_420_888, /*maxImages*/3);
        imageReader.setOnImageAvailableListener(this::onImageAvailable, cameraHandler);
        cacheIntrinsics(characteristics);
    }

    private Size chooseBestSize(Size[] choices) {
        if (choices == null || choices.length == 0) {
            return null;
        }
        List<Size> sizeList = Arrays.asList(choices);
        // Prefer 1280x720-ish, fallback to the largest under 1920x1080, else largest overall
        Size bestUnder1080 = null;
        Size bestOverall = sizeList.get(0);
        for (Size s : sizeList) {
            if (s.getWidth() >= bestOverall.getWidth() && s.getHeight() >= bestOverall.getHeight()) {
                bestOverall = s;
            }
            if (s.getWidth() <= 1920 && s.getHeight() <= 1080) {
                if (bestUnder1080 == null || (s.getWidth() >= bestUnder1080.getWidth() && s.getHeight() >= bestUnder1080.getHeight())) {
                    bestUnder1080 = s;
                }
            }
            if ((s.getWidth() == 1280 && s.getHeight() == 720) || (s.getWidth() == 720 && s.getHeight() == 1280)) {
                return s;
            }
        }
        return bestUnder1080 != null ? bestUnder1080 : bestOverall;
    }

    private final CameraDevice.StateCallback stateCallback = new CameraDevice.StateCallback() {
        @Override
        public void onOpened(CameraDevice camera) {
            cameraDevice = camera;
            createSession();
            Log.i(TAG, "stateCallback.onOpened: " + camera.getId());
        }

        @Override
        public void onDisconnected(CameraDevice camera) {
            Log.w(TAG, "Camera disconnected");
            closeQuietly(camera);
            cameraDevice = null;
            startRequested.set(false);
        }

        @Override
        public void onError(CameraDevice camera, int error) {
            Log.e(TAG, "Camera error: " + error);
            closeQuietly(camera);
            cameraDevice = null;
            startRequested.set(false);
        }
    };

    private void createSession() {
        if (cameraDevice == null || imageReader == null) {
            Log.w(TAG, "createSession: invalid state");
            return;
        }
        try {
            CaptureRequest.Builder builder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);
            builder.addTarget(imageReader.getSurface());
            builder.set(CaptureRequest.CONTROL_MODE, CaptureRequest.CONTROL_MODE_AUTO);
            cameraDevice.createCaptureSession(Collections.singletonList(imageReader.getSurface()), sessionCallback, cameraHandler);
        } catch (CameraAccessException ex) {
            Log.e(TAG, "createSession: CameraAccessException", ex);
            stop();
        }
    }

    private final CameraCaptureSession.StateCallback sessionCallback = new CameraCaptureSession.StateCallback() {
        @Override
        public void onConfigured(CameraCaptureSession session) {
            if (cameraDevice == null) {
                return;
            }
            captureSession = session;
            try {
                CaptureRequest.Builder builder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);
                builder.addTarget(imageReader.getSurface());
                builder.set(CaptureRequest.CONTROL_MODE, CaptureRequest.CONTROL_MODE_AUTO);
                builder.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);
                builder.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON);
                session.setRepeatingRequest(builder.build(), null, cameraHandler);
                Log.i(TAG, "Camera session configured at " + captureSize);
            } catch (CameraAccessException ex) {
                Log.e(TAG, "setRepeatingRequest failed", ex);
                stop();
            }
        }

        @Override
        public void onConfigureFailed(CameraCaptureSession session) {
            Log.e(TAG, "Camera session configure failed");
            stop();
        }
    };

    private void onImageAvailable(ImageReader reader) {
        Image image = reader.acquireLatestImage();
        if (image == null) {
            return;
        }
        try {
            Image.Plane[] planes = image.getPlanes();
            if (planes == null || planes.length == 0) {
                return;
            }
            Image.Plane lumaPlane = planes[0];
            ByteBuffer buffer = lumaPlane.getBuffer();
            int width = image.getWidth();
            int height = image.getHeight();
            int rowStride = lumaPlane.getRowStride();
            int pixelStride = lumaPlane.getPixelStride();
            if (buffer == null) {
                return;
            }
            byte[] data = copyLuma(buffer, width, height, rowStride, pixelStride);
            FrameData frame = new FrameData(data, width, height, width, image.getTimestamp());
            listener.onFrame(frame);
        } catch (Exception ex) {
            Log.e(TAG, "onImageAvailable: unexpected", ex);
        } finally {
            image.close();
        }
    }

    private byte[] copyLuma(ByteBuffer buffer, int width, int height, int rowStride, int pixelStride) {
        int needed = width * height;
        byte[] out = new byte[needed];
        buffer.rewind();
        if (pixelStride == 1 && rowStride == width) {
            buffer.get(out, 0, needed);
            return out;
        }
        ensureRowCapacity(rowStride);
        int offset = 0;
        for (int y = 0; y < height; y++) {
            int toRead = Math.min(rowStride, buffer.remaining());
            buffer.get(rowBuffer, 0, toRead);
            if (pixelStride == 1) {
                System.arraycopy(rowBuffer, 0, out, offset, Math.min(width, toRead));
                offset += width;
            } else {
                int limit = Math.min(width, toRead / Math.max(1, pixelStride));
                for (int x = 0; x < limit; x++) {
                    out[offset++] = rowBuffer[x * pixelStride];
                }
                offset += (width - limit);
            }
        }
        return out;
    }

    private void ensureRowCapacity(int size) {
        if (rowBuffer.length < size) {
            rowBuffer = new byte[size];
        }
    }

    private void cacheIntrinsics(CameraCharacteristics characteristics) {
        if (captureSize == null) {
            return;
        }
        float fx = 0f;
        float fy = 0f;
        float cx = captureSize.getWidth() * 0.5f;
        float cy = captureSize.getHeight() * 0.5f;
        float skew = 0f;

        Rect activeRect = characteristics.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE);
        float[] lensIntrinsics = characteristics.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION);
        if (lensIntrinsics != null && lensIntrinsics.length >= 4 && activeRect != null && activeRect.width() > 0 && activeRect.height() > 0) {
            float scaleX = captureSize.getWidth() / (float) activeRect.width();
            float scaleY = captureSize.getHeight() / (float) activeRect.height();
            fx = lensIntrinsics[0] * scaleX;
            fy = lensIntrinsics[1] * scaleY;
            cx = (lensIntrinsics[2] - activeRect.left) * scaleX;
            cy = (lensIntrinsics[3] - activeRect.top) * scaleY;
            if (lensIntrinsics.length > 4) {
                skew = lensIntrinsics[4] * scaleX;
            }
        } else {
            fx = captureSize.getWidth() * 0.9f;
            fy = captureSize.getHeight() * 0.9f;
        }

        float[] distortion = characteristics.get(CameraCharacteristics.LENS_DISTORTION);
        if (distortion != null) {
            distortion = Arrays.copyOf(distortion, distortion.length);
        }

        cachedIntrinsics = new IntrinsicData(fx, fy, cx, cy, skew, distortion, captureSize.getWidth(), captureSize.getHeight());
        Log.i(TAG, "Cached intrinsics fx=" + fx + " fy=" + fy + " cx=" + cx + " cy=" + cy);
    }

    IntrinsicData getCachedIntrinsics() {
        return cachedIntrinsics;
    }

    private static void closeQuietly(CameraCaptureSession session) {
        if (session != null) {
            try {
                session.close();
            } catch (Exception ignore) {
            }
        }
    }

    private static void closeQuietly(CameraDevice device) {
        if (device != null) {
            try {
                device.close();
            } catch (Exception ignore) {
            }
        }
    }
}
