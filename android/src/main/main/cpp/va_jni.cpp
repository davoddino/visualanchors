#include <jni.h>

#include <cstdarg>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <android/log.h>

#include "va_math.h"
#include "va_pose.h"
#include "va_qr_detector.h"

namespace {

constexpr char kTag[] = "VisualAnchors";

inline void log_info(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    __android_log_vprint(ANDROID_LOG_INFO, kTag, fmt, args);
    va_end(args);
}

void throwException(JNIEnv* env, const char* className, const std::string& message) {
    jclass clazz = env->FindClass(className);
    if (clazz == nullptr) {
        clazz = env->FindClass("java/lang/RuntimeException");
    }
    __android_log_print(ANDROID_LOG_ERROR, kTag, "Throwing %s: %s", className, message.c_str());
    env->ThrowNew(clazz, message.c_str());
}

std::map<std::string, double> buildOverrideMap(JNIEnv* env, jobjectArray ids, jdoubleArray sizes) {
    std::map<std::string, double> map;
    if (!ids || !sizes) {
        return map;
    }
    jsize len = env->GetArrayLength(ids);
    jsize sizeLen = env->GetArrayLength(sizes);
    if (len != sizeLen) {
        return map;
    }
    jdouble* sizeElements = env->GetDoubleArrayElements(sizes, nullptr);
    for (jsize i = 0; i < len; ++i) {
        auto str = static_cast<jstring>(env->GetObjectArrayElement(ids, i));
        const char* utf = env->GetStringUTFChars(str, nullptr);
        map.emplace(std::string(utf ? utf : ""), sizeElements[i]);
        env->ReleaseStringUTFChars(str, utf);
        env->DeleteLocalRef(str);
    }
    env->ReleaseDoubleArrayElements(sizes, sizeElements, JNI_ABORT);
    return map;
}

cv::Matx33d cameraMatrix(double fx, double fy, double cx, double cy) {
    return cv::Matx33d(
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
    );
}

cv::Mat distortionCoeffs(JNIEnv* env, jdoubleArray array) {
    if (array == nullptr) {
        return cv::Mat();
    }
    jsize len = env->GetArrayLength(array);
    cv::Mat dist(len, 1, CV_64F);
    jdouble* elems = env->GetDoubleArrayElements(array, nullptr);
    for (jsize i = 0; i < len; ++i) {
        dist.at<double>(i, 0) = elems[i];
    }
    env->ReleaseDoubleArrayElements(array, elems, JNI_ABORT);
    return dist;
}

} // namespace

extern "C" JNIEXPORT jobjectArray JNICALL
Java_org_example_visualanchors_QrDetectorOpenCV_nativeDetectAndEstimate(
        JNIEnv* env,
        jclass,
        jbyteArray yPlane,
        jint width,
        jint height,
        jint rowStride,
        jlong /*timestampNs*/,
        jdouble fx,
        jdouble fy,
        jdouble cx,
        jdouble cy,
        jdoubleArray distCoeffs,
        jdouble defaultSizeMeters,
        jobjectArray payloadIds,
        jdoubleArray payloadSizes,
        jfloatArray externalCameraFromXR,
        jdouble gatingMaxReprojErrPx,
        jdouble gatingMinAreaPx,
        jdouble gatingMaxCondH) {
    if (yPlane == nullptr || width <= 0 || height <= 0 || rowStride <= 0) {
        throwException(env, "org/example/visualanchors/Errors$InvalidInput",
                       "Invalid frame arguments");
        return nullptr;
    }
    if (!(fx > 0.0) || !(fy > 0.0)) {
        throwException(env, "org/example/visualanchors/Errors$ConfigurationMissing",
                       "Camera intrinsics not set");
        return nullptr;
    }
    if (!(defaultSizeMeters > 0.0)) {
        throwException(env, "org/example/visualanchors/Errors$ConfigurationMissing",
                       "Default QR size missing");
        return nullptr;
    }

    jsize byteLen = env->GetArrayLength(yPlane);
    jbyte* frameBytes = env->GetByteArrayElements(yPlane, nullptr);

    cv::Mat gray(height, width, CV_8UC1, reinterpret_cast<unsigned char*>(frameBytes), rowStride);
    cv::Matx33d K = cameraMatrix(fx, fy, cx, cy);
    cv::Mat dist = distortionCoeffs(env, distCoeffs);

    std::map<std::string, double> overrides = buildOverrideMap(env, payloadIds, payloadSizes);

    log_info("nativeDetectAndEstimate frame=%dx%d stride=%d overrides=%zu default=%.3f fx=%.2f fy=%.2f",
             width, height, rowStride, overrides.size(), defaultSizeMeters, fx, fy);

    std::vector<float> externalVec = {
            1.f, 0.f, 0.f, 0.f,
            0.f, 1.f, 0.f, 0.f,
            0.f, 0.f, 1.f, 0.f,
            0.f, 0.f, 0.f, 1.f
    };
    if (externalCameraFromXR != nullptr) {
        jfloat* externalPtr = env->GetFloatArrayElements(externalCameraFromXR, nullptr);
        for (int i = 0; i < 16; ++i) {
            externalVec[i] = externalPtr[i];
        }
        env->ReleaseFloatArrayElements(externalCameraFromXR, externalPtr, JNI_ABORT);
    }
    cv::Matx44f external = va::mat_from_column_major(externalVec.data(), externalVec.size());

    va::QrDetector detector;
    std::vector<va::QrDetection> detections;
    try {
        detections = detector.detect(gray);
    } catch (const cv::Exception& ex) {
        env->ReleaseByteArrayElements(yPlane, frameBytes, JNI_ABORT);
        __android_log_print(ANDROID_LOG_ERROR, kTag, "detect threw cv::Exception: %s", ex.what());
        throwException(env, "org/example/visualanchors/Errors$PoseComputationFailed",
                       std::string("OpenCV detect failed: ") + ex.what());
        return nullptr;
    } catch (const std::exception& ex) {
        env->ReleaseByteArrayElements(yPlane, frameBytes, JNI_ABORT);
        __android_log_print(ANDROID_LOG_ERROR, kTag, "detect threw std::exception: %s", ex.what());
        throwException(env, "org/example/visualanchors/Errors$PoseComputationFailed",
                       std::string("Detector failed: ") + ex.what());
        return nullptr;
    } catch (...) {
        env->ReleaseByteArrayElements(yPlane, frameBytes, JNI_ABORT);
        __android_log_print(ANDROID_LOG_ERROR, kTag, "detect threw unknown exception");
        throwException(env, "org/example/visualanchors/Errors$PoseComputationFailed",
                       "Detector failed: unknown exception");
        return nullptr;
    }

    env->ReleaseByteArrayElements(yPlane, frameBytes, JNI_ABORT);

    if (detections.empty()) {
        log_info("nativeDetectAndEstimate no detections");
        jclass detectionClass = env->FindClass("org/example/visualanchors/QrDetectorOpenCV$DetectionResult");
        return env->NewObjectArray(0, detectionClass, nullptr);
    }

    std::vector<va::PoseResult> poseResults;
    poseResults.reserve(detections.size());
    std::vector<std::array<float, 8>> orderedCorners;
    orderedCorners.reserve(detections.size());
    std::vector<std::string> payloads;
    payloads.reserve(detections.size());
    std::vector<double> areas;
    areas.reserve(detections.size());

    for (const auto& det : detections) {
        log_info("Detection payload=%s area_px=%.2f cond_h=%.4f", det.payload.c_str(), det.area_px, det.cond_h);
        if (gatingMinAreaPx > 0.0 && det.area_px < gatingMinAreaPx) {
            log_info("Skipping %s: area %.2f < min_area_px %.2f", det.payload.c_str(), det.area_px, gatingMinAreaPx);
            continue;
        }
        if (gatingMaxCondH > 0.0 && det.cond_h > gatingMaxCondH) {
            log_info("Skipping %s: cond_h %.3f > max_cond_h %.3f", det.payload.c_str(), det.cond_h, gatingMaxCondH);
            continue;
        }
        std::string payload_id = det.payload;
        if (payload_id.empty()) {
            // Skip detections without payload text; size lookup cannot work.
            continue;
        }
        double sizeMeters = defaultSizeMeters;
        auto it = overrides.find(payload_id);
        if (it != overrides.end()) {
            sizeMeters = it->second;
        }
        if (!(sizeMeters > 0.0)) {
            throwException(env, "org/example/visualanchors/Errors$ConfigurationMissing",
                           "QR size missing for payload " + payload_id);
            return nullptr;
        }
        log_info("Estimating pose payload=%s size_m=%.4f", payload_id.c_str(), sizeMeters);
        va::PoseResult pose;
        try {
            pose = va::estimate_pose_ippe_square(det.corners, sizeMeters, K, dist, external);
        } catch (const cv::Exception& ex) {
            __android_log_print(ANDROID_LOG_ERROR, kTag, "estimate_pose threw cv::Exception: %s", ex.what());
            throwException(env, "org/example/visualanchors/Errors$PoseComputationFailed",
                           std::string("Pose estimation failed for payload ") + payload_id + ": " + ex.what());
            return nullptr;
        } catch (const std::exception& ex) {
            __android_log_print(ANDROID_LOG_ERROR, kTag, "estimate_pose threw std::exception: %s", ex.what());
            throwException(env, "org/example/visualanchors/Errors$PoseComputationFailed",
                           std::string("Pose estimation failed for payload ") + payload_id + ": " + ex.what());
            return nullptr;
        } catch (...) {
            __android_log_print(ANDROID_LOG_ERROR, kTag, "estimate_pose threw unknown exception");
            throwException(env, "org/example/visualanchors/Errors$PoseComputationFailed",
                           std::string("Pose estimation failed for payload ") + payload_id + ": unknown exception");
            return nullptr;
        }
        if (!pose.ok) {
            throwException(env, "org/example/visualanchors/Errors$PoseComputationFailed",
                           "IPPE failed for payload " + payload_id);
            return nullptr;
        }
        if (gatingMaxReprojErrPx > 0.0 && pose.reprojection_error_px > gatingMaxReprojErrPx) {
            log_info("Skipping %s: reproj %.3f > max_reproj_err %.3f", payload_id.c_str(), pose.reprojection_error_px, gatingMaxReprojErrPx);
            continue;
        }
        log_info("Pose accepted payload=%s reproj=%.3f", payload_id.c_str(), pose.reprojection_error_px);
        poseResults.push_back(pose);
        cv::Vec3d trans = va::translation_from(pose.T_cam);
        cv::Matx33d rot = va::mat3_from_column_major(pose.T_cam);
        log_info("Pose camera translation=(%.3f, %.3f, %.3f) forward=(%.3f, %.3f, %.3f)",
                 trans[0], trans[1], trans[2],
                 rot(0, 2), rot(1, 2), rot(2, 2));
        std::array<float, 8> flatCorners{};
        for (int i = 0; i < 4; ++i) {
            flatCorners[2 * i] = det.corners[i].x;
            flatCorners[2 * i + 1] = det.corners[i].y;
        }
        orderedCorners.push_back(flatCorners);
        payloads.push_back(payload_id);
        areas.push_back(det.area_px);
    }

    log_info("nativeDetectAndEstimate detections=%zu poseResults=%zu", detections.size(), poseResults.size());

    jclass detectionClass = env->FindClass("org/example/visualanchors/QrDetectorOpenCV$DetectionResult");
    if (detectionClass == nullptr) {
        throwException(env, "java/lang/RuntimeException", "DetectionResult class not found");
        return nullptr;
    }
    jmethodID ctor = env->GetMethodID(detectionClass, "<init>",
                                      "(Ljava/lang/String;[F[FDD)V");
    if (ctor == nullptr) {
        throwException(env, "java/lang/RuntimeException", "DetectionResult constructor not found");
        return nullptr;
    }

    jobjectArray outArray = env->NewObjectArray(static_cast<jsize>(poseResults.size()), detectionClass, nullptr);
    for (size_t i = 0; i < poseResults.size(); ++i) {
        jstring payloadStr = env->NewStringUTF(payloads[i].c_str());
        jfloatArray cornersArray = env->NewFloatArray(8);
        env->SetFloatArrayRegion(cornersArray, 0, 8, orderedCorners[i].data());

        std::array<float, 16> transform = va::mat_to_column_major(poseResults[i].T_xr);
        jfloatArray transformArray = env->NewFloatArray(16);
        env->SetFloatArrayRegion(transformArray, 0, 16, transform.data());

        jobject detection = env->NewObject(
                detectionClass,
                ctor,
                payloadStr,
                cornersArray,
                transformArray,
                static_cast<jdouble>(poseResults[i].reprojection_error_px),
                static_cast<jdouble>(areas[i]));
        env->SetObjectArrayElement(outArray, static_cast<jsize>(i), detection);

        env->DeleteLocalRef(payloadStr);
        env->DeleteLocalRef(cornersArray);
        env->DeleteLocalRef(transformArray);
        env->DeleteLocalRef(detection);
    }
    return outArray;
}
