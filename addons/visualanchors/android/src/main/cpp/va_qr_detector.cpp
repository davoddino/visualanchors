#include "va_qr_detector.h"

#include <opencv2/imgproc.hpp>
#include <android/log.h>
#include <opencv2/core/hal/interface.h>

namespace va {

QrDetector::QrDetector() {
    detector_.setUseAlignmentMarkers(true);
}

std::vector<QrDetection> QrDetector::detect(const cv::Mat& gray) {
    std::vector<QrDetection> detections;
    if (gray.empty()) {
        return detections;
    }
    std::vector<cv::String> payloads;
    cv::Mat points;
    std::vector<cv::Mat> straight;
    bool ok = false;
    try {
        ok = detector_.detectAndDecodeMulti(gray, payloads, points, straight);
    } catch (const cv::Exception& ex) {
        __android_log_print(ANDROID_LOG_ERROR, "VisualAnchors", "detectAndDecodeMulti threw: %s", ex.what());
        points.release();
        payloads.clear();
        return detections;
    }
    if (!ok && !points.empty()) {
        // When decoding fails but detection succeeded, attempt without decode.
        payloads.assign(points.rows, cv::String());
    }
    if (points.empty() || points.cols < 4) {
        return detections;
    }
    detections.reserve(points.rows);
    const bool isFloat = points.type() == CV_32FC2;
    const bool isDouble = points.type() == CV_64FC2;
    if (!isFloat && !isDouble) {
        __android_log_print(ANDROID_LOG_ERROR, "VisualAnchors", "Unexpected points Mat type=%d rows=%d cols=%d", points.type(), points.rows, points.cols);
        return detections;
    }
    for (int i = 0; i < points.rows; ++i) {
        std::vector<cv::Point2f> raw;
        raw.reserve(4);
        if (isFloat) {
            const cv::Vec2f* row = points.ptr<cv::Vec2f>(i);
            for (int j = 0; j < points.cols; ++j) {
                raw.emplace_back(row[j][0], row[j][1]);
            }
        } else {
            const cv::Vec2d* row = points.ptr<cv::Vec2d>(i);
            for (int j = 0; j < points.cols; ++j) {
                raw.emplace_back(static_cast<float>(row[j][0]), static_cast<float>(row[j][1]));
            }
        }
        auto ordered = order_corners(raw);
        double area = polygon_area(ordered);
        if (area <= 0.0) {
            continue;
        }
        QrDetection det;
        det.payload = i < static_cast<int>(payloads.size()) ? std::string(payloads[i]) : std::string();
        det.corners = ordered;
        det.area_px = area;
        det.cond_h = homography_condition(ordered);
        detections.push_back(det);
    }
    return detections;
}

} // namespace va
