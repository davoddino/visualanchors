#pragma once

#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>

#include "va_math.h"

namespace va {

struct QrDetection {
    std::string payload;
    std::array<cv::Point2f, 4> corners;
    double area_px = 0.0;
    double cond_h = 0.0;
};

class QrDetector {
public:
    QrDetector();

    std::vector<QrDetection> detect(const cv::Mat& gray);

private:
    cv::QRCodeDetector detector_;
};

} // namespace va
