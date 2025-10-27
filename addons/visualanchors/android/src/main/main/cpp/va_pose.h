#pragma once

#include <array>

#include <opencv2/calib3d.hpp>

#include "va_math.h"

namespace va {

struct PoseResult {
    bool ok = false;
    cv::Matx44f T_cam = cv::Matx44f::eye();
    cv::Matx44f T_xr = cv::Matx44f::eye();
    double reprojection_error_px = 0.0;
};

PoseResult estimate_pose_ippe_square(const std::array<cv::Point2f, 4>& imgPts,
                                     double qr_size_m,
                                     const cv::Matx33d& K,
                                     const cv::Mat& distCoeffs,
                                     const cv::Matx44f& external_cam_from_xr);

} // namespace va
