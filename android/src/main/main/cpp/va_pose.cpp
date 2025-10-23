#include "va_pose.h"

#include <limits>

namespace va {

PoseResult estimate_pose_ippe_square(const std::array<cv::Point2f, 4>& imgPts,
                                     double qr_size_m,
                                     const cv::Matx33d& K,
                                     const cv::Mat& distCoeffs,
                                     const cv::Matx44f& external_cam_from_xr) {
    PoseResult result;
    if (!(qr_size_m > 0.0)) {
        return result;
    }
    float half = static_cast<float>(qr_size_m * 0.5);
    std::vector<cv::Point3f> objPts = {
            {-half,  half, 0.f},
            { half,  half, 0.f},
            { half, -half, 0.f},
            {-half, -half, 0.f}
    };
    std::vector<cv::Point2f> imgVec(imgPts.begin(), imgPts.end());

    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat reproj;
    bool solved = cv::solvePnPGeneric(objPts, imgVec, K, distCoeffs, rvecs, tvecs,
                                      false, cv::SOLVEPNP_IPPE_SQUARE, reproj);
    if (!solved || rvecs.empty() || tvecs.empty()) {
        return result;
    }
    std::vector<cv::Vec3d> tvecValues;
    tvecValues.reserve(tvecs.size());
    std::vector<cv::Vec3d> rvecValues;
    rvecValues.reserve(rvecs.size());
    for (size_t i = 0; i < rvecs.size(); ++i) {
        cv::Mat r64, t64;
        rvecs[i].convertTo(r64, CV_64F);
        tvecs[i].convertTo(t64, CV_64F);
        rvecValues.emplace_back(r64.at<double>(0), r64.at<double>(1), r64.at<double>(2));
        tvecValues.emplace_back(t64.at<double>(0), t64.at<double>(1), t64.at<double>(2));
    }

    double bestError = std::numeric_limits<double>::infinity();
    int bestIdx = -1;
    for (size_t i = 0; i < tvecValues.size(); ++i) {
        const cv::Vec3d& tVec = tvecValues[i];
        double err = reproj.empty() ? 0.0 : reproj.at<double>(static_cast<int>(i), 0);
        if (tVec[2] <= 0.0) {
            continue;
        }
        if (err < bestError) {
            bestError = err;
            bestIdx = static_cast<int>(i);
        }
    }
    if (bestIdx < 0) {
        // fall back to minimal reprojection error even if Z <= 0 (cheirality failure)
        for (size_t i = 0; i < tvecValues.size(); ++i) {
            double err = reproj.empty() ? 0.0 : reproj.at<double>(static_cast<int>(i), 0);
            if (err < bestError) {
                bestError = err;
                bestIdx = static_cast<int>(i);
            }
        }
    }
    if (bestIdx < 0) {
        return result;
    }
    cv::Vec3d rVec = rvecValues[bestIdx];
    cv::Vec3d tVec = tvecValues[bestIdx];

    if (!imgVec.empty()) {
        cv::Mat rMat(3, 1, CV_64F);
        cv::Mat tMat(3, 1, CV_64F);
        for (int i = 0; i < 3; ++i) {
            rMat.at<double>(i, 0) = rVec[i];
            tMat.at<double>(i, 0) = tVec[i];
        }
        try {
            cv::solvePnPRefineLM(objPts, imgVec, K, distCoeffs, rMat, tMat);
            for (int i = 0; i < 3; ++i) {
                rVec[i] = rMat.at<double>(i, 0);
                tVec[i] = tMat.at<double>(i, 0);
            }
        } catch (const cv::Exception&) {
            // Refine may fail for degenerate views; fall back to the original solution.
        }
    }

    cv::Matx33d R_cam_from_marker_cv;
    cv::Rodrigues(rVec, R_cam_from_marker_cv);
    const cv::Matx33d S = cv::Matx33d::diag(cv::Vec3d(1.0, -1.0, -1.0));
    cv::Matx33d R_cam_from_marker = S * R_cam_from_marker_cv;
    cv::Vec3d t_cam_from_marker = S * tVec;

    cv::Matx44f T_cam = make_transform(R_cam_from_marker, t_cam_from_marker);
    cv::Matx44f T_xr = multiply(external_cam_from_xr, T_cam);

    result.ok = true;
    result.T_cam = T_cam;
    result.T_xr = T_xr;
    if (!imgVec.empty()) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(objPts, rVec, tVec, K, distCoeffs, projected);
        double sumSq = 0.0;
        for (size_t i = 0; i < projected.size() && i < imgVec.size(); ++i) {
            double dx = static_cast<double>(projected[i].x) - static_cast<double>(imgVec[i].x);
            double dy = static_cast<double>(projected[i].y) - static_cast<double>(imgVec[i].y);
            sumSq += dx * dx + dy * dy;
        }
        size_t denom = std::min(projected.size(), imgVec.size());
        result.reprojection_error_px = denom > 0 ? std::sqrt(sumSq / static_cast<double>(denom)) : 0.0;
    } else {
        result.reprojection_error_px = bestError >= 0.0 ? bestError : 0.0;
    }
    return result;
}

} // namespace va
