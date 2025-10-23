#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace va {

inline std::array<cv::Point2f, 4> order_corners(const std::vector<cv::Point2f>& pts) {
    std::array<cv::Point2f, 4> ordered{};
    if (pts.size() < 4) {
        return ordered;
    }
    std::array<int, 4> indices = {0, 1, 2, 3};
    float minSum = std::numeric_limits<float>::max();
    float maxSum = -std::numeric_limits<float>::max();
    float minDiff = std::numeric_limits<float>::max();
    float maxDiff = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < 4; ++i) {
        const auto& p = pts[i];
        float sum = p.x + p.y;
        float diff = p.x - p.y;
        if (sum < minSum) {
            minSum = sum;
            indices[0] = static_cast<int>(i); // top-left
        }
        if (sum > maxSum) {
            maxSum = sum;
            indices[2] = static_cast<int>(i); // bottom-right
        }
        if (diff < minDiff) {
            minDiff = diff;
            indices[1] = static_cast<int>(i); // top-right
        }
        if (diff > maxDiff) {
            maxDiff = diff;
            indices[3] = static_cast<int>(i); // bottom-left
        }
    }
    // Ensure all indices unique; otherwise fall back to the first four points.
    std::array<int, 4> check = indices;
    std::sort(check.begin(), check.end());
    if (std::unique(check.begin(), check.end()) != check.end()) {
        for (size_t i = 0; i < 4; ++i) {
            ordered[i] = pts[i];
        }
    } else {
        for (size_t i = 0; i < 4; ++i) {
            ordered[i] = pts[indices[i]];
        }
    }
    // Enforce counter-clockwise winding (top-left -> top-right -> bottom-right -> bottom-left).
    const cv::Point2f& tl = ordered[0];
    const cv::Point2f& tr = ordered[1];
    const cv::Point2f& br = ordered[2];
    float cross = (tr.x - tl.x) * (br.y - tl.y) - (tr.y - tl.y) * (br.x - tl.x);
    if (cross < 0.0f) {
        std::swap(ordered[1], ordered[3]);
    }
    return ordered;
}

inline double polygon_area(const std::array<cv::Point2f, 4>& pts) {
    double area = 0.0;
    for (int i = 0; i < 4; ++i) {
        const cv::Point2f& p0 = pts[i];
        const cv::Point2f& p1 = pts[(i + 1) % 4];
        area += static_cast<double>(p0.x) * p1.y - static_cast<double>(p1.x) * p0.y;
    }
    return std::abs(area) * 0.5;
}

inline double homography_condition(const std::array<cv::Point2f, 4>& imgPts) {
    std::vector<cv::Point2f> objPts = {
            {-0.5f, -0.5f},
            { 0.5f, -0.5f},
            { 0.5f,  0.5f},
            {-0.5f,  0.5f}
    };
    std::vector<cv::Point2f> imgVec(imgPts.begin(), imgPts.end());
    cv::Mat H = cv::getPerspectiveTransform(objPts, imgVec);
    cv::Mat A = H(cv::Rect(0, 0, 3, 3)).clone();
    cv::SVD svd(A, cv::SVD::NO_UV);
    if (svd.w.rows < 3 || std::abs(svd.w.at<double>(2)) < 1e-9) {
        return std::numeric_limits<double>::infinity();
    }
    double s0 = svd.w.at<double>(0);
    double s2 = svd.w.at<double>(2);
    if (s2 == 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return std::abs(s0 / s2);
}

inline cv::Matx44f mat_from_column_major(const float* data, size_t len) {
    if (!data || len < 16) {
        return cv::Matx44f::eye();
    }
    cv::Matx44f M;
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            M(row, col) = data[col * 4 + row];
        }
    }
    return M;
}

inline std::array<float, 16> mat_to_column_major(const cv::Matx44f& M) {
    std::array<float, 16> out{};
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            out[col * 4 + row] = M(row, col);
        }
    }
    return out;
}

inline cv::Matx44f multiply(const cv::Matx44f& A, const cv::Matx44f& B) {
    cv::Matx44f R = A * B;
    return R;
}

inline cv::Matx44f make_transform(const cv::Matx33d& R, const cv::Vec3d& t) {
    cv::Matx44f T = cv::Matx44f::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            T(r, c) = static_cast<float>(R(r, c));
        }
        T(r, 3) = static_cast<float>(t[r]);
    }
    return T;
}

inline cv::Matx33d mat3_from_column_major(const cv::Matx44f& M) {
    cv::Matx33d R;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            R(r, c) = M(r, c);
        }
    }
    return R;
}

inline cv::Vec3d translation_from(const cv::Matx44f& M) {
    return cv::Vec3d(M(0, 3), M(1, 3), M(2, 3));
}

inline cv::Matx44f invert_rigid(const cv::Matx44f& T) {
    cv::Matx33d R = mat3_from_column_major(T);
    cv::Vec3d t = translation_from(T);
    cv::Matx33d Rt = R.t();
    cv::Vec3d t_inv = -Rt * t;
    cv::Matx44f inv = cv::Matx44f::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            inv(r, c) = static_cast<float>(Rt(r, c));
        }
        inv(r, 3) = static_cast<float>(t_inv[r]);
    }
    return inv;
}

} // namespace va
