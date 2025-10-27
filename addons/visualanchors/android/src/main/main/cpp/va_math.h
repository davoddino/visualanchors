#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace va {

inline std::array<cv::Point2f, 4> order_corners_by_sumdiff(const std::vector<cv::Point2f>& pts) {
    std::array<cv::Point2f, 4> ordered{};
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
            indices[3] = static_cast<int>(i); // bottom-left
        }
        if (diff > maxDiff) {
            maxDiff = diff;
            indices[1] = static_cast<int>(i); // top-right
        }
    }
    for (size_t i = 0; i < 4; ++i) {
        ordered[i] = pts[indices[i]];
    }
    const cv::Point2f& tl = ordered[0];
    const cv::Point2f& tr = ordered[1];
    const cv::Point2f& br = ordered[2];
    float cross = (tr.x - tl.x) * (br.y - tl.y) - (tr.y - tl.y) * (br.x - tl.x);
    if (cross < 0.0f) {
        std::swap(ordered[2], ordered[3]);
    }
    return ordered;
}

inline bool nearly_equal(const cv::Point2f& a, const cv::Point2f& b, float eps = 1e-4f) {
    return std::abs(a.x - b.x) < eps && std::abs(a.y - b.y) < eps;
}

inline std::array<cv::Point2f, 4> order_corners(const std::vector<cv::Point2f>& pts) {
    std::array<cv::Point2f, 4> ordered{};
    if (pts.size() < 4) {
        return ordered;
    }
    std::vector<cv::Point2f> sorted(pts.begin(), pts.begin() + 4);
    std::sort(sorted.begin(), sorted.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        if (std::abs(a.y - b.y) < 1e-3f) {
            return a.x < b.x;
        }
        return a.y < b.y;
    });
    cv::Point2f tl, tr, br, bl;
    const cv::Point2f& top0 = sorted[0];
    const cv::Point2f& top1 = sorted[1];
    if (top0.x <= top1.x) {
        tl = top0;
        tr = top1;
    } else {
        tl = top1;
        tr = top0;
    }
    const cv::Point2f& bottom0 = sorted[2];
    const cv::Point2f& bottom1 = sorted[3];
    if (bottom0.x <= bottom1.x) {
        bl = bottom0;
        br = bottom1;
    } else {
        bl = bottom1;
        br = bottom0;
    }
    ordered[0] = tl;
    ordered[1] = tr;
    ordered[2] = br;
    ordered[3] = bl;
    auto duplicates = [&](const cv::Point2f& p, const cv::Point2f& q) {
        return nearly_equal(p, q);
    };
    if (duplicates(ordered[0], ordered[1]) ||
        duplicates(ordered[0], ordered[2]) ||
        duplicates(ordered[0], ordered[3]) ||
        duplicates(ordered[1], ordered[2]) ||
        duplicates(ordered[1], ordered[3]) ||
        duplicates(ordered[2], ordered[3])) {
        return order_corners_by_sumdiff(pts);
    }
    const cv::Point2f& tlRef = ordered[0];
    const cv::Point2f& trRef = ordered[1];
    const cv::Point2f& brRef = ordered[2];
    float cross = (trRef.x - tlRef.x) * (brRef.y - tlRef.y) - (trRef.y - tlRef.y) * (brRef.x - tlRef.x);
    if (cross < 0.0f) {
        std::swap(ordered[2], ordered[3]);
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
