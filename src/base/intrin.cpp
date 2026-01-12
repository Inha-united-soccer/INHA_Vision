#include "booster_vision/base/intrin.h"

#include <stdexcept>

namespace booster_vision {

// TODO(SS): check input distortion coeffs length

Intrinsics::Intrinsics(const YAML::Node &node) {
    if (!node) {
        throw std::runtime_error("Intrinsics: Invalid YAML node");
    }
    // 초점거리
    fx = node["fx"].as<float>();
    fy = node["fy"].as<float>();
    // 주점
    cx = node["cx"].as<float>();
    cy = node["cy"].as<float>();
    // TODO(SS): fix model type conversion
    model = static_cast<DistortionModel>(node["distortion_model"].as<int>()); // 현재 0 (distortion 없음)
    distortion_coeffs = node["distortion_coeffs"].as<std::vector<float>>(); // 현재 없음
}

Intrinsics::Intrinsics(const cv::Mat intr, const std::vector<float> &distortion_coeffs_value, const DistortionModel &model_value) {
    if (intr.rows != 3 || intr.cols != 3) {
        throw std::runtime_error("Intrinsics: Invalid intrinsics matrix");
    }
    fx = intr.at<float>(0, 0);
    fy = intr.at<float>(1, 1);
    cx = intr.at<float>(0, 2);
    cy = intr.at<float>(1, 2);
    distortion_coeffs = distortion_coeffs_value;
    model = model_value;
}

Intrinsics::Intrinsics(float fx_value, float fy_value, float cx_value, float cy_value, const std::vector<float> &distortion_coeffs_value, DistortionModel model_value) {
    fx = fx_value;
    fy = fy_value;
    cx = cx_value;
    cy = cy_value;
    distortion_coeffs = distortion_coeffs_value;
    model = model_value;
}

// 3D 카메라 좌표계 점 (X,Y,Z)을 카메라 내부 파라미터 + 왜곡 모델을 이용해 2D 이미지 픽셀 좌표 (u,v)로 투영(project) 하는 함수
cv::Point2f Intrinsics::Project(const cv::Point3f &point) const {
    cv::Point2f uv;
    switch (model) {
    case DistortionModel::kNone:
    case DistortionModel::kInverseBrownConrady: {
        uv.x = fx * point.x / point.z + cx;
        uv.y = fy * point.y / point.z + cy;
        break;
    }
    case DistortionModel::kBrownConrady: {
        float x = point.x / point.z;
        float y = point.y / point.z;
        float r2 = x * x + y * y;
        float r4 = r2 * r2;
        float r6 = r4 * r2;
        const float* d = distortion_coeffs.data();

        float x_prime = x * (1 + d[0] * r2 + d[1] * r4 + d[4] * r6) + 2 * d[2] * x * y + d[3] * (r2 + 2 * x * x);
        float y_prime = y * (1 + d[0] * r2 + d[1] * r4 + d[4] * r6) + d[2] * (r2 + 2 * y * y) + 2 * d[3] * x * y;

        uv.x = fx * x_prime + cx;
        uv.y = fy * y_prime + cy;
    }
    }
    return uv;
}

// 픽셀 좌표 (u,v) + 깊이(depth)를 받아서 카메라 좌표계의 3D 점 (X,Y,Z)으로 되돌리는 정석적인 Back-Projection(역투영) 구현
cv::Point3f Intrinsics::BackProject(const cv::Point2f &point, float depth) const {
    cv::Point3f xyz;
    switch (model) {
    // 카메라 원점에서 앞(z)으로 1만큼 간 평면 위의 점을 만들고, 그 방향으로 쭉 나가는 광선을 생각.
    case DistortionModel::kNone: { // depth 기본으로 1 
        xyz.x = (point.x - cx) * depth / fx;
        xyz.y = (point.y - cy) * depth / fy;
        xyz.z = depth;
        break;
    }
    case DistortionModel::kBrownConrady: {
        std::vector<cv::Point2f> distorted_points = {point};
        std::vector<cv::Point2f> undistort_points;
        cv::undistortPoints(distorted_points, undistort_points, get_intrinsics_matrix(), distortion_coeffs);
        xyz = cv::Point3f(undistort_points[0].x, undistort_points[0].y, 1) * depth;
        break;
    }
    case DistortionModel::kInverseBrownConrady: {
        float x = (point.x - cx) / fx;
        float y = (point.y - cy) / fy;

        float xo = x;
        float yo = y;

        // need to loop until convergence
        // 10 iterations determined empirically
        for (int i = 0; i < 10; i++) {
            float r2 = x * x + y * y;
            float icdist = (float)1 / (float)(1 + ((distortion_coeffs[4] * r2 + distortion_coeffs[1]) * r2 + distortion_coeffs[0]) * r2);
            float xq = x / icdist;
            float yq = y / icdist;
            float delta_x = 2 * distortion_coeffs[2] * xq * yq + distortion_coeffs[3] * (r2 + 2 * xq * xq);
            float delta_y = 2 * distortion_coeffs[3] * xq * yq + distortion_coeffs[2] * (r2 + 2 * yq * yq);
            x = (xo - delta_x) * icdist;
            y = (yo - delta_y) * icdist;
        }
        xyz = cv::Point3f(x, y, 1) * depth;
        break;
    }
    }
    return xyz;
}

cv::Point2f Intrinsics::UnDistort(const cv::Point2f &point) const {
    cv::Point2f undistorted_point;
    switch (model) {
    case DistortionModel::kBrownConrady: // TODO(SS): fix this later
    case DistortionModel::kInverseBrownConrady:
        undistorted_point = Project(BackProject(point));
    case DistortionModel::kNone:
    default:
        undistorted_point = point;
    }
    return undistorted_point;
}

} // namespace booster_vision