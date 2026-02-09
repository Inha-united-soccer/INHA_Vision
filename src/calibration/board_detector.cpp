#include "booster_vision/calibration/board_detector.h"

#include <opencv2/opencv.hpp>

namespace booster_vision {

BoardDetectorBase::BoardDetectorBase(const cv::Size &board_size, const float square_size) :
    board_size_(board_size), square_size_(square_size) {
    // 보드 좌표계 3D 좌표 생성
    for (int i = 0; i < board_size_.height; i++) {
        for (int j = 0; j < board_size_.width; j++) {
            board_points_.push_back(cv::Point3f(j * square_size_, i * square_size_, 0));
        }
    }
}

// 체커보드 코너의 2D 픽셀 좌표를 서브픽셀 정밀도로 미세 보정
std::vector<cv::Point2f> BoardDetectorBase::getBoardUVsSubpixel() {
    if (board_uvs_.empty()) {
        return {};
    }
    // 코너 위치 중심으로 작은 윈도우를 잡고 밝기 기울기(gradient)를 이용해 진짜 코너가 있는 위치를 연속 좌표로 최적화 
    // 픽셀 정수 -> 실수 좌표
    // cv::Size(11,11): 탐색 윈도우 크기, cv::Size(-1,-1): dead zone (사용 안함)
    cv::cornerSubPix(gray_, board_uvs_, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    return board_uvs_;
}

cv::Mat BoardDetectorBase::getBoardMask(const cv::Mat &img) {
    cv::Mat mask = cv::Mat::zeros(img.size(), img.type());
    if (board_uvs_.empty() || board_uvs_.size() != board_size_.width * board_size_.height) {
        return mask;
    }
    std::vector<cv::Point> board_corners;
    board_corners.push_back(board_uvs_[0]); // 좌상단
    board_corners.push_back(board_uvs_[board_size_.width - 1]); // 우상단
    board_corners.push_back(board_uvs_[board_size_.width * board_size_.height - 1]); // 우하단
    board_corners.push_back(board_uvs_[board_size_.width * (board_size_.height - 1)]); // 좌하단
    std::vector<std::vector<cv::Point>> contour;
    contour.push_back(board_corners);

    cv::fillPoly(mask, contour, cv::Scalar(0, 255, 0));
    return mask;
}

bool BoardDetectorBase::DetectBoard(const cv::Mat &img) {
    board_uvs_.clear();
    cv::cvtColor(img, gray_, cv::COLOR_BGR2GRAY);
    // bool found = cv::findChessboardCorners(gray_, board_size_, board_uvs_, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
    // 체커보드 내부 코너 검출 -> output: 이미지 픽셀 코너 좌표
    bool found = cv::findChessboardCorners(gray_, board_size_, board_uvs_, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    return found;
}

BoardDetector::BoardDetector(const cv::Size &board_size, const float square_size, const Intrinsics intr) :
    BoardDetectorBase(board_size, square_size), intr_(intr) {}

Pose BoardDetector::getBoardPose() {
    cv::Mat rvec;
    cv::Mat tvec;
    // solvePnP는 R, t를 구함
    // u ~ ㅠ(K(RX + t)) 이고 
    // X는 보드 좌표계 3D 좌표, u는 이미지 픽셀 2D 좌표, K는 카메라 내부 파라미터 행렬, R과 t는 보드 좌표계 -> 카메라 좌표계 변환 행렬
    bool res = cv::solvePnP(board_points_, board_uvs_, intr_.get_intrinsics_matrix(), intr_.distortion_coeffs, rvec, tvec);
    if (!res) {
        return Pose();
    }
    return Pose(rvec, tvec);
}

IntrinsicBoardDetector::IntrinsicBoardDetector(const cv::Size &board_size, const float square_size) :
    BoardDetectorBase(board_size, square_size) {}
    
} // namespace booster_vision