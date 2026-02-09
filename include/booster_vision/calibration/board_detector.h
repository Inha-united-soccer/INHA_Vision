#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "booster_vision/base/intrin.h"
#include "booster_vision/base/pose.h"

namespace booster_vision {

class BoardDetectorBase {
public:
    BoardDetectorBase(const cv::Size &board_size, const float square_size);

    std::vector<cv::Point3f> getBoardPoints() { return board_points_; }
    std::vector<cv::Point2f> getBoardUVs() { return board_uvs_; }
    std::vector<cv::Point2f> getBoardUVsSubpixel();
    cv::Mat getBoardMask(const cv::Mat &img);
    bool DetectBoard(const cv::Mat &img);

protected:
    cv::Mat gray_;
    cv::Size board_size_;
    float square_size_;
    std::vector<cv::Point3f> board_points_;
    std::vector<cv::Point2f> board_uvs_;
};

class BoardDetector : public BoardDetectorBase {
public:
    BoardDetector(const cv::Size &board_size, const float square_size, const Intrinsics intr);
    Pose getBoardPose();

private:
    Intrinsics intr_;
};

class IntrinsicBoardDetector : public BoardDetectorBase {
public:
    IntrinsicBoardDetector(const cv::Size &board_size, const float square_size);
};
} // namespace booster_vision