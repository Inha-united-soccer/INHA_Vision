#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <eigen3/Eigen/Dense>

#include "booster_vision/base/intrin.h"
#include "booster_vision/base/pose.h"
#include "booster_vision/base/data_syncer.hpp"
#include "booster_vision/base/misc_utils.hpp"
#include "booster_vision/img_bridge.h"
#include "booster_vision/calibration/optimizor.hpp"
#include "booster_vision/calibration/calibration.h"
#include "booster_vision/calibration/board_detector.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace booster_vision {

class StereoExtrinsicCalibrationNode : public rclcpp::Node {
public:
    StereoExtrinsicCalibrationNode(const std::string &node_name);
    ~StereoExtrinsicCalibrationNode() = default;

    void Init(const std::string &left_topic, const std::string &right_topic);
    void StereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr &right_msg);

private:
    void RunStereoCalibration(cv::Size img_size);

    int board_w_ = 0;
    int board_h_ = 0;
    float board_square_size_ = 0.05;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    std::vector<cv::Mat> LeftImages_, RightImages_;
    cv::Mat mtxL_, distL_, mtxR_, distR_;
    std::shared_ptr<BoardDetectorBase> board_detector_;
};


StereoExtrinsicCalibrationNode::StereoExtrinsicCalibrationNode(const std::string &node_name) 
    : Node(node_name) {
    this->declare_parameter<int>("board_w", 11);
    this->declare_parameter<int>("board_h", 8);
    this->declare_parameter<float>("board_square_size", 0.05);
}


void StereoExtrinsicCalibrationNode::Init(const std::string &left_topic, const std::string &right_topic) {
    // 알고 있는 파라미터 로드 (실제 값으로 대체 필요)
    mtxL_ = (cv::Mat_<double>(3,3) << 214.6289244900283, 0, 296.4174816336022, 0, 211.8074482264692, 237.831684090016, 0, 0, 1);
    distL_ = (cv::Mat_<double>(1,5) << -0.00521494, 0.000599017, 0, 0, 0);
    mtxR_ = (cv::Mat_<double>(3,3) << 220.1785466231719, 0, 296.7111514701778, 0, 217.5501888328994, 238.7777779171296, 0, 0, 1);
    distR_ = (cv::Mat_<double>(1,5) << 0.0110078, -0.000658914, 0, 0, 0);

    // Message Filters 연결
    left_sub_.subscribe(this, left_topic);
    right_sub_.subscribe(this, right_topic);
    
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), left_sub_, right_sub_);
    sync_->registerCallback(std::bind(&StereoExtrinsicCalibrationNode::StereoCallback, this, 
                            std::placeholders::_1, std::placeholders::_2));

    // 보드 디텍터 초기화
    this->get_parameter("board_w", board_w_);
    this->get_parameter("board_h", board_h_);
    this->get_parameter("board_square_size", board_square_size_);
    board_detector_ = std::make_shared<BoardDetectorBase>(cv::Size(board_w_, board_h_), board_square_size_);

    RCLCPP_INFO(this->get_logger(), "Stereo Calibration Node Initialized.");
}

// 스테레오 이미지 동기화 콜백
void StereoExtrinsicCalibrationNode::StereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &right_msg) {

    cv::Mat imgL = toCVMat(*left_msg);
    cv::Mat imgR = toCVMat(*right_msg);

    // 시각화 로직
    cv::Mat visL = imgL.clone();
    cv::Mat visR = imgR.clone();
    bool foundL = board_detector_->DetectBoard(imgL);
    cv::drawChessboardCorners(visL, cv::Size(board_w_, board_h_), board_detector_->getBoardUVs(), foundL);
    
    bool foundR = board_detector_->DetectBoard(imgR);
    cv::drawChessboardCorners(visR, cv::Size(board_w_, board_h_), board_detector_->getBoardUVs(), foundR);

    cv::Mat canvas;
    cv::hconcat(visL, visR, canvas);
    cv::imshow("Calibration View (Press 's' to save, 'c' to calculate)", canvas);

    char key = cv::waitKey(1);
    if (key == 's' && foundL && foundR) {
        LeftImages_.push_back(imgL);
        RightImages_.push_back(imgR);
        RCLCPP_INFO(this->get_logger(), "Stored stereo pair: %ld", LeftImages_.size());
    } else if (key == 'c') {
        RunStereoCalibration(imgL.size());
    }
}

// 캘리브레이션 실행 함수
void StereoExtrinsicCalibrationNode::RunStereoCalibration(cv::Size img_size) {
    if (LeftImages_.size() < 10) {
        RCLCPP_WARN(this->get_logger(), "Insufficient data: current %ld, recommended > 10", LeftImages_.size());
        return;
    }

    std::vector<std::vector<cv::Point3f>> objPoints;
    std::vector<std::vector<cv::Point2f>> imgPointsL, imgPointsR;

    for (size_t i = 0; i < LeftImages_.size(); ++i) {
        board_detector_->DetectBoard(LeftImages_[i]);
        objPoints.push_back(board_detector_->getBoardPoints());
        imgPointsL.push_back(board_detector_->getBoardUVsSubpixel());

        board_detector_->DetectBoard(RightImages_[i]);
        imgPointsR.push_back(board_detector_->getBoardUVsSubpixel());
    }

    cv::Mat R, T, E, F;
    int flags = cv::CALIB_FIX_INTRINSIC; 
    
    double rms = cv::stereoCalibrate(objPoints, imgPointsL, imgPointsR,
                                     mtxL_, distL_, mtxR_, distR_,
                                     img_size, R, T, E, F, flags);

    RCLCPP_INFO(this->get_logger(), "Stereo Calibration Complete! RMS: %f", rms);

    // Rectification 행렬 계산
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(mtxL_, distL_, mtxR_, distR_, img_size, R, T, 
                      R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, img_size);

    // --- YAML 파일 저장 시작 ---
    std::string filename = "stereo_extrinsic_calibration_result.yaml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    if (fs.isOpened()) {
        // 현재 시간 기록
        time_t rawtime;
        time(&rawtime);
        fs << "calibration_time" << asctime(localtime(&rawtime));

        // 1. 원본 파라미터 (Input)
        fs << "mtxL" << mtxL_;
        fs << "distL" << distL_;
        fs << "mtxR" << mtxR_;
        fs << "distR" << distR_;

        // 2. 외인성 파라미터 (Extrinsics)
        fs << "R" << R;
        fs << "T" << T;
        fs << "E" << E;
        fs << "F" << F;

        // 3. 정렬 및 투영 행렬 (Rectification)
        fs << "R1" << R1;
        fs << "R2" << R2;
        fs << "P1" << P1;
        fs << "P2" << P2;
        fs << "Q" << Q;

        fs.release();
        RCLCPP_INFO(this->get_logger(), "All parameters saved to: %s", filename.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s", filename.c_str());
    }

    // 터미널 확인용 출력
    std::cout << "\n[Results Saved Successfully]\n";
    std::cout << "Baseline (T.x): " << T.at<double>(0) << " meters\n";
    std::cout << "RMS Error: " << rms << "\n";
}

} // namespace booster_vision


const std::string kArguments =
    "{help h usage ? |      | print this message}"
    "{@left_topic    |      | left image topic name}"   // 첫 번째 인자
    "{@right_topic   |      | right image topic name}"; // 두 번째 인자

int main(int argc, char **argv) {

    cv::CommandLineParser parser(argc, argv, kArguments);
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    if (!parser.check()) {
        parser.printErrors();
        return -1;
    }

    std::string left_topic = parser.get<std::string>("@left_topic");
    std::string right_topic = parser.get<std::string>("@right_topic");

    if (left_topic.empty() || right_topic.empty()) {
        std::cerr << "Error: Both left_topic and right_topic are required!" << std::endl;
        std::cerr << "Usage: ros2 run <pkg> <node> /camera/left/image /camera/right/image" << std::endl;
        return -1;
    }

    rclcpp::init(argc, argv);

    // 3. 노드 생성 및 이름 변경 (Stereo로 명확하게)
    std::string node_name = "stereo_extrinsic_calibration_node";
    auto node = std::make_shared<booster_vision::StereoExtrinsicCalibrationNode>(node_name);

    // 4. 수정된 Init 호출 (두 개의 토픽 전달)
    node->Init(left_topic, right_topic);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}