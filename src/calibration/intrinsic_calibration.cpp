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
#include "booster_vision/calibration/optimizer.hpp"
#include "booster_vision/calibration/calibration.h"
#include "booster_vision/calibration/board_detector.h"

namespace booster_vision {
    
// calibratiion node
class IntrinsicCalibrationNode : public rclcpp::Node {
public:
    IntrinsicCalibrationNode(const std::string &node_name);
    ~IntrinsicCalibrationNode() = default;

    void Init(bool offline_mode = false);
    void ColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void RunIntrinsicCalibrationProcess(const SyncedDataBlock &data_block);

private:
    bool is_offline_ = false;

    int board_w_ = 0;
    int board_h_ = 0;
    float board_square_size_ = 0.05;
    std::string camera_type_ = "";
    int sync_time_diff_ms_ = 1500; // ms

    YAML::Node cfg_node_;

    std::shared_ptr<rclcpp::Node> nh_;
    SyncedDataBlock data_block_;

    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber color_sub_;


    std::shared_ptr<DataSyncer> data_syncer_;
    std::vector<SyncedDataBlock> cali_data_;

    std::shared_ptr<IntrinsicBoardDetector> board_detector_;
    std::vector<std::vector<MarkerCoordinates>> marker_coords_vec_;

    // for display
    cv::Mat board_position_mask_;
};

// 체커 보드 정의
IntrinsicCalibrationNode::IntrinsicCalibrationNode(const std::string &node_name) :
    rclcpp::Node(node_name) {
    this->declare_parameter<int>("board_w", 11);
    this->declare_parameter<int>("board_h", 8);
    this->declare_parameter<float>("board_square_size", 0.05);
}

void IntrinsicCalibrationNode::Init(bool is_offline, std::string color_topic) {

    rclcpp::CallbackGroup::SharedPtr callback_group_sub_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_1 = rclcpp::SubscriptionOptions();
    sub_opt_1.callback_group = callback_group_sub_1;

    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    color_sub_ = it_->subscribe(color_topic, 2, &IntrinsicCalibrationNode::ColorCallback, this, nullptr, sub_opt_1);
    is_offline_ = is_offline;

    this->get_parameter("board_w", board_w_);
    this->get_parameter("board_h", board_h_);
    this->get_parameter("board_square_size", board_square_size_);

    board_detector_ = std::make_shared<IntrinsicBoardDetector>(cv::Size(board_w_, board_h_), board_square_size_);
    data_syncer_ = std::make_shared<DataSyncer>(false);
}

void IntrinsicCalibrationNode::RunIntrinsicCalibrationProcess(const SyncedDataBlock &data_block) {

    auto img = data_block.color_data.data;
    double timestamp = data_block.color_data.timestamp;

    // 체커보드 검출
    bool found = board_detector_->DetectBoard(img);
    std::vector<cv::Point2f> corners = board_detector_->getBoardUVs(); // 이미지 픽셀 코너 좌표

    // 체커보드 이동 히스토리 시각화 (과거에 찍었던 곳 표시)
    double alpha = 0.25;
    cv::Mat display_img = img.clone();
    if (board_position_mask_.empty()) { // 마스크 초기화
        board_position_mask_ = cv::Mat::zeros(img.size(), img.type());
    }
    // alpha * (과거 보드 위치들) + (1 - alpha) * (현재 이미지)
    cv::addWeighted(board_position_mask_, alpha, display_img, 1 - alpha, 0, display_img);

    // 현재 프레임의 체커보드 표시
    cv::drawChessboardCorners(display_img, cv::Size(board_w_, board_h_), corners, found);
    std::string progress_status_text = std::to_string(cali_data_.size()) + "/8 frames collected";
    cv::putText(display_img, progress_status_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    cv::putText(display_img, "press h for help info in terminal", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);

    // 유효 촬영 영역 표시
    // 화면 가장자리 15%씩 제외 + 가운데 70% 영역만 유효
    cv::Rect valid_area(img.cols * 0.15, img.rows * 0.15, img.cols * 0.7, img.rows * 0.7);
    cv::rectangle(display_img, valid_area, cv::Scalar(0, 0, 255), 2);
    cv::imshow("chessboard", display_img);

    int wait_time = is_offline_ ? 0 : 10;
    const char key = cv::waitKey(wait_time); // 10ms 동안만 키 입력을 기다리고 바로 다음 프레임 진행
    switch (key) {
    case 's': {
        std::cout << "select current snap short for calibration!" << std::endl;
        // TODO(GW): order check
        if (found) {
            // update board mask
            cv::Mat mask = board_detector_->getBoardMask(img);
            cv::putText(mask, std::to_string(cali_data_.size()), cv::Point(corners[0].x, corners[0].y + 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
            cv::bitwise_or(mask, board_position_mask_, board_position_mask_); // 이전 마스크와 합성

            cali_data_.emplace_back(data_block); // 실제 캘리브레이션 데이터 저장
        }
        break;
    }
    case 'c': {
        // calibrate
        std::cout << "start calibration computation!" << std::endl;
        // prepare data for calibration
        std::vector<std::vector<cv::Point3f>> all_corners_3d_; // 체커보드 코너의 3D 좌표 (보드 좌표계)
        std::vector<std::vector<cv::Point2f>> all_corners_2d_; // 이미지에서 검출된 2D 좌표 (픽셀 좌표)

        for (int i = 0; i < cali_data_.size(); i++) {
            cv::Mat img = cali_data_[i].color_data.data.clone();

            if (board_detector_->DetectBoard(img)) {
                all_corners_3d_.push_back(board_detector_->getBoardPoints()); // 보드 좌표계 3D 좌표
                all_corners_2d_.push_back(board_detector_->getBoardUVsSubpixel()); // 이미지 픽셀 2D 좌표 (서브픽셀 정밀도)

                auto corner_3d = all_corners_3d_.back();
                auto corner_2d = all_corners_2d_.back();
                
                std::cout << "number " << i << " th board detected!" << std::endl;
            }
        }
        // 이미지 사이즈 
        cv::Size image_size(cali_data_[0].color_data.data.cols, cali_data_[0].color_data.data.rows);
        // 캘리브레이션 수행
        IntrinsicCalibration(all_corners_3d_, all_corners_2d_, image_size);

        std::cout << "finish intrinsic calibration process" << std::endl;
        std::cout << "auto exit after calibration" << std::endl;
        rclcpp::shutdown();
        exit(0);
        break;
    }
    case 'r': {
        std::cout << "rest intrinsic calibration process!!!" << std::endl;
        cali_data_.clear();
        board_position_mask_ = cv::Mat();
        break;
    }
    case 'q': {
        // exit
        std::cout << "exit intrinsic calibration process" << std::endl;
        rclcpp::shutdown();
        exit(0);
        break;
    }
    case 'h': {
        std::cout << std::endl
                  << "operation key binding:" << std::endl
                  << "s: save data for calibration" << std::endl
                  << "c: start calibration if data number exceeds 8" << std::endl
                  << "r: restart calibration process" << std::endl
                  << "q: exit" << std::endl;
        break;
    }
    default: {
        break;
    }
    }
}

void IntrinsicCalibrationNode::ColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {

    if (!msg) {
        std::cerr << "empty image message." << std::endl;
        return;
    }

    cv::Mat img;
    try {
        img = toCVMat(*msg);
    } catch (std::exception &e) {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        return;
    }

    double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    auto data_block = data_syncer_->getSyncedDataBlock(ColorDataBlock(img, timestamp));
    RunIntrinsicCalibrationProcess(data_block);
}

}// namespace booster_vision

const std::string kArguments =
    "{help h usage ? |      | print this message}"
    "{@color_topic   |      | color image topic name}";

int main(int argc, char **argv){
    cv::CommandLineParser parser(argc, argv, kArguments);
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    if (!parser.check()) {
        parser.printErrors();
        return -1;
    }
    std::string color_topic = parser.get<std::string>("@color_topic");
    if (color_topic.empty()) {
        std::cerr << "color_topic is required!" << std::endl;
        return -1;
    }

    rclcpp::init(argc, argv);

    std::string node_name = "intrinsic_calibration_node";
    auto node = std::make_shared<booster_vision::IntrinsicCalibrationNode>(node_name);

    node->Init(false, color_topic);
    std::cout << "calibration node initialized" << std::endl;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    return 0;
}