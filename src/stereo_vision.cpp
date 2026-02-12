#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "booster_vision/img_bridge.h"  // toCVMat

namespace booster_vision {

cv::Mat disp32f_global;
cv::Mat Q_global;

class CalibrationUtils {
public:
    static bool LoadCalibrationYAML(const std::string &filename,
                                    cv::Mat &mtxL, cv::Mat &distL,
                                    cv::Mat &mtxR, cv::Mat &distR,
                                    cv::Mat &R1, cv::Mat &R2,
                                    cv::Mat &P1, cv::Mat &P2,
                                    cv::Mat &Q);

    static void ComputeRectifyMaps(const cv::Mat &mtxL, const cv::Mat &distL,
                                   const cv::Mat &mtxR, const cv::Mat &distR,
                                   const cv::Mat &R1, const cv::Mat &R2,
                                   const cv::Mat &P1, const cv::Mat &P2,
                                   const cv::Size &img_size,
                                   cv::Mat &map1L, cv::Mat &map2L,
                                   cv::Mat &map1R, cv::Mat &map2R);

    static void ApplyRectificationAndDrawLines(const cv::Mat &imgL, const cv::Mat &imgR,
                                               const cv::Mat &map1L, const cv::Mat &map2L,
                                               const cv::Mat &map1R, const cv::Mat &map2R,
                                               cv::Mat &canvas);
};

class StereoRectifyNode : public rclcpp::Node {
public:
    StereoRectifyNode(const std::string &node_name);

private:
    void StereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr &right_msg);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<Sync> sync_;

    // Calibration
    cv::Mat mtxL_, distL_, mtxR_, distR_;
    cv::Mat R1_, R2_, P1_, P2_, Q_;

    // Rectification maps
    cv::Mat map1L_, map2L_, map1R_, map2R_;
    bool rectify_maps_ready_;
};

bool booster_vision::CalibrationUtils::LoadCalibrationYAML(const std::string &filename,
                                                           cv::Mat &mtxL, cv::Mat &distL,
                                                           cv::Mat &mtxR, cv::Mat &distR,
                                                           cv::Mat &R1, cv::Mat &R2,
                                                           cv::Mat &P1, cv::Mat &P2,
                                                           cv::Mat &Q) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Cannot open YAML file: " << filename << std::endl;
        return false;
    }
    fs["mtxL"] >> mtxL;
    fs["distL"] >> distL;
    fs["mtxR"] >> mtxR;
    fs["distR"] >> distR;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;
    fs.release();
    return true;
}
void booster_vision::CalibrationUtils::ComputeRectifyMaps(const cv::Mat &mtxL, const cv::Mat &distL,
                                                          const cv::Mat &mtxR, const cv::Mat &distR,
                                                          const cv::Mat &R1, const cv::Mat &R2,
                                                          const cv::Mat &P1, const cv::Mat &P2,
                                                          const cv::Size &img_size,
                                                          cv::Mat &map1L, cv::Mat &map2L,
                                                          cv::Mat &map1R, cv::Mat &map2R) {
    cv::initUndistortRectifyMap(mtxL, distL, R1, P1, img_size, CV_32FC1, map1L, map2L);
    cv::initUndistortRectifyMap(mtxR, distR, R2, P2, img_size, CV_32FC1, map1R, map2R);
}

void booster_vision::CalibrationUtils::ApplyRectificationAndDrawLines(
    const cv::Mat &imgL, const cv::Mat &imgR,
    const cv::Mat &map1L, const cv::Mat &map2L,
    const cv::Mat &map1R, const cv::Mat &map2R,
    cv::Mat &canvas) 
{
    // 1. Rectify
    cv::Mat rectL, rectR;
    cv::remap(imgL, rectL, map1L, map2L, cv::INTER_LINEAR);
    cv::remap(imgR, rectR, map1R, map2R, cv::INTER_LINEAR);

    // 2. 체커보드 코너 찾기
    // cv::Size board_size(8, 6); // 체커보드 행·열 수
    // std::vector<cv::Point2f> cornersL, cornersR;
    // bool foundL = cv::findChessboardCorners(rectL, board_size, cornersL,
    //                                         cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    // bool foundR = cv::findChessboardCorners(rectR, board_size, cornersR,
    //                                         cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    // 좌우 연결 canvas
    // cv::hconcat(rectL, rectR, canvas);

    // if (foundL) {
    //     // 체커보드 좌측 코너 그리기
    //     cv::drawChessboardCorners(canvas(cv::Rect(0,0,rectL.cols,rectL.rows)), board_size, cornersL, foundL);
    // }
    // if (foundR) {
    //     // 체커보드 우측 코너 그리기
    //     cv::drawChessboardCorners(canvas(cv::Rect(rectL.cols,0,rectR.cols,rectR.rows)), board_size, cornersR, foundR);
    // }

    // 3. Row별 수평 가이드라인 (4색 반복)
    // cv::Scalar colors[4] = { cv::Scalar(0,0,255), cv::Scalar(0,165,255),
    //                          cv::Scalar(0,255,0), cv::Scalar(255,0,0) }; // 빨강, 주황, 초록, 파랑

    // if (foundL) {
    //     for (int row = 0; row < board_size.height; ++row) {
    //         int color_idx = row % 4;
    //         float y = 0;
    //         // row별 평균 y값 계산
    //         int count = 0;
    //         for (int col = 0; col < board_size.width; ++col) {
    //             int idx = row * board_size.width + col;
    //             y += cornersL[idx].y;
    //             ++count;
    //         }
    //         y /= count;
    //         // 수평선 그리기
    //         cv::line(canvas, cv::Point(0, static_cast<int>(y)), 
    //                           cv::Point(canvas.cols, static_cast<int>(y)), colors[color_idx], 1, cv::LINE_AA);
    //     }
    // }

    // // 4. 좌우 대응점 연결선 그리기
    // if (foundL && foundR) {
    //     for (size_t i = 0; i < cornersL.size(); ++i) {
    //         cv::Point2f ptL = cornersL[i];
    //         cv::Point2f ptR = cornersR[i];
    //         ptR.x += rectL.cols; // 우측 이미지는 canvas에서 오른쪽에 붙어 있으므로 offset
    //         cv::line(canvas, ptL, ptR, cv::Scalar(255,255,0), 1, cv::LINE_AA); // 청록색
    //     }
    // }

    // gray로 변경
    cv::Mat grayL, grayR;
    if (rectL.channels() == 3) {
        cv::cvtColor(rectL, grayL, cv::COLOR_BGR2GRAY);
    } else {
        grayL = rectL;
    }
    if (rectR.channels() == 3) {
        cv::cvtColor(rectR, grayR, cv::COLOR_BGR2GRAY);
    } else {
        grayR = rectR;
    }

    // StereoSGBM 생성
    int minDisparity = 0;
    int numDisparities = 16 * 6;  // 반드시 16의 배수
    int blockSize = 7;

    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
        minDisparity,
        numDisparities,
        blockSize
    );

    stereo->setP1(8 * grayL.channels() * blockSize * blockSize);
    stereo->setP2(32 * grayL.channels() * blockSize * blockSize);
    stereo->setMode(cv::StereoSGBM::MODE_SGBM);

    cv::Mat disparity;
    stereo->compute(grayL, grayR, disparity);
    disparity.convertTo(disp32f_global, CV_32F, 1.0/16.0);

    // Disparity 시각화
    // disparity → float 변환 (16으로 나눔)
    cv::Mat disparity_float;
    disparity.convertTo(disparity_float, CV_32F, 1.0 / 16.0);

    // 음수 제거 (optional but 추천)
    cv::Mat disparity_positive = disparity_float.clone();
    disparity_positive.setTo(0, disparity_positive < 0);

    // Normalize to 0~255
    cv::Mat disparity_visual;
    cv::normalize(disparity_positive, disparity_visual, 0, 255, cv::NORM_MINMAX, CV_8U);

    // 컬러맵 적용
    cv::applyColorMap(disparity_visual, disparity_visual, cv::COLORMAP_JET);

    // 출력
    cv::imshow("Disparity", disparity_visual);
    
    cv::setMouseCallback("Disparity", [](int event, int x, int y, int, void*) {
        if (event == cv::EVENT_LBUTTONDOWN) {

            float d = disp32f_global.at<float>(y, x);

            if (d <= 0.0f) {
                std::cout << "Invalid disparity at (" << x << "," << y << ")\n";
                return;
            }

            // Q matrix 사용
            double Q03 = Q_global.at<double>(0,3);
            double Q13 = Q_global.at<double>(1,3);
            double Q23 = Q_global.at<double>(2,3);
            double Q32 = Q_global.at<double>(3,2);
            double Q33 = Q_global.at<double>(3,3);

            double W = Q32 * d + Q33;
            double Z = Q23 / W;   // depth (meter)

            double depth_cm = Z * 100.0;

            std::cout << "Clicked at (" << x << "," << y << ") "
                    << "disparity = " << d
                    << " , depth = " << depth_cm << " cm"
                    << std::endl;
        }
    });
    cv::waitKey(1);
}


// ========================================
// StereoRectifyNode 구현
// ========================================
booster_vision::StereoRectifyNode::StereoRectifyNode(const std::string &node_name)
    : Node(node_name), rectify_maps_ready_(false) {

    this->declare_parameter<std::string>("yaml_file", "stereo_extrinsic_calibration_result.yaml");
    this->declare_parameter<std::string>("left_topic", "/camera/left/image_raw");
    this->declare_parameter<std::string>("right_topic", "/camera/right/image_raw");

    std::string yaml_file, left_topic, right_topic;
    this->get_parameter("yaml_file", yaml_file);
    this->get_parameter("left_topic", left_topic);
    this->get_parameter("right_topic", right_topic);

    // YAML 읽기
    if (!booster_vision::CalibrationUtils::LoadCalibrationYAML(yaml_file,
                                                               mtxL_, distL_, mtxR_, distR_,
                                                               R1_, R2_, P1_, P2_, Q_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load calibration YAML");
        return;
    }

    // rectification map 계산
    cv::Size img_size = cv::Size(544, 448); // 실제 이미지 크기
    booster_vision::CalibrationUtils::ComputeRectifyMaps(mtxL_, distL_, mtxR_, distR_,
                                                         R1_, R2_, P1_, P2_, img_size,
                                                         map1L_, map2L_, map1R_, map2R_);
    rectify_maps_ready_ = true;
    
    Q_global = Q_.clone();
    
    // ROS2 subscriber
    left_sub_.subscribe(this, left_topic);
    right_sub_.subscribe(this, right_topic);
    sync_ = std::make_shared<Sync>(SyncPolicy(10), left_sub_, right_sub_);
    sync_->registerCallback(std::bind(&StereoRectifyNode::StereoCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Stereo Rectification Node Initialized.");
}

void booster_vision::StereoRectifyNode::StereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &right_msg) {

    if (!rectify_maps_ready_) return;

    cv::Mat imgL = toCVMat(*left_msg);
    cv::Mat imgR = toCVMat(*right_msg);

    cv::Mat canvas;
    booster_vision::CalibrationUtils::ApplyRectificationAndDrawLines(imgL, imgR,
                                                                     map1L_, map2L_,
                                                                     map1R_, map2R_,
                                                                     canvas);
    //cv::imshow("Rectified Stereo", canvas);
    cv::waitKey(1);
}


} // namespace booster_vision

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<booster_vision::StereoRectifyNode>("stereo_rectify_node");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}