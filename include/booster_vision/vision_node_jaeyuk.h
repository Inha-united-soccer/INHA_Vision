#pragma once

#include <memory>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "vision_interface/msg/detections.hpp"

#include "vision_interface/msg/line_segments.hpp"
#include "vision_interface/msg/cal_param.hpp"
#include "vision_interface/msg/ball.hpp"

#include <yaml-cpp/yaml.h>

#include "booster_vision/base/intrin.h"
#include "booster_vision/base/pose.h"

#include "booster_vision/color_classifier.hpp"

// 승재욱 추가
#include "booster_interface/msg/low_state.hpp"
#include <mutex>
#include <Eigen/Core>

namespace booster_vision {

class DataLogger;
class DataSyncer;
class PoseEstimator;
class YoloV8Detector;
class YoloV8Segmentor;
class SyncedDataBlock;

class VisionNode : public rclcpp::Node {
public:
    VisionNode(const std::string &node_name);
    // 승재욱 변경 -> 소멸자 사용
    ~VisionNode();

    void Init(const std::string &cfg_template_path, const std::string &cfg_path);
    void ColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void DepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void PoseTFCallBack(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
    void PoseCallBack(const geometry_msgs::msg::Pose::SharedPtr msg);
    void CalParamCallback(const vision_interface::msg::CalParam::SharedPtr msg);
    void ProcessData(SyncedDataBlock &synced_data, vision_interface::msg::Detections &detections);
    void ProcessSegmentationData(SyncedDataBlock &synced_data, vision_interface::msg::LineSegments &field_line_segs_msg);
    // 승재욱 추가 
    void lowStateCallback(const booster_interface::msg::LowState &msg);
    void RunImageProcessingLoop(); // 이미지 받은 것 여기서 처리

private:
    bool use_depth_ = false;
    bool show_det_ = false;
    bool show_seg_ = false;
    bool save_data_ = false;
    bool save_depth_ = false;
    bool offline_mode_ = false;
    std::string detection_model_path;
    std::string segmentation_model_path;
    // 승재욱 추가
    bool use_imu_ = false;

    int save_cnt_ = 0;
    int save_every_n_frame_ = 0;

    std::string camera_type_;
    std::string img_log_path_;

    Intrinsics intr_;
    Pose p_eye2head_;
    Pose p_headprime2head_;
    Pose p_previous_head2base_;
    float z_compensation_ = 0;
    int line_segment_area_threshold_ = 10; // threshold for line segment detection

    // post processing
    bool enable_post_process_ = false;
    bool single_ball_assumption_ = false;
    std::vector<std::string> classnames_;

    std::map<std::string, float> confidence_map_;


    rclcpp::Publisher<vision_interface::msg::Detections>::SharedPtr detection_pub_;
    rclcpp::Publisher<vision_interface::msg::LineSegments>::SharedPtr field_line_pub_;
    rclcpp::Publisher<vision_interface::msg::Ball>::SharedPtr ball_pub_;


    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pose_tf_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr pose_tf_sub_;

    // 승재욱 추가 
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<vision_interface::msg::CalParam>::SharedPtr calParam_sub_; // Sub for calibration params

    rclcpp::CallbackGroup::SharedPtr callback_group_sub_1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub_2_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub_3_;
    // 승재욱 추가
    rclcpp::CallbackGroup::SharedPtr callback_group_sub_4_;

    std::shared_ptr<ColorClassifier> color_classifier_;

    std::shared_ptr<DataLogger> data_logger_;
    std::shared_ptr<DataSyncer> data_syncer_;
    std::shared_ptr<YoloV8Detector> detector_;
    std::shared_ptr<YoloV8Segmentor> segmentor_;
    std::shared_ptr<PoseEstimator> pose_estimator_;
    std::map<std::string, std::shared_ptr<PoseEstimator>> pose_estimator_map_;

    // 승재욱 추가
    mutable std::mutex imu_mutex_;
    Eigen::Vector3f imu_rpy_{0.f, 0.f, 0.f};
    Eigen::Vector3f imu_gyro_{0.f, 0.f, 0.f};
    Eigen::Vector3f imu_acc_{0.f, 0.f, 0.f};
    bool imu_ready_ = false;
    double imu_timestamp_ = 0.0;

    // 승재욱 추가 -> image qos 변경으로 인한 변수 추가
    std::mutex image_mutex_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_image_msg_;
    std::atomic<bool> new_image_received_{false};
    // 이미지 받은거 thread에서 따로 처리
    std::thread processing_thread_;
    std::atomic<bool> is_running_{false};
};

} // namespace booster_vision
