#ifndef RM_VISION_CPP__DETECTOR_NODE_HPP_
#define RM_VISION_CPP__DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>

#include <rm_interfaces/msg/target.hpp>
#include "rm_vision_cpp/trt_yolo.hpp"

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <cstddef>

namespace rm_vision_cpp
{

class DetectorNode : public rclcpp::Node
{
public:
  DetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  std::string resolve_engine_path();
  std::optional<cv::Mat> to_bgr_image(const sensor_msgs::msg::Image::SharedPtr & msg);
  cv::Mat draw_target_overlay(const cv::Mat & frame, const Detection * det, bool is_predicted);
  std::string class_name_of(int class_id) const;
  void update_kf_transition(float dt);
  static float bbox_iou(const Detection & a, const Detection & b);

  // TensorRT YOLO 推理引擎
  std::unique_ptr<TrtYolo> yolo_;

  // 类别名称映射
  std::map<int, std::string> class_names_;

  // ROS 通信
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr target_image_pub_;

  // 参数
  std::string image_topic_;
  std::string target_topic_;
  std::string target_image_topic_;
  int queue_size_;
  std::string engine_file_;
  bool target_image_enabled_;
  float conf_thresh_;
  float nms_thresh_;

  // 卡尔曼滤波
  cv::KalmanFilter kf_;
  bool kf_initialized_;
  int missed_frames_;
  int max_missed_frames_;
  double kf_process_noise_;
  double kf_measurement_noise_;
  std::string last_target_class_;

  // 目标关联参数
  double association_max_distance_;
  double association_min_iou_;
  bool association_require_same_class_;

  // 时间步与性能统计
  bool has_last_stamp_{false};
  rclcpp::Time last_stamp_;
  int perf_log_interval_;
  std::size_t perf_frame_count_{0};
  double perf_total_ms_sum_{0.0};
  double perf_infer_ms_sum_{0.0};
  double perf_vis_ms_sum_{0.0};
};

}  // namespace rm_vision_cpp

#endif  // RM_VISION_CPP__DETECTOR_NODE_HPP_
