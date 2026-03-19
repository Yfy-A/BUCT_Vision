#include "rm_vision_cpp/detector_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/qos.hpp>

namespace rm_vision_cpp
{

DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
: Node("detector_node", options)
{
  // ==================== 参数声明 ====================
  this->declare_parameter("image_topic", std::string("/camera/image_raw"));
  this->declare_parameter("target_topic", std::string("/vision/target_info"));
  this->declare_parameter("target_image_topic", std::string("/vision/target_image"));
  this->declare_parameter("queue_size", 10);
  this->declare_parameter("engine_file", std::string("RM_Red_Vision.engine"));
  this->declare_parameter("target_image_enabled", true);
  this->declare_parameter("conf_thresh", 0.25);
  this->declare_parameter("nms_thresh", 0.45);

  // 性能日志参数
  this->declare_parameter("perf_log_interval", 30);

  // 类别名称 (以字符串列表传入，索引即为 class_id)
  this->declare_parameter("class_names", std::vector<std::string>{});

  // ==================== 获取参数 ====================
  image_topic_ = this->get_parameter("image_topic").as_string();
  target_topic_ = this->get_parameter("target_topic").as_string();
  target_image_topic_ = this->get_parameter("target_image_topic").as_string();

  queue_size_ = this->get_parameter("queue_size").as_int();
  engine_file_ = this->get_parameter("engine_file").as_string();
  target_image_enabled_ = this->get_parameter("target_image_enabled").as_bool();

  conf_thresh_ = static_cast<float>(this->get_parameter("conf_thresh").as_double());
  nms_thresh_ = static_cast<float>(this->get_parameter("nms_thresh").as_double());

  perf_log_interval_ =
    std::max(1, static_cast<int>(this->get_parameter("perf_log_interval").as_int()));

  auto class_names_param = this->get_parameter("class_names");
  if (class_names_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    auto names_vec = class_names_param.as_string_array();
    for (size_t i = 0; i < names_vec.size(); ++i) {
      class_names_[static_cast<int>(i)] = names_vec[i];
    }
  } else if (class_names_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    auto names_vec = class_names_param.as_integer_array();
    for (size_t i = 0; i < names_vec.size(); ++i) {
      class_names_[static_cast<int>(i)] = std::to_string(names_vec[i]);
    }
    RCLCPP_WARN(this->get_logger(),
      "Parameter 'class_names' is integer array, converted to strings automatically.");
  }

  // ==================== 加载 TensorRT 模型 ====================
  std::string engine_path = resolve_engine_path();
  RCLCPP_INFO(this->get_logger(), "Loading YOLO model from: %s", engine_path.c_str());
  yolo_ = std::make_unique<TrtYolo>(engine_path, conf_thresh_, nms_thresh_);
  RCLCPP_INFO(this->get_logger(), "YOLO model loaded. Input: %dx%d",
    yolo_->input_width(), yolo_->input_height());

  // ==================== ROS 订阅与发布 ======================================
  // 传感器场景采用 best-effort + keep_last(1)，避免高负载时可靠 QoS 回压导致帧率下降。
  auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic_, image_qos,
    std::bind(&DetectorNode::image_callback, this, std::placeholders::_1));

  target_pub_ = this->create_publisher<rm_interfaces::msg::Target>(
    target_topic_, queue_size_);

  target_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    target_image_topic_, queue_size_);

  RCLCPP_INFO(this->get_logger(), "Detector image_topic: %s", image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Detector target_topic: %s", target_topic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "Detector target_image_topic: %s (enabled=%s)",
    target_image_topic_.c_str(), target_image_enabled_ ? "true" : "false");
}

std::string DetectorNode::class_name_of(int class_id) const
{
  auto name_it = class_names_.find(class_id);
  if (name_it != class_names_.end()) {
    return name_it->second;
  }
  return std::to_string(class_id);
}

std::optional<cv::Mat> DetectorNode::to_bgr_image(const sensor_msgs::msg::Image::SharedPtr & msg)
{
  namespace enc = sensor_msgs::image_encodings;
  try {
    if (msg->encoding == enc::BGR8) {
      return cv_bridge::toCvCopy(msg, enc::BGR8)->image;
    }
    if (msg->encoding == enc::RGB8) {
      return cv_bridge::toCvCopy(msg, enc::BGR8)->image;
    }

    if (msg->encoding == enc::MONO8) {
      cv::Mat mono = cv_bridge::toCvCopy(msg, enc::MONO8)->image;
      cv::Mat bgr;
      cv::cvtColor(mono, bgr, cv::COLOR_GRAY2BGR);
      return bgr;
    }

    // 其他编码交给 cv_bridge 处理，若失败会抛异常
    return cv_bridge::toCvCopy(msg, enc::BGR8)->image;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Image convert failed: src_encoding=%s, error=%s",
      msg->encoding.c_str(), e.what());
    return std::nullopt;
  }
}

std::string DetectorNode::resolve_engine_path()
{
  // 优先寻找安装目录 (install/share)
  try {
    std::string share_dir = ament_index_cpp::get_package_share_directory("rm_vision_cpp");
    std::string install_path = share_dir + "/weights/" + engine_file_;
    if (std::filesystem::exists(install_path)) {
      return install_path;
    }
  } catch (...) {
    // rm_vision_cpp share 未找到，继续尝试
  }

  // 尝试从 rm_vision 的 share 目录查找
  try {
    std::string share_dir = ament_index_cpp::get_package_share_directory("rm_vision");
    std::string install_path = share_dir + "/weights/" + engine_file_;
    if (std::filesystem::exists(install_path)) {
      return install_path;
    }
  } catch (...) {
    // 继续尝试
  }

  // 回退: 从源码树查找
  std::string source_path = std::string(SOURCE_DIR) + "/weights/" + engine_file_;
  if (std::filesystem::exists(source_path)) {
    RCLCPP_WARN(this->get_logger(),
      "Engine file not found in install share, fallback to source tree: %s",
      source_path.c_str());
    return source_path;
  }

  throw std::runtime_error(
    "YOLO engine not found: " + engine_file_ +
    ". Place it in the weights/ directory.");
}

void DetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  const auto callback_start = std::chrono::steady_clock::now();

  // 1. ROS Image -> OpenCV BGR
  auto maybe_bgr = to_bgr_image(msg);
  if (!maybe_bgr.has_value()) {
    return;
  }
  cv::Mat cv_image = maybe_bgr.value();

  // 2. TensorRT YOLO 推理
  const auto infer_start = std::chrono::steady_clock::now();
  auto detections = yolo_->detect(cv_image);
  const auto infer_end = std::chrono::steady_clock::now();

  // 3. 选择置信度最高的目标
  Detection * best_det = nullptr;
  if (!detections.empty()) {
    auto it = std::max_element(detections.begin(), detections.end(),
      [](const Detection & a, const Detection & b) {
        return a.confidence < b.confidence;
      });
    best_det = &(*it);
  }

  // 4. 构建并发布目标消息
  rm_interfaces::msg::Target target_msg;

  if (best_det != nullptr) {
    float cx = (best_det->x1 + best_det->x2) / 2.0f;
    float cy = (best_det->y1 + best_det->y2) / 2.0f;

    target_msg.x = cx;
    target_msg.y = cy;
    target_msg.class_name = class_name_of(best_det->class_id);
  } else {
    target_msg.x = 0.0f;
    target_msg.y = 0.0f;
    target_msg.class_name = "";
  }

  if (!rclcpp::ok()) return;
  target_pub_->publish(target_msg);

  // 5. 生成并发布 target_image 可视化图像
  auto vis_start = std::chrono::steady_clock::now();
  if (target_image_enabled_) {
    cv::Mat annotated = draw_target_overlay(cv_image, best_det);
    auto target_image_msg = cv_bridge::CvImage(msg->header, "bgr8", annotated).toImageMsg();
    target_image_pub_->publish(*target_image_msg);
  }
  auto vis_end = std::chrono::steady_clock::now();

  const double infer_ms = std::chrono::duration<double, std::milli>(infer_end - infer_start).count();
  const double vis_ms = std::chrono::duration<double, std::milli>(vis_end - vis_start).count();
  const double total_ms = std::chrono::duration<double, std::milli>(vis_end - callback_start).count();

  perf_frame_count_++;
  perf_infer_ms_sum_ += infer_ms;
  perf_vis_ms_sum_ += vis_ms;
  perf_total_ms_sum_ += total_ms;
  if (perf_frame_count_ % static_cast<std::size_t>(perf_log_interval_) == 0) {
    const double avg_total = perf_total_ms_sum_ / perf_frame_count_;
    const double avg_infer = perf_infer_ms_sum_ / perf_frame_count_;
    const double avg_vis = perf_vis_ms_sum_ / perf_frame_count_;
    const double fps = (avg_total > 0.0) ? (1000.0 / avg_total) : 0.0;
    RCLCPP_INFO(
      this->get_logger(),
      "perf avg[%zu]: total=%.2fms infer=%.2fms vis=%.2fms fps=%.2f det=%zu",
      perf_frame_count_, avg_total, avg_infer, avg_vis, fps, detections.size());
  }
}

cv::Mat DetectorNode::draw_target_overlay(
  const cv::Mat & frame, const Detection * det)
{
  cv::Mat annotated = frame.clone();
  if (det == nullptr) {
    cv::putText(
      annotated, "NO TARGET", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX,
      1.0, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    return annotated;
  }

  int x1 = static_cast<int>(det->x1);
  int y1 = static_cast<int>(det->y1);
  int x2 = static_cast<int>(det->x2);
  int y2 = static_cast<int>(det->y2);
  float cx = (det->x1 + det->x2) / 2.0f;
  float cy = (det->y1 + det->y2) / 2.0f;

  cv::Scalar color = cv::Scalar(0, 255, 0);
  std::string class_name = class_name_of(det->class_id);
  char conf_str[32];
  snprintf(conf_str, sizeof(conf_str), " %.2f", det->confidence);
  std::string label = class_name + conf_str;

  cv::rectangle(annotated, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
  cv::circle(annotated, cv::Point(static_cast<int>(cx), static_cast<int>(cy)),
    4, cv::Scalar(0, 255, 255), -1);
  cv::putText(annotated, label, cv::Point(x1, std::max(y1 - 10, 30)),
    cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv::LINE_AA);

  return annotated;
}

}  // namespace rm_vision_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rm_vision_cpp::DetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
