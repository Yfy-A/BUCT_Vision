#include "rm_vision_cpp/detector_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/qos.hpp>

namespace rm_vision_cpp
{

DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
: Node("detector_node", options),
  kf_(8, 4, 0),
  kf_initialized_(false),
  missed_frames_(0)
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

  // 卡尔曼滤波参数
  this->declare_parameter("kf_process_noise", 1e-2);
  this->declare_parameter("kf_measurement_noise", 1e-1);
  this->declare_parameter("kf_max_missed_frames", 15);

  // 关联门控与性能日志参数
  this->declare_parameter("association_max_distance", 120.0);
  this->declare_parameter("association_min_iou", 0.05);
  this->declare_parameter("association_require_same_class", true);
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

  kf_process_noise_ = this->get_parameter("kf_process_noise").as_double();
  kf_measurement_noise_ = this->get_parameter("kf_measurement_noise").as_double();
  max_missed_frames_ = this->get_parameter("kf_max_missed_frames").as_int();
  association_max_distance_ = this->get_parameter("association_max_distance").as_double();
  association_min_iou_ = this->get_parameter("association_min_iou").as_double();
  association_require_same_class_ =
    this->get_parameter("association_require_same_class").as_bool();
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

  // ==================== 卡尔曼滤波初始化 ====================
  // 状态向量: [x, y, w, h, dx, dy, dw, dh]
  // 观测向量: [x, y, w, h]
  kf_.transitionMatrix = (cv::Mat_<float>(8, 8) <<
    1, 0, 0, 0, 1, 0, 0, 0,
    0, 1, 0, 0, 0, 1, 0, 0,
    0, 0, 1, 0, 0, 0, 1, 0,
    0, 0, 0, 1, 0, 0, 0, 1,
    0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 1);

  kf_.measurementMatrix = (cv::Mat_<float>(4, 8) <<
    1, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0);

  cv::setIdentity(kf_.processNoiseCov, cv::Scalar(kf_process_noise_));
  cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar(kf_measurement_noise_));
  cv::setIdentity(kf_.errorCovPost);

  // ==================== ROS 订阅与发布 ====================
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

void DetectorNode::update_kf_transition(float dt)
{
  kf_.transitionMatrix.at<float>(0, 4) = dt;
  kf_.transitionMatrix.at<float>(1, 5) = dt;
  kf_.transitionMatrix.at<float>(2, 6) = dt;
  kf_.transitionMatrix.at<float>(3, 7) = dt;

  cv::setIdentity(kf_.processNoiseCov, cv::Scalar(0));
  const float q_pos = static_cast<float>(kf_process_noise_ * dt * dt);
  const float q_vel = static_cast<float>(kf_process_noise_ * dt);
  for (int i = 0; i < 4; ++i) {
    kf_.processNoiseCov.at<float>(i, i) = q_pos;
    kf_.processNoiseCov.at<float>(i + 4, i + 4) = q_vel;
  }
}

float DetectorNode::bbox_iou(const Detection & a, const Detection & b)
{
  const float ix1 = std::max(a.x1, b.x1);
  const float iy1 = std::max(a.y1, b.y1);
  const float ix2 = std::min(a.x2, b.x2);
  const float iy2 = std::min(a.y2, b.y2);
  const float inter = std::max(0.0f, ix2 - ix1) * std::max(0.0f, iy2 - iy1);
  const float area_a = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
  const float area_b = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
  const float denom = area_a + area_b - inter + 1e-6f;
  return inter / denom;
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

  float dt = 1.0f / 30.0f;
  rclcpp::Time current_stamp = msg->header.stamp;
  if (has_last_stamp_) {
    const double delta = (current_stamp - last_stamp_).seconds();
    dt = static_cast<float>(std::clamp(delta, 1.0 / 240.0, 0.2));
  }
  last_stamp_ = current_stamp;
  has_last_stamp_ = true;

  cv::Mat prediction;
  bool has_prediction = false;
  if (kf_initialized_) {
    update_kf_transition(dt);
    prediction = kf_.predict();
    has_prediction = true;
  }

  // 2. TensorRT YOLO 推理
  const auto infer_start = std::chrono::steady_clock::now();
  auto detections = yolo_->detect(cv_image);
  const auto infer_end = std::chrono::steady_clock::now();

  // 3. 关联门控：优先选择与预测状态一致的目标，减少跳目标
  Detection * best_det = nullptr;
  Detection predicted_det{};
  if (has_prediction) {
    const float pcx = prediction.at<float>(0);
    const float pcy = prediction.at<float>(1);
    const float pw = std::max(2.0f, prediction.at<float>(2));
    const float ph = std::max(2.0f, prediction.at<float>(3));
    predicted_det.x1 = pcx - pw / 2.0f;
    predicted_det.y1 = pcy - ph / 2.0f;
    predicted_det.x2 = pcx + pw / 2.0f;
    predicted_det.y2 = pcy + ph / 2.0f;
  }

  if (!detections.empty()) {
    if (has_prediction) {
      float best_score = -std::numeric_limits<float>::infinity();
      for (auto & det : detections) {
        const float cx = 0.5f * (det.x1 + det.x2);
        const float cy = 0.5f * (det.y1 + det.y2);
        const float pcx = prediction.at<float>(0);
        const float pcy = prediction.at<float>(1);
        const float dx = cx - pcx;
        const float dy = cy - pcy;
        const float dist = std::sqrt(dx * dx + dy * dy);
        const float iou = bbox_iou(det, predicted_det);
        const bool class_ok = (!association_require_same_class_) ||
          last_target_class_.empty() || class_name_of(det.class_id) == last_target_class_;
        const bool gated = class_ok && (dist <= static_cast<float>(association_max_distance_) ||
          iou >= static_cast<float>(association_min_iou_));
        if (!gated) {
          continue;
        }

        const float score = det.confidence + 0.2f * iou - 0.002f * dist;
        if (score > best_score) {
          best_score = score;
          best_det = &det;
        }
      }
    }

    if (best_det == nullptr && !kf_initialized_) {
      auto it = std::max_element(detections.begin(), detections.end(),
        [](const Detection & a, const Detection & b) {
          return a.confidence < b.confidence;
        });
      best_det = &(*it);
    }
  }

  // 4. 卡尔曼滤波更新与预测
  rm_interfaces::msg::Target target_msg;
  bool is_predicted = false;

  if (best_det != nullptr) {
    float cx = (best_det->x1 + best_det->x2) / 2.0f;
    float cy = (best_det->y1 + best_det->y2) / 2.0f;
    float w = best_det->x2 - best_det->x1;
    float h = best_det->y2 - best_det->y1;

    // 获取类别名称
    std::string class_name = class_name_of(best_det->class_id);
    last_target_class_ = class_name;

    cv::Mat measurement = (cv::Mat_<float>(4, 1) << cx, cy, w, h);

    if (!kf_initialized_) {
      kf_.statePost = (cv::Mat_<float>(8, 1) << cx, cy, w, h, 0, 0, 0, 0);
      kf_initialized_ = true;
    } else {
      kf_.correct(measurement);
    }

    missed_frames_ = 0;

    target_msg.x = kf_.statePost.at<float>(0);
    target_msg.y = kf_.statePost.at<float>(1);
    target_msg.class_name = last_target_class_;
  } else {
    // 未检测到目标，使用卡尔曼滤波预测补偿
    if (kf_initialized_ && has_prediction && missed_frames_ < max_missed_frames_) {
      missed_frames_++;

      float cx = prediction.at<float>(0);
      float cy = prediction.at<float>(1);
      float w = std::max(2.0f, prediction.at<float>(2));
      float h = std::max(2.0f, prediction.at<float>(3));

      // 创建一个虚拟 detection 用于可视化
      static Detection pred_det;
      pred_det.x1 = cx - w / 2.0f;
      pred_det.y1 = cy - h / 2.0f;
      pred_det.x2 = cx + w / 2.0f;
      pred_det.y2 = cy + h / 2.0f;
      pred_det.confidence = 0.0f;
      pred_det.class_id = -1;
      best_det = &pred_det;
      is_predicted = true;

      target_msg.x = cx;
      target_msg.y = cy;
      target_msg.class_name = last_target_class_;
    } else {
      // 丢失目标太久，重置卡尔曼滤波器
      kf_initialized_ = false;
      target_msg.x = 0.0f;
      target_msg.y = 0.0f;
      target_msg.class_name = "";
    }
  }

  // 发布目标消息
  if (!rclcpp::ok()) return;
  target_pub_->publish(target_msg);

  // 5. 生成并发布 target_image 可视化图像
  auto vis_start = std::chrono::steady_clock::now();
  if (target_image_enabled_) {
    cv::Mat annotated = draw_target_overlay(cv_image, best_det, is_predicted);
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
      "perf avg[%zu]: total=%.2fms infer=%.2fms vis=%.2fms fps=%.2f kf=%s miss=%d det=%zu",
      perf_frame_count_, avg_total, avg_infer, avg_vis, fps,
      kf_initialized_ ? "on" : "off", missed_frames_, detections.size());
  }
}

cv::Mat DetectorNode::draw_target_overlay(
  const cv::Mat & frame, const Detection * det, bool is_predicted)
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

  cv::Scalar color;
  std::string label;

  if (is_predicted) {
    color = cv::Scalar(0, 165, 255);  // 橙色 - 卡尔曼滤波预测
    label = last_target_class_ + " (pred)";
  } else {
    color = cv::Scalar(0, 255, 0);  // 绿色 - 实际检测
    std::string class_name = class_name_of(det->class_id);
    char conf_str[32];
    snprintf(conf_str, sizeof(conf_str), " %.2f", det->confidence);
    label = class_name + conf_str;
  }

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
