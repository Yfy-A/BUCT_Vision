#include "rm_vision_cpp/video_node.hpp"

#include <chrono>

namespace rm_vision_cpp
{

VideoNode::VideoNode(const rclcpp::NodeOptions & options)
: Node("video_node", options)
{
  // 声明参数
  this->declare_parameter("use_video_file", true);
  this->declare_parameter("video_path", std::string(""));
  this->declare_parameter("device_id", 0);
  this->declare_parameter("fps", 30.0);

  use_video_file_ = this->get_parameter("use_video_file").as_bool();
  video_path_ = this->get_parameter("video_path").as_string();
  device_id_ = this->get_parameter("device_id").as_int();
  fps_ = this->get_parameter("fps").as_double();

  // 创建图像发布者，队列深度设小以保证实时性
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 1);

  // 初始化 OpenCV 相机或视频流对象
  if (use_video_file_) {
    cap_.open(video_path_);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open video file: %s", video_path_.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Video file opened successfully: %s", video_path_.c_str());
  } else {
    cap_.open(device_id_);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera device %d", device_id_);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Camera opened successfully on device %d", device_id_);
  }

  // 设置定时器
  auto timer_period = std::chrono::duration<double>(1.0 / fps_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&VideoNode::timer_callback, this));
}

VideoNode::~VideoNode()
{
  if (cap_.isOpened()) {
    cap_.release();
    RCLCPP_INFO(this->get_logger(), "Camera device released.");
  }
}

void VideoNode::timer_callback()
{
  cv::Mat frame;
  bool ret = cap_.read(frame);

  // 如果是视频流且读取结束，循环重播
  if (!ret && use_video_file_) {
    cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
    ret = cap_.read(frame);
  }

  if (ret) {
    auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    img_msg->header.stamp = this->get_clock()->now();
    img_msg->header.frame_id = "camera_link";
    publisher_->publish(*img_msg);
  } else {
    RCLCPP_WARN(this->get_logger(),
      "Failed to grab continuous frame from camera! Please check the connection.");
  }
}

}  // namespace rm_vision_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rm_vision_cpp::VideoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
