#ifndef RM_VISION_CPP__VIDEO_NODE_HPP_
#define RM_VISION_CPP__VIDEO_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>

namespace rm_vision_cpp
{

class VideoNode : public rclcpp::Node
{
public:
  VideoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VideoNode() override;

private:
  void timer_callback();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  cv::VideoCapture cap_;
  bool use_video_file_;
  std::string video_path_;
  int device_id_;
  double fps_;
};

}  // namespace rm_vision_cpp

#endif  // RM_VISION_CPP__VIDEO_NODE_HPP_
