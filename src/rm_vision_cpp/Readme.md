# rm_vision_cpp(V1.0)

创建日期:`2026/3/15`
作者：`yfy`

# 1. 文件说明
`package.xml`	                                                包清单，声明对 rclcpp等的依赖

`sensor_msgs、rm_interfaces、cv_bridge` 

`CMakeLists.txt`                                                构建配置，链接 OpenCV、TensorRT、

`CUDA`

`include/rm_vision_cpp/video_node.hpp`	                         VideoNode 头文件

`src/video_node.cpp`	                                             相机/视频采集节点，使用 OpenCV 

VideoCapture + 定时器发布 Image

`include/rm_vision_cpp/trt_yolo.hpp`	                        TensorRT YOLOv8 推理引擎封装头

`src/trt_yolo.cpp`	                                             TRT 10.x API 推理实现（加载 

.engine、letterbox 预处理、后处理 + NMS）

`include/rm_vision_cpp/detector_node.hpp`	                         DetectorNode 头文件

`src/detector_node.cpp`	                                         检测节点：订阅图像 -> TRT 推理 -> 

卡尔曼滤波跟踪 -> 发布 Target + debug 图

`config/camera_params.yaml`	                                     相机参数（与 Python 版一致）

`config/detector_params.yaml`	                                     检测器参数（含 conf_thresh、

`nms_thresh 和卡尔曼参数）

`launch/vision_cpp.launch.py`	                                     一键启动 video_node + 

`detector_node`

运行测试：

# 编译（注意需绕过 conda python）
colcon build --packages-select rm_vision_cpp --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# 启动
source install/setup.bash
ros2 launch rm_vision_cpp vision_cpp.launch.py