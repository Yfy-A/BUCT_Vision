# 百花机甲rm_vision 使用说明
创建日期:`2026/3/15`

作者：`yfy`

rm_vision 是一个基于 ROS 2 Humble 的 Python 视觉包，核心流程是：

图像输入 -> YOLO TensorRT 推理 -> 目标信息发布 -> 可选调试图像发布

该包当前已经支持混合环境运行（系统环境跑图像输入，Conda 环境跑推理），用于规避 `cv_bridge` 与不同 NumPy 版本的 ABI 冲突。

## 1. 功能概览

- `video_node`：从本地视频或摄像头采图，发布 `/camera/image_raw`
- `detector_node`：订阅图像，调用 TensorRT engine 推理，发布目标信息
- 串口转发功能已独立到 `rm_serial` 包

## 2. 代码结构

```text
rm_vision/
├── config/
│   ├── camera_params.yaml        # 视频/相机输入参数
│   └── detector_params.yaml      # 推理、发布、卡尔曼与性能统计参数
├── launch/
│   └── vision.launch.py          # launch 方式启动 video+detector
├── rm_vision/
│   ├── video_node.py             # 图像采集与发布
│   ├── detector_node.py          # TensorRT 推理与目标发布
│   └── __init__.py
├── weights/
│   ├── RM_Red_Vision_INT8.engine # 当前常用 INT8 引擎
│   └── ...                       # 其他引擎
├── package.xml
└── setup.py
```

## 3. Topic 与消息

### 3.1 输入

- `video_node` 发布：`/camera/image_raw`
- 消息类型：`sensor_msgs/msg/Image`

### 3.2 输出

- `detector_node` 发布：`/vision/target_info`
- 消息类型：`rm_interfaces/msg/Target`
- 字段：
  - `x: float32`
  - `y: float32`
  - `class_name: string`

- `detector_node` 可选发布：`/vision/target_image`
- 消息类型：`sensor_msgs/msg/Image`

## 4. 运行方式

下面按推荐顺序给出运行方式。

### 4.1 推荐：仓库根目录脚本一键启动（混合环境）

在工作空间根目录执行：

```bash
cd ~/RM_WS
./start_vision_python.sh video
```

或相机模式：

```bash
cd ~/RM_WS
./start_vision_python.sh
```

说明：

- 脚本会在系统环境启动 `video_node` 或相机驱动
- 在 Conda 环境 `sam_env2` 中启动 `detector_node`
- 启动前会清理 Python 相关环境变量，降低跨环境污染风险

### 4.2 仅使用 launch（同进程逻辑较简单，调试方便）

```bash
cd ~/RM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_vision vision.launch.py
```

### 4.3 手动分开启动（便于定位问题）

终端 1（系统环境）：

```bash
cd ~/RM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rm_vision video_node --ros-args --params-file src/rm_vision/config/camera_params.yaml
```

终端 2（Conda 环境）：

```bash
cd ~/RM_WS
source ~/miniconda3/etc/profile.d/conda.sh
conda activate sam_env2
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/rm_vision/rm_vision/detector_node.py --ros-args --params-file src/rm_vision/config/detector_params.yaml
```

## 5. 参数说明

### 5.1 camera_params.yaml

关键参数：

- `use_video_file`：`true` 用本地视频，`false` 用硬件相机
- `video_path`：视频绝对路径
- `device_id`：摄像头设备号
- `fps`：采集帧率上限

### 5.2 detector_params.yaml

关键参数：

- `image_topic`：输入图像 topic
- `target_topic`：目标发布 topic
- `debug_image_topic`：调试图像 topic
- `engine_file`：TensorRT engine 文件名
- `debug_image_enabled`：是否发布调试图（会影响性能）

性能统计参数：

- `perf_log_enabled`：是否打印分阶段耗时
- `perf_log_interval`：每累计多少帧打印一次
- `perf_warmup_frames`：预热帧数（不计入统计）

日志示例：

```text
[PERF][avg:30] convert=1.20ms, infer=5.80ms, post=0.60ms, publish=2.10ms, total=9.70ms, fps=103.09
```

## 6. 编译与安装

在工作空间根目录执行：

```bash
cd ~/RM_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select rm_vision
source install/setup.bash
```

## 7. 常见问题与注意事项

### 7.1 NumPy / cv_bridge ABI 冲突

现象：

- `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`
- `AttributeError: _ARRAY_API not found`

原因：

- `cv_bridge` 与 NumPy 主版本 ABI 不匹配
- Conda 与系统 Python 路径混用导致导入错位

建议：

- 保持系统与 Conda 依赖边界清晰
- 在推理环境中固定 NumPy 主版本（当前工程建议 `<2`）
- 使用仓库中的启动脚本，避免手动启动时遗漏环境清理

### 7.2 INT8 不一定明显快于 FP16

可能原因：

- 输入 FPS 已封顶
- 调试图绘制/发布占据较多时间
- 后处理或消息发布成为瓶颈

排查建议：

- 先看 `[PERF]` 分阶段日志
- 临时关闭 `debug_image_enabled` 再对比
- 分别比较 `infer` 与 `total` 的变化幅度

### 7.3 权重文件路径

`detector_node` 会优先在安装目录寻找 engine，其次回退源码目录。若改名或替换权重，请同步更新 `engine_file`。

## 8. 相关文件速查

- 包入口：`src/rm_vision/setup.py`
- 检测节点：`src/rm_vision/rm_vision/detector_node.py`
- 视频节点：`src/rm_vision/rm_vision/video_node.py`
- 串口节点（已迁移）：`src/rm_serial/rm_serial/serial_node.py`
- 检测参数：`src/rm_vision/config/detector_params.yaml`
- 相机参数：`src/rm_vision/config/camera_params.yaml`
- 一键脚本：`start_vision_test.sh`

---

如后续准备把 Python 版和 C++ 版统一到同一套启动参数，

新增一个统一的 `config/profile/*.yaml`（例如 `dev.yaml`、`match.yaml`），方便比赛和调试快速切换。