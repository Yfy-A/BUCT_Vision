# ROS2 QoS (Quality of Service) 配置说明

## 概述

本文档详细说明了 BUCT_Vision 项目中 ROS2 通信的 QoS (Quality of Service) 策略配置，解释了如何保证图像流、云台控制指令等数据传输的实时性和稳定性。

## ROS2 QoS 基础

ROS2 的 QoS 策略包含多个维度：

### 1. Reliability (可靠性)
- **RELIABLE**: 保证消息送达，丢失时会重传（类似 TCP）
- **BEST_EFFORT**: 尽力而为，不保证送达，不重传（类似 UDP）

### 2. History (历史策略)
- **KEEP_LAST(n)**: 只保留最近的 n 条消息
- **KEEP_ALL**: 保留所有消息（受限于队列深度）

### 3. Durability (持久性)
- **VOLATILE**: 只对订阅后的消息负责
- **TRANSIENT_LOCAL**: 保留最近的消息给新订阅者

### 4. Deadline (截止时间)
- 消息必须在指定时间内到达，否则触发回调

### 5. Liveliness (活跃度)
- 监控发布者/订阅者的存活状态

---

## 本项目 QoS 配置策略

### 一、图像流传输 (Image Streaming)

#### 1.1 C++ 视觉检测节点 (`rm_vision_cpp/detector_node.cpp`)

```cpp
// 传感器场景采用 best-effort + keep_last(1)，避免高负载时可靠 QoS 回压导致帧率下降。
auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
  image_topic_, image_qos,
  std::bind(&DetectorNode::image_callback, this, std::placeholders::_1));
```

**配置说明**：
- **QoS Profile**: `SensorDataQoS` (预定义配置)
  - Reliability: `BEST_EFFORT`
  - History: `KEEP_LAST(1)` - 仅保留最新一帧
  - Durability: `VOLATILE`
- **适用场景**: 高频图像流（摄像头数据）
- **设计理由**:
  1. **避免帧率下降**: `BEST_EFFORT` 避免了 `RELIABLE` 模式下的重传和回压机制，在 CPU 高负载时不会因为等待重传而阻塞新帧
  2. **降低延迟**: `KEEP_LAST(1)` 确保处理的始终是最新帧，丢弃旧帧避免积压
  3. **实时性优先**: 对于视觉检测，处理最新帧比保证历史帧完整性更重要

#### 1.2 Python 视觉检测节点 (`rm_vision/detector_node.py`)

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# 创建 SensorData QoS 配置
image_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

self.subscription = self.create_subscription(
    Image,
    self.image_topic,
    self.image_callback,
    image_qos
)
```

**配置说明**：
- 与 C++ 版本保持一致，使用 `BEST_EFFORT` + `KEEP_LAST(1)`
- **注意**: 默认的 ROS2 QoS 是 `RELIABLE`，必须显式配置为 `BEST_EFFORT`

#### 1.3 摄像头节点 (`mvcc_camera_ros2`)

**发布端** (`create_camera_image_pub.cpp`):
```cpp
// 队列长度从 200 减少到 2，避免内存膨胀和处理积压导致的延迟累积
image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/mvcc_camera/Image", 2);
```

**订阅端** (`create_camera_image_sub.cpp`):
```cpp
image_sub_ = this->create_subscription<ImageMsg>("/mvcc_camera/Image", 100, ...);
```

**配置说明**：
- **发布端队列深度**: 2（极小缓冲，优先实时性）
- **订阅端队列深度**: 100（较大缓冲，避免丢帧）
- **设计理由**:
  1. 发布端小队列避免内存积压
  2. 订阅端大队列提供处理缓冲，应对短时 CPU 峰值

#### 1.4 视频捕获节点 (`video_node.cpp`)

```cpp
// 创建图像发布者，队列深度为 1 确保实时性
publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 1);
```

**配置说明**：
- **队列深度**: 1（最小延迟）
- **设计理由**: 摄像头源头应最快速发布最新帧，不积压历史帧

---

### 二、云台控制指令传输 (Gimbal Control Commands)

#### 2.1 串口通信节点 (`rm_serial/serial_node.py`)

```python
self.subscription = self.create_subscription(
    GimbalCmd,
    self.gimbal_cmd_topic,
    self.gimbal_cmd_callback,
    self.queue_size,  # 默认 20
)
```

**配置说明**：
- **QoS Profile**: 默认（RELIABLE + KEEP_LAST）
- **队列深度**: 20
- **设计理由**:
  1. **可靠传输**: 云台控制指令必须送达，使用 `RELIABLE`
  2. **适中缓冲**: 队列深度 20 提供足够缓冲应对短时处理延迟
  3. **串口保护**: 配合串口超时机制（0.01s）确保不阻塞

**串口通信延迟处理**:
```python
self.declare_parameter('write_timeout_sec', 0.01)      # 写入超时
self.declare_parameter('reconnect_interval_sec', 1.0)   # 重连间隔
self.declare_parameter('gimbal_response_delay_s', 0.1) # 云台响应延迟补偿
```

- **超时保护**: 0.01 秒写入超时避免阻塞
- **自动重连**: 1.0 秒重连定时器应对串口断开
- **延迟补偿**: 0.1 秒云台响应延迟预测补偿

---

### 三、检测结果发布

#### 3.1 目标检测结果 (`Target` 消息)

```python
self.publisher = self.create_publisher(Target, self.target_topic, self.queue_size)
```

**配置说明**：
- **QoS Profile**: 默认（RELIABLE + KEEP_LAST）
- **队列深度**: 10
- **设计理由**:
  1. 检测结果信息量小，使用 `RELIABLE` 保证送达
  2. 队列深度 10 平衡实时性和可靠性

#### 3.2 调试图像发布 (`debug_image_topic`)

```python
self.debug_image_publisher = self.create_publisher(Image, self.debug_image_topic, self.queue_size)
```

**配置说明**：
- **QoS Profile**: 默认（RELIABLE + KEEP_LAST）
- **队列深度**: 10
- **优化建议**: 调试图像应改用 `BEST_EFFORT` + `KEEP_LAST(1)`，降低调试开销

---

## CPU 高负载场景处理

### 问题：CPU 满载导致通信延迟激增

**症状**：
1. 图像帧率下降
2. 检测延迟增加
3. 云台响应迟缓
4. 消息队列积压

### 解决方案

#### 1. 图像流使用 BEST_EFFORT QoS

```cpp
// C++ 示例
auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
```

**效果**：
- 避免 `RELIABLE` 模式的重传机制，CPU 高负载时不会阻塞等待确认
- `KEEP_LAST(1)` 自动丢弃旧帧，处理最新数据

#### 2. 小队列深度

```cpp
publisher_ = this->create_publisher<Image>("/camera/image_raw", 1);
```

**效果**：
- 队列深度 1-2 避免内存积压
- 防止延迟累积（processing lag）

#### 3. 性能监控

本项目内置性能统计功能：

```python
# detector_node.py 中的性能统计
self.perf_acc = {
    'convert_ms': 0.0,  # 图像转换耗时
    'infer_ms': 0.0,    # 推理耗时
    'post_ms': 0.0,     # 后处理耗时
    'publish_ms': 0.0,  # 发布耗时
    'total_ms': 0.0,    # 总耗时
    'samples': 0,
}
```

**日志输出示例**：
```
[INFO] [detector_node]: 性能统计 (最近 30 帧):
  - 图像转换: 2.3 ms
  - 推理: 15.8 ms
  - 后处理: 3.1 ms
  - 发布: 0.8 ms
  - 总耗时: 22.0 ms
  - 帧率: 45.5 FPS
```

#### 4. CPU 亲和性优化（可选）

在 Jetson 平台上可设置 CPU 亲和性：

```bash
# 启动时绑定到大核
taskset -c 4-7 ros2 run rm_vision_cpp detector_node
```

#### 5. 线程优先级（可选）

```cpp
// C++ 节点可设置实时优先级
#include <pthread.h>
#include <sched.h>

struct sched_param param;
param.sched_priority = 80;  // 高优先级
pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
```

---

## QoS 配置参数化

### YAML 配置文件

**`rm_vision/config/detector_params.yaml`**:
```yaml
/**:
  ros__parameters:
    # QoS 配置
    queue_size: 10
    image_qos_reliability: 'best_effort'  # 'reliable' | 'best_effort'
    image_qos_depth: 1

    # 性能配置
    gimbal_response_delay_s: 0.1
    profile_interval_frames: 30
```

**`rm_serial/config/serial_params.yaml`**:
```yaml
/**:
  ros__parameters:
    queue_size: 20
    write_timeout_sec: 0.01
    reconnect_interval_sec: 1.0
```

---

## 实战测试与验证

### 1. 延迟测试

**测试工具**: `ros2 topic delay`

```bash
# 测试图像流延迟
ros2 topic delay /camera/image_raw

# 测试检测结果延迟
ros2 topic delay /vision/target_info
```

**正常指标**：
- 图像流延迟: < 50 ms
- 检测结果延迟: < 100 ms

### 2. 吞吐量测试

```bash
# 测试消息发布频率
ros2 topic hz /camera/image_raw

# 测试消息丢失率
ros2 topic echo --no-arr /camera/image_raw | grep -c "data:"
```

**正常指标**：
- 图像流帧率: 30-60 FPS
- 消息丢失率: < 1%

### 3. CPU 负载测试

```bash
# 监控 CPU 使用率
htop

# 或使用 ROS2 工具
ros2 run demo_nodes_cpp listener __ns:=/vision &
stress-ng --cpu 8 --timeout 60s
```

**测试场景**：
1. 空载运行: 验证基础性能
2. CPU 满载: 验证 QoS 策略有效性
3. 网络拥塞: 验证可靠性策略

---

## 最佳实践总结

### 1. 图像流（高频传感器数据）

✅ **推荐配置**:
- QoS: `BEST_EFFORT` + `KEEP_LAST(1)`
- 队列深度: 1-2

❌ **避免配置**:
- QoS: `RELIABLE` (会导致 CPU 高负载时阻塞)
- 大队列深度 (会导致延迟累积)

### 2. 控制指令（云台、底盘）

✅ **推荐配置**:
- QoS: `RELIABLE` + `KEEP_LAST(10-20)`
- 超时保护: 0.01-0.1 秒

❌ **避免配置**:
- QoS: `BEST_EFFORT` (可能丢失关键指令)
- 无超时保护 (可能阻塞)

### 3. 检测结果（轻量级消息）

✅ **推荐配置**:
- QoS: `RELIABLE` + `KEEP_LAST(10)`
- 队列深度: 10

### 4. 调试数据（可选订阅）

✅ **推荐配置**:
- QoS: `BEST_EFFORT` + `KEEP_LAST(1)`
- 避免影响主流程性能

---

## 常见问题 FAQ

### Q1: 为什么图像流使用 BEST_EFFORT 而不是 RELIABLE？

**A**:
- `RELIABLE` 会在消息丢失时重传，CPU 高负载时会导致阻塞和回压
- 图像流是连续数据，丢失一帧不影响后续处理
- `BEST_EFFORT` 确保始终处理最新帧，避免延迟累积

### Q2: CPU 满载时如何保证通信不中断？

**A**:
1. 图像流使用 `BEST_EFFORT` QoS，避免阻塞
2. 小队列深度（1-2）避免内存积压
3. 串口通信使用超时保护（0.01s）
4. 自动重连机制（1.0s 定时器）
5. 性能监控及时发现瓶颈

### Q3: 队列深度如何选择？

**A**:
- **图像源头发布**: 1-2（实时性优先）
- **图像订阅处理**: 10-100（缓冲峰值）
- **控制指令**: 10-20（平衡实时性和可靠性）
- **调试数据**: 1（最小开销）

### Q4: 如何验证 QoS 配置是否生效？

**A**:
```bash
# 查看 topic 的 QoS 信息
ros2 topic info -v /camera/image_raw

# 输出示例：
# QoS profile:
#   Reliability: BEST_EFFORT
#   History (Depth): KEEP_LAST (1)
#   Durability: VOLATILE
```

### Q5: 遇到通信延迟激增怎么办？

**A**:
1. 检查 CPU 使用率: `htop`
2. 检查消息频率: `ros2 topic hz /camera/image_raw`
3. 检查消息延迟: `ros2 topic delay /camera/image_raw`
4. 查看性能统计日志
5. 考虑降低图像分辨率或帧率
6. 优化推理模型（TensorRT INT8 量化）

---

## 参考资料

- [ROS2 QoS Policies Documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [ROS2 DDS Tuning Guide](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)
- [rclcpp QoS API](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1QoS.html)

---

**文档版本**: v1.0
**最后更新**: 2026-03-27
**维护者**: BUCT Vision Team
