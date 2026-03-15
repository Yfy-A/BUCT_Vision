# rm_serial

独立串口桥接包：订阅视觉云台误差角并按自定义协议发送到下位机。

## 功能

- 订阅 `rm_interfaces/msg/GimbalCmd`
- 可配置串口参数（端口、波特率、超时）
- 自动重连
- 可选校验模式（`crc8` / `sum8`）
- 类名到 ID 的映射
- 发送统计与可选十六进制日志

## 数据帧格式

默认协议：

`[HEADER:1][yaw_err:float32][pitch_err:float32][distance:float32][class_id:uint8][valid:uint8][checksum:uint8][FOOTER:1]`

- `valid=1` 表示当前帧有可用云台指令
- `valid=0` 表示当前无有效目标
- `checksum` 对 payload 计算（yaw_err,pitch_err,distance,class_id,valid）

## 运行

```bash
cd ~/RM_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select rm_serial
source install/setup.bash
ros2 launch rm_serial serial.launch.py
```

或直接运行：

```bash
ros2 run rm_serial serial_node --ros-args --params-file ~/RM_WS/src/rm_serial/config/serial_params.yaml
```

## 参数

参数文件：`config/serial_params.yaml`

建议优先调节：

- `port`
- `baud_rate`
- `class_id_pairs`
- `checksum_mode`
- `send_invalid_target_frame`
