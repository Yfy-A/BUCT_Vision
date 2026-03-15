"""
串口桥接节点（rm_serial）。

功能：
    订阅视觉节点发布的云台误差角信息（rm_interfaces/msg/GimbalCmd），
    按可配置协议打包后通过串口发送给下位机。

默认协议（字节数说明）：
    [HEADER:1][yaw_err:float32(4)][pitch_err:float32(4)][distance:float32(4)]
    [class_id:uint8(1)][valid:uint8(1)][checksum:uint8(1)][FOOTER:1]

字段字节数说明：
    - HEADER: 1 byte
    - yaw_err (float32): 4 bytes (小端)
    - pitch_err (float32): 4 bytes (小端)
    - distance (float32): 4 bytes (小端)
    - class_id (uint8): 1 byte
    - valid (uint8): 1 byte
    - checksum (uint8): 1 byte (基于 payload 计算，payload = yaw_err+pitch_err+distance+class_id+valid)
    - FOOTER: 1 byte

总长度： HEADER(1) + pa


yload(4+4+4+1+1=14) + checksum(1) + FOOTER(1) = 17 字节

说明：
    - yaw_err/pitch_err/distance 使用小端 float32 编码。
    - valid=1 表示当前帧有可用云台指令；valid=0 表示当前无目标。
    - checksum 基于 payload（yaw_err,pitch_err,distance,class_id,valid）计算，支持 sum8 与 crc8。
"""
import struct
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rm_interfaces.msg import GimbalCmd
import serial


class SerialNode(Node):
    """将视觉云台指令消息转换为串口帧并发送的 ROS 2 节点。"""

    def __init__(self):
        super().__init__('serial_node')

        # 1) 参数声明与读取
        self._declare_parameters()
        self._load_parameters()

        # 2) 运行时状态
        self.ser: Optional[serial.Serial] = None
        self.last_connect_try_sec = 0.0
        self.sent_count = 0
        self.drop_count = 0

        # 3) 订阅云台误差角消息
        self.subscription = self.create_subscription(
            GimbalCmd,
            self.gimbal_cmd_topic,
            self.gimbal_cmd_callback,
            self.queue_size, 
        )

        # 4) 定时重连机制（串口断开后自动恢复）
        self.reconnect_timer = self.create_timer(self.reconnect_interval_sec, self._reconnect_timer_cb)

        self._connect_serial(force_log=True)
        self.get_logger().info('rm_serial 已启动，等待云台指令消息...')

    def _declare_parameters(self):
        """声明 ROS 参数，便于通过 YAML 统一调参。"""
        self.declare_parameter('gimbal_cmd_topic', '/vision/gimbal_cmd')
        self.declare_parameter('queue_size', 20)

        self.declare_parameter('enabled', True)
        # 默认改为 Jetson 常见串口，避免未加载 YAML 时误走 USB 串口。
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout_sec', 0.01)
        self.declare_parameter('write_timeout_sec', 0.01)
        self.declare_parameter('reconnect_interval_sec', 1.0)

        self.declare_parameter('header_byte', 0x5A)
        self.declare_parameter('footer_byte', 0xA5)
        self.declare_parameter('checksum_mode', 'crc8')
        self.declare_parameter('crc_poly', 0x07)
        self.declare_parameter('crc_init', 0x00)

        self.declare_parameter('default_class_id', 0)
        self.declare_parameter('class_id_pairs', [
            'red_robot:1',
            'blue_robot:2',
            'red_armor:3',
            'blue_armor:4',
        ])

        self.declare_parameter('send_invalid_target_frame', True)
        self.declare_parameter('log_hex_frame', False)
        self.declare_parameter('stats_interval', 100)

    def _load_parameters(self):
        """读取并解析 ROS 参数。"""
        self.gimbal_cmd_topic = str(self.get_parameter('gimbal_cmd_topic').value)
        self.queue_size = int(self.get_parameter('queue_size').value)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.port = str(self.get_parameter('port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.write_timeout_sec = float(self.get_parameter('write_timeout_sec').value)
        self.reconnect_interval_sec = max(0.1, float(self.get_parameter('reconnect_interval_sec').value))

        self.header_byte = int(self.get_parameter('header_byte').value) & 0xFF
        self.footer_byte = int(self.get_parameter('footer_byte').value) & 0xFF
        self.checksum_mode = str(self.get_parameter('checksum_mode').value).lower()
        self.crc_poly = int(self.get_parameter('crc_poly').value) & 0xFF
        self.crc_init = int(self.get_parameter('crc_init').value) & 0xFF

        self.default_class_id = int(self.get_parameter('default_class_id').value) & 0xFF
        self.class_id_map = self._parse_class_id_pairs(self.get_parameter('class_id_pairs').value)

        self.send_invalid_target_frame = bool(self.get_parameter('send_invalid_target_frame').value)
        self.log_hex_frame = bool(self.get_parameter('log_hex_frame').value)
        self.stats_interval = max(1, int(self.get_parameter('stats_interval').value)) 

        if self.checksum_mode not in ('crc8', 'sum8'):
            self.get_logger().warning(
                f"不支持的 checksum_mode={self.checksum_mode}，已回退为 crc8"
            )
            self.checksum_mode = 'crc8'

    def _parse_class_id_pairs(self, pairs) -> Dict[str, int]:
        """解析类别映射字符串列表，格式示例：red_robot:1。"""
        result: Dict[str, int] = {}
        for item in pairs:
            text = str(item).strip()
            if ':' not in text:
                self.get_logger().warning(
                    f'忽略非法 class_id 映射项: {text}。期望格式为 "class_name:id"，例如 "red_robot:1"'
                )
                continue
            key, value = text.split(':', 1)
            key = key.strip()
            try:
                result[key] = int(value.strip()) & 0xFF
            except ValueError:
                self.get_logger().warning(f'忽略非法 class_id 数值: {text}')
        return result

    def _connect_serial(self, force_log: bool = False) -> bool:
        """尝试建立串口连接。连接成功返回 True，否则返回 False。"""
        if not self.enabled:
            return False

        if self.ser is not None and self.ser.is_open:
            return True

        now_sec = time.time()
        if not force_log and (now_sec - self.last_connect_try_sec) < self.reconnect_interval_sec:
            return False

        self.last_connect_try_sec = now_sec
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                # timeout=self.timeout_sec,
                # write_timeout=self.write_timeout_sec,
            )
            self.get_logger().info(f'串口连接成功: {self.port} @ {self.baud_rate}')
            return True
        except serial.SerialException as exc:
            self.ser = None
            self.get_logger().warning(f'串口连接失败 ({self.port}): {exc}')
            return False

    def _reconnect_timer_cb(self):
        """定时重连回调。"""
        self._connect_serial(force_log=False)

    def _checksum(self, payload: bytes) -> int:
        """根据配置计算校验值，支持 sum8 与 crc8。"""
        if self.checksum_mode == 'sum8':
            return sum(payload) & 0xFF

        crc = self.crc_init
        for byte in payload:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ self.crc_poly) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def _build_frame(self, msg: GimbalCmd) -> Optional[bytes]:
        """将 GimbalCmd 消息打包为串口帧。若配置为不发送无效帧则返回 None。"""
        valid = 1 if msg.valid else 0
        if valid == 0 and not self.send_invalid_target_frame:
            return None

        # class_name 由 rm_vision/detector_node 给出；映射不到则使用 default_class_id。
        class_id = self.class_id_map.get(msg.class_name, self.default_class_id)
        # 打包格式说明（小端）：
        #  '<'   : little-endian
        #  'f f f' : yaw_err (4 bytes), pitch_err (4 bytes), distance (4 bytes)
        #  'B B'   : class_id (1 byte), valid (1 byte)
        #  payload 总字节数 = 4+4+4+1+1 = 14 bytes
        payload = struct.pack(
            '<fffBB',
            float(msg.yaw_err_deg),
            float(msg.pitch_err_deg),
            float(msg.distance_m),
            class_id,
            valid,
        )
    
        
        check = self._checksum(payload)

        
        frame = bytes([self.header_byte]) + payload + bytes([check, self.footer_byte])
        return frame

    def gimbal_cmd_callback(self, msg: GimbalCmd):
        """云台指令回调：打包、连接检查、发送、统计。"""
        frame = self._build_frame(msg)
        if frame is None:
            return

        if not self._connect_serial(force_log=False):
            self.drop_count += 1
            return

        try:
            assert self.ser is not None
            self.ser.write(frame)
            self.sent_count += 1

            if self.log_hex_frame:
                self.get_logger().info(f'发送帧: {frame.hex(" ")}')
                

            if self.sent_count % self.stats_interval == 0:
                self.get_logger().info(
                    f'串口统计: sent={self.sent_count}, dropped={self.drop_count}'
                )
        except (serial.SerialException, OSError) as exc:
            self.drop_count += 1
            self.get_logger().warning(f'串口发送失败: {exc}')
            self._close_serial()

    def _close_serial(self):
        """安全关闭串口句柄。"""
        if self.ser is None:
            return
        try:
            if self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def destroy_node(self):
        """节点析构时释放串口资源。"""
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
