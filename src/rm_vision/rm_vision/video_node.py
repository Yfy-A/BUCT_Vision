"""
相机捕获节点 (Camera Node)

功能描述:
    读取摄像头硬件设备数据（如 USB 相机或工业相机），将图像流转换并发布为 ROS 2 图像话题。

输入 (Inputs):
    - 硬件: 本地相机设备，例如 `/dev/video0`

输出 (Publishes):
    - Topic: `/camera/image_raw`
    - Type:  `sensor_msgs/msg/Image`
    - 格式:   标准的 ROS 2 图像格式流 (此处默认为 BGR8)
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    """
    负责持续采集硬件摄像头图像并发布的 ROS 2 节点
    """
    def __init__(self):
        super().__init__('video_node')
        
        # 初始化图像流转换工具
        self.bridge = CvBridge()
        
        # 创建图像发布者，队列深度设小一点以保证实时性（丢弃旧图像）
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 1)
        
        # ------------------- 参数声明与获取 (Parameters Definition) -------------------
        # 是否使用本地视频文件替代真实摄像头进行调试
        self.declare_parameter('use_video_file', True)
        # 本地视频文件的路径 (仅在 use_video_file 为 True 时生效)
        self.declare_parameter('video_path', '')
        # 硬件摄像头设备的 ID 编号 (例如 0 对应 /dev/video0)
        self.declare_parameter('device_id', 0)
        # 摄像头期望的采集帧率 (Frames Per Second)
        self.declare_parameter('fps', 30.0)

        # 获取参数并统一保存为内部成员变量
        self.use_video_file = self.get_parameter('use_video_file').value
        self.video_path = self.get_parameter('video_path').value
        self.device_id = self.get_parameter('device_id').value
        self.fps = self.get_parameter('fps').value
        # ----------------------------------------------------------------------------

        # 初始化 OpenCV 相机或视频流对象
        if self.use_video_file:
            self.cap = cv2.VideoCapture(self.video_path)
            if not self.cap.isOpened():
                self.get_logger().error(f"Failed to open video file: {self.video_path}!")
                return
            self.get_logger().info(f"Video file opened successfully: {self.video_path}.")
        else:
            self.cap = cv2.VideoCapture(self.device_id)
            if not self.cap.isOpened():
                self.get_logger().error(f"Failed to open camera device {self.device_id}!")
                return
            self.get_logger().info(f"Camera opened successfully on device {self.device_id}.")
        
        # 设置读取定时器
        timer_period = 1.0 / self.fps  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        定时器回调函数：
        1.从硬件侧或视频流拉取一帧图像 -> 
        2.转换为 ROS 消息格式 -> 
        3.发布话题
        如果是视频流且读取结束，循环重播。
        """
        ret, frame = self.cap.read()
        
        # 如果是视频流且读取结束，尝试循环重播
        if not ret and self.use_video_file:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()

        if ret:
            try:
                # 'bgr8' 为常规 OpenCV 读取格式的封装协议
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                
                # 手动打上时间戳
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "camera_link"
                
                self.publisher.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to convert frame or publish: {e}")
        else:
            self.get_logger().warning("Failed to grab continuous frame from camera! Please check the connection.")

    def destroy_node(self):
        """
        在节点被销毁时正确关闭摄像头设备资源
        """
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera device released.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
