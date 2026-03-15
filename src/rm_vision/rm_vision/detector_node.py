"""
视觉推理节点 (Detector Node)

功能描述:
    订阅相机图像话题，将图像转换为 OpenCV 格式送入 YOLOv8 (通过 TensorRT) 进行推理。
    提取置信度最高的目标，将其中心坐标和类别名称以自定义消息格式发布。

输入 (Subscribes):
    - Topic: `/camera/image_raw`
    - Type:  `sensor_msgs/msg/Image`
    - 格式:   标准的 ROS 2 图像格式流

输出 (Publishes):
    - Topic: `/vision/target_info`
    - Type:  `rm_interfaces/msg/Target`
    - 格式:   包含检测目标的中心坐标:
             `x` (float32, 像素坐标 x), 
             `y` (float32, 像素坐标 y), 
             `class_name` (string, 例如 "red_robot", "blue_armor" 等)
    - Topic: `/vision/debug_image`
    - Type:  `sensor_msgs/msg/Image`
    - 格式:   在原始图像上叠加检测框、类别、置信度的可视化画面
    - Topic: `/vision/gimbal_cmd`
    - Type:  `rm_interfaces/msg/GimbalCmd`
    - 格式:   云台偏差角（yaw_err_deg/pitch_err_deg）与距离 distance_m
"""
import os
import time
import math
from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.parameter import Parameter
from rm_interfaces.msg import GimbalCmd, Target
from sensor_msgs.msg import Image
from ultralytics import YOLO

class DetectorNode(Node):
    """
    负责执行目标检测推理的 ROS 2 节点
    """
    def __init__(self):
        """初始化检测节点：参数、模型、通信接口与滤波器状态。"""
        super().__init__('detector_node')
        self.bridge = CvBridge()

        # ------------------- 参数声明 -------------------
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('target_topic', '/vision/target_info')
        self.declare_parameter('debug_image_topic', '/vision/debug_image')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('engine_file', 'RM_Red_Vision.engine')
        self.declare_parameter('debug_image_enabled', True)
        self.declare_parameter('perf_log_enabled', True)
        self.declare_parameter('perf_log_interval', 30)
        self.declare_parameter('perf_warmup_frames', 10)
        self.declare_parameter('pnp_topic', '/vision/pnp_info')
        self.declare_parameter('gimbal_cmd_topic', '/vision/gimbal_cmd')
        self.declare_parameter('armor_corner_enabled', True)
        self.declare_parameter('pnp_enabled', True)
        self.declare_parameter('opencv_corners_vis_enabled', True)
        self.declare_parameter('opencv_corners_vis_radius', 20)

        # 阈值分组 profile: manual | auto | indoor | outdoor | match
        self.declare_parameter('profile_mode', 'auto')
        self.declare_parameter('profile_switch_interval_frames', 10)
        self.declare_parameter('profile_auto_brightness_low', 90.0)
        self.declare_parameter('profile_auto_brightness_high', 150.0)

        # OpenCV 灯条提角点参数
        self.declare_parameter('roi_expand_ratio', 0.15)
        self.declare_parameter('light_binary_thresh', 190)
        self.declare_parameter('light_min_area', 18.0)
        self.declare_parameter('light_min_aspect_ratio', 1.6)
        self.declare_parameter('light_max_aspect_ratio', 16.0)
        self.declare_parameter('light_max_angle_diff_deg', 18.0)
        self.declare_parameter('light_max_center_y_diff_ratio', 0.6)
        self.declare_parameter('light_min_pair_gap_ratio', 0.6)
        self.declare_parameter('light_max_pair_gap_ratio', 6.0)

        # 室内参数
        self.declare_parameter('indoor_light_binary_thresh', 120)
        self.declare_parameter('indoor_light_min_area', 20.0)
        self.declare_parameter('indoor_light_min_aspect_ratio', 0.8)
        self.declare_parameter('indoor_light_max_aspect_ratio', 20.0)
        self.declare_parameter('indoor_light_max_angle_diff_deg', 80.0)
        self.declare_parameter('indoor_light_max_center_y_diff_ratio', 2.0)
        self.declare_parameter('indoor_light_min_pair_gap_ratio', 3.0)
        self.declare_parameter('indoor_light_max_pair_gap_ratio', 8.5)

        # 室外参数
        self.declare_parameter('outdoor_light_binary_thresh', 170)
        self.declare_parameter('outdoor_light_min_area', 18.0)
        self.declare_parameter('outdoor_light_min_aspect_ratio', 1.2)
        self.declare_parameter('outdoor_light_max_aspect_ratio', 16.0)
        self.declare_parameter('outdoor_light_max_angle_diff_deg', 35.0)
        self.declare_parameter('outdoor_light_max_center_y_diff_ratio', 1.0)
        self.declare_parameter('outdoor_light_min_pair_gap_ratio', 1.0)
        self.declare_parameter('outdoor_light_max_pair_gap_ratio', 5.0)

        # match profile
        self.declare_parameter('match_light_binary_thresh', 145)
        self.declare_parameter('match_light_min_area', 20.0)
        self.declare_parameter('match_light_min_aspect_ratio', 1.0)
        self.declare_parameter('match_light_max_aspect_ratio', 18.0)
        self.declare_parameter('match_light_max_angle_diff_deg', 45.0)
        self.declare_parameter('match_light_max_center_y_diff_ratio', 1.4)
        self.declare_parameter('match_light_min_pair_gap_ratio', 1.5)
        self.declare_parameter('match_light_max_pair_gap_ratio', 6.0)

        # PnP 参数：相机内参与装甲板几何尺寸（单位：米）
        self.declare_parameter('camera_matrix', [1000.0, 0.0, 640.0, 0.0, 1000.0, 360.0, 0.0, 0.0, 1.0])
        self.declare_parameter('dist_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('armor_width_m', 0.135)
        self.declare_parameter('armor_height_m', 0.055)
        self.declare_parameter('armor_width_small_m', 0.135)
        self.declare_parameter('armor_height_small_m', 0.055)
        self.declare_parameter('armor_width_large_m', 0.230)
        self.declare_parameter('armor_height_large_m', 0.055)
        self.declare_parameter('armor_size_auto_enabled', True)
        self.declare_parameter('armor_size_default', 'small')
        self.declare_parameter('armor_size_ratio_threshold', 3.0)
        # 使用“类型声明”而非空数组默认值，避免 Humble 将 [] 推断为 BYTE_ARRAY。
        # YAML 覆盖值例如: ['hero:large', 'infantry:small']。
        self.declare_parameter('armor_class_size_pairs', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('cam_to_gimbal_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('cam_to_gimbal_t_xyz_m', [0.0, 0.0, 0.0])
        self.declare_parameter('ballistic_enabled', True)
        self.declare_parameter('projectile_speed_mps', 28.0)
        self.declare_parameter('gravity_mps2', 9.81)
        self.declare_parameter('yaw_zero_offset_deg', 0.0)
        self.declare_parameter('pitch_zero_offset_deg', 0.0)
        
        # 卡尔曼滤波相关参数
        self.declare_parameter('kf_process_noise', 1e-2)
        self.declare_parameter('kf_measurement_noise', 1e-1)
        self.declare_parameter('kf_max_missed_frames', 15)

        # ------------------- 参数读取 -------------------
        self._load_parameters()

        # ------------------- 模型初始化 -------------------
        engine_path = self._resolve_engine_path()
        self.get_logger().info(f"Loading YOLO model from: {engine_path}")
        self.model = YOLO(engine_path)

        # ------------------- 订阅与发布 -------------------
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            self.queue_size
        )

        self.publisher = self.create_publisher(Target, self.target_topic, self.queue_size)
        self.debug_image_publisher = self.create_publisher(Image, self.debug_image_topic, self.queue_size)
        self.pnp_publisher = self.create_publisher(Vector3, self.pnp_topic, self.queue_size)
        self.gimbal_cmd_publisher = self.create_publisher(GimbalCmd, self.gimbal_cmd_topic, self.queue_size)

        # ------------------- 跟踪与性能统计状态 -------------------
        self._init_kalman_filter()
        self.kf_initialized = False
        self.missed_frames = 0
        self.last_target_class = ""
        self.frame_count = 0
        self.perf_acc = {
            'convert_ms': 0.0,
            'infer_ms': 0.0,
            'post_ms': 0.0,
            'publish_ms': 0.0,
            'total_ms': 0.0,
            'samples': 0,
        }
        self.profile_frame_count = 0

    def _load_parameters(self):
        """读取并解析 ROS 参数。"""
        self.image_topic = self.get_parameter('image_topic').value
        self.target_topic = self.get_parameter('target_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.queue_size = int(self.get_parameter('queue_size').value) 
        self.engine_file = self.get_parameter('engine_file').value
        self.debug_image_enabled = bool(self.get_parameter('debug_image_enabled').value)
        self.perf_log_enabled = bool(self.get_parameter('perf_log_enabled').value)
        self.perf_log_interval = max(1, int(self.get_parameter('perf_log_interval').value))
        self.perf_warmup_frames = max(0, int(self.get_parameter('perf_warmup_frames').value))
        self.pnp_topic = self.get_parameter('pnp_topic').value
        self.gimbal_cmd_topic = self.get_parameter('gimbal_cmd_topic').value
        self.armor_corner_enabled = bool(self.get_parameter('armor_corner_enabled').value)
        self.pnp_enabled = bool(self.get_parameter('pnp_enabled').value)
        self.opencv_corners_vis_enabled = bool(self.get_parameter('opencv_corners_vis_enabled').value)
        self.opencv_corners_vis_radius = max(1, int(self.get_parameter('opencv_corners_vis_radius').value))

        self.profile_mode = str(self.get_parameter('profile_mode').value).lower()
        self.profile_switch_interval_frames = max(1, int(self.get_parameter('profile_switch_interval_frames').value))
        self.profile_auto_brightness_low = float(self.get_parameter('profile_auto_brightness_low').value)
        self.profile_auto_brightness_high = float(self.get_parameter('profile_auto_brightness_high').value)

        self.roi_expand_ratio = float(self.get_parameter('roi_expand_ratio').value)
        self.light_binary_thresh = int(self.get_parameter('light_binary_thresh').value)
        self.light_min_area = float(self.get_parameter('light_min_area').value)
        self.light_min_aspect_ratio = float(self.get_parameter('light_min_aspect_ratio').value)
        self.light_max_aspect_ratio = float(self.get_parameter('light_max_aspect_ratio').value)
        self.light_max_angle_diff_deg = float(self.get_parameter('light_max_angle_diff_deg').value)
        self.light_max_center_y_diff_ratio = float(self.get_parameter('light_max_center_y_diff_ratio').value)
        self.light_min_pair_gap_ratio = float(self.get_parameter('light_min_pair_gap_ratio').value)
        self.light_max_pair_gap_ratio = float(self.get_parameter('light_max_pair_gap_ratio').value)

        self.profile_thresholds = {
            'indoor': {
                'light_binary_thresh': int(self.get_parameter('indoor_light_binary_thresh').value),
                'light_min_area': float(self.get_parameter('indoor_light_min_area').value),
                'light_min_aspect_ratio': float(self.get_parameter('indoor_light_min_aspect_ratio').value),
                'light_max_aspect_ratio': float(self.get_parameter('indoor_light_max_aspect_ratio').value),
                'light_max_angle_diff_deg': float(self.get_parameter('indoor_light_max_angle_diff_deg').value),
                'light_max_center_y_diff_ratio': float(self.get_parameter('indoor_light_max_center_y_diff_ratio').value),
                'light_min_pair_gap_ratio': float(self.get_parameter('indoor_light_min_pair_gap_ratio').value),
                'light_max_pair_gap_ratio': float(self.get_parameter('indoor_light_max_pair_gap_ratio').value),
            },
            'outdoor': {
                'light_binary_thresh': int(self.get_parameter('outdoor_light_binary_thresh').value),
                'light_min_area': float(self.get_parameter('outdoor_light_min_area').value),
                'light_min_aspect_ratio': float(self.get_parameter('outdoor_light_min_aspect_ratio').value),
                'light_max_aspect_ratio': float(self.get_parameter('outdoor_light_max_aspect_ratio').value),
                'light_max_angle_diff_deg': float(self.get_parameter('outdoor_light_max_angle_diff_deg').value),
                'light_max_center_y_diff_ratio': float(self.get_parameter('outdoor_light_max_center_y_diff_ratio').value),
                'light_min_pair_gap_ratio': float(self.get_parameter('outdoor_light_min_pair_gap_ratio').value),
                'light_max_pair_gap_ratio': float(self.get_parameter('outdoor_light_max_pair_gap_ratio').value),
            },
            'match': {
                'light_binary_thresh': int(self.get_parameter('match_light_binary_thresh').value),
                'light_min_area': float(self.get_parameter('match_light_min_area').value),
                'light_min_aspect_ratio': float(self.get_parameter('match_light_min_aspect_ratio').value),
                'light_max_aspect_ratio': float(self.get_parameter('match_light_max_aspect_ratio').value),
                'light_max_angle_diff_deg': float(self.get_parameter('match_light_max_angle_diff_deg').value),
                'light_max_center_y_diff_ratio': float(self.get_parameter('match_light_max_center_y_diff_ratio').value),
                'light_min_pair_gap_ratio': float(self.get_parameter('match_light_min_pair_gap_ratio').value),
                'light_max_pair_gap_ratio': float(self.get_parameter('match_light_max_pair_gap_ratio').value),
            },
        }

        self.camera_matrix = self._parse_camera_matrix(self.get_parameter('camera_matrix').value)
        self.dist_coeffs = self._parse_dist_coeffs(self.get_parameter('dist_coeffs').value)
        self.armor_width_m = float(self.get_parameter('armor_width_m').value)
        self.armor_height_m = float(self.get_parameter('armor_height_m').value)
        self.armor_width_small_m = float(self.get_parameter('armor_width_small_m').value)
        self.armor_height_small_m = float(self.get_parameter('armor_height_small_m').value)
        self.armor_width_large_m = float(self.get_parameter('armor_width_large_m').value)
        self.armor_height_large_m = float(self.get_parameter('armor_height_large_m').value)
        self.armor_size_auto_enabled = bool(self.get_parameter('armor_size_auto_enabled').value)
        self.armor_size_default = str(self.get_parameter('armor_size_default').value).lower()
        self.armor_size_ratio_threshold = float(self.get_parameter('armor_size_ratio_threshold').value)
        self.armor_class_size_map = self._parse_armor_class_size_pairs(self.get_parameter('armor_class_size_pairs').value)

        self.object_points_map = {
            'small': self._build_object_points(self.armor_width_small_m, self.armor_height_small_m),
            'large': self._build_object_points(self.armor_width_large_m, self.armor_height_large_m),
        }
        if self.armor_size_default not in self.object_points_map:
            self.armor_size_default = 'small'

        self.cam_to_gimbal_rot = self._parse_rpy_rotation(self.get_parameter('cam_to_gimbal_rpy_deg').value)
        self.cam_to_gimbal_trans = self._parse_xyz_vector(self.get_parameter('cam_to_gimbal_t_xyz_m').value)
        self.ballistic_enabled = bool(self.get_parameter('ballistic_enabled').value)
        self.projectile_speed_mps = max(1e-3, float(self.get_parameter('projectile_speed_mps').value))
        self.gravity_mps2 = float(self.get_parameter('gravity_mps2').value)
        self.yaw_zero_offset_deg = float(self.get_parameter('yaw_zero_offset_deg').value)
        self.pitch_zero_offset_deg = float(self.get_parameter('pitch_zero_offset_deg').value)
        
        self.kf_process_noise = float(self.get_parameter('kf_process_noise').value)
        self.kf_measurement_noise = float(self.get_parameter('kf_measurement_noise').value)
        self.max_missed_frames = int(self.get_parameter('kf_max_missed_frames').value)

        if self.profile_mode in self.profile_thresholds:
            self._apply_profile(self.profile_mode)
            self.active_profile = self.profile_mode
        elif self.profile_mode == 'auto':
            self.active_profile = 'auto'
        else:
            self.active_profile = 'manual'

    def _build_object_points(self, armor_width_m: float, armor_height_m: float):
        """根据装甲板宽高生成 PnP 四角 3D 点。"""
        return np.array([
            [-armor_width_m / 2.0,  armor_height_m / 2.0, 0.0],
            [ armor_width_m / 2.0,  armor_height_m / 2.0, 0.0],
            [ armor_width_m / 2.0, -armor_height_m / 2.0, 0.0],
            [-armor_width_m / 2.0, -armor_height_m / 2.0, 0.0],
        ], dtype=np.float32)

    def _parse_armor_class_size_pairs(self, pairs) -> Dict[str, str]:
        """解析类别到装甲板尺寸映射，格式: class_name:small|large。"""
        result: Dict[str, str] = {}
        for item in pairs:
            text = str(item).strip()
            if ':' not in text:
                continue
            key, value = text.split(':', 1)
            class_name = key.strip()
            size_text = value.strip().lower()
            if size_text not in ('small', 'large'):
                continue
            result[class_name] = size_text
        return result

    def _apply_profile(self, profile_name: str):
        """将选定 profile 的阈值应用到当前灯条参数。"""
        cfg = self.profile_thresholds.get(profile_name)
        if cfg is None:
            return
        self.light_binary_thresh = int(cfg['light_binary_thresh'])
        self.light_min_area = float(cfg['light_min_area'])
        self.light_min_aspect_ratio = float(cfg['light_min_aspect_ratio'])
        self.light_max_aspect_ratio = float(cfg['light_max_aspect_ratio'])
        self.light_max_angle_diff_deg = float(cfg['light_max_angle_diff_deg'])
        self.light_max_center_y_diff_ratio = float(cfg['light_max_center_y_diff_ratio'])
        self.light_min_pair_gap_ratio = float(cfg['light_min_pair_gap_ratio'])
        self.light_max_pair_gap_ratio = float(cfg['light_max_pair_gap_ratio'])

    def _update_profile_by_frame(self, cv_image):
        """按 profile_mode 选择/切换阈值配置。"""
        self.profile_frame_count += 1
        if self.profile_mode == 'manual':
            if self.active_profile != 'manual':
                self.active_profile = 'manual'
                self.get_logger().info('profile 切换: manual（使用基础阈值参数）')
            return

        if (self.profile_frame_count % self.profile_switch_interval_frames) != 0:
            return

        mode = self.profile_mode
        selected = mode
        if mode == 'auto':
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            mean_luma = float(np.mean(gray))
            if mean_luma < self.profile_auto_brightness_low:
                selected = 'indoor'
            elif mean_luma > self.profile_auto_brightness_high:
                selected = 'outdoor'
            else:
                selected = 'match'

        if selected not in self.profile_thresholds:
            return
        if selected != self.active_profile:
            self._apply_profile(selected)
            self.active_profile = selected
            self.get_logger().info(f'profile 切换: {selected}')

    def _estimate_armor_size(self, detection: Dict, corners: np.ndarray, pair_gap_ratio: float) -> str:
        """自动估计 small/large 装甲板尺寸。"""
        class_name = str(detection.get('class_name', ''))
        if class_name in self.armor_class_size_map:
            return self.armor_class_size_map[class_name]

        if not self.armor_size_auto_enabled:
            return self.armor_size_default

        top_w = float(np.linalg.norm(corners[1] - corners[0]))
        bottom_w = float(np.linalg.norm(corners[2] - corners[3]))
        left_h = float(np.linalg.norm(corners[3] - corners[0]))
        right_h = float(np.linalg.norm(corners[2] - corners[1]))

        mean_w = 0.5 * (top_w + bottom_w)
        mean_h = max(1e-6, 0.5 * (left_h + right_h))
        wh_ratio = mean_w / mean_h

        score = max(wh_ratio, pair_gap_ratio)
        return 'large' if score >= self.armor_size_ratio_threshold else 'small'

    def _init_kalman_filter(self):
        """初始化卡尔曼滤波器。"""
        self.kf = cv2.KalmanFilter(8, 4)
        # 状态转移矩阵 [x, y, w, h, dx, dy, dw, dh]
        self.kf.transitionMatrix = np.array([
            [1, 0, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 0, 0, 0, 1],
            [0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1]
        ], np.float32)
        # 测量矩阵 [x, y, w, h]
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0]
        ], np.float32)

        self.kf.processNoiseCov = np.eye(8, dtype=np.float32) * self.kf_process_noise
        self.kf.measurementNoiseCov = np.eye(4, dtype=np.float32) * self.kf_measurement_noise
        self.kf.errorCovPost = np.eye(8, dtype=np.float32)

    def _parse_camera_matrix(self, values):
        """将 9 元素列表转换为 3x3 相机内参矩阵。"""
        if len(values) != 9:
            raise ValueError('camera_matrix must contain 9 elements')
        return np.array(values, dtype=np.float32).reshape(3, 3)

    def _parse_dist_coeffs(self, values):
        """将畸变参数列表转换为 Nx1 列向量。"""
        if len(values) not in (4, 5, 8):
            raise ValueError('dist_coeffs must contain 4, 5 or 8 elements')
        return np.array(values, dtype=np.float32).reshape(-1, 1)

    def _parse_xyz_vector(self, values):
        """将 3 元素列表转换为平移向量。"""
        if len(values) != 3: 
            raise ValueError('cam_to_gimbal_t_xyz_m must contain 3 elements')
        return np.array(values, dtype=np.float32).reshape(3)

    def _parse_rpy_rotation(self, values):
        """将 roll/pitch/yaw（度）转换为旋转矩阵 R(gimbal<-camera)。"""
        if len(values) != 3:
            raise ValueError('cam_to_gimbal_rpy_deg must contain 3 elements')

        roll, pitch, yaw = [math.radians(float(v)) for v in values]

        rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0, math.cos(roll), -math.sin(roll)],
            [0.0, math.sin(roll), math.cos(roll)],
        ], dtype=np.float32)
        ry = np.array([
            [math.cos(pitch), 0.0, math.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-math.sin(pitch), 0.0, math.cos(pitch)],
        ], dtype=np.float32)
        rz = np.array([
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)
        return rz @ ry @ rx

    def _wrap_angle_deg(self, angle_deg: float) -> float:
        """将角度归一化到 [-180, 180)。"""
        wrapped = (angle_deg + 180.0) % 360.0 - 180.0
        return float(wrapped)

    def _compute_gimbal_cmd_from_pnp(self, pnp_result: Dict) -> Dict:
        """将相机系 PnP 结果变换为云台系误差角，并执行弹道补偿。"""
        tvec = pnp_result['tvec'].reshape(3).astype(np.float32)
        point_gimbal = self.cam_to_gimbal_rot @ tvec + self.cam_to_gimbal_trans

        xg, yg, zg = float(point_gimbal[0]), float(point_gimbal[1]), float(point_gimbal[2])
        horizontal = max(1e-6, math.sqrt(xg * xg + zg * zg))
        distance = float(math.sqrt(xg * xg + yg * yg + zg * zg))

        yaw_deg = math.degrees(math.atan2(xg, max(1e-6, zg)))
        pitch_deg = math.degrees(math.atan2(-yg, horizontal))

        if self.ballistic_enabled:
            flight_time = distance / self.projectile_speed_mps
            drop_m = 0.5 * self.gravity_mps2 * flight_time * flight_time
            pitch_deg = math.degrees(math.atan2(-yg + drop_m, horizontal))

        yaw_err_deg = self._wrap_angle_deg(yaw_deg + self.yaw_zero_offset_deg)
        pitch_err_deg = self._wrap_angle_deg(pitch_deg + self.pitch_zero_offset_deg)
        return {
            'yaw_err_deg': yaw_err_deg,
            'pitch_err_deg': pitch_err_deg,
            'distance_m': distance,
        }

    def _light_bar_from_contour(self, contour):
        """从单个轮廓中提取灯条几何信息，不满足约束时返回 None。"""
        area = cv2.contourArea(contour)
        if area < self.light_min_area:
            return None

        rect = cv2.minAreaRect(contour)
        (_, _), (w, h), _ = rect
        if w <= 1e-3 or h <= 1e-3:
            return None

        long_side = max(w, h)
        short_side = min(w, h)
        aspect = long_side / short_side
        if not (self.light_min_aspect_ratio <= aspect <= self.light_max_aspect_ratio):
            return None

        pts = cv2.boxPoints(rect).astype(np.float32)
        top_two = pts[np.argsort(pts[:, 1])[:2]]
        bottom_two = pts[np.argsort(pts[:, 1])[2:]]

        top = top_two.mean(axis=0)
        bottom = bottom_two.mean(axis=0)
        center = (top + bottom) * 0.5

        vec = bottom - top
        angle_deg = abs(math.degrees(math.atan2(vec[1], vec[0])))

        return {
            'top': top,
            'bottom': bottom,
            'center': center,
            'length': long_side,
            'angle_deg': angle_deg,
        }

    def _extract_armor_corners(self, frame, bbox):
        """在检测框 ROI 内提取装甲板四角点（左上/右上/右下/左下）。"""
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = bbox

        bw = max(1, x2 - x1)
        bh = max(1, y2 - y1)
        ex = int(bw * self.roi_expand_ratio)
        ey = int(bh * self.roi_expand_ratio)

        rx1 = max(0, x1 - ex)
        ry1 = max(0, y1 - ey)
        rx2 = min(w, x2 + ex)
        ry2 = min(h, y2 + ey)

        roi = frame[ry1:ry2, rx1:rx2]
        if roi.size == 0:
            return None

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, self.light_binary_thresh, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bars = []
        for contour in contours:
            bar = self._light_bar_from_contour(contour)
            if bar is not None:
                bars.append(bar)

        if len(bars) < 2:
            return None

        bars.sort(key=lambda b: b['center'][0])
        best_pair = None
        best_score = float('inf')

        for i in range(len(bars) - 1):
            for j in range(i + 1, len(bars)):
                left = bars[i]
                right = bars[j]

                mean_len = 0.5 * (left['length'] + right['length'])
                if mean_len <= 1e-3:
                    continue

                length_ratio = max(left['length'], right['length']) / max(1e-3, min(left['length'], right['length']))
                if length_ratio > 1.8:
                    continue

                y_diff = abs(left['center'][1] - right['center'][1])
                if y_diff > self.light_max_center_y_diff_ratio * mean_len:
                    continue

                x_gap = right['center'][0] - left['center'][0]
                gap_ratio = x_gap / mean_len
                if not (self.light_min_pair_gap_ratio <= gap_ratio <= self.light_max_pair_gap_ratio):
                    continue

                angle_diff = abs(left['angle_deg'] - right['angle_deg'])
                if angle_diff > self.light_max_angle_diff_deg:
                    continue

                # 分数越小越优：优先选择同高、等长且中心间距合理的一对灯条
                score = y_diff + abs(length_ratio - 1.0) * 8.0 + abs(gap_ratio - 2.5) * 4.0
                if score < best_score:
                    best_score = score
                    best_pair = (left, right)

        if best_pair is None:
            return None

        left, right = best_pair
        corners = np.array([
            [left['top'][0] + rx1, left['top'][1] + ry1],
            [right['top'][0] + rx1, right['top'][1] + ry1],
            [right['bottom'][0] + rx1, right['bottom'][1] + ry1],
            [left['bottom'][0] + rx1, left['bottom'][1] + ry1],
        ], dtype=np.float32)
        mean_len = max(1e-6, 0.5 * (left['length'] + right['length']))
        pair_gap_ratio = float((right['center'][0] - left['center'][0]) / mean_len)
        return {
            'corners': corners,
            'pair_gap_ratio': pair_gap_ratio,
        }

    def _solve_pnp(self, corners, armor_size: str):
        """根据四角点执行 PnP，输出 yaw/pitch/distance 与位姿向量。"""
        object_points = self.object_points_map.get(armor_size, self.object_points_map[self.armor_size_default])
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            return None

        tx, ty, tz = float(tvec[0, 0]), float(tvec[1, 0]), float(tvec[2, 0])
        distance = float(np.linalg.norm(tvec))
        yaw_deg = math.degrees(math.atan2(tx, max(1e-6, tz)))
        pitch_deg = math.degrees(math.atan2(-ty, max(1e-6, math.sqrt(tx * tx + tz * tz))))

        return {
            'yaw_deg': yaw_deg,
            'pitch_deg': pitch_deg,
            'distance_m': distance,
            'tvec': tvec,
            'rvec': rvec,
        }

    def _resolve_engine_path(self) -> str:
        """
        动态寻址 Engine 权重文件位置。
        优先寻找安装目录 (install/share)，如果未找到则回退到源码目录 (src)。
        
        Returns:
            str: 绝对路径形式的 engine 文件路径
        """
        package_share_directory = get_package_share_directory('rm_vision')
        install_engine_path = os.path.join(package_share_directory, 'weights', self.engine_file)
        
        if os.path.exists(install_engine_path):
            return install_engine_path

        # 回退策略：寻址当前源文件上两级的 weights 目录
        source_engine_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'weights',
            self.engine_file
        )
        if os.path.exists(source_engine_path):
            self.get_logger().warning(
                f"Engine file not found in install share, fallback to source tree: {source_engine_path}"
            )
            return source_engine_path

        raise FileNotFoundError(
            f"YOLO engine not found. Checked: {install_engine_path} and {source_engine_path}"
        )

    def _extract_best_detection(self, result):
        """
        从当前帧结果中提取置信度最高的目标。

        Returns:
            dict | None: 包含框坐标、中心点、类别和置信度；无目标时返回 None。
        """
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return None

        best_index = int(boxes.conf.argmax().item())
        x1, y1, x2, y2 = boxes.xyxy[best_index].tolist()
        class_id = int(boxes.cls[best_index].item())
        confidence = float(boxes.conf[best_index].item())
        names = result.names

        if isinstance(names, dict):
            class_name = names.get(class_id, str(class_id))
        else:
            class_name = names[class_id]

        return {
            'bbox': (int(x1), int(y1), int(x2), int(y2)),
            'center': (float((x1 + x2) / 2.0), float((y1 + y2) / 2.0)),
            'class_name': class_name,
            'confidence': confidence,
        }
    def _draw_debug_overlay(self, frame, detection):
        """
        在原始图像上叠加检测框、中心点和标签。
        """
        annotated_frame = frame.copy()

        if detection is not None:
            x1, y1, x2, y2 = detection['bbox']
            center_x, center_y = detection['center']
            
            is_pred = detection.get('is_predicted', False)
            if is_pred:
                color = (0, 165, 255) # 橙色作为卡尔曼滤波预测框
                label = f"{detection['class_name']} (pred)"
            else:
                color = (0, 255, 0)   # 绿色作为实际检测框
                label = f"{detection['class_name']} {detection['confidence']:.2f}"

            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
            cv2.circle(annotated_frame, (int(center_x), int(center_y)), 4, (0, 255, 255), -1)
            cv2.putText(
                annotated_frame,
                label,
                (x1, max(y1 - 10, 30)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
                cv2.LINE_AA,
            )

            corners = detection.get('armor_corners')
            if corners is not None and self.opencv_corners_vis_enabled:
                for idx, p in enumerate(corners.astype(int)):
                    cv2.circle(annotated_frame, tuple(p), self.opencv_corners_vis_radius, (255, 180, 0), -1)
                    cv2.putText(
                        annotated_frame,
                        str(idx),
                        (int(p[0]) + 3, int(p[1]) - 3),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (255, 180, 0),
                        1,
                        cv2.LINE_AA,
                    )

        return annotated_frame

    def _publish_debug_image(self, frame, header):
        """
        发布带有可视化叠加层的调试图像。
        """
        debug_image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        debug_image_msg.header = header
        self.debug_image_publisher.publish(debug_image_msg)

    def _run_detection_and_geometry(self, cv_image) -> Optional[Dict]:
        """运行模型推理，并在检测结果上追加角点与 PnP 信息。"""
        self._update_profile_by_frame(cv_image)

        results = self.model(cv_image, verbose=False)
        if not results:
            return None

        result = results[0]
        detection = self._extract_best_detection(result)
        if detection is None:
            return None

        if self.armor_corner_enabled:
            corners_info = self._extract_armor_corners(cv_image, detection['bbox'])
            if corners_info is not None:
                corners = corners_info['corners']
                detection['armor_corners'] = corners
                armor_size = self._estimate_armor_size(detection, corners, float(corners_info['pair_gap_ratio']))
                detection['armor_size'] = armor_size
                if self.pnp_enabled:
                    pnp_result = self._solve_pnp(corners, armor_size)
                    if pnp_result is not None:
                        detection['pnp'] = pnp_result
                        pnp_msg = Vector3()
                        pnp_msg.x = float(pnp_result['yaw_deg'])
                        pnp_msg.y = float(pnp_result['pitch_deg'])
                        pnp_msg.z = float(pnp_result['distance_m'])
                        self.pnp_publisher.publish(pnp_msg)

                        gimbal_cmd = self._compute_gimbal_cmd_from_pnp(pnp_result)
                        detection['gimbal_cmd'] = gimbal_cmd
                        gimbal_msg = GimbalCmd()
                        gimbal_msg.yaw_err_deg = float(gimbal_cmd['yaw_err_deg'])
                        gimbal_msg.pitch_err_deg = float(gimbal_cmd['pitch_err_deg'])
                        gimbal_msg.distance_m = float(gimbal_cmd['distance_m'])
                        gimbal_msg.valid = True
                        gimbal_msg.class_name = str(detection.get('class_name', ''))
                        self.gimbal_cmd_publisher.publish(gimbal_msg)

        return detection

    def _build_target_message_with_tracking(self, best_detection: Optional[Dict]):
        """融合卡尔曼滤波，构建目标发布消息并返回用于可视化的检测结果。"""
        target_msg = Target()

        if best_detection is not None:
            x1, y1, x2, y2 = best_detection['bbox']
            cx, cy = best_detection['center']
            w = float(x2 - x1)
            h = float(y2 - y1)
            self.last_target_class = best_detection['class_name']

            measurement = np.array([[np.float32(cx)], [np.float32(cy)], [np.float32(w)], [np.float32(h)]])

            if not self.kf_initialized:
                self.kf.statePost = np.array([
                    [np.float32(cx)], [np.float32(cy)], [np.float32(w)], [np.float32(h)],
                    [0.], [0.], [0.], [0.]
                ], np.float32)
                self.kf_initialized = True
            else:
                self.kf.predict()
                self.kf.correct(measurement)

            self.missed_frames = 0
            best_detection['is_predicted'] = False

            target_msg.x = float(cx)
            target_msg.y = float(cy)
            target_msg.class_name = self.last_target_class
            return target_msg, best_detection

        if self.kf_initialized and self.missed_frames < self.max_missed_frames:
            self.missed_frames += 1
            prediction = self.kf.predict()

            cx, cy, w, h = prediction[0, 0], prediction[1, 0], prediction[2, 0], prediction[3, 0]
            x1 = int(cx - w / 2)
            y1 = int(cy - h / 2)
            x2 = int(cx + w / 2)
            y2 = int(cy + h / 2)

            pred_detection = {
                'bbox': (x1, y1, x2, y2),
                'center': (cx, cy),
                'class_name': f"{self.last_target_class}_pred",
                'confidence': 0.0,
                'is_predicted': True
            }

            target_msg.x = float(cx)
            target_msg.y = float(cy)
            target_msg.class_name = self.last_target_class
            return target_msg, pred_detection

        self.kf_initialized = False
        target_msg.x = 0.0
        target_msg.y = 0.0
        target_msg.class_name = ""
        return target_msg, None

    def _update_performance_log(self, frame_start, convert_end, infer_end, post_end, publish_end):
        """累计并按间隔输出性能统计日志（平均时延与 FPS）。"""
        if not self.perf_log_enabled:
            return

        self.frame_count += 1
        if self.frame_count <= self.perf_warmup_frames:
            return

        convert_ms = (convert_end - frame_start) * 1000.0
        infer_ms = (infer_end - convert_end) * 1000.0
        post_ms = (post_end - infer_end) * 1000.0
        publish_ms = (publish_end - post_end) * 1000.0
        total_ms = (publish_end - frame_start) * 1000.0

        self.perf_acc['convert_ms'] += convert_ms
        self.perf_acc['infer_ms'] += infer_ms
        self.perf_acc['post_ms'] += post_ms
        self.perf_acc['publish_ms'] += publish_ms
        self.perf_acc['total_ms'] += total_ms
        self.perf_acc['samples'] += 1

        if self.perf_acc['samples'] % self.perf_log_interval != 0:
            return

        n = self.perf_acc['samples']
        avg_convert = self.perf_acc['convert_ms'] / n
        avg_infer = self.perf_acc['infer_ms'] / n
        avg_post = self.perf_acc['post_ms'] / n
        avg_publish = self.perf_acc['publish_ms'] / n
        avg_total = self.perf_acc['total_ms'] / n
        fps = 1000.0 / avg_total if avg_total > 0.0 else 0.0

        self.get_logger().info(
            (
                f"[PERF][avg:{n}] convert={avg_convert:.2f}ms, "
                f"infer={avg_infer:.2f}ms, "
                f"post={avg_post:.2f}ms, "
                f"publish={avg_publish:.2f}ms, "
                f"total={avg_total:.2f}ms, fps={fps:.2f}"
            )
        )

    def image_callback(self, msg: Image):
        """
        图像接收回调。执行：
        1. 格式转换 ROS Image -> OpenCV BGR。
        2. TensorRT YOLO 推理。
        3. 结果打包并发布。
        4. 输出带画框的可视化图像。
        """
        frame_start = time.perf_counter()

        # 1. ROS 图像转 OpenCV 格式 (BGR8)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        convert_end = time.perf_counter()

        # 2. 模型推理 + 几何解算
        best_detection = self._run_detection_and_geometry(cv_image)
        infer_end = time.perf_counter()

        # 3. 跟踪与消息打包
        target_msg, best_detection = self._build_target_message_with_tracking(best_detection)
        post_end = time.perf_counter()

        # 即使没有检测到目标也会发布空消息表示帧的处理结果 (x=0, y=0, class_name="")
        if not rclpy.ok():
            return
            
        self.publisher.publish(target_msg)

        # 当前帧无法给出角点/PnP时，主动发布无效云台指令，避免下游使用过期目标。
        gimbal_cmd = best_detection.get('gimbal_cmd') if best_detection is not None else None
        if gimbal_cmd is None:
            gimbal_msg = GimbalCmd()
            gimbal_msg.yaw_err_deg = 0.0
            gimbal_msg.pitch_err_deg = 0.0
            gimbal_msg.distance_m = 0.0
            gimbal_msg.valid = False
            gimbal_msg.class_name = ''
            self.gimbal_cmd_publisher.publish(gimbal_msg)

        # 5. 生成并发布 Debug 可视化图像
        if self.debug_image_enabled:
            annotated_frame = self._draw_debug_overlay(cv_image, best_detection)
            self._publish_debug_image(annotated_frame, msg.header)
        publish_end = time.perf_counter()

        self._update_performance_log(frame_start, convert_end, infer_end, post_end, publish_end)

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
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