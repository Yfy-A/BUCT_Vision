#!/bin/bash

# ==================================================
#    一键启动视觉系统 (C++ 版本)
# ==================================================
# 使用说明：
# ./start_vision_pro.sh          -> 默认启动海康真实相机模式
# ./start_vision_pro.sh video    -> 启动本地视频推流测试模式
# ==================================================

MODE=${1:-camera}  # 默认模式为 camera

echo "=================================================="
if [ "$MODE" = "video" ]; then
    echo "    正在以 [视频流测试] 模式启动系统 (C++)"
else
    echo "    正在以 [海康真实相机] 模式启动系统 (C++)"
fi
echo "=================================================="

# 获取当前工作空间完整路径
WS_DIR="$HOME/RM_WS"

# 清理 Python 相关环境变量，避免系统环境与 Conda/用户 site-packages 互相污染
COMMON_CLEAN_ENV='unset PYTHONPATH PYTHONHOME; export PYTHONNOUSERSITE=1'

# 捕获 Ctrl+C 信号，确保可以一键关闭相关进程
trap "echo -e '\n[!] 正在终止所有节点...'; kill $PID_CAMERA $PID_DETECTOR 2>/dev/null; wait $PID_CAMERA $PID_DETECTOR 2>/dev/null; exit 0" SIGINT SIGTERM

# -------------------------------------------------------------
# 1. 在系统默认环境中启动 Camera / Video Node
# -------------------------------------------------------------
if [ "$MODE" = "video" ]; then
    echo "[1/2] 正在系统环境下启动 rm_vision_cpp/video_node (视频流测试)..."
    bash -c "
        ${COMMON_CLEAN_ENV}
        source /opt/ros/humble/setup.bash
        source ${WS_DIR}/install/setup.bash
        ros2 run rm_vision_cpp video_node --ros-args --params-file ${WS_DIR}/src/rm_vision_cpp/config/camera_params.yaml
    " &
    PID_CAMERA=$!
else
    echo "[1/2] 正在系统环境下启动 mvcc_camera_ros2 (hik_camera_image_pub)..."
    bash -c "
        ${COMMON_CLEAN_ENV}
        source /opt/ros/humble/setup.bash
        source ${WS_DIR}/install/setup.bash
        ros2 launch mvcc_camera_ros2 hik_camera.launch.py
    " &
    PID_CAMERA=$!
fi

# 等待2秒确保相机正常启动
sleep 2 

# -------------------------------------------------------------
# 2. 在系统环境中启动 C++ Detector Node
# -------------------------------------------------------------
echo "[2/2] 正在启动 rm_vision_cpp/detector_node..."

# 根据运行模式，动态覆盖 detector_node 订阅的话题
# 视频模式: /camera/image_raw
# 相机模式: /mvcc_camera/Image
if [ "$MODE" = "video" ]; then
    TOPIC_REMAP="--ros-args --params-file ${WS_DIR}/src/rm_vision_cpp/config/detector_params.yaml"
else
    TOPIC_REMAP="--ros-args --params-file ${WS_DIR}/src/rm_vision_cpp/config/detector_params.yaml -p image_topic:=/mvcc_camera/Image"
fi

bash -c "
    ${COMMON_CLEAN_ENV}
    source /opt/ros/humble/setup.bash
    source ${WS_DIR}/install/setup.bash
    ros2 run rm_vision_cpp detector_node $TOPIC_REMAP
" &
PID_DETECTOR=$!

echo "=================================================="
echo " 启动完成！按 Ctrl+C 可一键关闭全部节点。"
echo "=================================================="

# 挂起主脚本等待子进程，避免立即退出
wait $PID_CAMERA $PID_DETECTOR
