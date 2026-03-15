#!/bin/bash

# ==================================================
#    🚀 一键启动视觉系统 (混合环境变量隔离版)
# ==================================================
# 使用说明：
# ./start_vision_python.sh          -> 默认启动海康真实相机模式
# ./start_vision_python.sh video    -> 启动本地视频推流测试模式
# ==================================================

MODE=${1:-camera}  # 默认模式为 camera
PID_CAMERA=""
PID_DETECTOR=""
PID_SERIAL=""

echo "=================================================="
if [ "$MODE" = "video" ]; then
    echo "    🚀 正在以 [视频流测试] 模式启动系统"
else
    echo "    🚀 正在以 [海康真实相机] 模式启动系统"
fi
echo "=================================================="

# 获取当前工作空间完整路径
WS_DIR="$HOME/RM_WS"

# 清理 Python 相关环境变量，避免系统环境与 Conda/用户 site-packages 互相污染
COMMON_CLEAN_ENV='unset PYTHONPATH PYTHONHOME; export PYTHONNOUSERSITE=1'

# 捕获 Ctrl+C 信号，确保可以一键关闭全部节点（相机/视频、检测、串口）
trap "echo -e '\n[!] 正在终止所有节点...'; kill -9 $PID_CAMERA $PID_DETECTOR $PID_SERIAL 2>/dev/null; exit 0" SIGINT SIGTERM

# -------------------------------------------------------------
# 1. 在系统默认环境中启动 Camera / Video Node
# -------------------------------------------------------------
if [ "$MODE" = "video" ]; then
    echo "[1/3] 正在系统环境下启动 video_node (视频流测试)..."
    bash -c "
        ${COMMON_CLEAN_ENV}

        source /opt/ros/humble/setup.bash
        source ${WS_DIR}/install/setup.bash
        ros2 run rm_vision video_node --ros-args --params-file ${WS_DIR}/src/rm_vision/config/camera_params.yaml
    " &
    PID_CAMERA=$!
else
    echo "[1/3] 正在系统环境下启动 mvcc_camera_ros2 (hik_camera_image_pub)..."
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
# 2. 在 Conda (sam_env2) 环境中启动 Detector Node
# -------------------------------------------------------------
echo "[2/3] 正在 Conda 虚拟环境(sam_env2)下启动 detector_node..."

# 根据运行模式，动态覆盖 detector_node 订阅的话题
# 视频模式: /camera/image_raw
# 相机模式: /mvcc_camera/Image
if [ "$MODE" = "video" ]; then
    TOPIC_REMAP="--ros-args --params-file ${WS_DIR}/src/rm_vision/config/detector_params.yaml"
else
    TOPIC_REMAP="--ros-args --params-file ${WS_DIR}/src/rm_vision/config/detector_params.yaml -p image_topic:=/mvcc_camera/Image"
fi

bash -c "
	# 初始化 Conda
	if [ -f \"$HOME/miniconda3/etc/profile.d/conda.sh\" ]; then
		source \"$HOME/miniconda3/etc/profile.d/conda.sh\"
	elif [ -f \"$HOME/anaconda3/etc/profile.d/conda.sh\" ]; then
		source \"$HOME/anaconda3/etc/profile.d/conda.sh\"
	fi

    # 先清理，再激活，避免继承到上层 shell 的 Python 路径
    ${COMMON_CLEAN_ENV}
	conda activate sam_env2
    ${COMMON_CLEAN_ENV}

	source /opt/ros/humble/setup.bash
	source ${WS_DIR}/install/setup.bash

	# 绕过 ros2 run，直接使用 conda 的 python3 运行节点
	python3 ${WS_DIR}/src/rm_vision/rm_vision/detector_node.py $TOPIC_REMAP
" &
PID_DETECTOR=$!

# -------------------------------------------------------------
# 3. 在系统默认环境中启动 Serial Node
# -------------------------------------------------------------
echo "[3/3] 正在系统环境下启动 serial_node (rm_serial)..."

# serial_node 由系统 /usr/bin/python3 运行；依赖检查必须使用同一解释器，避免环境差异导致误报。
if ! bash -c "
    ${COMMON_CLEAN_ENV}
    source /opt/ros/humble/setup.bash
    /usr/bin/python3 -c 'import serial'
" >/dev/null 2>&1; then
    echo "[ERROR] 系统 Python 缺少 pyserial，无法启动 rm_serial。"
    echo "[FIX] 请先安装: sudo apt update && sudo apt install -y python3-serial"
    echo "[INFO] 将终止已启动的 camera/video 与 detector 进程。"
    kill -9 $PID_CAMERA $PID_DETECTOR 2>/dev/null
    exit 1
fi

bash -c "
    ${COMMON_CLEAN_ENV}

    source /opt/ros/humble/setup.bash
    source ${WS_DIR}/install/setup.bash

    # 使用 rm_serial 的独立参数文件；其中可配置端口(如 /dev/ttyTHS1)和映射规则
    ros2 run rm_serial serial_node --ros-args --params-file ${WS_DIR}/src/rm_serial/config/serial_params.yaml
" &
PID_SERIAL=$!

echo "=================================================="
echo " ✅ 启动完成！按 Ctrl+C 可一键关闭全部节点。"
echo "=================================================="

# 挂起主脚本等待子进程，避免立即退出
wait $PID_CAMERA $PID_DETECTOR $PID_SERIAL
