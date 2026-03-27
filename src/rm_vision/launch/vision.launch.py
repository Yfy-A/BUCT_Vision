import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 获取相机配置文件的安装路径
    config_path = os.path.join(
        get_package_share_directory('rm_vision'),
        'config',
        'camera_params.yaml'
    )
    
    # 1. 运行系统环境中的视频流节点
    video_node = Node(
        package='rm_vision',
        executable='video_node',
        name='video_node',
        parameters=[config_path],
        output='screen'
    )

    # 2. 运行虚拟环境中的自瞄检测节点 (绕过 ros2 run 限制)
    # 使用 ExecuteProcess 调用 bash 脚本，先激活 conda 环境，再执行源码中的 Python 脚本
    conda_sh_path = os.path.expanduser('~/miniconda3/etc/profile.d/conda.sh')
    detector_script_path = os.path.expanduser('~/RM_WS/src/rm_vision/rm_vision/detector_node.py')
    
    detector_command = (
        f"source {conda_sh_path} && "
        f"conda activate sam_env2 && "
        f"python3 {detector_script_path}"
    )

    detector_node = ExecuteProcess(
        cmd=['bash', '-c', detector_command],
        output='screen'
    )

    return LaunchDescription([
        video_node,
        detector_node
    ])
