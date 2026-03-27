import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('mvcc_camera_ros2'),
        'config',
        'hik_camera_config.yaml'
    )

    hik_camera_node = Node(
        package='mvcc_camera_ros2',
        executable='hik_camera_image_pub',
        name='mvcc_camera_image_publisher',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        hik_camera_node
    ])
