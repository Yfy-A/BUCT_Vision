import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('rm_serial'),
        'config',
        'serial_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='rm_serial',
            executable='serial_node',
            name='serial_node',
            parameters=[config_path],
            output='screen',
        )
    ])
