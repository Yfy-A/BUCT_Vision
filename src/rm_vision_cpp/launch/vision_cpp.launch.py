import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rm_vision_cpp")
    mode = LaunchConfiguration("mode")

    camera_config = os.path.join(pkg_share, "config", "camera_params.yaml")
    detector_config = os.path.join(pkg_share, "config", "detector_params.yaml")

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="video",
        description="运行模式: video 或 camera",
    )

    mode_is_video = PythonExpression(["'", mode, "' == 'video'"])
    mode_is_camera = PythonExpression(["'", mode, "' == 'camera'"])

    video_node = Node(
        package="rm_vision_cpp",
        executable="video_node",
        name="video_node",
        parameters=[camera_config],
        output="screen",
        condition=IfCondition(mode_is_video),
    )

    hik_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mvcc_camera_ros2"),
                "launch",
                "hik_camera.launch.py",
            )
        ),
        condition=IfCondition(mode_is_camera),
    )

    detector_video_node = Node(
        package="rm_vision_cpp",
        executable="detector_node",
        name="detector_node",
        parameters=[detector_config],
        output="screen",
        condition=IfCondition(mode_is_video),
    )

    detector_camera_node = Node(
        package="rm_vision_cpp",
        executable="detector_node",
        name="detector_node",
        parameters=[detector_config, {"image_topic": "/mvcc_camera/Image"}],
        output="screen",
        condition=UnlessCondition(mode_is_video),
    )

    return LaunchDescription(
        [
            mode_arg,
            video_node,
            hik_camera_launch,
            detector_video_node,
            detector_camera_node,
        ]
    )
