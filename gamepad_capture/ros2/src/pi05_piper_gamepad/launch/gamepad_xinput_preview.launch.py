"""Read Windows XInput frames via local TCP and echo standardized ROS Joy."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="18901"),
        DeclareLaunchArgument("timeout_ms", default_value="250"),
        Node(
            package="pi05_piper_gamepad",
            executable="gamepad_xinput_ingress",
            name="gamepad_xinput_ingress",
            parameters=[{
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "timeout_ms": LaunchConfiguration("timeout_ms"),
            }],
            output="screen",
        ),
        Node(
            package="pi05_piper_gamepad",
            executable="gamepad_input_echo",
            name="gamepad_xinput_input_echo",
            output="screen",
        ),
    ])
