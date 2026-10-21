"""Read Xbox input and display standardized Joy messages; no robot topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("joy_device_name", default_value=""),
        Node(
            package="joy",
            executable="game_controller_node",
            name="xbox_joy_input_preview",
            parameters=[{
                "device_name": LaunchConfiguration("joy_device_name"),
                "deadzone": 0.0,
                "autorepeat_rate": 20.0,
                "sticky_buttons": False,
            }],
            output="screen",
        ),
        Node(
            package="pi05_piper_gamepad",
            executable="gamepad_input_echo",
            name="gamepad_input_echo",
            output="screen",
        ),
    ])
