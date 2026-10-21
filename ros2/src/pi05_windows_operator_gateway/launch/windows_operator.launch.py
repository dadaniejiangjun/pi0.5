from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("http_host", default_value="auto"),
        DeclareLaunchArgument("http_port", default_value="18771"),
        DeclareLaunchArgument("allowed_client_ip", default_value="172.27.64.1"),
        DeclareLaunchArgument("control_mode", default_value="disabled"),
        DeclareLaunchArgument("image_preview_hz", default_value="15.0"),
        Node(
            package="pi05_windows_operator_gateway",
            executable="windows_operator_gateway",
            name="windows_operator_gateway",
            output="screen",
            parameters=[{
                "http_host": LaunchConfiguration("http_host"),
                "http_port": LaunchConfiguration("http_port"),
                "allowed_client_ip": LaunchConfiguration("allowed_client_ip"),
                "control_mode": LaunchConfiguration("control_mode"),
                "image_preview_hz": LaunchConfiguration("image_preview_hz"),
                "state_topic": "/piper/state_snapshot",
                "base_image_topic": "/native/cameras/base/image_raw",
                "wrist_image_topic": "/native/cameras/wrist/image_raw",
                "camera_metadata_topic": "/piper/cameras/frame_metadata",
            }],
        ),
    ])
