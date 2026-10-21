from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value=EnvironmentVariable(
            "PI05_ROS_GATEWAY_HOST", default_value="")),
        DeclareLaunchArgument("control_port", default_value=EnvironmentVariable(
            "PI05_ROS_CONTROL_PORT", default_value="18765")),
        DeclareLaunchArgument("base_image_port", default_value=EnvironmentVariable(
            "PI05_ROS_BASE_IMAGE_PORT", default_value="18766")),
        DeclareLaunchArgument("wrist_image_port", default_value=EnvironmentVariable(
            "PI05_ROS_WRIST_IMAGE_PORT", default_value="18767")),
        DeclareLaunchArgument("image_publish_hz", default_value=EnvironmentVariable(
            "PI05_ROS_IMAGE_PUBLISH_HZ", default_value="30.0")),
        Node(
            package="pi05_piper_bridge",
            executable="read_only_bridge",
            name="pi05_read_only_bridge",
            output="screen",
            parameters=[{
                "host": LaunchConfiguration("host"),
                "control_port": LaunchConfiguration("control_port"),
                "base_image_port": LaunchConfiguration("base_image_port"),
                "wrist_image_port": LaunchConfiguration("wrist_image_port"),
                "image_publish_hz": ParameterValue(
                    LaunchConfiguration("image_publish_hz"), value_type=float
                ),
            }],
        ),
    ])
