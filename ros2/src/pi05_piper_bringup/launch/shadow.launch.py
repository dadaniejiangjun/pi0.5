from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("policy_host", default_value=EnvironmentVariable(
            "PI05_POLICY_HOST", default_value="127.0.0.1")),
        DeclareLaunchArgument("policy_port", default_value=EnvironmentVariable(
            "PI05_POLICY_PORT", default_value="8021")),
        DeclareLaunchArgument("policy_backend", default_value="openpi"),
        DeclareLaunchArgument(
            "identity_lock_path",
            default_value=EnvironmentVariable(
                "PI05_POLICY_IDENTITY_LOCK_PATH",
                default_value=(
                    "/mnt/e/01_NUS/02_5003/12_pi0.5/"
                    "validation/live5000_identity_20261008.json"
                ),
            ),
        ),
        DeclareLaunchArgument("allow_fake_observations", default_value="false"),
        DeclareLaunchArgument("inference_hz", default_value=EnvironmentVariable(
            "PI05_ROS_POLICY_HZ", default_value="10.0")),
        DeclareLaunchArgument("connect_timeout_s", default_value="5.0"),
        DeclareLaunchArgument("request_timeout_s", default_value="15.0"),
        Node(
            package="pi05_piper_runtime",
            executable="policy_shadow",
            name="pi05_policy_shadow",
            output="screen",
            parameters=[{
                "policy_host": LaunchConfiguration("policy_host"),
                "policy_port": ParameterValue(LaunchConfiguration("policy_port"), value_type=int),
                "policy_backend": LaunchConfiguration("policy_backend"),
                "identity_lock_path": LaunchConfiguration("identity_lock_path"),
                "allow_fake_observations": ParameterValue(
                    LaunchConfiguration("allow_fake_observations"), value_type=bool
                ),
                "connect_timeout_s": ParameterValue(
                    LaunchConfiguration("connect_timeout_s"), value_type=float
                ),
                "request_timeout_s": ParameterValue(
                    LaunchConfiguration("request_timeout_s"), value_type=float
                ),
                "inference_hz": ParameterValue(
                    LaunchConfiguration("inference_hz"), value_type=float
                ),
            }],
        ),
    ])
