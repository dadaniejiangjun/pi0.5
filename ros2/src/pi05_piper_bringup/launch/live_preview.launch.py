"""Start read-only Windows gateway, policy shadow, and rolling preview nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gateway_host", default_value=EnvironmentVariable(
            "PI05_ROS_GATEWAY_HOST", default_value="")),
        DeclareLaunchArgument("control_port", default_value=EnvironmentVariable(
            "PI05_ROS_CONTROL_PORT", default_value="18765")),
        DeclareLaunchArgument("base_image_port", default_value=EnvironmentVariable(
            "PI05_ROS_BASE_IMAGE_PORT", default_value="18766")),
        DeclareLaunchArgument("wrist_image_port", default_value=EnvironmentVariable(
            "PI05_ROS_WRIST_IMAGE_PORT", default_value="18767")),
        DeclareLaunchArgument("image_publish_hz", default_value=EnvironmentVariable(
            "PI05_ROS_IMAGE_PUBLISH_HZ", default_value="30.0")),
        DeclareLaunchArgument("policy_host", default_value=EnvironmentVariable(
            "PI05_POLICY_HOST", default_value="127.0.0.1")),
        DeclareLaunchArgument("policy_port", default_value=EnvironmentVariable(
            "PI05_POLICY_PORT", default_value="8021")),
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
        DeclareLaunchArgument("enable_prediction_only_action", default_value="false"),
        Node(
            package="pi05_piper_bridge",
            executable="read_only_bridge",
            name="pi05_read_only_bridge",
            output="screen",
            parameters=[{
                "host": LaunchConfiguration("gateway_host"),
                "control_port": ParameterValue(
                    LaunchConfiguration("control_port"), value_type=int
                ),
                "base_image_port": ParameterValue(
                    LaunchConfiguration("base_image_port"), value_type=int
                ),
                "wrist_image_port": ParameterValue(
                    LaunchConfiguration("wrist_image_port"), value_type=int
                ),
                "image_publish_hz": ParameterValue(
                    LaunchConfiguration("image_publish_hz"), value_type=float
                ),
            }],
        ),
        Node(
            package="pi05_piper_runtime",
            executable="policy_shadow",
            name="pi05_policy_shadow",
            output="screen",
            parameters=[{
                "policy_host": LaunchConfiguration("policy_host"),
                "policy_port": ParameterValue(LaunchConfiguration("policy_port"), value_type=int),
                "identity_lock_path": LaunchConfiguration("identity_lock_path"),
                "allow_fake_observations": ParameterValue(
                    LaunchConfiguration("allow_fake_observations"), value_type=bool
                ),
                "connect_timeout_s": 5.0,
                "request_timeout_s": 15.0,
                "inference_hz": ParameterValue(
                    EnvironmentVariable("PI05_ROS_POLICY_HZ", default_value="10"),
                    value_type=float,
                ),
            }],
        ),
        Node(
            package="pi05_piper_runtime",
            executable="target_preview",
            name="pi05_target_preview",
            output="screen",
            parameters=[{
                "allow_fake_observations": ParameterValue(
                    LaunchConfiguration("allow_fake_observations"), value_type=bool
                ),
                "max_joint_step_deg": 20.0,
                "speed_percent": 5,
                "update_period_ms": ParameterValue(
                    EnvironmentVariable("PI05_ROS_TARGET_PERIOD_MS", default_value="100"),
                    value_type=int,
                ),
                "max_state_age_ms": 120,
                "max_chunk_age_ms": 250,
                "max_response_age_ms": 250,
                "max_observation_age_ms": 500,
                "max_inference_latency_ms": 500,
                "max_preview_window_s": 120.0,
            }],
        ),
        Node(
            package="pi05_piper_runtime",
            executable="run_task_action",
            name="pi05_run_task_action",
            output="screen",
            condition=IfCondition(
                LaunchConfiguration("enable_prediction_only_action")
            ),
            parameters=[{"execution_mode": "prediction_only"}],
        ),
    ])
