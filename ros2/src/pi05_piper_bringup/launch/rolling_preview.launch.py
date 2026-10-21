from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("policy_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("policy_port", default_value="8021"),
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
                "inference_hz": 10.0,
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
                "update_period_ms": 100,
                "max_state_age_ms": 120,
                "max_chunk_age_ms": 250,
                "max_response_age_ms": 250,
                "max_observation_age_ms": 500,
                "max_inference_latency_ms": 500,
                "max_preview_window_s": 120.0,
            }],
        ),
    ])
