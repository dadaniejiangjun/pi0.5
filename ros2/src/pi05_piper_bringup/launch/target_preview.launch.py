from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("allow_fake_observations", default_value="false"),
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
