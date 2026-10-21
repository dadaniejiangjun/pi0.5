"""Xbox gamepad teleoperation, preview, supervised execution and recording input."""

from datetime import datetime, timezone
from pathlib import Path
import os
import uuid

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _input_source_nodes(context, *args, **kwargs):
    input_source = LaunchConfiguration("input_source").perform(context).strip().lower()
    if input_source == "sdl":
        return [
            Node(
                package="joy",
                executable="game_controller_node",
                name="xbox_joy",
                parameters=[{
                    "device_name": LaunchConfiguration("joy_device_name"),
                    "deadzone": 0.0,
                    "autorepeat_rate": 20.0,
                }],
                output="screen",
            )
        ]
    if input_source == "xinput":
        return [
            Node(
                package="pi05_piper_gamepad",
                executable="gamepad_xinput_ingress",
                name="gamepad_xinput_ingress",
                parameters=[{
                    "host": LaunchConfiguration("xinput_host"),
                    "port": ParameterValue(
                        LaunchConfiguration("xinput_port"), value_type=int
                    ),
                    "timeout_ms": ParameterValue(
                        LaunchConfiguration("xinput_timeout_ms"), value_type=int
                    ),
                }],
                output="screen",
            )
        ]
    raise RuntimeError("input_source must be 'sdl' or 'xinput'")


def generate_launch_description():
    package_share = Path(get_package_share_directory("pi05_piper_gamepad"))
    project = Path(os.environ.get(
        "PI05_PROJECT_ROOT", "/mnt/e/01_NUS/02_5003/12_pi0.5"
    ))
    log_dir = project / "gamepad_capture" / "data" / "execution"
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log_path = str(log_dir / f"{stamp}_{uuid.uuid4().hex[:8]}.jsonl")

    return LaunchDescription([
        DeclareLaunchArgument(
            "input_source",
            default_value="sdl",
            description="Use the WSL SDL device or the Windows XInput ROS ingress.",
        ),
        DeclareLaunchArgument("joy_device_name", default_value=""),
        DeclareLaunchArgument("xinput_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("xinput_port", default_value="18901"),
        DeclareLaunchArgument("xinput_timeout_ms", default_value="250"),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("joint_target_smoothing", default_value="true"),
        DeclareLaunchArgument("task_scope_id", default_value=""),
        DeclareLaunchArgument("kinematics_backend", default_value="builtin"),
        OpaqueFunction(function=_input_source_nodes),
        Node(
            package="pi05_piper_gamepad",
            executable="gamepad_operator",
            name="gamepad_operator",
            parameters=[str(package_share / "config" / "xbox_mapping.yaml"),
                        {"input_source": LaunchConfiguration("input_source"),
                         "kinematics_backend": LaunchConfiguration("kinematics_backend")}],
            output="screen",
        ),
        Node(
            package="pi05_piper_quest",
            executable="quest_execution",
            name="gamepad_execution",
            parameters=[{
                "input_source": "gamepad",
                "target_topic": "/gamepad/target_preview",
                "operator_enabled_topic": "/gamepad/operator_enabled",
                "execution_event_topic": "/gamepad/execution_events",
                "execute": ParameterValue(LaunchConfiguration("execute"), value_type=bool),
                "task_scope_id": LaunchConfiguration("task_scope_id"),
                "log_path": log_path,
                "joint_step_deg": 5.0,
                "joint_target_smoothing": ParameterValue(LaunchConfiguration("joint_target_smoothing"), value_type=bool),
            }],
            output="screen",
        ),
    ])
