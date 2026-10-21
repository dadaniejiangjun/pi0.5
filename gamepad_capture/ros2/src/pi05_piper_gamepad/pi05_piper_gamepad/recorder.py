"""ROS entry point for fixed-duration and manually ended capture episodes."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import sys


EXTRA_TOPICS = (
    "/joy",
    "/gamepad/input_status",
    "/gamepad/target_preview",
    "/gamepad/operator_enabled",
    "/gamepad/target_events",
    "/gamepad/execution_events",
    "/piper/command_receipt",
    "/gamepad/command_send_events",
)


def main() -> int:
    project_value = os.environ.get("PI05_PROJECT_ROOT")
    if not project_value:
        raise RuntimeError("PI05_PROJECT_ROOT must point to the checked-out project")
    project_root = Path(project_value).resolve()
    recorder_path = project_root / "ros2" / "scripts" / "record_operator_demo_bag.py"
    if not recorder_path.is_file():
        raise FileNotFoundError(f"shared ROS demonstration recorder not found: {recorder_path}")

    module_spec = importlib.util.spec_from_file_location(
        "pi05_shared_operator_demo_recorder", recorder_path
    )
    if module_spec is None or module_spec.loader is None:
        raise RuntimeError("could not load the shared ROS demonstration recorder")
    shared_recorder = importlib.util.module_from_spec(module_spec)
    sys.path.insert(0, str(project_root))
    try:
        module_spec.loader.exec_module(shared_recorder)
        package_data = project_root / "gamepad_capture" / "data"
        defaults = [
            "--archive-root", str(package_data / "rosbag"),
            "--report-root", str(package_data / "manifests"),
        ]
        topics = list(EXTRA_TOPICS)
        if "--manual-lifecycle" in sys.argv:
            topics.append("/gamepad/recording_lifecycle")
        if os.environ.get("PI05_GAMEPAD_SENSOR_SOURCE", "windows") == "native":
            defaults.extend((
                "--base-device-name", os.environ.get(
                    "PI05_GAMEPAD_BASE_DEVICE", "D435i SDK serial:_262422074776"),
                "--wrist-device-name", os.environ.get(
                    "PI05_GAMEPAD_WRIST_DEVICE",
                    "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Dabai_DC1_CC1T35300DK-video-index0"),
            ))
        for topic in topics:
            defaults.extend(("--extra-topic", topic))
        sys.argv = [sys.argv[0], *defaults, *sys.argv[1:]]
        return int(shared_recorder.main())
    finally:
        try:
            sys.path.remove(str(project_root))
        except ValueError:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
