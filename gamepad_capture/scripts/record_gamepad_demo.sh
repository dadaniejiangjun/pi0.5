#!/usr/bin/env bash
# Legacy/default recorder backend: rosbag via the existing ROS 2 recorder.
# LeRobot capture is launched by gamepad_teleop.ps1 -RecorderBackend LeRobot
# so the pinned Windows LeRobot runtime remains the sole dataset writer.
set -eo pipefail
PROJECT="${PI05_PROJECT_ROOT:-/mnt/e/01_NUS/02_5003/12_pi0.5}"
source "$PROJECT/gamepad_capture/scripts/gamepad_environment.sh"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT/ros2/config/fastdds_large_images.xml"
GAMEPAD_WORKSPACE="${PI05_GAMEPAD_WORKSPACE:-$HOME/pi05_gamepad_ws}"
if [[ ! -f "$GAMEPAD_WORKSPACE/install/local_setup.bash" ]]; then
  echo "Gamepad overlay is missing; run gamepad_teleop.ps1 -Mode Deploy first." >&2
  exit 2
fi
# Keep the current P2 interface overlay selected by gamepad_environment.sh.
# Re-sourcing the old p6 underlay here silently replaces message type support.
exec ros2 run pi05_piper_gamepad gamepad_demo_recorder "$@"
