#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/gamepad_environment.sh"
base_port=0
wrist_port=0
if [[ "${1:-}" == "--cameras" ]]; then
  base_port=18766
  wrist_port=18767
  shift
fi
exec ros2 launch pi05_piper_bringup read_only.launch.py \
  host:="$PI05_ROS_GATEWAY_HOST" control_port:=18768 \
  base_image_port:="$base_port" wrist_image_port:="$wrist_port" image_publish_hz:=30.0 "$@"
