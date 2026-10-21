#!/usr/bin/env bash
# Replace the native receive-only CAN node; never run both owners together.
set -eo pipefail
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source /opt/ros/humble/setup.bash
source "${PI05_GAMEPAD_NATIVE_OVERLAY:-$HOME/pi05_native_can_ws/install}/local_setup.bash"
export PYTHONPATH="$project/ros2/src/pi05_piper_bridge:$project${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-222}"
if [[ "${1:-}" == --check-environment ]]; then
  /usr/bin/python3 -c 'import pi05_piper_bridge.native_manual_gamepad; print("NATIVE_MANUAL_IMPORT_PASS; no hardware opened")'
  exit 0
fi
[[ "${1:-}" == --onsite-confirmed && "${2:-}" =~ ^[A-Za-z0-9_-]{1,128}$ ]] || {
  echo 'Requires --onsite-confirmed SCOPE after verifying ownership, neutral sticks, mode, enables, clear space and E-stop.' >&2
  exit 2
}
if pgrep -af '/lib/pi05_piper_bridge/native_(can_state|manual_gamepad)( |$)'; then
  echo 'Stop/replace the existing CAN owner first. Cameras may keep running.' >&2
  exit 2
fi
run_id=$(date -u +%Y%m%dT%H%M%S)_$$
duration="${3:-600}"
[[ "$duration" =~ ^[0-9]+$ ]] && (( duration >= 1 && duration <= 3600 )) || {
  echo 'Duration must be 1..3600 seconds' >&2; exit 2;
}
mkdir -p "$project/cache/gamepad_native_control/$run_id"
echo "MANUAL_CONTROL_DURATION: ${duration}s; expires after this duration"
exec timeout --signal=INT --kill-after=8s "$((duration+5))s" ros2 run pi05_piper_bridge native_manual_gamepad --ros-args \
  -p interface:=can0 -p log_path:="$project/cache/gamepad_native_control/$run_id/can.jsonl" \
  -p state_publish_hz:=20.0 \
  -p enable_manual_tx:=true -p task_scope_id:="$2" -p duration_s:="$duration" \
  -p operator_approved:=true -p physical_estop_ready:=true \
  -p low_speed_confirmed:=true -p motion_envelope_cleared:=true
