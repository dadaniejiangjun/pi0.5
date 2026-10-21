#!/usr/bin/env bash
# Reuse the source environment proven by P4_ARRAY_PAYLOAD_20261015_r1.
# Starts receive-only CAN and cameras, never a policy or robot command node.
set -eo pipefail
cameras_only=false
if [[ "${1:-}" == --cameras-only ]]; then
  cameras_only=true
  shift
fi
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source "$project/ros2/scripts/cameras/camera_environment.sh"
source "${PI05_GAMEPAD_NATIVE_OVERLAY:-$HOME/pi05_native_can_ws/install}/local_setup.bash"
export PYTHONPATH="$project/ros2/src/pi05_piper_bridge:$project/ros2/src/pi05_piper_runtime:$project${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-222}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$project/ros2/config/fastdds_large_images.xml"
/usr/bin/python3 - "$project" <<'PY'
import hashlib, json, pathlib, sys
import pi05_piper_bridge.native_camera_metadata as camera
source = pathlib.Path(camera.__file__).resolve()
expected = pathlib.Path(sys.argv[1], 'ros2/src/pi05_piper_bridge').resolve()
if expected not in source.parents:
    raise RuntimeError(f'Camera adapter must load project source: {source}')
print(json.dumps({'status': 'NATIVE_CAPTURE_SOURCE_ENVIRONMENT_READY',
                  'module': str(source),
                  'module_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                  'camera_publish_hz_requested': 30.0,
                  'can_tx_requested': False}, ensure_ascii=False), flush=True)
PY
for package in pi05_piper_bridge realsense2_camera usb_cam; do
  ros2 pkg prefix "$package" >/dev/null
done
if [[ "${1:-}" == --check-environment ]]; then
  exit 0
fi
pattern='/lib/pi05_piper_bridge/native_(can_state|manual_gamepad|camera_metadata)( |$)'
if $cameras_only; then pattern='/lib/pi05_piper_bridge/native_camera_metadata( |$)'; fi
if existing_nodes=$(pgrep -af "$pattern"); then
  printf 'Existing native input nodes must be reused, not duplicated:\n%s\n' "$existing_nodes" >&2
  exit 2
fi
# Same ownership preflight as P4; never steal an existing camera consumer.
/usr/bin/python3 "$project/ros2/scripts/cameras/check_native_camera_owners.py" | /usr/bin/python3 -c '
import json, sys
report = json.load(sys.stdin)
print(json.dumps(report))
if report.get("idle") is not True:
    raise SystemExit("Camera owner present or uncertain; existing sources must be reused.")
'
ip link show can0 >/dev/null
run_id=$(date -u +%Y%m%dT%H%M%S)_$$
log_dir="${PI05_NATIVE_LOG_DIR:-$project/cache/gamepad_native_sources/$run_id}"
mkdir -p "$log_dir"
source_duration="${PI05_GAMEPAD_SOURCE_DURATION_S:-0}"
[[ "$source_duration" =~ ^[0-9]+$ ]] && (( source_duration == 0 || (source_duration >= 30 && source_duration <= 3600) )) \
  || { echo 'Source duration must be 0 (until Ctrl+C) or 30..3600 seconds' >&2; exit 2; }
runner=()
if (( source_duration > 0 )); then
  runner=(timeout --signal=INT --kill-after=8s "${source_duration}s")
else
  printf 'Camera/source session stays running until Ctrl+C; reuse it across episodes.\n'
fi
printf 'Native source logs: %s; duration=%s seconds (0 means until Ctrl+C)\n' "$log_dir" "$source_duration"
exec "${runner[@]}" \
  ros2 launch "$project/ros2/launch/native_sensor_inputs.launch.py" \
  can_log:="$log_dir/can.jsonl" camera_log:="$log_dir/camera_timing.jsonl" \
  can_enabled:="$([[ "$cameras_only" == true ]] && echo false || echo true)" \
  image_publish_hz:=30.0 "$@"
