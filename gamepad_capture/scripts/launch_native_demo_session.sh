#!/usr/bin/env bash
# One bounded lifecycle for cameras, manual control, executor, and recording.
set -eo pipefail
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
[[ "${1:-}" == --onsite-confirmed ]] || { echo 'Requires current onsite confirmation' >&2; exit 2; }
if pgrep -af '/lib/(pi05_piper_bridge/native_(can_state|manual_gamepad|camera_metadata)|pi05_piper_gamepad/gamepad_operator|pi05_piper_quest/quest_execution)( |$)'; then
  echo 'Existing source/control nodes must finish first' >&2; exit 2
fi
export PI05_GAMEPAD_SENSOR_SOURCE=native
run_id=$(date -u +%Y%m%dT%H%M%S)_$$
scope="demo_$run_id"
logs="$project/cache/gamepad_native_sessions/$run_id"
mkdir -p "$logs"
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT INT TERM
setsid bash "$project/gamepad_capture/scripts/launch_native_capture_sources.sh" \
  --cameras-only wrist_pixel_format:=mjpeg2rgb >"$logs/cameras.log" 2>&1 &
pids+=("$!")
setsid bash "$project/gamepad_capture/scripts/launch_gamepad_capture.sh" \
  input_source:=xinput execute:=true task_scope_id:="$scope" >"$logs/executor.log" 2>&1 &
pids+=("$!")
setsid bash "$project/gamepad_capture/scripts/launch_native_manual_control.sh" \
  --onsite-confirmed "$scope" >"$logs/control.log" 2>&1 &
pids+=("$!")
echo "SESSION_START: scope=$scope logs=$logs; start Windows XInput sender now"
sleep 4
cd "$project"
setsid bash "$project/gamepad_capture/scripts/record_gamepad_demo.sh" \
  --operator "${PI05_OPERATOR:-86153}" --duration 20 --lead-in 20 --startup-timeout 35 &
recorder_pid=$!
pids+=("$recorder_pid")
wait "$recorder_pid"
