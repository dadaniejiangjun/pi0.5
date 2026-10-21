#!/usr/bin/env bash
set -eo pipefail
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/pi05_gamepad_${UID}.lock"
if ! flock -n 9; then
  echo '已有手柄入口运行 / Gamepad launch already running. Close its Execute window first.' >&2
  exit 2
fi
if owners=$(pgrep -af '/lib/pi05_piper_gamepad/(gamepad_operator|gamepad_xinput_ingress)( |$)'); then
  echo '检测到旧手柄节点，拒绝重复启动 / Existing gamepad nodes:' >&2
  echo "$owners" >&2
  exit 2
fi
source "$(dirname "${BASH_SOURCE[0]}")/gamepad_environment.sh"
if [[ "${PI05_GAMEPAD_IK_BACKEND:-builtin}" == trac_ik ]]; then
  /usr/bin/python3 -c 'from trac_ik import TracIK; print("TRAC_IK_RUNTIME_READY; no device access")'
fi
if [[ "${1:-}" == --check-environment ]]; then exit 0; fi
set -- "kinematics_backend:=${PI05_GAMEPAD_IK_BACKEND:-builtin}" "$@"
exec ros2 launch pi05_piper_gamepad gamepad_capture.launch.py "$@"
