#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/gamepad_environment.sh"
exec ros2 launch pi05_piper_gamepad gamepad_input_preview.launch.py "$@"
