#!/usr/bin/env bash
# Build the gamepad adapter as an isolated overlay; does not open robot devices.
set -eo pipefail
PROJECT="${PI05_PROJECT_ROOT:-/mnt/e/01_NUS/02_5003/12_pi0.5}"
WORKSPACE="${PI05_GAMEPAD_WORKSPACE:-$HOME/pi05_gamepad_ws}"
QUEST_SOURCE="$PROJECT/quest_capture/ros2/src/pi05_piper_quest"
GAMEPAD_SOURCE="$PROJECT/gamepad_capture/ros2/src/pi05_piper_gamepad"

source "$PROJECT/quest_capture/scripts/quest_environment.sh"
rosdep install --from-paths "$QUEST_SOURCE" "$GAMEPAD_SOURCE" \
  --ignore-src --rosdistro humble -y --skip-keys ament_python
colcon --log-base "$WORKSPACE/log" build \
  --base-paths "$QUEST_SOURCE" "$GAMEPAD_SOURCE" \
  --build-base "$WORKSPACE/build" --install-base "$WORKSPACE/install" \
  --allow-overriding pi05_piper_quest \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
printf 'GAMEPAD_SOFTWARE_BUILD_COMPLETE: %s\n' "$WORKSPACE"
