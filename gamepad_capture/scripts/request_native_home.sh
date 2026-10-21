#!/usr/bin/env bash
set -euo pipefail

project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
# ROS 2 setup scripts inspect optional AMENT/COLCON trace variables. Keep
# nounset for this wrapper's own logic, but disable it while sourcing.
set +u
source /opt/ros/humble/setup.bash
source "${PI05_GAMEPAD_NATIVE_OVERLAY:-$HOME/pi05_native_can_ws/install}/local_setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-222}"
exec /usr/bin/python3 "$project/gamepad_capture/scripts/request_native_home.py" "$@"
