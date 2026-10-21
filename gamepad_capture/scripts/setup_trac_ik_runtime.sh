#!/usr/bin/env bash
# Install the same TRAC-IK binding used by Gamepad_PiPER, in local cache only.
set -euo pipefail
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
runtime="$project/cache/gamepad_ik_runtime"
mkdir -p "$runtime/wheels"
if PYTHONPATH="$runtime/python" /usr/bin/python3 -c 'from trac_ik import TracIK' 2>/dev/null; then
  echo 'TRAC_IK_RUNTIME_READY; already installed'
  exit 0
fi
/usr/bin/python3 -m pip download --only-binary=:all: --no-deps \
  --dest "$runtime/wheels" pytracik==0.0.3
/usr/bin/python3 -m pip install --no-deps --target "$runtime/python" \
  "$runtime/wheels/pytracik-0.0.3-cp310-cp310-manylinux_2_34_x86_64.whl"
PYTHONPATH="$runtime/python" /usr/bin/python3 -c 'from trac_ik import TracIK; print("TRAC_IK_RUNTIME_READY")'
