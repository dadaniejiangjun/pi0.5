"""Small auditable JSON store for operator-saved feedback poses."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
from typing import Any


SCHEMA = "piper_gamepad_pose_presets_v1"
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))


class PosePresetStore:
    def __init__(self, path: str | Path):
        self.path = Path(path).expanduser().resolve()
        self.presets: list[dict[str, Any]] = []
        if self.path.exists():
            self.presets = self._read()

    def _read(self) -> list[dict[str, Any]]:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise ValueError(f"cannot read pose preset store: {exc}") from exc
        if not isinstance(data, dict) or data.get("schema") != SCHEMA:
            raise ValueError("unsupported pose preset store schema")
        values = data.get("presets")
        if not isinstance(values, list):
            raise ValueError("pose preset store has no preset list")
        for index, preset in enumerate(values, start=1):
            self._validate(preset, expected_id=f"pose-{index:04d}")
        return values

    @staticmethod
    def _validate(preset: Any, *, expected_id: str | None = None) -> None:
        if not isinstance(preset, dict):
            raise ValueError("pose preset must be an object")
        if expected_id is not None and preset.get("preset_id") != expected_id:
            raise ValueError("pose preset ids must be sequential and auditable")
        joints = preset.get("joint_position_rad")
        if (not isinstance(joints, list) or len(joints) != 6
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in joints)):
            raise ValueError("pose preset must retain six finite joint radians")
        if type(preset.get("gripper_raw")) is not int:
            raise ValueError("pose preset must retain integer gripper raw feedback")
        for key in ("created_at_utc", "source_session_id", "coordinate_frame"):
            if not isinstance(preset.get(key), str) or not preset[key]:
                raise ValueError(f"pose preset is missing {key}")
        sequence = preset.get("source_state_sequence")
        if type(sequence) is not int or sequence < 0:
            raise ValueError("pose preset source_state_sequence must be non-negative")

    def save_feedback(self, *, joints_rad: tuple[float, ...], gripper_raw: int,
                      session_id: str, state_sequence: int,
                      coordinate_frame: str = "base_link") -> dict[str, Any]:
        if len(joints_rad) != 6 or any(not math.isfinite(float(v)) for v in joints_rad):
            raise ValueError("six finite joint feedback values are required")
        if type(gripper_raw) is not int or gripper_raw < 0:
            raise ValueError("gripper feedback must be a non-negative integer raw value")
        if not session_id or type(state_sequence) is not int or state_sequence < 0:
            raise ValueError("session and feedback sequence are required")
        if not coordinate_frame:
            raise ValueError("coordinate frame is required")
        preset = {
            "preset_id": f"pose-{len(self.presets) + 1:04d}",
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "source_session_id": session_id,
            "source_state_sequence": state_sequence,
            "coordinate_frame": coordinate_frame,
            "joint_names": list(JOINT_NAMES),
            "joint_position_rad": [float(value) for value in joints_rad],
            "gripper_raw": gripper_raw,
        }
        self._validate(preset)
        next_presets = self.presets + [preset]
        payload = json.dumps({"schema": SCHEMA, "presets": next_presets},
                             allow_nan=False, indent=2) + "\n"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(self.path.name + ".tmp")
        try:
            temporary.write_text(payload, encoding="utf-8", newline="\n")
            os.replace(temporary, self.path)
        finally:
            if temporary.exists():
                temporary.unlink()
        self.presets = next_presets
        return preset

    def cycle(self, selected_index: int) -> int:
        if not self.presets:
            raise ValueError("no saved pose presets")
        return (selected_index + 1) % len(self.presets)

    def selected(self, selected_index: int) -> dict[str, Any]:
        if not self.presets:
            raise ValueError("no saved pose presets")
        if not 0 <= selected_index < len(self.presets):
            raise ValueError("selected pose preset index is invalid")
        preset = self.presets[selected_index]
        self._validate(preset)
        return preset
