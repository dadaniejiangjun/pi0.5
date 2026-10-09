"""Pure PiPER joint-command preview; there is deliberately no CAN sender.

The V2 interface transmits all six targets across 0x155-0x157. A proposed
single-joint step therefore must validate the other five held targets too.
Passing this encoder is NOT permission to enable or move a real PiPER.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Sequence

from laptop.robot_client.replay_preflight import NOMINAL_JOINT_BOUNDS_DEG


@dataclass(frozen=True)
class CanFramePreview:
    arbitration_id: int
    data: bytes


@dataclass(frozen=True)
class JointStepPreview:
    source_deg: tuple[float, ...]
    target_deg: tuple[float, ...]
    frames: tuple[CanFramePreview, ...]
    hardware_control_allowed: bool = False


def _joints(value: Sequence[float]) -> tuple[float, ...]:
    if (not isinstance(value, (tuple, list)) or len(value) != 6
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in value)):
        raise ValueError("six finite joint angles in degrees required")
    return tuple(float(x) for x in value)


def preview_single_joint_step(
    current_deg: Sequence[float], *, joint_index: int, delta_deg: float,
    max_step_deg: float = 0.2,
) -> JointStepPreview:
    """Reject uncertain start/target values, then encode a non-executable preview."""
    current = _joints(current_deg)
    if (type(joint_index) is not int or not 0 <= joint_index < 6
            or type(delta_deg) not in (int, float) or not math.isfinite(delta_deg)
            or type(max_step_deg) not in (int, float) or not math.isfinite(max_step_deg)
            or max_step_deg <= 0 or not 0 < abs(delta_deg) <= min(max_step_deg, 0.2)):
        raise ValueError("one bounded nonzero joint step required")
    if any(not lo <= angle <= hi for angle, (lo, hi) in
           zip(current, NOMINAL_JOINT_BOUNDS_DEG)):
        raise ValueError("current joint outside official nominal bounds")
    target = list(current)
    target[joint_index] += float(delta_deg)
    if any(not lo <= angle <= hi for angle, (lo, hi) in
           zip(target, NOMINAL_JOINT_BOUNDS_DEG)):
        raise ValueError("target joint outside official nominal bounds")
    # Explicit integer rounding follows the SDK's 0.001-degree command unit.
    encoded = tuple(round(angle * 1000) for angle in target)
    quantized = tuple(raw / 1000 for raw in encoded)
    if any(not lo <= angle <= hi for angle, (lo, hi) in
           zip(quantized, NOMINAL_JOINT_BOUNDS_DEG)):
        raise ValueError("quantized target outside official nominal bounds")
    frames = tuple(
        CanFramePreview(0x155 + pair, struct.pack(">ii", *encoded[2 * pair:2 * pair + 2]))
        for pair in range(3)
    )
    return JointStepPreview(current, quantized, frames)
