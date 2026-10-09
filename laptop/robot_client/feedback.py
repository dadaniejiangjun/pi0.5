"""Pure PiPER feedback content; aggregation, SDK parsing and freshness stay callers."""
from dataclasses import dataclass
import struct


@dataclass(frozen=True)
class DecodedPiperFeedback:
    arbitration_id: int
    received_monotonic_ns: int
    fields: dict


def decode_feedback_frame(arbitration_id, data, received_monotonic_ns):
    """Decode content using the former PiperSession expressions exactly.

    Callers retain frame flag/length filtering. This is not a full snapshot,
    authorization, validation gate, or accumulator. Unknown IDs return None.
    """
    if arbitration_id == 0x2A1:
        fields = dict(ctrl_mode_raw=data[0], arm_status_raw=data[1],
                      move_mode_raw=data[2], teach_status_raw=data[3],
                      error_code=int.from_bytes(data[6:8], "big"))
    elif arbitration_id in (0x2A5, 0x2A6, 0x2A7):
        fields = dict(joint_pair=struct.unpack(">ii", data))
    elif arbitration_id == 0x2A8:
        fields = dict(position_raw=int.from_bytes(data[:4], "big", signed=True),
                      effort_raw=int.from_bytes(data[4:6], "big", signed=True),
                      status_raw=data[6])
    else:
        return None
    return DecodedPiperFeedback(arbitration_id, received_monotonic_ns, fields)


def sdk_feedback_snapshot(message, *, gripper_details=False):
    """Project vendor-parser accumulated fields; do not replace DecodeMessage.

    Preserve SDK enum string rendering and missing-channel zero state. Raw
    frame decoding above and SDK accumulation are deliberately distinct.
    """
    gripper = {"angle_or_stroke": message.gripper_feedback.grippers_angle}
    if gripper_details:
        gripper.update(effort=message.gripper_feedback.grippers_effort,
                       status_code=message.gripper_feedback.status_code)
    return {
        "joint_raw": [getattr(message.arm_joint_feedback, f"joint_{i}")
                      for i in range(1, 7)],
        "gripper_raw": gripper,
        "status": {
            "ctrl_mode": str(message.arm_status_msgs.ctrl_mode),
            "arm_status": str(message.arm_status_msgs.arm_status),
            "teach_status": str(message.arm_status_msgs.teach_status),
            "err_code": message.arm_status_msgs.err_code,
        },
    }
