"""Sample and wire contracts for the live gamepad dataset path.

The module keeps the data shape independent of ROS and LeRobot. In-memory
samples use NumPy arrays; the stdio boundary converts arrays to ordinary
MessagePack-compatible values and raw RGB bytes.
"""

from __future__ import annotations

import math
from typing import Any, Mapping

import numpy as np


STATE_SIZE = 7
ACTION_SIZE = 7
CAMERA_ROLES = ("base", "wrist")
MAX_IMAGE_BYTES = 16 * 1024 * 1024
MAX_MESSAGE_BYTES = 64 * 1024 * 1024


class ContractError(ValueError):
    """Raised when a source message cannot satisfy the sample contract."""


def _header_stamp_ns(message: Any) -> int:
    """Read the ROS Header stamp from a mapping or sensor message."""
    if isinstance(message, Mapping):
        if "header_stamp_ns" in message:
            value = message["header_stamp_ns"]
            if type(value) is not int:
                raise ContractError("header_stamp_ns must be an integer")
            return value
        header = message.get("header")
    else:
        header = getattr(message, "header", None)
    if header is None:
        raise ContractError("image has no header")
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        raise ContractError("image header has no stamp")
    seconds = int(getattr(stamp, "sec", 0))
    nanoseconds = int(getattr(stamp, "nanosec", 0))
    if not 0 <= nanoseconds < 1_000_000_000:
        raise ContractError("header nanoseconds are outside [0, 1e9)")
    return seconds * 1_000_000_000 + nanoseconds


def state_vector(state: Mapping[str, Any]) -> np.ndarray:
    """Build the float32 [7] state from six joints and nominal gripper meters."""
    joints = state.get("joint_position_rad")
    gripper = state.get("gripper_nominal_m")
    if not isinstance(joints, (list, tuple, np.ndarray)) or len(joints) != 6:
        raise ContractError("state must contain exactly six joint_position_rad values")
    values = np.asarray([*joints, gripper], dtype=np.float32)
    if values.shape != (STATE_SIZE,) or not np.isfinite(values).all():
        raise ContractError("state must be finite float32[7]")
    return values


def action_vector(values: Any) -> np.ndarray:
    """Validate a float32 [7] nominal target vector."""
    array = np.asarray(values, dtype=np.float32)
    if array.shape != (ACTION_SIZE,) or not np.isfinite(array).all():
        raise ContractError("action must be finite float32[7]")
    return array


def image_to_rgb(image: Any) -> np.ndarray:
    """Convert an Image-like value to an owned contiguous HWC RGB uint8 array.

    Supported encodings are the two encodings used by this project's camera
    source, ``bgr8`` and ``rgb8``. Row padding is honored through ``step``.
    """
    if isinstance(image, Mapping):
        height = int(image.get("height", 0))
        width = int(image.get("width", 0))
        encoding = str(image.get("encoding", ""))
        step = int(image.get("step", width * 3))
        data = image.get("data", b"")
    else:
        height = int(getattr(image, "height", 0))
        width = int(getattr(image, "width", 0))
        encoding = str(getattr(image, "encoding", ""))
        step = int(getattr(image, "step", width * 3))
        data = getattr(image, "data", b"")
    if height <= 0 or width <= 0 or encoding not in ("bgr8", "rgb8"):
        raise ContractError("image requires positive dimensions and bgr8 or rgb8 encoding")
    row_bytes = width * 3
    if step < row_bytes or step * height > MAX_IMAGE_BYTES:
        raise ContractError("image step or byte size exceeds the supported limit")
    try:
        pixels = np.frombuffer(data, dtype=np.uint8)
    except (TypeError, ValueError) as exc:
        raise ContractError("image data must expose a byte buffer") from exc
    if pixels.size != step * height:
        raise ContractError("image data length does not equal step * height")
    rows = pixels.reshape(height, step)
    packed = rows[:, :row_bytes].reshape(height, width, 3)
    if encoding == "bgr8":
        packed = packed[:, :, ::-1]
    result = np.ascontiguousarray(packed, dtype=np.uint8)
    if result.nbytes > MAX_IMAGE_BYTES:
        raise ContractError("decoded RGB image exceeds the supported limit")
    return result


def validate_rgb_image(image: Any) -> np.ndarray:
    """Validate and copy one in-memory RGB image."""
    array = np.asarray(image)
    if array.dtype != np.uint8 or array.ndim != 3 or array.shape[2] != 3:
        raise ContractError("image must be HWC RGB uint8")
    if array.shape[0] <= 0 or array.shape[1] <= 0 or array.nbytes > MAX_IMAGE_BYTES:
        raise ContractError("image dimensions or byte size exceed the supported limit")
    return np.ascontiguousarray(array).copy()


def _plain(value: Any) -> Any:
    """Convert nested values to the MessagePack standard type subset."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if isinstance(value, (str, bytes, int, float, bool)) or value is None:
        if isinstance(value, float) and not math.isfinite(value):
            raise ContractError("wire payload cannot contain non-finite floats")
        return value
    raise ContractError(f"unsupported wire value type: {type(value).__name__}")


def sample_to_wire(sample: Mapping[str, Any]) -> dict[str, Any]:
    """Return the shared writer envelope payload with byte-packed RGB images."""
    state = np.asarray(sample.get("state"), dtype=np.float32)
    action = np.asarray(sample.get("action"), dtype=np.float32)
    if state.shape != (STATE_SIZE,) or action.shape != (ACTION_SIZE,):
        raise ContractError("wire sample state/action must each have seven values")
    if not np.isfinite(state).all() or not np.isfinite(action).all():
        raise ContractError("wire sample state/action must be finite")
    images: dict[str, dict[str, Any]] = {}
    raw_image_bytes = 0
    sample_images = sample.get("images")
    if not isinstance(sample_images, Mapping):
        raise ContractError("wire sample images must be a mapping")
    for role in CAMERA_ROLES:
        rgb = validate_rgb_image(sample_images.get(role))
        packed = rgb.tobytes(order="C")
        raw_image_bytes += len(packed)
        images[role] = {
            "encoding": "rgb8",
            "shape": [int(rgb.shape[0]), int(rgb.shape[1]), 3],
            "data": packed,
        }
    camera_valid = sample.get("camera_valid")
    if not isinstance(camera_valid, (list, tuple, np.ndarray)) or len(camera_valid) != 2:
        raise ContractError("camera_valid must contain [base, wrist]")
    if raw_image_bytes > MAX_MESSAGE_BYTES:
        raise ContractError("sample image payload exceeds the stdio message limit")
    return {
        "state": state.tolist(),
        "action": action.tolist(),
        "images": images,
        "action_valid": bool(sample.get("action_valid")),
        "observation_valid": bool(sample.get("observation_valid")),
        "camera_valid": [bool(camera_valid[0]), bool(camera_valid[1])],
        "task": str(sample.get("task", "")),
        "run_id": str(sample.get("run_id", "")),
        "source_map": _plain(sample.get("source_map", {})),
    }


def wire_envelope_sample(sample: Mapping[str, Any]) -> dict[str, Any]:
    """Wrap a normalized sample for the stdio protocol."""
    return {
        "type": "sample",
        "schema": "piper_live_sample_v1",
        "sample": sample_to_wire(sample),
    }


def image_header_stamp_ns(message: Any) -> int:
    """Public wrapper for reading an Image Header timestamp."""
    return _header_stamp_ns(message)
