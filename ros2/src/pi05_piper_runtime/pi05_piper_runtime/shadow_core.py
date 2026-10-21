"""ROS-independent input validation and observation assembly for policy shadow."""
from __future__ import annotations

import math
import numbers
from typing import Any

import numpy as np


MAX_STATE_AGE_NS = 120_000_000
MAX_IMAGE_AGE_NS = 250_000_000
MAX_FUTURE_STAMP_NS = 20_000_000
MAX_JOINT_SKEW_NS = 20_000_000
# The deployment plan's initial camera-pair reference is 100 ms. This is a
# host-arrival tolerance for prediction-only observations, not exposure sync.
MAX_CAMERA_SKEW_NS = 100_000_000
MAX_SENSOR_SKEW_NS = 250_000_000
MAX_CLOCK_UNCERTAINTY_NS = 20_000_000
MAX_IMAGE_BYTES = 8_000_000
LOCAL_MONOTONIC_CLOCK_ID = "wsl_perf_counter_ns"


class ObservationRejected(ValueError):
    """Raised when one ROS snapshot is incomplete, stale, or misaligned."""


def _field(value: Any, name: str) -> Any:
    if isinstance(value, dict):
        return value[name]
    return getattr(value, name)


def _stamp_ns(header: Any) -> int:
    stamp = _field(header, "stamp")
    sec = _field(stamp, "sec")
    nanosec = _field(stamp, "nanosec")
    if (isinstance(sec, bool) or isinstance(nanosec, bool)
            or not isinstance(sec, numbers.Integral)
            or not isinstance(nanosec, numbers.Integral)):
        raise ObservationRejected("ROS stamp fields must be integers")
    sec, nanosec = int(sec), int(nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ObservationRejected("ROS stamp is outside its valid range")
    return sec * 1_000_000_000 + nanosec


def _require_int(value: Any, name: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Integral) or value < minimum:
        raise ObservationRejected(f"{name} must be an integer >= {minimum}")
    return int(value)


def _image_array(image: Any, metadata: Any, role: str) -> tuple[np.ndarray, int, int]:
    if _field(image, "encoding") != "bgr8":
        raise ObservationRejected(f"{role} image encoding must be bgr8")
    width = _require_int(_field(image, "width"), f"{role}.width", minimum=1)
    height = _require_int(_field(image, "height"), f"{role}.height", minimum=1)
    step = _require_int(_field(image, "step"), f"{role}.step", minimum=1)
    expected_step = width * 3
    if step != expected_step:
        raise ObservationRejected(f"{role} image step does not match packed bgr8 width")
    if width * height * 3 > MAX_IMAGE_BYTES:
        raise ObservationRejected(f"{role} image exceeds the configured byte bound")
    if (_field(metadata, "width") != width or _field(metadata, "height") != height
            or _field(metadata, "encoding") != "bgr8"):
        raise ObservationRejected(f"{role} image and metadata shape/encoding differ")
    data = bytes(_field(image, "data"))
    if len(data) != height * step:
        raise ObservationRejected(f"{role} image payload length does not match its shape")
    bgr = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
    return np.ascontiguousarray(bgr[:, :, ::-1]), width, height


def _check_ros_age(now_ros_ns: int, stamp_ns: int, maximum_age_ns: int, name: str) -> int:
    age_ns = now_ros_ns - stamp_ns
    if age_ns < -MAX_FUTURE_STAMP_NS:
        raise ObservationRejected(f"{name} ROS timestamp is too far in the future")
    if age_ns > maximum_age_ns:
        raise ObservationRejected(f"stale {name} ROS timestamp")
    return max(0, age_ns)


def build_shadow_observation(
    *,
    state: Any,
    base_image: Any,
    base_metadata: Any,
    wrist_image: Any,
    wrist_metadata: Any,
    now_ros_ns: int,
    now_monotonic_ns: int,
    previous_sequences: dict[str, int] | None = None,
    allow_fake: bool = False,
    max_camera_skew_ns: int = MAX_CAMERA_SKEW_NS,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Validate a same-session sensor set and produce the existing PiPER input."""
    now_ros_ns = _require_int(now_ros_ns, "now_ros_ns", minimum=1)
    now_monotonic_ns = _require_int(now_monotonic_ns, "now_monotonic_ns", minimum=1)
    max_camera_skew_ns = _require_int(
        max_camera_skew_ns, "max_camera_skew_ns", minimum=1
    )
    try:
        session_id = _field(state, "session_id")
        state_sequence = _require_int(_field(state, "sequence"), "state.sequence", minimum=1)
        if not isinstance(session_id, str) or not session_id:
            raise ObservationRejected("state session_id is missing")
        if not _field(state, "feedback_fresh"):
            raise ObservationRejected("state feedback is not fresh")
        fake = _field(state, "fake")
        connected = _field(state, "hardware_connected")
        if type(fake) is not bool or type(connected) is not bool:
            raise ObservationRejected("state hardware flags are malformed")
        if fake and not allow_fake:
            raise ObservationRejected("fake gateway input is disabled for this profile")
        if not fake and not connected:
            raise ObservationRejected("hardware gateway is not connected")
        if _field(state, "error_code") != 0:
            raise ObservationRejected("robot reports a nonzero fault code")

        state_stamp_ns = _stamp_ns(_field(state, "header"))
        state_age_ns = _check_ros_age(
            now_ros_ns, state_stamp_ns, MAX_STATE_AGE_NS, "state"
        )
        state_source_ns = _require_int(
            _field(state, "source_monotonic_ns"), "state.source_monotonic_ns", minimum=1
        )
        if _field(state, "source_clock_id") not in ("windows_perf_counter_ns", "linux_perf_counter_ns"):
            raise ObservationRejected("state source timestamp uses an unsupported clock")
        state_local_ns = _require_int(
            _field(state, "source_local_monotonic_ns"),
            "state.source_local_monotonic_ns", minimum=1,
        )
        if _field(state, "source_local_clock_id") != LOCAL_MONOTONIC_CLOCK_ID:
            raise ObservationRejected("state source timestamp is not mapped to the WSL monotonic clock")
        state_uncertainty_ns = _require_int(
            _field(state, "source_clock_mapping_uncertainty_ns"),
            "state.source_clock_mapping_uncertainty_ns",
        )
        if state_uncertainty_ns > MAX_CLOCK_UNCERTAINTY_NS:
            raise ObservationRejected("state clock mapping uncertainty exceeds 20 ms")
        if (_field(state, "source_clock_id") == "linux_perf_counter_ns"
                and (state_local_ns != state_source_ns or state_uncertainty_ns != 0)):
            raise ObservationRejected("native state must use an identity local clock mapping")
        state_local_age_ns = now_monotonic_ns - state_local_ns
        if state_local_age_ns < -state_uncertainty_ns:
            raise ObservationRejected("state mapped monotonic timestamp is in the future")
        if state_local_age_ns > MAX_STATE_AGE_NS + state_uncertainty_ns:
            raise ObservationRejected("stale state source timestamp in the WSL monotonic clock")
        joint_skew_ns = _require_int(_field(state, "joint_skew_ns"), "joint_skew_ns")
        if joint_skew_ns > MAX_JOINT_SKEW_NS:
            raise ObservationRejected("joint feedback skew exceeds 20 ms")
        joint_pairs = list(_field(state, "joint_pair_monotonic_ns"))
        motor_times = list(_field(state, "motor_monotonic_ns"))
        if len(joint_pairs) != 3 or len(motor_times) != 6:
            raise ObservationRejected("state channel timestamp arrays have invalid lengths")
        channel_times = [
            _require_int(_field(state, "status_monotonic_ns"), "status_monotonic_ns", minimum=1),
            *[_require_int(item, "joint_pair_monotonic_ns[]", minimum=1) for item in joint_pairs],
            _require_int(_field(state, "gripper_monotonic_ns"), "gripper_monotonic_ns", minimum=1),
            *[_require_int(item, "motor_monotonic_ns[]", minimum=1) for item in motor_times],
        ]
        if max(channel_times) - min(channel_times) > MAX_SENSOR_SKEW_NS:
            raise ObservationRejected("CAN feedback channel span exceeds 250 ms")

        role_inputs = (
            ("base", base_image, base_metadata),
            ("wrist", wrist_image, wrist_metadata),
        )
        images: dict[str, np.ndarray] = {}
        source_times: dict[str, int] = {"state": state_source_ns}
        local_source_times: dict[str, int] = {"state": state_local_ns}
        ros_ages: dict[str, int] = {"state": state_age_ns}
        camera_sequences: dict[str, int] = {}
        for role, image, metadata in role_inputs:
            if _field(metadata, "role") != role:
                raise ObservationRejected(f"{role} metadata role mismatch")
            if _field(metadata, "session_id") != session_id:
                raise ObservationRejected(f"{role} metadata belongs to a different gateway session")
            if _field(metadata, "source_clock_id") != _field(state, "source_clock_id"):
                raise ObservationRejected(f"{role} metadata uses an unsupported source clock")
            uncertainty_ns = _require_int(
                _field(metadata, "clock_mapping_uncertainty_ns"),
                f"{role}.clock_mapping_uncertainty_ns", minimum=0,
            )
            if uncertainty_ns > MAX_CLOCK_UNCERTAINTY_NS:
                raise ObservationRejected(f"{role} clock mapping uncertainty exceeds 20 ms")
            if _stamp_ns(_field(image, "header")) != _stamp_ns(_field(metadata, "header")):
                raise ObservationRejected(f"{role} image does not match its frame metadata")
            camera_stamp_ns = _stamp_ns(_field(image, "header"))
            ros_ages[role] = _check_ros_age(
                now_ros_ns, camera_stamp_ns, MAX_IMAGE_AGE_NS, f"{role} image"
            )
            source_times[role] = _require_int(
                _field(metadata, "source_monotonic_ns"), f"{role}.source_monotonic_ns", minimum=1
            )
            if _field(metadata, "source_local_clock_id") != LOCAL_MONOTONIC_CLOCK_ID:
                raise ObservationRejected(f"{role} timestamp is not mapped to the WSL monotonic clock")
            local_source_ns = _require_int(
                _field(metadata, "source_local_monotonic_ns"),
                f"{role}.source_local_monotonic_ns", minimum=1,
            )
            if (_field(metadata, "source_clock_id") == "linux_perf_counter_ns"
                    and (local_source_ns != source_times[role] or uncertainty_ns != 0)):
                raise ObservationRejected("native camera must use an identity local clock mapping")
            local_age_ns = now_monotonic_ns - local_source_ns
            if local_age_ns < -uncertainty_ns:
                raise ObservationRejected(f"{role} mapped monotonic timestamp is in the future")
            if local_age_ns > MAX_IMAGE_AGE_NS + uncertainty_ns:
                raise ObservationRejected(f"stale {role} source timestamp in the WSL monotonic clock")
            local_source_times[role] = local_source_ns
            camera_sequences[role] = _require_int(
                _field(metadata, "sequence"), f"{role}.sequence", minimum=1
            )
            images[role], _, _ = _image_array(image, metadata, role)

        camera_skew_ns = abs(source_times["base"] - source_times["wrist"])
        if camera_skew_ns > max_camera_skew_ns:
            raise ObservationRejected(
                "Base/Wrist host-after-read timestamp skew exceeds "
                f"{max_camera_skew_ns / 1e6:g} ms"
            )
        all_source_times = [*channel_times, source_times["base"], source_times["wrist"]]
        sensor_span_ns = max(all_source_times) - min(all_source_times)
        if sensor_span_ns > MAX_SENSOR_SKEW_NS:
            raise ObservationRejected("camera/CAN source timestamp span exceeds 250 ms")
        local_sensor_span_ns = max(local_source_times.values()) - min(local_source_times.values())
        if local_sensor_span_ns > MAX_SENSOR_SKEW_NS + MAX_CLOCK_UNCERTAINTY_NS:
            raise ObservationRejected("mapped camera/CAN source span exceeds 270 ms")

        last = previous_sequences or {}
        if state_sequence <= last.get("state", 0):
            raise ObservationRejected("duplicate or out-of-order state sequence")
        for role, sequence in camera_sequences.items():
            if sequence <= last.get(role, 0):
                raise ObservationRejected(f"duplicate or out-of-order {role} image sequence")

        joints = np.asarray(_field(state, "joint_position_rad"), dtype=np.float64)
        gripper = _field(state, "gripper_nominal_m")
        if joints.shape != (6,) or not np.isfinite(joints).all():
            raise ObservationRejected("joint feedback must be six finite radians")
        if isinstance(gripper, bool) or not isinstance(gripper, (int, float)) or not math.isfinite(gripper):
            raise ObservationRejected("gripper feedback must be a finite nominal value")

        # Keep preprocessing identical to the existing live shadow path:
        # BGR8 -> RGB8, six joint radians plus the nominal gripper value.
        from laptop.client.observation_schema import PIPER_PROMPT, build_piper_observation

        observation = build_piper_observation(
            images["base"], images["wrist"],
            [*joints.tolist(), float(gripper)], PIPER_PROMPT,
        )
        from laptop.client.observation_schema import validate_piper_semantic_observation
        validate_piper_semantic_observation(observation)
        sequence_key = {
            "state": state_sequence,
            "base": camera_sequences["base"],
            "wrist": camera_sequences["wrist"],
        }
        observation_id = f"{session_id}:{state_sequence}:{camera_sequences['base']}:{camera_sequences['wrist']}"
        timing = {
            "session_id": session_id,
            "observation_id": observation_id,
            "source_monotonic_ns": state_source_ns,
            "source_clock_id": _field(state, "source_clock_id"),
            "local_clock_id": _field(state, "source_local_clock_id"),
            "observation_local_monotonic_ns": max(local_source_times.values()),
            "local_source_times_ns": local_source_times,
            "state_header_stamp_ns": state_stamp_ns,
            "state_sequence": state_sequence,
            "camera_sequences": camera_sequences,
            "source_times_ns": source_times,
            "camera_skew_ms": camera_skew_ns / 1e6,
            "camera_timestamp_semantics": (
                "linux_ros_callback_arrival_not_exposure_sync"
                if _field(state, "source_clock_id") == "linux_perf_counter_ns"
                else "windows_host_after_read_software_pairing_not_exposure_sync"),
            "sensor_span_ms": sensor_span_ns / 1e6,
            "ros_age_ms": {key: value / 1e6 for key, value in ros_ages.items()},
            "clock_uncertainty_ns": {
                role: _field(metadata, "clock_mapping_uncertainty_ns")
                for role, _, metadata in role_inputs
            },
            "fake": fake,
            "hardware_connected": connected,
            "sequence_key": sequence_key,
        }
        return observation, timing
    except ObservationRejected:
        raise
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ObservationRejected(f"malformed ROS observation: {exc}") from exc


def validate_policy_actions(result: Any, metadata: dict[str, Any], identity_lock: dict[str, Any],
                            *, backend: str = "openpi", request: dict[str, Any] | None = None) -> np.ndarray:
    """Apply a backend-specific identity lock and finite 16x7 output contract."""
    from laptop.client.policy_client import validate_policy_metadata

    validate_policy_metadata(metadata, identity_lock, backend=backend)
    if backend == "lerobot_pi05":
        if not isinstance(result, dict):
            raise ObservationRejected("LeRobot PI05 response must be a mapping")
        response_identity = {
            "backend": metadata["backend"],
            "checkpoint_sha256": metadata["checkpoint_sha256"],
            "processor_identity_sha256": metadata["processor_identity_sha256"],
            "prediction_only": True,
        }
        if any(result.get(key) != value for key, value in response_identity.items()):
            raise ObservationRejected("LeRobot PI05 response identity differs from handshake metadata")
        if request is not None:
            for key in ("session_id", "request_id", "observation_ns"):
                if result.get(key) != request.get(key):
                    raise ObservationRejected(f"LeRobot PI05 response {key} does not match its request")
    actions = np.asarray(result.get("actions") if isinstance(result, dict) else None, dtype=np.float64)
    if actions.shape != (16, 7):
        raise ObservationRejected(f"policy actions must have shape (16, 7), got {actions.shape}")
    if not np.isfinite(actions).all():
        raise ObservationRejected("policy actions contain a non-finite value")
    if metadata.get("action_horizon") != 16 or metadata.get("action_dimension") != 7:
        raise ObservationRejected("policy metadata action dimensions differ from the PiPER contract")
    return actions
