"""Official LeRobot Robot and Teleoperator adapters for PiPER.

The module owns no ROS node, socket, camera, USB device, CAN transport, or
robot session. A caller must inject an existing read-only observation source
and, separately, an already-authorized target delegate.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping, Protocol

import numpy as np
from lerobot.robots import Robot, RobotConfig
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig


JOINT_KEYS = tuple(f"joint_{index}.pos" for index in range(1, 7))
GRIPPER_KEY = "gripper.pos"
STATE_KEY = "observation.state"
BASE_IMAGE_KEY = "observation.images.base"
WRIST_IMAGE_KEY = "observation.images.wrist"
CAMERA_SHAPE = (480, 640, 3)
STATE_UNITS = "joint radians; gripper nominal meters (not calibrated)"
ACTION_UNITS = "joint radians; gripper nominal meters (not calibrated)"


class ObservationSource(Protocol):
    """Read-only facade over the existing ROS/gateway bridge or replay."""

    @property
    def is_connected(self) -> bool: ...

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def latest_observation(self) -> Mapping[str, Any]: ...


class OperatorTargetSource(Protocol):
    """Read the existing post-IK operator target stream; never owns a device."""

    @property
    def is_connected(self) -> bool: ...

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def latest_target(self) -> Mapping[str, Any]: ...


class TargetDelegate(Protocol):
    """Delegate one target to an already-running owner/queue/SafetyBridge."""

    def submit_target(
        self,
        target: np.ndarray,
        *,
        authority: Any,
        task_scope: str,
        session_id: str,
        source_sequence: int,
    ) -> Mapping[str, Any]: ...


@dataclass(frozen=True)
class ControlGrant:
    """Opaque existing-scope binding supplied by the caller, never minted here."""

    task_scope: str
    session_id: str
    authority: Any
    explicitly_enabled: bool
    non_fake_confirmed: bool


_observation_source: ObservationSource | None = None
_operator_source: OperatorTargetSource | None = None
_target_delegate: TargetDelegate | None = None
_control_grant: ControlGrant | None = None


def install_backends(
    *,
    observation_source: ObservationSource,
    operator_source: OperatorTargetSource | None = None,
    target_delegate: TargetDelegate | None = None,
    control_grant: ControlGrant | None = None,
) -> None:
    """Inject existing facades; supplying control is an explicit opt-in.

    This function does not construct or start any backend. The caller remains
    responsible for providing a live bridge with its real session and safety
    checks, or an offline replay backend.
    """
    global _observation_source, _operator_source, _target_delegate, _control_grant
    if (target_delegate is None) != (control_grant is None):
        raise ValueError("target_delegate and control_grant must be supplied together")
    _observation_source = observation_source
    _operator_source = operator_source
    _target_delegate = target_delegate
    _control_grant = control_grant


def clear_backends() -> None:
    """Remove process-local injected backends without invoking device actions."""
    global _observation_source, _operator_source, _target_delegate, _control_grant
    _observation_source = None
    _operator_source = None
    _target_delegate = None
    _control_grant = None


def bind_existing_ros_control(
    grant: ControlGrant,
    *,
    backend: Any | None = None,
    validity_ms: int = 200,
    service_timeout_s: float = 1.5,
) -> None:
    """Explicitly bind the existing scoped ROS services to an existing grant."""
    global _observation_source, _operator_source, _target_delegate, _control_grant
    from .ros_topic_backend import ExistingRosTopicBackend
    source = backend or _observation_source
    if not isinstance(source, ExistingRosTopicBackend) or not source.is_connected:
        raise RuntimeError("an already-connected ExistingRosTopicBackend is required")
    if not isinstance(grant, ControlGrant):
        raise TypeError("grant must be an explicit ControlGrant")
    if (
        not grant.explicitly_enabled or not grant.non_fake_confirmed
        or not grant.task_scope or not grant.session_id or grant.authority is None
    ):
        raise PermissionError("grant must carry explicit existing scope and non-fake confirmation")
    _observation_source = source
    _operator_source = _operator_source or source
    _target_delegate = source.make_control_delegate(
        validity_ms=validity_ms,
        service_timeout_s=service_timeout_s,
    )
    _control_grant = grant


def _finite_vector(values: Any, *, name: str) -> np.ndarray:
    array = np.asarray(values, dtype=np.float32)
    if array.shape != (7,) or not np.isfinite(array).all():
        raise ValueError(f"{name} must be a finite float32[7] vector")
    return array


def flatten_observation_for_policy(observation: Mapping[str, Any]) -> np.ndarray:
    """Extract the seven nominal state values from an existing sample."""
    if STATE_KEY in observation:
        return _finite_vector(observation[STATE_KEY], name=STATE_KEY).copy()
    if "state" in observation:
        return _finite_vector(observation["state"], name="state").copy()
    raise ValueError("observation is missing observation.state or state")


def named_action_to_target(action: Mapping[str, Any]) -> np.ndarray:
    """Convert LeRobot's seven named position fields to the existing target order."""
    expected = (*JOINT_KEYS, GRIPPER_KEY)
    missing = [key for key in expected if key not in action]
    if missing:
        raise ValueError(f"action is missing fields: {missing}")
    return _finite_vector([action[key] for key in expected], name="action target")


def _features() -> dict[str, Any]:
    return {
        **{key: float for key in JOINT_KEYS},
        GRIPPER_KEY: float,
        BASE_IMAGE_KEY: CAMERA_SHAPE,
        WRIST_IMAGE_KEY: CAMERA_SHAPE,
    }


@RobotConfig.register_subclass("piper_follower")
@dataclass(kw_only=True)
class PiperRobotConfig(RobotConfig):
    """LeRobot configuration; it contains no live endpoint or CAN settings."""

    observation_mode: str = "read_only"
    backend: str = "existing_ros_topics"

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.observation_mode != "read_only":
            raise ValueError("PiperRobot supports read_only observations only")
        if self.backend not in {"existing_ros_topics", "injected"}:
            raise ValueError("backend must be existing_ros_topics or injected")


@TeleoperatorConfig.register_subclass("piper_gamepad")
@dataclass(kw_only=True)
class PiperGamepadConfig(TeleoperatorConfig):
    """LeRobot configuration for consuming an existing operator target stream."""

    source: str = "existing_post_ik_target_stream"
    backend: str = "existing_ros_topics"

    def __post_init__(self) -> None:
        if self.source != "existing_post_ik_target_stream":
            raise ValueError("PiperGamepad only consumes the existing post-IK stream")
        if self.backend not in {"existing_ros_topics", "injected"}:
            raise ValueError("backend must be existing_ros_topics or injected")


class PiperRobot(Robot):
    """LeRobot Robot facade over an injected read-only observation bridge."""

    config_class = PiperRobotConfig
    name = "piper_follower"

    def __init__(self, config: PiperRobotConfig) -> None:
        super().__init__(config)
        self.config = config
        self._connected = False
        self._last_metadata: dict[str, Any] = {}
        self._last_receipt: Mapping[str, Any] | None = None
        self._uses_default_backend = False

    @property
    def observation_features(self) -> dict[str, Any]:
        return {
            STATE_KEY: (7,),
            BASE_IMAGE_KEY: CAMERA_SHAPE,
            WRIST_IMAGE_KEY: CAMERA_SHAPE,
        }

    @property
    def action_features(self) -> dict[str, Any]:
        return {key: float for key in (*JOINT_KEYS, GRIPPER_KEY)}

    @property
    def is_connected(self) -> bool:
        return bool(self._connected and _observation_source and _observation_source.is_connected)

    @property
    def is_calibrated(self) -> bool:
        # The project nominal gripper meter representation is not a calibration.
        return False

    def connect(self, calibrate: bool = True) -> None:
        global _observation_source
        if calibrate:
            raise PermissionError("automatic calibration is unsupported and disabled")
        if _observation_source is None:
            if self.config.backend != "existing_ros_topics":
                raise RuntimeError("backend=injected requires install_backends()")
            from .ros_topic_backend import acquire_default_backend
            _observation_source = acquire_default_backend()
            self._uses_default_backend = True
        elif self.config.backend == "existing_ros_topics" and (
            _observation_source is _default_backend_reference()
        ):
            from .ros_topic_backend import acquire_default_backend
            acquire_default_backend()
            self._uses_default_backend = True
        if not _observation_source.is_connected:
            _observation_source.connect()
        if not _observation_source.is_connected:
            raise RuntimeError("observation source did not report connected")
        self._connected = True

    def calibrate(self) -> None:
        raise PermissionError("PiPER adapter has no calibrated conversion; calibration is not implemented")

    def configure(self) -> None:
        # Deliberately no mode switch, enable, reset, camera open, or transport setup.
        return None

    def reset(self) -> None:
        """Clear local provenance only; never issue a hardware reset command."""
        self._last_metadata = {}
        self._last_receipt = None

    @property
    def connection_status(self) -> dict[str, Any]:
        return {
            "connected": self.is_connected,
            "observation_source_injected": _observation_source is not None,
            "control_enabled": _control_grant is not None,
        }

    @property
    def last_metadata(self) -> Mapping[str, Any]:
        return dict(self._last_metadata)

    @property
    def last_receipt(self) -> Mapping[str, Any] | None:
        return self._last_receipt

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected or _observation_source is None:
            raise RuntimeError("PiperRobot is disconnected")
        sample = _observation_source.latest_observation()
        metadata = sample.get("source_map", sample.get("metadata", {}))
        if not isinstance(metadata, Mapping):
            raise ValueError("observation metadata must be a mapping")
        state = _finite_vector(sample.get("state"), name="state")
        images = sample.get("images")
        if not isinstance(images, Mapping):
            raise ValueError("observation images must be a mapping")
        result: dict[str, Any] = {STATE_KEY: state}
        for role, key in (("base", BASE_IMAGE_KEY), ("wrist", WRIST_IMAGE_KEY)):
            image = np.asarray(images.get(role))
            if image.dtype != np.uint8 or image.ndim != 3 or image.shape[-1] != 3:
                raise ValueError(f"{role} image must be HWC uint8 RGB")
            result[key] = np.ascontiguousarray(image)
        self._last_metadata = dict(metadata)
        return result

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("PiperRobot is disconnected")
        grant = _control_grant
        if (
            grant is None or _target_delegate is None
            or not grant.explicitly_enabled or not grant.non_fake_confirmed
            or not grant.task_scope or not grant.session_id
            or grant.authority is None
        ):
            raise PermissionError("no explicit existing non-fake task-scope grant")
        if _observation_source is None:
            raise RuntimeError("observation source disappeared")
        observation = _observation_source.latest_observation()
        metadata = observation.get("source_map", observation.get("metadata", {}))
        state_meta = metadata.get("state", metadata) if isinstance(metadata, Mapping) else {}
        if (
            not isinstance(state_meta, Mapping)
            or state_meta.get("session_id") != grant.session_id
            or type(state_meta.get("sequence")) is not int
            or state_meta.get("sequence", -1) < 0
            or state_meta.get("feedback_fresh") is not True
            or state_meta.get("hardware_connected") is not True
            or state_meta.get("fake") is not False
        ):
            raise PermissionError("current feedback is not fresh and bound to the granted session/sequence")
        observed_state = self._last_metadata.get("state", {})
        if (
            not isinstance(observed_state, Mapping)
            or observed_state.get("session_id") != state_meta.get("session_id")
            or observed_state.get("sequence") != state_meta.get("sequence")
        ):
            raise PermissionError("action is no longer bound to the exact observation used to produce it")
        target = named_action_to_target(action)
        receipt = _target_delegate.submit_target(
            target,
            authority=grant.authority,
            task_scope=grant.task_scope,
            session_id=grant.session_id,
            source_sequence=state_meta["sequence"],
        )
        if not isinstance(receipt, Mapping):
            raise TypeError("target delegate must return a provenance mapping")
        # Preserve separate semantic records. A receipt is not physical feedback.
        self._last_receipt = dict(receipt)
        if receipt.get("adapter_accepted") is not True:
            raise PermissionError("existing target delegate did not accept the action")
        return dict(action)

    def disconnect(self) -> None:
        global _observation_source
        self._connected = False
        if self._uses_default_backend and _observation_source is not None:
            from .ros_topic_backend import release_default_backend
            release_default_backend(_observation_source)
            self._uses_default_backend = False
            if _default_backend_reference() is None:
                _observation_source = None
        elif _observation_source is not None and _observation_source.is_connected:
            _observation_source.disconnect()


class PiperGamepad(Teleoperator):
    """LeRobot Teleoperator consuming existing IK-processed gamepad targets."""

    config_class = PiperGamepadConfig
    name = "piper_gamepad"

    def __init__(self, config: PiperGamepadConfig) -> None:
        super().__init__(config)
        self.config = config
        self._connected = False
        self._last_provenance: dict[str, Any] = {}
        self._uses_default_backend = False

    @property
    def action_features(self) -> dict[str, Any]:
        return {key: float for key in (*JOINT_KEYS, GRIPPER_KEY)}

    @property
    def feedback_features(self) -> dict[str, Any]:
        return {key: float for key in (*JOINT_KEYS, GRIPPER_KEY)}

    @property
    def is_connected(self) -> bool:
        return bool(self._connected and _operator_source and _operator_source.is_connected)

    @property
    def is_calibrated(self) -> bool:
        return False

    @property
    def last_provenance(self) -> Mapping[str, Any]:
        return dict(self._last_provenance)

    def connect(self, calibrate: bool = True) -> None:
        global _operator_source, _observation_source
        if calibrate:
            raise PermissionError("automatic calibration is unsupported and disabled")
        if _operator_source is None:
            if self.config.backend != "existing_ros_topics":
                raise RuntimeError("backend=injected requires install_backends()")
            from .ros_topic_backend import acquire_default_backend
            backend = acquire_default_backend()
            _observation_source = _observation_source or backend
            _operator_source = backend
            self._uses_default_backend = True
        elif self.config.backend == "existing_ros_topics" and (
            _operator_source is _default_backend_reference()
        ):
            from .ros_topic_backend import acquire_default_backend
            acquire_default_backend()
            self._uses_default_backend = True
        if not _operator_source.is_connected:
            _operator_source.connect()
        if not _operator_source.is_connected:
            raise RuntimeError("operator source did not report connected")
        self._connected = True

    def calibrate(self) -> None:
        raise PermissionError("operator target units are nominal and not calibrated")

    def configure(self) -> None:
        return None

    def reset(self) -> None:
        self._last_provenance = {}

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected or _operator_source is None:
            raise RuntimeError("PiperGamepad is disconnected")
        sample = _operator_source.latest_target()
        if not isinstance(sample, Mapping):
            raise TypeError("operator source must return a mapping")
        intent = sample.get("operator_intent")
        safe_target = sample.get("safe_target")
        if intent is None or safe_target is None:
            raise PermissionError("operator intent and post-IK safe target must remain distinct and present")
        target = _finite_vector(safe_target, name="safe_target")
        self._last_provenance = {
            "operator_intent": intent,
            "safe_target": target.tolist(),
            "accepted": sample.get("accepted"),
            "feedback": sample.get("feedback"),
            "session_id": sample.get("session_id"),
            "source_sequence": sample.get("source_sequence"),
            "units": ACTION_UNITS,
        }
        return dict(zip((*JOINT_KEYS, GRIPPER_KEY), map(float, target)))

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        if not self.is_connected:
            raise RuntimeError("PiperGamepad is disconnected")
        # This adapter is a consumer. Feedback cannot be sent to a controller here.
        _finite_vector([feedback[key] for key in (*JOINT_KEYS, GRIPPER_KEY)], name="feedback")

    def disconnect(self) -> None:
        global _operator_source, _observation_source
        self._connected = False
        if self._uses_default_backend and _operator_source is not None:
            from .ros_topic_backend import release_default_backend
            release_default_backend(_operator_source)
            self._uses_default_backend = False
            _operator_source = None
            if _default_backend_reference() is None:
                _observation_source = None
        elif _operator_source is not None and _operator_source.is_connected:
            _operator_source.disconnect()


def _default_backend_reference() -> Any:
    try:
        from . import ros_topic_backend
        return ros_topic_backend._default_backend
    except ImportError:
        return None
