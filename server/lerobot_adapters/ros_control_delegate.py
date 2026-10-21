"""Client for the existing task-scoped PiPER ROS target services."""

from __future__ import annotations

import math
import threading
import time
import uuid
from typing import Any, Mapping

import numpy as np


class ExistingRosControlDelegate:
    """Forward a 7D action through the existing joint and gripper services.

    This client does not own a robot transport. Both ROS services independently
    validate the task scope, source gateway session/sequence, bounded validity,
    and the existing Windows owner/SafetyBridge policy. Since the installed
    ROS API has separate joint and gripper services, the result records a
    possible partial acceptance and never retries either request.
    """

    def __init__(
        self,
        *,
        backend: Any,
        service_types: Mapping[str, Any],
        joint_state_type: Any,
        validity_ms: int = 200,
        service_timeout_s: float = 1.5,
        arrival_clock=time.perf_counter_ns,
    ) -> None:
        if type(validity_ms) is not int or not 1 <= validity_ms <= 250:
            raise ValueError("validity_ms must be within the existing 1..250 ms service bound")
        if not math.isfinite(service_timeout_s) or not 0.05 <= service_timeout_s <= 5.0:
            raise ValueError("service_timeout_s must be within 0.05..5 seconds")
        if not {"SetJointTarget", "SetGripper"}.issubset(service_types):
            raise ValueError("both existing target service types are required")
        self.backend = backend
        self.service_types = dict(service_types)
        self.joint_state_type = joint_state_type
        self.validity_ms = validity_ms
        self.service_timeout_s = service_timeout_s
        self.arrival_clock = arrival_clock
        self.joint_client = backend.node.create_client(
            self.service_types["SetJointTarget"], "/piper/control/set_joint_target"
        )
        self.gripper_client = backend.node.create_client(
            self.service_types["SetGripper"], "/piper/control/set_gripper"
        )

    @staticmethod
    def _scope_from_authority(authority: Any) -> str | None:
        if isinstance(authority, Mapping):
            value = authority.get("task_scope_id")
        else:
            value = getattr(authority, "task_scope_id", None)
        return value if isinstance(value, str) and value.strip() else None

    def _call(self, client: Any, request: Any, deadline_ns: int) -> Any:
        remaining = max(0.0, (deadline_ns - self.arrival_clock()) / 1e9)
        if remaining <= 0 or not client.wait_for_service(timeout_sec=remaining):
            raise TimeoutError("existing PiPER target service was unavailable before deadline")
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        remaining = max(0.0, (deadline_ns - self.arrival_clock()) / 1e9)
        if remaining <= 0 or not done.wait(remaining):
            raise TimeoutError("existing PiPER target service did not reply before deadline")
        response = future.result()
        if response is None:
            raise RuntimeError("existing PiPER target service returned no response")
        return response

    def submit_target(
        self,
        target: np.ndarray,
        *,
        authority: Any,
        task_scope: str,
        session_id: str,
        source_sequence: int,
    ) -> Mapping[str, Any]:
        target = np.asarray(target, dtype=np.float64)
        if target.shape != (7,) or not np.isfinite(target).all():
            raise ValueError("target must be finite float64[7]")
        if self._scope_from_authority(authority) != task_scope:
            raise PermissionError("opaque authority is not bound to this task scope")
        if not isinstance(session_id, str) or not session_id or type(source_sequence) is not int:
            raise ValueError("a bound source session and sequence are required")

        # Use the same project nominal conversion and task limits as the owner.
        from laptop.piper.control_profile import (
            CONTINUOUS_GRIPPER_LIMITS,
            GRIPPER_NOMINAL_METERS_PER_RAW,
            validate_continuous_gripper_target,
        )

        observation = self.backend.latest_observation()
        state = observation.get("source_map", {}).get("state", {})
        if (
            state.get("session_id") != session_id
            or state.get("sequence") != source_sequence
            or state.get("feedback_fresh") is not True
            or state.get("hardware_connected") is not True
            or state.get("fake") is not False
        ):
            raise PermissionError("source state is not fresh and bound to the task session")
        state_values = np.asarray(observation.get("state"), dtype=np.float64)
        if state_values.shape != (7,) or not np.isfinite(state_values).all():
            raise ValueError("existing bridge observation must contain finite float64[7]")

        gripper_raw = int(round(float(target[6]) / GRIPPER_NOMINAL_METERS_PER_RAW))
        if abs(float(target[6]) - gripper_raw * GRIPPER_NOMINAL_METERS_PER_RAW) > (
            GRIPPER_NOMINAL_METERS_PER_RAW / 2 + 1e-12
        ):
            raise ValueError("gripper nominal meters do not map to the project raw increment")
        source_gripper_raw = int(round(float(state_values[6]) / GRIPPER_NOMINAL_METERS_PER_RAW))
        validate_continuous_gripper_target(
            source_gripper_raw, gripper_raw, CONTINUOUS_GRIPPER_LIMITS.effort_raw
        )

        deadline_ns = self.arrival_clock() + int(self.service_timeout_s * 1e9)
        request_id = uuid.uuid4().hex
        joint_request = self.service_types["SetJointTarget"].Request()
        joint_request.request_id = request_id + ".j"
        joint_request.task_scope_id = task_scope
        joint_request.source_session_id = session_id
        joint_request.source_state_sequence = source_sequence
        joint_request.validity_ms = self.validity_ms
        joint_target = self.joint_state_type()
        joint_target.name = [f"joint{index}" for index in range(1, 7)]
        joint_target.position = [float(value) for value in target[:6]]
        joint_request.target = joint_target
        try:
            joint_response = self._call(self.joint_client, joint_request, deadline_ns)
        except TimeoutError as exc:
            return {
                "decision": "JOINT_SERVICE_TIMEOUT_ACCEPTANCE_UNKNOWN",
                "reason": str(exc),
                "request_id": request_id,
                "adapter_accepted": False,
                "physical_feedback_matched": False,
                "can_tx_calls": None,
            }
        joint_accepted = getattr(joint_response, "accepted", False) is True
        result: dict[str, Any] = {
            "decision": str(getattr(joint_response, "decision", "MISSING_JOINT_DECISION")),
            "reason": str(getattr(joint_response, "reason", "")),
            "request_id": request_id,
            "session_id": session_id,
            "source_state_sequence": source_sequence,
            "task_scope_id": task_scope,
            "joint_service_accepted": joint_accepted,
            "gripper_service_accepted": False,
            "adapter_accepted": False,
            "physical_feedback_matched": False,
            "can_tx_calls": None,
            "units": {
                "joint": "rad",
                "gripper": "nominal_m; uncalibrated",
                "gripper_raw_conversion": GRIPPER_NOMINAL_METERS_PER_RAW,
            },
        }
        if not joint_accepted:
            return result

        gripper_request = self.service_types["SetGripper"].Request()
        gripper_request.request_id = request_id + ".g"
        gripper_request.task_scope_id = task_scope
        gripper_request.source_session_id = session_id
        gripper_request.source_state_sequence = source_sequence
        gripper_request.validity_ms = self.validity_ms
        gripper_request.gripper_raw = gripper_raw
        gripper_request.nominal_position_m = float(target[6])
        gripper_request.nominal_effort_nm = CONTINUOUS_GRIPPER_LIMITS.effort_raw / 1000.0
        try:
            gripper_response = self._call(self.gripper_client, gripper_request, deadline_ns)
        except TimeoutError as exc:
            result.update({
                "decision": "PARTIAL_JOINT_ACCEPTED_GRIPPER_TIMEOUT_UNKNOWN",
                "reason": str(exc),
                "gripper_acceptance_unknown": True,
            })
            return result
        gripper_accepted = getattr(gripper_response, "accepted", False) is True
        result.update({
            "decision": (
                "BOTH_EXISTING_SERVICES_ACCEPTED" if gripper_accepted
                else "PARTIAL_JOINT_ACCEPTED_GRIPPER_REJECTED"
            ),
            "reason": str(getattr(gripper_response, "reason", "")),
            "gripper_service_accepted": gripper_accepted,
            "adapter_accepted": gripper_accepted,
        })
        return result
