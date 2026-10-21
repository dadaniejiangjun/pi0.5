"""Explicit local composition of the ROS gateway and supervised CAN owner.

Constructing this runtime opens no device. A completed site profile and a
current local task/session approval are required before position RPC dispatch.
No mode switch, enable, reset, or homing command is performed here.
"""
import argparse
from dataclasses import asdict
from functools import partial
import json
from pathlib import Path
import time
import uuid

from laptop.robot_client.piper_session import PiperSession
from laptop.robot_client.supervised_site_profile import SupervisedSiteProfile, load_supervised_site_profile
from laptop.ros_gateway.local_authorization import WindowsLocalPositionSessionAuthority
from laptop.ros_gateway.position_mapper import map_position_target
from laptop.ros_gateway.runtime_profiles import load_runtime_profile
from laptop.ros_gateway.windows_read_only_gateway import WindowsReadOnlyGateway
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge
from laptop.safety.task_scope import TaskScopedPositionAuthorization
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport


def _camera_gateway_options(profile_name, *, roles_confirmed):
    """Reuse an existing camera profile with the single supervised CAN owner."""
    if profile_name is None:
        return {
            "base_image_port": None,
            "wrist_image_port": None,
        }, {"camera_enabled": False}
    if not roles_confirmed:
        raise PermissionError("physical Base/Wrist camera role confirmation required")
    profile, digest = load_runtime_profile(profile_name)
    gateway = profile["gateway"]
    cameras = profile.get("cameras", {})
    rates = profile.get("rates", {})
    if (gateway.get("camera_enabled") is not True
            or gateway.get("base_image_port") is None
            or gateway.get("wrist_image_port") is None
            or not isinstance(cameras.get("base"), dict)
            or not isinstance(cameras.get("wrist"), dict)):
        raise ValueError("selected profile does not define both camera streams")
    options = {
        "base_image_port": gateway["base_image_port"],
        "wrist_image_port": gateway["wrist_image_port"],
        "base_name": cameras["base"]["device_name"],
        "wrist_name": cameras["wrist"]["device_name"],
        "base_source": cameras["base"].get("source", "directshow"),
        "image_stream_hz": gateway.get("image_stream_hz", 30.0),
        "camera_capture_fps": rates.get("camera_capture_hz", 30.0),
    }
    return options, {
        "camera_enabled": True,
        "camera_profile": profile_name,
        "camera_profile_sha256": digest,
        "image_ports": {
            "base": options["base_image_port"],
            "wrist": options["wrist_image_port"],
        },
        "device_names": {
            "base": options["base_name"],
            "wrist": options["wrist_name"],
        },
        "image_stream_hz": options["image_stream_hz"],
        "camera_capture_fps": options["camera_capture_fps"],
    }


class WindowsPositionGateway:
    """Reuse one gateway, queue, PiperSession and SafetyBridge for control.

The caller supplies a locally loaded site profile. Start only after confirming
device ownership. Approve only after confirming the current movement envelope.
"""

    def __init__(self, *, site_profile, transport_factory=GsUsbPiperCanTransport,
                 **gateway_options):
        if type(site_profile) is not SupervisedSiteProfile:
            raise TypeError("locally loaded SupervisedSiteProfile required")
        if site_profile.p12_evidence.missing:
            raise PermissionError("site evidence is incomplete")
        self.site_profile = site_profile
        self._started = False
        self._authorization = None
        self.authority = WindowsLocalPositionSessionAuthority(
            active_task_scope_provider=lambda: self.gateway.task_request_queue.active_task_scope_id,
        )

        def session_factory(**options):
            bridge = SafetyBridge(
                mode=RuntimeMode.SUPERVISED_POLICY_STEP,
                transport=transport_factory(), limits=site_profile.limits,
                supervised_position_tx_enabled=True,
            )
            return PiperSession(bridge=bridge, **options)

        self.gateway = WindowsReadOnlyGateway(
            **gateway_options, local_session_factory=session_factory,
            local_position_target_factory=partial(
                map_position_target, holding_effort_raw=site_profile.gripper_effort_raw),
            local_position_session_authority=self.authority,
        )

    def start(self):
        if self._started:
            raise RuntimeError("position gateway is single-use")
        self._started = True
        self.gateway.start()
        return self

    def approve_from_console(self, *, session_id, authorization, input_fn=input,
                             output_fn=print, timeout_s=2.0):
        if not self._started or self.gateway.stop_event.is_set():
            raise RuntimeError("gateway is not running")
        if type(authorization) is not TaskScopedPositionAuthorization:
            raise TypeError("local typed task authorization required")
        expected = vars(self.site_profile.p12_evidence)
        if dict(authorization.site_gate_attestations) != expected:
            raise PermissionError("task evidence differs from loaded site profile")
        try:
            future = self.gateway.task_request_queue.submit_begin(
                request_id=f"begin:{authorization.authorization_id}",
                authorization=authorization,
            )
            future.result(timeout=timeout_s)
            self._authorization = authorization
            return self.authority.issue_from_console(
                session_id=session_id, authorization=authorization,
                input_fn=input_fn, output_fn=output_fn,
            )
        except BaseException:
            authorization.revoke()
            self.authority.revoke_session(session_id)
            raise

    def close(self):
        if self._authorization is not None:
            self._authorization.revoke()
        self.gateway.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site-profile", type=Path, required=True)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--host",
                        help="Windows interface reachable from the selected WSL distribution")
    parser.add_argument("--control-port", type=int, default=18768)
    parser.add_argument("--log-path", type=Path)
    parser.add_argument("--duration", type=int, choices=range(1, 121), default=120)
    parser.add_argument("--confirm-device-owner", action="store_true")
    parser.add_argument("--confirm-supervised-motion", action="store_true")
    parser.add_argument("--camera-profile", choices=("p3_p4_shadow",))
    parser.add_argument("--confirm-camera-roles", action="store_true")
    args = parser.parse_args(argv)
    runtime = None
    try:
        # Incomplete evidence fails before any transport is constructed.
        profile = load_supervised_site_profile(args.site_profile)
        camera_options, camera_audit = _camera_gateway_options(
            args.camera_profile,
            roles_confirmed=args.confirm_camera_roles or args.validate_only,
        )
        if args.validate_only:
            print(json.dumps({"status": "SITE_PROFILE_VALIDATED_ONLY",
                              "can_opened": False, "motion_commanded": False,
                              "camera_opened": False,
                              "camera_configuration": camera_audit,
                              "evidence_refs_verified": False}))
            return 0
        if not args.confirm_device_owner or not args.confirm_supervised_motion:
            raise PermissionError("current exclusive CAN ownership and supervision confirmation required")
        if args.log_path is None:
            raise ValueError("explicit unique --log-path required")
        runtime = WindowsPositionGateway(
            site_profile=profile, host=args.host, control_port=args.control_port,
            **camera_options,
            log_path=args.log_path, raw_can_log=True,
        ).start()
        print(json.dumps({"status": "GATEWAY_STARTED_NO_POSITION_SESSION",
                          "control_port": args.control_port,
                          "camera_configuration": camera_audit,
                          "camera_enabled": runtime.gateway.cameras_enabled,
                          "connect_ros_bridge_first": True}), flush=True)
        session_id = input("Enter the current ROS control TCP session_id (blank cancels): ").strip()
        if not session_id:
            return 0
        now = time.perf_counter_ns()
        token = uuid.uuid4().hex
        authorization = TaskScopedPositionAuthorization(
            authorization_id=token, task_scope_id="p2_" + token,
            task=("Supervised P2 Xbox demonstration capture"
                  if camera_audit["camera_enabled"]
                  else "Supervised P2 ROS position commissioning"),
            model_profile="manual_p2_ros", checkpoint_id="manual_no_model",
            operator_attestation_id=profile.commissioned_by,
            issued_monotonic_ns=now,
            valid_until_monotonic_ns=now + args.duration * 1_000_000_000,
            max_duration_s=args.duration, record_full_evidence=True,
            site_gate_attestations=tuple(asdict(profile.p12_evidence).items()),
        )
        binding = runtime.approve_from_console(
            session_id=session_id, authorization=authorization,
        )
        print(json.dumps({"status": "LOCAL_POSITION_SESSION_APPROVED", **asdict(binding),
                          "speed_percent": 5, "gateway_joint_step_limit_deg": 20,
                          "physical_motion_verified": False}), flush=True)
        while (time.perf_counter_ns() < authorization.valid_until_monotonic_ns
               and not runtime.gateway.stop_event.wait(.1)):
            if authorization.revoked:
                break
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(json.dumps({"status": "FAILED", "error": f"{type(exc).__name__}: {exc}",
                          "physical_motion_verified": False}), flush=True)
        return 1
    finally:
        if runtime is not None:
            runtime.close()


if __name__ == "__main__":
    raise SystemExit(main())
