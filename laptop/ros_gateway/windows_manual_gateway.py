"""Locally approved P2 manual target or time-limited Xbox teleop session."""
import argparse
import json
from pathlib import Path
import time
import uuid

from laptop.robot_client.piper_session import PiperSession
from laptop.ros_gateway.manual_commissioning import (
    ManualCommissioningDispatcher,
    ManualGamepadTeleopDispatcher,
)
from laptop.ros_gateway.windows_read_only_gateway import WindowsReadOnlyGateway
from laptop.ros_gateway.windows_position_gateway import _camera_gateway_options
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport
from laptop.piper.control_profile import SINGLE_STEP_GRIPPER_LIMITS


def _session_source_state_provider(gateway):
    """Adapt dispatcher argument order to the gateway cache lookup API."""
    return lambda session_id, sequence: gateway.recent_fresh_state(sequence, session_id)


def _gamepad_control_readiness(state):
    if state.get("ctrl_mode_raw") != 1 or state.get("move_mode_raw") != 1:
        return "CAN_CTRL and MOVE_J must already be active"
    enabled = state.get("motor_enabled")
    if not isinstance(enabled, (list, tuple)) or len(enabled) != 6 or not all(v is True for v in enabled):
        return "all six motors must already be enabled"
    if state.get("arm_status_raw") != 0 or state.get("error_code") != 0:
        return "robot reports an abnormal state"
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=18765)
    parser.add_argument("--log-path", type=Path, required=True)
    targets = parser.add_mutually_exclusive_group(required=False)
    targets.add_argument("--joint-target-rad", type=float, nargs=6)
    targets.add_argument("--gripper-target-raw", type=int)
    targets.add_argument("--gamepad-stream", action="store_true",
                         help="approve a bounded live Xbox target stream instead of one fixed target")
    parser.add_argument("--duration", type=int, default=120)
    parser.add_argument("--session-wait-s", type=int, default=60,
                        help="seconds to wait for the existing ROS bridge and fresh PiPER feedback")
    parser.add_argument("--gripper-step-limit-raw", type=int,
                        default=SINGLE_STEP_GRIPPER_LIMITS.maximum_step_raw)
    parser.add_argument("--joint-step-deg", type=float, default=5.0)
    parser.add_argument("--state-snapshot-hz", type=float, default=20.0,
                        help="complete PiPER feedback snapshots per second")
    parser.add_argument("--camera-profile", choices=("p3_p4_shadow",))
    parser.add_argument("--confirm-camera-roles", action="store_true")
    for flag in ("confirm-device-owner","physical-estop-ready","low-speed-confirmed","motion-envelope-cleared"):
        parser.add_argument("--"+flag,action="store_true")
    args = parser.parse_args(argv)
    gateway = None
    try:
        if not all((args.confirm_device_owner,args.physical_estop_ready,
                    args.low_speed_confirmed,args.motion_envelope_cleared)):
            raise PermissionError("current device ownership and physical site confirmations required")
        if not 1<=args.duration<=120:
            raise ValueError("duration must be 1..120 seconds")
        if not 1 <= args.session_wait_s <= 120:
            raise ValueError("ROS session wait must be 1..120 seconds")
        if args.gamepad_stream:
            if args.joint_target_rad is not None or args.gripper_target_raw is not None:
                raise ValueError("gamepad stream cannot be combined with a fixed target")
            if not 0 < args.joint_step_deg <= 5.0:
                raise ValueError("gamepad joint step must be within 0..5 degrees")
            if not 1 <= args.gripper_step_limit_raw <= 1000:
                raise ValueError("gamepad gripper step must be within 1..1000 raw")
        elif args.joint_target_rad is None and args.gripper_target_raw is None:
            raise ValueError("provide one fixed target or --gamepad-stream")
        elif args.camera_profile is not None:
            raise ValueError("camera profile is supported only for gamepad data collection")
        elif not 1<=args.gripper_step_limit_raw<=10000:
            raise ValueError("gripper step limit must be 1..10000 raw")

        if not args.gamepad_stream and args.joint_target_rad is not None:
            operation="set_joint_target"
            arguments={"joint_position_rad": args.joint_target_rad}
        elif not args.gamepad_stream:
            if not SINGLE_STEP_GRIPPER_LIMITS.minimum_raw<=args.gripper_target_raw<=SINGLE_STEP_GRIPPER_LIMITS.maximum_raw:
                raise ValueError("single-step gripper target must be in the tested 0..10000 raw envelope")
            operation="set_gripper"
            arguments=dict(gripper_raw=args.gripper_target_raw,
                           nominal_position_m=args.gripper_target_raw/1e6,
                           nominal_effort_nm=0.0)
        if not args.gamepad_stream:
            # Validate fixed targets before hardware access; this is not a calibration gate.
            from laptop.ros_gateway.control_rpc_dispatcher import _validated_arguments
            _validated_arguments(operation,arguments)
        camera_options, camera_audit = _camera_gateway_options(
            args.camera_profile,
            roles_confirmed=args.confirm_camera_roles,
        )
        def session_factory(**options):
            manual_gamepad = args.gamepad_stream
            return PiperSession(bridge=SafetyBridge(
                mode=(RuntimeMode.SUPERVISED_MANUAL_GAMEPAD if manual_gamepad
                      else RuntimeMode.SUPERVISED_J1_COMMISSIONING),
                transport=GsUsbPiperCanTransport(manual_gamepad_request_budget=1200),
                limits=UNVERIFIED_ANALYSIS_LIMITS,
                supervised_j1_commissioning_tx_enabled=not manual_gamepad,
                supervised_manual_gamepad_tx_enabled=manual_gamepad), **options)
        gateway=WindowsReadOnlyGateway(
            host=args.host,control_port=args.port,
            **camera_options,
            log_path=args.log_path,raw_can_log=False,
            state_snapshot_hz=args.state_snapshot_hz,
            local_session_factory=session_factory).start()
        if args.gamepad_stream:
            print(json.dumps({
                "status":"MANUAL_GAMEPAD_GATEWAY_WAITING_LOCAL_APPROVAL",
                "camera_configuration":camera_audit,
                "nominal_units_only":True,
                "mode_speed_frame":"CAN/MOVE_J/5%",
                "max_joint_change_deg":args.joint_step_deg,
                "max_gripper_change_raw":args.gripper_step_limit_raw,
                "state_snapshot_hz":gateway.state_snapshot_hz,
                "duration_s":args.duration,
                "absolute_joint_limits_from_shared_task_profile":True,
                "p12_model_profile_required":False,
            }),flush=True)
        else:
            print(json.dumps({"status":"MANUAL_GATEWAY_WAITING_LOCAL_APPROVAL",
                              "operation":operation,"arguments":arguments,
                              "nominal_units_only":True,"mode_speed_frame":"CAN/MOVE_J/5%",
                              "max_joint_change_raw":1000,"max_gripper_change_raw":args.gripper_step_limit_raw}),flush=True)
        if args.gamepad_stream:
            print(json.dumps({"status":"WAITING_FOR_ROS_CONTROL_SESSION",
                              "timeout_s":args.session_wait_s}, separators=(",",":")),flush=True)
            deadline_ns = time.perf_counter_ns() + args.session_wait_s * 1_000_000_000
            session_id = ""
            while time.perf_counter_ns() < deadline_ns and not gateway.stop_event.is_set():
                candidate = gateway.active_control_session_id
                latest = gateway.latest_state()
                if candidate and latest is not None:
                    fresh = gateway.recent_fresh_state(latest.get("sequence"), candidate)
                    if fresh is not None and fresh.get("feedback_fresh") is True:
                        readiness = _gamepad_control_readiness(fresh)
                        if readiness:
                            raise PermissionError("GAMEPAD_NOT_READY: " + readiness)
                        session_id = candidate
                        break
                gateway.stop_event.wait(.05)
            if not session_id:
                raise TimeoutError("no active ROS bridge session with fresh PiPER feedback")
            print(json.dumps({"status":"ROS_CONTROL_SESSION_READY",
                              "session_id":session_id},separators=(",",":")),flush=True)
        else:
            session_id=input("Current ROS TCP session_id (blank cancels): ").strip()
            if not session_id:
                return 0
        scope="gamepad_"+uuid.uuid4().hex if args.gamepad_stream else "manual_"+uuid.uuid4().hex
        if args.gamepad_stream:
            phrase=f"AUTHORIZE P2 GAMEPAD SESSION {session_id} SCOPE {scope}"
            prompt=(f"Authorize up to {args.duration}s at CAN/MOVE_J/5%, "
                    f"joint step <= {args.joint_step_deg:g} deg, gripper step <= "
                    f"{args.gripper_step_limit_raw} raw, shared joint/gripper limits, "
                    f"fresh feedback, E-stop ready and clear motion envelope. "
                    f"Type exactly: {phrase}\n> ")
            if input(prompt)!=phrase:
                raise PermissionError("exact local gamepad-session confirmation did not match")
            dispatcher=ManualGamepadTeleopDispatcher(
                queue=gateway.task_request_queue,
                latest_state_provider=gateway.latest_state,
                source_state_provider=_session_source_state_provider(gateway),
                session_id=session_id,task_scope_id=scope,
                valid_until_monotonic_ns=time.perf_counter_ns()+args.duration*1_000_000_000,
                operator_approved=True,physical_estop_ready=True,
                low_speed_confirmed=True,motion_envelope_cleared=True,
                joint_step_limit_raw=round(args.joint_step_deg*1000),
                gripper_step_limit_raw=args.gripper_step_limit_raw)
        else:
            phrase=f"AUTHORIZE MANUAL SESSION {session_id} TASK {scope}"
            if input(f"Approve the exact displayed target including CAN/MOVE_J/5%: {phrase}\n> ")!=phrase:
                raise PermissionError("exact local confirmation did not match")
            dispatcher=ManualCommissioningDispatcher(
                queue=gateway.task_request_queue,latest_state_provider=gateway.latest_state,
                session_id=session_id,task_scope_id=scope,operation=operation,
                approved_arguments=arguments,valid_until_monotonic_ns=time.perf_counter_ns()+args.duration*1_000_000_000,
                operator_approved=True,physical_estop_ready=True,low_speed_confirmed=True,
                motion_envelope_cleared=True,source_state_provider=_session_source_state_provider(gateway),
                gripper_step_limit_raw=args.gripper_step_limit_raw)
        gateway.control_rpc_dispatcher=dispatcher
        gateway._local_session_revoker=dispatcher.revoke_session
        if args.gamepad_stream:
            print(json.dumps({
                "status":"MANUAL_GAMEPAD_SESSION_APPROVED",
                "session_id":session_id,"task_scope_id":scope,
                "duration_s":args.duration,"speed_percent":5,
                "max_joint_step_deg":args.joint_step_deg,
                "max_gripper_step_raw":args.gripper_step_limit_raw,
                "camera_configuration":camera_audit,
                "physical_motion_verified":False,
            }),flush=True)
        else:
            print(json.dumps({"status":"MANUAL_SESSION_APPROVED","session_id":session_id,
                              "task_scope_id":scope,"operation":operation,"arguments":arguments,
                              "single_use":True,"physical_motion_verified":False}),flush=True)
        while time.perf_counter_ns()<dispatcher.expiry and not gateway.stop_event.wait(.1):
            if args.gamepad_stream and gateway.active_control_session_id != session_id:
                dispatcher.revoke_session(session_id)
                print(json.dumps({"status":"MANUAL_GAMEPAD_SESSION_REVOKED",
                                  "reason":"ROS control session disconnected or changed",
                                  "physical_stop_verified":False}),flush=True)
                break
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(json.dumps({"status":"FAILED","reason":f"{type(exc).__name__}: {exc}"}),flush=True)
        return 1
    finally:
        if gateway is not None:
            gateway.close()


if __name__=="__main__":
    raise SystemExit(main())
