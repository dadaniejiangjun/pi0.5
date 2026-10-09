"""One approved nominal 1 mm gripper opening with persistent raw bus audit."""
from laptop.robot_client.piper_session import PiperSession
from laptop.runtime.modes import RuntimeMode
from laptop.safety.bridge import SafetyBridge, UNVERIFIED_ANALYSIS_LIMITS
from laptop.transport.gs_usb_piper_can import GsUsbPiperCanTransport


PHRASE = "AUTHORIZE GRIPPER OPEN +1.000 MM COMMISSIONING STEP"


def execute(*, log_path, physical_estop_ready, low_speed_confirmed,
            motion_envelope_cleared, confirm, transport=None, opening_mm=1):
    if type(opening_mm) is not int or opening_mm not in (1, 5):
        raise ValueError("opening_mm must be explicitly 1 or 5")
    delta_raw = opening_mm * 1000
    effort_raw = 100 if opening_mm == 1 else 300
    phrase = PHRASE if opening_mm == 1 else "AUTHORIZE GRIPPER OPEN +5.000 MM AT 0.300 NM COMMISSIONING STEP"
    site = dict(physical_estop_ready=physical_estop_ready,
                low_speed_confirmed=low_speed_confirmed,
                motion_envelope_cleared=motion_envelope_cleared)
    if not all(value is True for value in site.values()) or not callable(confirm):
        raise PermissionError("site confirmation and exact scope approval required")
    bridge = SafetyBridge(mode=RuntimeMode.SUPERVISED_J1_COMMISSIONING,
        transport=transport if transport is not None else GsUsbPiperCanTransport(),
        limits=UNVERIFIED_ANALYSIS_LIMITS, supervised_j1_commissioning_tx_enabled=True)
    with PiperSession(log_path, bridge=bridge) as robot:
        robot.poll(0.6)
        snapshot, gripper = bridge.gripper_opening_snapshot(**site)
        source = int.from_bytes(gripper[1][:4], "big", signed=True)
        preview = dict(source_gripper_raw=source, target_gripper_raw=source+delta_raw,
                       nominal_step_mm=opening_mm, nominal_effort_nm=effort_raw/1000,
                       joint_targets_sent=False, camera_opened=False)
        if confirm(preview) != phrase:
            raise PermissionError("exact gripper opening approval phrase required")
        receipt = robot.open_gripper_step(operator_approved=True,
            delta_raw=delta_raw, effort_raw=effort_raw, **site)
        # Reuse one owner/connection. Observe only; never repeat the opening.
        robot.poll(1.5)
        final = robot.state()
        observed = final.get("gripper", {}).get("position_raw")
        held = bool(final.get("joint_raw_0p001deg") is not None and all(
            abs(b-a) <= 50 for a,b in zip(snapshot.joint_raw_0p001deg,
                                        final["joint_raw_0p001deg"])))
        matched = bool(final["complete_fresh"] and held and observed is not None
            and abs(observed-(source+delta_raw)) <= 50 and final.get("ctrl_mode_raw") == 1
            and final.get("arm_status_raw") == 0 and final.get("error_code") == 0
            and all(final["motor_enabled"]) and not (final["gripper"]["status_raw"] & 0xBF))
        return dict(**preview, status="GRIPPER_FEEDBACK_MATCH" if matched else
            "GRIPPER_FEEDBACK_UNCONFIRMED_NO_RETRY", final_feedback=final,
            adapter_accepted_frame=dict(can_id=hex(receipt.arbitration_id),
                data_hex=receipt.data.hex(), accepted_monotonic_ns=receipt.adapter_accepted_monotonic_ns),
            held_joints_confirmed=held, gripper_delta_raw=None if observed is None else observed-source,
            automatic_retry=False, physical_width_scale_verified=False)
