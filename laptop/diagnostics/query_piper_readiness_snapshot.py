"""Collect one bounded firmware, joint, and gripper getter snapshot.

This is a convenience orchestrator over the existing SafetyBridge-owned
diagnostic queries. Each stage opens its own bridge, requires fresh feedback,
and sends only its existing fixed getter allowlist. The maximum is 14
non-motion query frames (1 firmware, 12 joint parameters, 1 gripper setup).
Any exception or incomplete stage stops the sequence; this tool never retries.
It does not query or claim physical safe-hold behavior.
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

from laptop.diagnostics import (
    query_all_joint_parameters,
    query_piper_firmware,
    query_piper_gripper_parameters,
)


MAX_GETTER_FRAMES = 14
STAGE_CONTRACTS = (
    ("firmware", "FIRMWARE_VERSION_READ", 1),
    ("joints", "ALL_SIX_JOINTS_READ", 12),
    ("gripper", "GRIPPER_PARAMETERS_READ", 1),
)


class DiagnosticReportError(ValueError):
    """A stage result does not prove it stayed within its getter-only contract."""


def _stage_plan() -> tuple[tuple[str, Callable[..., dict], str, int], ...]:
    """Resolve callables at run time so tests can replace every hardware stage."""

    functions = {
        "firmware": query_piper_firmware.query,
        "joints": query_all_joint_parameters.query,
        "gripper": query_piper_gripper_parameters.query,
    }
    return tuple(
        (name, functions[name], expected_status, maximum)
        for name, expected_status, maximum in STAGE_CONTRACTS
    )


def _validated_tx_count(name: str, report: dict, maximum: int) -> int:
    safety = report.get("safety")
    if not isinstance(safety, dict):
        raise DiagnosticReportError(f"{name} report has no safety object")
    count = safety.get("can_data_tx_calls")
    if type(count) is not int or not 0 <= count <= maximum:
        raise DiagnosticReportError(f"{name} report has invalid CAN TX count")
    invariants = {
        "command_tx_enabled": False,
        "robot_motion_commanded": False,
        "robot_sdk_interface_constructed": False,
    }
    for field, expected in invariants.items():
        if safety.get(field) is not expected:
            raise DiagnosticReportError(f"{name} report violates {field}")

    getter_flags = {
        "firmware": (
            "only_documented_firmware_query_sent",
            "mode_enable_motion_gripper_or_parameter_set_frames_sent",
        ),
        "joints": (
            "only_documented_0x472_read_requests_sent",
            "mode_enable_reset_joint_target_gripper_or_setter_frames_sent",
        ),
        "gripper": (
            "only_documented_0x477_read_query_sent",
            "mode_enable_reset_motion_or_gripper_frames_sent",
        ),
    }
    allowed_flag, forbidden_flag = getter_flags[name]
    if safety.get(allowed_flag) is not True or safety.get(forbidden_flag) is not False:
        raise DiagnosticReportError(f"{name} report violates its getter-only frame contract")

    if name == "joints":
        sent = report.get("query_protocol", {}).get("request_frames_sent")
        if type(sent) is not int or sent != count:
            raise DiagnosticReportError("joint query frame count does not match its safety count")
    if name == "gripper" and safety.get("query_frames_sent") != count:
        raise DiagnosticReportError("gripper query frame count does not match its safety count")
    return count


def run(*, approved: bool = False) -> dict:
    """Run the fixed, ordered getter sequence after an explicit CLI approval."""

    if type(approved) is not bool or not approved:
        raise PermissionError("explicit approval is required for documented CAN getter queries")

    started_ns = time.perf_counter_ns()
    results: dict[str, dict] = {}
    total_tx = 0
    tx_count_known = True
    failed_stage: str | None = None
    failure: dict | None = None

    for name, query, expected_status, maximum in _stage_plan():
        try:
            stage_report = query(approved=True)
            if not isinstance(stage_report, dict):
                raise DiagnosticReportError(f"{name} query returned a non-object report")
            stage_tx = _validated_tx_count(name, stage_report, maximum)
        except Exception as exc:
            failed_stage = name
            tx_count_known = False
            failure = {
                "stage": name,
                "error_type": type(exc).__name__,
                "error": str(exc),
                "delivery_note": (
                    "The failed getter may have reached CAN before the exception. "
                    "No retry was attempted; re-enumerate the adapter and obtain "
                    "fresh read-only feedback before another query."
                ),
            }
            results[name] = {"status": "STAGE_EXCEPTION", "safety": {
                "can_data_tx_calls": "unknown_after_exception",
                "command_tx_enabled": False,
            }}
            break

        results[name] = stage_report
        total_tx += stage_tx
        if stage_report.get("status") != expected_status:
            failed_stage = name
            failure = {
                "stage": name,
                "error_type": "IncompleteGetterResponse",
                "error": f"expected {expected_status}, got {stage_report.get('status')!r}",
                "delivery_note": "No retry or later diagnostic stage was attempted.",
            }
            break

    complete = len(results) == len(STAGE_CONTRACTS) and failed_stage is None
    return {
        "schema": "piper_readiness_getter_snapshot_v1",
        "status": "ALL_GETTER_READS_COMPLETE" if complete else "PARTIAL_GETTER_READS",
        "test_utc": datetime.now(timezone.utc).isoformat(),
        "duration_s": round((time.perf_counter_ns() - started_ns) / 1_000_000_000, 3),
        "operator_approved_documented_getters": True,
        "stages": results,
        "failed_stage": failed_stage,
        "failure": failure,
        "query_budget": {
            "maximum_getter_frames": MAX_GETTER_FRAMES,
            "can_data_tx_calls": total_tx if tx_count_known else None,
            "tx_count_known": tx_count_known,
            "retries": 0,
            "later_stages_skipped_after_failure": failed_stage is not None,
        },
        "safety": {
            "getter_only": True,
            "command_tx_enabled": False,
            "robot_motion_commanded": False,
            "robot_sdk_interface_constructed": False,
            "camera_opened": False,
            "safe_hold_verified": False,
            "hardware_control_allowed": False,
        },
        "limitations": [
            "CAN getter frames are data transmissions and require the explicit CLI flag.",
            "Fresh feedback is rechecked independently before each diagnostic stage.",
            "Configured limits and getter replies do not prove physical hard limits.",
            "This snapshot cannot verify safe-hold, stop, or recovery behavior.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--send-documented-getter-queries",
        action="store_true",
        help=(
            "explicitly authorize up to 14 fixed, non-motion getter frames; "
            "each stage requires fresh feedback and stops without retry on failure"
        ),
    )
    parser.add_argument("--output", type=Path, help="exclusive-create JSON report path")
    args = parser.parse_args()
    if not args.send_documented_getter_queries:
        parser.error("pass --send-documented-getter-queries only when authorizing CAN getter frames")
    report = run(approved=True)
    encoded = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    print(encoded, end="")
    return 0 if report["status"] == "ALL_GETTER_READS_COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
