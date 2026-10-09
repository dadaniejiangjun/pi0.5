"""Capture a bounded sequence of feedback-only operator demonstrations.

Each episode is a separate immutable raw capture with a separate outcome
label. This coordinator never sends CAN data frames or issues robot commands.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import time
import uuid
from datetime import datetime, timezone
from getpass import getuser
from pathlib import Path
from typing import Callable

from laptop.recorder.label_episode import label_episode
from laptop.recorder.passive_capture import capture_supervised


SESSION_SCHEMA = "piper_operator_demo_capture_session_v1"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _append(path: Path, row: dict) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(row, sort_keys=True, ensure_ascii=False) + "\n")


def run_session(raw_root: Path, session_root: Path, *, camera_identity: dict,
                outcome_root: Path = Path("datasets/manifests/outcomes"),
                operator: str, episodes: int = 5, duration_s: float = 45,
                lead_in_s: float = 5, approved: bool = False, task: str | None = None,
                capture_fn: Callable = capture_supervised,
                label_fn: Callable = label_episode,
                prompt_fn: Callable[[str], str] = input,
                emit: Callable[[str], None] = print,
                session_id: str | None = None) -> dict:
    """Run one operator-confirmed capture at a time; tests inject all I/O."""
    if not approved:
        return {"status": "BLOCKED_APPROVAL_REQUIRED", "camera_started": False,
                "can_started": False, "can_data_tx_calls": 0}
    if camera_identity.get("schema") != "piper_camera_role_identity_v1":
        raise ValueError("operator-confirmed Base/Wrist identity is required")
    if not operator.strip():
        raise ValueError("operator name is required")
    if type(episodes) is not int or not 1 <= episodes <= 50:
        raise ValueError("episodes must be an integer in [1, 50]")
    if not isinstance(duration_s, (int, float)) or not 10 <= duration_s <= 60:
        raise ValueError("duration must be between 10 and 60 seconds")
    if not isinstance(lead_in_s, (int, float)) or not 0 <= lead_in_s <= 10:
        raise ValueError("lead-in must be between 0 and 10 seconds")

    raw_root = Path(raw_root).resolve()
    session_root = Path(session_root).resolve()
    outcome_root = Path(outcome_root).resolve()
    roots = (raw_root, session_root, outcome_root)
    for index, root in enumerate(roots):
        for other in roots[index + 1:]:
            if root.is_relative_to(other) or other.is_relative_to(root):
                raise ValueError("raw episodes, session manifests, and outcome labels must be separate")
    session_id = session_id or uuid.uuid4().hex
    if not session_id.isalnum():
        raise ValueError("session_id must be alphanumeric")
    session_path = session_root / session_id
    session_path.mkdir(parents=True, exist_ok=False)
    journal_path = session_path / "events.jsonl"
    journal_path.open("x", encoding="utf-8").close()
    started_ns = time.perf_counter_ns()
    _append(journal_path, {"event": "session_start", "monotonic_ns": started_ns,
                           "episode_count_requested": episodes,
                           "action_target_semantics": "derived_future_observed_state_position_target",
                           "can_data_tx_calls": 0, "robot_motion_commanded": False})

    captured: list[dict] = []
    session_status = "COMPLETE"
    for ordinal in range(1, episodes + 1):
        emit(f"Episode {ordinal}/{episodes}: robot must be safe and ready. "
             "Press Enter to arm passive camera/feedback recording, or type q to stop.")
        if prompt_fn("Ready? [Enter/q]: ").strip().lower() == "q":
            session_status = "INTERRUPTED_BY_OPERATOR"
            break

        emit(f"Capture is bounded to {duration_s:g}s. After the {lead_in_s:g}s countdown, "
             "the explicit CAPTURE_START cue tells you when to begin the task.")
        capture_options = {}
        if camera_identity.get("roles"):
            capture_options.update({f"{role}_index": camera_identity["roles"][role]["opencv_index"]
                                    for role in ("base", "wrist")})
        if task is not None:
            capture_options["task"] = task
        result = capture_fn(
            raw_root, camera_identity=camera_identity, approved=True,
            operator_demo=True, duration_s=float(duration_s), lead_in_s=float(lead_in_s),
            **capture_options,
        )
        episode_path = Path(result.get("path", "")).resolve()
        manifest_path = episode_path / "manifest.json"
        marker_path = Path(result.get("completion_marker", ""))
        row = {
            "ordinal": ordinal,
            "episode_id": episode_path.name if result.get("path") else None,
            "capture_status": result.get("status"),
            "raw_episode_path": str(episode_path) if result.get("path") else None,
            "raw_manifest_sha256": _sha256(manifest_path) if manifest_path.is_file() else None,
            "completion_marker_sha256": _sha256(marker_path) if marker_path.is_file() else None,
            "outcome_label_path": None,
            "outcome_label_sha256": None,
            "success": None,
            "task": task,
            "capture_quality": result.get("integrity"),
            "training_eligible": False,
            "can_data_tx_calls": result.get("can_data_tx_calls", 0),
            "robot_motion_commanded": result.get("robot_motion_commanded", False),
        }
        _append(journal_path, {"event": "capture_result", "monotonic_ns": time.perf_counter_ns(), **row})
        if (result.get("status") != "OPERATOR_DEMO_CAPTURE_RECORDED"
                or not manifest_path.is_file()
                or row["raw_manifest_sha256"] is None
                or result.get("can_data_tx_calls") != 0
                or result.get("robot_motion_commanded") is not False):
            captured.append(row)
            session_status = "PARTIAL_CAPTURE_STOPPED"
            break

        while True:
            answer = prompt_fn("Outcome [s=task succeeded, f=failed/partial, q=leave unlabeled and stop]: ").strip().lower()
            if answer in ("s", "f", "q"):
                break
            emit("Enter s, f, or q.")
        if answer == "q":
            captured.append(row)
            session_status = "INTERRUPTED_UNLABELED_EPISODE"
            break
        notes = prompt_fn("Brief outcome note (required): ").strip()
        if not notes:
            notes = "Operator confirmed task success." if answer == "s" else "Operator marked task failed or partial."
        label = label_fn(episode_path, outcome_root,
                         success=(answer == "s"), notes=notes, operator=operator)
        row.update({
            "outcome_label_path": label["path"],
            "outcome_label_sha256": _sha256(Path(label["path"])),
            "success": answer == "s",
        })
        captured.append(row)
        _append(journal_path, {"event": "outcome_labeled", "monotonic_ns": time.perf_counter_ns(),
                               "episode_id": row["episode_id"], "success": row["success"],
                               "outcome_label_sha256": row["outcome_label_sha256"]})

    successes = sum(row["success"] is True for row in captured)
    failures = sum(row["success"] is False for row in captured)
    manifest = {
        "schema": SESSION_SCHEMA,
        "session_id": session_id,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "status": session_status,
        "operator": operator.strip(),
        "task": task,
        "camera_identity": camera_identity,
        "outcome_root": str(outcome_root),
        "capture_mode": "feedback_only_operator_demonstration",
        "requested_episodes": episodes,
        "captured_episodes": len(captured),
        "successful_episodes": successes,
        "failed_or_partial_episodes": failures,
        "unlabeled_episodes": sum(row["success"] is None for row in captured),
        "duration_s": float(duration_s),
        "lead_in_s": float(lead_in_s),
        "action_target_semantics": "derived_future_observed_state_position_target",
        "raw_can_command_supervision": False,
        "joint_units": "nominal_radian_unverified",
        "gripper_units": "nominal_meter_unverified",
        "episodes": captured,
        "session_events_sha256": _sha256(journal_path),
        "training_eligible": False,
        "hardware_control_allowed": False,
        "can_data_tx_calls": 0,
        "robot_motion_commanded_by_recorder": False,
    }
    manifest_path = session_path / "manifest.json"
    with manifest_path.open("x", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2, ensure_ascii=False, sort_keys=True)
        handle.write("\n")
    return {"status": session_status, "path": str(session_path), **manifest}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path, default=Path("datasets/raw"))
    parser.add_argument("--session-root", type=Path, default=Path("datasets/manifests/capture_sessions"))
    parser.add_argument("--outcome-root", type=Path, default=Path("datasets/manifests/outcomes"))
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--duration", type=float, default=45)
    parser.add_argument("--lead-in", type=float, default=5)
    parser.add_argument("--base-index", type=int, required=True)
    parser.add_argument("--wrist-index", type=int, required=True)
    parser.add_argument("--base-device-id", required=True)
    parser.add_argument("--wrist-device-id", required=True)
    parser.add_argument("--roles-visually-confirmed", action="store_true")
    parser.add_argument("--operator", default=getuser())
    parser.add_argument("--operator-demo-capture-approved", action="store_true")
    args = parser.parse_args()
    if not args.operator_demo_capture_approved:
        print(json.dumps({"status": "BLOCKED_APPROVAL_REQUIRED", "camera_started": False,
                          "can_started": False, "can_data_tx_calls": 0}, indent=2))
        return 2
    from laptop.diagnostics.camera_identity import bind_camera_roles
    identity = bind_camera_roles(
        args.base_index, args.wrist_index,
        base_device_id=args.base_device_id, wrist_device_id=args.wrist_device_id,
        roles_visually_confirmed=args.roles_visually_confirmed,
    )
    report = run_session(args.raw_root, args.session_root, camera_identity=identity,
                         outcome_root=args.outcome_root,
                         operator=args.operator, episodes=args.episodes,
                         duration_s=args.duration, lead_in_s=args.lead_in,
                         approved=True)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["status"] == "COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
