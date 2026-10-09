"""Canonical interactive PiPER demonstration recorder. Never sends control frames."""
from __future__ import annotations

import argparse
from getpass import getuser
import json
from pathlib import Path

from laptop.camera.dual_camera import BASE_NAME, WRIST_NAME, inventory, resolve_roles
from laptop.recorder.capture_session import run_session
from laptop.recorder.episode import PROMPT

PROJECT_ROOT = Path(__file__).resolve().parents[2]


def camera_identity(devices, *, base_name=BASE_NAME, wrist_name=WRIST_NAME):
    """Resolve current DSHOW indices by the previously confirmed physical names."""
    roles = resolve_roles(devices, base_name=base_name, wrist_name=wrist_name)
    if any(not item["device_path"] for item in roles.values()):
        raise ValueError("physical camera device paths required; virtual cameras rejected")
    return {"schema": "piper_camera_role_identity_v1",
            "binding_method": "current_DSHOW_name_enumeration_with_previously_confirmed_roles",
            "index_to_instance_id_automatically_proven": False,
            "roles": roles}


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=1, help="independent recordings, not copies")
    parser.add_argument("--duration", type=float, default=20)
    parser.add_argument("--lead-in", type=float, default=5)
    parser.add_argument("--task", default=PROMPT)
    parser.add_argument("--operator", default=getuser())
    parser.add_argument("--base-name", default=BASE_NAME)
    parser.add_argument("--wrist-name", default=WRIST_NAME)
    parser.add_argument("--raw-root", type=Path, default=PROJECT_ROOT / "datasets/raw")
    parser.add_argument("--session-root", type=Path, default=PROJECT_ROOT / "datasets/manifests/capture_sessions")
    parser.add_argument("--outcome-root", type=Path, default=PROJECT_ROOT / "datasets/manifests/outcomes")
    parser.add_argument("--record", action="store_true", help="authorize passive capture; interactive readiness still required")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if not args.record:
        print("No devices opened. Add --record to start interactive passive recording.")
        return 2
    if not args.task.strip() or not 1 <= args.episodes <= 50 or not 10 <= args.duration <= 60 or not 0 <= args.lead_in <= 10:
        raise ValueError("task required; episodes 1..50; duration 10..60s; lead-in 0..10s")
    identity = camera_identity(inventory(), base_name=args.base_name, wrist_name=args.wrist_name)
    print(json.dumps(identity, indent=2, ensure_ascii=False), flush=True)
    print("Task: " + args.task, flush=True)
    report = run_session(args.raw_root, args.session_root, camera_identity=identity,
                         outcome_root=args.outcome_root, operator=args.operator,
                         episodes=args.episodes, duration_s=args.duration,
                         lead_in_s=args.lead_in, task=args.task, approved=True)
    print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    return 0 if report["status"] == "COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
