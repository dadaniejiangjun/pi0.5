"""Reconcile project phase metadata from existing deployment evidence only.

This utility performs no model, server, GPU, or hardware operation. It only
updates project status metadata and documentation.
"""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PHASE_PATH = ROOT / "docs/PI05_PIPER_PHASE_GATES.json"
README_PATH = ROOT / "README.md"
AGENTS_PATH = ROOT / "AGENTS.md"
MANIFEST_PATH = ROOT / "provenance/deployment_manifest.json"
REPORT_PATH = ROOT / "reports/P0_P1_DEPLOYMENT_REPORT.md"
PLAN_PATH = ROOT / "docs/PI05_PIPER_MASTER_PLAN.md"

AUTHORITY = "docs/PI05_PIPER_PHASE_GATES.json"
NONCLAIMS = {
    "REAL_PIPER_CONNECTED": "NO",
    "REAL_ROBOT_MOTION": "NO",
    "REAL_DATA_COLLECTION": "NO",
    "FORMAL_FINE_TUNING": "NO",
}
PHASE_SUMMARY = {"P0": "PASS", "P1": "PASS", "P2": "READY"}


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")


def main() -> None:
    for path in (REPORT_PATH, MANIFEST_PATH, PLAN_PATH, PHASE_PATH, README_PATH):
        if not path.is_file():
            raise SystemExit(f"missing required evidence or target: {path}")

    report_text = REPORT_PATH.read_text()
    plan_text = PLAN_PATH.read_text()
    for marker in (
        "| `OPENPI_ENV` | `PASS`",
        "| `PI05_BASE_LOAD` | `PASS`",
        "| `PI05_INFERENCE_SMOKE` | `PASS`",
        "| `LOCAL_CLIENT_ROUNDTRIP` | `PASS`",
        "READY_FOR_LAPTOP_P2 = YES",
        "TEST_REAL_MOTION = NO",
        "REAL_PIPER_CONNECTED = NO",
        "REAL_DATA_COLLECTION = NO",
        "FORMAL_FINE_TUNING = NO",
    ):
        if marker not in report_text:
            raise SystemExit(f"deployment report does not contain required fact: {marker}")
    for marker in (
        "P2_LAPTOP_HARDWARE_AUDIT = NOT_EXECUTED",
        "REAL_ROBOT_MOTION = NO",
        "REAL_DATA_COLLECTION = NO",
        "FORMAL_FINE_TUNING = NO",
    ):
        if marker not in plan_text:
            raise SystemExit(f"master plan does not contain required fact: {marker}")

    phase = json.loads(PHASE_PATH.read_text())
    manifest = json.loads(MANIFEST_PATH.read_text())

    phase["status"] = "P2_READY"
    phase["current_phase"] = "P2"
    phase["phase_status_authority"] = AUTHORITY
    phase["autonomous_execution_authorized"] = False
    phase["phase_summary"] = PHASE_SUMMARY
    phase["nonclaims"] = NONCLAIMS

    for gate in phase.get("gates", []):
        gate_id = gate.get("id")
        if gate_id == "P0":
            gate["status"] = "PASS"
            gate["evidence"] = [
                "provenance/openpi_git.json",
                "provenance/openpi_contract.json",
                "docs/OPENPI_PROVENANCE.md",
                "docs/REMOTE_ARCHITECTURE.md",
                "reports/P0_P1_DEPLOYMENT_REPORT.md",
            ]
        elif gate_id == "P1":
            gate["status"] = "PASS"
            gate["requires"] = ["P0"]
            gate["evidence"] = [
                "reports/P0_P1_DEPLOYMENT_REPORT.md",
                "reports/pi05_base_inference_smoke.json",
                "reports/local_policy_server_smoke.json",
                "reports/local_client_roundtrip.json",
                "reports/pi05_server_performance.json",
            ]
        elif gate_id == "P2":
            gate["status"] = "READY"
            gate["requires"] = ["P0", "P1"]
            gate["evidence"] = [
                "laptop/client/README.md",
                "laptop/README_FIRST_CONNECTION.md",
                "docs/SSH_TUNNEL_SETUP.md",
                "validation/laptop_probe_static_audit.json",
            ]
            gate["ready_for"] = [
                "laptop_read_only_piper_probe",
                "laptop_camera_audit",
                "ssh_tunnel_test",
                "firmware_and_teach_behavior_audit",
            ]
        elif gate_id == "P3" and gate.get("status") == "BLOCKED_ON_P0":
            gate["status"] = "BLOCKED_ON_P2"
            gate["requires"] = ["P2"]

    readme = README_PATH.read_text()
    status_start = readme.index("## Current status")
    status_end = readme.index("\nStart with:", status_start)
    current_status = "\n".join(
        [
            "## Current status",
            "",
            "```text",
            "P0 = PASS",
            "P1 = PASS",
            "P2 = READY",
            "autonomous_execution_authorized = false",
            "```",
            "",
            "This status is reconciled from the completed P0/P1 evidence in",
            "`reports/P0_P1_DEPLOYMENT_REPORT.md`,",
            "`provenance/deployment_manifest.json`, and",
            "`docs/PI05_PIPER_MASTER_PLAN.md`. It is metadata reconciliation only;",
            "no experiment, download, inference, training, benchmark, server startup,",
            "or CAN access is performed by a status update.",
            "",
            "The current safety nonclaims remain:",
            "",
            "```text",
            "REAL_PIPER_CONNECTED = NO",
            "REAL_ROBOT_MOTION = NO",
            "REAL_DATA_COLLECTION = NO",
            "FORMAL_FINE_TUNING = NO",
            "```",
            "",
            "The single machine-readable authority for phase status is",
            "`docs/PI05_PIPER_PHASE_GATES.json`. After any phase completion, update",
            "that file together with this README current-status block and",
            "`provenance/deployment_manifest.json`.",
        ]
    )
    readme = readme[:status_start] + current_status + readme[status_end:]
    old_tail = (
        "The only valid path to autonomous execution is through the gates in\n"
        "`docs/PI05_PIPER_PHASE_GATES.json`. A technical check, a shadow prediction,\n"
        "and a real-robot success are separate claims."
    )
    new_tail = (
        "The unique machine-readable authority for phase status is\n"
        "`docs/PI05_PIPER_PHASE_GATES.json`. A technical check, a shadow prediction,\n"
        "and a real-robot success are separate claims.\n"
        "`autonomous_execution_authorized` remains `false` until the safety-gated\n"
        "project process explicitly changes it."
    )
    if old_tail not in readme:
        raise SystemExit("README status tail was not found")
    README_PATH.write_text(readme.replace(old_tail, new_tail))

    rule = "\n".join(
        [
            "",
            "## Phase status authority",
            "",
            "The unique machine-readable authority for phase status is",
            "`docs/PI05_PIPER_PHASE_GATES.json`. After any phase completion, synchronize",
            "all three locations:",
            "",
            "- `docs/PI05_PIPER_PHASE_GATES.json`",
            "- the current-status block in `README.md`",
            "- `provenance/deployment_manifest.json`",
            "",
            "Status reconciliation must use the declared evidence sources and must",
            "not be used to imply a new experiment, inference, training run, server",
            "benchmark, CAN access, real data collection, or autonomous execution.",
            "",
        ]
    )
    agents = AGENTS_PATH.read_text()
    if "## Phase status authority" not in agents:
        AGENTS_PATH.write_text(agents.rstrip() + rule)

    manifest["phase_status_authority"] = AUTHORITY
    manifest["project_phase_status"] = PHASE_SUMMARY
    manifest["current_status"] = "P0_PASS_P1_PASS_P2_READY"
    manifest["autonomous_execution_authorized"] = False
    manifest["nonclaims"] = NONCLAIMS
    manifest["status_reconciliation"] = {
        "metadata_only": True,
        "evidence_sources": [
            "reports/P0_P1_DEPLOYMENT_REPORT.md",
            "provenance/deployment_manifest.json",
            "docs/PI05_PIPER_MASTER_PLAN.md",
        ],
        "experiments_rerun": False,
        "download_rerun": False,
        "inference_rerun": False,
        "training_rerun": False,
        "server_benchmark_rerun": False,
        "can_access": False,
    }

    write_json(PHASE_PATH, phase)
    write_json(MANIFEST_PATH, manifest)
    print("status metadata reconciled; no experiment or hardware operation performed")


if __name__ == "__main__":
    main()
