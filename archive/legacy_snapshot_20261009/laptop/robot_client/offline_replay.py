"""Replay recorded candidate actions through the offline guard without sending.

Input JSONL rows contain snapshot, response, and local now_ns. This module has
no PiPER SDK, CAN, or motion import.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable, Mapping

from laptop.safety.offline_guard import Decision, SafetyPolicy, validate_candidate


_NETWORK_FAILURE_REASONS = {
    "timeout": "policy_network_timeout",
    "connection_refused": "policy_network_connection_refused",
    "connection_reset": "policy_network_connection_reset",
}


class FakeRobotTransport:
    """A source of recorded rows with a hard veto on all send attempts."""

    def __init__(self, records: Iterable[Mapping[str, object]]) -> None:
        self._records = tuple(records)
        self.send_attempts = 0

    def records(self) -> tuple[Mapping[str, object], ...]:
        return self._records

    def send(self, _action: object) -> None:
        self.send_attempts += 1
        raise PermissionError("FakeRobotTransport cannot send robot commands")


class OfflineReplayClient:
    def __init__(self, transport: FakeRobotTransport, policy: SafetyPolicy) -> None:
        self.transport = transport
        self.policy = policy
        self.last_response_seq = -1

    def run(self) -> list[Decision]:
        decisions = []
        for row in self.transport.records():
            snapshot = row.get("snapshot")
            response = row.get("response")
            decision = validate_candidate(
                snapshot if isinstance(snapshot, dict) else {},
                response if isinstance(response, dict) else None,
                self.policy, now_ns=row.get("now_ns"),
                last_response_seq=self.last_response_seq,
            )
            transport_error = row.get("policy_transport_error")
            if (response is None and isinstance(transport_error, str)
                    and transport_error in _NETWORK_FAILURE_REASONS):
                decision = Decision(
                    False,
                    tuple(dict.fromkeys((
                        *decision.reasons,
                        _NETWORK_FAILURE_REASONS[transport_error],
                    ))),
                )
            elif response is not None and transport_error is not None:
                decision = Decision(
                    False,
                    tuple(dict.fromkeys((*decision.reasons, "response_with_transport_error"))),
                )
            elif response is None and transport_error is not None:
                decision = Decision(
                    False,
                    tuple(dict.fromkeys((*decision.reasons, "unknown_policy_transport_error"))),
                )
            decisions.append(decision)
            if decision.accepted_for_offline_analysis:
                self.last_response_seq = response["response_seq"]
        return decisions


def replay(records: Iterable[Mapping[str, object]], policy: SafetyPolicy) -> dict:
    transport = FakeRobotTransport(records)
    decisions = OfflineReplayClient(transport, policy).run()
    return {
        "status": "OFFLINE_REPLAY_ONLY",
        "total": len(decisions),
        "accepted_for_offline_analysis": sum(d.accepted_for_offline_analysis for d in decisions),
        "rejected": sum(not d.accepted_for_offline_analysis for d in decisions),
        "decisions": [{"accepted_for_offline_analysis": d.accepted_for_offline_analysis,
                       "reasons": list(d.reasons)} for d in decisions],
        "send_attempts": transport.send_attempts,
        "hardware_control_allowed": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--events", required=True, type=Path, help="offline JSONL, never a live socket")
    parser.add_argument("--policy", required=True, type=Path, help="offline JSON safety policy")
    args = parser.parse_args()
    with args.policy.open(encoding="utf-8") as handle:
        policy = SafetyPolicy(**json.load(handle))
    with args.events.open(encoding="utf-8") as handle:
        records = [json.loads(line) for line in handle if line.strip()]
    report = replay(records, policy)
    print(json.dumps(report, indent=2))
    return 0 if report["total"] and report["rejected"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
