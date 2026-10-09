from collections import deque
import json
import struct
import time
from types import SimpleNamespace
import pytest
from laptop.robot_client.piper_session import PiperSession
from laptop.transport.can_transport import CanFrame


class Bridge:
    mode = SimpleNamespace(value="TEST")
    state = SimpleNamespace(value="READY")
    def __init__(self, frames=()):
        self.frames = deque(frames)
        self.opens = self.closes = self.sends = 0
    def start_read_only(self):
        self.opens += 1
    def close(self):
        self.closes += 1
    def receive_frame(self, timeout_ms):
        return self.frames.popleft() if self.frames else None
    def send_diagnostic_query(self, query, **kwargs):
        self.sends += 1
        raise RuntimeError("delivery unknown")


def test_decode_and_freshness_and_log(tmp_path):
    now = time.perf_counter_ns()
    payloads = {0x2A1: bytes((1, 0, 1, 2, 0, 0, 0, 0)),
                0x2A5: struct.pack(">ii", 929, -1000),
                0x2A6: bytes(8), 0x2A7: bytes(8),
                0x2A8: struct.pack(">ihBB", 1234, -12, 64, 0),
                **{i: bytes((0, 0, 0, 0, 0, 64, 0, 0)) for i in range(0x261, 0x267)}}
    frames = [CanFrame(arbitration_id=i, data=data, received_monotonic_ns=now,
                       is_extended_id=False, is_error_frame=False)
              for i, data in payloads.items()]
    bridge = Bridge(frames)
    path = tmp_path / "bus.jsonl"
    with PiperSession(path, bridge=bridge) as session:
        assert session.poll(0.001) == 11
        state = session.state()
        assert state["complete_fresh"]
        assert state["joint_deg_nominal"][:2] == [0.929, -1]
        assert state["gripper"] == dict(position_raw=1234, effort_raw=-12, status_raw=64)
        assert all(state["motor_enabled"])
        assert not session.state(max_age_ms=0.0001)["complete_fresh"]
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert len([r for r in rows if r["event"] == "rx"]) == 11
    assert rows[-1]["event"] == "closed"
    assert bridge.opens == bridge.closes == 1


def test_log_collision_prevents_connect(tmp_path):
    path = tmp_path / "bus.jsonl"
    path.write_text("existing")
    bridge = Bridge()
    with pytest.raises(FileExistsError):
        PiperSession(path, bridge=bridge).connect()
    assert bridge.opens == 0


def test_send_failure_audited_and_not_retried(tmp_path):
    bridge = Bridge()
    path = tmp_path / "bus.jsonl"
    with PiperSession(path, bridge=bridge) as session:
        with pytest.raises(RuntimeError, match="delivery unknown"):
            session.send_query("fake", operator_approved=True)
        assert session.state()["missing_channels"]
    assert bridge.sends == 1
    events = [json.loads(line)["event"] for line in path.read_text().splitlines()]
    assert "tx_requested" in events and "tx_failed_no_retry" in events


def test_target_wait_missing_feedback_never_sends(tmp_path):
    bridge = Bridge()
    with PiperSession(tmp_path / "bus.jsonl", bridge=bridge) as session:
        result = session.wait_joint_target([1000, 0, 0, 0, 0, 0], timeout_s=0.001)
        assert result["status"] == "TARGET_NOT_CONFIRMED_NO_RETRY"
    assert bridge.sends == 0
