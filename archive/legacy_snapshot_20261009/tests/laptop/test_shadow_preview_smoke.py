import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

from laptop.client import shadow_preview_smoke as preview_smoke
from laptop.client.live_shadow import load_artifact_identity_lock


ROOT = Path(__file__).resolve().parents[2]


class FakeSocket:
    def close(self):
        self.closed = True


class FakePolicyClient:
    def __init__(self, metadata):
        self._ws = FakeSocket()
        self.metadata = metadata

    def get_server_metadata(self):
        return self.metadata


def _metadata(lock):
    return {
        **lock,
        "action_horizon": 16,
        "first_target_dt_s": 0.5,
        "prediction_only": True,
        "control_enabled": False,
        "robot_control_ready": False,
    }


def _preview_tree(tmp_path):
    raw = tmp_path / "raw-episode"
    preview = tmp_path / "preview"
    raw.mkdir()
    preview.mkdir()
    (preview / "aligned_episode.json").write_text(json.dumps({
        "source": "passive_wiring_smoke",
        "raw_episode_id": raw.name,
        "training_eligible": False,
    }), encoding="utf-8")
    rows = []
    for index in range(3):
        Image.fromarray(np.zeros((8, 8, 3), dtype=np.uint8)).save(
            preview / f"base_{index}.png"
        )
        Image.fromarray(np.full((8, 8, 3), index, dtype=np.uint8)).save(
            preview / f"wrist_{index}.png"
        )
        rows.append({
            "base_image": f"base_{index}.png",
            "wrist_image": f"wrist_{index}.png",
            "joint_rad_nominal_only": [0.0] * 6,
            "gripper_raw": 40000,
        })
    (preview / "frames.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
    )
    return raw, preview


def test_source_verified_preview_runs_to_reject_only_without_tx(monkeypatch, tmp_path):
    lock = load_artifact_identity_lock(
        ROOT / "configs/runtime/pi05_proxy_shadow_identity_lock.json"
    )
    client = FakePolicyClient(_metadata(lock))
    raw, preview = _preview_tree(tmp_path)
    monkeypatch.setattr(preview_smoke, "verify_source", lambda *_: {
        "source_frame_extraction_verified": True,
        "samples_checked": 3,
    })
    monkeypatch.setattr(
        preview_smoke,
        "_bounded_infer",
        lambda *_: {"actions": np.zeros((16, 7), dtype=np.float32)},
    )

    report = preview_smoke.run(
        raw,
        preview,
        client_factory=lambda: client,
        identity_lock_path=ROOT / "configs/runtime/pi05_proxy_shadow_identity_lock.json",
    )

    assert report["status"] == "RECORDED_PREVIEW_REJECTED_NO_TX"
    assert report["training_eligible"] is False
    assert len(report["samples"]) == 3
    assert report["safety_audit"]["transport_send_attempts"] == 0
    assert report["safety_audit"]["can_data_tx_calls"] == 0
    for decision in report["safety_audit"]["decisions"]:
        assert decision["hardware_control_allowed"] is False
        assert decision["transport_send_attempted"] is False
        assert "next_observed_state_proxy_is_not_command" in decision["reasons"]
        assert "robot_disconnected" in decision["reasons"]
    assert client._ws.closed is True


def test_preview_rejects_server_identity_mismatch_and_still_closes_client(monkeypatch, tmp_path):
    lock_path = ROOT / "configs/runtime/pi05_proxy_shadow_identity_lock.json"
    lock = load_artifact_identity_lock(lock_path)
    bad_metadata = _metadata(lock)
    bad_metadata["checkpoint_sha256"] = "0" * 64
    client = FakePolicyClient(bad_metadata)
    raw, preview = _preview_tree(tmp_path)
    monkeypatch.setattr(preview_smoke, "verify_source", lambda *_: {
        "source_frame_extraction_verified": True,
    })
    with pytest.raises(PermissionError, match="artifact identity"):
        preview_smoke.run(
            raw,
            preview,
            client_factory=lambda: client,
            identity_lock_path=lock_path,
        )
    assert client._ws.closed is True
