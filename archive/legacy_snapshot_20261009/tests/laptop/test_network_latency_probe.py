from __future__ import annotations

import math

import numpy as np
import pytest

from laptop.client import network_latency_probe as probe
from laptop.client.network_latency_probe import percentile_summary


def test_percentile_summary_reports_tail_and_extrema():
    result = percentile_summary([1.0, 2.0, 3.0, 4.0, 5.0])

    assert result["min"] == 1.0
    assert result["p50"] == 3.0
    assert result["p90"] == 4.6
    assert result["p95"] == 4.8
    assert result["p99"] == 4.96
    assert result["max"] == 5.0
    assert result["mean"] == 3.0


@pytest.mark.parametrize("values", [[], [float("nan")], [float("inf")], [-0.1]])
def test_percentile_summary_rejects_empty_or_invalid_samples(values):
    with pytest.raises(ValueError):
        percentile_summary(values)


class _FakeSocket:
    def close(self):
        pass


class _FakeClient:
    def __init__(self, metadata):
        self._ws = _FakeSocket()
        self._metadata = metadata

    def get_server_metadata(self):
        return self._metadata


def _prepare_benchmark(monkeypatch, metadata_sequence, infer):
    clients = iter(_FakeClient(metadata) for metadata in metadata_sequence)
    monkeypatch.setattr(probe, "_check_listener", lambda *args: None)
    monkeypatch.setattr(probe, "load_artifact_identity_lock", lambda path: {})
    monkeypatch.setattr(probe, "validate_policy_metadata", lambda *args: None)
    monkeypatch.setattr(probe, "_bounded_infer", infer)
    return lambda: next(clients)


def _synthetic_response():
    return {
        "actions": np.zeros((16, 7), dtype=np.float32),
        "policy_timing": {"infer_ms": 0.0},
        "server_timing": {"infer_ms": 0.0},
    }


def _run_fake_benchmark(monkeypatch, client_factory):
    return probe.benchmark(
        host="127.0.0.1",
        port=8003,
        requests=1,
        warmup=1,
        timeout_s=1.0,
        identity_lock_path=probe.Path("unused.json"),
        transport_label="unit-test",
        client_factory=client_factory,
    )


def test_benchmark_passes_synthetic_requests_and_reconnect_identity(monkeypatch):
    factory = _prepare_benchmark(
        monkeypatch,
        metadata_sequence=[{"artifact": "locked"}, {"artifact": "locked"}],
        infer=lambda *args: _synthetic_response(),
    )

    report = _run_fake_benchmark(monkeypatch, factory)

    assert report["status"] == "PASS_SYNTHETIC_REQUEST_BENCHMARK"
    assert report["requests"]["measured_successfully"] == 1
    assert report["reconnect"]["metadata_identity_matches"] is True
    assert report["safety"]["hardware_control_allowed"] is False
    assert report["safety"]["robot_command_sent"] is False


def test_benchmark_fails_closed_on_inference_timeout(monkeypatch):
    factory = _prepare_benchmark(
        monkeypatch,
        metadata_sequence=[{"artifact": "locked"}],
        infer=lambda *args: (_ for _ in ()).throw(TimeoutError("deadline")),
    )

    report = _run_fake_benchmark(monkeypatch, factory)

    assert report["status"] == "FAIL_OR_INCOMPLETE"
    assert report["requests"]["measured_successfully"] == 0
    assert report["requests"]["failures"][0]["error_type"] == "TimeoutError"
    assert report["reconnect"]["metadata_identity_matches"] is False


def test_benchmark_fails_closed_on_invalid_response(monkeypatch):
    factory = _prepare_benchmark(
        monkeypatch,
        metadata_sequence=[{"artifact": "locked"}],
        infer=lambda *args: (_ for _ in ()).throw(ValueError("invalid 16x7 actions")),
    )

    report = _run_fake_benchmark(monkeypatch, factory)

    assert report["status"] == "FAIL_OR_INCOMPLETE"
    assert report["requests"]["failures"][0]["error_type"] == "ValueError"
    assert report["safety"]["hardware_control_allowed"] is False


def test_benchmark_fails_closed_when_identity_changes_after_reconnect(monkeypatch):
    factory = _prepare_benchmark(
        monkeypatch,
        metadata_sequence=[{"artifact": "initial"}, {"artifact": "changed"}],
        infer=lambda *args: _synthetic_response(),
    )

    report = _run_fake_benchmark(monkeypatch, factory)

    assert report["status"] == "FAIL_OR_INCOMPLETE"
    assert report["reconnect"]["metadata_identity_matches"] is False
    assert report["reconnect"]["failure"]["error_type"] == "PermissionError"
