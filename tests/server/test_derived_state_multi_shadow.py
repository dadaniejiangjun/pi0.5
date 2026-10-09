from __future__ import annotations

from pathlib import Path

import pytest

from server.training.proxy_multi_episode_contract import ACTION_SEMANTICS, ACTION_TARGET_KIND
from server.serve_derived_state_multi_shadow import (
    build_server_metadata,
    create_server,
    validate_runtime,
    validate_training_record,
)


def _training(tmp_path: Path) -> tuple[dict, dict]:
    dataset = tmp_path / "dataset"
    checkpoint = tmp_path / "checkpoint"
    stats = "a" * 64
    checkpoint_sha = "b" * 64
    dataset_sha = "c" * 64
    current_commit = "d" * 40
    record = {
        "schema": "piper_derived_state_multi_episode_training_v1",
        "action_semantics": ACTION_SEMANTICS,
        "action_target_kind": ACTION_TARGET_KIND,
        "prediction_only": True,
        "hardware_control_allowed": False,
        "training_scope": "all_successful_episodes_train_only_no_holdout",
        "same_data_evaluation_only": True,
        "dataset_path": str(dataset),
        "dataset_sha256": dataset_sha,
        "checkpoint_path": str(checkpoint),
        "checkpoint_tree_sha256": checkpoint_sha,
        "norm_stats_sha256": stats,
        "openpi_commit": current_commit,
        "dataset_episodes": 5,
        "repo_id": "local/five_demos",
        "steps": 20,
        "experiment_name": "five_demos_pilot",
        "base_params_path": str(tmp_path / "base_params"),
        "training_provenance_path": str(tmp_path / "training_provenance.json"),
    }
    observed = {
        "dataset_path": dataset,
        "checkpoint": checkpoint,
        "dataset_sha256": dataset_sha,
        "checkpoint_sha256": checkpoint_sha,
        "stats_sha256": stats,
        "checkpoint_stats_sha256": stats,
        "current_openpi_commit": current_commit,
    }
    return record, observed


def test_shadow_runtime_is_loopback_and_bounded() -> None:
    validate_runtime()
    with pytest.raises(ValueError, match="loopback"):
        validate_runtime(host="0.0.0.0")
    with pytest.raises(ValueError, match="sampling_steps"):
        validate_runtime(sampling_steps=11)


def test_training_record_must_match_dataset_checkpoint_stats_and_openpi(tmp_path: Path) -> None:
    record, observed = _training(tmp_path)
    validate_training_record(record, **observed)
    validate_training_record(dict(record, dataset_episodes=1), **observed)

    wrong_hash = dict(observed, checkpoint_sha256="e" * 64)
    with pytest.raises(ValueError, match="checkpoint"):
        validate_training_record(record, **wrong_hash)

    wrong_commit = dict(observed, current_openpi_commit="f" * 40)
    with pytest.raises(ValueError, match="OpenPI commit"):
        validate_training_record(record, **wrong_commit)


@pytest.mark.parametrize(
    "field,value,match",
    [
        ("hardware_control_allowed", True, "prediction-only"),
        ("action_semantics", "absolute_joint_position", "prediction-only"),
        ("dataset_episodes", 0, "at least one"),
    ],
)
def test_training_record_rejects_control_semantics_and_too_few_episodes(
        tmp_path: Path, field: str, value, match: str) -> None:
    record, observed = _training(tmp_path)
    record[field] = value
    with pytest.raises(ValueError, match=match):
        validate_training_record(record, **observed)


def test_server_factory_receives_locked_prediction_only_metadata(
        tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    provenance = tmp_path / "training_provenance.json"
    provenance.write_text("{}\n", encoding="utf-8")
    checkpoint = tmp_path / "checkpoint"
    config = object()
    monkeypatch.setattr("server.serve_derived_state_multi_shadow.build_config",
                        lambda **kwargs: config)
    calls = {}

    def policy_factory(actual_config, actual_checkpoint, *, sample_kwargs):
        calls["policy"] = (actual_config, actual_checkpoint, sample_kwargs)
        return "fake-policy"

    class FakeServer:
        def __init__(self, policy, **kwargs):
            calls["server"] = (policy, kwargs)

    training = {
        "repo_id": "local/five_demos",
        "dataset_sha256": "c" * 64,
        "action_semantics": ACTION_SEMANTICS,
        "action_target_kind": ACTION_TARGET_KIND,
        "raw_command_supervision": False,
        "steps": 20,
        "experiment_name": "five_demos_pilot",
    }
    artifacts = {
        "audit": {"repo_id": "local/five_demos", "hf_lerobot_home": str(tmp_path)},
        "training": training,
        "training_provenance_path": provenance,
        "checkpoint": checkpoint,
        "checkpoint_sha256": "b" * 64,
        "normalization_sha256": "a" * 64,
        "openpi_commit": "d" * 40,
        "base_params_path": tmp_path / "base_params",
    }

    server = create_server(artifacts, host="127.0.0.1", port=8020,
                           sampling_steps=10, policy_factory=policy_factory,
                           server_factory=FakeServer)

    assert isinstance(server, FakeServer)
    assert calls["policy"] == (config, checkpoint, {"num_steps": 10})
    policy, server_kwargs = calls["server"]
    assert policy == "fake-policy"
    assert server_kwargs["host"] == "127.0.0.1"
    assert server_kwargs["port"] == 8020
    assert server_kwargs["metadata"]["prediction_only"] is True
    assert server_kwargs["metadata"]["hardware_control_allowed"] is False
    assert server_kwargs["metadata"]["action_horizon"] == 16
    assert server_kwargs["metadata"]["action_dimension"] == 7
    assert server_kwargs["metadata"]["first_target_dt_s"] == 0.1
    assert server_kwargs["metadata"]["action_target_kind"] == ACTION_TARGET_KIND
    assert server_kwargs["metadata"]["can_opened"] is False
    assert server_kwargs["metadata"]["can_data_tx_calls"] == 0
