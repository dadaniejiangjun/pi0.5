"""Train-only statistics and pinned official processor persistence."""
from __future__ import annotations

import json
from pathlib import Path

from .data import CAMERAS, MANIFEST, admit_dataset, sha256, write_json


def train_statistics(root: str | Path, *, software_only: bool = False) -> dict:
    """Compute statistics only from declared training episodes, not metadata aggregate."""
    import numpy as np
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    root = Path(root)
    manifest = admit_dataset(root, software_only=software_only)
    ds = LeRobotDataset("local/piper_training", root=root, episodes=manifest["splits"]["train"])
    arrays = {key: [] for key in ("observation.state", "action", *CAMERAS)}
    image_moments = {key: {"sum": np.zeros(3), "square": np.zeros(3), "count": 0,
                           "min": np.full(3, np.inf), "max": np.full(3, -np.inf)} for key in CAMERAS}
    for row in ds:
        for key in arrays:
            value = row[key].numpy()
            if key in CAMERAS:
                pixels = value.reshape(3, -1).astype(np.float64)
                moments = image_moments[key]
                moments["sum"] += pixels.sum(1)
                moments["square"] += np.square(pixels).sum(1)
                moments["count"] += pixels.shape[1]
                moments["min"] = np.minimum(moments["min"], pixels.min(1))
                moments["max"] = np.maximum(moments["max"], pixels.max(1))
                # Bounded deterministic image quantiles; vector quantiles remain exact.
                arrays[key].append(pixels[:, ::max(1, (pixels.shape[1] + 4095) // 4096)].astype(np.float32))
            else:
                arrays[key].append(value.reshape(7, 1))
    stats = {}
    for key, values in arrays.items():
        axis_values = np.concatenate(values, axis=1)
        shape = (3, 1, 1) if key in CAMERAS else (7,)
        stats[key] = {"mean": axis_values.mean(1).reshape(shape).astype(np.float32),
                      "std": np.maximum(axis_values.std(1), 1e-6).reshape(shape).astype(np.float32),
                      "min": axis_values.min(1).reshape(shape).astype(np.float32),
                      "max": axis_values.max(1).reshape(shape).astype(np.float32),
                      "q01": np.quantile(axis_values, .01, axis=1).reshape(shape).astype(np.float32),
                      "q99": np.quantile(axis_values, .99, axis=1).reshape(shape).astype(np.float32)}
        if key in CAMERAS:
            moments = image_moments[key]
            mean = moments["sum"] / moments["count"]
            std = np.sqrt(np.maximum(0, moments["square"] / moments["count"] - mean ** 2))
            stats[key].update(mean=mean.reshape(shape).astype(np.float32),
                              std=np.maximum(std, 1e-6).reshape(shape).astype(np.float32),
                              min=moments["min"].reshape(shape).astype(np.float32),
                              max=moments["max"].reshape(shape).astype(np.float32))
    return stats


def save_act_processors(config, root: str | Path, output: str | Path, *, software_only=False):
    """Use official ACT factory; PI05 must use its own policy processor factory."""
    import torch
    from lerobot.policies.act.processor_act import make_act_pre_post_processors
    root, output = Path(root), Path(output)
    stats = train_statistics(root, software_only=software_only)
    tensors = {k: {n: torch.as_tensor(v) for n, v in values.items()} for k, values in stats.items()}
    pre, post = make_act_pre_post_processors(config, tensors)
    output.mkdir(parents=True, exist_ok=True)
    pre.save_pretrained(output, config_filename="policy_preprocessor.json")
    post.save_pretrained(output, config_filename="policy_postprocessor.json")
    write_json(output / "project_processor_identity.json", {
        "schema": "piper_processor_identity_v1", "policy_type": "act",
        "data_manifest_sha256": sha256(root / MANIFEST),
        "training_episodes": json.loads((root / MANIFEST).read_text())["splits"]["train"],
        "statistics_scope": "training episodes only", "source_kind": "synthetic" if software_only else "field",
        "files": {p.name: sha256(p) for p in output.iterdir()
                  if p.is_file() and (p.suffix == ".safetensors" or p.name.startswith("policy_"))}})
    return pre, post


def load_act_processors(output: str | Path):
    """Verify identity before official pipeline load; reject damaged state/config."""
    from lerobot.processor import PolicyProcessorPipeline
    from lerobot.processor.converters import (batch_to_transition, transition_to_batch,
                                             policy_action_to_transition, transition_to_policy_action)
    output = Path(output)
    identity = json.loads((output / "project_processor_identity.json").read_text())
    if identity.get("policy_type") != "act":
        raise ValueError("policy processor identity mismatch")
    for name, digest in identity["files"].items():
        file = output / name
        if not file.is_file() or sha256(file) != digest:
            raise ValueError(f"processor config/state missing or changed: {name}")
    pre = PolicyProcessorPipeline.from_pretrained(output, "policy_preprocessor.json", local_files_only=True,
                                                   to_transition=batch_to_transition, to_output=transition_to_batch)
    post = PolicyProcessorPipeline.from_pretrained(output, "policy_postprocessor.json", local_files_only=True,
                                                    to_transition=policy_action_to_transition,
                                                    to_output=transition_to_policy_action)
    return pre, post
