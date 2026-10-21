"""Verify online next-state captures and prepare independent offline training copies."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .data import CAMERAS, MANIFEST, contract, fingerprint, sha256, write_json

SEMANTICS = "next_observed_state_proxy_nominal_units"


def validate_row(row, recorded, *, next_row=None):
    """Verify measured target provenance independently of command audit."""
    import numpy as np
    source = row.get("source_map", {})
    current = source.get("state", {})
    target = source.get("proxy_target", {})
    future = target.get("state", {})
    if not all(row.get(k) is True for k in
               ("action_valid", "observation_valid", "dataset_camera_valid")):
        return False
    if (row.get("writer_placeholder") or row.get("action_semantics") != SEMANTICS
            or target.get("action_label_valid") is not True):
        return False
    for state in (current, future):
        if (state.get("feedback_fresh") is not True or state.get("fake") is not False
                or state.get("hardware_connected") is not True):
            return False
    if (not current.get("session_id") or current.get("session_id") != future.get("session_id")
            or not current.get("source_clock_id")
            or current.get("source_clock_id") != future.get("source_clock_id")):
        return False
    dt = future["source_monotonic_ns"] - current["source_monotonic_ns"]
    if not 75_000_000 <= dt <= 125_000_000 or dt != target.get("target_offset_ns"):
        return False
    if source.get("camera_pair_valid") is not True:
        return False
    for role in ("base", "wrist"):
        image = source.get("images", {}).get(role, {})
        if (image.get("valid") is not True or not image.get("device_name")
                or image.get("session_id") != current["session_id"]):
            return False
    for key, value in (("observation.state", current.get("values_float32")),
                       ("action", future.get("values_float32"))):
        array = np.asarray(value, dtype=np.float32)
        if array.shape != (7,) or not np.isfinite(array).all():
            return False
        if not np.array_equal(array, np.asarray(recorded[key], dtype=np.float32)):
            return False
    if next_row is not None:
        next_state = next_row["source_map"]["state"]
        if (next_state.get("sequence") != future.get("sequence")
                or next_state.get("source_monotonic_ns") != future.get("source_monotonic_ns")
                or next_state.get("session_id") != future.get("session_id")):
            return False
    return True


def contiguous_segments(valid, minimum=16):
    """Never join observations across an invalid interval."""
    segments, start = [], None
    for i, value in enumerate([*valid, False]):
        if value and start is None:
            start = i
        elif not value and start is not None:
            if i - start >= minimum:
                segments.append((start, i))
            start = None
    return segments


def verify_admission(root, manifest, *, software_only=False):
    if (software_only or manifest.get("training_eligible") is not True
            or manifest.get("formal_training_eligible") is not False
            or manifest.get("hardware_control_allowed") is not False
            or manifest.get("action_semantics") != SEMANTICS
            or manifest.get("raw_command_supervision") is not False):
        raise ValueError("online proxy qualification or action semantics changed")
    evidence_path = (root / manifest.get("online_admission_evidence", "__missing__")).resolve()
    if (not evidence_path.is_relative_to(root.resolve()) or not evidence_path.is_file()
            or sha256(evidence_path) != manifest.get("online_admission_evidence_sha256")):
        raise ValueError("online proxy admission evidence missing or changed")
    evidence = json.loads(evidence_path.read_text())
    if (evidence.get("schema") != "piper_online_proxy_admission_v1"
            or evidence.get("status") != "PASS_OFFLINE_PROXY"
            or evidence.get("all_fields_match") is not True
            or evidence.get("readback_rows", 0) < 32
            or len(evidence.get("sources", [])) < 2):
        raise ValueError("online proxy readback evidence incomplete")
    segments = evidence.get("segments", [])
    for split, episodes in manifest["splits"].items():
        expected = [x["episode_index"] for x in segments if x["split"] == split]
        if episodes != expected:
            raise ValueError("online proxy source split mismatch")
    if any(manifest["episode_origins"].get(str(x["episode_index"])) != x["origin"]
           or x["source_end_exclusive"] - x["source_start"] < 16 for x in segments):
        raise ValueError("online proxy segment provenance mismatch")


def prepare(sources, output, repo_id="local/piper_red_cube_online_proxy"):
    import av
    import numpy as np
    import pyarrow.parquet as pq
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    sources = sorted(Path(p).resolve(strict=True) for p in sources)
    if len(sources) < 2 or len(set(sources)) != len(sources):
        raise ValueError("need unique independent train and validation source episodes")
    output = Path(output).resolve()
    if output.exists() or any(output.is_relative_to(p) or p.is_relative_to(output) for p in sources):
        raise ValueError("training copy must use a new independent directory")
    holdout = set(sources[-max(1, len(sources) // 5):])
    features = {k: {"dtype": "float32", "shape": (7,), "names": [f"joint_{i}" for i in range(1, 7)] + ["gripper"]}
                for k in ("observation.state", "action")}
    features.update({k: {"dtype": "image", "shape": (480, 640, 3),
                         "names": ["height", "width", "channels"]} for k in CAMERAS})
    ds = LeRobotDataset.create(repo_id=repo_id, root=output, fps=10, features=features,
                               use_videos=False, robot_type="piper")
    evidence = {"schema": "piper_online_proxy_admission_v1", "status": "RUNNING",
                "action_semantics": SEMANTICS, "sources": [], "segments": [],
                "formal_training_eligible": False, "hardware_control_allowed": False}
    origins, splits, row_hashes, episode = {}, {"train": [], "validation": []}, [], 0
    for path in sources:
        before = fingerprint(path)
        prov = path / "provenance"
        if (prov / "pending_run.json").exists():
            raise ValueError(f"source still pending: {path}")
        labels = [json.loads(x) for x in (prov / "run_labels.jsonl").read_text().splitlines()]
        if len(labels) != 1 or not (labels[0].get("demonstration_label") == "SUCCESS"
                and labels[0].get("successful_complete_end") is True):
            raise ValueError(f"source lacks complete operator success: {path}")
        info = json.loads((path / "meta/info.json").read_text())
        if info["fps"] != 10 or info["total_episodes"] != 1:
            raise ValueError("requires single-episode 10 Hz captures")
        records = pq.read_table(next(path.glob("data/chunk-*/episode_*.parquet"))).to_pylist()
        rows = [json.loads(x) for x in (prov / "source_map.jsonl").read_text().splitlines()]
        if len(rows) != len(records) or len(rows) != info["total_frames"]:
            raise ValueError("source map/parquet row count mismatch")
        if any(abs(record["timestamp"] - index / 10) > 1e-4
               or rows[index].get("episode_frame_index") != index
               for index, record in enumerate(records)):
            raise ValueError("source grid is not a continuous indexed 10 Hz episode")
        valid = [validate_row(row, record, next_row=rows[i+1] if i+1 < len(rows) else None)
                 for i, (row, record) in enumerate(zip(rows, records))]
        segments = contiguous_segments(valid)
        if not segments:
            raise ValueError(f"source has no H16 valid continuous segment: {path}")
        videos = {}
        for key in CAMERAS:
            video_path = path / info["video_path"].format(episode_chunk=0, episode_index=0, video_key=key)
            with av.open(str(video_path)) as stream:
                videos[key] = [f.to_ndarray(format="rgb24") for f in stream.decode(video=0)]
            if len(videos[key]) != len(rows):
                raise ValueError(f"video count mismatch: {video_path}")
        for start, end in segments:
            split = "validation" if path in holdout else "train"
            splits[split].append(episode)
            origins[str(episode)] = path.name
            for index in range(start, end):
                frame = {k: np.asarray(records[index][k], dtype=np.float32) for k in ("observation.state", "action")}
                frame.update({k: videos[k][index] for k in CAMERAS})
                frame["task"] = rows[index]["task"]
                ds.add_frame(frame)
                row_hashes.append({"episode_index": episode, "source": path.name, "source_row": index,
                    "fields": {k: __import__('hashlib').sha256(frame[k].tobytes()).hexdigest() for k in features}})
            ds.save_episode()
            evidence["segments"].append({"episode_index": episode, "origin": path.name,
                                         "source_start": start, "source_end_exclusive": end, "split": split})
            episode += 1
        if fingerprint(path) != before:
            raise RuntimeError("source changed during preparation")
        evidence["sources"].append({"origin": path.name, "files": before,
            "rows": len(rows), "invalid_rows_excluded": valid.count(False),
            "retained_rows": sum(b-a for a,b in segments), "operator_success_only": True})
        print(json.dumps({"prepared_source": path.name, "segments": len(segments),
                          "retained_rows": sum(b-a for a,b in segments)}), flush=True)
        del videos
    ds.finalize()
    # Decode every output frame and verify every field against the source copy.
    readback = LeRobotDataset(repo_id, root=output)
    for i, expected in enumerate(row_hashes):
        record = readback[i]
        for key, digest in expected["fields"].items():
            array = record[key].numpy()
            if key in CAMERAS:
                array = np.rint(array.transpose(1, 2, 0) * 255).astype(np.uint8)
            else:
                array = array.astype(np.float32)
            if __import__('hashlib').sha256(array.tobytes()).hexdigest() != digest:
                raise ValueError(f"prepared readback mismatch at {i} {key}")
    if not splits["train"] or not splits["validation"]:
        raise ValueError("missing independent train/validation episodes")
    write_json(output / "provenance/online_proxy_rows.json", row_hashes)
    evidence.update(status="PASS_OFFLINE_PROXY", readback_rows=len(row_hashes), all_fields_match=True)
    evidence_path = output / "provenance/online_proxy_admission.json"
    write_json(evidence_path, evidence)
    c = contract(); c["units"] = "nominal_unverified"
    manifest = {"schema": "piper_lerobot_training_v1", "source_kind": "verified_online_feedback_proxy",
        "repo_id": repo_id, "training_eligible": True, "formal_training_eligible": False,
        "hardware_control_allowed": False, "action_semantics": SEMANTICS, "raw_command_supervision": False,
        "invalid_rows": 0, "contract": c, "splits": splits, "episode_origins": origins,
        "online_admission_evidence": str(evidence_path.relative_to(output)),
        "online_admission_evidence_sha256": sha256(evidence_path),
        "dataset_files": fingerprint(output)}
    write_json(output / MANIFEST, manifest)
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sources", nargs="+", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    prepare(args.sources, args.output)
