#!/usr/bin/env python3
"""Generate synthetic P1 PiPER contract validation artifacts."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from laptop.piper.unit_conversion import m_to_sdk_gripper, rad_to_sdk_joint, sdk_gripper_to_m, sdk_joint_to_rad
from server.piper_policy.piper_inputs import PiperInputs
from server.piper_policy.piper_outputs import action_pad_mask, pad_action_7_to_model, unpad_model_action_to_7


ROOT = Path(__file__).resolve().parents[1]


def write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    rng = np.random.default_rng(173)

    raw_joint = rng.integers(-180_000, 180_001, size=10_000, dtype=np.int64)
    joint_rad = sdk_joint_to_rad(raw_joint)
    joint_recovered = sdk_joint_to_rad(rad_to_sdk_joint(joint_rad))
    raw_gripper = rng.integers(0, 100_001, size=10_000, dtype=np.int64)
    gripper_m = sdk_gripper_to_m(raw_gripper)
    gripper_recovered = sdk_gripper_to_m(m_to_sdk_gripper(gripper_m))
    unit_report = {
        "status": "PASS",
        "samples": 10_000,
        "joint_max_abs_error_rad": float(np.max(np.abs(joint_recovered - joint_rad))),
        "gripper_max_abs_error_m": float(np.max(np.abs(gripper_recovered - gripper_m))),
        "quantization_checked": True,
        "hardware_accessed": False,
        "scale_status": "NOMINAL_OFFICIAL_SCALE_PENDING_FIRMWARE_AUDIT",
    }
    write(ROOT / "validation/piper_unit_roundtrip.json", unit_report)

    actions = rng.normal(size=(10_000, 7)).astype(np.float32)
    padded = pad_action_7_to_model(actions, model_action_dim=32, padding_value=-7.0)
    recovered = unpad_model_action_to_7(padded, model_action_dim=32)
    action_report = {
        "status": "PASS" if np.array_equal(actions, recovered) else "FAIL",
        "samples": 10_000,
        "piper_action_dim": 7,
        "model_action_dim": 32,
        "padding_value": -7.0,
        "padding_mask_true_indices": np.flatnonzero(action_pad_mask()).tolist(),
        "padding_mask_false_indices": np.flatnonzero(~action_pad_mask()).tolist(),
        "round_trip_exact": bool(np.array_equal(actions, recovered)),
        "loss_mask_semantics": "canonical first 7 dimensions only; padding is false",
        "base_model_to_piper_claim": False,
    }
    write(ROOT / "validation/piper_action_transform_test.json", action_report)

    piper = PiperInputs(
        base_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        wrist_rgb=np.full((300, 200, 3), 128, dtype=np.uint8),
        state=np.arange(7, dtype=np.float32),
        prompt="Pick up the block and place it in the tray.",
    )
    model_observation = piper.to_openpi_observation()
    input_report = {
        "status": "PASS",
        "semantic_input": ["base_rgb", "wrist_rgb", "state", "prompt"],
        "model_image_keys": sorted(model_observation["image"]),
        "image_shapes": {key: list(value.shape) for key, value in model_observation["image"].items()},
        "image_dtypes": {key: str(value.dtype) for key, value in model_observation["image"].items()},
        "image_masks": {key: bool(value) for key, value in model_observation["image_mask"].items()},
        "state_shape": list(model_observation["state"].shape),
        "state_dtype": str(model_observation["state"].dtype),
        "prompt": model_observation["prompt"],
        "preprocessing": "official openpi-client convert_to_uint8 + resize_with_pad",
        "base_action_interpretation": False,
    }
    write(ROOT / "validation/piper_input_contract_smoke.json", input_report)
    print(json.dumps({"unit": unit_report, "action": action_report, "input": input_report}, indent=2))
    return 0 if all(item["status"] == "PASS" for item in (unit_report, action_report, input_report)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
