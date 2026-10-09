from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from server.training.piper_openpi_config import make_unregistered_config
from server.training.check_piper_readiness import inspect
from server.training.piper_train_entry import build_config
from server.training.proxy_piper_tiny_fit import EXP_NAME, make_config


class PiperOpenPIOfflineTest(unittest.TestCase):
    def test_train_eval_serve_share_one_norm_stats_directory(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            config = build_config(
                Path(root), repo_id="local/piper_v1", base_params_path="/params",
                mode="local-tiny-fit", steps=2,
            )
            self.assertIsNone(config.data.assets.assets_dir)
            self.assertEqual(config.assets_dirs, Path(root) / "assets" / config.name)
            self.assertEqual(
                config.data.create(config.assets_dirs, config.model).asset_id,
                "piper",
            )

    def test_proxy_experiment_is_isolated_and_name_is_bounded(self) -> None:
        original = make_config()
        ten_step = make_config(steps=10, exp_name="cef166e2_ten_step")
        self.assertEqual(original.exp_name, EXP_NAME)
        self.assertNotEqual(original.checkpoint_dir, ten_step.checkpoint_dir)
        self.assertEqual(ten_step.num_train_steps, 10)
        self.assertFalse(ten_step.checkpoint_dir == original.checkpoint_dir)
        with self.assertRaises(ValueError):
            make_config(exp_name="../unsafe")

    def test_empty_server_dataset_remains_blocked(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            report = inspect(Path(root))
            self.assertEqual(report["status"], "BLOCKED")
            self.assertFalse(report["training_started"])
            self.assertFalse(any(report["checks"].values()))

    def test_two_camera_and_7d_transform(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            config = make_unregistered_config(
                repo_id="offline/piper_synthetic_smoke",
                base_params_path="/nonexistent/params",
                assets_root=root,
            )
            data = config.data.create(Path(root), config.model)
            self.assertIsNone(data.norm_stats)
            self.assertEqual(data.action_sequence_keys, ("action",))
            raw = {
                "observation.images.image": np.zeros((3, 64, 64), dtype=np.float32),
                "observation.images.wrist_image": np.zeros((64, 64, 3), dtype=np.uint8),
                "observation.state": np.arange(7, dtype=np.float32),
                "action": np.zeros((16, 7), dtype=np.float32),
                "prompt": "synthetic test",
            }
            mapped = data.repack_transforms.inputs[0](raw)
            model_input = data.data_transforms.inputs[0](mapped)
            self.assertEqual(model_input["state"].shape, (7,))
            self.assertEqual(model_input["actions"].shape, (16, 7))
            self.assertEqual(model_input["image"]["base_0_rgb"].shape, (64, 64, 3))
            self.assertFalse(model_input["image_mask"]["right_wrist_0_rgb"])
            output = data.data_transforms.outputs[0]({"actions": np.zeros((16, 32), dtype=np.float32)})
            self.assertEqual(output["actions"].shape, (16, 7))

    def test_rejects_bad_action_and_nonfinite_state(self) -> None:
        from server.training.piper_openpi_config import PiperOpenPIInputs

        common = {"base_rgb": np.zeros((4, 4, 3), dtype=np.uint8),
                  "wrist_rgb": np.zeros((4, 4, 3), dtype=np.uint8)}
        with self.assertRaises(ValueError):
            PiperOpenPIInputs()({**common, "state": np.zeros(7), "actions": np.zeros((16, 6))})
        state = np.zeros(7)
        state[0] = np.nan
        with self.assertRaises(ValueError):
            PiperOpenPIInputs()({**common, "state": state})


if __name__ == "__main__":
    unittest.main()
