from __future__ import annotations

import unittest

import numpy as np

from server.piper_policy.piper_inputs import PiperInputs
from server.piper_policy.piper_outputs import (
    DEFAULT_MODEL_ACTION_DIM,
    PiperOutputs,
    action_pad_mask,
    pad_action_7_to_model,
    unpad_model_action_to_7,
)


class PiperContractTest(unittest.TestCase):
    def test_synthetic_input_maps_to_openpi_keys(self) -> None:
        observation = PiperInputs(
            base_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
            wrist_rgb=np.full((240, 320, 3), 127, dtype=np.uint8),
            state=np.arange(7, dtype=np.float32),
            prompt="Pick up the block and place it in the tray.",
        ).to_openpi_observation()
        self.assertEqual(observation["image"]["base_0_rgb"].shape, (224, 224, 3))
        self.assertEqual(observation["image"]["left_wrist_0_rgb"].shape, (224, 224, 3))
        self.assertEqual(observation["image"]["right_wrist_0_rgb"].shape, (224, 224, 3))
        self.assertEqual(observation["state"].dtype, np.float32)
        self.assertEqual(observation["state"].shape, (7,))
        self.assertTrue(bool(observation["image_mask"]["base_0_rgb"]))
        self.assertTrue(bool(observation["image_mask"]["left_wrist_0_rgb"]))
        self.assertFalse(bool(observation["image_mask"]["right_wrist_0_rgb"]))

    def test_10000_action_pad_unpad_samples(self) -> None:
        rng = np.random.default_rng(172)
        actions = rng.normal(size=(10_000, 7)).astype(np.float32)
        padded = pad_action_7_to_model(actions, model_action_dim=DEFAULT_MODEL_ACTION_DIM, padding_value=-7.0)
        self.assertEqual(padded.shape, (10_000, DEFAULT_MODEL_ACTION_DIM))
        np.testing.assert_array_equal(padded[:, 7:], -7.0)
        np.testing.assert_array_equal(unpad_model_action_to_7(padded), actions)
        np.testing.assert_array_equal(action_pad_mask(), np.array([True] * 7 + [False] * 25))

    def test_canonical_output_is_not_an_sdk_command(self) -> None:
        action = PiperOutputs.from_model_action(np.arange(DEFAULT_MODEL_ACTION_DIM, dtype=np.float32))
        self.assertEqual(action.action.shape, (7,))
        np.testing.assert_array_equal(action.action, np.arange(7, dtype=np.float32))


if __name__ == "__main__":
    unittest.main()
