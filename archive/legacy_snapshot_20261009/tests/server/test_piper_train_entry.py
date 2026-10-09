from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from server.training.piper_train_entry import execute_training, prepare


class PiperTrainEntryTest(unittest.TestCase):
    def test_candidate_is_wired_but_entry_remains_locked(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            report = prepare(Path(temp), repo_id="offline/fixture", base_params_path="/missing/params")
        self.assertEqual(report["status"], "TRAINING_LOCKED")
        self.assertFalse(report["training_started"])
        self.assertTrue(report["candidate_config"]["pi05"])
        self.assertEqual(report["candidate_config"]["model_action_dim"], 32)
        self.assertEqual(report["candidate_config"]["action_horizon"], 16)
        self.assertTrue(report["candidate_config"]["piper_data_factory"])

    def test_execute_is_unconditionally_refused(self) -> None:
        with self.assertRaisesRegex(PermissionError, "locked"):
            execute_training()


if __name__ == "__main__":
    unittest.main()
