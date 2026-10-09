from __future__ import annotations

import unittest

from server.training.public_data_contract import (
    LOCAL_ACTION_SEMANTICS, PUBLIC_ACTION_SEMANTICS,
    PUBLIC_ACTION_UNITS, inspect_public_acceptance,
)


class PublicDataContractTest(unittest.TestCase):
    def test_public_contract_is_not_local_contract(self) -> None:
        self.assertNotEqual(PUBLIC_ACTION_SEMANTICS, LOCAL_ACTION_SEMANTICS)
        self.assertEqual(PUBLIC_ACTION_UNITS, ("degree",) * 6 + ("binary_open_fraction",))

    def test_readiness_requires_reviewed_units_source_and_prediction_only(self) -> None:
        provenance = {"source_revision": "revision-1", "archive_sha256": "sha-1"}
        acceptance = {
            "schema": "piper_public_data_acceptance_v1",
            "action_semantics": PUBLIC_ACTION_SEMANTICS,
            "joint_unit": "degree",
            "gripper_unit": "binary_open_fraction",
            "units_reviewed": True,
            "source_revision": "revision-1",
            "archive_sha256": "sha-1",
            "hardware_control_allowed": False,
        }
        self.assertTrue(all(inspect_public_acceptance(acceptance, provenance).values()))
        for changed in (
            {"action_semantics": LOCAL_ACTION_SEMANTICS},
            {"joint_unit": "radian"},
            {"gripper_unit": "meter"},
            {"units_reviewed": False},
            {"archive_sha256": "other"},
            {"hardware_control_allowed": True},
        ):
            self.assertFalse(all(inspect_public_acceptance(
                {**acceptance, **changed}, provenance).values()))


if __name__ == "__main__":
    unittest.main()
