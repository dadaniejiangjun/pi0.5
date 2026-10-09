from __future__ import annotations

import subprocess
import unittest
from unittest.mock import patch

from laptop.diagnostics.camera_identity import (
    _parse_pnputil_camera_inventory,
    bind_camera_roles,
    windows_camera_inventory,
)


PNPUTIL_OUTPUT = r"""Microsoft PnP Utility

Instance ID:                USB\VID_0408&PID_4035&MI_00\6&239fb1a8&0&0000
Device Description:         ACER HD User Facing
Class Name:                 Camera
Status:                     Started

Instance ID:                USB\VID_2BC5&PID_0557&MI_00\7&3164c1cd&0&0000
Device Description:         Dabai DC1
Class Name:                 Camera
Status:                     Started
"""


class CameraIdentityInventoryTest(unittest.TestCase):
    def test_pnputil_parser_maps_started_devices_to_ok(self) -> None:
        inventory = _parse_pnputil_camera_inventory(PNPUTIL_OUTPUT)

        self.assertEqual(len(inventory), 2)
        self.assertEqual(inventory[0]["FriendlyName"], "ACER HD User Facing")
        self.assertEqual(inventory[0]["Status"], "OK")
        self.assertEqual(inventory[1]["FriendlyName"], "Dabai DC1")

    def test_wmi_permission_error_falls_back_to_connected_pnputil_listing(self) -> None:
        wmi_error = subprocess.CalledProcessError(1, ["powershell.exe"], stderr="Access denied")
        pnputil_result = subprocess.CompletedProcess(
            args=["pnputil.exe"], returncode=0, stdout=PNPUTIL_OUTPUT, stderr="",
        )
        with patch("laptop.diagnostics.camera_identity.subprocess.run",
                   side_effect=[wmi_error, pnputil_result]) as run:
            inventory = windows_camera_inventory()

        self.assertEqual(len(inventory), 2)
        self.assertEqual(inventory[0]["Status"], "OK")
        self.assertIn("/connected", run.call_args_list[1].args[0])

    def test_role_binding_matches_windows_instance_ids_case_insensitively(self) -> None:
        inventory = _parse_pnputil_camera_inventory(PNPUTIL_OUTPUT)
        bound = bind_camera_roles(
            0, 1,
            base_device_id=r"USB\VID_0408&PID_4035&MI_00\6&239FB1A8&0&0000",
            wrist_device_id=r"USB\VID_2BC5&PID_0557&MI_00\7&3164C1CD&0&0000",
            roles_visually_confirmed=True,
            inventory=inventory,
        )

        self.assertEqual(
            bound["roles"]["wrist"]["windows_instance_id"],
            r"USB\VID_2BC5&PID_0557&MI_00\7&3164c1cd&0&0000",
        )

    def test_pnputil_does_not_authorize_disabled_camera(self) -> None:
        inventory = _parse_pnputil_camera_inventory(
            PNPUTIL_OUTPUT.replace("Status:                     Started", "Status:                     Stopped")
        )
        with self.assertRaisesRegex(ValueError, "not present and OK"):
            bind_camera_roles(
                0, 1,
                base_device_id=r"USB\VID_0408&PID_4035&MI_00\6&239FB1A8&0&0000",
                wrist_device_id=r"USB\VID_2BC5&PID_0557&MI_00\7&3164C1CD&0&0000",
                roles_visually_confirmed=True,
                inventory=inventory,
            )


if __name__ == "__main__":
    unittest.main()
