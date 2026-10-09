"""Record Windows camera PnP identity plus operator-confirmed OpenCV role binding."""

from __future__ import annotations

import json
import re
import subprocess


def _parse_pnputil_camera_inventory(output: str) -> list[dict]:
    """Parse the read-only PnPUtil listing used when WMI access is denied."""
    inventory = []
    for block in re.split(r"\r?\n\s*\r?\n", output.strip()):
        fields = {}
        for line in block.splitlines():
            key, separator, value = line.partition(":")
            if separator:
                fields[key.strip().casefold()] = value.strip()
        instance_id = fields.get("instance id")
        if not instance_id:
            continue
        class_name = fields.get("class name")
        if class_name and class_name.casefold() != "camera":
            continue
        status = fields.get("status", "")
        normalized_status = "OK" if status.casefold() in {"started", "ok"} else status
        inventory.append({
            "FriendlyName": fields.get("device description", ""),
            "InstanceId": instance_id,
            "Status": normalized_status,
        })
    return inventory


def _pnputil_camera_inventory() -> list[dict]:
    process = subprocess.run(
        ["pnputil.exe", "/enum-devices", "/class", "Camera", "/connected"],
        capture_output=True, text=True, timeout=8, check=True,
    )
    return _parse_pnputil_camera_inventory(process.stdout)


def windows_camera_inventory() -> list[dict]:
    command = (
        "Get-PnpDevice -Class Camera -PresentOnly | "
        "Select-Object FriendlyName,InstanceId,Status | ConvertTo-Json -Compress"
    )
    try:
        process = subprocess.run(
            ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", command],
            capture_output=True, text=True, timeout=8, check=True,
        )
        parsed = json.loads(process.stdout)
        if isinstance(parsed, list):
            return parsed
        if isinstance(parsed, dict):
            return [parsed]
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired,
            json.JSONDecodeError, OSError):
        # On restricted Windows accounts Get-PnpDevice may fail with WMI
        # HRESULT 0x80041003. PnPUtil reads the same connected-device identity
        # without opening a camera stream or requiring the WMI provider.
        pass
    return _pnputil_camera_inventory()


def bind_camera_roles(base_index: int, wrist_index: int, *,
                      base_device_id: str, wrist_device_id: str,
                      roles_visually_confirmed: bool,
                      inventory: list[dict] | None = None) -> dict:
    if base_index == wrist_index or not roles_visually_confirmed:
        raise ValueError("distinct indices and visual role confirmation are required")
    if not base_device_id or not wrist_device_id or base_device_id == wrist_device_id:
        raise ValueError("two distinct stable camera device IDs are required")
    inventory = windows_camera_inventory() if inventory is None else inventory
    by_id = {
        item["InstanceId"].casefold(): item
        for item in inventory
        if isinstance(item, dict) and isinstance(item.get("InstanceId"), str)
    }
    for device_id in (base_device_id, wrist_device_id):
        item = by_id.get(device_id.casefold())
        if item is None or item.get("Status") != "OK":
            raise ValueError(f"expected camera is not present and OK: {device_id}")
    return {
        "schema": "piper_camera_role_identity_v1",
        "binding_method": "operator_visual_confirmation_for_this_session",
        "index_to_instance_id_automatically_proven": False,
        "roles": {
            role: {"opencv_index": index, "windows_instance_id": by_id[device_id.casefold()]["InstanceId"],
                   "friendly_name": by_id[device_id.casefold()].get("FriendlyName"),
                   "pnp_status": by_id[device_id.casefold()]["Status"]}
            for role, index, device_id in (
                ("base", base_index, base_device_id),
                ("wrist", wrist_index, wrist_device_id),
            )
        },
    }
