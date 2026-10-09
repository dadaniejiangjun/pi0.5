"""List candleLight USB bus/address identities without opening CAN."""

from __future__ import annotations

import json

from laptop.transport.gs_usb_piper_can import enumerate_candlelight_usb_adapters


def main() -> int:
    adapters = enumerate_candlelight_usb_adapters()
    print(json.dumps({
        "status": "USB_ENUMERATION_ONLY",
        "can_started": False,
        "bitrate_set": False,
        "can_data_tx_calls": 0,
        "adapters": adapters,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
