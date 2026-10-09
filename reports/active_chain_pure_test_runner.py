"""Offline current-chain tests; allow recorded videos and in-memory adapters only."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

def main():
    import cv2, usb.core
    from laptop.transport import gs_usb_piper_can as transport
    from openpi_client import websocket_client_policy
    from server.training import proxy_multi_episode_pilot as pilot
    def forbidden(*a, **k):
        raise AssertionError("ACTIVE_CHAIN_CLEANUP: hardware/network/training prohibited")
    video_capture = cv2.VideoCapture
    def recorded_only(source, *a, **k):
        if isinstance(source, (str, Path)) and Path(source).is_file():
            return video_capture(str(source), *a, **k)
        return forbidden(source, *a, **k)
    cv2.VideoCapture = recorded_only
    usb.core.find = forbidden
    old_open = transport._GsUsbReceiveAdapter.open_read_only
    def fake_adapter_only(self):
        module = sys.modules.get('gs_usb.gs_usb')
        if module is not None and not getattr(module, '__file__', None):
            return old_open(self)
        return forbidden()
    transport._GsUsbReceiveAdapter.open_read_only = fake_adapter_only
    websocket_client_policy.WebsocketClientPolicy.__init__ = forbidden
    pilot.execute = forbidden
    import pytest
    return pytest.main(['-q', '-p', 'no:cacheprovider', *sys.argv[1:]])

if __name__ == '__main__':
    raise SystemExit(main())
