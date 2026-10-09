"""Offline-only scoped test runner; deny actual device/network opening."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import cv2
from laptop.transport import gs_usb_piper_can as transport
def forbidden(*args,**kwargs):
    raise AssertionError('PHASE_A: real hardware/network access prohibited')
cv2.VideoCapture=forbidden
transport._GsUsbReceiveAdapter.open_read_only=forbidden
from openpi_client import websocket_client_policy
websocket_client_policy.WebsocketClientPolicy.__init__=forbidden
import pytest
raise SystemExit(pytest.main(['-q','-p','no:cacheprovider',*sys.argv[1:]]))
