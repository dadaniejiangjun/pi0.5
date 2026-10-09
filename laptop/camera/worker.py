"""Public acquisition worker; identical buffer, timestamp, error and callback behavior."""
from __future__ import annotations
import threading
import time


def camera_loop(index: int, role: str, shared: dict, lock: threading.Lock,
                 stop: threading.Event, errors: list[str], on_frame=None) -> None:
    import cv2
    from laptop.camera.source import open_camera
    camera = open_camera(index)
    try:
        if not camera.isOpened():
            raise RuntimeError(f"{role} camera did not open")
        while not stop.is_set():
            ok, frame = camera.read()
            stamp = time.perf_counter_ns()
            if ok and frame is not None:
                with lock:
                    shared[role] = {"frame": frame, "timestamp_ns": stamp}
                if on_frame is not None:
                    on_frame(role, frame, stamp)
    except Exception as exc:
        errors.append(f"{role}: {type(exc).__name__}: {exc}")
        stop.set()
    finally:
        camera.release()
