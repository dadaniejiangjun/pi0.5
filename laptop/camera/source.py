"""Common OpenCV camera opener; real ACER access requires unsandboxed execution."""
def backend_constant(cv2, name=None):
    name = (name or ("DSHOW" if hasattr(cv2, "CAP_DSHOW") else "ANY")).upper()
    value = {"DSHOW": getattr(cv2, "CAP_DSHOW", None),
             "MSMF": getattr(cv2, "CAP_MSMF", None), "ANY": cv2.CAP_ANY}.get(name)
    if value is None:
        raise ValueError(f"unsupported camera backend: {name}")
    return value, name


def open_camera(index, *, backend_name=None):
    """Return a VideoCapture; caller checks isOpened and always calls release.

    No index scan, virtual-camera fallback, process termination, or robot access.
    """
    if type(index) is not int or index < 0:
        raise ValueError("camera index must be a nonnegative integer")
    import cv2
    backend, _ = backend_constant(cv2, backend_name)
    return cv2.VideoCapture(index, backend)


def request_capture_mode(capture, *, width=640, height=480, fps=30.0):
    """Ask a live capture for the project's standard 640x480/30 FPS mode.

    Camera backends may reject or ignore these requests. The return value
    records both setter acceptance and what the backend reports; callers must
    still measure the actual frame cadence and must not treat this as proof of
    the negotiated mode.
    """
    import math
    import cv2

    if type(width) is not int or width <= 0:
        raise ValueError("capture width must be a positive integer")
    if type(height) is not int or height <= 0:
        raise ValueError("capture height must be a positive integer")
    if isinstance(fps, bool) or not isinstance(fps, (int, float)):
        raise TypeError("capture fps must be numeric")
    if not math.isfinite(float(fps)) or fps <= 0:
        raise ValueError("capture fps must be finite and positive")

    requested = {"width": width, "height": height, "fps": float(fps)}
    properties = (
        ("width", cv2.CAP_PROP_FRAME_WIDTH),
        ("height", cv2.CAP_PROP_FRAME_HEIGHT),
        ("fps", cv2.CAP_PROP_FPS),
    )
    results = {}
    for name, property_id in properties:
        try:
            accepted = bool(capture.set(property_id, requested[name]))
        except Exception as exc:
            accepted = False
            error = f"{type(exc).__name__}: {exc}"
        else:
            error = None
        try:
            value = float(capture.get(property_id))
            reported = value if math.isfinite(value) and value >= 0 else None
        except Exception:
            reported = None
        results[name] = {
            "set_accepted": accepted,
            "reported_value": reported,
            "error": error,
        }
    return {"requested": requested, "properties": results}
