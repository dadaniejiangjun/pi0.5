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
