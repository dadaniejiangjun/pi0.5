# Laptop policy client

This package is intentionally separate from the server environment. It needs
the frozen official `openpi-client`, NumPy, Pillow, and optionally OpenCV; it
does not install JAX, the full OpenPI checkout, or a model checkpoint.

The default endpoint is `127.0.0.1:8000`, which is the local end of the SSH
tunnel. Do not replace it with the server public IP. The scripts only perform
policy/network/camera diagnostics and contain no robot command path.

Install on the laptop during P2, after creating the SSH tunnel:

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
python policy_client_smoke.py --host 127.0.0.1 --port 8000 --requests 100
```

Current package status: `LAPTOP_PACKAGE_READY = PASS`; laptop hardware and
remote tunnel execution are not claimed in P0/P1.
