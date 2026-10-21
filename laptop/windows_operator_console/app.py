"""Local Windows web console and same-origin proxy for the WSL ROS API."""
from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import secrets
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
import webbrowser


BODY_LIMIT = 64 * 1024
ROOT = Path(__file__).resolve().parent
UI_FILE = ROOT / "index.html"


class FakeApi:
    """Explicit software-only backend used by the desktop app's --fake mode."""

    def __init__(self):
        self.source_session_id = uuid.uuid4().hex
        self.app_session_id = ""
        self.grants = {}
        self.used_request_ids = set()
        self.frame_sequences = {"base": 0, "wrist": 0}
        self.started_ns = time.monotonic_ns()

    def state(self):
        now_ns = time.monotonic_ns()
        metadata = {
            "base": {"device_name": "Fake Base camera", "encoding": "bgr8", "width": 640, "height": 480},
            "wrist": {"device_name": "Fake Wrist camera", "encoding": "bgr8", "width": 640, "height": 480},
        }
        return {
            "protocol_version": 1,
            "state": {
                "available": True,
                "fresh": True,
                "feedback_fresh": True,
                "source_session_id": self.source_session_id,
                "sequence": 1,
                "source_monotonic_ns": now_ns,
                "source_clock_id": "fake_monotonic_ns",
                "source_local_monotonic_ns": now_ns,
                "source_local_clock_id": "fake_monotonic_ns",
                "clock_mapping_uncertainty_ns": 0,
                "age_ns": 0,
                "age_ms": 0.0,
                "joint_position_rad": [0.1, -0.2, 0.3, -0.1, 0.2, 0.0],
                "gripper_nominal_m": 0.02,
                "gripper_raw": 20000,
                "ctrl_mode_raw": 0,
                "arm_status_raw": 0,
                "error_code": 0,
                "motor_enabled": [False] * 6,
                "hardware_connected": False,
                "fake": True,
            },
            "joint_state": {
                "names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                "position": [0.1, -0.2, 0.3, -0.1, 0.2, 0.0],
                "velocity": [0.0] * 6,
                "effort": [0.0] * 6,
            },
            "status": {"items": [{"name": "fake_backend", "level": 0, "message": "synthetic input only"}]},
            "cameras": {
                role: {
                    "available": True,
                    "sequence": self.frame_sequences[role],
                    "age_ms": 0,
                    "encoding": metadata[role]["encoding"],
                    "width": 640,
                    "height": 480,
                    "frame_id": f"fake_{role}_frame",
                    "error": "",
                    "metadata": metadata[role],
                }
                for role in ("base", "wrist")
            },
            "control_mode": "fake",
            "hardware_control_allowed": False,
        }

    def camera_frame(self, role):
        self.frame_sequences[role] += 1
        sequence = self.frame_sequences[role]
        bg = "#12233a" if role == "base" else "#30213b"
        fg = "#f2f6fc"
        svg = (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="640" height="480" '
            f'viewBox="0 0 640 480"><rect width="640" height="480" fill="{bg}"/>'
            f'<text x="32" y="54" fill="{fg}" font-size="28">FAKE {role.upper()} PREVIEW</text>'
            f'<text x="32" y="100" fill="{fg}" font-size="18">No camera was opened</text>'
            f'<text x="32" y="456" fill="{fg}" font-size="16">synthetic frame {sequence}</text></svg>'
        ).encode("utf-8")
        return 200, {"Content-Type": "image/svg+xml", "X-Frame-Sequence": str(sequence)}, svg

    def post(self, path, payload):
        state = self.state()["state"]
        if path == "/api/v1/session":
            self.app_session_id = uuid.uuid4().hex
            self.grants.clear()
            self.used_request_ids.clear()
            return 200, {
                "protocol_version": 1,
                "session_id": self.app_session_id,
                "control_mode": "fake",
                "source_state_session_id": self.source_session_id,
                "hardware_control_allowed": False,
            }
        if path == "/api/v1/authorize":
            if payload.get("session_id") != self.app_session_id or not self.app_session_id:
                return 200, self._reject("REJECTED_SESSION_MISMATCH", "operator session is not active")
            if payload.get("operator_confirmed") is not True:
                return 200, self._reject("REJECTED_OPERATOR_CONFIRMATION_REQUIRED", "explicit operator confirmation is required")
            operation = payload.get("operation")
            if operation not in {"set_joint_target", "set_gripper"}:
                return 200, self._reject("REJECTED_OPERATION_UNSUPPORTED", "only joint and gripper target proposals are supported")
            if payload.get("source_state_sequence") != state["sequence"]:
                return 200, self._reject("REJECTED_STALE_STATE_SEQUENCE", "request does not reference the current state sequence")
            if payload.get("source_state_session_id") != state["source_session_id"]:
                return 200, self._reject("REJECTED_SOURCE_SESSION_MISMATCH", "request is not bound to the current source feedback session")
            scope = payload.get("task_scope_id")
            if not isinstance(scope, str) or not scope.isascii() or not scope.strip() or len(scope) > 128:
                return 200, self._reject("REJECTED_INVALID_TASK_SCOPE", "task_scope_id must be bounded non-empty ASCII text")
            grant_id = secrets.token_urlsafe(24)
            issued_ns = time.monotonic_ns()
            self.grants[grant_id] = (operation, scope, state["source_session_id"], state["sequence"], issued_ns)
            return 200, {
                "accepted": True,
                "decision": "ONE_REQUEST_AUTHORIZED",
                "authorization_id": grant_id,
                "expires_in_ms": 5000,
                "hardware_control_allowed": False,
            }
        if path == "/api/v1/control/request":
            return 200, self._propose(payload, state)
        return 404, {"error": "route not found"}

    def _propose(self, payload, state):
        request_id = payload.get("request_id")
        if not isinstance(request_id, str) or not request_id.isascii() or not request_id.strip() or len(request_id) > 128:
            return self._reject("REJECTED_INVALID_REQUEST_ID", "request_id must be non-empty bounded ASCII text", "")
        if request_id in self.used_request_ids:
            return self._reject("REJECTED_REPLAY", "request_id was already used", request_id)
        if payload.get("session_id") != self.app_session_id or not self.app_session_id:
            return self._reject("REJECTED_SESSION_MISMATCH", "operator session is not active", request_id)
        self.used_request_ids.add(request_id)
        validity_ms = payload.get("validity_ms")
        if type(validity_ms) is not int or not 1 <= validity_ms <= 5000:
            return self._reject("REJECTED_INVALID_EXPIRY", "validity_ms must be in 1..5000", request_id)
        operation = payload.get("operation")
        if operation == "set_joint_target":
            values = payload.get("arguments", {}).get("joint_position_rad") if isinstance(payload.get("arguments"), dict) else None
            valid_args = isinstance(values, list) and len(values) == 6 and all(_finite_number(v) for v in values)
        elif operation == "set_gripper":
            args = payload.get("arguments")
            valid_args = isinstance(args, dict) and _finite_number(args.get("gripper_nominal_m"))
        else:
            valid_args = False
        if not valid_args:
            return self._reject("REJECTED_INVALID_ARGUMENTS", "target arguments have the wrong shape or contain non-finite values", request_id)
        if payload.get("source_state_sequence") != state["sequence"]:
            return self._reject("REJECTED_STALE_STATE_SEQUENCE", "request does not reference the current state sequence", request_id)
        if payload.get("source_state_session_id") != state["source_session_id"]:
            return self._reject("REJECTED_SOURCE_SESSION_MISMATCH", "request is not bound to the current source feedback session", request_id)
        scope = payload.get("task_scope_id")
        if not isinstance(scope, str) or not scope.isascii() or not scope.strip() or len(scope) > 128:
            return self._reject("REJECTED_INVALID_TASK_SCOPE", "task_scope_id must be bounded non-empty ASCII text", request_id)
        grant_id = payload.get("authorization_id")
        grant = self.grants.pop(grant_id, None) if isinstance(grant_id, str) else None
        if grant is None:
            return self._reject("REJECTED_AUTHORIZATION_REQUIRED", "request needs a current one-use authorization", request_id)
        granted_operation, granted_scope, source_session, sequence, issued_ns = grant
        now_ns = time.monotonic_ns()
        if (now_ns >= issued_ns + min(validity_ms, 5000) * 1_000_000
                or now_ns >= issued_ns + 5_000_000_000):
            return self._reject("REJECTED_REQUEST_EXPIRED", "request expired before the server received it", request_id)
        if (granted_operation != operation or granted_scope != scope or source_session != state["source_session_id"]
                or sequence != payload.get("source_state_sequence")):
            return self._reject("REJECTED_AUTHORIZATION_SCOPE_MISMATCH", "authorization is bound to another state or operation", request_id)
        return {
            "accepted": True,
            "decision": "SIMULATED_ACCEPTED",
            "reason": "fake backend accepted the proposal; no camera, ROS control service, or CAN path was opened",
            "request_id": request_id,
            "hardware_control_allowed": False,
            "can_tx_calls": 0,
            "simulated_only": True,
        }

    @staticmethod
    def _reject(decision, reason, request_id=""):
        return {
            "accepted": False,
            "decision": decision,
            "reason": reason,
            "request_id": request_id,
            "hardware_control_allowed": False,
            "can_tx_calls": 0,
            "simulated_only": False,
        }


def _finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


class LocalServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, *, api_url, fake):
        self.api_url = api_url.rstrip("/") if api_url else ""
        self.fake = FakeApi() if fake else None
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    server: LocalServer
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        print("[console] " + fmt % args, flush=True)

    def do_GET(self):
        parsed = urllib.parse.urlsplit(self.path)
        if parsed.path.startswith("/api/v1/"):
            if self.server.fake:
                if parsed.path == "/api/v1/state":
                    value = self.server.fake.state()
                    self._json(200, value)
                    return
                parts = parsed.path.strip("/").split("/")
                if len(parts) == 5 and parts[:3] == ["api", "v1", "camera"] and parts[4] == "frame":
                    role = parts[3]
                    if role not in {"base", "wrist"}:
                        self._json(404, {"error": "camera role must be base or wrist"})
                        return
                    status, headers, body = self.server.fake.camera_frame(role)
                    self._response(status, headers, body)
                    return
                self._json(404, {"error": "route not found"})
                return
            self._proxy("GET", parsed.path + (("?" + parsed.query) if parsed.query else ""), b"")
            return
        if parsed.path not in {"/", "/index.html"}:
            self._json(404, {"error": "route not found"})
            return
        try:
            body = UI_FILE.read_bytes()
        except OSError as exc:
            self._json(500, {"error": f"UI file is unavailable: {exc}"})
            return
        self._response(200, {"Content-Type": "text/html; charset=utf-8"}, body)

    def do_POST(self):
        path = urllib.parse.urlsplit(self.path).path
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 <= length <= BODY_LIMIT:
                raise ValueError("request body must be within 64 KiB")
            raw = self.rfile.read(length)
            payload = json.loads(raw.decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("JSON body must be an object")
        except (ValueError, UnicodeError, json.JSONDecodeError) as exc:
            self._json(400, {"error": str(exc)})
            return
        if self.server.fake:
            status, response = self.server.fake.post(path, payload)
            self._json(status, response)
            return
        self._proxy("POST", path, json.dumps(payload, allow_nan=False).encode("utf-8"))

    def _proxy(self, method, path, body):
        if not self.server.api_url:
            self._json(503, {"error": "WSL API URL is not configured"})
            return
        request = urllib.request.Request(
            self.server.api_url + path,
            data=body if method == "POST" else None,
            headers={"Content-Type": "application/json"} if method == "POST" else {},
            method=method,
        )
        try:
            with urllib.request.urlopen(request, timeout=3.5) as response:
                forwarded_headers = {"Content-Type": response.headers.get("Content-Type", "application/octet-stream")}
                for name in ("Cache-Control", "X-Frame-Sequence", "X-Frame-Source-Stamp-Ns", "X-Frame-Id"):
                    if response.headers.get(name):
                        forwarded_headers[name] = response.headers[name]
                self._response(response.status, forwarded_headers, response.read())
        except urllib.error.HTTPError as exc:
            headers = {"Content-Type": exc.headers.get("Content-Type", "application/json")}
            self._response(exc.code, headers, exc.read())
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            detail = getattr(exc, "reason", exc)
            self._json(502, {"error": f"WSL API unavailable: {detail}"})

    def _json(self, status, value):
        body = json.dumps(value, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self._response(status, {"Content-Type": "application/json; charset=utf-8", "Cache-Control": "no-store"}, body)

    def _response(self, status, headers, body):
        self.send_response(status)
        for name, value in headers.items():
            self.send_header(name, value)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        self.end_headers()
        if body:
            self.wfile.write(body)
        self.close_connection = True


def main():
    parser = argparse.ArgumentParser(description="Windows operator console for a WSL ROS preview API")
    parser.add_argument("--host", default="127.0.0.1", help="local UI bind address (loopback only by default)")
    parser.add_argument("--port", type=int, default=18780)
    parser.add_argument("--wsl-api-url", help="WSL operator API base URL, for example http://172.27.x.y:18771")
    parser.add_argument("--fake", action="store_true", help="run a local synthetic state/video and fake-only control backend")
    parser.add_argument("--no-browser", action="store_true", help="do not open the local console in the default browser")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be within 1..65535")
    if args.host not in {"127.0.0.1", "localhost", "::1"}:
        parser.error("the Windows console must bind to loopback")
    if args.fake and args.wsl_api_url:
        parser.error("--fake and --wsl-api-url are mutually exclusive")
    if not args.fake and not args.wsl_api_url:
        parser.error("pass --wsl-api-url or --fake")
    if args.wsl_api_url:
        parsed = urllib.parse.urlsplit(args.wsl_api_url)
        if parsed.scheme not in {"http", "https"} or not parsed.hostname or parsed.path not in {"", "/"}:
            parser.error("--wsl-api-url must be an http(s) origin without a path")
    server = LocalServer((args.host, args.port), api_url=args.wsl_api_url, fake=args.fake)
    console_url = f"http://{args.host}:{args.port}"
    print(f"Windows operator console: {console_url}", flush=True)
    print(f"backend={'FAKE_ONLY' if args.fake else args.wsl_api_url}; hardware_control_allowed=false", flush=True)
    if not args.no_browser:
        webbrowser.open(console_url)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("stopping local console", flush=True)
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
