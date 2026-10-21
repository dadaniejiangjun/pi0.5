"""Small atomic runtime owner records shared by Windows and WSL processes.

The records are diagnostic hints keyed by process ID. They do not arbitrate
ownership or grant permission to open hardware.
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import threading
from typing import Any


OWNER_SCHEMA = "pi05_runtime_owner_v1"
_ROLE_RE = re.compile(r"^[a-z][a-z0-9_]{1,47}$")


def runtime_owner_directory(project_root: str | Path) -> Path:
    """Return the configured shared owner-record directory."""
    override = os.environ.get("PI05_RUNTIME_OWNER_DIR")
    return Path(override) if override else Path(project_root) / "cache" / "ros_runtime" / "owners"


class RuntimeOwnerRegistration:
    """Publish one process's current role, endpoint, and optional session ID."""

    def __init__(
        self,
        role: str,
        directory: str | Path,
        *,
        process_id: int | None = None,
        details: dict[str, Any] | None = None,
    ) -> None:
        if not isinstance(role, str) or not _ROLE_RE.fullmatch(role):
            raise ValueError("runtime owner role must be a short lowercase identifier")
        self.role = role
        self.process_id = os.getpid() if process_id is None else process_id
        if type(self.process_id) is not int or self.process_id <= 0:
            raise ValueError("runtime owner process_id must be a positive integer")
        self.path = Path(directory) / f"{role}-{self.process_id}.json"
        self._lock = threading.Lock()
        self._details = dict(details or {})
        self._session_id: str | None = None
        self._status = "starting"
        self._process_started_utc = self._utc_now()
        self._write()

    @staticmethod
    def _utc_now() -> str:
        return datetime.now(timezone.utc).isoformat()

    def _write(self) -> dict[str, Any]:
        record = {
            **self._details,
            "schema": OWNER_SCHEMA,
            "role": self.role,
            "pid": self.process_id,
            "process_started_utc": self._process_started_utc,
            "updated_utc": self._utc_now(),
            "status": self._status,
            "session_id": self._session_id,
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(self.path.name + f".{threading.get_ident()}.tmp")
        try:
            temporary.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            os.replace(temporary, self.path)
        finally:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
        return record

    def update(
        self,
        *,
        status: str,
        session_id: str | None = None,
        **details: Any,
    ) -> dict[str, Any]:
        if status not in {"starting", "running", "listening", "connected", "reconnecting"}:
            raise ValueError("unsupported runtime owner status")
        if session_id is not None and (not isinstance(session_id, str) or not session_id):
            raise ValueError("session_id must be a non-empty string or None")
        with self._lock:
            self._status = status
            self._session_id = session_id
            self._details.update(details)
            return self._write()

    def close(self) -> None:
        """Remove this process's live record during normal shutdown."""
        with self._lock:
            try:
                self.path.unlink()
            except FileNotFoundError:
                pass
