"""Bounded-time WebSocket adapter for the pinned OpenPI client codec."""
from __future__ import annotations

import inspect
import os


class OpenPiWebsocketPolicy:
    """Small BasePolicy-compatible client with finite connect/receive timeouts."""

    def __init__(self, host: str, port: int, *, connect_timeout_s: float,
                 request_timeout_s: float, api_key: str | None = None) -> None:
        if not host or not 1 <= int(port) <= 65535:
            raise ValueError("policy host and port are invalid")
        if connect_timeout_s <= 0 or request_timeout_s <= 0:
            raise ValueError("WebSocket timeouts must be positive")
        from openpi_client import msgpack_numpy
        from websockets.sync.client import connect

        uri = host if host.startswith("ws://") or host.startswith("wss://") else f"ws://{host}:{port}"
        kwargs = {
            "open_timeout": float(connect_timeout_s),
            "close_timeout": 1.0,
            "compression": None,
            "max_size": None,
        }
        selected_api_key = api_key or os.environ.get("OPENPI_API_KEY")
        if selected_api_key:
            header_name = (
                "additional_headers"
                if "additional_headers" in inspect.signature(connect).parameters
                else "extra_headers"
            )
            kwargs[header_name] = {"Authorization": f"Api-Key {selected_api_key}"}
        self._ws = connect(uri, **kwargs)
        self._codec = msgpack_numpy
        self._packer = msgpack_numpy.Packer()
        self._request_timeout_s = float(request_timeout_s)
        try:
            metadata = msgpack_numpy.unpackb(self._ws.recv(timeout=self._request_timeout_s))
            if not isinstance(metadata, dict):
                raise ValueError("policy server metadata must be a mapping")
            self._server_metadata = metadata
        except Exception:
            self.close()
            raise

    def get_server_metadata(self) -> dict:
        return dict(self._server_metadata)

    def infer(self, observation: dict) -> dict:
        self._ws.send(self._packer.pack(observation))
        response = self._ws.recv(timeout=self._request_timeout_s)
        if isinstance(response, str):
            raise RuntimeError(f"policy inference server returned an error: {response}")
        result = self._codec.unpackb(response)
        if not isinstance(result, dict):
            raise ValueError("policy inference response must be a mapping")
        return result

    def close(self) -> None:
        ws = getattr(self, "_ws", None)
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass
