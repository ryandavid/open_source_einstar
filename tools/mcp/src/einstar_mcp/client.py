"""A connection to an app's agent socket: JSON-RPC 2.0, one JSON object per line."""

from __future__ import annotations

import json
import socket
import time
from typing import Any

# The app's error codes (libs/agent/include/einstar/agent/protocol.hpp).
ERROR_NAMES = {
    -32700: "parse error",
    -32600: "invalid request",
    -32601: "no such method",
    -32602: "invalid parameters",
    -32603: "internal error",
    -32000: "busy",
    -32001: "not found",
    -32002: "ambiguous",
    -32003: "refused",
    -32004: "timed out",
}


class AgentError(Exception):
    def __init__(self, code: int, message: str, data: Any = None):
        super().__init__(message)
        self.code = code
        self.message = message
        self.data = data

    def __str__(self) -> str:
        text = f"{ERROR_NAMES.get(self.code, 'error')}: {self.message}"
        if self.data is not None:
            text += "\n" + json.dumps(self.data, indent=1)
        return text


class AgentClient:
    def __init__(self, socket_path: str):
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.connect(socket_path)
        self._buffer = b""
        self._next_id = 1

    def close(self) -> None:
        self._sock.close()

    def call(self, method: str, params: dict[str, Any] | None = None, timeout_s: float = 60.0) -> Any:
        """The result, or AgentError. A reply that arrives after its call timed out is skipped later (ids)."""
        request_id = self._next_id
        self._next_id += 1
        line = json.dumps({"jsonrpc": "2.0", "id": request_id, "method": method, "params": params or {}}) + "\n"
        self._sock.sendall(line.encode())
        deadline = time.monotonic() + timeout_s
        while True:
            while b"\n" in self._buffer:
                raw, self._buffer = self._buffer.split(b"\n", 1)
                try:
                    reply = json.loads(raw)
                except ValueError:
                    continue
                if reply.get("id") != request_id:
                    continue
                if "error" in reply:
                    e = reply["error"]
                    raise AgentError(e.get("code", -32603), e.get("message", "(no message)"), e.get("data"))
                return reply.get("result")
            left = deadline - time.monotonic()
            if left <= 0:
                raise AgentError(-32004, f"{method}: no reply within {timeout_s:.0f} s")
            self._sock.settimeout(left)
            try:
                chunk = self._sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                raise AgentError(-32603, "the app closed the connection")
            self._buffer += chunk
