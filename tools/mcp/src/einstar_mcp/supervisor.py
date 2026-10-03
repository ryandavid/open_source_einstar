"""The apps this server runs: launched with --mcp, up once they print AGENT_READY, their output kept (so a crash
comes back with what preceded it), quit on request or when this server exits. At most one instance per app.
(An app launched this way also quits by itself if this process goes away.)"""

from __future__ import annotations

import atexit
import collections
import os
import pathlib
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Any

from . import paths
from .client import AgentClient, AgentError

TAIL_LINES = 2000


class LaunchError(Exception):
    pass


@dataclass
class _Proc:
    app: str
    popen: subprocess.Popen
    socket: str
    visible: bool
    tail: collections.deque = field(default_factory=lambda: collections.deque(maxlen=TAIL_LINES))
    lock: threading.Lock = field(default_factory=threading.Lock)
    client: AgentClient | None = None
    ready: threading.Event = field(default_factory=threading.Event)

    def add(self, line: str) -> None:
        with self.lock:
            self.tail.append(line)
        if line.startswith("AGENT_READY "):
            self.ready.set()

    def last(self, n: int) -> str:
        with self.lock:
            return "\n".join(list(self.tail)[-n:])


class Supervisor:
    def __init__(self) -> None:
        self._procs: dict[str, _Proc] = {}
        self._gone: dict[str, _Proc] = {}  # the last instance of an app that is not running, for its output
        atexit.register(self.quit_all)

    def launch(self, app: str, visible: bool = False, data_dir: str | None = None) -> dict[str, Any]:
        if app in self._procs:
            self.quit(app)
        exe = paths.app_binary(app)
        if not exe.exists():
            raise LaunchError(f"{exe} does not exist: the apps are not built. app_build() builds them (or `cmake -B build && cmake --build build`).")
        sock = f"/tmp/einstar_mcp_{app}_{os.getpid()}.sock"
        env = dict(os.environ)
        if data_dir:
            env[paths.APPS[app][2]] = data_dir
        args = [str(exe), f"--mcp={sock}"] + (["--visible"] if visible else [])
        popen = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env, text=True, errors="replace")
        proc = _Proc(app, popen, sock, visible)
        for stream in (popen.stdout, popen.stderr):
            threading.Thread(target=self._pump, args=(proc, stream), daemon=True).start()
        # Up once it says so; 60 s, checked in short polls.
        for _ in range(600):
            if proc.ready.wait(0.1):
                break
            if popen.poll() is not None:
                self._gone[app] = proc
                raise LaunchError(f"the {app} app exited during start-up (status {popen.returncode}):\n{proc.last(60)}")
        else:
            self._stop(proc)
            self._gone[app] = proc
            raise LaunchError(f"the {app} app did not report ready within 60 s:\n{proc.last(60)}")
        proc.client = AgentClient(sock)
        self._procs[app] = proc
        info = {"app": app, "pid": popen.pid, "socket": sock, "visible": visible}
        try:
            info["info"] = proc.client.call("app.info", timeout_s=10)
        except AgentError:
            pass
        return info

    @staticmethod
    def _pump(proc: _Proc, stream) -> None:
        for line in stream:
            proc.add(line.rstrip("\n"))

    def _stop(self, proc: _Proc) -> int | None:
        p = proc.popen
        if p.poll() is None:
            if proc.client:
                try:
                    proc.client.call("app.quit", timeout_s=3)
                except (AgentError, OSError):
                    pass
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.terminate()
                try:
                    p.wait(2)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.wait()
        if proc.client:
            proc.client.close()
            proc.client = None
        pathlib.Path(proc.socket).unlink(missing_ok=True)
        return p.returncode

    def quit(self, app: str) -> dict[str, Any]:
        proc = self._procs.pop(app, None)
        if proc is None:
            raise LaunchError(f"the {app} app is not running")
        status = self._stop(proc)
        self._gone[app] = proc
        return {"quit": True, "exit_status": status}

    def quit_all(self) -> None:
        for app in list(self._procs):
            try:
                self.quit(app)
            except LaunchError:
                pass

    def list(self) -> list[dict[str, Any]]:
        out = []
        for app, proc in self._procs.items():
            status = proc.popen.poll()
            entry = {"app": app, "pid": proc.popen.pid, "running": status is None, "visible": proc.visible}
            if status is not None:
                entry["exit_status"] = status
            out.append(entry)
        return out

    def call(self, app: str, method: str, params: dict[str, Any], timeout_s: float) -> Any:
        proc = self._procs.get(app)
        if proc is None:
            raise AgentError(-32003, f'the {app} app is not running: app_launch(app="{app}") first')
        if proc.popen.poll() is not None:
            raise AgentError(-32003, f"the {app} app has exited (status {proc.popen.returncode}); its last output:\n{proc.last(80)}")
        try:
            return proc.client.call(method, params, timeout_s)
        except (AgentError, OSError) as e:
            time.sleep(0.2)  # (a crash takes a moment to be reaped)
            if proc.popen.poll() is not None:
                raise AgentError(-32003, f"{e}\nthe {app} app has exited (status {proc.popen.returncode}); its last output:\n{proc.last(80)}") from e
            if isinstance(e, AgentError):
                raise
            raise AgentError(-32603, str(e)) from e

    def output(self, app: str, lines: int) -> str:
        proc = self._procs.get(app) or self._gone.get(app)
        return proc.last(lines) if proc else ""
