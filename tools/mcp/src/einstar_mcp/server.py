"""The MCP side: lifecycle tools, plus one tool per agent method in methods.json (the C++ table, generated)."""

from __future__ import annotations

import copy
import json
import subprocess
from typing import Any

import anyio
import mcp.types as types
from mcp.server.lowlevel import Server

from . import paths
from .client import AgentError
from .supervisor import LaunchError, Supervisor

INSTRUCTIONS = """\
Drives the Einstar apps (the open EXStar replacement) headless: 'scan' (scanning: connect, scan, edit, process,
export) and 'calibration' (the camera calibration). Start with app_launch(app); if the apps are not built yet,
app_build() builds them. The built-in emulator needs no hardware: device_connect(emulator=true) /
calib_connect(emulator=true). Look with ui_screenshot and the app's own state (scan_state / calib_state); find
widgets with ui_snapshot and drive them with input_* (window points, origin top left). Methods answer once their
effect has been drawn. app_logs reads the app's log with a cursor; app_output shows what a crashed app printed."""

APP_PARAM = {"type": "string", "enum": ["scan", "calibration"], "description": "Which app: scan or calibration."}

# Lifecycle tools (this server's own); every other tool is an agent method of the same (dotted) name.
LIFECYCLE: dict[str, tuple[str, dict[str, Any]]] = {
    "app_build": (
        "Builds the apps (configures build/ first if needed): the command and the tail of its output. Needed once on "
        "a fresh clone and after C++ changes; apps already running keep their old build until relaunched.",
        {"type": "object", "properties": {"target": {"type": "string", "description": "A CMake target (default: both apps)."}}},
    ),
    "app_launch": (
        "Launches an app under agent control, replacing a running instance of it: 'scan' (scanning) or 'calibration'. "
        "Hidden unless visible=true. Recordings and captures go to a temporary folder unless data_dir is given. "
        "Nothing connects by itself: then device_connect / calib_connect.",
        {
            "type": "object",
            "properties": {
                "app": APP_PARAM,
                "visible": {"type": "boolean", "description": "Show the window (default false)."},
                "data_dir": {"type": "string", "description": "Where scans / captures are written."},
            },
            "required": ["app"],
        },
    ),
    "app_quit": ("Quits an app.", {"type": "object", "properties": {"app": APP_PARAM}, "required": ["app"]}),
    "app_list": ("The apps this server runs, and whether they are still running.", {"type": "object", "properties": {}}),
    "app_output": (
        "The last lines the app process printed (stdout and stderr, as captured here): what preceded a crash. For the "
        "app's own log with a cursor, use app_logs.",
        {"type": "object", "properties": {"app": APP_PARAM, "lines": {"type": "integer", "description": "How many (default 80)."}}, "required": ["app"]},
    ),
    "app_call": (
        "Calls any agent method by its dotted name (rpc_methods lists what the running app answers).",
        {"type": "object", "properties": {"app": APP_PARAM, "method": {"type": "string"}, "params": {"type": "object"}}, "required": ["app", "method"]},
    ),
}


def tool_name(method: str) -> str:
    return method.replace(".", "_")


def load_methods() -> list[dict[str, Any]]:
    # app.quit is the lifecycle tool app_quit instead (it also reaps the process).
    return [m for m in json.loads(paths.methods_json().read_text()) if m["name"] != "app.quit"]


def method_tools(methods: list[dict[str, Any]]) -> list[types.Tool]:
    tools = []
    for m in methods:
        schema = copy.deepcopy(m["params"])
        description = m["description"]
        if m["app"] == "any":
            schema.setdefault("properties", {})["app"] = APP_PARAM
            schema["required"] = ["app"] + schema.get("required", [])
        else:
            description = f"[{m['app']} app] {description}"
        tools.append(types.Tool(name=tool_name(m["name"]), description=description, input_schema=schema))
    return tools


def text(t: str) -> types.TextContent:
    return types.TextContent(type="text", text=t)


def ok(t: str) -> types.CallToolResult:
    return types.CallToolResult(content=[text(t)], is_error=False)


def fail(t: str) -> types.CallToolResult:
    return types.CallToolResult(content=[text(t)], is_error=True)


def build(target: str | None) -> types.CallToolResult:
    root, build_dir = paths.repo_root(), paths.build_dir()
    steps = []
    if not (build_dir / "CMakeCache.txt").exists():
        steps.append(["cmake", "-S", str(root), "-B", str(build_dir)])
    targets = [target] if target else [paths.APPS[a][1] for a in paths.APPS]
    steps.append(["cmake", "--build", str(build_dir), "-j", "--target", *targets])
    log = []
    for cmd in steps:
        p = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
        lines = (p.stdout + p.stderr).splitlines()
        log.append(f"$ {' '.join(cmd)}  -> exit {p.returncode}\n" + "\n".join(lines[-60:]))
        if p.returncode != 0:
            return fail("\n".join(log))
    return ok("\n".join(log))


class EinstarMcp:
    def __init__(self) -> None:
        self.supervisor = Supervisor()
        self.methods = {tool_name(m["name"]): m for m in load_methods()}
        self.tools = [types.Tool(name=n, description=d, input_schema=s) for n, (d, s) in LIFECYCLE.items()] + method_tools(list(self.methods.values()))

    def call(self, name: str, args: dict[str, Any]) -> types.CallToolResult:
        """One tool call (blocking: run off the event loop)."""
        sup = self.supervisor
        app = args.get("app")
        if app is not None and app not in paths.APPS:
            return fail("app is scan or calibration")
        try:
            if name == "app_build":
                return build(args.get("target"))
            if name == "app_launch":
                if not app:
                    return fail("app is scan or calibration")
                return ok(json.dumps(sup.launch(app, bool(args.get("visible", False)), args.get("data_dir")), indent=1))
            if name == "app_quit":
                return ok(json.dumps(sup.quit(app)))
            if name == "app_list":
                return ok(json.dumps(sup.list(), indent=1))
            if name == "app_output":
                out = sup.output(app, max(1, min(int(args.get("lines", 80)), 2000)))
                return ok(out or "(nothing captured)")
            if name == "app_call":
                method = args.get("method", "")
                return self._result(sup.call(app, method, args.get("params") or {}, 120), method == "ui.screenshot")
            m = self.methods.get(name)
            if m is None:
                return fail(f"no tool '{name}'")
            target = app if m["app"] == "any" else m["app"]
            if not target:
                return fail("app is scan or calibration")
            params = {k: v for k, v in args.items() if k != "app"}
            # Waits answer within their own timeout; everything else within a minute (a stuck UI answers `busy`).
            timeout = max(60.0, params.get("timeout_ms", 0) / 1000.0 + 10.0)
            return self._result(sup.call(target, m["name"], params, timeout), m["name"] == "ui.screenshot")
        except (AgentError, LaunchError) as e:
            return fail(str(e))

    @staticmethod
    def _result(result: Any, image: bool) -> types.CallToolResult:
        if image and isinstance(result, dict) and "png_base64" in result:
            meta = {k: v for k, v in result.items() if k != "png_base64"}
            return types.CallToolResult(
                content=[types.ImageContent(type="image", data=result["png_base64"], mime_type="image/png"), text(json.dumps(meta))],
                is_error=False,
            )
        return ok(json.dumps(result, indent=1))


def make_server() -> tuple[Server, EinstarMcp]:
    impl = EinstarMcp()

    async def list_tools(ctx, params) -> types.ListToolsResult:
        return types.ListToolsResult(tools=impl.tools)

    async def call_tool(ctx, params: types.CallToolRequestParams) -> types.CallToolResult:
        return await anyio.to_thread.run_sync(impl.call, params.name, dict(params.arguments or {}))

    server = Server("einstar", version="0.1.0", instructions=INSTRUCTIONS, on_list_tools=list_tools, on_call_tool=call_tool)
    return server, impl
