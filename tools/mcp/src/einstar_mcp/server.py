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

# Methods that may work for minutes before answering (processing a scan): the app keeps the call open meanwhile.
LONG_WORK = {"model.open", "model.reprocess", "scan.open"}

INSTRUCTIONS = """\
Drives the Einstar apps (the open EXStar replacement): 'scan' (scanning: connect, scan, edit, process, export),
'calibration' (the camera calibration) and 'model' (scan to CAD: faces, holes, fillets, constraints, a solid, STEP).
Start with app_launch(app) (headless unless visible=true); to work in the modelling window the user has open, use
app_attach(app="model") instead. If the apps are not built yet, app_build() builds them. The built-in emulator needs no
hardware: device_connect(emulator=true) / calib_connect(emulator=true). Look with ui_screenshot and the app's own
state (scan_state / calib_state / model_summary); find widgets with ui_snapshot and drive them with input_* (window
points, origin top left). Methods answer once their effect has been drawn. app_logs reads the app's log with a cursor;
app_output shows what a crashed app printed.

Modelling (the model app) -- the user describes the part in words and measurements; you turn that into the model:
- model_summary first and after changes: labels (named regions of the scan, each with a fitted surface and a role),
  holes, fillets, datums, constraints (with their status from the last solve), the built model.
- Wrap each request you carry out in model_begin_change(description) .. model_end_change: the user undoes it as one
  step and sees it as yours in the history.
- Faces: model_detect proposes them all; rename labels to what the user calls them (model_label_update). To pick out a
  face yourself: model_raycast or model_label for positions, then model_paint (a few mm on the face) and model_grow.
- 'Make it square': model_datum_create (z from the main face, x from a side), then model_square. Prefer one datum
  with aligned faces over pairwise perpendicular constraints.
- Measurements: model_hole_update(diameter) for a measured hole, model_fillet_update(radius), and
  model_constraint_add (distance between faces, offset from the datum, diameter, angle, axis_distance for hole
  pitches and bolt circles, tangent, symmetric, equal_radius ...). Holes can be named wherever an axis is meant.
- Holes: detection measures counterbores, countersinks and drill points; model_hole_update sets or corrects them.
- A face no primitive fits (a domed or sculpted surface): model_freeform on a label painted on it.
- model_solve's 'scale' says whether the user's measurements show the scan too large or small; offer model_scale
  when it is significant (and suggest checking the scanner's calibration).
- Where the scan is open (the bottom it stood on, a blind hole's floor), the model needs the user's knowledge:
  model_face_add_plane for an unseen face, model_hole_update(depth) for a blind hole. Ask when you do not know.
- model_solve, then model_build; check model_deviation (hot spots: where the model departs from the scan; edge bands
  are the scan rounding sharp edges, not errors) and model_view + ui_screenshot to look. Report conflicts and what
  each measurement cost (how far it moved the surfaces off the scan) rather than hiding them.
- model_export_step writes the STEP file the user imports into CAD.
- Photos (model_summary lists them; model_photo_list has their annotations): the user's photos of the part, often taken
  somewhere else than the scan. Look at them (model_photo_get, grid=true to read pixel positions) before asking the user
  what you could see. Annotations (D1, DIA2, A3, C4, N5 ...) hold measured values and remarks; links say which labels or
  holes they are about. When the user tells you something about the part, keep it: model_photo_annotate on a photo that
  shows it, or model_note_add.
- When model_summary's scan.source is 'changed' (the .estr was scanned further or edited since), model_reprocess
  makes the mesh again and carries the labels over; solve and build again after. In the scanning app, scan_open
  brings a recording back as a paused scan (edit it, process it, or connect the scanner and continue it)."""

APP_PARAM = {"type": "string", "enum": ["scan", "calibration", "model"], "description": "Which app: scan, calibration or model."}

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
    "app_attach": (
        "Connects to the modelling app the user has open, so you work in their window alongside them (their undo history "
        "shows your changes as yours). Fails if it is not open: then ask the user to open it, or app_launch it.",
        {"type": "object", "properties": {"app": {"type": "string", "enum": ["model"], "description": "The app (model)."}}, "required": ["app"]},
    ),
    "app_quit": ("Quits an app (an attached one is only let go of).", {"type": "object", "properties": {"app": APP_PARAM}, "required": ["app"]}),
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
            return fail("app is scan, calibration or model")
        try:
            if name == "app_build":
                return build(args.get("target"))
            if name == "app_attach":
                if not app:
                    return fail("app is model")
                return ok(json.dumps(sup.attach(app), indent=1))
            if name == "app_launch":
                if not app:
                    return fail("app is scan, calibration or model")
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
                return self._result(sup.call(app, method, args.get("params") or {}, 120))
            m = self.methods.get(name)
            if m is None:
                return fail(f"no tool '{name}'")
            target = app if m["app"] == "any" else m["app"]
            if not target:
                return fail("app is scan, calibration or model")
            params = {k: v for k, v in args.items() if k != "app"}
            # Waits answer within their own timeout; processing a scan can take minutes; everything else answers within
            # a minute (a stuck UI answers `busy`).
            timeout = max(60.0, params.get("timeout_ms", 0) / 1000.0 + 10.0)
            if m["name"] in LONG_WORK:
                timeout = 1800.0
            return self._result(sup.call(target, m["name"], params, timeout))
        except (AgentError, LaunchError) as e:
            return fail(str(e))

    @staticmethod
    def _result(result: Any) -> types.CallToolResult:
        # Images (ui.screenshot, model.photo.get) go to the client as images, the rest of the result as text.
        if isinstance(result, dict):
            for key, mime in (("png_base64", "image/png"), ("jpeg_base64", "image/jpeg")):
                if key in result:
                    meta = {k: v for k, v in result.items() if k != key}
                    return types.CallToolResult(
                        content=[types.ImageContent(type="image", data=result[key], mime_type=mime), text(json.dumps(meta))],
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
