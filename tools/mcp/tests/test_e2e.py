"""einstar-mcp end to end: the server (in process) drives the real apps, headless, with the emulator.
Run by ctest (tests/CMakeLists.txt) or:  uv run --directory tools/mcp python tests/test_e2e.py"""

from __future__ import annotations

import json
import pathlib
import shutil
import sys
import tempfile
import time

from einstar_mcp.server import EinstarMcp


def ok(m: EinstarMcp, tool: str, **args):
    r = m.call(tool, args)
    assert not r.is_error, f"{tool}: {r.content[0].text}"
    return r


def result(m: EinstarMcp, tool: str, **args):
    return json.loads(ok(m, tool, **args).content[0].text)


def until(m: EinstarMcp, tool: str, done, polls: int = 600):
    """Polls `tool` until done(result) -- counted, not timed (processes here can stall for seconds)."""
    for _ in range(polls):
        r = result(m, tool)
        if done(r):
            return r
        time.sleep(0.1)
    raise AssertionError(f"{tool} never got there: {r}")


def main() -> int:
    data = pathlib.Path(tempfile.mkdtemp(prefix="einstar_mcp_e2e_"))
    m = EinstarMcp()
    try:
        assert len(m.tools) > 50
        assert m.call("scan_state", {}).is_error  # not launched yet

        # The scanning app: scan with the emulator, delete a block, process, export.
        ok(m, "app_launch", app="scan", data_dir=str(data))
        ok(m, "device_connect", emulator=True)
        ok(m, "scan_start")
        until(m, "scan_state", lambda s: s["recording"]["frames"] >= 40)
        ok(m, "scan_pause")
        before = result(m, "scan_state")["hud"]["model_points"]
        ok(m, "edit_select", polygon=[[500, 250], [760, 250], [760, 600], [500, 600]])
        assert result(m, "edit_delete")["undo_depth"] == 1
        assert result(m, "scan_state")["hud"]["model_points"] < before
        assert not result(m, "ui_find", app="scan", target="Einstar/Undo").get("disabled", False)
        shot = ok(m, "ui_screenshot", app="scan", max_size=400)
        assert shot.content[0].type == "image"
        ok(m, "process_run", resolution_mm="1.0")
        done = until(m, "process_status", lambda p: not p["running"] and p["done"])
        assert "triangles" in done["summary"]
        mesh = data / "agent_test.stl"
        ok(m, "process_export", path=str(mesh))
        assert mesh.stat().st_size > 1000

        # The calibration app alongside it.
        ok(m, "app_launch", app="calibration", data_dir=str(data / "calibration"))
        ok(m, "calib_connect", emulator=True)
        until(m, "calib_state", lambda s: "guidance" in s["live"])
        ok(m, "calib_auto_capture", on=False)
        captured = result(m, "calib_state")["captured"]
        assert result(m, "calib_capture")["count"] == captured + 1

        assert len(result(m, "app_list")) == 2
        ok(m, "app_quit", app="scan")
        ok(m, "app_quit", app="calibration")
        assert m.call("scan_state", {}).is_error
        assert "AGENT_READY" in ok(m, "app_output", app="scan").content[0].text
    finally:
        m.supervisor.quit_all()
        shutil.rmtree(data, ignore_errors=True)
    print("einstar-mcp e2e: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
