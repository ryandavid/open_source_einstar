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

        # The recording again, in a new run: opened with no scanner, edited, then continued with the emulator.
        recording = result(m, "scan_state")["recording"]["path"]
        ok(m, "app_quit", app="scan")
        ok(m, "app_launch", app="scan", data_dir=str(data))
        opened = result(m, "scan_open", path=recording)
        if opened["loading"]:
            opened = until(m, "scan_state", lambda s: not s["load"]["loading"])["load"]
        assert opened["loaded"] and opened["resumable"] and opened["frames"] >= 40, opened
        state = result(m, "scan_state")
        assert state["connection"]["kind"] == "recording" and state["hud"]["model_points"] > 0, state
        assert m.call("scan_start", {}).is_error  # no scanner
        ok(m, "edit_select", polygon=[[500, 250], [760, 250], [760, 600], [500, 600]])
        assert result(m, "edit_delete")["undo_depth"] == 1
        ok(m, "device_connect", emulator=True)  # reopens the recording with the scanner
        until(m, "scan_state", lambda s: s["load"]["loaded"] and not s["load"]["loading"])
        ok(m, "scan_start")
        until(m, "scan_state", lambda s: s["recording"]["frames"] >= 20)
        ok(m, "scan_pause")
        state = result(m, "scan_state")
        assert state["recording"]["path"] == recording, state["recording"]

        # The calibration app alongside it.
        ok(m, "app_launch", app="calibration", data_dir=str(data / "calibration"))
        ok(m, "calib_connect", emulator=True)
        until(m, "calib_state", lambda s: "guidance" in s["live"])
        ok(m, "calib_auto_capture", on=False)
        captured = result(m, "calib_state")["captured"]
        assert result(m, "calib_capture")["count"] == captured + 1

        # The modelling app: the demo part from scan to STEP, as an agent would do it, then the scan made above.
        ok(m, "app_launch", app="model")
        ok(m, "model_open_demo")
        ok(m, "model_begin_change", description="detect the faces and square the part")
        assert len(result(m, "model_detect")["holes"]) == 5
        top = result(m, "model_raycast", origin=[10, 8, 100], direction=[0, 0, -1])["label"]
        right = result(m, "model_raycast", origin=[80, 3, 12], direction=[-1, 0, 0])["label"]
        ok(m, "model_label_update", label=top, name="top")
        ok(m, "model_label_update", label=right, name="right")
        ok(m, "model_datum_create", name="part", z="top", x="right")
        assert len(result(m, "model_square", datum="part")["added"]) == 10
        ok(m, "model_face_add_plane", name="bottom", datum="part", axis="z", offset=-20, facing="-")
        ok(m, "model_end_change")
        history = result(m, "model_history")
        assert len(history["undo"]) == 1 and history["undo"][0]["author"] == "agent"
        assert result(m, "model_solve")["converged"]
        built = result(m, "model_build")
        assert built["ok"] and built["closed"], built
        assert result(m, "model_deviation")["overall"]["p95_mm"] < 0.15
        ok(m, "model_view", mode="deviation", preset="iso", frame="all")
        shot = ok(m, "ui_screenshot", app="model", max_size=400)
        assert shot.content[0].type == "image"
        step = data / "part.step"
        assert result(m, "model_export_step", path=str(step))["read_back"]["valid"]
        assert step.stat().st_size > 10000
        assert result(m, "model_open", path=str(mesh))["scan"]["triangles"] > 1000
        assert m.call("model_grow", {}).is_error  # refused: nothing painted, said so

        assert len(result(m, "app_list")) == 3
        ok(m, "app_quit", app="model")
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
