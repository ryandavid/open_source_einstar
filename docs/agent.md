# Agent control (`--mcp`, `einstar-mcp`)

Both apps can be driven by an agent: launched headless, looked at, clicked, and asked for their state. The
control endpoint is always compiled in and only listens when an app is started with `--mcp`.

```
Claude Code
    |  MCP (stdio)
    v
tools/mcp (Python, uv)          einstar-mcp: launches the apps, tools from methods.json, screenshots as images
    |  JSON-RPC 2.0, one JSON object per line, over an AF_UNIX socket
    +--> Einstar.app            --mcp=<socket> [--visible]
    +--> EinstarCalibration.app --mcp=<socket> [--visible]
```

The same design as the Redline dashboard's `agent_control`, adapted to Dear ImGui:

- **`libs/agent`.** `protocol.hpp` describes every method once (`method_specs()`): its name, the app that
  answers it, whether it changes state, a description and a JSON schema of its parameters. The apps register
  handlers against these specs. The table is also committed as `tools/mcp/methods.json`, which the MCP server
  turns into tools; `test_agent_methods` fails if the file differs from the C++ table, so they cannot drift
  apart.
- **The server** (`server.hpp`) runs in the app. A socket thread queues requests; the app calls `pump()` once
  per frame on its UI thread, before `ImGui::NewFrame()`, and every handler runs there, touching the app's
  state directly. A handler answers at once or returns a poll the server calls each frame (a screenshot of the
  next frame, a wait, a click that takes several frames). A mutating method answers after two more frames, so
  a screenshot straight after it shows its effect. If the UI thread does not get to a request in time the
  caller gets `busy` -- an answer about the app's health rather than a hung caller.
- **ImGui keeps no widget tree**, so the inspector records every item as it is drawn, through ImGui's
  test-engine hooks (`IMGUI_ENABLE_TEST_ENGINE`, set on the `imgui` target; `libs/agent/src/ui.cpp`
  implements the hooks). `ui.snapshot` lists the last frame's windows and items with their rects and state.
- **Synthetic input** goes through ImGui's own input queue, one step per frame (a click is move, press,
  release). The scanning app's 3D view reads ImGui's input too, so orbiting, panning, zooming and the
  Shift-drag lasso work from an agent as from a mouse.
- **einstar-mcp** (`tools/mcp`, Python with `uv`) launches an app with `--mcp=<socket>`, waits for its
  `AGENT_READY <socket> <pid>` line, keeps the tail of its stdout and stderr (`app_output`: what preceded a
  crash), and quits it. One instance per app. An app launched this way quits by itself when einstar-mcp goes
  away. It is Python so that it starts without a build: on a fresh clone the agent has its tools at once,
  and `app_build` builds the apps (configuring `build/` first if needed).

## Under Claude Code

`.mcp.json` in the repository registers the server (`uv run --directory tools/mcp einstar-mcp`; `uv.lock` pins
its dependencies). The `--directory` path is relative, so the client must start it from the repository root
(Claude Code does). It finds the repository by walking up for `.git` and `CMakeLists.txt`
(`EINSTAR_REPO_ROOT` overrides) and the apps in `build/apps` (`EINSTAR_BUILD_DIR` overrides).

```
app_build()                                    # once on a fresh clone, and after C++ changes
app_launch(app="scan")                         # hidden; visible=true to watch it
device_connect(emulator=true)                  # no hardware needed
scan_start()  ...  scan_pause()
scan_state()                                   # connection, workflow, HUD, recording, processing, editing, view
ui_screenshot(app="scan")                      # an image of the window
edit_select(polygon=[[500,250],[760,250],[760,600],[500,600]]); edit_delete()
process_run(); process_status(); process_export(path="/tmp/scan.stl")
scan_open(path="scan.estr")                    # a recording as a paused scan: edit, process, or connect and resume
app_launch(app="calibration"); calib_connect(emulator=true); calib_state()
```

Agent-run scans and captures go to a temporary folder unless `app_launch` is given `data_dir` (or the
app's `EINSTAR_SCAN_DIR` / `EINSTAR_CALIBRATION_DIR` is set).

## Attaching to an open app

Einstar Model can also be driven in the window the user has open, rather than in a headless copy:
- **Started by the user** (not with `--mcp`), it listens on
  `~/Library/Application Support/Einstar/model-agent.sock`. The socket is mode 0600, and it is not tied to the
  process that launched the app.
- **`app_attach(app="model")`** connects to it. `app_quit` then only lets go: the app stays open for the user.
- **A second instance** of the app leaves the first one's socket alone.
- **Changes the agent makes** show in the app's history as the agent's. `model.begin_change` / `model.end_change`
  around a request make it one undo step.

## By hand

```
build/apps/Einstar.app/Contents/MacOS/Einstar --mcp=/tmp/e.sock     # prints AGENT_READY /tmp/e.sock <pid>
printf '{"jsonrpc":"2.0","id":1,"method":"scan.state","params":{}}\n' | nc -U /tmp/e.sock
```

## Addressing widgets

A target is a ref from `ui.snapshot` (`#1a2b3c4d`, the item's ImGui ID, stable while it exists), `Window/Label`
(the root window's name and the item's visible label; labels may contain `/`, so every split whose prefix is a
window is tried), or a bare label. A target matching nothing or more than one item is an error listing the
candidates -- never a silent first match. Disabled items are reported as such; `interactive_only` leaves them
out. Static text is not an item: read it from the app's own state method or a screenshot.

## Adding a method

Add its spec to `libs/agent/src/specs.cpp` and register a handler in the app (`apps/*/agent_methods.cpp`), then
regenerate the server's copy: `build/tools/einstar-agent-methods > tools/mcp/methods.json`. The server reads it at
start-up, so reconnect it (`/mcp` in Claude Code) to see the new tool. `app_call` reaches any method meanwhile.

## Coordinates

Every coordinate is a logical window point (ImGui's), origin at the window's top left. `ui.screenshot`
returns `scale`, `logical_rect` and `image_size`, so an image pixel (px, py) is the point
(`logical_rect.x + px / scale`, `logical_rect.y + py / scale`). The 3D view fills the window, so a lasso
(`edit.select`) is a polygon in the same points, taken through the current view.

## Hardware

An agent has the same control as a person at the keyboard, including a real scanner and the calibration
write (`calib.write` / `calib.restore` take `confirm: true` in place of the dialog; the gates and the backup
are the dialog's). The compile-time guard on dangerous scanner commands (`device/opcodes.hpp`) still applies.

## Gotchas

- **Visible mode shares the mouse.** With `visible=true` the real mouse also feeds ImGui; while it is over
  the window it can fight the agent's synthetic position. Hidden mode has no real input.
- **Input is one step per frame**, and the app paces headless frames at about 60 Hz: a long drag path takes
  as many frames as it has points.
- **A screenshot is the next frame**, not the last one: it waits for a frame to be drawn.
- **`edit.delete` and `edit.undo` answer once the scan pipeline has done them** (a few hundred ms on a large
  model); `process.run` and `calib.solve` start background work -- poll `process.status` / `calib.state`.

## Tests

`test_agent` (the protocol, the dispatcher, selectors, synthetic input, logs, screenshots and the socket,
against a headless ImGui), `test_agent_methods` (methods.json is the C++ table) and `einstar_mcp_e2e`
(`tools/mcp/tests/test_e2e.py`, registered when `uv` is installed: the server driving both apps headless with
the emulator -- a scan, a delete, processing, an export, a calibration capture).
