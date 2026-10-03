// Every agent method, once: the apps register handlers against these, einstar-mcp lists them as tools.

#include <algorithm>

#include "einstar/agent/protocol.hpp"

namespace einstar::agent {
namespace {

json object(json properties = json::object(), json required = json::array()) {
    json s = {{"type", "object"}, {"properties", std::move(properties)}, {"additionalProperties", false}};
    if (!required.empty()) s["required"] = std::move(required);
    return s;
}
json str(std::string d) { return {{"type", "string"}, {"description", std::move(d)}}; }
json num(std::string d) { return {{"type", "number"}, {"description", std::move(d)}}; }
json integer(std::string d) { return {{"type", "integer"}, {"description", std::move(d)}}; }
json boolean(std::string d) { return {{"type", "boolean"}, {"description", std::move(d)}}; }
json one_of(std::vector<std::string> values, std::string d) { return {{"type", "string"}, {"enum", std::move(values)}, {"description", std::move(d)}}; }
json point(std::string d) { return {{"type", "array"}, {"items", {{"type", "number"}}}, {"minItems", 2}, {"maxItems", 2}, {"description", std::move(d)}}; }
json modifiers() {
    return {{"type", "array"}, {"items", {{"type", "string"}, {"enum", {"shift", "ctrl", "alt", "super"}}}},
            {"description", "Modifier keys held during the action (alt = Option, super = Cmd)."}};
}

const json kTarget = str(
    "An ImGui item: its ref from ui.snapshot ('#1a2b3c4d', stable while the item exists), 'Window/Label' (the window's "
    "name and the item's visible label), or a bare label. Matching nothing or more than one item is an error that lists "
    "the candidates.");

std::vector<MethodSpec> build() {
    std::vector<MethodSpec> m;
    auto add = [&](std::string name, App app, Kind kind, std::string description, json params = object()) {
        m.push_back({std::move(name), app, kind, std::move(description), std::move(params)});
    };
    using enum App;
    using enum Kind;

    // ---- generic: every app ----
    add("rpc.methods", any, read_only, "The methods this running app answers (names only).");
    add("app.info", any, read_only,
        "The app: its name, pid, frames drawn, window size in points, the framebuffer scale, and whether its window is visible.");
    add("app.logs", any, read_only,
        "The app's log since a cursor: pass the returned next_seq as since_seq to get only what is new. `dropped` > 0 means "
        "the ring wrapped and records were lost before they were read.",
        object({{"since_seq", integer("Return records with seq >= this (0: from the oldest kept).")},
                {"limit", integer("At most this many records (default 200).")},
                {"level", one_of({"trace", "debug", "info", "warn", "error"}, "Minimum level.")},
                {"grep", str("Only records containing this text.")}}));
    add("app.quit", any, read_only, "Quits the app (as closing its window would); answers before it goes.");
    add("frame.wait", any, read_only, "Waits until the app has drawn this many more frames (default 1).",
        object({{"frames", integer("Frames to wait for (1-600).")}}));

    add("ui.snapshot", any, read_only,
        "Every ImGui window and item drawn in the last frame: ref, window, label, rect [x, y, w, h] in window points, and "
        "state (checkable/checked, openable/opened, inputable, disabled, visible). Static text is not an item: use the app's "
        "own state methods or a screenshot to read it.",
        object({{"window", str("Only items of this window (its name).")},
                {"interactive_only", boolean("Only items that take input (default false).")}}));
    add("ui.find", any, read_only, "Resolves a target to one item (its ref, window, label, rect and state).",
        object({{"target", kTarget}}, {"target"}));
    add("ui.screenshot", any, read_only,
        "A PNG of the next frame drawn: the whole window, an item's rect (target) or a rect in points. Returns png_base64, "
        "scale (image px per point), logical_rect [x, y, w, h] in points and image_size; an image pixel (px, py) is the "
        "point (logical_rect.x + px / scale, logical_rect.y + py / scale).",
        object({{"target", kTarget},
                {"rect", {{"type", "array"}, {"items", {{"type", "number"}}}, {"minItems", 4}, {"maxItems", 4},
                          {"description", "[x, y, w, h] in window points."}}},
                {"max_size", integer("Downscale so the longer side is at most this many px (default 1600).")}}));
    add("ui.wait_for", any, read_only,
        "Waits until an item exists (or is gone, or is enabled), checking each frame. Errors with `timeout` if it does not "
        "happen within timeout_ms.",
        object({{"target", kTarget},
                {"state", one_of({"exists", "gone", "enabled"}, "What to wait for (default exists).")},
                {"timeout_ms", integer("How long to wait (default 5000).")}},
               {"target"}));

    add("input.click", any, mutating,
        "Clicks an item (at its centre, or at x, y within it) or a point in the window. Synthesised through ImGui's input "
        "queue, so it reaches the 3D view as well as widgets.",
        object({{"target", kTarget},
                {"x", num("Window points (with a target: points from the item's top left).")},
                {"y", num("Window points (with a target: points from the item's top left).")},
                {"button", one_of({"left", "right", "middle"}, "Default left.")},
                {"double", boolean("A double click.")},
                {"modifiers", modifiers()}}));
    add("input.move", any, mutating, "Moves the mouse to a point in the window (hover).",
        object({{"x", num("Window points.")}, {"y", num("Window points.")}}, {"x", "y"}));
    add("input.drag", any, mutating,
        "Presses at `from`, moves in steps to `to` and releases: orbiting (left) or panning (right) the 3D view, a slider, "
        "or with modifiers a lasso in the scanning app (shift: select).",
        object({{"from", point("[x, y] window points.")},
                {"to", point("[x, y] window points.")},
                {"path", {{"type", "array"}, {"items", {{"type", "array"}, {"items", {{"type", "number"}}}}},
                          {"description", "Instead of from/to: every [x, y] point of the drag in order (e.g. a lasso outline)."}}},
                {"button", one_of({"left", "right", "middle"}, "Default left.")},
                {"modifiers", modifiers()},
                {"steps", integer("Intermediate moves between from and to (default 12).")}}));
    add("input.scroll", any, mutating, "Scrolls the mouse wheel at a point (zooms the 3D view).",
        object({{"x", num("Window points (default: where the mouse is).")},
                {"y", num("Window points (default: where the mouse is).")},
                {"dx", num("Horizontal wheel steps.")},
                {"dy", num("Vertical wheel steps (positive: up / zoom in).")}}));
    add("input.key", any, mutating,
        "Presses and releases a key (ImGui key names: 'Escape', 'Delete', 'Backspace', 'Enter', 'Z', 'Space', 'F1', ...).",
        object({{"key", str("ImGui key name.")}, {"modifiers", modifiers()}}, {"key"}));
    add("input.type", any, mutating, "Types text into the focused text field.", object({{"text", str("The text.")}}, {"text"}));

    // ---- the scanning app ----
    add("scan.state", scan, read_only,
        "Everything about the scanning app now: connection, workflow step and scan type, whether it is scanning, the HUD "
        "(tracking, fps, frames, model points, markers, distance), the recording, processing, editing and view state.");
    add("device.connect", scan, mutating,
        "Connects the scanner over USB, or the built-in emulator (a simulated scanner and scene, no hardware).",
        object({{"emulator", boolean("Use the emulator instead of a scanner.")}}, {"emulator"}));
    add("device.disconnect", scan, mutating, "Disconnects (stops scanning).");
    add("workflow.goto", scan, mutating, "Opens a workflow step (as clicking its header). Not while scanning.",
        object({{"step", one_of({"connect", "scan_type", "markers", "scan", "process"}, "The step.")}}, {"step"}));
    add("scan.set_type", scan, mutating,
        "The scan type: hybrid (surface + markers, the default), surface (shape only) or global_markers (capture and "
        "optimise a marker map first). Not while scanning.",
        object({{"type", one_of({"hybrid", "surface", "global_markers"}, "The type.")}}, {"type"}));
    add("scan.start", scan, mutating, "Starts or resumes scanning (opens the scan step).");
    add("scan.pause", scan, mutating, "Pauses scanning; the model and recording are kept.");
    add("scan.new", scan, mutating, "A new scan: clears the model and starts a new recording.",
        object({{"discard", boolean("Delete the current recording (default false: it is kept).")}}));
    add("scan.settings", scan, mutating,
        "Reads, and with any field given changes, the scanner's settings.",
        object({{"brightness", integer("Brightness level (sets exposure and gain).")},
                {"exposure", integer("Exposure.")},
                {"gain", integer("Gain.")},
                {"laser_percent", integer("Projector power 0-100.")},
                {"strobe", integer("Strobe.")},
                {"record_raw_ir", boolean("Also record raw IR images.")}}));
    add("markers.optimize", scan, mutating, "Bundle-adjusts the captured global markers into a fixed map (markers step, paused).");
    add("markers.clear", scan, mutating, "Discards the global markers (and their map).");
    add("markers.save", scan, mutating, "Saves the global-marker map.", object({{"path", str("File to write.")}}, {"path"}));
    add("markers.load", scan, mutating, "Loads a global-marker map.", object({{"path", str("File to read.")}}, {"path"}));
    add("process.run", scan, mutating,
        "Processes the recorded scan into a mesh (in the background; poll process.status). Not while scanning.",
        object({{"resolution_mm", one_of({"0.3", "0.5", "1.0"}, "Voxel size (default 0.5).")},
                {"optimize_poses", boolean("Loop closure / pose graph (default true).")},
                {"smoothing", integer("Smoothing iterations 0-10 (default 0).")},
                {"simplify", boolean("Simplify within 0.02 mm (default true).")}}));
    add("process.status", scan, read_only, "Processing: running, stage, fraction, done, summary.");
    add("process.cancel", scan, mutating, "Cancels processing.");
    add("process.export", scan, mutating, "Exports the processed mesh (.stl, .ply or .obj by extension).",
        object({{"path", str("File to write.")}}, {"path"}));
    add("view.get", scan, read_only,
        "The 3D view's camera (target, distance, orientation as a quaternion [w, x, y, z], eye, forward) and options.");
    add("view.set", scan, mutating,
        "Moves the 3D view's camera (leaves follow mode) and/or changes its options. Orientation takes camera axes (x right, "
        "y down, z forward) to world axes.",
        object({{"target", {{"type", "array"}, {"items", {{"type", "number"}}}, {"minItems", 3}, {"maxItems", 3}, {"description", "World mm."}}},
                {"distance", num("Eye to target, mm.")},
                {"orientation", {{"type", "array"}, {"items", {{"type", "number"}}}, {"minItems", 4}, {"maxItems", 4},
                                 {"description", "Quaternion [w, x, y, z]."}}},
                {"orbit", point("[dx, dy] radians, as a left-drag would.")},
                {"reset", boolean("Back to the default view first.")},
                {"follow", boolean("Follow the scanner.")},
                {"point_size_mm", num("Splat size.")},
                {"lighting", boolean("Shading.")},
                {"show_points", boolean("Live points.")},
                {"show_mesh", boolean("Processed mesh.")},
                {"show_scanner", boolean("The scanner model.")}}));
    add("edit.select", scan, mutating,
        "Adds a lasso stroke to the selection (paused scans): a polygon in window points, as seen in the current view; "
        "everything inside it, front to back, is selected (subtract: removed from the selection).",
        object({{"polygon", {{"type", "array"}, {"items", {{"type", "array"}, {"items", {{"type", "number"}}}}}, {"minItems", 3},
                             {"description", "[[x, y], ...] window points."}}},
                {"subtract", boolean("Deselect instead.")}},
               {"polygon"}));
    add("edit.clear", scan, mutating, "Clears the selection.");
    add("edit.delete", scan, mutating,
        "Deletes the selection from the live model and the recording (processing never sees it); undoable until scanning "
        "resumes. Answers once done.");
    add("edit.undo", scan, mutating, "Undoes the last delete. Answers once done.");
    add("edit.status", scan, read_only, "Selection strokes, undoable deletes, the last message.");

    // ---- the calibration app ----
    add("calib.state", calibration, read_only,
        "The calibration app now: connection, the 25-view plan with what is captured, the active group, the live view "
        "(distance, tilts, roll, offset, hints, whether in position and steady), lighting, and the solve.");
    add("calib.connect", calibration, mutating, "Connects the scanner, or the emulator (a simulated scanner and board).",
        object({{"emulator", boolean("Use the emulator.")}}, {"emulator"}));
    add("calib.disconnect", calibration, mutating, "Disconnects.");
    add("calib.group", calibration, mutating,
        "Selects the orientation group to capture next: 0 face-on, 1 top edge near, 2 bottom edge near, 3 right edge "
        "near, 4 left edge near.",
        object({{"group", integer("0-4.")}}, {"group"}));
    add("calib.auto_capture", calibration, mutating, "Turns capturing automatically when steady on or off.",
        object({{"on", boolean("On.")}}, {"on"}));
    add("calib.capture", calibration, mutating, "Captures the current view now (as the Capture now button).");
    add("calib.clear", calibration, mutating, "Clears one capture, or all of them (a new session).",
        object({{"index", integer("Plan index 0-24 (omit: all).")}}));
    add("calib.load_folder", calibration, mutating, "Loads a folder of captures (imageLeftN / imageRightN).",
        object({{"path", str("Folder.")}}, {"path"}));
    add("calib.load_reference", calibration, mutating, "Loads a reference calibration to compare with.",
        object({{"path", str("File.")}}, {"path"}));
    add("calib.solve", calibration, mutating, "Solves the stereo calibration from the captures (background; poll calib.state).",
        object({{"keep_factory_distortion", boolean("Keep the factory distortion (default false).")}}));
    add("calib.save", calibration, mutating, "Saves the solved calibration (returns the path).");
    add("calib.plan_write", calibration, read_only, "What writing to the scanner would do: the gates (all must pass), the backup path.");
    add("calib.write", calibration, mutating,
        "Writes the solved calibration into the scanner's flash (stream paused, current pages backed up first, read back and "
        "verified). Real hardware: this replaces the scanner's calibration.",
        object({{"confirm", boolean("Must be true.")}}, {"confirm"}));
    add("calib.restore", calibration, mutating, "Puts a calibration backup back into the scanner's flash.",
        object({{"path", str("Backup file (flash-backup-*.bin).")}, {"confirm", boolean("Must be true.")}}, {"path", "confirm"}));
    add("calib.lighting", calibration, mutating, "Reads, and with any field given changes, the lighting.",
        object({{"exposure", integer("300-8000.")}, {"gain", integer("Gain % 16-800.")},
                {"ring_light", integer("0-9000.")}, {"white_leds", integer("0-9000.")}}));
    add("calib.tab", calibration, mutating, "Shows a tab of the main view.",
        object({{"tab", one_of({"live", "captures", "results"}, "The tab.")}}, {"tab"}));
    return m;
}

}  // namespace

std::string_view app_name(App a) {
    switch (a) {
        case App::any: return "any";
        case App::scan: return "scan";
        case App::calibration: return "calibration";
    }
    return "any";
}

std::optional<App> app_from_name(std::string_view name) {
    for (const App a : {App::any, App::scan, App::calibration})
        if (app_name(a) == name) return a;
    return std::nullopt;
}

const std::vector<MethodSpec>& method_specs() {
    static const std::vector<MethodSpec> specs = build();
    return specs;
}

json method_specs_json() {
    json out = json::array();
    for (const auto& s : method_specs())
        out.push_back({{"name", s.name},
                       {"app", app_name(s.app)},
                       {"kind", s.kind == Kind::mutating ? "mutating" : "read_only"},
                       {"description", s.description},
                       {"params", s.params}});
    return out;
}

const MethodSpec* find_spec(std::string_view name) {
    const auto& s = method_specs();
    const auto it = std::ranges::find(s, name, &MethodSpec::name);
    return it == s.end() ? nullptr : &*it;
}

}  // namespace einstar::agent
