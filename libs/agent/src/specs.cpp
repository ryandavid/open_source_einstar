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
json vec3(std::string d) { return {{"type", "array"}, {"items", {{"type", "number"}}}, {"minItems", 3}, {"maxItems", 3}, {"description", std::move(d)}}; }
json ref(std::string d) { return {{"type", {"string", "integer"}}, {"description", std::move(d)}}; }
json refs(std::string d) { return {{"type", "array"}, {"items", {{"type", {"string", "integer"}}}}, {"description", std::move(d)}}; }
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
    add("scan.open", scan, mutating,
        "Opens a recording (.estr) as a paused scan: its model is rebuilt from the recorded poses, and it can be viewed, "
        "edited (edit.*) and processed without a scanner. With the scanner it was made with connected, scan.start "
        "continues it: the scanner finds its place on the model and new frames are appended to the same file. Answers "
        "when loaded, or with loading=true if it takes longer (then scan.state's 'load').",
        object({{"path", str("The .estr file.")}}, {"path"}));
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

    // ---- the modelling app: scan -> faces, holes, fillets -> constraints -> solid -> STEP ----
    // These are the document's commands (libs/model, Document::apply); a test keeps the two lists equal.
    const json kLabel = ref("A label: its name or id.");
    const json kAxis = one_of({"x", "y", "z"}, "A datum axis.");
    add("model.summary", model, read_only,
        "The whole document: the scan (size, bounds, voxel), every label (role, triangles, painted seeds, fitted and solved "
        "surface, noise), holes, fillets, datums, constraints with their status from the last solve, the built model's state, "
        "and undo depth. Start here, and check it after changes.");
    add("model.label", model, read_only, "One label in detail: its surface, area, centroid, bounds and the labels it borders.",
        object({{"label", kLabel}}, {"label"}));
    add("model.deviation", model, read_only,
        "How far the scan is from the built model: overall and per label (rms, p95, max, share within tolerance), hot spots "
        "(patches beyond tolerance: centroid, size, mean and peak, the model face and label), the number of edge bands (thin "
        "strips where the scan rounds sharp edges: expected, not errors), and model faces the scan does not cover. "
        "+ means the scan lies outside the model (material missing from the model).",
        object({{"max_hot_spots", integer("At most this many hot spots (default 10).")},
                {"include_edge_bands", boolean("List edge bands too (default false).")}}));
    add("model.raycast", model, read_only, "The scan point (and its label) where a ray first meets the scan.",
        object({{"origin", vec3("Ray start, mm.")}, {"direction", vec3("Ray direction.")}}, {"origin", "direction"}));
    add("model.history", model, read_only, "The undo and redo steps (description and author).");
    add("model.open", model, mutating,
        "Opens a document (.emodel) or a scan to model: an .estr recording (processed first: seconds to minutes), or an STL/PLY "
        "mesh. A scan starts a new document.",
        object({{"path", str("The file.")}, {"fine", boolean("For .estr: process at 0.3 mm voxels (default 0.5).")}}, {"path"}));
    add("model.open_demo", model, mutating, "Opens the demo part: a scanned box with mounting flanges, fillets and holes.");
    add("model.save", model, read_only, "Saves the document (.emodel: the scan's mesh and everything modelled on it).",
        object({{"path", str("Where (default: where it was opened or last saved).")}}));
    add("model.label.create", model, mutating,
        "Creates an empty label (a named region of the scan). Paint seeds into it with model.paint, then model.grow.",
        object({{"name", str("Unique name (default 'label N').")},
                {"role", one_of({"face", "hole", "fillet", "ignore"}, "face (default): a face of the part; hole: a hole's wall; fillet; "
                                                                     "ignore: scan that is not the part (table, fixture).")},
                {"kinds", {{"type", "array"}, {"items", {{"type", "string"}, {"enum", {"plane", "cylinder", "cone", "sphere", "torus"}}}},
                           {"description", "Surface kinds it may be (default: any; the simplest that fits is chosen)."}}}}));
    add("model.label.update", model, mutating, "Renames a label or changes its role or allowed kinds.",
        object({{"label", kLabel}, {"name", str("New name.")}, {"role", one_of({"face", "hole", "fillet", "ignore"}, "New role.")},
                {"kinds", {{"type", "array"}, {"items", {{"type", "string"}}}, {"description", "Allowed surface kinds."}}}},
               {"label"}));
    add("model.label.delete", model, mutating, "Deletes a label with its paint, its region, and the constraints, holes and fillets on it.",
        object({{"label", kLabel}}, {"label"}));
    add("model.paint", model, mutating,
        "Paints seed triangles of a label (or erases paint): a disc of the scan around a point, or listed triangles. A few mm "
        "on the middle of a face is enough; model.grow extends it to the whole face.",
        object({{"label", kLabel},
                {"point", vec3("A point on or near the scan (mm); the disc is grown from the nearest scan triangle.")},
                {"radius", num("Disc radius, mm.")},
                {"triangles", {{"type", "array"}, {"items", {{"type", "integer"}}}, {"description", "Triangle indices instead of a disc."}}},
                {"erase", boolean("Erase paint instead (of this label, or of any label when 'label' is not given).")}}));
    add("model.grow", model, mutating,
        "Grows painted labels into whole faces (all together, so neighbours meet at their edge) and fits each one's surface.",
        object({{"labels", refs("Only these (default: every label with paint).")}}));
    add("model.detect", model, mutating,
        "Finds faces on the unlabelled scan automatically: planes first, then cylinders, cones, spheres and tori. Labels them "
        "('plane 1', 'cylinder 2', ...), sorts out fillets (cylinders along the edge of two planes), hole walls and fillet "
        "corners, and finds the holes in the planes. Rename the labels to meaningful names afterwards.");
    add("model.freeform", model, mutating,
        "Makes a label a freeform face (a smooth B-spline surface) for a region no plane, cylinder, cone, sphere or torus "
        "describes, e.g. a domed or sculpted top. Fitted to the label's region, or to its paint taken out over the "
        "connected unlabelled scan up to sharp creases. Freeform faces are held as fitted (no constraints on them).",
        object({{"label", kLabel},
                {"extend", boolean("Take in the connected unlabelled scan first (default: when the label has no region).")},
                {"crease_deg", num("Edges sharper than this stop the extension (default 20).")},
                {"spacing_mm", num("Control point spacing (default: about 12 across the region).")},
                {"smoothness", num("Bending against fit (default 0.02).")}},
               {"label"}));
    add("model.find_holes", model, mutating,
        "Finds round holes in a plane label (openings in its region): centre, axis, diameter (from the wall if scanned, else "
        "from the opening), through or blind.",
        object({{"label", kLabel}}, {"label"}));
    add("model.hole.update", model, mutating,
        "Sets a hole's true size from a measurement (overrides the scan's), its depth, or makes it through; and its forms: "
        "a counterbore (diameter and depth), a countersink (diameter at the face and included angle, default 90), a drill "
        "point at a blind hole's bottom (included angle, 118 for a twist drill; the depth is then to the shoulder). "
        "Detection measures the forms it sees; null removes one.",
        object({{"hole", ref("A hole: its name or id.")}, {"diameter", num("Measured diameter, mm (null: back to the scan's).")},
                {"depth", num("Blind hole depth, mm.")}, {"through", boolean("A through hole.")}, {"name", str("New name.")},
                {"counterbore_diameter", num("mm.")}, {"counterbore_depth", num("mm, from the face.")},
                {"countersink_diameter", num("mm, at the face.")}, {"countersink_angle_deg", num("Included angle.")},
                {"point_angle_deg", num("Included angle of a drill point.")}},
               {"hole"}));
    add("model.hole.delete", model, mutating, "Deletes a hole.", object({{"hole", ref("A hole: its name or id.")}}, {"hole"}));
    add("model.fillet.add", model, mutating,
        "A fillet on the edge between two faces: from a fillet label (its faces and radius found from it), or between labels "
        "'a' and 'b' with a radius.",
        object({{"label", kLabel}, {"a", kLabel}, {"b", kLabel}, {"radius", num("Radius, mm (default: measured from the label).")},
                {"name", str("Name.")}}));
    add("model.fillet.update", model, mutating, "Sets a fillet's radius (null: back to the measured one).",
        object({{"fillet", ref("A fillet: its name or id.")}, {"radius", num("Radius, mm.")}}, {"fillet"}));
    add("model.fillet.delete", model, mutating, "Deletes a fillet.", object({{"fillet", ref("A fillet: its name or id.")}}, {"fillet"}));
    add("model.datum.create", model, mutating,
        "A coordinate frame for the part: z along label 'z' (a face's normal or an axis), x as near label 'x''s direction as "
        "possible. Solved with the faces; constraints align faces to its axes and offset them from its origin, and the "
        "exported part sits in it.",
        object({{"name", str("Name (default 'datum').")}, {"z", kLabel}, {"x", kLabel}}, {"z", "x"}));
    add("model.datum.delete", model, mutating, "Deletes a datum and its constraints.", object({{"datum", ref("A datum: its name or id.")}}, {"datum"}));
    add("model.face.add_plane", model, mutating,
        "Adds a face the scan did not see (e.g. the bottom the part stood on), as a plane square to a datum axis at an offset "
        "from its origin. Needed where the scan is open, or the solid cannot be closed.",
        object({{"name", str("Name.")}, {"datum", ref("The datum.")}, {"axis", kAxis}, {"offset", num("Along the axis from the datum origin, mm.")},
                {"facing", one_of({"+", "-"}, "Which way the face looks out of the part: along (+, default) or against (-) the axis.")}},
               {"datum", "axis", "offset"}));
    add("model.constraint.add", model, mutating,
        "Adds a constraint (held exactly by model.solve): aligned {label, datum, axis}: a face's normal or an axis along a "
        "datum axis; parallel / perpendicular / coplanar / coaxial {a, b}; angle {a, b, degrees}; radius {label, value}; "
        "diameter {label or hole, value} (a measured hole size); distance {a, b, value}: between parallel planes; offset "
        "{label, datum, axis, value}: a face or axis at a position along a datum axis; axis_distance {a, b, value}: two "
        "parallel axes this far apart (a hole pitch, or a hole from a boss or bolt-circle centre); tangent {a, b}: a plane "
        "and a cylinder or sphere, or two cylinders, touching (a fillet against its faces, a slot's rounded end); "
        "symmetric {a, b, datum, axis}: two faces or axes mirrored about the datum's plane normal to that axis; "
        "equal_radius {a, b}: the same radius, whatever it is. Where an axis is meant (aligned, parallel, perpendicular, "
        "angle, coaxial, offset, axis_distance, symmetric) a hole may be named in place of a label. Prefer one datum with "
        "aligned faces over many pairwise perpendicular constraints.",
        object({{"type", one_of({"aligned", "parallel", "perpendicular", "angle", "coplanar", "coaxial", "radius", "diameter", "distance", "offset",
                                 "axis_distance", "tangent", "symmetric", "equal_radius"},
                                "The constraint.")},
                {"label", kLabel}, {"a", kLabel}, {"b", kLabel}, {"hole", ref("A hole (diameter).")}, {"datum", ref("A datum.")},
                {"axis", kAxis}, {"value", num("mm.")}, {"degrees", num("0-90.")}},
               {"type"}));
    add("model.constraint.remove", model, mutating, "Removes a constraint.", object({{"constraint", integer("Its id.")}}, {"constraint"}));
    add("model.square", model, mutating,
        "Aligns plane faces to the nearest axis of a datum (the usual first step for a machined part: 'make the box square').",
        object({{"datum", ref("The datum.")}, {"labels", refs("Only these (default: every plane face).")},
                {"max_angle_deg", num("Leave out faces further than this from every axis (default 10).")}},
               {"datum"}));
    add("model.solve", model, mutating,
        "Fits every face together under the constraints. Per constraint: satisfied / redundant / conflict / invalid, and what "
        "it costs (how far it moved surfaces off the scan); per label: rms, movement from its own best fit, uncertainty; "
        "and 'scale': what the measured lengths say about the scan's scale (a factor, and whether it is significant).");
    add("model.build", model, mutating,
        "Builds the solid from the faces, holes and fillets, then measures the scan's deviation from it (see model.deviation). "
        "Says when the faces do not enclose the part (add the missing face where the scan is open).",
        object({{"tolerance_mm", num("Deviation tolerance (default 0.1).")}, {"deflection_mm", num("Tessellation accuracy (default 0.02).")}}));
    add("model.scale", model, mutating,
        "Scales the scan and everything modelled on it (a calibration correction), e.g. by the factor model.solve's "
        "'scale' check suggests from the user's measurements. Clears the undo history.",
        object({{"factor", num("0.9-1.1.")}}, {"factor"}));
    add("model.reprocess", model, mutating,
        "Makes the scan's mesh again from its source file, when it has changed (model.summary's scan.source is "
        "'changed': e.g. an .estr scanned further or edited in the scanning app). Labels move to the new mesh by "
        "nearest triangle, hole openings to the nearest vertices, and fitted faces are refitted; solve and build "
        "again after. Clears the undo history. Seconds to minutes for an .estr.",
        object({{"path", str("The source (default: the scan's own, e.g. after it was moved).")},
                {"fine", boolean("For .estr: 0.3 mm voxels (default: as it was processed before).")}}));
    // Photos of the part (taken anywhere: they need not show the scan's setting) and what the user marked on them.
    const json points = {{"type", "array"}, {"items", {{"type", "array"}, {"items", {{"type", "number"}}}, {"minItems", 2}, {"maxItems", 2}}},
                         {"description", "Photo pixels [[x, y], ...] (as shown, origin top left, y down): dimension 2 (its ends), diameter "
                                         "3 on the rim or 2 (centre, rim), angle 3 (the vertex in the middle), callout 2 (what it points "
                                         "at, then where its text sits), note 1."}};
    const json value = {{"type", {"number", "string", "null"}},
                        {"description", "mm (an angle: degrees), or as typed: '42 mm', '1.5 in', '1 1/2\"', 'Ø6', 'R2', '30°', with an optional "
                                        "'±0.05' (null: none)."}};
    add("model.photo.import", model, mutating,
        "Adds photos of the part to the document (stored in the .emodel as the files are: JPEG, HEIC, PNG ...). Photos may be "
        "taken anywhere, in another setting than the scan.",
        object({{"path", str("A photo file.")}, {"paths", {{"type", "array"}, {"items", {{"type", "string"}}}, {"description", "Photo files."}}},
                {"data_base64", str("A photo's bytes instead of a file.")}, {"name", str("Name (default: the file's name).")},
                {"caption", str("What it shows.")}}));
    add("model.photo.update", model, mutating, "Renames a photo, sets its caption, or whether it may colour the scan.",
        object({{"photo", ref("A photo: its name or id.")}, {"name", str("New name.")}, {"caption", str("Caption.")},
                {"use_for_colour", boolean("Once registered, may colour the scan (off when something covers the part).")}},
               {"photo"}));
    add("model.photo.delete", model, mutating, "Removes a photo and its annotations.", object({{"photo", ref("A photo: its name or id.")}}, {"photo"}));
    add("model.photo.annotate", model, mutating,
        "Marks something on a photo: a dimension (two points and a length), a diameter (a circle), an angle, a callout (an "
        "arrow with text and/or a value) or a note. Named D1, DIA2, A3, C4, N5 ... Links say which labels, holes or fillets it "
        "is about. Use it to keep what the user tells you, attached to the picture.",
        object({{"photo", ref("A photo: its name or id.")}, {"kind", one_of({"dimension", "diameter", "angle", "callout", "note"}, "What it is.")},
                {"points", points}, {"value", value}, {"tolerance", num("+- in the value's unit.")}, {"text", str("What it says.")},
                {"links", refs("Labels, holes or fillets it is about.")}},
               {"photo", "kind", "points"}));
    add("model.photo.annotation.update", model, mutating, "Changes an annotation's points, value, tolerance, text or links.",
        object({{"annotation", ref("An annotation: its name (D1 ...) or id.")}, {"points", points}, {"value", value},
                {"tolerance", num("+- in the value's unit (null: none).")}, {"text", str("What it says.")},
                {"links", refs("Labels, holes or fillets it is about (replaces them).")}},
               {"annotation"}));
    add("model.photo.annotation.delete", model, mutating, "Deletes an annotation.",
        object({{"annotation", ref("An annotation: its name (D1 ...) or id.")}}, {"annotation"}));
    add("model.photo.list", model, read_only,
        "The photos with their annotations in full (points, values, text, links), and the part's notes. model.photo.get shows one.",
        object({{"photo", ref("Only this photo.")}}));
    add("model.photo.get", model, read_only,
        "A photo as an image to look at, scaled down, with its annotations drawn and named as in model.photo.list. Optionally "
        "a crop of it, or a grid labelled with photo pixel coordinates to point at things with (model.photo.annotate).",
        object({{"photo", ref("A photo: its name or id.")}, {"max_size", integer("Longer side of the image, px (default 1600).")},
                {"crop", {{"type", "array"}, {"items", {{"type", "number"}}}, {"minItems", 4}, {"maxItems", 4},
                          {"description", "[x, y, w, h] in photo pixels."}}},
                {"annotations", boolean("Draw the annotations (default true).")}, {"grid", boolean("Draw a pixel grid (default false).")},
                {"overlay", boolean("A registered photo: draw the model's edges (or the label boundaries) over it, to check the "
                                    "registration and compare model and part (default false).")}},
               {"photo"}));
    add("model.photo.correspond", model, mutating,
        "Matches a point of the part in a photo (pixel) to the same point on the scan (e.g. from model.raycast). With 6 "
        "matched points (4 when EXIF gives the focal length) the photo's camera is solved, relative to the part only: "
        "then annotations get the scan's own reading, links are suggested, and model.photo.get can draw the model over the "
        "photo. Spread the points over the part, at corners and edges that are easy to place exactly. Points on scan "
        "labelled ignore (not the part) are refused.",
        object({{"photo", ref("A photo: its name or id.")}, {"pixel", point("[x, y] in photo pixels.")}, {"point", vec3("The same point on the scan, mm.")}},
               {"photo", "pixel", "point"}));
    add("model.photo.correspond.remove", model, mutating, "Removes a matched point (the camera is solved again).",
        object({{"photo", ref("A photo: its name or id.")}, {"id", integer("The matched point's id.")}}, {"photo", "id"}));
    add("model.photo.register", model, mutating,
        "Solves the photo's camera again from its matched points; reports each point's error in px and which disagree with "
        "the rest.",
        object({{"photo", ref("A photo: its name or id.")}}, {"photo"}));
    add("model.photo.apply", model, mutating,
        "Makes an annotation's value hold on the model, by what it is linked to: two faces, a distance; two holes, their "
        "pitch; a hole (a diameter, or a value typed with Ø), its size; a fillet (typed with R), its radius; a cylinder, its "
        "diameter or radius; two faces with an angle, the angle. The constraint's status and cost show in model.photo.list "
        "after model.solve. Applying again replaces it.",
        object({{"annotation", ref("An annotation: its name (D1 ...) or id.")}}, {"annotation"}));
    add("model.photo.project", model, read_only,
        "For a registered photo: what a pixel shows on the part (a point and its label), and/or where a scan point appears in "
        "the photo (and whether the part hides it there).",
        object({{"photo", ref("A photo: its name or id.")}, {"pixel", point("[x, y] in photo pixels.")}, {"point", vec3("A point, mm.")}},
               {"photo"}));
    add("model.note.add", model, mutating,
        "Writes down a fact about the part that is not in the scan: material, finish, a thread, 'the bottom is flat', what it "
        "mates with. Kept in the document for later sessions.",
        object({{"text", str("The note.")}}, {"text"}));
    add("model.note.update", model, mutating, "Rewrites a note.", object({{"note", integer("Its id.")}, {"text", str("The note.")}}, {"note", "text"}));
    add("model.note.delete", model, mutating, "Deletes a note.", object({{"note", integer("Its id.")}}, {"note"}));
    add("model.export_step", model, read_only, "Writes the built solid as STEP (AP242, mm, faces named after labels) and reads it back to check.",
        object({{"path", str("The .step file.")}}, {"path"}));
    add("model.view", model, mutating,
        "What the modelling app's 3D view shows and from where (then ui.screenshot to see it). mode: labels (each label's "
        "colour; painted seeds brighter), deviation (green within tolerance, yellow-red where the scan is outside the model, "
        "cyan-blue inside), model (the built solid), photos (the scan coloured from its registered photos). preset: look from top, bottom, front, back, left, right or iso (in the "
        "first datum's axes when there is one). frame: fit a label (or 'all') in the view.",
        object({{"mode", one_of({"labels", "deviation", "model", "photos"}, "What to show.")},
                {"preset", one_of({"top", "bottom", "front", "back", "left", "right", "iso", "scanned"},
                                  "Look from (scanned: the side the scanner saw the surface from).")},
                {"frame", str("A label's name, or 'all'.")},
                {"edges", boolean("Draw the model's edges.")},
                {"orbit", point("[dx, dy] radians, as a left-drag would.")},
                {"zoom", num("Multiply the distance by this (0.5: twice as close).")}}));
    add("model.begin_change", model, mutating,
        "Starts a change: everything until model.end_change is one undo step (use it around each request you carry out).",
        object({{"description", str("What the change does, as the user sees it in the history.")}}));
    add("model.end_change", model, mutating, "Ends the change begun with model.begin_change.");
    add("model.undo", model, mutating, "Undoes the last change.");
    add("model.redo", model, mutating, "Redoes the last undone change.");
    return m;
}

}  // namespace

std::string_view app_name(App a) {
    switch (a) {
        case App::any: return "any";
        case App::scan: return "scan";
        case App::calibration: return "calibration";
        case App::model: return "model";
    }
    return "any";
}

std::optional<App> app_from_name(std::string_view name) {
    for (const App a : {App::any, App::scan, App::calibration, App::model})
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
