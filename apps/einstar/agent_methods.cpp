#include "agent_methods.hpp"

#include <algorithm>
#include <format>

namespace einstar::app {
namespace {

using agent::ErrorCode;
using agent::error;
using agent::json;
using agent::opt_param;
using agent::param;
using Outcome = agent::Server::Outcome;
using Step = WorkflowUi::Step;
using ScanType = WorkflowUi::ScanType;

constexpr std::pair<Step, const char*> kSteps[] = {
    {Step::connect, "connect"}, {Step::scan_type, "scan_type"}, {Step::markers, "markers"}, {Step::scan, "scan"}, {Step::process, "process"}};
constexpr std::pair<ScanType, const char*> kTypes[] = {{ScanType::hybrid, "hybrid"}, {ScanType::surface, "surface"}, {ScanType::global_markers, "global_markers"}};

template <typename E, std::size_t N>
const char* name_of(const std::pair<E, const char*> (&table)[N], E v) {
    for (const auto& [e, n] : table)
        if (e == v) return n;
    return "?";
}
template <typename E, std::size_t N>
std::optional<E> from_name(const std::pair<E, const char*> (&table)[N], std::string_view s) {
    for (const auto& [e, n] : table)
        if (s == n) return e;
    return std::nullopt;
}

json vec(const Eigen::Vector3f& v) { return {v.x(), v.y(), v.z()}; }

json connection_json(const Connection& c) {
    const char* kind = c.kind == Connection::Kind::scanner ? "scanner" : c.kind == Connection::Kind::emulator ? "emulator" : "none";
    return {{"kind", kind}, {"online", c.online}, {"device", c.device}, {"calibration", c.calibration}, {"error", c.error}};
}

json settings_json(const AppState& s) {
    return {{"brightness", s.settings.brightness}, {"exposure", s.settings.exposure},   {"gain", s.settings.gain},
            {"laser_percent", s.settings.laser_percent}, {"strobe", s.settings.strobe}, {"record_raw_ir", s.record_raw_ir()}};
}

json process_json(const ProcessStatus& p) {
    return {{"running", p.running}, {"done", p.done}, {"stage", p.stage}, {"fraction", p.fraction}, {"summary", p.summary}};
}

json edit_json(const AppState& s) {
    const auto e = s.edit_status();
    return {{"can_edit", s.can_edit()}, {"selection_strokes", s.selection().strokes().size()}, {"selection_empty", s.selection().empty()},
            {"undo_depth", e.undo_depth}, {"busy", e.busy}, {"message", e.message}};
}

json view_json(const ScanAgentHost& h) {
    const auto& c = h.camera;
    return {{"target", vec(c.target)},
            {"distance", c.distance},
            {"orientation", {c.orientation.w(), c.orientation.x(), c.orientation.y(), c.orientation.z()}},
            {"eye", vec(c.eye())},
            {"forward", vec(c.forward())},
            {"follow", h.state.follow_scanner},
            {"point_size_mm", h.settings.point_size_mm},
            {"lighting", h.settings.lighting},
            {"show_points", h.settings.show_points},
            {"show_mesh", h.settings.show_mesh},
            {"show_scanner", h.settings.show_scanner}};
}

agent::Error refused(std::string why) { return error(ErrorCode::refused, std::move(why)); }

}  // namespace

LassoStroke lasso_stroke(const render::ViewCamera& camera, const std::vector<ImVec2>& outline, bool subtract) {
    const ImGuiIO& io = ImGui::GetIO();
    const float fb_w = io.DisplaySize.x * io.DisplayFramebufferScale.x, fb_h = io.DisplaySize.y * io.DisplayFramebufferScale.y;
    LassoStroke stroke;
    stroke.view_proj = camera.projection(fb_w / std::max(fb_h, 1.0f)) * camera.view();
    stroke.viewport = Eigen::Vector2f(fb_w, fb_h);
    stroke.subtract = subtract;
    for (const auto& q : outline) stroke.polygon.emplace_back(q.x * io.DisplayFramebufferScale.x, q.y * io.DisplayFramebufferScale.y);
    return stroke;
}

void register_scan_agent(agent::Server& server, ScanAgentHost h) {
    auto& st = h.state;
    auto& ui = h.workflow;

    server.handle("scan.state", [h, &st, &ui](const json&) -> Outcome {
        const auto hud = st.hud();
        return json{{"connection", connection_json(st.connection())},
                    {"status", st.status()},
                    {"workflow", {{"step", name_of(kSteps, ui.step)}, {"type", name_of(kTypes, ui.type)}}},
                    {"scanning", st.scanning()},
                    {"hud",
                     {{"tracking_lost", hud.tracking_lost},
                      {"reason", hud.reason},
                      {"fps", hud.fps},
                      {"frames", hud.frames},
                      {"model_points", hud.model_points},
                      {"markers_in_frame", hud.markers},
                      {"markers_matched", hud.markers_matched},
                      {"map_markers", hud.map_markers},
                      {"global_markers", hud.global_markers},
                      {"keyframes", hud.keyframes},
                      {"phase", hud.phase == pipeline::ScanPhase::global_markers ? "global_markers" : "surface"},
                      {"distance_mm", hud.distance_mm},
                      {"depth_ms", hud.depth_ms},
                      {"track_ms", hud.track_ms},
                      {"queue_depth", hud.queue_depth},
                      {"dropped", hud.dropped},
                      {"temperature_c", hud.temperature_c > -273.0f ? json(hud.temperature_c) : json(nullptr)},
                      {"notice", hud.notice}}},
                    {"recording", {{"path", st.recording_path()}, {"frames", hud.recorded_frames}, {"raw_frames", hud.raw_frames}}},
                    {"global_markers", st.global_marker_status()},
                    {"settings", settings_json(st)},
                    {"process", process_json(st.process_status())},
                    {"edit", edit_json(st)},
                    {"view", view_json(h)}};
    });

    server.handle("device.connect", [&st, &ui](const json& p) -> Outcome {
        if (st.scanning()) return refused("scanning: pause first");
        if (!st.connect(param<bool>(p, "emulator"))) return refused("could not connect: " + st.connection().error);
        enter_step(st, ui, Step::scan_type);
        return connection_json(st.connection());
    });
    server.handle("device.disconnect", [&st, &ui](const json&) -> Outcome {
        st.disconnect();
        ui.step = Step::connect;
        return json{{"connected", false}};
    });
    server.handle("workflow.goto", [&st, &ui](const json& p) -> Outcome {
        const auto step = from_name(kSteps, param<std::string>(p, "step"));
        if (!step) throw agent::Server::BadParams("step is connect, scan_type, markers, scan or process");
        if (st.scanning()) return refused("scanning: pause first");
        if (*step != Step::connect && !st.connected()) return refused("not connected: device.connect first");
        if (*step == Step::markers && ui.type != ScanType::global_markers) return refused("the markers step belongs to the global_markers scan type");
        enter_step(st, ui, *step);
        return json{{"step", name_of(kSteps, ui.step)}};
    });
    server.handle("scan.set_type", [&st, &ui](const json& p) -> Outcome {
        const auto type = from_name(kTypes, param<std::string>(p, "type"));
        if (!type) throw agent::Server::BadParams("type is hybrid, surface or global_markers");
        if (st.scanning()) return refused("scanning: pause first");
        ui.type = *type;
        if (ui.step == Step::scan) enter_step(st, ui, Step::scan);  // the alignment follows the type
        return json{{"type", name_of(kTypes, ui.type)}};
    });
    server.handle("scan.start", [&st, &ui](const json&) -> Outcome {
        if (!st.connected()) return refused("not connected: device.connect first");
        if (st.scanning()) return json{{"scanning", true}};
        if (ui.step != Step::scan && ui.step != Step::markers)
            enter_step(st, ui, ui.type == ScanType::global_markers && st.hud().global_markers == 0 ? Step::markers : Step::scan);
        st.start_scan();
        if (!st.scanning()) return refused("could not start: " + st.status());
        return json{{"scanning", true}, {"step", name_of(kSteps, ui.step)}};
    });
    server.handle("scan.pause", [&st](const json&) -> Outcome {
        st.stop_scan();
        return json{{"scanning", st.scanning()}};
    });
    server.handle("scan.new", [&st, &ui](const json& p) -> Outcome {
        if (!st.connected()) return refused("not connected");
        st.new_scan(opt_param<bool>(p, "discard").value_or(false));
        ui.scanned = {};
        return json{{"new_scan", true}};
    });
    server.handle("scan.settings", [&st](const json& p) -> Outcome {
        if (const auto b = opt_param<int>(p, "brightness")) st.set_brightness(*b);
        bool changed = false;
        if (const auto v = opt_param<int>(p, "exposure")) st.settings.exposure = *v, changed = true;
        if (const auto v = opt_param<int>(p, "gain")) st.settings.gain = *v, changed = true;
        if (const auto v = opt_param<int>(p, "laser_percent")) st.settings.laser_percent = std::clamp(*v, 0, 100), changed = true;
        if (const auto v = opt_param<int>(p, "strobe")) st.settings.strobe = *v, changed = true;
        if (changed) st.apply_settings();
        if (const auto v = opt_param<bool>(p, "record_raw_ir")) st.set_record_raw_ir(*v);
        return settings_json(st);
    });

    server.handle("markers.optimize", [&st](const json&) -> Outcome {
        if (st.scanning()) return refused("scanning: pause first");
        st.optimize_global_markers();
        return json{{"status", st.global_marker_status()}};
    });
    server.handle("markers.clear", [&st](const json&) -> Outcome {
        st.clear_global_markers();
        return json{{"status", st.global_marker_status()}};
    });
    server.handle("markers.save", [&st](const json& p) -> Outcome {
        if (!st.save_global_markers(param<std::string>(p, "path"))) return refused("could not save: " + st.global_marker_status());
        return json{{"saved", true}};
    });
    server.handle("markers.load", [&st](const json& p) -> Outcome {
        if (!st.load_global_markers(param<std::string>(p, "path"))) return refused("could not load: " + st.global_marker_status());
        return json{{"status", st.global_marker_status()}};
    });

    server.handle("process.run", [&st, &ui](const json& p) -> Outcome {
        if (st.scanning()) return refused("scanning: pause first");
        if (st.hud().recorded_frames == 0) return refused("nothing recorded yet");
        if (st.process_status().running) return refused("already processing");
        const auto res = opt_param<std::string>(p, "resolution_mm").value_or("0.5");
        if (res != "0.3" && res != "0.5" && res != "1.0") throw agent::Server::BadParams("resolution_mm is 0.3, 0.5 or 1.0");
        ui.voxel_choice = res == "0.3" ? 0 : res == "1.0" ? 2 : 1;
        ui.optimise_poses = opt_param<bool>(p, "optimize_poses").value_or(true);
        ui.smooth_iterations = std::clamp(opt_param<int>(p, "smoothing").value_or(0), 0, 10);
        ui.simplify_mesh = opt_param<bool>(p, "simplify").value_or(true);
        recon::ProcessParams pp;  // (as the Process button)
        pp.tsdf.voxel_mm = ui.voxel_choice == 0 ? 0.3f : ui.voxel_choice == 2 ? 1.0f : 0.5f;
        pp.tsdf.truncation_mm = 5.0f * pp.tsdf.voxel_mm;
        pp.optimize_poses = ui.optimise_poses;
        pp.smooth_iterations = ui.smooth_iterations;
        pp.simplify = ui.simplify_mesh;
        enter_step(st, ui, Step::process);
        st.process_scan(pp);
        return json{{"started", true}};
    });
    server.handle("process.status", [&st](const json&) -> Outcome { return process_json(st.process_status()); });
    server.handle("process.cancel", [&st](const json&) -> Outcome {
        st.cancel_processing();
        return process_json(st.process_status());
    });
    server.handle("process.export", [&st](const json& p) -> Outcome {
        if (!st.process_status().done) return refused("no processed mesh: process.run first");
        const auto path = param<std::string>(p, "path");
        if (!st.export_mesh(path)) return refused("could not export to " + path);
        return json{{"path", path}};
    });

    server.handle("view.get", [h](const json&) -> Outcome { return view_json(h); });
    server.handle("view.set", [h](const json& p) mutable -> Outcome {
        auto& c = h.camera;
        if (opt_param<bool>(p, "reset").value_or(false)) c.reset();
        const bool moved = p.contains("target") || p.contains("distance") || p.contains("orientation") || p.contains("orbit");
        if (moved) h.state.follow_scanner = false;
        if (const auto t = opt_param<std::vector<float>>(p, "target")) {
            if (t->size() != 3) throw agent::Server::BadParams("target is [x, y, z]");
            c.target = Eigen::Vector3f((*t)[0], (*t)[1], (*t)[2]);
        }
        if (const auto d = opt_param<float>(p, "distance")) c.distance = std::clamp(*d, 10.0f, 20000.0f);
        if (const auto q = opt_param<std::vector<float>>(p, "orientation")) {
            if (q->size() != 4) throw agent::Server::BadParams("orientation is [w, x, y, z]");
            c.orientation = render::Quatf((*q)[0], (*q)[1], (*q)[2], (*q)[3]).normalized();
        }
        if (const auto o = opt_param<std::vector<float>>(p, "orbit")) {
            if (o->size() != 2) throw agent::Server::BadParams("orbit is [dx, dy]");
            c.orbit((*o)[0], (*o)[1]);
        }
        if (const auto f = opt_param<bool>(p, "follow")) h.state.follow_scanner = *f;
        if (const auto v = opt_param<float>(p, "point_size_mm")) h.settings.point_size_mm = std::clamp(*v, 0.1f, 3.0f);
        if (const auto v = opt_param<bool>(p, "lighting")) h.settings.lighting = *v;
        if (const auto v = opt_param<bool>(p, "show_points")) h.settings.show_points = *v;
        if (const auto v = opt_param<bool>(p, "show_mesh")) h.settings.show_mesh = *v;
        if (const auto v = opt_param<bool>(p, "show_scanner")) h.settings.show_scanner = *v;
        return view_json(h);
    });

    server.handle("edit.select", [h, &st](const json& p) -> Outcome {
        if (!st.can_edit()) return refused("nothing to edit: a paused scan with a model is needed");
        std::vector<ImVec2> outline;
        for (const auto& q : param<std::vector<std::vector<float>>>(p, "polygon")) {
            if (q.size() != 2) throw agent::Server::BadParams("polygon points are [x, y]");
            outline.emplace_back(q[0], q[1]);
        }
        if (outline.size() < 3) throw agent::Server::BadParams("a polygon needs at least three points");
        const auto before = st.selection().strokes().size();
        st.add_lasso(lasso_stroke(h.camera, outline, opt_param<bool>(p, "subtract").value_or(false)));
        if (st.selection().strokes().size() == before) return refused("the polygon has no area on screen");
        return edit_json(st);
    });
    server.handle("edit.clear", [&st](const json&) -> Outcome {
        st.clear_selection();
        return edit_json(st);
    });
    // Delete and undo run on the scan pipeline: answer once it is done.
    auto when_edited = [&st]() -> agent::Server::Poll {
        return [&st]() -> std::optional<agent::Result> {
            if (st.edit_status().busy) return std::nullopt;
            return edit_json(st);
        };
    };
    server.handle("edit.delete", [&st, when_edited](const json&) -> Outcome {
        if (!st.can_edit()) return refused("nothing to edit: a paused scan with a model is needed");
        if (st.selection().empty()) return refused("nothing selected: edit.select first");
        st.delete_selection();
        return when_edited();
    });
    server.handle("edit.undo", [&st, when_edited](const json&) -> Outcome {
        const auto e = st.edit_status();
        if (st.scanning() || e.undo_depth == 0) return refused("nothing to undo (deletes are final once scanning resumes)");
        st.undo_delete();
        return when_edited();
    });
    server.handle("edit.status", [&st](const json&) -> Outcome { return edit_json(st); });
}

}  // namespace einstar::app
