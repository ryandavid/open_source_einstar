#include "agent_methods.hpp"

#include <algorithm>
#include <chrono>
#include <format>

namespace einstar::app {
namespace {

using agent::ErrorCode;
using agent::error;
using agent::json;
using agent::opt_param;
using agent::param;
using Outcome = agent::Server::Outcome;

constexpr const char* kGroups[] = {"face-on", "top edge near", "bottom edge near", "right edge near", "left edge near"};

agent::Error refused(std::string why) { return error(ErrorCode::refused, std::move(why)); }

json measure_json(const calibrate::BoardMeasure& m) {
    // Roll as the view shows it: 0 with the board's long side across the view (the image's 90).
    return {{"distance_mm", m.distance_mm},
            {"offset_mm", {m.offset_mm.x(), m.offset_mm.y()}},
            {"tilt_x_deg", m.tilt_x_deg},
            {"tilt_y_deg", m.tilt_y_deg},
            {"roll_deg", std::remainder(m.roll_deg - 90.0, 360.0)}};
}

json report_json(const calibrate::CalibrationReport& r) {
    return {{"rms_px", r.rms_px}, {"row_rms_px", r.row_rms_px}, {"max_row_px", r.max_row_px}, {"dots", r.dots}, {"dropped", r.dropped},
            {"views", r.views.size()}, {"baseline_mm", r.rig.T_right_left.translation().norm()}};
}

json state_json(const CalibrationController& ctl) {
    const auto& plan = ctl.plan();
    const auto caps = ctl.captures();
    json targets = json::array();
    for (std::size_t i = 0; i < plan.size(); ++i)
        targets.push_back({{"index", i},
                           {"group", plan[i].group},
                           {"label", plan[i].label},
                           {"distance_mm", plan[i].distance_mm},
                           {"tilt_x_deg", plan[i].tilt_x_deg},
                           {"tilt_y_deg", plan[i].tilt_y_deg},
                           {"captured", i < caps.size() && caps[i].has_value()}});
    const auto live = ctl.live();
    json live_j = {{"target", live.target},
                   {"target_label", live.target >= 0 && static_cast<std::size_t>(live.target) < plan.size() ? json(plan[static_cast<std::size_t>(live.target)].label) : json(nullptr)},
                   {"dots_left", live.det_left ? live.det_left->pixels.size() : 0},
                   {"dots_right", live.det_right ? live.det_right->pixels.size() : 0},
                   {"common_dots", live.common_dots},
                   {"steady_s", live.steady_s},
                   {"steady_needed_s", live.steady_needed_s},
                   {"fps", live.fps},
                   {"mean_level", live.mean_level},
                   {"saturated_percent", live.saturated_permille / 10.0}};
    if (live.measure) live_j["measure"] = measure_json(*live.measure);
    if (live.guidance)
        live_j["guidance"] = {{"in_position", live.guidance->ok()},
                              {"distance_ok", live.guidance->distance_ok},
                              {"tilt_ok", live.guidance->tilt_ok},
                              {"offset_ok", live.guidance->offset_ok},
                              {"roll_ok", live.guidance->roll_ok},
                              {"hints", live.guidance->hints}};
    const auto st = ctl.solve_state();
    json solve = {{"running", st.running}, {"error", st.error}, {"saved_path", st.saved_path}, {"solve_ms", st.solve_ms}};
    if (st.ours) solve["ours"] = report_json(*st.ours);
    if (st.flash) solve["flash_on_these_captures"] = report_json(*st.flash);
    if (st.factory) solve["factory_on_these_captures"] = report_json(*st.factory);
    const int g = ctl.active_group();
    return {{"connected", ctl.connected()},
            {"emulated", ctl.emulated()},
            {"device", ctl.description()},
            {"status", ctl.status()},
            {"flash_calibration_time", ctl.flash_time()},
            {"session_dir", ctl.session_dir().string()},
            {"captured", ctl.captured_count()},
            {"active_group", g},
            {"active_group_name", g >= 0 && g < 5 ? kGroups[g] : "?"},
            {"auto_capture", ctl.auto_capture.load()},
            {"keep_factory_distortion", ctl.keep_factory_distortion},
            {"lighting", {{"exposure", ctl.lighting.exposure}, {"gain", ctl.lighting.gain}, {"ring_light", ctl.lighting.ring_light}, {"white_leds", ctl.lighting.white_leds}}},
            {"plan", targets},
            {"live", live_j},
            {"solve", solve}};
}

json write_plan_json(const CalibrationController& ctl, const CalibrationController::WritePlan& p) {
    json gates = json::array();
    for (const auto& gt : p.gates) gates.push_back({{"what", gt.what}, {"ok", gt.ok}});
    json j = {{"ready", p.ready()}, {"error", p.error}, {"gates", gates}, {"reference_view", p.reference_view},
              {"backup_path", p.backup_path.string()}, {"current", ctl.flash_time()}, {"emulated", ctl.emulated()}};
    if (p.update) {
        j["pages"] = p.update->pages;
        j["calibration_time"] = p.update->calibration_time;
    }
    return j;
}

}  // namespace

void register_calibration_agent(agent::Server& server, CalibrationAgentHost h) {
    auto& ctl = h.ctl;
    server.handle("calib.state", [&ctl](const json&) -> Outcome { return state_json(ctl); });
    server.handle("calib.connect", [&ctl](const json& p) -> Outcome {
        if (auto r = ctl.connect(param<bool>(p, "emulator")); !r) return refused("could not connect: " + r.error().message);
        return json{{"connected", true}, {"device", ctl.description()}, {"emulated", ctl.emulated()}};
    });
    server.handle("calib.disconnect", [&ctl](const json&) -> Outcome {
        ctl.disconnect();
        return json{{"connected", false}};
    });
    server.handle("calib.group", [&ctl](const json& p) -> Outcome {
        const int g = param<int>(p, "group");
        if (g < 0 || g > 4) throw agent::Server::BadParams("group is 0-4");
        ctl.set_active_group(g);
        return json{{"active_group", g}, {"name", kGroups[g]}};
    });
    server.handle("calib.auto_capture", [&ctl](const json& p) -> Outcome {
        ctl.auto_capture = param<bool>(p, "on");
        return json{{"auto_capture", ctl.auto_capture.load()}};
    });
    server.handle("calib.capture", [&ctl](const json&) -> Outcome {
        if (!ctl.connected()) return refused("not connected: calib.connect first");
        const int before = ctl.captured_count();
        ctl.capture_now();
        // Answers once the capture has been taken (or after 5 s, with why not).
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        return agent::Server::Poll([&ctl, before, deadline]() -> std::optional<agent::Result> {
            if (ctl.captured_count() > before) return json{{"captured", true}, {"count", ctl.captured_count()}};
            if (std::chrono::steady_clock::now() < deadline) return std::nullopt;
            return refused("no capture within 5 s: " + ctl.status());
        });
    });
    server.handle("calib.clear", [&ctl](const json& p) -> Outcome {
        if (const auto i = opt_param<int>(p, "index")) {
            if (*i < 0 || static_cast<std::size_t>(*i) >= ctl.plan().size()) throw agent::Server::BadParams("index is a plan index");
            ctl.clear_capture(*i);
        } else {
            ctl.clear_all();
        }
        return json{{"captured", ctl.captured_count()}};
    });
    server.handle("calib.load_folder", [h](const json& p) -> Outcome {
        if (auto r = h.ctl.load_folder(param<std::string>(p, "path")); !r) return refused(r.error().message);
        h.tab_request = 1;
        return json{{"captured", h.ctl.captured_count()}};
    });
    server.handle("calib.load_reference", [&ctl](const json& p) -> Outcome {
        if (auto r = ctl.load_reference(param<std::string>(p, "path")); !r) return refused(r.error().message);
        return json{{"loaded", true}};
    });
    server.handle("calib.solve", [h](const json& p) -> Outcome {
        if (h.ctl.solve_state().running) return refused("already solving");
        if (h.ctl.captured_count() < 4) return refused(std::format("{} captures: the solve needs at least 4", h.ctl.captured_count()));
        if (const auto k = opt_param<bool>(p, "keep_factory_distortion")) h.ctl.keep_factory_distortion = *k;
        h.ctl.solve();
        h.tab_request = 2;
        return json{{"started", true}};
    });
    server.handle("calib.save", [&ctl](const json&) -> Outcome {
        auto r = ctl.save_result();
        if (!r) return refused(r.error().message);
        return json{{"path", r->string()}};
    });
    server.handle("calib.plan_write", [&ctl](const json&) -> Outcome { return write_plan_json(ctl, ctl.plan_write()); });
    server.handle("calib.write", [&ctl](const json& p) -> Outcome {
        if (!param<bool>(p, "confirm")) throw agent::Server::BadParams("confirm must be true");
        if (!ctl.connected()) return refused("not connected");
        const auto plan = ctl.plan_write();
        if (!plan.ready()) return error(ErrorCode::refused, plan.error.empty() ? "a gate fails" : plan.error, write_plan_json(ctl, plan));
        auto r = ctl.write_to_scanner(plan);  // (the same plan and gates as the dialog)
        if (!r) return refused(r.error().message);
        return json{{"result", *r}, {"backup_path", plan.backup_path.string()}};
    });
    server.handle("calib.restore", [&ctl](const json& p) -> Outcome {
        if (!param<bool>(p, "confirm")) throw agent::Server::BadParams("confirm must be true");
        if (!ctl.connected()) return refused("not connected");
        auto r = ctl.restore_backup(param<std::string>(p, "path"));
        if (!r) return refused(r.error().message);
        return json{{"result", *r}};
    });
    server.handle("calib.lighting", [&ctl](const json& p) -> Outcome {
        bool changed = false;
        if (const auto v = opt_param<int>(p, "exposure")) ctl.lighting.exposure = std::clamp(*v, 300, 8000), changed = true;
        if (const auto v = opt_param<int>(p, "gain")) ctl.lighting.gain = std::clamp(*v, 16, 800), changed = true;
        if (const auto v = opt_param<int>(p, "ring_light")) ctl.lighting.ring_light = std::clamp(*v, 0, 9000), changed = true;
        if (const auto v = opt_param<int>(p, "white_leds")) ctl.lighting.white_leds = std::clamp(*v, 0, 9000), changed = true;
        if (changed && ctl.connected())
            if (auto r = ctl.apply_lighting(); !r) return refused(r.error().message);
        return json{{"exposure", ctl.lighting.exposure}, {"gain", ctl.lighting.gain}, {"ring_light", ctl.lighting.ring_light}, {"white_leds", ctl.lighting.white_leds}};
    });
    server.handle("calib.tab", [h](const json& p) -> Outcome {
        const auto t = param<std::string>(p, "tab");
        if (t == "live") h.tab_request = 0;
        else if (t == "captures") h.tab_request = 1;
        else if (t == "results") h.tab_request = 2;
        else throw agent::Server::BadParams("tab is live, captures or results");
        return json{{"tab", t}};
    });
}

}  // namespace einstar::app
