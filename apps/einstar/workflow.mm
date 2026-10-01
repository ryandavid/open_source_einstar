#include "workflow.hpp"

#import <AppKit/AppKit.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <string>

#include "imgui.h"

namespace einstar::app {
namespace {

using Step = WorkflowUi::Step;
using ScanType = WorkflowUi::ScanType;

const ImVec4 kGreen(0.35f, 0.85f, 0.45f, 1.0f);
const ImVec4 kAmber(1.0f, 0.75f, 0.2f, 1.0f);
const ImVec4 kRed(0.95f, 0.35f, 0.3f, 1.0f);
const ImVec4 kGrey(0.6f, 0.6f, 0.6f, 1.0f);
const ImVec4 kBlue(0.45f, 0.7f, 1.0f, 1.0f);
const ImVec4 kPurple(0.75f, 0.55f, 1.0f, 1.0f);

std::string choose_file(NSString* message) {
    NSOpenPanel* panel = [NSOpenPanel openPanel];
    panel.canChooseDirectories = NO;
    panel.canChooseFiles = YES;
    panel.allowsMultipleSelection = NO;
    panel.message = message;
    if (const char* home = std::getenv("HOME"))
        panel.directoryURL = [NSURL fileURLWithPath:[NSString stringWithUTF8String:(std::string(home) + "/Documents/Einstar/Scans").c_str()]];
    if ([panel runModal] == NSModalResponseOK) return panel.URL.path.UTF8String;
    return {};
}

std::string format_duration(std::chrono::steady_clock::duration d) {
    const auto s = std::chrono::duration_cast<std::chrono::seconds>(d).count();
    return std::format("{:02}:{:02}", s / 60, s % 60);
}

const char* step_title(Step s) {
    switch (s) {
        case Step::connect: return "Scanner";
        case Step::scan_type: return "Scan type";
        case Step::markers: return "Global markers";
        case Step::scan: return "Scan";
        case Step::process: return "Process & export";
    }
    return "";
}

// A full-width button in a colour (the step's main action).
bool big_button(const char* label, const ImVec4& colour) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(colour.x * 0.55f, colour.y * 0.55f, colour.z * 0.55f, 1));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(colour.x * 0.7f, colour.y * 0.7f, colour.z * 0.7f, 1));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, colour);
    const bool r = ImGui::Button(label, ImVec2(-1, 40));
    ImGui::PopStyleColor(3);
    return r;
}

// A radio choice with a one-line explanation under it.
bool choice(const char* label, const char* help, bool selected) {
    const bool clicked = ImGui::RadioButton(label, selected);
    ImGui::Indent(26);
    ImGui::PushStyleColor(ImGuiCol_Text, kGrey);
    ImGui::TextWrapped("%s", help);
    ImGui::PopStyleColor();
    ImGui::Unindent(26);
    ImGui::Spacing();
    return clicked;
}

void enter_step(AppState& state, WorkflowUi& ui, Step step) {
    ui.step = step;
    ui.confirm_discard = false;
    // The scanner's phase follows the step.
    if (step == Step::markers) state.set_phase(pipeline::ScanPhase::global_markers);
    if (step == Step::scan) {
        state.set_phase(pipeline::ScanPhase::surface);
        state.set_align_mode(ui.type == ScanType::surface ? track::AlignMode::geometry : track::AlignMode::hybrid);
    }
}

// ---- the steps ----

void connect_step(AppState& state, WorkflowUi& ui) {
    const auto c = state.connection();
    if (c.kind == Connection::Kind::none) {
        ImGui::TextWrapped("Plug the Einstar in (USB) and close EXStar, then connect.");
        if (big_button("Connect scanner", kGreen) && state.connect(false)) enter_step(state, ui, Step::scan_type);
        if (ImGui::Button("Use the emulator instead", ImVec2(-1, 0)) && state.connect(true)) enter_step(state, ui, Step::scan_type);
        if (!c.error.empty()) ImGui::TextColored(kRed, "%s", c.error.c_str());
        return;
    }
    ImGui::TextColored(c.kind == Connection::Kind::emulator ? kBlue : c.online ? kGreen : kAmber, "%s",
                       c.kind == Connection::Kind::emulator ? "Emulator (no scanner)" : c.online ? "Scanner connected" : "Scanner offline: reconnecting...");
    ImGui::TextWrapped("%s", c.device.c_str());
    ImGui::TextWrapped("Calibration: %s", c.calibration.c_str());
    if (ImGui::Button("Continue", ImVec2(-1, 0))) enter_step(state, ui, Step::scan_type);
    if (ImGui::Button("Disconnect", ImVec2(-1, 0))) {
        state.disconnect();
        ui.step = Step::connect;
    }
}

void scan_type_step(AppState& state, WorkflowUi& ui) {
    const auto hud = state.hud();
    if (choice("Surface + markers", "Tracks on the shape and on any marker stickers in view. The default.", ui.type == ScanType::hybrid))
        ui.type = ScanType::hybrid;
    if (choice("Surface only", "Tracks on the shape alone (ignores markers). For objects with no stickers.", ui.type == ScanType::surface))
        ui.type = ScanType::surface;
    if (choice("Global markers first",
               "First sweep over all the marker stickers (nothing else is recorded), then scan the surface locked to that marker "
               "map. Best for large objects and to avoid drift.",
               ui.type == ScanType::global_markers))
        ui.type = ScanType::global_markers;
    ImGui::Separator();
    if (hud.global_markers > 0) {
        ImGui::TextColored(kPurple, "A global marker map is in use (%d markers)", hud.global_markers);
        ImGui::SameLine();
        if (ImGui::SmallButton("Discard")) state.clear_global_markers();
    }
    if (ImGui::Button("Use global markers from an earlier scan...", ImVec2(-1, 0))) {
        const auto path = choose_file(@"A scan (.estr) whose global markers to reuse, or a markers file");
        if (!path.empty() && state.load_global_markers(path)) {
            ui.type = ScanType::hybrid;
            enter_step(state, ui, Step::scan);
        }
    }
    if (const auto st = state.global_marker_status(); !st.empty()) ImGui::TextWrapped("%s", st.c_str());
    ImGui::Spacing();
    if (big_button(ui.type == ScanType::global_markers ? "Next: capture the markers" : "Next: scan", kGreen))
        enter_step(state, ui, ui.type == ScanType::global_markers ? Step::markers : Step::scan);
}

void markers_step(AppState& state, WorkflowUi& ui) {
    const auto hud = state.hud();
    const bool scanning = state.scanning();
    ImGui::TextWrapped("Sweep slowly over every marker sticker, from a few directions. Only markers are recorded in this step.");
    ImGui::Text("Keyframes %d, markers %d", hud.keyframes, hud.map_markers);
    if (!scanning) {
        if (big_button(hud.keyframes > 0 ? "Resume marker capture" : "Start marker capture", kPurple)) state.start_scan();
    } else if (big_button("Pause", kAmber)) {
        state.stop_scan();
    }
    ImGui::TextDisabled("(or the scanner's start / pause button)");
    ImGui::Separator();
    const bool optimised = hud.global_markers > 0;
    ImGui::BeginDisabled(scanning || hud.keyframes < 3);
    if (ImGui::Button(optimised ? "Optimise again" : "Optimise the marker map", ImVec2(-1, 0))) state.optimize_global_markers();
    ImGui::EndDisabled();
    if (scanning) ImGui::TextDisabled("Pause to optimise.");
    if (const auto st = state.global_marker_status(); !st.empty()) ImGui::TextWrapped("%s", st.c_str());
    if (optimised) {
        ImGui::TextColored(kGreen, "%d global markers ready", hud.global_markers);
        ImGui::BeginDisabled(scanning);
        if (big_button("Next: scan the surface", kGreen)) enter_step(state, ui, Step::scan);
        ImGui::EndDisabled();
    }
    if (ImGui::CollapsingHeader("Save / load the marker map")) {
        ImGui::InputText("##markers", ui.marker_path.data(), ui.marker_path.size());
        if (ImGui::Button("Save")) (void)state.save_global_markers(ui.marker_path.data());
        ImGui::SameLine();
        if (ImGui::Button("Load")) (void)state.load_global_markers(ui.marker_path.data());
        ImGui::TextDisabled("The map is also stored in every scan recorded with it.");
    }
    ImGui::BeginDisabled(scanning);
    if (ImGui::Button("Discard and capture again", ImVec2(-1, 0))) state.clear_global_markers();
    ImGui::EndDisabled();
}

void scan_step(AppState& state, WorkflowUi& ui) {
    const auto hud = state.hud();
    const bool scanning = state.scanning();
    ImGui::Text("Mode: %s%s", ui.type == ScanType::surface ? "surface only" : "surface + markers",
                hud.global_markers > 0 ? std::format(", locked to {} global markers", hud.global_markers).c_str() : "");
    if (!scanning) {
        if (big_button(hud.frames > 0 ? "Resume scanning" : "Start scanning", kRed)) state.start_scan();
    } else if (big_button("Pause", kAmber)) {
        state.stop_scan();
    }
    ImGui::TextDisabled("(or the scanner's start / pause button)");
    const auto path = state.recording_path();
    if (!path.empty())
        ImGui::TextWrapped("Recording %s (%llu frames)", std::filesystem::path(path).filename().c_str(),
                           static_cast<unsigned long long>(hud.recorded_frames));
    ImGui::Separator();
    ImGui::BeginDisabled(scanning || hud.recorded_frames == 0);
    if (big_button("Finish: process the scan", kGreen)) enter_step(state, ui, Step::process);
    ImGui::EndDisabled();
    ImGui::BeginDisabled(scanning);
    if (!ui.confirm_discard) {
        if (ImGui::Button("Discard this scan and start over", ImVec2(-1, 0))) ui.confirm_discard = true;
    } else {
        ImGui::TextColored(kRed, "Delete the recording and clear the model?");
        if (ImGui::Button("Delete")) {
            state.new_scan(true);
            ui.scanned = {};
            ui.confirm_discard = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Keep")) ui.confirm_discard = false;
    }
    ImGui::EndDisabled();
}

void process_step(AppState& state, WorkflowUi& ui, render::RenderSettings& settings) {
    const auto ps = state.process_status();
    ImGui::BeginDisabled(ps.running);
    ImGui::Combo("Resolution", &ui.voxel_choice, "0.3 mm (fine)\0" "0.5 mm\0" "1.0 mm (fast)\0");
    ImGui::Checkbox("Optimise poses (loop closure)", &ui.optimise_poses);
    ImGui::SliderInt("Smoothing", &ui.smooth_iterations, 0, 10);
    ImGui::Checkbox("Simplify (within 0.02 mm)", &ui.simplify_mesh);
    if (big_button(ps.done ? "Process again" : "Process", kGreen)) {
        recon::ProcessParams pp;
        pp.tsdf.voxel_mm = ui.voxel_choice == 0 ? 0.3f : ui.voxel_choice == 2 ? 1.0f : 0.5f;
        pp.tsdf.truncation_mm = 5.0f * pp.tsdf.voxel_mm;
        pp.optimize_poses = ui.optimise_poses;
        pp.smooth_iterations = ui.smooth_iterations;
        pp.simplify = ui.simplify_mesh;
        state.process_scan(pp);
    }
    ImGui::EndDisabled();
    if (ps.running) {
        ImGui::ProgressBar(static_cast<float>(ps.fraction), ImVec2(-1, 0), ps.stage.c_str());
        if (ImGui::Button("Cancel")) state.cancel_processing();
    }
    if (!ps.summary.empty()) ImGui::TextWrapped("%s", ps.summary.c_str());
    if (ps.done) {
        ImGui::Checkbox("Show mesh", &settings.show_mesh);
        ImGui::SameLine();
        ImGui::Checkbox("Show points", &settings.show_points);
        ImGui::InputText("##export", ui.export_path.data(), ui.export_path.size());
        if (ImGui::Button("Export (.stl / .ply / .obj)", ImVec2(-1, 0))) (void)state.export_mesh(ui.export_path.data());
    }
    ImGui::Separator();
    ImGui::BeginDisabled(ps.running);
    if (ImGui::Button("Back: scan more of this object", ImVec2(-1, 0))) enter_step(state, ui, Step::scan);
    if (ImGui::Button("New scan (this one is kept)", ImVec2(-1, 0))) {
        state.new_scan(false);
        ui.scanned = {};
        enter_step(state, ui, Step::scan_type);
    }
    ImGui::EndDisabled();
}

}  // namespace

WorkflowUi::WorkflowUi() {
    if (const char* home = std::getenv("HOME")) {
        std::snprintf(export_path.data(), export_path.size(), "%s/Documents/Einstar/scan.stl", home);
        std::snprintf(marker_path.data(), marker_path.size(), "%s/Documents/Einstar/global_markers.txt", home);
    }
}

void draw_workflow(AppState& state, WorkflowUi& ui, render::RenderSettings& settings) {
    if (!state.connected()) ui.step = Step::connect;
    const bool scanning = state.scanning();
    // Scanning time of the current scan.
    const auto now = std::chrono::steady_clock::now();
    if (scanning && !ui.was_scanning) ui.scan_started = now;
    if (!scanning && ui.was_scanning) ui.scanned += now - ui.scan_started;
    ui.was_scanning = scanning;
    // The scanner's button only starts / pauses where that is the next action.
    state.scanner_button_enabled = ui.step == Step::markers || ui.step == Step::scan;

    const std::array<Step, 5> steps{Step::connect, Step::scan_type, Step::markers, Step::scan, Step::process};
    int number = 0;
    for (const Step s : steps) {
        if (s == Step::markers && ui.type != ScanType::global_markers) continue;
        ++number;
        const bool current = s == ui.step;
        const bool done = static_cast<int>(s) < static_cast<int>(ui.step);
        // Earlier steps can be reopened while the scanner is idle; later ones open through the flow.
        const bool reachable = done && !scanning && (s != Step::connect || true);
        ImGui::PushStyleColor(ImGuiCol_Text, current ? ImVec4(1, 1, 1, 1) : done ? kGreen : kGrey);
        const std::string label = std::format("{} {}  {}", done ? "v" : current ? ">" : " ", number, step_title(s));
        ImGui::SetNextItemOpen(current, ImGuiCond_Always);
        const bool open = ImGui::CollapsingHeader(label.c_str(), current || reachable ? 0 : ImGuiTreeNodeFlags_Leaf);
        ImGui::PopStyleColor();
        if (!current && open && reachable) enter_step(state, ui, s);
        if (!current) continue;
        ImGui::Indent(8);
        switch (s) {
            case Step::connect: connect_step(state, ui); break;
            case Step::scan_type: scan_type_step(state, ui); break;
            case Step::markers: markers_step(state, ui); break;
            case Step::scan: scan_step(state, ui); break;
            case Step::process: process_step(state, ui, settings); break;
        }
        ImGui::Unindent(8);
        ImGui::Spacing();
    }
}

void draw_status_banner(const AppState& state, const WorkflowUi& ui) {
    const auto c = state.connection();
    const auto hud = state.hud();
    const auto ps = state.process_status();
    const bool scanning = state.scanning();
    std::string text, detail;
    ImVec4 colour = kGrey;
    if (c.kind == Connection::Kind::none) {
        text = "NOT CONNECTED";
        detail = "Connect the scanner (step 1)";
    } else if (c.kind == Connection::Kind::scanner && !c.online) {
        text = "SCANNER OFFLINE";
        detail = "Reconnecting...";
        colour = kAmber;
    } else if (ps.running) {
        text = std::format("PROCESSING {:.0f}%", 100 * ps.fraction);
        detail = ps.stage;
        colour = kBlue;
    } else if (scanning && hud.phase == pipeline::ScanPhase::global_markers) {
        text = "CAPTURING MARKERS";
        detail = std::format("{} keyframes, {} markers; no surface is recorded", hud.keyframes, hud.map_markers);
        colour = kPurple;
    } else if (scanning) {
        const auto elapsed = ui.scanned + (std::chrono::steady_clock::now() - ui.scan_started);
        text = hud.tracking_lost ? "SCANNING: TRACKING LOST" : "SCANNING";
        detail = hud.tracking_lost ? (hud.reason.empty() ? "Move back to the grey frustum" : hud.reason)
                                   : std::format("{}  {} frames recorded", format_duration(elapsed), hud.recorded_frames);
        colour = hud.tracking_lost ? kAmber : kRed;
    } else if (ui.step == Step::markers) {
        text = "MARKER CAPTURE PAUSED";
        detail = hud.global_markers > 0 ? std::format("{} global markers ready", hud.global_markers) : "Start the capture or optimise";
        colour = kPurple;
    } else if (ui.step == Step::scan) {
        text = hud.recorded_frames > 0 ? "PAUSED" : "READY TO SCAN";
        detail = hud.recorded_frames > 0 ? std::format("{}  {} frames recorded", format_duration(ui.scanned), hud.recorded_frames)
                                         : "Press Start or the scanner's button";
        colour = hud.recorded_frames > 0 ? kAmber : kGreen;
    } else if (ui.step == Step::process) {
        text = ps.done ? "PROCESSED" : "READY TO PROCESS";
        detail = ps.done ? "Export the mesh, scan more, or start a new scan" : "Choose the resolution and press Process";
        colour = ps.done ? kGreen : kBlue;
    } else {
        text = c.kind == Connection::Kind::emulator ? "EMULATOR CONNECTED" : "SCANNER CONNECTED";
        detail = std::format("Step: {}", step_title(ui.step));
        colour = c.kind == Connection::Kind::emulator ? kBlue : kGreen;
    }
    const ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, 10), ImGuiCond_Always, ImVec2(0.5f, 0));
    ImGui::SetNextWindowBgAlpha(0.75f);
    ImGui::Begin("##banner", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoFocusOnAppearing |
                     ImGuiWindowFlags_NoNav);
    ImGui::SetWindowFontScale(1.6f);
    if (scanning) {
        // A blinking recording dot.
        const float h = ImGui::GetTextLineHeight();
        const ImVec2 p = ImGui::GetCursorScreenPos();
        if (std::fmod(ImGui::GetTime(), 1.0) < 0.6)
            ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(p.x + h * 0.5f, p.y + h * 0.5f), h * 0.35f, ImGui::GetColorU32(colour));
        ImGui::Dummy(ImVec2(h, h));
        ImGui::SameLine();
    }
    ImGui::TextColored(colour, "%s", text.c_str());
    ImGui::SetWindowFontScale(1.0f);
    ImGui::TextUnformatted(detail.c_str());
    if (c.kind != Connection::Kind::none)
        ImGui::TextDisabled("%s", c.kind == Connection::Kind::emulator ? "Emulator (no scanner attached)" : c.device.c_str());
    ImGui::End();
}

}  // namespace einstar::app
