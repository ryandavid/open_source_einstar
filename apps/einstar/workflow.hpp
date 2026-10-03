#pragma once

// The app's guided workflow: connect the scanner, choose a scan type, capture global markers if that
// type needs them, scan, then process and export. One step is open at a time; earlier steps can be
// revisited while not scanning. A banner over the 3D view always says what the scanner is doing.

#include <array>
#include <chrono>

#include "app_state.hpp"
#include "einstar/render/scene_renderer.hpp"

namespace einstar::app {

struct WorkflowUi {
    enum class Step { connect, scan_type, markers, scan, process };
    enum class ScanType { surface, hybrid, global_markers };
    Step step = Step::connect;
    ScanType type = ScanType::hybrid;

    // Process options.
    int voxel_choice = 1;  // 0.3 / 0.5 / 1.0 mm
    bool optimise_poses = true;
    int smooth_iterations = 0;
    bool simplify_mesh = true;
    std::array<char, 512> export_path{};
    std::array<char, 512> marker_path{};

    // Scanning time of the current scan (shown in the banner).
    std::chrono::steady_clock::duration scanned{};
    std::chrono::steady_clock::time_point scan_started{};
    bool was_scanning = false;
    bool confirm_discard = false;

    WorkflowUi();
};

// Opens a step (as clicking its header does): the scanner's phase and alignment follow it.
void enter_step(AppState& state, WorkflowUi& ui, WorkflowUi::Step step);
// The workflow panel (left). Also tells `state` whether the scanner's start / pause button may start
// or pause the scanner in the current step.
void draw_workflow(AppState& state, WorkflowUi& ui, render::RenderSettings& settings);
// The status banner over the 3D view (top centre).
void draw_status_banner(const AppState& state, const WorkflowUi& ui);

}  // namespace einstar::app
