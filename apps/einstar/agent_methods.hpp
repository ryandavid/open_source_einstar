#pragma once

// The scanning app's agent methods (agent/protocol.hpp: scan.*, device.*, workflow.*, markers.*, process.*,
// view.*, edit.*). They drive the app through the same calls its UI makes.

#include <vector>

#include <imgui.h>

#include "app_state.hpp"
#include "einstar/agent/server.hpp"
#include "einstar/core/lasso.hpp"
#include "einstar/render/scene_renderer.hpp"
#include "workflow.hpp"

namespace einstar::app {

struct ScanAgentHost {
    AppState& state;
    WorkflowUi& workflow;
    render::ViewCamera& camera;
    render::RenderSettings& settings;
};

void register_scan_agent(agent::Server& server, ScanAgentHost host);

// A lasso outline in window points, as seen through `camera` (the 3D view fills the window), as a stroke.
[[nodiscard]] LassoStroke lasso_stroke(const render::ViewCamera& camera, const std::vector<ImVec2>& outline, bool subtract);

}  // namespace einstar::app
