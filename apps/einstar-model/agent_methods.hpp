#pragma once

// The modelling app's agent methods (libs/agent): every model.* document command, and model.view.

#include "einstar/agent/server.hpp"
#include "einstar/render/view_camera.hpp"
#include "model_app.hpp"

namespace einstar::modelapp {

void register_model_agent(agent::Server& server, ModelApp& app, render::ViewCamera& camera);

// Whether an app is listening on this socket (connecting succeeds).
[[nodiscard]] bool agent_socket_in_use(const std::string& path);

}  // namespace einstar::modelapp
