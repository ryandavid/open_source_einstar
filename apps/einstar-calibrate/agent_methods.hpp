#pragma once

// The calibration app's agent methods (agent/protocol.hpp: calib.*). They drive the app through the same
// calls its UI makes.

#include <string>

#include "calibration_controller.hpp"
#include "einstar/agent/server.hpp"

namespace einstar::app {

struct CalibrationAgentHost {
    CalibrationController& ctl;
    int& tab_request;    // 0 live, 1 captures, 2 results (the main view's tabs)
    std::string& error;  // the error line under the side panel
};

void register_calibration_agent(agent::Server& server, CalibrationAgentHost host);

}  // namespace einstar::app
