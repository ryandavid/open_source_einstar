#pragma once

// The board-centred 3D guide: the calibration board at the origin, the capture plan as five lines from its
// centre with a dot per distance (grey: to do, pulsing: next, amber: in position -- hold still, green:
// captured), a ghost of the scanner at every captured view, and the scanner's live pose with its aim line
// and field of view on the board. Drawn with ImGui's draw list (wireframe, painter's order); drag to orbit,
// scroll to zoom, double-click to reset.

#include <optional>
#include <vector>

#include "imgui.h"

#include "einstar/calibrate/plan.hpp"

namespace einstar::app {

struct BoardViewInput {
    const std::vector<calibrate::PoseTarget>* plan = nullptr;
    std::vector<bool> captured;               // per plan index
    std::vector<std::optional<SE3>> ghosts;   // the scanner in board coordinates at each captured view
    int target = -1;                          // the plan index being guided to
    std::optional<SE3> scanner;               // the scanner in board coordinates now (nullopt: board not seen)
    bool in_position = false;                 // all of distance, tilt, centring and roll within tolerance
    bool aim_ok = false;                      // aimed at the board centre (offset within tolerance)
    double steady_frac = 0;                   // 0..1 of the hold-still time, while in position
};

class BoardView {
public:
    // Draws at the window's cursor, filling `size` pixels.
    void draw(const BoardViewInput& in, ImVec2 size);
    void reset_view() { yaw_ = pitch_ = 0, zoom_ = 1; }

private:
    double yaw_ = 0, pitch_ = 0, zoom_ = 1;
    std::optional<SE3> last_scanner_;  // shown greyed while the board is out of view
};

}  // namespace einstar::app
