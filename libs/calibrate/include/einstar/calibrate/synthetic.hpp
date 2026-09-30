#pragma once

// The calibration board rendered through a rig (einstar::synth): the device emulator's scene in
// einstar-calibrate, and ground truth for end-to-end tests of detection and solving.

#include <cstdint>
#include <utility>

#include "einstar/calibrate/board.hpp"

namespace einstar::calibrate {

struct SyntheticBoardParams {
    int supersample = 2;
    double board_grey = 0.35;  // board reflectance under the ring light (dots saturate)
    double noise_sigma = 2.0;
    std::uint32_t seed = 1;
};

// Left and right raw IR images of the board at T_left_board.
[[nodiscard]] std::pair<ImageU8, ImageU8> render_board_pair(const RigCalibration& rig, const SE3& T_left_board, const BoardSpec& board = {},
                                                           const SyntheticBoardParams& params = {});

}  // namespace einstar::calibrate
