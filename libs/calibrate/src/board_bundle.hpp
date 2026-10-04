#pragma once

// The final stage of solve_stereo: the stereo bundle adjustment with the board's dots free.
//
// optim::refine_stereo_calibration holds the board at its nominal grid. A real board is not that grid to
// a few hundredths of a millimetre (print scale, a slight bow, dot centres off their nominal spots), and
// with the board held as exact the solve fits those errors with the cameras: mostly the distortion, which
// then bends the rows where no dot was (the image's corners). Here every dot is a 3D point of its own,
// seen by both cameras in every view, so the two cameras are fitted to each other through the dots
// themselves: the board's shape is measured along the way, and its nominal grid only fixes the scale
// (and is a weak prior on each dot).

#include <array>
#include <vector>

#include "einstar/core/camera.hpp"
#include "einstar/optim/stereo_calibration.hpp"

namespace einstar::calibrate::detail {

struct BoardBundleOptions {
    bool free_distortion = true;
    double huber_px = 1.0;
    double board_sigma_mm = 0.5;   // prior on each dot's offset from its nominal spot
    int max_iterations = 200;
};

struct BoardBundleResult {
    RigCalibration rig;
    std::vector<SE3> T_left_board;
    std::vector<Vec3> board;       // the fitted dots, in the order of `nominal`
    double rms = 0;
};

// `ids[v][k]`: index into `nominal` of view v's k-th dot (views[v].board[k] is its nominal position).
[[nodiscard]] BoardBundleResult refine_with_board(const RigCalibration& initial, const std::vector<optim::BoardView>& views,
                                                  const std::vector<std::vector<int>>& ids, const std::vector<Vec3>& nominal,
                                                  const BoardBundleOptions& options);

}  // namespace einstar::calibrate::detail
