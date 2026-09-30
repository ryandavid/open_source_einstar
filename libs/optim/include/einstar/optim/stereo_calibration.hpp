#pragma once

// Stereo calibration refinement from views of a planar board: joint bundle adjustment of both IR
// cameras' intrinsics (fx, fy, cx, cy; optionally the distortion), their relative pose and each view's
// board pose, minimising the reprojection error of the board's dots in both raw images. Views can be
// grouped into sets (e.g. EXStar's calibration captures and later ones) so the fit can be reported per
// set: if one model fits every set, the cameras are unchanged between them.

#include <vector>

#include "einstar/core/camera.hpp"
#include "einstar/core/se3.hpp"

namespace einstar::optim {

struct BoardView {
    std::vector<Vec3> board;        // dot positions on the board (mm, z = 0)
    std::vector<Vec2> left, right;  // raw (distorted) pixel observations, same order as `board`
    SE3 T_left_board = SE3::Identity();  // initial board pose in the left camera
    int set = 0;
};

struct StereoCalibOptions {
    bool free_intrinsics = true;  // fx, fy, cx, cy of both cameras
    bool free_distortion = false;
    bool free_rig = true;         // left -> right pose
    double huber_px = 1.0;
    int max_iterations = 200;
};

struct StereoCalibResult {
    RigCalibration rig;
    std::vector<SE3> T_left_board;   // per view
    std::vector<double> rms_by_set;  // reprojection rms (px) per set, both cameras
    double rms = 0;
};

[[nodiscard]] StereoCalibResult refine_stereo_calibration(const RigCalibration& initial, const std::vector<BoardView>& views,
                                                          const StereoCalibOptions& options = {});

}  // namespace einstar::optim
