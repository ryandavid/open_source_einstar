#pragma once

// Global-marker bundle adjustment: jointly refines keyframe poses and world marker positions by
// minimising reprojection error of the marker centres in both rectified cameras. This is what makes
// a "global marker" map globally consistent (EXStar's map is a running mean without adjustment).

#include <map>
#include <vector>

#include "einstar/core/se3.hpp"
#include "einstar/depth/point_image.hpp"

namespace einstar::optim {

struct MarkerObservation {
    int frame = -1;
    int marker = -1;
    Vec2 left;   // rectified full-resolution pixel (left camera)
    Vec2 right;  // rectified full-resolution pixel (right camera, same row ideally)
};

struct MarkerBundle {
    depth::RectifiedGeometry geometry;   // full-resolution rectified geometry (f, cx, cy, baseline)
    std::map<int, SE3> T_world_camera;   // keyframe poses (rectified-left camera -> world)
    std::map<int, Vec3> markers;         // world positions
    std::vector<MarkerObservation> observations;
};

struct BundleParams {
    double huber_px = 1.0;
    double outlier_px = 3.0;   // observations with a larger final residual are dropped and re-solved
    int max_iterations = 100;
    int fixed_frame = -1;      // gauge; -1 = the lowest frame id
};

struct BundleReport {
    double rms_before_px = 0;
    double rms_after_px = 0;
    int observations = 0;
    int outliers_removed = 0;
    bool converged = false;
};

BundleReport optimize(MarkerBundle& bundle, const BundleParams& params = {});

// Reprojection RMS (px) of the current state.
[[nodiscard]] double reprojection_rms(const MarkerBundle& bundle);

}  // namespace einstar::optim
