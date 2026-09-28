#pragma once

#include "einstar/core/image.hpp"
#include "einstar/core/se3.hpp"

namespace einstar::depth {

// Geometry of a rectified stereo pair (both cameras share K after rectification).
struct RectifiedGeometry {
    double f = 0;          // focal length in pixels
    double cx = 0, cy = 0; // principal point (identical for both rectified cameras)
    double baseline = 0;   // mm; right camera centre at (+baseline, 0, 0) in the rectified left frame

    [[nodiscard]] double depth_from_disparity(double d) const { return f * baseline / d; }
    [[nodiscard]] double disparity_from_depth(double z) const { return f * baseline / z; }
};

// Organised point image in the rectified left camera frame. Invalid points have z == 0.
struct PointImage {
    Image<Vec3f> points;
    Image<Vec3f> normals;   // unit, pointing towards the camera; zero where unknown
    ImageF32 weights;       // 0..1
};

struct PointImageParams {
    float min_depth = 150.0f;     // mm
    float max_depth = 700.0f;     // mm
    float max_depth_jump = 4.0f;  // mm between neighbours used for normals
};

[[nodiscard]] PointImage disparity_to_points(const ImageF32& disparity, const ImageF32& confidence,
                                             const RectifiedGeometry& geom, const PointImageParams& params = {});

}  // namespace einstar::depth
