#pragma once

#include "einstar/core/image.hpp"
#include "einstar/core/se3.hpp"

namespace einstar::depth {

// Geometry of a rectified stereo pair. Both rectified cameras share f and cy; the right camera's
// principal point sits `cx_offset` pixels right of the left one's, so that the cameras' overlap stays
// in frame (the Einstar's cameras converge: a shared cx would push half the overlap out of the images).
// Disparity d = x_left - x_right is then signed: 0 at depth f * baseline / cx_offset, negative beyond.
struct RectifiedGeometry {
    double f = 0;          // focal length in pixels
    double cx = 0, cy = 0; // left principal point (cy is shared)
    double baseline = 0;   // mm; right camera centre at (+baseline, 0, 0) in the rectified left frame
    double cx_offset = 0;  // right cx - left cx (0 in recordings made before it existed)

    [[nodiscard]] double right_cx() const { return cx + cx_offset; }
    [[nodiscard]] double depth_from_disparity(double d) const { return f * baseline / (d + cx_offset); }
    [[nodiscard]] double disparity_from_depth(double z) const { return f * baseline / z - cx_offset; }
    // The same pair at 1/2^level resolution (2x2 box downsampling per level).
    [[nodiscard]] RectifiedGeometry scaled(int level) const {
        const double s = 1.0 / static_cast<double>(1 << level);
        return {f * s, (cx + 0.5) * s - 0.5, (cy + 0.5) * s - 0.5, baseline, cx_offset * s};
    }
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
