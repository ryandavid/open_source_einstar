#pragma once

#include <cstdint>

#include "einstar/core/camera.hpp"
#include "einstar/core/image.hpp"
#include "einstar/depth/point_image.hpp"

namespace einstar::calib {

// Epipolar rectification of the left/right IR pair (Bouguet-style: each camera rotates by
// half of the relative rotation, then both align their x-axis with the baseline).
struct StereoRectification {
    Mat3 R_left = Mat3::Identity();   // original left camera frame -> rectified left frame
    Mat3 R_right = Mat3::Identity();  // original right camera frame -> rectified right frame
    CameraModel rectified;            // left ideal pinhole model (no distortion)
    CameraModel rectified_right;      // the same but for cx (geometry.cx_offset to the right)
    depth::RectifiedGeometry geometry;

    // Points in the rectified left frame -> original left camera frame.
    [[nodiscard]] SE3 T_left_rectified() const {
        SE3 t = SE3::Identity();
        t.linear() = R_left.transpose();
        return t;
    }
};

struct RectificationOptions {
    double scale = 1.0;  // rectified image size relative to the sensors'
    // Depth at which a point has zero disparity: the right camera's principal point is shifted so that
    // the scene at this depth lands on the same columns in both images. Matching then loses only the
    // columns |disparity| wide at the image edges, nearest this depth (0 = one shared principal point,
    // as before; the overlap then shifts by f * baseline / z, half the image at 300 mm).
    double reference_depth_mm = 320.0;
};

[[nodiscard]] StereoRectification compute_rectification(const RigCalibration& rig, const RectificationOptions& options = {});

// Look-up table: for each rectified pixel, the source pixel in the original (distorted) image.
struct RemapTable {
    int width = 0, height = 0;
    ImageF32 map_x, map_y;
};

[[nodiscard]] RemapTable build_remap(const CameraModel& original, const Mat3& R_rect, const CameraModel& rectified);

// Bilinear remap; out-of-bounds samples become 0.
void remap(ImageView<const std::uint8_t> src, const RemapTable& table, ImageView<std::uint8_t> dst);
[[nodiscard]] ImageU8 remap(ImageView<const std::uint8_t> src, const RemapTable& table);

// Distorted pixel -> undistorted normalised coordinates (iterative inversion).
[[nodiscard]] Vec2 undistort_to_normalized(const CameraModel& cam, const Vec2& pixel, int iterations = 20);

}  // namespace einstar::calib
