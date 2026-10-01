#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "einstar/core/device_data.hpp"
#include "einstar/core/image.hpp"
#include "einstar/core/se3.hpp"

namespace einstar::track {

// Pinhole intrinsics of an organised depth/point image (rectified, undistorted).
struct Intrinsics {
    int width = 0, height = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0;

    [[nodiscard]] Intrinsics scaled(double s) const {
        return {static_cast<int>(width * s), static_cast<int>(height * s), fx * s, fy * s, (cx + 0.5) * s - 0.5,
                (cy + 0.5) * s - 0.5};
    }
    [[nodiscard]] bool project(const Vec3f& p, float& u, float& v) const {
        if (p.z() <= 0) return false;
        u = static_cast<float>(fx * p.x() / p.z() + cx);
        v = static_cast<float>(fy * p.y() / p.z() + cy);
        return u >= 0 && v >= 0 && u <= static_cast<float>(width - 1) && v <= static_cast<float>(height - 1);
    }
};

struct MarkerPoint {
    Vec3 position;  // camera frame, mm
    Vec3 normal;
    double diameter = 6.0;
    int id = -1;    // global id if known
    Vec2 left_rect = Vec2::Constant(-1);   // rectified full-resolution image centres (for bundle adjustment)
    Vec2 right_rect = Vec2::Constant(-1);
};

// One depth frame ready for tracking: organised points/normals in the camera frame.
//
// A frame may live on the GPU (`device` set by a GPU frontend). Its CPU images are then empty
// until ensure_cpu() is called; GPU consumers read `device` directly, CPU code calls ensure_cpu().
struct DepthFrame {
    std::uint64_t index = 0;
    double timestamp_s = 0;
    Intrinsics intrinsics;
    mutable Image<Vec3f> points;   // z == 0 where invalid
    mutable Image<Vec3f> normals;  // zero where unknown
    mutable ImageF32 weights;      // 0..1
    std::vector<MarkerPoint> markers;
    std::shared_ptr<const DeviceFrameData> device;

    // Fills the CPU images from `device` if they are empty (no-op for CPU frames).
    void ensure_cpu() const;
    [[nodiscard]] int width() const { return device ? device->width() : points.width(); }
    [[nodiscard]] int height() const { return device ? device->height() : points.height(); }
};

// Builds points/normals/weights from a Z-depth image (e.g. EXStar fixture frames).
[[nodiscard]] DepthFrame make_depth_frame(const ImageF32& depth, const Intrinsics& k, double max_normal_jump_mm = 3.0);

// Depth at object edges is the least reliable: stereo smears the foreground's disparity across the
// silhouette ("edge fattening") and pixels straddling a depth step land between the surfaces ("flying
// pixels"). Both leave a fringe of fused surface along outlines. This removes depth within `radius_px`
// of a depth step (neighbours further apart than max(min_jump_mm, jump_ratio x depth)) and, optionally,
// of the edge of the valid region (radius_rim_px: off by default; the scanner's depth is sparse, so
// every hole has a rim, and eroding them costs real surface and changes registration).
// Defaults chosen on a real scan (display + glossy bucket, 2026-10-01), with filter_grazing, the
// grazing weight in fusion and a 5-frame observation minimum (recon::ProcessParams).
struct DepthEdgeFilter {
    int radius_px = 2;
    int radius_rim_px = 0;
    float min_jump_mm = 4.0f;
    float jump_ratio = 0.01f;
    // Speckle filter: connected regions (neighbours within the jump threshold) smaller than this are
    // removed: isolated stereo mismatches, which fuse into floating flakes. 0 = off. (Pixels of the
    // recorded 640 x 512 depth; EXStar's minPatchSize is 100.)
    int min_region_px = 100;
};
void filter_depth_edges(ImageF32& depth, const DepthEdgeFilter& params);

// Viewing-angle filters on a frame with normals, after EXStar's range-image processing (its E10
// BuildSetting.ini [ProcessDataSection]: maxCameraAngle 70, boundaryWidth 2, boundaryAngleOnly 1):
// points whose surface is seen more obliquely than `max_view_angle_deg` are dropped, and the
// `rim_px`-wide border of the valid region is dropped where the surface there is steeper than
// `rim_angle_deg` (a border on a face-on surface stays). Inside the valid region, a pixel without a
// normal (its neighbours too far apart: a step, or too steep to measure) counts as too oblique.
// 0 disables either.
struct GrazingFilter {
    double max_view_angle_deg = 70.0;
    int rim_px = 2;
    double rim_angle_deg = 45.0;
};
void filter_grazing(DepthFrame& frame, const GrazingFilter& params);

}  // namespace einstar::track
