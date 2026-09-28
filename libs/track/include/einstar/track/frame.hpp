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

}  // namespace einstar::track
