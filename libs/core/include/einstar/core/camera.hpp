#pragma once

#include <array>

#include "einstar/core/se3.hpp"

namespace einstar {

// Pinhole intrinsics with Brown-Conrady distortion (k1, k2, p1, p2, k3).
struct CameraModel {
    int width = 0;
    int height = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0, skew = 0;
    std::array<double, 5> dist{};  // k1 k2 p1 p2 k3

    [[nodiscard]] Mat3 K() const {
        Mat3 k;
        k << fx, skew, cx, 0, fy, cy, 0, 0, 1;
        return k;
    }

    // Normalised (undistorted) coords -> distorted pixel coords.
    [[nodiscard]] Vec2 distort_normalized(const Vec2& n) const {
        const double x = n.x(), y = n.y();
        const double r2 = x * x + y * y;
        const double radial = 1 + dist[0] * r2 + dist[1] * r2 * r2 + dist[4] * r2 * r2 * r2;
        const double xd = x * radial + 2 * dist[2] * x * y + dist[3] * (r2 + 2 * x * x);
        const double yd = y * radial + dist[2] * (r2 + 2 * y * y) + 2 * dist[3] * x * y;
        return {fx * xd + skew * yd + cx, fy * yd + cy};
    }

    [[nodiscard]] Vec2 project(const Vec3& p_cam) const { return distort_normalized(p_cam.head<2>() / p_cam.z()); }
};

// Three-camera rig: left IR (reference), right IR, texture RGB.
struct RigCalibration {
    CameraModel left;
    CameraModel right;
    CameraModel texture;
    SE3 T_right_left = SE3::Identity();    // left-camera coords -> right-camera coords
    SE3 T_texture_left = SE3::Identity();  // left-camera coords -> texture-camera coords

    [[nodiscard]] double baseline_mm() const { return T_right_left.translation().norm(); }
};

}  // namespace einstar
