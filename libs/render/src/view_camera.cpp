#include "einstar/render/view_camera.hpp"

#include <algorithm>
#include <cmath>

namespace einstar::render {

Mat4f look_at(const Vec3f& eye, const Vec3f& target, const Vec3f& up) {
    const Vec3f f = (target - eye).normalized();
    const Vec3f s = f.cross(up).normalized();
    const Vec3f u = s.cross(f);
    Mat4f m = Mat4f::Identity();
    m.row(0) << s.x(), s.y(), s.z(), -s.dot(eye);
    m.row(1) << u.x(), u.y(), u.z(), -u.dot(eye);
    m.row(2) << -f.x(), -f.y(), -f.z(), f.dot(eye);
    return m;
}

Mat4f perspective(float fov_y, float aspect, float n, float f) {
    const float t = 1.0f / std::tan(fov_y * 0.5f);
    Mat4f m = Mat4f::Zero();
    m(0, 0) = t / aspect;
    m(1, 1) = t;
    m(2, 2) = f / (n - f);
    m(2, 3) = n * f / (n - f);
    m(3, 2) = -1.0f;
    return m;
}

Vec3f ViewCamera::forward() const { return orientation * Vec3f::UnitZ(); }

Vec3f ViewCamera::eye() const { return target - distance * forward(); }

Mat4f ViewCamera::view() const { return look_at(eye(), target, orientation * Vec3f(0, -1, 0)); }

Mat4f ViewCamera::projection(float aspect) const { return perspective(fov_y, aspect, near_plane, far_plane); }

void ViewCamera::orbit(float dx, float dy) {
    // Local axes: about the camera's y (down) by -dx swings the eye to the right, about its x by dy swings it down.
    orientation = (orientation * Eigen::AngleAxisf(-dx, Vec3f::UnitY()) * Eigen::AngleAxisf(dy, Vec3f::UnitX())).normalized();
}

void ViewCamera::pan(float dx_px, float dy_px, float viewport_h) {
    const float scale = 2.0f * distance * std::tan(fov_y * 0.5f) / std::max(viewport_h, 1.0f);
    const Mat4f v = view();
    const Vec3f right = v.block<1, 3>(0, 0).transpose();
    const Vec3f up = v.block<1, 3>(1, 0).transpose();
    target += (-dx_px * right + dy_px * up) * scale;
}

void ViewCamera::zoom(float factor) { distance = std::clamp(distance * factor, 10.0f, 20000.0f); }

void ViewCamera::follow(const Mat4f& T_world_scanner, float blend) {
    const Eigen::Matrix3f R = T_world_scanner.block<3, 3>(0, 0);
    const Vec3f aim = (T_world_scanner * Eigen::Vector4f(0, 0, kFollowAimMm, 1)).head<3>();
    // Upright, then the eye swung up the screen (towards the scanner's top) about the view's x axis.
    const Quatf goal = Quatf(R) * scanner_upright() * Quatf(Eigen::AngleAxisf(-kFollowRaiseRad, Vec3f::UnitX()));
    orientation = orientation.slerp(blend, goal).normalized();
    target += blend * (aim - target);
}

void ViewCamera::reset() {
    const ViewCamera defaults;
    target = defaults.target;
    distance = defaults.distance;
    orientation = defaults.orientation;
}

}  // namespace einstar::render
