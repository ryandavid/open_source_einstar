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

Vec3f ViewCamera::eye() const {
    // Spherical offset from target; yaw=pitch=0 puts the eye at target - z*distance.
    const Vec3f dir(std::sin(yaw) * std::cos(pitch), std::sin(pitch), -std::cos(yaw) * std::cos(pitch));
    return target + distance * dir;
}

Mat4f ViewCamera::view() const { return look_at(eye(), target, Vec3f(0, -1, 0)); }

Mat4f ViewCamera::projection(float aspect) const { return perspective(fov_y, aspect, near_plane, far_plane); }

void ViewCamera::orbit(float dx, float dy) {
    yaw += dx;
    pitch = std::clamp(pitch + dy, -1.5f, 1.5f);
}

void ViewCamera::pan(float dx_px, float dy_px, float viewport_h) {
    const float scale = 2.0f * distance * std::tan(fov_y * 0.5f) / std::max(viewport_h, 1.0f);
    const Mat4f v = view();
    const Vec3f right = v.block<1, 3>(0, 0).transpose();
    const Vec3f up = v.block<1, 3>(1, 0).transpose();
    target += (-dx_px * right + dy_px * up) * scale;
}

void ViewCamera::zoom(float factor) { distance = std::clamp(distance * factor, 10.0f, 20000.0f); }

}  // namespace einstar::render
