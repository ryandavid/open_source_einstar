#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace einstar::render {

using Mat4f = Eigen::Matrix4f;
using Vec3f = Eigen::Vector3f;

// Orbit camera for inspecting the scan, plus a "follow" mode that sits behind the scanner.
struct ViewCamera {
    Vec3f target{0, 0, 300};
    float distance = 600.0f;  // mm
    float yaw = 0.0f;         // rad
    float pitch = 0.0f;       // rad
    float fov_y = 0.8f;       // rad
    float near_plane = 5.0f;
    float far_plane = 5000.0f;

    // Scanner optical frame is +z forward, +y down; the default view looks along +z from behind.
    [[nodiscard]] Vec3f eye() const;
    [[nodiscard]] Mat4f view() const;
    [[nodiscard]] Mat4f projection(float aspect) const;

    void orbit(float dx_rad, float dy_rad);
    void pan(float dx_px, float dy_px, float viewport_height_px);
    void zoom(float factor);
};

[[nodiscard]] Mat4f look_at(const Vec3f& eye, const Vec3f& target, const Vec3f& up);
[[nodiscard]] Mat4f perspective(float fov_y, float aspect, float near_plane, float far_plane);  // Metal clip space (z in [0,1])

}  // namespace einstar::render
