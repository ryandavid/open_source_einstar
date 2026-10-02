#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace einstar::render {

using Mat4f = Eigen::Matrix4f;
using Vec3f = Eigen::Vector3f;
using Quatf = Eigen::Quaternionf;

// Orbit camera for inspecting the scan, plus a "follow" mode that sits behind the scanner.
struct ViewCamera {
    // The camera looks along its own +z with +y down, as the scanner's optical frame does; `orientation`
    // takes camera axes to world axes, so the identity looks along world +z (the scan's first view).
    Vec3f target{0, 0, 300};
    float distance = 600.0f;  // mm, eye to target
    Quatf orientation = Quatf::Identity();
    float fov_y = 0.8f;  // rad
    float near_plane = 5.0f;
    float far_plane = 5000.0f;

    // Follow mode looks at the point this far in front of the scanner (its working distance).
    static constexpr float kFollowAimMm = 300.0f;

    [[nodiscard]] Vec3f eye() const;
    [[nodiscard]] Vec3f forward() const;
    [[nodiscard]] Mat4f view() const;
    [[nodiscard]] Mat4f projection(float aspect) const;

    // Turns about the camera's own up and right axes: no pole and no clamp, so every orientation is
    // reachable (dragging right moves the eye right, dragging down moves it down).
    void orbit(float dx_rad, float dy_rad);
    void pan(float dx_px, float dy_px, float viewport_height_px);
    void zoom(float factor);
    // Moves a fraction `blend` (0..1) of the way to the view from behind the scanner along its line of
    // sight: same orientation as T_world_scanner, looking at the point kFollowAimMm in front of it.
    void follow(const Mat4f& T_world_scanner, float blend);
    // The default view: from behind the scan's first scanner position, along its line of sight.
    void reset();
};

[[nodiscard]] Mat4f look_at(const Vec3f& eye, const Vec3f& target, const Vec3f& up);
[[nodiscard]] Mat4f perspective(float fov_y, float aspect, float near_plane, float far_plane);  // Metal clip space (z in [0,1])

}  // namespace einstar::render
