#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <numbers>

#include "einstar/render/view_camera.hpp"

using namespace einstar;
using render::Vec3f;
using Catch::Approx;

namespace {

bool near(const Vec3f& a, const Vec3f& b, float tol = 1e-3f) { return (a - b).norm() < tol; }

}  // namespace

TEST_CASE("the default view looks along +z from behind the target, upright as the scanner is held") {
    const render::ViewCamera cam;
    CHECK(near(cam.eye(), Vec3f(0, 0, -500)));
    CHECK(near(cam.forward(), Vec3f::UnitZ()));
    // View space: x right, y up, looking down -z. The scanner's top (world +x) is up, its +y to the right.
    const Eigen::Vector4f top = cam.view() * Eigen::Vector4f(10, 0, 300, 1), side = cam.view() * Eigen::Vector4f(0, 10, 300, 1);
    CHECK(top.y() == Approx(10.0f).margin(1e-3));
    CHECK(top.x() == Approx(0.0f).margin(1e-3));
    CHECK(side.x() == Approx(10.0f).margin(1e-3));
    CHECK(top.z() == Approx(-800.0f));
}

TEST_CASE("orbit goes past straight up or down without stopping") {
    render::ViewCamera cam;
    // Dragging down swings the eye down the screen (world -x, the scanner's top being up); a half turn in
    // steps ends on the far side, looking back.
    for (int i = 0; i < 100; ++i) cam.orbit(0.0f, std::numbers::pi_v<float> / 100);
    CHECK(near(cam.eye(), Vec3f(0, 0, 1100), 0.05f));
    CHECK(near(cam.forward(), -Vec3f::UnitZ(), 1e-4f));
    // The first quarter of it passed straight under the target (where a pitch clamp used to stop).
    render::ViewCamera q;
    q.orbit(0.0f, std::numbers::pi_v<float> / 2);
    CHECK(near(q.eye(), Vec3f(-800, 0, 300), 0.05f));
    // Dragging right swings the eye right (world +y).
    render::ViewCamera r;
    r.orbit(0.1f, 0.0f);
    CHECK(r.eye().y() > 0);
    r.reset();
    CHECK(near(r.eye(), Vec3f(0, 0, -500)));
}

TEST_CASE("follow ends behind and above the scanner, looking where it points, over its top") {
    render::ViewCamera cam;
    cam.distance = 500;
    // A scanner at (100, 0, 0) turned 90 degrees about y: its line of sight is world -x.
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = Eigen::AngleAxisf(-std::numbers::pi_v<float> / 2, Vec3f::UnitY()).toRotationMatrix();
    T.block<3, 1>(0, 3) = Vec3f(100, 0, 0);
    REQUIRE(near(T.block<3, 3>(0, 0) * Vec3f::UnitZ(), -Vec3f::UnitX()));
    const Eigen::Matrix3f R = T.block<3, 3>(0, 0);
    const Vec3f boresight = R * Vec3f::UnitZ(), top = R * Vec3f::UnitX(), side = R * Vec3f::UnitY();
    cam.follow(T, 0.5f);  // part of the way
    const Vec3f half = cam.forward();
    for (int i = 0; i < 60; ++i) cam.follow(T, 0.5f);
    CHECK(!near(half, cam.forward(), 1e-2f));
    // Looking at the aim point, the line of sight raised 25 degrees from the boresight towards the top.
    CHECK(near(cam.target, Vec3f(100, 0, 0) + render::ViewCamera::kFollowAimMm * boresight, 1e-2f));
    CHECK(cam.forward().dot(boresight) == Approx(std::cos(render::ViewCamera::kFollowRaiseRad)).margin(1e-4));
    CHECK(cam.forward().dot(top) < 0);  // looking down from above the top
    CHECK(cam.forward().dot(side) == Approx(0.0f).margin(1e-4));
    // Upright: the scanner's +y is the view's right, its top towards the view's up.
    CHECK(near(cam.orientation * Vec3f::UnitX(), side, 1e-4f));
    CHECK((cam.orientation * Vec3f(0, -1, 0)).dot(top) > 0.9f);
    // The line of sight passes over the scanner (220 mm long, its top 110 mm above the axis, its front at
    // z = 0 and back at z = -52 in its frame), at this distance and zoomed in.
    for (const float d : {500.0f, 350.0f}) {
        cam.distance = d;
        const Eigen::Matrix4f S = T.inverse();
        for (int i = 0; i <= 200; ++i) {
            const Vec3f p = (S * (cam.eye() + (cam.target - cam.eye()) * (static_cast<float>(i) / 200)).homogeneous()).head<3>();
            if (p.z() <= 0 && p.z() >= -52) CHECK(p.x() > 125);
        }
    }
}
