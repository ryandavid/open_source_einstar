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

TEST_CASE("the default view looks along +z from behind the target, +y down") {
    const render::ViewCamera cam;
    CHECK(near(cam.eye(), Vec3f(0, 0, -300)));
    CHECK(near(cam.forward(), Vec3f::UnitZ()));
    // View space: x right, y up, looking down -z; world +y (down) is view -y.
    const Eigen::Vector4f p = cam.view() * Eigen::Vector4f(0, 10, 300, 1);
    CHECK(p.y() < 0);
    CHECK(p.z() == Approx(-600.0f));
}

TEST_CASE("orbit goes past straight up or down without stopping") {
    render::ViewCamera cam;
    // Dragging down swings the eye down (to +y); a half turn in steps ends on the far side, looking back.
    for (int i = 0; i < 100; ++i) cam.orbit(0.0f, std::numbers::pi_v<float> / 100);
    CHECK(near(cam.eye(), Vec3f(0, 0, 900), 0.05f));
    CHECK(near(cam.forward(), -Vec3f::UnitZ(), 1e-4f));
    // The first quarter of it passed straight under the target (where a pitch clamp used to stop).
    render::ViewCamera q;
    q.orbit(0.0f, std::numbers::pi_v<float> / 2);
    CHECK(near(q.eye(), Vec3f(0, 600, 300), 0.05f));
    // Dragging right swings the eye right.
    render::ViewCamera r;
    r.orbit(0.1f, 0.0f);
    CHECK(r.eye().x() > 0);
    r.reset();
    CHECK(near(r.eye(), Vec3f(0, 0, -300)));
}

TEST_CASE("follow ends behind the scanner, looking where it points") {
    render::ViewCamera cam;
    cam.distance = 500;
    // A scanner at (100, 0, 0) turned 90 degrees about y: its line of sight is world -x.
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = Eigen::AngleAxisf(-std::numbers::pi_v<float> / 2, Vec3f::UnitY()).toRotationMatrix();
    T.block<3, 1>(0, 3) = Vec3f(100, 0, 0);
    REQUIRE(near(T.block<3, 3>(0, 0) * Vec3f::UnitZ(), -Vec3f::UnitX()));
    cam.follow(T, 0.5f);  // part of the way
    CHECK(!near(cam.forward(), -Vec3f::UnitX(), 1e-2f));
    for (int i = 0; i < 60; ++i) cam.follow(T, 0.5f);
    CHECK(near(cam.forward(), -Vec3f::UnitX(), 1e-4f));
    CHECK(near(cam.target, Vec3f(100 - render::ViewCamera::kFollowAimMm, 0, 0), 1e-2f));
    CHECK(near(cam.eye(), Vec3f(100 - render::ViewCamera::kFollowAimMm + 500, 0, 0), 1e-2f));  // 200 mm behind it
    // The scanner's down is the view's down.
    CHECK(near(cam.orientation * Vec3f::UnitY(), T.block<3, 3>(0, 0) * Vec3f::UnitY(), 1e-4f));
}
