#include <catch2/catch_test_macros.hpp>

#include "einstar/core/lasso.hpp"

using namespace einstar;
using Eigen::Vector2f;
using Eigen::Vector3f;

namespace {

// An orthographic view onto a 200 x 200 px viewport: world (x, y) in [-100, 100] maps to px (x + 100, 100 - y).
Eigen::Matrix4f ortho() {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m(0, 0) = m(1, 1) = 0.01f;
    m(2, 2) = 0.0f;
    return m;
}

LassoStroke stroke(std::vector<Vector2f> poly, bool subtract = false, Eigen::Matrix4f vp = ortho()) {
    return {vp, Vector2f(200, 200), std::move(poly), subtract};
}

std::vector<Vector2f> square(float x0, float y0, float x1, float y1) { return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; }

}  // namespace

TEST_CASE("a lasso selects through all depths what projects inside it") {
    LassoSelection sel;
    CHECK(sel.empty());
    CHECK_FALSE(sel.contains(Vector3f(0, 0, 0)));
    sel.add(stroke(square(50, 50, 150, 150)));  // world x, y in [-50, 50]
    REQUIRE_FALSE(sel.empty());
    for (const float z : {-500.0f, 0.0f, 300.0f, 5000.0f}) {
        CHECK(sel.contains(Vector3f(0, 0, z)));
        CHECK(sel.contains(Vector3f(-49.5f, 49.5f, z)));
        CHECK_FALSE(sel.contains(Vector3f(-50.6f, 0, z)));
        CHECK_FALSE(sel.contains(Vector3f(0, 51, z)));
    }
    // Pixel centres decide: px 50.5 is the first column inside, 49.5 is not.
    CHECK(sel.contains(Vector3f(-49.5f, 0, 0)));
    CHECK_FALSE(sel.contains(Vector3f(-50.5f, 0, 0)));
}

TEST_CASE("lasso strokes combine in order, and polygons fill even-odd") {
    LassoSelection sel;
    sel.add(stroke(square(20, 20, 180, 180)));
    sel.add(stroke(square(80, 80, 120, 120), true));  // a hole
    CHECK(sel.contains(Vector3f(-70, 0, 0)));
    CHECK_FALSE(sel.contains(Vector3f(0, 0, 0)));
    sel.add(stroke(square(95, 95, 105, 105)));  // added back after the hole
    CHECK(sel.contains(Vector3f(0, 0, 0)));
    // A subtractive stroke alone selects nothing.
    LassoSelection sub;
    sub.add(stroke(square(0, 0, 200, 200), true));
    CHECK(sub.empty());
    CHECK_FALSE(sub.contains(Vector3f(0, 0, 0)));
    // Concave: a U shape leaves its notch out.
    LassoSelection u;
    u.add(stroke({{20, 20}, {60, 20}, {60, 140}, {140, 140}, {140, 20}, {180, 20}, {180, 180}, {20, 180}}));
    CHECK(u.contains(Vector3f(-60, 0, 0)));   // px (40, 100): left arm
    CHECK_FALSE(u.contains(Vector3f(0, 20, 0)));  // px (100, 80): the notch
    CHECK(u.contains(Vector3f(0, -60, 0)));   // px (100, 160): the base
}

TEST_CASE("a lasso ignores degenerate polygons and points behind its view") {
    LassoSelection sel;
    sel.add(stroke({{10, 10}, {100, 100}}));           // two points
    sel.add(stroke({{10, 10}, {100, 10}, {190, 10}}));  // no area
    sel.add(stroke(square(-50, -50, -10, -10)));       // outside the viewport
    CHECK(sel.strokes().empty());
    // Perspective: w = z, so z <= 0 is behind the eye even where x / w lands inside.
    Eigen::Matrix4f persp = Eigen::Matrix4f::Zero();
    persp(0, 0) = persp(1, 1) = 1.0f;
    persp(3, 2) = 1.0f;
    sel.add(stroke(square(0, 0, 200, 200), false, persp));
    CHECK(sel.contains(Vector3f(10, 10, 100)));
    CHECK_FALSE(sel.contains(Vector3f(10, 10, -100)));
    CHECK_FALSE(sel.contains(Vector3f(0, 0, 0)));
}
