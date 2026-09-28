#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "einstar/core/se3.hpp"

using namespace einstar;
using Catch::Matchers::WithinAbs;

TEST_CASE("se3 exp/log round-trip") {
    for (const Vec6 xi : {Vec6{1, 2, 3, 0.1, -0.2, 0.3}, Vec6{0, 0, 0, 0, 0, 0}, Vec6{5, 0, -1, 1e-10, 0, 0},
                          Vec6{-3, 4, 10, 2.5, 0.4, -0.7}}) {
        const Vec6 back = se3_log(se3_exp(xi));
        REQUIRE((back - xi).norm() < 1e-9);
    }
}

TEST_CASE("se3 exp matches rotation about axis") {
    Vec6 xi = Vec6::Zero();
    xi(5) = M_PI / 2;  // 90 deg about z
    const SE3 t = se3_exp(xi);
    const Vec3 p = t * Vec3{1, 0, 0};
    REQUIRE_THAT(p.x(), WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(p.y(), WithinAbs(1.0, 1e-12));
    REQUIRE_THAT(rotation_angle(t), WithinAbs(M_PI / 2, 1e-12));
}
