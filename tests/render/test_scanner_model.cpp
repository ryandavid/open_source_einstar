#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include "einstar/render/scanner_model.hpp"

using namespace einstar;
using Eigen::Vector3f;

TEST_CASE("the scanner model is a closed bar facing out, the size of an Einstar") {
    const auto& m = render::scanner_model();
    REQUIRE(m.body.size() == 7 * 8 + 2);
    Vector3f lo = Vector3f::Constant(1e9f), hi = -lo;
    for (const auto& f : m.body) {
        REQUIRE(f.v.size() >= 3);
        Vector3f c = Vector3f::Zero();
        for (const auto& v : f.v) {
            c += v;
            lo = lo.cwiseMin(v);
            hi = hi.cwiseMax(v);
        }
        c /= static_cast<float>(f.v.size());
        const Vector3f n = (f.v[1] - f.v[0]).cross(f.v[2] - f.v[0]).normalized();
        for (const auto& v : f.v) CHECK(std::abs(n.dot(v - f.v[0])) < 1e-3f);  // planar
        // Outward: caps along x away from the middle, the rest away from the bar's axis.
        if (std::abs(n.x()) > 0.9f) CHECK(n.x() * c.x() > 0);
        else CHECK(n.dot(c - Vector3f(c.x(), 0, -23)) > 0);
    }
    CHECK(hi.x() - lo.x() == Catch::Approx(220.0f));
    CHECK(hi.y() - lo.y() == Catch::Approx(46.0f));
    CHECK(hi.z() - lo.z() == Catch::Approx(52.0f));
    // Front details face forward, in front of the glass.
    for (const auto& f : m.front) {
        CHECK((f.v[1] - f.v[0]).cross(f.v[2] - f.v[0]).normalized().z() > 0.99f);
        CHECK(f.v[0].z() > 0);
    }
}
