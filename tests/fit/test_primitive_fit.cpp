#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <print>
#include <random>

#include "einstar/fit/primitive_fit.hpp"

using namespace einstar;
using namespace einstar::fit;

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

// Oriented samples of a partial surface patch, with Gaussian noise along the normal and a share of
// outliers (points from a neighbouring face or debris).
struct Samples {
    std::vector<Vec3> points, normals;
    [[nodiscard]] PointSet view() const { return {points, normals, {}}; }
};

template <class F>
Samples sample(F&& param, int nu, int nv, double noise, double outliers, std::uint32_t seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> n(0, noise);
    std::uniform_real_distribution<double> u01(0, 1), off(-3, 3);
    Samples s;
    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j) {
            Vec3 p, nn;
            param((i + u01(rng)) / nu, (j + u01(rng)) / nv, p, nn);
            if (u01(rng) < outliers) p += Vec3(off(rng), off(rng), off(rng));
            else p += n(rng) * nn;
            s.points.push_back(p);
            s.normals.push_back(nn);
        }
    return s;
}

// A rigid placement away from the origin, so nothing depends on axis-aligned data.
const SE3& placement() {
    static const SE3 T = [] {
        SE3 t = SE3::Identity();
        t.linear() = (Eigen::AngleAxisd(0.4, Vec3(1, 2, 3).normalized())).toRotationMatrix();
        t.translation() = Vec3(40, -25, 130);
        return t;
    }();
    return T;
}

Samples placed(Samples s) {
    for (auto& p : s.points) p = placement() * p;
    for (auto& n : s.normals) n = placement().linear() * n;
    return s;
}

double angle_between_axes(const Vec3& a, const Vec3& b) { return std::acos(std::min(1.0, std::abs(a.dot(b)))); }

}  // namespace

TEST_CASE("plane fit: noisy patch with outliers") {
    const Samples s = placed(sample([](double u, double v, Vec3& p, Vec3& n) {
        p = Vec3(40 * u, 25 * v, 0);
        n = Vec3::UnitZ();
    }, 40, 25, 0.03, 0.05, 1));
    const auto r = fit_surface(SurfaceKind::plane, s.view());
    REQUIRE(r);
    const auto& pl = std::get<Plane>(r->surface);
    const Vec3 n_true = placement().linear() * Vec3::UnitZ();
    CHECK(angle_between_axes(pl.normal, n_true) < 0.05 * kDeg);
    CHECK(pl.normal.dot(n_true) > 0);  // oriented with the scan normals
    CHECK(std::abs(pl.offset - n_true.dot(placement().translation())) < 0.01);
    CHECK(std::abs(r->sigma - 0.03) < 0.006);
}

TEST_CASE("cylinder fit: a 90 degree arc of a 2.75 mm hole wall and a 20 mm boss") {
    for (const double radius : {2.75, 20.0}) {
        const Samples s = placed(sample([&](double u, double v, Vec3& p, Vec3& n) {
            const double a = u * 90 * kDeg;
            n = Vec3(std::cos(a), std::sin(a), 0);
            p = radius * n + Vec3(0, 0, 10 * v);
        }, 40, 20, 0.03, 0.05, 2));
        const auto r = fit_surface(SurfaceKind::cylinder, s.view());
        REQUIRE(r);
        const auto& c = std::get<Cylinder>(r->surface);
        INFO("radius " << radius);
        CHECK(std::abs(c.radius - radius) < 0.02);
        CHECK(angle_between_axes(c.axis, placement().linear() * Vec3::UnitZ()) < 0.3 * kDeg);
        const Vec3 axis_point = placement().translation();
        CHECK((c.point - axis_point - (c.point - axis_point).dot(c.axis) * c.axis).norm() < 0.03);
    }
}

TEST_CASE("cone, sphere and torus fits") {
    SECTION("cone: 30 degree half angle, part of a countersink") {
        const double ha = 30 * kDeg;
        const Samples s = placed(sample([&](double u, double v, Vec3& p, Vec3& n) {
            const double a = u * 180 * kDeg, h = 3 + 6 * v;
            const Vec3 radial(std::cos(a), std::sin(a), 0);
            p = h * Vec3::UnitZ() + h * std::tan(ha) * radial;
            n = std::cos(ha) * radial - std::sin(ha) * Vec3::UnitZ();
        }, 40, 20, 0.02, 0.03, 3));
        const auto r = fit_surface(SurfaceKind::cone, s.view());
        REQUIRE(r);
        const auto& c = std::get<Cone>(r->surface);
        CHECK(std::abs(c.half_angle - ha) < 0.2 * kDeg);
        CHECK((c.apex - placement().translation()).norm() < 0.05);
        CHECK(c.axis.dot(placement().linear() * Vec3::UnitZ()) > 0.9999);
    }
    SECTION("sphere: a 60 degree cap") {
        const Samples s = placed(sample([](double u, double v, Vec3& p, Vec3& n) {
            const double th = u * 60 * kDeg, ph = v * 2 * std::numbers::pi;
            n = Vec3(std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th));
            p = 12.0 * n;
        }, 30, 30, 0.03, 0.05, 4));
        const auto r = fit_surface(SurfaceKind::sphere, s.view());
        REQUIRE(r);
        const auto& sp = std::get<Sphere>(r->surface);
        CHECK(std::abs(sp.radius - 12) < 0.03);
        CHECK((sp.center - placement().translation()).norm() < 0.05);
    }
    SECTION("torus: a 3 mm fillet around a 25 mm shaft (a quarter of the tube)") {
        const Samples s = placed(sample([](double u, double v, Vec3& p, Vec3& n) {
            const double a = u * 2 * std::numbers::pi, b = (180 + 90 * v) * kDeg;
            const Vec3 radial(std::cos(a), std::sin(a), 0);
            n = std::cos(b) * radial + std::sin(b) * Vec3::UnitZ();
            p = 28.0 * radial + 3.0 * n;
        }, 80, 12, 0.02, 0.03, 5));
        const auto r = fit_surface(SurfaceKind::torus, s.view());
        REQUIRE(r);
        const auto& t = std::get<Torus>(r->surface);
        CHECK(std::abs(t.major - 28) < 0.05);
        CHECK(std::abs(t.minor - 3) < 0.03);
        CHECK(angle_between_axes(t.axis, placement().linear() * Vec3::UnitZ()) < 0.2 * kDeg);
    }
}

TEST_CASE("fit_best picks the right kind") {
    const std::array kinds{SurfaceKind::plane, SurfaceKind::cylinder, SurfaceKind::cone, SurfaceKind::sphere};
    const Samples plane = placed(sample([](double u, double v, Vec3& p, Vec3& n) {
        p = Vec3(30 * u, 30 * v, 0);
        n = Vec3::UnitZ();
    }, 30, 30, 0.03, 0.0, 6));
    const Samples cyl = placed(sample([](double u, double v, Vec3& p, Vec3& n) {
        const double a = u * 120 * kDeg;
        n = Vec3(std::cos(a), std::sin(a), 0);
        p = 8.0 * n + Vec3(0, 0, 15 * v);
    }, 30, 30, 0.03, 0.0, 7));
    const Samples sph = placed(sample([](double u, double v, Vec3& p, Vec3& n) {
        const double th = u * 70 * kDeg, ph = v * 2 * std::numbers::pi;
        n = Vec3(std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th));
        p = 10.0 * n;
    }, 30, 30, 0.03, 0.0, 8));
    const auto rp = fit_best(kinds, plane.view());
    const auto rc = fit_best(kinds, cyl.view());
    const auto rs = fit_best(kinds, sph.view());
    REQUIRE(rp);
    REQUIRE(rc);
    REQUIRE(rs);
    CHECK(kind_of(rp->surface) == SurfaceKind::plane);
    CHECK(kind_of(rc->surface) == SurfaceKind::cylinder);
    CHECK(kind_of(rs->surface) == SurfaceKind::sphere);
}
