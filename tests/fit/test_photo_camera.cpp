#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <print>
#include <random>

#include "einstar/fit/photo_camera.hpp"
#include "einstar/fit/synthetic_part.hpp"

using namespace einstar;

namespace {

// A phone photo (4032 x 3024, 26 mm equivalent) of the flanged box from above and to one side.
fit::PinholeCamera phone_camera() {
    fit::PinholeCamera c;
    c.focal = fit::focal_from_35mm(26, 4032, 3024);
    c.principal = Vec2(2016, 1512);
    const Vec3 eye(120, -160, 170), target(0, 0, 8);
    const Vec3 z = (target - eye).normalized();
    const Vec3 x = z.cross(Vec3::UnitZ()).normalized();  // right in the photo
    const Vec3 y = z.cross(x);                           // down
    Mat3 R;
    R.row(0) = x.transpose();
    R.row(1) = y.transpose();
    R.row(2) = z.transpose();
    c.T_camera_world.linear() = R;
    c.T_camera_world.translation() = -R * eye;
    return c;
}

// Points a user might click: corners and hole edges of the part, seen in the photo.
std::vector<Vec3> part_points() {
    return {{-50, -25, 4}, {50, -25, 4}, {50, 25, 4}, {-50, 25, 4}, {-30, -20, 20}, {30, -20, 20}, {30, 20, 20}, {-30, 20, 20},
            {42, -19, 4},  {-42, 19, 4}, {0, 0, 20},  {30, -20, 4}, {-30, -20, 4}, {50, -25, 0}};
}

double angle_deg(const Mat3& a, const Mat3& b) { return Eigen::AngleAxisd(a.transpose() * b).angle() * 180 / std::numbers::pi; }

}  // namespace

TEST_CASE("a photo's camera from points matched on the part") {
    const fit::PinholeCamera truth = phone_camera();
    const auto pts = part_points();
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0, 0.5);
    std::vector<Vec2> px;
    for (const auto& p : pts) {
        const auto q = truth.project(p);
        REQUIRE(q);
        REQUIRE((q->x() > 0 && q->x() < 4032 && q->y() > 0 && q->y() < 3024));
        px.push_back(*q + Vec2(noise(rng), noise(rng)));
    }
    fit::CameraFitOptions o;
    o.width = 4032;
    o.height = 3024;

    SECTION("all the pairs, focal length unknown") {
        const auto fit = fit::fit_camera(px, pts, o);
        REQUIRE(fit);
        std::println("camera: rms {:.2f} px, focal {:.1f} (true {:.1f}), centre off {:.3f} mm", fit->rms_px, fit->camera.focal, truth.focal,
                     (fit->camera.center() - truth.center()).norm());
        CHECK(fit->rms_px < 1.0);
        CHECK(std::abs(fit->camera.focal / truth.focal - 1) < 0.01);
        CHECK((fit->camera.center() - truth.center()).norm() < 0.5 * truth.center().norm() / 100);  // < 0.5% of the distance
        CHECK(angle_deg(fit->camera.T_camera_world.linear(), truth.T_camera_world.linear()) < 0.2);
        for (const bool o_ : fit->outlier) CHECK(!o_);
        // A ray through a pixel passes the point seen there.
        const auto [origin, dir] = fit->camera.ray(px[10]);
        const Vec3 d = pts[10] - origin;
        CHECK((d - d.dot(dir) * dir).norm() < 0.5);
    }
    SECTION("four pairs with the focal length from EXIF, all on the top faces") {
        o.focal_prior = truth.focal;
        const std::vector<Vec3> few = {pts[4], pts[5], pts[6], pts[7]};
        const std::vector<Vec2> fpx = {px[4], px[5], px[6], px[7]};
        const auto fit = fit::fit_camera(fpx, few, o);
        REQUIRE(fit);
        CHECK(!fit->focal_fitted);
        CHECK(angle_deg(fit->camera.T_camera_world.linear(), truth.T_camera_world.linear()) < 1.0);
        CHECK((fit->camera.center() - truth.center()).norm() < 3.0);
    }
    SECTION("a mis-clicked pair is found") {
        px[3] += Vec2(60, -45);
        const auto fit = fit::fit_camera(px, pts, o);
        REQUIRE(fit);
        CHECK(fit->outlier[3]);
        int flagged = 0;
        for (const bool b : fit->outlier) flagged += b;
        CHECK(flagged == 1);
    }
    SECTION("too few pairs") {
        const std::vector<Vec3> few = {pts[0], pts[1], pts[2], pts[3], pts[4]};
        const std::vector<Vec2> fpx = {px[0], px[1], px[2], px[3], px[4]};
        CHECK(!fit::fit_camera(fpx, few, o));
    }
}
