#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <random>
#include <tuple>

#include "einstar/calib/rectify.hpp"

using namespace einstar;

namespace {

RigCalibration make_rig() {
    RigCalibration rig;
    rig.left.width = rig.right.width = 1280;
    rig.left.height = rig.right.height = 1024;
    rig.left.fx = 1165.2; rig.left.fy = 1155.4; rig.left.cx = 624.4; rig.left.cy = 516.4;
    rig.left.dist = {-0.08, 0.12, 0.0005, -0.0003, 0.0};
    rig.right = rig.left;
    rig.right.fx = 1170.1; rig.right.fy = 1160.0; rig.right.cx = 640.2; rig.right.cy = 505.0;
    rig.right.dist = {-0.07, 0.10, -0.0004, 0.0002, 0.0};
    // Right camera ~110 mm to the right, slightly toed-in and rolled.
    SE3 T = SE3::Identity();
    T.linear() = (Eigen::AngleAxisd(-0.12, Vec3::UnitY()) * Eigen::AngleAxisd(0.01, Vec3::UnitZ()) *
                  Eigen::AngleAxisd(0.005, Vec3::UnitX())).toRotationMatrix();
    const Vec3 right_center_in_left(110.0, 2.0, 3.0);
    T.translation() = -T.linear() * right_center_in_left;
    rig.T_right_left = T;
    return rig;
}

}  // namespace

TEST_CASE("rectified projections are row aligned with disparity f*B/z - cx_offset") {
    const RigCalibration rig = make_rig();
    const auto rect = calib::compute_rectification(rig);
    REQUIRE(std::abs(rect.geometry.baseline - rig.baseline_mm()) < 1e-9);

    std::mt19937 rng(3);
    std::uniform_real_distribution<double> ux(-120, 120), uy(-100, 100), uz(200, 600);
    for (int i = 0; i < 500; ++i) {
        const Vec3 p_left(ux(rng), uy(rng), uz(rng));
        const Vec3 p_right = rig.T_right_left * p_left;
        // Observed distorted pixels -> rectified pixels.
        auto to_rect = [&](const CameraModel& cam, const Mat3& R, const CameraModel& rc, const Vec3& p) {
            const Vec2 px = cam.project(p);
            const Vec2 n = calib::undistort_to_normalized(cam, px);
            const Vec3 ray = R * Vec3(n.x(), n.y(), 1.0);
            return Vec2(rc.fx * ray.x() / ray.z() + rc.cx, rc.fy * ray.y() / ray.z() + rc.cy);
        };
        const Vec2 l = to_rect(rig.left, rect.R_left, rect.rectified, p_left);
        const Vec2 r = to_rect(rig.right, rect.R_right, rect.rectified_right, p_right);
        REQUIRE(std::abs(l.y() - r.y()) < 1e-6);
        const double z_rect = (rect.R_left * p_left).z();
        REQUIRE(std::abs((l.x() - r.x()) - rect.geometry.disparity_from_depth(z_rect)) < 1e-6);
        REQUIRE(std::abs(rect.geometry.depth_from_disparity(l.x() - r.x()) - z_rect) < 1e-6 * z_rect);
    }
}

TEST_CASE("rectification keeps the cameras' overlap in frame at the reference depth") {
    // A converging rig like the Einstar's (160 mm baseline, 22 degrees toe-in): with one shared principal
    // point the overlap at 300 mm would sit ~f*B/z = 600 px apart in the two images.
    RigCalibration rig = make_rig();
    rig.right = rig.left;
    SE3 T = SE3::Identity();
    T.linear() = Eigen::AngleAxisd(22.0 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
    T.translation() = -T.linear() * Vec3(160.0, 0, 0);
    rig.T_right_left = T;
    const auto rect = calib::compute_rectification(rig, {.reference_depth_mm = 300.0});
    const auto shared = calib::compute_rectification(rig, {.reference_depth_mm = 0.0});
    CHECK(shared.geometry.cx_offset == 0.0);
    // Points on the plane z = 300 mm (rectified frame) across the left image: how many have their right
    // view inside the right image.
    auto matchable = [&](const calib::StereoRectification& r) {
        int in = 0, total = 0;
        for (int y = 0; y < r.rectified.height; y += 16)
            for (int x = 0; x < r.rectified.width; x += 16) {
                ++total;
                const double d = r.geometry.disparity_from_depth(300.0);
                const double xr = x - d;
                in += xr >= 0 && xr < r.rectified_right.width;
            }
        return static_cast<double>(in) / total;
    };
    INFO("shared cx keeps " << matchable(shared) << ", per-camera cx " << matchable(rect));
    CHECK(matchable(shared) < 0.6);
    CHECK(matchable(rect) > 0.99);
    // Both sensors' centres stay well inside their rectified windows.
    for (const auto& [cam, R, rc] : {std::tuple{rig.left, rect.R_left, rect.rectified}, std::tuple{rig.right, rect.R_right, rect.rectified_right}}) {
        const Vec2 n = calib::undistort_to_normalized(cam, Vec2(cam.cx, cam.cy));
        const Vec3 ray = R * Vec3(n.x(), n.y(), 1.0);
        const double u = rc.fx * ray.x() / ray.z() + rc.cx;
        CHECK(u > 0.2 * rc.width);
        CHECK(u < 0.8 * rc.width);
    }
}

TEST_CASE("remap table points at the distorted source pixel") {
    const RigCalibration rig = make_rig();
    const auto rect = calib::compute_rectification(rig);
    const auto table = calib::build_remap(rig.left, rect.R_left, rect.rectified);
    // Take a rectified pixel, back-project to a 3D point in the left frame, project with distortion.
    for (const auto [x, y] : {std::pair{100, 100}, std::pair{640, 512}, std::pair{1200, 900}}) {
        const Vec3 ray_rect((x - rect.rectified.cx) / rect.rectified.fx, (y - rect.rectified.cy) / rect.rectified.fy, 1);
        const Vec3 p = rect.R_left.transpose() * (ray_rect * 400.0);
        const Vec2 src = rig.left.project(p);
        REQUIRE(std::abs(table.map_x(x, y) - src.x()) < 1e-2);
        REQUIRE(std::abs(table.map_y(x, y) - src.y()) < 1e-2);
    }
}
