#include <catch2/catch_test_macros.hpp>

#include <random>

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

TEST_CASE("rectified projections are row aligned with disparity f*B/z") {
    const RigCalibration rig = make_rig();
    const auto rect = calib::compute_rectification(rig);
    REQUIRE(std::abs(rect.geometry.baseline - rig.baseline_mm()) < 1e-9);

    std::mt19937 rng(3);
    std::uniform_real_distribution<double> ux(-120, 120), uy(-100, 100), uz(200, 600);
    for (int i = 0; i < 500; ++i) {
        const Vec3 p_left(ux(rng), uy(rng), uz(rng));
        const Vec3 p_right = rig.T_right_left * p_left;
        // Observed distorted pixels -> rectified pixels.
        auto to_rect = [&](const CameraModel& cam, const Mat3& R, const Vec3& p) {
            const Vec2 px = cam.project(p);
            const Vec2 n = calib::undistort_to_normalized(cam, px);
            const Vec3 ray = R * Vec3(n.x(), n.y(), 1.0);
            return Vec2(rect.rectified.fx * ray.x() / ray.z() + rect.rectified.cx,
                        rect.rectified.fy * ray.y() / ray.z() + rect.rectified.cy);
        };
        const Vec2 l = to_rect(rig.left, rect.R_left, p_left);
        const Vec2 r = to_rect(rig.right, rect.R_right, p_right);
        REQUIRE(std::abs(l.y() - r.y()) < 1e-6);
        const double z_rect = (rect.R_left * p_left).z();
        REQUIRE(std::abs((l.x() - r.x()) - rect.geometry.disparity_from_depth(z_rect)) < 1e-6);
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
