#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <random>

#include "einstar/calib/convention.hpp"

using namespace einstar;

namespace {

RigCalibration converging_rig() {
    RigCalibration rig;
    rig.left.width = rig.right.width = rig.texture.width = 1280;
    rig.left.height = rig.right.height = rig.texture.height = 1024;
    rig.left.fx = 1157.2, rig.left.fy = 1157.0, rig.left.cx = 624.7, rig.left.cy = 521.7;
    rig.left.dist = {-0.08, 0.12, 0.0005, -0.0003, 0.01};
    rig.right = rig.left;
    rig.right.fx = 1158.8, rig.right.fy = 1158.7, rig.right.cx = 633.2, rig.right.cy = 505.4;
    rig.right.dist = {-0.07, 0.10, -0.0004, 0.0002, 0.0};
    rig.texture = rig.left;
    SE3 T = SE3::Identity();
    T.linear() = (Eigen::AngleAxisd(-22.1 * M_PI / 180, Vec3::UnitY()) * Eigen::AngleAxisd(0.003, Vec3::UnitZ())).toRotationMatrix();
    T.translation() = -T.linear() * Vec3(159.9, 0.4, 30.7);
    rig.T_right_left = T;
    rig.T_texture_left = SE3::Identity();
    rig.T_texture_left.translation() = Vec3(80, -30, 5);
    return rig;
}

}  // namespace

TEST_CASE("the other camera convention sees every point at the turned pixels of the other camera") {
    const RigCalibration rig = converging_rig();
    const RigCalibration sw = calib::swap_camera_convention(rig);
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> ux(-100, 100), uy(-80, 80), uz(220, 500);
    // The new left camera is the old right one turned: a point's new-left pixel is its old-right pixel turned.
    SE3 D = SE3::Identity();
    D.linear() = Vec3(-1, -1, 1).asDiagonal();
    for (int i = 0; i < 200; ++i) {
        const Vec3 p_old_left(ux(rng), uy(rng), uz(rng));
        const Vec3 p_old_right = rig.T_right_left * p_old_left;
        const Vec2 ul = rig.left.project(p_old_left), ur = rig.right.project(p_old_right);
        const Vec3 p_new_left = D * p_old_right;
        const Vec2 nl = sw.left.project(p_new_left), nr = sw.right.project(sw.T_right_left * p_new_left);
        REQUIRE((nl - Vec2(1279 - ur.x(), 1023 - ur.y())).norm() < 1e-9);
        REQUIRE((nr - Vec2(1279 - ul.x(), 1023 - ul.y())).norm() < 1e-9);
        // The texture camera sees the point where it did.
        REQUIRE(((sw.T_texture_left * p_new_left) - (rig.T_texture_left * p_old_left)).norm() < 1e-9);
    }
    // Its own inverse, same baseline.
    const RigCalibration back = calib::swap_camera_convention(sw);
    CHECK(std::abs(back.left.cx - rig.left.cx) < 1e-12);
    CHECK(std::abs(back.right.cy - rig.right.cy) < 1e-12);
    CHECK((back.T_right_left.matrix() - rig.T_right_left.matrix()).norm() < 1e-12);
    CHECK(std::abs(sw.baseline_mm() - rig.baseline_mm()) < 1e-9);
}

TEST_CASE("a calibration's camera convention is told from a reference in EXStar's") {
    const RigCalibration factory = converging_rig();
    // A later calibration of the same scanner: principal points a few px off the factory's.
    RigCalibration later = factory;
    later.left.cx += 2.5, later.left.cy -= 1.8, later.right.cx -= 1.2, later.right.cy += 2.9;
    const auto same = calib::check_camera_convention(later, factory);
    CHECK(same.convention == calib::CameraConvention::exstar);
    const auto old = calib::check_camera_convention(calib::swap_camera_convention(later), factory);
    CHECK(old.convention == calib::CameraConvention::swapped);
    INFO("as is " << old.as_is_px << " px, swapped " << old.swapped_px << " px");
    CHECK(old.swapped_px < 10);
}
