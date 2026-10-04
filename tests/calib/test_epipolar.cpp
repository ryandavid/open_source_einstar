#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <random>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/calib/epipolar.hpp"
#include "einstar/calib/rectify.hpp"

using namespace einstar;

namespace {

constexpr double kRad = M_PI / 180.0;

RigCalibration flash_rig() {
    auto cal = calib::load_ccf_directory(EINSTAR_TEST_CALIBRATION_DIR);
    REQUIRE(cal);
    return cal->rig();
}

// Markers as a scan sees them: points at 230-550 mm over the left image (or the part of it in `region`,
// fractions of the width and height), their centres through `truth` with 0.08 px of noise, and a few
// mismatched pairs.
std::vector<calib::MarkerPair> scan_pairs(const RigCalibration& truth, int n, std::uint32_t seed, Eigen::Vector4d region = {0, 0, 1, 1}) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u(region(0), region(2)), v(region(1), region(3)), z(230, 550), any(0, 1);
    std::normal_distribution<double> noise(0, 0.08);
    std::vector<calib::MarkerPair> out;
    while (static_cast<int>(out.size()) < n) {
        const Vec2 px(u(rng) * (truth.left.width - 1), v(rng) * (truth.left.height - 1));
        const Vec2 nl = calib::undistort_to_normalized(truth.left, px);
        const double d = z(rng);
        const Vec3 p = truth.T_right_left * Vec3(nl.x() * d, nl.y() * d, d);
        Vec2 q = truth.right.project(p);
        if (q.x() < 0 || q.y() < 0 || q.x() > truth.right.width - 1 || q.y() > truth.right.height - 1) continue;
        if (any(rng) < 0.02) q.y() += 3 * (any(rng) - 0.5);  // a neighbour taken for the marker
        out.push_back({px + Vec2(noise(rng), noise(rng)), q + Vec2(noise(rng), noise(rng))});
    }
    return out;
}

}  // namespace

TEST_CASE("turning the right camera about the baseline moves its rows by f times the angle") {
    const RigCalibration rig = flash_rig();
    const auto e = calib::epipolar_frame(rig);
    CHECK(std::abs(e.f - 0.5 * (rig.left.fy + rig.right.fy)) < 1e-9);
    const double angle = 0.015 * kRad;
    const RigCalibration turned = calib::turn_right_camera(rig, angle, 0);
    // About its own centre: the baseline keeps its length, the right camera stays where it is.
    CHECK(std::abs(turned.baseline_mm() - rig.baseline_mm()) < 1e-9);
    const Vec3 c0 = -(rig.T_right_left.linear().transpose() * rig.T_right_left.translation());
    const Vec3 c1 = -(turned.T_right_left.linear().transpose() * turned.T_right_left.translation());
    CHECK((c0 - c1).norm() < 1e-9);
    // Seen through the turned rig, rectified through the original: every row off by about f * angle.
    const auto a = calib::row_agreement(turned, rig);
    CHECK(std::abs(std::abs(a.mean) - e.f * angle) < 0.02);
    CHECK(a.max_abs - std::abs(a.mean) < 0.06);  // (1 + y^2): a little more towards the top and bottom
    // About the optical axis: the rows tilt across the image, opposite at its left and right edges.
    const RigCalibration rolled = calib::turn_right_camera(rig, 0, 0.02 * kRad);
    const Vec2 lo(100, 512), hi(1180, 512);
    auto through = [&](const Vec2& px) {
        const Vec2 n = calib::undistort_to_normalized(rolled.left, px);
        return calib::MarkerPair{px, rolled.right.project(rolled.T_right_left * Vec3(n.x() * 400, n.y() * 400, 400))};
    };
    const double dlo = calib::row_difference(rig, through(lo)), dhi = calib::row_difference(rig, through(hi));
    CHECK(dlo * dhi < 0);
    CHECK(std::abs(dhi - dlo) > 0.5 * 0.02 * kRad * (hi.x() - lo.x()));
}

TEST_CASE("rectified marker centres go back to the raw pixels they came from") {
    const RigCalibration rig = flash_rig();
    const auto rect = calib::compute_rectification(rig);
    for (const auto& p : scan_pairs(rig, 50, 3)) {
        auto to_rect = [&](const CameraModel& cam, const Mat3& R, const Vec2& px) {
            const Vec2 n = calib::undistort_to_normalized(cam, px);
            const Vec3 r = R * Vec3(n.x(), n.y(), 1);
            return Vec2(rect.rectified.fx * r.x() / r.z() + rect.rectified.cx, rect.rectified.fy * r.y() / r.z() + rect.rectified.cy);
        };
        const auto back = calib::raw_marker_pair(rig, rect.R_left, rect.R_right, rect.rectified, to_rect(rig.left, rect.R_left, p.left),
                                                 to_rect(rig.right, rect.R_right, p.right));
        CHECK((back.left - p.left).norm() < 1e-6);
        CHECK((back.right - p.right).norm() < 1e-6);
    }
}

TEST_CASE("the epipolar self-check finds a turned right camera and corrects it") {
    const RigCalibration calibrated = flash_rig();
    // Since its calibration the right camera has turned by 0.015 degrees about the baseline (-0.3 px of
    // rows) and 0.01 about its axis.
    const RigCalibration now = calib::turn_right_camera(calibrated, -0.015 * kRad, 0.01 * kRad);
    const auto pairs = scan_pairs(now, 3000, 7);
    const auto c = calib::check_epipolar(calibrated, pairs);
    INFO(c.verdict);
    REQUIRE(c.apply);
    CHECK(c.optical_axis_fitted);
    CHECK(c.inliers > 2900);
    CHECK(c.cells == 9);
    CHECK(std::abs(c.before.median) > 0.2);
    CHECK(std::abs(c.after.median) < 0.02);
    CHECK(std::abs(c.about_baseline_deg + 0.015) < 0.001);
    CHECK(std::abs(c.about_optical_axis_deg - 0.01) < 0.002);
    const auto a = calib::row_agreement(now, c.corrected);
    CHECK(std::abs(a.mean) < 0.02);
    CHECK(a.max_abs < 0.06);

    // Once corrected, the same markers find nothing more to do.
    const auto again = calib::check_epipolar(c.corrected, pairs);
    CHECK_FALSE(again.apply);
    CHECK(again.verdict.find("agrees") != std::string::npos);
}

TEST_CASE("the epipolar self-check leaves the rig alone without the evidence") {
    const RigCalibration calibrated = flash_rig();
    const RigCalibration now = calib::turn_right_camera(calibrated, -0.015 * kRad, 0);
    SECTION("too few pairs") {
        const auto c = calib::check_epipolar(calibrated, scan_pairs(now, 120, 1));
        CHECK_FALSE(c.apply);
        CHECK(c.corrected.T_right_left.isApprox(calibrated.T_right_left));
    }
    SECTION("markers in one corner of the image only") {
        const auto c = calib::check_epipolar(calibrated, scan_pairs(now, 2000, 2, {0.0, 0.0, 0.3, 0.3}));
        CHECK_FALSE(c.apply);
        CHECK(c.cells < 5);
    }
    SECTION("an error that is not a turn: the right camera's distortion changed") {
        RigCalibration lens = calibrated;
        lens.right.dist[0] += 0.006;
        const auto c = calib::check_epipolar(calibrated, scan_pairs(lens, 3000, 4));
        INFO(c.verdict);
        CHECK_FALSE(c.apply);
    }
    SECTION("too large to be drift") {
        const auto c = calib::check_epipolar(calibrated, scan_pairs(calib::turn_right_camera(calibrated, 0.15 * kRad, 0), 3000, 5), {.gate_px = 5.0});
        CHECK_FALSE(c.apply);
        CHECK(c.verdict.find("recalibrate") != std::string::npos);
    }
}

TEST_CASE("epipolar samples keep the newest pairs of each image region") {
    calib::EpipolarSamples s(1280, 1024, 2, 3);
    for (int i = 0; i < 10; ++i) s.add({Vec2(100, 100), Vec2(static_cast<double>(i), 0)});
    s.add({Vec2(1000, 900), Vec2(42, 0)});
    CHECK(s.size() == 4);
    std::vector<double> kept;
    for (const auto& p : s.pairs()) kept.push_back(p.right.x());
    std::ranges::sort(kept);
    CHECK(kept == std::vector<double>{7, 8, 9, 42});
    s.clear();
    CHECK(s.size() == 0);
}
