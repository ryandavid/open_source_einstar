#include <catch2/catch_test_macros.hpp>

#include <print>
#include <random>

#include "einstar/optim/marker_bundle.hpp"

using namespace einstar;

TEST_CASE("bundle adjustment recovers a marker map from noisy poses and observations") {
    std::mt19937 rng(9);
    std::uniform_real_distribution<double> u(-150, 150);
    std::normal_distribution<double> px(0, 0.08), pose_t(0, 0.6), pose_r(0, 0.002), pt(0, 0.6);
    const depth::RectifiedGeometry g{1157.0, 640.0, 512.0, 159.9};

    std::map<int, Vec3> truth;
    for (int i = 0; i < 40; ++i) truth[i] = Vec3(u(rng), u(rng) * 0.6, 20 * std::sin(u(rng) / 40));
    std::map<int, SE3> poses_true;
    for (int k = 0; k < 50; ++k) {
        const double a = 0.02 * k;
        const Vec3 eye(-120 + 5 * k, -40 + 20 * std::sin(a * 3), -300);
        SE3 T = SE3::Identity();
        T.linear() = (Eigen::AngleAxisd(0.3 * std::sin(a * 2), Vec3::UnitY()) * Eigen::AngleAxisd(0.1 * std::cos(a), Vec3::UnitX()))
                         .toRotationMatrix();
        T.translation() = eye;
        poses_true[k] = T;
    }
    optim::MarkerBundle b;
    b.geometry = g;
    for (const auto& [k, T] : poses_true) {
        for (const auto& [id, X] : truth) {
            const Vec3 p = T.inverse() * X;
            if (p.z() < 150 || p.z() > 700) continue;
            const Vec2 l(g.f * p.x() / p.z() + g.cx + px(rng), g.f * p.y() / p.z() + g.cy + px(rng));
            const Vec2 r(g.f * (p.x() - g.baseline) / p.z() + g.cx + px(rng), g.f * p.y() / p.z() + g.cy + px(rng));
            if (l.x() < 0 || l.x() > 1280 || r.x() < 0 || l.y() < 0 || l.y() > 1024) continue;
            b.observations.push_back({k, id, l, r});
        }
        // Initial poses: truth + drift-like noise (first frame exact: it is the gauge).
        SE3 noisy = T;
        if (k > 0) {
            Vec6 xi;
            xi << pose_t(rng), pose_t(rng), pose_t(rng), pose_r(rng), pose_r(rng), pose_r(rng);
            noisy = se3_exp(xi) * T;
        }
        b.T_world_camera[k] = noisy;
    }
    for (const auto& [id, X] : truth) b.markers[id] = X + Vec3(pt(rng), pt(rng), pt(rng));
    // A few gross outliers (mismatched detections).
    for (int i = 0; i < 5; ++i) {
        auto o = b.observations[static_cast<std::size_t>(i * 37)];
        o.left.x() += 25;
        b.observations.push_back(o);
    }
    double err_before = 0;
    for (const auto& [id, X] : truth) err_before = std::max(err_before, (b.markers[id] - X).norm());
    const auto rep = optim::optimize(b);
    double err_after = 0, sum = 0;
    std::map<int, int> obs_count;
    for (const auto& o : b.observations) ++obs_count[o.marker];
    int scored = 0;
    for (const auto& [id, X] : truth) {
        if (obs_count[id] < 2) continue;  // never (or once) seen: nothing to optimise against
        ++scored;
        const double e = (b.markers[id] - X).norm();
        err_after = std::max(err_after, e);
        sum += e;
    }
    std::println("BA: {} obs, outliers removed {}, rms {:.3f} -> {:.3f} px; marker max error {:.3f} -> {:.3f} mm (mean {:.4f})",
                 rep.observations, rep.outliers_removed, rep.rms_before_px, rep.rms_after_px, err_before, err_after,
                 sum / std::max(1, scored));
    REQUIRE(scored >= 20);
    CHECK(rep.outliers_removed >= 5);
    CHECK(rep.rms_after_px < 0.12);
    CHECK(err_after < 0.1);
}
