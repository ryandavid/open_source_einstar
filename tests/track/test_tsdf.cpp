#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <print>

#include "einstar/track/icp.hpp"
#include "einstar/track/tracker.hpp"
#include "einstar/track/tsdf.hpp"

using namespace einstar;
using namespace einstar::track;

namespace {

const Intrinsics kK{320, 256, 290.0, 290.0, 160.0, 128.0};

// Depth image of a scene made of a tilted plane with a bump, seen from `T_world_camera`.
ImageF32 render_depth(const SE3& T_world_camera, const Intrinsics& k) {
    ImageF32 d(k.width, k.height, 0.0f);
    const SE3 T_cw = T_world_camera.inverse();
    for (int v = 0; v < k.height; ++v)
        for (int u = 0; u < k.width; ++u) {
            const Vec3 dir_c((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0);
            const Vec3 o = T_world_camera.translation();
            const Vec3 dir = T_world_camera.linear() * dir_c;
            // March to an implicit surface: tilted plane with two bumps and a ridge (no symmetry axis).
            double t = 100, prev_f = 0;
            bool have = false;
            for (int i = 0; i < 2000; ++i, t += 0.5) {
                const Vec3 p = o + t * dir;
                const double bump1 = 15 * std::exp(-(p.x() * p.x() + p.y() * p.y()) / 800.0);
                const double bump2 = -8 * std::exp(-((p.x() - 35) * (p.x() - 35) + (p.y() + 25) * (p.y() + 25)) / 300.0);
                const double ridge = 5 * std::exp(-(p.y() - 0.5 * p.x() - 30) * (p.y() - 0.5 * p.x() - 30) / 50.0);
                const double f = p.z() - (300 + 0.2 * p.x() + bump1 + bump2 + ridge);
                if (have && prev_f < 0 && f >= 0) {
                    const double th = t - 0.5 * f / (f - prev_f);
                    d(u, v) = static_cast<float>((T_cw * (o + th * dir)).z());
                    break;
                }
                prev_f = f;
                have = true;
            }
        }
    return d;
}

}  // namespace

TEST_CASE("TSDF raycast reproduces the integrated surface") {
    TsdfVolume vol;
    const SE3 pose = SE3::Identity();
    const auto depth = render_depth(pose, kK);
    const auto frame = make_depth_frame(depth, kK);
    vol.integrate(frame, pose);
    REQUIRE(vol.brick_count() > 100);

    const auto ray = vol.raycast(pose, kK);
    int valid = 0, good = 0, truth = 0;
    for (int v = 0; v < kK.height; ++v)
        for (int u = 0; u < kK.width; ++u) {
            if (depth(u, v) > 0) ++truth;
            if (!ray.valid(u, v)) continue;
            ++valid;
            if (depth(u, v) > 0 && std::abs(ray.points(u, v).z() - depth(u, v)) < 0.3f) ++good;
        }
    std::println("raycast valid {} / {} truth, {} within 0.3 mm", valid, truth, good);
    REQUIRE(valid > truth * 0.8);
    REQUIRE(good > valid * 0.95);

    const auto pts = vol.extract_points();
    REQUIRE(pts.size() > 1000);
}

TEST_CASE("ICP recovers a small camera motion against the model") {
    TsdfVolume vol;
    const SE3 pose0 = SE3::Identity();
    vol.integrate(make_depth_frame(render_depth(pose0, kK), kK), pose0);

    Vec6 xi;
    xi << 1.5, -1.0, 2.0, 0.01, -0.015, 0.008;
    const SE3 pose1 = se3_exp(xi);
    const auto frame1 = make_depth_frame(render_depth(pose1, kK), kK);
    const auto model = vol.raycast(pose0, kK);
    IcpParams no_prior;
    no_prior.prior_sigma_mm = 1e6;
    no_prior.prior_sigma_deg = 1e6;
    const auto r = icp_point_to_plane(frame1, model, pose0, pose0, no_prior);
    REQUIRE(r.converged);
    const SE3 err = pose1.inverse() * r.T_world_camera;
    std::println("icp: rms {:.3f} inliers {:.2f} err {:.3f} mm {:.4f} deg eig ratio {:.2e}", r.rms_mm, r.inlier_ratio,
                 translation_norm(err), rotation_angle(err) * 180 / M_PI, r.min_eigenvalue_ratio);
    CHECK(translation_norm(err) < 0.1);
    CHECK(rotation_angle(err) * 180 / M_PI < 0.02);
}

TEST_CASE("a scan does not start on a frame without depth (the scanner's first frames are dark)") {
    Tracker tracker;
    auto frame_at = [](const SE3& pose, double t, bool dark) {
        auto f = make_depth_frame(dark ? ImageF32(kK.width, kK.height, 0.0f) : render_depth(pose, kK), kK);
        f.timestamp_s = t;
        return f;
    };
    Vec6 step;
    step << 0.3, -0.2, 0.2, 0.002, -0.001, 0.0015;
    for (int i = 0; i < 3; ++i) {
        const auto r = tracker.process(frame_at(SE3::Identity(), 0.0227 * i, true));
        CHECK(!r.accepted);
        CHECK(r.reason.starts_with("waiting for depth"));
    }
    int accepted = 0;
    for (int i = 0; i < 8; ++i) accepted += tracker.process(frame_at(se3_exp(step * i), 0.07 + 0.0227 * i, false)).accepted;
    CHECK(accepted == 8);  // started on the first frame with depth and tracked from there
}

TEST_CASE("marker-only capture does not start without markers") {
    TrackerParams p;
    p.fuse_surface = false;  // global-marker capture
    Tracker tracker(p);
    auto f = make_depth_frame(render_depth(SE3::Identity(), kK), kK);
    const auto r = tracker.process(f);
    CHECK(!r.accepted);
    CHECK(r.reason.starts_with("waiting for 3 markers"));
    for (int i = 0; i < 4; ++i) f.markers.push_back({Vec3(-30 + 20.0 * i, 10.0 * (i % 2), 300), Vec3(0, 0, -1), 6.0});
    CHECK(tracker.process(f).accepted);
}

