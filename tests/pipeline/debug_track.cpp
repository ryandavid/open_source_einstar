// Debug helper: render the e2e trajectory, run frontend + tracker directly, print per-frame ICP stats.
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <print>
#include <unordered_map>
#include <vector>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/pipeline/stereo_frontend.hpp"
#include "einstar/synth/speckle_scene.hpp"
#include "einstar/track/icp.hpp"
#include "einstar/track/tracker.hpp"

using namespace einstar;

static SE3 truth_pose(std::uint32_t frame) {
    // Hand-held style sweep: the scanner slides sideways and rises while its aim drifts, so the
    // whole view (plane, sphere and boxes) moves in the camera frame.
    const double t = 0.05 * frame;
    const Vec3 eye(-190 + 55 * t + 15 * std::sin(1.3 * t), -130 - 12 * t, -240 + 20 * std::sin(0.7 * t));
    const Vec3 target(-100 + 30 * t, 25 + 8 * std::sin(0.9 * t), 10 + 10 * t);
    const Vec3 fwd = (target - eye).normalized();
    const Vec3 right = Vec3(0, 1, 0).cross(fwd).normalized() * -1.0;
    const Vec3 down = fwd.cross(right);
    SE3 T = SE3::Identity();
    T.linear().col(0) = right;
    T.linear().col(1) = down;
    T.linear().col(2) = fwd;
    T.translation() = eye;
    // Toe-in: the left camera looks ~11 deg to the right of the rig's bisector.
    T.linear() = T.linear() * Eigen::AngleAxisd(11.07 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
    return T;
}

int main(int argc, char** argv) {
    const int mode = argc > 1 ? std::atoi(argv[1]) : 0;
    auto cal = calib::load_ccf_directory(EINSTAR_TEST_CALIBRATION_DIR);
    if (!cal) return 1;
    const auto rig = cal->rig();
    const SE3 T_left_right = rig.T_right_left.inverse();
    synth::Scene scene;
    scene.primitives.push_back(synth::Plane{Vec3(0, 70, 0), Vec3(0, -1, 0)});
    auto box = [&](Vec3 c, Vec3 half, double yaw_deg, double tilt_deg) {
        SE3 T = SE3::Identity();
        T.linear() = (Eigen::AngleAxisd(yaw_deg * M_PI / 180, Vec3::UnitY()) * Eigen::AngleAxisd(tilt_deg * M_PI / 180, Vec3::UnitX())).toRotationMatrix();
        T.translation() = c;
        scene.primitives.push_back(synth::Box{T, half});
    };
    scene.primitives.push_back(synth::Sphere{Vec3(-80, 20, 20), 45.0});
    box(Vec3(-10, 45, 40), Vec3(25, 25, 18), 30, 0);
    box(Vec3(-150, 50, -10), Vec3(20, 20, 30), -20, 0);
    box(Vec3(-60, 55, -70), Vec3(35, 15, 12), 55, 0);
    box(Vec3(-120, 30, 70), Vec3(12, 40, 12), 10, 15);
    synth::Projector proj;
    proj.model.fx = proj.model.fy = 800;
    proj.model.cx = 640;
    proj.model.cy = 400;
    proj.pattern = synth::DotPattern::random(1280, 800, 9000, 3.5, 11);
    pipeline::StereoFrontend fe(rig);
    const SE3 T_left_rect = fe.rectification().T_left_rectified();
    if (mode == 9) {
        // Overlap of frames 0 and 10 placed with the TRUE poses: validates the truth convention.
        std::vector<Vec3f> world[2];
        int slot = 0;
        for (std::uint32_t f : {0u, 10u}) {
            const SE3 T_wl = truth_pose(f);
            synth::Projector p = proj;
            p.T_world_projector = T_wl;
            p.T_world_projector.translation() = T_wl * (0.5 * T_left_right.translation());
            synth::RenderParams rp;
            rp.supersample = 1;
            const auto l = synth::render_view(scene, p, rig.left, T_wl, rp);
            const auto r = synth::render_view(scene, p, rig.right, T_wl * T_left_right, rp);
            const auto d = fe.process(l.image, r.image);
            const Eigen::Matrix4f T = (T_wl * T_left_rect).matrix().cast<float>();
            for (int y = 0; y < d.frame.points.height(); y += 2)
                for (int x = 0; x < d.frame.points.width(); x += 2)
                    if (d.frame.points(x, y).z() > 0) world[slot].push_back((T * d.frame.points(x, y).homogeneous()).head<3>());
            ++slot;
        }
        // Brute-force-ish NN via voxel hash of frame 0.
        std::unordered_map<std::int64_t, std::vector<Vec3f>> grid;
        auto key = [](const Vec3f& p) {
            return (static_cast<std::int64_t>(std::floor(p.x() / 2)) & 0xFFFFF) << 40 | (static_cast<std::int64_t>(std::floor(p.y() / 2)) & 0xFFFFF) << 20 |
                   (static_cast<std::int64_t>(std::floor(p.z() / 2)) & 0xFFFFF);
        };
        for (const auto& p : world[0]) grid[key(p)].push_back(p);
        std::vector<float> dists;
        for (const auto& q : world[1]) {
            float best = 1e9f;
            for (int dz = -1; dz <= 1; ++dz)
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        auto it = grid.find(key(q + Vec3f(2.0f * dx, 2.0f * dy, 2.0f * dz)));
                        if (it == grid.end()) continue;
                        for (const auto& p : it->second) best = std::min(best, (p - q).norm());
                    }
            if (best < 1e8f) dists.push_back(best);
        }
        std::ranges::sort(dists);
        std::println("true-pose overlap: {} of {} pts within 2 voxels, median NN {:.3f} mm", dists.size(), world[1].size(),
                     dists.empty() ? -1.0f : dists[dists.size() / 2]);
        return 0;
    }
    if (mode == 7) {
        const SE3 T_wl = truth_pose(0);
        synth::Projector p = proj;
        p.T_world_projector = T_wl;
        p.T_world_projector.translation() = T_wl * (0.5 * T_left_right.translation());
        synth::RenderParams rp;
        rp.supersample = 1;
        const auto l = synth::render_view(scene, p, rig.left, T_wl, rp);
        std::FILE* f = std::fopen(argv[2], "wb");
        std::fprintf(f, "P5\n%d %d\n255\n", l.image.width(), l.image.height());
        std::fwrite(l.image.data(), 1, l.image.size(), f);
        std::fclose(f);
        return 0;
    }
    if (mode == 8) {
        // ICP of frame k against a model of frame 0, started at the TRUE relative pose.
        auto render = [&](std::uint32_t f) {
            const SE3 T_wl = truth_pose(f);
            synth::Projector p = proj;
            p.T_world_projector = T_wl;
            p.T_world_projector.translation() = T_wl * (0.5 * T_left_right.translation());
            synth::RenderParams rp;
            rp.supersample = 1;
            const auto l = synth::render_view(scene, p, rig.left, T_wl, rp);
            const auto r = synth::render_view(scene, p, rig.right, T_wl * T_left_right, rp);
            return fe.process(l.image, r.image);
        };
        const auto d0 = render(0);
        track::TsdfVolume vol;
        vol.integrate(d0.frame, SE3::Identity());
        const SE3 truth0 = truth_pose(0) * T_left_rect;
        for (std::uint32_t k : {1u, 3u, 5u}) {
            const auto dk = render(k);
            const SE3 rel = truth0.inverse() * truth_pose(k) * T_left_rect;
            for (const auto& [name, init] : {std::pair{"from truth", rel}, std::pair{"from identity", SE3::Identity()}}) {
                const auto model = vol.raycast(init, dk.frame.intrinsics.scaled(0.5));
                const auto res = track::icp_point_to_plane(dk.frame, model, init, init, {});
                const SE3 e = rel.inverse() * res.T_world_camera;
                std::println("k={} {:13}: rms {:.3f} inl {:.2f} -> err {:.2f} mm {:.3f} deg (truth motion {:.1f} mm {:.2f} deg)", k, name,
                             res.rms_mm, res.inlier_ratio, translation_norm(e), rotation_angle(e) * 180 / M_PI, translation_norm(rel),
                             rotation_angle(rel) * 180 / M_PI);
            }
        }
        return 0;
    }
    track::TrackerParams tp;
    if (mode == 1) { tp.icp.prior_sigma_mm = 1e6; tp.icp.prior_sigma_deg = 1e6; }
    if (mode == 2) { tp.icp.iterations = {30, 20, 15}; }
    if (mode == 3) { tp.icp.prior_sigma_mm = 1e6; tp.icp.prior_sigma_deg = 1e6; tp.icp.iterations = {30, 20, 15}; }
    if (mode == 4) tp.icp.normal_balance_alpha = 0.0;
    if (mode == 5) tp.icp.normal_balance_alpha = 0.5;
    track::Tracker tracker(tp);
    SE3 truth0, est0;
    for (std::uint32_t f = 0; f < 32; ++f) {
        const SE3 T_wl = truth_pose(f);
        synth::Projector p = proj;
        p.T_world_projector = T_wl;
        p.T_world_projector.translation() = T_wl * (0.5 * T_left_right.translation());
        p.T_world_projector.linear() = T_wl.linear() * Eigen::AngleAxisd(-11.07 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
        synth::RenderParams rp;
        rp.supersample = 1;
        rp.seed = f;
        const auto l = synth::render_view(scene, p, rig.left, T_wl, rp);
        const auto r = synth::render_view(scene, p, rig.right, T_wl * T_left_right, rp);
        auto d = fe.process(l.image, r.image);
        d.frame.timestamp_s = 0.068 * f;
        const auto res = tracker.process(d.frame);
        const SE3 truth = T_wl * T_left_rect;
        if (f == 0) { truth0 = truth; est0 = res.T_world_camera; }
        const SE3 tr = truth0.inverse() * truth, es = est0.inverse() * res.T_world_camera;
        const SE3 e = tr.inverse() * es;
        int valid = 0;
        for (const auto& q : d.frame.points.pixels()) valid += q.z() > 0;
        std::println("f{:2} valid {:6} acc {} rms {:.3f} inl {:.2f} cov {:.2f} eig {:.1e} degdirs {} | truth {:5.1f} mm {:4.1f} deg est {:5.1f} mm {:4.1f} deg err {:5.1f} mm {}",
                     f, valid, res.accepted, res.icp.rms_mm, res.icp.inlier_ratio, res.icp.coverage, res.icp.min_eigenvalue_ratio,
                     res.icp.degenerate_directions, translation_norm(tr), rotation_angle(tr) * 180 / M_PI, translation_norm(es),
                     rotation_angle(es) * 180 / M_PI, translation_norm(e), res.reason);
    }
}
