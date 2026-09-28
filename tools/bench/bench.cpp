// Stage-by-stage timing of the depth + tracking path (CPU reference vs GPU once ported).
#include <filesystem>
#include <array>
#include <format>
#include <functional>
#include <print>
#include <string_view>
#include <vector>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/depth/point_image.hpp"
#include "einstar/depth/stereo.hpp"
#include "einstar/fixtures/exstar_project.hpp"
#include "einstar/pipeline/stereo_frontend.hpp"
#include "einstar/synth/speckle_scene.hpp"
#include "einstar/track/icp.hpp"
#include "einstar/track/tracker.hpp"
#include "einstar/track/tsdf.hpp"
#include "einstar/track_metal/metal_icp.hpp"
#include "einstar/track_metal/metal_tsdf.hpp"

using namespace einstar;

static void time_it(const char* name, int n, const std::function<void()>& f) {
    f();  // warm-up
    TimingStats st(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        Stopwatch sw;
        f();
        st.add(sw.elapsed_ms());
    }
    std::println("{:28} median {:7.2f} ms  p95 {:7.2f} ms", name, st.median(), st.percentile(0.95));
}

// Full per-frame path (frontend + tracker) on pre-rendered emulated frames of a sweep.
static void bench_pipeline(const RigCalibration& rig) {
    const SE3 T_lr = rig.T_right_left.inverse();
    synth::Scene scene;
    scene.primitives.push_back(synth::Plane{Vec3(0, 70, 0), Vec3(0, -1, 0)});
    scene.primitives.push_back(synth::Sphere{Vec3(-80, 20, 20), 45.0});
    SE3 B = SE3::Identity();
    B.linear() = Eigen::AngleAxisd(0.5, Vec3::UnitY()).toRotationMatrix();
    B.translation() = Vec3(-10, 45, 40);
    scene.primitives.push_back(synth::Box{B, Vec3(25, 25, 18)});
    synth::Projector proj;
    proj.model.fx = proj.model.fy = 800;
    proj.model.cx = 640;
    proj.model.cy = 400;
    proj.pattern = synth::DotPattern::random(1280, 800, 9000, 3.5, 11);
    const double half_toe = 0.5 * rotation_angle(rig.T_right_left);
    std::vector<std::pair<ImageU8, ImageU8>> frames;
    for (int f = 0; f < 40; ++f) {
        const double t = 0.05 * f;
        const Vec3 eye(-190 + 55 * t, -130 - 12 * t, -240);
        const Vec3 target(-100 + 30 * t, 25, 10 + 10 * t);
        const Vec3 fwd = (target - eye).normalized();
        const Vec3 right = -Vec3(0, 1, 0).cross(fwd).normalized();
        SE3 T = SE3::Identity();
        T.linear().col(0) = right;
        T.linear().col(1) = fwd.cross(right);
        T.linear().col(2) = fwd;
        T.translation() = eye;
        T.linear() = T.linear() * Eigen::AngleAxisd(half_toe, Vec3::UnitY()).toRotationMatrix();
        synth::Projector p = proj;
        p.T_world_projector = T;
        p.T_world_projector.translation() = T * (0.5 * T_lr.translation());
        synth::RenderParams rp;
        rp.supersample = 1;
        rp.seed = static_cast<std::uint32_t>(f);
        frames.emplace_back(synth::render_view(scene, p, rig.left, T, rp).image,
                            synth::render_view(scene, p, rig.right, T * T_lr, rp).image);
    }
    pipeline::StereoFrontend fe(rig);
    auto ctx = gpu::Context::create();
    auto vol = track_metal::MetalTsdfVolume::create(*ctx);
    auto icp = track_metal::MetalIcp::create(*ctx);
    track::Tracker tracker({}, std::move(*vol));
    tracker.set_icp_solver((*icp)->as_function());
    TimingStats st(64), tt(64), total(64);
    for (std::size_t i = 0; i < frames.size(); ++i) {
        Stopwatch sw;
        auto d = fe.process(frames[i].first, frames[i].second);
        const double s_ms = sw.elapsed_ms();
        d.frame.timestamp_s = 0.068 * static_cast<double>(i);
        d.frame.index = i;
        const auto r = tracker.process(d.frame);
        if (i >= 5) {
            st.add(s_ms);
            tt.add(r.ms);
            total.add(sw.elapsed_ms());
        }
    }
    std::println("== full frame path (GPU, emulated sweep, {} frames) ==", frames.size());
    std::println("{:28} median {:7.2f} ms  p95 {:7.2f} ms", "frontend", st.median(), st.percentile(0.95));
    std::println("{:28} median {:7.2f} ms  p95 {:7.2f} ms", "tracker", tt.median(), tt.percentile(0.95));
    std::println("{:28} median {:7.2f} ms  p95 {:7.2f} ms", "total", total.median(), total.percentile(0.95));
}

int main(int argc, char** argv) {
    if (argc > 1 && std::string_view(argv[1]) == "pipeline") {
        auto cal = calib::load_ccf_directory("/Applications/EXStar.app/Contents/Resources/res/Einscan-E10/200x150");
        if (!cal) return 1;
        bench_pipeline(cal->rig());
        return 0;
    }
    auto cal = calib::load_ccf_directory("/Applications/EXStar.app/Contents/Resources/res/Einscan-E10/200x150");
    if (!cal) {
        std::println(stderr, "needs EXStar calibration cache");
        return 1;
    }
    const auto rig = cal->rig();
    const SE3 T_lr = rig.T_right_left.inverse();
    synth::Scene scene;
    scene.primitives.push_back(synth::Plane{Vec3(0, 70, 0), Vec3(0, -1, 0)});
    scene.primitives.push_back(synth::Sphere{Vec3(-80, 20, 20), 45.0});
    synth::Projector proj;
    proj.model.fx = proj.model.fy = 800;
    proj.model.cx = 640;
    proj.model.cy = 400;
    proj.pattern = synth::DotPattern::random(1280, 800, 9000, 3.5, 11);
    SE3 T_wl = SE3::Identity();
    T_wl.linear() = Eigen::AngleAxisd(11.07 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
    T_wl.translation() = Vec3(-160, -40, -260);
    proj.T_world_projector = T_wl;
    proj.T_world_projector.translation() = T_wl * (0.5 * T_lr.translation());
    synth::RenderParams rp;
    rp.supersample = 1;
    const auto l = synth::render_view(scene, proj, rig.left, T_wl, rp);
    const auto r = synth::render_view(scene, proj, rig.right, T_wl * T_lr, rp);

    pipeline::StereoFrontend fe(rig);
    const auto& rect = fe.rectification();
    const auto map_l = calib::build_remap(rig.left, rect.R_left, rect.rectified);
    ImageU8 rl, rr, hl, hr;
    std::println("== stereo (1280x1024 raw -> 640x512 depth) ==");
    time_it("remap x2 (full res)", 20, [&] {
        rl = calib::remap(l.image.view(), map_l);
        rr = calib::remap(r.image.view(), map_l);
    });
    time_it("downsample x2", 20, [&] {
        hl = depth::downsample2(rl.view());
        hr = depth::downsample2(rr.view());
    });
    const auto ql = depth::downsample2(hl.view()), qr = depth::downsample2(hr.view());
    depth::StereoParams sp;
    sp.pyramid_levels = 1;
    const auto& g = rect.geometry;
    sp.sgm.min_disparity = static_cast<int>(g.disparity_from_depth(700) / 4) - 2;
    sp.sgm.num_disparities = static_cast<int>(g.disparity_from_depth(150) / 4) - sp.sgm.min_disparity + 4;
    time_it("SGM quarter res", 10, [&] { (void)depth::sgm_disparity(ql.view(), qr.view(), sp.sgm, true); });
    depth::StereoResult sr;
    time_it("compute_disparity total", 10, [&] { sr = depth::compute_disparity(hl.view(), hr.view(), sp); });
    time_it("disparity_to_points", 20, [&] { (void)depth::disparity_to_points(sr.disparity, sr.confidence, g); });
    time_it("frontend.process total", 10, [&] { (void)fe.process(l.image, r.image); });

    const auto mustang = std::filesystem::path(std::getenv("HOME")) / "Documents/EXStar/mustang_differential/Project1.ir_E10_prj";
    auto proj_f = fixtures::ExstarProject::open(mustang);
    if (!proj_f) return 0;
    std::println("== tracking (EXStar fixture frames, model from 60 frames) ==");
    track::TsdfVolume vol;
    track::Intrinsics k;
    track::DepthFrame df;
    SE3 pose;
    for (std::size_t i = 0; i < 60; ++i) {
        auto f = (*proj_f)->read_frame(i);
        k = {640, 512, f->intrinsics.fx, f->intrinsics.fy, f->intrinsics.cx, f->intrinsics.cy};
        df = track::make_depth_frame(f->depth, k);
        pose = f->T_world_camera;
        vol.integrate(df, pose);
    }
    std::println("model bricks {}", vol.brick_count());
    track::RaycastResult model;
    time_it("raycast 320x256", 20, [&] { model = vol.raycast(pose, k.scaled(0.5)); });
    time_it("icp (3 levels)", 20, [&] { (void)track::icp_point_to_plane(df, model, pose, pose, {}); });
    time_it("integrate", 20, [&] { vol.integrate(df, pose); });
    time_it("make_depth_frame", 20, [&] { (void)track::make_depth_frame((*proj_f)->read_frame(60)->depth, k); });
    const auto dirty = vol.bricks_updated_since(vol.frame_counter() - 1);
    time_it("extract dirty bricks", 10, [&] { (void)vol.extract_points(dirty); });

    std::println("== tracking, Metal volume ==");
    auto ctx = gpu::Context::create();
    auto mv = track_metal::MetalTsdfVolume::create(*ctx);
    if (!mv) return 1;
    for (std::size_t i = 0; i < 60; ++i) {
        auto f = (*proj_f)->read_frame(i);
        (*mv)->integrate(track::make_depth_frame(f->depth, k), f->T_world_camera);
    }
    std::println("model bricks {}", (*mv)->brick_count());
    time_it("raycast 320x256", 20, [&] { model = (*mv)->raycast(pose, k.scaled(0.5)); });
    time_it("icp (3 levels)", 20, [&] { (void)track::icp_point_to_plane(df, model, pose, pose, {}); });
    time_it("integrate", 20, [&] { (*mv)->integrate(df, pose); });
    time_it("extract all", 10, [&] { (void)(*mv)->extract_points(0); });
    auto gicp = track_metal::MetalIcp::create(*ctx);
    if (gicp) {
        for (const auto iters : {std::array<int, 3>{1, 1, 1}, std::array<int, 3>{5, 5, 5}, std::array<int, 3>{15, 10, 8}}) {
            track::IcpParams ip;
            ip.iterations = iters;
            TimingStats gt(20);
            time_it(std::format("gpu icp iters {}/{}/{}", iters[0], iters[1], iters[2]).c_str(), 20, [&] {
                (void)(*gicp)->solve(df, model, pose, pose, ip);
                gt.add((*gicp)->last_gpu_ms());
            });
            std::println("{:28} gpu-side median {:.2f} ms", "", gt.median());
        }
    }
    return 0;
}
