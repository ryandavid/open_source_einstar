// Stage-by-stage timing of the depth + tracking path (CPU reference vs GPU once ported).
#include <filesystem>
#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <sys/resource.h>
#include <time.h>
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
#include "einstar/gpu/profile.hpp"
#include "einstar/pipeline/scan_pipeline.hpp"
#include "einstar/synth/demo.hpp"
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

static double process_cpu_ms() {
    rusage u{};
    getrusage(RUSAGE_SELF, &u);
    return (static_cast<double>(u.ru_utime.tv_sec) + static_cast<double>(u.ru_stime.tv_sec)) * 1e3 +
           (static_cast<double>(u.ru_utime.tv_usec) + static_cast<double>(u.ru_stime.tv_usec)) * 1e-3;
}

static double thread_cpu_ms() {
    timespec ts{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return static_cast<double>(ts.tv_sec) * 1e3 + static_cast<double>(ts.tv_nsec) * 1e-6;
}

// Full live path on pre-rendered emulator frames (table scene with marker stickers): wall time and
// CPU time per frame, with the scan pipeline running as in the app (tracking, overlays, recording).
static void bench_pipeline(const RigCalibration& rig) {
    const SE3 T_lr = rig.T_right_left.inverse();
    synth::Scene scene = synth::table_scene();
    scene.markers = synth::scatter_markers(45, 5);
    const synth::Projector proj = synth::speckle_projector();
    const double half_toe = 0.5 * rotation_angle(rig.T_right_left);
    const int n = 60;
    std::vector<usb::FrameGroup> groups;
    for (int f = 0; f < n; ++f) {
        const double t = 0.05 * f;
        SE3 T = synth::look_at(Vec3(-190 + 55 * t, -130 - 12 * t, -240), Vec3(-100 + 30 * t, 25, 10 + 10 * t));
        T.linear() = T.linear() * Eigen::AngleAxisd(half_toe, Vec3::UnitY()).toRotationMatrix();
        synth::Projector p = proj;
        p.T_world_projector = T;
        p.T_world_projector.translation() = T * (0.5 * T_lr.translation());
        p.T_world_projector.linear() = T.linear() * Eigen::AngleAxisd(-half_toe, Vec3::UnitY()).toRotationMatrix();
        synth::RenderParams rp;
        rp.supersample = 1;
        rp.seed = static_cast<std::uint32_t>(f);
        usb::FrameGroup g;
        g.frame_id = static_cast<std::uint32_t>(f);
        g.timestamp = static_cast<std::uint64_t>(f) * 68000;
        for (int sensor = 0; sensor < 2; ++sensor) {
            usb::StreamFrame sf;
            sf.sensor = sensor;
            sf.frame_id = g.frame_id;
            sf.pixels = synth::render_view(scene, p, sensor ? rig.right : rig.left, sensor ? T * T_lr : T, rp).image;
            g.sensors[static_cast<std::size_t>(sensor)] = std::move(sf);
        }
        groups.push_back(std::move(g));
    }

    // Frontend alone: wall vs CPU time of the calling thread (GPU waits do not burn CPU).
    {
        pipeline::StereoFrontend fe(rig);
        TimingStats wall(64), cpu(64);
        double candidates = 0;
        for (int f = 0; f < n; ++f) {
            Stopwatch sw;
            const double c0 = thread_cpu_ms();
            auto d = fe.process(groups[static_cast<std::size_t>(f)]);
            if (d) candidates += d->marker_candidates;
            if (f >= 5) {
                wall.add(sw.elapsed_ms());
                cpu.add(thread_cpu_ms() - c0);
            }
        }
        std::println("== frontend (stereo + markers), {} frames ==", n);
        std::println("{:28} median {:7.2f} ms  p95 {:7.2f} ms", "wall", wall.median(), wall.percentile(0.95));
        std::println("{:28} median {:7.2f} ms  p95 {:7.2f} ms", "calling-thread CPU", cpu.median(), cpu.percentile(0.95));
        std::println("{:28} {:.1f} per frame (both images)", "marker fit candidates", candidates / n);
    }

    // Whole pipeline as in the app; variants show where the CPU time goes.
    auto run = [&](const char* name, bool record, bool markers) {
        const auto dir = std::filesystem::temp_directory_path() / "einstar_bench_recording";
        std::filesystem::remove_all(dir);
        pipeline::ScanPipelineParams pp;
        pp.block_when_full = true;
        pp.tracker.deterministic_relocalisation = false;  // as live: never wait for the relocaliser
        if (std::getenv("EINSTAR_BENCH_NO_GLOBAL_RELOC")) pp.tracker.global_relocalization = false;
        pipeline::StereoFrontendParams fp;
        fp.detect_markers = markers;
        std::atomic<int> done{0};
        pipeline::ScanPipeline pipe(std::make_unique<pipeline::StereoFrontend>(rig, fp), pp, [&](pipeline::LiveUpdate&&) { ++done; });
        if (record) pipe.set_recording_directory(dir.string());
        pipe.start();
        auto wait_all = [&](int count) {
            (void)pipe.flush_recording();  // ordered after every frame pushed so far
            while (done.load() < count) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        };
        for (int f = 0; f < 5; ++f) {  // warm-up: pipelines, sensor order
            auto g = groups[static_cast<std::size_t>(f)];
            pipe.push(std::move(g));
        }
        wait_all(5);
        gpu::profile::reset();
        const double c0 = process_cpu_ms();
        Stopwatch sw;
        for (int f = 5; f < n; ++f) {
            auto g = groups[static_cast<std::size_t>(f)];
            pipe.push(std::move(g));
        }
        wait_all(n);
        const double wall = sw.elapsed_ms(), cpu = process_cpu_ms() - c0;
        pipe.stop();
        std::filesystem::remove_all(dir);
        std::println("{:34} wall {:6.2f} ms/frame, process CPU {:6.2f} ms/frame", name, wall / (n - 5), cpu / (n - 5));
        if (gpu::profile::enabled()) std::print("{}", gpu::profile::report(n - 5));
    };
    std::println("== scan pipeline ({} frames) ==", n - 5);
    run("full (markers, recording)", true, true);
    run("no recording", false, true);
    run("no recording, no markers", false, false);
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
    gpu::profile::reset();
    time_it("extract all", 10, [&] { (void)(*mv)->extract_points(0); });
    if (gpu::profile::enabled()) std::print("{}", gpu::profile::report(11));
    auto gicp = track_metal::MetalIcp::create(*ctx);
    if (gicp) {
        for (const auto iters : {std::array<int, 3>{1, 1, 1}, std::array<int, 3>{5, 5, 5}, std::array<int, 3>{15, 10, 8}}) {
            track::IcpParams ip;
            ip.iterations = iters;
            TimingStats gt(20);
            double its = 0;
            time_it(std::format("gpu icp iters {}/{}/{}", iters[0], iters[1], iters[2]).c_str(), 20, [&] {
                (void)(*gicp)->solve(df, model, pose, pose, ip);
                gt.add((*gicp)->last_gpu_ms());
                its = (*gicp)->last_iterations();
            });
            std::println("{:28} gpu-side median {:.2f} ms, {} iterations ran", "", gt.median(), its);
            if (std::getenv("EINSTAR_ICP_DUMP")) {
                // Exact result of repeated solves (determinism / kernel-change parity).
                for (int rep = 0; rep < 3; ++rep) {
                    const auto r = (*gicp)->solve(df, model, pose, pose, ip);
                    std::print("  pose");
                    for (int e = 0; e < 12; ++e) std::print(" {:a}", static_cast<float>(r.T_world_camera.matrix()(e % 3, e / 3)));
                    std::println("  n {} rms {:a}", r.correspondences, static_cast<float>(r.rms_mm));
                }
            }
        }
    }
    return 0;
}
