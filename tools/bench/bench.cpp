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

#include "einstar/calib/convention.hpp"
#include "einstar/calib/device_calibration.hpp"
#include "einstar/calibrate/captures.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/depth/point_image.hpp"
#include "einstar/depth/stereo.hpp"
#include "einstar/fixtures/exstar_project.hpp"
#include "einstar/pipeline/stereo_frontend.hpp"
#include "einstar/gpu/profile.hpp"
#include "einstar/pipeline/scan_pipeline.hpp"
#include "einstar/render/scene_renderer.hpp"
#include "einstar/render/view_camera.hpp"
#include "einstar/session/session.hpp"
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
        pipeline::LiveStats stats;  // written by the pipeline thread, read after stop()
        pipeline::ScanPipeline pipe(std::make_unique<pipeline::StereoFrontend>(rig, fp), pp, [&](pipeline::LiveUpdate&& u) {
            stats = u.stats;
            ++done;
        });
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
        std::println("{:34} wall {:6.2f} ms/frame, process CPU {:6.2f} ms/frame (median stereo {:.1f}, markers {:.1f}, track {:.1f} ms)", name,
                     wall / (n - 5), cpu / (n - 5), stats.stereo_ms, stats.marker_ms, stats.track_ms);
        if (gpu::profile::enabled()) std::print("{}", gpu::profile::report(n - 5));
    };
    std::println("== scan pipeline ({} frames) ==", n - 5);
    run("full (markers, recording)", true, true);
    run("no recording", false, true);
    run("no recording, no markers", false, false);
}

// Tracking as live on a real scan: the recording's depth frames and markers through the tracker (Metal
// volume and ICP), with the model snapshot the pipeline publishes every 250 ms (every 4th frame at 14.7 Hz).
// Real scans have more surface, larger models and motion that synthetic frames do not; this times them.
static int bench_session(const char* path, int count) {
    auto reader = session::SessionReader::open(path);
    if (!reader) {
        std::println(stderr, "{}", reader.error().message);
        return 1;
    }
    const auto& s = **reader;
    auto ctx = gpu::Context::create();
    if (!ctx) return 1;
    auto vol = track_metal::MetalTsdfVolume::create(*ctx);
    auto icp = track_metal::MetalIcp::create(*ctx);
    if (!vol || !icp) return 1;
    const auto* mv = vol->get();
    track::TrackerParams tp;
    tp.deterministic_relocalisation = false;  // as live
    track::Tracker tracker(tp, std::move(*vol));
    tracker.set_icp_solver((*icp)->as_function());
    const auto n = std::min(s.frame_count(), static_cast<std::size_t>(count));
    std::vector<double> track_ms, snap_ms;
    int accepted = 0;
    gpu::profile::reset();
    for (std::size_t i = 0; i < n; ++i) {
        auto rec = s.read(i);
        if (!rec) continue;
        const auto f = rec->depth_frame(s.header().depth_intrinsics);
        Stopwatch sw;
        const auto r = tracker.process(f);
        track_ms.push_back(sw.elapsed_ms());
        accepted += r.accepted;
        if (r.integrated && i % 4 == 0) {
            Stopwatch ss;
            (void)mv->extract_render_points();
            snap_ms.push_back(ss.elapsed_ms());
        }
    }
    const auto pct = [](std::vector<double> v, double p) {
        if (v.empty()) return 0.0;
        std::ranges::sort(v);
        return v[std::min(v.size() - 1, static_cast<std::size_t>(p * static_cast<double>(v.size())))];
    };
    std::println("== tracking on {} ({} frames, {} accepted, {} bricks) ==", path, n, accepted, tracker.volume().brick_count());
    std::println("{:28} median {:7.2f} ms  p95 {:7.2f} ms  max {:7.2f} ms", "track", pct(track_ms, 0.5), pct(track_ms, 0.95), pct(track_ms, 1.0));
    std::println("{:28} median {:7.2f} ms  p95 {:7.2f} ms  max {:7.2f} ms", "model snapshot", pct(snap_ms, 0.5), pct(snap_ms, 0.95), pct(snap_ms, 1.0));
    if (gpu::profile::enabled()) std::print("{}", gpu::profile::report(static_cast<int>(n)));

    // The live view of that model (the app draws at 60 Hz, ~4 times per scanner frame, on the same GPU):
    // following the scanner, at a 1600 x 1000 point window on a 2x display.
    auto renderer = render::SceneRenderer::create(*ctx, MTL::PixelFormatBGRA8Unorm, MTL::PixelFormatDepth32Float);
    if (!renderer) return 1;
    const auto model = mv->extract_render_points();
    (*renderer)->set_model_buffer(model.buffer, model.count);
    render::ViewCamera cam;
    cam.follow(tracker.last_good_pose().matrix().cast<float>(), 1.0f);
    constexpr int W = 3200, H = 2000;
    auto texture = [&](MTL::PixelFormat format) {
        auto* d = MTL::TextureDescriptor::texture2DDescriptor(format, W, H, false);
        d->setUsage(MTL::TextureUsageRenderTarget);
        d->setStorageMode(MTL::StorageModePrivate);
        return gpu::Ref<MTL::Texture>((*ctx)->device()->newTexture(d));
    };
    const auto color = texture(MTL::PixelFormatBGRA8Unorm), depth = texture(MTL::PixelFormatDepth32Float);
    auto* pass = MTL::RenderPassDescriptor::renderPassDescriptor();
    pass->colorAttachments()->object(0)->setTexture(color.get());
    pass->colorAttachments()->object(0)->setLoadAction(MTL::LoadActionClear);
    pass->colorAttachments()->object(0)->setStoreAction(MTL::StoreActionStore);
    pass->depthAttachment()->setTexture(depth.get());
    pass->depthAttachment()->setLoadAction(MTL::LoadActionClear);
    pass->depthAttachment()->setClearDepth(1.0);
    TimingStats draw(20);
    for (int f = 0; f < 21; ++f) {
        auto* cmd = (*ctx)->queue()->commandBuffer();
        auto* enc = cmd->renderCommandEncoder(pass);
        (*renderer)->encode(enc, cam, float(W), float(H), {});  // framebuffer pixels, as the app
        enc->endEncoding();
        cmd->commit();
        cmd->waitUntilCompleted();
        if (f > 0) draw.add((cmd->GPUEndTime() - cmd->GPUStartTime()) * 1e3);
    }
    std::println("{:28} median {:7.2f} ms GPU per draw ({} points, {}x{} px)", "live view", draw.median(), model.count, W, H);
    return 0;
}

// Stereo and markers as live on the scanner's own IR images: `einstar-cli hw-capture` folders
// (<root>/calibration, <root>/<pose>/scan/gNNN_sS.pgm, sensor 0 left as captured then, so the calibration
// is converted to that convention). Real IR has other marker candidates and depth coverage than rendered frames.
static int bench_frontend(const std::filesystem::path& root) {
    auto cal = calib::load_ccf_directory(root / "calibration");
    if (!cal) {
        std::println(stderr, "no calibration in {}", (root / "calibration").string());
        return 1;
    }
    const RigCalibration rig = calib::swap_camera_convention(cal->rig());
    std::vector<usb::FrameGroup> groups;
    for (const auto& pose : std::filesystem::directory_iterator(root)) {
        const auto dir = pose.path() / "scan";
        for (int g = 0; std::filesystem::exists(dir / std::format("g{:03}_s0.pgm", g)); ++g) {
            usb::FrameGroup group;
            group.frame_id = static_cast<std::uint32_t>(groups.size());
            for (int sensor = 0; sensor < 2; ++sensor) {
                auto img = calibrate::read_pgm(dir / std::format("g{:03}_s{}.pgm", g, sensor));
                if (!img) continue;
                usb::StreamFrame sf;
                sf.sensor = sensor;
                sf.frame_id = group.frame_id;
                sf.pixels = std::move(*img);
                group.sensors[static_cast<std::size_t>(sensor)] = std::move(sf);
            }
            groups.push_back(std::move(group));
        }
    }
    if (groups.empty()) {
        std::println(stderr, "no <pose>/scan/gNNN_sS.pgm captures under {}", root.string());
        return 1;
    }
    pipeline::StereoFrontend fe(rig);
    fe.set_left_sensor(0);
    for (int f = 0; f < 3; ++f) (void)fe.process(groups[static_cast<std::size_t>(f) % groups.size()]);  // warm-up
    gpu::profile::reset();
    TimingStats wall(groups.size()), cpu(groups.size()), stereo(groups.size()), markers(groups.size());
    double candidates = 0, found = 0, valid = 0;
    for (const auto& g : groups) {
        Stopwatch sw;
        const double c0 = thread_cpu_ms();
        auto d = fe.process(g);
        wall.add(sw.elapsed_ms());
        cpu.add(thread_cpu_ms() - c0);
        if (!d) continue;
        stereo.add(d->stereo_ms);
        markers.add(d->marker_ms);
        candidates += d->marker_candidates;
        found += static_cast<double>(d->markers.size());
        const auto* dev = d->frame.device.get();
        const float* p = dev->points_xyzw();
        std::size_t n = 0;
        for (std::size_t i = 0; i < static_cast<std::size_t>(dev->width() * dev->height()); ++i) n += p[4 * i + 2] > 0;
        valid += static_cast<double>(n) / (dev->width() * dev->height());
    }
    const double k = static_cast<double>(groups.size());
    std::println("== frontend on real IR ({} frames from {}) ==", groups.size(), root.string());
    std::println("{:28} median {:7.2f} ms  p95 {:7.2f} ms", "wall", wall.median(), wall.percentile(0.95));
    std::println("{:28} median {:7.2f} ms  p95 {:7.2f} ms", "calling-thread CPU", cpu.median(), cpu.percentile(0.95));
    std::println("{:28} median {:7.2f} ms  (markers after it {:.2f} ms)", "stereo", stereo.median(), markers.median());
    std::println("{:28} {:.1f} candidates, {:.1f} stereo markers per frame; valid depth {:.1f}%", "markers", candidates / k, found / k, 100 * valid / k);
    if (gpu::profile::enabled()) std::print("{}", gpu::profile::report(static_cast<int>(groups.size())));
    return 0;
}

// The scanner's calibration: EXStar's cache when installed, else the copy in tests/fixtures.
static RigCalibration bench_rig() {
    if (auto cal = calib::load_ccf_directory("/Applications/EXStar.app/Contents/Resources/res/Einscan-E10/200x150")) return cal->rig();
    if (auto cal = calib::load_ccf_directory(EINSTAR_CALIBRATION_FIXTURE)) return cal->rig();
    std::println(stderr, "no calibration found; using the synthetic Einstar rig");
    return synth::synthetic_einstar_rig();
}

int main(int argc, char** argv) {
    if (argc > 1 && std::string_view(argv[1]) == "pipeline") {
        bench_pipeline(bench_rig());
        return 0;
    }
    if (argc > 2 && std::string_view(argv[1]) == "frontend") return bench_frontend(argv[2]);
    if (argc > 2 && std::string_view(argv[1]) == "session") return bench_session(argv[2], argc > 3 ? std::atoi(argv[3]) : 1 << 30);
    const auto rig = bench_rig();
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
    sp.sgm.min_disparity = static_cast<int>(std::floor(g.disparity_from_depth(700) / 4)) - 2;
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
                    const auto res = (*gicp)->solve(df, model, pose, pose, ip);
                    std::print("  pose");
                    for (int e = 0; e < 12; ++e) std::print(" {:a}", static_cast<float>(res.T_world_camera.matrix()(e % 3, e / 3)));
                    std::println("  n {} rms {:a}", res.correspondences, static_cast<float>(res.rms_mm));
                }
            }
        }
    }
    return 0;
}
