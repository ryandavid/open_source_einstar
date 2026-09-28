// End-to-end without hardware: emulated scanner (speckle rendered through a realistic rig and
// packetised over the USB protocol) -> device layer -> rectification -> stereo -> tracking.

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <map>
#include <mutex>
#include <format>
#include <print>
#include <thread>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/device/einstar_device.hpp"
#include "einstar/pipeline/scan_pipeline.hpp"
#include "einstar/sim/sim_transport.hpp"
#include "einstar/synth/speckle_scene.hpp"

using namespace einstar;
using namespace std::chrono_literals;

namespace {

RigCalibration einstar_like_rig() {
    // Real calibration if EXStar's cache is present, otherwise a rig with the same geometry.
    const char* cache = "/Applications/EXStar.app/Contents/Resources/res/Einscan-E10/200x150";
    if (std::filesystem::exists(std::string(cache) + "/LeftCCF.txt"))
        if (auto cal = calib::load_ccf_directory(cache)) return cal->rig();
    RigCalibration rig;
    rig.left.width = rig.right.width = 1280;
    rig.left.height = rig.right.height = 1024;
    rig.left.fx = rig.left.fy = 1157.3;
    rig.left.cx = 625.4;
    rig.left.cy = 522.4;
    rig.left.dist = {-0.156, 0.158, 0, 0.0003, 0.039};
    rig.right = rig.left;
    rig.right.cx = 633.8;
    rig.right.cy = 506.0;
    SE3 T = SE3::Identity();
    T.linear() = Eigen::AngleAxisd(-22.15 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
    T.translation() = -T.linear() * Vec3(156.9, 0.2, -30.7);
    rig.T_right_left = T;
    return rig;
}

// Scanner pose (left IR camera in world) for a frame: slow arc around the object.
SE3 truth_pose(std::uint32_t frame) {
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

}  // namespace

TEST_CASE("emulated scanner through the full live pipeline tracks the true trajectory") {
    const RigCalibration rig = einstar_like_rig();
    const SE3 T_left_right = rig.T_right_left.inverse();

    synth::Scene scene;
    scene.primitives.push_back(synth::Plane{Vec3(0, 70, 0), Vec3(0, -1, 0)});
    // Cluttered, asymmetric arrangement so every view constrains all six degrees of freedom.
    auto box = [&](Vec3 c, Vec3 half, double yaw_deg, double tilt_deg) {
        SE3 T = SE3::Identity();
        T.linear() = (Eigen::AngleAxisd(yaw_deg * M_PI / 180, Vec3::UnitY()) *
                      Eigen::AngleAxisd(tilt_deg * M_PI / 180, Vec3::UnitX())).toRotationMatrix();
        T.translation() = c;
        scene.primitives.push_back(synth::Box{T, half});
    };
    scene.primitives.push_back(synth::Sphere{Vec3(-80, 20, 20), 45.0});
    box(Vec3(-10, 45, 40), Vec3(25, 25, 18), 30, 0);
    box(Vec3(-150, 50, -10), Vec3(20, 20, 30), -20, 0);
    box(Vec3(-60, 55, -70), Vec3(35, 15, 12), 55, 0);
    box(Vec3(-120, 30, 70), Vec3(12, 40, 12), 10, 15);
    scene.primitives.push_back(synth::Sphere{Vec3(20, 55, -40), 15.0});
    synth::Projector proj;
    proj.model.fx = proj.model.fy = 800;
    proj.model.cx = 640;
    proj.model.cy = 400;
    proj.pattern = synth::DotPattern::random(1280, 800, 9000, 3.5, 11);

    sim::SimConfig cfg;
    auto transport = std::make_unique<sim::SimTransport>(cfg);
    auto* sim = transport.get();
    sim->set_frame_provider([&](int sensor, std::uint32_t frame_id, ImageU8& out) {
        const SE3 T_wl = truth_pose(frame_id);
        const SE3 T_wr = T_wl * T_left_right;
        synth::Projector p = proj;
        p.T_world_projector = T_wl;
        p.T_world_projector.translation() = T_wl * (0.5 * T_left_right.translation());
        p.T_world_projector.linear() = T_wl.linear() * Eigen::AngleAxisd(-11.07 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
        synth::RenderParams rp;
        rp.supersample = 1;
        rp.seed = frame_id * 3 + static_cast<std::uint32_t>(sensor);
        const auto view = synth::render_view(scene, p, sensor == 0 ? rig.left : rig.right, sensor == 0 ? T_wl : T_wr, rp);
        out = view.image;
    });

    device::ConnectOptions opts;
    opts.heartbeat_ms = 0;
    auto dev = device::EinstarDevice::connect(std::move(transport), opts);
    REQUIRE(dev.has_value());

    std::mutex m;
    std::map<std::uint64_t, std::pair<bool, SE3>> results;
    pipeline::LiveStats last_stats;
    auto frontend = std::make_unique<pipeline::StereoFrontend>(rig);
    const SE3 T_left_rect = frontend->rectification().T_left_rectified();
    pipeline::ScanPipelineParams pp;
    pp.block_when_full = true;
    pipeline::ScanPipeline pipe(std::move(frontend), pp, [&](pipeline::LiveUpdate&& u) {
        std::lock_guard lock(m);
        results[u.frame_id] = {u.accepted, *u.pose};
        last_stats = u.stats;
    });
    pipe.start();
    // Paced so the CPU pipeline keeps up while the emulator renders on the same cores (no drops).
    REQUIRE((*dev)->configure_scan_mode(150000).has_value());  // paced by pipeline backpressure
    REQUIRE((*dev)->start_stream([&](usb::FrameGroup&& g) { pipe.push(std::move(g)); }).has_value());

    const auto deadline = std::chrono::steady_clock::now() + 90s;
    while (std::chrono::steady_clock::now() < deadline) {
        {
            std::lock_guard lock(m);
            if (results.size() >= 30) break;
        }
        std::this_thread::sleep_for(50ms);
    }
    (*dev)->stop_stream();
    pipe.stop();

    std::lock_guard lock(m);
    REQUIRE(results.size() >= 20);
    // Align the first processed frame, then compare relative motion to ground truth.
    const auto first = results.begin();
    const SE3 truth0 = truth_pose(static_cast<std::uint32_t>(first->first)) * T_left_rect;
    const SE3 est0 = first->second.second;
    int accepted = 0;
    double max_t = 0, max_r = 0;
    for (const auto& [id, res] : results) {
        if (!res.first) continue;
        ++accepted;
        const SE3 truth = truth0.inverse() * truth_pose(static_cast<std::uint32_t>(id)) * T_left_rect;
        const SE3 est = est0.inverse() * res.second;
        const SE3 e = truth.inverse() * est;
        if (id % 5 == 0)
            std::println("  frame {:3}: truth moved {:6.1f} mm {:5.1f} deg | est moved {:6.1f} mm {:5.1f} deg | err {:6.1f} mm", id,
                         translation_norm(truth), rotation_angle(truth) * 180 / M_PI, translation_norm(est),
                         rotation_angle(est) * 180 / M_PI, translation_norm(e));
        max_t = std::max(max_t, translation_norm(e));
        max_r = std::max(max_r, rotation_angle(e) * 180 / M_PI);
    }
    const auto ts = (*dev)->transport_stats();
    std::string ids;
    for (const auto& [id, r] : results) ids += std::format("{} ", id);
    std::println("processed frame ids: {}", ids);
    std::println("pipeline: in {} processed {} dropped {} stereo {:.1f} ms track {:.1f} ms", last_stats.frames_in,
                 last_stats.frames_processed, last_stats.dropped, last_stats.stereo_ms, last_stats.track_ms);
    std::println("e2e: {} groups processed, {} tracked, max drift {:.2f} mm {:.3f} deg; {} stream packets, resyncs {}",
                 results.size(), accepted, max_t, max_r, ts.stream_packets, (*dev)->stream_stats().resyncs);
    // The whole chain must work (no packet loss, every frame tracked) and drift must stay bounded.
    // This sweep constantly enters new territory with a young single-pass model, so it measures
    // open-loop drift (~0.03 deg/frame); drift over a full scan is removed by the process step's
    // pose-graph optimisation, not by live tracking.
    CHECK((*dev)->stream_stats().resyncs == 0);
    CHECK(last_stats.dropped == 0);
    CHECK(accepted >= static_cast<int>(results.size()) - 1);
    CHECK(max_t < 8.0);
    CHECK(max_r < 3.0);  // known: ~2 deg over this 32-frame open-loop sweep; see docs/status.md
}
