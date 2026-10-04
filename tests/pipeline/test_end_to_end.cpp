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
#include "einstar/usb/constants.hpp"
#include "synthetic_setup.hpp"

using namespace einstar;
using namespace std::chrono_literals;
using e2e::einstar_like_rig;
using e2e::truth_pose;

TEST_CASE("emulated scanner through the full live pipeline tracks the true trajectory") {
    const RigCalibration rig = einstar_like_rig();
    const auto setup = e2e::make_scene();

    auto [emulator, transport] = sim::make_sim_scanner();
    emulator->set_frame_provider([&](int sensor, std::uint32_t frame_id, ImageU8& out) {
        // (render_sensor takes the camera: 0 left, 1 right; the stream's left camera is usb::kLeftSensor.)
        const int camera = sensor == usb::kLeftSensor ? 0 : 1;
        e2e::render_sensor(setup, rig, truth_pose(frame_id), camera, frame_id * 3 + static_cast<std::uint32_t>(camera), out);
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
