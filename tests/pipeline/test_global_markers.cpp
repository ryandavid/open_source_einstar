// Global markers end to end on synthetic data: marker stickers rendered into the IR images ->
// live detection and stereo -> markers-only constellation capture -> bundle adjustment -> a surface
// scan locked to the fixed map.

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <future>
#include <map>
#include <mutex>
#include <print>
#include <random>
#include <thread>

#include "einstar/pipeline/scan_pipeline.hpp"
#include "einstar/recon/process.hpp"
#include "einstar/session/session.hpp"
#include "synthetic_setup.hpp"

using namespace einstar;
using e2e::truth_pose;

namespace {

std::vector<synth::Marker> scatter_markers(int count, std::uint32_t seed) {
    // Irregular placement on the ground plane (y = 70), like stickers applied by hand.
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ux(-260, 120), uz(-120, 170);
    std::vector<synth::Marker> out;
    for (int tries = 0; tries < 20000 && static_cast<int>(out.size()) < count; ++tries) {
        const Vec3 c(ux(rng), 70.0, uz(rng));
        bool ok = true;
        for (const auto& m : out) ok = ok && (m.center - c).norm() > 24.0;
        if (ok) out.push_back({c, Vec3(0, -1, 0), 6.0, 10.0});
    }
    return out;
}

usb::FrameGroup render_group(const e2e::SyntheticSetup& s, const RigCalibration& rig, std::uint32_t id, std::uint64_t t_us) {
    usb::FrameGroup g;
    g.frame_id = id;
    g.timestamp = t_us;
    for (int sensor = 0; sensor < 2; ++sensor) {
        usb::StreamFrame f;
        f.sensor = sensor;
        f.frame_id = id;
        f.timestamp = t_us;
        e2e::render_sensor(s, rig, truth_pose(id), sensor, id * 3 + static_cast<std::uint32_t>(sensor), f.pixels);
        g.sensors[static_cast<std::size_t>(sensor)] = std::move(f);
    }
    return g;
}

}  // namespace

TEST_CASE("global markers: capture, bundle adjustment and a surface scan locked to the map") {
    const RigCalibration rig = e2e::einstar_like_rig();
    auto setup = e2e::make_scene();
    setup.scene.markers = scatter_markers(45, 5);

    auto frontend = std::make_unique<pipeline::StereoFrontend>(rig);
    const SE3 T_left_rect = frontend->rectification().T_left_rectified();
    pipeline::ScanPipelineParams pp;
    pp.block_when_full = true;
    pp.keyframe_translation_mm = 12.0;

    struct Seen {
        bool accepted;
        SE3 pose;
        pipeline::LiveStats stats;
        std::size_t marker_instances;
    };
    std::mutex m;
    std::map<std::uint64_t, Seen> results;
    pipeline::ScanPipeline pipe(std::move(frontend), pp, [&](pipeline::LiveUpdate&& u) {
        std::lock_guard lock(m);
        results[u.frame_id] = {u.accepted, *u.pose, u.stats, u.markers.size()};
    });
    const auto rec_dir = std::filesystem::temp_directory_path() / "einstar_e2e_recording";
    std::filesystem::remove_all(rec_dir);
    pipe.set_recording_directory(rec_dir.string());
    pipe.start();

    // 1. Constellation capture: a markers-only pass over the scene (nothing is fused).
    pipe.set_phase(pipeline::ScanPhase::global_markers);
    std::uint64_t t = 0;
    const std::uint32_t first = 0;
    for (std::uint32_t id = first; id <= 60; id += 3, t += 68000 * 3) pipe.push(render_group(setup, rig, id, t));
    std::promise<pipeline::GlobalMarkerReport> done;
    pipe.optimize_global_markers([&](const pipeline::GlobalMarkerReport& r) { done.set_value(r); });
    const auto rep = done.get_future().get();

    int capture_tracked = 0, seen_sum = 0, frames = 0;
    {
        std::lock_guard lock(m);
        for (const auto& [id, s] : results) {
            ++frames;
            capture_tracked += s.accepted;
            std::println("  capture frame {:3}: {} markers {} matched {} map {} {}", id, s.accepted ? "tracked" : "lost   ",
                         s.stats.markers_in_frame, s.stats.markers_matched, s.stats.map_markers, s.stats.reason);
            seen_sum += s.stats.markers_in_frame;
            CHECK(s.stats.model_points == 0);  // nothing fused during the capture
        }
    }
    std::println("capture: {} frames, {} tracked, {:.1f} markers/frame; BA: {} keyframes, {} markers, rms {:.3f} -> {:.3f} px, "
                 "{} outliers, max shift {:.3f} mm {}",
                 frames, capture_tracked, frames ? static_cast<double>(seen_sum) / frames : 0.0, rep.keyframes, rep.markers,
                 rep.bundle.rms_before_px, rep.bundle.rms_after_px, rep.bundle.outliers_removed, rep.max_shift_mm, rep.error);
    REQUIRE(rep.error.empty());
    CHECK(capture_tracked >= frames - 1);
    CHECK(rep.markers >= 15);
    CHECK(rep.bundle.rms_after_px < 0.15);

    // The map lives in the first capture frame's rectified-left camera frame.
    const SE3 T_truth_map = truth_pose(first) * T_left_rect;
    const auto map = pipe.global_markers();
    double max_err = 0, sum_err = 0;
    for (const auto& mk : map) {
        const Vec3 w = T_truth_map * mk.position;
        double best = 1e9;
        for (const auto& s : setup.scene.markers) best = std::min(best, (s.center - w).norm());
        max_err = std::max(max_err, best);
        sum_err += best;
        CHECK(mk.fixed);
    }
    std::println("global map: {} markers, error vs truth mean {:.3f} mm max {:.3f} mm", map.size(),
                 map.empty() ? 0.0 : sum_err / static_cast<double>(map.size()), max_err);
    CHECK(max_err < 0.25);

    // 2. Surface scan: starts by locating itself on the map, then tracks in the map's frame.
    {
        std::lock_guard lock(m);
        results.clear();
    }
    pipe.set_phase(pipeline::ScanPhase::surface);
    for (std::uint32_t id = 10; id < 40; ++id, t += 68000) pipe.push(render_group(setup, rig, id, t));
    for (int i = 0; i < 600; ++i) {
        {
            std::lock_guard lock(m);
            if (results.size() >= 30) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    const auto session_path = pipe.flush_recording();
    pipe.stop();

    // 3. The recorded session through the process step: one world frame (the global map's), the fixed
    //    markers anchor the pose graph, and the mesh lies on the scene.
    {
        REQUIRE(!session_path.empty());
        auto ses = session::SessionReader::open(session_path);
        REQUIRE(ses.has_value());
        CHECK((*ses)->frame_count() == 21 + 30);
        CHECK((*ses)->global_markers().size() == map.size());
        // Mesh error against the analytic scene: mean distance and fraction of samples off it.
        auto mesh_error = [&](const recon::TriangleMesh& mesh) {
            double sum = 0;
            int cnt = 0, off = 0;
            for (std::size_t v = 0; v < mesh.vertices.size(); v += 11) {
                const Vec3 p = T_truth_map * mesh.vertices[v].cast<double>();
                const Vec3 nrm = T_truth_map.linear() * mesh.normals[v].cast<double>();
                double best = 1e9;
                for (const double sg : {1.0, -1.0})
                    if (const auto hit = setup.scene.intersect(p - sg * 2.0 * nrm, sg * nrm)) best = std::min(best, std::abs(hit->t - 2.0));
                if (best > 1.0) {
                    ++off;
                    continue;
                }
                sum += best;
                ++cnt;
            }
            return std::pair{cnt ? sum / cnt : 1e9, static_cast<double>(off) / std::max(1, cnt + off)};
        };
        recon::ProcessParams live_only;
        live_only.optimize_poses = false;
        auto base = recon::process_session(**ses, live_only);
        recon::ProcessParams full;
        full.fragment_frames = 10;
        auto res = recon::process_session(**ses, full);
        REQUIRE(base.has_value());
        REQUIRE(res.has_value());
        const auto [base_mean, base_off] = mesh_error(base->mesh);
        const auto [mean, off] = mesh_error(res->mesh);
        const auto& pr = res->report;
        std::println("process on the recording: {} frames, {} loop edges, {} landmarks ({} fixed), mesh {} triangles; error vs scene: "
                     "live poses {:.3f} mm ({:.1f}% off) -> processed {:.3f} mm ({:.1f}% off)",
                     pr.frames_used, pr.loop_edges, pr.marker_landmarks, (*ses)->global_markers().size(), pr.triangles, base_mean,
                     100 * base_off, mean, 100 * off);
        CHECK(pr.marker_landmarks > 0);
        CHECK(pr.triangles > 10000);
        CHECK(mean < 0.2);               // real (synthetic-speckle) stereo depth, not exact depth
        CHECK(mean < base_mean + 0.02);  // processing must not make an already-anchored scan worse
        CHECK(off < 0.05);
    }
    std::filesystem::remove_all(rec_dir);

    // Every marker the tracker knows (fixed + any added while scanning) must be a real sticker.
    int phantoms = 0;
    for (const auto& mk : pipe.tracker().marker_map().markers()) {
        if (!pipe.tracker().marker_map().confirmed(mk)) continue;  // unconfirmed candidates are never used
        const Vec3 w = T_truth_map * mk.position;
        double best = 1e9;
        for (const auto& s : setup.scene.markers) best = std::min(best, (s.center - w).norm());
        if (best > 1.0) {
            ++phantoms;
            std::println("  phantom marker id {} at ({:.1f}, {:.1f}, {:.1f}) world, {:.1f} mm from the nearest sticker, {} observations, fixed {}",
                         mk.id, w.x(), w.y(), w.z(), best, mk.observations, mk.fixed);
        }
    }
    std::println("final marker map: {} confirmed markers, {} phantoms", pipe.tracker().marker_map().confirmed_count(), phantoms);
    CHECK(phantoms == 0);

    std::lock_guard lock(m);
    int tracked = 0;
    double max_t = 0, max_r = 0;
    std::size_t model_points = 0;
    for (const auto& [id, s] : results) {
        if (!s.accepted) continue;
        ++tracked;
        // Absolute error in the map frame: no per-scan alignment is applied.
        const SE3 truth = T_truth_map.inverse() * truth_pose(static_cast<std::uint32_t>(id)) * T_left_rect;
        const SE3 e = truth.inverse() * s.pose;
        max_t = std::max(max_t, translation_norm(e));
        max_r = std::max(max_r, rotation_angle(e) * 180 / M_PI);
        model_points = std::max(model_points, s.stats.model_points);
        CHECK(s.marker_instances > 0);
    }
    std::println("surface scan on the global map: {}/{} tracked, absolute error max {:.3f} mm {:.3f} deg, {} model points",
                 tracked, results.size(), max_t, max_r, model_points);
    CHECK(tracked >= static_cast<int>(results.size()) - 1);
    CHECK(max_t < 1.0);
    CHECK(max_r < 0.3);
    CHECK(model_points > 0);
}
