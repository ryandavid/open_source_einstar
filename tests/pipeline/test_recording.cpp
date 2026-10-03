// What a live scan records: the scanner, every processed frame with its capture settings and
// tracking diagnostics, a record for every frame that could not be processed, and (optionally) the
// raw IR images of every frame that reached the host.

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <set>
#include <thread>

#include "einstar/pipeline/scan_pipeline.hpp"
#include "einstar/session/session.hpp"
#include "synthetic_setup.hpp"

using namespace einstar;

TEST_CASE("live recording keeps every frame that reaches the host") {
    const RigCalibration rig = e2e::einstar_like_rig();
    const auto setup = e2e::make_scene();
    const auto dir = std::filesystem::temp_directory_path() / "einstar_test_recording";
    std::filesystem::remove_all(dir);

    pipeline::ScanPipelineParams pp;
    pp.queue_capacity = 3;
    pp.record_raw_ir = true;
    pipeline::ScanPipeline pipe(std::make_unique<pipeline::StereoFrontend>(rig), pp, [](pipeline::LiveUpdate&&) {});
    pipe.set_recording_directory(dir.string());  // not started: applied at once
    session::DeviceRecord dev;
    dev.serial = "TESTSERIAL";
    dev.rig = rig;
    pipe.set_device_record(dev);
    session::CaptureSettings cs;
    cs.exposure = {4400, 4400};
    cs.gain = {120, 120};
    cs.laser_percent = 100;
    cs.strobe = 6000;
    cs.trigger_period_us = 68000;
    pipe.set_capture_settings(cs);
    pipe.set_temperature(37.5f);

    // Eight frames arrive before the worker runs: the 3-deep queue keeps the last three.
    std::vector<usb::FrameGroup> sent;
    for (std::uint32_t id = 10; id < 18; ++id) {
        usb::FrameGroup g;
        g.frame_id = id;
        g.timestamp = static_cast<std::uint64_t>(id) * 68000;
        for (int sensor = 0; sensor < 2; ++sensor) {
            usb::StreamFrame f;
            f.sensor = sensor;
            f.frame_id = id;
            e2e::render_sensor(setup, rig, e2e::truth_pose(id), sensor, id * 3 + static_cast<std::uint32_t>(sensor), f.pixels);
            g.sensors[static_cast<std::size_t>(sensor)] = std::move(f);
        }
        sent.push_back(g);
        pipe.push(std::move(g));
    }
    pipe.start();
    const auto path = pipe.flush_recording();
    pipe.stop();
    REQUIRE_FALSE(path.empty());

    auto r = session::SessionReader::open(path);
    REQUIRE(r.has_value());
    REQUIRE((*r)->device().has_value());
    CHECK((*r)->device()->serial == "TESTSERIAL");
    CHECK((*r)->device()->rig.left.fx == rig.left.fx);

    std::set<std::uint64_t> covered;
    REQUIRE((*r)->frame_count() == 3);
    for (std::size_t i = 0; i < (*r)->frame_count(); ++i) {
        const auto& m = (*r)->meta(i);
        covered.insert(m.index);
        REQUIRE(m.extras.has_value());
        CHECK(m.extras->capture.exposure[0] == 4400);
        CHECK(m.extras->capture.strobe == 6000);
        CHECK(m.extras->capture.temperature_c == 37.5f);
        CHECK(m.extras->left_sensor >= 0);
        CHECK(m.extras->tracking.stereo_ms > 0.0f);
    }
    REQUIRE((*r)->dropped().size() == 5);
    for (const auto& d : (*r)->dropped()) {
        CHECK(d.reason == "live queue full");
        covered.insert(d.index);
    }
    CHECK(covered.size() == sent.size());  // every frame is either processed or accounted for

    // Raw IR for all eight, bit-exact.
    REQUIRE((*r)->raw_count() == sent.size());
    for (std::size_t i = 0; i < sent.size(); ++i) {
        auto raw = (*r)->read_raw(i);
        REQUIRE(raw.has_value());
        CHECK(raw->index == sent[i].frame_id);
        REQUIRE(raw->images.size() == 2);
        for (const auto& [sensor, img] : raw->images) {
            const auto& orig = sent[i].sensors[static_cast<std::size_t>(sensor)]->pixels.pixels();
            CHECK(std::equal(img.pixels().begin(), img.pixels().end(), orig.begin(), orig.end()));
        }
    }
    std::filesystem::remove_all(dir);
}

TEST_CASE("a paused scan's erase leaves the live model and the recording, and undo puts it back") {
    const RigCalibration rig = e2e::einstar_like_rig();
    const auto setup = e2e::make_scene();
    const auto dir = std::filesystem::temp_directory_path() / "einstar_test_erase";
    std::filesystem::remove_all(dir);
    pipeline::ScanPipelineParams pp;
    pp.queue_capacity = 16;
    pipeline::ScanPipeline pipe(std::make_unique<pipeline::StereoFrontend>(rig), pp, [](pipeline::LiveUpdate&&) {});
    pipe.set_recording_directory(dir.string());
    for (std::uint32_t id = 10; id < 18; ++id) {
        usb::FrameGroup g;
        g.frame_id = id;
        g.timestamp = static_cast<std::uint64_t>(id) * 68000;
        for (int sensor = 0; sensor < 2; ++sensor) {
            usb::StreamFrame f;
            f.sensor = sensor;
            f.frame_id = id;
            e2e::render_sensor(setup, rig, e2e::truth_pose(id), sensor, id * 3 + static_cast<std::uint32_t>(sensor), f.pixels);
            g.sensors[static_cast<std::size_t>(sensor)] = std::move(f);
        }
        pipe.push(std::move(g));
    }
    pipe.start();
    const auto path = pipe.flush_recording();  // every frame processed: the scan is paused
    REQUIRE_FALSE(path.empty());

    // An edit's result arrives on the worker: wait for it by counting polls (no wall-clock deadline).
    auto run = [&](auto&& issue) {
        std::atomic<bool> got{false};
        pipeline::EditResult out;
        issue([&](pipeline::EditResult r) {
            out = std::move(r);
            got = true;
        });
        for (int polls = 0; !got && polls < 20000; ++polls) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        REQUIRE(got);
        return out;
    };
    // The lasso: world x < 0 (the first frame's camera looks along +z), through all depths.
    Eigen::Matrix4f vp = Eigen::Matrix4f::Identity();
    vp(0, 0) = vp(1, 1) = 0.001f;
    vp(2, 2) = 0.0f;
    LassoSelection west;
    west.add({vp, Eigen::Vector2f(200, 200), {{0, 0}, {100, 0}, {100, 200}, {0, 200}}, false});

    const auto none = run([&](auto done) { pipe.erase(LassoSelection{}, std::move(done)); });
    const std::size_t full = none.model.size();
    CHECK(none.erased_voxels == 0);
    REQUIRE(full > 10000);
    const auto first = run([&](auto done) { pipe.erase(west, std::move(done)); });
    CHECK(first.erased_voxels > 0);
    CHECK(first.undo_depth == 1);
    CHECK(first.model.size() < full * 8 / 10);
    CHECK(first.model.size() > full * 2 / 10);
    for (const auto& p : first.model.points) CHECK(p.px > -1.0f);  // (CPU volume only: the GPU model stays on the GPU)
    const auto undone = run([&](auto done) { pipe.undo_erase(std::move(done)); });
    CHECK(undone.undo_depth == 0);
    CHECK(undone.model.size() == full);
    const auto again = run([&](auto done) { pipe.erase(west, std::move(done)); });
    CHECK(again.model.size() == first.model.size());
    pipe.stop();

    // The recording keeps the second erase (the first was undone), over every frame recorded before it.
    auto r = session::SessionReader::open(path);
    REQUIRE(r.has_value());
    REQUIRE((*r)->erasures().size() == 1);
    CHECK((*r)->erasures()[0].frames_before == (*r)->frame_count());
    const auto& k = (*r)->header().depth_intrinsics;
    std::size_t west_depth = 0, east_depth = 0;
    for (std::size_t i = 0; i < (*r)->frame_count(); ++i) {
        const auto f = (*r)->read(i);
        REQUIRE(f.has_value());
        if (!f->accepted()) continue;
        for (int v = 0; v < f->depth.height(); ++v)
            for (int u = 0; u < f->depth.width(); ++u) {
                const float z = f->depth(u, v);
                if (z <= 0) continue;
                const Vec3 p = f->T_world_camera * Vec3((u - k.cx) * z / k.fx, (v - k.cy) * z / k.fy, z);
                (p.x() < -1 ? west_depth : east_depth) += 1;
            }
    }
    CHECK(west_depth == 0);
    CHECK(east_depth > 10000);
    std::filesystem::remove_all(dir);
}
