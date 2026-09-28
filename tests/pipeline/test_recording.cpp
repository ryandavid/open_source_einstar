// What a live scan records: the scanner, every processed frame with its capture settings and
// tracking diagnostics, a record for every frame that could not be processed, and (optionally) the
// raw IR images of every frame that reached the host.

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <set>

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
        CHECK(m.extras->tracking.stereo_ms > 0);
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
