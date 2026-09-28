#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <random>

#include "einstar/session/session.hpp"

using namespace einstar;

namespace {

session::FrameRecord make_frame(std::uint64_t i, std::mt19937& rng) {
    session::FrameRecord f;
    f.index = i;
    f.timestamp_s = 0.068 * static_cast<double>(i);
    f.flags = session::frame_accepted | (i % 2 ? session::frame_marker_pose : 0u);
    f.T_world_camera = SE3::Identity();
    f.T_world_camera.linear() = Eigen::AngleAxisd(0.01 * static_cast<double>(i), Vec3(0.3, 1, 0.2).normalized()).toRotationMatrix();
    f.T_world_camera.translation() = Vec3(1.5 * static_cast<double>(i), -2, 300);
    std::uniform_real_distribution<float> z(180, 650);
    f.depth = ImageF32(64, 48, 0.0f);
    f.confidence = ImageF32(64, 48, 0.0f);
    for (int y = 0; y < 48; ++y)
        for (int x = 0; x < 64; ++x)
            if ((x + y) % 7) {
                f.depth(x, y) = z(rng);
                f.confidence(x, y) = static_cast<float>((x * y) % 256) / 255.0f;
            }
    f.markers.push_back({Vec3(1, 2, 300), Vec3(0, 0, -1), 6.1, 17, Vec2(100.25, 200.5), Vec2(50.125, 200.5)});
    return f;
}

}  // namespace

TEST_CASE("session files round-trip frames, markers, poses and global markers") {
    const auto path = (std::filesystem::temp_directory_path() / "einstar_test_session.estr").string();
    std::mt19937 rng(3);
    std::vector<session::FrameRecord> frames;
    for (std::uint64_t i = 0; i < 20; ++i) frames.push_back(make_frame(i, rng));
    {
        session::SessionHeader h;
        h.depth_intrinsics = {64, 48, 58.0, 58.0, 32.0, 24.0};
        h.rect_f = 1157.2;
        h.baseline_mm = 159.9;
        h.description = "unit test";
        auto w = session::SessionWriter::create(path, h);
        REQUIRE(w.has_value());
        for (int i = 0; i < 10; ++i) (*w)->write(frames[static_cast<std::size_t>(i)]);
        (*w)->write_global_markers({{3, Vec3(1, 2, 3), 6.0, 5, true}});
        for (int i = 10; i < 20; ++i) (*w)->write(frames[static_cast<std::size_t>(i)]);
        (*w)->close();
        CHECK((*w)->frames_written() == 20);
    }
    auto r = session::SessionReader::open(path);
    REQUIRE(r.has_value());
    CHECK((*r)->header().description == "unit test");
    CHECK((*r)->header().depth_intrinsics.width == 64);
    CHECK((*r)->header().rect_f == 1157.2);
    REQUIRE((*r)->frame_count() == 20);
    REQUIRE((*r)->global_markers().size() == 1);
    CHECK((*r)->global_markers()[0].id == 3);
    for (std::size_t i = 0; i < 20; ++i) {
        auto f = (*r)->read(i);
        REQUIRE(f.has_value());
        const auto& e = frames[i];
        CHECK(f->index == e.index);
        CHECK(f->flags == e.flags);
        CHECK((f->T_world_camera.matrix() - e.T_world_camera.matrix()).norm() < 1e-12);
        REQUIRE(f->markers.size() == 1);
        CHECK(f->markers[0].map_id == 17);
        CHECK(f->markers[0].left_rect == e.markers[0].left_rect);
        float max_dz = 0, max_dc = 0;
        for (std::size_t k = 0; k < e.depth.pixels().size(); ++k) {
            max_dz = std::max(max_dz, std::abs(f->depth.data()[k] - e.depth.data()[k]));
            max_dc = std::max(max_dc, std::abs(f->confidence.data()[k] - e.confidence.data()[k]));
        }
        CHECK(max_dz <= 0.0101f);  // 1/50 mm quantisation
        CHECK(max_dc <= 0.5f / 255.0f + 1e-6f);
    }

    // A crash mid-write leaves a truncated last record: everything before it must still load.
    const auto size = std::filesystem::file_size(path);
    std::filesystem::resize_file(path, size - 100);
    auto t = session::SessionReader::open(path);
    REQUIRE(t.has_value());
    CHECK((*t)->frame_count() == 19);
    CHECK((*t)->read(18).has_value());
    std::filesystem::remove(path);
}

TEST_CASE("session files carry the scanner, per-frame extras, drops and raw IR") {
    const auto path = (std::filesystem::temp_directory_path() / "einstar_test_session_extras.estr").string();
    std::mt19937 rng(5);
    session::DeviceRecord dev;
    dev.vendor = "Shining3D";
    dev.product = "EINSCAN10_01";
    dev.serial = "0009011402CF0C20";
    dev.firmware = "V2.10";
    dev.calibration_blob = {1, 2, 3, 250, 0, 7};
    dev.rig.left.width = 1280;
    dev.rig.left.fx = 1165.25;
    dev.rig.left.dist = {0.1, -0.2, 0.001, 0.002, 0.05};
    dev.rig.T_right_left.translation() = Vec3(-159.9, 0.2, 1.1);
    dev.R_rect_left = Eigen::AngleAxisd(0.1, Vec3::UnitY()).toRotationMatrix();
    dev.rectified.fx = 1157.2;

    auto raw_image = [&](int w, int h) {
        ImageU8 img(w, h, 0);
        std::uniform_int_distribution<int> px(0, 255);
        for (auto& p : img.pixels()) p = static_cast<std::uint8_t>(px(rng));
        return img;
    };
    std::vector<session::RawFrame> raws;
    for (std::uint64_t i = 0; i < 3; ++i) {
        session::RawFrame rf;
        rf.index = 100 + i;
        rf.timestamp_s = 0.068 * static_cast<double>(i);
        rf.images.emplace_back(0, raw_image(128, 96));
        rf.images.emplace_back(1, raw_image(128, 96));
        raws.push_back(rf);
    }
    {
        session::SessionHeader h;
        h.depth_intrinsics = {64, 48, 58.0, 58.0, 32.0, 24.0};
        auto w = session::SessionWriter::create(path, h);
        REQUIRE(w.has_value());
        (*w)->write_device(dev);
        (*w)->write_dropped({103, 0.2, "live queue full"});
        for (std::uint64_t i = 0; i < 3; ++i) {
            auto f = make_frame(100 + i, rng);
            session::FrameExtras ex;
            ex.left_sensor = 1;
            ex.capture.exposure = {4400, 4410};
            ex.capture.gain = {120, 121};
            ex.capture.laser_percent = 100;
            ex.capture.strobe = 6000;
            ex.capture.trigger_period_us = 68000;
            ex.capture.temperature_c = 38.5f;
            ex.tracking.state = 1;
            ex.tracking.icp_rms_mm = 0.15f;
            ex.tracking.correspondences = 12345;
            ex.tracking.reason = i == 1 ? "inconsistent (60% inliers)" : "";
            f.extras = ex;
            (*w)->write(f);
            REQUIRE((*w)->write_raw(raws[i]));
            (*w)->flush();  // fixed record order: the file ends with the last raw frame
        }
        (*w)->close();
        CHECK((*w)->raw_frames_written() == 3);
    }
    auto r = session::SessionReader::open(path);
    REQUIRE(r.has_value());
    REQUIRE((*r)->device().has_value());
    const auto& d = *(*r)->device();
    CHECK(d.serial == dev.serial);
    CHECK(d.firmware == dev.firmware);
    CHECK(d.calibration_blob == dev.calibration_blob);
    CHECK(d.rig.left.fx == dev.rig.left.fx);
    CHECK(d.rig.left.dist == dev.rig.left.dist);
    CHECK((d.rig.T_right_left.matrix() - dev.rig.T_right_left.matrix()).norm() < 1e-12);
    CHECK((d.R_rect_left - dev.R_rect_left).norm() < 1e-12);
    CHECK(d.rectified.fx == dev.rectified.fx);

    REQUIRE((*r)->frame_count() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        const auto& m = (*r)->meta(i);
        REQUIRE(m.extras.has_value());
        CHECK(m.extras->left_sensor == 1);
        CHECK(m.extras->capture.exposure[1] == 4410);
        CHECK(m.extras->capture.gain[0] == 120);
        CHECK(m.extras->capture.temperature_c == 38.5f);
        CHECK(m.extras->tracking.correspondences == 12345);
        CHECK(m.extras->tracking.reason == (i == 1 ? "inconsistent (60% inliers)" : ""));
    }
    REQUIRE((*r)->dropped().size() == 1);
    CHECK((*r)->dropped()[0].index == 103);
    CHECK((*r)->dropped()[0].reason == "live queue full");

    REQUIRE((*r)->raw_count() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        CHECK((*r)->raw_index(i) == 100 + i);
        auto rf = (*r)->read_raw(i);
        REQUIRE(rf.has_value());
        REQUIRE(rf->images.size() == 2);
        for (std::size_t k = 0; k < 2; ++k) {
            CHECK(rf->images[k].first == raws[i].images[k].first);
            const auto& a = rf->images[k].second.pixels();
            const auto& b = raws[i].images[k].second.pixels();
            CHECK(std::equal(a.begin(), a.end(), b.begin(), b.end()));  // lossless
        }
    }

    // Truncated in the middle of the last raw record (~24 KB of incompressible pixels): the rest still loads.
    std::filesystem::resize_file(path, std::filesystem::file_size(path) - 5000);
    auto t = session::SessionReader::open(path);
    REQUIRE(t.has_value());
    CHECK((*t)->frame_count() == 3);
    CHECK((*t)->raw_count() == 2);
    CHECK((*t)->read_raw(1).has_value());
    std::filesystem::remove(path);
}
