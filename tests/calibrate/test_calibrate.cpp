#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <print>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/calibrate/captures.hpp"
#include "einstar/calibrate/plan.hpp"
#include "einstar/calibrate/solve.hpp"
#include "einstar/calibrate/synthetic.hpp"
#include "real_data.hpp"

using namespace einstar;
using namespace einstar::calibrate;

namespace {

RigCalibration flash_rig() {
    auto cal = calib::load_ccf_directory(EINSTAR_TEST_CALIBRATION_DIR);
    REQUIRE(cal);
    return cal->rig();
}

double rotation_deg(const SE3& a, const SE3& b) { return Eigen::AngleAxisd(a.linear() * b.linear().transpose()).angle() * 180.0 / M_PI; }

}  // namespace

TEST_CASE("plan targets measure back as themselves") {
    const auto rig = flash_rig();
    const auto plan = default_plan();
    REQUIRE(plan.size() == 25);
    for (const auto& t : plan) {
        for (const double roll : {0.0, 90.0, -120.0}) {
            const auto m = measure_board(target_pose(t, roll, rig), rig);
            CHECK(std::abs(m.distance_mm - t.distance_mm) < 1e-6);
            CHECK(std::abs(m.tilt_x_deg - t.tilt_x_deg) < 1e-6);
            CHECK(std::abs(m.tilt_y_deg - t.tilt_y_deg) < 1e-6);
            CHECK(std::abs(std::remainder(m.roll_deg - roll, 360.0)) < 1e-6);
            CHECK(m.offset_mm.norm() < 1e-6);
            CHECK(guide(m, t).ok());
        }
    }
    // Off target: the largest error leads the hints.
    auto m = measure_board(target_pose(plan[0], 90, rig), rig);
    m.distance_mm += 100;
    m.tilt_y_deg += 10;
    const auto g = guide(m, plan[0]);
    REQUIRE(!g.ok());
    REQUIRE(g.hints.size() == 2);
    CHECK(g.hints[0].starts_with("Move closer"));
    CHECK(g.hints[1].find("left edge") != std::string::npos);
}

TEST_CASE("steadiness gate waits for a still board") {
    SteadinessGate gate;
    SE3 T = SE3::Identity();
    T.translation() = Vec3(0, 0, 300);
    CHECK(gate.update(T, 0.0) == 0.0);
    CHECK(gate.update(T, 0.3) > 0.29);
    CHECK(!gate.ready(gate.update(T, 0.4)));
    CHECK(gate.ready(gate.update(T, 0.7)));
    T.translation().x() += 5;
    CHECK(gate.update(T, 0.8) == 0.0);
}

TEST_CASE("synthetic board views calibrate back to the rig that rendered them") {
    const RigCalibration truth = flash_rig();
    const BoardSpec board;
    std::vector<StereoCapture> captures;
    for (const auto& t : default_plan()) {
        if (t.step % 2 == 1) continue;  // 15 of the 25 views keep the test quick
        SyntheticBoardParams sp;
        sp.seed = static_cast<std::uint32_t>(captures.size() + 1);
        const auto [l, r] = render_board_pair(truth, target_pose(t, 90, truth, board), board, sp);
        StereoCapture c;
        c.name = t.label;
        auto dl = detect_board(l.view(), board), dr = detect_board(r.view(), board);
        REQUIRE(dl);
        REQUIRE(dr);
        CHECK(dl->size() >= 30);
        c.left = std::move(*dl);
        c.right = std::move(*dr);
        captures.push_back(std::move(c));
    }
    const auto r = solve_stereo(captures, truth.left.width, truth.left.height, board);
    REQUIRE(r);
    std::println("synthetic: rms {:.3f} px, rows {:.3f} px, left f {:.2f} {:.2f} c {:.2f} {:.2f}", r->rms_px, r->row_rms_px, r->rig.left.fx, r->rig.left.fy,
                 r->rig.left.cx, r->rig.left.cy);
    CHECK(r->rms_px < 0.15);
    CHECK(r->row_rms_px < 0.1);
    for (const auto& [est, cam] : {std::pair{r->rig.left, truth.left}, std::pair{r->rig.right, truth.right}}) {
        CHECK(std::abs(est.fx - cam.fx) < 1.5);
        CHECK(std::abs(est.fy - cam.fy) < 1.5);
        CHECK(std::abs(est.cx - cam.cx) < 1.5);
        CHECK(std::abs(est.cy - cam.cy) < 1.5);
    }
    CHECK(rotation_deg(r->rig.T_right_left, truth.T_right_left) < 0.05);
    CHECK((r->rig.T_right_left.translation() - truth.T_right_left.translation()).norm() < 0.5);
    const auto d = compare_calibrations(truth, r->rig);
    CHECK(d.left.mapping_px < 2.0);
    CHECK(d.right.mapping_px < 2.0);
}

TEST_CASE("EXStar's calibration captures solve to its calibration") {
    const auto dir = test_data::calibration_board();
    if (!dir) SKIP("EXStar calibration captures not available");
    const auto loaded = load_captures(*dir);
    REQUIRE(loaded);
    REQUIRE(loaded->captures.size() == 25);
    int complete = 0;
    for (const auto& c : loaded->captures) {
        CHECK(c.left.size() >= 24);
        CHECK(c.right.size() >= 24);
        complete += c.left.size() == 40;
    }
    CHECK(complete >= 15);

    // EXStar's quick calibration keeps the factory distortion and re-fits the rest: do the same.
    const RigCalibration flash = flash_rig();
    SolveOptions so;
    so.fixed_distortion = std::array{flash.left.dist, flash.right.dist};
    const auto r = solve_stereo(loaded->captures, loaded->width, loaded->height, {}, so);
    REQUIRE(r);
    const auto d = compare_calibrations(flash, r->rig);
    std::println("EXStar captures: rms {:.3f} px, rows {:.3f} px; vs flash: left f {:+.2f} {:+.2f} c {:+.2f} {:+.2f}, rig {:.3f} deg, baseline {:+.3f} mm", r->rms_px,
                 r->row_rms_px, d.left.dfx, d.left.dfy, d.left.dcx, d.left.dcy, rotation_deg(r->rig.T_right_left, flash.T_right_left), d.baseline_mm);
    CHECK(r->rms_px < 0.4);
    CHECK(r->row_rms_px < 0.08);
    for (const auto& c : {d.left, d.right}) {
        CHECK(std::abs(c.dfx) < 1.5);
        CHECK(std::abs(c.dfy) < 1.5);
        CHECK(std::abs(c.dcx) < 3.0);
        CHECK(std::abs(c.dcy) < 6.0);
    }
    CHECK(rotation_deg(r->rig.T_right_left, flash.T_right_left) < 0.15);
    CHECK(std::abs(d.baseline_mm) < 0.3);

    // The flash calibration aligns these captures' rows as well as ours does.
    const auto e = evaluate_calibration(flash, loaded->captures);
    REQUIRE(e);
    CHECK(e->row_rms_px < 0.08);
}

TEST_CASE("calibration file round trip") {
    CalibrationFile f;
    f.rig = flash_rig();
    f.serial = "0009011402CF0C20";
    f.created = "2026-09-30 10:00";
    f.source = "test";
    f.rms_px = 0.1234, f.row_rms_px = 0.05, f.views = 25;
    const auto path = std::filesystem::temp_directory_path() / "einstar_calibration_test.txt";
    REQUIRE(write_calibration_file(path.string(), f));
    const auto back = read_calibration_file(path.string());
    std::filesystem::remove(path);
    REQUIRE(back);
    CHECK(back->serial == f.serial);
    CHECK(back->created == f.created);
    CHECK(back->views == 25);
    CHECK(std::abs(back->rig.left.fx - f.rig.left.fx) < 1e-6);
    CHECK(std::abs(back->rig.right.dist[4] - f.rig.right.dist[4]) < 1e-9);
    CHECK(rotation_deg(back->rig.T_right_left, f.rig.T_right_left) < 1e-8);
    CHECK((back->rig.T_texture_left.translation() - f.rig.T_texture_left.translation()).norm() < 1e-6);
}
