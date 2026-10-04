#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <print>
#include <random>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/calib/epipolar.hpp"
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
            CHECK(guide(m, t).ok() == (roll == 90.0));  // the plan holds the board's roll at 90 degrees
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
    CHECK(g.hints[1].find("bottom edge") != std::string::npos);  // +tilt_y: the view's top edge is too near
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
    // With the board's dots fitted (it is 0.1 mm off its nominal grid) the captures reproject at 0.04 px, and
    // the solve is EXStar's calibration: principal points within 0.3 px, the rig within 0.01 degrees, the
    // rows it rectifies within 0.05 px of EXStar's everywhere in the image.
    CHECK(r->rms_px < 0.1);
    CHECK(r->row_rms_px < 0.06);
    CHECK(r->board_rms_mm > 0.05);
    CHECK(r->board_rms_mm < 0.2);
    for (const auto& c : {d.left, d.right}) {
        CHECK(std::abs(c.dfx) < 1.0);
        CHECK(std::abs(c.dfy) < 1.0);
        CHECK(std::abs(c.dcx) < 1.0);
        CHECK(std::abs(c.dcy) < 1.0);
    }
    CHECK(rotation_deg(r->rig.T_right_left, flash.T_right_left) < 0.03);
    CHECK(std::abs(d.baseline_mm) < 0.3);
    std::println("  rows vs flash: {:+.3f} px on average, up to {:.3f}", d.row_mean_px, d.row_max_px);
    CHECK(std::abs(d.row_mean_px) < 0.03);
    CHECK(d.row_max_px < 0.1);
    // Held at the nominal grid, the board's errors go into the cameras (principal points 4.5 px off).
    so.refine_board = false;
    const auto nominal = solve_stereo(loaded->captures, loaded->width, loaded->height, {}, so);
    REQUIRE(nominal);
    CHECK(nominal->rms_px > 0.2);

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

TEST_CASE("a solve becomes a flash blob that decodes to it and keeps everything else") {
    const auto l = calib::load_ccf_directory(EINSTAR_TEST_CALIBRATION_DIR);
    REQUIRE(l);
    auto read = [](const std::string& p) {
        std::ifstream f(p, std::ios::binary);
        return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(f), {});
    };
    const std::string dir = EINSTAR_TEST_CALIBRATION_DIR;
    auto blob = calib::encode_quick_flash_blob(read(dir + "/LeftCCF.txt"), read(dir + "/RightCCF.txt"), read(dir + "/TexCCF.txt"), "2026-09-27 13:57");
    for (std::size_t i = 0; i < 0x39B; ++i) blob[i] = static_cast<std::uint8_t>(i * 13 + 1);  // stand-in factory / colour data
    const RigCalibration old = l->rig();
    RigCalibration ours = old;
    ours.right.cy += 2.0;
    ours.left.dist[0] += 0.003;
    ours.T_right_left.linear() = Eigen::AngleAxisd(0.003, Vec3::UnitX()).toRotationMatrix() * ours.T_right_left.linear();
    SE3 world = SE3::Identity();
    world.translation() = Vec3(-60, -50, 200);
    const auto u = build_flash_update(blob, ours, world, "2026-09-30 12:00", 7);
    REQUIRE(u);
    CHECK(u->pages == std::vector<int>{0});  // the CCF data and time all lie below 0x1000
    for (std::size_t i = 0; i < 0x39B; ++i) REQUIRE(u->blob[i] == blob[i]);
    for (std::size_t i = 0x12BC; i < blob.size(); ++i) REQUIRE(u->blob[i] == blob[i]);
    const auto back = calib::decode_flash_blob(u->blob);
    REQUIRE(back);
    const auto d = compare_calibrations(ours, back->rig());
    CHECK(d.left.mapping_px < 1e-6);
    CHECK(d.right.mapping_px < 1e-6);
    CHECK(d.rotation_deg.norm() < 1e-9);
    CHECK(rotation_deg(back->rig().T_texture_left, old.T_texture_left) < 1e-9);
    CHECK((back->left.t_cam_world - world.translation()).norm() < 1e-9);
    CHECK(!build_flash_update(std::vector<std::uint8_t>(calib::kFlashBlobSize, 0), ours, world, "x", 1));
}

TEST_CASE("roll guidance: the hint's direction brings the roll back") {
    const auto rig = flash_rig();
    // Face-on: turning the scanner about its own axis changes the roll only. (For a tilted view it also
    // swings which edge is near, and the tilt hint then follows.)
    const auto t = default_plan()[2];
    CHECK(t.roll_deg == 90.0);
    const SE3 off = target_pose(t, 125, rig);  // board turned 35 degrees too far in the view
    const auto m = measure_board(off, rig);
    const auto g = guide(m, t);
    REQUIRE_FALSE(g.roll_ok);
    CHECK(std::abs(g.roll_error_deg - 35) < 1e-6);
    REQUIRE_FALSE(g.hints.empty());
    CHECK(g.hints.front().starts_with("Turn the scanner clockwise"));
    // Turning the scanner clockwise (seen from behind it) about its axis by those 35 degrees fixes it.
    const SE3 S_from_left = scanner_from_left(rig);
    SE3 turn = SE3::Identity();
    turn.linear() = Eigen::AngleAxisd(35.0 * M_PI / 180.0, Vec3::UnitZ()).toRotationMatrix().transpose();
    const auto fixed = measure_board(S_from_left.inverse() * turn * S_from_left * off, rig);
    CHECK(std::abs(std::remainder(fixed.roll_deg - 90, 360.0)) < 1e-6);
    CHECK(guide(fixed, t).ok());
    CHECK(guide(measure_board(target_pose(t, 180, rig), rig), t).hints.front().find("long side") != std::string::npos);
}

TEST_CASE("the plan seen from the board: five lines from its centre, the distances along them") {
    const auto rig = flash_rig();
    const BoardSpec board;
    const auto plan = default_plan();
    std::array<Vec3, 5> dir{};
    for (const auto& t : plan) {
        // The same pose whether it comes from the target or from measuring the target's board pose.
        const SE3 B_S = board_from_scanner(t);
        const SE3 B_S_measured = board_from_scanner(measure_board(target_pose(t, t.roll_deg, rig), rig));
        CHECK((B_S.translation() - B_S_measured.translation()).norm() < 1e-6);
        CHECK(rotation_deg(B_S, B_S_measured) < 1e-6);
        // The scanner sits t.distance_mm from the board centre, looking at it.
        const Vec3 v = B_S.translation() - board.centre();
        CHECK(std::abs(v.norm() - t.distance_mm) < 1e-6);
        CHECK((B_S.linear().col(2) + v.normalized()).norm() < 1e-9);
        // ... on its group's line: one direction per group, the face-on one along the normal (the dots face -z).
        auto& d = dir[static_cast<std::size_t>(t.group)];
        if (t.step == 0) d = v.normalized();
        else CHECK((v.normalized() - d).norm() < 1e-9);
    }
    CHECK((dir[0] + Vec3::UnitZ()).norm() < 1e-9);
    for (int g = 1; g < 5; ++g) CHECK(std::abs(std::acos(-dir[static_cast<std::size_t>(g)].z()) * 180 / M_PI - 30) < 1e-6);
    // Opposite groups lie opposite each other across the normal.
    CHECK((dir[1] + dir[2]).normalized().dot(-Vec3::UnitZ()) > 1 - 1e-9);
    CHECK((dir[3] + dir[4]).normalized().dot(-Vec3::UnitZ()) > 1 - 1e-9);
}

TEST_CASE("nearest target: the uncaptured view the scanner is closest to") {
    const auto plan = default_plan();
    std::vector<bool> captured(plan.size(), false);
    auto m = target_measure(plan[13]);
    m.distance_mm += 10;
    auto n = nearest_target(plan, captured, m);
    CHECK(n.index == 13);
    CHECK(n.error < 1);
    captured[13] = true;
    n = nearest_target(plan, captured, m);
    CHECK(n.index != 13);
    CHECK(plan[static_cast<std::size_t>(n.index)].group == plan[13].group);  // the next step on the same line
    CHECK(n.error > 1);
    std::fill(captured.begin(), captured.end(), true);
    CHECK(nearest_target(plan, captured, m).index == -1);
}

TEST_CASE("a board off its nominal grid does not bend the rectified rows") {
    // A board bowed by 0.3 mm with its dots up to 0.1 mm off their spots, seen in the plan's 25 views
    // (dot centres projected exactly, 0.05 px noise). Held at its nominal grid, the solve takes the bow
    // into the distortion and the rows go wrong where no dot was; with the dots fitted they do not.
    const RigCalibration truth = flash_rig();
    const BoardSpec board;
    std::mt19937 rng(5);
    std::normal_distribution<double> noise(0, 0.05);
    std::uniform_real_distribution<double> jitter(-0.1, 0.1);
    std::vector<Vec3> dots;
    for (int gy = 0; gy < board.rows; ++gy)
        for (int gx = 0; gx < board.cols; ++gx) {
            Vec3 p = board.point(Vec2(gx, gy));
            const double u = (gx - 0.5 * (board.cols - 1)) / (0.5 * (board.cols - 1)), v = (gy - 0.5 * (board.rows - 1)) / (0.5 * (board.rows - 1));
            p += Vec3(jitter(rng), jitter(rng), -0.3 * (1 - u * u) * (1 - v * v));
            dots.push_back(p);
        }
    auto seen = [&](const CameraModel& cam, const SE3& T) {
        BoardDetection d;
        for (int i = 0; i < board.dots(); ++i) {
            const Vec2 px = cam.project(T * dots[static_cast<std::size_t>(i)]) + Vec2(noise(rng), noise(rng));
            if (px.x() < 0 || px.y() < 0 || px.x() > cam.width - 1 || px.y() > cam.height - 1) continue;
            d.grid.push_back(Vec2(i % board.cols, i / board.cols));
            d.pixels.push_back(px);
        }
        return d;
    };
    std::vector<StereoCapture> captures;
    for (const auto& t : default_plan()) {
        const SE3 T = target_pose(t, 90, truth, board);
        captures.push_back({t.label, seen(truth.left, T), seen(truth.right, truth.T_right_left * T)});
    }
    SolveOptions so;
    so.refine_board = false;
    const auto nominal = solve_stereo(captures, 1280, 1024, board, so);
    so.refine_board = true;
    const auto fitted = solve_stereo(captures, 1280, 1024, board, so);
    REQUIRE(nominal);
    REQUIRE(fitted);
    const auto rn = calib::row_agreement(truth, nominal->rig), rf = calib::row_agreement(truth, fitted->rig);
    std::println("bowed board: nominal grid rms {:.3f} px, rows {:+.3f} up to {:.2f} px; dots fitted rms {:.3f} px ({:.3f} mm off the grid), rows {:+.3f} up to {:.2f} px",
                 nominal->rms_px, rn.mean, rn.max_abs, fitted->rms_px, fitted->board_rms_mm, rf.mean, rf.max_abs);
    CHECK(rn.max_abs > 1.0);
    CHECK(fitted->rms_px < 0.08);
    CHECK(std::abs(rf.mean) < 0.05);
    CHECK(rf.max_abs < 0.7);  // in the image's corners, beyond every dot, the distortion is extrapolated
    CHECK(std::abs(fitted->rig.baseline_mm() - truth.baseline_mm()) < 0.1);
    // With the distortion known (EXStar's quick calibration keeps the factory's) the corners are right too.
    so.fixed_distortion = std::array{truth.left.dist, truth.right.dist};
    const auto fixed = solve_stereo(captures, 1280, 1024, board, so);
    REQUIRE(fixed);
    const auto rx = calib::row_agreement(truth, fixed->rig);
    std::println("  factory distortion: rows {:+.3f} up to {:.2f} px", rx.mean, rx.max_abs);
    CHECK(std::abs(rx.mean) < 0.03);
    CHECK(rx.max_abs < 0.15);
    // The board as fitted: its bow (about its mean, which goes into the board poses).
    double mean = 0, ss = 0;
    for (const auto& p : dots) mean += p.z() / static_cast<double>(dots.size());
    for (const auto& p : dots) ss += (p.z() - mean) * (p.z() - mean);
    CHECK(std::abs(fitted->board_flatness_mm - std::sqrt(ss / static_cast<double>(dots.size()))) < 0.02);
}
