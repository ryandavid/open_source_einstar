#include "einstar/calibrate/solve.hpp"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <format>
#include <fstream>
#include <sstream>

#include <Eigen/Dense>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/calib/epipolar.hpp"
#include "einstar/calib/rectify.hpp"
#include "einstar/calibrate/plan.hpp"
#include "einstar/optim/stereo_calibration.hpp"

#include "board_bundle.hpp"

namespace einstar::calibrate {
namespace {

constexpr double kDeg = 180.0 / M_PI;
constexpr std::size_t kMinCommonDots = 20;

struct Views {
    std::vector<optim::BoardView> views;
    std::vector<std::vector<int>> ids;  // each dot's index on the board (row * cols + column)
    std::vector<std::size_t> capture;   // index into the captures of each view
};

// Dots seen by both cameras, in board coordinates.
Views common_views(const std::vector<StereoCapture>& captures, const BoardSpec& board) {
    Views out;
    for (std::size_t c = 0; c < captures.size(); ++c) {
        const auto& cap = captures[c];
        optim::BoardView v;
        std::vector<int> ids;
        for (std::size_t i = 0; i < cap.left.grid.size(); ++i)
            for (std::size_t j = 0; j < cap.right.grid.size(); ++j)
                if (cap.left.grid[i] == cap.right.grid[j]) {
                    v.board.push_back(board.point(cap.left.grid[i]));
                    v.left.push_back(cap.left.pixels[i]);
                    v.right.push_back(cap.right.pixels[j]);
                    ids.push_back(static_cast<int>(std::lround(cap.left.grid[i].y())) * board.cols + static_cast<int>(std::lround(cap.left.grid[i].x())));
                }
        if (v.board.size() < kMinCommonDots) continue;
        out.views.push_back(std::move(v));
        out.ids.push_back(std::move(ids));
        out.capture.push_back(c);
    }
    return out;
}

Vec2 rectified_px(const CameraModel& cam, const Mat3& R_rect, const CameraModel& rect, const Vec2& px) {
    const Vec2 n = calib::undistort_to_normalized(cam, px);
    const Vec3 p = R_rect * Vec3(n.x(), n.y(), 1.0);
    return {rect.fx * p.x() / p.z() + rect.cx, rect.fy * p.y() / p.z() + rect.cy};
}

CalibrationReport make_report(const RigCalibration& rig, const Views& v, const std::vector<SE3>& T_left_board,
                              const std::vector<StereoCapture>& captures, const BoardSpec& board) {
    CalibrationReport r;
    r.rig = rig;
    const auto rect = calib::compute_rectification(rig);
    double ss = 0, rows = 0;
    for (std::size_t c = 0; c < captures.size(); ++c) {
        ViewReport vr;
        vr.name = captures[c].name;
        const auto it = std::ranges::find(v.capture, c);
        if (it == v.capture.end()) {
            vr.used = false;
            r.views.push_back(vr);
            continue;
        }
        const auto k = static_cast<std::size_t>(it - v.capture.begin());
        const auto& bv = v.views[k];
        const SE3& Tl = T_left_board[k];
        const SE3 Tr = rig.T_right_left * Tl;
        double vs = 0, vrow = 0;
        for (std::size_t i = 0; i < bv.board.size(); ++i) {
            const double e = (rig.left.project(Tl * bv.board[i]) - bv.left[i]).squaredNorm() + (rig.right.project(Tr * bv.board[i]) - bv.right[i]).squaredNorm();
            const double dy = rectified_px(rig.left, rect.R_left, rect.rectified, bv.left[i]).y() -
                              rectified_px(rig.right, rect.R_right, rect.rectified, bv.right[i]).y();
            vs += e, vrow += dy * dy;
            r.max_row_px = std::max(r.max_row_px, std::abs(dy));
        }
        const auto n = static_cast<double>(bv.board.size());
        vr.dots = static_cast<int>(bv.board.size());
        vr.rms_px = std::sqrt(vs / (2 * n));
        vr.row_rms_px = std::sqrt(vrow / n);
        const auto m = measure_board(Tl, rig, board);
        vr.distance_mm = m.distance_mm, vr.tilt_x_deg = m.tilt_x_deg, vr.tilt_y_deg = m.tilt_y_deg;
        vr.T_left_board = Tl;
        ss += vs, rows += vrow, r.dots += vr.dots;
        r.views.push_back(vr);
    }
    if (r.dots > 0) {
        r.rms_px = std::sqrt(ss / (2.0 * r.dots));
        r.row_rms_px = std::sqrt(rows / r.dots);
    }
    return r;
}

SE3 average_pose(const std::vector<SE3>& poses) {
    Mat3 sum = Mat3::Zero();
    Vec3 t = Vec3::Zero();
    for (const auto& p : poses) sum += p.linear(), t += p.translation();
    Eigen::JacobiSVD<Mat3> svd(sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3 R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) R = svd.matrixU() * Vec3(1, 1, -1).asDiagonal() * svd.matrixV().transpose();
    SE3 out = SE3::Identity();
    out.linear() = R;
    out.translation() = t / static_cast<double>(poses.size());
    return out;
}

}  // namespace

std::optional<CameraModel> initial_intrinsics(const std::vector<std::pair<std::vector<Vec2>, std::vector<Vec2>>>& views, int width,
                                              int height) {
    const double cx = 0.5 * (width - 1), cy = 0.5 * (height - 1);
    Eigen::MatrixXd A(2 * views.size(), 2);
    Eigen::VectorXd b(2 * views.size());
    Eigen::Index row = 0;
    for (const auto& [board, px] : views) {
        if (board.size() < 6) continue;
        Mat3 H = fit_homography(board, px);
        Mat3 Tc;
        Tc << 1, 0, -cx, 0, 1, -cy, 0, 0, 1;
        H = Tc * H;
        Vec3 h = H.col(0), v = H.col(1);
        Vec3 d1 = 0.5 * (h + v), d2 = 0.5 * (h - v);
        h.normalize(), v.normalize(), d1.normalize(), d2.normalize();
        A.row(row) << h.x() * v.x(), h.y() * v.y();
        b(row++) = -h.z() * v.z();
        A.row(row) << d1.x() * d2.x(), d1.y() * d2.y();
        b(row++) = -d1.z() * d2.z();
    }
    if (row < 4) return std::nullopt;
    const Eigen::Vector2d f = A.topRows(row).colPivHouseholderQr().solve(b.head(row));
    if (!(f.x() != 0 && f.y() != 0)) return std::nullopt;
    CameraModel cam;
    cam.width = width, cam.height = height;
    cam.fx = std::sqrt(std::abs(1.0 / f.x()));
    cam.fy = std::sqrt(std::abs(1.0 / f.y()));
    cam.cx = cx, cam.cy = cy;
    if (!std::isfinite(cam.fx) || !std::isfinite(cam.fy) || cam.fx < 50 || cam.fy < 50) return std::nullopt;
    return cam;
}

Result<CalibrationReport> solve_stereo(const std::vector<StereoCapture>& captures, int width, int height, const BoardSpec& board,
                                       const SolveOptions& o) {
    Views v = common_views(captures, board);
    if (v.views.size() < 4)
        return make_error(Errc::invalid_argument, std::format("{} usable views (>= {} dots in both cameras); need at least 4", v.views.size(), kMinCommonDots));

    // Each camera alone: focal length from every view it saw the board in.
    auto intrinsics = [&](bool left) -> std::optional<CameraModel> {
        std::vector<std::pair<std::vector<Vec2>, std::vector<Vec2>>> hv;
        for (const auto& c : captures) {
            const auto& d = left ? c.left : c.right;
            if (d.size() < 12) continue;
            std::vector<Vec2> xy;
            for (const auto& g : d.grid) xy.push_back(board.point(g).head<2>());
            hv.emplace_back(std::move(xy), d.pixels);
        }
        return initial_intrinsics(hv, width, height);
    };
    const auto kl = intrinsics(true), kr = intrinsics(false);
    if (!kl || !kr) return make_error(Errc::invalid_argument, "could not estimate the focal lengths (board views too similar?)");
    RigCalibration rig;
    rig.left = *kl;
    rig.right = *kr;
    if (o.fixed_distortion) rig.left.dist = (*o.fixed_distortion)[0], rig.right.dist = (*o.fixed_distortion)[1];

    // Board poses in each camera, and the left -> right pose they imply.
    std::vector<SE3> T_rl;
    for (std::size_t k = 0; k < v.views.size(); ++k) {
        const auto& cap = captures[v.capture[k]];
        const auto pl = board_pose(cap.left, rig.left, board), pr = board_pose(cap.right, rig.right, board);
        if (!pl || !pr) return make_error(Errc::invalid_argument, std::format("no board pose for {}", cap.name));
        v.views[k].T_left_board = pl->T_cam_board;
        T_rl.push_back(pr->T_cam_board * pl->T_cam_board.inverse());
    }
    rig.T_right_left = average_pose(T_rl);

    // Bundle adjustment: pinhole first, then with distortion.
    optim::StereoCalibOptions so;
    so.huber_px = o.huber_px;
    so.free_distortion = false;
    auto r = optim::refine_stereo_calibration(rig, v.views, so);
    so.free_distortion = o.free_distortion && !o.fixed_distortion;
    for (std::size_t k = 0; k < v.views.size(); ++k) v.views[k].T_left_board = r.T_left_board[k];
    r = optim::refine_stereo_calibration(r.rig, v.views, so);

    // Drop gross outliers (a mis-fitted dot) once, and solve again.
    int dropped = 0;
    const double limit = std::max(o.outlier_px, 4.0 * r.rms);
    for (std::size_t k = 0; k < v.views.size(); ++k) {
        auto& bv = v.views[k];
        const SE3 Tl = r.T_left_board[k], Tr = r.rig.T_right_left * Tl;
        optim::BoardView kept;
        std::vector<int> kept_ids;
        kept.T_left_board = Tl;
        for (std::size_t i = 0; i < bv.board.size(); ++i) {
            const double el = (r.rig.left.project(Tl * bv.board[i]) - bv.left[i]).norm();
            const double er = (r.rig.right.project(Tr * bv.board[i]) - bv.right[i]).norm();
            if (std::max(el, er) > limit) {
                ++dropped;
                continue;
            }
            kept.board.push_back(bv.board[i]), kept.left.push_back(bv.left[i]), kept.right.push_back(bv.right[i]);
            kept_ids.push_back(v.ids[k][i]);
        }
        bv = std::move(kept);
        v.ids[k] = std::move(kept_ids);
    }
    if (dropped > 0) r = optim::refine_stereo_calibration(r.rig, v.views, so);

    // Last, the board's dots free: the cameras fitted to each other through the dots themselves.
    double board_rms_mm = 0, board_flatness_mm = 0;
    if (o.refine_board) {
        std::vector<Vec3> nominal;
        for (int gy = 0; gy < board.rows; ++gy)
            for (int gx = 0; gx < board.cols; ++gx) nominal.push_back(board.point(Vec2(gx, gy)));
        for (std::size_t k = 0; k < v.views.size(); ++k) v.views[k].T_left_board = r.T_left_board[k];
        detail::BoardBundleOptions bo;
        bo.free_distortion = so.free_distortion;
        bo.huber_px = o.huber_px;
        const auto b = detail::refine_with_board(r.rig, v.views, v.ids, nominal, bo);
        r.rig = b.rig;
        r.T_left_board = b.T_left_board;
        for (std::size_t k = 0; k < v.views.size(); ++k)
            for (std::size_t i = 0; i < v.views[k].board.size(); ++i) v.views[k].board[i] = b.board[static_cast<std::size_t>(v.ids[k][i])];
        double ss = 0, sz = 0;
        for (std::size_t i = 0; i < nominal.size(); ++i) ss += (b.board[i] - nominal[i]).squaredNorm(), sz += b.board[i].z() * b.board[i].z();
        board_rms_mm = std::sqrt(ss / static_cast<double>(nominal.size()));
        board_flatness_mm = std::sqrt(sz / static_cast<double>(nominal.size()));
    }

    rig = r.rig;
    rig.texture = rig.left;
    rig.T_texture_left = SE3::Identity();
    auto report = make_report(rig, v, r.T_left_board, captures, board);
    report.dropped = dropped;
    report.board_rms_mm = board_rms_mm;
    report.board_flatness_mm = board_flatness_mm;
    return report;
}

Result<CalibrationReport> evaluate_calibration(const RigCalibration& rig, const std::vector<StereoCapture>& captures, const BoardSpec& board) {
    Views v = common_views(captures, board);
    if (v.views.empty()) return make_error(Errc::invalid_argument, "no views with the board in both cameras");
    for (std::size_t k = 0; k < v.views.size(); ++k) {
        const auto p = board_pose(captures[v.capture[k]].left, rig.left, board);
        if (!p) return make_error(Errc::invalid_argument, std::format("no board pose for {}", captures[v.capture[k]].name));
        v.views[k].T_left_board = p->T_cam_board;
    }
    optim::StereoCalibOptions so;
    so.free_intrinsics = so.free_distortion = so.free_rig = false;
    const auto r = optim::refine_stereo_calibration(rig, v.views, so);
    return make_report(rig, v, r.T_left_board, captures, board);
}

CalibrationDiff compare_calibrations(const RigCalibration& a, const RigCalibration& b) {
    CalibrationDiff d;
    auto camera = [](const CameraModel& ca, const CameraModel& cb) {
        CalibrationDiff::Camera c;
        c.dfx = cb.fx - ca.fx, c.dfy = cb.fy - ca.fy, c.dcx = cb.cx - ca.cx, c.dcy = cb.cy - ca.cy;
        CameraModel only_dist = ca;
        only_dist.dist = cb.dist;
        for (int gy = 0; gy <= 16; ++gy)
            for (int gx = 0; gx <= 20; ++gx) {
                const Vec2 px(ca.width * (0.01 + 0.98 * gx / 20.0), ca.height * (0.01 + 0.98 * gy / 16.0));
                const Vec2 n = calib::undistort_to_normalized(ca, px);
                c.distortion_px = std::max(c.distortion_px, (only_dist.distort_normalized(n) - px).norm());
                c.mapping_px = std::max(c.mapping_px, (cb.distort_normalized(n) - px).norm());
            }
        return c;
    };
    d.left = camera(a.left, b.left);
    d.right = camera(a.right, b.right);
    const Eigen::AngleAxisd aa(b.T_right_left.linear() * a.T_right_left.linear().transpose());
    d.rotation_deg = aa.axis() * aa.angle() * kDeg;
    d.translation_mm = b.T_right_left.translation() - a.T_right_left.translation();
    d.baseline_mm = b.baseline_mm() - a.baseline_mm();
    const auto rows = calib::row_agreement(a, b);
    d.row_mean_px = rows.mean, d.row_max_px = rows.max_abs;
    return d;
}

namespace {

void write_camera(std::ostream& f, const char* name, const CameraModel& m) {
    f << std::format("{} {} {} {:.12g} {:.12g} {:.12g} {:.12g} {:.12g}", name, m.width, m.height, m.fx, m.fy, m.cx, m.cy, m.skew);
    for (const double d : m.dist) f << std::format(" {:.12g}", d);
    f << '\n';
}

void write_pose(std::ostream& f, const char* name, const SE3& T) {
    f << name;
    for (int i = 0; i < 9; ++i) f << std::format(" {:.12g}", T.linear()(i / 3, i % 3));
    for (int i = 0; i < 3; ++i) f << std::format(" {:.12g}", T.translation()(i));
    f << '\n';
}

}  // namespace

Result<void> write_calibration_file(const std::string& path, const CalibrationFile& c) {
    std::ofstream f(path);
    if (!f) return make_error(Errc::io, std::format("cannot write {}", path));
    f << "# Einstar host-side calibration (einstar-calibrate). Cameras: width height fx fy cx cy skew(px) k1 k2 p1 p2 k3,\n"
         "# OpenCV pixel convention. Poses: row-major R then t (mm), x_to = R x_from + t.\n";
    f << "format einstar-calibration 1\n";
    f << "serial " << (c.serial.empty() ? "-" : c.serial) << '\n';
    f << "created " << (c.created.empty() ? "-" : c.created) << '\n';
    f << "source " << (c.source.empty() ? "-" : c.source) << '\n';
    f << std::format("fit {} {:.4f} {:.4f}\n", c.views, c.rms_px, c.row_rms_px);
    write_camera(f, "left", c.rig.left);
    write_camera(f, "right", c.rig.right);
    write_camera(f, "texture", c.rig.texture);
    write_pose(f, "T_right_left", c.rig.T_right_left);
    write_pose(f, "T_texture_left", c.rig.T_texture_left);
    if (!f) return make_error(Errc::io, std::format("cannot write {}", path));
    return {};
}

Result<CalibrationFile> read_calibration_file(const std::string& path) {
    std::ifstream f(path);
    if (!f) return make_error(Errc::not_found, std::format("cannot read {}", path));
    CalibrationFile c;
    bool header = false;
    int found = 0;
    for (std::string line; std::getline(f, line);) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream in(line);
        std::string key;
        in >> key;
        auto rest = [&] {
            std::string s;
            std::getline(in >> std::ws, s);
            return s == "-" ? std::string() : s;
        };
        if (key == "format") {
            std::string name;
            int version = 0;
            in >> name >> version;
            header = name == "einstar-calibration" && version == 1;
        } else if (key == "serial") {
            c.serial = rest();
        } else if (key == "created") {
            c.created = rest();
        } else if (key == "source") {
            c.source = rest();
        } else if (key == "fit") {
            in >> c.views >> c.rms_px >> c.row_rms_px;
        } else if (key == "left" || key == "right" || key == "texture") {
            CameraModel& m = key == "left" ? c.rig.left : key == "right" ? c.rig.right : c.rig.texture;
            in >> m.width >> m.height >> m.fx >> m.fy >> m.cx >> m.cy >> m.skew;
            for (auto& d : m.dist) in >> d;
            if (!in) return make_error(Errc::protocol, std::format("{}: bad {} line", path, key));
            ++found;
        } else if (key == "T_right_left" || key == "T_texture_left") {
            SE3& T = key == "T_right_left" ? c.rig.T_right_left : c.rig.T_texture_left;
            for (int i = 0; i < 9; ++i) in >> T.linear()(i / 3, i % 3);
            for (int i = 0; i < 3; ++i) in >> T.translation()(i);
            if (!in) return make_error(Errc::protocol, std::format("{}: bad {} line", path, key));
            ++found;
        }
    }
    if (!header || found != 5) return make_error(Errc::protocol, std::format("{} is not an einstar-calibration file", path));
    return c;
}

Result<FlashUpdate> build_flash_update(std::span<const std::uint8_t> current, const RigCalibration& rig, const SE3& T_left_world,
                                       const std::string& calibration_time, std::uint32_t seed) {
    auto old = calib::decode_flash_blob(current);
    if (!old) return make_error(Errc::invalid_argument, "the scanner's calibration does not decode: " + old.error().message);
    const RigCalibration old_rig = old->rig();
    calib::DeviceCalibration cal = *old;
    auto camera = [&](const CameraModel& m, const SE3& T_cam_world, double rms) {
        calib::CameraCalibration c;
        c.model = m;
        c.R_cam_world = T_cam_world.linear();
        c.t_cam_world = T_cam_world.translation();
        c.rms_error = rms;
        return c;
    };
    cal.left = camera(rig.left, T_left_world, 0);
    cal.right = camera(rig.right, rig.T_right_left * T_left_world, 0);
    cal.texture = camera(old->texture.model, old_rig.T_texture_left * T_left_world, old->texture.rms_error);
    cal.calibration_time = calibration_time;
    const auto files = calib::encode_ccf_files(cal, seed);
    auto blob = calib::replace_quick_section(current, files);
    if (!blob) return std::unexpected(blob.error());

    // What was built must decode to what was asked for.
    const auto back = calib::decode_flash_blob(*blob);
    if (!back) return make_error(Errc::invalid_argument, "internal error: the new calibration does not decode: " + back.error().message);
    const RigCalibration b = back->rig();
    const auto d = compare_calibrations(rig, b);
    if (d.left.mapping_px > 1e-6 || d.right.mapping_px > 1e-6 || d.rotation_deg.norm() > 1e-9 || d.translation_mm.norm() > 1e-9)
        return make_error(Errc::invalid_argument, "internal error: the new calibration decodes differently");
    if ((b.T_texture_left.translation() - old_rig.T_texture_left.translation()).norm() > 1e-9 ||
        Eigen::AngleAxisd(b.T_texture_left.linear() * old_rig.T_texture_left.linear().transpose()).angle() > 1e-12)
        return make_error(Errc::invalid_argument, "internal error: the colour camera would move");
    if (back->calibration_time != calibration_time) return make_error(Errc::invalid_argument, "internal error: calibration time");

    FlashUpdate u;
    u.blob = std::move(*blob);
    u.calibration_time = calibration_time;
    for (int page = 0; page < 2; ++page) {
        const std::size_t lo = static_cast<std::size_t>(page) * 4096, hi = std::min<std::size_t>(lo + 4096, u.blob.size());
        if (!std::equal(u.blob.begin() + static_cast<std::ptrdiff_t>(lo), u.blob.begin() + static_cast<std::ptrdiff_t>(hi), current.begin() + static_cast<std::ptrdiff_t>(lo)))
            u.pages.push_back(page);
    }
    return u;
}


}  // namespace einstar::calibrate
