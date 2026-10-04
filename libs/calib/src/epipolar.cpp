#include "einstar/calib/epipolar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>

#include <Eigen/Dense>

#include "einstar/calib/rectify.hpp"

namespace einstar::calib {
namespace {

constexpr double kDeg = 180.0 / M_PI;

// Both centres as undistorted normalised rays, computed once: the fit only turns the right camera.
struct Rays {
    std::vector<Vec3> left, right;
};

Rays undistort_pairs(const RigCalibration& rig, std::span<const MarkerPair> pairs) {
    Rays r;
    r.left.reserve(pairs.size()), r.right.reserve(pairs.size());
    for (const auto& p : pairs) {
        const Vec2 l = undistort_to_normalized(rig.left, p.left), q = undistort_to_normalized(rig.right, p.right);
        r.left.emplace_back(l.x(), l.y(), 1.0);
        r.right.emplace_back(q.x(), q.y(), 1.0);
    }
    return r;
}

// Rectified (row difference, disparity) of one pair of rays.
Vec2 rows(const EpipolarFrame& e, const Vec3& left, const Vec3& right) {
    const Vec3 l = e.R_left * left, r = e.R_right * right;
    return {e.f * (l.y() / l.z() - r.y() / r.z()), e.f * (l.x() / l.z() - r.x() / r.z())};
}

// The rig turned by p = (about the baseline, about the optical axis), its row differences on `idx`.
Eigen::VectorXd residuals(const RigCalibration& rig, const Rays& rays, const std::vector<std::size_t>& idx, const Eigen::Vector2d& p) {
    const auto e = epipolar_frame(turn_right_camera(rig, p.x(), p.y()));
    Eigen::VectorXd r(static_cast<Eigen::Index>(idx.size()));
    for (std::size_t k = 0; k < idx.size(); ++k) r(static_cast<Eigen::Index>(k)) = rows(e, rays.left[idx[k]], rays.right[idx[k]]).x();
    return r;
}

double median(std::vector<double> v) {
    if (v.empty()) return 0;
    const auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
    std::ranges::nth_element(v, mid);
    return *mid;
}

double percentile(std::vector<double> v, double q) {
    if (v.empty()) return 0;
    const auto at = v.begin() + static_cast<std::ptrdiff_t>(std::clamp(q, 0.0, 1.0) * static_cast<double>(v.size() - 1));
    std::ranges::nth_element(v, at);
    return *at;
}

double robust_sigma(const std::vector<double>& v, double centre) {
    std::vector<double> d;
    d.reserve(v.size());
    for (const double x : v) d.push_back(std::abs(x - centre));
    return 1.4826 * median(std::move(d));
}

struct Fit {
    Eigen::Vector2d p = Eigen::Vector2d::Zero();
    Eigen::Vector2d sigma = Eigen::Vector2d::Zero();
};

// Huber-weighted Gauss-Newton on the row differences; `both` also frees the turn about the optical axis.
Fit fit_turn(const RigCalibration& rig, const Rays& rays, const std::vector<std::size_t>& idx, bool both, double huber_px) {
    Fit fit;
    const int n = both ? 2 : 1;
    constexpr double h = 1e-6;
    Eigen::MatrixXd J(static_cast<Eigen::Index>(idx.size()), n);
    Eigen::VectorXd r;
    for (int iter = 0; iter < 8; ++iter) {
        r = residuals(rig, rays, idx, fit.p);
        for (int j = 0; j < n; ++j) {
            Eigen::Vector2d d = Eigen::Vector2d::Zero();
            d(j) = h;
            J.col(j) = (residuals(rig, rays, idx, fit.p + d) - residuals(rig, rays, idx, fit.p - d)) / (2 * h);
        }
        const Eigen::VectorXd w = r.unaryExpr([&](double x) { return std::abs(x) <= huber_px ? 1.0 : huber_px / std::abs(x); });
        const Eigen::MatrixXd A = J.transpose() * w.asDiagonal() * J;
        const Eigen::VectorXd step = A.ldlt().solve(-(J.transpose() * w.asDiagonal() * r));
        fit.p.head(n) += step;
        if (step.cwiseAbs().maxCoeff() < 1e-10) break;
    }
    r = residuals(rig, rays, idx, fit.p);
    std::vector<double> rv(r.data(), r.data() + r.size());
    const double s = robust_sigma(rv, median(rv));
    const Eigen::MatrixXd cov = (J.transpose() * J).inverse() * s * s;
    for (int j = 0; j < n; ++j) fit.sigma(j) = std::sqrt(cov(j, j));
    return fit;
}

EpipolarCheck::Rows row_stats(const std::vector<double>& dy, const std::vector<Vec2>& left_px, const std::vector<double>& disparity, int width, int height) {
    EpipolarCheck::Rows s;
    if (dy.empty()) return s;
    std::vector<double> a;
    a.reserve(dy.size());
    for (const double d : dy) a.push_back(std::abs(d));
    s.median = median(dy);
    s.abs_median = median(a);
    s.p95 = percentile(a, 0.95);
    s.robust_sigma = robust_sigma(dy, s.median);
    Eigen::MatrixXd A(static_cast<Eigen::Index>(dy.size()), 4);
    Eigen::VectorXd b(static_cast<Eigen::Index>(dy.size()));
    for (std::size_t i = 0; i < dy.size(); ++i) {
        const auto k = static_cast<Eigen::Index>(i);
        A.row(k) << 1.0, (left_px[i].x() - 0.5 * width) / (0.5 * width), (left_px[i].y() - 0.5 * height) / (0.5 * height), disparity[i] / 300.0;
        b(k) = dy[i];
    }
    const Eigen::Vector4d c = A.colPivHouseholderQr().solve(b);
    s.offset = c(0), s.slope_x = c(1), s.slope_y = c(2), s.slope_disparity = c(3);
    return s;
}

}  // namespace

EpipolarFrame epipolar_frame(const RigCalibration& rig) {
    // As the rectification: each camera turns by half the rotation between them, then x goes along the baseline.
    const Mat3 R = rig.T_right_left.linear();
    const Vec3 t = rig.T_right_left.translation();
    const Eigen::AngleAxisd aa(R);
    const Mat3 r_half = Eigen::AngleAxisd(-0.5 * aa.angle(), aa.axis()).toRotationMatrix();
    const Vec3 e1 = -(r_half * t).normalized();
    Vec3 e2 = Vec3::UnitZ().cross(e1);
    if (e2.norm() < 1e-9) e2 = Vec3::UnitY();
    e2.normalize();
    Mat3 align;
    align.row(0) = e1.transpose();
    align.row(1) = e2.transpose();
    align.row(2) = e1.cross(e2).transpose();
    EpipolarFrame e;
    e.R_left = align * r_half.transpose();
    e.R_right = align * r_half;
    e.f = 0.5 * (rig.left.fy + rig.right.fy);
    return e;
}

MarkerPair raw_marker_pair(const RigCalibration& rig, const Mat3& R_rect_left, const Mat3& R_rect_right, const CameraModel& rectified_left,
                           const CameraModel& rectified_right, const Vec2& left_rect, const Vec2& right_rect) {
    auto back = [&](const CameraModel& cam, const Mat3& R_rect, const CameraModel& rc, const Vec2& px) {
        const Vec3 ray((px.x() - rc.cx) / rc.fx, (px.y() - rc.cy) / rc.fy, 1.0);
        return cam.project(R_rect.transpose() * ray);
    };
    return {back(rig.left, R_rect_left, rectified_left, left_rect), back(rig.right, R_rect_right, rectified_right, right_rect)};
}

double row_difference(const RigCalibration& rig, const MarkerPair& pair) {
    const auto rays = undistort_pairs(rig, std::span(&pair, 1));
    return rows(epipolar_frame(rig), rays.left[0], rays.right[0]).x();
}

std::vector<double> row_differences(const RigCalibration& rig, std::span<const MarkerPair> pairs) {
    const auto rays = undistort_pairs(rig, pairs);
    const auto e = epipolar_frame(rig);
    std::vector<double> out;
    out.reserve(pairs.size());
    for (std::size_t i = 0; i < pairs.size(); ++i) out.push_back(rows(e, rays.left[i], rays.right[i]).x());
    return out;
}

RigCalibration turn_right_camera(const RigCalibration& rig, double about_baseline_rad, double about_optical_axis_rad) {
    const auto e = epipolar_frame(rig);
    const Mat3 Q = (Eigen::AngleAxisd(about_baseline_rad, Vec3::UnitX()) * Eigen::AngleAxisd(about_optical_axis_rad, Vec3::UnitZ())).toRotationMatrix();
    // The same turn in the right camera's own frame; about its centre, so the baseline vector turns with it.
    const Mat3 Qc = e.R_right.transpose() * Q * e.R_right;
    RigCalibration out = rig;
    out.T_right_left.linear() = Qc * rig.T_right_left.linear();
    out.T_right_left.translation() = Qc * rig.T_right_left.translation();
    return out;
}

EpipolarCheck check_epipolar(const RigCalibration& rig, std::span<const MarkerPair> pairs, const EpipolarCheckOptions& o) {
    EpipolarCheck c;
    c.pairs = static_cast<int>(pairs.size());
    c.corrected = rig;
    const int width = rig.left.width > 0 ? rig.left.width : 1280, height = rig.left.height > 0 ? rig.left.height : 1024;
    const auto rays = undistort_pairs(rig, pairs);
    const auto e0 = epipolar_frame(rig);
    std::vector<Vec2> dy_disp;
    dy_disp.reserve(pairs.size());
    for (std::size_t i = 0; i < pairs.size(); ++i) dy_disp.push_back(rows(e0, rays.left[i], rays.right[i]));

    // Mismatches out: far from the median row difference, or behind the rig.
    std::vector<double> all;
    for (const auto& d : dy_disp) all.push_back(d.x());
    const double m0 = median(all);
    std::vector<std::size_t> idx;
    for (std::size_t i = 0; i < pairs.size(); ++i)
        if (std::isfinite(dy_disp[i].x()) && std::abs(dy_disp[i].x() - m0) < o.gate_px && dy_disp[i].y() > 0) idx.push_back(i);
    if (static_cast<int>(idx.size()) < 3) {
        c.verdict = std::format("{} marker pairs: too few to check the calibration", c.pairs);
        return c;
    }

    auto fit_all = [&](const std::vector<std::size_t>& use) {
        Fit offset_only = fit_turn(rig, rays, use, false, o.huber_px);
        c.optical_axis_fitted = false;
        if (!o.fit_optical_axis || c.x_span < o.min_x_span) return offset_only;
        const Fit both = fit_turn(rig, rays, use, true, o.huber_px);
        // The tilt is kept only where it is measured: clearly non-zero and large enough to matter at the image's edges.
        const double edge_px = std::abs(both.p.y()) * 0.5 * width;
        if (edge_px < o.min_optical_axis_px || std::abs(both.p.y()) < 3 * both.sigma.y()) return offset_only;
        c.optical_axis_fitted = true;
        return both;
    };
    std::vector<double> xs;
    for (const auto i : idx) xs.push_back(pairs[i].left.x());
    c.x_span = (percentile(xs, 0.95) - percentile(xs, 0.05)) / width;
    Fit fit = fit_all(idx);

    // Once more without what the fit shows to be mismatches.
    {
        const auto r = residuals(rig, rays, idx, fit.p);
        std::vector<double> rv(r.data(), r.data() + r.size());
        const double gate = std::max(0.3, 5 * robust_sigma(rv, median(rv)));
        std::vector<std::size_t> kept;
        for (std::size_t k = 0; k < idx.size(); ++k)
            if (std::abs(r(static_cast<Eigen::Index>(k))) < gate) kept.push_back(idx[k]);
        if (kept.size() >= 3 && kept.size() < idx.size()) {
            idx = std::move(kept);
            xs.clear();
            for (const auto i : idx) xs.push_back(pairs[i].left.x());
            c.x_span = (percentile(xs, 0.95) - percentile(xs, 0.05)) / width;
            fit = fit_all(idx);
        }
    }
    c.inliers = static_cast<int>(idx.size());
    c.about_baseline_deg = fit.p.x() * kDeg;
    c.about_optical_axis_deg = fit.p.y() * kDeg;
    c.sigma_baseline_deg = fit.sigma.x() * kDeg;
    c.sigma_optical_axis_deg = fit.sigma.y() * kDeg;

    const RigCalibration turned = turn_right_camera(rig, fit.p.x(), fit.p.y());
    const auto e1 = epipolar_frame(turned);
    std::vector<double> before, after, disparity;
    std::vector<Vec2> left_px;
    for (const auto i : idx) {
        before.push_back(dy_disp[i].x());
        disparity.push_back(dy_disp[i].y());
        after.push_back(rows(e1, rays.left[i], rays.right[i]).x());
        left_px.push_back(pairs[i].left);
    }
    c.before = row_stats(before, left_px, disparity, width, height);
    c.after = row_stats(after, left_px, disparity, width, height);

    // Spread: the left image in cells; each covered cell must agree with the correction.
    const int g = std::max(1, o.grid);
    std::vector<std::vector<double>> cells(static_cast<std::size_t>(g * g));
    for (std::size_t k = 0; k < idx.size(); ++k) {
        const int cx = std::clamp(static_cast<int>(left_px[k].x() / width * g), 0, g - 1);
        const int cy = std::clamp(static_cast<int>(left_px[k].y() / height * g), 0, g - 1);
        cells[static_cast<std::size_t>(cy * g + cx)].push_back(after[k]);
    }
    for (const auto& cell : cells)
        if (static_cast<int>(cell.size()) >= o.min_cell_pairs) {
            ++c.cells;
            c.worst_cell_px = std::max(c.worst_cell_px, std::abs(median(cell)));
        }

    // The size of the correction: how far it moves rows, at most, over the image.
    const double size_px = std::abs(fit.p.x()) * e0.f + std::abs(fit.p.y()) * 0.5 * width;
    const std::string found = std::format("rows {:+.3f} px (|dy| median {:.3f}) over {} of {} pairs; turn about the baseline {:+.4f} deg{}", c.before.median,
                                          c.before.abs_median, c.inliers, c.pairs, c.about_baseline_deg,
                                          c.optical_axis_fitted ? std::format(", optical axis {:+.4f} deg", c.about_optical_axis_deg) : "");
    std::string why;
    if (c.inliers < o.min_pairs) why = std::format("too few pairs (fewer than {})", o.min_pairs);
    else if (c.cells < o.min_cells) why = std::format("markers seen in {} image regions only (need {})", c.cells, o.min_cells);
    else if (size_px < o.min_correction_px) why = "the calibration agrees with the markers";
    else if (size_px > o.max_correction_px) why = std::format("too large for drift ({:.2f} px): recalibrate", size_px);
    else if (c.worst_cell_px > o.agree_px) why = std::format("the image regions disagree (one is {:.2f} px off after the turn): not a rotation", c.worst_cell_px);
    else if (c.after.abs_median >= c.before.abs_median) why = "the turn does not reduce the row error";
    c.apply = why.empty();
    if (c.apply) c.corrected = turned;
    c.verdict = c.apply ? std::format("{} -> correct: rows {:+.3f} px (|dy| median {:.3f}) after", found, c.after.median, c.after.abs_median)
                        : std::format("{} -> leave: {}", found, why);
    return c;
}

EpipolarSamples::EpipolarSamples(int width, int height, int cells, int per_cell)
    : width_(std::max(1, width)), height_(std::max(1, height)), cells_(std::max(1, cells)), per_cell_(std::max(1, per_cell)),
      ring_(static_cast<std::size_t>(cells_ * cells_)), next_(ring_.size(), 0) {}

void EpipolarSamples::add(const MarkerPair& pair) {
    const int cx = std::clamp(static_cast<int>(pair.left.x() / width_ * cells_), 0, cells_ - 1);
    const int cy = std::clamp(static_cast<int>(pair.left.y() / height_ * cells_), 0, cells_ - 1);
    const auto c = static_cast<std::size_t>(cy * cells_ + cx);
    auto& ring = ring_[c];
    if (static_cast<int>(ring.size()) < per_cell_) {
        ring.push_back(pair);
        return;
    }
    ring[next_[c]] = pair;
    next_[c] = (next_[c] + 1) % ring.size();
}

void EpipolarSamples::clear() {
    for (auto& r : ring_) r.clear();
    std::ranges::fill(next_, 0);
}

std::size_t EpipolarSamples::size() const {
    std::size_t n = 0;
    for (const auto& r : ring_) n += r.size();
    return n;
}

std::vector<MarkerPair> EpipolarSamples::pairs() const {
    std::vector<MarkerPair> out;
    out.reserve(size());
    for (const auto& r : ring_) out.insert(out.end(), r.begin(), r.end());
    return out;
}

RowAgreement row_agreement(const RigCalibration& truth, const RigCalibration& model, double near_mm, double far_mm) {
    RowAgreement a;
    std::vector<MarkerPair> pairs;
    const CameraModel& L = truth.left;
    const CameraModel& R = truth.right;
    for (int iz = 0; iz < 6; ++iz) {
        const double z = near_mm + (far_mm - near_mm) * iz / 5.0;
        for (int gy = 0; gy <= 12; ++gy)
            for (int gx = 0; gx <= 16; ++gx) {
                const Vec2 px(L.width * (0.04 + 0.92 * gx / 16.0), L.height * (0.04 + 0.92 * gy / 12.0));
                const Vec2 n = undistort_to_normalized(L, px);
                const Vec3 p = truth.T_right_left * Vec3(n.x() * z, n.y() * z, z);
                if (p.z() <= 0) continue;
                const Vec2 q = R.project(p);
                if (q.x() < 0 || q.y() < 0 || q.x() > R.width - 1 || q.y() > R.height - 1) continue;
                pairs.push_back({px, q});
            }
    }
    if (pairs.empty()) return a;
    double sum = 0, ss = 0;
    for (const double d : row_differences(model, pairs)) {
        sum += d, ss += d * d;
        a.max_abs = std::max(a.max_abs, std::abs(d));
    }
    a.mean = sum / static_cast<double>(pairs.size());
    a.rms = std::sqrt(ss / static_cast<double>(pairs.size()));
    return a;
}

}  // namespace einstar::calib
