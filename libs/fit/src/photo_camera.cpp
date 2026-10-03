#include "einstar/fit/photo_camera.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <Eigen/Dense>

namespace einstar::fit {
namespace {

using VecX = Eigen::VectorXd;
using MatX = Eigen::MatrixXd;

Mat3 exp_so3(const Vec3& w) {
    const double a = w.norm();
    if (a < 1e-12) return Mat3::Identity();
    return Eigen::AngleAxisd(a, w / a).toRotationMatrix();
}

struct Problem {
    std::span<const Vec2> pixels;
    std::span<const Vec3> points;
    std::vector<int> use;  // which pairs take part
    double huber = 3;
};

// The camera moved by a step: rotation (left, 3), translation (3), log focal, k1.
PinholeCamera stepped(const PinholeCamera& c, const VecX& d, bool fit_focal, bool fit_k1) {
    PinholeCamera n = c;
    const Mat3 R = exp_so3(d.head<3>()) * c.T_camera_world.linear();
    n.T_camera_world.linear() = R;
    n.T_camera_world.translation() = c.T_camera_world.translation() + d.segment<3>(3);
    int k = 6;
    if (fit_focal) n.focal = c.focal * std::exp(d[k++]);
    if (fit_k1) n.k1 = c.k1 + d[k++];
    return n;
}

// Reprojection errors (px) of the pairs in use; points behind the camera count as far off.
std::vector<Vec2> errors(const PinholeCamera& c, const Problem& p) {
    std::vector<Vec2> e;
    e.reserve(p.use.size());
    for (const int i : p.use) {
        const auto q = c.project(p.points[static_cast<std::size_t>(i)]);
        e.push_back(q ? Vec2(*q - p.pixels[static_cast<std::size_t>(i)]) : Vec2(1e4, 1e4));
    }
    return e;
}

double robust_cost(const std::vector<Vec2>& e, double h) {
    double s = 0;
    for (const auto& v : e) {
        const double r = v.norm();
        s += r <= h ? 0.5 * r * r : h * (r - 0.5 * h);
    }
    return s;
}

// Levenberg-Marquardt with Huber weights (reweighted each iteration).
PinholeCamera refine(PinholeCamera c, const Problem& p, bool fit_focal, bool fit_k1, int iterations) {
    const int n = 6 + (fit_focal ? 1 : 0) + (fit_k1 ? 1 : 0);
    double lambda = 1e-3;
    auto e = errors(c, p);
    double cost = robust_cost(e, p.huber);
    for (int it = 0; it < iterations; ++it) {
        const auto m = static_cast<Eigen::Index>(e.size() * 2);
        VecX r(m);
        VecX w(m);
        for (std::size_t i = 0; i < e.size(); ++i) {
            const double norm = e[i].norm();
            const double wi = norm <= p.huber ? 1.0 : std::sqrt(p.huber / norm);
            r.segment<2>(static_cast<Eigen::Index>(2 * i)) = e[i];
            w.segment<2>(static_cast<Eigen::Index>(2 * i)).setConstant(wi);
        }
        // The Jacobian by forward differences (a handful of parameters and pairs).
        MatX J(m, n);
        for (int k = 0; k < n; ++k) {
            VecX d = VecX::Zero(n);
            const double h = k < 3 ? 1e-6 : k < 6 ? 1e-5 * std::max(1.0, c.T_camera_world.translation().norm()) : 1e-6;
            d[k] = h;
            const auto e2 = errors(stepped(c, d, fit_focal, fit_k1), p);
            for (std::size_t i = 0; i < e2.size(); ++i) J.block<2, 1>(static_cast<Eigen::Index>(2 * i), k) = (e2[i] - e[i]) / h;
        }
        const MatX Jw = w.asDiagonal() * J;
        const VecX rw = w.cwiseProduct(r);
        const MatX A = Jw.transpose() * Jw;
        const VecX g = Jw.transpose() * rw;
        bool improved = false;
        for (int tries = 0; tries < 8 && !improved; ++tries) {
            MatX Ad = A;
            Ad.diagonal() += lambda * (A.diagonal().array() + 1e-9).matrix();
            const VecX d = Ad.ldlt().solve(-g);
            if (!d.allFinite()) break;
            const PinholeCamera cand = stepped(c, d, fit_focal, fit_k1);
            const auto ce = errors(cand, p);
            const double cc = robust_cost(ce, p.huber);
            if (cc < cost && cand.focal > 0) {
                improved = true;
                const double gain = cost - cc;
                c = cand;
                e = ce;
                cost = cc;
                lambda = std::max(lambda / 3, 1e-9);
                if (gain < 1e-10 * std::max(1.0, cost)) return c;
            } else {
                lambda *= 4;
            }
        }
        if (!improved) break;
    }
    return c;
}

// A starting camera looking along `view` at the points, its roll, distance and offset from a weak-perspective
// (scaled orthographic) fit of the pixels.
std::optional<PinholeCamera> weak_perspective_start(const Problem& p, const Vec3& view, double focal, const Vec2& principal) {
    Vec3 c = Vec3::Zero();
    Vec2 u = Vec2::Zero();
    for (const int i : p.use) {
        c += p.points[static_cast<std::size_t>(i)];
        u += p.pixels[static_cast<std::size_t>(i)] - principal;
    }
    const double count = static_cast<double>(p.use.size());
    c /= count;
    u /= count;
    const Vec3 z = view.normalized();
    const Vec3 a = std::abs(z.x()) < 0.9 ? Vec3::UnitX() : Vec3::UnitY();
    const Vec3 x = (a - a.dot(z) * z).normalized();  // any roll: the fit below finds it
    const Vec3 y = z.cross(x);
    // Procrustes: rotation and scale taking the projected points (centred) to the pixels (centred).
    double sxx = 0, sxy = 0, qq = 0;
    std::vector<std::pair<Vec2, Vec2>> pairs;
    for (const int i : p.use) {
        const Vec3 d = p.points[static_cast<std::size_t>(i)] - c;
        const Vec2 q(d.dot(x), d.dot(y));
        const Vec2 v = p.pixels[static_cast<std::size_t>(i)] - principal - u;
        sxx += q.dot(v);
        sxy += q.x() * v.y() - q.y() * v.x();
        qq += q.squaredNorm();
    }
    if (qq < 1e-12) return std::nullopt;
    const double theta = std::atan2(sxy, sxx);
    const double s = std::hypot(sxx, sxy) / qq;  // px per mm
    if (!(s > 1e-9)) return std::nullopt;
    const Vec3 xc = std::cos(theta) * x - std::sin(theta) * y;
    const Vec3 yc = std::sin(theta) * x + std::cos(theta) * y;
    const double Z = focal / s;
    const Vec3 centroid_cam(u.x() * Z / focal, u.y() * Z / focal, Z);
    const Vec3 centre = c - (xc * centroid_cam.x() + yc * centroid_cam.y() + z * centroid_cam.z());
    PinholeCamera cam;
    cam.focal = focal;
    cam.principal = principal;
    Mat3 R;
    R.row(0) = xc.transpose();
    R.row(1) = yc.transpose();
    R.row(2) = z.transpose();
    cam.T_camera_world.linear() = R;
    cam.T_camera_world.translation() = -R * centre;
    return cam;
}

}  // namespace

std::optional<Vec2> PinholeCamera::project(const Vec3& world) const {
    const Vec3 q = T_camera_world * world;
    if (q.z() <= 1e-9) return std::nullopt;
    const Vec2 n(q.x() / q.z(), q.y() / q.z());
    return principal + focal * n * (1 + k1 * n.squaredNorm());
}

std::pair<Vec3, Vec3> PinholeCamera::ray(const Vec2& pixel) const {
    const Vec2 d = (pixel - principal) / focal;
    Vec2 n = d;
    for (int i = 0; i < 20; ++i) n = d / (1 + k1 * n.squaredNorm());  // undistort
    const Mat3 R = T_camera_world.linear();
    const Vec3 dir = R.transpose() * Vec3(n.x(), n.y(), 1).normalized();
    return {center(), dir};
}

Vec3 PinholeCamera::center() const { return -T_camera_world.linear().transpose() * T_camera_world.translation(); }

int min_camera_pairs(bool focal_known) { return focal_known ? 4 : 6; }

double focal_from_35mm(double focal_35mm, int width, int height) { return focal_35mm / 36.0 * std::max(width, height); }

std::optional<CameraFit> fit_camera(std::span<const Vec2> pixels, std::span<const Vec3> points, const CameraFitOptions& o) {
    const int n = static_cast<int>(std::min(pixels.size(), points.size()));
    if (n < min_camera_pairs(o.focal_prior.has_value()) || o.width <= 0 || o.height <= 0) return std::nullopt;
    Problem p{pixels, points, {}, o.huber_px};
    for (int i = 0; i < n; ++i) p.use.push_back(i);
    const Vec2 principal(o.width / 2.0, o.height / 2.0);
    const bool fit_focal = n >= 6;
    const bool fit_k1 = n >= 10;
    // Without EXIF, a typical lens: 28 mm equivalent.
    const double focal0 = o.focal_prior.value_or(focal_from_35mm(28, o.width, o.height));

    // Starts from views all round (a Fibonacci sphere), each refined briefly with the focal length held; the best
    // few are refined fully.
    std::vector<std::pair<double, PinholeCamera>> starts;
    const int views = 240;
    for (int k = 0; k < views; ++k) {
        const double zc = 1 - 2 * (k + 0.5) / views;
        const double r = std::sqrt(std::max(0.0, 1 - zc * zc));
        const double phi = k * std::numbers::pi * (3 - std::sqrt(5.0));
        const auto start = weak_perspective_start(p, Vec3(r * std::cos(phi), r * std::sin(phi), zc), focal0, principal);
        if (!start) continue;
        const PinholeCamera c = refine(*start, p, false, false, 12);
        starts.emplace_back(robust_cost(errors(c, p), p.huber), c);
    }
    if (starts.empty()) return std::nullopt;
    std::ranges::sort(starts, {}, &std::pair<double, PinholeCamera>::first);
    std::optional<PinholeCamera> best;
    double best_cost = 1e300;
    for (std::size_t k = 0; k < std::min<std::size_t>(5, starts.size()); ++k) {
        PinholeCamera c = refine(starts[k].second, p, false, false, 50);
        if (fit_focal) c = refine(c, p, true, false, 100);
        if (fit_k1) c = refine(c, p, true, true, 100);
        const double cost = robust_cost(errors(c, p), p.huber);
        if (cost < best_cost) {
            best_cost = cost;
            best = c;
        }
    }
    if (!best) return std::nullopt;

    CameraFit out;
    out.camera = *best;
    out.focal_fitted = fit_focal;
    out.k1_fitted = fit_k1;
    const auto e = errors(*best, p);
    double ss = 0;
    for (const auto& v : e) {
        out.residuals.push_back(v.norm());
        ss += v.squaredNorm();
    }
    out.rms_px = std::sqrt(ss / n);
    // A pair is an outlier when the camera solved from the others puts it far from where it was clicked.
    out.outlier.assign(static_cast<std::size_t>(n), false);
    if (n > min_camera_pairs(o.focal_prior.has_value())) {
        for (int i = 0; i < n; ++i) {
            if (out.residuals[static_cast<std::size_t>(i)] < 3) continue;
            Problem q = p;
            std::erase(q.use, i);
            const PinholeCamera c = refine(*best, q, fit_focal && n - 1 >= 6, fit_k1 && n - 1 >= 10, 40);
            const auto others = errors(c, q);
            double rms = 0;
            for (const auto& v : others) rms += v.squaredNorm();
            rms = std::sqrt(rms / static_cast<double>(others.size()));
            const auto mine = c.project(points[static_cast<std::size_t>(i)]);
            const double r = mine ? (*mine - pixels[static_cast<std::size_t>(i)]).norm() : 1e9;
            out.outlier[static_cast<std::size_t>(i)] = r > std::max(8.0, 5 * rms);
        }
    }
    return out;
}

}  // namespace einstar::fit
