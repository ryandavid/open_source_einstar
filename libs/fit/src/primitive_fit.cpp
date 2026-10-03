#include "einstar/fit/primitive_fit.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <Eigen/Dense>

namespace einstar::fit {
namespace {

using VecX = Eigen::VectorXd;
using MatX = Eigen::MatrixXd;

template <class... Ts>
struct Overloaded : Ts... {
    using Ts::operator()...;
};

double weight_of(const PointSet& d, std::size_t i) { return d.weights.empty() ? 1.0 : d.weights[i]; }

Vec3 weighted_centroid(const PointSet& d) {
    Vec3 c = Vec3::Zero();
    double w = 0;
    for (std::size_t i = 0; i < d.points.size(); ++i) {
        c += weight_of(d, i) * d.points[i];
        w += weight_of(d, i);
    }
    return w > 0 ? Vec3(c / w) : c;
}

Surface retract(const Surface& s, const VecX& d) { return perturbed(s, std::span<const double>(d.data(), static_cast<std::size_t>(d.size()))); }

// Keeps the reference point of an axis near the data (the axis' closest point to the centroid).
Surface normalise(const Surface& s, const Vec3& centroid) {
    return std::visit(Overloaded{
                          [&](const Cylinder& c) -> Surface {
                              return Cylinder{c.point + (centroid - c.point).dot(c.axis) * c.axis, c.axis, std::abs(c.radius)};
                          },
                          [&](const Sphere& sp) -> Surface { return Sphere{sp.center, std::abs(sp.radius)}; },
                          [&](const auto& other) -> Surface { return other; },
                      },
                      s);
}

// Kasa circle fit in 2D: centre and radius.
bool fit_circle(std::span<const Vec2> p, std::span<const double> w, Vec2& center, double& radius) {
    if (p.size() < 3) return false;
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Eigen::Vector3d row(2 * p[i].x(), 2 * p[i].y(), 1.0);
        const double wi = w.empty() ? 1.0 : w[i];
        A += wi * row * row.transpose();
        b += wi * row * p[i].squaredNorm();
    }
    const Eigen::Vector3d x = A.ldlt().solve(b);
    if (!x.allFinite()) return false;
    center = x.head<2>();
    const double r2 = x[2] + center.squaredNorm();
    if (r2 <= 0) return false;
    radius = std::sqrt(r2);
    return true;
}

// Axis line met by every normal line (surfaces of revolution): the null vector of the Plücker incidence
// rows [ (p x n)^T  n^T ] (a . (p x n) + m . n = 0 for the axis line (a, m = c x a)).
bool revolution_axis(const PointSet& d, Vec3& axis, Vec3& point) {
    if (d.normals.size() != d.points.size() || d.points.size() < 6) return false;
    const Vec3 c0 = weighted_centroid(d);
    Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero();
    for (std::size_t i = 0; i < d.points.size(); ++i) {
        const Vec3 p = d.points[i] - c0;
        Eigen::Matrix<double, 6, 1> row;
        row << p.cross(d.normals[i]), d.normals[i];
        M += weight_of(d, i) * row * row.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(M);
    const Eigen::Matrix<double, 6, 1> x = es.eigenvectors().col(0);
    const double an = x.head<3>().norm();
    if (an < 1e-9) return false;
    axis = x.head<3>() / an;
    const Vec3 m = x.tail<3>() / an;
    point = c0 + axis.cross(m);
    return axis.allFinite() && point.allFinite();
}

std::optional<Surface> initial_estimate(SurfaceKind kind, const PointSet& d) {
    const std::size_t n = d.points.size();
    const bool has_normals = d.normals.size() == n;
    switch (kind) {
        case SurfaceKind::plane: {
            if (n < 3) return std::nullopt;
            const Vec3 c = weighted_centroid(d);
            Mat3 cov = Mat3::Zero();
            Vec3 mean_n = Vec3::Zero();
            for (std::size_t i = 0; i < n; ++i) {
                const Vec3 q = d.points[i] - c;
                cov += weight_of(d, i) * q * q.transpose();
                if (has_normals) mean_n += weight_of(d, i) * d.normals[i];
            }
            Vec3 normal = Eigen::SelfAdjointEigenSolver<Mat3>(cov).eigenvectors().col(0);
            if (normal.dot(mean_n) < 0) normal = -normal;
            return Plane{normal, normal.dot(c)};
        }
        case SurfaceKind::sphere: {
            if (n < 4) return std::nullopt;
            const Vec3 c0 = weighted_centroid(d);
            Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
            Eigen::Vector4d b = Eigen::Vector4d::Zero();
            for (std::size_t i = 0; i < n; ++i) {
                const Vec3 p = d.points[i] - c0;
                const Eigen::Vector4d row(2 * p.x(), 2 * p.y(), 2 * p.z(), 1.0);
                A += weight_of(d, i) * row * row.transpose();
                b += weight_of(d, i) * row * p.squaredNorm();
            }
            const Eigen::Vector4d x = A.ldlt().solve(b);
            const double r2 = x[3] + x.head<3>().squaredNorm();
            if (!x.allFinite() || r2 <= 0) return std::nullopt;
            return Sphere{c0 + x.head<3>(), std::sqrt(r2)};
        }
        case SurfaceKind::cylinder: {
            if (!has_normals || n < 5) return std::nullopt;
            Mat3 nn = Mat3::Zero();
            for (std::size_t i = 0; i < n; ++i) nn += weight_of(d, i) * d.normals[i] * d.normals[i].transpose();
            const Vec3 axis = Eigen::SelfAdjointEigenSolver<Mat3>(nn).eigenvectors().col(0);
            const Vec3 c0 = weighted_centroid(d);
            const Vec3 u = any_perpendicular(axis), v = axis.cross(u);
            std::vector<Vec2> uv(n);
            for (std::size_t i = 0; i < n; ++i) {
                const Vec3 q = d.points[i] - c0;
                uv[i] = Vec2(q.dot(u), q.dot(v));
            }
            Vec2 center;
            double radius = 0;
            if (!fit_circle(uv, d.weights, center, radius)) return std::nullopt;
            return Cylinder{c0 + center.x() * u + center.y() * v, axis, radius};
        }
        case SurfaceKind::cone: {
            Vec3 axis, point;
            if (!revolution_axis(d, axis, point)) return std::nullopt;
            // Apex: every generator from the apex is perpendicular to the normal, n . (p - apex) = 0.
            Mat3 A = Mat3::Zero();
            Vec3 b = Vec3::Zero();
            for (std::size_t i = 0; i < n; ++i) {
                const Vec3& nn = d.normals[i];
                A += weight_of(d, i) * nn * nn.transpose();
                b += weight_of(d, i) * nn * nn.dot(d.points[i]);
            }
            const Eigen::JacobiSVD<Mat3> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
            if (svd.singularValues()[2] < 1e-6 * svd.singularValues()[0]) return std::nullopt;
            Vec3 apex = svd.solve(b);
            apex = point + (apex - point).dot(axis) * axis;  // onto the axis
            double h = 0, rho = 0;
            for (std::size_t i = 0; i < n; ++i) {
                const Vec3 q = d.points[i] - apex;
                h += q.dot(axis);
            }
            if (h < 0) axis = -axis;
            h = 0;
            for (std::size_t i = 0; i < n; ++i) {
                const Vec3 q = d.points[i] - apex;
                const double hi = q.dot(axis);
                h += hi;
                rho += (q - hi * axis).norm();
            }
            const double angle = std::atan2(rho, h);
            if (!(angle > 1e-3 && angle < std::numbers::pi / 2 - 1e-3) || (apex - point).norm() > 1e4) return std::nullopt;
            return Cone{apex, axis, angle};
        }
        case SurfaceKind::torus: {
            Vec3 axis, point;
            if (!revolution_axis(d, axis, point)) return std::nullopt;
            std::vector<Vec2> rh(n);
            for (std::size_t i = 0; i < n; ++i) {
                const Vec3 q = d.points[i] - point;
                const double h = q.dot(axis);
                rh[i] = Vec2((q - h * axis).norm(), h);
            }
            Vec2 center;
            double minor = 0;
            if (!fit_circle(rh, d.weights, center, minor) || center.x() <= 0) return std::nullopt;
            return Torus{point + center.y() * axis, axis, center.x(), minor};
        }
    }
    return std::nullopt;
}

struct Evaluation {
    std::vector<double> residuals;
    double sigma = 0;
};

Evaluation evaluate(const Surface& s, const PointSet& d, double min_sigma) {
    Evaluation e;
    e.residuals.resize(d.points.size());
    for (std::size_t i = 0; i < d.points.size(); ++i) e.residuals[i] = signed_distance(s, d.points[i]);
    e.sigma = std::max(robust_sigma(e.residuals), min_sigma);
    return e;
}

FitResult finish(const Surface& s, const PointSet& d, const FitOptions& o) {
    const Evaluation e = evaluate(s, d, o.min_sigma);
    FitResult r;
    r.surface = s;
    r.sigma = e.sigma;
    r.points = d.points.size();
    double ss = 0;
    for (const double x : e.residuals)
        if (std::abs(x) < 3 * e.sigma) {
            ss += x * x;
            ++r.inliers;
        }
    r.rms = r.inliers > 0 ? std::sqrt(ss / static_cast<double>(r.inliers)) : 0;
    const auto n = static_cast<double>(std::max<std::size_t>(r.points, 1));
    r.bic = n * std::log(e.sigma * e.sigma) + parameter_count(kind_of(s)) * std::log(n);
    return r;
}

}  // namespace

double robust_sigma(std::span<const double> residuals) {
    if (residuals.empty()) return 0;
    std::vector<double> a(residuals.size());
    for (std::size_t i = 0; i < a.size(); ++i) a[i] = std::abs(residuals[i]);
    const auto mid = a.begin() + static_cast<std::ptrdiff_t>(a.size() / 2);
    std::nth_element(a.begin(), mid, a.end());
    return 1.4826 * *mid;
}

FitResult refine_surface(const Surface& initial, const PointSet& d, const FitOptions& o) {
    const int k = parameter_count(kind_of(initial));
    const std::size_t n = d.points.size();
    const Vec3 centroid = weighted_centroid(d);
    Surface s = normalise(initial, centroid);
    double lambda = 1e-6;
    constexpr double kStep = 1e-6;
    for (int it = 0; it < o.iterations; ++it) {
        const Evaluation e = evaluate(s, d, o.min_sigma);
        const double c = o.tukey * e.sigma;
        std::vector<double> w(n);
        double cost = 0;
        for (std::size_t i = 0; i < n; ++i) {
            const double u = e.residuals[i] / c;
            w[i] = std::abs(u) < 1 ? weight_of(d, i) * (1 - u * u) * (1 - u * u) : 0.0;
            cost += w[i] * e.residuals[i] * e.residuals[i];
        }
        // Numeric Jacobian of the residuals in the local parameters.
        MatX J(static_cast<Eigen::Index>(n), k);
        for (int j = 0; j < k; ++j) {
            VecX delta = VecX::Zero(k);
            delta[j] = kStep;
            const Surface sj = retract(s, delta);
            for (std::size_t i = 0; i < n; ++i)
                J(static_cast<Eigen::Index>(i), j) = (signed_distance(sj, d.points[i]) - e.residuals[i]) / kStep;
        }
        // Normal equations in one product: J^T W J and J^T W r.
        const Eigen::Map<const VecX> wv(w.data(), static_cast<Eigen::Index>(n));
        const Eigen::Map<const VecX> rv(e.residuals.data(), static_cast<Eigen::Index>(n));
        const MatX WJ = wv.asDiagonal() * J;
        const MatX H = J.transpose() * WJ;
        const VecX g = WJ.transpose() * rv;
        bool improved = false;
        for (int attempt = 0; attempt < 8 && !improved; ++attempt) {
            MatX Hd = H;
            Hd.diagonal().array() += lambda * (H.diagonal().array() + 1e-12);
            const VecX step = Hd.ldlt().solve(-g);
            if (!step.allFinite()) break;
            const Surface trial = normalise(retract(s, step), centroid);
            // Compared under the same weights, so a step cannot win by pushing points out of the window.
            double trial_cost = 0;
            for (std::size_t i = 0; i < n; ++i) {
                if (w[i] == 0) continue;
                const double r = signed_distance(trial, d.points[i]);
                trial_cost += w[i] * r * r;
            }
            if (trial_cost <= cost) {
                s = trial;
                lambda = std::max(lambda * 0.1, 1e-9);
                improved = true;
                if (step.norm() < 1e-10) it = o.iterations;
            } else {
                lambda *= 10;
            }
        }
        if (!improved) break;
    }
    return finish(s, d, o);
}

std::optional<FitResult> fit_surface(SurfaceKind kind, const PointSet& d, const FitOptions& o) {
    if (d.points.size() < static_cast<std::size_t>(parameter_count(kind)) + 1) return std::nullopt;
    const auto init = initial_estimate(kind, d);
    if (!init) return std::nullopt;
    FitResult r = refine_surface(*init, d, o);
    if (!std::isfinite(signed_distance(r.surface, d.points[0])) || !std::isfinite(r.sigma)) return std::nullopt;
    return r;
}

std::optional<FitResult> fit_best(std::span<const SurfaceKind> kinds, const PointSet& d, const FitOptions& o) {
    // Size of the data, to recognise curved fits that are really planes (a radius far larger than the
    // patch bends it by less than the noise).
    Eigen::AlignedBox3d box;
    for (const auto& p : d.points) box.extend(p);
    const double extent = box.diagonal().norm();
    std::vector<FitResult> candidates;
    for (const SurfaceKind k : kinds) {
        auto r = fit_surface(k, d, o);
        if (!r) continue;
        // Bend of the patch for curvature 1; a curved fit must bend it by clearly more than the noise.
        const double sagitta_scale = extent * extent / 8, bend_min = 3 * r->sigma;
        const bool flat = std::visit(Overloaded{
                                         [&](const Cylinder& c) { return sagitta_scale / c.radius < bend_min; },
                                         [&](const Sphere& sp) { return sagitta_scale / sp.radius < bend_min; },
                                         [&](const Cone& c) {
                                             return c.half_angle > std::numbers::pi / 2 - 0.0035 || c.half_angle < 0.0035;
                                         },
                                         [&](const Torus& t) { return sagitta_scale / std::max(t.minor, 1e-9) < bend_min; },
                                         [](const Plane&) { return false; },
                                     },
                                     r->surface);
        if (!flat) candidates.push_back(std::move(*r));
    }
    if (candidates.empty()) return std::nullopt;
    // Fewest parameters first; a richer kind must explain the data clearly better (inlier rms 20% lower) to
    // win, so noise alone never turns a plane into a cylinder. (The rms, not the median-based sigma, which
    // jumps about on small seeds.)
    std::ranges::sort(candidates, [](const FitResult& a, const FitResult& b) {
        return parameter_count(kind_of(a.surface)) < parameter_count(kind_of(b.surface));
    });
    FitResult best = candidates.front();
    for (std::size_t i = 1; i < candidates.size(); ++i)
        if (candidates[i].rms < 0.8 * best.rms) best = candidates[i];
    return best;
}

}  // namespace einstar::fit
