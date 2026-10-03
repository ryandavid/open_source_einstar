#include <algorithm>
#include <cmath>
#include <format>

#include <Eigen/Eigenvalues>
#include <Eigen/SparseCholesky>

#include "einstar/fit/primitive_fit.hpp"

namespace einstar::fit {
namespace {

void cubic_basis(double t, std::array<double, 4>& b) {
    const double t2 = t * t, t3 = t2 * t, s = 1 - t;
    b = {s * s * s / 6, (3 * t3 - 6 * t2 + 4) / 6, (-3 * t3 + 3 * t2 + 3 * t + 1) / 6, t3 / 6};
}

}  // namespace

std::optional<FitResult> fit_freeform(const PointSet& d, const FreeformOptions& o, std::string* why) {
    const auto fail = [&](std::string m) -> std::optional<FitResult> {
        if (why) *why = std::move(m);
        return std::nullopt;
    };
    const std::size_t n = d.points.size();
    if (n < 30) return fail("too few points for a freeform face");
    const auto weight = [&](std::size_t i) { return d.weights.empty() ? 1.0 : d.weights[i]; };

    // The frame: the best-fit plane, z along the mean normal.
    Vec3 c = Vec3::Zero(), mean_n = Vec3::Zero();
    double wsum = 0;
    for (std::size_t i = 0; i < n; ++i) {
        c += weight(i) * d.points[i];
        wsum += weight(i);
        if (d.normals.size() == n) mean_n += weight(i) * d.normals[i];
    }
    c /= wsum;
    Mat3 cov = Mat3::Zero();
    for (std::size_t i = 0; i < n; ++i) cov += weight(i) * (d.points[i] - c) * (d.points[i] - c).transpose();
    const Eigen::SelfAdjointEigenSolver<Mat3> es(cov);
    Vec3 z = es.eigenvectors().col(0), x = es.eigenvectors().col(2);
    if (z.dot(mean_n) < 0) z = -z;
    const Vec3 y = z.cross(x);
    Freeform f;
    f.frame.linear().col(0) = x;
    f.frame.linear().col(1) = y;
    f.frame.linear().col(2) = z;
    f.frame.translation() = c;

    // One height per place: the scan's normals must all face up from the plane.
    if (d.normals.size() == n) {
        std::size_t folded = 0;
        for (const auto& nn : d.normals) folded += nn.dot(z) < 0.1;
        if (folded > n / 20) return fail("the region folds over itself (no single height over its plane): split it into faces that do not");
    }
    std::vector<Vec3> q(n);
    Eigen::AlignedBox2d box;
    for (std::size_t i = 0; i < n; ++i) {
        q[i] = f.frame.inverse() * d.points[i];
        box.extend(Vec2(q[i].x(), q[i].y()));
    }
    const double extent = box.sizes().maxCoeff();
    const double spacing = o.spacing_mm > 0 ? o.spacing_mm : std::max(1.5, extent / 12);
    const double margin = o.margin_mm > 0 ? o.margin_mm : std::max(5.0, 0.2 * extent);
    f.du = f.dv = spacing;
    f.u0 = box.min().x() - margin;
    f.v0 = box.min().y() - margin;
    f.nu = static_cast<int>(std::ceil((box.sizes().x() + 2 * margin) / spacing)) + 3;
    f.nv = static_cast<int>(std::ceil((box.sizes().y() + 2 * margin) / spacing)) + 3;
    const int nu = f.nu, nv = f.nv, m = nu * nv;
    const auto idx = [&](int i, int j) { return i * nv + j; };

    // Bending energy: second differences along u and v and the twist, on the control heights.
    std::vector<Eigen::Triplet<double>> bend;
    const auto add_row = [&](std::initializer_list<std::pair<int, double>> row, double w) {
        for (const auto& [a, va] : row)
            for (const auto& [b, vb] : row) bend.emplace_back(a, b, w * va * vb);
    };
    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j) {
            if (i + 2 < nu) add_row({{idx(i, j), 1}, {idx(i + 1, j), -2}, {idx(i + 2, j), 1}}, 1);
            if (j + 2 < nv) add_row({{idx(i, j), 1}, {idx(i, j + 1), -2}, {idx(i, j + 2), 1}}, 1);
            if (i + 1 < nu && j + 1 < nv) add_row({{idx(i, j), 1}, {idx(i + 1, j), -1}, {idx(i, j + 1), -1}, {idx(i + 1, j + 1), 1}}, 2);
        }
    Eigen::SparseMatrix<double> B(m, m);
    B.setFromTriplets(bend.begin(), bend.end());

    // Each point's 16 controls and weights.
    struct Row {
        std::array<int, 16> k;
        std::array<double, 16> b;
    };
    std::vector<Row> rows(n);
    for (std::size_t p = 0; p < n; ++p) {
        const double tu = (q[p].x() - f.u0) / f.du, tv = (q[p].y() - f.v0) / f.dv;
        const int cu = std::clamp(static_cast<int>(tu), 0, nu - 4), cv = std::clamp(static_cast<int>(tv), 0, nv - 4);
        std::array<double, 4> bu, bv;
        cubic_basis(tu - cu, bu);
        cubic_basis(tv - cv, bv);
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                rows[p].k[static_cast<std::size_t>(i * 4 + j)] = idx(cu + i, cv + j);
                rows[p].b[static_cast<std::size_t>(i * 4 + j)] = bu[static_cast<std::size_t>(i)] * bv[static_cast<std::size_t>(j)];
            }
    }
    // The bending term's weight: per control, against the data it has (scale-free in the number of points).
    const double lambda = o.smoothness * wsum / m;
    std::vector<double> robust(n, 1.0);
    Eigen::VectorXd H = Eigen::VectorXd::Zero(m);
    std::vector<double> res(n);
    double sigma = 0;
    for (int it = 0; it < std::max(1, o.iterations); ++it) {
        std::vector<Eigen::Triplet<double>> trip;
        trip.reserve(n * 256);
        Eigen::VectorXd rhs = Eigen::VectorXd::Zero(m);
        for (std::size_t p = 0; p < n; ++p) {
            const double w = weight(p) * robust[p];
            if (w == 0) continue;
            for (std::size_t a = 0; a < 16; ++a) {
                rhs[rows[p].k[a]] += w * rows[p].b[a] * q[p].z();
                for (std::size_t b = 0; b < 16; ++b) trip.emplace_back(rows[p].k[a], rows[p].k[b], w * rows[p].b[a] * rows[p].b[b]);
            }
        }
        Eigen::SparseMatrix<double> A(m, m);
        A.setFromTriplets(trip.begin(), trip.end());
        A += lambda * B;
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(A);
        if (solver.info() != Eigen::Success) return fail("the freeform fit could not be solved");
        H = solver.solve(rhs);
        if (!H.allFinite()) return fail("the freeform fit could not be solved");
        for (std::size_t p = 0; p < n; ++p) {
            double h = 0;
            for (std::size_t a = 0; a < 16; ++a) h += rows[p].b[a] * H[rows[p].k[a]];
            res[p] = q[p].z() - h;
        }
        sigma = std::max(robust_sigma(res), 1e-4);
        for (std::size_t p = 0; p < n; ++p) {
            const double u = res[p] / (4 * sigma);
            robust[p] = std::abs(u) < 1 ? (1 - u * u) * (1 - u * u) : 0.0;
        }
    }
    f.heights = std::make_shared<std::vector<double>>(H.data(), H.data() + m);

    FitResult r;
    r.surface = f;
    r.points = n;
    double ss = 0;
    for (std::size_t p = 0; p < n; ++p) res[p] = signed_distance(r.surface, d.points[p]);
    r.sigma = std::max(robust_sigma(res), 1e-4);
    for (const double e : res)
        if (std::abs(e) < 3 * r.sigma) {
            ss += e * e;
            ++r.inliers;
        }
    r.rms = r.inliers ? std::sqrt(ss / static_cast<double>(r.inliers)) : 0;
    r.bic = 0;
    return r;
}

}  // namespace einstar::fit
