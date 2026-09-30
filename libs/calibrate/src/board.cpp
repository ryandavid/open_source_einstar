#include "einstar/calibrate/board.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "einstar/calib/rectify.hpp"

namespace einstar::calibrate {
namespace {

// Affine grid -> pixel map from the four large dots (three of them collinear, so no homography yet).
Mat3 affine_from_large(const std::array<Vec2, 4>& grid, const std::array<Vec2, 4>& px) {
    Eigen::Matrix<double, 8, 6> A = Eigen::Matrix<double, 8, 6>::Zero();
    Eigen::Matrix<double, 8, 1> b;
    for (int i = 0; i < 4; ++i) {
        const auto& g = grid[static_cast<std::size_t>(i)];
        A.row(2 * i) << g.x(), g.y(), 1, 0, 0, 0;
        A.row(2 * i + 1) << 0, 0, 0, g.x(), g.y(), 1;
        b(2 * i) = px[static_cast<std::size_t>(i)].x();
        b(2 * i + 1) = px[static_cast<std::size_t>(i)].y();
    }
    const Eigen::Matrix<double, 6, 1> a = A.colPivHouseholderQr().solve(b);
    Mat3 H;
    H << a(0), a(1), a(2), a(3), a(4), a(5), 0, 0, 1;
    return H;
}

// Orders four large-dot candidates as BoardSpec::large: the lone one is farthest from the line through
// the other three; of the row's endpoints, column 2 is the one farther from the middle dot (column 4).
std::optional<std::array<Vec2, 4>> order_large(const std::array<Vec2, 4>& big) {
    std::size_t lone = 0;
    double best_line = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < 4; ++i) {
        std::array<Vec2, 3> o;
        for (std::size_t j = 0, k = 0; j < 4; ++j)
            if (j != i) o[k++] = big[j];
        // Collinearity of the other three: twice their triangle's area over the longest side squared.
        const Vec2 u = o[1] - o[0], v = o[2] - o[0];
        const double longest = std::max({u.squaredNorm(), v.squaredNorm(), (o[2] - o[1]).squaredNorm()});
        if (longest <= 0) return std::nullopt;
        const double flat = std::abs(u.x() * v.y() - u.y() * v.x()) / longest;
        if (flat < best_line) best_line = flat, lone = i;
    }
    if (best_line > 0.08) return std::nullopt;  // no three are collinear
    std::array<Vec2, 3> row;
    for (std::size_t j = 0, k = 0; j < 4; ++j)
        if (j != lone) row[k++] = big[j];
    std::size_t e1 = 0, e2 = 1;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = i + 1; j < 3; ++j)
            if ((row[i] - row[j]).norm() > (row[e1] - row[e2]).norm()) e1 = i, e2 = j;
    const Vec2 mid = row[3 - e1 - e2];
    const bool first_is_col2 = (row[e1] - mid).norm() > (row[e2] - mid).norm();
    return std::array<Vec2, 4>{first_is_col2 ? row[e1] : row[e2], mid, first_is_col2 ? row[e2] : row[e1], big[lone]};
}

// Grid association from an initial grid -> pixel map: each grid dot takes the nearest ellipse within a
// fraction of the local grid pitch; the homography is refitted and the association repeated.
BoardDetection associate(const std::vector<Vec2>& centres, Mat3 H, const BoardSpec& board, double gate) {
    BoardDetection d;
    for (int iter = 0; iter < 4; ++iter) {
        d.grid.clear();
        d.pixels.clear();
        for (int gy = 0; gy < board.rows; ++gy)
            for (int gx = 0; gx < board.cols; ++gx) {
                const Vec2 g(gx, gy);
                const Vec2 pred = apply_homography(H, g);
                const double pitch = 0.5 * ((apply_homography(H, g + Vec2(1, 0)) - pred).norm() + (apply_homography(H, g + Vec2(0, 1)) - pred).norm());
                std::size_t best = centres.size();
                double bd = gate * pitch;
                for (std::size_t i = 0; i < centres.size(); ++i)
                    if (const double dist = (centres[i] - pred).norm(); dist < bd) bd = dist, best = i;
                if (best < centres.size()) d.grid.push_back(g), d.pixels.push_back(centres[best]);
            }
        if (d.grid.size() < 8) return d;
        H = fit_homography(d.grid, d.pixels);
    }
    d.H = H;
    double ss = 0;
    for (std::size_t i = 0; i < d.grid.size(); ++i) ss += (apply_homography(H, d.grid[i]) - d.pixels[i]).squaredNorm();
    d.residual_px = std::sqrt(ss / static_cast<double>(d.grid.size()));
    return d;
}

}  // namespace

std::array<Vec3, 4> BoardSpec::outline() const {
    const double x0 = -0.5 * pitch_mm, y0 = -0.5 * pitch_mm, x1 = (cols - 0.5) * pitch_mm, y1 = (rows - 0.5) * pitch_mm;
    return {Vec3(x0, y0, 0), Vec3(x1, y0, 0), Vec3(x1, y1, 0), Vec3(x0, y1, 0)};
}

BoardDetectParams board_detect_params() {
    BoardDetectParams p;
    p.markers.max_diameter_px = 200;  // large dots on a board ~150 mm away
    return p;
}

Mat3 fit_homography(const std::vector<Vec2>& src, const std::vector<Vec2>& dst) {
    // Hartley normalisation of both point sets.
    auto normaliser = [](const std::vector<Vec2>& p) {
        Vec2 c = Vec2::Zero();
        for (const auto& q : p) c += q;
        c /= static_cast<double>(p.size());
        double s = 0;
        for (const auto& q : p) s += (q - c).norm();
        s = s > 0 ? std::sqrt(2.0) * static_cast<double>(p.size()) / s : 1.0;
        Mat3 T;
        T << s, 0, -s * c.x(), 0, s, -s * c.y(), 0, 0, 1;
        return T;
    };
    const Mat3 Ts = normaliser(src), Td = normaliser(dst);
    Eigen::MatrixXd A(2 * src.size(), 9);
    for (std::size_t i = 0; i < src.size(); ++i) {
        const Vec2 s = apply_homography(Ts, src[i]), d = apply_homography(Td, dst[i]);
        const double x = s.x(), y = s.y(), u = d.x(), v = d.y();
        A.row(static_cast<Eigen::Index>(2 * i)) << -x, -y, -1, 0, 0, 0, u * x, u * y, u;
        A.row(static_cast<Eigen::Index>(2 * i + 1)) << 0, 0, 0, -x, -y, -1, v * x, v * y, v;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    const Eigen::VectorXd h = svd.matrixV().col(8);
    Mat3 Hn;
    Hn << h(0), h(1), h(2), h(3), h(4), h(5), h(6), h(7), h(8);
    Mat3 H = Td.inverse() * Hn * Ts;
    return H / H(2, 2);
}

Vec2 apply_homography(const Mat3& H, const Vec2& p) {
    const Vec3 q = H * Vec3(p.x(), p.y(), 1);
    return {q.x() / q.z(), q.y() / q.z()};
}

std::optional<BoardDetection> associate_board(const std::vector<markers::Ellipse>& ellipses, const BoardSpec& board,
                                              const BoardDetectParams& params) {
    if (static_cast<int>(ellipses.size()) < params.min_dots) return std::nullopt;
    std::vector<double> sizes;
    for (const auto& e : ellipses) sizes.push_back(e.a + e.b);
    std::ranges::nth_element(sizes, sizes.begin() + static_cast<std::ptrdiff_t>(sizes.size() / 2));
    const double median = sizes[sizes.size() / 2];
    std::vector<Vec2> centres, large;
    for (const auto& e : ellipses) {
        centres.push_back(e.center);
        if (e.a + e.b > 1.4 * median) large.push_back(e.center);
    }
    if (large.size() < 4 || large.size() > 8) return std::nullopt;
    // Every choice of four large candidates (usually exactly four): the one explaining the most dots wins.
    std::optional<BoardDetection> best;
    const std::size_t n = large.size();
    for (std::size_t a = 0; a < n; ++a)
        for (std::size_t b = a + 1; b < n; ++b)
            for (std::size_t c = b + 1; c < n; ++c)
                for (std::size_t e = c + 1; e < n; ++e) {
                    const auto ordered = order_large({large[a], large[b], large[c], large[e]});
                    if (!ordered) continue;
                    auto d = associate(centres, affine_from_large(board.large, *ordered), board, params.gate);
                    if (static_cast<int>(d.grid.size()) < params.min_dots) continue;
                    if (!best || d.grid.size() > best->grid.size() || (d.grid.size() == best->grid.size() && d.residual_px < best->residual_px)) {
                        d.large = *ordered;
                        best = std::move(d);
                    }
                }
    if (best) best->blobs = static_cast<int>(ellipses.size());
    return best;
}

std::optional<BoardDetection> detect_board(ImageView<const std::uint8_t> image, const BoardSpec& board, const BoardDetectParams& params) {
    return associate_board(markers::detect_markers(image, params.markers), board, params);
}

std::optional<BoardPose> board_pose(const BoardDetection& d, const CameraModel& cam, const BoardSpec& board) {
    if (d.grid.size() < 6) return std::nullopt;
    std::vector<Vec2> xy, n;
    for (std::size_t i = 0; i < d.grid.size(); ++i) {
        xy.push_back(board.point(d.grid[i]).head<2>());
        n.push_back(calib::undistort_to_normalized(cam, d.pixels[i]));
    }
    // Homography board (mm) -> normalised image: columns are r1, r2, t up to scale.
    const Mat3 H = fit_homography(xy, n);
    const double lambda = 2.0 / (H.col(0).norm() + H.col(1).norm());
    Mat3 R;
    R.col(0) = lambda * H.col(0);
    R.col(1) = lambda * H.col(1);
    R.col(2) = R.col(0).cross(R.col(1));
    Vec3 t = lambda * H.col(2);
    if (t.z() < 0) R.col(0) = -R.col(0), R.col(1) = -R.col(1), t = -t;
    Eigen::JacobiSVD<Mat3> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) R = svd.matrixU() * Vec3(1, 1, -1).asDiagonal() * svd.matrixV().transpose();
    BoardPose out;
    out.T_cam_board.linear() = R;
    out.T_cam_board.translation() = t;

    // Levenberg-Marquardt on the pixel reprojection error (6 dof, numeric Jacobian).
    const auto m = static_cast<Eigen::Index>(2 * d.grid.size());
    auto residuals = [&](const SE3& T, Eigen::VectorXd& e) {
        for (std::size_t i = 0; i < d.grid.size(); ++i) {
            const Vec3 c = T * board.point(d.grid[i]);
            if (c.z() <= 1) return false;
            const Vec2 r = cam.project(c) - d.pixels[i];
            e(static_cast<Eigen::Index>(2 * i)) = r.x();
            e(static_cast<Eigen::Index>(2 * i + 1)) = r.y();
        }
        return true;
    };
    auto perturb = [](const SE3& T, const Eigen::Matrix<double, 6, 1>& s) {
        SE3 P = T;
        const Vec3 w = s.head<3>();
        if (w.norm() > 0) P.linear() = Eigen::AngleAxisd(w.norm(), w.normalized()).toRotationMatrix() * T.linear();
        P.translation() = T.translation() + s.tail<3>();
        return P;
    };
    Eigen::VectorXd e(m), e2(m);
    if (!residuals(out.T_cam_board, e)) return std::nullopt;
    double cost = e.squaredNorm(), mu = 1e-3;
    for (int it = 0; it < 30; ++it) {
        Eigen::MatrixXd J(m, 6);
        for (int k = 0; k < 6; ++k) {
            Eigen::Matrix<double, 6, 1> s = Eigen::Matrix<double, 6, 1>::Zero();
            s(k) = k < 3 ? 1e-6 : 1e-4;
            if (!residuals(perturb(out.T_cam_board, s), e2)) return std::nullopt;
            J.col(k) = (e2 - e) / s(k);
        }
        const Eigen::Matrix<double, 6, 6> JtJ = J.transpose() * J;
        const Eigen::Matrix<double, 6, 1> g = J.transpose() * e;
        bool improved = false;
        for (int tries = 0; tries < 8 && !improved; ++tries) {
            Eigen::Matrix<double, 6, 6> A = JtJ;
            A.diagonal() *= 1 + mu;
            const Eigen::Matrix<double, 6, 1> step = -A.ldlt().solve(g);
            const SE3 P = perturb(out.T_cam_board, step);
            if (residuals(P, e2) && e2.squaredNorm() < cost) {
                const double gain = cost - e2.squaredNorm();
                out.T_cam_board = P;
                e = e2;
                cost = e.squaredNorm();
                mu = std::max(mu * 0.3, 1e-9);
                improved = true;
                if (gain < 1e-10 * cost) it = 30;
            } else {
                mu *= 10;
            }
        }
        if (!improved) break;
    }
    out.rms_px = std::sqrt(cost / static_cast<double>(m));
    return out;
}

}  // namespace einstar::calibrate
