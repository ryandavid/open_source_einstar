#include "einstar/track/icp.hpp"

#include <array>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

namespace einstar::track {
namespace {

struct Normal6 {
    Mat6 A = Mat6::Zero();
    Vec6 b = Vec6::Zero();
    double sq = 0;
    int n = 0;
    int considered = 0;
    int on_model = 0;

    Normal6& operator+=(const Normal6& o) {
        A += o.A;
        b += o.b;
        sq += o.sq;
        n += o.n;
        considered += o.considered;
        on_model += o.on_model;
        return *this;
    }
};

// Smallest/largest eigenvalue of the Hessian with rotations expressed as displacement at a
// 100 mm lever arm, so translation and rotation are comparable (a unit-free degeneracy measure).
double eigen_ratio(const Mat6& A) {
    Vec6 s;
    s << 1, 1, 1, 1.0 / 100.0, 1.0 / 100.0, 1.0 / 100.0;
    const Mat6 As = s.asDiagonal() * A * s.asDiagonal();
    Eigen::SelfAdjointEigenSolver<Mat6> es(As);
    const auto ev = es.eigenvalues();
    return ev(5) > 0 ? ev(0) / ev(5) : 0.0;
}

// Coarse direction bin for a unit normal (octahedral-ish 8 x 16 lat/long grid).
inline int normal_bin(const Vec3f& n) {
    const float theta = std::acos(std::clamp(n.z(), -1.0f, 1.0f));      // 0..pi
    const float phi = std::atan2(n.y(), n.x()) + static_cast<float>(M_PI);  // 0..2pi
    const int bt = std::min(7, static_cast<int>(theta / static_cast<float>(M_PI) * 8.0f));
    const int bp = std::min(15, static_cast<int>(phi / (2.0f * static_cast<float>(M_PI)) * 16.0f));
    return bt * 16 + bp;
}

}  // namespace

// Twists (v, w) about the origin vs about a point c: v_origin = v_c + c x w (w is unchanged).
Vec6 twist_from_center(const Vec6& xc, const Vec3& c) {
    Vec6 xo = xc;
    xo.head<3>() += c.cross(xc.tail<3>());
    return xo;
}
Vec6 twist_to_center(const Vec6& xo, const Vec3& c) {
    Vec6 xc = xo;
    xc.head<3>() -= c.cross(xo.tail<3>());
    return xc;
}

Vec3 icp_center(const DepthFrame& frame, const SE3& T_world_camera) {
    Vec3 c = Vec3::Zero();
    int n = 0;
    const int w = frame.width(), h = frame.height();
    // GPU frames are read in place (shared memory); only a sparse subsample is touched.
    const float* dev = frame.points.empty() && frame.device ? frame.device->points_xyzw() : nullptr;
    for (int y = 0; y < h; y += 16)
        for (int x = 0; x < w; x += 16) {
            Vec3f p;
            if (dev) {
                const float* q = dev + 4 * (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x));
                p = Vec3f(q[0], q[1], q[2]);
            } else {
                p = frame.points(x, y);
            }
            if (p.z() > 0) {
                c += T_world_camera * p.cast<double>();
                ++n;
            }
        }
    return n > 0 ? Vec3(c / n) : T_world_camera.translation();
}

std::vector<float> normal_balance_weights(const DepthFrame& frame, double alpha) {
    std::vector<float> balance;
    if (alpha <= 0) return balance;
    frame.ensure_cpu();
    const int W = frame.points.width(), H = frame.points.height();
    std::array<int, 128> hist{};
    int total = 0;
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            const Vec3f& n = frame.normals(x, y);
            if (frame.points(x, y).z() <= 0 || n.squaredNorm() == 0) continue;
            ++hist[static_cast<std::size_t>(normal_bin(n))];
            ++total;
        }
    int occupied = 0;
    for (int c : hist) occupied += c > 0;
    const double mean = occupied ? static_cast<double>(total) / occupied : 1.0;
    std::array<float, 128> w_bin{};
    for (std::size_t b = 0; b < hist.size(); ++b)
        w_bin[b] = hist[b] ? static_cast<float>(std::pow(mean / hist[b], alpha)) : 0.0f;
    // Cap the boost so a handful of noisy normals cannot dominate either.
    for (auto& w : w_bin) w = std::min(w, 20.0f);
    balance.assign(static_cast<std::size_t>(W * H), 1.0f);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            const Vec3f& n = frame.normals(x, y);
            if (n.squaredNorm() > 0) balance[static_cast<std::size_t>(y * W + x)] = w_bin[static_cast<std::size_t>(normal_bin(n))];
        }
    return balance;
}

IcpResult icp_point_to_plane(const DepthFrame& frame, const RaycastResult& model, const SE3& T_model_camera,
                             const SE3& T_init, const IcpParams& p) {
    frame.ensure_cpu();
    model.ensure_cpu();
    IcpResult res;
    SE3 T = T_init;
    // Linearise about the centroid of the observed surface rather than the world origin: the
    // rotation/translation coupling (and so the degeneracy analysis) then does not depend on how
    // far the scanner is from where the scan started, and the system stays well conditioned.
    const Vec3 c = icp_center(frame, T_init);
    res.center = c;
    const SE3 T_mc_inv = T_model_camera.inverse();  // world -> model camera
    const Eigen::Matrix4d M_cw = T_mc_inv.matrix();
    const float cos_max = std::cos(p.max_normal_angle_deg * static_cast<float>(M_PI) / 180.0f);
    const auto& mk = model.intrinsics;
    const int W = frame.points.width(), H = frame.points.height();

    const std::vector<float> balance = normal_balance_weights(frame, p.normal_balance_alpha);

    for (int level = p.levels - 1; level >= 0; --level) {
        const int step = 1 << level;
        const double gate = static_cast<double>(p.max_distance_mm) * (1 + level);
        const double huber = static_cast<double>(p.huber_mm) * (1 + level);
        const int iters = p.iterations[static_cast<std::size_t>(std::min(level, 2))];
        for (int it = 0; it < iters; ++it) {
            const Eigen::Matrix4d Tm = T.matrix();
            const Normal6 sys = tbb::parallel_reduce(
                tbb::blocked_range<int>(0, H / step), Normal6{},
                [&](const tbb::blocked_range<int>& rows, Normal6 acc) {
                    for (int ry = rows.begin(); ry != rows.end(); ++ry) {
                        const int y = ry * step;
                        for (int x = 0; x < W; x += step) {
                            const Vec3f& pc = frame.points(x, y);
                            if (pc.z() <= 0) continue;
                            ++acc.considered;
                            const Eigen::Vector4d pw4 = Tm * Eigen::Vector4d(pc.x(), pc.y(), pc.z(), 1.0);
                            const Eigen::Vector4d pm = M_cw * pw4;
                            if (pm.z() <= 0) continue;
                            const int u = static_cast<int>(std::lround(mk.fx * pm.x() / pm.z() + mk.cx));
                            const int v = static_cast<int>(std::lround(mk.fy * pm.y() / pm.z() + mk.cy));
                            if (u < 0 || v < 0 || u >= mk.width || v >= mk.height || !model.valid(u, v)) continue;
                            ++acc.on_model;
                            const Vec3 q = model.points(u, v).cast<double>();
                            const Vec3 nq = model.normals(u, v).cast<double>();
                            const Vec3 pw = pw4.head<3>();
                            const double r = nq.dot(pw - q);
                            if (std::abs(r) > gate || (pw - q).norm() > 2 * gate) continue;
                            const Vec3f& ns = frame.normals(x, y);
                            if (ns.squaredNorm() > 0) {
                                const Vec3 nsw = T.linear() * ns.cast<double>();
                                if (nsw.dot(nq) < cos_max) continue;
                            }
                            const double w_huber = std::abs(r) <= huber ? 1.0 : huber / std::abs(r);
                            double w = w_huber * std::max(0.1f, frame.weights.empty() ? 1.0f : frame.weights(x, y));
                            if (!balance.empty()) w *= balance[static_cast<std::size_t>(y * W + x)];
                            Vec6 J;
                            J.head<3>() = nq;             // translation (left-perturbation twist v, w)
                            J.tail<3>() = (pw - c).cross(nq);  // rotation about the centroid
                            acc.A.noalias() += w * J * J.transpose();
                            acc.b.noalias() -= w * J * r;
                            acc.sq += w * r * r;
                            ++acc.n;
                        }
                    }
                    return acc;
                },
                [](Normal6 a, const Normal6& b) { return a += b; });

            if (sys.n < 50) {
                res.converged = false;
                res.correspondences = sys.n;
                return res;
            }
            // Solve in unit-scaled coordinates (rotations as displacement at a 100 mm lever arm).
            Vec6 scale;
            scale << 1, 1, 1, 0.01, 0.01, 0.01;  // x = S xs; xs rotations are mm of displacement at 100 mm
            const Mat6 As = scale.asDiagonal() * sys.A * scale.asDiagonal();
            const Vec6 bs = scale.cwiseProduct(sys.b);
            // Step back to the prediction, expressed as a twist about the centroid.
            const Vec6 prior_s = twist_to_center(se3_log(T_init * T.inverse()), c).cwiseQuotient(scale);

            // Weak prior (motion-model uncertainty vs depth noise).
            const double s2 = p.data_sigma_mm * p.data_sigma_mm;
            const double sr_mm = p.prior_sigma_deg * M_PI / 180.0 * 100.0;
            Vec6 lambda;
            lambda.head<3>().setConstant(s2 / (p.prior_sigma_mm * p.prior_sigma_mm));
            lambda.tail<3>().setConstant(s2 / (sr_mm * sr_mm));
            Mat6 Ar = As;
            Ar.diagonal() += lambda;
            Vec6 xs = Ar.ldlt().solve(bs + lambda.cwiseProduct(prior_s));

            // Degeneracy-aware remapping: in eigen-directions the geometry cannot constrain, follow
            // the prediction rather than noise (prevents sliding on cylinders, planes, grooves).
            Eigen::SelfAdjointEigenSolver<Mat6> es(As);
            const auto& ev = es.eigenvalues();
            const auto& V = es.eigenvectors();
            int degenerate = 0;
            Mat6 basis = Mat6::Zero();
            if (ev(5) > 0) {
                for (int d = 0; d < 6; ++d) {
                    if (ev(d) >= p.degenerate_direction_ratio * ev(5)) continue;
                    const Vec6 v = V.col(d);
                    xs += v * (v.dot(prior_s) - v.dot(xs));
                    basis.col(degenerate++) = v;
                }
            }
            if (level == 0) res.degenerate_basis = basis;
            const Vec6 dx = scale.cwiseProduct(xs);
            if (level == 0) res.degenerate_directions = degenerate;
            T = se3_exp(twist_from_center(dx, c)) * T;

            if (level == 0 && it == iters - 1) {
                res.min_eigenvalue_ratio = eigen_ratio(sys.A);
                res.correspondences = sys.n;
                res.candidates = sys.considered;
                res.rms_mm = std::sqrt(sys.sq / std::max(1.0, static_cast<double>(sys.n)));
                res.inlier_ratio = sys.on_model ? static_cast<double>(sys.n) / sys.on_model : 0.0;
                res.coverage = sys.considered ? static_cast<double>(sys.on_model) / sys.considered : 0.0;
            }
            if (dx.head<3>().norm() < 1e-4 && dx.tail<3>().norm() < 1e-6) {
                if (level == 0) {
                    res.min_eigenvalue_ratio = eigen_ratio(sys.A);
                    res.correspondences = sys.n;
                    res.candidates = sys.considered;
                    res.rms_mm = std::sqrt(sys.sq / std::max(1.0, static_cast<double>(sys.n)));
                    res.inlier_ratio = sys.on_model ? static_cast<double>(sys.n) / sys.on_model : 0.0;
                    res.coverage = sys.considered ? static_cast<double>(sys.on_model) / sys.considered : 0.0;
                }
                break;
            }
        }
    }
    res.T_world_camera = T;
    res.converged = true;
    return res;
}

}  // namespace einstar::track
