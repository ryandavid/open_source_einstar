#include "einstar/optim/marker_bundle.hpp"

#include <cmath>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace einstar::optim {
namespace {

// Camera parameterised as camera_T_world: quaternion (x, y, z, w) + translation.
struct StereoReprojection {
    StereoReprojection(const Vec2& l, const Vec2& r, const depth::RectifiedGeometry& g) : l_(l), r_(r), g_(g) {}

    template <typename T>
    bool operator()(const T* q, const T* t, const T* X, T* res) const {
        const Eigen::Quaternion<T> quat(q[3], q[0], q[1], q[2]);
        const Eigen::Matrix<T, 3, 1> Xw(X[0], X[1], X[2]);
        const Eigen::Matrix<T, 3, 1> p = quat * Xw + Eigen::Matrix<T, 3, 1>(t[0], t[1], t[2]);
        if (p.z() <= T(1e-3)) return false;
        const T f(g_.f), cx(g_.cx), cy(g_.cy), B(g_.baseline);
        res[0] = f * p.x() / p.z() + cx - T(l_.x());
        res[1] = f * p.y() / p.z() + cy - T(l_.y());
        res[2] = f * (p.x() - B) / p.z() + cx - T(r_.x());
        res[3] = f * p.y() / p.z() + cy - T(r_.y());
        return true;
    }

    static ceres::CostFunction* create(const Vec2& l, const Vec2& r, const depth::RectifiedGeometry& g) {
        return new ceres::AutoDiffCostFunction<StereoReprojection, 4, 4, 3, 3>(new StereoReprojection(l, r, g));
    }

    Vec2 l_, r_;
    depth::RectifiedGeometry g_;
};

Eigen::Vector4d residual(const MarkerBundle& b, const MarkerObservation& o) {
    const SE3 T_cw = b.T_world_camera.at(o.frame).inverse();
    const Vec3 p = T_cw * b.markers.at(o.marker);
    const auto& g = b.geometry;
    return {g.f * p.x() / p.z() + g.cx - o.left.x(), g.f * p.y() / p.z() + g.cy - o.left.y(),
            g.f * (p.x() - g.baseline) / p.z() + g.cx - o.right.x(), g.f * p.y() / p.z() + g.cy - o.right.y()};
}

}  // namespace

double reprojection_rms(const MarkerBundle& b) {
    double ss = 0;
    int n = 0;
    for (const auto& o : b.observations) {
        if (!b.T_world_camera.contains(o.frame) || !b.markers.contains(o.marker)) continue;
        ss += residual(b, o).squaredNorm();
        n += 4;
    }
    return n ? std::sqrt(ss / n) : 0.0;
}

BundleReport optimize(MarkerBundle& b, const BundleParams& p) {
    BundleReport rep;
    rep.rms_before_px = reprojection_rms(b);
    const int fixed = p.fixed_frame >= 0 ? p.fixed_frame : (b.T_world_camera.empty() ? -1 : b.T_world_camera.begin()->first);

    for (int pass = 0; pass < 2; ++pass) {
        // Parameter storage.
        std::map<int, std::array<double, 7>> cams;
        for (const auto& [id, T] : b.T_world_camera) {
            const SE3 cw = T.inverse();
            const Eigen::Quaterniond q(cw.linear());
            cams[id] = {q.x(), q.y(), q.z(), q.w(), cw.translation().x(), cw.translation().y(), cw.translation().z()};
        }
        std::map<int, std::array<double, 3>> pts;
        for (const auto& [id, X] : b.markers) pts[id] = {X.x(), X.y(), X.z()};

        ceres::Problem problem;
        int used = 0;
        for (const auto& o : b.observations) {
            auto c = cams.find(o.frame);
            auto x = pts.find(o.marker);
            if (c == cams.end() || x == pts.end()) continue;
            problem.AddResidualBlock(StereoReprojection::create(o.left, o.right, b.geometry), new ceres::HuberLoss(p.huber_px),
                                     c->second.data(), c->second.data() + 4, x->second.data());
            ++used;
        }
        if (used == 0) return rep;
        for (auto& [id, c] : cams) {
            if (!problem.HasParameterBlock(c.data())) continue;
            problem.SetManifold(c.data(), new ceres::EigenQuaternionManifold);
            if (id == fixed) {
                problem.SetParameterBlockConstant(c.data());
                problem.SetParameterBlockConstant(c.data() + 4);
            }
        }
        ceres::Solver::Options opt;
        opt.linear_solver_type = ceres::SPARSE_SCHUR;
        opt.max_num_iterations = p.max_iterations;
        opt.num_threads = 8;
        ceres::Solver::Summary summary;
        ceres::Solve(opt, &problem, &summary);
        rep.converged = summary.termination_type == ceres::CONVERGENCE;

        for (const auto& [id, c] : cams) {
            SE3 cw = SE3::Identity();
            cw.linear() = Eigen::Quaterniond(c[3], c[0], c[1], c[2]).normalized().toRotationMatrix();
            cw.translation() = Vec3(c[4], c[5], c[6]);
            b.T_world_camera[id] = cw.inverse();
        }
        for (const auto& [id, x] : pts) b.markers[id] = Vec3(x[0], x[1], x[2]);

        // Drop outlier observations and re-solve once.
        if (pass == 0) {
            const auto before = b.observations.size();
            std::erase_if(b.observations, [&](const MarkerObservation& o) {
                return b.T_world_camera.contains(o.frame) && b.markers.contains(o.marker) &&
                       residual(b, o).cwiseAbs().maxCoeff() > p.outlier_px;
            });
            rep.outliers_removed = static_cast<int>(before - b.observations.size());
            if (rep.outliers_removed == 0) break;
        }
    }
    rep.observations = static_cast<int>(b.observations.size());
    rep.rms_after_px = reprojection_rms(b);
    return rep;
}

}  // namespace einstar::optim
