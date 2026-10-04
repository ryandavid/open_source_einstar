#include "board_bundle.hpp"

#include <cmath>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace einstar::calibrate::detail {
namespace {

// Brown-Conrady projection of a camera-frame point, as CameraModel::project, with the skew fixed.
template <typename T>
void project(const T* intr, const T* dist, double skew, const T* p, T* uv) {
    const T x = p[0] / p[2], y = p[1] / p[2];
    const T r2 = x * x + y * y;
    const T radial = T(1) + dist[0] * r2 + dist[1] * r2 * r2 + dist[4] * r2 * r2 * r2;
    const T xd = x * radial + T(2) * dist[2] * x * y + dist[3] * (r2 + T(2) * x * x);
    const T yd = y * radial + dist[2] * (r2 + T(2) * y * y) + T(2) * dist[3] * x * y;
    uv[0] = intr[0] * xd + T(skew) * yd + intr[2];
    uv[1] = intr[1] * yd + intr[3];
}

// One dot in one view, both cameras; the dot's board position is a parameter.
struct DotCost {
    Vec2 left, right;
    double skew_l, skew_r;

    template <typename T>
    bool operator()(const T* pose, const T* rig, const T* il, const T* dl, const T* ir, const T* dr, const T* dot, T* res) const {
        T pl[3], pr[3];
        ceres::AngleAxisRotatePoint(pose, dot, pl);
        for (int i = 0; i < 3; ++i) pl[i] += pose[3 + i];
        ceres::AngleAxisRotatePoint(rig, pl, pr);
        for (int i = 0; i < 3; ++i) pr[i] += rig[3 + i];
        T ul[2], ur[2];
        project(il, dl, skew_l, pl, ul);
        project(ir, dr, skew_r, pr, ur);
        res[0] = ul[0] - T(left.x());
        res[1] = ul[1] - T(left.y());
        res[2] = ur[0] - T(right.x());
        res[3] = ur[1] - T(right.y());
        return true;
    }
};

struct DotPrior {
    Vec3 nominal;
    double inv_sigma;

    template <typename T>
    bool operator()(const T* dot, T* res) const {
        for (int i = 0; i < 3; ++i) res[i] = (dot[i] - T(nominal(i))) * T(inv_sigma);
        return true;
    }
};

std::array<double, 6> to_params(const SE3& T) {
    std::array<double, 6> p{};
    const Mat3 R = T.linear();
    ceres::RotationMatrixToAngleAxis(ceres::ColumnMajorAdapter3x3(R.data()), p.data());
    for (int i = 0; i < 3; ++i) p[static_cast<std::size_t>(3 + i)] = T.translation()(i);
    return p;
}

SE3 from_params(const std::array<double, 6>& p) {
    Mat3 R;
    ceres::AngleAxisToRotationMatrix(p.data(), ceres::ColumnMajorAdapter3x3(R.data()));
    SE3 T = SE3::Identity();
    T.linear() = R;
    T.translation() = Vec3(p[3], p[4], p[5]);
    return T;
}

}  // namespace

BoardBundleResult refine_with_board(const RigCalibration& initial, const std::vector<optim::BoardView>& views, const std::vector<std::vector<int>>& ids,
                                    const std::vector<Vec3>& nominal, const BoardBundleOptions& o) {
    std::array<double, 4> il{initial.left.fx, initial.left.fy, initial.left.cx, initial.left.cy};
    std::array<double, 4> ir{initial.right.fx, initial.right.fy, initial.right.cx, initial.right.cy};
    std::array<double, 5> dl = initial.left.dist, dr = initial.right.dist;
    std::array<double, 6> rig = to_params(initial.T_right_left);
    std::vector<std::array<double, 6>> poses;
    for (const auto& v : views) poses.push_back(to_params(v.T_left_board));
    std::vector<std::array<double, 3>> dots;
    for (const auto& p : nominal) dots.push_back({p.x(), p.y(), p.z()});

    ceres::Problem problem;
    std::vector<bool> seen(nominal.size(), false);
    for (std::size_t vi = 0; vi < views.size(); ++vi) {
        const auto& v = views[vi];
        for (std::size_t k = 0; k < v.board.size(); ++k) {
            const auto id = static_cast<std::size_t>(ids[vi][k]);
            seen[id] = true;
            auto* cost = new ceres::AutoDiffCostFunction<DotCost, 4, 6, 6, 4, 5, 4, 5, 3>(new DotCost{v.left[k], v.right[k], initial.left.skew, initial.right.skew});
            problem.AddResidualBlock(cost, new ceres::HuberLoss(o.huber_px), poses[vi].data(), rig.data(), il.data(), dl.data(), ir.data(), dr.data(),
                                     dots[id].data());
        }
    }
    for (std::size_t i = 0; i < nominal.size(); ++i)
        if (seen[i])
            problem.AddResidualBlock(new ceres::AutoDiffCostFunction<DotPrior, 3, 3>(new DotPrior{nominal[i], 1.0 / o.board_sigma_mm}), nullptr, dots[i].data());
    if (!o.free_distortion) {
        problem.SetParameterBlockConstant(dl.data());
        problem.SetParameterBlockConstant(dr.data());
    }
    ceres::Solver::Options so;
    so.linear_solver_type = ceres::DENSE_SCHUR;
    so.max_num_iterations = o.max_iterations;
    so.logging_type = ceres::SILENT;
    ceres::Solver::Summary summary;
    ceres::Solve(so, &problem, &summary);

    BoardBundleResult r;
    r.rig = initial;
    r.rig.left.fx = il[0], r.rig.left.fy = il[1], r.rig.left.cx = il[2], r.rig.left.cy = il[3];
    r.rig.right.fx = ir[0], r.rig.right.fy = ir[1], r.rig.right.cx = ir[2], r.rig.right.cy = ir[3];
    r.rig.left.dist = dl;
    r.rig.right.dist = dr;
    r.rig.T_right_left = from_params(rig);
    for (const auto& p : poses) r.T_left_board.push_back(from_params(p));
    for (const auto& d : dots) r.board.emplace_back(d[0], d[1], d[2]);
    double ss = 0, n = 0;
    for (std::size_t vi = 0; vi < views.size(); ++vi) {
        const SE3 Tl = r.T_left_board[vi], Tr = r.rig.T_right_left * Tl;
        for (std::size_t k = 0; k < views[vi].board.size(); ++k) {
            const Vec3& p = r.board[static_cast<std::size_t>(ids[vi][k])];
            ss += (r.rig.left.project(Tl * p) - views[vi].left[k]).squaredNorm() + (r.rig.right.project(Tr * p) - views[vi].right[k]).squaredNorm();
            n += 2;
        }
    }
    r.rms = n > 0 ? std::sqrt(ss / n) : 0.0;
    return r;
}

}  // namespace einstar::calibrate::detail
