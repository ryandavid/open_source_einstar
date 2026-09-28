#include "einstar/optim/pose_graph.hpp"

#include <array>
#include <cmath>

#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include "einstar/core/log.hpp"

namespace einstar::optim {
namespace {

struct NodeParams {
    std::array<double, 4> q;  // Eigen order: x, y, z, w
    std::array<double, 3> t;
};

NodeParams to_params(const SE3& T) {
    const Eigen::Quaterniond q(T.linear());
    return {{q.x(), q.y(), q.z(), q.w()}, {T.translation().x(), T.translation().y(), T.translation().z()}};
}

SE3 from_params(const NodeParams& p) {
    SE3 T = SE3::Identity();
    T.linear() = Eigen::Quaterniond(p.q[3], p.q[0], p.q[1], p.q[2]).normalized().toRotationMatrix();
    T.translation() = Vec3(p.t[0], p.t[1], p.t[2]);
    return T;
}

struct EdgeCost {
    Eigen::Quaterniond q_meas;
    Vec3 t_meas;
    Mat6 sqrt_info;  // L^T with information = L L^T

    template <typename T>
    bool operator()(const T* qi_p, const T* ti_p, const T* qj_p, const T* tj_p, T* r) const {
        const Eigen::Map<const Eigen::Quaternion<T>> qi(qi_p), qj(qj_p);
        const Eigen::Map<const Eigen::Matrix<T, 3, 1>> ti(ti_p), tj(tj_p);
        const Eigen::Quaternion<T> q_est = qi.conjugate() * qj;
        const Eigen::Matrix<T, 3, 1> t_est = qi.conjugate() * (tj - ti);
        const Eigen::Quaternion<T> qm = q_meas.cast<T>();
        Eigen::Quaternion<T> q_err = qm.conjugate() * q_est;
        const Eigen::Matrix<T, 3, 1> t_err = qm.conjugate() * (t_est - t_meas.cast<T>());
        if (q_err.w() < T(0)) q_err.coeffs() = -q_err.coeffs();
        Eigen::Matrix<T, 6, 1> e;
        e.template head<3>() = T(2) * q_err.vec();
        e.template tail<3>() = t_err;
        Eigen::Map<Eigen::Matrix<T, 6, 1>> res(r);
        res = sqrt_info.cast<T>() * e;
        return true;
    }
};

struct PriorCost {
    Eigen::Quaterniond q_prior;
    Vec3 t_prior;
    Mat6 sqrt_info;

    template <typename T>
    bool operator()(const T* q_p, const T* t_p, T* r) const {
        const Eigen::Map<const Eigen::Quaternion<T>> q(q_p);
        const Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(t_p);
        const Eigen::Quaternion<T> qp = q_prior.cast<T>();
        Eigen::Quaternion<T> q_err = qp.conjugate() * q;
        const Eigen::Matrix<T, 3, 1> t_err = qp.conjugate() * (t - t_prior.cast<T>());
        if (q_err.w() < T(0)) q_err.coeffs() = -q_err.coeffs();
        Eigen::Matrix<T, 6, 1> e;
        e.template head<3>() = T(2) * q_err.vec();
        e.template tail<3>() = t_err;
        Eigen::Map<Eigen::Matrix<T, 6, 1>> res(r);
        res = sqrt_info.cast<T>() * e;
        return true;
    }
};

struct LandmarkCost {
    Vec3 p;
    double inv_sigma;

    template <typename T>
    bool operator()(const T* q_p, const T* t_p, const T* m_p, T* r) const {
        const Eigen::Map<const Eigen::Quaternion<T>> q(q_p);
        const Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(t_p), m(m_p);
        Eigen::Map<Eigen::Matrix<T, 3, 1>> res(r);
        res = (q * p.cast<T>() + t - m) * T(inv_sigma);
        return true;
    }
};

Mat6 sqrt_information(const Mat6& info) {
    Eigen::LLT<Mat6> llt(0.5 * (info + info.transpose()));
    if (llt.info() == Eigen::Success) return llt.matrixL().transpose();
    // Not positive definite (fully degenerate direction): regularise slightly.
    Eigen::LLT<Mat6> llt2(0.5 * (info + info.transpose()) + 1e-9 * info.trace() * Mat6::Identity() + 1e-12 * Mat6::Identity());
    return llt2.matrixL().transpose();
}

double solve(PoseGraph& g, const PoseGraphParams& params, double& cost_before) {
    std::map<int, NodeParams> np;
    for (const auto& [id, T] : g.nodes) np[id] = to_params(T);
    std::map<int, std::array<double, 3>> lm;
    for (const auto& [id, p] : g.landmarks) lm[id] = {p.x(), p.y(), p.z()};

    ceres::Problem problem;
    for (const auto& e : g.edges) {
        if (!np.contains(e.i) || !np.contains(e.j)) continue;
        auto* cost = new ceres::AutoDiffCostFunction<EdgeCost, 6, 4, 3, 4, 3>(
            new EdgeCost{Eigen::Quaterniond(e.T_i_j.linear()), e.T_i_j.translation(), sqrt_information(e.information)});
        auto& a = np[e.i];
        auto& b = np[e.j];
        problem.AddResidualBlock(cost, nullptr, a.q.data(), a.t.data(), b.q.data(), b.t.data());
    }
    for (const auto& p : g.priors) {
        if (!np.contains(p.node)) continue;
        auto* cost = new ceres::AutoDiffCostFunction<PriorCost, 6, 4, 3>(
            new PriorCost{Eigen::Quaterniond(p.T_prior.linear()), p.T_prior.translation(), sqrt_information(p.information)});
        auto& n = np[p.node];
        problem.AddResidualBlock(cost, nullptr, n.q.data(), n.t.data());
    }
    bool have_fixed_landmark = false;
    for (const auto& o : g.observations) {
        if (!np.contains(o.node) || !lm.contains(o.landmark)) continue;
        auto* cost = new ceres::AutoDiffCostFunction<LandmarkCost, 3, 4, 3, 3>(new LandmarkCost{o.p_node, 1.0 / o.sigma_mm});
        auto& n = np[o.node];
        problem.AddResidualBlock(cost, new ceres::HuberLoss(3.0), n.q.data(), n.t.data(), lm[o.landmark].data());
        if (g.fixed_landmarks.contains(o.landmark)) {
            problem.SetParameterBlockConstant(lm[o.landmark].data());
            have_fixed_landmark = true;
        }
    }
    for (auto& [id, p] : np)
        if (problem.HasParameterBlock(p.q.data())) problem.SetManifold(p.q.data(), new ceres::EigenQuaternionManifold());
    if (!have_fixed_landmark && !np.empty()) {
        const int fixed = params.fixed_node >= 0 && np.contains(params.fixed_node) ? params.fixed_node : np.begin()->first;
        auto& p = np[fixed];
        if (problem.HasParameterBlock(p.q.data())) {
            problem.SetParameterBlockConstant(p.q.data());
            problem.SetParameterBlockConstant(p.t.data());
        }
    }
    ceres::Solver::Options opt;
    opt.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    opt.max_num_iterations = params.max_iterations;
    opt.num_threads = 8;
    ceres::Solver::Summary summary;
    ceres::Solve(opt, &problem, &summary);
    cost_before = summary.initial_cost;
    for (auto& [id, T] : g.nodes) T = from_params(np[id]);
    for (auto& [id, p] : g.landmarks) p = Vec3(lm[id][0], lm[id][1], lm[id][2]);
    return summary.final_cost;
}

}  // namespace

Vec6 edge_error(const PoseGraph& graph, const PoseEdge& e) {
    const SE3 est = graph.nodes.at(e.i).inverse() * graph.nodes.at(e.j);
    const SE3 err = e.T_i_j.inverse() * est;
    Eigen::Quaterniond q(err.linear());
    if (q.w() < 0) q.coeffs() = -q.coeffs();
    Vec6 out;
    out.head<3>() = 2.0 * q.vec();
    out.tail<3>() = err.translation();
    return out;
}

PoseGraphReport optimize(PoseGraph& graph, const PoseGraphParams& params) {
    PoseGraphReport rep;
    for (const auto& e : graph.edges) rep.loop_edges += e.loop;
    const auto initial = graph.nodes;
    const auto initial_landmarks = graph.landmarks;
    for (int round = 0; round <= params.max_prune_rounds; ++round) {
        // Always restart from the initial poses: a wrong loop edge must not leave its distortion behind.
        graph.nodes = initial;
        graph.landmarks = initial_landmarks;
        double before = 0;
        rep.cost_after = solve(graph, params, before);
        if (round == 0) rep.cost_before = before;
        ++rep.solves;
        // Remove the worst inconsistent loop closure (one at a time: a single bad edge distorts others).
        double worst = params.prune_chi;
        std::size_t worst_i = graph.edges.size();
        for (std::size_t k = 0; k < graph.edges.size(); ++k) {
            const auto& e = graph.edges[k];
            if (!e.loop || !graph.nodes.contains(e.i) || !graph.nodes.contains(e.j)) continue;
            const Vec6 err = edge_error(graph, e);
            const double chi = std::sqrt(std::max(0.0, err.dot(e.information * err)));
            if (chi > worst) {
                worst = chi;
                worst_i = k;
            }
        }
        if (worst_i == graph.edges.size()) {
            rep.converged = true;
            break;
        }
        log::debug("pose graph: pruning loop edge {}-{} (chi {:.1f})", graph.edges[worst_i].i, graph.edges[worst_i].j, worst);
        graph.edges.erase(graph.edges.begin() + static_cast<std::ptrdiff_t>(worst_i));
        ++rep.pruned_edges;
        --rep.loop_edges;
    }
    return rep;
}

}  // namespace einstar::optim
