#include "einstar/recon/registration.hpp"

#include <algorithm>
#include <cmath>
#include <atomic>
#include <unordered_map>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <nanoflann.hpp>
#include <tbb/parallel_for.h>

namespace einstar::recon {
namespace {

struct Adaptor {
    const Cloud* c;
    [[nodiscard]] std::size_t kdtree_get_point_count() const { return c->points.size(); }
    [[nodiscard]] float kdtree_get_pt(std::size_t i, std::size_t d) const { return c->points[i][static_cast<Eigen::Index>(d)]; }
    template <class B>
    bool kdtree_get_bbox(B&) const {
        return false;
    }
};
using Tree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, Adaptor>, Adaptor, 3, std::uint32_t>;

}  // namespace

struct CloudIndex::Impl {
    Adaptor adaptor;
    Tree tree;
    explicit Impl(const Cloud& c) : adaptor{&c}, tree(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(16)) { tree.buildIndex(); }
};

CloudIndex::CloudIndex(const Cloud& cloud) : cloud_(&cloud), impl_(std::make_unique<Impl>(cloud)) {}
CloudIndex::~CloudIndex() = default;
CloudIndex::CloudIndex(CloudIndex&&) noexcept = default;
CloudIndex& CloudIndex::operator=(CloudIndex&&) noexcept = default;

int CloudIndex::nearest(const Vec3f& p, float max_mm) const {
    if (cloud_->points.empty()) return -1;
    std::uint32_t idx = 0;
    float d2 = 0;
    nanoflann::KNNResultSet<float, std::uint32_t> rs(1);
    rs.init(&idx, &d2);
    impl_->tree.findNeighbors(rs, p.data());
    return rs.size() > 0 && d2 <= max_mm * max_mm ? static_cast<int>(idx) : -1;
}

Cloud voxel_downsample(const Cloud& in, float voxel_mm) {
    struct Acc {
        Vec3f p = Vec3f::Zero(), n = Vec3f::Zero();
        int count = 0;
    };
    std::unordered_map<std::int64_t, Acc> cells;
    cells.reserve(in.size() / 2);
    const float inv = 1.0f / voxel_mm;
    for (std::size_t i = 0; i < in.size(); ++i) {
        const Vec3f& p = in.points[i];
        const auto x = static_cast<std::int64_t>(std::floor(p.x() * inv)) & 0x1FFFFF;
        const auto y = static_cast<std::int64_t>(std::floor(p.y() * inv)) & 0x1FFFFF;
        const auto z = static_cast<std::int64_t>(std::floor(p.z() * inv)) & 0x1FFFFF;
        auto& a = cells[x << 42 | y << 21 | z];
        a.p += p;
        a.n += in.normals[i];
        ++a.count;
    }
    Cloud out;
    out.points.reserve(cells.size());
    out.normals.reserve(cells.size());
    for (const auto& [k, a] : cells) {
        const float nl = a.n.norm();
        if (nl < 1e-6f) continue;
        out.points.push_back(a.p / static_cast<float>(a.count));
        out.normals.push_back(a.n / nl);
    }
    return out;
}

Cloud transformed(const Cloud& in, const SE3& T) {
    const Eigen::Matrix3f R = T.linear().cast<float>();
    const Vec3f t = T.translation().cast<float>();
    Cloud out;
    out.points.resize(in.size());
    out.normals.resize(in.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
        out.points[i] = R * in.points[i] + t;
        out.normals[i] = R * in.normals[i];
    }
    return out;
}

RegistrationResult register_point_to_plane(const Cloud& source, const CloudIndex& target, const SE3& initial,
                                           const RegistrationParams& params) {
    RegistrationResult res;
    res.T_target_source = initial;
    if (source.empty() || target.cloud().empty()) return res;
    // Evenly strided subset of the source.
    std::vector<std::uint32_t> sel;
    const std::size_t stride = std::max<std::size_t>(1, source.size() / params.max_source_points);
    for (std::size_t i = 0; i < source.size(); i += stride) sel.push_back(static_cast<std::uint32_t>(i));
    const auto& tc = target.cloud();

    struct Corr {
        int target = -1;
        float r = 0;
    };
    std::vector<Corr> corr(sel.size());
    SE3 T = initial;
    auto associate = [&](float gate) {
        const Eigen::Matrix3f R = T.linear().cast<float>();
        const Vec3f t = T.translation().cast<float>();
        tbb::parallel_for(std::size_t{0}, sel.size(), [&](std::size_t k) {
            const Vec3f p = R * source.points[sel[k]] + t;
            const int j = target.nearest(p, gate);
            corr[k].target = -1;
            if (j < 0) return;
            const Vec3f& n = tc.normals[static_cast<std::size_t>(j)];
            if (n.dot(R * source.normals[sel[k]]) < params.min_normal_dot) return;
            corr[k].target = j;
            corr[k].r = n.dot(p - tc.points[static_cast<std::size_t>(j)]);
        });
    };
    // Rows of the Jacobian w.r.t. the right perturbation [w; v] of T: [ (p x R^T n)^T, (R^T n)^T ].
    auto jacobian = [&](std::size_t k, const Mat3& Rt) {
        const Vec3 p = source.points[sel[k]].cast<double>();
        const Vec3 m = Rt * tc.normals[static_cast<std::size_t>(corr[k].target)].cast<double>();
        Vec6 J;
        J.head<3>() = p.cross(m);
        J.tail<3>() = m;
        return J;
    };

    for (int it = 0; it < params.iterations; ++it) {
        const float f = params.iterations > 1 ? static_cast<float>(it) / static_cast<float>(params.iterations - 1) : 1.0f;
        const float gate = params.start_distance_mm + f * (params.final_distance_mm - params.start_distance_mm);
        associate(gate);
        Mat6 H = Mat6::Zero();
        Vec6 g = Vec6::Zero();
        int n = 0;
        const Mat3 Rt = T.linear().transpose();
        for (std::size_t k = 0; k < sel.size(); ++k) {
            if (corr[k].target < 0) continue;
            const Vec6 J = jacobian(k, Rt);
            const double r = corr[k].r;
            const double w = std::abs(r) <= params.huber_mm ? 1.0 : params.huber_mm / std::abs(r);
            H += w * J * J.transpose();
            g += w * J * r;
            ++n;
        }
        if (n < 12) return res;
        const Vec6 d = -(H + 1e-9 * H.trace() * Mat6::Identity()).ldlt().solve(g);
        SE3 step = SE3::Identity();
        const Vec3 w = d.head<3>();
        if (w.norm() > 1e-12) step.linear() = Eigen::AngleAxisd(w.norm(), w.normalized()).toRotationMatrix();
        step.translation() = d.tail<3>();
        T = T * step;
        if (w.norm() < 1e-6 && d.tail<3>().norm() < 1e-4 && f > 0.5f) {
            res.converged = true;
            break;
        }
        res.converged = it == params.iterations - 1;
    }

    // Final statistics at the tight gate.
    associate(params.inlier_mm);
    Mat6 H = Mat6::Zero();
    const Mat3 Rt = T.linear().transpose();
    double ss = 0;
    Vec3 centroid = Vec3::Zero();
    int n = 0;
    for (std::size_t k = 0; k < sel.size(); ++k) {
        if (corr[k].target < 0) continue;
        const Vec6 J = jacobian(k, Rt);
        H += J * J.transpose();
        ss += static_cast<double>(corr[k].r) * corr[k].r;
        centroid += source.points[sel[k]].cast<double>();
        ++n;
    }
    res.T_target_source = T;
    res.correspondences = n;
    res.fitness = static_cast<double>(n) / static_cast<double>(sel.size());
    if (n < 12) {
        res.converged = false;
        return res;
    }
    res.rms_mm = std::sqrt(ss / n);
    const double scale = std::min(1.0, params.max_independent / n) / (params.sigma_mm * params.sigma_mm);
    res.information = H * scale;
    // Conditioning in coordinates centred on the overlap with a 100 mm lever (unit-free).
    centroid /= n;
    Mat6 Hc = Mat6::Zero();
    for (std::size_t k = 0; k < sel.size(); ++k) {
        if (corr[k].target < 0) continue;
        const Vec3 m = Rt * tc.normals[static_cast<std::size_t>(corr[k].target)].cast<double>();
        Vec6 J;
        J.head<3>() = (source.points[sel[k]].cast<double>() - centroid).cross(m) / 100.0;
        J.tail<3>() = m;
        Hc += J * J.transpose();
    }
    // Conflict: nearby geometry that disagrees.
    {
        const Eigen::Matrix3f R = T.linear().cast<float>();
        const Vec3f t = T.translation().cast<float>();
        std::atomic<int> near{0}, bad{0};
        tbb::parallel_for(std::size_t{0}, sel.size(), [&](std::size_t k) {
            const Vec3f p = R * source.points[sel[k]] + t;
            const int j = target.nearest(p, params.conflict_radius_mm);
            if (j < 0) return;
            ++near;
            const Vec3f& n = tc.normals[static_cast<std::size_t>(j)];
            if (std::abs(n.dot(p - tc.points[static_cast<std::size_t>(j)])) > params.conflict_distance_mm ||
                n.dot(R * source.normals[sel[k]]) < params.conflict_normal_dot)
                ++bad;
        });
        res.conflict = near > 0 ? static_cast<double>(bad) / near : 0.0;
    }
    const Eigen::SelfAdjointEigenSolver<Mat6> es(Hc);
    res.min_eigen_ratio = es.eigenvalues()(5) > 0 ? es.eigenvalues()(0) / es.eigenvalues()(5) : 0.0;
    return res;
}

}  // namespace einstar::recon
