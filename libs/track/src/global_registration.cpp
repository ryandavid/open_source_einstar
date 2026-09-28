#include "einstar/track/global_registration.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <nanoflann.hpp>
#include <tbb/parallel_for.h>

namespace einstar::track {
namespace {

struct PointAdaptor {
    const std::vector<Vec3f>* pts;
    [[nodiscard]] std::size_t kdtree_get_point_count() const { return pts->size(); }
    [[nodiscard]] float kdtree_get_pt(std::size_t i, std::size_t d) const { return (*pts)[i][static_cast<Eigen::Index>(d)]; }
    template <class B>
    bool kdtree_get_bbox(B&) const { return false; }
};
using PointTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointAdaptor>, PointAdaptor, 3, std::uint32_t>;

struct FeatureAdaptor {
    const std::vector<Fpfh>* f;
    [[nodiscard]] std::size_t kdtree_get_point_count() const { return f->size(); }
    [[nodiscard]] float kdtree_get_pt(std::size_t i, std::size_t d) const { return (*f)[i][d]; }
    template <class B>
    bool kdtree_get_bbox(B&) const { return false; }
};
using FeatureTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, FeatureAdaptor>, FeatureAdaptor, 33, std::uint32_t>;

// Darboux-frame pair features (PCL convention) accumulated into an 11+11+11 histogram.
void add_pair(const Vec3f& p1, const Vec3f& n1, const Vec3f& p2, const Vec3f& n2, Fpfh& h, float weight) {
    Vec3f d = p2 - p1;
    const float dist = d.norm();
    if (dist < 1e-6f) return;
    d /= dist;
    Vec3f ps = p1, ns = n1, pt = p2, nt = n2;
    if (std::acos(std::clamp(std::abs(n1.dot(d)), 0.0f, 1.0f)) > std::acos(std::clamp(std::abs(n2.dot(d)), 0.0f, 1.0f))) {
        std::swap(ps, pt);
        std::swap(ns, nt);
        d = -d;
    }
    const Vec3f u = ns;
    Vec3f v = u.cross(d);
    const float vn = v.norm();
    if (vn < 1e-6f) return;
    v /= vn;
    const Vec3f w = u.cross(v);
    const float alpha = v.dot(nt);
    const float phi = u.dot(d);
    const float theta = std::atan2(w.dot(nt), u.dot(nt));
    auto bin = [](float x, float lo, float hi) {
        return std::clamp(static_cast<int>((x - lo) / (hi - lo) * 11.0f), 0, 10);
    };
    h[static_cast<std::size_t>(bin(alpha, -1, 1))] += weight;
    h[static_cast<std::size_t>(11 + bin(phi, -1, 1))] += weight;
    h[static_cast<std::size_t>(22 + bin(theta, -static_cast<float>(M_PI), static_cast<float>(M_PI)))] += weight;
}

void normalize(Fpfh& h) {
    for (int part = 0; part < 3; ++part) {
        float s = 0;
        for (int i = 0; i < 11; ++i) s += h[static_cast<std::size_t>(part * 11 + i)];
        if (s > 0)
            for (int i = 0; i < 11; ++i) h[static_cast<std::size_t>(part * 11 + i)] *= 100.0f / s;
    }
}

SE3 kabsch(const std::vector<Vec3f>& src, const std::vector<Vec3f>& dst) {
    Eigen::Matrix<double, 3, Eigen::Dynamic> a(3, static_cast<Eigen::Index>(src.size())), b(3, static_cast<Eigen::Index>(dst.size()));
    for (std::size_t i = 0; i < src.size(); ++i) {
        a.col(static_cast<Eigen::Index>(i)) = src[i].cast<double>();
        b.col(static_cast<Eigen::Index>(i)) = dst[i].cast<double>();
    }
    SE3 t;
    t.matrix() = Eigen::umeyama(a, b, false);
    return t;
}

// PCA normals over a radius (orientation kept consistent with the input normals). Descriptors are
// very sensitive to normal noise, and frame normals from pixel differences are much noisier than
// model normals from the TSDF gradient, so both sides are re-estimated the same way.
void refine_normals(OrientedCloud& c, float radius) {
    PointAdaptor pa{&c.points};
    PointTree tree(3, pa, {10});
    const float r2 = radius * radius;
    std::vector<Vec3f> out(c.normals.size());
    tbb::parallel_for(std::size_t{0}, c.points.size(), [&](std::size_t i) {
        std::vector<nanoflann::ResultItem<std::uint32_t, float>> nb;
        tree.radiusSearch(c.points[i].data(), r2, nb);
        out[i] = c.normals[i];
        if (nb.size() < 5) return;
        Vec3f mean = Vec3f::Zero();
        for (const auto& m : nb) mean += c.points[m.first];
        mean /= static_cast<float>(nb.size());
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (const auto& m : nb) {
            const Vec3f d = c.points[m.first] - mean;
            cov += d * d.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
        Vec3f n = es.eigenvectors().col(0);
        if (n.dot(c.normals[i]) < 0) n = -n;
        out[i] = n;
    });
    c.normals = std::move(out);
}

}  // namespace

struct FeatureModel::Index {
    PointAdaptor pa;
    FeatureAdaptor fa;
    PointTree points;
    FeatureTree features;
    Index(const OrientedCloud& c, const std::vector<Fpfh>& f)
        : pa{&c.points}, fa{&f}, points(3, pa, {10}), features(33, fa, {10}) {}
};

OrientedCloud voxel_downsample(const OrientedCloud& in, float voxel) {
    struct Acc {
        Vec3f p = Vec3f::Zero(), n = Vec3f::Zero();
        int count = 0;
    };
    std::unordered_map<std::int64_t, Acc> cells;
    const float inv = 1.0f / voxel;
    for (std::size_t i = 0; i < in.points.size(); ++i) {
        const Vec3f& p = in.points[i];
        const auto key = (static_cast<std::int64_t>(std::floor(p.x() * inv)) & 0x1FFFFF) << 42 |
                         (static_cast<std::int64_t>(std::floor(p.y() * inv)) & 0x1FFFFF) << 21 |
                         (static_cast<std::int64_t>(std::floor(p.z() * inv)) & 0x1FFFFF);
        auto& a = cells[key];
        a.p += p;
        a.n += in.normals[i];
        ++a.count;
    }
    OrientedCloud out;
    out.points.reserve(cells.size());
    out.normals.reserve(cells.size());
    for (const auto& [k, a] : cells) {
        const float nn = a.n.norm();
        if (nn < 1e-6f) continue;
        out.points.push_back(a.p / static_cast<float>(a.count));
        out.normals.push_back(a.n / nn);
    }
    return out;
}

std::vector<Fpfh> compute_fpfh(const OrientedCloud& c, float radius) {
    const std::size_t n = c.points.size();
    PointAdaptor pa{&c.points};
    PointTree tree(3, pa, {10});
    const float r2 = radius * radius;
    std::vector<std::vector<nanoflann::ResultItem<std::uint32_t, float>>> nbrs(n);
    std::vector<Fpfh> spfh(n);
    tbb::parallel_for(std::size_t{0}, n, [&](std::size_t i) {
        tree.radiusSearch(c.points[i].data(), r2, nbrs[i]);
        Fpfh h{};
        for (const auto& m : nbrs[i])
            if (m.first != i) add_pair(c.points[i], c.normals[i], c.points[m.first], c.normals[m.first], h, 1.0f);
        normalize(h);
        spfh[i] = h;
    });
    std::vector<Fpfh> out(n);
    tbb::parallel_for(std::size_t{0}, n, [&](std::size_t i) {
        Fpfh h = spfh[i];
        int k = 0;
        Fpfh acc{};
        for (const auto& m : nbrs[i]) {
            if (m.first == i) continue;
            const float w = 1.0f / std::max(std::sqrt(m.second), 1e-3f);
            for (std::size_t b = 0; b < 33; ++b) acc[b] += w * spfh[m.first][b];
            ++k;
        }
        if (k > 0)
            for (std::size_t b = 0; b < 33; ++b) h[b] += acc[b] / static_cast<float>(k);
        normalize(h);
        out[i] = h;
    });
    return out;
}

FeatureModel::FeatureModel(OrientedCloud cloud, float radius) : cloud_(std::move(cloud)) {
    refine_normals(cloud_, radius * 0.5f);
    features_ = compute_fpfh(cloud_, radius);
    index_ = std::make_unique<Index>(cloud_, features_);
}
FeatureModel::FeatureModel() = default;
FeatureModel::~FeatureModel() = default;
FeatureModel::FeatureModel(FeatureModel&& o) noexcept = default;
FeatureModel& FeatureModel::operator=(FeatureModel&& o) noexcept {
    cloud_ = std::move(o.cloud_);
    features_ = std::move(o.features_);
    index_ = std::move(o.index_);
    // Re-point the adaptors at the moved storage.
    if (index_) index_ = std::make_unique<Index>(cloud_, features_);
    return *this;
}

void FeatureModel::nearest_features(const Fpfh& f, int k, std::vector<std::uint32_t>& out) const {
    out.assign(static_cast<std::size_t>(k), 0);
    std::vector<float> d(static_cast<std::size_t>(k));
    const auto found = index_->features.knnSearch(f.data(), static_cast<std::size_t>(k), out.data(), d.data());
    out.resize(found);
}

std::optional<std::uint32_t> FeatureModel::nearest_point(const Vec3f& p, float radius) const {
    std::uint32_t idx;
    float d2;
    if (index_->points.knnSearch(p.data(), 1, &idx, &d2) == 0 || d2 > radius * radius) return std::nullopt;
    return idx;
}

std::optional<GlobalRegistrationResult> register_global(const OrientedCloud& frame_cloud, const FeatureModel& model,
                                                        const GlobalRegistrationParams& p, std::uint32_t seed) {
    if (model.empty()) return std::nullopt;
    OrientedCloud src = voxel_downsample(frame_cloud, p.voxel_mm);
    if (src.points.size() < 50) return std::nullopt;
    refine_normals(src, p.feature_radius_mm * 0.5f);
    const auto feats = compute_fpfh(src, p.feature_radius_mm);

    // Mutual nearest neighbours in feature space.
    FeatureAdaptor fa{&feats};
    FeatureTree src_tree(33, fa, {10});
    std::vector<std::pair<std::uint32_t, std::uint32_t>> corr;
    {
        std::vector<std::pair<std::uint32_t, std::uint32_t>> all(src.points.size(), {~0u, ~0u});
        tbb::parallel_for(std::size_t{0}, src.points.size(), [&](std::size_t i) {
            std::vector<std::uint32_t> nn;
            model.nearest_features(feats[i], 1, nn);
            if (nn.empty()) return;
            std::uint32_t back;
            float d;
            src_tree.knnSearch(model.features()[nn[0]].data(), 1, &back, &d);
            if (back == i) all[i] = {static_cast<std::uint32_t>(i), nn[0]};
        });
        for (const auto& c : all)
            if (c.first != ~0u) corr.push_back(c);
    }
    if (corr.size() < 10) return std::nullopt;

    const auto& dst = model.cloud().points;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<std::size_t> pick(0, corr.size() - 1);
    const float inl2 = p.inlier_distance_mm * p.inlier_distance_mm;
    // Best few *distinct* hypotheses (a symmetric part has several good ones).
    struct Hyp {
        SE3 T;
        int inliers;
    };
    std::vector<Hyp> hyps;
    auto distinct = [&](const SE3& a, const SE3& b) {
        const SE3 d = a.inverse() * b;
        return translation_norm(d) > p.distinct_mm || rotation_angle(d) * 180.0 / M_PI > p.distinct_deg;
    };
    auto consider = [&](const SE3& T, int inliers) {
        for (auto& h : hyps)
            if (!distinct(h.T, T)) {
                if (inliers > h.inliers) h = {T, inliers};
                return;
            }
        hyps.push_back({T, inliers});
        std::ranges::sort(hyps, [](const Hyp& a, const Hyp& b) { return a.inliers > b.inliers; });
        if (static_cast<int>(hyps.size()) > 2 * p.hypotheses) hyps.pop_back();
    };
    std::vector<Vec3f> a(3), b(3);
    for (int it = 0; it < p.ransac_iterations; ++it) {
        std::size_t idx[3] = {pick(rng), pick(rng), pick(rng)};
        if (idx[0] == idx[1] || idx[1] == idx[2] || idx[0] == idx[2]) continue;
        bool ok = true;
        for (int k = 0; k < 3 && ok; ++k) {
            a[static_cast<std::size_t>(k)] = src.points[corr[idx[k]].first];
            b[static_cast<std::size_t>(k)] = dst[corr[idx[k]].second];
        }
        // Edge-length consistency: rigid motions preserve distances.
        for (int k = 0; k < 3 && ok; ++k) {
            const float la = (a[static_cast<std::size_t>(k)] - a[static_cast<std::size_t>((k + 1) % 3)]).norm();
            const float lb = (b[static_cast<std::size_t>(k)] - b[static_cast<std::size_t>((k + 1) % 3)]).norm();
            if (la < 3 * p.voxel_mm || std::min(la, lb) < p.edge_ratio * std::max(la, lb)) ok = false;
        }
        if (!ok) continue;
        const SE3 T = kabsch(a, b);
        const Eigen::Matrix4f Tf = T.matrix().cast<float>();
        int inliers = 0;
        for (const auto& [s, d] : corr)
            if (((Tf * src.points[s].homogeneous()).head<3>() - dst[d]).squaredNorm() < inl2) ++inliers;
        if (inliers >= 3) consider(T, inliers);
    }
    if (hyps.empty()) return std::nullopt;

    // Refit each on its correspondence inliers, then score against the whole model.
    std::vector<GlobalRegistrationCandidate> scored;
    for (std::size_t h = 0; h < hyps.size() && static_cast<int>(h) < p.hypotheses; ++h) {
        SE3 T = hyps[h].T;
        std::vector<Vec3f> sa, sb;
        const Eigen::Matrix4f Tb = T.matrix().cast<float>();
        for (const auto& [s, d] : corr)
            if (((Tb * src.points[s].homogeneous()).head<3>() - dst[d]).squaredNorm() < inl2) {
                sa.push_back(src.points[s]);
                sb.push_back(dst[d]);
            }
        if (sa.size() >= 3) T = kabsch(sa, sb);
        const Eigen::Matrix4f Tr = T.matrix().cast<float>();
        int fit = 0;
        for (const auto& q : src.points)
            if (model.nearest_point((Tr * q.homogeneous()).head<3>(), p.inlier_distance_mm)) ++fit;
        scored.push_back({T, fit, static_cast<double>(fit) / static_cast<double>(src.points.size())});
    }
    std::ranges::sort(scored, [](const auto& x, const auto& y) { return x.inliers > y.inliers; });
    // Refits can converge onto the same pose: keep distinct ones only.
    std::vector<GlobalRegistrationCandidate> uniq;
    for (const auto& c : scored)
        if (std::ranges::all_of(uniq, [&](const auto& u) { return distinct(u.T_model_frame, c.T_model_frame); })) uniq.push_back(c);
    if (uniq.empty() || uniq.front().inliers < p.min_inliers) return std::nullopt;
    GlobalRegistrationResult out;
    out.T_model_frame = uniq.front().T_model_frame;
    out.inliers = uniq.front().inliers;
    out.fitness = uniq.front().fitness;
    for (const auto& c : uniq)
        if (c.inliers >= p.ambiguity_ratio * uniq.front().inliers) out.candidates.push_back(c);
    out.ambiguous = out.candidates.size() > 1;
    return out;
}

}  // namespace einstar::track
