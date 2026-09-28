#include "einstar/markers/marker_map.hpp"

#include <algorithm>
#include <random>
#include <cmath>
#include <fstream>
#include <sstream>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace einstar::markers {
namespace {

constexpr double kCell = 10.0;  // mm

}  // namespace

std::optional<SE3> fit_rigid(const std::vector<Correspondence>& pairs) {
    if (pairs.size() < 3) return std::nullopt;
    Eigen::Matrix<double, 3, Eigen::Dynamic> a(3, static_cast<Eigen::Index>(pairs.size())), b(3, static_cast<Eigen::Index>(pairs.size()));
    for (std::size_t i = 0; i < pairs.size(); ++i) {
        a.col(static_cast<Eigen::Index>(i)) = pairs[i].p_camera;
        b.col(static_cast<Eigen::Index>(i)) = pairs[i].q_world;
    }
    // Reject degenerate (collinear) configurations.
    const Vec3 ca = a.rowwise().mean();
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (Eigen::Index i = 0; i < a.cols(); ++i) cov += (a.col(i) - ca) * (a.col(i) - ca).transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    if (es.eigenvalues()(1) < 1.0) return std::nullopt;  // needs > ~1 mm spread off the line
    SE3 T = SE3::Identity();
    T.matrix() = Eigen::umeyama(a, b, false);
    return T;
}

void MarkerMap::clear() {
    markers_.clear();
    grid_.clear();
    next_id_ = 0;
    pairs_dirty_ = true;
}

std::int64_t MarkerMap::cell(const Vec3& p) const {
    return (static_cast<std::int64_t>(std::floor(p.x() / kCell)) & 0x1FFFFF) << 42 |
           (static_cast<std::int64_t>(std::floor(p.y() / kCell)) & 0x1FFFFF) << 21 |
           (static_cast<std::int64_t>(std::floor(p.z() / kCell)) & 0x1FFFFF);
}

void MarkerMap::index(int slot) { grid_[cell(markers_[static_cast<std::size_t>(slot)].position)].push_back(slot); }

std::size_t MarkerMap::confirmed_count() const {
    return static_cast<std::size_t>(std::ranges::count_if(markers_, [&](const MapMarker& m) { return confirmed(m); }));
}

void MarkerMap::confirm_all() {
    for (auto& m : markers_) m.observations = std::max(m.observations, params_.min_observations);
    pairs_dirty_ = true;
}

void MarkerMap::set_markers(std::vector<MapMarker> m) {
    clear();
    markers_ = std::move(m);
    for (std::size_t i = 0; i < markers_.size(); ++i) {
        next_id_ = std::max(next_id_, markers_[i].id + 1);
        index(static_cast<int>(i));
    }
}

std::optional<int> MarkerMap::nearest(const Vec3& p, double radius, bool confirmed_only) const {
    const int r = static_cast<int>(std::ceil(radius / kCell));
    const int cx = static_cast<int>(std::floor(p.x() / kCell)), cy = static_cast<int>(std::floor(p.y() / kCell)),
              cz = static_cast<int>(std::floor(p.z() / kCell));
    double best = radius * radius;
    std::optional<int> out;
    for (int z = cz - r; z <= cz + r; ++z)
        for (int y = cy - r; y <= cy + r; ++y)
            for (int x = cx - r; x <= cx + r; ++x) {
                const auto key = (static_cast<std::int64_t>(x) & 0x1FFFFF) << 42 | (static_cast<std::int64_t>(y) & 0x1FFFFF) << 21 |
                                 (static_cast<std::int64_t>(z) & 0x1FFFFF);
                auto it = grid_.find(key);
                if (it == grid_.end()) continue;
                for (const int s : it->second) {
                    if (confirmed_only && !confirmed(markers_[static_cast<std::size_t>(s)])) continue;
                    const double d2 = (markers_[static_cast<std::size_t>(s)].position - p).squaredNorm();
                    if (d2 <= best) {
                        best = d2;
                        out = s;
                    }
                }
            }
    return out;
}

std::optional<PoseEstimate> MarkerMap::track(const std::vector<Vec3>& fm, const SE3& T_guess) const {
    if (markers_.empty() || fm.size() < 3) return std::nullopt;
    auto associate = [&](const SE3& T, double radius) {
        std::vector<Correspondence> pairs;
        std::vector<int> used(markers_.size(), 0);
        for (std::size_t i = 0; i < fm.size(); ++i)
            if (auto s = nearest(T * fm[i], radius); s && !used[static_cast<std::size_t>(*s)]++) {
                const auto& m = markers_[static_cast<std::size_t>(*s)];
                pairs.push_back({static_cast<int>(i), m.id, fm[i], m.position});
            }
        return pairs;
    };
    // Robust fit: wrong associations (a phantom or mis-matched marker, a neighbour inside the search
    // radius) must not drag the pose. Small RANSAC over triplets, then a least-squares refit on the
    // consensus set.
    auto robust_fit = [&](const std::vector<Correspondence>& pairs, const SE3& T0) -> std::optional<SE3> {
        const int n = static_cast<int>(pairs.size());
        if (n < 3) return std::nullopt;
        const double gate = 1.5 * params_.inlier_mm;
        auto consensus = [&](const SE3& T) {
            std::vector<Correspondence> in;
            for (const auto& c : pairs)
                if ((T * c.p_camera - c.q_world).norm() <= gate) in.push_back(c);
            return in;
        };
        std::vector<Correspondence> best = consensus(T0);
        if (static_cast<int>(best.size()) < n) {
            std::mt19937 rng(static_cast<std::uint32_t>(n) * 2654435761u);
            std::uniform_int_distribution<int> pick(0, n - 1);
            const int iters = n <= 8 ? 56 : 150;
            for (int it = 0; it < iters && static_cast<int>(best.size()) < n; ++it) {
                const int i = pick(rng), j = pick(rng), k = pick(rng);
                if (i == j || j == k || i == k) continue;
                const auto T = fit_rigid({pairs[static_cast<std::size_t>(i)], pairs[static_cast<std::size_t>(j)], pairs[static_cast<std::size_t>(k)]});
                if (!T) continue;
                auto in = consensus(*T);
                if (in.size() > best.size()) best = std::move(in);
            }
        }
        if (best.size() < 3) return std::nullopt;
        auto T = fit_rigid(best);
        if (!T) return std::nullopt;
        // One more consensus pass at the refined pose.
        auto in = consensus(*T);
        if (in.size() >= best.size()) {
            if (auto T2 = fit_rigid(in)) T = T2;
        }
        return T;
    };
    auto T = robust_fit(associate(T_guess, params_.match_radius_mm), T_guess);
    if (!T) return std::nullopt;
    // Re-associate tightly at the refined pose (picks up markers the guess was too far off for).
    auto pairs = associate(*T, 2.0 * params_.inlier_mm);
    T = robust_fit(pairs, *T);
    if (!T) return std::nullopt;
    PoseEstimate est;
    est.T_world_camera = *T;
    double ss = 0;
    for (const auto& c : pairs) {
        const double e = (*T * c.p_camera - c.q_world).norm();
        if (e <= params_.inlier_mm) {
            est.inliers.push_back(c);
            ss += e * e;
        }
    }
    if (static_cast<int>(est.inliers.size()) < 3) return std::nullopt;
    est.rms_mm = std::sqrt(ss / static_cast<double>(est.inliers.size()));
    return est;
}

void MarkerMap::rebuild_pairs() const {
    pairs_.clear();
    for (std::size_t a = 0; a < markers_.size(); ++a)
        for (std::size_t b = a + 1; b < markers_.size(); ++b) {
            if (!confirmed(markers_[a]) || !confirmed(markers_[b])) continue;
            const double d = (markers_[a].position - markers_[b].position).norm();
            if (d >= params_.min_pair_mm && d <= params_.max_pair_mm)
                pairs_.push_back({static_cast<float>(d), static_cast<int>(a), static_cast<int>(b)});
        }
    std::ranges::sort(pairs_, {}, &Pair::d);
    pairs_dirty_ = false;
}

std::optional<PoseEstimate> MarkerMap::relocalize(const std::vector<Vec3>& fm, std::uint32_t seed) const {
    const int n = static_cast<int>(fm.size());
    if (markers_.size() < 3 || n < std::max(3, params_.min_inliers)) return std::nullopt;
    if (pairs_dirty_) rebuild_pairs();
    const double tol = params_.signature_tolerance_mm;
    // Map pairs whose length matches d (either orientation).
    auto matching = [&](double d, std::vector<std::pair<int, int>>& out) {
        out.clear();
        auto lo = std::ranges::lower_bound(pairs_, static_cast<float>(d - tol), {}, &Pair::d);
        for (auto it = lo; it != pairs_.end() && it->d <= d + tol; ++it) {
            out.emplace_back(it->a, it->b);
            out.emplace_back(it->b, it->a);
        }
    };
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> pick(0, n - 1);
    std::optional<PoseEstimate> best;
    std::vector<std::pair<int, int>> ab, ac;
    for (int it = 0; it < params_.ransac_iterations; ++it) {
        const int i = pick(rng), j = pick(rng), k = pick(rng);
        if (i == j || j == k || i == k) continue;
        const double dij = (fm[i] - fm[j]).norm(), dik = (fm[i] - fm[k]).norm(), djk = (fm[j] - fm[k]).norm();
        if (std::min({dij, dik, djk}) < params_.min_pair_mm) continue;
        matching(dij, ab);
        if (ab.empty()) continue;
        matching(dik, ac);
        for (const auto& [a, b] : ab)
            for (const auto& [a2, c] : ac) {
                if (a2 != a || c == b) continue;
                if (std::abs((markers_[static_cast<std::size_t>(b)].position - markers_[static_cast<std::size_t>(c)].position).norm() - djk) > tol)
                    continue;
                std::vector<Correspondence> tri{{i, markers_[static_cast<std::size_t>(a)].id, fm[i], markers_[static_cast<std::size_t>(a)].position},
                                                {j, markers_[static_cast<std::size_t>(b)].id, fm[j], markers_[static_cast<std::size_t>(b)].position},
                                                {k, markers_[static_cast<std::size_t>(c)].id, fm[k], markers_[static_cast<std::size_t>(c)].position}};
                auto T = fit_rigid(tri);
                if (!T) continue;
                // Verify against the whole frame.
                auto est = track(fm, *T);
                if (!est) continue;
                const int inl = static_cast<int>(est->inliers.size());
                if (inl < params_.min_inliers || inl < params_.min_inlier_fraction * n) continue;
                if (!best || inl > static_cast<int>(best->inliers.size()) ||
                    (inl == static_cast<int>(best->inliers.size()) && est->rms_mm < best->rms_mm))
                    best = std::move(est);
                if (static_cast<int>(best->inliers.size()) == n) return best;
            }
    }
    return best;
}

void MarkerMap::update(const std::vector<Vec3>& fm, const std::vector<double>& diameters, const SE3& T,
                       const std::vector<Correspondence>& matched) {
    std::vector<int> frame_matched(fm.size(), -1);
    for (const auto& c : matched) frame_matched[static_cast<std::size_t>(c.frame_index)] = c.map_id;
    std::unordered_map<int, int> slot_of;
    for (std::size_t s = 0; s < markers_.size(); ++s) slot_of[markers_[s].id] = static_cast<int>(s);
    for (std::size_t i = 0; i < fm.size(); ++i) {
        const Vec3 w = T * fm[i];
        int slot = -1;
        if (frame_matched[i] >= 0) slot = slot_of[frame_matched[i]];
        else if (auto s = nearest(w, params_.merge_radius_mm)) slot = *s;
        if (slot >= 0) {
            auto& m = markers_[static_cast<std::size_t>(slot)];
            if (m.fixed) continue;
            const double k = static_cast<double>(m.observations);
            const Vec3 old = m.position;
            m.position = (old * k + w) / (k + 1);
            m.observations += 1;
            if (i < diameters.size()) m.diameter = (m.diameter * k + diameters[i]) / (k + 1);
            if (cell(old) != cell(m.position)) {
                auto& v = grid_[cell(old)];
                std::erase(v, slot);
                index(slot);
            }
        } else {
            markers_.push_back({next_id_++, w, i < diameters.size() ? diameters[i] : 0.0, 1, false});
            index(static_cast<int>(markers_.size() - 1));
        }
        pairs_dirty_ = true;
    }
}

bool save_markers(const std::string& path, const std::vector<MapMarker>& markers) {
    std::ofstream f(path);
    if (!f) return false;
    f << "# einstar global markers: id x y z diameter (mm)\n";
    f.precision(6);
    f << std::fixed;
    for (const auto& m : markers)
        f << m.id << ' ' << m.position.x() << ' ' << m.position.y() << ' ' << m.position.z() << ' ' << m.diameter << '\n';
    return static_cast<bool>(f);
}

std::optional<std::vector<MapMarker>> load_markers(const std::string& path) {
    std::ifstream f(path);
    if (!f) return std::nullopt;
    std::vector<MapMarker> out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        MapMarker m;
        if (!(ss >> m.id >> m.position.x() >> m.position.y() >> m.position.z() >> m.diameter)) return std::nullopt;
        m.fixed = true;
        m.observations = 1;
        out.push_back(m);
    }
    return out;
}

}  // namespace einstar::markers
