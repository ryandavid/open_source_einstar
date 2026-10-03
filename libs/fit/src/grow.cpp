#include "einstar/fit/grow.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <queue>
#include <unordered_set>

namespace einstar::fit {
namespace {

constexpr std::array<SurfaceKind, 4> kDefaultKinds{SurfaceKind::plane, SurfaceKind::cylinder, SurfaceKind::cone, SurfaceKind::sphere};

struct Candidate {
    double score;
    std::uint32_t triangle;
    std::uint16_t region;
    bool operator>(const Candidate& o) const { return score > o.score; }
};

struct Active {
    FitResult fit;
    std::vector<std::uint32_t> members;
    std::size_t fitted_size = 0;
    double gate = 0;
};

// Candidates are judged by their centroid, which averages three vertices' noise: sigma (measured on
// vertices) / sqrt(3).
double gate_of(const FitResult& f, const GrowOptions& o) {
    const double sigma = o.max_sigma_mm > 0 ? std::min(f.sigma, o.max_sigma_mm) : f.sigma;
    return std::max(o.gate_sigma * sigma / std::numbers::sqrt3, o.min_gate_mm);
}

// A dab of a few triangles cannot pin down a curved surface's axis (normals of 0.5 mm triangles carry
// degrees of noise), so seeds are widened by rings of neighbours to at least this many triangles.
constexpr std::size_t kMinSeedTriangles = 40;

void widen_small_seed(const MeshTopology& topo, std::vector<std::uint32_t>& seed, std::span<const std::uint8_t> blocked) {
    if (seed.empty() || seed.size() >= kMinSeedTriangles) return;
    std::unordered_set<std::uint32_t> in(seed.begin(), seed.end());
    for (int ring = 0; ring < 4 && seed.size() < kMinSeedTriangles; ++ring) {
        const std::size_t n = seed.size();
        for (std::size_t i = 0; i < n; ++i)
            for (const auto nb : topo.neighbors(seed[i]))
                if (nb != kNoTriangle && (blocked.empty() || !blocked[nb]) && in.insert(nb).second) seed.push_back(nb);
    }
}

// Growth order decides who gets a triangle where two regions meet; afterwards each border triangle goes to
// the neighbouring region whose surface it lies closer to (the first rows of a plane, tangent to a fillet,
// to the plane), and the regions are refitted, a few rounds.
void reassign_borders(const MeshTopology& topo, std::vector<Active>& active, GrowWorkspace& ws, const GrowOptions& o, double cos_max) {
    if (active.size() < 2) return;
    auto& owner = ws.owner;
    for (int round = 0; round < 4; ++round) {
        std::size_t moved = 0;
        for (const std::uint32_t t : ws.owned) {
            const std::size_t a = owner[t] - 1u;
            const Vec3 c = topo.centroid(t).cast<double>();
            const Vec3 n = topo.normal(t).cast<double>();
            double best = std::abs(signed_distance(active[a].fit.surface, c));
            std::size_t to = a;
            for (const auto nb : topo.neighbors(t)) {
                if (nb == kNoTriangle || owner[nb] == 0 || owner[nb] - 1u == a) continue;
                const std::size_t b = owner[nb] - 1u;
                const double d = std::abs(signed_distance(active[b].fit.surface, c));
                if (d < 0.7 * best && d < active[b].gate && std::abs(normal_at(active[b].fit.surface, c).dot(n)) >= cos_max) {
                    best = d;
                    to = b;
                }
            }
            if (to != a) {
                owner[t] = static_cast<std::uint16_t>(to + 1);
                ++moved;
            }
        }
        if (moved == 0) break;
        for (auto& a : active) a.members.clear();
        for (const std::uint32_t t : ws.owned) active[owner[t] - 1u].members.push_back(t);
        for (auto& a : active) {
            if (a.members.empty()) continue;
            const RegionPoints pts = region_points(topo, a.members, o.max_fit_points);
            if (pts.points.size() > static_cast<std::size_t>(parameter_count(kind_of(a.fit.surface)))) a.fit = refine_surface(a.fit.surface, pts.view(), o.fit);
            a.gate = gate_of(a.fit, o);
        }
    }
}

}  // namespace

// The surface a seed grows with. Seeds are small, and the edges of a face bend in a scan (sharp edges are
// rounded over a voxel, the mesh frays where observation stops), so a curved fit can beat the plane on the
// seed alone. Each plausible kind is grown on its own and the simplest one that grows clearly furthest
// (by 20%) wins: the face's extent decides, not the few triangles under the brush.
std::optional<FitResult> choose_seed_surface(const MeshTopology& topo, const RegionSeed& seed, std::span<const SurfaceKind> kinds,
                                             const RegionPoints& pts, const GrowOptions& o, std::span<const std::uint8_t> blocked,
                                             GrowWorkspace& ws) {
    std::vector<FitResult> candidates;
    for (const SurfaceKind k : kinds) {
        const std::array one{k};
        if (auto r = fit_best(one, pts.view(), o.fit)) candidates.push_back(std::move(*r));
    }
    if (candidates.size() <= 1) return candidates.empty() ? std::nullopt : std::optional(candidates.front());
    std::ranges::sort(candidates, [](const FitResult& a, const FitResult& b) {
        return parameter_count(kind_of(a.surface)) < parameter_count(kind_of(b.surface));
    });
    std::size_t best = 0, best_size = 0;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        const RegionSeed trial{seed.triangles, {kind_of(candidates[i].surface)}, candidates[i].surface};
        const GrowResult g = grow_regions(topo, std::span(&trial, 1), o, blocked, ws);
        const std::size_t size = g.regions[0].triangles.size();
        if (i == 0 || static_cast<double>(size) > 1.2 * static_cast<double>(best_size)) {
            best = i;
            best_size = size;
        }
    }
    return candidates[best];
}

RegionPoints region_points(const MeshTopology& topo, std::span<const std::uint32_t> triangles, std::size_t max_points) {
    const recon::TriangleMesh& mesh = topo.mesh();
    struct Corner {
        std::uint32_t vertex;
        std::uint32_t triangle;
    };
    // A large region is thinned by triangles first (a vertex has about two triangles), so only what is kept
    // gets sorted.
    const std::size_t step = max_points > 0 && triangles.size() > 2 * max_points ? triangles.size() / (2 * max_points) : 1;
    std::vector<Corner> corners;
    corners.reserve(triangles.size() / step * 3 + 3);
    for (std::size_t i = 0; i < triangles.size(); i += step)
        for (const auto v : mesh.triangles[triangles[i]]) corners.push_back({v, triangles[i]});
    std::ranges::sort(corners, [](const Corner& a, const Corner& b) { return a.vertex < b.vertex; });
    RegionPoints out;
    for (std::size_t i = 0; i < corners.size();) {
        std::size_t j = i;
        Vec3 n = Vec3::Zero();
        double w = 0;
        for (; j < corners.size() && corners[j].vertex == corners[i].vertex; ++j) {
            const double a = topo.area(corners[j].triangle);
            n += a * topo.normal(corners[j].triangle).cast<double>();
            w += a / 3;
        }
        if (w > 0 && n.squaredNorm() > 0) {
            out.points.push_back(mesh.vertices[corners[i].vertex].cast<double>());
            out.normals.push_back(n.normalized());
            out.weights.push_back(w);
        }
        i = j;
    }
    if (max_points > 0 && out.points.size() > max_points) {
        const std::size_t stride = (out.points.size() + max_points - 1) / max_points;
        std::size_t k = 0;
        for (std::size_t i = 0; i < out.points.size(); i += stride, ++k) {
            out.points[k] = out.points[i];
            out.normals[k] = out.normals[i];
            out.weights[k] = out.weights[i];
        }
        out.points.resize(k);
        out.normals.resize(k);
        out.weights.resize(k);
    }
    return out;
}

std::vector<std::uint32_t> triangles_within(const MeshTopology& topo, std::uint32_t start, const Vec3f& center, float radius) {
    std::vector<std::uint32_t> out;
    if (start >= topo.triangle_count()) return out;
    std::unordered_set<std::uint32_t> seen{start};
    std::vector<std::uint32_t> stack{start};
    const float r2 = radius * radius;
    while (!stack.empty()) {
        const std::uint32_t t = stack.back();
        stack.pop_back();
        out.push_back(t);
        for (const auto n : topo.neighbors(t))
            if (n != kNoTriangle && (topo.centroid(n) - center).squaredNorm() <= r2 && seen.insert(n).second) stack.push_back(n);
    }
    std::ranges::sort(out);
    return out;
}

GrowResult grow_regions(const MeshTopology& topo, std::span<const RegionSeed> seeds, const GrowOptions& o,
                        std::span<const std::uint8_t> blocked) {
    GrowWorkspace ws;
    GrowResult result = grow_regions(topo, seeds, o, blocked, ws);
    // (The workspace is clean again; the owners come back from the regions.)
    result.owner.assign(topo.triangle_count(), 0);
    for (std::size_t r = 0; r < result.regions.size(); ++r)
        for (const auto t : result.regions[r].triangles) result.owner[t] = static_cast<std::uint16_t>(r + 1);
    return result;
}

GrowResult grow_regions(const MeshTopology& topo, std::span<const RegionSeed> seeds, const GrowOptions& o,
                        std::span<const std::uint8_t> blocked, GrowWorkspace& ws) {
    const std::size_t nt = topo.triangle_count();
    if (ws.owner.size() != nt) {
        ws.owner.assign(nt, 0);
        ws.owned.clear();
    }
    auto& owner = ws.owner;
    const auto own = [&](std::uint32_t t, std::size_t r) {
        if (owner[t] == 0) ws.owned.push_back(t);
        owner[t] = static_cast<std::uint16_t>(r + 1);
    };
    GrowResult result;
    result.regions.resize(seeds.size());
    const double cos_max = std::cos(o.max_normal_angle_deg * std::numbers::pi / 180.0);
    const auto is_blocked = [&](std::uint32_t t) { return !blocked.empty() && blocked[t] != 0; };

    std::vector<Active> active(seeds.size());
    std::priority_queue<Candidate, std::vector<Candidate>, std::greater<>> queue;

    const auto fits = [&](const Active& a, std::uint32_t t, double gate, double& score) {
        const Vec3 c = topo.centroid(t).cast<double>();
        const double d = std::abs(signed_distance(a.fit.surface, c));
        if (d > gate) return false;
        if (std::abs(normal_at(a.fit.surface, c).dot(topo.normal(t).cast<double>())) < cos_max) return false;
        score = d / gate;
        return true;
    };
    const auto push_neighbours = [&](std::uint16_t r, std::uint32_t t) {
        const Active& a = active[r];
        for (const auto n : topo.neighbors(t)) {
            if (n == kNoTriangle || owner[n] != 0 || is_blocked(n)) continue;
            double score = 0;
            if (fits(a, n, a.gate, score)) queue.push({score, n, r});
        }
    };

    for (std::size_t r = 0; r < seeds.size(); ++r) {
        RegionSeed seed = seeds[r];
        widen_small_seed(topo, seed.triangles, blocked);
        const RegionPoints pts = region_points(topo, seed.triangles, o.max_fit_points);
        std::optional<FitResult> fit;
        if (seed.surface) {
            fit = refine_surface(*seed.surface, pts.view(), o.fit);
        } else {
            const std::span<const SurfaceKind> kinds = seed.kinds.empty() ? std::span<const SurfaceKind>(kDefaultKinds) : seed.kinds;
            fit = choose_seed_surface(topo, seed, kinds, pts, o, blocked, ws);
        }
        if (!fit) continue;
        Active& a = active[r];
        a.fit = *fit;
        a.gate = gate_of(a.fit, o);
        // Seed triangles join only if they lie on the fitted surface: a dab that slipped over an edge does
        // not take the neighbouring face with it.
        for (const auto t : seed.triangles) {
            double score = 0;
            if (t < nt && owner[t] == 0 && !is_blocked(t) && fits(a, t, 2 * a.gate, score)) {
                own(t, r);
                a.members.push_back(t);
            }
        }
        a.fitted_size = a.members.size();
        result.regions[r].ok = true;
    }
    for (std::size_t r = 0; r < seeds.size(); ++r)
        for (const auto t : active[r].members) push_neighbours(static_cast<std::uint16_t>(r), t);

    const auto run_queue = [&] {
        while (!queue.empty()) {
            const Candidate c = queue.top();
            queue.pop();
            if (owner[c.triangle] != 0) continue;
            Active& a = active[c.region];
            double score = 0;
            if (!fits(a, c.triangle, a.gate, score)) continue;  // the surface moved since it was queued
            own(c.triangle, c.region);
            a.members.push_back(c.triangle);
            if (a.members.size() >= 2 * a.fitted_size + 16) {
                const RegionPoints pts = region_points(topo, a.members, o.max_fit_points);
                a.fit = refine_surface(a.fit.surface, pts.view(), o.fit);
                a.gate = gate_of(a.fit, o);
                a.fitted_size = a.members.size();
            }
            push_neighbours(c.region, c.triangle);
        }
    };
    run_queue();

    // A small seed cannot always tell a gentle curve from a plane. Once a region has stopped growing, its
    // kind is chosen again on all of its triangles; if that changes it, the region grows on with the new
    // surface (a dab on a large boss first grows as a plane strip, then as the cylinder).
    for (int round = 0; round < 3; ++round) {
        bool changed = false;
        for (std::size_t r = 0; r < seeds.size(); ++r) {
            if (!result.regions[r].ok || seeds[r].surface || seeds[r].kinds.size() == 1) continue;
            Active& a = active[r];
            const RegionPoints pts = region_points(topo, a.members, o.max_fit_points);
            const std::span<const SurfaceKind> kinds = seeds[r].kinds.empty() ? std::span<const SurfaceKind>(kDefaultKinds) : seeds[r].kinds;
            const auto best = fit_best(kinds, pts.view(), o.fit);
            if (!best || kind_of(best->surface) == kind_of(a.fit.surface)) continue;
            a.fit = *best;
            a.gate = gate_of(a.fit, o);
            a.fitted_size = a.members.size();
            for (const auto t : a.members) push_neighbours(static_cast<std::uint16_t>(r), t);
            changed = true;
        }
        if (!changed) break;
        run_queue();
    }

    reassign_borders(topo, active, ws, o, cos_max);
    for (const auto t : ws.owned) owner[t] = 0;
    ws.owned.clear();

    for (std::size_t r = 0; r < seeds.size(); ++r) {
        if (!result.regions[r].ok) continue;
        Active& a = active[r];
        std::ranges::sort(a.members);
        const RegionPoints pts = region_points(topo, a.members, o.max_fit_points);
        if (pts.points.size() > static_cast<std::size_t>(parameter_count(kind_of(a.fit.surface)))) a.fit = refine_surface(a.fit.surface, pts.view(), o.fit);
        result.regions[r].fit = a.fit;
        result.regions[r].triangles = std::move(a.members);
    }
    return result;
}

}  // namespace einstar::fit
