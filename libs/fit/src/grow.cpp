#include "einstar/fit/grow.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <queue>

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

double gate_of(const FitResult& f, const GrowOptions& o) { return std::max(o.gate_sigma * f.sigma, o.min_gate_mm); }

}  // namespace

// The surface a seed grows with. Seeds are small, and the edges of a face bend in a scan (sharp edges are
// rounded over a voxel, the mesh frays where observation stops), so a curved fit can beat the plane on the
// seed alone. Each plausible kind is grown on its own and the simplest one that grows clearly furthest
// (by 20%) wins: the face's extent decides, not the few triangles under the brush.
std::optional<FitResult> choose_seed_surface(const MeshTopology& topo, const RegionSeed& seed, std::span<const SurfaceKind> kinds,
                                             const RegionPoints& pts, const GrowOptions& o, std::span<const std::uint8_t> blocked) {
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
        const GrowResult g = grow_regions(topo, std::span(&trial, 1), o, blocked);
        const std::size_t size = g.regions[0].triangles.size();
        if (i == 0 || static_cast<double>(size) > 1.2 * static_cast<double>(best_size)) {
            best = i;
            best_size = size;
        }
    }
    return candidates[best];
}

RegionPoints region_points(const MeshTopology& topo, std::span<const std::uint32_t> triangles, std::size_t max_points) {
    RegionPoints out;
    const std::size_t n = triangles.size();
    const std::size_t stride = n > max_points && max_points > 0 ? (n + max_points - 1) / max_points : 1;
    out.points.reserve(n / stride + 1);
    for (std::size_t i = 0; i < n; i += stride) {
        const std::uint32_t t = triangles[i];
        if (topo.area(t) <= 0) continue;
        out.points.push_back(topo.centroid(t).cast<double>());
        out.normals.push_back(topo.normal(t).cast<double>());
        out.weights.push_back(topo.area(t));
    }
    return out;
}

std::vector<std::uint32_t> triangles_within(const MeshTopology& topo, std::uint32_t start, const Vec3f& center, float radius) {
    std::vector<std::uint32_t> out;
    if (start >= topo.triangle_count()) return out;
    std::vector<std::uint8_t> seen(topo.triangle_count(), 0);
    std::vector<std::uint32_t> stack{start};
    seen[start] = 1;
    const float r2 = radius * radius;
    while (!stack.empty()) {
        const std::uint32_t t = stack.back();
        stack.pop_back();
        out.push_back(t);
        for (const auto n : topo.neighbors(t))
            if (n != kNoTriangle && !seen[n] && (topo.centroid(n) - center).squaredNorm() <= r2) {
                seen[n] = 1;
                stack.push_back(n);
            }
    }
    std::ranges::sort(out);
    return out;
}

GrowResult grow_regions(const MeshTopology& topo, std::span<const RegionSeed> seeds, const GrowOptions& o,
                        std::span<const std::uint8_t> blocked) {
    const std::size_t nt = topo.triangle_count();
    GrowResult result;
    result.owner.assign(nt, 0);
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
            if (n == kNoTriangle || result.owner[n] != 0 || is_blocked(n)) continue;
            double score = 0;
            if (fits(a, n, a.gate, score)) queue.push({score, n, r});
        }
    };

    for (std::size_t r = 0; r < seeds.size(); ++r) {
        const RegionSeed& seed = seeds[r];
        const RegionPoints pts = region_points(topo, seed.triangles, o.max_fit_points);
        std::optional<FitResult> fit;
        if (seed.surface) {
            fit = refine_surface(*seed.surface, pts.view(), o.fit);
        } else {
            const std::span<const SurfaceKind> kinds = seed.kinds.empty() ? std::span<const SurfaceKind>(kDefaultKinds) : seed.kinds;
            fit = choose_seed_surface(topo, seed, kinds, pts, o, blocked);
        }
        if (!fit) continue;
        Active& a = active[r];
        a.fit = *fit;
        a.gate = gate_of(a.fit, o);
        // Seed triangles join only if they lie on the fitted surface: a dab that slipped over an edge does
        // not take the neighbouring face with it.
        for (const auto t : seed.triangles) {
            double score = 0;
            if (t < nt && result.owner[t] == 0 && !is_blocked(t) && fits(a, t, 2 * a.gate, score)) {
                result.owner[t] = static_cast<std::uint16_t>(r + 1);
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
            if (result.owner[c.triangle] != 0) continue;
            Active& a = active[c.region];
            double score = 0;
            if (!fits(a, c.triangle, a.gate, score)) continue;  // the surface moved since it was queued
            result.owner[c.triangle] = static_cast<std::uint16_t>(c.region + 1);
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
