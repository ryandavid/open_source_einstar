#include "einstar/fit/detect.hpp"

#include <algorithm>
#include <numeric>

namespace einstar::fit {
namespace {

// How much a triangle's normal disagrees with its neighbours' (1 - the smallest dot product).
std::vector<float> roughness(const MeshTopology& topo) {
    std::vector<float> r(topo.triangle_count(), 2.0f);
    for (std::uint32_t t = 0; t < topo.triangle_count(); ++t) {
        float worst = 1.0f;
        int count = 0;
        for (const auto n : topo.neighbors(t))
            if (n != kNoTriangle) {
                worst = std::min(worst, topo.normal(t).dot(topo.normal(n)));
                ++count;
            }
        if (count == 3) r[t] = 1.0f - worst;
    }
    return r;
}

// The tightest radius of curvature of a surface (infinite for a plane).
double smallest_radius(const Surface& s) {
    if (const auto* c = std::get_if<Cylinder>(&s)) return c->radius;
    if (const auto* sp = std::get_if<Sphere>(&s)) return sp->radius;
    if (const auto* t = std::get_if<Torus>(&s)) return t->minor;
    return 1e30;
}

constexpr std::array kAnyKind{SurfaceKind::plane, SurfaceKind::cylinder, SurfaceKind::sphere};

double area_of(const MeshTopology& topo, std::span<const std::uint32_t> tris) {
    double a = 0;
    for (const auto t : tris) a += topo.area(t);
    return a;
}

// Neighbouring regions that are one face (two seeds grew parts of the same hole wall): merged when one
// surface fits both about as well as each fits alone.
void merge_same_surfaces(const MeshTopology& topo, std::vector<DetectedRegion>& regions, const DetectOptions& o, double noise) {
    for (bool merged = true; merged;) {
        merged = false;
        std::vector<std::uint32_t> owner(topo.triangle_count(), kNoTriangle);
        for (std::uint32_t r = 0; r < regions.size(); ++r)
            for (const auto t : regions[r].triangles) owner[t] = r;
        for (std::uint32_t a = 0; a < regions.size() && !merged; ++a) {
            std::vector<std::uint32_t> touching;
            for (const auto t : regions[a].triangles)
                for (const auto n : topo.neighbors(t))
                    if (n != kNoTriangle && owner[n] != kNoTriangle && owner[n] > a) touching.push_back(owner[n]);
            std::ranges::sort(touching);
            touching.erase(std::unique(touching.begin(), touching.end()), touching.end());
            for (const auto b : touching) {
                if (kind_of(regions[a].fit.surface) != kind_of(regions[b].fit.surface)) continue;
                std::vector<std::uint32_t> both = regions[a].triangles;
                both.insert(both.end(), regions[b].triangles.begin(), regions[b].triangles.end());
                const RegionPoints pts = region_points(topo, both, o.grow.max_fit_points);
                const FitResult fit = refine_surface(regions[a].triangles.size() >= regions[b].triangles.size() ? regions[a].fit.surface
                                                                                                                : regions[b].fit.surface,
                                                     pts.view(), o.grow.fit);
                // Each region's own points must sit on the merged surface about as well as on its own (the
                // robust fit alone would let a large region absorb a small one as outliers).
                const auto fits_part = [&](const DetectedRegion& part) {
                    const RegionPoints pp = region_points(topo, part.triangles, o.grow.max_fit_points);
                    std::vector<double> res(pp.points.size());
                    for (std::size_t i = 0; i < res.size(); ++i) res[i] = signed_distance(fit.surface, pp.points[i]);
                    // (A small region overfits its own noise, so the scan's noise level is a floor.)
                    return robust_sigma(res) < std::max(1.25 * part.fit.sigma, 1.5 * noise);
                };
                if (!fits_part(regions[a]) || !fits_part(regions[b])) continue;
                std::ranges::sort(both);
                regions[a] = {std::move(both), fit, regions[a].area_mm2 + regions[b].area_mm2};
                regions.erase(regions.begin() + b);
                merged = true;
                break;
            }
        }
    }
}

}  // namespace

double estimate_noise(const MeshTopology& topo, std::size_t samples, float patch_radius_mm) {
    const std::size_t nt = topo.triangle_count();
    if (nt == 0) return 0;
    std::vector<double> sigmas;
    const std::size_t stride = std::max<std::size_t>(1, nt / samples);
    const std::array plane{SurfaceKind::plane};
    for (std::size_t t = 0; t < nt; t += stride) {
        const auto tri = static_cast<std::uint32_t>(t);
        const auto patch = triangles_within(topo, tri, topo.centroid(tri), patch_radius_mm);
        if (patch.size() < 12) continue;
        const RegionPoints pts = region_points(topo, patch, 0);
        if (const auto r = fit_best(plane, pts.view())) sigmas.push_back(r->sigma);
    }
    if (sigmas.empty()) return 0;
    const auto q = sigmas.begin() + static_cast<std::ptrdiff_t>(sigmas.size() / 4);
    std::nth_element(sigmas.begin(), q, sigmas.end());
    return *q;
}

std::vector<DetectedRegion> detect_regions(const MeshTopology& topo, const DetectOptions& options) {
    DetectOptions o = options;
    const double noise = estimate_noise(topo);
    if (o.grow.max_sigma_mm <= 0 && noise > 0) o.grow.max_sigma_mm = 1.5 * noise;
    const std::size_t nt = topo.triangle_count();
    std::vector<std::uint8_t> taken(nt, 0), tried(nt, 0);
    for (std::size_t t = 0; t < o.blocked.size() && t < nt; ++t) taken[t] = o.blocked[t];
    const std::vector<float> rough = roughness(topo);
    std::vector<std::uint32_t> order(nt);
    std::iota(order.begin(), order.end(), 0u);
    std::ranges::stable_sort(order, [&](std::uint32_t a, std::uint32_t b) { return rough[a] < rough[b]; });

    std::vector<DetectedRegion> out;
    const auto pass = [&](std::span<const SurfaceKind> kinds, double min_area) {
        const bool planes_only = kinds.size() == 1 && kinds[0] == SurfaceKind::plane;
        std::ranges::fill(tried, 0);
        for (const std::uint32_t start : order) {
            if (taken[start] || tried[start] || rough[start] > 0.5f) continue;
            std::vector<std::uint32_t> seed = triangles_within(topo, start, topo.centroid(start), o.seed_radius_mm);
            if (planes_only) {
                // Only where the patch itself is flat, judged on the whole patch (the strip of a fillet left
                // between two planes is narrow enough to look flat).
                const RegionPoints pts = region_points(topo, seed, o.grow.max_fit_points);
                const auto best = fit_best(kAnyKind, pts.view(), o.grow.fit);
                if (!best || kind_of(best->surface) != SurfaceKind::plane) {
                    // Its middle is curved too: no need to try those as starts again.
                    const float r2 = 0.25f * o.seed_radius_mm * o.seed_radius_mm;
                    for (const auto t : seed)
                        if ((topo.centroid(t) - topo.centroid(start)).squaredNorm() < r2) tried[t] = 1;
                    continue;
                }
            }
            std::erase_if(seed, [&](std::uint32_t t) { return taken[t] != 0; });
            for (const auto t : seed) tried[t] = 1;
            if (seed.size() < 8) continue;
            const RegionSeed rs{std::move(seed), {kinds.begin(), kinds.end()}, std::nullopt};
            const GrowResult g = grow_regions(topo, std::span(&rs, 1), o.grow, taken);
            if (!g.regions[0].ok) continue;
            const double area = area_of(topo, g.regions[0].triangles);
            // A surface that does not fit (a plane laid on a fillet) shows as noise well above the scan's.
            if (area < min_area || (noise > 0 && g.regions[0].fit.sigma > (planes_only ? 1.5 : 2.0) * noise)) continue;
            if (o.min_radius_mm > 0 && smallest_radius(g.regions[0].fit.surface) < o.min_radius_mm) continue;
            for (const auto t : g.regions[0].triangles) taken[t] = 1;
            out.push_back({g.regions[0].triangles, g.regions[0].fit, area});
        }
    };
    const std::array planes{SurfaceKind::plane};
    pass(planes, o.min_plane_area_mm2);
    if (o.curved) {
        const std::array curved{SurfaceKind::cylinder, SurfaceKind::cone, SurfaceKind::sphere, SurfaceKind::torus};
        pass(curved, o.min_curved_area_mm2);
    }
    merge_same_surfaces(topo, out, o, noise);
    return out;
}

}  // namespace einstar::fit
