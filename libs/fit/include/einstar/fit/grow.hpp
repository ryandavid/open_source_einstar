#pragma once

// Growing brushed seeds into whole faces.
//
// A seed is a few triangles the user (or the agent) painted on a face. Its surface is fitted, then the
// region grows across triangle edges to every neighbour that lies on that surface (within a few sigma)
// and faces the same way. All seeds grow at once from one priority queue ordered by how well each
// candidate fits, so two labels meeting at a fillet split it where their surfaces part, and each surface is
// refitted as its region doubles.

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "einstar/fit/mesh_topology.hpp"
#include "einstar/fit/primitive_fit.hpp"

namespace einstar::fit {

struct GrowOptions {
    double gate_sigma = 3.0;     // a triangle joins within this many sigma (of its centroid) of the surface...
    double min_gate_mm = 0.06;   // ...but never a narrower band than this
    double max_normal_angle_deg = 25;
    // Cap on the sigma the gate is computed from (0: none). Without it a surface that does not fit (a
    // plane on a fillet) widens its own gate as it takes worse triangles.
    double max_sigma_mm = 0;
    std::size_t max_fit_points = 4000;
    FitOptions fit;
};

struct RegionSeed {
    std::vector<std::uint32_t> triangles;
    // Kinds to try for the seed's surface (see choose_seed_surface); empty: plane, cylinder, cone, sphere.
    std::vector<SurfaceKind> kinds;
    // Start from this surface instead of fitting the seed (e.g. a refit after constraints).
    std::optional<Surface> surface;
};

struct GrownRegion {
    std::vector<std::uint32_t> triangles;
    FitResult fit;
    bool ok = false;  // false: the seed could not be fitted
};

struct GrowResult {
    std::vector<std::uint16_t> owner;  // per triangle: 0 none, i + 1 = region i
    std::vector<GrownRegion> regions;
};

// `blocked[t] != 0` keeps triangle t out of every region (may be empty).
[[nodiscard]] GrowResult grow_regions(const MeshTopology& topo, std::span<const RegionSeed> seeds, const GrowOptions& options = {},
                                      std::span<const std::uint8_t> blocked = {});

struct RegionPoints;
// The surface a seed grows with when its kind is not given (see grow.cpp).
[[nodiscard]] std::optional<FitResult> choose_seed_surface(const MeshTopology& topo, const RegionSeed& seed, std::span<const SurfaceKind> kinds,
                                                           const RegionPoints& points, const GrowOptions& options,
                                                           std::span<const std::uint8_t> blocked = {});

// Triangles connected to `start` whose centroids lie within `radius` of `center` (a brush dab).
[[nodiscard]] std::vector<std::uint32_t> triangles_within(const MeshTopology& topo, std::uint32_t start, const Vec3f& center,
                                                          float radius);

// Fit data of a set of triangles: their vertices (on the surface, unlike centroids, which sit inside a
// tight curve by the chord's sagitta), normals from the set's own triangles, and a third of each triangle's
// area per corner; evenly subsampled to at most max_points.
struct RegionPoints {
    std::vector<Vec3> points, normals;
    std::vector<double> weights;
    [[nodiscard]] PointSet view() const { return {points, normals, weights}; }
};
[[nodiscard]] RegionPoints region_points(const MeshTopology& topo, std::span<const std::uint32_t> triangles, std::size_t max_points);

}  // namespace einstar::fit
