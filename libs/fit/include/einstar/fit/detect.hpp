#pragma once

// Proposing faces without seeds: the app's "auto-detect", whose regions the user or the agent accepts,
// names or discards.
//
// Seeds are placed automatically, smoothest surface first (where neighbouring triangles agree best), and
// grown like brushed seeds. Planes are taken first over the whole scan, then curved faces (cylinders,
// cones, spheres, tori) in what is left, so a fillet never swallows the flat faces it joins.

#include <cstdint>
#include <span>
#include <vector>

#include "einstar/fit/grow.hpp"

namespace einstar::fit {

struct DetectOptions {
    GrowOptions grow;  // max_sigma_mm 0: 1.5 x the scan's noise (estimate_noise)
    float seed_radius_mm = 1.5f;
    double min_plane_area_mm2 = 12.0;
    double min_curved_area_mm2 = 3.0;
    bool curved = true;
    // Triangles to leave alone (already labelled); may be empty.
    std::span<const std::uint8_t> blocked;
};

struct DetectedRegion {
    std::vector<std::uint32_t> triangles;
    FitResult fit;
    double area_mm2 = 0;
};

// The scan's noise level: the lower quartile of plane-fit sigmas over small patches spread across the mesh
// (curved and edge patches fit worse, flat ones show the noise).
[[nodiscard]] double estimate_noise(const MeshTopology& topo, std::size_t samples = 300, float patch_radius_mm = 1.5f);

[[nodiscard]] std::vector<DetectedRegion> detect_regions(const MeshTopology& topo, const DetectOptions& options = {});

}  // namespace einstar::fit
