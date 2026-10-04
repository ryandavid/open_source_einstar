#pragma once

// Bridge to PoissonRecon (screened Poisson surface reconstruction, Kazhdan & Hoppe 2013). Its code
// needs C++20 (it uses std::is_pod_v), so it is compiled in a translation unit of its own behind
// this plain interface.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace einstar::recon::detail {

struct PoissonInput {
    const float* points = nullptr;   // xyz per sample
    const float* normals = nullptr;  // xyz per sample, unit, pointing out of the surface
    std::size_t count = 0;
    double cell_mm = 0.5;          // finest octree cell
    double point_weight = 4.0;     // screening: how strongly the surface is held to the samples
    double samples_per_node = 1.5;
};

struct PoissonOutput {
    std::vector<float> vertices;  // xyz
    std::vector<std::uint32_t> triangles;
    std::string error;            // non-empty on failure
};

[[nodiscard]] PoissonOutput screened_poisson(const PoissonInput& in);

}  // namespace einstar::recon::detail
