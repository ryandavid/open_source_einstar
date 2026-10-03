#pragma once

// How far the model departs from the scan, both ways.
//
// Scan to model: the signed distance from every scan vertex to the model's surface (positive where the scan
// lies outside the model: material the model is missing). Shown as a colour map, and summarised for the agent
// per label (rms, p95, share within tolerance) and as hot spots: connected patches of the scan beyond the
// tolerance on one side, with their size, place and the model face they are closest to.
//
// Model to scan: the parts of each model face with no scan near them (extended or guessed surface).

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "einstar/fit/bvh.hpp"
#include "einstar/fit/mesh_topology.hpp"

namespace einstar::fit {

struct DeviationOptions {
    double tolerance_mm = 0.1;
    double max_distance_mm = 5.0;      // farther scan vertices count as unmatched (not part of the model)
    double min_hot_spot_area_mm2 = 0.5;
    double coverage_distance_mm = 1.0; // model surface farther than this from the scan is unsupported
    // Hot spots narrower than this are edge bands: a scan rounds sharp edges (over about a voxel), which
    // shows as thin strips along the model's edges, not as a modelling error.
    double edge_band_width_mm = 1.0;
};

struct DeviationField {
    std::vector<float> distance;  // per scan vertex, signed; NaN where unmatched
    std::vector<int> model_face;  // per scan vertex: the model face of its closest point (-1 unmatched)
};

// `model_triangle_face`: a face index per model triangle (e.g. brep::Tessellation::triangle_face).
[[nodiscard]] DeviationField scan_to_model(const recon::TriangleMesh& scan, const TriangleBvh& model,
                                           std::span<const int> model_triangle_face, const DeviationOptions& options = {});

struct DeviationStats {
    int group = -1;
    std::size_t vertices = 0;
    double rms = 0, p95 = 0, max_abs = 0, mean = 0;
    double within_tolerance = 0;  // share of vertices
};

struct HotSpot {
    Vec3 centroid = Vec3::Zero();
    double area_mm2 = 0;
    double mean_mm = 0;  // signed: + the scan is outside the model
    double peak_mm = 0;  // signed, largest magnitude
    int model_face = -1; // the most common closest model face
    int group = -1;      // the most common group (label) of its vertices
    std::size_t vertices = 0;
    double length_mm = 0, width_mm = 0;  // along its longest extent, and area / length
    bool edge_band = false;              // a thin strip (see DeviationOptions::edge_band_width_mm)
};

struct DeviationReport {
    DeviationStats overall;
    std::vector<DeviationStats> groups;  // by group id, ascending
    std::vector<HotSpot> hot_spots;      // largest (area x mean deviation) first, edge bands after the rest
    std::size_t unmatched = 0;
};

// `vertex_group`: a group (label) per scan vertex, -1 none; groups below -1 are left out entirely (e.g.
// scan of a fixture the model is not meant to cover). May be empty.
[[nodiscard]] DeviationReport summarise(const MeshTopology& scan, const DeviationField& field, std::span<const int> vertex_group,
                                        const DeviationOptions& options = {});

struct Coverage {
    std::vector<double> face_area, unsupported_area;  // per model face, mm^2
    std::vector<std::uint8_t> triangle_supported;     // per model triangle
};
[[nodiscard]] Coverage model_coverage(const recon::TriangleMesh& model, std::span<const int> model_triangle_face, int face_count,
                                      const TriangleBvh& scan, const DeviationOptions& options = {});

// Colour of a deviation: green within the tolerance, through yellow to red above it (scan outside the
// model), through cyan to blue below; grey for NaN. `range` is the deviation shown fully saturated.
[[nodiscard]] std::array<std::uint8_t, 4> deviation_color(float d, float tolerance, float range);

}  // namespace einstar::fit
