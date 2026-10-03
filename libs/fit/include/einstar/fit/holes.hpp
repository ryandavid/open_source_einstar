#pragma once

// Round holes in a face.
//
// A handheld scan sees little of a small hole's wall, so a hole is found from its opening in the host
// face: an inner boundary of the face's region that is close to a circle. The diameter comes from the
// wall where enough of it was scanned (a circle fit about an axis normal to the face), otherwise from the
// opening, corrected for the scan rounding the hole's edge (the face's region stops short of it).

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "einstar/fit/bvh.hpp"
#include "einstar/fit/mesh_topology.hpp"
#include "einstar/fit/surface.hpp"

namespace einstar::fit {

struct HoleOptions {
    double min_diameter = 1.0, max_diameter = 60.0;  // mm
    double max_roundness_error = 0.08;   // rms radial residual of the opening / radius
    double voxel_mm = 0.5;               // the scan's voxel size (the edge rounding scales with it)
    // The opening is this much wider than the hole (radius, in voxels): measured on synthetic surface-nets
    // scans whose hole walls were not seen; real scans may need their own value.
    double rim_bias_voxels = 1.05;
    std::size_t min_wall_triangles = 20;
    double min_wall_arc_deg = 120;       // wall data must cover this much of the circle
};

struct HoleCandidate {
    Vec3 center = Vec3::Zero();  // on the host face
    Vec3 axis = Vec3::UnitZ();   // unit, into the material
    double diameter = 0;         // best estimate (wall if available, else corrected opening)
    double opening_diameter = 0; // circle through the host face's region boundary
    std::optional<double> wall_diameter;
    double roundness_error = 0;  // of the opening, relative
    std::vector<std::uint32_t> wall;   // wall triangles (between the opening and the deepest wall seen)
    double wall_depth = 0;             // how deep the wall was scanned, mm
    std::optional<double> floor_depth; // a floor was seen: a blind hole this deep (to the shoulder, if pointed)
    // The hole's forms, when the scan shows them.
    std::optional<double> counterbore_diameter, counterbore_depth;
    std::optional<double> countersink_diameter, countersink_angle_deg;  // included angle
    std::optional<double> point_angle_deg;                               // a drill point's included angle
    std::vector<std::uint32_t> opening;  // the opening's boundary loop (vertex indices)
};

// Closed loops of the boundary of a set of triangles (vertex indices), outermost (longest) first.
[[nodiscard]] std::vector<std::vector<std::uint32_t>> region_boundary_loops(const recon::TriangleMesh& mesh, const MeshTopology& topo,
                                                                             std::span<const std::uint8_t> in_region);

// Holes in a plane face given by its region's triangles and fitted plane (normal out of the material).
[[nodiscard]] std::vector<HoleCandidate> find_holes(const recon::TriangleMesh& mesh, const MeshTopology& topo, const TriangleBvh& bvh,
                                                    std::span<const std::uint32_t> plane_region, const Plane& plane,
                                                    const HoleOptions& options = {});

// A 2D circle fitted to points (algebraic start, then geometric Gauss-Newton with Tukey weights).
struct Circle2 {
    Vec2 center = Vec2::Zero();
    double radius = 0;
    double rms = 0;  // radial residual of the inliers
};
[[nodiscard]] std::optional<Circle2> fit_circle_2d(std::span<const Vec2> points, std::span<const double> weights = {});

}  // namespace einstar::fit
