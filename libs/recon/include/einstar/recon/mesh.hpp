#pragma once

// Triangle meshes: extraction from the fused TSDF, cleanup and export.

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>

#include "einstar/core/error.hpp"
#include "einstar/core/se3.hpp"
#include "einstar/track/tsdf.hpp"

namespace einstar::recon {

struct TriangleMesh {
    std::vector<Vec3f> vertices;
    std::vector<Vec3f> normals;  // per vertex (may be empty)
    std::vector<std::array<std::uint32_t, 3>> triangles;

    [[nodiscard]] bool empty() const { return triangles.empty(); }
    [[nodiscard]] double area() const;
    void compute_normals();  // area-weighted vertex normals from the faces
};

struct ExtractParams {
    float min_weight = 1.0f;  // voxels observed less than this are treated as unknown
    // A sign change between two voxels only counts as surface when both lie within this fraction of
    // the truncation band (a +1 / -1 pair is the edge of the band, e.g. behind a thin wall).
    float max_crossing_jump = 0.6f;
    // Surface only where both voxels of a crossing were observed by at least this many frames
    // (needs TsdfParams::count_observations; 0 = off).
    int min_observations = 0;
};

// Surface nets on the TSDF zero level: one vertex per cell that straddles the surface (the mean of
// its edge crossings, then projected onto the trilinear zero level), quads across every crossing
// voxel edge. Normals from the SDF gradient (pointing into free space).
[[nodiscard]] TriangleMesh extract_mesh(const track::Volume& volume, const ExtractParams& params = {});

// Pieces are triangles connected through edges two triangles share: a flap hanging off the surface by
// a non-manifold edge or a single vertex (surface nets make a few where sheets touch) is a piece of
// its own.
struct CleanupParams {
    // Pieces smaller than this are removed: crumbs and flaps (EXStar removes 25 mm^2 at 0.5 mm).
    double min_component_area_mm2 = 25.0;
    // Pieces up to this fraction of the total area with no larger piece within about isolation_mm
    // are removed: floaters and fragments of the background. Fragments of the surface broken off by
    // holes lie next to it and stay.
    double isolated_component_fraction = 0.01;
    double isolation_mm = 50.0;
    // Pieces smaller than this fraction of the largest piece, or with fewer triangles, are removed
    // (0 = off; 0.02 / 200 was the default before the absolute rules, and also removed real
    // fragments: parts seen through openings, a bucket's rim).
    double min_component_fraction = 0.0;
    std::size_t min_component_triangles = 0;
};
struct CleanupReport {
    std::size_t components = 0;
    std::size_t removed_components = 0;
    std::size_t removed_isolated = 0;  // ... of them by the isolation rule
    std::size_t removed_triangles = 0;
    double removed_area_mm2 = 0;
};
CleanupReport remove_small_components(TriangleMesh& mesh, const CleanupParams& params = {});
void remove_unreferenced_vertices(TriangleMesh& mesh);

// Marker stickers: the depth front end fills the hole a sticker leaves in the depth with a plane,
// but the surface over it still comes out with a bump or dent (0.3-0.8 mm over ~8 mm on the car
// display recordings: the sticker's dark ring, stereo bleeding, a plane on a curved part). The
// surface over each marker is replaced by the smooth surface around it: a quadric height field
// fitted to an annulus, blended in at the edge. Markers whose surroundings are not smooth (an edge,
// a step) are left alone.
struct MarkerDisc {
    Vec3f center;
    Vec3f normal;  // out of the surface
    float radius = 3.0f;
};
struct MarkerFlattenParams {
    float cover_radii = 2.2f;   // replaced out to this many sticker radii...
    float blend_mm = 2.0f;      // ... blended into the surrounding surface over this width
    float ring_mm = 5.0f;       // the annulus outside that the surface is fitted to
    float max_ring_rms_mm = 0.2f;
    float max_height_mm = 3.0f;  // vertices this far off the marker's plane belong to other surfaces
};
// Returns the number of markers flattened.
std::size_t flatten_markers(TriangleMesh& mesh, const std::vector<MarkerDisc>& markers, const MarkerFlattenParams& params = {});
// Taubin (lambda/mu) smoothing: reduces voxel-scale noise without shrinking the surface.
void taubin_smooth(TriangleMesh& mesh, int iterations = 5, float lambda = 0.5f, float mu = -0.53f);

struct SimplifyParams {
    double target_ratio = 0.2;            // of the input triangles (ignored when target_triangles > 0)
    std::size_t target_triangles = 0;
    double max_error_mm = 0.02;           // stop before the (area-averaged) deviation exceeds this
    double boundary_weight = 100.0;       // keeps scan borders in place
    double min_normal_dot = 0.2;          // collapses may not turn a face by more than ~78 degrees
    // Meshes larger than this are simplified in parallel spatial blocks of this size.
    std::size_t parallel_min_triangles = 400000;
    double block_mm = 40.0;
};
struct SimplifyReport {
    std::size_t triangles_before = 0, triangles_after = 0;
    double max_error_mm = 0;
};
// Quadric-error edge collapse (Garland & Heckbert); keeps the surface manifold and borders fixed.
SimplifyReport simplify(TriangleMesh& mesh, const SimplifyParams& params = {});

enum class MeshFormat { stl, ply, obj };
[[nodiscard]] std::optional<MeshFormat> format_from_extension(const std::filesystem::path& path);
Result<void> save_mesh(const TriangleMesh& mesh, const std::filesystem::path& path);  // format from the extension

}  // namespace einstar::recon
