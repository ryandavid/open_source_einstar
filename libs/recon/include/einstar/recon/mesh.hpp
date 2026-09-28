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
};

// Surface nets on the TSDF zero level: one vertex per cell that straddles the surface (the mean of
// its edge crossings, then projected onto the trilinear zero level), quads across every crossing
// voxel edge. Normals from the SDF gradient (pointing into free space).
[[nodiscard]] TriangleMesh extract_mesh(const track::Volume& volume, const ExtractParams& params = {});

struct CleanupParams {
    // Connected pieces smaller than this fraction of the largest piece (or this many triangles) are
    // removed: stray blobs from stereo outliers, fragments of the background.
    double min_component_fraction = 0.02;
    std::size_t min_component_triangles = 200;
};
struct CleanupReport {
    std::size_t components = 0;
    std::size_t removed_components = 0;
    std::size_t removed_triangles = 0;
};
CleanupReport remove_small_components(TriangleMesh& mesh, const CleanupParams& params = {});
void remove_unreferenced_vertices(TriangleMesh& mesh);
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
