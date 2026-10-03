#pragma once

// Connectivity of a scan mesh: triangle neighbours across edges, triangles around each vertex, the
// mesh's boundary loops (holes in the scan, openings of holes in the part).

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "einstar/recon/mesh.hpp"

namespace einstar::fit {

inline constexpr std::uint32_t kNoTriangle = 0xffffffffu;

class MeshTopology {
public:
    explicit MeshTopology(const recon::TriangleMesh& mesh);

    [[nodiscard]] std::size_t triangle_count() const { return neighbors_.size(); }
    [[nodiscard]] std::size_t vertex_count() const { return vertex_offsets_.size() - 1; }

    // Neighbour across edge e (from corner e to corner e+1), or kNoTriangle on a boundary or a
    // non-manifold edge (shared by more than two triangles).
    [[nodiscard]] const std::array<std::uint32_t, 3>& neighbors(std::uint32_t tri) const { return neighbors_[tri]; }
    [[nodiscard]] std::span<const std::uint32_t> vertex_triangles(std::uint32_t v) const {
        return {vertex_triangles_.data() + vertex_offsets_[v], vertex_offsets_[v + 1] - vertex_offsets_[v]};
    }

    [[nodiscard]] const Vec3f& normal(std::uint32_t tri) const { return normals_[tri]; }  // unit (zero if degenerate)
    [[nodiscard]] float area(std::uint32_t tri) const { return areas_[tri]; }
    [[nodiscard]] const Vec3f& centroid(std::uint32_t tri) const { return centroids_[tri]; }
    [[nodiscard]] std::span<const Vec3f> normals() const { return normals_; }
    [[nodiscard]] std::span<const float> areas() const { return areas_; }
    [[nodiscard]] std::span<const Vec3f> centroids() const { return centroids_; }

    // Closed loops of boundary edges, as vertex indices in order (the first vertex is not repeated).
    // Each loop runs the way its triangles' edges do (the surface on the left, seen from outside).
    [[nodiscard]] const std::vector<std::vector<std::uint32_t>>& boundary_loops() const { return loops_; }
    [[nodiscard]] bool is_boundary_vertex(std::uint32_t v) const { return boundary_vertex_[v] != 0; }

    // Connected components over shared edges: a component id per triangle, and the count.
    [[nodiscard]] std::vector<std::uint32_t> components(std::uint32_t& count) const;

private:
    std::vector<std::array<std::uint32_t, 3>> neighbors_;
    std::vector<std::uint32_t> vertex_offsets_, vertex_triangles_;
    std::vector<Vec3f> normals_, centroids_;
    std::vector<float> areas_;
    std::vector<std::vector<std::uint32_t>> loops_;
    std::vector<std::uint8_t> boundary_vertex_;
};

}  // namespace einstar::fit
