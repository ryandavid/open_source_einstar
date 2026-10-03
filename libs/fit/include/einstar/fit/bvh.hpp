#pragma once

// Bounding-volume hierarchy over a mesh's triangles: ray casts (picking, seeding a label at a 3D point)
// and closest points (distances between scan and model).

#include <cstdint>
#include <optional>
#include <vector>

#include "einstar/recon/mesh.hpp"

namespace einstar::fit {

struct RayHit {
    std::uint32_t triangle = 0;
    float t = 0;  // along the ray direction (in units of its length)
    Vec3f point = Vec3f::Zero();
};

struct ClosestPoint {
    std::uint32_t triangle = 0;
    Vec3f point = Vec3f::Zero();
    float distance = 0;
};

class TriangleBvh {
public:
    // The mesh must outlive the hierarchy and not change.
    explicit TriangleBvh(const recon::TriangleMesh& mesh);

    // First triangle hit by origin + t * dir, t in [t_min, t_max]. Both faces count.
    [[nodiscard]] std::optional<RayHit> raycast(const Vec3f& origin, const Vec3f& dir, float t_min = 0,
                                                float t_max = 1e30f) const;
    // Closest point on any triangle within max_distance.
    [[nodiscard]] std::optional<ClosestPoint> closest(const Vec3f& p, float max_distance = 1e30f) const;

    [[nodiscard]] const recon::TriangleMesh& mesh() const { return *mesh_; }

private:
    struct Node {
        Eigen::AlignedBox3f box;
        std::uint32_t first = 0;  // leaf: first index into order_; inner: right child (left is this + 1)
        std::uint32_t count = 0;  // leaf: triangles; inner: 0
    };
    std::uint32_t build(std::uint32_t begin, std::uint32_t end, const std::vector<Vec3f>& centroids);

    const recon::TriangleMesh* mesh_;
    std::vector<Node> nodes_;
    std::vector<std::uint32_t> order_;
};

// Closest point on triangle (a, b, c) to p (Ericson, Real-Time Collision Detection 5.1.5).
[[nodiscard]] Vec3f closest_on_triangle(const Vec3f& p, const Vec3f& a, const Vec3f& b, const Vec3f& c);

}  // namespace einstar::fit
