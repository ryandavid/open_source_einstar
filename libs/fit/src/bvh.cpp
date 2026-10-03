#include "einstar/fit/bvh.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>

namespace einstar::fit {
namespace {

constexpr std::uint32_t kLeafSize = 4;

// Slab test; returns the entry distance or +inf.
float ray_box(const Eigen::AlignedBox3f& box, const Vec3f& o, const Vec3f& inv_dir, float t_min, float t_max) {
    for (int a = 0; a < 3; ++a) {
        float t0 = (box.min()[a] - o[a]) * inv_dir[a];
        float t1 = (box.max()[a] - o[a]) * inv_dir[a];
        if (t0 > t1) std::swap(t0, t1);
        t_min = std::max(t_min, t0);
        t_max = std::min(t_max, t1);
        if (t_max < t_min) return INFINITY;
    }
    return t_min;
}

// Möller-Trumbore, both faces.
bool ray_triangle(const Vec3f& o, const Vec3f& d, const Vec3f& a, const Vec3f& b, const Vec3f& c, float& t) {
    const Vec3f e1 = b - a, e2 = c - a;
    const Vec3f p = d.cross(e2);
    const float det = e1.dot(p);
    if (std::abs(det) < 1e-12f) return false;
    const float inv = 1.0f / det;
    const Vec3f s = o - a;
    const float u = s.dot(p) * inv;
    if (u < 0 || u > 1) return false;
    const Vec3f q = s.cross(e1);
    const float v = d.dot(q) * inv;
    if (v < 0 || u + v > 1) return false;
    t = e2.dot(q) * inv;
    return true;
}

}  // namespace

Vec3f closest_on_triangle(const Vec3f& p, const Vec3f& a, const Vec3f& b, const Vec3f& c) {
    const Vec3f ab = b - a, ac = c - a, ap = p - a;
    const float d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const Vec3f bp = p - b;
    const float d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + (d1 / (d1 - d3)) * ab;
    const Vec3f cp = p - c;
    const float d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + (d2 / (d2 - d6)) * ac;
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);
    const float denom = 1.0f / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

TriangleBvh::TriangleBvh(const recon::TriangleMesh& mesh) : mesh_(&mesh) {
    const std::size_t n = mesh.triangles.size();
    order_.resize(n);
    std::iota(order_.begin(), order_.end(), 0u);
    std::vector<Vec3f> centroids(n);
    for (std::size_t t = 0; t < n; ++t) {
        const auto& tri = mesh.triangles[t];
        centroids[t] = (mesh.vertices[tri[0]] + mesh.vertices[tri[1]] + mesh.vertices[tri[2]]) / 3.0f;
    }
    nodes_.reserve(n > 0 ? 2 * n / kLeafSize + 1 : 0);
    if (n > 0) build(0, static_cast<std::uint32_t>(n), centroids);
}

std::uint32_t TriangleBvh::build(std::uint32_t begin, std::uint32_t end, const std::vector<Vec3f>& centroids) {
    const auto index = static_cast<std::uint32_t>(nodes_.size());
    nodes_.push_back({});
    Eigen::AlignedBox3f box, cbox;
    for (std::uint32_t i = begin; i < end; ++i) {
        const auto& tri = mesh_->triangles[order_[i]];
        for (const auto v : tri) box.extend(mesh_->vertices[v]);
        cbox.extend(centroids[order_[i]]);
    }
    nodes_[index].box = box;
    if (end - begin <= kLeafSize) {
        nodes_[index].first = begin;
        nodes_[index].count = end - begin;
        return index;
    }
    int axis = 0;
    cbox.sizes().maxCoeff(&axis);
    const std::uint32_t mid = begin + (end - begin) / 2;
    std::nth_element(order_.begin() + begin, order_.begin() + mid, order_.begin() + end,
                     [&](std::uint32_t a, std::uint32_t b) { return centroids[a][axis] < centroids[b][axis]; });
    build(begin, mid, centroids);
    const std::uint32_t right = build(mid, end, centroids);
    nodes_[index].first = right;
    nodes_[index].count = 0;
    return index;
}

std::optional<RayHit> TriangleBvh::raycast(const Vec3f& origin, const Vec3f& dir, float t_min, float t_max) const {
    if (nodes_.empty()) return std::nullopt;
    const Vec3f inv(1.0f / dir.x(), 1.0f / dir.y(), 1.0f / dir.z());
    std::optional<RayHit> best;
    std::array<std::uint32_t, 64> stack{};
    std::size_t top = 0;
    stack[top++] = 0;
    while (top > 0) {
        const Node& node = nodes_[stack[--top]];
        if (ray_box(node.box, origin, inv, t_min, t_max) == INFINITY) continue;
        if (node.count > 0) {
            for (std::uint32_t i = node.first; i < node.first + node.count; ++i) {
                const auto& tri = mesh_->triangles[order_[i]];
                float t = 0;
                if (ray_triangle(origin, dir, mesh_->vertices[tri[0]], mesh_->vertices[tri[1]], mesh_->vertices[tri[2]], t) &&
                    t >= t_min && t <= t_max) {
                    t_max = t;
                    best = RayHit{order_[i], t, origin + t * dir};
                }
            }
            continue;
        }
        const auto self = static_cast<std::uint32_t>(&node - nodes_.data());
        const std::uint32_t left = self + 1, right = node.first;
        const float tl = ray_box(nodes_[left].box, origin, inv, t_min, t_max);
        const float tr = ray_box(nodes_[right].box, origin, inv, t_min, t_max);
        // Nearer child last, so it is visited first.
        if (tl < tr) {
            if (tr != INFINITY) stack[top++] = right;
            if (tl != INFINITY) stack[top++] = left;
        } else {
            if (tl != INFINITY) stack[top++] = left;
            if (tr != INFINITY) stack[top++] = right;
        }
    }
    return best;
}

std::optional<ClosestPoint> TriangleBvh::closest(const Vec3f& p, float max_distance) const {
    if (nodes_.empty()) return std::nullopt;
    float best_d2 = max_distance * max_distance;
    std::optional<ClosestPoint> best;
    std::array<std::uint32_t, 64> stack{};
    std::size_t top = 0;
    stack[top++] = 0;
    while (top > 0) {
        const Node& node = nodes_[stack[--top]];
        if (node.box.squaredExteriorDistance(p) > best_d2) continue;
        if (node.count > 0) {
            for (std::uint32_t i = node.first; i < node.first + node.count; ++i) {
                const auto& tri = mesh_->triangles[order_[i]];
                const Vec3f q = closest_on_triangle(p, mesh_->vertices[tri[0]], mesh_->vertices[tri[1]], mesh_->vertices[tri[2]]);
                const float d2 = (q - p).squaredNorm();
                if (d2 <= best_d2) {
                    best_d2 = d2;
                    best = ClosestPoint{order_[i], q, 0};
                }
            }
            continue;
        }
        const auto self = static_cast<std::uint32_t>(&node - nodes_.data());
        const std::uint32_t left = self + 1, right = node.first;
        const float dl = nodes_[left].box.squaredExteriorDistance(p), dr = nodes_[right].box.squaredExteriorDistance(p);
        if (dl < dr) {
            stack[top++] = right;
            stack[top++] = left;
        } else {
            stack[top++] = left;
            stack[top++] = right;
        }
    }
    if (best) best->distance = std::sqrt(best_d2);
    return best;
}

}  // namespace einstar::fit
