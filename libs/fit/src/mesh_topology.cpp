#include "einstar/fit/mesh_topology.hpp"

#include <algorithm>
#include <unordered_map>

namespace einstar::fit {
namespace {

std::uint64_t edge_key(std::uint32_t a, std::uint32_t b) {
    return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
}

}  // namespace

MeshTopology::MeshTopology(const recon::TriangleMesh& mesh) {
    const std::size_t nt = mesh.triangles.size(), nv = mesh.vertices.size();
    neighbors_.assign(nt, {kNoTriangle, kNoTriangle, kNoTriangle});
    normals_.resize(nt);
    centroids_.resize(nt);
    areas_.resize(nt);
    for (std::size_t t = 0; t < nt; ++t) {
        const auto& tri = mesh.triangles[t];
        const Vec3f &a = mesh.vertices[tri[0]], &b = mesh.vertices[tri[1]], &c = mesh.vertices[tri[2]];
        const Vec3f n = (b - a).cross(c - a);
        const float len = n.norm();
        areas_[t] = 0.5f * len;
        normals_[t] = len > 0 ? Vec3f(n / len) : Vec3f::Zero();
        centroids_[t] = (a + b + c) / 3.0f;
    }

    // Vertex -> triangles (CSR).
    vertex_offsets_.assign(nv + 1, 0);
    for (const auto& tri : mesh.triangles)
        for (const auto v : tri) ++vertex_offsets_[v + 1];
    for (std::size_t v = 0; v < nv; ++v) vertex_offsets_[v + 1] += vertex_offsets_[v];
    vertex_triangles_.resize(vertex_offsets_[nv]);
    {
        std::vector<std::uint32_t> fill(vertex_offsets_.begin(), vertex_offsets_.end() - 1);
        for (std::size_t t = 0; t < nt; ++t)
            for (const auto v : mesh.triangles[t]) vertex_triangles_[fill[v]++] = static_cast<std::uint32_t>(t);
    }

    // Half-edges sorted by undirected edge: pairs become neighbours, singles are boundary.
    struct HalfEdge {
        std::uint64_t key;
        std::uint32_t tri;
        std::uint32_t corner;
    };
    std::vector<HalfEdge> half;
    half.reserve(nt * 3);
    for (std::size_t t = 0; t < nt; ++t)
        for (std::uint32_t e = 0; e < 3; ++e)
            half.push_back({edge_key(mesh.triangles[t][e], mesh.triangles[t][(e + 1) % 3]), static_cast<std::uint32_t>(t), e});
    std::ranges::sort(half, [](const HalfEdge& a, const HalfEdge& b) { return a.key < b.key; });

    boundary_vertex_.assign(nv, 0);
    std::unordered_multimap<std::uint32_t, std::uint32_t> boundary_next;  // from -> to, along triangle orientation
    for (std::size_t i = 0; i < half.size();) {
        std::size_t j = i + 1;
        while (j < half.size() && half[j].key == half[i].key) ++j;
        if (j - i == 2) {
            neighbors_[half[i].tri][half[i].corner] = half[i + 1].tri;
            neighbors_[half[i + 1].tri][half[i + 1].corner] = half[i].tri;
        } else if (j - i == 1) {
            const auto& tri = mesh.triangles[half[i].tri];
            const std::uint32_t a = tri[half[i].corner], b = tri[(half[i].corner + 1) % 3];
            boundary_next.emplace(a, b);
            boundary_vertex_[a] = boundary_vertex_[b] = 1;
        }
        i = j;
    }

    // Chain boundary edges into loops. At a vertex where several loops touch, any outgoing edge is taken.
    while (!boundary_next.empty()) {
        auto it = boundary_next.begin();
        const std::uint32_t start = it->first;
        std::vector<std::uint32_t> loop{start};
        std::uint32_t cur = it->second;
        boundary_next.erase(it);
        while (cur != start) {
            auto nx = boundary_next.find(cur);
            if (nx == boundary_next.end()) break;  // open chain (non-manifold); kept as is
            loop.push_back(cur);
            cur = nx->second;
            boundary_next.erase(nx);
        }
        loops_.push_back(std::move(loop));
    }
    std::ranges::sort(loops_, [](const auto& a, const auto& b) { return a.size() > b.size(); });
}

std::vector<std::uint32_t> MeshTopology::components(std::uint32_t& count) const {
    std::vector<std::uint32_t> comp(neighbors_.size(), kNoTriangle);
    std::vector<std::uint32_t> stack;
    count = 0;
    for (std::uint32_t seed = 0; seed < neighbors_.size(); ++seed) {
        if (comp[seed] != kNoTriangle) continue;
        comp[seed] = count;
        stack.push_back(seed);
        while (!stack.empty()) {
            const std::uint32_t t = stack.back();
            stack.pop_back();
            for (const auto n : neighbors_[t])
                if (n != kNoTriangle && comp[n] == kNoTriangle) {
                    comp[n] = count;
                    stack.push_back(n);
                }
        }
        ++count;
    }
    return comp;
}

}  // namespace einstar::fit
