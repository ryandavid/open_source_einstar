#include <algorithm>
#include <cmath>
#include <unordered_map>

#include <tbb/parallel_for.h>

#include "einstar/recon/mesh.hpp"

namespace einstar::recon {
namespace {

using track::BrickCoord;
using track::kBrickSize;
using track::kBrickVoxels;

inline int floor_div8(int a) { return a >= 0 ? a / 8 : -((-a + 7) / 8); }
inline int local_index(int x, int y, int z) { return (z * kBrickSize + y) * kBrickSize + x; }

// CPU copy of the volume with global voxel lookup.
struct Grid {
    std::vector<BrickCoord> coords;
    std::vector<std::array<float, kBrickVoxels>> sdf, weight;
    std::vector<std::array<std::uint8_t, kBrickVoxels>> observations;  // empty: not counted
    int min_observations = 0;
    std::unordered_map<BrickCoord, int, track::BrickCoordHash> index;

    [[nodiscard]] int brick(const BrickCoord& c) const {
        const auto it = index.find(c);
        return it == index.end() ? -1 : it->second;
    }
    // Voxel at global coordinate g; false if unobserved.
    [[nodiscard]] bool get(int gx, int gy, int gz, float min_weight, float& s) const {
        const BrickCoord c{floor_div8(gx), floor_div8(gy), floor_div8(gz)};
        const int b = brick(c);
        if (b < 0) return false;
        const int i = local_index(gx - c.x * kBrickSize, gy - c.y * kBrickSize, gz - c.z * kBrickSize);
        if (weight[static_cast<std::size_t>(b)][static_cast<std::size_t>(i)] < min_weight) return false;
        if (!observations.empty() && observations[static_cast<std::size_t>(b)][static_cast<std::size_t>(i)] < min_observations) return false;
        s = sdf[static_cast<std::size_t>(b)][static_cast<std::size_t>(i)];
        return true;
    }
};

// Cube corners (bit 0 = x, 1 = y, 2 = z) and edges as corner pairs.
constexpr std::array<std::array<int, 2>, 12> kEdges{{{0, 1}, {2, 3}, {4, 5}, {6, 7},  // along x
                                                      {0, 2}, {1, 3}, {4, 6}, {5, 7},  // along y
                                                      {0, 4}, {1, 5}, {2, 6}, {3, 7}}};  // along z

Vec3f corner_offset(int c) { return {static_cast<float>(c & 1), static_cast<float>((c >> 1) & 1), static_cast<float>((c >> 2) & 1)}; }

// Trilinear value and gradient inside the unit cell.
float trilinear(const std::array<float, 8>& s, const Vec3f& u, Vec3f& grad) {
    const float x = u.x(), y = u.y(), z = u.z();
    const float c00 = s[0] * (1 - x) + s[1] * x, c10 = s[2] * (1 - x) + s[3] * x;
    const float c01 = s[4] * (1 - x) + s[5] * x, c11 = s[6] * (1 - x) + s[7] * x;
    const float c0 = c00 * (1 - y) + c10 * y, c1 = c01 * (1 - y) + c11 * y;
    const float dx0 = (s[1] - s[0]) * (1 - y) + (s[3] - s[2]) * y, dx1 = (s[5] - s[4]) * (1 - y) + (s[7] - s[6]) * y;
    grad.x() = dx0 * (1 - z) + dx1 * z;
    grad.y() = (c10 - c00) * (1 - z) + (c11 - c01) * z;
    grad.z() = c1 - c0;
    return c0 * (1 - z) + c1 * z;
}

}  // namespace

TriangleMesh extract_mesh(const track::Volume& volume, const ExtractParams& params) {
    Grid g;
    const bool use_obs = params.min_observations > 0;
    g.min_observations = params.min_observations;
    volume.for_each_brick([&](const BrickCoord& c, std::span<const float> sdf, std::span<const float> weight,
                              std::span<const std::uint8_t> observations) {
        // Skip bricks that were allocated but never observed.
        if (std::ranges::none_of(weight, [&](float w) { return w >= params.min_weight; })) return;
        g.index[c] = static_cast<int>(g.coords.size());
        g.coords.push_back(c);
        auto& s = g.sdf.emplace_back();
        auto& w = g.weight.emplace_back();
        std::ranges::copy(sdf, s.begin());
        std::ranges::copy(weight, w.begin());
        if (use_obs && observations.size() == kBrickVoxels) std::ranges::copy(observations, g.observations.emplace_back().begin());
    });
    if (g.observations.size() != g.coords.size()) g.observations.clear();  // not counted by this volume: no filter
    const float voxel = volume.params().voxel_mm;
    const float mw = params.min_weight;
    const std::size_t nb = g.coords.size();

    // Pass 1: one vertex per surface cell (cell origin = voxel g, corners g + {0,1}^3).
    std::vector<std::array<std::int32_t, kBrickVoxels>> cell_vertex(nb);
    std::vector<std::vector<Vec3f>> local_pos(nb), local_nrm(nb);
    tbb::parallel_for(std::size_t{0}, nb, [&](std::size_t bi) {
        cell_vertex[bi].fill(-1);
        const BrickCoord c = g.coords[bi];
        for (int z = 0; z < kBrickSize; ++z)
            for (int y = 0; y < kBrickSize; ++y)
                for (int x = 0; x < kBrickSize; ++x) {
                    const int gx = c.x * kBrickSize + x, gy = c.y * kBrickSize + y, gz = c.z * kBrickSize + z;
                    std::array<float, 8> s{};
                    bool ok = true;
                    int positive = 0;
                    for (int k = 0; k < 8 && ok; ++k) {
                        ok = g.get(gx + (k & 1), gy + ((k >> 1) & 1), gz + ((k >> 2) & 1), mw, s[static_cast<std::size_t>(k)]);
                        positive += s[static_cast<std::size_t>(k)] > 0;
                    }
                    if (!ok || positive == 0 || positive == 8) continue;
                    Vec3f sum = Vec3f::Zero();
                    int n = 0;
                    for (const auto& e : kEdges) {
                        const float s0 = s[static_cast<std::size_t>(e[0])], s1 = s[static_cast<std::size_t>(e[1])];
                        if ((s0 > 0) == (s1 > 0) || std::abs(s0 - s1) > params.max_crossing_jump) continue;
                        const float t = s0 / (s0 - s1);
                        sum += corner_offset(e[0]) + t * (corner_offset(e[1]) - corner_offset(e[0]));
                        ++n;
                    }
                    if (n == 0) continue;
                    Vec3f u = sum / static_cast<float>(n);
                    Vec3f grad;
                    const float v = trilinear(s, u, grad);
                    const float g2 = grad.squaredNorm();
                    if (g2 < 1e-12f) continue;
                    // One Newton step onto the trilinear zero level, kept inside the cell.
                    u = (u - v * grad / g2).cwiseMax(0.0f).cwiseMin(1.0f);
                    trilinear(s, u, grad);
                    const Vec3f pos = (Vec3f(static_cast<float>(gx), static_cast<float>(gy), static_cast<float>(gz)) + Vec3f::Constant(0.5f) + u) * voxel;
                    cell_vertex[bi][static_cast<std::size_t>(local_index(x, y, z))] = static_cast<std::int32_t>(local_pos[bi].size());
                    local_pos[bi].push_back(pos);
                    local_nrm[bi].push_back(grad.normalized());
                }
    });
    std::vector<std::uint32_t> offset(nb + 1, 0);
    for (std::size_t b = 0; b < nb; ++b) offset[b + 1] = offset[b] + static_cast<std::uint32_t>(local_pos[b].size());
    TriangleMesh mesh;
    mesh.vertices.resize(offset[nb]);
    mesh.normals.resize(offset[nb]);
    tbb::parallel_for(std::size_t{0}, nb, [&](std::size_t b) {
        std::ranges::copy(local_pos[b], mesh.vertices.begin() + offset[b]);
        std::ranges::copy(local_nrm[b], mesh.normals.begin() + offset[b]);
    });
    auto vertex_of = [&](int gx, int gy, int gz) -> std::int64_t {
        const BrickCoord c{floor_div8(gx), floor_div8(gy), floor_div8(gz)};
        const int b = g.brick(c);
        if (b < 0) return -1;
        const auto v = cell_vertex[static_cast<std::size_t>(b)][static_cast<std::size_t>(
            local_index(gx - c.x * kBrickSize, gy - c.y * kBrickSize, gz - c.z * kBrickSize))];
        return v < 0 ? -1 : static_cast<std::int64_t>(offset[static_cast<std::size_t>(b)]) + v;
    };

    // Pass 2: a quad around every voxel edge that crosses the surface.
    std::vector<std::vector<std::array<std::uint32_t, 3>>> local_tris(nb);
    tbb::parallel_for(std::size_t{0}, nb, [&](std::size_t bi) {
        const BrickCoord c = g.coords[bi];
        auto& out = local_tris[bi];
        for (int z = 0; z < kBrickSize; ++z)
            for (int y = 0; y < kBrickSize; ++y)
                for (int x = 0; x < kBrickSize; ++x) {
                    const int gx = c.x * kBrickSize + x, gy = c.y * kBrickSize + y, gz = c.z * kBrickSize + z;
                    float s0;
                    if (!g.get(gx, gy, gz, mw, s0)) continue;
                    for (int axis = 0; axis < 3; ++axis) {
                        const int ax = axis == 0, ay = axis == 1, az = axis == 2;
                        float s1;
                        if (!g.get(gx + ax, gy + ay, gz + az, mw, s1)) continue;
                        if ((s0 > 0) == (s1 > 0) || std::abs(s0 - s1) > params.max_crossing_jump) continue;
                        // The four cells sharing this edge: offsets 0 / -1 along the two other axes.
                        const int u_axis = (axis + 1) % 3, w_axis = (axis + 2) % 3;
                        auto cell = [&](int du, int dw) {
                            int o[3] = {gx, gy, gz};
                            o[u_axis] -= du;
                            o[w_axis] -= dw;
                            return vertex_of(o[0], o[1], o[2]);
                        };
                        const std::int64_t q[4] = {cell(0, 0), cell(1, 0), cell(1, 1), cell(0, 1)};
                        if (q[0] < 0 || q[1] < 0 || q[2] < 0 || q[3] < 0) continue;
                        std::array<std::uint32_t, 4> v{static_cast<std::uint32_t>(q[0]), static_cast<std::uint32_t>(q[1]),
                                                       static_cast<std::uint32_t>(q[2]), static_cast<std::uint32_t>(q[3])};
                        // Winding: the face normal must agree with the SDF gradient (into free space).
                        const Vec3f& p0 = mesh.vertices[v[0]];
                        const Vec3f fn = (mesh.vertices[v[1]] - p0).cross(mesh.vertices[v[2]] - p0) +
                                         (mesh.vertices[v[2]] - p0).cross(mesh.vertices[v[3]] - p0);
                        const Vec3f vn = mesh.normals[v[0]] + mesh.normals[v[1]] + mesh.normals[v[2]] + mesh.normals[v[3]];
                        if (fn.dot(vn) < 0) std::swap(v[1], v[3]);
                        // Split along the diagonal whose triangles agree best with the surface normals
                        // (the shorter diagonal can fold where the surface is tangent to a grid plane).
                        auto agreement = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
                            const Vec3f n = (mesh.vertices[b] - mesh.vertices[a]).cross(mesh.vertices[c] - mesh.vertices[a]);
                            const float l = n.norm();
                            if (l < 1e-12f) return -1.0f;
                            return (n / l).dot((mesh.normals[a] + mesh.normals[b] + mesh.normals[c]).normalized());
                        };
                        const float d02 = std::min(agreement(v[0], v[1], v[2]), agreement(v[0], v[2], v[3]));
                        const float d13 = std::min(agreement(v[0], v[1], v[3]), agreement(v[1], v[2], v[3]));
                        if (d02 >= d13) {
                            out.push_back({v[0], v[1], v[2]});
                            out.push_back({v[0], v[2], v[3]});
                        } else {
                            out.push_back({v[0], v[1], v[3]});
                            out.push_back({v[1], v[2], v[3]});
                        }
                    }
                }
    });
    std::size_t total = 0;
    for (const auto& t : local_tris) total += t.size();
    mesh.triangles.reserve(total);
    for (const auto& t : local_tris) mesh.triangles.insert(mesh.triangles.end(), t.begin(), t.end());
    return mesh;
}

}  // namespace einstar::recon
