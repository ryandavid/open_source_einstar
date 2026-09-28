// Quadric error metric edge-collapse simplification (Garland & Heckbert 1997), written for the
// meshes the process step produces: millions of triangles, manifold except at scan borders.

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>

#include <tbb/parallel_for.h>

#include <Eigen/Dense>

#include "einstar/recon/mesh.hpp"

namespace einstar::recon {
namespace {

// Symmetric 4x4 quadric stored as its 10 unique entries; plus the area it has accumulated so the
// error can be reported as an area-averaged squared distance (mm^2).
struct Quadric {
    double a[10] = {};
    double area = 0;

    static Quadric plane(const Vec3& n, double d, double w) {
        Quadric q;
        const double v[4] = {n.x(), n.y(), n.z(), d};
        int k = 0;
        for (int i = 0; i < 4; ++i)
            for (int j = i; j < 4; ++j) q.a[k++] = w * v[i] * v[j];
        q.area = w;
        return q;
    }
    Quadric& operator+=(const Quadric& o) {
        for (int i = 0; i < 10; ++i) a[i] += o.a[i];
        area += o.area;
        return *this;
    }
    [[nodiscard]] double eval(const Vec3& p) const {
        const double x = p.x(), y = p.y(), z = p.z();
        return a[0] * x * x + 2 * a[1] * x * y + 2 * a[2] * x * z + 2 * a[3] * x + a[4] * y * y + 2 * a[5] * y * z + 2 * a[6] * y +
               a[7] * z * z + 2 * a[8] * z + a[9];
    }
    // Position minimising the quadric, if well conditioned.
    [[nodiscard]] bool optimum(Vec3& out) const {
        Mat3 A;
        A << a[0], a[1], a[2], a[1], a[4], a[5], a[2], a[5], a[7];
        const Vec3 b(-a[3], -a[6], -a[8]);
        const Eigen::LDLT<Mat3> ldlt(A);
        if (ldlt.info() != Eigen::Success || std::abs(A.determinant()) < 1e-12 * std::pow(A.trace() + 1e-30, 3)) return false;
        out = ldlt.solve(b);
        return out.allFinite();
    }
};

struct Candidate {
    double cost;
    std::uint32_t v0, v1;
    std::uint32_t stamp0, stamp1;
    Vec3 target;
    bool operator>(const Candidate& o) const { return cost > o.cost; }
};

// Simplifies one block; `locked` vertices (shared with other blocks) are neither removed nor moved.
// `source` (optional) receives, for each vertex of the result, its index in the input.
SimplifyReport simplify_block(TriangleMesh& mesh, const std::vector<bool>& locked, const SimplifyParams& params,
                              std::vector<std::uint32_t>* source = nullptr) {
    SimplifyReport rep;
    rep.triangles_before = mesh.triangles.size();
    const std::size_t nv = mesh.vertices.size(), nf = mesh.triangles.size();
    const std::size_t target = params.target_triangles > 0
                                   ? params.target_triangles
                                   : static_cast<std::size_t>(params.target_ratio * static_cast<double>(nf));
    if (nf == 0 || target >= nf) {
        rep.triangles_after = nf;
        if (source) {
            source->resize(nv);
            for (std::uint32_t i = 0; i < nv; ++i) (*source)[i] = i;
        }
        return rep;
    }
    std::vector<Vec3> pos(nv);
    for (std::size_t i = 0; i < nv; ++i) pos[i] = mesh.vertices[i].cast<double>();
    auto& tris = mesh.triangles;
    std::vector<bool> face_dead(nf, false), vert_dead(nv, false);
    std::vector<std::uint32_t> stamp(nv, 0);
    std::vector<std::vector<std::uint32_t>> vf(nv);  // vertex -> faces
    for (std::uint32_t f = 0; f < nf; ++f)
        for (const auto v : tris[f]) vf[v].push_back(f);

    auto face_normal = [&](const std::array<std::uint32_t, 3>& t) {
        return Vec3((pos[t[1]] - pos[t[0]]).cross(pos[t[2]] - pos[t[0]]));
    };

    // Quadrics: face planes (area weighted) plus heavy planes perpendicular to boundary edges.
    std::vector<Quadric> Q(nv);
    for (std::uint32_t f = 0; f < nf; ++f) {
        const Vec3 n2 = face_normal(tris[f]);
        const double area = 0.5 * n2.norm();
        if (area <= 0) continue;
        const Vec3 n = n2.normalized();
        const Quadric q = Quadric::plane(n, -n.dot(pos[tris[f][0]]), area);
        for (const auto v : tris[f]) Q[v] += q;
    }
    std::vector<bool> boundary(nv, false);
    std::vector<std::uint64_t> unique_edges;
    {
        std::vector<std::pair<std::uint64_t, std::uint32_t>> edges;  // (key, face)
        edges.reserve(nf * 3);
        for (std::uint32_t f = 0; f < nf; ++f)
            for (int e = 0; e < 3; ++e) {
                const auto a = tris[f][static_cast<std::size_t>(e)], b = tris[f][static_cast<std::size_t>((e + 1) % 3)];
                edges.emplace_back(static_cast<std::uint64_t>(std::min(a, b)) << 32 | std::max(a, b), f);
            }
        std::ranges::sort(edges);
        for (std::size_t i = 0; i < edges.size();) {
            std::size_t j = i;
            while (j < edges.size() && edges[j].first == edges[i].first) ++j;
            unique_edges.push_back(edges[i].first);
            if (j - i == 1) {  // boundary edge: keep the border in place
                const auto a = static_cast<std::uint32_t>(edges[i].first >> 32), b = static_cast<std::uint32_t>(edges[i].first & 0xFFFFFFFF);
                boundary[a] = boundary[b] = true;
                const Vec3 n = face_normal(tris[edges[i].second]).normalized();
                const Vec3 e = pos[b] - pos[a];
                const Vec3 m = e.cross(n);
                if (m.norm() > 1e-12) {
                    const Vec3 mn = m.normalized();
                    const Quadric q = Quadric::plane(mn, -mn.dot(pos[a]), params.boundary_weight * e.squaredNorm());
                    Q[a] += q;
                    Q[b] += q;
                }
            }
            i = j;
        }
    }

    const double max_err2 = params.max_error_mm * params.max_error_mm;
    auto make_candidate = [&](std::uint32_t v0, std::uint32_t v1) -> std::optional<Candidate> {
        Quadric q = Q[v0];
        q += Q[v1];
        Vec3 best;
        double cost;
        if (!q.optimum(best) || (best - 0.5 * (pos[v0] + pos[v1])).norm() > 2.0 * (pos[v0] - pos[v1]).norm()) {
            // Fall back to the better endpoint / midpoint.
            const Vec3 opts[3] = {pos[v0], pos[v1], 0.5 * (pos[v0] + pos[v1])};
            cost = 1e300;
            for (const auto& o : opts)
                if (const double c = q.eval(o); c < cost) cost = c, best = o;
        } else {
            cost = q.eval(best);
        }
        cost = std::max(0.0, cost) / std::max(q.area, 1e-12);  // area-averaged squared distance
        if (cost > max_err2) return std::nullopt;
        return Candidate{cost, v0, v1, stamp[v0], stamp[v1], best};
    };

    std::priority_queue<Candidate, std::vector<Candidate>, std::greater<>> heap;
    for (const auto key : unique_edges)
        if (auto c = make_candidate(static_cast<std::uint32_t>(key >> 32), static_cast<std::uint32_t>(key & 0xFFFFFFFF))) heap.push(*c);

    std::size_t alive = nf;
    std::vector<std::uint32_t> n0, n1;
    auto neighbours = [&](std::uint32_t v, std::vector<std::uint32_t>& out) {
        out.clear();
        for (const auto f : vf[v])
            if (!face_dead[f])
                for (const auto u : tris[f])
                    if (u != v) out.push_back(u);
        std::ranges::sort(out);
        out.erase(std::unique(out.begin(), out.end()), out.end());
    };
    while (alive > target && !heap.empty()) {
        const Candidate c = heap.top();
        heap.pop();
        if (vert_dead[c.v0] || vert_dead[c.v1] || stamp[c.v0] != c.stamp0 || stamp[c.v1] != c.stamp1) continue;
        if (!locked.empty() && (locked[c.v0] || locked[c.v1])) continue;
        const auto v0 = c.v0, v1 = c.v1;
        // Faces shared by the edge, and the link condition (keeps the surface manifold).
        int shared = 0;
        for (const auto f : vf[v0])
            if (!face_dead[f] && (tris[f][0] == v1 || tris[f][1] == v1 || tris[f][2] == v1)) ++shared;
        if (shared == 0) continue;
        neighbours(v0, n0);
        neighbours(v1, n1);
        std::vector<std::uint32_t> common;
        std::ranges::set_intersection(n0, n1, std::back_inserter(common));
        if (static_cast<int>(common.size()) != shared) continue;
        if (boundary[v0] && boundary[v1] && shared != 1) continue;  // would pinch two borders together
        // No face may flip or degenerate.
        bool ok = true;
        for (const auto v : {v0, v1}) {
            for (const auto f : vf[v]) {
                if (face_dead[f]) continue;
                auto t = tris[f];
                if ((t[0] == v0 || t[1] == v0 || t[2] == v0) && (t[0] == v1 || t[1] == v1 || t[2] == v1)) continue;
                const Vec3 before = face_normal(t);
                for (auto& x : t)
                    if (x == v) x = std::numeric_limits<std::uint32_t>::max();
                const Vec3* P[3];
                for (int k = 0; k < 3; ++k)
                    P[k] = t[static_cast<std::size_t>(k)] == std::numeric_limits<std::uint32_t>::max() ? &c.target : &pos[t[static_cast<std::size_t>(k)]];
                const Vec3 after = (*P[1] - *P[0]).cross(*P[2] - *P[0]);
                if (after.norm() < 1e-12 || after.dot(before) < params.min_normal_dot * after.norm() * before.norm()) {
                    ok = false;
                    break;
                }
            }
            if (!ok) break;
        }
        if (!ok) continue;
        // Collapse v1 into v0.
        for (const auto f : vf[v1]) {
            if (face_dead[f]) continue;
            auto& t = tris[f];
            if (t[0] == v0 || t[1] == v0 || t[2] == v0) {
                face_dead[f] = true;
                --alive;
                continue;
            }
            for (auto& x : t)
                if (x == v1) x = v0;
            vf[v0].push_back(f);
        }
        std::erase_if(vf[v0], [&](std::uint32_t f) { return face_dead[f]; });
        vf[v1].clear();
        vert_dead[v1] = true;
        pos[v0] = c.target;
        Q[v0] += Q[v1];
        boundary[v0] = boundary[v0] || boundary[v1];
        // Only edges at the merged vertex changed (their other endpoints' quadrics did not).
        ++stamp[v0];
        neighbours(v0, n0);
        for (const auto u : n0)
            if (auto cand = make_candidate(std::min(v0, u), std::max(v0, u))) heap.push(*cand);
        rep.max_error_mm = std::max(rep.max_error_mm, std::sqrt(c.cost));
    }

    // Compact.
    std::vector<std::array<std::uint32_t, 3>> out;
    out.reserve(alive);
    for (std::size_t f = 0; f < nf; ++f)
        if (!face_dead[f]) out.push_back(tris[f]);
    mesh.triangles = std::move(out);
    // Compact the vertices, remembering where each came from.
    std::vector<std::int64_t> remap(nv, -1);
    std::vector<Vec3f> verts;
    std::vector<std::uint32_t> src;
    for (auto& t : mesh.triangles)
        for (auto& v : t) {
            if (remap[v] < 0) {
                remap[v] = static_cast<std::int64_t>(verts.size());
                verts.push_back(pos[v].cast<float>());
                src.push_back(v);
            }
            v = static_cast<std::uint32_t>(remap[v]);
        }
    mesh.vertices = std::move(verts);
    mesh.normals.clear();
    if (source) *source = std::move(src);
    rep.triangles_after = mesh.triangles.size();
    return rep;
}

}  // namespace

SimplifyReport simplify(TriangleMesh& mesh, const SimplifyParams& params) {
    SimplifyReport rep;
    rep.triangles_before = mesh.triangles.size();
    if (mesh.triangles.size() < params.parallel_min_triangles) {
        auto r = simplify_block(mesh, {}, params);
        mesh.compute_normals();
        r.triangles_before = rep.triangles_before;
        return r;
    }
    // Large meshes: independent spatial blocks in parallel (vertices shared between blocks are
    // locked), then a second pass on a grid shifted by half a block to simplify the seams.
    const std::size_t total_target = params.target_triangles > 0
                                         ? params.target_triangles
                                         : static_cast<std::size_t>(params.target_ratio * static_cast<double>(rep.triangles_before));
    for (int pass = 0; pass < 2; ++pass) {
        // The second (seam) pass only works towards whatever is left of the overall target.
        SimplifyParams block_params = params;
        block_params.target_triangles = 0;
        block_params.target_ratio =
            std::min(1.0, static_cast<double>(total_target) / static_cast<double>(std::max<std::size_t>(1, mesh.triangles.size())));
        if (pass == 1 && block_params.target_ratio >= 1.0) break;
        const double cell = params.block_mm;
        const double shift = pass == 0 ? 0.0 : 0.5 * cell;
        std::vector<std::int64_t> block_of(mesh.triangles.size());
        std::unordered_map<std::int64_t, std::vector<std::uint32_t>> blocks;
        for (std::size_t f = 0; f < mesh.triangles.size(); ++f) {
            const auto& t = mesh.triangles[f];
            const Vec3f c = (mesh.vertices[t[0]] + mesh.vertices[t[1]] + mesh.vertices[t[2]]) / 3.0f;
            const auto key = [&](float v) { return static_cast<std::int64_t>(std::floor((v + shift) / cell)) & 0x1FFFFF; };
            block_of[f] = key(c.x()) << 42 | key(c.y()) << 21 | key(c.z());
            blocks[block_of[f]].push_back(static_cast<std::uint32_t>(f));
        }
        // A vertex used by triangles of two blocks is locked in both.
        std::vector<std::int64_t> owner(mesh.vertices.size(), -1);
        std::vector<bool> shared(mesh.vertices.size(), false);
        for (std::size_t f = 0; f < mesh.triangles.size(); ++f)
            for (const auto v : mesh.triangles[f]) {
                if (owner[v] == -1) owner[v] = block_of[f];
                else if (owner[v] != block_of[f]) shared[v] = true;
            }
        std::vector<std::pair<std::int64_t, std::vector<std::uint32_t>>> list(blocks.begin(), blocks.end());
        std::vector<TriangleMesh> results(list.size());
        std::vector<std::vector<std::uint32_t>> globals(list.size());  // local vertex -> global vertex
        std::vector<std::vector<std::uint32_t>> sources(list.size());  // result vertex -> local vertex
        std::vector<double> errors(list.size(), 0.0);
        tbb::parallel_for(std::size_t{0}, list.size(), [&](std::size_t b) {
            auto& local = results[b];
            auto& g = globals[b];
            std::unordered_map<std::uint32_t, std::uint32_t> to_local;
            std::vector<bool> locked;
            for (const auto f : list[b].second) {
                std::array<std::uint32_t, 3> t{};
                for (int k = 0; k < 3; ++k) {
                    const auto v = mesh.triangles[f][static_cast<std::size_t>(k)];
                    auto [it, inserted] = to_local.try_emplace(v, static_cast<std::uint32_t>(local.vertices.size()));
                    if (inserted) {
                        local.vertices.push_back(mesh.vertices[v]);
                        g.push_back(v);
                        locked.push_back(shared[v]);
                    }
                    t[static_cast<std::size_t>(k)] = it->second;
                }
                local.triangles.push_back(t);
            }
            errors[b] = simplify_block(local, locked, block_params, &sources[b]).max_error_mm;
        });
        // Reassemble: locked vertices keep their global identity; interior ones are appended.
        TriangleMesh merged;
        std::vector<std::int64_t> global_to_merged(mesh.vertices.size(), -1);
        for (std::size_t b = 0; b < list.size(); ++b) {
            const auto& local = results[b];
            std::vector<std::uint32_t> map(local.vertices.size());
            for (std::size_t v = 0; v < local.vertices.size(); ++v) {
                const auto gv = globals[b][sources[b][v]];
                if (shared[gv]) {
                    if (global_to_merged[gv] < 0) {
                        global_to_merged[gv] = static_cast<std::int64_t>(merged.vertices.size());
                        merged.vertices.push_back(mesh.vertices[gv]);
                    }
                    map[v] = static_cast<std::uint32_t>(global_to_merged[gv]);
                } else {
                    map[v] = static_cast<std::uint32_t>(merged.vertices.size());
                    merged.vertices.push_back(local.vertices[v]);
                }
            }
            for (const auto& t : local.triangles) merged.triangles.push_back({map[t[0]], map[t[1]], map[t[2]]});
            rep.max_error_mm = std::max(rep.max_error_mm, errors[b]);
        }
        mesh = std::move(merged);
    }
    mesh.compute_normals();
    rep.triangles_after = mesh.triangles.size();
    return rep;
}

}  // namespace einstar::recon
