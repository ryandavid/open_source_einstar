#include "einstar/recon/mesh.hpp"
#include "poisson.hpp"

namespace einstar::recon {

Result<TriangleMesh> watertight_mesh(const TriangleMesh& surface, const WatertightParams& params) {
    if (surface.vertices.empty()) return make_error(Errc::invalid_argument, "no surface to close");
    TriangleMesh oriented;
    const TriangleMesh* src = &surface;
    if (surface.normals.size() != surface.vertices.size()) {
        oriented = surface;
        oriented.compute_normals();
        src = &oriented;
    }
    // Only vertices that are part of a triangle, with a normal.
    std::vector<std::uint8_t> used(src->vertices.size(), 0);
    for (const auto& t : src->triangles)
        for (const auto v : t) used[v] = 1;
    std::vector<float> points, normals;
    points.reserve(3 * src->vertices.size());
    normals.reserve(3 * src->vertices.size());
    for (std::size_t i = 0; i < src->vertices.size(); ++i) {
        if (!used[i] || src->normals[i].squaredNorm() < 0.5f) continue;
        points.insert(points.end(), {src->vertices[i].x(), src->vertices[i].y(), src->vertices[i].z()});
        normals.insert(normals.end(), {src->normals[i].x(), src->normals[i].y(), src->normals[i].z()});
    }
    detail::PoissonInput in;
    in.points = points.data();
    in.normals = normals.data();
    in.count = points.size() / 3;
    in.cell_mm = params.cell_mm > 0 ? params.cell_mm : 0.5;
    in.point_weight = params.point_weight;
    in.samples_per_node = params.samples_per_node;
    auto res = detail::screened_poisson(in);
    if (!res.error.empty()) return make_error(Errc::unsupported, "Poisson reconstruction failed: " + res.error);
    TriangleMesh out;
    out.vertices.resize(res.vertices.size() / 3);
    for (std::size_t i = 0; i < out.vertices.size(); ++i) out.vertices[i] = Vec3f(res.vertices[3 * i], res.vertices[3 * i + 1], res.vertices[3 * i + 2]);
    out.triangles.resize(res.triangles.size() / 3);
    for (std::size_t i = 0; i < out.triangles.size(); ++i) out.triangles[i] = {res.triangles[3 * i], res.triangles[3 * i + 1], res.triangles[3 * i + 2]};
    remove_small_components(out, params.cleanup);
    out.compute_normals();
    return out;
}

}  // namespace einstar::recon
