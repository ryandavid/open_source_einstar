#include <algorithm>
#include <cstdio>
#include <fstream>
#include <numeric>

#include "einstar/recon/mesh.hpp"

namespace einstar::recon {

double TriangleMesh::area() const {
    double a = 0;
    for (const auto& t : triangles)
        a += 0.5 * (vertices[t[1]] - vertices[t[0]]).cross(vertices[t[2]] - vertices[t[0]]).cast<double>().norm();
    return a;
}

void TriangleMesh::compute_normals() {
    normals.assign(vertices.size(), Vec3f::Zero());
    for (const auto& t : triangles) {
        const Vec3f n = (vertices[t[1]] - vertices[t[0]]).cross(vertices[t[2]] - vertices[t[0]]);  // area weighted
        for (const auto v : t) normals[v] += n;
    }
    for (auto& n : normals) {
        const float l = n.norm();
        n = l > 0 ? Vec3f(n / l) : Vec3f::Zero();
    }
}

namespace {

struct UnionFind {
    std::vector<std::uint32_t> parent;
    explicit UnionFind(std::size_t n) : parent(n) { std::iota(parent.begin(), parent.end(), 0u); }
    std::uint32_t find(std::uint32_t x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    }
    void unite(std::uint32_t a, std::uint32_t b) {
        a = find(a), b = find(b);
        if (a != b) parent[std::max(a, b)] = std::min(a, b);
    }
};

}  // namespace

CleanupReport remove_small_components(TriangleMesh& mesh, const CleanupParams& params) {
    CleanupReport rep;
    if (mesh.triangles.empty()) return rep;
    UnionFind uf(mesh.vertices.size());
    for (const auto& t : mesh.triangles) {
        uf.unite(t[0], t[1]);
        uf.unite(t[0], t[2]);
    }
    std::vector<std::size_t> tri_count(mesh.vertices.size(), 0);
    for (const auto& t : mesh.triangles) ++tri_count[uf.find(t[0])];
    std::size_t largest = 0;
    for (std::size_t v = 0; v < tri_count.size(); ++v)
        if (tri_count[v] > 0) {
            ++rep.components;
            largest = std::max(largest, tri_count[v]);
        }
    const auto keep_min = std::max(params.min_component_triangles,
                                   static_cast<std::size_t>(params.min_component_fraction * static_cast<double>(largest)));
    const auto before = mesh.triangles.size();
    for (std::size_t v = 0; v < tri_count.size(); ++v)
        if (tri_count[v] > 0 && tri_count[v] < keep_min) ++rep.removed_components;
    std::erase_if(mesh.triangles, [&](const auto& t) { return tri_count[uf.find(t[0])] < keep_min; });
    rep.removed_triangles = before - mesh.triangles.size();
    remove_unreferenced_vertices(mesh);
    return rep;
}

void remove_unreferenced_vertices(TriangleMesh& mesh) {
    std::vector<std::int64_t> remap(mesh.vertices.size(), -1);
    std::size_t n = 0;
    for (const auto& t : mesh.triangles)
        for (const auto v : t)
            if (remap[v] < 0) remap[v] = static_cast<std::int64_t>(n++);
    std::vector<Vec3f> vertices(n), normals(mesh.normals.empty() ? 0 : n);
    for (std::size_t v = 0; v < remap.size(); ++v)
        if (remap[v] >= 0) {
            vertices[static_cast<std::size_t>(remap[v])] = mesh.vertices[v];
            if (!mesh.normals.empty()) normals[static_cast<std::size_t>(remap[v])] = mesh.normals[v];
        }
    for (auto& t : mesh.triangles)
        for (auto& v : t) v = static_cast<std::uint32_t>(remap[v]);
    mesh.vertices = std::move(vertices);
    mesh.normals = std::move(normals);
}

void taubin_smooth(TriangleMesh& mesh, int iterations, float lambda, float mu) {
    // Uniform-weight umbrella operator over the edge graph.
    const std::size_t n = mesh.vertices.size();
    std::vector<std::vector<std::uint32_t>> nbr(n);
    for (const auto& t : mesh.triangles)
        for (int k = 0; k < 3; ++k) {
            nbr[t[static_cast<std::size_t>(k)]].push_back(t[static_cast<std::size_t>((k + 1) % 3)]);
            nbr[t[static_cast<std::size_t>((k + 1) % 3)]].push_back(t[static_cast<std::size_t>(k)]);
        }
    for (auto& v : nbr) {
        std::ranges::sort(v);
        v.erase(std::unique(v.begin(), v.end()), v.end());
    }
    std::vector<Vec3f> next(n);
    auto step = [&](float f) {
        for (std::size_t i = 0; i < n; ++i) {
            if (nbr[i].empty()) {
                next[i] = mesh.vertices[i];
                continue;
            }
            Vec3f avg = Vec3f::Zero();
            for (const auto j : nbr[i]) avg += mesh.vertices[j];
            avg /= static_cast<float>(nbr[i].size());
            next[i] = mesh.vertices[i] + f * (avg - mesh.vertices[i]);
        }
        mesh.vertices.swap(next);
    };
    for (int it = 0; it < iterations; ++it) {
        step(lambda);
        step(mu);
    }
    mesh.compute_normals();
}

std::optional<MeshFormat> format_from_extension(const std::filesystem::path& path) {
    auto ext = path.extension().string();
    std::ranges::transform(ext, ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (ext == ".stl") return MeshFormat::stl;
    if (ext == ".ply") return MeshFormat::ply;
    if (ext == ".obj") return MeshFormat::obj;
    return std::nullopt;
}

namespace {

template <typename T>
void put(std::ofstream& f, const T& v) {
    f.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

Result<void> save_stl(const TriangleMesh& m, const std::filesystem::path& path) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return make_error(Errc::io, "cannot write " + path.string());
    char header[80] = {};
    std::snprintf(header, sizeof header, "einstar scan, units mm");
    f.write(header, 80);
    put(f, static_cast<std::uint32_t>(m.triangles.size()));
    for (const auto& t : m.triangles) {
        const Vec3f& a = m.vertices[t[0]];
        const Vec3f& b = m.vertices[t[1]];
        const Vec3f& c = m.vertices[t[2]];
        Vec3f n = (b - a).cross(c - a);
        if (n.norm() > 0) n.normalize();
        for (const Vec3f& v : {n, a, b, c}) put(f, v.x()), put(f, v.y()), put(f, v.z());
        put(f, std::uint16_t{0});
    }
    return f ? Result<void>{} : make_error(Errc::io, "write failed: " + path.string());
}

Result<void> save_ply(const TriangleMesh& m, const std::filesystem::path& path) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return make_error(Errc::io, "cannot write " + path.string());
    const bool normals = m.normals.size() == m.vertices.size();
    f << "ply\nformat binary_little_endian 1.0\ncomment einstar scan, units mm\n";
    f << "element vertex " << m.vertices.size() << "\nproperty float x\nproperty float y\nproperty float z\n";
    if (normals) f << "property float nx\nproperty float ny\nproperty float nz\n";
    f << "element face " << m.triangles.size() << "\nproperty list uchar uint vertex_indices\nend_header\n";
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        put(f, m.vertices[i].x()), put(f, m.vertices[i].y()), put(f, m.vertices[i].z());
        if (normals) put(f, m.normals[i].x()), put(f, m.normals[i].y()), put(f, m.normals[i].z());
    }
    for (const auto& t : m.triangles) {
        put(f, std::uint8_t{3});
        put(f, t[0]), put(f, t[1]), put(f, t[2]);
    }
    return f ? Result<void>{} : make_error(Errc::io, "write failed: " + path.string());
}

Result<void> save_obj(const TriangleMesh& m, const std::filesystem::path& path) {
    std::ofstream f(path);
    if (!f) return make_error(Errc::io, "cannot write " + path.string());
    const bool normals = m.normals.size() == m.vertices.size();
    f << "# einstar scan, units mm\n";
    char buf[128];
    for (const auto& v : m.vertices) {
        const int n = std::snprintf(buf, sizeof buf, "v %.5f %.5f %.5f\n", v.x(), v.y(), v.z());
        f.write(buf, n);
    }
    if (normals)
        for (const auto& v : m.normals) {
            const int n = std::snprintf(buf, sizeof buf, "vn %.4f %.4f %.4f\n", v.x(), v.y(), v.z());
            f.write(buf, n);
        }
    for (const auto& t : m.triangles) {
        const int n = normals ? std::snprintf(buf, sizeof buf, "f %u//%u %u//%u %u//%u\n", t[0] + 1, t[0] + 1, t[1] + 1, t[1] + 1, t[2] + 1, t[2] + 1)
                              : std::snprintf(buf, sizeof buf, "f %u %u %u\n", t[0] + 1, t[1] + 1, t[2] + 1);
        f.write(buf, n);
    }
    return f ? Result<void>{} : make_error(Errc::io, "write failed: " + path.string());
}

}  // namespace

Result<void> save_mesh(const TriangleMesh& mesh, const std::filesystem::path& path) {
    const auto fmt = format_from_extension(path);
    if (!fmt) return make_error(Errc::invalid_argument, "unknown mesh format (use .stl, .ply or .obj): " + path.string());
    if (const auto dir = path.parent_path(); !dir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
    }
    switch (*fmt) {
        case MeshFormat::stl: return save_stl(mesh, path);
        case MeshFormat::ply: return save_ply(mesh, path);
        case MeshFormat::obj: return save_obj(mesh, path);
    }
    return make_error(Errc::invalid_argument, "unknown mesh format");
}

}  // namespace einstar::recon
