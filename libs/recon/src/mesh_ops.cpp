#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Eigenvalues>

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
    const std::size_t nt = mesh.triangles.size();
    // Pieces: triangles joined across edges with exactly two triangles.
    UnionFind uf(nt);
    {
        std::vector<std::pair<std::uint64_t, std::uint32_t>> edges;  // (edge key, triangle)
        edges.reserve(3 * nt);
        for (std::uint32_t f = 0; f < nt; ++f)
            for (int e = 0; e < 3; ++e) {
                const auto a = mesh.triangles[f][static_cast<std::size_t>(e)], b = mesh.triangles[f][static_cast<std::size_t>((e + 1) % 3)];
                edges.emplace_back(static_cast<std::uint64_t>(std::min(a, b)) << 32 | std::max(a, b), f);
            }
        std::ranges::sort(edges);
        for (std::size_t i = 0; i < edges.size();) {
            std::size_t j = i;
            while (j < edges.size() && edges[j].first == edges[i].first) ++j;
            if (j - i == 2) uf.unite(edges[i].second, edges[i + 1].second);
            i = j;
        }
    }
    std::vector<double> area(nt, 0.0);
    std::vector<std::size_t> count(nt, 0);
    for (std::uint32_t f = 0; f < nt; ++f) {
        const auto& t = mesh.triangles[f];
        const auto r = uf.find(f);
        area[r] += 0.5 * (mesh.vertices[t[1]] - mesh.vertices[t[0]]).cross(mesh.vertices[t[2]] - mesh.vertices[t[0]]).cast<double>().norm();
        ++count[r];
    }
    std::size_t largest = 0;
    double total = 0;
    for (std::size_t r = 0; r < nt; ++r)
        if (count[r] > 0) {
            ++rep.components;
            largest = std::max(largest, count[r]);
            total += area[r];
        }
    const auto keep_min = std::max(params.min_component_triangles,
                                   static_cast<std::size_t>(params.min_component_fraction * static_cast<double>(largest)));
    std::vector<std::uint8_t> removed(nt, 0);  // per root
    std::vector<std::uint32_t> candidates;     // roots of small kept pieces (isolation rule)
    for (std::uint32_t r = 0; r < nt; ++r) {
        if (count[r] == 0) continue;
        if (count[r] < keep_min || area[r] < params.min_component_area_mm2) removed[r] = 1;
        else if (area[r] < params.isolated_component_fraction * total) candidates.push_back(r);
    }
    if (!candidates.empty() && params.isolation_mm > 0) {
        // Occupancy of the kept pieces on a grid of a third of the isolation distance: a piece is
        // isolated when no cell within three cells of its own holds a larger kept piece (anything
        // within isolation_mm is found; up to ~2.3x that may count as near).
        const double cell = params.isolation_mm / 3.0;
        auto key = [](std::int64_t x, std::int64_t y, std::int64_t z) { return (x + (1 << 20)) << 42 | (y + (1 << 20)) << 21 | (z + (1 << 20)); };
        auto cell_key = [&](const Vec3f& p) {
            return key(static_cast<std::int64_t>(std::floor(p.x() / cell)), static_cast<std::int64_t>(std::floor(p.y() / cell)),
                       static_cast<std::int64_t>(std::floor(p.z() / cell)));
        };
        std::vector<std::vector<std::uint32_t>> tris_of(nt);
        for (std::uint32_t f = 0; f < nt; ++f) {
            const auto r = uf.find(f);
            if (!removed[r] && area[r] < params.isolated_component_fraction * total) tris_of[r].push_back(f);
        }
        std::unordered_set<std::int64_t> occupied;
        for (std::uint32_t f = 0; f < nt; ++f) {
            const auto r = uf.find(f);
            if (!removed[r] && tris_of[r].empty())
                for (const auto v : mesh.triangles[f]) occupied.insert(cell_key(mesh.vertices[v]));
        }
        std::ranges::sort(candidates, [&](auto a, auto b) { return area[a] > area[b]; });
        constexpr std::int64_t kMask = (1 << 21) - 1;
        for (const auto r : candidates) {
            std::unordered_set<std::int64_t> own;
            for (const auto f : tris_of[r])
                for (const auto v : mesh.triangles[f]) own.insert(cell_key(mesh.vertices[v]));
            bool near = false;
            for (auto it = own.begin(); it != own.end() && !near; ++it) {
                const std::int64_t x = (*it >> 42) - (1 << 20), y = ((*it >> 21) & kMask) - (1 << 20), z = (*it & kMask) - (1 << 20);
                for (int dz = -3; dz <= 3 && !near; ++dz)
                    for (int dy = -3; dy <= 3 && !near; ++dy)
                        for (int dx = -3; dx <= 3 && !near; ++dx) near = occupied.contains(key(x + dx, y + dy, z + dz));
            }
            if (near) {
                occupied.insert(own.begin(), own.end());
            } else {
                removed[r] = 1;
                ++rep.removed_isolated;
            }
        }
    }
    for (std::size_t r = 0; r < nt; ++r)
        if (count[r] > 0 && removed[r]) {
            ++rep.removed_components;
            rep.removed_area_mm2 += area[r];
        }
    std::vector<std::uint8_t> drop(nt);
    for (std::uint32_t f = 0; f < nt; ++f) drop[f] = removed[uf.find(f)];
    std::size_t f = 0;
    std::erase_if(mesh.triangles, [&](const auto&) { return drop[f++] != 0; });
    rep.removed_triangles = nt - mesh.triangles.size();
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

std::size_t flatten_markers(TriangleMesh& mesh, const std::vector<MarkerDisc>& markers, const MarkerFlattenParams& p) {
    if (markers.empty() || mesh.vertices.empty()) return 0;
    if (mesh.normals.size() != mesh.vertices.size()) mesh.compute_normals();
    // Vertices bucketed on a grid of the largest neighbourhood.
    float reach = 0;
    for (const auto& m : markers) reach = std::max(reach, p.cover_radii * m.radius + p.blend_mm + p.ring_mm);
    if (reach <= 0) return 0;
    const float cell = reach;
    auto key = [](std::int64_t x, std::int64_t y, std::int64_t z) { return (x + (1 << 20)) << 42 | (y + (1 << 20)) << 21 | (z + (1 << 20)); };
    auto coord = [&](float v) { return static_cast<std::int64_t>(std::floor(v / cell)); };
    std::unordered_map<std::int64_t, std::vector<std::uint32_t>> grid;
    for (std::uint32_t i = 0; i < mesh.vertices.size(); ++i) {
        const Vec3f& v = mesh.vertices[i];
        grid[key(coord(v.x()), coord(v.y()), coord(v.z()))].push_back(i);
    }
    std::size_t done = 0;
    for (const auto& mk : markers) {
        const Vec3f n = mk.normal.normalized();
        const float inner = p.cover_radii * mk.radius, outer = inner + p.blend_mm, ring = outer + p.ring_mm;
        // Vertices of the marker's surface around it (same side, near its plane).
        std::vector<std::uint32_t> near;
        for (std::int64_t dz = -1; dz <= 1; ++dz)
            for (std::int64_t dy = -1; dy <= 1; ++dy)
                for (std::int64_t dx = -1; dx <= 1; ++dx) {
                    const auto it = grid.find(key(coord(mk.center.x()) + dx, coord(mk.center.y()) + dy, coord(mk.center.z()) + dz));
                    if (it == grid.end()) continue;
                    for (const auto i : it->second) {
                        const Vec3f d = mesh.vertices[i] - mk.center;
                        const float h = d.dot(n);
                        if (std::abs(h) > p.max_height_mm || (d - h * n).norm() > ring || mesh.normals[i].dot(n) < 0.5f) continue;
                        near.push_back(i);
                    }
                }
        // The annulus: its plane, then a quadric height field over it.
        Vec3 c = Vec3::Zero();
        std::size_t na = 0;
        std::array<int, 8> sectors{};
        const Vec3f e0 = n.unitOrthogonal(), e1 = n.cross(e0);
        auto radial = [&](std::uint32_t i) {
            const Vec3f d = mesh.vertices[i] - mk.center;
            return (d - d.dot(n) * n).norm();
        };
        for (const auto i : near)
            if (const float r = radial(i); r >= outer) {
                c += mesh.vertices[i].cast<double>();
                ++na;
                const Vec3f d = mesh.vertices[i] - mk.center;
                const double a = std::atan2(d.dot(e1), d.dot(e0));
                ++sectors[static_cast<std::size_t>(std::clamp(static_cast<int>((a + M_PI) / (2 * M_PI) * 8), 0, 7))];
            }
        if (na < 30 || std::ranges::count(sectors, 0) > 1) continue;  // the surround must be there on (nearly) all sides
        c /= static_cast<double>(na);
        Mat3 cov = Mat3::Zero();
        for (const auto i : near)
            if (radial(i) >= outer) {
                const Vec3 d = mesh.vertices[i].cast<double>() - c;
                cov += d * d.transpose();
            }
        const Eigen::SelfAdjointEigenSolver<Mat3> es(cov);
        Vec3 pn = es.eigenvectors().col(0);
        if (pn.dot(n.cast<double>()) < 0) pn = -pn;
        if (pn.dot(n.cast<double>()) < 0.8) continue;  // the marker does not lie on that surface
        const Vec3 u = es.eigenvectors().col(2), w = pn.cross(u);
        auto basis = [&](const Vec3& q) {
            const double x = (q - c).dot(u), y = (q - c).dot(w);
            return Eigen::Matrix<double, 6, 1>(1, x, y, x * x, x * y, y * y);
        };
        Eigen::Matrix<double, 6, 6> AtA = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> Atb = Eigen::Matrix<double, 6, 1>::Zero();
        for (const auto i : near)
            if (radial(i) >= outer) {
                const Vec3 q = mesh.vertices[i].cast<double>();
                const auto b = basis(q);
                AtA += b * b.transpose();
                Atb += b * (q - c).dot(pn);
            }
        const Eigen::Matrix<double, 6, 1> co = AtA.ldlt().solve(Atb);
        if (!co.allFinite()) continue;
        double ss = 0;
        for (const auto i : near)
            if (radial(i) >= outer) {
                const Vec3 q = mesh.vertices[i].cast<double>();
                const double e = (q - c).dot(pn) - basis(q).dot(co);
                ss += e * e;
            }
        if (std::sqrt(ss / static_cast<double>(na)) > p.max_ring_rms_mm) continue;
        // Inside: onto the fitted surface (along its normal), blended out to `outer`.
        for (const auto i : near) {
            const float r = radial(i);
            if (r >= outer) continue;
            const double t = r <= inner ? 1.0 : 1.0 - (r - inner) / p.blend_mm;
            const Vec3 q = mesh.vertices[i].cast<double>();
            const double x = (q - c).dot(u), y = (q - c).dot(w);
            const double target = co[0] + co[1] * x + co[2] * y + co[3] * x * x + co[4] * x * y + co[5] * y * y;
            mesh.vertices[i] = (q + t * (target - (q - c).dot(pn)) * pn).cast<float>();
            const Vec3 nq = (pn - (co[1] + 2 * co[3] * x + co[4] * y) * u - (co[2] + co[4] * x + 2 * co[5] * y) * w).normalized();
            mesh.normals[i] = ((1 - t) * mesh.normals[i].cast<double>() + t * nq).normalized().cast<float>();
        }
        ++done;
    }
    return done;
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
