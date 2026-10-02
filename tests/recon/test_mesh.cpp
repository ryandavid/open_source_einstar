#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <print>

#include "einstar/core/timing.hpp"
#include "einstar/fixtures/exstar_project.hpp"
#include "einstar/recon/mesh.hpp"
#include "einstar/synth/demo.hpp"
#include "einstar/track/tsdf.hpp"

using namespace einstar;

namespace {

// Analytic depth image of a sphere seen from a camera pose.
ImageF32 sphere_depth(const Vec3& center, double radius, const SE3& T_wc, const track::Intrinsics& k) {
    ImageF32 d(k.width, k.height, 0.0f);
    const SE3 T_cw = T_wc.inverse();
    const Vec3 c = T_cw * center;
    for (int v = 0; v < k.height; ++v)
        for (int u = 0; u < k.width; ++u) {
            const Vec3 dir = Vec3((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0).normalized();
            const double b = dir.dot(c), disc = b * b - (c.squaredNorm() - radius * radius);
            if (disc < 0) continue;
            const double t = b - std::sqrt(disc);
            if (t > 0) d(u, v) = static_cast<float>(t * dir.z());
        }
    return d;
}

}  // namespace

TEST_CASE("surface nets on a fused sphere: accurate, (nearly) closed, outward normals, exports") {
    track::TsdfParams tp;
    tp.voxel_mm = 0.5f;
    track::TsdfVolume vol(tp);
    const track::Intrinsics k{320, 256, 290.0, 290.0, 160.0, 128.0};
    const Vec3 center(0, 0, 0);
    const double radius = 30.0;
    // Views from all around so the surface is closed.
    for (int i = 0; i < 6; ++i)
        for (int j = -1; j <= 1; ++j) {
            const double a = i * M_PI / 3, e = j * 0.9;
            const Vec3 eye = 250.0 * Vec3(std::cos(a) * std::cos(e), std::sin(e), std::sin(a) * std::cos(e));
            const SE3 T = synth::look_at(eye, center);
            vol.integrate(track::make_depth_frame(sphere_depth(center, radius, T, k), k), T);
        }
    vol.integrate(track::make_depth_frame(sphere_depth(center, radius, synth::look_at(Vec3(0, 250, 1), center), k), k), synth::look_at(Vec3(0, 250, 1), center));
    vol.integrate(track::make_depth_frame(sphere_depth(center, radius, synth::look_at(Vec3(0, -250, 1), center), k), k), synth::look_at(Vec3(0, -250, 1), center));

    Stopwatch sw;
    auto mesh = recon::extract_mesh(vol);
    const double ms = sw.elapsed_ms();
    REQUIRE(!mesh.empty());
    double max_err = 0, sum_err = 0;
    for (const auto& v : mesh.vertices) {
        const double e = std::abs(v.cast<double>().norm() - radius);
        max_err = std::max(max_err, e);
        sum_err += e;
    }
    int outward = 0;
    for (const auto& t : mesh.triangles) {
        const Vec3f& a = mesh.vertices[t[0]];
        const Vec3f n = (mesh.vertices[t[1]] - a).cross(mesh.vertices[t[2]] - a);
        outward += n.dot(a) > 0;
    }
    // Closed surface: every edge is shared by exactly two triangles.
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> edges;
    for (const auto& t : mesh.triangles)
        for (int e = 0; e < 3; ++e) {
            auto a = t[static_cast<std::size_t>(e)], b = t[static_cast<std::size_t>((e + 1) % 3)];
            ++edges[{std::min(a, b), std::max(a, b)}];
        }
    const auto boundary = std::ranges::count_if(edges, [](const auto& kv) { return kv.second != 2; });
    const double area = mesh.area(), true_area = 4 * M_PI * radius * radius;
    std::println("sphere: {} vertices, {} triangles in {:.1f} ms; radial error mean {:.4f} max {:.4f} mm; area {:.1f} vs {:.1f}; "
                 "{} non-manifold/boundary edges; {} of {} faces outward",
                 mesh.vertices.size(), mesh.triangles.size(), ms, sum_err / static_cast<double>(mesh.vertices.size()), max_err, area,
                 true_area, boundary, outward, mesh.triangles.size());
    CHECK(sum_err / static_cast<double>(mesh.vertices.size()) < 0.05);
    CHECK(max_err < 0.25);  // 0.5 mm voxels
    CHECK(std::abs(area - true_area) / true_area < 0.02);
    // Closed apart from rare surface-nets saddle configurations where the sphere is tangent to a
    // grid plane (a handful of edges shared by four faces).
    CHECK(static_cast<double>(boundary) < 2e-4 * static_cast<double>(edges.size()));
    CHECK(outward >= static_cast<int>(0.9995 * static_cast<double>(mesh.triangles.size())));

    const auto rep = recon::remove_small_components(mesh);
    CHECK(rep.components == 1);
    CHECK(rep.removed_triangles == 0);

    const auto dir = std::filesystem::temp_directory_path() / "einstar_mesh_test";
    for (const char* name : {"sphere.stl", "sphere.ply", "sphere.obj"}) REQUIRE(recon::save_mesh(mesh, dir / name).has_value());
    auto stl = fixtures::load_stl(dir / "sphere.stl");
    REQUIRE(stl.has_value());
    CHECK(stl->vertices.size() == 3 * mesh.triangles.size());
    CHECK(std::filesystem::file_size(dir / "sphere.ply") > 0);
    std::filesystem::remove_all(dir);
}

TEST_CASE("small disconnected pieces are removed") {
    recon::TriangleMesh m;
    auto add_quad = [&](Vec3f o, float s) {
        const auto b = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.insert(m.vertices.end(), {o, o + Vec3f(s, 0, 0), o + Vec3f(s, s, 0), o + Vec3f(0, s, 0)});
        m.triangles.push_back({b, b + 1, b + 2});
        m.triangles.push_back({b, b + 2, b + 3});
    };
    // A large connected grid and two isolated specks.
    const int n = 31;
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) m.vertices.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.0f);
    for (int y = 0; y + 1 < n; ++y)
        for (int x = 0; x + 1 < n; ++x) {
            const auto i = static_cast<std::uint32_t>(y * n + x);
            m.triangles.push_back({i, i + 1, i + static_cast<std::uint32_t>(n) + 1});
            m.triangles.push_back({i, i + static_cast<std::uint32_t>(n) + 1, i + static_cast<std::uint32_t>(n)});
        }
    add_quad(Vec3f(100, 0, 0), 1);
    add_quad(Vec3f(200, 0, 0), 1);
    const auto rep = recon::remove_small_components(m);
    CHECK(rep.components == 3);
    CHECK(rep.removed_components == 2);
    CHECK(m.triangles.size() == 2u * 30u * 30u);
    CHECK(m.vertices.size() == static_cast<std::size_t>(n * n));
}

TEST_CASE("simplification keeps the shape, manifoldness and orientation") {
    track::TsdfParams tp;
    tp.voxel_mm = 0.5f;
    track::TsdfVolume vol(tp);
    const track::Intrinsics k{320, 256, 290.0, 290.0, 160.0, 128.0};
    const double radius = 30.0;
    for (int i = 0; i < 6; ++i)
        for (int j = -1; j <= 1; ++j) {
            const double a = i * M_PI / 3, e = j * 0.9;
            const SE3 T = synth::look_at(250.0 * Vec3(std::cos(a) * std::cos(e), std::sin(e), std::sin(a) * std::cos(e)), Vec3::Zero());
            vol.integrate(track::make_depth_frame(sphere_depth(Vec3::Zero(), radius, T, k), k), T);
        }
    for (const double y : {250.0, -250.0}) {
        const SE3 T = synth::look_at(Vec3(0, y, 1), Vec3::Zero());
        vol.integrate(track::make_depth_frame(sphere_depth(Vec3::Zero(), radius, T, k), k), T);
    }
    auto mesh = recon::extract_mesh(vol);
    auto count_bad_edges = [](const recon::TriangleMesh& m) {
        std::map<std::pair<std::uint32_t, std::uint32_t>, int> edges;
        for (const auto& t : m.triangles)
            for (int e = 0; e < 3; ++e) {
                auto a = t[static_cast<std::size_t>(e)], b = t[static_cast<std::size_t>((e + 1) % 3)];
                ++edges[{std::min(a, b), std::max(a, b)}];
            }
        return std::ranges::count_if(edges, [](const auto& kv) { return kv.second != 2; });
    };
    const auto bad_before = count_bad_edges(mesh);
    recon::SimplifyParams sp;
    sp.target_ratio = 0.1;
    sp.max_error_mm = 0.05;
    SECTION("single block") {}
    SECTION("parallel blocks") {
        sp.parallel_min_triangles = 0;  // force the block path with small blocks: many seams
        sp.block_mm = 15.0;
    }
    Stopwatch sw;
    const auto rep = recon::simplify(mesh, sp);
    const double ms = sw.elapsed_ms();
    double max_err = 0, sum = 0;
    for (const auto& v : mesh.vertices) {
        const double e = std::abs(v.cast<double>().norm() - radius);
        max_err = std::max(max_err, e);
        sum += e;
    }
    int outward = 0;
    for (const auto& t : mesh.triangles) outward += (mesh.vertices[t[1]] - mesh.vertices[t[0]]).cross(mesh.vertices[t[2]] - mesh.vertices[t[0]]).dot(mesh.vertices[t[0]]) > 0;
    const auto bad_after = count_bad_edges(mesh);
    std::println("simplify: {} -> {} triangles in {:.0f} ms (reported error {:.3f} mm); radial error mean {:.4f} max {:.4f} mm; "
                 "non-manifold/boundary edges {} -> {}; {} of {} faces outward",
                 rep.triangles_before, rep.triangles_after, ms, rep.max_error_mm, sum / static_cast<double>(mesh.vertices.size()), max_err,
                 bad_before, bad_after, outward, mesh.triangles.size());
    CHECK(rep.triangles_after <= rep.triangles_before / 5);
    CHECK(max_err < 0.3);
    CHECK(bad_after <= bad_before);
    CHECK(outward >= static_cast<int>(0.999 * static_cast<double>(mesh.triangles.size())));
}

TEST_CASE("surface seen by too few frames is dropped when observations are required") {
    track::TsdfParams tp;
    tp.voxel_mm = 0.5f;
    tp.count_observations = true;
    track::TsdfVolume vol(tp);
    const track::Intrinsics k{320, 256, 290.0, 290.0, 160.0, 128.0};
    const Vec3 a(0, 0, 0), b(60, 0, 0);  // a: seen by four frames; b: a stray blob in two
    for (int i = 0; i < 4; ++i) {
        const SE3 T = synth::look_at(Vec3(-8.0 + 5.0 * i, 3.0 * (i % 2), -250), a);
        vol.integrate(track::make_depth_frame(sphere_depth(a, 20.0, T, k), k), T);
    }
    for (int i = 0; i < 2; ++i) {  // b: a stray blob in two frames (one frame alone never reaches the weight threshold)
        const SE3 T1 = synth::look_at(Vec3(60.0 + 3 * i, 0, -250), b);
        vol.integrate(track::make_depth_frame(sphere_depth(b, 8.0, T1, k), k), T1);
    }
    auto near_b = [&](const recon::TriangleMesh& m) {
        return std::ranges::count_if(m.vertices, [&](const Vec3f& v) { return (v.cast<double>() - b).norm() < 12.0; });
    };
    auto near_a = [&](const recon::TriangleMesh& m) {
        return std::ranges::count_if(m.vertices, [&](const Vec3f& v) { return (v.cast<double>() - a).norm() < 24.0; });
    };
    const auto all = recon::extract_mesh(vol);
    CHECK(near_b(all) > 100);
    recon::ExtractParams ep;
    ep.min_observations = 3;
    const auto seen_thrice = recon::extract_mesh(vol, ep);
    CHECK(near_b(seen_thrice) == 0);
    CHECK(static_cast<double>(near_a(seen_thrice)) > 0.8 * static_cast<double>(near_a(all)));
}
