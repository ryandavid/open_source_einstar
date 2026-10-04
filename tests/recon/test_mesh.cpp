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

TEST_CASE("cleanup: flaps on a non-manifold edge go, near fragments stay, isolated ones go") {
    recon::TriangleMesh m;
    // A w x w grid of 1 mm squares with its corner at o, in the plane z = o.z.
    auto add_grid = [&](Vec3f o, int w) {
        const auto b = static_cast<std::uint32_t>(m.vertices.size());
        const auto n = static_cast<std::uint32_t>(w + 1);
        for (std::uint32_t y = 0; y < n; ++y)
            for (std::uint32_t x = 0; x < n; ++x) m.vertices.push_back(o + Vec3f(static_cast<float>(x), static_cast<float>(y), 0.0f));
        for (std::uint32_t y = 0; y + 1 < n; ++y)
            for (std::uint32_t x = 0; x + 1 < n; ++x) {
                const auto i = b + y * n + x;
                m.triangles.push_back({i, i + 1, i + n + 1});
                m.triangles.push_back({i, i + n + 1, i + n});
            }
    };
    add_grid(Vec3f(0, 0, 0), 30);   // 900 mm^2
    add_grid(Vec3f(40, 0, 0), 8);   // 64 mm^2, 10 mm away: a fragment broken off by a hole
    add_grid(Vec3f(300, 0, 0), 8);  // 64 mm^2, far from everything
    const auto main_triangles = 2u * 30u * 30u;
    // A two-triangle flap standing on an inner edge of the main grid (that edge then has four
    // triangles): connected by vertices, but a piece of its own.
    const auto a = static_cast<std::uint32_t>(5 * 31 + 5), b = a + 1;
    const auto top = static_cast<std::uint32_t>(m.vertices.size());
    m.vertices.insert(m.vertices.end(), {Vec3f(5, 5, 1), Vec3f(6, 5, 1)});
    m.triangles.push_back({a, b, top + 1});
    m.triangles.push_back({a, top + 1, top});
    recon::CleanupParams p;
    p.isolated_component_fraction = 0.1;  // pieces up to ~100 mm^2 are tested for isolation
    const auto rep = recon::remove_small_components(m, p);
    CHECK(rep.components == 4);
    CHECK(rep.removed_components == 2);
    CHECK(rep.removed_isolated == 1);
    CHECK(m.triangles.size() == main_triangles + 2u * 8u * 8u);
    CHECK(std::ranges::all_of(m.vertices, [](const Vec3f& v) { return v.x() < 100 && v.z() == 0; }));
}

TEST_CASE("marker stickers: the bump over a sticker is flattened onto the surface around it") {
    // A gently curved surface z = (x^2 + y^2) / 400 (0.5 mm grid) with a 0.6 mm bump over a 6 mm
    // marker at (10, 0); a second marker at (-20, 0) sits next to a 3 mm step and is left alone.
    recon::TriangleMesh m;
    auto surface = [](float x, float y) { return (x * x + y * y) / 400.0f + (x < -20.0f ? 3.0f : 0.0f); };
    auto bump = [](float x, float y) { return 0.6f * std::exp(-((x - 10) * (x - 10) + y * y) / (2 * 2.5f * 2.5f)); };
    const int n = 121;  // -30 .. 30 mm
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const float x = -30.0f + 0.5f * static_cast<float>(i), y = -30.0f + 0.5f * static_cast<float>(j);
            m.vertices.emplace_back(x, y, surface(x, y) + bump(x, y));
        }
    for (int j = 0; j + 1 < n; ++j)
        for (int i = 0; i + 1 < n; ++i) {
            const auto v = static_cast<std::uint32_t>(j * n + i), un = static_cast<std::uint32_t>(n);
            m.triangles.push_back({v, v + 1, v + un + 1});
            m.triangles.push_back({v, v + un + 1, v + un});
        }
    m.compute_normals();
    const auto before = m.vertices;
    const std::vector<recon::MarkerDisc> markers{{Vec3f(10, 0, surface(10, 0) + 0.6f), Vec3f(0, 0, 1), 3.0f},
                                                 {Vec3f(-20, 0, surface(-20, 0)), Vec3f(0, 0, 1), 3.0f}};
    CHECK(recon::flatten_markers(m, markers) == 1);
    double worst = 0, moved_far = 0, moved_step = 0;
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3f& p = m.vertices[i];
        const float r = std::hypot(p.x() - 10, p.y());
        if (r < 6) worst = std::max(worst, static_cast<double>(std::abs(p.z() - surface(p.x(), p.y()))));
        if (r > 12) moved_far = std::max(moved_far, static_cast<double>((p - before[i]).norm()));
        if (std::hypot(p.x() + 20, p.y()) < 10) moved_step = std::max(moved_step, static_cast<double>((p - before[i]).norm()));
    }
    CHECK(worst < 0.05);
    CHECK(moved_far == 0);
    CHECK(moved_step == 0);
}

TEST_CASE("small holes are closed, consistently oriented; larger ones stay open") {
    // A 20 x 20 grid of 0.5 mm squares with a pinhole of one square, one of a single triangle and a
    // 3 x 3 square hole (6 mm round).
    recon::TriangleMesh m;
    const int n = 21;
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) m.vertices.emplace_back(0.5f * static_cast<float>(x), 0.5f * static_cast<float>(y), 0.0f);
    for (int y = 0; y + 1 < n; ++y)
        for (int x = 0; x + 1 < n; ++x) {
            if (x == 4 && y == 4) continue;                                // pinhole
            if (x >= 10 && x < 13 && y >= 10 && y < 13) continue;          // larger hole
            const auto i = static_cast<std::uint32_t>(y * n + x), un = static_cast<std::uint32_t>(n);
            m.triangles.push_back({i, i + 1, i + un + 1});
            if (!(x == 15 && y == 4)) m.triangles.push_back({i, i + un + 1, i + un});  // one missing triangle
        }
    m.compute_normals();
    const auto before = m.triangles.size();
    CHECK(recon::fill_small_holes(m, 2.5) == 2);
    CHECK(m.triangles.size() == before + 4 + 1);  // a fan of four, one triangle
    // Every directed edge once (orientation consistent), boundary only along the outline and the large hole.
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> directed;
    for (const auto& t : m.triangles)
        for (int e = 0; e < 3; ++e) ++directed[{t[static_cast<std::size_t>(e)], t[static_cast<std::size_t>((e + 1) % 3)]}];
    int boundary = 0;
    for (const auto& [e, c] : directed) {
        CHECK(c == 1);
        boundary += !directed.contains({e.second, e.first});
    }
    CHECK(boundary == 4 * 20 + 4 * 3);
    CHECK(m.normals.back().z() > 0.99f);  // the pinhole's new vertex
}

TEST_CASE("watertight: a sphere with a cap missing comes out closed") {
    // A UV sphere (radius 20 mm, ~1 mm triangles) without the cap above 60 degrees latitude.
    recon::TriangleMesh m;
    const double r = 20;
    const int rings = 64, segs = 128;
    for (int i = 0; i <= rings; ++i)
        for (int j = 0; j < segs; ++j) {
            const double th = M_PI * i / rings, ph = 2 * M_PI * j / segs;
            const Vec3 p(r * std::sin(th) * std::cos(ph), r * std::sin(th) * std::sin(ph), r * std::cos(th));
            m.vertices.push_back(p.cast<float>());
            m.normals.push_back(p.normalized().cast<float>());
        }
    auto id = [&](int i, int j) { return static_cast<std::uint32_t>(i * segs + (j % segs)); };
    for (int i = 0; i < rings; ++i) {
        if (M_PI * i / rings < M_PI / 6) continue;  // the cap
        for (int j = 0; j < segs; ++j) {
            m.triangles.push_back({id(i, j), id(i + 1, j), id(i + 1, j + 1)});
            m.triangles.push_back({id(i, j), id(i + 1, j + 1), id(i, j + 1)});
        }
    }
    recon::WatertightParams p;
    p.cell_mm = 0.5;
    auto closed = recon::watertight_mesh(m, p);
    REQUIRE(closed.has_value());
    REQUIRE(!closed->empty());
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> edges;
    for (const auto& t : closed->triangles)
        for (int e = 0; e < 3; ++e) {
            const auto a = t[static_cast<std::size_t>(e)], b = t[static_cast<std::size_t>((e + 1) % 3)];
            ++edges[{std::min(a, b), std::max(a, b)}];
        }
    const auto open = std::ranges::count_if(edges, [](const auto& kv) { return kv.second != 2; });
    double worst = 0;
    for (const auto& v : closed->vertices)
        if (v.z() < 0.8 * r) worst = std::max(worst, std::abs(v.cast<double>().norm() - r));  // where there was data
    std::println("watertight sphere: {} triangles, {} edges not shared by two, radial error {:.3f} mm where scanned",
                 closed->triangles.size(), open, worst);
    CHECK(open == 0);
    CHECK(worst < 0.2);
    // Outward normals: the surface encloses the centre.
    CHECK(closed->normals.front().dot(closed->vertices.front().normalized()) > 0.9f);
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
