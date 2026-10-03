#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <print>
#include <random>

#include "einstar/core/timing.hpp"
#include "einstar/fit/bvh.hpp"
#include "einstar/fit/mesh_topology.hpp"
#include "einstar/fit/synthetic_part.hpp"
#include "einstar/recon/registration.hpp"

using namespace einstar;
using namespace einstar::fit;

namespace {

const SyntheticPart& flanged_box() {
    static const SyntheticPart part = make_synthetic_part(flanged_box_spec());
    return part;
}

}  // namespace

TEST_CASE("synthetic flanged box: scan-like mesh with truth labels") {
    const SyntheticPart& part = flanged_box();
    REQUIRE(part.mesh.triangles.size() > 50000);
    REQUIRE(part.triangle_face.size() == part.mesh.triangles.size());

    // Every visible face of the part has triangles, the unseen bottom has none.
    for (const char* name : {"box0 +z", "box0 +x", "box0 -y", "box1 +z", "box1 -x", "box0 fillet x+y+", "box0 corner +++", "hole0 wall"}) {
        const auto it = std::ranges::find(part.faces, std::string(name), &TruthFace::name);
        REQUIRE(it != part.faces.end());
        INFO(name);
        CHECK(part.triangles_of(it->id) > 20);
    }
    const auto bottom = std::ranges::find(part.faces, std::string("box1 -z"), &TruthFace::name);
    CHECK(part.triangles_of(bottom->id) == 0);

    // Triangles lie on their labelled surface (noise 0.03 mm; the voxel-scale rounding of sharp edges
    // puts a few triangles further off).
    std::size_t near = 0;
    for (std::size_t t = 0; t < part.mesh.triangles.size(); ++t) {
        const auto& tri = part.mesh.triangles[t];
        const Vec3 c = ((part.mesh.vertices[tri[0]] + part.mesh.vertices[tri[1]] + part.mesh.vertices[tri[2]]) / 3.0f).cast<double>();
        near += std::abs(signed_distance(part.face(part.triangle_face[t])->surface, c)) < 0.1;
    }
    const double frac = static_cast<double>(near) / static_cast<double>(part.mesh.triangles.size());
    std::println("flanged box: {} triangles, {:.1f}% within 0.1 mm of their truth face", part.mesh.triangles.size(), 100 * frac);
    CHECK(frac > 0.95);
}

TEST_CASE("mesh topology: boundary loops are the scan's openings") {
    const SyntheticPart& part = flanged_box();
    const MeshTopology topo(part.mesh);
    // The unseen bottom outline, the four through holes where they reach the bottom, and the blind hole
    // whose deep wall and floor are unseen.
    std::println("boundary loops: {}", topo.boundary_loops().size());
    for (const auto& loop : topo.boundary_loops()) std::println("  {} vertices", loop.size());
    CHECK(topo.boundary_loops().size() == 6);

    std::uint32_t count = 0;
    (void)topo.components(count);
    CHECK(count == 1);

    // Neighbours are symmetric.
    for (std::uint32_t t = 0; t < topo.triangle_count(); t += 97)
        for (const auto n : topo.neighbors(t))
            if (n != kNoTriangle) CHECK(std::ranges::count(topo.neighbors(n), t) == 1);
}

TEST_CASE("triangle BVH agrees with brute force") {
    const SyntheticPart& part = flanged_box();
    const auto& mesh = part.mesh;
    Stopwatch sw;
    const TriangleBvh bvh(mesh);
    std::println("BVH over {} triangles in {:.1f} ms", mesh.triangles.size(), sw.elapsed_ms());

    std::mt19937 rng(7);
    std::uniform_real_distribution<float> ux(-60, 60), uy(-35, 35), uz(-10, 30);
    for (int i = 0; i < 40; ++i) {
        const Vec3f p(ux(rng), uy(rng), uz(rng));
        float best = 1e30f;
        for (const auto& tri : mesh.triangles)
            best = std::min(best, (closest_on_triangle(p, mesh.vertices[tri[0]], mesh.vertices[tri[1]], mesh.vertices[tri[2]]) - p).norm());
        const auto hit = bvh.closest(p);
        REQUIRE(hit);
        CHECK(std::abs(hit->distance - best) < 1e-4f);
    }

    // Rays straight down onto the top of the body hit z = 20, onto the flange z = 4.
    const auto top = bvh.raycast(Vec3f(10, 5, 100), Vec3f(0, 0, -1));
    REQUIRE(top);
    CHECK(std::abs(top->point.z() - 20.0f) < 0.1f);
    const auto flange = bvh.raycast(Vec3f(40, 0, 100), Vec3f(0, 0, -1));
    REQUIRE(flange);
    CHECK(std::abs(flange->point.z() - 4.0f) < 0.1f);
    // The blind hole's floor was never seen, so a ray down its middle passes through; a slanted one hits
    // its wall.
    CHECK(!bvh.raycast(Vec3f(0.5f, 0, 100), Vec3f(0, 0, -1)));
    const auto wall = bvh.raycast(Vec3f(-2, 0, 22), Vec3f(1, 0, -1).normalized());
    REQUIRE(wall);
    CHECK(std::abs(wall->point.x() - 4.0f) < 0.1f);
    CHECK(!bvh.raycast(Vec3f(0, 40, 100), Vec3f(0, 0, -1)));  // misses the part
}

TEST_CASE("cloud index: knn and radius queries") {
    recon::Cloud cloud;
    for (int i = 0; i < 100; ++i) {
        cloud.points.emplace_back(static_cast<float>(i), 0.0f, 0.0f);
        cloud.normals.emplace_back(0.0f, 0.0f, 1.0f);
    }
    const recon::CloudIndex index(cloud);
    const auto k = index.knn(Vec3f(10.2f, 0, 0), 3);
    REQUIRE(k.size() == 3);
    CHECK(k[0] == 10);
    CHECK(k[1] == 11);
    CHECK(k[2] == 9);
    const auto r = index.radius(Vec3f(50, 0, 0), 2.5f);
    CHECK(r.size() == 5);
    CHECK(r[0] == 50);
}
