#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <print>

#include "einstar/core/timing.hpp"
#include "einstar/fit/bvh.hpp"
#include "einstar/fit/grow.hpp"
#include "einstar/fit/synthetic_part.hpp"

using namespace einstar;
using namespace einstar::fit;

namespace {

struct Fixture {
    SyntheticPart part = make_synthetic_part(flanged_box_spec());
    MeshTopology topo{part.mesh};
    TriangleBvh bvh{part.mesh};
};

const Fixture& fixture() {
    static const Fixture f;
    return f;
}

int face_id(const SyntheticPart& part, std::string_view name) {
    const auto it = std::ranges::find(part.faces, std::string(name), &TruthFace::name);
    REQUIRE(it != part.faces.end());
    return it->id;
}

// A brush dab: the triangles within `radius` of where a ray hits the mesh.
std::vector<std::uint32_t> dab(const Fixture& f, const Vec3f& from, const Vec3f& dir, float radius) {
    const auto hit = f.bvh.raycast(from, dir.normalized());
    REQUIRE(hit);
    return triangles_within(f.topo, hit->triangle, hit->point, radius);
}

// Triangles of the truth face connected (through triangles of that face) to `start`.
std::vector<std::uint32_t> truth_component(const Fixture& f, int face, std::uint32_t start) {
    std::vector<std::uint8_t> seen(f.topo.triangle_count(), 0);
    std::vector<std::uint32_t> out, stack{start};
    seen[start] = 1;
    while (!stack.empty()) {
        const auto t = stack.back();
        stack.pop_back();
        out.push_back(t);
        for (const auto n : f.topo.neighbors(t))
            if (n != kNoTriangle && !seen[n] && f.part.triangle_face[n] == face) {
                seen[n] = 1;
                stack.push_back(n);
            }
    }
    std::ranges::sort(out);
    return out;
}

struct Overlap {
    double precision, recall, iou;
};
Overlap overlap(std::vector<std::uint32_t> got, const std::vector<std::uint32_t>& truth) {
    std::ranges::sort(got);
    std::vector<std::uint32_t> both;
    std::ranges::set_intersection(got, truth, std::back_inserter(both));
    const auto i = static_cast<double>(both.size());
    return {i / static_cast<double>(got.size()), i / static_cast<double>(truth.size()),
            i / static_cast<double>(got.size() + truth.size() - both.size())};
}

}  // namespace

TEST_CASE("seeded growth recovers whole faces of the flanged box") {
    const Fixture& f = fixture();
    struct Case {
        const char* face;
        Vec3f from, dir;
        SurfaceKind kind;
        double min_iou;
    };
    const Case cases[] = {
        {"box0 +z", {10, 8, 50}, {0, 0, -1}, SurfaceKind::plane, 0.95},
        {"box0 +x", {80, 3, 12}, {-1, 0, 0}, SurfaceKind::plane, 0.92},  // the first row of each fillet is within noise of the plane
        {"box1 +z", {40, -10, 50}, {0, 0, -1}, SurfaceKind::plane, 0.93},
        {"box1 -y", {-40, -80, 2}, {0, 1, 0}, SurfaceKind::plane, 0.88},  // a 4 mm strip, mostly edge
        {"box0 fillet x+y+", {48.6f, 38.6f, 12}, {-1, -1, 0}, SurfaceKind::cylinder, 0.70},  // the planes' first rows lie within noise of it
        {"hole0 wall", {-40.0f, -19, 6}, {-1, 0, -1}, SurfaceKind::cylinder, 0.85},
    };
    for (const Case& c : cases) {
        INFO(c.face);
        const int id = face_id(f.part, c.face);
        const auto seed = dab(f, c.from, c.dir, 1.5f);
        REQUIRE(!seed.empty());
        const std::uint32_t start = *std::ranges::find_if(seed, [&](auto t) { return f.part.triangle_face[t] == id; });
        const RegionSeed rs{seed, {}, std::nullopt};
        Stopwatch sw;
        const GrowResult g = grow_regions(f.topo, std::span(&rs, 1));
        const double ms = sw.elapsed_ms();
        REQUIRE(g.regions[0].ok);
        const auto truth = truth_component(f, id, start);
        const Overlap o = overlap(g.regions[0].triangles, truth);
        std::println("{:18} {:8} sigma {:.3f} mm, {:5} triangles (truth {:5}) precision {:.3f} recall {:.3f} IoU {:.3f}  {:.1f} ms", c.face,
                     kind_name(kind_of(g.regions[0].fit.surface)), g.regions[0].fit.sigma, g.regions[0].triangles.size(), truth.size(),
                     o.precision, o.recall, o.iou, ms);
        CHECK(kind_of(g.regions[0].fit.surface) == c.kind);
        CHECK(o.iou > c.min_iou);
        // The fitted surface matches the truth.
        const Surface& truth_surface = f.part.face(id)->surface;
        double worst = 0;
        for (const auto t : truth) worst = std::max(worst, std::abs(signed_distance(g.regions[0].fit.surface, project(truth_surface, f.topo.centroid(t).cast<double>()))));
        CHECK(worst < 0.03);
    }
}

TEST_CASE("seeds grow together: neighbouring faces split at their edge, a fillet between them stays apart") {
    const Fixture& f = fixture();
    // Body top, body +x side, and the fillet between them.
    const RegionSeed seeds[] = {
        {dab(f, {10, 8, 50}, {0, 0, -1}, 1.5f), {}, std::nullopt},
        {dab(f, {80, 3, 12}, {-1, 0, 0}, 1.5f), {}, std::nullopt},
        {dab(f, {58.6f, 3, 48.6f}, {-1, 0, -1}, 0.8f), {SurfaceKind::cylinder}, std::nullopt},
    };
    const GrowResult g = grow_regions(f.topo, seeds);
    const int top = face_id(f.part, "box0 +z"), side = face_id(f.part, "box0 +x"), fillet = face_id(f.part, "box0 fillet z+x+");
    for (std::size_t r = 0; r < 3; ++r) {
        REQUIRE(g.regions[r].ok);
        // Triangles clearly off the face (the first row of a fillet is within noise of its plane and may
        // go either way).
        std::size_t wrong = 0;
        const int want = r == 0 ? top : r == 1 ? side : fillet;
        const Surface& truth = f.part.face(want)->surface;
        for (const auto t : g.regions[r].triangles)
            wrong += std::abs(signed_distance(truth, f.topo.centroid(t).cast<double>())) > 0.1;
        std::println("region {} ({}): {} triangles, {} off the face, sigma {:.3f}", r, kind_name(kind_of(g.regions[r].fit.surface)),
                     g.regions[r].triangles.size(), wrong, g.regions[r].fit.sigma);
        CHECK(static_cast<double>(wrong) < 0.005 * static_cast<double>(g.regions[r].triangles.size()));
    }
    // The fillet between top and +x side is the y-axis edge with x+, z+.
    const auto& cyl = std::get<Cylinder>(g.regions[2].fit.surface);
    CHECK(std::abs(cyl.radius - 2.0) < 0.1);
    CHECK(std::abs(std::abs(cyl.axis.y()) - 1.0) < 1e-3);
}
