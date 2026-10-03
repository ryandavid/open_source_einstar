#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <print>

#include "einstar/core/timing.hpp"
#include "einstar/fit/detect.hpp"
#include "einstar/fit/holes.hpp"
#include "einstar/fit/synthetic_part.hpp"

using namespace einstar;
using namespace einstar::fit;

namespace {

struct Scan {
    SyntheticPart part;
    MeshTopology topo;
    TriangleBvh bvh;
    explicit Scan(const PartSpec& spec) : part(make_synthetic_part(spec)), topo(part.mesh), bvh(part.mesh) {}
};

// The plane region of a truth face, grown from a dab at its middle.
GrownRegion grow_face(const Scan& s, const Vec3f& from, const Vec3f& dir) {
    const auto hit = s.bvh.raycast(from, dir.normalized());
    REQUIRE(hit);
    const RegionSeed seed{triangles_within(s.topo, hit->triangle, hit->point, 1.5f), {SurfaceKind::plane}, std::nullopt};
    GrowResult g = grow_regions(s.topo, std::span(&seed, 1));
    REQUIRE(g.regions[0].ok);
    return std::move(g.regions[0]);
}

const PartHole* nearest_hole(const PartSpec& spec, const Vec3& c) {
    const PartHole* best = nullptr;
    for (const auto& h : spec.holes)
        if (!best || (h.entry - c).norm() < (best->entry - c).norm()) best = &h;
    return best;
}

}  // namespace

TEST_CASE("holes in the flange and the body top: position, diameter, through or blind") {
    const PartSpec spec = flanged_box_spec();
    const Scan s(spec);

    const GrownRegion flange = grow_face(s, {40, -10, 50}, {0, 0, -1});
    const auto flange_holes = find_holes(s.part.mesh, s.topo, s.bvh, flange.triangles, std::get<Plane>(flange.fit.surface));
    REQUIRE(flange_holes.size() == 4);
    for (const auto& h : flange_holes) {
        const PartHole* truth = nearest_hole(spec, h.center);
        std::println("flange hole at ({:.3f}, {:.3f}, {:.3f}): diameter {:.3f} (opening {:.3f}, wall {}), wall depth {:.2f}, floor {}",
                     h.center.x(), h.center.y(), h.center.z(), h.diameter, h.opening_diameter,
                     h.wall_diameter ? std::format("{:.3f}", *h.wall_diameter) : "-", h.wall_depth,
                     h.floor_depth ? std::format("{:.2f}", *h.floor_depth) : "-");
        CHECK((h.center - truth->entry).norm() < 0.05);
        CHECK(h.wall_diameter);
        CHECK(std::abs(h.diameter - truth->diameter) < 0.05);
        CHECK(h.axis.dot(truth->axis) > 0.9999);
        CHECK(!h.floor_depth);  // through
    }

    const GrownRegion top = grow_face(s, {10, 8, 50}, {0, 0, -1});
    const auto top_holes = find_holes(s.part.mesh, s.topo, s.bvh, top.triangles, std::get<Plane>(top.fit.surface));
    REQUIRE(top_holes.size() == 1);
    const auto& blind = top_holes[0];
    std::println("blind hole: diameter {:.3f} (opening {:.3f}), wall depth {:.2f}", blind.diameter, blind.opening_diameter, blind.wall_depth);
    CHECK(std::abs(blind.diameter - 8.0) < 0.05);
    CHECK((blind.center - Vec3(0, 0, 20)).norm() < 0.05);
    CHECK(blind.wall_depth > 7.0);
}

TEST_CASE("a hole whose wall was barely scanned: diameter from its opening") {
    PartSpec spec = flanged_box_spec();
    spec.hole_wall_depth = 0.02;  // only the rim of each hole was seen
    const Scan s(spec);
    const GrownRegion flange = grow_face(s, {40, -10, 50}, {0, 0, -1});
    const auto holes = find_holes(s.part.mesh, s.topo, s.bvh, flange.triangles, std::get<Plane>(flange.fit.surface));
    REQUIRE(holes.size() == 4);
    for (const auto& h : holes) {
        std::println("rim-only hole: diameter {:.3f} (opening {:.3f}), wall {}", h.diameter, h.opening_diameter, h.wall_diameter.has_value());
        CHECK(!h.wall_diameter);
        CHECK(std::abs(h.diameter - 5.5) < 0.1);
        CHECK((h.center - nearest_hole(spec, h.center)->entry).norm() < 0.08);
    }
}

TEST_CASE("auto-detect finds every face of the flanged box") {
    const Scan s(flanged_box_spec());
    Stopwatch sw;
    const auto regions = detect_regions(s.topo);
    std::println("detected {} regions in {:.0f} ms", regions.size(), sw.elapsed_ms());

    // Best match of each truth face with a reasonable number of triangles.
    std::map<int, std::size_t> truth_count;
    for (const int f : s.part.triangle_face) ++truth_count[f];
    std::size_t matched = 0, faces = 0;
    for (const auto& [face, count] : truth_count) {
        if (count < 150) continue;  // corner patches and slivers
        ++faces;
        const TruthFace* tf = s.part.face(face);
        double best_iou = 0;
        const DetectedRegion* best = nullptr;
        for (const auto& r : regions) {
            std::size_t both = 0;
            for (const auto t : r.triangles) both += s.part.triangle_face[t] == face;
            const double iou = static_cast<double>(both) / static_cast<double>(r.triangles.size() + count - both);
            if (iou > best_iou) {
                best_iou = iou;
                best = &r;
            }
        }
        const bool kind_ok = best && kind_of(best->fit.surface) == kind_of(tf->surface);
        std::println("  {:18} {:5} triangles: best IoU {:.3f} as {}", tf->name, count, best_iou,
                     best ? kind_name(kind_of(best->fit.surface)) : "-");
        if (best_iou > 0.7 && kind_ok) ++matched;
    }
    CHECK(matched == faces);
    for (const auto& r : regions)
        std::println("  region {:8} {:6.1f} mm2 sigma {:.3f}", kind_name(kind_of(r.fit.surface)), r.area_mm2, r.fit.sigma);
}
