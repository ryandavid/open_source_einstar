#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <print>

#include "einstar/brep/brep.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/fit/deviation.hpp"
#include "einstar/fit/grow.hpp"
#include "einstar/fit/primitive_fit.hpp"
#include "einstar/fit/synthetic_part.hpp"

using namespace einstar;

namespace {

const fit::SyntheticPart& part() {
    static const fit::SyntheticPart p = fit::make_synthetic_part(fit::flanged_box_spec());
    return p;
}

// The scan's support of a truth face: its triangles' vertices and normals.
brep::FaceInput face(const std::string& name, bool exact = true) {
    const auto& p = part();
    const auto it = std::ranges::find(p.faces, name, &fit::TruthFace::name);
    REQUIRE(it != p.faces.end());
    brep::FaceInput f{name, it->surface, {}, {}};
    std::vector<std::uint32_t> tris;
    for (std::uint32_t t = 0; t < p.triangle_face.size(); ++t)
        if (p.triangle_face[t] == it->id) tris.push_back(t);
    const fit::MeshTopology topo(p.mesh);
    const fit::RegionPoints pts = fit::region_points(topo, tris, 0);
    f.points = pts.points;
    f.normals = pts.normals;
    if (!exact) {
        const auto r = fit::fit_surface(fit::kind_of(it->surface), pts.view());
        REQUIRE(r);
        f.surface = r->surface;
    }
    return f;
}

// The flanged box: the body's and flange's faces, the bottom the scan did not see, the body's eight
// fillets, the five holes.
brep::BuildInput flanged_box(bool exact) {
    brep::BuildInput in;
    for (const char* n : {"box0 +z", "box0 +x", "box0 -x", "box0 +y", "box0 -y", "box1 +z", "box1 +x", "box1 -x", "box1 +y", "box1 -y"})
        in.faces.push_back(face(n, exact));
    in.faces.push_back({"bottom", fit::Plane{-Vec3::UnitZ(), 0.0}, {}, {}});  // not scanned: given by the user
    const int top = 0, px = 1, mx = 2, py = 3, my = 4;
    for (const int side : {px, mx, py, my}) in.fillets.push_back({std::format("fillet top {}", side), top, side, 2.0});
    for (const int a : {px, mx})
        for (const int b : {py, my}) in.fillets.push_back({std::format("fillet {} {}", a, b), a, b, 2.0});
    for (const auto& h : fit::flanged_box_spec().holes)
        in.holes.push_back({std::format("hole {:.0f} {:.0f}", h.entry.x(), h.entry.y()), h.entry, h.axis, h.diameter,
                            h.depth < 100 ? std::optional(h.depth) : std::nullopt});
    return in;
}

// Volume of the flanged box: the flange plate, the body above it (a box rounded on all edges, cut at z = 4),
// minus the holes.
double analytic_volume() {
    const double a = 60, b = 40, r = 2;
    const auto rounded_rect = [](double w, double h, double rho) { return w * h - (4 - std::numbers::pi) * rho * rho; };
    double body = 0;
    const int steps = 20000;
    for (int i = 0; i < steps; ++i) {
        const double z = 4 + 16.0 * (i + 0.5) / steps;
        if (z <= 18) {
            body += rounded_rect(a, b, r);
        } else {
            const double rho = std::sqrt(std::max(0.0, r * r - (z - 18) * (z - 18)));  // corner radius in the top cap
            body += rounded_rect(a - 2 * r + 2 * rho, b - 2 * r + 2 * rho, rho);
        }
    }
    body *= 16.0 / steps;
    const double flange = 100 * 50 * 4;
    const double holes = 4 * std::numbers::pi * 2.75 * 2.75 * 4 + std::numbers::pi * 16 * 10;
    return flange + body - holes;
}

}  // namespace

TEST_CASE("solid of the flanged box from its exact surfaces, exported and read back") {
    const brep::BuildInput in = flanged_box(true);
    Stopwatch sw;
    const brep::BuildResult r = brep::build(in);
    std::println("built in {:.0f} ms: ok {} closed {} solids {} faces {} edges {} volume {:.2f} (analytic {:.2f})", sw.elapsed_ms(), r.ok, r.closed,
                 r.solids, r.faces, r.edges, r.volume, analytic_volume());
    for (const auto& line : r.log) std::println("  {}", line);
    REQUIRE(r.ok);
    CHECK(r.closed);
    CHECK(r.solids == 1);
    CHECK(std::abs(r.volume - analytic_volume()) < 1e-3 * analytic_volume());
    CHECK(r.scan_coverage > 0.99);
    CHECK(std::ranges::count(r.face_names, std::string()) == 0);  // every face is named after what it came from

    const auto path = std::filesystem::temp_directory_path() / "einstar_flanged_box.step";
    REQUIRE(brep::write_step(r, path, "flanged box"));
    const brep::StepSummary s = brep::read_step(path);
    std::println("STEP: ok {} valid {} solids {} faces {} volume {:.2f}; names {}; radii:", s.ok, s.valid, s.solids, s.faces, s.volume, s.face_names.size());
    for (const auto& [k, n] : s.surface_kinds) std::println("  {} {}", k, n);
    for (const double rr : s.cylinder_radii) std::println("  radius {:.4f}", rr);
    REQUIRE(s.ok);
    CHECK(s.valid);
    CHECK(s.solids == 1);
    CHECK(std::abs(s.volume - r.volume) < 1e-6 * r.volume);
    // Holes and fillets come back as true cylinders with their radii.
    for (const double radius : {2.0, 2.75, 4.0})
        CHECK(std::ranges::any_of(s.cylinder_radii, [&](double x) { return std::abs(x - radius) < 1e-9; }));
    CHECK(s.face_names.size() == static_cast<std::size_t>(s.faces));
    CHECK(std::ranges::count(s.face_names, std::string("box0 +z")) == 1);
    CHECK(std::ranges::count(s.face_names, std::string("box1 +z")) == 1);
    CHECK(std::ranges::count(s.face_names, std::string("hole 0 0")) == 1);
}

TEST_CASE("solid from noisy fits: the cell vote copes with surfaces that are only nearly right") {
    const brep::BuildInput in = flanged_box(false);
    const brep::BuildResult r = brep::build(in);
    for (const auto& line : r.log) std::println("  {}", line);
    REQUIRE(r.ok);
    CHECK(r.solids == 1);
    std::println("noisy fits: volume {:.2f} vs {:.2f} ({:+.3f}%)", r.volume, analytic_volume(), 100 * (r.volume / analytic_volume() - 1));
    CHECK(std::abs(r.volume - analytic_volume()) < 5e-3 * analytic_volume());
}

TEST_CASE("without a face where the scan is open, the build says so") {
    brep::BuildInput in = flanged_box(true);
    in.faces.pop_back();  // the bottom
    in.fillets.clear();
    in.holes.clear();
    const brep::BuildResult r = brep::build(in);
    for (const auto& line : r.log) std::println("  {}", line);
    std::println("open: ok {} closed {} solids {} volume {:.1f}, scan coverage {:.2f}", r.ok, r.closed, r.solids, r.volume, r.scan_coverage);
    CHECK(r.scan_coverage < 0.9);
    CHECK(std::ranges::any_of(r.log, [](const std::string& l) { return l.find("Add a face where the scan is open") != std::string::npos; }));
}

TEST_CASE("deviation of the scan from the model: small where it fits, a bump is one hot spot, the unseen bottom is unsupported") {
    const brep::BuildResult model = brep::build(flanged_box(true));
    REQUIRE(model.ok);
    const brep::Tessellation tess = brep::tessellate(model, 0.01);
    const fit::TriangleBvh model_bvh(tess.mesh);

    // The scan with a 0.5 mm high bump (raised cosine, 3 mm radius) on the body's top at (12, -8).
    recon::TriangleMesh scan = part().mesh;
    const Vec3f bump(12, -8, 20);
    for (std::size_t v = 0; v < scan.vertices.size(); ++v) {
        const float r = (scan.vertices[v] - bump).norm();
        if (r < 3 && scan.vertices[v].z() > 19.5f) scan.vertices[v].z() += 0.25f * (1 + std::cos(std::numbers::pi_v<float> * r / 3));
    }
    const fit::MeshTopology topo(scan);
    Stopwatch sw;
    const fit::DeviationField field = fit::scan_to_model(scan, model_bvh, tess.triangle_face);
    const fit::DeviationReport rep = fit::summarise(topo, field, {});
    std::println("deviation of {} vertices in {:.0f} ms: rms {:.4f} p95 {:.4f} max {:.4f} within 0.1 mm {:.1f}%, {} unmatched, {} hot spots",
                 scan.vertices.size(), sw.elapsed_ms(), rep.overall.rms, rep.overall.p95, rep.overall.max_abs, 100 * rep.overall.within_tolerance,
                 rep.unmatched, rep.hot_spots.size());
    for (const auto& h : rep.hot_spots)
        if (!h.edge_band)
            std::println("  hot spot at ({:.2f}, {:.2f}, {:.2f}) {:.2f} mm2 ({:.1f} x {:.1f} mm), mean {:+.3f} peak {:+.3f} mm, face '{}'", h.centroid.x(),
                         h.centroid.y(), h.centroid.z(), h.area_mm2, h.length_mm, h.width_mm, h.mean_mm, h.peak_mm,
                         h.model_face >= 0 ? model.face_names[static_cast<std::size_t>(h.model_face)] : "-");
    std::println("  {} edge bands", std::ranges::count_if(rep.hot_spots, &fit::HotSpot::edge_band));
    CHECK(rep.overall.p95 < 0.1);
    REQUIRE(!rep.hot_spots.empty());
    const fit::HotSpot& h = rep.hot_spots.front();
    CHECK((h.centroid - bump.cast<double>()).norm() < 0.5);
    CHECK(h.mean_mm > 0.15);
    CHECK(std::abs(h.peak_mm - 0.5) < 0.06);
    CHECK(model.face_names[static_cast<std::size_t>(h.model_face)] == "box0 +z");
    // Only the bump stands out; the scan's rounding of sharp edges shows as edge bands.
    CHECK(!h.edge_band);
    for (std::size_t i = 1; i < rep.hot_spots.size(); ++i) CHECK(rep.hot_spots[i].edge_band);

    // The model's bottom has no scan behind it; its top does.
    const fit::TriangleBvh scan_bvh(scan);
    const fit::Coverage cov = fit::model_coverage(tess.mesh, tess.triangle_face, static_cast<int>(model.face_names.size()), scan_bvh);
    for (std::size_t f = 0; f < model.face_names.size(); ++f) {
        if (model.face_names[f] == "bottom") CHECK(cov.unsupported_area[f] > 0.9 * cov.face_area[f]);
        if (model.face_names[f] == "box0 +z") CHECK(cov.unsupported_area[f] < 0.01 * cov.face_area[f]);
    }
}
