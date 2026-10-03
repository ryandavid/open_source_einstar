#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <print>

#include "einstar/core/timing.hpp"
#include "einstar/fit/constraints.hpp"
#include "einstar/fit/grow.hpp"
#include "einstar/fit/primitive_fit.hpp"
#include "einstar/fit/synthetic_part.hpp"

using namespace einstar;
using namespace einstar::fit;

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

// The flanged box, moved to an arbitrary pose so nothing lines up with the scan's axes.
struct Part {
    SyntheticPart part;
    SE3 pose;
    std::unique_ptr<MeshTopology> topo;

    Part() : part(make_synthetic_part(flanged_box_spec())) {
        pose = SE3::Identity();
        pose.linear() = Eigen::AngleAxisd(0.7, Vec3(0.3, -1, 0.5).normalized()).toRotationMatrix();
        pose.translation() = Vec3(120, -40, 300);
        for (auto& v : part.mesh.vertices) v = (pose * v.cast<double>()).cast<float>();
        part.mesh.compute_normals();
        topo = std::make_unique<MeshTopology>(part.mesh);
    }

    // A feature from the triangles of a truth face, starting from its free fit.
    [[nodiscard]] Feature feature(std::string_view name, std::span<const std::uint32_t> only = {}) const {
        const auto it = std::ranges::find(part.faces, std::string(name), &TruthFace::name);
        REQUIRE(it != part.faces.end());
        std::vector<std::uint32_t> tris;
        if (only.empty()) {
            for (std::uint32_t t = 0; t < part.triangle_face.size(); ++t)
                if (part.triangle_face[t] == it->id) tris.push_back(t);
        } else {
            tris.assign(only.begin(), only.end());
        }
        const RegionPoints pts = region_points(*topo, tris, 0);
        const auto fit = fit_surface(kind_of(it->surface), pts.view());
        REQUIRE(fit);
        return {fit->surface, pts.points, pts.weights, false};
    }
};

const Part& part() {
    static const Part p;
    return p;
}

enum F { top, px, mx, py, my, flange, hole0, hole1, hole2, hole3, blind, count };

SolveInput features() {
    const Part& p = part();
    SolveInput in;
    for (const char* name : {"box0 +z", "box0 +x", "box0 -x", "box0 +y", "box0 -y", "box1 +z", "hole0 wall", "hole1 wall", "hole2 wall",
                             "hole3 wall", "hole4 wall"})
        in.features.push_back(p.feature(name));
    return in;
}

// The box's faces square to one datum: z from the top, x from the +x side.
SolveInput squared() {
    SolveInput in = features();
    in.datums.push_back({datum_from(in.features[top].surface, 2, in.features[px].surface), false});
    for (const int f : {top, flange, hole0, hole1, hole2, hole3, blind}) in.constraints.push_back(Aligned{f, 0, 2});
    for (const int f : {px, mx}) in.constraints.push_back(Aligned{f, 0, 0});
    for (const int f : {py, my}) in.constraints.push_back(Aligned{f, 0, 1});
    return in;
}

double angle_to(const Vec3& a, const Vec3& b) { return a.cross(b).norm(); }

}  // namespace

TEST_CASE("solve without constraints keeps the free fits") {
    const SolveInput in = features();
    const SolveResult r = solve(in);
    CHECK(r.converged);
    for (std::size_t f = 0; f < in.features.size(); ++f) CHECK(r.features[f].max_move_mm < 0.01);
}

TEST_CASE("faces squared to a datum: exact, and the datum finds the part's axes") {
    const SolveInput in = squared();
    Stopwatch sw;
    const SolveResult r = solve(in);
    std::println("squared: {} iterations, {:.0f} ms", r.iterations, sw.elapsed_ms());
    REQUIRE(r.converged);
    const Mat3 R = r.datums[0].linear();
    for (std::size_t i = 0; i < in.constraints.size(); ++i) {
        INFO(describe(in.constraints[i]));
        CHECK(r.constraints[i].status == ConstraintStatus::satisfied);
        const auto& a = std::get<Aligned>(in.constraints[i]);
        CHECK(angle_to(*direction_of(r.surfaces[static_cast<std::size_t>(a.feature)]), R.col(a.axis)) < 1e-12);
    }
    // The datum's axes are the part's (up to sense), within the scan's accuracy.
    for (int k = 0; k < 3; ++k) CHECK(angle_to(R.col(k), part().pose.linear().col(k)) < 0.03 * kDeg);
    // Squaring costs the fit next to nothing on a part that is square.
    for (std::size_t f = 0; f < in.features.size(); ++f) {
        INFO(f);
        CHECK(r.features[f].max_move_mm < 0.03);
        std::println("  feature {:2}: rms {:.4f} sigma {:.4f} moved {:.4f} mm, radius sd {:.4f} mm, direction sd {:.4f} deg", f, r.features[f].rms,
                     r.features[f].sigma, r.features[f].max_move_mm, r.features[f].radius_sd, r.features[f].direction_sd_deg);
    }
}

TEST_CASE("redundant and conflicting constraints are reported, the rest still hold") {
    SolveInput in = squared();
    const std::size_t redundant = in.constraints.size();
    in.constraints.push_back(Parallel{top, flange});  // both are already along z
    in.constraints.push_back(Perpendicular{top, px});  // z and x
    const std::size_t conflict = in.constraints.size();
    in.constraints.push_back(Aligned{top, 0, 0});  // the top cannot be along x as well
    const std::size_t invalid = in.constraints.size();
    in.constraints.push_back(Diameter{top, 5});  // a plane has no diameter
    const SolveResult r = solve(in);
    CHECK(r.constraints[redundant].status == ConstraintStatus::redundant);
    CHECK(r.constraints[redundant + 1].status == ConstraintStatus::redundant);
    CHECK(r.constraints[conflict].status == ConstraintStatus::conflict);
    CHECK(r.constraints[invalid].status == ConstraintStatus::invalid);
    for (std::size_t i = 0; i < redundant; ++i) CHECK(r.constraints[i].status == ConstraintStatus::satisfied);
    CHECK(angle_to(*direction_of(r.surfaces[top]), r.datums[0].linear().col(2)) < 1e-12);
}

TEST_CASE("measured dimensions hold exactly and report what they cost") {
    SolveInput in = squared();
    const std::size_t d0 = in.constraints.size();
    in.constraints.push_back(Diameter{hole0, 5.5});
    in.constraints.push_back(Diameter{hole1, 5.7});  // 0.2 mm larger than the part: the wall moves 0.1 mm
    in.constraints.push_back(Distance{px, mx, 60.0});
    in.constraints.push_back(Radius{blind, 4.0});
    const SolveResult r = solve(in);
    REQUIRE(r.converged);
    CHECK(std::abs(2 * std::get<Cylinder>(r.surfaces[hole0]).radius - 5.5) < 1e-12);
    CHECK(std::abs(2 * std::get<Cylinder>(r.surfaces[hole1]).radius - 5.7) < 1e-12);
    CHECK(std::abs(std::get<Cylinder>(r.surfaces[blind]).radius - 4.0) < 1e-12);
    const auto& a = std::get<Plane>(r.surfaces[px]);
    const auto& b = std::get<Plane>(r.surfaces[mx]);
    CHECK(std::abs(std::abs(a.normal.dot(position_of(b)) - a.offset) - 60.0) < 1e-10);
    for (std::size_t i = d0; i < in.constraints.size(); ++i)
        std::println("  {:40} {:10} moved {:.4f} mm, rms {:+.4f} mm", describe(in.constraints[i]), status_name(r.constraints[i].status),
                     r.constraints[i].max_move_mm, r.constraints[i].delta_rms_mm);
    // The free fit of the hole is ~5.48 (the mesher pulls concave walls in slightly).
    const double free_d = 2 * std::get<Cylinder>(in.features[hole1].surface).radius;
    CHECK(std::abs(r.constraints[d0 + 1].max_move_mm - (5.7 - free_d) / 2) < 0.01);
    CHECK(r.constraints[d0 + 1].delta_rms_mm > 0.03);  // rms of the inliers (within 3 sigma), so it understates a shift
    CHECK(r.constraints[d0].max_move_mm < 0.03);  // the true size costs little
}

TEST_CASE("offsets from the datum place the part's origin; hole positions read off it") {
    SolveInput in = squared();
    in.constraints.push_back(Offset{flange, 0, 2, 4.0});
    in.constraints.push_back(Offset{px, 0, 0, 30.0});
    in.constraints.push_back(Offset{mx, 0, 0, -30.0});
    in.constraints.push_back(Offset{py, 0, 1, 20.0});
    in.constraints.push_back(Offset{my, 0, 1, -20.0});
    // Hole 0 of the spec is at (-42, -19); the datum's x and y may come out reversed, so compare magnitudes.
    const SolveResult r = solve(in);
    REQUIRE(r.converged);
    CHECK((r.datums[0].translation() - part().pose.translation()).norm() < 0.05);
    const SE3 inv = r.datums[0].inverse();
    for (const int h : {hole0, hole1, hole2, hole3}) {
        const Vec3 c = inv * std::get<Cylinder>(r.surfaces[static_cast<std::size_t>(h)]).point;
        INFO(h);
        CHECK(std::abs(std::abs(c.x()) - 42) < 0.05);
        CHECK(std::abs(std::abs(c.y()) - 19) < 0.05);
    }
}

TEST_CASE("two halves of a hole wall made coaxial share one axis") {
    const Part& p = part();
    const auto it = std::ranges::find(p.part.faces, std::string("hole0 wall"), &TruthFace::name);
    const Vec3 axis_point = p.pose * std::get<Cylinder>(it->surface).point;
    const Vec3 axis = p.pose.linear() * std::get<Cylinder>(it->surface).axis;
    const Vec3 side = any_perpendicular(axis);
    std::vector<std::uint32_t> a, b;
    for (std::uint32_t t = 0; t < p.part.triangle_face.size(); ++t)
        if (p.part.triangle_face[t] == it->id) ((p.topo->centroid(t).cast<double>() - axis_point).dot(side) > 0 ? a : b).push_back(t);
    SolveInput in;
    in.features.push_back(p.feature("hole0 wall", a));
    in.features.push_back(p.feature("hole0 wall", b));
    in.constraints.push_back(Coaxial{0, 1});
    const SolveResult r = solve(in);
    REQUIRE(r.converged);
    const auto& ca = std::get<Cylinder>(r.surfaces[0]);
    const auto& cb = std::get<Cylinder>(r.surfaces[1]);
    CHECK(angle_to(ca.axis, cb.axis) < 1e-12);
    const Vec3 d = cb.point - ca.point;
    CHECK((d - d.dot(ca.axis) * ca.axis).norm() < 1e-10);
    CHECK((ca.point - axis_point - (ca.point - axis_point).dot(axis) * axis).norm() < 0.03);
}
