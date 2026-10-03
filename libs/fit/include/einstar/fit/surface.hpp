#pragma once

// Analytic surfaces fitted to scan regions: the faces a CAD model is made of.

#include <memory>
#include <span>
#include <string_view>
#include <variant>
#include <vector>

#include "einstar/core/se3.hpp"

namespace einstar::fit {

// Signed distances are positive on the side of the surface's own normal: along `normal` for a plane, away
// from the axis for a cylinder or cone, away from the centre for a sphere, away from the tube's core for a
// torus. Which side is material is decided by the feature using the surface, not here.

struct Plane {
    Vec3 normal = Vec3::UnitZ();  // unit
    double offset = 0;            // normal . x = offset on the plane
};

struct Cylinder {
    Vec3 point = Vec3::Zero();  // on the axis
    Vec3 axis = Vec3::UnitZ();  // unit
    double radius = 1;
};

// One nappe of a circular cone, opening along +axis from the apex.
struct Cone {
    Vec3 apex = Vec3::Zero();
    Vec3 axis = Vec3::UnitZ();  // unit
    double half_angle = 0.5;    // rad, in (0, pi/2)
};

struct Sphere {
    Vec3 center = Vec3::Zero();
    double radius = 1;
};

struct Torus {
    Vec3 center = Vec3::Zero();
    Vec3 axis = Vec3::UnitZ();  // unit, normal to the plane of the core circle
    double major = 2;           // core circle radius
    double minor = 1;           // tube radius
};

// A freeform face: a height field over a plane, a uniform bicubic B-spline. In the frame's coordinates the surface
// is z = h(x, y) over x in [u0, u0 + (nu - 3) du], y in [v0, v0 + (nv - 3) dv]; h's nu x nv control heights sit
// over the uniform knots u0 + (i - 3) du (so the spline is exactly OpenCASCADE's non-clamped bicubic B-spline with
// those knots, its poles at the Greville points). Its normal is the frame's +z side. Not a solver parameter: a
// freeform face is fitted once to its region and then held.
struct Freeform {
    SE3 frame = SE3::Identity();
    double u0 = 0, v0 = 0, du = 1, dv = 1;
    int nu = 4, nv = 4;
    std::shared_ptr<const std::vector<double>> heights;  // nu * nv, index i * nv + j

    // Height and its gradient at frame coordinates (x, y), clamped to the domain.
    [[nodiscard]] double height(double x, double y, Vec2* gradient = nullptr) const;
    [[nodiscard]] double u_max() const { return u0 + (nu - 3) * du; }
    [[nodiscard]] double v_max() const { return v0 + (nv - 3) * dv; }
};

using Surface = std::variant<Plane, Cylinder, Cone, Sphere, Torus, Freeform>;

enum class SurfaceKind { plane, cylinder, cone, sphere, torus, freeform };

[[nodiscard]] SurfaceKind kind_of(const Surface& s);
[[nodiscard]] std::string_view kind_name(SurfaceKind k);
[[nodiscard]] double signed_distance(const Surface& s, const Vec3& p);
// Unit gradient of the signed distance at p (the surface normal at p's projection).
[[nodiscard]] Vec3 normal_at(const Surface& s, const Vec3& p);
// Closest point on the surface.
[[nodiscard]] Vec3 project(const Surface& s, const Vec3& p);
// Rigid motion of a surface: x' = T x.
[[nodiscard]] Surface transformed(const Surface& s, const SE3& T);
// The surface scaled about the origin: x' = k x.
[[nodiscard]] Surface scaled(const Surface& s, double k);

// Number of free parameters of a surface kind, and a small change of them (for fitting and solving):
//   plane     [normal tilt u, normal tilt v, offset]
//   cylinder  [axis tilt u, axis tilt v, axis shift u, axis shift v, radius]
//   cone      [axis tilt u, axis tilt v, apex x, apex y, apex z, half angle]
//   sphere    [centre x, y, z, radius]
//   torus     [axis tilt u, axis tilt v, centre x, y, z, major radius, minor radius]
//   freeform  [] (held as fitted)
// where (u, v) is a basis perpendicular to the current direction (any_perpendicular, and direction x it).
[[nodiscard]] int parameter_count(SurfaceKind kind);
[[nodiscard]] Surface perturbed(const Surface& s, std::span<const double> delta);

// A unit vector perpendicular to n.
[[nodiscard]] Vec3 any_perpendicular(const Vec3& n);

}  // namespace einstar::fit
