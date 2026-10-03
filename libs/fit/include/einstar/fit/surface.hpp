#pragma once

// Analytic surfaces fitted to scan regions: the faces a CAD model is made of.

#include <string_view>
#include <variant>

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

using Surface = std::variant<Plane, Cylinder, Cone, Sphere, Torus>;

enum class SurfaceKind { plane, cylinder, cone, sphere, torus };

[[nodiscard]] SurfaceKind kind_of(const Surface& s);
[[nodiscard]] std::string_view kind_name(SurfaceKind k);
[[nodiscard]] double signed_distance(const Surface& s, const Vec3& p);
// Unit gradient of the signed distance at p (the surface normal at p's projection).
[[nodiscard]] Vec3 normal_at(const Surface& s, const Vec3& p);
// Closest point on the surface.
[[nodiscard]] Vec3 project(const Surface& s, const Vec3& p);
// Rigid motion of a surface: x' = T x.
[[nodiscard]] Surface transformed(const Surface& s, const SE3& T);

// A unit vector perpendicular to n.
[[nodiscard]] Vec3 any_perpendicular(const Vec3& n);

}  // namespace einstar::fit
