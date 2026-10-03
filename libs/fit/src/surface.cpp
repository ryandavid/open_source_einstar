#include "einstar/fit/surface.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace einstar::fit {
namespace {

template <class... Ts>
struct Overloaded : Ts... {
    using Ts::operator()...;
};

// Axial and radial coordinates of p about an axis through `origin`, and the radial unit vector.
struct AxisCoords {
    double h, rho;
    Vec3 radial;
};
AxisCoords axis_coords(const Vec3& origin, const Vec3& axis, const Vec3& p) {
    const Vec3 v = p - origin;
    const double h = v.dot(axis);
    const Vec3 r = v - h * axis;
    const double rho = r.norm();
    return {h, rho, rho > 1e-12 ? Vec3(r / rho) : any_perpendicular(axis)};
}

Vec3 rotate_axis(const Vec3& a, double d0, double d1) {
    const Vec3 u = any_perpendicular(a), v = a.cross(u);
    return (a + d0 * u + d1 * v).normalized();
}

}  // namespace

int parameter_count(SurfaceKind kind) {
    switch (kind) {
        case SurfaceKind::plane: return 3;
        case SurfaceKind::cylinder: return 5;
        case SurfaceKind::cone: return 6;
        case SurfaceKind::sphere: return 4;
        case SurfaceKind::torus: return 7;
        case SurfaceKind::freeform: return 0;
    }
    return 0;
}

Surface perturbed(const Surface& s, std::span<const double> d) {
    return std::visit(Overloaded{
                          [&](const Plane& p) -> Surface { return Plane{rotate_axis(p.normal, d[0], d[1]), p.offset + d[2]}; },
                          [&](const Cylinder& c) -> Surface {
                              const Vec3 u = any_perpendicular(c.axis), v = c.axis.cross(u);
                              return Cylinder{c.point + d[2] * u + d[3] * v, rotate_axis(c.axis, d[0], d[1]), c.radius + d[4]};
                          },
                          [&](const Cone& c) -> Surface {
                              return Cone{c.apex + Vec3(d[2], d[3], d[4]), rotate_axis(c.axis, d[0], d[1]),
                                          std::clamp(c.half_angle + d[5], 1e-4, std::numbers::pi / 2 - 1e-4)};
                          },
                          [&](const Sphere& sp) -> Surface { return Sphere{sp.center + Vec3(d[0], d[1], d[2]), sp.radius + d[3]}; },
                          [&](const Torus& t) -> Surface {
                              return Torus{t.center + Vec3(d[2], d[3], d[4]), rotate_axis(t.axis, d[0], d[1]), t.major + d[5], t.minor + d[6]};
                          },
                          [&](const Freeform& f) -> Surface { return f; },
                      },
                      s);
}

double Freeform::height(double x, double y, Vec2* gradient) const {
    // Uniform cubic B-spline basis on the cell holding (x, y), and its derivative.
    const auto basis = [](double t, std::array<double, 4>& b, std::array<double, 4>& db) {
        const double t2 = t * t, t3 = t2 * t, s = 1 - t;
        b = {s * s * s / 6, (3 * t3 - 6 * t2 + 4) / 6, (-3 * t3 + 3 * t2 + 3 * t + 1) / 6, t3 / 6};
        db = {-s * s / 2, (9 * t2 - 12 * t) / 6, (-9 * t2 + 6 * t + 3) / 6, t2 / 2};
    };
    const double tu = std::clamp((x - u0) / du, 0.0, static_cast<double>(nu - 3)), tv = std::clamp((y - v0) / dv, 0.0, static_cast<double>(nv - 3));
    const int cu = std::min(static_cast<int>(tu), nu - 4), cv = std::min(static_cast<int>(tv), nv - 4);
    std::array<double, 4> bu, dbu, bv, dbv;
    basis(tu - cu, bu, dbu);
    basis(tv - cv, bv, dbv);
    double h = 0, hx = 0, hy = 0;
    const auto& H = *heights;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            const double c = H[static_cast<std::size_t>((cu + i) * nv + cv + j)];
            h += bu[static_cast<std::size_t>(i)] * bv[static_cast<std::size_t>(j)] * c;
            hx += dbu[static_cast<std::size_t>(i)] * bv[static_cast<std::size_t>(j)] * c;
            hy += bu[static_cast<std::size_t>(i)] * dbv[static_cast<std::size_t>(j)] * c;
        }
    if (gradient) *gradient = Vec2(hx / du, hy / dv);
    return h;
}

Surface scaled(const Surface& s, double k) {
    return std::visit(Overloaded{
                          [&](const Plane& pl) -> Surface { return Plane{pl.normal, k * pl.offset}; },
                          [&](const Cylinder& c) -> Surface { return Cylinder{k * c.point, c.axis, k * c.radius}; },
                          [&](const Cone& c) -> Surface { return Cone{k * c.apex, c.axis, c.half_angle}; },
                          [&](const Sphere& sp) -> Surface { return Sphere{k * sp.center, k * sp.radius}; },
                          [&](const Torus& t) -> Surface { return Torus{k * t.center, t.axis, k * t.major, k * t.minor}; },
                          [&](const Freeform& f) -> Surface {
                              Freeform out = f;
                              out.frame.translation() *= k;
                              out.u0 *= k, out.v0 *= k, out.du *= k, out.dv *= k;
                              auto h = std::make_shared<std::vector<double>>(*f.heights);
                              for (double& x : *h) x *= k;
                              out.heights = std::move(h);
                              return out;
                          },
                      },
                      s);
}

Vec3 any_perpendicular(const Vec3& n) {
    const Vec3 a = std::abs(n.x()) < 0.9 ? Vec3::UnitX() : Vec3::UnitY();
    return n.cross(a).normalized();
}

SurfaceKind kind_of(const Surface& s) { return static_cast<SurfaceKind>(s.index()); }

std::string_view kind_name(SurfaceKind k) {
    switch (k) {
        case SurfaceKind::plane: return "plane";
        case SurfaceKind::cylinder: return "cylinder";
        case SurfaceKind::cone: return "cone";
        case SurfaceKind::sphere: return "sphere";
        case SurfaceKind::torus: return "torus";
        case SurfaceKind::freeform: return "freeform";
    }
    return "?";
}

double signed_distance(const Surface& s, const Vec3& p) {
    return std::visit(Overloaded{
                          [&](const Plane& pl) { return pl.normal.dot(p) - pl.offset; },
                          [&](const Cylinder& c) { return axis_coords(c.point, c.axis, p).rho - c.radius; },
                          [&](const Cone& c) {
                              const AxisCoords a = axis_coords(c.apex, c.axis, p);
                              return a.rho * std::cos(c.half_angle) - a.h * std::sin(c.half_angle);
                          },
                          [&](const Sphere& sp) { return (p - sp.center).norm() - sp.radius; },
                          [&](const Torus& t) {
                              const AxisCoords a = axis_coords(t.center, t.axis, p);
                              return std::hypot(a.rho - t.major, a.h) - t.minor;
                          },
                          [&](const Freeform& f) {
                              // Height above the surface, scaled to the distance along its normal (exact at the
                              // surface, first-order near it).
                              const Vec3 q = f.frame.inverse() * p;
                              Vec2 g;
                              const double h = f.height(q.x(), q.y(), &g);
                              return (q.z() - h) / std::sqrt(1 + g.squaredNorm());
                          },
                      },
                      s);
}

Vec3 normal_at(const Surface& s, const Vec3& p) {
    return std::visit(Overloaded{
                          [&](const Plane& pl) { return pl.normal; },
                          [&](const Cylinder& c) { return axis_coords(c.point, c.axis, p).radial; },
                          [&](const Cone& c) {
                              const AxisCoords a = axis_coords(c.apex, c.axis, p);
                              return Vec3(std::cos(c.half_angle) * a.radial - std::sin(c.half_angle) * c.axis);
                          },
                          [&](const Sphere& sp) {
                              const Vec3 d = p - sp.center;
                              const double n = d.norm();
                              return n > 1e-12 ? Vec3(d / n) : Vec3(Vec3::UnitZ());
                          },
                          [&](const Torus& t) {
                              const AxisCoords a = axis_coords(t.center, t.axis, p);
                              const Vec3 q = (a.rho - t.major) * a.radial + a.h * t.axis;
                              const double n = q.norm();
                              return n > 1e-12 ? Vec3(q / n) : a.radial;
                          },
                          [&](const Freeform& f) {
                              const Vec3 q = f.frame.inverse() * p;
                              Vec2 g;
                              (void)f.height(q.x(), q.y(), &g);
                              return Vec3(f.frame.linear() * Vec3(-g.x(), -g.y(), 1.0).normalized());
                          },
                      },
                      s);
}

Vec3 project(const Surface& s, const Vec3& p) { return p - signed_distance(s, p) * normal_at(s, p); }

Surface transformed(const Surface& s, const SE3& T) {
    const Mat3 R = T.linear();
    return std::visit(Overloaded{
                          [&](const Plane& pl) -> Surface {
                              const Vec3 n = R * pl.normal;
                              return Plane{n, pl.offset + n.dot(T.translation())};
                          },
                          [&](const Cylinder& c) -> Surface { return Cylinder{T * c.point, R * c.axis, c.radius}; },
                          [&](const Cone& c) -> Surface { return Cone{T * c.apex, R * c.axis, c.half_angle}; },
                          [&](const Sphere& sp) -> Surface { return Sphere{T * sp.center, sp.radius}; },
                          [&](const Torus& t) -> Surface { return Torus{T * t.center, R * t.axis, t.major, t.minor}; },
                          [&](const Freeform& f) -> Surface {
                              Freeform out = f;
                              out.frame = T * f.frame;
                              return out;
                          },
                      },
                      s);
}

}  // namespace einstar::fit
