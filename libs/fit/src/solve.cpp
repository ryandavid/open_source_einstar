#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>

#include <Eigen/Dense>

#include "einstar/fit/constraints.hpp"
#include "einstar/fit/primitive_fit.hpp"

namespace einstar::fit {
namespace {

using VecX = Eigen::VectorXd;
using MatX = Eigen::MatrixXd;

template <class... Ts>
struct Overloaded : Ts... {
    using Ts::operator()...;
};

constexpr std::array<char, 3> kAxis{'x', 'y', 'z'};

struct State {
    std::vector<Surface> surfaces;
    std::vector<SE3> datums;
};

// Where each free feature's and datum's parameters sit in the solve's vector (-1: fixed).
struct Layout {
    std::vector<int> feature_offset, feature_size, datum_offset;
    int n = 0;
};

Layout make_layout(const SolveInput& in) {
    Layout L;
    for (const auto& f : in.features) {
        const int k = parameter_count(kind_of(f.surface));
        L.feature_offset.push_back(f.fixed ? -1 : L.n);
        L.feature_size.push_back(k);
        if (!f.fixed) L.n += k;
    }
    for (const auto& d : in.datums) {
        L.datum_offset.push_back(d.fixed ? -1 : L.n);
        if (!d.fixed) L.n += 6;
    }
    return L;
}

void apply_feature(State& s, const Layout& L, std::size_t f, const double* d) {
    s.surfaces[f] = perturbed(s.surfaces[f], std::span<const double>(d, static_cast<std::size_t>(L.feature_size[f])));
}
void apply_datum(State& s, std::size_t k, const double* d) {
    SE3& T = s.datums[k];
    T.linear() = T.linear() * so3_exp(Vec3(d[0], d[1], d[2]));
    T.translation() += Vec3(d[3], d[4], d[5]);
}

State apply(const State& s, const Layout& L, const VecX& dx) {
    State out = s;
    for (std::size_t f = 0; f < s.surfaces.size(); ++f)
        if (L.feature_offset[f] >= 0) apply_feature(out, L, f, dx.data() + L.feature_offset[f]);
    for (std::size_t k = 0; k < s.datums.size(); ++k)
        if (L.datum_offset[k] >= 0) apply_datum(out, k, dx.data() + L.datum_offset[k]);
    return out;
}

// --- constraints -----------------------------------------------------------------------------------

std::optional<int> radius_index(SurfaceKind k) {
    switch (k) {
        case SurfaceKind::cylinder: return 4;
        case SurfaceKind::sphere: return 3;
        case SurfaceKind::torus: return 6;
        case SurfaceKind::plane:
        case SurfaceKind::cone:
        case SurfaceKind::freeform: return std::nullopt;
    }
    return std::nullopt;
}

double radius_of(const Surface& s) {
    return std::visit(Overloaded{
                          [](const Cylinder& c) { return c.radius; },
                          [](const Sphere& sp) { return sp.radius; },
                          [](const Torus& t) { return t.minor; },
                          [](const auto&) { return 0.0; },
                      },
                      s);
}

int rows_of(const Constraint& c) {
    return std::visit(Overloaded{
                          [](const Aligned&) { return 2; },
                          [](const Parallel&) { return 2; },
                          [](const Perpendicular&) { return 1; },
                          [](const Angle&) { return 1; },
                          [](const Coplanar&) { return 3; },
                          [](const Coaxial&) { return 4; },
                          [](const Radius&) { return 1; },
                          [](const Diameter&) { return 1; },
                          [](const Distance&) { return 3; },
                          [](const Offset&) { return 1; },
                          [](const AxisDistance&) { return 3; },
                          [](const Tangent&) { return 3; },
                          [](const Symmetric&) { return 1; },
                          [](const EqualRadius&) { return 1; },
                      },
                      c);
}

struct Involved {
    std::vector<int> features, datums;
};
Involved involved(const Constraint& c) {
    return std::visit(Overloaded{
                          [](const Aligned& a) { return Involved{{a.feature}, {a.datum}}; },
                          [](const Offset& o) { return Involved{{o.feature}, {o.datum}}; },
                          [](const Symmetric& o) { return Involved{{o.a, o.b}, {o.datum}}; },
                          [](const Radius& r) { return Involved{{r.feature}, {}}; },
                          [](const Diameter& d) { return Involved{{d.feature}, {}}; },
                          [](const auto& pair) { return Involved{{pair.a, pair.b}, {}}; },
                      },
                      c);
}

// Validity against the features' kinds; empty if valid.
std::string check(const Constraint& c, const SolveInput& in) {
    const auto feature_ok = [&](int f) { return f >= 0 && static_cast<std::size_t>(f) < in.features.size(); };
    const auto datum_ok = [&](int d) { return d >= 0 && static_cast<std::size_t>(d) < in.datums.size(); };
    const Involved inv = involved(c);
    for (const int f : inv.features)
        if (!feature_ok(f)) return std::format("no feature {}", f);
    for (const int d : inv.datums)
        if (!datum_ok(d)) return std::format("no datum {}", d);
    const auto kind = [&](int f) { return kind_of(in.features[static_cast<std::size_t>(f)].surface); };
    const auto has_direction = [&](int f) { return kind(f) != SurfaceKind::sphere && kind(f) != SurfaceKind::freeform; };
    const auto is_plane = [&](int f) { return kind(f) == SurfaceKind::plane; };
    return std::visit(Overloaded{
                          [&](const Aligned& a) -> std::string {
                              if (a.axis < 0 || a.axis > 2) return "axis must be 0, 1 or 2";
                              return has_direction(a.feature) ? "" : "a sphere or freeform face has no direction";
                          },
                          [&](const Offset& o) -> std::string {
                              if (o.axis < 0 || o.axis > 2) return "axis must be 0, 1 or 2";
                              return kind(o.feature) == SurfaceKind::freeform ? "a freeform face has no position to offset" : "";
                          },
                          [&](const Parallel& p) -> std::string {
                              return has_direction(p.a) && has_direction(p.b) ? "" : "a sphere or freeform face has no direction";
                          },
                          [&](const Perpendicular& p) -> std::string {
                              return has_direction(p.a) && has_direction(p.b) ? "" : "a sphere or freeform face has no direction";
                          },
                          [&](const Angle& p) -> std::string {
                              if (p.degrees < 0 || p.degrees > 90) return "angle must be within 0..90 degrees";
                              return has_direction(p.a) && has_direction(p.b) ? "" : "a sphere or freeform face has no direction";
                          },
                          [&](const Coplanar& p) -> std::string { return is_plane(p.a) && is_plane(p.b) ? "" : "coplanar needs two planes"; },
                          [&](const Coaxial& p) -> std::string {
                              return has_direction(p.a) && has_direction(p.b) && !is_plane(p.a) && !is_plane(p.b) ? ""
                                                                                                                    : "coaxial needs two axes";
                          },
                          [&](const Radius& r) -> std::string {
                              return radius_index(kind(r.feature)) ? "" : "only cylinders, spheres and tori have a radius";
                          },
                          [&](const Diameter& d) -> std::string {
                              return radius_index(kind(d.feature)) ? "" : "only cylinders, spheres and tori have a diameter";
                          },
                          [&](const Distance& d) -> std::string { return is_plane(d.a) && is_plane(d.b) ? "" : "distance needs two planes"; },
                          [&](const Tangent& t) -> std::string {
                              const auto ka = kind(t.a), kb = kind(t.b);
                              const auto one = [&](SurfaceKind x, SurfaceKind y) { return (ka == x && kb == y) || (ka == y && kb == x); };
                              if (one(SurfaceKind::plane, SurfaceKind::cylinder) || one(SurfaceKind::plane, SurfaceKind::sphere) ||
                                  one(SurfaceKind::cylinder, SurfaceKind::cylinder))
                                  return "";
                              return "tangent needs a plane and a cylinder or sphere, or two cylinders";
                          },
                          [&](const Symmetric& o) -> std::string {
                              if (o.axis < 0 || o.axis > 2) return "axis must be 0, 1 or 2";
                              if (kind(o.a) != kind(o.b)) return "symmetric needs two faces or axes of the same kind";
                              return kind(o.a) == SurfaceKind::freeform ? "a freeform face has no position" : "";
                          },
                          [&](const EqualRadius& e) -> std::string {
                              return radius_index(kind(e.a)) && radius_index(kind(e.b)) ? "" : "equal radius needs cylinders, spheres or tori";
                          },
                          [&](const AxisDistance& d) -> std::string {
                              if (!(d.value > 0)) return "the distance must be positive";
                              return has_direction(d.a) && has_direction(d.b) && !is_plane(d.a) && !is_plane(d.b) ? ""
                                                                                                                  : "axis distance needs two axes";
                          },
                      },
                      c);
}

// Quantities held fixed during one linearisation: a basis perpendicular to a reference direction (to
// express "parallel" as two rows) and the sense of signed relations.
struct Frozen {
    Vec3 b1 = Vec3::UnitX(), b2 = Vec3::UnitY();
    double sign = 1;
};

Vec3 direction(const State& s, int f) { return *direction_of(s.surfaces[static_cast<std::size_t>(f)]); }
Vec3 position(const State& s, int f) { return position_of(s.surfaces[static_cast<std::size_t>(f)]); }
Vec3 datum_axis(const State& s, int d, int k) { return s.datums[static_cast<std::size_t>(d)].linear().col(k); }

// Where a face or axis is along a datum axis, from the datum origin: a plane where the axis line meets it, an axis
// (or centre) by its point's projection.
double along(const State& s, int f, int datum, int axis) {
    const SE3& T = s.datums[static_cast<std::size_t>(datum)];
    const Vec3 e = T.linear().col(axis);
    if (const auto* pl = std::get_if<Plane>(&s.surfaces[static_cast<std::size_t>(f)])) {
        const double ne = pl->normal.dot(e);
        if (std::abs(ne) > 1e-6) return (pl->offset - pl->normal.dot(T.translation())) / ne;
    }
    return (position(s, f) - T.translation()).dot(e);
}

Frozen freeze(const Constraint& c, const State& s) {
    Frozen fz;
    const auto basis = [&](const Vec3& e) {
        fz.b1 = any_perpendicular(e);
        fz.b2 = e.cross(fz.b1);
    };
    std::visit(Overloaded{
                   [&](const Aligned& a) { basis(datum_axis(s, a.datum, a.axis)); },
                   [&](const Parallel& p) { basis(direction(s, p.b)); },
                   [&](const Coplanar& p) { basis(direction(s, p.b)); },
                   [&](const Coaxial& p) { basis(direction(s, p.a)); },
                   [&](const AxisDistance& p) { basis(direction(s, p.a)); },
                   [&](const Tangent& t) {
                       const Surface& a = s.surfaces[static_cast<std::size_t>(t.a)];
                       const Surface& b = s.surfaces[static_cast<std::size_t>(t.b)];
                       if (std::holds_alternative<Cylinder>(a) && std::holds_alternative<Cylinder>(b)) {
                           basis(direction(s, t.a));
                           const auto& ca = std::get<Cylinder>(a);
                           const auto& cb = std::get<Cylinder>(b);
                           Vec3 dd = cb.point - ca.point;
                           dd -= dd.dot(ca.axis) * ca.axis;
                           // Touching outside (centres r_a + r_b apart) or inside (|r_a - r_b|): whichever is nearer now.
                           fz.sign = std::abs(dd.norm() - (ca.radius + cb.radius)) <= std::abs(dd.norm() - std::abs(ca.radius - cb.radius)) ? 1 : -1;
                       } else {
                           const bool plane_first = std::holds_alternative<Plane>(a);
                           const auto& pl = std::get<Plane>(plane_first ? a : b);
                           const Surface& other = plane_first ? b : a;
                           fz.sign = pl.normal.dot(position_of(other)) - pl.offset >= 0 ? 1 : -1;
                       }
                   },
                   [&](const Distance& d) {
                       basis(direction(s, d.b));
                       const auto& pa = std::get<Plane>(s.surfaces[static_cast<std::size_t>(d.a)]);
                       fz.sign = pa.normal.dot(position(s, d.b)) - pa.offset >= 0 ? 1 : -1;
                   },
                   [&](const Angle& a) { fz.sign = direction(s, a.a).dot(direction(s, a.b)) >= 0 ? 1 : -1; },
                   [](const auto&) {},
               },
               c);
    return fz;
}

void evaluate(const Constraint& c, const State& s, const Frozen& fz, double* out) {
    const auto parallel_rows = [&](const Vec3& u, const Vec3& v) {
        const Vec3 x = u.cross(v);
        out[0] = x.dot(fz.b1);
        out[1] = x.dot(fz.b2);
    };
    std::visit(Overloaded{
                   [&](const Aligned& a) { parallel_rows(direction(s, a.feature), datum_axis(s, a.datum, a.axis)); },
                   [&](const Parallel& p) { parallel_rows(direction(s, p.a), direction(s, p.b)); },
                   [&](const Perpendicular& p) { out[0] = direction(s, p.a).dot(direction(s, p.b)); },
                   [&](const Angle& a) {
                       out[0] = fz.sign * direction(s, a.a).dot(direction(s, a.b)) - std::cos(a.degrees * std::numbers::pi / 180.0);
                   },
                   [&](const Coplanar& p) {
                       parallel_rows(direction(s, p.a), direction(s, p.b));
                       const auto& pa = std::get<Plane>(s.surfaces[static_cast<std::size_t>(p.a)]);
                       out[2] = pa.normal.dot(position(s, p.b)) - pa.offset;
                   },
                   [&](const Coaxial& p) {
                       const Vec3 ua = direction(s, p.a);
                       parallel_rows(direction(s, p.b), ua);
                       Vec3 d = position(s, p.b) - position(s, p.a);
                       d -= d.dot(ua) * ua;
                       out[2] = d.dot(fz.b1);
                       out[3] = d.dot(fz.b2);
                   },
                   [&](const Radius& r) { out[0] = radius_of(s.surfaces[static_cast<std::size_t>(r.feature)]) - r.value; },
                   [&](const Diameter& d) { out[0] = 2 * radius_of(s.surfaces[static_cast<std::size_t>(d.feature)]) - d.value; },
                   [&](const Distance& d) {
                       parallel_rows(direction(s, d.a), direction(s, d.b));
                       const auto& pa = std::get<Plane>(s.surfaces[static_cast<std::size_t>(d.a)]);
                       out[2] = pa.normal.dot(position(s, d.b)) - pa.offset - fz.sign * d.value;
                   },
                   [&](const AxisDistance& p) {
                       const Vec3 ua = direction(s, p.a);
                       parallel_rows(direction(s, p.b), ua);
                       Vec3 d = position(s, p.b) - position(s, p.a);
                       d -= d.dot(ua) * ua;
                       out[2] = d.norm() - p.value;
                   },
                   [&](const Offset& o) { out[0] = along(s, o.feature, o.datum, o.axis) - o.value; },
                   [&](const Symmetric& o) { out[0] = along(s, o.a, o.datum, o.axis) + along(s, o.b, o.datum, o.axis); },
                   [&](const EqualRadius& e) {
                       out[0] = radius_of(s.surfaces[static_cast<std::size_t>(e.a)]) - radius_of(s.surfaces[static_cast<std::size_t>(e.b)]);
                   },
                   [&](const Tangent& t) {
                       const Surface& a = s.surfaces[static_cast<std::size_t>(t.a)];
                       const Surface& b = s.surfaces[static_cast<std::size_t>(t.b)];
                       out[0] = out[1] = out[2] = 0;  // rows a kind of pair does not use stay zero
                       if (std::holds_alternative<Cylinder>(a) && std::holds_alternative<Cylinder>(b)) {
                           const auto& ca = std::get<Cylinder>(a);
                           const auto& cb = std::get<Cylinder>(b);
                           parallel_rows(cb.axis, ca.axis);
                           Vec3 dd = cb.point - ca.point;
                           dd -= dd.dot(ca.axis) * ca.axis;
                           out[2] = dd.norm() - (fz.sign > 0 ? ca.radius + cb.radius : std::abs(ca.radius - cb.radius));
                           return;
                       }
                       const bool plane_first = std::holds_alternative<Plane>(a);
                       const auto& pl = std::get<Plane>(plane_first ? a : b);
                       const Surface& other = plane_first ? b : a;
                       out[0] = fz.sign * (pl.normal.dot(position_of(other)) - pl.offset) - radius_of(other);
                       if (const auto* cyl = std::get_if<Cylinder>(&other)) out[1] = pl.normal.dot(cyl->axis);  // the axis along the plane
                   },
               },
               c);
}

// --- the solve -------------------------------------------------------------------------------------

struct Support {
    std::vector<Vec3> points;
    std::vector<double> weights;
    double sigma = 0;
};

std::vector<Support> make_support(const SolveInput& in, const SolveOptions& o) {
    std::vector<Support> out(in.features.size());
    for (std::size_t f = 0; f < in.features.size(); ++f) {
        const Feature& feat = in.features[f];
        const std::size_t n = feat.points.size();
        const std::size_t stride = n > o.max_points ? (n + o.max_points - 1) / o.max_points : 1;
        std::vector<double> res;
        for (std::size_t i = 0; i < n; i += stride) {
            out[f].points.push_back(feat.points[i]);
            out[f].weights.push_back(feat.weights.empty() ? 1.0 : feat.weights[i]);
            res.push_back(signed_distance(feat.surface, feat.points[i]));
        }
        // Weights are normalised to a mean of 1, so a feature counts by its number of points.
        double sum = 0;
        for (const double w : out[f].weights) sum += w;
        if (sum > 0)
            for (double& w : out[f].weights) w *= static_cast<double>(out[f].weights.size()) / sum;
        out[f].sigma = std::max(robust_sigma(res), o.min_sigma);
    }
    return out;
}

struct Problem {
    const SolveInput& in;
    const SolveOptions& o;
    const Layout& L;
    const std::vector<Support>& support;
    std::vector<std::size_t> active;  // constraints taking part (valid, not left out)
};

struct Linearisation {
    MatX H;
    VecX g;
    double cost = 0;
    MatX C;     // rows of the active constraints
    VecX c;
    std::vector<std::size_t> row_constraint;  // constraint index of each row
};

double huber_cost(double r, double cut) { return std::abs(r) <= cut ? 0.5 * r * r : cut * std::abs(r) - 0.5 * cut * cut; }

double data_cost(const Problem& P, const State& s) {
    double cost = 0;
    for (std::size_t f = 0; f < s.surfaces.size(); ++f) {
        const Support& sp = P.support[f];
        const double cut = P.o.huber * sp.sigma, inv = 1.0 / (sp.sigma * sp.sigma);
        for (std::size_t i = 0; i < sp.points.size(); ++i) cost += sp.weights[i] * inv * huber_cost(signed_distance(s.surfaces[f], sp.points[i]), cut);
    }
    return cost;
}

std::vector<Frozen> freeze_all(const Problem& P, const State& s) {
    std::vector<Frozen> fz(P.in.constraints.size());
    for (const auto ci : P.active) fz[ci] = freeze(P.in.constraints[ci], s);
    return fz;
}

VecX constraint_values(const Problem& P, const State& s, const std::vector<Frozen>& fz) {
    int m = 0;
    for (const auto ci : P.active) m += rows_of(P.in.constraints[ci]);
    VecX c(m);
    int row = 0;
    for (const auto ci : P.active) {
        evaluate(P.in.constraints[ci], s, fz[ci], c.data() + row);
        row += rows_of(P.in.constraints[ci]);
    }
    return c;
}

Linearisation linearise(const Problem& P, const State& s, const std::vector<Frozen>& fz) {
    const int n = P.L.n;
    Linearisation lin;
    lin.H = MatX::Zero(n, n);
    lin.g = VecX::Zero(n);
    constexpr double kStep = 1e-7;

    for (std::size_t f = 0; f < s.surfaces.size(); ++f) {
        const int off = P.L.feature_offset[f], k = P.L.feature_size[f];
        const Support& sp = P.support[f];
        const double cut = P.o.huber * sp.sigma, inv = 1.0 / (sp.sigma * sp.sigma);
        std::vector<double> r(sp.points.size());
        for (std::size_t i = 0; i < r.size(); ++i) {
            r[i] = signed_distance(s.surfaces[f], sp.points[i]);
            lin.cost += sp.weights[i] * inv * huber_cost(r[i], cut);
        }
        if (off < 0) continue;
        MatX J(static_cast<Eigen::Index>(r.size()), k);
        for (int j = 0; j < k; ++j) {
            std::array<double, 7> d{};
            d[static_cast<std::size_t>(j)] = kStep;
            const Surface sj = perturbed(s.surfaces[f], std::span<const double>(d.data(), static_cast<std::size_t>(k)));
            for (std::size_t i = 0; i < r.size(); ++i) J(static_cast<Eigen::Index>(i), j) = (signed_distance(sj, sp.points[i]) - r[i]) / kStep;
        }
        for (std::size_t i = 0; i < r.size(); ++i) {
            const double w = sp.weights[i] * inv * (std::abs(r[i]) <= cut ? 1.0 : cut / std::abs(r[i]));
            const auto row = J.row(static_cast<Eigen::Index>(i));
            lin.H.block(off, off, k, k).noalias() += w * row.transpose() * row;
            lin.g.segment(off, k).noalias() += w * row.transpose() * r[i];
        }
    }

    lin.c = constraint_values(P, s, fz);
    lin.C = MatX::Zero(lin.c.size(), n);
    int row = 0;
    for (const auto ci : P.active) {
        const Constraint& con = P.in.constraints[ci];
        const int rows = rows_of(con);
        for (int i = 0; i < rows; ++i) lin.row_constraint.push_back(ci);
        const Involved inv = involved(con);
        std::array<double, 8> plus{}, minus{};
        const auto column = [&](int global, const State& sp, const State& sm) {
            evaluate(con, sp, fz[ci], plus.data());
            evaluate(con, sm, fz[ci], minus.data());
            for (int i = 0; i < rows; ++i) lin.C(row + i, global) = (plus[static_cast<std::size_t>(i)] - minus[static_cast<std::size_t>(i)]) / (2 * kStep);
        };
        std::vector<int> seen;
        for (const int f : inv.features) {
            const int off = P.L.feature_offset[static_cast<std::size_t>(f)];
            if (off < 0 || std::ranges::count(seen, f)) continue;
            seen.push_back(f);
            for (int j = 0; j < P.L.feature_size[static_cast<std::size_t>(f)]; ++j) {
                std::array<double, 7> d{};
                State sp = s, sm = s;
                d[static_cast<std::size_t>(j)] = kStep;
                apply_feature(sp, P.L, static_cast<std::size_t>(f), d.data());
                d[static_cast<std::size_t>(j)] = -kStep;
                apply_feature(sm, P.L, static_cast<std::size_t>(f), d.data());
                column(off + j, sp, sm);
            }
        }
        for (const int dk : inv.datums) {
            const int off = P.L.datum_offset[static_cast<std::size_t>(dk)];
            if (off < 0) continue;
            for (int j = 0; j < 6; ++j) {
                std::array<double, 6> d{};
                State sp = s, sm = s;
                d[static_cast<std::size_t>(j)] = kStep;
                apply_datum(sp, static_cast<std::size_t>(dk), d.data());
                d[static_cast<std::size_t>(j)] = -kStep;
                apply_datum(sm, static_cast<std::size_t>(dk), d.data());
                column(off + j, sp, sm);
            }
        }
        row += rows;
    }
    return lin;
}

// Rows independent of the rows before them (in constraint order), by Gram-Schmidt.
std::vector<bool> independent_rows(const MatX& C) {
    std::vector<bool> keep(static_cast<std::size_t>(C.rows()), false);
    std::vector<VecX> basis;
    for (Eigen::Index i = 0; i < C.rows(); ++i) {
        VecX v = C.row(i).transpose();
        const double norm = v.norm();
        for (int pass = 0; pass < 2; ++pass)
            for (const auto& q : basis) v -= q.dot(v) * q;
        if (v.norm() > 1e-7 * std::max(norm, 1e-12) && norm > 1e-12) {
            basis.push_back(v.normalized());
            keep[static_cast<std::size_t>(i)] = true;
        }
    }
    return keep;
}

struct CoreResult {
    State state;
    bool converged = false;
    int iterations = 0;
    std::vector<bool> row_kept;
    std::vector<std::size_t> row_constraint;
    MatX kkt;  // at the solution, kept rows only
    int kept_rows = 0;
};

CoreResult solve_core(const Problem& P, State s) {
    CoreResult out;
    const int n = P.L.n;
    for (int it = 0; it < P.o.max_iterations; ++it) {
        out.iterations = it + 1;
        const std::vector<Frozen> fz = freeze_all(P, s);
        const Linearisation lin = linearise(P, s, fz);
        out.row_kept = independent_rows(lin.C);
        out.row_constraint = lin.row_constraint;
        std::vector<Eigen::Index> rows;
        for (std::size_t i = 0; i < out.row_kept.size(); ++i)
            if (out.row_kept[i]) rows.push_back(static_cast<Eigen::Index>(i));
        const auto m = static_cast<Eigen::Index>(rows.size());
        MatX K = MatX::Zero(n + m, n + m);
        VecX rhs(n + m);
        K.topLeftCorner(n, n) = lin.H;
        K.topLeftCorner(n, n).diagonal().array() += 1e-9 * (lin.H.diagonal().array() + 1.0);
        for (Eigen::Index i = 0; i < m; ++i) {
            K.block(n + i, 0, 1, n) = lin.C.row(rows[static_cast<std::size_t>(i)]);
            K.block(0, n + i, n, 1) = lin.C.row(rows[static_cast<std::size_t>(i)]).transpose();
            rhs[n + i] = -lin.c[rows[static_cast<std::size_t>(i)]];
        }
        rhs.head(n) = -lin.g;
        out.kkt = K;
        out.kept_rows = static_cast<int>(m);
        const VecX sol = K.partialPivLu().solve(rhs);
        if (!sol.allFinite()) break;
        const VecX dx = sol.head(n);

        // Backtrack if the step makes both the fit and the constraints worse.
        double violation = 0;
        for (const auto r : rows) violation = std::max(violation, std::abs(lin.c[r]));
        double alpha = 1;
        State next = apply(s, P.L, dx);
        for (int b = 0; b < 8; ++b) {
            const VecX cn = constraint_values(P, next, fz);
            double vn = 0;
            for (const auto r : rows) vn = std::max(vn, std::abs(cn[r]));
            if (data_cost(P, next) <= lin.cost * (1 + 1e-12) + 1e-12 || vn < violation) break;
            alpha *= 0.5;
            next = apply(s, P.L, alpha * dx);
        }
        s = std::move(next);
        // Converged once the constraints hold and the step or the cost's change has become negligible (the
        // Jacobians are numeric, so steps do not shrink to zero).
        const double new_cost = data_cost(P, s);
        const bool stalled = std::abs(lin.cost - new_cost) <= 1e-10 * std::max(lin.cost, 1.0);
        if (violation < P.o.tolerance && (alpha * dx.lpNorm<Eigen::Infinity>() < 1e-9 || (stalled && it > 0))) {
            out.converged = true;
            break;
        }
    }
    out.state = std::move(s);
    return out;
}

double rms_of(const Support& sp, const Surface& s) {
    double ss = 0;
    std::size_t n = 0;
    for (const auto& p : sp.points) {
        const double r = signed_distance(s, p);
        if (std::abs(r) < 3 * sp.sigma) {
            ss += r * r;
            ++n;
        }
    }
    return n > 0 ? std::sqrt(ss / static_cast<double>(n)) : 0;
}

double max_move(const Support& sp, const Surface& a, const Surface& b) {
    double m = 0;
    for (const auto& p : sp.points) m = std::max(m, std::abs(signed_distance(a, p) - signed_distance(b, p)));
    return m;
}

// Directions related by the constraints, checked before solving: the solve linearises, and two direction
// constraints that contradict each other (a face along x and along z) are not dependent rows but a pull in
// two directions. Union-find over classes of parallel directions; a datum's three axes are mutually
// perpendicular. Returns, per constraint, why it conflicts with the ones before it (empty: no conflict).
std::vector<std::string> direction_conflicts(const SolveInput& in, const std::vector<std::size_t>& active) {
    const std::size_t nf = in.features.size();
    // Nodes: features, then each datum's three axes.
    std::vector<std::size_t> parent(nf + 3 * in.datums.size());
    for (std::size_t i = 0; i < parent.size(); ++i) parent[i] = i;
    const auto find = [&](std::size_t x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };
    const auto axis_node = [&](int d, int k) { return nf + 3 * static_cast<std::size_t>(d) + static_cast<std::size_t>(k); };
    std::vector<std::pair<std::size_t, std::size_t>> perpendicular;  // node pairs
    for (std::size_t d = 0; d < in.datums.size(); ++d)
        for (int i = 0; i < 3; ++i)
            for (int j = i + 1; j < 3; ++j) perpendicular.emplace_back(axis_node(static_cast<int>(d), i), axis_node(static_cast<int>(d), j));
    const auto are_perpendicular = [&](std::size_t a, std::size_t b) {
        return std::ranges::any_of(perpendicular, [&](const auto& p) {
            const std::size_t x = find(p.first), y = find(p.second);
            return (x == a && y == b) || (x == b && y == a);
        });
    };
    std::vector<std::string> why(in.constraints.size());
    const auto join = [&](std::size_t ci, std::size_t a, std::size_t b) {
        const std::size_t ra = find(a), rb = find(b);
        if (ra == rb) return;
        if (are_perpendicular(ra, rb)) {
            why[ci] = "makes parallel two directions the constraints before it make perpendicular";
            return;
        }
        parent[ra] = rb;
    };
    const auto separate = [&](std::size_t ci, std::size_t a, std::size_t b) {
        if (find(a) == find(b)) {
            why[ci] = "makes perpendicular two directions the constraints before it make parallel";
            return;
        }
        perpendicular.emplace_back(a, b);
    };
    for (const auto ci : active) {
        const auto f = [](int i) { return static_cast<std::size_t>(i); };
        std::visit(Overloaded{
                       [&](const Aligned& a) { join(ci, f(a.feature), axis_node(a.datum, a.axis)); },
                       [&](const Parallel& p) { join(ci, f(p.a), f(p.b)); },
                       [&](const Coplanar& p) { join(ci, f(p.a), f(p.b)); },
                       [&](const Coaxial& p) { join(ci, f(p.a), f(p.b)); },
                       [&](const Distance& d) { join(ci, f(d.a), f(d.b)); },
                       [&](const AxisDistance& d) { join(ci, f(d.a), f(d.b)); },
                       [&](const Tangent& t) {
                           const auto ka = kind_of(in.features[f(t.a)].surface), kb = kind_of(in.features[f(t.b)].surface);
                           if (ka == SurfaceKind::cylinder && kb == SurfaceKind::cylinder) join(ci, f(t.a), f(t.b));
                           else if (ka != SurfaceKind::sphere && kb != SurfaceKind::sphere) separate(ci, f(t.a), f(t.b));  // the axis along the plane
                       },
                       [&](const Perpendicular& p) { separate(ci, f(p.a), f(p.b)); },
                       [&](const Angle& a) {
                           if (a.degrees < 1e-9) join(ci, f(a.a), f(a.b));
                           else if (a.degrees > 90 - 1e-9) separate(ci, f(a.a), f(a.b));
                           else if (find(f(a.a)) == find(f(a.b))) why[ci] = "sets an angle between directions the constraints before it make parallel";
                           else if (are_perpendicular(find(f(a.a)), find(f(a.b)))) why[ci] = "sets an angle between directions the constraints before it make perpendicular";
                       },
                       [](const auto&) {},
                   },
                   in.constraints[ci]);
    }
    return why;
}

}  // namespace

std::optional<Vec3> direction_of(const Surface& s) {
    return std::visit(Overloaded{
                          [](const Plane& p) -> std::optional<Vec3> { return p.normal; },
                          [](const Cylinder& c) -> std::optional<Vec3> { return c.axis; },
                          [](const Cone& c) -> std::optional<Vec3> { return c.axis; },
                          [](const Torus& t) -> std::optional<Vec3> { return t.axis; },
                          [](const Sphere&) -> std::optional<Vec3> { return std::nullopt; },
                          [](const Freeform&) -> std::optional<Vec3> { return std::nullopt; },
                      },
                      s);
}

Vec3 position_of(const Surface& s) {
    return std::visit(Overloaded{
                          [](const Plane& p) { return Vec3(p.offset * p.normal); },
                          [](const Cylinder& c) { return c.point; },
                          [](const Cone& c) { return c.apex; },
                          [](const Sphere& sp) { return sp.center; },
                          [](const Torus& t) { return t.center; },
                          [](const Freeform& f) { return Vec3(f.frame.translation()); },
                      },
                      s);
}

SE3 datum_from(const Surface& a, int first_axis, const Surface& b) {
    const Vec3 za = direction_of(a).value_or(Vec3::UnitZ());
    Vec3 xb = direction_of(b).value_or(any_perpendicular(za));
    xb -= xb.dot(za) * za;
    xb = xb.norm() > 1e-9 ? Vec3(xb.normalized()) : any_perpendicular(za);
    SE3 T = SE3::Identity();
    const int k = std::clamp(first_axis, 0, 2);
    Mat3 R;
    R.col(k) = za;
    R.col((k + 1) % 3) = xb;
    R.col((k + 2) % 3) = za.cross(xb);
    T.linear() = R;
    T.translation() = project(a, position_of(b));
    return T;
}

std::string_view status_name(ConstraintStatus s) {
    switch (s) {
        case ConstraintStatus::satisfied: return "satisfied";
        case ConstraintStatus::redundant: return "redundant";
        case ConstraintStatus::conflict: return "conflict";
        case ConstraintStatus::invalid: return "invalid";
    }
    return "?";
}

std::string describe(const Constraint& c) {
    return std::visit(Overloaded{
                          [](const Aligned& a) { return std::format("feature {} along datum {} {}", a.feature, a.datum, kAxis[static_cast<std::size_t>(std::clamp(a.axis, 0, 2))]); },
                          [](const Parallel& p) { return std::format("features {} and {} parallel", p.a, p.b); },
                          [](const Perpendicular& p) { return std::format("features {} and {} perpendicular", p.a, p.b); },
                          [](const Angle& p) { return std::format("features {} and {} at {:.3f} deg", p.a, p.b, p.degrees); },
                          [](const Coplanar& p) { return std::format("features {} and {} coplanar", p.a, p.b); },
                          [](const Coaxial& p) { return std::format("features {} and {} coaxial", p.a, p.b); },
                          [](const Radius& r) { return std::format("feature {} radius {:.4f}", r.feature, r.value); },
                          [](const Diameter& d) { return std::format("feature {} diameter {:.4f}", d.feature, d.value); },
                          [](const Distance& d) { return std::format("features {} and {} {:.4f} apart", d.a, d.b, d.value); },
                          [](const AxisDistance& d) { return std::format("axes of features {} and {} {:.4f} apart", d.a, d.b, d.value); },
                          [](const Tangent& t) { return std::format("features {} and {} tangent", t.a, t.b); },
                          [](const Symmetric& o) {
                              return std::format("features {} and {} symmetric about datum {} {}", o.a, o.b, o.datum, kAxis[static_cast<std::size_t>(std::clamp(o.axis, 0, 2))]);
                          },
                          [](const EqualRadius& e) { return std::format("features {} and {} of equal radius", e.a, e.b); },
                          [](const Offset& o) {
                              return std::format("feature {} at {} {:.4f} from datum {}", o.feature, kAxis[static_cast<std::size_t>(std::clamp(o.axis, 0, 2))], o.value, o.datum);
                          },
                      },
                      c);
}

SolveResult solve(const SolveInput& in, const SolveOptions& o) {
    SolveResult result;
    const Layout L = make_layout(in);
    const std::vector<Support> support = make_support(in, o);
    result.constraints.resize(in.constraints.size());

    Problem P{in, o, L, support, {}};
    for (std::size_t i = 0; i < in.constraints.size(); ++i) {
        const std::string why = check(in.constraints[i], in);
        if (!why.empty()) {
            result.constraints[i].status = ConstraintStatus::invalid;
            result.constraints[i].message = why;
        } else {
            P.active.push_back(i);
        }
    }
    const std::vector<std::string> conflicts = direction_conflicts(in, P.active);
    std::erase_if(P.active, [&](std::size_t i) {
        if (conflicts[i].empty()) return false;
        result.constraints[i].status = ConstraintStatus::conflict;
        result.constraints[i].message = conflicts[i];
        return true;
    });

    State start;
    for (const auto& f : in.features) start.surfaces.push_back(f.surface);
    for (const auto& d : in.datums) start.datums.push_back(d.frame);
    CoreResult core = solve_core(P, start);
    result.converged = core.converged;
    result.iterations = core.iterations;
    result.surfaces = core.state.surfaces;
    result.datums = core.state.datums;

    // Constraint status from the final rows: violation, and whether any row added something new.
    const std::vector<Frozen> fz = freeze_all(P, core.state);
    const VecX c = constraint_values(P, core.state, fz);
    {
        std::size_t row = 0;
        for (const auto ci : P.active) {
            const int rows = rows_of(in.constraints[ci]);
            bool any_kept = false;
            double violation = 0;
            for (int r = 0; r < rows; ++r, ++row) {
                any_kept = any_kept || (row < core.row_kept.size() && core.row_kept[row]);
                violation = std::max(violation, std::abs(c[static_cast<Eigen::Index>(row)]));
            }
            ConstraintReport& rep = result.constraints[ci];
            rep.violation = violation;
            if (violation > 1e-6) {
                rep.status = ConstraintStatus::conflict;
                rep.message = "contradicts the constraints before it";
            } else if (!any_kept) {
                rep.status = ConstraintStatus::redundant;
                rep.message = "already implied by the constraints before it";
            }
        }
    }

    // Per feature: fit, movement from the starting surface, and uncertainty under the constraints.
    const auto lu = core.kkt.size() > 0 ? std::optional(core.kkt.partialPivLu()) : std::nullopt;
    const auto variance = [&](int index) {
        VecX e = VecX::Zero(core.kkt.rows());
        e[index] = 1;
        return std::max(0.0, (lu->solve(e))[index]);
    };
    result.features.resize(in.features.size());
    for (std::size_t f = 0; f < in.features.size(); ++f) {
        FeatureReport& fr = result.features[f];
        fr.sigma = support[f].sigma;
        fr.rms = rms_of(support[f], result.surfaces[f]);
        fr.max_move_mm = max_move(support[f], result.surfaces[f], in.features[f].surface);
        const int off = L.feature_offset[f];
        if (off < 0 || !lu) continue;
        const SurfaceKind k = kind_of(result.surfaces[f]);
        if (const auto ri = radius_index(k)) fr.radius_sd = std::sqrt(variance(off + *ri));
        if (k != SurfaceKind::sphere) fr.direction_sd_deg = std::sqrt(std::max(variance(off), variance(off + 1))) * 180.0 / std::numbers::pi;
    }

    // What each constraint costs: the solve without it, from this solution.
    if (o.analyse_costs) {
        for (const auto ci : P.active) {
            ConstraintReport& rep = result.constraints[ci];
            if (rep.status != ConstraintStatus::satisfied) continue;
            Problem without = P;
            std::erase(without.active, ci);
            const CoreResult alt = solve_core(without, core.state);
            for (std::size_t f = 0; f < in.features.size(); ++f)
                rep.max_move_mm = std::max(rep.max_move_mm, max_move(support[f], core.state.surfaces[f], alt.state.surfaces[f]));
            double with_ss = 0, without_ss = 0;
            for (const int f : involved(in.constraints[ci]).features) {
                const auto fi = static_cast<std::size_t>(f);
                with_ss += std::pow(rms_of(support[fi], core.state.surfaces[fi]), 2);
                without_ss += std::pow(rms_of(support[fi], alt.state.surfaces[fi]), 2);
            }
            rep.delta_rms_mm = std::sqrt(with_ss) - std::sqrt(without_ss);
        }
    }
    return result;
}

}  // namespace einstar::fit
