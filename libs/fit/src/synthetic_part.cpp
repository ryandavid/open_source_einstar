#include "einstar/fit/synthetic_part.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <numbers>
#include <random>

#include <tbb/parallel_for.h>

#include "einstar/track/tsdf.hpp"

namespace einstar::fit {
namespace {

constexpr int kFacesPerBox = 6 + 12 + 8;  // planes, edge fillets, corner spheres

constexpr std::array<char, 3> kAxisName{'x', 'y', 'z'};
char sign_char(int s) { return s > 0 ? '+' : '-'; }

// Face id of a box: planes 0..5 (axis * 2 + (sign > 0)), edges 6..17 (axis along * 4 + sign bits of the
// other two), corners 18..25 (sign bits).
int plane_index(int axis, int sign) { return axis * 2 + (sign > 0 ? 1 : 0); }
int edge_index(int along, int s1, int s2) { return 6 + along * 4 + (s1 > 0 ? 1 : 0) + (s2 > 0 ? 2 : 0); }
int corner_index(const std::array<int, 3>& s) { return 18 + (s[0] > 0 ? 1 : 0) + (s[1] > 0 ? 2 : 0) + (s[2] > 0 ? 4 : 0); }

struct BoxEval {
    double distance;
    int face;  // index within the box
};

BoxEval eval_box_flat(const PartBox& b, const Vec3& p);

// A box with a domed top: the box raised by the dome's height, cut by the dome's height field.
BoxEval eval_box(const PartBox& b, const Vec3& p) {
    if (b.dome <= 0) return eval_box_flat(b, p);
    PartBox high = b;
    high.max.z() += b.dome;
    const BoxEval e = eval_box_flat(high, p);
    const Vec3 c = 0.5 * (b.min + b.max), h = 0.5 * (b.max - b.min);
    const double u = std::clamp((p.x() - c.x()) / h.x(), -1.0, 1.0), v = std::clamp((p.y() - c.y()) / h.y(), -1.0, 1.0);
    const double f = b.max.z() + b.dome * (1 - u * u) * (1 - v * v);
    const Vec2 g(b.dome * -2 * u / h.x() * (1 - v * v), b.dome * (1 - u * u) * -2 * v / h.y());
    const double d = (p.z() - f) / std::sqrt(1 + g.squaredNorm());
    return d > e.distance ? BoxEval{d, plane_index(2, 1)} : e;
}

BoxEval eval_box_flat(const PartBox& b, const Vec3& p) {
    const Vec3 c = 0.5 * (b.min + b.max), h = 0.5 * (b.max - b.min);
    const double r = b.fillet;
    std::array<int, 3> s{};
    Vec3 q;
    for (int i = 0; i < 3; ++i) {
        s[static_cast<std::size_t>(i)] = p[i] >= c[i] ? 1 : -1;
        q[i] = std::abs(p[i] - c[i]) - (h[i] - r);
    }
    const double d = q.cwiseMax(0.0).norm() + std::min(q.maxCoeff(), 0.0) - r;
    int outside = 0;
    for (int i = 0; i < 3; ++i) outside += q[i] > 0;
    int face = 0;
    if (r <= 0 || outside <= 1) {
        int axis = 0;
        q.maxCoeff(&axis);
        face = plane_index(axis, s[static_cast<std::size_t>(axis)]);
    } else if (outside == 2) {
        int along = 0;
        for (int i = 0; i < 3; ++i)
            if (q[i] <= 0) along = i;
        const int a1 = (along + 1) % 3, a2 = (along + 2) % 3;
        face = edge_index(along, s[static_cast<std::size_t>(a1)], s[static_cast<std::size_t>(a2)]);
    } else {
        face = corner_index(s);
    }
    return {d, face};
}

// Hole cutters: the bore (a capped cylinder from just outside the entry), with a counterbore, a countersink and a
// drill point when given. Face indices within a hole: 0 wall, 1 flat bottom, 2 counterbore wall, 3 counterbore
// floor, 4 countersink, 5 drill point.
constexpr double kHoleLead = 1.0;  // mm the cutter starts outside the entry face
constexpr int kFacesPerHole = 6;

struct HoleEval {
    double distance;  // signed distance of the cutter solid
    int face;
};

// A capped cylinder along the hole's axis from -lead to `bottom`: distance, and whether the bottom cap is nearest.
std::pair<double, bool> capped(double rho, double h, double radius, double bottom) {
    const double half = 0.5 * (bottom + kHoleLead), mid = 0.5 * (bottom - kHoleLead);
    const Vec2 d(rho - radius, std::abs(h - mid) - half);
    return {d.cwiseMax(0.0).norm() + std::min(d.maxCoeff(), 0.0), d.y() > d.x() && h > mid};
}

// One nappe of a cone opening towards the entry (decreasing depth) from its apex at depth `apex`.
double cone_towards_entry(double rho, double h, double apex, double half_angle) {
    const double t = apex - h;  // height above the apex
    if (t < 0) return std::hypot(rho, t);
    return rho * std::cos(half_angle) - t * std::sin(half_angle);
}

HoleEval eval_hole(const PartHole& hole, const Vec3& p) {
    const Vec3 v = p - hole.entry;
    const double h = v.dot(hole.axis);
    const double rho = (v - h * hole.axis).norm();
    const double r = 0.5 * hole.diameter;
    const auto [bore, bottom] = capped(rho, h, r, hole.depth);
    HoleEval best{bore, bottom && hole.point_angle_deg <= 0 ? 1 : 0};
    const auto consider = [&](double d, int face) {
        if (d < best.distance) best = {d, face};
    };
    if (hole.counterbore_diameter > 0) {
        const auto [cb, floor] = capped(rho, h, 0.5 * hole.counterbore_diameter, hole.counterbore_depth);
        consider(cb, floor ? 3 : 2);
    }
    if (hole.countersink_diameter > 0) {
        const double a = 0.5 * hole.countersink_angle_deg * std::numbers::pi / 180;
        consider(cone_towards_entry(rho, h, 0.5 * hole.countersink_diameter / std::tan(a), a), 4);
    }
    if (hole.point_angle_deg > 0 && hole.depth < 1e8) {
        const double b = 0.5 * hole.point_angle_deg * std::numbers::pi / 180;
        // The tip below the shoulder (above it the bore covers).
        consider(std::max(cone_towards_entry(rho, h, hole.depth + r / std::tan(b), b), hole.depth - h), 5);
    }
    return best;
}

struct PartEval {
    double distance;
    int face;
};

class PartField {
public:
    explicit PartField(const PartSpec& spec) : spec_(spec) {}

    [[nodiscard]] PartEval operator()(const Vec3& p) const {
        double d = 1e30;
        int face = -1;
        for (std::size_t b = 0; b < spec_.boxes.size(); ++b) {
            const BoxEval e = eval_box(spec_.boxes[b], p);
            if (e.distance < d) {
                d = e.distance;
                face = static_cast<int>(b) * kFacesPerBox + e.face;
            }
        }
        const int hole_base = static_cast<int>(spec_.boxes.size()) * kFacesPerBox;
        for (std::size_t k = 0; k < spec_.holes.size(); ++k) {
            const HoleEval e = eval_hole(spec_.holes[k], p);
            if (-e.distance > d) {
                d = -e.distance;
                face = hole_base + static_cast<int>(k) * kFacesPerHole + e.face;
            }
        }
        return {d, face};
    }

    // Whether a scanner would have seen the space at p. Deep inside a hole it sees nothing, so the wall and
    // floor there (whose surface needs the voxels on both sides) are missing, while the material around
    // the hole is still seen.
    [[nodiscard]] bool observed(const Vec3& p) const {
        if (p.z() < spec_.unseen_below_z) return false;
        for (const auto& hole : spec_.holes) {
            const Vec3 v = p - hole.entry;
            const double h = v.dot(hole.axis);
            const double rho = (v - h * hole.axis).norm();
            if (rho < 0.5 * hole.diameter && h > spec_.hole_wall_depth * hole.diameter && h < hole.depth) return false;
        }
        return true;
    }

private:
    const PartSpec& spec_;
};

// Volume whose bricks are sampled from the analytic part, for recon::extract_mesh.
class AnalyticVolume final : public track::Volume {
public:
    AnalyticVolume(const PartField& field, const Eigen::AlignedBox3d& bounds, track::TsdfParams params)
        : field_(field), params_(params) {
        const double brick_mm = params.voxel_mm * track::kBrickSize;
        const Vec3 lo = bounds.min().array() - 2 * params.truncation_mm, hi = bounds.max().array() + 2 * params.truncation_mm;
        for (int i = 0; i < 3; ++i) {
            lo_[static_cast<std::size_t>(i)] = static_cast<int>(std::floor(lo[i] / brick_mm));
            hi_[static_cast<std::size_t>(i)] = static_cast<int>(std::floor(hi[i] / brick_mm));
        }
    }

    void for_each_brick(const BrickVisitor& fn) const override {
        const double voxel = params_.voxel_mm, trunc = params_.truncation_mm;
        std::array<float, track::kBrickVoxels> sdf{}, weight{};
        for (int bz = lo_[2]; bz <= hi_[2]; ++bz)
            for (int by = lo_[1]; by <= hi_[1]; ++by)
                for (int bx = lo_[0]; bx <= hi_[0]; ++bx) {
                    bool near = false;
                    for (int z = 0; z < track::kBrickSize; ++z)
                        for (int y = 0; y < track::kBrickSize; ++y)
                            for (int x = 0; x < track::kBrickSize; ++x) {
                                const Vec3 p = (Vec3(bx * 8 + x, by * 8 + y, bz * 8 + z).array() + 0.5) * voxel;
                                const double d = field_(p).distance;
                                const auto i = static_cast<std::size_t>((z * 8 + y) * 8 + x);
                                sdf[i] = static_cast<float>(std::clamp(d / trunc, -1.0, 1.0));
                                const bool seen = field_.observed(p);
                                weight[i] = seen ? 1.0f : 0.0f;
                                near |= seen && std::abs(d) < trunc;
                            }
                    if (near) fn(track::BrickCoord{bx, by, bz}, sdf, weight, {});
                }
    }

    void integrate(const track::DepthFrame&, const SE3&, float, bool) override {}
    [[nodiscard]] track::RaycastResult raycast(const SE3&, const track::Intrinsics& k) const override {
        return track::RaycastResult{k, {}, {}, {}, nullptr};
    }
    [[nodiscard]] std::vector<track::SurfacePoint> extract_points(std::uint32_t, float, bool) const override { return {}; }
    [[nodiscard]] std::vector<track::BrickCoord> bricks_updated_since(std::uint32_t) const override { return {}; }
    [[nodiscard]] std::vector<track::SurfacePoint> extract_points(const std::vector<track::BrickCoord>&, float) const override {
        return {};
    }
    [[nodiscard]] std::size_t brick_count() const override { return 0; }
    [[nodiscard]] std::uint32_t frame_counter() const override { return 0; }
    [[nodiscard]] const track::TsdfParams& params() const override { return params_; }
    void clear() override {}
    track::ErasedVoxels erase(const LassoSelection&) override { return {}; }
    void restore(const track::ErasedVoxels&) override {}

private:
    const PartField& field_;
    track::TsdfParams params_;
    std::array<int, 3> lo_{}, hi_{};
};

std::vector<TruthFace> truth_faces(const PartSpec& spec) {
    std::vector<TruthFace> faces;
    for (std::size_t b = 0; b < spec.boxes.size(); ++b) {
        const PartBox& box = spec.boxes[b];
        const Vec3 c = 0.5 * (box.min + box.max), h = 0.5 * (box.max - box.min);
        const double r = box.fillet;
        const int base = static_cast<int>(b) * kFacesPerBox;
        for (int axis = 0; axis < 3; ++axis)
            for (const int s : {-1, 1}) {
                const Vec3 n = s * Vec3::Unit(axis);
                faces.push_back({base + plane_index(axis, s), std::format("box{} {}{}", b, sign_char(s), kAxisName[static_cast<std::size_t>(axis)]),
                                 Plane{n, s * c[axis] + h[axis]}, false});
            }
        if (r <= 0) continue;
        for (int along = 0; along < 3; ++along) {
            const int a1 = (along + 1) % 3, a2 = (along + 2) % 3;
            for (const int s1 : {-1, 1})
                for (const int s2 : {-1, 1}) {
                    const Vec3 p = c + s1 * (h[a1] - r) * Vec3::Unit(a1) + s2 * (h[a2] - r) * Vec3::Unit(a2);
                    faces.push_back({base + edge_index(along, s1, s2),
                                     std::format("box{} fillet {}{}{}{}", b, kAxisName[static_cast<std::size_t>(a1)], sign_char(s1),
                                                 kAxisName[static_cast<std::size_t>(a2)], sign_char(s2)),
                                     Cylinder{p, Vec3::Unit(along), r}, false});
                }
        }
        for (int bits = 0; bits < 8; ++bits) {
            const std::array<int, 3> s{bits & 1 ? 1 : -1, bits & 2 ? 1 : -1, bits & 4 ? 1 : -1};
            Vec3 center = c;
            for (int i = 0; i < 3; ++i) center[i] += s[static_cast<std::size_t>(i)] * (h[i] - r);
            faces.push_back({base + corner_index(s), std::format("box{} corner {}{}{}", b, sign_char(s[0]), sign_char(s[1]), sign_char(s[2])),
                             Sphere{center, r}, false});
        }
    }
    const int hole_base = static_cast<int>(spec.boxes.size()) * kFacesPerBox;
    for (std::size_t k = 0; k < spec.holes.size(); ++k) {
        const PartHole& hole = spec.holes[k];
        const int base = hole_base + static_cast<int>(k) * kFacesPerHole;
        const Vec3 out = -hole.axis;
        faces.push_back({base, std::format("hole{} wall", k), Cylinder{hole.entry, hole.axis, 0.5 * hole.diameter}, true});
        const Vec3 bottom = hole.entry + hole.depth * hole.axis;
        faces.push_back({base + 1, std::format("hole{} bottom", k), Plane{out, out.dot(bottom)}, true});
        faces.push_back({base + 2, std::format("hole{} counterbore", k), Cylinder{hole.entry, hole.axis, 0.5 * hole.counterbore_diameter}, true});
        const Vec3 cb_floor = hole.entry + hole.counterbore_depth * hole.axis;
        faces.push_back({base + 3, std::format("hole{} counterbore floor", k), Plane{out, out.dot(cb_floor)}, true});
        const double a = 0.5 * hole.countersink_angle_deg * std::numbers::pi / 180;
        faces.push_back({base + 4, std::format("hole{} countersink", k),
                         Cone{hole.entry + 0.5 * hole.countersink_diameter / std::tan(a) * hole.axis, out, a}, true});
        const double b = std::max(0.5 * hole.point_angle_deg, 1.0) * std::numbers::pi / 180;
        faces.push_back({base + 5, std::format("hole{} point", k),
                         Cone{hole.entry + (hole.depth + 0.5 * hole.diameter / std::tan(b)) * hole.axis, out, b}, true});
    }
    std::ranges::sort(faces, [](const TruthFace& a, const TruthFace& b) { return a.id < b.id; });
    return faces;
}

}  // namespace

const TruthFace* SyntheticPart::face(int id) const {
    const auto it = std::ranges::find(faces, id, &TruthFace::id);
    return it == faces.end() ? nullptr : &*it;
}

std::size_t SyntheticPart::triangles_of(int id) const { return static_cast<std::size_t>(std::ranges::count(triangle_face, id)); }

SyntheticPart make_synthetic_part(const PartSpec& spec) {
    const PartField field(spec);
    Eigen::AlignedBox3d bounds;
    for (const auto& b : spec.boxes) {
        bounds.extend(b.min);
        bounds.extend(b.max);
    }
    track::TsdfParams tp;
    tp.voxel_mm = static_cast<float>(spec.voxel_mm);
    tp.truncation_mm = static_cast<float>(5 * spec.voxel_mm);
    const AnalyticVolume volume(field, bounds, tp);

    SyntheticPart part;
    part.mesh = recon::extract_mesh(volume);
    recon::remove_small_components(part.mesh);
    if (spec.noise_mm > 0) {
        std::mt19937 rng(spec.seed);
        std::normal_distribution<float> noise(0.0f, static_cast<float>(spec.noise_mm));
        for (std::size_t v = 0; v < part.mesh.vertices.size(); ++v) part.mesh.vertices[v] += noise(rng) * part.mesh.normals[v];
    }
    if (spec.simplify) recon::simplify(part.mesh);
    part.mesh.compute_normals();

    part.triangle_face.resize(part.mesh.triangles.size());
    tbb::parallel_for(std::size_t{0}, part.mesh.triangles.size(), [&](std::size_t t) {
        const auto& tri = part.mesh.triangles[t];
        const Vec3 c = ((part.mesh.vertices[tri[0]] + part.mesh.vertices[tri[1]] + part.mesh.vertices[tri[2]]) / 3.0f).cast<double>();
        part.triangle_face[t] = field(c).face;
    });
    part.faces = truth_faces(spec);
    return part;
}

PartSpec flanged_box_spec() {
    PartSpec spec;
    spec.boxes.push_back({Vec3(-30, -20, 0), Vec3(30, 20, 20), 2.0});
    spec.boxes.push_back({Vec3(-50, -25, 0), Vec3(50, 25, 4), 0.0});
    for (const double x : {-42.0, 42.0})
        for (const double y : {-19.0, 19.0}) spec.holes.push_back({Vec3(x, y, 4), -Vec3::UnitZ(), 5.5, 1e9});
    spec.holes.push_back({Vec3(0, 0, 20), -Vec3::UnitZ(), 8.0, 10.0});
    spec.unseen_below_z = 0.3;
    return spec;
}

PartSpec hole_forms_spec() {
    PartSpec spec;
    spec.boxes.push_back({Vec3(-30, -20, 0), Vec3(30, 20, 12), 0.0});
    PartHole cb{Vec3(-18, 0, 12), -Vec3::UnitZ(), 6.6, 1e9};
    cb.counterbore_diameter = 11;
    cb.counterbore_depth = 6.5;
    PartHole cs{Vec3(0, 0, 12), -Vec3::UnitZ(), 4.5, 1e9};
    cs.countersink_diameter = 9;
    cs.countersink_angle_deg = 90;
    PartHole drilled{Vec3(18, 0, 12), -Vec3::UnitZ(), 5.0, 8.0};
    drilled.point_angle_deg = 118;
    spec.holes = {cb, cs, drilled};
    spec.hole_wall_depth = 4;  // a careful scan: the holes seen to the bottom
    return spec;
}

PartSpec domed_block_spec() {
    PartSpec spec;
    PartBox block{Vec3(-25, -15, 0), Vec3(25, 15, 10), 0.0, 4.0};
    spec.boxes.push_back(block);
    return spec;
}

}  // namespace einstar::fit
