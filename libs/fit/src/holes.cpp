#include "einstar/fit/holes.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <Eigen/Dense>

#include "einstar/fit/grow.hpp"
#include "einstar/fit/primitive_fit.hpp"

namespace einstar::fit {

std::vector<std::vector<std::uint32_t>> region_boundary_loops(const recon::TriangleMesh& mesh, const MeshTopology& topo,
                                                               std::span<const std::uint8_t> in_region) {
    // Boundary half-edges are (triangle, edge) pairs, 3 t + e. The one after a half-edge a -> b is found by
    // turning about b through the region's triangles until an edge leaves the region again, so a vertex
    // where the region touches itself only at a corner does not join two loops.
    const auto is_boundary = [&](std::uint32_t t, std::uint32_t e) {
        const std::uint32_t n = topo.neighbors(t)[e];
        return n == kNoTriangle || !in_region[n];
    };
    const auto next = [&](std::uint32_t h) -> std::uint32_t {
        std::uint32_t t = h / 3;
        const std::uint32_t b = mesh.triangles[t][(h % 3 + 1) % 3];
        std::uint32_t e = (h % 3 + 1) % 3;  // the edge of t leaving b
        for (int guard = 0; guard < 256; ++guard) {
            if (is_boundary(t, e)) return 3 * t + e;
            t = topo.neighbors(t)[e];
            e = 0;
            while (e < 3 && mesh.triangles[t][e] != b) ++e;
            if (e == 3) return kNoTriangle;
        }
        return kNoTriangle;
    };
    std::vector<std::uint8_t> used(topo.triangle_count() * 3, 0);
    std::vector<std::vector<std::uint32_t>> loops;
    for (std::uint32_t t = 0; t < topo.triangle_count(); ++t) {
        if (!in_region[t]) continue;
        for (std::uint32_t e = 0; e < 3; ++e) {
            const std::uint32_t first = 3 * t + e;
            if (used[first] || !is_boundary(t, e)) continue;
            std::vector<std::uint32_t> loop;
            std::uint32_t h = first;
            while (h != kNoTriangle && !used[h]) {
                used[h] = 1;
                loop.push_back(mesh.triangles[h / 3][h % 3]);
                h = next(h);
            }
            if (loop.size() >= 3) loops.push_back(std::move(loop));
        }
    }
    std::ranges::sort(loops, [](const auto& a, const auto& b) { return a.size() > b.size(); });
    return loops;
}

std::optional<Circle2> fit_circle_2d(std::span<const Vec2> p, std::span<const double> w) {
    if (p.size() < 3) return std::nullopt;
    const auto weight = [&](std::size_t i) { return w.empty() ? 1.0 : w[i]; };
    // Kasa start.
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    Vec2 mean = Vec2::Zero();
    for (const auto& q : p) mean += q;
    mean /= static_cast<double>(p.size());
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec2 q = p[i] - mean;
        const Eigen::Vector3d row(2 * q.x(), 2 * q.y(), 1.0);
        A += weight(i) * row * row.transpose();
        b += weight(i) * row * q.squaredNorm();
    }
    const Eigen::Vector3d x = A.ldlt().solve(b);
    Circle2 c;
    c.center = mean + x.head<2>();
    const double r2 = x[2] + x.head<2>().squaredNorm();
    if (!x.allFinite() || r2 <= 0) return std::nullopt;
    c.radius = std::sqrt(r2);
    // Geometric refinement with Tukey weights.
    std::vector<double> res(p.size());
    for (int it = 0; it < 20; ++it) {
        for (std::size_t i = 0; i < p.size(); ++i) res[i] = (p[i] - c.center).norm() - c.radius;
        const double cut = 4 * std::max(robust_sigma(res), 1e-6);
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d g = Eigen::Vector3d::Zero();
        for (std::size_t i = 0; i < p.size(); ++i) {
            const double u = res[i] / cut;
            if (std::abs(u) >= 1) continue;
            const double wi = weight(i) * (1 - u * u) * (1 - u * u);
            const Vec2 d = p[i] - c.center;
            const double n = d.norm();
            if (n < 1e-12) continue;
            const Eigen::Vector3d J(-d.x() / n, -d.y() / n, -1.0);
            H += wi * J * J.transpose();
            g += wi * J * res[i];
        }
        const Eigen::Vector3d step = H.ldlt().solve(-g);
        if (!step.allFinite()) break;
        c.center += step.head<2>();
        c.radius += step[2];
        if (step.norm() < 1e-9) break;
    }
    double ss = 0;
    std::size_t n = 0;
    for (std::size_t i = 0; i < p.size(); ++i) res[i] = (p[i] - c.center).norm() - c.radius;
    const double cut = 3 * std::max(robust_sigma(res), 1e-6);
    for (const double r : res)
        if (std::abs(r) < cut) {
            ss += r * r;
            ++n;
        }
    c.rms = n > 0 ? std::sqrt(ss / static_cast<double>(n)) : 0;
    if (!(c.radius > 0) || !c.center.allFinite()) return std::nullopt;
    return c;
}

namespace {

// Largest angular gap (deg) between points around a centre; 360 minus it is the arc covered.
double covered_arc_deg(std::span<const Vec2> p, const Vec2& center) {
    if (p.empty()) return 0;
    std::vector<double> a(p.size());
    for (std::size_t i = 0; i < p.size(); ++i) a[i] = std::atan2(p[i].y() - center.y(), p[i].x() - center.x());
    std::ranges::sort(a);
    double gap = a.front() + 2 * std::numbers::pi - a.back();
    for (std::size_t i = 1; i < a.size(); ++i) gap = std::max(gap, a[i] - a[i - 1]);
    return 360.0 - gap * 180.0 / std::numbers::pi;
}

}  // namespace

// The hole's wall, from the triangles reached from its opening: the bore's diameter, and its forms where the scan
// shows them.
//  - A counterbore: the straight wall at two radii (the larger above), with a floor between them.
//  - A countersink: a conical band between the opening and a narrower bore; its line in (depth, radius) gives
//    the angle and the diameter at the face.
//  - A drill point: the floor of a blind hole sloping to a tip; its line in (radius, depth) gives the angle and
//    the depth of the shoulder.
template <class To2>
void analyse_wall(const MeshTopology& topo, HoleCandidate& h, const std::vector<std::uint32_t>& straight, const Circle2& opening, const To2& to2,
                  const Vec3& origin, const Vec3& u, const Vec3& v, const HoleOptions& o) {
    const double vox = o.voxel_mm;
    const auto radial = [&](const Vec3& p) { return (to2(p) - opening.center).norm(); };
    const auto depth_of = [&](const Vec3& p) { return (p - h.center).dot(h.axis); };
    const auto fit_wall = [&](const std::vector<Vec2>& pts, const std::vector<double>& w) -> std::optional<Circle2> {
        if (pts.size() < o.min_wall_triangles) return std::nullopt;
        const auto c = fit_circle_2d(pts, w);
        if (!c || covered_arc_deg(pts, c->center) < o.min_wall_arc_deg) return std::nullopt;
        return c;
    };
    std::optional<Circle2> bore;
    if (straight.size() >= o.min_wall_triangles) {
        const RegionPoints wp = region_points(topo, straight, 0);
        std::vector<std::pair<double, std::size_t>> by_radius;
        for (std::size_t i = 0; i < wp.points.size(); ++i) by_radius.emplace_back(radial(wp.points[i]), i);
        std::ranges::sort(by_radius);
        // The largest gap in radius splits a counterbore's wall from the bore's.
        std::size_t split = 0;
        double gap = 0;
        for (std::size_t i = 1; i < by_radius.size(); ++i)
            if (by_radius[i].first - by_radius[i - 1].first > gap) {
                gap = by_radius[i].first - by_radius[i - 1].first;
                split = i;
            }
        const auto subset = [&](std::size_t from, std::size_t to) {
            std::pair<std::vector<Vec2>, std::vector<double>> out;
            for (std::size_t i = from; i < to; ++i) {
                out.first.push_back(to2(wp.points[by_radius[i].second]));
                out.second.push_back(wp.weights[by_radius[i].second]);
            }
            return out;
        };
        if (gap > 3 * vox) {
            const auto [in_p, in_w] = subset(0, split);
            const auto [out_p, out_w] = subset(split, by_radius.size());
            const auto inner = fit_wall(in_p, in_w), outer = fit_wall(out_p, out_w);
            if (inner && outer) {
                bore = inner;
                h.counterbore_diameter = 2 * outer->radius;
                // Its floor: faces looking back out of the hole between the two walls.
                double sum = 0, area = 0;
                for (const auto t : h.wall) {
                    const Vec3 c = topo.centroid(t).cast<double>();
                    const double r = radial(c);
                    if (topo.normal(t).cast<double>().dot(h.axis) < -0.9 && r > inner->radius + vox && r < outer->radius - vox) {
                        sum += topo.area(t) * depth_of(c);
                        area += topo.area(t);
                    }
                }
                if (area > 0) h.counterbore_depth = sum / area;
            }
        }
        if (!bore) {
            const auto [all_p, all_w] = subset(0, by_radius.size());
            bore = fit_wall(all_p, all_w);
        }
    }
    // A countersink: a cone between a bore clearly narrower than the opening and the face.
    if (bore && !h.counterbore_diameter && bore->radius < opening.radius - 2 * vox) {
        Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
        Eigen::Vector2d b = Eigen::Vector2d::Zero();
        int n = 0;
        for (const auto t : h.wall) {
            const Vec3 c = topo.centroid(t).cast<double>();
            const double r = radial(c), dn = -topo.normal(t).cast<double>().dot(h.axis);  // a countersink looks back out
            if (dn < 0.3 || dn > 0.97 || r < bore->radius + vox || r > opening.radius + vox) continue;
            const Eigen::Vector2d row(1.0, depth_of(c));
            A += topo.area(t) * row * row.transpose();
            b += topo.area(t) * row * r;
            ++n;
        }
        if (n >= 10) {
            const Eigen::Vector2d x = A.ldlt().solve(b);  // radius = x0 + x1 depth
            const double half = std::atan(-x[1]);
            if (x.allFinite() && half > 15 * std::numbers::pi / 180 && half < 75 * std::numbers::pi / 180) {
                h.countersink_diameter = 2 * x[0];
                h.countersink_angle_deg = 2 * half * 180 / std::numbers::pi;
            }
        }
    }
    const bool formed = h.counterbore_diameter || h.countersink_diameter;
    if (bore && (formed || std::abs(bore->radius - opening.radius) < 2 * vox)) {
        h.wall_diameter = 2 * bore->radius;
        h.center = origin + bore->center.x() * u + bore->center.y() * v;
    }
    h.diameter = h.wall_diameter ? *h.wall_diameter : 2 * (opening.radius - o.rim_bias_voxels * vox);

    // A blind hole's floor: flat, or a drill point.
    const double r = 0.5 * h.diameter;
    Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
    Eigen::Vector2d bb = Eigen::Vector2d::Zero();
    double area = 0;
    for (const auto t : h.wall) {
        const Vec3 c = topo.centroid(t).cast<double>();
        const double rr = radial(c), d = depth_of(c);
        if (-topo.normal(t).cast<double>().dot(h.axis) < 0.45 || rr > r - 0.5 * vox || d < std::max(0.5 * r, 2 * vox)) continue;
        if (h.counterbore_depth && d < *h.counterbore_depth + vox) continue;
        const Eigen::Vector2d row(1.0, rr);
        A += topo.area(t) * row * row.transpose();
        bb += topo.area(t) * row * d;
        area += topo.area(t);
    }
    if (area > 4 * vox * vox) {
        const Eigen::Vector2d x = A.ldlt().solve(bb);  // depth = x0 + x1 radius
        if (x.allFinite()) {
            if (x[1] < -0.15) {
                const double half = std::atan(-1.0 / x[1]);
                h.point_angle_deg = 2 * half * 180 / std::numbers::pi;
                h.floor_depth = x[0] + x[1] * r;  // the shoulder
            } else {
                h.floor_depth = bb[0] / A(0, 0);
            }
        }
    }
}

std::vector<HoleCandidate> find_holes(const recon::TriangleMesh& mesh, const MeshTopology& topo, const TriangleBvh& bvh,
                                      std::span<const std::uint32_t> plane_region, const Plane& plane, const HoleOptions& o) {
    std::vector<std::uint8_t> in_region(topo.triangle_count(), 0);
    for (const auto t : plane_region) in_region[t] = 1;
    const Vec3 n = plane.normal;
    const Vec3 u = any_perpendicular(n), v = n.cross(u);
    const Vec3 origin = plane.offset * n;
    const auto to2 = [&](const Vec3& p) { return Vec2((p - origin).dot(u), (p - origin).dot(v)); };

    std::vector<HoleCandidate> holes;
    for (const auto& loop : region_boundary_loops(mesh, topo, in_region)) {
        std::vector<Vec2> pts(loop.size());
        double signed_area = 0;
        for (std::size_t i = 0; i < loop.size(); ++i) pts[i] = to2(mesh.vertices[loop[i]].cast<double>());
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const Vec2& a = pts[i];
            const Vec2& b = pts[(i + 1) % pts.size()];
            signed_area += a.x() * b.y() - a.y() * b.x();
        }
        // The region lies to the left of its boundary seen from outside (+n): its outline runs
        // anticlockwise, the openings inside it clockwise.
        if (signed_area >= 0) continue;
        const auto circle = fit_circle_2d(pts);
        if (!circle) continue;
        const double roundness = circle->rms / circle->radius;
        if (roundness > o.max_roundness_error || 2 * circle->radius < o.min_diameter || 2 * circle->radius > o.max_diameter) continue;
        // The loop must go round the circle (not a short, nearly straight notch a large circle happens to fit).
        double perimeter = 0;
        for (std::size_t i = 0; i < pts.size(); ++i) perimeter += (pts[(i + 1) % pts.size()] - pts[i]).norm();
        const double circumference = 2 * std::numbers::pi * circle->radius;
        if (covered_arc_deg(pts, circle->center) < 300 || perimeter < 0.8 * circumference || perimeter > 1.6 * circumference) continue;

        HoleCandidate h;
        h.axis = -n;
        h.center = origin + circle->center.x() * u + circle->center.y() * v;
        h.opening_diameter = 2 * circle->radius;
        h.roundness_error = roundness;
        h.opening = loop;

        // The wall: triangles off the face, inside the opening's cylinder, reached from the opening.
        const double r_max = circle->radius + o.voxel_mm;
        const double depth_max = 6 * h.opening_diameter;
        // (A hole's surfaces look back towards its entry or across it; a face looking away from the entry is the
        // outside of the part where a through hole comes out.)
        const auto on_wall = [&](std::uint32_t t) {
            if (in_region[t] || topo.normal(t).cast<double>().dot(h.axis) > 0.5) return false;
            const Vec3 c = topo.centroid(t).cast<double>() - h.center;
            const double depth = c.dot(h.axis);
            return depth > -o.voxel_mm && depth < depth_max && (c - depth * h.axis).norm() < r_max;
        };
        std::vector<std::uint8_t> seen(topo.triangle_count(), 0);
        std::vector<std::uint32_t> stack;
        for (const auto vtx : loop)
            for (const auto t : topo.vertex_triangles(vtx))
                if (!seen[t] && on_wall(t)) {
                    seen[t] = 1;
                    stack.push_back(t);
                }
        std::vector<std::uint32_t> straight;
        while (!stack.empty()) {
            const auto t = stack.back();
            stack.pop_back();
            h.wall.push_back(t);
            const Vec3 c = topo.centroid(t).cast<double>();
            const double depth = (c - h.center).dot(h.axis);
            h.wall_depth = std::max(h.wall_depth, depth);
            // Fit only the straight wall below the rounded rim.
            if (depth > o.voxel_mm && std::abs(topo.normal(t).cast<double>().dot(h.axis)) < 0.35) straight.push_back(t);
            for (const auto nb : topo.neighbors(t))
                if (nb != kNoTriangle && !seen[nb] && on_wall(nb)) {
                    seen[nb] = 1;
                    stack.push_back(nb);
                }
        }
        std::ranges::sort(h.wall);
        analyse_wall(topo, h, straight, *circle, to2, origin, u, v, o);

        // A floor: looking down the axis from above the face, the first surface facing back up the axis (unless the
        // wall analysis found a pointed one).
        if (!h.floor_depth) {
            const Vec3 from = h.center - 2.0 * h.axis;
            if (const auto hit = bvh.raycast(from.cast<float>(), h.axis.cast<float>(), 0.0f, static_cast<float>(depth_max + 2.0))) {
                const double depth = hit->t - 2.0;
                if (depth > o.voxel_mm && std::abs(topo.normal(hit->triangle).cast<double>().dot(h.axis)) > 0.8) h.floor_depth = depth;
            }
        }
        holes.push_back(std::move(h));
    }
    return holes;
}

}  // namespace einstar::fit
