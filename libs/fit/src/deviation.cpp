#include "einstar/fit/deviation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

#include <Eigen/Eigenvalues>
#include <tbb/parallel_for.h>

namespace einstar::fit {
namespace {

DeviationStats stats_of(std::vector<float> d, int group, double tolerance) {
    DeviationStats s;
    s.group = group;
    s.vertices = d.size();
    if (d.empty()) return s;
    double ss = 0, sum = 0;
    std::size_t within = 0;
    for (const float x : d) {
        ss += static_cast<double>(x) * x;
        sum += x;
        s.max_abs = std::max(s.max_abs, static_cast<double>(std::abs(x)));
        within += std::abs(x) <= tolerance;
    }
    const auto n = static_cast<double>(d.size());
    s.rms = std::sqrt(ss / n);
    s.mean = sum / n;
    s.within_tolerance = static_cast<double>(within) / n;
    for (float& x : d) x = std::abs(x);
    const auto k = d.begin() + static_cast<std::ptrdiff_t>(std::min(d.size() - 1, static_cast<std::size_t>(0.95 * n)));
    std::nth_element(d.begin(), k, d.end());
    s.p95 = *k;
    return s;
}

template <class T>
T most_common(const std::map<T, double>& votes, T none) {
    T best = none;
    double best_w = -1;
    for (const auto& [k, w] : votes)
        if (w > best_w) {
            best = k;
            best_w = w;
        }
    return best;
}

}  // namespace

DeviationField scan_to_model(const recon::TriangleMesh& scan, const TriangleBvh& model, std::span<const int> model_triangle_face,
                             const DeviationOptions& o) {
    DeviationField f;
    const std::size_t nv = scan.vertices.size();
    f.distance.assign(nv, std::numeric_limits<float>::quiet_NaN());
    f.model_face.assign(nv, -1);
    const auto& mesh = model.mesh();
    tbb::parallel_for(std::size_t{0}, nv, [&](std::size_t v) {
        const Vec3f& p = scan.vertices[v];
        const auto hit = model.closest(p, static_cast<float>(o.max_distance_mm));
        if (!hit) return;
        const auto& tri = mesh.triangles[hit->triangle];
        const Vec3f n = (mesh.vertices[tri[1]] - mesh.vertices[tri[0]]).cross(mesh.vertices[tri[2]] - mesh.vertices[tri[0]]);
        const float side = (p - hit->point).dot(n);
        // When the closest point is on an edge of the model, the offset runs sideways to its triangle; the scan's
        // own normal (out of the material) then tells inside from outside.
        float sign = side >= 0 ? 1.0f : -1.0f;
        if (std::abs(side) < 0.5f * n.norm() * hit->distance && v < scan.normals.size())
            sign = scan.normals[v].dot(p - hit->point) >= 0 ? 1.0f : -1.0f;
        f.distance[v] = sign * hit->distance;
        f.model_face[v] = hit->triangle < model_triangle_face.size() ? model_triangle_face[hit->triangle] : -1;
    });
    return f;
}

DeviationReport summarise(const MeshTopology& topo, const DeviationField& field, std::span<const int> vertex_group, const DeviationOptions& o) {
    DeviationReport r;
    const auto& scan = topo.mesh();
    const std::size_t nv = scan.vertices.size();
    const auto group_of = [&](std::size_t v) { return vertex_group.empty() ? -1 : vertex_group[v]; };
    const auto counted = [&](std::size_t v) { return group_of(v) >= -1 && !std::isnan(field.distance[v]); };

    std::vector<float> all;
    std::map<int, std::vector<float>> by_group;
    for (std::size_t v = 0; v < nv; ++v) {
        if (group_of(v) < -1) continue;
        if (std::isnan(field.distance[v])) {
            ++r.unmatched;
            continue;
        }
        all.push_back(field.distance[v]);
        if (group_of(v) >= 0) by_group[group_of(v)].push_back(field.distance[v]);
    }
    r.overall = stats_of(std::move(all), -1, o.tolerance_mm);
    for (auto& [g, d] : by_group) r.groups.push_back(stats_of(std::move(d), g, o.tolerance_mm));

    // Hot spots: vertices beyond the tolerance, joined across triangle edges when on the same side.
    std::vector<float> vertex_area(nv, 0);
    for (std::uint32_t t = 0; t < topo.triangle_count(); ++t)
        for (const auto v : scan.triangles[t]) vertex_area[v] += topo.area(t) / 3;
    std::vector<int> side(nv, 0);
    for (std::size_t v = 0; v < nv; ++v)
        if (counted(v) && std::abs(field.distance[v]) > o.tolerance_mm) side[v] = field.distance[v] > 0 ? 1 : -1;
    std::vector<std::uint8_t> seen(nv, 0);
    std::vector<std::uint32_t> stack;
    for (std::uint32_t start = 0; start < nv; ++start) {
        if (side[start] == 0 || seen[start]) continue;
        HotSpot h;
        double weighted = 0;
        std::map<int, double> faces, groups;
        std::vector<std::uint32_t> members;
        seen[start] = 1;
        stack.push_back(start);
        while (!stack.empty()) {
            const std::uint32_t v = stack.back();
            stack.pop_back();
            members.push_back(v);
            const double a = vertex_area[v];
            const double d = field.distance[v];
            h.area_mm2 += a;
            h.centroid += a * scan.vertices[v].cast<double>();
            weighted += a * d;
            if (std::abs(d) > std::abs(h.peak_mm)) h.peak_mm = d;
            faces[field.model_face[v]] += a;
            groups[group_of(v)] += a;
            ++h.vertices;
            for (const auto t : topo.vertex_triangles(v))
                for (const auto u : scan.triangles[t])
                    if (!seen[u] && side[u] == side[start]) {
                        seen[u] = 1;
                        stack.push_back(u);
                    }
        }
        if (h.area_mm2 < o.min_hot_spot_area_mm2) continue;
        h.centroid /= h.area_mm2;
        h.mean_mm = weighted / h.area_mm2;
        // Length from the spread along the main axis (a uniform strip of length L has variance L^2 / 12).
        Mat3 cov = Mat3::Zero();
        for (const auto v : members) {
            const Vec3 d = scan.vertices[v].cast<double>() - h.centroid;
            cov += vertex_area[v] * d * d.transpose();
        }
        cov /= h.area_mm2;
        const double spread = std::max(0.0, Eigen::SelfAdjointEigenSolver<Mat3>(cov).eigenvalues()[2]);
        h.length_mm = std::sqrt(12 * spread);
        h.width_mm = h.length_mm > 0 ? h.area_mm2 / h.length_mm : 0;
        h.edge_band = h.width_mm < o.edge_band_width_mm;
        h.model_face = most_common(faces, -1);
        h.group = most_common(groups, -1);
        r.hot_spots.push_back(h);
    }
    std::ranges::sort(r.hot_spots, [](const HotSpot& a, const HotSpot& b) {
        if (a.edge_band != b.edge_band) return !a.edge_band;
        return a.area_mm2 * std::abs(a.mean_mm) > b.area_mm2 * std::abs(b.mean_mm);
    });
    return r;
}

Coverage model_coverage(const recon::TriangleMesh& model, std::span<const int> model_triangle_face, int face_count, const TriangleBvh& scan,
                        const DeviationOptions& o) {
    Coverage c;
    c.face_area.assign(static_cast<std::size_t>(std::max(face_count, 0)), 0);
    c.unsupported_area.assign(c.face_area.size(), 0);
    c.triangle_supported.assign(model.triangles.size(), 0);
    tbb::parallel_for(std::size_t{0}, model.triangles.size(), [&](std::size_t t) {
        const auto& tri = model.triangles[t];
        const Vec3f centroid = (model.vertices[tri[0]] + model.vertices[tri[1]] + model.vertices[tri[2]]) / 3.0f;
        c.triangle_supported[t] = scan.closest(centroid, static_cast<float>(o.coverage_distance_mm)).has_value();
    });
    for (std::size_t t = 0; t < model.triangles.size(); ++t) {
        const int f = t < model_triangle_face.size() ? model_triangle_face[t] : -1;
        if (f < 0 || f >= face_count) continue;
        const auto& tri = model.triangles[t];
        const double a = 0.5 * (model.vertices[tri[1]] - model.vertices[tri[0]]).cross(model.vertices[tri[2]] - model.vertices[tri[0]]).norm();
        c.face_area[static_cast<std::size_t>(f)] += a;
        if (!c.triangle_supported[t]) c.unsupported_area[static_cast<std::size_t>(f)] += a;
    }
    return c;
}

std::array<std::uint8_t, 4> deviation_color(float d, float tolerance, float range) {
    if (std::isnan(d)) return {128, 128, 128, 255};
    const auto mix = [](const std::array<float, 3>& a, const std::array<float, 3>& b, float t) {
        t = std::clamp(t, 0.0f, 1.0f);
        return std::array<std::uint8_t, 4>{static_cast<std::uint8_t>(255 * (a[0] + t * (b[0] - a[0]))),
                                           static_cast<std::uint8_t>(255 * (a[1] + t * (b[1] - a[1]))),
                                           static_cast<std::uint8_t>(255 * (a[2] + t * (b[2] - a[2]))), 255};
    };
    constexpr std::array<float, 3> green{0.2f, 0.75f, 0.3f}, yellow{0.95f, 0.85f, 0.1f}, red{0.85f, 0.1f, 0.1f};
    constexpr std::array<float, 3> cyan{0.1f, 0.75f, 0.85f}, blue{0.1f, 0.2f, 0.85f};
    const float a = std::abs(d);
    if (a <= tolerance) return mix(green, green, 0);
    const float t = (a - tolerance) / std::max(range - tolerance, 1e-6f);
    return d > 0 ? mix(yellow, red, t) : mix(cyan, blue, t);
}

}  // namespace einstar::fit
