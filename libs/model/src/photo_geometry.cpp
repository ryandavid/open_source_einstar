#include "einstar/model/photo_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace einstar::model {

fit::PinholeCamera pinhole_of(const PhotoCamera& c) {
    fit::PinholeCamera p;
    p.focal = c.focal_px;
    p.principal = c.principal;
    p.k1 = c.k1;
    p.T_camera_world = c.T_camera_world;
    return p;
}

bool is_background(const Document& doc, std::uint32_t triangle) {
    const int id = (*doc.state().region)[triangle];
    if (id == 0) return false;
    const Label* l = doc.label(id);
    return l && l->role == Role::ignore;
}

std::optional<ScanHit> first_part_hit(const Document& doc, const Vec3& origin, const Vec3& direction) {
    if (!doc.has_scan()) return std::nullopt;
    const Vec3f o = origin.cast<float>(), d = direction.normalized().cast<float>();
    float t0 = 0;
    for (int pass = 0; pass < 64; ++pass) {
        const auto hit = doc.bvh().raycast(o, d, t0);
        if (!hit) return std::nullopt;
        if (!is_background(doc, hit->triangle)) return ScanHit{hit->point.cast<double>(), hit->triangle, (*doc.state().region)[hit->triangle]};
        t0 = hit->t + 1e-3f;
    }
    return std::nullopt;
}

std::optional<ScanHit> photo_to_scan(const Document& doc, const Photo& photo, const Vec2& pixel) {
    if (!photo.camera) return std::nullopt;
    const auto [origin, dir] = pinhole_of(*photo.camera).ray(pixel);
    return first_part_hit(doc, origin, dir);
}

namespace {

// Seen from the camera, nothing of the part lies in front of the point (within a tolerance for the scan's
// rounding of edges and noise).
bool unhidden(const Document& doc, const Vec3& eye, const Vec3& point) {
    const Vec3 d = point - eye;
    const double dist = d.norm();
    const auto hit = first_part_hit(doc, eye, d / dist);
    return !hit || (hit->point - eye).norm() >= dist - std::max(1.5, 0.01 * dist);
}

}  // namespace

std::optional<PhotoPoint> scan_to_photo(const Document& doc, const Photo& photo, const Vec3& point) {
    if (!photo.camera) return std::nullopt;
    const auto cam = pinhole_of(*photo.camera);
    const auto px = cam.project(point);
    if (!px) return std::nullopt;
    return PhotoPoint{*px, doc.has_scan() && unhidden(doc, cam.center(), point)};
}

ScanReading read_on_scan(const Document& doc, const Photo& ph, const Annotation& a) {
    ScanReading out;
    for (const auto& q : a.points) out.hits.push_back(photo_to_scan(doc, ph, q));
    // The face the points are on: a plane one of them hit, or the face a hit hole wall opens in.
    const auto cam = pinhole_of(*ph.camera);
    const fit::Plane* plane = nullptr;
    for (const auto& h : out.hits) {
        const Label* l = h && h->label ? doc.label(h->label) : nullptr;
        if (!l || !l->fit) continue;
        if ((plane = std::get_if<fit::Plane>(&*l->fit))) break;
        for (const auto& hole : doc.state().holes)
            if (hole.wall_label == l->id)
                if (const Label* host = doc.label(hole.host); host && host->fit && (plane = std::get_if<fit::Plane>(&*host->fit))) break;
        if (plane) break;
    }
    const auto on_plane = [&](const Vec2& px) -> std::optional<ScanHit> {
        const auto [o, d] = cam.ray(px);
        const double den = plane->normal.dot(d);
        if (std::abs(den) < 1e-6) return std::nullopt;
        const double t = (plane->offset - plane->normal.dot(o)) / den;
        if (t <= 0) return std::nullopt;
        return ScanHit{o + t * d, 0, 0};
    };
    // A diameter is a rim: its points are taken where their rays cross the face (the scan rounds rims, and a ray
    // just inside one goes down the hole). Otherwise a point that misses the scan (past an edge) is taken there.
    if (plane)
        for (std::size_t i = 0; i < out.hits.size(); ++i)
            if (!out.hits[i] || a.kind == AnnotationKind::diameter)
                if (const auto h = on_plane(a.points[i])) out.hits[i] = ScanHit{h->point, 0, out.hits[i] ? out.hits[i]->label : 0};
    const auto& hits = out.hits;
    const bool all = !hits.empty() && std::ranges::all_of(hits, [](const auto& h) { return h.has_value(); });
    std::optional<Vec3> centre;
    if (all) {
        if (a.kind == AnnotationKind::dimension && hits.size() == 2) out.value = (hits[0]->point - hits[1]->point).norm();
        if (a.kind == AnnotationKind::angle && hits.size() == 3) {
            const Vec3 u = hits[0]->point - hits[1]->point, v = hits[2]->point - hits[1]->point;
            if (u.norm() > 1e-9 && v.norm() > 1e-9) out.value = std::acos(std::clamp(u.normalized().dot(v.normalized()), -1.0, 1.0)) * 180 / std::numbers::pi;
        }
        if (a.kind == AnnotationKind::diameter && hits.size() == 2) {
            out.value = 2 * (hits[1]->point - hits[0]->point).norm();
            centre = hits[0]->point;
        }
        if (a.kind == AnnotationKind::diameter && hits.size() == 3) {
            // The circle through three points in space.
            const Vec3 p = hits[0]->point, ab = hits[1]->point - p, ac = hits[2]->point - p;
            const Vec3 n = ab.cross(ac);
            if (n.squaredNorm() > 1e-12) {
                const Vec3 c = p + (ac.squaredNorm() * n.cross(ab) + ab.squaredNorm() * ac.cross(n)) / (2 * n.squaredNorm());
                out.value = 2 * (c - p).norm();
                centre = c;
            }
        }
    }
    for (const auto& h : hits)
        if (h && h->label && std::ranges::find(out.suggested_links, h->label) == out.suggested_links.end()) out.suggested_links.push_back(h->label);
    for (const auto& hole : doc.state().holes) {
        const double r = 0.5 * hole.used_diameter();
        const auto near_axis = [&](const Vec3& x) {
            const Vec3 q = x - hole.center;
            return (q - q.dot(hole.axis) * hole.axis).norm() < r * 1.5;
        };
        if ((centre && near_axis(*centre)) || std::ranges::any_of(hits, [&](const auto& h) { return h && near_axis(h->point); }))
            out.suggested_links.push_back(hole.id);
    }
    return out;
}

std::vector<std::pair<Vec2, Vec2>> overlay_lines(const Document& doc, const Photo& photo) {
    std::vector<std::pair<Vec2, Vec2>> out;
    if (!photo.camera || !doc.has_scan()) return out;
    const auto cam = pinhole_of(*photo.camera);
    const Vec3 eye = cam.center();
    // Pieces short enough that the lens's curvature and partial hiding come out right.
    const double piece = std::max(1.0, 2 * doc.voxel_mm());
    const auto add = [&](const Vec3& a, const Vec3& b) {
        const int n = std::clamp(static_cast<int>(std::ceil((b - a).norm() / piece)), 1, 200);
        for (int i = 0; i < n; ++i) {
            const Vec3 p = a + (b - a) * (i / static_cast<double>(n)), q = a + (b - a) * ((i + 1) / static_cast<double>(n));
            if (!unhidden(doc, eye, 0.5 * (p + q))) continue;
            const auto u = cam.project(p), v = cam.project(q);
            if (u && v) out.emplace_back(*u, *v);
        }
    };
    if (doc.built() && doc.built()->result.ok) {
        for (const auto& e : doc.built()->tessellation.edges) add(e[0].cast<double>(), e[1].cast<double>());
        return out;
    }
    // The boundaries between labels (and between a label and the rest), at most ~20k edges.
    const auto& mesh = doc.mesh();
    const auto& region = *doc.state().region;
    const auto& topo = doc.topology();
    std::vector<std::pair<std::uint32_t, std::uint32_t>> edges;
    for (std::uint32_t t = 0; t < mesh.triangles.size(); ++t) {
        const int lt = region[t];
        if (lt == 0 || is_background(doc, t)) continue;
        for (int k = 0; k < 3; ++k) {
            const auto nb = topo.neighbors(t)[static_cast<std::size_t>(k)];
            // Each boundary once: from the side with the smaller label when both sides are labelled part.
            if (nb != fit::kNoTriangle && (region[nb] == lt || (region[nb] != 0 && !is_background(doc, nb) && region[nb] < lt))) continue;
            edges.emplace_back(mesh.triangles[t][static_cast<std::size_t>(k)], mesh.triangles[t][static_cast<std::size_t>((k + 1) % 3)]);
        }
    }
    const std::size_t stride = std::max<std::size_t>(1, edges.size() / 20000);
    for (std::size_t i = 0; i < edges.size(); i += stride) add(mesh.vertices[edges[i].first].cast<double>(), mesh.vertices[edges[i].second].cast<double>());
    return out;
}

}  // namespace einstar::model
