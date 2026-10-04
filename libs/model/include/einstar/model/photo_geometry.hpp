#pragma once

// A registered photo (one whose camera has been solved from points matched on the part) and the scan: what a
// pixel shows on the scan, where a scan point appears in the photo, and the model's lines drawn over the photo.
//
// Only the part counts: scan labelled `ignore` (a table, a fixture, background the scanner caught) neither takes a
// matched point nor hides anything, since the photo may have been taken somewhere else entirely.

#include <array>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "einstar/fit/photo_camera.hpp"
#include "einstar/model/document.hpp"

namespace einstar::model {

[[nodiscard]] fit::PinholeCamera pinhole_of(const PhotoCamera& c);

struct ScanHit {
    Vec3 point = Vec3::Zero();
    std::uint32_t triangle = 0;
    int label = 0;  // 0: unlabelled
};

// Whether a triangle belongs to scan that is not the part (labelled `ignore`).
[[nodiscard]] bool is_background(const Document& doc, std::uint32_t triangle);

// The first point of the part along a world ray (background passed through).
[[nodiscard]] std::optional<ScanHit> first_part_hit(const Document& doc, const Vec3& origin, const Vec3& direction);
// What a pixel of a registered photo shows on the part.
[[nodiscard]] std::optional<ScanHit> photo_to_scan(const Document& doc, const Photo& photo, const Vec2& pixel);
// Where a point appears in the photo, and whether the part hides it there.
struct PhotoPoint {
    Vec2 pixel = Vec2::Zero();
    bool visible = false;
};
[[nodiscard]] std::optional<PhotoPoint> scan_to_photo(const Document& doc, const Photo& photo, const Vec3& point);

// What an annotation on a registered photo reads on the scan: its points on the part, the scan's measure of it
// (a dimension's length, a diameter, an angle), and what it is likely about (labels under its points, a hole at
// its centre).
struct ScanReading {
    std::vector<std::optional<ScanHit>> hits;
    std::optional<double> value;
    std::vector<int> suggested_links;
};
[[nodiscard]] ScanReading read_on_scan(const Document& doc, const Photo& photo, const Annotation& annotation);

// A diameter marked on a registered photo, as a circle on a plane face: its points taken where the photo's rays
// cross the plane (what the scan shows there does not matter, so a hole the scan bridged over still has its rim).
struct PlaneCircle {
    Vec3 centre = Vec3::Zero();
    double diameter = 0;
};
[[nodiscard]] std::optional<PlaneCircle> circle_on_plane(const Photo& photo, const Annotation& annotation, const fit::Plane& plane);

// The scan coloured from its registered photos (those allowed to colour it): each vertex takes the colour of the
// photo that sees it most head-on, where nothing of the part hides it. Alpha 0: no photo sees it, or it is not
// the part. Photos are decoded with their longer side at most max_edge px.
[[nodiscard]] std::vector<std::array<std::uint8_t, 4>> photo_colours(const Document& doc, int max_edge = 2048);

// The model's lines as the photo sees them (visible parts only): the built solid's edges, or without one, the
// boundaries between labels.
[[nodiscard]] std::vector<std::pair<Vec2, Vec2>> overlay_lines(const Document& doc, const Photo& photo);

}  // namespace einstar::model
