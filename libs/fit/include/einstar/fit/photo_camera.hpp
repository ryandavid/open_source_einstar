#pragma once

// Where a photo was taken from, relative to the part: a camera solved from points the user matched between the
// photo and the scan. The photo may have been taken anywhere; only the part's own points are used.
//
// The camera is a pinhole with one radial distortion term, its principal point at the image centre (as phone and
// most other cameras are, near enough):
//   n = (x / z, y / z) for the point in the camera frame (x right, y down, z forward),
//   pixel = principal + focal * n * (1 + k1 |n|^2).

#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "einstar/core/se3.hpp"

namespace einstar::fit {

struct PinholeCamera {
    double focal = 0;            // px
    Vec2 principal = Vec2::Zero();
    double k1 = 0;
    SE3 T_camera_world = SE3::Identity();

    // The pixel a world point appears at (nullopt: behind the camera).
    [[nodiscard]] std::optional<Vec2> project(const Vec3& world) const;
    // The world ray through a pixel: the camera centre and a unit direction.
    [[nodiscard]] std::pair<Vec3, Vec3> ray(const Vec2& pixel) const;
    [[nodiscard]] Vec3 center() const;
};

struct CameraFitOptions {
    int width = 0, height = 0;           // the photo's size (the principal point is its centre)
    std::optional<double> focal_prior;   // px, e.g. from the EXIF 35 mm equivalent
    double huber_px = 3;                 // residuals beyond this count linearly
};

struct CameraFit {
    PinholeCamera camera;
    double rms_px = 0;
    std::vector<double> residuals;     // px, per pair
    std::vector<bool> outlier;         // a pair the others disagree with
    bool focal_fitted = false, k1_fitted = false;
};

// The fewest pairs a solve needs: 4 with a focal length known (EXIF), else 6.
[[nodiscard]] int min_camera_pairs(bool focal_known);

// The camera that best explains the pairs (pixels[i] is where points[i] is seen). nullopt with too few pairs
// or no consistent camera.
[[nodiscard]] std::optional<CameraFit> fit_camera(std::span<const Vec2> pixels, std::span<const Vec3> points, const CameraFitOptions& options);

// The focal length in px of a photo with a 35 mm equivalent focal length (along the longer side: 36 mm).
[[nodiscard]] double focal_from_35mm(double focal_35mm, int width, int height);

}  // namespace einstar::fit
