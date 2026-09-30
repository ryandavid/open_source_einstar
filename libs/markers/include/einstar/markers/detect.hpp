#pragma once

// Retro-reflective marker detection in a single (raw, distorted) IR image.
//
// 1. Blobs above a grey threshold (connected components), gated by size, aspect and fill.
// 2. Sub-pixel contour at the half-intensity level between local background and peak
//    (robust for saturated markers), traced by marching squares.
// 3. Direct least-squares ellipse fit (Fitzgibbon) with residual, axis-ratio and angular-coverage
//    checks. The marker centre is the ellipse centre.
// 4. The sticker's dark ring must surround the disc (rejects laser speckle and highlights).

#include <cstdint>
#include <array>
#include <vector>

#include "einstar/core/image.hpp"
#include "einstar/core/se3.hpp"

namespace einstar::markers {

struct DetectParams {
    int threshold = 150;            // grey level a marker must exceed (EXStar live value)
    // Blob bounding-box limits. 6 px: a 6 mm sticker at 700 mm is ~10 px; laser speckle dots and glints
    // on real IR are 3-7 px and would otherwise pass the ring test near bright surfaces.
    double min_diameter_px = 6.0;
    double max_diameter_px = 60.0;
    double max_aspect = 4.0;        // blob box aspect ratio
    double min_fill = 0.5;          // blob pixels / box area
    double max_axis_ratio = 3.0;    // fitted a/b (foreshortening limit, ~70 deg)
    double max_residual_px = 0.35;  // RMS geometric residual of contour points
    double min_coverage = 0.7;      // fraction of 12 angular sectors with contour points
    double ring_scale = 1.4;        // dark-ring test at this multiple of the fitted ellipse (0 = off)
    // Ring samples must stay within this fraction of the disc contrast above the background (a
    // Gaussian speckle dot is still at ~26% of its peak there; the sticker ring is near black).
    double ring_max_contrast = 0.2;
    double max_ring_bright_fraction = 0.1;
    // Saturated blobs (a retro-reflective sticker under the ring light saturates the sensor; projector
    // speckle mostly does not) get a looser ring test: on the scanner's IR the sticker's dark surround
    // is not much darker than the surface, and glare lifts it further. 0.2 / 0.1 kept 4-5 of 10
    // stickers on a real scan, this 10 of 10 (einstar-cli markers-debug); the size limit rejects the
    // saturated glints it admits.
    int saturated_level = 250;
    double saturated_ring_max_contrast = 0.4;
    double saturated_max_ring_bright_fraction = 0.2;
    int border = 4;
};

struct Ellipse {
    Vec2 center;          // pixels (image coordinates, pixel centres at integers)
    double a = 0, b = 0;  // semi-axes, a >= b
    double angle = 0;     // of the major axis, radians
    double residual = 0;  // RMS geometric residual (px)
    int peak = 0;         // max grey level in the blob
    int points = 0;       // contour points used
};

// Outcome of each candidate blob in fit_blobs (diagnostics, e.g. tuning on real images).
enum class Reject : std::uint8_t { accepted, box_size, aspect, fill, border, contrast, contour, coverage, axis_ratio, residual, ellipse_size, ring, count_ };
struct FitStats {
    std::array<int, static_cast<std::size_t>(Reject::count_)> count{};  // by Reject
};
[[nodiscard]] const char* reject_name(Reject r);

[[nodiscard]] std::vector<Ellipse> detect_markers(ImageView<const std::uint8_t> image, const DetectParams& params = {},
                                                  FitStats* stats = nullptr);

// The two stages of detect_markers: connected blobs above the threshold (also computed on the GPU,
// see depth_metal::MetalStereo), then the sub-pixel contour / ellipse / ring tests per blob.
struct Blob {
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;  // inclusive box
    int pixels = 0;
    int peak = 0;
};
[[nodiscard]] std::vector<Blob> find_blobs_cpu(ImageView<const std::uint8_t> image, int threshold);
[[nodiscard]] std::vector<Ellipse> fit_blobs(ImageView<const std::uint8_t> image, const std::vector<Blob>& blobs, const DetectParams& params,
                                             FitStats* stats = nullptr);

// Fitzgibbon direct ellipse fit; returns false for degenerate input.
[[nodiscard]] bool fit_ellipse(const std::vector<Vec2>& points, Ellipse& out);

}  // namespace einstar::markers
