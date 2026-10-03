#pragma once

// Robust fits of single analytic surfaces to oriented scan points.
//
// Each fit starts from a closed-form estimate and refines it with iteratively reweighted Gauss-Newton
// (Tukey weights at a multiple of the robust noise level), so a region that strays onto a neighbouring
// face or a fillet does not drag the surface.

#include <optional>
#include <span>
#include <vector>

#include "einstar/fit/surface.hpp"

namespace einstar::fit {

struct FitOptions {
    int iterations = 15;
    double tukey = 4.0;        // in units of sigma
    double min_sigma = 0.005;  // mm; floor of the noise estimate (exact synthetic data)
};

struct FitResult {
    Surface surface;
    double sigma = 0;     // robust noise level (1.4826 MAD of the residuals), mm
    double rms = 0;       // of the inliers, mm
    std::size_t inliers = 0;  // within 3 sigma
    std::size_t points = 0;
    double bic = 0;       // for choosing between kinds: n ln(sigma^2) + k ln n
};

// Points and their normals (normals may be empty for plane and sphere fits; the others need them for
// the initial estimate). Weights are optional (e.g. triangle areas).
struct PointSet {
    std::span<const Vec3> points;
    std::span<const Vec3> normals;
    std::span<const double> weights;
};

[[nodiscard]] std::optional<FitResult> fit_surface(SurfaceKind kind, const PointSet& data, const FitOptions& options = {});

// Refines a given surface (keeps its kind) on new data.
[[nodiscard]] FitResult refine_surface(const Surface& initial, const PointSet& data, const FitOptions& options = {});

// The simplest kind among `kinds` that explains the data: a kind with more parameters wins only if its
// inlier rms is clearly (20%) lower. Curved fits that bend the data's extent by less than 3 sigma (a plane
// fitted as a huge cylinder) and degenerate cones are not considered.
[[nodiscard]] std::optional<FitResult> fit_best(std::span<const SurfaceKind> kinds, const PointSet& data, const FitOptions& options = {});

// Robust scale: 1.4826 * median(|r|).
[[nodiscard]] double robust_sigma(std::span<const double> residuals);

}  // namespace einstar::fit
