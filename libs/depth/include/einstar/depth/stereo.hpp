#pragma once

// CPU reference stereo matcher for rectified IR speckle pairs.
//
// Pipeline (coarse-to-fine):
//   1. Build image pyramids of the rectified left/right images.
//   2. At the coarsest level: census transform -> Hamming cost volume ->
//      semi-global aggregation (4 or 8 paths) -> winner-takes-all with a
//      uniqueness test -> left/right consistency check.
//   3. At each finer level: upsample disparity x2 and re-search a small
//      window around it with ZNCC, keeping only confident matches.
//   4. Subpixel interpolation, speckle (small region) removal.
//
// The Metal implementation must match this one on golden inputs.

#include <cstdint>

#include "einstar/core/image.hpp"

namespace einstar::depth {

// Disparities are signed (see depth::RectifiedGeometry), so "no match" is a value no disparity reaches.
inline constexpr float kInvalidDisparity = -1.0e9f;
[[nodiscard]] constexpr bool valid_disparity(float d) { return d > -1.0e8f; }

struct SgmParams {
    int min_disparity = 0;     // at the SGM level (may be negative)
    int num_disparities = 64;  // search range at the SGM level
    int census_radius_x = 2;   // 5x5 window by default
    int census_radius_y = 2;
    int p1 = 8;                // small-jump penalty
    int p2 = 96;               // large-jump penalty (scaled down across intensity edges when adaptive)
    bool adaptive_p2 = true;
    bool eight_paths = false;
    // Allow +/-2 disparity steps per pixel at the small penalty (steeply slanted surfaces are common
    // with the Einstar's wide, converging baseline).
    bool slant_steps = true;
    float uniqueness = 0.9f;   // best cost must be < uniqueness * second-best (non-adjacent)
    int lr_max_diff = 1;       // left/right consistency tolerance (pixels at SGM level)
};

struct RefineParams {
    int search_radius = 2;     // disparity search +/- around the upsampled value
    int zncc_radius = 3;       // 7x7 window
    float min_zncc = 0.5f;     // reject weaker matches
};

struct SpeckleParams {
    int max_region_size = 200;  // regions with fewer pixels are removed (at full resolution)
    float max_diff = 1.0f;      // disparity difference that still connects neighbours
};

struct StereoParams {
    int pyramid_levels = 2;  // SGM runs at 1/2^levels resolution; 0 means full resolution
    SgmParams sgm;
    RefineParams refine;
    SpeckleParams speckle;
    bool subpixel = true;
};

struct StereoResult {
    ImageF32 disparity;  // full-resolution left-referenced disparity, kInvalidDisparity where unknown
    ImageF32 confidence; // 0..1 (ZNCC score at full resolution)
};

// Both images must be rectified, same size, row-aligned; disparity = x_left - x_right (signed).
[[nodiscard]] StereoResult compute_disparity(ImageView<const std::uint8_t> left,
                                             ImageView<const std::uint8_t> right,
                                             const StereoParams& params);

// ---- building blocks, exposed for testing and for the Metal port ----

// Census transform with a (2rx+1)x(2ry+1) window (max 63 bits). Border pixels get 0.
[[nodiscard]] Image<std::uint64_t> census_transform(ImageView<const std::uint8_t> img, int rx, int ry);

// 2x2 box downsample.
[[nodiscard]] ImageU8 downsample2(ImageView<const std::uint8_t> img);

// Semi-global matching on one level. Returns left-referenced integer+subpixel disparity.
[[nodiscard]] ImageF32 sgm_disparity(ImageView<const std::uint8_t> left, ImageView<const std::uint8_t> right,
                                     const SgmParams& params, bool subpixel);

// Removes connected regions (4-neighbourhood, |d_a - d_b| <= max_diff) smaller than max_region_size.
void remove_speckles(ImageF32& disparity, const SpeckleParams& params);

}  // namespace einstar::depth
