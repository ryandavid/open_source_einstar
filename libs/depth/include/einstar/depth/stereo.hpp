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
    // A best score at either end of the search may be the flank of a peak outside it: no match.
    bool reject_edge_winners = true;
};

// Detail refinement on the full-resolution images: each half-resolution disparity is re-matched with a
// small window (2*radius full-resolution pixels square, slanted by the local disparity gradient) close
// to its value. The 7x7 half-resolution window (14x14 full-resolution pixels) is robust but blurs depth
// over its whole footprint, which rounds edges and makes the noise blobby (strongly correlated between
// neighbours); the small window keeps the detail. Matches it cannot confirm keep the first estimate.
struct DetailParams {
    // Synthetic speckle, 3 frames (debug_depth_errors): median error 0.137 -> 0.113 mm, neighbour error
    // correlation 0.75 -> 0.59, gross errors unchanged; a wider search lets small windows jump to wrong peaks.
    bool enabled = true;
    int radius = 5;            // 10x10 full-resolution window
    float search = 1.0f;       // full-resolution px either side of the first estimate
    float step = 0.5f;         // full-resolution px
    float min_zncc = 0.5f;
};

// After refinement: a pixel whose right view another left pixel, nearer the camera, also claims is
// occluded in the right image (its match is the foreground's); an isolated value far from its 3x3
// neighbours' median is a spike.
struct FilterParams {
    bool occlusion = true;
    float occlusion_tolerance = 1.0f;  // disparity px (stereo resolution) closer than the claimant
    bool spikes = true;
    float spike_threshold = 1.0f;      // disparity px from the 3x3 median
};

struct SpeckleParams {
    int max_region_size = 200;  // regions with fewer pixels are removed (at full resolution)
    float max_diff = 1.0f;      // disparity difference that still connects neighbours
};

struct StereoParams {
    int pyramid_levels = 2;  // SGM runs at 1/2^levels resolution; 0 means full resolution
    SgmParams sgm;
    RefineParams refine;
    DetailParams detail;     // needs the full-resolution images (compute_disparity's second form)
    FilterParams filter;
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
// The same, with the full-resolution images `left` and `right` were 2x2 box-downsampled from, for the
// detail refinement (params.detail).
[[nodiscard]] StereoResult compute_disparity(ImageView<const std::uint8_t> left, ImageView<const std::uint8_t> right,
                                             ImageView<const std::uint8_t> full_left, ImageView<const std::uint8_t> full_right,
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

// Detail refinement (see DetailParams) of a disparity at half the full images' resolution, in place.
void refine_detail(ImageView<const std::uint8_t> full_left, ImageView<const std::uint8_t> full_right, ImageF32& disparity,
                   const DetailParams& params, bool subpixel);

// Occlusion and spike filters (see FilterParams), in place.
void filter_disparity(ImageF32& disparity, const FilterParams& params);

}  // namespace einstar::depth
