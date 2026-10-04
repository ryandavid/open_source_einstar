#pragma once

// Epipolar self-check of a stereo rig from the markers it matches while scanning.
//
// Every marker seen by both IR cameras is a correspondence, so the row difference of its two rectified
// centres measures the rig's epipolar error directly, in the scanner's working conditions (temperature,
// handling) rather than the calibration board's. A calibration that has drifted shows as a row offset
// that is the same all over the image (the right camera turned about the baseline) and sometimes a row
// difference that grows across the image (turned about its optical axis). check_epipolar measures both
// from a recording's (or a live run's) marker pairs, and says whether the evidence is good enough to
// correct the rig: many pairs, spread over the image, agreeing on one small rotation.
//
// Rows are measured in the rig's own epipolar frame (both cameras turned to put x along the baseline,
// the rotation between them split in half, as the rectification does), at the rectified focal length
// mean(fy_L, fy_R) -- independent of how a pipeline chooses its rectified image.
//
// Only the right camera's orientation changes: the left camera (the depth maps' frame), the intrinsics
// and the baseline vector's length and the right camera's centre stay as they are.

#include <span>
#include <string>
#include <vector>

#include "einstar/core/camera.hpp"

namespace einstar::calib {

// Both cameras' rotation into the rig's epipolar frame (original camera -> epipolar), and the focal
// length rows are measured at.
struct EpipolarFrame {
    Mat3 R_left = Mat3::Identity(), R_right = Mat3::Identity();
    double f = 0;
};
[[nodiscard]] EpipolarFrame epipolar_frame(const RigCalibration& rig);

// One marker's centre in the two raw (distorted) IR images.
struct MarkerPair {
    Vec2 left, right;
};

// Back from rectified centres (e.g. session::FrameMarker::left_rect / right_rect) to raw pixels, through
// the rectification that produced them: rotations original -> rectified and the rectified camera.
[[nodiscard]] MarkerPair raw_marker_pair(const RigCalibration& rig, const Mat3& R_rect_left, const Mat3& R_rect_right, const CameraModel& rectified,
                                         const Vec2& left_rect, const Vec2& right_rect);

// Rectified row of the left centre minus the right one (px), in the rig's epipolar frame.
[[nodiscard]] double row_difference(const RigCalibration& rig, const MarkerPair& pair);
[[nodiscard]] std::vector<double> row_differences(const RigCalibration& rig, std::span<const MarkerPair> pairs);

// The rig with its right camera turned about its own centre: about the baseline (moves every rectified
// row of the right image by about f * angle) and about its optical axis (rows tilt across the image).
// Angles in radians, axes in the epipolar frame, positive by the right-hand rule.
[[nodiscard]] RigCalibration turn_right_camera(const RigCalibration& rig, double about_baseline_rad, double about_optical_axis_rad);

struct EpipolarCheckOptions {
    double gate_px = 1.5;              // pairs further than this from the median row difference are mismatches
    double huber_px = 0.15;            // robust loss of the fit
    int min_pairs = 300;               // fewer inliers: report only
    int grid = 3;                      // the left image split into grid x grid cells for the spread test
    int min_cell_pairs = 25;           // a cell counts as covered with this many inliers
    int min_cells = 5;                 // covered cells needed to correct
    double agree_px = 0.12;            // after the correction every covered cell's median row error is within this
    double min_correction_px = 0.05;   // smaller offsets are left alone (within the noise of a good calibration)
    double max_correction_px = 2.0;    // larger ones are not drift: recalibrate
    bool fit_optical_axis = true;      // also the turn about the optical axis, when the pairs span the image
    double min_x_span = 0.35;          // ... the inliers' 5-95% column span, as a fraction of the width
    double min_optical_axis_px = 0.05; // ... and its row change at the image's side edges is at least this
};

struct EpipolarCheck {
    // Row difference (left - right, px) statistics of the inliers under the given rig and the corrected one.
    struct Rows {
        double median = 0, abs_median = 0, p95 = 0, robust_sigma = 0;
        // Least-squares model dy = offset + slope_x * u + slope_y * v + slope_disparity * d, with u, v the left
        // centre's offset from the image centre in half-widths / half-heights and d = disparity / 300 px: what
        // is left after a correction shows as a slope.
        double offset = 0, slope_x = 0, slope_y = 0, slope_disparity = 0;
    };

    int pairs = 0, inliers = 0;
    int cells = 0;                     // covered cells of the spread test
    double x_span = 0;                 // inliers' column span (fraction of the width)
    Rows before, after;
    double worst_cell_px = 0;          // largest |median row error| of a covered cell after the correction
    double about_baseline_deg = 0;     // the correction (turn_right_camera)
    double about_optical_axis_deg = 0;
    double sigma_baseline_deg = 0, sigma_optical_axis_deg = 0;  // standard errors (pairs taken as independent)
    bool optical_axis_fitted = false;
    bool apply = false;                // the evidence agrees: `corrected` is better than the rig
    std::string verdict;               // one line: what was found and why it is (not) applied
    RigCalibration corrected;          // the rig with the correction when `apply`, else the rig unchanged
};

[[nodiscard]] EpipolarCheck check_epipolar(const RigCalibration& rig, std::span<const MarkerPair> pairs, const EpipolarCheckOptions& options = {});

// Marker pairs collected over a scan for check_epipolar, evenly over the image: the left image in cells,
// each keeping its newest `per_cell` pairs, so a long stay in one place neither swamps the rest nor grows
// memory, and the check follows a calibration that drifts during the scan. Not thread-safe.
class EpipolarSamples {
public:
    explicit EpipolarSamples(int width = 1280, int height = 1024, int cells = 6, int per_cell = 200);
    void add(const MarkerPair& pair);
    void clear();
    [[nodiscard]] std::size_t size() const;
    [[nodiscard]] std::vector<MarkerPair> pairs() const;

private:
    int width_, height_, cells_, per_cell_;
    std::vector<std::vector<MarkerPair>> ring_;  // per cell
    std::vector<std::size_t> next_;              // per cell: where the next pair goes once full
};

// How two rigs differ in the rows they rectify: points spread over the left image at the scanner's working
// distances are seen through `truth` and their rows compared through `model` (px).
struct RowAgreement {
    double mean = 0, rms = 0, max_abs = 0;
};
[[nodiscard]] RowAgreement row_agreement(const RigCalibration& truth, const RigCalibration& model, double near_mm = 250, double far_mm = 500);

}  // namespace einstar::calib
