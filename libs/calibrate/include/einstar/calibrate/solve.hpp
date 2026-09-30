#pragma once

// Stereo calibration of the IR pair from board captures, independent of any stored calibration:
// focal lengths from the board homographies (principal point at the image centre, no distortion),
// board poses, the left -> right pose, then a joint bundle adjustment of both cameras' intrinsics,
// distortion, their relative pose and every board pose (optim::refine_stereo_calibration).
//
// Also evaluates a given calibration on the same captures (e.g. the one in the scanner's flash, which
// EXStar made) so the two can be compared on equal terms.

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "einstar/calibrate/board.hpp"
#include "einstar/core/error.hpp"

namespace einstar::calibrate {

struct StereoCapture {
    std::string name;
    BoardDetection left, right;
};

struct SolveOptions {
    bool free_distortion = true;
    // Distortion held at these values (left, right) instead: EXStar's quick calibration keeps the factory
    // calibration's distortion and re-fits focal length, principal point and the rig.
    std::optional<std::array<std::array<double, 5>, 2>> fixed_distortion;
    double huber_px = 1.0;
    double outlier_px = 1.5;  // dots reprojecting worse than this (and 4x the RMS) are dropped once
};

struct ViewReport {
    std::string name;
    int dots = 0;                // seen in both cameras
    double rms_px = 0;           // reprojection, both cameras
    double row_rms_px = 0;       // rectified row difference left vs right
    double distance_mm = 0;      // board centre from the scanner
    double tilt_x_deg = 0, tilt_y_deg = 0;
    bool used = true;
};

struct CalibrationReport {
    RigCalibration rig;           // left, right and T_right_left (texture as given or copied from the left)
    std::vector<ViewReport> views;
    double rms_px = 0;            // reprojection over every dot
    double row_rms_px = 0;        // rectified row difference over every dot
    double max_row_px = 0;
    int dots = 0;
    int dropped = 0;              // outliers removed
};

// Views need >= 20 dots common to both cameras; at least 4 such views.
[[nodiscard]] Result<CalibrationReport> solve_stereo(const std::vector<StereoCapture>& captures, int width, int height,
                                                     const BoardSpec& board = {}, const SolveOptions& options = {});

// A fixed calibration on the captures: only the board poses are fitted.
[[nodiscard]] Result<CalibrationReport> evaluate_calibration(const RigCalibration& rig, const std::vector<StereoCapture>& captures,
                                                             const BoardSpec& board = {});

// Focal lengths from board homographies with the principal point at the image centre (OpenCV's
// initIntrinsicParams2D). `views`: board (mm, z = 0) and pixel points.
[[nodiscard]] std::optional<CameraModel> initial_intrinsics(const std::vector<std::pair<std::vector<Vec2>, std::vector<Vec2>>>& views,
                                                            int width, int height);

// How two calibrations differ, in terms that matter for depth.
struct CalibrationDiff {
    struct Camera {
        double dfx = 0, dfy = 0, dcx = 0, dcy = 0;
        double distortion_px = 0;  // largest difference of the distortion alone over the image (px)
        double mapping_px = 0;     // largest difference of where a ray lands, intrinsics + distortion (px)
    };
    Camera left, right;
    Vec3 rotation_deg{0, 0, 0};   // relative rotation of the right camera (axis-angle, left frame)
    Vec3 translation_mm{0, 0, 0};
    double baseline_mm = 0;       // b - a
};
[[nodiscard]] CalibrationDiff compare_calibrations(const RigCalibration& a, const RigCalibration& b);

// Host-side calibration file (text). The scanner's flash is never written.
struct CalibrationFile {
    RigCalibration rig;
    std::string serial;
    std::string created;  // "yyyy-MM-dd hh:mm"
    std::string source;   // what made it
    double rms_px = 0, row_rms_px = 0;
    int views = 0;
};
[[nodiscard]] Result<void> write_calibration_file(const std::string& path, const CalibrationFile& file);
[[nodiscard]] Result<CalibrationFile> read_calibration_file(const std::string& path);
// Where einstar-calibrate puts the calibration a scanner should scan with instead of its flash
// ($HOME/Documents/Einstar/Calibration/<serial>/active.txt); the Einstar app uses it when present.
[[nodiscard]] std::filesystem::path active_calibration_path(const std::string& serial);

}  // namespace einstar::calibrate
