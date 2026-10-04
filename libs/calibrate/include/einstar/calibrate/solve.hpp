#pragma once

// Stereo calibration of the IR pair from board captures, independent of any stored calibration:
// focal lengths from the board homographies (principal point at the image centre, no distortion),
// board poses, the left -> right pose, then a joint bundle adjustment of both cameras' intrinsics,
// distortion, their relative pose and every board pose (optim::refine_stereo_calibration), and last
// the same with every dot of the board free (the real board is not its nominal grid: held as exact,
// its errors bend the distortion, and with it the rectified rows where no dot was).
//
// Also evaluates a given calibration on the same captures (e.g. the one in the scanner's flash, which
// EXStar made) so the two can be compared on equal terms.

#include <array>
#include <filesystem>
#include <optional>
#include <span>
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
    // A last adjustment with every dot of the board free (its nominal grid a weak prior and the scale):
    // the board's own errors then stay in the board instead of bending the distortion.
    bool refine_board = true;
};

struct ViewReport {
    std::string name;
    int dots = 0;                // seen in both cameras
    double rms_px = 0;           // reprojection, both cameras
    double row_rms_px = 0;       // rectified row difference left vs right
    double distance_mm = 0;      // board centre from the scanner
    double tilt_x_deg = 0, tilt_y_deg = 0;
    SE3 T_left_board = SE3::Identity();
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
    double board_rms_mm = 0;      // the fitted dots' offset from the nominal grid (refine_board)
    double board_flatness_mm = 0; // ... out of the board's plane (rms)
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
    // Rectified rows of points seen through a, rectified through b, over the image at 250-500 mm (px):
    // what b would leave of the epipolar error if a were right.
    double row_mean_px = 0, row_max_px = 0;
};
[[nodiscard]] CalibrationDiff compare_calibrations(const RigCalibration& a, const RigCalibration& b);

// The scanner's calibration blob with its quick-calibration section replaced by a solve result, as
// EXStar's quick calibration does (docs/calibration.md 1.2): Left/Right from the solve, the colour
// camera's intrinsics and its pose relative to the left camera kept from `current`, the world frame
// the board at `T_left_world` (EXStar: its first, face-on view). Only builds and checks bytes.
struct FlashUpdate {
    std::vector<std::uint8_t> blob;   // the full 6568 bytes to store
    std::vector<int> pages;           // 4 KB flash pages that differ from `current` (0 and / or 1)
    std::string calibration_time;
};
[[nodiscard]] Result<FlashUpdate> build_flash_update(std::span<const std::uint8_t> current, const RigCalibration& rig, const SE3& T_left_world,
                                                     const std::string& calibration_time, std::uint32_t seed);

// Calibration file (text): the record of a solve, readable by einstar-cli wherever it takes a calibration.
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

}  // namespace einstar::calibrate
