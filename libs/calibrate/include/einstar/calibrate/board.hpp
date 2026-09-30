#pragma once

// The Einstar calibration board (docs/calibration.md 6): retro-reflective dots on a square grid of
// 8 x 5, with three of row 1's dots (columns 2, 4, 5) large, plus a lone large dot off the grid between
// columns 3 and 4 of row 3. The large dots fix which dot is which and the board's orientation.
//
// Board coordinates: x along the 8 columns, y along the 5 rows, z = 0 on the board (mm, origin at the
// dot of column 0, row 0).

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include "einstar/core/camera.hpp"
#include "einstar/core/image.hpp"
#include "einstar/markers/detect.hpp"

namespace einstar::calibrate {

struct BoardSpec {
    int cols = 8;
    int rows = 5;
    // Nominal 28 mm. Solving EXStar's own 25 calibration captures with 28 mm gives its baseline to
    // 0.03% (einstar-cli calib-solve); the 27.89 once triangulated with its calibration does not.
    double pitch_mm = 28.0;
    // Grid positions of the large dots: the three in row 1, then the lone one.
    std::array<Vec2, 4> large = {Vec2(2, 1), Vec2(4, 1), Vec2(5, 1), Vec2(3.5, 3)};

    [[nodiscard]] int dots() const { return cols * rows; }
    [[nodiscard]] Vec3 point(const Vec2& grid) const { return {grid.x() * pitch_mm, grid.y() * pitch_mm, 0}; }
    [[nodiscard]] Vec3 centre() const { return point(Vec2(0.5 * (cols - 1), 0.5 * (rows - 1))); }
    // Board outline half a pitch outside the outer dots, in board coordinates (for drawing).
    [[nodiscard]] std::array<Vec3, 4> outline() const;
};

// The board's grid dots found in one raw (distorted) image.
struct BoardDetection {
    std::vector<Vec2> grid;    // (column, row) of each dot found
    std::vector<Vec2> pixels;  // its ellipse centre, raw image pixels
    std::array<Vec2, 4> large{};  // the large dots, in BoardSpec::large order
    Mat3 H = Mat3::Identity();    // grid -> pixel homography fitted to the dots
    double residual_px = 0;       // RMS of the dots about H (lens distortion included)
    int blobs = 0;                // ellipses the marker detector found

    [[nodiscard]] std::size_t size() const { return grid.size(); }
    [[nodiscard]] bool complete(const BoardSpec& b) const { return static_cast<int>(grid.size()) == b.dots(); }
};

struct BoardDetectParams {
    markers::DetectParams markers;  // defaults widened for close boards (see board_detect_params)
    int min_dots = 20;              // fewer grid dots than this: not a board
    double gate = 0.3;              // association gate, fraction of the local grid pitch
};
[[nodiscard]] BoardDetectParams board_detect_params();

[[nodiscard]] std::optional<BoardDetection> detect_board(ImageView<const std::uint8_t> image, const BoardSpec& board = {},
                                                         const BoardDetectParams& params = board_detect_params());
// The same from already detected ellipses (e.g. the scan pipeline's).
[[nodiscard]] std::optional<BoardDetection> associate_board(const std::vector<markers::Ellipse>& ellipses, const BoardSpec& board = {},
                                                            const BoardDetectParams& params = board_detect_params());

// Board pose in a calibrated camera: planar homography on undistorted points, refined by Gauss-Newton
// on the reprojection error.
struct BoardPose {
    SE3 T_cam_board = SE3::Identity();
    double rms_px = 0;  // reprojection RMS
};
[[nodiscard]] std::optional<BoardPose> board_pose(const BoardDetection& detection, const CameraModel& camera, const BoardSpec& board = {});

// DLT homography dst ~ H src (>= 4 points, normalised for conditioning).
[[nodiscard]] Mat3 fit_homography(const std::vector<Vec2>& src, const std::vector<Vec2>& dst);
[[nodiscard]] Vec2 apply_homography(const Mat3& H, const Vec2& p);

}  // namespace einstar::calibrate
