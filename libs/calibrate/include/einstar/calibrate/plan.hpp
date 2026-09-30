#pragma once

// Capture plan and live guidance for calibrating the IR pair from board views.
//
// The plan mirrors what EXStar's quick calibration captures (rapidCameraCalibrate.xml, and measured on its
// 25 stored images with `einstar-cli board-poses`): five board orientations - face-on, and tilted about
// 30 degrees each way about the scanner's horizontal and vertical axes - each at five distances from
// ~200 to ~600 mm. Near views cover the image (distortion), tilted ones separate focal length from
// distance.
//
// Poses are measured in the scanner frame: origin midway between the two IR cameras, z along the
// bisector of their optical axes, x towards the right camera, y = z x x.
//
// Roll (the board's in-plane rotation in the view) is held at 90 +- 20 degrees: the board's long axis runs
// along the image's vertical, as in all of EXStar's 25 captures (76..107 degrees). The tilted groups are
// defined in the image's terms ("right edge near"), so a fixed roll makes each group one direction from
// the board: seen from the board, the plan is five lines from its centre with five distances on each.

#include <string>
#include <vector>

#include "einstar/calibrate/board.hpp"

namespace einstar::calibrate {

// Left IR camera -> scanner frame.
[[nodiscard]] SE3 scanner_from_left(const RigCalibration& rig);

// Where the board is, relative to the scanner.
struct BoardMeasure {
    double distance_mm = 0;    // board centre along the scanner's z axis
    Vec2 offset_mm{0, 0};      // board centre off the scanner's axis (x, y)
    double tilt_x_deg = 0;     // about the scanner's x axis (bottom edge nearer, image down being +y: +)
    double tilt_y_deg = 0;     // about the y axis (right edge nearer: +)
    double roll_deg = 0;       // board's in-plane rotation: board x from the scanner's x towards its y
};
// T_left_board: board -> left camera (board_pose).
[[nodiscard]] BoardMeasure measure_board(const SE3& T_left_board, const RigCalibration& rig, const BoardSpec& board = {});

struct PoseTarget {
    int group = 0, step = 0;
    std::string label;  // e.g. "face-on, 280 mm"
    double distance_mm = 0;
    double tilt_x_deg = 0, tilt_y_deg = 0;
    double roll_deg = 90;
    double distance_tol_mm = 25;
    double tilt_tol_deg = 7;
    double offset_tol_mm = 45;
    double roll_tol_deg = 20;
};
[[nodiscard]] std::vector<PoseTarget> default_plan();

// The target's board pose in the left camera, at the given roll (the target's own: t.roll_deg).
[[nodiscard]] SE3 target_pose(const PoseTarget& target, double roll_deg, const RigCalibration& rig, const BoardSpec& board = {});

// ---- the board-centred view: poses in board coordinates ----
// The board pose in the scanner frame that `m` describes (the inverse of measure_board, rig-free).
[[nodiscard]] SE3 scanner_from_board(const BoardMeasure& m, const BoardSpec& board = {});
// A target as a measure: on the scanner's axis (no offset), at its own roll.
[[nodiscard]] BoardMeasure target_measure(const PoseTarget& t);
// The scanner (its frame) in board coordinates, for a measure or a target.
[[nodiscard]] inline SE3 board_from_scanner(const BoardMeasure& m, const BoardSpec& board = {}) { return scanner_from_board(m, board).inverse(); }
[[nodiscard]] inline SE3 board_from_scanner(const PoseTarget& t, const BoardSpec& board = {}) { return board_from_scanner(target_measure(t), board); }

// The uncaptured target nearest the measured pose, and how near: the largest of its distance and tilt
// errors in units of their tolerances (<= 1: within both). -1 when all are captured.
struct NearestTarget {
    int index = -1;
    double error = 0;
};
[[nodiscard]] NearestTarget nearest_target(const std::vector<PoseTarget>& plan, const std::vector<bool>& captured, const BoardMeasure& m);

struct Guidance {
    bool distance_ok = false, tilt_ok = false, offset_ok = false, roll_ok = false;
    double distance_error_mm = 0;  // measured - target
    Vec2 tilt_error_deg{0, 0};     // measured - target (x, y)
    Vec2 offset_mm{0, 0};
    double roll_error_deg = 0;     // measured - target, -180..180
    std::vector<std::string> hints;  // what to change, most important first (empty when in position)

    [[nodiscard]] bool ok() const { return distance_ok && tilt_ok && offset_ok && roll_ok; }
    // 0..1 closeness across all criteria (for a progress-style indicator).
    [[nodiscard]] double score(const PoseTarget& t) const;
};
[[nodiscard]] Guidance guide(const BoardMeasure& measured, const PoseTarget& target);

// Holds a board still for capture: ready once the pose has changed by less than the limits over the
// last `hold_s` seconds.
class SteadinessGate {
public:
    struct Limits {
        double hold_s = 0.6;
        double move_mm = 2.0;
        double turn_deg = 0.6;
    };
    SteadinessGate() = default;
    explicit SteadinessGate(Limits l) : limits_(l) {}

    // Returns the time the board has been steady (seconds).
    double update(const SE3& T_left_board, double time_s);
    void reset() { anchor_valid_ = false; }
    [[nodiscard]] bool ready(double steady_s) const { return steady_s >= limits_.hold_s; }
    [[nodiscard]] const Limits& limits() const { return limits_; }

private:
    Limits limits_;
    bool anchor_valid_ = false;
    SE3 anchor_ = SE3::Identity();
    double anchor_time_ = 0;
};

}  // namespace einstar::calibrate
