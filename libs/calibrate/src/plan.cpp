#include "einstar/calibrate/plan.hpp"

#include <algorithm>
#include <cmath>
#include <format>

namespace einstar::calibrate {
namespace {

constexpr double kDeg = 180.0 / M_PI;

// Smallest rotation taking unit vector a to unit vector b.
Mat3 rotation_between(const Vec3& a, const Vec3& b) {
    return Eigen::Quaterniond::FromTwoVectors(a, b).toRotationMatrix();
}

Vec3 normal_from_tilts(double tilt_x_deg, double tilt_y_deg) {
    return Vec3(std::tan(tilt_y_deg / kDeg), std::tan(tilt_x_deg / kDeg), 1.0).normalized();
}

}  // namespace

SE3 scanner_from_left(const RigCalibration& rig) {
    const Mat3 R = rig.T_right_left.linear();
    const Vec3 right_centre = -R.transpose() * rig.T_right_left.translation();  // in the left camera frame
    const Vec3 mid = 0.5 * right_centre;
    const Vec3 z = (Vec3::UnitZ() + R.transpose() * Vec3::UnitZ()).normalized();
    const Vec3 x = (right_centre - right_centre.dot(z) * z).normalized();
    const Vec3 y = z.cross(x);
    SE3 T = SE3::Identity();  // left -> scanner: p_S = R_SL (p_L - mid)
    T.linear().row(0) = x.transpose();
    T.linear().row(1) = y.transpose();
    T.linear().row(2) = z.transpose();
    T.translation() = -T.linear() * mid;
    return T;
}

BoardMeasure measure_board(const SE3& T_left_board, const RigCalibration& rig, const BoardSpec& board) {
    const SE3 T_S_board = scanner_from_left(rig) * T_left_board;
    const Vec3 c = T_S_board * board.centre();
    const Mat3 R = T_S_board.linear();
    // The board's +z points away from the scanner when its dots face it.
    const Vec3 n = R.col(2);
    BoardMeasure m;
    m.distance_mm = c.z();
    m.offset_mm = c.head<2>();
    m.tilt_x_deg = std::atan2(n.y(), n.z()) * kDeg;
    m.tilt_y_deg = std::atan2(n.x(), n.z()) * kDeg;
    const Mat3 R0 = rotation_between(n, Vec3::UnitZ()) * R;  // tilt removed
    m.roll_deg = std::atan2(R0(1, 0), R0(0, 0)) * kDeg;
    return m;
}

std::vector<PoseTarget> default_plan() {
    struct Group {
        const char* name;
        double tx, ty;
        std::array<double, 5> distances;
    };
    // EXStar's captures: face-on at 200-510 mm, tilted groups from ~230-310 out to 600 mm (a tilted
    // board does not fit the view at 200 mm).
    const std::array<Group, 5> groups = {{
        {"face-on", 0, 0, {200, 280, 360, 440, 520}},
        {"right edge near", 0, 30, {300, 360, 420, 500, 580}},
        {"left edge near", 0, -30, {300, 360, 420, 500, 580}},
        {"bottom edge near", 30, 0, {280, 340, 420, 500, 580}},
        {"top edge near", -30, 0, {280, 340, 420, 500, 580}},
    }};
    std::vector<PoseTarget> plan;
    for (int g = 0; g < 5; ++g)
        for (int s = 0; s < 5; ++s) {
            const auto& G = groups[static_cast<std::size_t>(g)];
            PoseTarget t;
            t.group = g, t.step = s;
            t.tilt_x_deg = G.tx, t.tilt_y_deg = G.ty;
            t.distance_mm = G.distances[static_cast<std::size_t>(s)];
            t.label = std::format("{}, {:.0f} mm", G.name, t.distance_mm);
            plan.push_back(t);
        }
    return plan;
}

SE3 target_pose(const PoseTarget& t, double roll_deg, const RigCalibration& rig, const BoardSpec& board) {
    const Vec3 n = normal_from_tilts(t.tilt_x_deg, t.tilt_y_deg);
    SE3 T_S_board = SE3::Identity();
    T_S_board.linear() = rotation_between(Vec3::UnitZ(), n) * Eigen::AngleAxisd(roll_deg / kDeg, Vec3::UnitZ()).toRotationMatrix();
    T_S_board.translation() = Vec3(0, 0, t.distance_mm) - T_S_board.linear() * board.centre();
    return scanner_from_left(rig).inverse() * T_S_board;
}

double Guidance::score(const PoseTarget& t) const {
    const double worst = std::max({std::abs(distance_error_mm) / t.distance_tol_mm, tilt_error_deg.norm() / t.tilt_tol_deg,
                                   offset_mm.norm() / t.offset_tol_mm});
    return std::clamp(1.0 - (worst - 1.0) / 4.0, 0.0, 1.0) * (worst <= 1.0 ? 1.0 : 0.9);
}

Guidance guide(const BoardMeasure& m, const PoseTarget& t) {
    Guidance g;
    g.distance_error_mm = m.distance_mm - t.distance_mm;
    g.tilt_error_deg = Vec2(m.tilt_x_deg - t.tilt_x_deg, m.tilt_y_deg - t.tilt_y_deg);
    g.offset_mm = m.offset_mm;
    g.distance_ok = std::abs(g.distance_error_mm) <= t.distance_tol_mm;
    g.tilt_ok = g.tilt_error_deg.norm() <= t.tilt_tol_deg;
    g.offset_ok = g.offset_mm.norm() <= t.offset_tol_mm;
    // Hints in the camera view's terms (image right = scanner +x, image down = +y), largest error first.
    struct Hint {
        double weight;
        std::string text;
    };
    std::vector<Hint> hints;
    if (!g.distance_ok)
        hints.push_back({std::abs(g.distance_error_mm) / t.distance_tol_mm,
                         std::format("{} {:.0f} mm", g.distance_error_mm > 0 ? "Move closer" : "Move back", std::abs(g.distance_error_mm))});
    if (!g.tilt_ok) {
        // +tilt_x: the board's lower edge (in the view) is nearer; +tilt_y: its right edge is.
        const double ex = g.tilt_error_deg.x(), ey = g.tilt_error_deg.y();
        if (std::abs(ex) >= std::abs(ey))
            hints.push_back({g.tilt_error_deg.norm() / t.tilt_tol_deg,
                             std::format("Tilt: bring the board's {} edge closer ({:.0f} deg)", ex > 0 ? "top" : "bottom", std::abs(ex))});
        else
            hints.push_back({g.tilt_error_deg.norm() / t.tilt_tol_deg,
                             std::format("Tilt: bring the board's {} edge closer ({:.0f} deg)", ey > 0 ? "left" : "right", std::abs(ey))});
    }
    if (!g.offset_ok) {
        const Vec2 o = g.offset_mm;
        const char* dir = std::abs(o.x()) >= std::abs(o.y()) ? (o.x() > 0 ? "right" : "left") : (o.y() > 0 ? "down" : "up");
        hints.push_back({o.norm() / t.offset_tol_mm, std::format("Centre the board: aim the scanner {} ({:.0f} mm)", dir, o.norm())});
    }
    std::ranges::sort(hints, [](const Hint& a, const Hint& b) { return a.weight > b.weight; });
    for (auto& h : hints) g.hints.push_back(std::move(h.text));
    return g;
}

double SteadinessGate::update(const SE3& T, double time_s) {
    const double moved = anchor_valid_ ? (T.translation() - anchor_.translation()).norm() : 1e9;
    const double turned = anchor_valid_ ? Eigen::AngleAxisd(T.linear() * anchor_.linear().transpose()).angle() * kDeg : 1e9;
    if (moved > limits_.move_mm || turned > limits_.turn_deg) {
        anchor_ = T;
        anchor_time_ = time_s;
        anchor_valid_ = true;
        return 0;
    }
    return time_s - anchor_time_;
}

}  // namespace einstar::calibrate
