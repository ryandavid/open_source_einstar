#include "einstar/calib/convention.hpp"

namespace einstar::calib {
namespace {

// The camera turned 180 degrees about its optical axis: pixel (u, v) -> (W-1-u, H-1-v).
CameraModel turned(const CameraModel& c) {
    CameraModel t = c;
    t.cx = c.width - 1.0 - c.cx;
    t.cy = c.height - 1.0 - c.cy;
    t.dist[2] = -c.dist[2];
    t.dist[3] = -c.dist[3];
    return t;
}

}  // namespace

RigCalibration swap_camera_convention(const RigCalibration& rig) {
    // Camera frames turn with their images: X' = D X, D = diag(-1, -1, 1). The new left camera is the old
    // right one turned, the new right the old left turned, so new-left -> new-right = D T_rl^-1 D.
    SE3 D = SE3::Identity();
    D.linear() = Vec3(-1, -1, 1).asDiagonal();
    RigCalibration out;
    out.left = turned(rig.right);
    out.right = turned(rig.left);
    out.texture = rig.texture;
    out.T_right_left = D * rig.T_right_left.inverse() * D;
    // texture <- old left <- old right <- new left: old-right = D new-left, old-left = T_rl^-1 old-right.
    out.T_texture_left = rig.T_texture_left * rig.T_right_left.inverse() * D;
    return out;
}

ConventionCheck check_camera_convention(const RigCalibration& rig, const RigCalibration& reference) {
    auto distance = [&](const RigCalibration& r) {
        return Vec2(r.left.cx - reference.left.cx, r.left.cy - reference.left.cy).norm() +
               Vec2(r.right.cx - reference.right.cx, r.right.cy - reference.right.cy).norm();
    };
    ConventionCheck c;
    c.as_is_px = distance(rig);
    c.swapped_px = distance(swap_camera_convention(rig));
    if (c.swapped_px * 1.5 < c.as_is_px) c.convention = CameraConvention::swapped;
    else if (c.as_is_px * 1.5 < c.swapped_px) c.convention = CameraConvention::exstar;
    return c;
}

Result<FlashCalibration> read_flash_calibration(std::span<const std::uint8_t> blob) {
    auto cal = decode_flash_blob(blob);
    if (!cal) return std::unexpected(cal.error());
    FlashCalibration out;
    out.calibration = std::move(*cal);
    out.rig = out.calibration.rig();
    if (auto factory = decode_factory_section(blob)) {
        out.check = check_camera_convention(out.rig, factory->rig());
        if (out.check.convention == CameraConvention::swapped) {
            out.rig = swap_camera_convention(out.rig);
            out.converted = true;
        }
    }
    return out;
}

}  // namespace einstar::calib
