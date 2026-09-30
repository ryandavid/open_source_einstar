#pragma once

// Shared synthetic setup for the end-to-end tests: an Einstar-like rig, a hand-held style sweep and
// a cluttered speckle scene.

#include <filesystem>
#include <string>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/synth/demo.hpp"

namespace einstar::e2e {


inline RigCalibration einstar_like_rig() {
    // The scanner's calibration (tests/fixtures), so the scenes are the same on every machine.
    auto cal = calib::load_ccf_directory(EINSTAR_TEST_CALIBRATION_DIR);
    return cal ? cal->rig() : synth::synthetic_einstar_rig();
}

// Scanner pose (left IR camera in world) for a frame: slow arc around the object.
inline SE3 truth_pose(std::uint32_t frame) {
    // Hand-held style sweep: the scanner slides sideways and rises while its aim drifts, so the
    // whole view (plane, sphere and boxes) moves in the camera frame.
    const double t = 0.05 * frame;
    const Vec3 eye(-190 + 55 * t + 15 * std::sin(1.3 * t), -130 - 12 * t, -240 + 20 * std::sin(0.7 * t));
    const Vec3 target(-100 + 30 * t, 25 + 8 * std::sin(0.9 * t), 10 + 10 * t);
    SE3 T = synth::look_at(eye, target);
    // Toe-in: the left camera looks ~11 deg to the right of the rig's bisector.
    T.linear() = T.linear() * Eigen::AngleAxisd(11.07 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
    return T;
}


struct SyntheticSetup {
    synth::Scene scene;
    synth::Projector proj;
};

inline SyntheticSetup make_scene() { return {synth::table_scene(), synth::speckle_projector()}; }

// Renders the (left, right) raw IR pair for a scanner pose (left camera in world).
inline void render_sensor(const SyntheticSetup& s, const RigCalibration& rig, const SE3& T_wl, int sensor, std::uint32_t seed, ImageU8& out) {
    const SE3 T_left_right = rig.T_right_left.inverse();
    const SE3 T_wr = T_wl * T_left_right;
    synth::Projector p = s.proj;
    p.T_world_projector = T_wl;
    p.T_world_projector.translation() = T_wl * (0.5 * T_left_right.translation());
    p.T_world_projector.linear() = T_wl.linear() * Eigen::AngleAxisd(-11.07 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
    synth::RenderParams rp;
    rp.supersample = 1;
    rp.seed = seed;
    out = synth::render_view(s.scene, p, sensor == 0 ? rig.left : rig.right, sensor == 0 ? T_wl : T_wr, rp).image;
}

}  // namespace einstar::e2e
