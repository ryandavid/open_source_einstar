#pragma once

// Shared synthetic setup for the end-to-end tests: an Einstar-like rig, a hand-held style sweep and
// a cluttered speckle scene.

#include <filesystem>
#include <string>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/synth/speckle_scene.hpp"

namespace einstar::e2e {


inline RigCalibration einstar_like_rig() {
    // Real calibration if EXStar's cache is present, otherwise a rig with the same geometry.
    const char* cache = "/Applications/EXStar.app/Contents/Resources/res/Einscan-E10/200x150";
    if (std::filesystem::exists(std::string(cache) + "/LeftCCF.txt"))
        if (auto cal = calib::load_ccf_directory(cache)) return cal->rig();
    RigCalibration rig;
    rig.left.width = rig.right.width = 1280;
    rig.left.height = rig.right.height = 1024;
    rig.left.fx = rig.left.fy = 1157.3;
    rig.left.cx = 625.4;
    rig.left.cy = 522.4;
    rig.left.dist = {-0.156, 0.158, 0, 0.0003, 0.039};
    rig.right = rig.left;
    rig.right.cx = 633.8;
    rig.right.cy = 506.0;
    SE3 T = SE3::Identity();
    T.linear() = Eigen::AngleAxisd(-22.15 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
    T.translation() = -T.linear() * Vec3(156.9, 0.2, -30.7);
    rig.T_right_left = T;
    return rig;
}

// Scanner pose (left IR camera in world) for a frame: slow arc around the object.
inline SE3 truth_pose(std::uint32_t frame) {
    // Hand-held style sweep: the scanner slides sideways and rises while its aim drifts, so the
    // whole view (plane, sphere and boxes) moves in the camera frame.
    const double t = 0.05 * frame;
    const Vec3 eye(-190 + 55 * t + 15 * std::sin(1.3 * t), -130 - 12 * t, -240 + 20 * std::sin(0.7 * t));
    const Vec3 target(-100 + 30 * t, 25 + 8 * std::sin(0.9 * t), 10 + 10 * t);
    const Vec3 fwd = (target - eye).normalized();
    const Vec3 right = Vec3(0, 1, 0).cross(fwd).normalized() * -1.0;
    const Vec3 down = fwd.cross(right);
    SE3 T = SE3::Identity();
    T.linear().col(0) = right;
    T.linear().col(1) = down;
    T.linear().col(2) = fwd;
    T.translation() = eye;
    // Toe-in: the left camera looks ~11 deg to the right of the rig's bisector.
    T.linear() = T.linear() * Eigen::AngleAxisd(11.07 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
    return T;
}


struct SyntheticSetup {
    synth::Scene scene;
    synth::Projector proj;
};

inline SyntheticSetup make_scene() {
    synth::Scene scene;
    scene.primitives.push_back(synth::Plane{Vec3(0, 70, 0), Vec3(0, -1, 0)});
    // Cluttered, asymmetric arrangement so every view constrains all six degrees of freedom.
    auto box = [&](Vec3 c, Vec3 half, double yaw_deg, double tilt_deg) {
        SE3 T = SE3::Identity();
        T.linear() = (Eigen::AngleAxisd(yaw_deg * M_PI / 180, Vec3::UnitY()) *
                      Eigen::AngleAxisd(tilt_deg * M_PI / 180, Vec3::UnitX())).toRotationMatrix();
        T.translation() = c;
        scene.primitives.push_back(synth::Box{T, half});
    };
    scene.primitives.push_back(synth::Sphere{Vec3(-80, 20, 20), 45.0});
    box(Vec3(-10, 45, 40), Vec3(25, 25, 18), 30, 0);
    box(Vec3(-150, 50, -10), Vec3(20, 20, 30), -20, 0);
    box(Vec3(-60, 55, -70), Vec3(35, 15, 12), 55, 0);
    box(Vec3(-120, 30, 70), Vec3(12, 40, 12), 10, 15);
    scene.primitives.push_back(synth::Sphere{Vec3(20, 55, -40), 15.0});
    synth::Projector proj;
    proj.model.fx = proj.model.fy = 800;
    proj.model.cx = 640;
    proj.model.cy = 400;
    proj.pattern = synth::DotPattern::random(1280, 800, 9000, 3.5, 11);

    return {std::move(scene), std::move(proj)};
}

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
