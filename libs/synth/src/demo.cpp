#include "einstar/synth/demo.hpp"

#include <random>

namespace einstar::synth {

SE3 look_at(const Vec3& eye, const Vec3& target, const Vec3& down) {
    const Vec3 f = (target - eye).normalized();
    Vec3 r = f.cross(down);
    if (r.norm() < 1e-9) r = f.cross(Vec3::UnitX());
    r.normalize();
    SE3 T = SE3::Identity();
    T.linear().col(0) = r;
    T.linear().col(1) = f.cross(r);
    T.linear().col(2) = f;
    T.translation() = eye;
    return T;
}

RigCalibration synthetic_einstar_rig() {
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
    T.linear() = Eigen::AngleAxisd(22.15 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
    T.translation() = -T.linear() * Vec3(156.9, 0.2, 30.7);
    rig.T_right_left = T;
    return rig;
}

Scene table_scene() {
    Scene scene;
    scene.primitives.push_back(Plane{Vec3(0, 70, 0), Vec3(0, -1, 0)});
    auto box = [&](Vec3 c, Vec3 half, double yaw_deg, double tilt_deg) {
        SE3 T = SE3::Identity();
        T.linear() = (Eigen::AngleAxisd(yaw_deg * M_PI / 180, Vec3::UnitY()) * Eigen::AngleAxisd(tilt_deg * M_PI / 180, Vec3::UnitX()))
                         .toRotationMatrix();
        T.translation() = c;
        scene.primitives.push_back(Box{T, half});
    };
    scene.primitives.push_back(Sphere{Vec3(-80, 20, 20), 45.0});
    box(Vec3(-10, 45, 40), Vec3(25, 25, 18), 30, 0);
    box(Vec3(-150, 50, -10), Vec3(20, 20, 30), -20, 0);
    box(Vec3(-60, 55, -70), Vec3(35, 15, 12), 55, 0);
    box(Vec3(-120, 30, 70), Vec3(12, 40, 12), 10, 15);
    scene.primitives.push_back(Sphere{Vec3(20, 55, -40), 15.0});
    return scene;
}

Projector speckle_projector() {
    Projector p;
    p.model.fx = p.model.fy = 800;
    p.model.cx = 640;
    p.model.cy = 400;
    p.pattern = DotPattern::random(1280, 800, 9000, 3.5, 11);
    return p;
}

std::vector<Marker> scatter_markers(int count, std::uint32_t seed, double x_min, double x_max, double z_min, double z_max,
                                    double min_spacing_mm) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ux(x_min, x_max), uz(z_min, z_max);
    std::vector<Marker> out;
    for (int tries = 0; tries < 20000 && static_cast<int>(out.size()) < count; ++tries) {
        const Vec3 c(ux(rng), 70.0, uz(rng));
        bool ok = true;
        for (const auto& m : out) ok = ok && (m.center - c).norm() > min_spacing_mm;
        if (ok) out.push_back({c, Vec3(0, -1, 0), 6.0, 10.0});
    }
    return out;
}

}  // namespace einstar::synth
