#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace einstar {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Vec3f = Eigen::Vector3f;

// Rigid transform. Convention: T_a_b maps points expressed in frame b into frame a.
using SE3 = Eigen::Isometry3d;

[[nodiscard]] inline Mat3 hat(const Vec3& w) {
    Mat3 m;
    m << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
    return m;
}

// so(3) exponential (Rodrigues).
[[nodiscard]] inline Mat3 so3_exp(const Vec3& w) {
    const double theta = w.norm();
    if (theta < 1e-12) return Mat3::Identity() + hat(w);
    return Eigen::AngleAxisd(theta, w / theta).toRotationMatrix();
}

[[nodiscard]] inline Vec3 so3_log(const Mat3& r) {
    const Eigen::AngleAxisd aa(r);
    return aa.angle() * aa.axis();
}

// se(3) exponential with twist ordered (v, w): translation first, rotation second.
[[nodiscard]] inline SE3 se3_exp(const Vec6& xi) {
    const Vec3 v = xi.head<3>();
    const Vec3 w = xi.tail<3>();
    const double theta = w.norm();
    const Mat3 W = hat(w);
    Mat3 V;
    if (theta < 1e-9) {
        V = Mat3::Identity() + 0.5 * W;
    } else {
        const double t2 = theta * theta;
        V = Mat3::Identity() + (1.0 - std::cos(theta)) / t2 * W + (theta - std::sin(theta)) / (t2 * theta) * W * W;
    }
    SE3 out = SE3::Identity();
    out.linear() = so3_exp(w);
    out.translation() = V * v;
    return out;
}

[[nodiscard]] inline Vec6 se3_log(const SE3& t) {
    const Vec3 w = so3_log(t.linear());
    const double theta = w.norm();
    const Mat3 W = hat(w);
    Mat3 v_inv;
    if (theta < 1e-9) {
        v_inv = Mat3::Identity() - 0.5 * W;
    } else {
        const double half = 0.5 * theta;
        v_inv = Mat3::Identity() - 0.5 * W +
                (1.0 - half * std::cos(half) / std::sin(half)) / (theta * theta) * W * W;
    }
    Vec6 xi;
    xi.head<3>() = v_inv * t.translation();
    xi.tail<3>() = w;
    return xi;
}

// Rotation angle (rad) and translation magnitude of a relative motion.
[[nodiscard]] inline double rotation_angle(const SE3& t) { return Eigen::AngleAxisd(t.linear()).angle(); }
[[nodiscard]] inline double translation_norm(const SE3& t) { return t.translation().norm(); }

}  // namespace einstar
