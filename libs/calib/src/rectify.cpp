#include "einstar/calib/rectify.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <tbb/parallel_for.h>

namespace einstar::calib {

StereoRectification compute_rectification(const RigCalibration& rig, const RectificationOptions& options) {
    StereoRectification out;
    // Relative pose: right camera coords = R * left coords + t.
    const Mat3 R = rig.T_right_left.linear();
    const Vec3 t = rig.T_right_left.translation();

    // Split the relative rotation so both cameras turn by half.
    const Eigen::AngleAxisd aa(R);
    const Mat3 r_half = Eigen::AngleAxisd(-0.5 * aa.angle(), aa.axis()).toRotationMatrix();  // applied to right
    const Mat3 l_half = r_half.transpose();                                                // applied to left
    // Translation expressed after the half rotation of the right camera.
    const Vec3 tr = r_half * t;

    // New x axis along the baseline (pointing from left to right camera centre).
    // The right camera centre in left coords is -R^T t; after rotations both frames coincide,
    // so the baseline direction in the common frame is -tr.
    Vec3 e1 = -tr.normalized();
    Vec3 e2 = Vec3::UnitZ().cross(e1);
    if (e2.norm() < 1e-9) e2 = Vec3::UnitY();
    e2.normalize();
    const Vec3 e3 = e1.cross(e2);
    Mat3 r_align;
    r_align.row(0) = e1.transpose();
    r_align.row(1) = e2.transpose();
    r_align.row(2) = e3.transpose();

    out.R_left = r_align * l_half;
    out.R_right = r_align * r_half;

    // Intrinsics: one focal length (the smallest, so no image is upsampled) and one cy for both. The
    // window is placed on what both cameras see: each sensor's outline is mapped into the rectified
    // frame, the right one moved by the disparity of the reference depth, and the window centred on
    // their overlap. With the right camera's principal point shifted by that disparity, the scene at the
    // reference depth then occupies the same columns in both images.
    const double scale = options.scale;
    const double f = std::min({rig.left.fx, rig.left.fy, rig.right.fx, rig.right.fy}) * scale;
    // A sensor's extent in rectified coordinates (no principal point): mean of each edge's samples,
    // which ignores the corners that rectification rotates out of line.
    struct Extent { double x0, x1, y0, y1; };
    auto extent = [&](const CameraModel& cam, const Mat3& r) {
        constexpr int kSamples = 64;
        auto map = [&](double px, double py) {
            const Vec2 n = undistort_to_normalized(cam, Vec2(px, py));
            const Vec3 ray = r * Vec3(n.x(), n.y(), 1.0);
            return Vec2(f * ray.x() / ray.z(), f * ray.y() / ray.z());
        };
        Extent e{0, 0, 0, 0};
        const double sw = cam.width - 1.0, sh = cam.height - 1.0;
        for (int i = 0; i < kSamples; ++i) {
            const double s = (i + 0.5) / kSamples;
            e.x0 += map(0, s * sh).x() / kSamples;
            e.x1 += map(sw, s * sh).x() / kSamples;
            e.y0 += map(s * sw, 0).y() / kSamples;
            e.y1 += map(s * sw, sh).y() / kSamples;
        }
        return e;
    };
    const Extent el = extent(rig.left, out.R_left), er = extent(rig.right, out.R_right);
    const double baseline = rig.baseline_mm();
    const double shift = options.reference_depth_mm > 0 ? f * baseline / options.reference_depth_mm : 0.0;
    // Overlap at the reference depth, in left rectified coordinates: a right coordinate u is seen by
    // the left camera at u + f * B / z.
    const double ox0 = std::max(el.x0, er.x0 + shift), ox1 = std::min(el.x1, er.x1 + shift);
    const double oy0 = std::max(el.y0, er.y0), oy1 = std::min(el.y1, er.y1);
    const int w = static_cast<int>(std::lround(rig.left.width * scale));
    const int h = static_cast<int>(std::lround(rig.left.height * scale));
    out.rectified.width = w;
    out.rectified.height = h;
    out.rectified.fx = out.rectified.fy = f;
    if (options.reference_depth_mm > 0 && ox1 > ox0 && oy1 > oy0) {
        out.rectified.cx = 0.5 * (w - 1) - 0.5 * (ox0 + ox1);
        out.rectified.cy = 0.5 * (h - 1) - 0.5 * (oy0 + oy1);
    } else {
        // One shared principal point: the average of where each image centre lands (the old layout).
        auto rectified_center = [&](const CameraModel& cam, const Mat3& r) {
            const Vec2 n = undistort_to_normalized(cam, Vec2(cam.width * 0.5, cam.height * 0.5));
            const Vec3 ray = r * Vec3(n.x(), n.y(), 1.0);
            return Vec2(ray.x() / ray.z(), ray.y() / ray.z());
        };
        const Vec2 c = 0.5 * (rectified_center(rig.left, out.R_left) + rectified_center(rig.right, out.R_right));
        out.rectified.cx = w * 0.5 - f * c.x();
        out.rectified.cy = h * 0.5 - f * c.y();
    }
    out.rectified_right = out.rectified;
    out.rectified_right.cx = out.rectified.cx + shift;
    out.geometry = {f, out.rectified.cx, out.rectified.cy, baseline, shift};
    return out;
}

Vec2 undistort_to_normalized(const CameraModel& cam, const Vec2& pixel, int iterations) {
    // Initial guess ignoring distortion, then Newton iterations with a numeric Jacobian.
    const double y0 = (pixel.y() - cam.cy) / cam.fy;
    const double x0 = (pixel.x() - cam.cx - cam.skew * y0) / cam.fx;
    Vec2 n(x0, y0);
    constexpr double h = 1e-7;
    for (int i = 0; i < iterations; ++i) {
        const Vec2 err = pixel - cam.distort_normalized(n);
        if (err.squaredNorm() < 1e-20) break;
        Eigen::Matrix2d J;
        J.col(0) = (cam.distort_normalized(n + Vec2(h, 0)) - cam.distort_normalized(n - Vec2(h, 0))) / (2 * h);
        J.col(1) = (cam.distort_normalized(n + Vec2(0, h)) - cam.distort_normalized(n - Vec2(0, h))) / (2 * h);
        n += J.inverse() * err;
    }
    return n;
}

RemapTable build_remap(const CameraModel& original, const Mat3& R_rect, const CameraModel& rectified) {
    RemapTable t{rectified.width, rectified.height, ImageF32(rectified.width, rectified.height),
                 ImageF32(rectified.width, rectified.height)};
    const Mat3 r_inv = R_rect.transpose();
    tbb::parallel_for(0, rectified.height, [&](int y) {
        for (int x = 0; x < rectified.width; ++x) {
            const Vec3 ray_rect((x - rectified.cx) / rectified.fx, (y - rectified.cy) / rectified.fy, 1.0);
            const Vec3 ray = r_inv * ray_rect;
            const Vec2 src = original.distort_normalized(ray.head<2>() / ray.z());
            t.map_x(x, y) = static_cast<float>(src.x());
            t.map_y(x, y) = static_cast<float>(src.y());
        }
    });
    return t;
}

void remap(ImageView<const std::uint8_t> src, const RemapTable& table, ImageView<std::uint8_t> dst) {
    tbb::parallel_for(0, table.height, [&](int y) {
        for (int x = 0; x < table.width; ++x) {
            const float sx = table.map_x(x, y), sy = table.map_y(x, y);
            const int x0 = static_cast<int>(std::floor(sx)), y0 = static_cast<int>(std::floor(sy));
            if (x0 < 0 || y0 < 0 || x0 + 1 >= src.width || y0 + 1 >= src.height) {
                dst(x, y) = 0;
                continue;
            }
            const float fx = sx - static_cast<float>(x0), fy = sy - static_cast<float>(y0);
            const float a = src(x0, y0), b = src(x0 + 1, y0), c = src(x0, y0 + 1), d = src(x0 + 1, y0 + 1);
            const float v = (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy;
            dst(x, y) = static_cast<std::uint8_t>(std::clamp(v + 0.5f, 0.0f, 255.0f));
        }
    });
}

ImageU8 remap(ImageView<const std::uint8_t> src, const RemapTable& table) {
    ImageU8 out(table.width, table.height);
    remap(src, table, out.view());
    return out;
}

}  // namespace einstar::calib
