#include "einstar/track/frame.hpp"

#include <cmath>

#include <tbb/parallel_for.h>

namespace einstar::track {

void DepthFrame::ensure_cpu() const {
    if (!device || !points.empty()) return;
    const int w = device->width(), h = device->height();
    points = Image<Vec3f>(w, h);
    normals = Image<Vec3f>(w, h);
    weights = ImageF32(w, h);
    const float* p = device->points_xyzw();
    const float* n = device->normals_xyzw();
    const float* wt = device->weights();
    const auto count = static_cast<std::size_t>(w * h);
    for (std::size_t i = 0; i < count; ++i) {
        points.data()[i] = Vec3f(p[4 * i], p[4 * i + 1], p[4 * i + 2]);
        normals.data()[i] = Vec3f(n[4 * i], n[4 * i + 1], n[4 * i + 2]);
        weights.data()[i] = wt[i];
    }
}

DepthFrame make_depth_frame(const ImageF32& depth, const Intrinsics& k, double max_jump) {
    const int w = depth.width(), h = depth.height();
    DepthFrame f;
    f.intrinsics = k;
    f.points = Image<Vec3f>(w, h, Vec3f::Zero());
    f.normals = Image<Vec3f>(w, h, Vec3f::Zero());
    f.weights = ImageF32(w, h, 0.0f);
    tbb::parallel_for(0, h, [&](int v) {
        for (int u = 0; u < w; ++u) {
            const float z = depth(u, v);
            if (z <= 0) continue;
            f.points(u, v) = Vec3f(static_cast<float>((u - k.cx) * z / k.fx), static_cast<float>((v - k.cy) * z / k.fy), z);
        }
    });
    const auto jump = static_cast<float>(max_jump);
    tbb::parallel_for(1, h - 1, [&](int v) {
        for (int u = 1; u < w - 1; ++u) {
            const Vec3f& p = f.points(u, v);
            if (p.z() <= 0) continue;
            const Vec3f& l = f.points(u - 1, v);
            const Vec3f& r = f.points(u + 1, v);
            const Vec3f& t = f.points(u, v - 1);
            const Vec3f& b = f.points(u, v + 1);
            if (l.z() <= 0 || r.z() <= 0 || t.z() <= 0 || b.z() <= 0) continue;
            if (std::abs(l.z() - r.z()) > jump || std::abs(t.z() - b.z()) > jump) continue;
            Vec3f n = (r - l).cross(b - t);
            const float len = n.norm();
            if (len < 1e-9f) continue;
            n /= len;
            if (n.dot(p) > 0) n = -n;
            f.normals(u, v) = n;
            f.weights(u, v) = std::max(0.05f, -n.dot(p.normalized()));
        }
    });
    return f;
}

}  // namespace einstar::track
