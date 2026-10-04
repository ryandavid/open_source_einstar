#include "einstar/depth/point_image.hpp"
#include "einstar/depth/stereo.hpp"

#include <cmath>

#include <tbb/parallel_for.h>

namespace einstar::depth {

PointImage disparity_to_points(const ImageF32& disparity, const ImageF32& confidence, const RectifiedGeometry& g,
                               const PointImageParams& params) {
    const int w = disparity.width(), h = disparity.height();
    PointImage out{Image<Vec3f>(w, h, Vec3f::Zero()), Image<Vec3f>(w, h, Vec3f::Zero()), ImageF32(w, h, 0.0f)};

    tbb::parallel_for(0, h, [&](int y) {
        for (int x = 0; x < w; ++x) {
            const float d = disparity(x, y);
            if (!valid_disparity(d) || d + g.cx_offset <= 0.0) continue;
            const double z = g.depth_from_disparity(d);
            if (z < params.min_depth || z > params.max_depth) continue;
            out.points(x, y) = Vec3f(static_cast<float>((x - g.cx) * z / g.f), static_cast<float>((y - g.cy) * z / g.f),
                                     static_cast<float>(z));
            out.weights(x, y) = confidence.empty() ? 1.0f : confidence(x, y);
        }
    });

    // Normals from central differences, skipping depth discontinuities; weights (PointImageParams).
    const Vec3f right_centre(static_cast<float>(g.baseline), 0.0f, 0.0f);
    tbb::parallel_for(0, h, [&](int y) {
        for (int x = 0; x < w; ++x) {
            const Vec3f& p = out.points(x, y);
            if (p.z() == 0.0f) continue;
            float view = params.edge_weight;
            if (x > 0 && y > 0 && x + 1 < w && y + 1 < h) {
                const Vec3f& l = out.points(x - 1, y);
                const Vec3f& r = out.points(x + 1, y);
                const Vec3f& u = out.points(x, y - 1);
                const Vec3f& dn = out.points(x, y + 1);
                if (l.z() != 0 && r.z() != 0 && u.z() != 0 && dn.z() != 0 && std::abs(l.z() - r.z()) <= params.max_depth_jump &&
                    std::abs(u.z() - dn.z()) <= params.max_depth_jump) {
                    Vec3f n = (r - l).cross(dn - u);
                    const float len = n.norm();
                    if (len >= 1e-9f) {
                        n /= len;
                        if (n.dot(p) > 0) n = -n;
                        out.normals(x, y) = n;
                        view = std::max(0.0f, std::min(-n.dot(p.normalized()), -n.dot((p - right_centre).normalized())));
                    }
                }
            }
            out.weights(x, y) *= view * depth_noise_weight(p.z(), params.weight_reference_depth);
        }
    });
    return out;
}

}  // namespace einstar::depth
