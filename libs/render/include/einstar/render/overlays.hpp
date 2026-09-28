#pragma once

// CPU-side overlay geometry builders (scanner frustum, trajectory). No Metal dependency.

#include <span>
#include <vector>

#include <Eigen/Core>

#include "einstar/render/types.hpp"

namespace einstar::render {

using Vec3f = Eigen::Vector3f;

inline void append_frustum(std::vector<LineVertex>& out, const Eigen::Matrix4f& T, float fx, float fy, float cx, float cy,
                    int width, int height, float depth, Rgba8 color) {
    auto corner = [&](float u, float v) {
        const Eigen::Vector4f p((u - cx) / fx * depth, (v - cy) / fy * depth, depth, 1.0f);
        return Vec3f((T * p).head<3>());
    };
    const Vec3f o = T.block<3, 1>(0, 3);
    const Vec3f c[4] = {corner(0, 0), corner(static_cast<float>(width), 0),
                        corner(static_cast<float>(width), static_cast<float>(height)), corner(0, static_cast<float>(height))};
    auto seg = [&](const Vec3f& a, const Vec3f& b) {
        out.push_back({a.x(), a.y(), a.z(), color});
        out.push_back({b.x(), b.y(), b.z(), color});
    };
    for (int i = 0; i < 4; ++i) {
        seg(o, c[i]);
        seg(c[i], c[(i + 1) % 4]);
    }
}

inline void append_polyline(std::vector<LineVertex>& out, std::span<const Vec3f> pts, Rgba8 color) {
    for (std::size_t i = 1; i < pts.size(); ++i) {
        out.push_back({pts[i - 1].x(), pts[i - 1].y(), pts[i - 1].z(), color});
        out.push_back({pts[i].x(), pts[i].y(), pts[i].z(), color});
    }
}

}  // namespace einstar::render
