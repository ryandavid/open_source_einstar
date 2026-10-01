#include "einstar/track/frame.hpp"

#include <algorithm>
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

void filter_depth_edges(ImageF32& depth, const DepthEdgeFilter& p) {
    const int w = depth.width(), h = depth.height();
    if (w < 3 || h < 3 || (p.radius_px <= 0 && p.radius_rim_px <= 0)) return;
    std::vector<std::uint8_t> step(static_cast<std::size_t>(w) * h, 0), rim(static_cast<std::size_t>(w) * h, 0);
    tbb::parallel_for(0, h, [&](int y) {
        for (int x = 0; x < w; ++x) {
            const float z = depth(x, y);
            if (z <= 0) continue;
            const float jump = std::max(p.min_jump_mm, p.jump_ratio * z);
            bool is_step = false, is_rim = x == 0 || y == 0 || x == w - 1 || y == h - 1;
            for (int dy = -1; dy <= 1 && !(is_step && is_rim); ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx = x + dx, yy = y + dy;
                    if ((dx == 0 && dy == 0) || xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
                    const float n = depth(xx, yy);
                    if (n <= 0) is_rim = true;
                    else if (std::abs(n - z) > jump) is_step = true;
                }
            step[static_cast<std::size_t>(y) * w + x] = is_step;
            rim[static_cast<std::size_t>(y) * w + x] = is_rim;
        }
    });
    // Separable max filter of each mask, then drop the pixels within the radius.
    auto dilate = [&](std::vector<std::uint8_t>& m, int r) {
        if (r <= 0) return;
        std::vector<std::uint8_t> tmp(m.size(), 0);
        tbb::parallel_for(0, h, [&](int y) {
            for (int x = 0; x < w; ++x) {
                std::uint8_t v = 0;
                for (int d = -r; d <= r && !v; ++d) {
                    const int xx = std::clamp(x + d, 0, w - 1);
                    v = m[static_cast<std::size_t>(y) * w + xx];
                }
                tmp[static_cast<std::size_t>(y) * w + x] = v;
            }
        });
        tbb::parallel_for(0, h, [&](int y) {
            for (int x = 0; x < w; ++x) {
                std::uint8_t v = 0;
                for (int d = -r; d <= r && !v; ++d) {
                    const int yy = std::clamp(y + d, 0, h - 1);
                    v = tmp[static_cast<std::size_t>(yy) * w + x];
                }
                m[static_cast<std::size_t>(y) * w + x] = v;
            }
        });
    };
    if (p.radius_px > 0) dilate(step, p.radius_px - 1);  // radius 1 = the step pixels themselves
    else std::ranges::fill(step, std::uint8_t{0});
    if (p.radius_rim_px > 0) dilate(rim, p.radius_rim_px - 1);
    else std::ranges::fill(rim, std::uint8_t{0});
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            if (step[static_cast<std::size_t>(y) * w + x] || rim[static_cast<std::size_t>(y) * w + x]) depth(x, y) = 0.0f;
    if (p.min_region_px <= 0) return;
    // Speckle filter: flood fill over 4-neighbours whose depths agree; small regions are dropped.
    std::vector<int> label(static_cast<std::size_t>(w) * h, -1);
    std::vector<int> stack, members;
    for (int y0 = 0; y0 < h; ++y0)
        for (int x0 = 0; x0 < w; ++x0) {
            const std::size_t i0 = static_cast<std::size_t>(y0) * w + x0;
            if (label[i0] >= 0 || depth(x0, y0) <= 0) continue;
            members.clear();
            stack.assign(1, static_cast<int>(i0));
            label[i0] = 1;
            while (!stack.empty()) {
                const int i = stack.back();
                stack.pop_back();
                members.push_back(i);
                const int x = i % w, y = i / w;
                const float z = depth(x, y);
                const float jump = std::max(p.min_jump_mm, p.jump_ratio * z);
                const int nx[4] = {x - 1, x + 1, x, x}, ny[4] = {y, y, y - 1, y + 1};
                for (int k = 0; k < 4; ++k) {
                    if (nx[k] < 0 || ny[k] < 0 || nx[k] >= w || ny[k] >= h) continue;
                    const std::size_t j = static_cast<std::size_t>(ny[k]) * w + nx[k];
                    if (label[j] >= 0) continue;
                    const float zn = depth(nx[k], ny[k]);
                    if (zn <= 0 || std::abs(zn - z) > jump) continue;
                    label[j] = 1;
                    stack.push_back(static_cast<int>(j));
                }
            }
            if (static_cast<int>(members.size()) < p.min_region_px)
                for (const int i : members) depth(i % w, i / w) = 0.0f;
        }
}

void filter_grazing(DepthFrame& f, const GrazingFilter& p) {
    f.ensure_cpu();
    const int w = f.points.width(), h = f.points.height();
    if (w < 3 || h < 3) return;
    auto valid = [&](int x, int y) { return f.points(x, y).z() > 0; };
    // cos of the angle between the surface normal and the line of sight (1 = face-on), or -1 if unknown.
    auto facing = [&](int x, int y) -> double {
        const Vec3f& n = f.normals(x, y);
        if (n.squaredNorm() == 0) return -1;
        return std::max(0.0, -static_cast<double>(n.dot(f.points(x, y).normalized())));
    };
    const double cos_max = p.max_view_angle_deg > 0 ? std::cos(p.max_view_angle_deg * M_PI / 180.0) : -1;
    const double cos_rim = std::cos(p.rim_angle_deg * M_PI / 180.0);
    std::vector<std::uint8_t> drop(static_cast<std::size_t>(w) * h, 0), steep_rim(static_cast<std::size_t>(w) * h, 0);
    tbb::parallel_for(0, h, [&](int y) {
        for (int x = 0; x < w; ++x) {
            if (!valid(x, y)) continue;
            bool rim = x == 0 || y == 0 || x == w - 1 || y == h - 1;
            for (int dy = -1; dy <= 1 && !rim; ++dy)
                for (int dx = -1; dx <= 1 && !rim; ++dx) rim = !valid(x + dx, y + dy);
            const double c = facing(x, y);
            // Inside the valid region a missing normal means the neighbours are too far apart: a depth
            // step or a surface too steep to measure.
            if (cos_max > -1 && ((c >= 0 && c < cos_max) || (c < 0 && !rim))) drop[static_cast<std::size_t>(y) * w + x] = 1;
            if (p.rim_px <= 0 || !rim) continue;
            // Rim pixels have no normal of their own (a neighbour is missing): use the nearest one inside.
            double best = -1;
            int best_d = 99;
            for (int dy = -2; dy <= 2; ++dy)
                for (int dx = -2; dx <= 2; ++dx) {
                    const int xx = x + dx, yy = y + dy;
                    if (xx < 0 || yy < 0 || xx >= w || yy >= h || !valid(xx, yy)) continue;
                    const double cc = facing(xx, yy);
                    const int d = std::max(std::abs(dx), std::abs(dy));
                    if (cc >= 0 && d < best_d) best = cc, best_d = d;
                }
            if (best < 0 || best < cos_rim) steep_rim[static_cast<std::size_t>(y) * w + x] = 1;  // unknown counts as steep
        }
    });
    // Widen the steep rim to rim_px pixels, into the valid region.
    for (int pass = 1; pass < p.rim_px; ++pass) {
        auto next = steep_rim;
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                if (steep_rim[static_cast<std::size_t>(y) * w + x] || !valid(x, y)) continue;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int xx = x + dx, yy = y + dy;
                        if (xx >= 0 && yy >= 0 && xx < w && yy < h && steep_rim[static_cast<std::size_t>(yy) * w + xx]) next[static_cast<std::size_t>(y) * w + x] = 1;
                    }
            }
        steep_rim.swap(next);
    }
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            if (drop[static_cast<std::size_t>(y) * w + x] || steep_rim[static_cast<std::size_t>(y) * w + x]) {
                f.points(x, y) = Vec3f::Zero();
                f.normals(x, y) = Vec3f::Zero();
                if (!f.weights.empty()) f.weights(x, y) = 0.0f;
            }
}

}  // namespace einstar::track
