#include "einstar/core/lasso.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace einstar {

void LassoSelection::add(LassoStroke stroke) {
    const auto& poly = stroke.polygon;
    if (poly.size() < 3) return;
    Eigen::Vector2f lo = poly.front(), hi = poly.front();
    for (const auto& p : poly) {
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    }
    // The mask covers the polygon's pixels inside the viewport.
    const int x0 = std::max(0, static_cast<int>(std::floor(lo.x()))), y0 = std::max(0, static_cast<int>(std::floor(lo.y())));
    const int x1 = std::min(static_cast<int>(std::ceil(stroke.viewport.x())), static_cast<int>(std::ceil(hi.x())));
    const int y1 = std::min(static_cast<int>(std::ceil(stroke.viewport.y())), static_cast<int>(std::ceil(hi.y())));
    if (x1 <= x0 || y1 <= y0) return;
    const int w = x1 - x0, h = y1 - y0;
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0);
    // Even-odd fill of pixel centres, row by row.
    std::vector<float> crossings;
    bool any = false;
    for (int y = 0; y < h; ++y) {
        const float yc = static_cast<float>(y0 + y) + 0.5f;
        crossings.clear();
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const Eigen::Vector2f& a = poly[i];
            const Eigen::Vector2f& b = poly[(i + 1) % poly.size()];
            if ((a.y() <= yc) != (b.y() <= yc)) crossings.push_back(a.x() + (yc - a.y()) * (b.x() - a.x()) / (b.y() - a.y()));
        }
        std::ranges::sort(crossings);
        for (std::size_t i = 0; i + 1 < crossings.size(); i += 2) {
            // Pixels whose centre (x + 0.5) lies in [crossings[i], crossings[i + 1]).
            const int from = std::max(x0, static_cast<int>(std::ceil(crossings[i] - 0.5f)));
            const int to = std::min(x1, static_cast<int>(std::ceil(crossings[i + 1] - 0.5f)));
            for (int x = from; x < to; ++x) {
                mask[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x - x0)] = 1;
                any = true;
            }
        }
    }
    if (!any) return;

    GpuLassoStroke g{};
    std::copy_n(stroke.view_proj.data(), 16, g.view_proj);
    g.viewport[0] = stroke.viewport.x();
    g.viewport[1] = stroke.viewport.y();
    g.origin[0] = x0;
    g.origin[1] = y0;
    g.size[0] = w;
    g.size[1] = h;
    g.offset = static_cast<std::uint32_t>(masks_.size());
    g.subtract = stroke.subtract ? 1u : 0u;
    gpu_.push_back(g);
    masks_.insert(masks_.end(), mask.begin(), mask.end());
    strokes_.push_back(std::move(stroke));
}

void LassoSelection::clear() {
    strokes_.clear();
    gpu_.clear();
    masks_.clear();
}

bool LassoSelection::empty() const {
    return std::ranges::none_of(gpu_, [](const GpuLassoStroke& g) { return g.subtract == 0; });
}

bool LassoSelection::contains(const Eigen::Vector3f& world) const {
    bool selected = false;
    for (std::size_t i = 0; i < gpu_.size(); ++i) {
        const GpuLassoStroke& s = gpu_[i];
        const Eigen::Vector4f clip = strokes_[i].view_proj * world.homogeneous();
        bool inside = false;
        if (clip.w() > 1e-6f) {
            // The same arithmetic as lasso.metal.inc; bounds are checked in float before any conversion.
            const float px = (clip.x() / clip.w() * 0.5f + 0.5f) * s.viewport[0] - static_cast<float>(s.origin[0]);
            const float py = (0.5f - clip.y() / clip.w() * 0.5f) * s.viewport[1] - static_cast<float>(s.origin[1]);
            if (px >= 0 && py >= 0 && px < static_cast<float>(s.size[0]) && py < static_cast<float>(s.size[1])) {
                const auto ix = static_cast<std::size_t>(px), iy = static_cast<std::size_t>(py);
                inside = masks_[s.offset + iy * static_cast<std::size_t>(s.size[0]) + ix] != 0;
            }
        }
        selected = s.subtract ? (selected && !inside) : (selected || inside);
    }
    return selected;
}

}  // namespace einstar
