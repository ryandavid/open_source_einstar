#pragma once

// A lasso selection in 3D: polygons drawn on the screen, each selecting everything in front of the view it
// was drawn in that projects inside it (through all depths). Strokes combine in order: an additive stroke
// adds what it covers, a subtractive one removes it. Each polygon is rasterised once into a pixel mask and
// every test looks the point's pixel up in it, so the CPU (contains), the live GPU volume and the renderer
// (lasso.metal.inc, the same rule on the same masks) agree exactly.

#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace einstar {

struct LassoStroke {
    Eigen::Matrix4f view_proj = Eigen::Matrix4f::Identity();  // world -> clip of the view drawn in (Metal clip, w > 0 ahead)
    Eigen::Vector2f viewport{1, 1};                           // px
    std::vector<Eigen::Vector2f> polygon;                     // px, origin at the top left, y down
    bool subtract = false;
};

// The GPU form of a stroke (lasso.metal.inc: LassoStroke). 96 bytes, float4x4 first.
struct alignas(16) GpuLassoStroke {
    float view_proj[16];  // column-major
    float viewport[2];
    std::int32_t origin[2];  // the mask's top-left pixel
    std::int32_t size[2];    // the mask's width and height
    std::uint32_t offset;    // of the mask in the mask bytes
    std::uint32_t subtract;
};
static_assert(sizeof(GpuLassoStroke) == 96);

class LassoSelection {
public:
    // Adds a stroke (a polygon of fewer than three points, or of no area, is ignored).
    void add(LassoStroke stroke);
    void clear();
    // Nothing can be selected (no additive stroke).
    [[nodiscard]] bool empty() const;
    [[nodiscard]] const std::vector<LassoStroke>& strokes() const { return strokes_; }
    [[nodiscard]] bool contains(const Eigen::Vector3f& world) const;

    // For lasso.metal.inc: the strokes and their masks (one byte per pixel, 1 = inside).
    [[nodiscard]] const std::vector<GpuLassoStroke>& gpu_strokes() const { return gpu_; }
    [[nodiscard]] const std::vector<std::uint8_t>& mask_bytes() const { return masks_; }

private:
    std::vector<LassoStroke> strokes_;
    std::vector<GpuLassoStroke> gpu_;
    std::vector<std::uint8_t> masks_;
};

}  // namespace einstar
