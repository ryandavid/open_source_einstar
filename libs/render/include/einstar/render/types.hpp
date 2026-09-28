#pragma once

// Vertex/instance layouts shared by the renderer and producers (no Metal dependency).

#include <cstdint>

namespace einstar::render {

struct alignas(4) Rgba8 {
    std::uint8_t r = 255, g = 255, b = 255, a = 255;
};

// 28 bytes, matches the shader's packed layout.
struct PointVertex {
    float px, py, pz;
    float nx, ny, nz;
    Rgba8 color;
};
static_assert(sizeof(PointVertex) == 28);

enum class MarkerState : std::uint8_t { in_frame, in_map, global_fixed, rejected };

struct MarkerInstance {
    float px, py, pz;
    float nx, ny, nz;
    float radius;  // mm
    Rgba8 color;
};
static_assert(sizeof(MarkerInstance) == 32);

struct LineVertex {
    float px, py, pz;
    Rgba8 color;
};
static_assert(sizeof(LineVertex) == 16);

[[nodiscard]] constexpr Rgba8 marker_color(MarkerState s) {
    switch (s) {
        case MarkerState::in_frame: return {40, 220, 90, 255};
        case MarkerState::in_map: return {70, 140, 255, 255};
        case MarkerState::global_fixed: return {255, 190, 30, 255};
        case MarkerState::rejected: return {230, 50, 50, 255};
    }
    return {};
}

}  // namespace einstar::render
