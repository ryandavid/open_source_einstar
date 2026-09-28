#pragma once

// Live-view renderer: point splats (model + current frame), marker discs and line overlays
// (scanner frustum, trajectory, lost-tracking ghost). Draws into a caller-provided render pass.

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include <Metal/Metal.hpp>

#include "einstar/gpu/context.hpp"
#include "einstar/render/overlays.hpp"
#include "einstar/render/types.hpp"
#include "einstar/render/view_camera.hpp"

namespace einstar::render {

struct RenderSettings {
    float point_size_mm = 0.6f;   // splat diameter in world units
    float min_point_px = 1.5f;    // keep splats visible when zoomed out
    bool lighting = true;
};

class SceneRenderer {
public:
    static Result<std::unique_ptr<SceneRenderer>> create(std::shared_ptr<gpu::Context> ctx, MTL::PixelFormat color_format,
                                                         MTL::PixelFormat depth_format);

    // Layers: replace contents wholesale (the model layer supports incremental appends).
    void set_model_points(std::span<const PointVertex> points);
    void append_model_points(std::span<const PointVertex> points);
    void set_frame_points(std::span<const PointVertex> points);
    void set_markers(std::span<const MarkerInstance> markers);
    void set_lines(std::span<const LineVertex> lines);  // pairs of vertices (line list)

    [[nodiscard]] std::size_t model_point_count() const { return model_.count; }

    void encode(MTL::RenderCommandEncoder* enc, const ViewCamera& cam, float viewport_w, float viewport_h,
                const RenderSettings& settings);

private:
    struct Layer {
        gpu::Ref<MTL::Buffer> buffer;
        std::size_t count = 0;
        std::size_t capacity = 0;
    };
    void upload(Layer& layer, const void* data, std::size_t count, std::size_t stride, bool append);

    std::shared_ptr<gpu::Context> ctx_;
    gpu::Ref<MTL::RenderPipelineState> splat_pso_;
    gpu::Ref<MTL::RenderPipelineState> marker_pso_;
    gpu::Ref<MTL::RenderPipelineState> line_pso_;
    gpu::Ref<MTL::DepthStencilState> depth_state_;
    Layer model_, frame_, markers_, lines_;
};

}  // namespace einstar::render
