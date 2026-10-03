#pragma once

// Live-view renderer: point splats (model + current frame), marker discs and line overlays
// (scanner frustum, trajectory, lost-tracking ghost). Draws into a caller-provided render pass.

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include <Metal/Metal.hpp>

#include "einstar/core/lasso.hpp"
#include "einstar/gpu/context.hpp"
#include "einstar/render/overlays.hpp"
#include "einstar/render/types.hpp"
#include "einstar/render/view_camera.hpp"

namespace einstar::render {

struct RenderSettings {
    float point_size_mm = 0.6f;   // splat diameter in world units
    float min_point_px = 1.5f;    // keep splats visible when zoomed out
    bool lighting = true;
    bool show_points = true;      // model / frame splats
    bool show_mesh = true;        // processed mesh, when one is set
    bool show_scanner = true;     // the scanner model, when it has a pose
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
    // Zero-copy: draw PointVertex records that already live in a GPU buffer (the renderer keeps a
    // reference; the producer must not modify the buffer afterwards).
    void set_model_buffer(gpu::Ref<MTL::Buffer> buffer, std::size_t count);
    void set_frame_buffer(gpu::Ref<MTL::Buffer> buffer, std::size_t count);
    // Processed triangle mesh (replaces any previous one; empty clears it).
    void set_mesh(std::span<const MeshVertex> vertices, std::span<const std::uint32_t> indices);
    // A colour per mesh vertex (labels, a deviation map); empty: the default grey. Set after set_mesh (which
    // clears them); recolouring does not upload the geometry again.
    void set_mesh_colors(std::span<const Rgba8> colors);
    [[nodiscard]] bool has_mesh() const { return mesh_index_count_ > 0; }
    // Model points the selection contains are highlighted (an empty selection highlights nothing).
    void set_selection(const LassoSelection& selection);
    // The scanner model (scanner_model.hpp) at this pose; nullopt hides it.
    void set_scanner_pose(const std::optional<Mat4f>& T_world_scanner) { scanner_pose_ = T_world_scanner; }

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
    gpu::Ref<MTL::RenderPipelineState> mesh_pso_;
    gpu::Ref<MTL::RenderPipelineState> solid_pso_;
    gpu::Ref<MTL::Buffer> scanner_vertices_;  // the scanner model's triangles, in its frame
    std::size_t scanner_vertex_count_ = 0;
    std::optional<Mat4f> scanner_pose_;
    gpu::Ref<MTL::Buffer> lasso_strokes_, lasso_masks_;
    std::uint32_t lasso_count_ = 0;
    gpu::Ref<MTL::Buffer> mesh_vertices_, mesh_indices_, mesh_colors_;
    std::size_t mesh_index_count_ = 0;
    gpu::Ref<MTL::DepthStencilState> depth_state_;
    Layer model_, frame_, markers_, lines_;
};

}  // namespace einstar::render
