#include "einstar/render/scene_renderer.hpp"

#include <algorithm>
#include <cstring>
#include <format>
#include <vector>

#include "einstar/render/scanner_model.hpp"

namespace einstar::render {
namespace {

constexpr const char* kShaderSource =
#include "einstar/core/lasso.metal.inc"
#include "shaders.metal.inc"
    ;

struct Uniforms {
    Eigen::Matrix4f view;  // column-major, matches float4x4
    Eigen::Matrix4f proj;
    float viewport[2];
    float point_size;
    float min_point_px;
    float lighting;
    std::uint32_t lasso_count;
    float pad[2];
};
static_assert(sizeof(Uniforms) == 160);

Result<gpu::Ref<MTL::RenderPipelineState>> make_pipeline(gpu::Context& ctx, MTL::Library* lib, const char* vs,
                                                         const char* fs, MTL::PixelFormat color,
                                                         MTL::PixelFormat depth, bool blend) {
    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
    gpu::Ref<MTL::Function> vfn(lib->newFunction(gpu::ns_string(vs)));
    gpu::Ref<MTL::Function> ffn(lib->newFunction(gpu::ns_string(fs)));
    auto* desc = MTL::RenderPipelineDescriptor::alloc()->init();
    desc->setVertexFunction(vfn.get());
    desc->setFragmentFunction(ffn.get());
    desc->colorAttachments()->object(0)->setPixelFormat(color);
    if (blend) {
        auto* att = desc->colorAttachments()->object(0);
        att->setBlendingEnabled(true);
        att->setSourceRGBBlendFactor(MTL::BlendFactorSourceAlpha);
        att->setDestinationRGBBlendFactor(MTL::BlendFactorOneMinusSourceAlpha);
    }
    desc->setDepthAttachmentPixelFormat(depth);
    NS::Error* err = nullptr;
    gpu::Ref<MTL::RenderPipelineState> pso(ctx.device()->newRenderPipelineState(desc, &err));
    desc->release();
    if (!pso) {
        std::string msg = err ? err->localizedDescription()->utf8String() : "unknown";
        pool->release();
        return make_error(Errc::invalid_argument, std::format("render pipeline {}/{}: {}", vs, fs, msg));
    }
    pool->release();
    return pso;
}

}  // namespace

Result<std::unique_ptr<SceneRenderer>> SceneRenderer::create(std::shared_ptr<gpu::Context> ctx, MTL::PixelFormat color,
                                                             MTL::PixelFormat depth) {
    auto r = std::unique_ptr<SceneRenderer>(new SceneRenderer());
    r->ctx_ = std::move(ctx);
    auto lib = r->ctx_->library("render", kShaderSource);
    if (!lib) return std::unexpected(lib.error());
    auto splat = make_pipeline(*r->ctx_, *lib, "splat_vs", "splat_fs", color, depth, false);
    auto marker = make_pipeline(*r->ctx_, *lib, "marker_vs", "marker_fs", color, depth, false);
    auto line = make_pipeline(*r->ctx_, *lib, "line_vs", "line_fs", color, depth, true);
    auto mesh = make_pipeline(*r->ctx_, *lib, "mesh_vs", "mesh_fs", color, depth, false);
    if (!mesh) return std::unexpected(mesh.error());
    r->mesh_pso_ = std::move(*mesh);
    auto solid = make_pipeline(*r->ctx_, *lib, "solid_vs", "solid_fs", color, depth, false);
    if (!solid) return std::unexpected(solid.error());
    r->solid_pso_ = std::move(*solid);
    // Selection buffers are bound for every splat draw: placeholders until a selection is set.
    r->lasso_strokes_ = r->ctx_->mirrored_buffer(sizeof(GpuLassoStroke));
    r->lasso_masks_ = r->ctx_->mirrored_buffer(16);
    // The scanner model as triangles (fans over its convex faces), each face's vertices with its normal.
    {
        std::vector<PointVertex> tris;
        const auto& model = scanner_model();
        for (const auto* faces : {&model.body, &model.front})
            for (const auto& f : *faces) {
                const Vec3f n = (f.v[1] - f.v[0]).cross(f.v[2] - f.v[0]).normalized();
                const auto vertex = [&](const Vec3f& p) { return PointVertex{p.x(), p.y(), p.z(), n.x(), n.y(), n.z(), f.color}; };
                for (std::size_t i = 1; i + 1 < f.v.size(); ++i)
                    for (const auto* p : {&f.v[0], &f.v[i], &f.v[i + 1]}) tris.push_back(vertex(*p));
            }
        r->scanner_vertices_ = r->ctx_->mirrored_buffer(tris.size() * sizeof(PointVertex));
        if (!r->scanner_vertices_) return make_error(Errc::io, "scanner model buffer");
        std::memcpy(r->scanner_vertices_->contents(), tris.data(), tris.size() * sizeof(PointVertex));
        gpu::Context::cpu_modified(r->scanner_vertices_.get());
        r->scanner_vertex_count_ = tris.size();
    }
    if (!splat) return std::unexpected(splat.error());
    if (!marker) return std::unexpected(marker.error());
    if (!line) return std::unexpected(line.error());
    r->splat_pso_ = std::move(*splat);
    r->marker_pso_ = std::move(*marker);
    r->line_pso_ = std::move(*line);

    auto* ds = MTL::DepthStencilDescriptor::alloc()->init();
    ds->setDepthCompareFunction(MTL::CompareFunctionLess);
    ds->setDepthWriteEnabled(true);
    r->depth_state_ = gpu::Ref<MTL::DepthStencilState>(r->ctx_->device()->newDepthStencilState(ds));
    ds->release();
    return r;
}

void SceneRenderer::upload(Layer& layer, const void* data, std::size_t count, std::size_t stride, bool append) {
    const std::size_t start = append ? layer.count : 0;
    const std::size_t needed = start + count;
    std::size_t dirty_from = start;
    if (needed > layer.capacity || layer.capacity == 0) {
        dirty_from = 0;
        const std::size_t cap = std::max<std::size_t>(needed + needed / 2, 1024);
        auto grown = ctx_->mirrored_buffer(cap * stride);
        // (The old buffer may be an external GPU-only one: see set_model_buffer.)
        if (append && layer.buffer && layer.count > 0) ctx_->download(layer.buffer.get(), 0, grown->contents(), layer.count * stride);
        layer.buffer = std::move(grown);
        layer.capacity = cap;
    }
    if (count > 0) std::memcpy(static_cast<std::byte*>(layer.buffer->contents()) + start * stride, data, count * stride);
    gpu::Context::cpu_modified(layer.buffer.get(), dirty_from * stride, (needed - dirty_from) * stride);
    layer.count = needed;
}

void SceneRenderer::set_model_points(std::span<const PointVertex> p) { upload(model_, p.data(), p.size(), sizeof(PointVertex), false); }
void SceneRenderer::append_model_points(std::span<const PointVertex> p) { upload(model_, p.data(), p.size(), sizeof(PointVertex), true); }
void SceneRenderer::set_frame_points(std::span<const PointVertex> p) { upload(frame_, p.data(), p.size(), sizeof(PointVertex), false); }
void SceneRenderer::set_markers(std::span<const MarkerInstance> m) { upload(markers_, m.data(), m.size(), sizeof(MarkerInstance), false); }
void SceneRenderer::set_model_buffer(gpu::Ref<MTL::Buffer> buffer, std::size_t count) {
    model_.buffer = std::move(buffer);
    model_.count = count;
    model_.capacity = 0;  // external: never written into; the next upload allocates a new buffer
}

void SceneRenderer::set_frame_buffer(gpu::Ref<MTL::Buffer> buffer, std::size_t count) {
    frame_.buffer = std::move(buffer);
    frame_.count = count;
    frame_.capacity = 0;
}

void SceneRenderer::set_mesh_colors(std::span<const Rgba8> colors) {
    mesh_colors_ = {};
    if (colors.empty() || !mesh_vertices_ || colors.size() * sizeof(MeshVertex) != mesh_vertices_->length()) return;  // one per vertex
    mesh_colors_ = ctx_->mirrored_buffer(colors.size_bytes());
    if (!mesh_colors_) return;
    std::memcpy(mesh_colors_->contents(), colors.data(), colors.size_bytes());
    gpu::Context::cpu_modified(mesh_colors_.get());
}

void SceneRenderer::set_mesh(std::span<const MeshVertex> vertices, std::span<const std::uint32_t> indices) {
    mesh_index_count_ = 0;
    mesh_vertices_ = {};
    mesh_indices_ = {};
    mesh_colors_ = {};
    if (vertices.empty() || indices.empty()) return;
    mesh_vertices_ = ctx_->mirrored_buffer(vertices.size_bytes());
    mesh_indices_ = ctx_->mirrored_buffer(indices.size_bytes());
    if (!mesh_vertices_ || !mesh_indices_) return;
    std::memcpy(mesh_vertices_->contents(), vertices.data(), vertices.size_bytes());
    std::memcpy(mesh_indices_->contents(), indices.data(), indices.size_bytes());
    gpu::Context::cpu_modified(mesh_vertices_.get());
    gpu::Context::cpu_modified(mesh_indices_.get());
    mesh_index_count_ = indices.size();
}

void SceneRenderer::set_selection(const LassoSelection& selection) {
    const auto& strokes = selection.gpu_strokes();
    const auto& masks = selection.mask_bytes();
    lasso_count_ = 0;
    if (strokes.empty()) return;
    // Fresh buffers (a frame in flight keeps reading the old ones; command buffers retain them).
    const std::size_t stroke_bytes = strokes.size() * sizeof(GpuLassoStroke);
    auto stroke_buf = ctx_->mirrored_buffer(stroke_bytes);
    auto mask_buf = ctx_->mirrored_buffer(std::max<std::size_t>(masks.size(), 16));
    if (!stroke_buf || !mask_buf) return;
    lasso_strokes_ = std::move(stroke_buf);
    lasso_masks_ = std::move(mask_buf);
    std::memcpy(lasso_strokes_->contents(), strokes.data(), stroke_bytes);
    std::memcpy(lasso_masks_->contents(), masks.data(), masks.size());
    gpu::Context::cpu_modified(lasso_strokes_.get(), 0, stroke_bytes);
    gpu::Context::cpu_modified(lasso_masks_.get(), 0, masks.size());
    lasso_count_ = static_cast<std::uint32_t>(strokes.size());
}

void SceneRenderer::set_lines(std::span<const LineVertex> l) { upload(lines_, l.data(), l.size(), sizeof(LineVertex), false); }

void SceneRenderer::encode(MTL::RenderCommandEncoder* enc, const ViewCamera& cam, float vw, float vh,
                           const RenderSettings& s) {
    Uniforms u{};
    u.view = cam.view();
    u.proj = cam.projection(vw / std::max(vh, 1.0f));
    u.viewport[0] = vw;
    u.viewport[1] = vh;
    u.point_size = s.point_size_mm;
    u.min_point_px = s.min_point_px;
    u.lighting = s.lighting ? 1.0f : 0.0f;
    u.lasso_count = lasso_count_;

    enc->setDepthStencilState(depth_state_.get());
    enc->setVertexBytes(&u, sizeof(u), 1);

    auto draw_splats = [&](const Layer& layer) {
        if (layer.count == 0) return;
        const std::uint32_t selectable = &layer == &model_ ? 1u : 0u;  // only the model can be selected
        enc->setRenderPipelineState(splat_pso_.get());
        enc->setVertexBuffer(layer.buffer.get(), 0, 0);
        enc->setVertexBuffer(lasso_strokes_.get(), 0, 2);
        enc->setVertexBuffer(lasso_masks_.get(), 0, 3);
        enc->setVertexBytes(&selectable, sizeof(selectable), 4);
        enc->drawPrimitives(MTL::PrimitiveTypeTriangleStrip, NS::UInteger(0), NS::UInteger(4), NS::UInteger(layer.count));
    };
    if (s.show_mesh && mesh_index_count_ > 0) {
        enc->setRenderPipelineState(mesh_pso_.get());
        enc->setVertexBuffer(mesh_vertices_.get(), 0, 0);
        const std::uint32_t colored = mesh_colors_ ? 1u : 0u;
        enc->setVertexBuffer(mesh_colors_ ? mesh_colors_.get() : mesh_vertices_.get(), 0, 2);
        enc->setVertexBytes(&colored, sizeof(colored), 3);
        enc->drawIndexedPrimitives(MTL::PrimitiveTypeTriangle, NS::UInteger(mesh_index_count_), MTL::IndexTypeUInt32,
                                   mesh_indices_.get(), NS::UInteger(0));
    }
    if (s.show_points) {
        draw_splats(model_);
        draw_splats(frame_);
    }
    if (s.show_scanner && scanner_pose_ && scanner_vertex_count_ > 0) {
        enc->setRenderPipelineState(solid_pso_.get());
        enc->setVertexBuffer(scanner_vertices_.get(), 0, 0);
        enc->setVertexBytes(scanner_pose_->data(), sizeof(Mat4f), 2);
        enc->drawPrimitives(MTL::PrimitiveTypeTriangle, NS::UInteger(0), NS::UInteger(scanner_vertex_count_));
    }

    if (markers_.count > 0) {
        enc->setRenderPipelineState(marker_pso_.get());
        enc->setVertexBuffer(markers_.buffer.get(), 0, 0);
        enc->drawPrimitives(MTL::PrimitiveTypeTriangleStrip, NS::UInteger(0), NS::UInteger(4), NS::UInteger(markers_.count));
    }
    if (lines_.count > 0) {
        enc->setRenderPipelineState(line_pso_.get());
        enc->setVertexBuffer(lines_.buffer.get(), 0, 0);
        enc->drawPrimitives(MTL::PrimitiveTypeLine, NS::UInteger(0), NS::UInteger(lines_.count));
    }
}

}  // namespace einstar::render
