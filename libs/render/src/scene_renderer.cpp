#include "einstar/render/scene_renderer.hpp"

#include <algorithm>
#include <cstring>
#include <format>

namespace einstar::render {
namespace {

constexpr const char* kShaderSource =
#include "shaders.metal.inc"
    ;

struct Uniforms {
    Eigen::Matrix4f view;  // column-major, matches float4x4
    Eigen::Matrix4f proj;
    float viewport[2];
    float point_size;
    float min_point_px;
    float lighting;
    float pad[3];
};

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
    if (needed > layer.capacity || layer.capacity == 0) {
        const std::size_t cap = std::max<std::size_t>(needed + needed / 2, 1024);
        auto grown = ctx_->buffer(cap * stride);
        if (append && layer.buffer && layer.count > 0)
            std::memcpy(grown->contents(), layer.buffer->contents(), layer.count * stride);
        layer.buffer = std::move(grown);
        layer.capacity = cap;
    }
    if (count > 0) std::memcpy(static_cast<std::byte*>(layer.buffer->contents()) + start * stride, data, count * stride);
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

    enc->setDepthStencilState(depth_state_.get());
    enc->setVertexBytes(&u, sizeof(u), 1);

    auto draw_splats = [&](const Layer& layer) {
        if (layer.count == 0) return;
        enc->setRenderPipelineState(splat_pso_.get());
        enc->setVertexBuffer(layer.buffer.get(), 0, 0);
        enc->drawPrimitives(MTL::PrimitiveTypeTriangleStrip, NS::UInteger(0), NS::UInteger(4), NS::UInteger(layer.count));
    };
    draw_splats(model_);
    draw_splats(frame_);

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
