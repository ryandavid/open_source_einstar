#include "einstar/depth_metal/depth_packer.hpp"

namespace einstar::depth_metal {
namespace {

constexpr const char* kSource = R"METAL(
#include <metal_stdlib>
using namespace metal;

inline uint quantize(float z) { return z > 0 ? uint(clamp(round(z * 50.0f), 1.0f, 65535.0f)) : 0u; }

kernel void pack_depth(device const float4* pts [[buffer(0)]], device const float* wts [[buffer(1)]],
                       device ushort* depth [[buffer(2)]], device uchar* conf [[buffer(3)]], constant uint2& size [[buffer(4)]],
                       uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= size.x || gid.y >= size.y) return;
    const uint i = gid.y * size.x + gid.x;
    const uint q = quantize(pts[i].z);
    const uint left = gid.x > 0 ? quantize(pts[i - 1].z) : 0u;
    depth[i] = ushort((q - left) & 0xFFFFu);
    conf[i] = uchar(clamp(round(wts[i] * 255.0f), 0.0f, 255.0f));
}
)METAL";

}  // namespace

Result<std::unique_ptr<DepthPacker>> DepthPacker::create(std::shared_ptr<gpu::Context> ctx) {
    auto p = std::unique_ptr<DepthPacker>(new DepthPacker());
    p->ctx_ = std::move(ctx);
    auto lib = p->ctx_->library("depth_packer", kSource);
    if (!lib) return std::unexpected(lib.error());
    auto pso = p->ctx_->compute_pipeline(*lib, "pack_depth");
    if (!pso) return std::unexpected(pso.error());
    p->pso_ = std::move(*pso);
    return p;
}

std::shared_ptr<DepthPacker::Packed> DepthPacker::pack(std::shared_ptr<const gpu::MetalFrameData> frame_ptr) const {
    const auto& frame = *frame_ptr;
    auto out = std::make_shared<Packed>();
    out->frame = std::move(frame_ptr);
    out->width = frame.width();
    out->height = frame.height();
    const auto n = static_cast<std::size_t>(out->width * out->height);
    out->buffer = ctx_->buffer(n * 3);
    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
    out->command = gpu::Ref<MTL::CommandBuffer>(ctx_->queue()->commandBuffer()->retain());
    MTL::ComputeCommandEncoder* enc = out->command->computeCommandEncoder();
    const std::uint32_t size[2] = {static_cast<std::uint32_t>(out->width), static_cast<std::uint32_t>(out->height)};
    enc->setComputePipelineState(pso_.get());
    enc->setBuffer(frame.points_buffer(), 0, 0);
    enc->setBuffer(frame.weights_buffer(), 0, 1);
    enc->setBuffer(out->buffer.get(), 0, 2);
    enc->setBuffer(out->buffer.get(), n * 2, 3);
    enc->setBytes(size, sizeof(size), 4);
    enc->dispatchThreads(MTL::Size(size[0], size[1], 1), MTL::Size(16, 16, 1));
    enc->endEncoding();
    out->command->commit();
    pool->release();
    return out;
}

}  // namespace einstar::depth_metal
