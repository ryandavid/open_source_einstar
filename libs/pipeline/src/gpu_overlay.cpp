#include "gpu_overlay.hpp"

#include "einstar/gpu/profile.hpp"

#include <cstring>

namespace einstar::pipeline {
namespace {

constexpr const char* kSource = R"METAL(
#include <metal_stdlib>
using namespace metal;
struct Args { float4x4 T; uint w; uint h; uint step; uint capacity; uchar4 color; };
struct Vertex { packed_float3 p; packed_float3 n; uchar4 color; };
kernel void frame_overlay(device const float4* pts [[buffer(0)]], device const float4* nrm [[buffer(1)]],
                          device Vertex* out [[buffer(2)]], device atomic_uint* count [[buffer(3)]],
                          constant Args& a [[buffer(4)]], uint2 gid [[thread_position_in_grid]]) {
    const uint x = gid.x * a.step, y = gid.y * a.step;
    if (x >= a.w || y >= a.h) return;
    const uint i = y * a.w + x;
    const float4 p = pts[i];
    if (p.z <= 0) return;
    const uint slot = atomic_fetch_add_explicit(count, 1u, memory_order_relaxed);
    if (slot >= a.capacity) return;
    Vertex v;
    v.p = packed_float3((a.T * float4(p.xyz, 1)).xyz);
    v.n = packed_float3((a.T * float4(nrm[i].xyz, 0)).xyz);
    v.color = a.color;
    out[slot] = v;
}
)METAL";

struct Args {
    float T[16];
    std::uint32_t w, h, step, capacity;
    render::Rgba8 color;
    std::uint32_t pad[3];
};

}  // namespace

std::unique_ptr<GpuOverlay> GpuOverlay::create(std::shared_ptr<gpu::Context> ctx) {
    auto lib = ctx->library("overlay", kSource);
    if (!lib) return nullptr;
    auto pso = ctx->compute_pipeline(*lib, "frame_overlay");
    if (!pso) return nullptr;
    auto o = std::make_unique<GpuOverlay>();
    o->ctx_ = std::move(ctx);
    o->pso_ = std::move(*pso);
    o->counter_ = o->ctx_->mirrored_buffer(4);
    return o;
}

GpuOverlay::Points GpuOverlay::frame_points(const gpu::MetalFrameData& f, const SE3& T_wc, int step, render::Rgba8 color) {
    Points out;
    const int gw = (f.width() + step - 1) / step, gh = (f.height() + step - 1) / step;
    const auto cap = static_cast<std::size_t>(gw * gh);
    out.buffer = ctx_->gpu_buffer(cap * sizeof(render::PointVertex));  // drawn by the renderer only
    Args a{};
    const Eigen::Matrix4f T = T_wc.matrix().cast<float>();
    std::memcpy(a.T, T.data(), sizeof(a.T));
    a.w = static_cast<std::uint32_t>(f.width());
    a.h = static_cast<std::uint32_t>(f.height());
    a.step = static_cast<std::uint32_t>(step);
    a.capacity = static_cast<std::uint32_t>(cap);
    a.color = color;
    *static_cast<std::uint32_t*>(counter_->contents()) = 0;
    gpu::Context::cpu_modified(counter_.get());
    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
    MTL::CommandBuffer* cmd = ctx_->queue()->commandBuffer();
    MTL::ComputeCommandEncoder* enc = cmd->computeCommandEncoder();
    enc->setComputePipelineState(pso_.get());
    enc->setBuffer(f.points_buffer(), 0, 0);
    enc->setBuffer(f.normals_buffer(), 0, 1);
    enc->setBuffer(out.buffer.get(), 0, 2);
    enc->setBuffer(counter_.get(), 0, 3);
    enc->setBytes(&a, sizeof(a), 4);
    enc->dispatchThreads(MTL::Size(static_cast<NS::UInteger>(gw), static_cast<NS::UInteger>(gh), 1), MTL::Size(16, 16, 1));
    enc->endEncoding();
    gpu::Context::sync_for_cpu(cmd, {counter_.get()});
    gpu::profile::commit_and_wait(cmd, "overlay/frame points");
    pool->release();
    out.count = std::min<std::size_t>(*static_cast<std::uint32_t*>(counter_->contents()), cap);
    return out;
}

}  // namespace einstar::pipeline
