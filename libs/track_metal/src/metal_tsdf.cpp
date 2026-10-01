#include "einstar/track_metal/metal_tsdf.hpp"

#include <array>

#include <algorithm>
#include <cstring>
#include <functional>

#include "einstar/core/log.hpp"
#include "einstar/gpu/profile.hpp"
#include "einstar/gpu/device_data.hpp"

namespace einstar::track_metal {
namespace {

constexpr const char* kSource =
#include "tsdf_kernels.metal.inc"
    ;

struct VolumeArgs {
    float voxel, trunc, max_weight, min_depth, max_depth;
    std::uint32_t table_mask, brick_capacity, frame;
    float weight_scale;
    std::uint32_t extend_only;
    float min_weight;
    std::uint32_t since_frame;
    std::uint32_t count_observations;
};

struct CameraArgs {
    float T_wc[16];
    float T_cw[16];
    float fx, fy, cx, cy;
    std::uint32_t w, h;
    std::uint32_t pad[2];
};
static_assert(sizeof(CameraArgs) % 16 == 0);

struct GpuSurfacePoint {
    float p[3];
    float n[3];
    float w;
};

using gpu::Ref;

CameraArgs camera_args(const SE3& T_wc, double fx, double fy, double cx, double cy, int w, int h) {
    CameraArgs c{};
    const Eigen::Matrix4f a = T_wc.matrix().cast<float>();
    const Eigen::Matrix4f b = T_wc.inverse().matrix().cast<float>();
    std::memcpy(c.T_wc, a.data(), sizeof(c.T_wc));  // Eigen and Metal are both column-major
    std::memcpy(c.T_cw, b.data(), sizeof(c.T_cw));
    c.fx = static_cast<float>(fx);
    c.fy = static_cast<float>(fy);
    c.cx = static_cast<float>(cx);
    c.cy = static_cast<float>(cy);
    c.w = static_cast<std::uint32_t>(w);
    c.h = static_cast<std::uint32_t>(h);
    return c;
}

// CPU mirror of the kernel's key packing/hash, for lookups by coordinate.
bool pack_key(const track::BrickCoord& b, std::uint32_t& key) {
    if (b.x < -1024 || b.x > 1023 || b.y < -1024 || b.y > 1023 || b.z < -511 || b.z > 510) return false;
    key = static_cast<std::uint32_t>(b.x + 1024) | (static_cast<std::uint32_t>(b.y + 1024) << 11) |
          (static_cast<std::uint32_t>(b.z + 512) << 22);
    return true;
}
std::uint32_t hash_key(std::uint32_t key, std::uint32_t mask) {
    std::uint32_t h = key * 2654435761u;
    h ^= h >> 15;
    return h & mask;
}

}  // namespace

struct MetalTsdfVolume::Impl {
    std::shared_ptr<gpu::Context> ctx;
    MetalTsdfOptions opt;
    Ref<MTL::ComputePipelineState> init_voxels, allocate, integrate, raycast, extract, extract_render;
    mutable std::size_t render_capacity = 0;
    Ref<MTL::Buffer> keys, values, coords, brick_count, stamps, visible_count, visible, voxels, last_update, occupancy;
    Ref<MTL::Buffer> observations;  // one byte per voxel when counted, else a placeholder
    // per-frame inputs
    Ref<MTL::Buffer> points, weights;
    std::size_t frame_pixels = 0;
    // extraction
    mutable Ref<MTL::Buffer> ext_out, ext_count, ext_list;
    mutable std::size_t ext_capacity = 0;

    VolumeArgs args(const track::TsdfParams& p, std::uint32_t frame) const {
        VolumeArgs a{};
        a.voxel = p.voxel_mm;
        a.trunc = p.truncation_mm;
        a.max_weight = p.max_weight;
        a.min_depth = p.min_depth_mm;
        a.max_depth = p.max_depth_mm;
        a.table_mask = opt.table_size - 1;
        a.brick_capacity = opt.brick_capacity;
        a.frame = frame;
        a.weight_scale = 1.0f;
        a.count_observations = p.count_observations ? 1u : 0u;
        return a;
    }

    // Runs one command buffer; `read_back` are managed buffers the CPU reads afterwards.
    void run(const char* label, const std::function<void(MTL::ComputeCommandEncoder*)>& encode,
             std::initializer_list<MTL::Buffer*> read_back = {}) const {
        NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
        MTL::CommandBuffer* cmd = ctx->queue()->commandBuffer();
        MTL::ComputeCommandEncoder* enc = cmd->computeCommandEncoder();
        encode(enc);
        enc->endEncoding();
        gpu::Context::sync_for_cpu(cmd, read_back);
        gpu::profile::commit_and_wait(cmd, label);
        if (cmd->status() == MTL::CommandBufferStatusError) log::error("Metal TSDF command buffer failed");
        pool->release();
    }

    [[nodiscard]] std::uint32_t count() const { return std::min(*static_cast<std::uint32_t*>(brick_count->contents()), opt.brick_capacity); }

    // CPU view of the first `bytes` of a GPU buffer: in place on unified memory, else a copy in `scratch`.
    const void* host_view(MTL::Buffer* b, std::size_t bytes, std::vector<std::byte>& scratch) const {
        if (b->storageMode() == MTL::StorageModeShared) return b->contents();
        scratch.resize(bytes);
        ctx->download(b, 0, scratch.data(), bytes);
        return scratch.data();
    }

    int lookup(const track::BrickCoord& b, const std::uint32_t* k, const std::int32_t* v) const {
        std::uint32_t key;
        if (!pack_key(b, key)) return -1;
        std::uint32_t i = hash_key(key, opt.table_size - 1);
        for (int probe = 0; probe < 128; ++probe) {
            if (k[i] == key) return v[i];
            if (k[i] == 0xFFFFFFFFu) return -1;
            i = (i + 1) & (opt.table_size - 1);
        }
        return -1;
    }
};

MetalTsdfVolume::MetalTsdfVolume(std::unique_ptr<Impl> impl, track::TsdfParams params) : impl_(std::move(impl)), params_(params) {}
MetalTsdfVolume::~MetalTsdfVolume() = default;

Result<std::unique_ptr<MetalTsdfVolume>> MetalTsdfVolume::create(std::shared_ptr<gpu::Context> ctx, track::TsdfParams params,
                                                                 MetalTsdfOptions opt) {
    if ((opt.table_size & (opt.table_size - 1)) != 0 || opt.table_size < 2 * opt.brick_capacity)
        return make_error(Errc::invalid_argument, "hash table size must be a power of two >= 2x brick capacity");
    auto im = std::make_unique<Impl>();
    im->ctx = std::move(ctx);
    im->opt = opt;
    auto lib = im->ctx->library("tsdf", kSource);
    if (!lib) return std::unexpected(lib.error());
    for (auto [name, slot] : {std::pair{"init_voxels", &im->init_voxels}, std::pair{"allocate", &im->allocate},
                              std::pair{"integrate", &im->integrate}, std::pair{"raycast", &im->raycast},
                              std::pair{"extract", &im->extract}, std::pair{"extract_render", &im->extract_render}}) {
        auto p = im->ctx->compute_pipeline(*lib, name);
        if (!p) return std::unexpected(p.error());
        *slot = std::move(*p);
    }
    const std::size_t cap = opt.brick_capacity;
    // The volume lives on the GPU (the CPU only reads it back for processing); the counters are
    // written and read by the CPU around each dispatch (see gpu::Context).
    im->keys = im->ctx->gpu_buffer(opt.table_size * 4ull);
    im->values = im->ctx->gpu_buffer(opt.table_size * 4ull);
    im->coords = im->ctx->gpu_buffer(cap * 16);
    im->brick_count = im->ctx->mirrored_buffer(4);
    im->stamps = im->ctx->gpu_buffer(cap * 4);
    im->visible_count = im->ctx->mirrored_buffer(4);
    im->visible = im->ctx->gpu_buffer(cap * 4);
    im->voxels = im->ctx->gpu_buffer(cap * 512 * 4);
    im->observations = im->ctx->gpu_buffer(params.count_observations ? cap * 512 : 16);
    im->last_update = im->ctx->gpu_buffer(cap * 4);
    im->occupancy = im->ctx->gpu_buffer((256ull * 256 * 128) / 8);  // see occupancy_index in the kernels
    im->ext_count = im->ctx->mirrored_buffer(4);
    if (!im->voxels) return make_error(Errc::unsupported, "could not allocate the brick pool");
    auto vol = std::unique_ptr<MetalTsdfVolume>(new MetalTsdfVolume(std::move(im), params));
    vol->clear();
    log::info("Metal TSDF: {} bricks ({} MiB pool), {} hash slots", cap, cap * 512 * 4 / (1024 * 1024), opt.table_size);
    return vol;
}

void MetalTsdfVolume::clear() {
    auto& im = *impl_;
    im.ctx->fill(im.keys.get(), 0xFF);
    im.ctx->fill(im.values.get(), 0xFF);
    im.ctx->fill(im.stamps.get(), 0);
    im.ctx->fill(im.last_update.get(), 0);
    im.ctx->fill(im.occupancy.get(), 0);
    im.ctx->fill(im.observations.get(), 0);
    *static_cast<std::uint32_t*>(im.brick_count->contents()) = 0;
    gpu::Context::cpu_modified(im.brick_count.get());
    im.run("tsdf/init_voxels", [&](MTL::ComputeCommandEncoder* enc) {
        enc->setComputePipelineState(im.init_voxels.get());
        enc->setBuffer(im.voxels.get(), 0, 0);
        enc->dispatchThreads(MTL::Size(im.opt.brick_capacity * 512ull, 1, 1), MTL::Size(256, 1, 1));
    });
    frame_ = 0;
}

std::size_t MetalTsdfVolume::brick_count() const { return impl_->count(); }

void MetalTsdfVolume::for_each_brick(const BrickVisitor& fn) const {
    // The pool: half sdf, half weight per voxel (read in place on unified memory).
    const auto& im = *impl_;
    std::vector<std::byte> coords_copy, vox_copy, obs_copy;
    const auto* coords = static_cast<const std::int32_t*>(im.host_view(im.coords.get(), im.count() * 16ull, coords_copy));
    const auto* vox = static_cast<const _Float16*>(im.host_view(im.voxels.get(), im.count() * 512ull * 4, vox_copy));
    const auto* obs = params_.count_observations
                          ? static_cast<const std::uint8_t*>(im.host_view(im.observations.get(), im.count() * 512ull, obs_copy))
                          : nullptr;
    std::array<float, track::kBrickVoxels> sdf{}, weight{};
    for (std::uint32_t b = 0; b < im.count(); ++b) {
        const track::BrickCoord c{coords[4 * b], coords[4 * b + 1], coords[4 * b + 2]};
        const _Float16* v = vox + static_cast<std::size_t>(b) * track::kBrickVoxels * 2;
        for (int i = 0; i < track::kBrickVoxels; ++i) {
            sdf[static_cast<std::size_t>(i)] = static_cast<float>(v[2 * i]);
            weight[static_cast<std::size_t>(i)] = static_cast<float>(v[2 * i + 1]);
        }
        fn(c, sdf, weight, obs ? std::span<const std::uint8_t>(obs + static_cast<std::size_t>(b) * track::kBrickVoxels, track::kBrickVoxels)
                              : std::span<const std::uint8_t>{});
    }
}

bool MetalTsdfVolume::pool_exhausted() const {
    return *static_cast<std::uint32_t*>(impl_->brick_count->contents()) >= impl_->opt.brick_capacity;
}

void MetalTsdfVolume::integrate(const track::DepthFrame& frame, const SE3& T_world_camera, float weight_scale, bool extend_only) {
    auto& im = *impl_;
    const int w = frame.width(), h = frame.height();
    const auto n = static_cast<std::size_t>(w * h);
    MTL::Buffer* points_buf = nullptr;
    MTL::Buffer* weights_buf = nullptr;
    if (const auto* dev = dynamic_cast<const gpu::MetalFrameData*>(frame.device.get())) {
        // GPU-resident frame: bind its buffers directly.
        points_buf = dev->points_buffer();
        weights_buf = dev->weights_buffer();
    } else {
        if (n != im.frame_pixels) {
            im.points = im.ctx->mirrored_buffer(n * 16);
            im.weights = im.ctx->mirrored_buffer(n * 4);
            im.frame_pixels = n;
        }
        frame.ensure_cpu();
        auto* pts = static_cast<float*>(im.points->contents());
        auto* wts = static_cast<float*>(im.weights->contents());
        for (std::size_t i = 0; i < n; ++i) {
            const Vec3f& p = frame.points.data()[i];
            pts[4 * i] = p.x();
            pts[4 * i + 1] = p.y();
            pts[4 * i + 2] = p.z();
            pts[4 * i + 3] = 1.0f;
            wts[i] = frame.weights.empty() ? 1.0f : frame.weights.data()[i];
        }
        gpu::Context::cpu_modified(im.points.get());
        gpu::Context::cpu_modified(im.weights.get());
        points_buf = im.points.get();
        weights_buf = im.weights.get();
    }
    ++frame_;
    VolumeArgs a = im.args(params_, frame_);
    a.weight_scale = weight_scale;
    a.extend_only = extend_only ? 1u : 0u;
    const auto& k = frame.intrinsics;
    const CameraArgs cam = camera_args(T_world_camera, k.fx, k.fy, k.cx, k.cy, w, h);
    *static_cast<std::uint32_t*>(im.visible_count->contents()) = 0;
    gpu::Context::cpu_modified(im.visible_count.get());
    im.run("tsdf/allocate", [&](MTL::ComputeCommandEncoder* enc) {
        enc->setComputePipelineState(im.allocate.get());
        enc->setBuffer(points_buf, 0, 0);
        enc->setBuffer(im.keys.get(), 0, 1);
        enc->setBuffer(im.values.get(), 0, 2);
        enc->setBuffer(im.coords.get(), 0, 3);
        enc->setBuffer(im.brick_count.get(), 0, 4);
        enc->setBuffer(im.stamps.get(), 0, 5);
        enc->setBuffer(im.visible_count.get(), 0, 6);
        enc->setBuffer(im.visible.get(), 0, 7);
        enc->setBytes(&a, sizeof(a), 8);
        enc->setBytes(&cam, sizeof(cam), 9);
        enc->setBuffer(im.occupancy.get(), 0, 10);
        enc->dispatchThreads(MTL::Size(static_cast<NS::UInteger>(w), static_cast<NS::UInteger>(h), 1), MTL::Size(16, 16, 1));
    }, {im.visible_count.get(), im.brick_count.get()});
    const std::uint32_t visible = std::min(*static_cast<std::uint32_t*>(im.visible_count->contents()), im.opt.brick_capacity);
    if (pool_exhausted()) log::warn("Metal TSDF brick pool exhausted ({} bricks); new surface is not stored", im.opt.brick_capacity);
    if (visible == 0) return;
    im.run("tsdf/integrate", [&](MTL::ComputeCommandEncoder* enc) {
        enc->setComputePipelineState(im.integrate.get());
        enc->setBuffer(points_buf, 0, 0);
        enc->setBuffer(weights_buf, 0, 1);
        enc->setBuffer(im.coords.get(), 0, 2);
        enc->setBuffer(im.visible.get(), 0, 3);
        enc->setBuffer(im.voxels.get(), 0, 4);
        enc->setBuffer(im.last_update.get(), 0, 5);
        enc->setBytes(&a, sizeof(a), 6);
        enc->setBytes(&cam, sizeof(cam), 7);
        enc->setBuffer(im.observations.get(), 0, 8);
        enc->dispatchThreadgroups(MTL::Size(visible, 1, 1), MTL::Size(8, 8, 8));
    });
}

track::RaycastResult MetalTsdfVolume::raycast(const SE3& T_world_camera, const track::Intrinsics& k) const {
    auto& im = *impl_;
    const auto n = static_cast<std::size_t>(k.width * k.height);
    // Fresh output buffers each call: the result may be held (e.g. by ICP) while the next raycast runs.
    auto points = im.ctx->mirrored_buffer(n * 16);
    auto normals = im.ctx->mirrored_buffer(n * 16);
    const VolumeArgs a = im.args(params_, frame_);
    const CameraArgs cam = camera_args(T_world_camera, k.fx, k.fy, k.cx, k.cy, k.width, k.height);
    im.run("tsdf/raycast", [&](MTL::ComputeCommandEncoder* enc) {
        enc->setComputePipelineState(im.raycast.get());
        enc->setBuffer(im.keys.get(), 0, 0);
        enc->setBuffer(im.values.get(), 0, 1);
        enc->setBuffer(im.voxels.get(), 0, 2);
        enc->setBuffer(points.get(), 0, 3);
        enc->setBuffer(normals.get(), 0, 4);
        enc->setBytes(&a, sizeof(a), 5);
        enc->setBytes(&cam, sizeof(cam), 6);
        enc->setBuffer(im.occupancy.get(), 0, 7);
        // 8x8 tiles keep a (64-wide) AMD wavefront's rays together; Apple GPUs keep their 16x16.
        const NS::UInteger tile = im.ctx->apple_gpu() ? 16 : 8;
        enc->dispatchThreads(MTL::Size(static_cast<NS::UInteger>(k.width), static_cast<NS::UInteger>(k.height), 1), MTL::Size(tile, tile, 1));
    });
    track::RaycastResult out;
    out.intrinsics = k;
    out.device = std::make_shared<gpu::MetalRaycastData>(k.width, k.height, std::move(points), std::move(normals), im.ctx);
    return out;
}

std::vector<track::SurfacePoint> MetalTsdfVolume::extract_points(std::uint32_t since_frame, float min_weight, bool canonical) const {
    auto& im = *impl_;
    const std::uint32_t bricks = im.count();
    if (bricks == 0) return {};
    VolumeArgs a = im.args(params_, frame_);
    a.min_weight = min_weight;
    a.since_frame = since_frame;
    // Typical surface density is well under 128 points per brick; grow and retry if exceeded.
    for (int attempt = 0; attempt < 3; ++attempt) {
        const std::size_t cap = std::max<std::size_t>(im.ext_capacity, static_cast<std::size_t>(bricks) * 128);
        if (cap > im.ext_capacity) {
            im.ext_out = im.ctx->gpu_buffer(cap * sizeof(GpuSurfacePoint));
            im.ext_capacity = cap;
        }
        if (!im.ext_list) im.ext_list = im.ctx->gpu_buffer(4);
        *static_cast<std::uint32_t*>(im.ext_count->contents()) = 0;
        gpu::Context::cpu_modified(im.ext_count.get());
        const auto cap32 = static_cast<std::uint32_t>(im.ext_capacity);
        const std::uint32_t use_list = 0;
        im.run("tsdf/extract", [&](MTL::ComputeCommandEncoder* enc) {
            enc->setComputePipelineState(im.extract.get());
            enc->setBuffer(im.keys.get(), 0, 0);
            enc->setBuffer(im.values.get(), 0, 1);
            enc->setBuffer(im.voxels.get(), 0, 2);
            enc->setBuffer(im.coords.get(), 0, 3);
            enc->setBuffer(im.last_update.get(), 0, 4);
            enc->setBuffer(im.ext_out.get(), 0, 5);
            enc->setBuffer(im.ext_count.get(), 0, 6);
            enc->setBytes(&a, sizeof(a), 7);
            enc->setBytes(&cap32, sizeof(cap32), 8);
            enc->setBuffer(im.ext_list.get(), 0, 9);
            enc->setBytes(&use_list, sizeof(use_list), 10);
            enc->dispatchThreadgroups(MTL::Size(bricks, 1, 1), MTL::Size(8, 8, 8));
        }, {im.ext_count.get()});
        const std::uint32_t found = *static_cast<std::uint32_t*>(im.ext_count->contents());
        if (found > im.ext_capacity) {
            im.ext_capacity = found + found / 4;
            im.ext_out = im.ctx->gpu_buffer(im.ext_capacity * sizeof(GpuSurfacePoint));
            continue;
        }
        std::vector<track::SurfacePoint> out(found);
        std::vector<std::byte> copy;
        const auto* src = static_cast<const GpuSurfacePoint*>(im.host_view(im.ext_out.get(), found * sizeof(GpuSurfacePoint), copy));
        for (std::uint32_t i = 0; i < found; ++i)
            out[i] = {Vec3f(src[i].p[0], src[i].p[1], src[i].p[2]), Vec3f(src[i].n[0], src[i].n[1], src[i].n[2]), src[i].w};
        if (canonical) track::sort_canonical(out);  // atomic appends: the order varies between runs
        return out;
    }
    return {};
}

MetalTsdfVolume::RenderPoints MetalTsdfVolume::extract_render_points(float min_weight) const {
    auto& im = *impl_;
    RenderPoints out;
    const std::uint32_t bricks = im.count();
    if (bricks == 0) return out;
    VolumeArgs a = im.args(params_, frame_);
    a.min_weight = min_weight;
    constexpr std::size_t kVertex = 28;  // render::PointVertex
    for (int attempt = 0; attempt < 2; ++attempt) {
        const std::size_t cap = std::max<std::size_t>(im.render_capacity, static_cast<std::size_t>(bricks) * 96);
        out.buffer = im.ctx->gpu_buffer(cap * kVertex);  // drawn by the renderer only
        *static_cast<std::uint32_t*>(im.ext_count->contents()) = 0;
        gpu::Context::cpu_modified(im.ext_count.get());
        const auto cap32 = static_cast<std::uint32_t>(cap);
        im.run("tsdf/extract_render", [&](MTL::ComputeCommandEncoder* enc) {
            enc->setComputePipelineState(im.extract_render.get());
            enc->setBuffer(im.keys.get(), 0, 0);
            enc->setBuffer(im.values.get(), 0, 1);
            enc->setBuffer(im.voxels.get(), 0, 2);
            enc->setBuffer(im.coords.get(), 0, 3);
            enc->setBuffer(out.buffer.get(), 0, 4);
            enc->setBuffer(im.ext_count.get(), 0, 5);
            enc->setBytes(&a, sizeof(a), 6);
            enc->setBytes(&cap32, sizeof(cap32), 7);
            enc->dispatchThreadgroups(MTL::Size(bricks, 1, 1), MTL::Size(8, 8, 8));
        }, {im.ext_count.get()});
        const std::uint32_t found = *static_cast<std::uint32_t*>(im.ext_count->contents());
        if (found <= cap) {
            out.count = found;
            im.render_capacity = std::max(im.render_capacity, static_cast<std::size_t>(found) + found / 8);
            return out;
        }
        im.render_capacity = found + found / 4;  // grow and retry once
    }
    return out;
}

std::vector<track::BrickCoord> MetalTsdfVolume::bricks_updated_since(std::uint32_t frame) const {
    auto& im = *impl_;
    const std::uint32_t n = im.count();
    std::vector<std::byte> lu_copy, c_copy;
    const auto* lu = static_cast<const std::uint32_t*>(im.host_view(im.last_update.get(), n * 4ull, lu_copy));
    const auto* c = static_cast<const std::int32_t*>(im.host_view(im.coords.get(), n * 16ull, c_copy));
    std::vector<track::BrickCoord> out;
    for (std::uint32_t i = 0; i < n; ++i)
        if (lu[i] > frame) out.push_back({c[4 * i], c[4 * i + 1], c[4 * i + 2]});
    return out;
}

std::vector<track::SurfacePoint> MetalTsdfVolume::extract_points(const std::vector<track::BrickCoord>& bricks, float min_weight) const {
    auto& im = *impl_;
    std::vector<std::uint32_t> idx;
    idx.reserve(bricks.size());
    std::vector<std::byte> keys_copy, values_copy;
    const auto table_bytes = static_cast<std::size_t>(im.opt.table_size) * 4;
    const auto* keys = static_cast<const std::uint32_t*>(im.host_view(im.keys.get(), table_bytes, keys_copy));
    const auto* values = static_cast<const std::int32_t*>(im.host_view(im.values.get(), table_bytes, values_copy));
    for (const auto& b : bricks)
        if (const int i = im.lookup(b, keys, values); i >= 0) idx.push_back(static_cast<std::uint32_t>(i));
    if (idx.empty()) return {};
    im.ext_list = im.ctx->mirrored_buffer(idx.size() * 4);
    std::memcpy(im.ext_list->contents(), idx.data(), idx.size() * 4);
    gpu::Context::cpu_modified(im.ext_list.get());
    const std::size_t cap = idx.size() * 512;
    if (cap > im.ext_capacity) {
        im.ext_out = im.ctx->gpu_buffer(cap * sizeof(GpuSurfacePoint));
        im.ext_capacity = cap;
    }
    VolumeArgs a = im.args(params_, frame_);
    a.min_weight = min_weight;
    a.since_frame = 0;
    *static_cast<std::uint32_t*>(im.ext_count->contents()) = 0;
    gpu::Context::cpu_modified(im.ext_count.get());
    const auto cap32 = static_cast<std::uint32_t>(im.ext_capacity);
    const std::uint32_t use_list = 1;
    im.run("tsdf/extract", [&](MTL::ComputeCommandEncoder* enc) {
        enc->setComputePipelineState(im.extract.get());
        enc->setBuffer(im.keys.get(), 0, 0);
        enc->setBuffer(im.values.get(), 0, 1);
        enc->setBuffer(im.voxels.get(), 0, 2);
        enc->setBuffer(im.coords.get(), 0, 3);
        enc->setBuffer(im.last_update.get(), 0, 4);
        enc->setBuffer(im.ext_out.get(), 0, 5);
        enc->setBuffer(im.ext_count.get(), 0, 6);
        enc->setBytes(&a, sizeof(a), 7);
        enc->setBytes(&cap32, sizeof(cap32), 8);
        enc->setBuffer(im.ext_list.get(), 0, 9);
        enc->setBytes(&use_list, sizeof(use_list), 10);
        enc->dispatchThreadgroups(MTL::Size(idx.size(), 1, 1), MTL::Size(8, 8, 8));
    }, {im.ext_count.get()});
    const std::uint32_t found = std::min<std::uint32_t>(*static_cast<std::uint32_t*>(im.ext_count->contents()), cap32);
    std::vector<track::SurfacePoint> out(found);
    std::vector<std::byte> copy;
    const auto* src = static_cast<const GpuSurfacePoint*>(im.host_view(im.ext_out.get(), found * sizeof(GpuSurfacePoint), copy));
    for (std::uint32_t i = 0; i < found; ++i)
        out[i] = {Vec3f(src[i].p[0], src[i].p[1], src[i].p[2]), Vec3f(src[i].n[0], src[i].n[1], src[i].n[2]), src[i].w};
    track::sort_canonical(out);
    return out;
}

}  // namespace einstar::track_metal
