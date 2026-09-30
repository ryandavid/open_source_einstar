#include "einstar/depth_metal/metal_stereo.hpp"

#include <algorithm>
#include <cstring>
#include <optional>
#include <mutex>
#include <format>
#include <vector>

#include "einstar/core/timing.hpp"
#include "einstar/gpu/profile.hpp"

namespace einstar::depth_metal {
namespace {

constexpr const char* kSource =
#include "stereo_kernels.metal.inc"
    ;

struct Size2 { std::uint32_t w, h; };
struct RectifyArgs { std::uint32_t src_w, src_h, full_w, full_h, out_w, out_h; };
struct CensusArgs { std::uint32_t w, h; std::int32_t rx, ry; };
struct CostArgs { std::uint32_t w, h, nd; std::int32_t min_disp; std::uint16_t invalid_cost; std::uint16_t pad; };
struct PathArgs { std::uint32_t w, h, nd; std::int32_t dx, dy, p1, p2; std::uint32_t adaptive, slant, first; };
struct WtaArgs { std::uint32_t w, h, nd; std::int32_t min_disp; float uniqueness; std::uint32_t subpixel; };
struct LrArgs { std::uint32_t w, h; float max_diff; };
struct RefineArgs { std::uint32_t w, h, cw, ch; std::int32_t radius, search_radius; float min_zncc; std::uint32_t subpixel; };
struct CclArgs { std::uint32_t w, h; float max_diff; std::uint32_t min_size; };
struct PointsArgs { std::uint32_t w, h; float f, cx, cy, baseline, min_depth, max_depth, max_jump; };
struct BlobArgs {
    std::uint32_t w, h, threshold, min_diameter, max_diameter;
    float max_aspect, min_fill;
    std::uint32_t border;
    float max_axis_ratio, ring_scale, ring_contrast, ring_max_bright;
    std::uint32_t max_blobs;
};

using gpu::Ref;

}  // namespace

struct MetalStereo::Impl {
    std::shared_ptr<gpu::Context> ctx;
    depth::StereoParams p;
    int w = 0, h = 0;  // finest level
    std::vector<int> lw, lh;  // per level (0 = finest)
    int nd = 0;

    Ref<MTL::ComputePipelineState> rectify, down, census, cost, path, wta_l, wta_r, wta_rows, lr, median, refine;
    bool use_wta_rows = false;  // fused row WTA (non-Apple GPUs; see the kernel)
    Ref<MTL::ComputePipelineState> path_simd;  // barrier-free SGM paths (non-Apple GPUs; see the kernel)
    NS::UInteger path_simd_width = 0;
    Ref<MTL::ComputePipelineState> ccl_init, ccl_merge, ccl_count, ccl_filter, pts_kernel, nrm_kernel;
    Ref<MTL::ComputePipelineState> blob_init, blob_merge, blob_stats, blob_select;
    // Marker blob search (per side): labels, per-root count/box/peak, compacted candidates.
    struct BlobBuffers { Ref<MTL::Buffer> L, count, x0, y0, x1, y1, peak, moments, out, n_out; };
    std::array<BlobBuffers, 2> blob_bufs;
    std::optional<BlobParams> blob_params;
    // Raw inputs bound for this frame (the persistent upload buffers, or the caller's images wrapped in place).
    MTL::Buffer* bound_raw_l = nullptr;
    MTL::Buffer* bound_raw_r = nullptr;
    // Caller's images to copy into raw_l / raw_r at the start of the frame (discrete GPU).
    MTL::Buffer* stage_raw_l = nullptr;
    MTL::Buffer* stage_raw_r = nullptr;
    Ref<MTL::Buffer> labels, sizes;
    PointsArgs points_args{};
    bool have_points = false;
    // Output frame pool (frames are handed out as shared_ptr and come back when released).
    struct FrameBuffers { Ref<MTL::Buffer> points, normals, weights; };
    std::mutex pool_mutex;
    std::vector<FrameBuffers> pool;
    FrameBuffers current;
    std::vector<Ref<MTL::Buffer>> img_l, img_r;  // pyramid images per level
    Ref<MTL::Buffer> census_l, census_r, cost_vol, sum_vol, disp_coarse, rdisp;
    std::vector<Ref<MTL::Buffer>> disp, conf, med;  // per level
    // rectification
    Ref<MTL::Buffer> map_l, map_r, raw_l, raw_r;
    int raw_w = 0, raw_h = 0, full_w = 0, full_h = 0;

    // GPU-only working memory, and memory the CPU also reads or writes (see gpu::Context).
    Ref<MTL::Buffer> buf(std::size_t bytes) { return ctx->gpu_buffer(bytes); }
    Ref<MTL::Buffer> cpu_buf(std::size_t bytes) { return ctx->mirrored_buffer(bytes); }
    FrameBuffers new_frame_buffers() const {
        const auto n = static_cast<std::size_t>(w * h);
        return {ctx->mirrored_buffer(n * 16), ctx->mirrored_buffer(n * 16), ctx->mirrored_buffer(n * 4)};
    }
    // Copies rows into a GPU buffer (tightly packed).
    void upload_image(ImageView<const std::uint8_t> src, MTL::Buffer* dst) const {
        if (dst->storageMode() != MTL::StorageModePrivate) {
            auto* d = static_cast<std::uint8_t*>(dst->contents());
            for (int y = 0; y < src.height; ++y) std::memcpy(d + y * src.width, src.row(y), static_cast<std::size_t>(src.width));
            gpu::Context::cpu_modified(dst, 0, static_cast<std::size_t>(src.width * src.height));
            return;
        }
        std::vector<std::uint8_t> packed(static_cast<std::size_t>(src.width * src.height));
        for (int y = 0; y < src.height; ++y) std::memcpy(packed.data() + y * src.width, src.row(y), static_cast<std::size_t>(src.width));
        ctx->upload(dst, 0, packed.data(), packed.size());
    }
};

MetalStereo::MetalStereo(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
MetalStereo::~MetalStereo() = default;

Result<std::unique_ptr<MetalStereo>> MetalStereo::create(std::shared_ptr<gpu::Context> ctx, const depth::StereoParams& params,
                                                          int width, int height) {
    auto im = std::make_unique<Impl>();
    im->ctx = std::move(ctx);
    im->p = params;
    im->w = width;
    im->h = height;
    if (params.sgm.num_disparities > 512) return make_error(Errc::invalid_argument, "Metal SGM supports up to 512 disparities");
    if (params.sgm.census_radius_x * 2 + 1 > 9 || params.sgm.census_radius_y * 2 + 1 > 7)
        return make_error(Errc::invalid_argument, "census window too large for 64 bits");
    // Byte-sized cost volume and path values: census cost + P2 must stay below 256.
    if ((2 * params.sgm.census_radius_x + 1) * (2 * params.sgm.census_radius_y + 1) + params.sgm.p2 > 255)
        return make_error(Errc::invalid_argument, "census window + P2 exceed the byte-sized SGM costs");
    auto lib = im->ctx->library("stereo", kSource);
    if (!lib) return std::unexpected(lib.error());
    auto make = [&](const char* name, Ref<MTL::ComputePipelineState>& out) -> Result<void> {
        auto p = im->ctx->compute_pipeline(*lib, name);
        if (!p) return std::unexpected(p.error());
        out = std::move(*p);
        return {};
    };
    for (auto [n, slot] : {std::pair{"rectify_half", &im->rectify}, std::pair{"downsample2", &im->down},
                           std::pair{"census", &im->census}, std::pair{"cost_volume", &im->cost},
                           std::pair{"sgm_path", &im->path}, std::pair{"wta_left", &im->wta_l},
                           std::pair{"wta_right", &im->wta_r}, std::pair{"wta_rows", &im->wta_rows}, std::pair{"lr_check", &im->lr},
                           std::pair{"median3", &im->median},
                           std::pair{"ccl_init", &im->ccl_init}, std::pair{"ccl_merge", &im->ccl_merge},
                           std::pair{"ccl_count", &im->ccl_count}, std::pair{"ccl_filter", &im->ccl_filter},
                           std::pair{"disparity_points", &im->pts_kernel}, std::pair{"point_normals", &im->nrm_kernel},
                           std::pair{"blob_init", &im->blob_init}, std::pair{"blob_merge", &im->blob_merge},
                           std::pair{"blob_stats", &im->blob_stats}, std::pair{"blob_select", &im->blob_select}})
        if (auto r = make(n, *slot); !r) return std::unexpected(r.error());
    {
        const auto candidates = static_cast<std::uint32_t>(4 * params.refine.search_radius + 1);  // 0.5 px steps
        if (params.refine.search_radius < 1 || candidates > 17)
            return make_error(Errc::invalid_argument, "Metal stereo: refine search radius must be 1..4");
        const std::pair<int, std::uint32_t> constants[] = {{0, candidates}};
        auto p = im->ctx->compute_pipeline(*lib, "refine_slanted", constants);
        if (!p) return std::unexpected(p.error());
        im->refine = std::move(*p);
    }

    const int levels = params.pyramid_levels;
    im->lw.push_back(width);
    im->lh.push_back(height);
    for (int i = 0; i < levels; ++i) {
        im->lw.push_back(im->lw.back() / 2);
        im->lh.push_back(im->lh.back() / 2);
    }
    for (int i = 0; i <= levels; ++i) {
        const auto px = static_cast<std::size_t>(im->lw[static_cast<std::size_t>(i)] * im->lh[static_cast<std::size_t>(i)]);
        im->img_l.push_back(im->buf(px));
        im->img_r.push_back(im->buf(px));
        im->disp.push_back(im->buf(px * 4));
        im->conf.push_back(im->buf(px * 4));
        im->med.push_back(im->buf(px * 4));
    }
    const int cw = im->lw.back(), ch = im->lh.back();
    im->nd = params.sgm.num_disparities;
    im->use_wta_rows = !im->ctx->apple_gpu() && cw <= 1024;
    if (!im->ctx->apple_gpu()) {
        // One SIMD group per scanline: pick the disparities per lane for this GPU's SIMD width.
        for (const int dpl : {2, 4, 8}) {
            auto p = im->ctx->compute_pipeline(*lib, std::format("sgm_path_simd{}", dpl));
            if (!p) return std::unexpected(p.error());
            const NS::UInteger simd_width = (*p)->threadExecutionWidth();
            if ((simd_width != 32 && simd_width != 64) || static_cast<NS::UInteger>(im->nd) > simd_width * static_cast<NS::UInteger>(dpl)) continue;
            im->path_simd = std::move(*p);
            im->path_simd_width = simd_width;
            break;
        }
    }
    const auto cpx = static_cast<std::size_t>(cw * ch);
    im->census_l = im->buf(cpx * 8);
    im->census_r = im->buf(cpx * 8);
    im->cost_vol = im->buf(cpx * static_cast<std::size_t>(im->nd));  // bytes (see cost_volume)
    im->sum_vol = im->buf(cpx * static_cast<std::size_t>(im->nd) * 2);
    im->rdisp = im->buf(cpx * 4);
    const auto fpx = static_cast<std::size_t>(width * height);
    im->labels = im->buf(fpx * 4);
    im->sizes = im->buf(fpx * 4);
    return std::unique_ptr<MetalStereo>(new MetalStereo(std::move(im)));
}

Result<void> MetalStereo::set_rectification(const calib::RemapTable& left, const calib::RemapTable& right, int raw_w, int raw_h) {
    auto& im = *impl_;
    if (left.width / 2 != im.w || left.height / 2 != im.h)
        return make_error(Errc::invalid_argument, "remap resolution must be twice the stereo resolution");
    im.full_w = left.width;
    im.full_h = left.height;
    im.raw_w = raw_w;
    im.raw_h = raw_h;
    const auto n = static_cast<std::size_t>(left.width * left.height);
    im.map_l = im.buf(n * 8);
    im.map_r = im.buf(n * 8);
    auto pack = [&](const calib::RemapTable& t, MTL::Buffer* b) {
        std::vector<float> d(2 * n);
        for (std::size_t i = 0; i < n; ++i) {
            d[2 * i] = t.map_x.data()[i];
            d[2 * i + 1] = t.map_y.data()[i];
        }
        im.ctx->upload(b, 0, d.data(), d.size() * sizeof(float));
    };
    pack(left, im.map_l.get());
    pack(right, im.map_r.get());
    // Written by the CPU each frame (unless the caller's images are bound in place).
    im.raw_l = im.cpu_buf(static_cast<std::size_t>(raw_w * raw_h));
    im.raw_r = im.cpu_buf(static_cast<std::size_t>(raw_w * raw_h));
    if (im.blob_params) set_blob_params(*im.blob_params);
    return {};
}

Result<depth::StereoResult> MetalStereo::compute(ImageView<const std::uint8_t> left, ImageView<const std::uint8_t> right) {
    auto& im = *impl_;
    if (left.width != im.w || left.height != im.h || right.width != im.w || right.height != im.h)
        return make_error(Errc::invalid_argument, "image size does not match the configured stereo size");
    im.upload_image(left, im.img_l[0].get());
    im.upload_image(right, im.img_r[0].get());
    if (auto r = encode_and_run(false, false, nullptr, nullptr); !r) return std::unexpected(r.error());
    return read_result();
}

void MetalStereo::set_point_params(const depth::RectifiedGeometry& g, float min_depth, float max_depth, float max_jump) {
    auto& im = *impl_;
    im.points_args = {static_cast<std::uint32_t>(im.w), static_cast<std::uint32_t>(im.h), static_cast<float>(g.f),
                      static_cast<float>(g.cx), static_cast<float>(g.cy), static_cast<float>(g.baseline), min_depth, max_depth, max_jump};
    im.have_points = true;
}

Result<std::shared_ptr<gpu::MetalFrameData>> MetalStereo::compute_frame_raw(ImageView<const std::uint8_t> raw_left,
                                                                            ImageView<const std::uint8_t> raw_right,
                                                                            ImageU8* rect_left, ImageU8* rect_right) {
    auto& im = *impl_;
    if (!im.have_points) return make_error(Errc::invalid_argument, "set_point_params() not called");
    if (!im.map_l) return make_error(Errc::invalid_argument, "set_rectification() not called");
    if (raw_left.width != im.raw_w || raw_left.height != im.raw_h) return make_error(Errc::invalid_argument, "raw size mismatch");
    {
        std::lock_guard lock(im.pool_mutex);
        if (!im.pool.empty()) {
            im.current = std::move(im.pool.back());
            im.pool.pop_back();
        } else {
            im.current = im.new_frame_buffers();
        }
    }
    im.upload_image(raw_left, im.raw_l.get());
    im.upload_image(raw_right, im.raw_r.get());
    if (auto r = encode_and_run(true, true, rect_left, rect_right); !r) return std::unexpected(r.error());
    auto bufs = std::move(im.current);
    Impl* owner = impl_.get();
    auto keep = std::make_shared<Impl::FrameBuffers>(bufs);
    return std::make_shared<gpu::MetalFrameData>(im.w, im.h, bufs.points, bufs.normals, bufs.weights, [owner, keep] {
        std::lock_guard lock(owner->pool_mutex);
        if (owner->pool.size() < 4) owner->pool.push_back(*keep);
    }, im.ctx);
}

void MetalStereo::set_blob_params(const BlobParams& params) {
    auto& im = *impl_;
    im.blob_params = params;
    if (im.raw_w == 0) return;  // buffers are sized when the rectification (raw size) is known
    const auto n = static_cast<std::size_t>(im.raw_w * im.raw_h);
    for (auto& b : im.blob_bufs) {
        if (b.L && b.L->length() >= n * 4 && b.out->length() >= params.max_blobs * sizeof(BlobBox)) continue;
        b = {im.buf(n * 4), im.buf(n * 4), im.buf(n * 4), im.buf(n * 4), im.buf(n * 4), im.buf(n * 4), im.buf(n * 4), im.buf(n * 20),
             im.cpu_buf(params.max_blobs * sizeof(BlobBox)), im.cpu_buf(4)};
    }
}

Result<FrameOutputs> MetalStereo::compute_frame(const ImageU8& raw_left, const ImageU8& raw_right, const FrameRequest& request) {
    auto& im = *impl_;
    if (!im.have_points) return make_error(Errc::invalid_argument, "set_point_params() not called");
    if (!im.map_l) return make_error(Errc::invalid_argument, "set_rectification() not called");
    if (raw_left.width() != im.raw_w || raw_left.height() != im.raw_h || raw_right.width() != im.raw_w || raw_right.height() != im.raw_h)
        return make_error(Errc::invalid_argument, "raw size mismatch");
    if (request.marker_blobs && !im.blob_params) return make_error(Errc::invalid_argument, "set_blob_params() not called");
    if (request.marker_blobs) set_blob_params(*im.blob_params);  // (sizes buffers on first use)
    {
        std::lock_guard lock(im.pool_mutex);
        if (!im.pool.empty()) {
            im.current = std::move(im.pool.back());
            im.pool.pop_back();
        } else {
            im.current = im.new_frame_buffers();
        }
    }
    // einstar::Image storage is page aligned and padded to whole pages: wrap it, do not copy.
    auto wrap = [&](const ImageU8& img) -> Ref<MTL::Buffer> {
        const auto addr = reinterpret_cast<std::uintptr_t>(img.data());
        if (addr % kImagePageBytes != 0) return {};
        const std::size_t len = (img.size() + kImagePageBytes - 1) / kImagePageBytes * kImagePageBytes;
        return Ref<MTL::Buffer>(im.ctx->device()->newBuffer(img.data(), len, MTL::ResourceStorageModeShared, nullptr));
    };
    Ref<MTL::Buffer> wl = wrap(raw_left), wr = wrap(raw_right);
    if (!wl) im.upload_image(raw_left.view(), im.raw_l.get());
    if (!wr) im.upload_image(raw_right.view(), im.raw_r.get());
    // Unified memory: the kernels read the caller's images in place. Discrete GPU: one blit into VRAM
    // at the start of the frame (the kernels read the raw images many times).
    im.stage_raw_l = !im.ctx->unified_memory() && wl ? wl.get() : nullptr;
    im.stage_raw_r = !im.ctx->unified_memory() && wr ? wr.get() : nullptr;
    im.bound_raw_l = wl && !im.stage_raw_l ? wl.get() : im.raw_l.get();
    im.bound_raw_r = wr && !im.stage_raw_r ? wr.get() : im.raw_r.get();
    FrameOutputs out;
    const auto r = encode_and_run(true, true, request.preview_images ? &out.rect_left : nullptr, request.preview_images ? &out.rect_right : nullptr,
                                  &request, &out);
    im.bound_raw_l = im.bound_raw_r = im.stage_raw_l = im.stage_raw_r = nullptr;
    if (!r) return std::unexpected(r.error());
    auto bufs = std::move(im.current);
    Impl* owner = impl_.get();
    auto keep = std::make_shared<Impl::FrameBuffers>(bufs);
    out.frame = std::make_shared<gpu::MetalFrameData>(im.w, im.h, bufs.points, bufs.normals, bufs.weights, [owner, keep] {
        std::lock_guard lock(owner->pool_mutex);
        if (owner->pool.size() < 4) owner->pool.push_back(*keep);
    }, im.ctx);
    if (request.cpu_frame_access) out.frame->mark_cpu_current();
    return out;
}

Result<depth::StereoResult> MetalStereo::compute_raw(ImageView<const std::uint8_t> raw_left, ImageView<const std::uint8_t> raw_right,
                                                     ImageU8* rect_left, ImageU8* rect_right) {
    auto& im = *impl_;
    if (!im.map_l) return make_error(Errc::invalid_argument, "set_rectification() not called");
    if (raw_left.width != im.raw_w || raw_left.height != im.raw_h) return make_error(Errc::invalid_argument, "raw size mismatch");
    im.upload_image(raw_left, im.raw_l.get());
    im.upload_image(raw_right, im.raw_r.get());
    if (auto r = encode_and_run(true, false, rect_left, rect_right); !r) return std::unexpected(r.error());
    return read_result();
}

depth::StereoResult MetalStereo::read_result() const {
    const auto& im = *impl_;
    depth::StereoResult res;
    res.disparity = ImageF32(im.w, im.h);
    res.confidence = ImageF32(im.w, im.h);
    const auto n = static_cast<std::size_t>(im.w * im.h);
    im.ctx->download(im.disp[0].get(), 0, res.disparity.data(), n * 4);
    if (im.p.pyramid_levels == 0) {
        for (std::size_t i = 0; i < n; ++i) res.confidence.data()[i] = res.disparity.data()[i] < 0 ? 0.0f : 1.0f;
    } else {
        im.ctx->download(im.conf[0].get(), 0, res.confidence.data(), n * 4);
    }
    return res;
}

Result<void> MetalStereo::encode_and_run(bool from_raw, bool make_points, ImageU8* rect_left, ImageU8* rect_right,
                                         const FrameRequest* request, FrameOutputs* outputs) {
    auto& im = *impl_;
    Stopwatch total;
    const auto& sp = im.p.sgm;
    const int levels = im.p.pyramid_levels;
    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
    MTL::CommandBuffer* cmd = im.ctx->queue()->commandBuffer();
    if (im.stage_raw_l || im.stage_raw_r) {
        MTL::BlitCommandEncoder* be = cmd->blitCommandEncoder();
        const auto bytes = static_cast<NS::UInteger>(im.raw_w * im.raw_h);
        if (im.stage_raw_l) be->copyFromBuffer(im.stage_raw_l, 0, im.raw_l.get(), 0, bytes);
        if (im.stage_raw_r) be->copyFromBuffer(im.stage_raw_r, 0, im.raw_r.get(), 0, bytes);
        be->endEncoding();
    }
    MTL::ComputeCommandEncoder* enc = cmd->computeCommandEncoder();

    auto dispatch2d = [&](MTL::ComputePipelineState* pso, int w, int h) {
        enc->setComputePipelineState(pso);
        const NS::UInteger tw = 16, th = 16;
        enc->dispatchThreads(MTL::Size(static_cast<NS::UInteger>(w), static_cast<NS::UInteger>(h), 1), MTL::Size(tw, th, 1));
    };

    const bool blobs = from_raw && request && request->marker_blobs && im.blob_params;
    if (blobs) {
        // Marker blob search on both raw images (independent of the stereo below).
        const auto& bp = *im.blob_params;
        const BlobArgs ba{static_cast<std::uint32_t>(im.raw_w), static_cast<std::uint32_t>(im.raw_h), bp.threshold, bp.min_diameter,
                          bp.max_diameter, bp.max_aspect, bp.min_fill, bp.border, bp.max_axis_ratio, bp.ring_scale, bp.ring_contrast,
                          bp.ring_max_bright, bp.max_blobs};
        MTL::Buffer* raw[2] = {im.bound_raw_l ? im.bound_raw_l : im.raw_l.get(), im.bound_raw_r ? im.bound_raw_r : im.raw_r.get()};
        for (int side = 0; side < 2; ++side) {
            const auto& b = im.blob_bufs[static_cast<std::size_t>(side)];
            *static_cast<std::uint32_t*>(b.n_out->contents()) = 0;
            gpu::Context::cpu_modified(b.n_out.get());
            auto bind_stats = [&] {
                enc->setBuffer(raw[side], 0, 0);
                enc->setBuffer(b.L.get(), 0, 1);
                enc->setBuffer(b.count.get(), 0, 2);
                enc->setBuffer(b.x0.get(), 0, 3);
                enc->setBuffer(b.y0.get(), 0, 4);
                enc->setBuffer(b.x1.get(), 0, 5);
                enc->setBuffer(b.y1.get(), 0, 6);
                enc->setBuffer(b.peak.get(), 0, 7);
                enc->setBuffer(b.moments.get(), 0, 8);
            };
            bind_stats();
            enc->setBytes(&ba, sizeof(ba), 9);
            dispatch2d(im.blob_init.get(), im.raw_w, im.raw_h);
            gpu::profile::split(im.ctx->queue(), cmd, enc, "stereo/blobs: init");
            enc->memoryBarrier(MTL::BarrierScopeBuffers);
            enc->setBuffer(raw[side], 0, 0);
            enc->setBuffer(b.L.get(), 0, 1);
            enc->setBytes(&ba, sizeof(ba), 2);
            dispatch2d(im.blob_merge.get(), im.raw_w, im.raw_h);
            gpu::profile::split(im.ctx->queue(), cmd, enc, "stereo/blobs: merge");
            enc->memoryBarrier(MTL::BarrierScopeBuffers);
            bind_stats();
            enc->setBytes(&ba, sizeof(ba), 9);
            dispatch2d(im.blob_stats.get(), im.raw_w, im.raw_h);
            gpu::profile::split(im.ctx->queue(), cmd, enc, "stereo/blobs: stats");
            enc->memoryBarrier(MTL::BarrierScopeBuffers);
            bind_stats();
            enc->setBuffer(b.out.get(), 0, 9);
            enc->setBuffer(b.n_out.get(), 0, 10);
            enc->setBytes(&ba, sizeof(ba), 11);
            dispatch2d(im.blob_select.get(), im.raw_w, im.raw_h);
        }
    }
    // The blob search is its own command buffer: its results are handed to the caller (on_blobs) while
    // the stereo below still runs.
    MTL::CommandBuffer* blob_cmd = nullptr;
    if (blobs) {
        enc->endEncoding();
        gpu::Context::sync_for_cpu(cmd, {im.blob_bufs[0].out.get(), im.blob_bufs[0].n_out.get(), im.blob_bufs[1].out.get(),
                                         im.blob_bufs[1].n_out.get()});
        if (gpu::profile::enabled()) gpu::profile::commit_and_wait(cmd, "stereo/blobs: select");
        else cmd->commit();
        blob_cmd = cmd;
        cmd = im.ctx->queue()->commandBuffer();
        enc = cmd->computeCommandEncoder();
    }
    if (from_raw) {
        const RectifyArgs ra{static_cast<std::uint32_t>(im.raw_w), static_cast<std::uint32_t>(im.raw_h),
                             static_cast<std::uint32_t>(im.full_w), static_cast<std::uint32_t>(im.full_h),
                             static_cast<std::uint32_t>(im.w), static_cast<std::uint32_t>(im.h)};
        MTL::Buffer* raw[2] = {im.bound_raw_l ? im.bound_raw_l : im.raw_l.get(), im.bound_raw_r ? im.bound_raw_r : im.raw_r.get()};
        for (int side = 0; side < 2; ++side) {
            enc->setBuffer(raw[side], 0, 0);
            enc->setBuffer(side ? im.map_r.get() : im.map_l.get(), 0, 1);
            enc->setBuffer(side ? im.img_r[0].get() : im.img_l[0].get(), 0, 2);
            enc->setBytes(&ra, sizeof(ra), 3);
            dispatch2d(im.rectify.get(), im.w, im.h);
        }
    }
    gpu::profile::split(im.ctx->queue(), cmd, enc, "stereo/rectify");
    for (int l = 0; l < levels; ++l) {
        const Size2 s{static_cast<std::uint32_t>(im.lw[static_cast<std::size_t>(l)]), static_cast<std::uint32_t>(im.lh[static_cast<std::size_t>(l)])};
        for (int side = 0; side < 2; ++side) {
            enc->setBuffer((side ? im.img_r : im.img_l)[static_cast<std::size_t>(l)].get(), 0, 0);
            enc->setBuffer((side ? im.img_r : im.img_l)[static_cast<std::size_t>(l + 1)].get(), 0, 1);
            enc->setBytes(&s, sizeof(s), 2);
            dispatch2d(im.down.get(), im.lw[static_cast<std::size_t>(l + 1)], im.lh[static_cast<std::size_t>(l + 1)]);
        }
    }
    gpu::profile::split(im.ctx->queue(), cmd, enc, "stereo/pyramid");
    // --- SGM at the coarsest level ---
    const auto top = static_cast<std::size_t>(levels);
    const int cw = im.lw[top], ch = im.lh[top];
    const CensusArgs ca{static_cast<std::uint32_t>(cw), static_cast<std::uint32_t>(ch), sp.census_radius_x, sp.census_radius_y};
    for (int side = 0; side < 2; ++side) {
        enc->setBuffer((side ? im.img_r : im.img_l)[top].get(), 0, 0);
        enc->setBuffer(side ? im.census_r.get() : im.census_l.get(), 0, 1);
        enc->setBytes(&ca, sizeof(ca), 2);
        dispatch2d(im.census.get(), cw, ch);
    }
    const auto invalid_cost = static_cast<std::uint16_t>((2 * sp.census_radius_x + 1) * (2 * sp.census_radius_y + 1));
    const CostArgs co{static_cast<std::uint32_t>(cw), static_cast<std::uint32_t>(ch), static_cast<std::uint32_t>(im.nd),
                      sp.min_disparity, invalid_cost, 0};
    enc->setComputePipelineState(im.cost.get());
    enc->setBuffer(im.census_l.get(), 0, 0);
    enc->setBuffer(im.census_r.get(), 0, 1);
    enc->setBuffer(im.cost_vol.get(), 0, 2);
    enc->setBytes(&co, sizeof(co), 3);
    enc->dispatchThreads(MTL::Size(static_cast<NS::UInteger>(im.nd), static_cast<NS::UInteger>(cw), static_cast<NS::UInteger>(ch)),
                         MTL::Size(64, 4, 1));
    gpu::profile::split(im.ctx->queue(), cmd, enc, "stereo/census+cost");
    enc->memoryBarrier(MTL::BarrierScopeBuffers);
    // Accumulate each path direction in turn (the first one initialises the sums).
    struct Dir { int dx, dy; };
    std::vector<Dir> dirs = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    if (sp.eight_paths) dirs.insert(dirs.end(), {{1, 1}, {-1, -1}, {1, -1}, {-1, 1}});

    for (std::size_t di = 0; di < dirs.size(); ++di) {
        const Dir d = dirs[di];
        const PathArgs pa{static_cast<std::uint32_t>(cw), static_cast<std::uint32_t>(ch), static_cast<std::uint32_t>(im.nd),
                          d.dx, d.dy, sp.p1, sp.p2, sp.adaptive_p2 ? 1u : 0u, sp.slant_steps ? 1u : 0u,
                          di == 0 ? 1u : 0u};
        const int lines = d.dy == 0 ? ch : d.dx == 0 ? cw : ch + cw - 1;
        enc->setComputePipelineState(im.path_simd ? im.path_simd.get() : im.path.get());
        enc->setBuffer(im.cost_vol.get(), 0, 0);
        enc->setBuffer(im.sum_vol.get(), 0, 1);
        enc->setBuffer(im.img_l[top].get(), 0, 2);
        enc->setBytes(&pa, sizeof(pa), 3);
        const NS::UInteger tg = im.path_simd ? im.path_simd_width : static_cast<NS::UInteger>((im.nd + 31) / 32 * 32);
        enc->dispatchThreadgroups(MTL::Size(static_cast<NS::UInteger>(lines), 1, 1), MTL::Size(tg, 1, 1));
        // Directions accumulate into the same sums: serialise them.
        enc->memoryBarrier(MTL::BarrierScopeBuffers);
    }
    gpu::profile::split(im.ctx->queue(), cmd, enc, "stereo/sgm paths");
    const WtaArgs wa{static_cast<std::uint32_t>(cw), static_cast<std::uint32_t>(ch), static_cast<std::uint32_t>(im.nd),
                     sp.min_disparity, sp.uniqueness, im.p.subpixel ? 1u : 0u};
    if (im.use_wta_rows) {
        enc->setComputePipelineState(im.wta_rows.get());
        enc->setBuffer(im.sum_vol.get(), 0, 0);
        enc->setBuffer(im.disp[top].get(), 0, 1);
        enc->setBuffer(im.rdisp.get(), 0, 2);
        enc->setBytes(&wa, sizeof(wa), 3);
        enc->dispatchThreadgroups(MTL::Size(static_cast<NS::UInteger>(ch), 1, 1), MTL::Size(256, 1, 1));
        gpu::profile::split(im.ctx->queue(), cmd, enc, "stereo/wta rows");
    } else {
        enc->setBuffer(im.sum_vol.get(), 0, 0);
        enc->setBuffer(im.disp[top].get(), 0, 1);
        enc->setBytes(&wa, sizeof(wa), 2);
        dispatch2d(im.wta_l.get(), cw, ch);
        gpu::profile::split(im.ctx->queue(), cmd, enc, "stereo/wta left");
        enc->setBuffer(im.sum_vol.get(), 0, 0);
        enc->setBuffer(im.rdisp.get(), 0, 1);
        enc->setBytes(&wa, sizeof(wa), 2);
        dispatch2d(im.wta_r.get(), cw, ch);
        gpu::profile::split(im.ctx->queue(), cmd, enc, "stereo/wta right");
    }
    enc->memoryBarrier(MTL::BarrierScopeBuffers);
    const LrArgs la{static_cast<std::uint32_t>(cw), static_cast<std::uint32_t>(ch), static_cast<float>(sp.lr_max_diff)};
    enc->setBuffer(im.disp[top].get(), 0, 0);
    enc->setBuffer(im.rdisp.get(), 0, 1);
    enc->setBytes(&la, sizeof(la), 2);
    dispatch2d(im.lr.get(), cw, ch);
    gpu::profile::split(im.ctx->queue(), cmd, enc, "stereo/lr check");

    // --- coarse-to-fine refinement ---
    for (int l = levels - 1; l >= 0; --l) {
        const auto lc = static_cast<std::size_t>(l + 1), lf = static_cast<std::size_t>(l);
        const Size2 cs{static_cast<std::uint32_t>(im.lw[lc]), static_cast<std::uint32_t>(im.lh[lc])};
        enc->memoryBarrier(MTL::BarrierScopeBuffers);
        enc->setBuffer(im.disp[lc].get(), 0, 0);
        enc->setBuffer(im.med[lc].get(), 0, 1);
        enc->setBytes(&cs, sizeof(cs), 2);
        dispatch2d(im.median.get(), im.lw[lc], im.lh[lc]);
        enc->memoryBarrier(MTL::BarrierScopeBuffers);
        const RefineArgs rf{static_cast<std::uint32_t>(im.lw[lf]), static_cast<std::uint32_t>(im.lh[lf]),
                            static_cast<std::uint32_t>(im.lw[lc]), static_cast<std::uint32_t>(im.lh[lc]), im.p.refine.zncc_radius,
                            im.p.refine.search_radius, im.p.refine.min_zncc, im.p.subpixel ? 1u : 0u};
        enc->setBuffer(im.img_l[lf].get(), 0, 0);
        enc->setBuffer(im.img_r[lf].get(), 0, 1);
        enc->setBuffer(im.med[lc].get(), 0, 2);
        enc->setBuffer(im.disp[lf].get(), 0, 3);
        enc->setBuffer(im.conf[lf].get(), 0, 4);
        enc->setBytes(&rf, sizeof(rf), 5);
        dispatch2d(im.refine.get(), im.lw[lf], im.lh[lf]);
    }
    gpu::profile::split(im.ctx->queue(), cmd, enc, "stereo/median+refine");
    // --- speckle removal: connected components on the GPU (same rule as depth::remove_speckles) ---
    const int fw = im.lw[0], fh = im.lh[0];
    if (im.p.speckle.max_region_size > 0) {
        const CclArgs cc{static_cast<std::uint32_t>(fw), static_cast<std::uint32_t>(fh), im.p.speckle.max_diff,
                         static_cast<std::uint32_t>(im.p.speckle.max_region_size)};
        enc->memoryBarrier(MTL::BarrierScopeBuffers);
        enc->setBuffer(im.disp[0].get(), 0, 0);
        enc->setBuffer(im.labels.get(), 0, 1);
        enc->setBuffer(im.sizes.get(), 0, 2);
        enc->setBytes(&cc, sizeof(cc), 3);
        dispatch2d(im.ccl_init.get(), fw, fh);
        enc->memoryBarrier(MTL::BarrierScopeBuffers);
        enc->setBuffer(im.disp[0].get(), 0, 0);
        enc->setBuffer(im.labels.get(), 0, 1);
        enc->setBytes(&cc, sizeof(cc), 2);
        dispatch2d(im.ccl_merge.get(), fw, fh);
        enc->memoryBarrier(MTL::BarrierScopeBuffers);
        enc->setBuffer(im.disp[0].get(), 0, 0);
        enc->setBuffer(im.labels.get(), 0, 1);
        enc->setBuffer(im.sizes.get(), 0, 2);
        enc->setBytes(&cc, sizeof(cc), 3);
        dispatch2d(im.ccl_count.get(), fw, fh);
        enc->memoryBarrier(MTL::BarrierScopeBuffers);
        enc->setBuffer(im.disp[0].get(), 0, 0);
        enc->setBuffer(im.conf[0].get(), 0, 1);
        enc->setBuffer(im.labels.get(), 0, 2);
        enc->setBuffer(im.sizes.get(), 0, 3);
        enc->setBytes(&cc, sizeof(cc), 4);
        dispatch2d(im.ccl_filter.get(), fw, fh);
    }
    gpu::profile::split(im.ctx->queue(), cmd, enc, "stereo/speckle ccl");
    // --- points / normals / weights ---
    if (make_points) {
        enc->memoryBarrier(MTL::BarrierScopeBuffers);
        enc->setBuffer(im.disp[0].get(), 0, 0);
        enc->setBuffer(im.conf[0].get(), 0, 1);
        enc->setBuffer(im.current.points.get(), 0, 2);
        enc->setBytes(&im.points_args, sizeof(im.points_args), 3);
        dispatch2d(im.pts_kernel.get(), fw, fh);
        enc->memoryBarrier(MTL::BarrierScopeBuffers);
        enc->setBuffer(im.current.points.get(), 0, 0);
        enc->setBuffer(im.conf[0].get(), 0, 1);
        enc->setBuffer(im.current.normals.get(), 0, 2);
        enc->setBuffer(im.current.weights.get(), 0, 3);
        enc->setBytes(&im.points_args, sizeof(im.points_args), 4);
        dispatch2d(im.nrm_kernel.get(), fw, fh);
    }
    enc->endEncoding();
    if (gpu::profile::enabled()) {
        // Close out the last stage on its own so the previews blit is timed separately.
        gpu::profile::commit_and_wait(cmd, make_points ? "stereo/points+normals" : "stereo/tail");
        cmd = im.ctx->queue()->commandBuffer();
    }
    if (request && request->preview_textures && outputs) {
        // Previews stay on the GPU: copy the rectified pair into textures for display.
        auto* td = MTL::TextureDescriptor::texture2DDescriptor(MTL::PixelFormatR8Unorm, static_cast<NS::UInteger>(fw),
                                                               static_cast<NS::UInteger>(fh), false);
        td->setStorageMode(MTL::StorageModePrivate);
        td->setUsage(MTL::TextureUsageShaderRead);
        td->setSwizzle(MTL::TextureSwizzleChannels(MTL::TextureSwizzleRed, MTL::TextureSwizzleRed, MTL::TextureSwizzleRed, MTL::TextureSwizzleOne));
        outputs->preview_left = Ref<MTL::Texture>(im.ctx->device()->newTexture(td));
        outputs->preview_right = Ref<MTL::Texture>(im.ctx->device()->newTexture(td));
        MTL::BlitCommandEncoder* be = cmd->blitCommandEncoder();
        for (int side = 0; side < 2; ++side)
            be->copyFromBuffer((side ? im.img_r : im.img_l)[0].get(), 0, static_cast<NS::UInteger>(fw), static_cast<NS::UInteger>(fw * fh),
                               MTL::Size(static_cast<NS::UInteger>(fw), static_cast<NS::UInteger>(fh), 1),
                               (side ? outputs->preview_right : outputs->preview_left).get(), 0, 0, MTL::Origin(0, 0, 0));
        be->endEncoding();
    }
    // Discrete GPU: copy the frame back with it when the caller will read it (else lazily, on first read).
    if (make_points && request && request->cpu_frame_access)
        gpu::Context::sync_for_cpu(cmd, {im.current.points.get(), im.current.normals.get(), im.current.weights.get()});
    if (gpu::profile::enabled()) gpu::profile::commit_and_wait(cmd, "stereo/previews");
    else cmd->commit();
    if (blob_cmd) {
        blob_cmd->waitUntilCompleted();
        if (blob_cmd->status() != MTL::CommandBufferStatusError && outputs) {
            for (int side = 0; side < 2; ++side) {
                const auto& b = im.blob_bufs[static_cast<std::size_t>(side)];
                const std::uint32_t found = *static_cast<const std::uint32_t*>(b.n_out->contents());
                const std::uint32_t kept = std::min(found, im.blob_params->max_blobs);
                const auto* src = static_cast<const BlobBox*>(b.out->contents());
                auto& dst = outputs->blobs[static_cast<std::size_t>(side)];
                dst.assign(src, src + kept);
                // Candidate order depends on GPU scheduling: sort for reproducible results.
                std::ranges::sort(dst, [](const BlobBox& p, const BlobBox& q) { return p.y0 != q.y0 ? p.y0 < q.y0 : p.x0 < q.x0; });
                outputs->blobs_found[static_cast<std::size_t>(side)] = found;
            }
            if (request->on_blobs) request->on_blobs(outputs->blobs);  // overlaps the stereo still running
        }
    }
    cmd->waitUntilCompleted();
    timings_.gpu_ms = (cmd->GPUEndTime() - cmd->GPUStartTime()) * 1000.0;
    timings_.speckle_ms = 0;
    const bool failed = cmd->status() == MTL::CommandBufferStatusError || (blob_cmd && blob_cmd->status() == MTL::CommandBufferStatusError);
    pool->release();
    if (failed) return make_error(Errc::io, "Metal stereo command buffer failed");
    const auto n = static_cast<std::size_t>(fw * fh);
    if (rect_left) {
        *rect_left = ImageU8(fw, fh);
        im.ctx->download(im.img_l[0].get(), 0, rect_left->data(), n);
    }
    if (rect_right) {
        *rect_right = ImageU8(fw, fh);
        im.ctx->download(im.img_r[0].get(), 0, rect_right->data(), n);
    }
    timings_.total_ms = total.elapsed_ms();
    return {};
}

}  // namespace einstar::depth_metal
