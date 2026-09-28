#include "einstar/depth_metal/metal_stereo.hpp"

#include <cstring>
#include <mutex>
#include <format>
#include <vector>

#include "einstar/core/timing.hpp"

namespace einstar::depth_metal {
namespace {

constexpr const char* kSource =
#include "stereo_kernels.metal.inc"
    ;

struct Size2 { std::uint32_t w, h; };
struct RectifyArgs { std::uint32_t src_w, src_h, full_w, full_h, out_w, out_h; };
struct CensusArgs { std::uint32_t w, h; std::int32_t rx, ry; };
struct CostArgs { std::uint32_t w, h, nd; std::int32_t min_disp; std::uint16_t invalid_cost; std::uint16_t pad; };
struct PathArgs { std::uint32_t w, h, nd; std::int32_t dx, dy, p1, p2; std::uint32_t adaptive, slant; };
struct WtaArgs { std::uint32_t w, h, nd; std::int32_t min_disp; float uniqueness; std::uint32_t subpixel; };
struct LrArgs { std::uint32_t w, h; float max_diff; };
struct RefineArgs { std::uint32_t w, h, cw, ch; std::int32_t radius, search_radius; float min_zncc; std::uint32_t subpixel; };
struct CclArgs { std::uint32_t w, h; float max_diff; std::uint32_t min_size; };
struct PointsArgs { std::uint32_t w, h; float f, cx, cy, baseline, min_depth, max_depth, max_jump; };

using gpu::Ref;

}  // namespace

struct MetalStereo::Impl {
    std::shared_ptr<gpu::Context> ctx;
    depth::StereoParams p;
    int w = 0, h = 0;  // finest level
    std::vector<int> lw, lh;  // per level (0 = finest)
    int nd = 0;

    Ref<MTL::ComputePipelineState> rectify, down, census, cost, path, wta_l, wta_r, lr, median, refine;
    Ref<MTL::ComputePipelineState> ccl_init, ccl_merge, ccl_count, ccl_filter, pts_kernel, nrm_kernel;
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

    Ref<MTL::Buffer> buf(std::size_t bytes) { return ctx->buffer(bytes); }
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
                           std::pair{"wta_right", &im->wta_r}, std::pair{"lr_check", &im->lr},
                           std::pair{"median3", &im->median}, std::pair{"refine_slanted", &im->refine},
                           std::pair{"ccl_init", &im->ccl_init}, std::pair{"ccl_merge", &im->ccl_merge},
                           std::pair{"ccl_count", &im->ccl_count}, std::pair{"ccl_filter", &im->ccl_filter},
                           std::pair{"disparity_points", &im->pts_kernel}, std::pair{"point_normals", &im->nrm_kernel}})
        if (auto r = make(n, *slot); !r) return std::unexpected(r.error());

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
    const auto cpx = static_cast<std::size_t>(cw * ch);
    im->census_l = im->buf(cpx * 8);
    im->census_r = im->buf(cpx * 8);
    im->cost_vol = im->buf(cpx * static_cast<std::size_t>(im->nd) * 2);
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
    auto pack = [n](const calib::RemapTable& t, MTL::Buffer* b) {
        auto* d = static_cast<float*>(b->contents());
        for (std::size_t i = 0; i < n; ++i) {
            d[2 * i] = t.map_x.data()[i];
            d[2 * i + 1] = t.map_y.data()[i];
        }
    };
    pack(left, im.map_l.get());
    pack(right, im.map_r.get());
    im.raw_l = im.buf(static_cast<std::size_t>(raw_w * raw_h));
    im.raw_r = im.buf(static_cast<std::size_t>(raw_w * raw_h));
    return {};
}

Result<depth::StereoResult> MetalStereo::compute(ImageView<const std::uint8_t> left, ImageView<const std::uint8_t> right) {
    auto& im = *impl_;
    if (left.width != im.w || left.height != im.h || right.width != im.w || right.height != im.h)
        return make_error(Errc::invalid_argument, "image size does not match the configured stereo size");
    auto copy = [&](ImageView<const std::uint8_t> src, MTL::Buffer* dst) {
        auto* d = static_cast<std::uint8_t*>(dst->contents());
        for (int y = 0; y < src.height; ++y) std::memcpy(d + y * src.width, src.row(y), static_cast<std::size_t>(src.width));
    };
    copy(left, im.img_l[0].get());
    copy(right, im.img_r[0].get());
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
            const auto n = static_cast<std::size_t>(im.w * im.h);
            im.current = {im.buf(n * 16), im.buf(n * 16), im.buf(n * 4)};
        }
    }
    auto copy = [&](ImageView<const std::uint8_t> src, MTL::Buffer* dst) {
        auto* d = static_cast<std::uint8_t*>(dst->contents());
        for (int y = 0; y < src.height; ++y) std::memcpy(d + y * src.width, src.row(y), static_cast<std::size_t>(src.width));
    };
    copy(raw_left, im.raw_l.get());
    copy(raw_right, im.raw_r.get());
    if (auto r = encode_and_run(true, true, rect_left, rect_right); !r) return std::unexpected(r.error());
    auto bufs = std::move(im.current);
    Impl* owner = impl_.get();
    auto keep = std::make_shared<Impl::FrameBuffers>(bufs);
    return std::make_shared<gpu::MetalFrameData>(im.w, im.h, bufs.points, bufs.normals, bufs.weights, [owner, keep] {
        std::lock_guard lock(owner->pool_mutex);
        if (owner->pool.size() < 4) owner->pool.push_back(*keep);
    });
}

Result<depth::StereoResult> MetalStereo::compute_raw(ImageView<const std::uint8_t> raw_left, ImageView<const std::uint8_t> raw_right,
                                                     ImageU8* rect_left, ImageU8* rect_right) {
    auto& im = *impl_;
    if (!im.map_l) return make_error(Errc::invalid_argument, "set_rectification() not called");
    if (raw_left.width != im.raw_w || raw_left.height != im.raw_h) return make_error(Errc::invalid_argument, "raw size mismatch");
    auto copy = [&](ImageView<const std::uint8_t> src, MTL::Buffer* dst) {
        auto* d = static_cast<std::uint8_t*>(dst->contents());
        for (int y = 0; y < src.height; ++y) std::memcpy(d + y * src.width, src.row(y), static_cast<std::size_t>(src.width));
    };
    copy(raw_left, im.raw_l.get());
    copy(raw_right, im.raw_r.get());
    if (auto r = encode_and_run(true, false, rect_left, rect_right); !r) return std::unexpected(r.error());
    return read_result();
}

depth::StereoResult MetalStereo::read_result() const {
    const auto& im = *impl_;
    depth::StereoResult res;
    res.disparity = ImageF32(im.w, im.h);
    res.confidence = ImageF32(im.w, im.h);
    const auto n = static_cast<std::size_t>(im.w * im.h);
    std::memcpy(res.disparity.data(), im.disp[0]->contents(), n * 4);
    if (im.p.pyramid_levels == 0) {
        for (std::size_t i = 0; i < n; ++i) res.confidence.data()[i] = res.disparity.data()[i] < 0 ? 0.0f : 1.0f;
    } else {
        std::memcpy(res.confidence.data(), im.conf[0]->contents(), n * 4);
    }
    return res;
}

Result<void> MetalStereo::encode_and_run(bool from_raw, bool make_points, ImageU8* rect_left, ImageU8* rect_right) {
    auto& im = *impl_;
    Stopwatch total;
    const auto& sp = im.p.sgm;
    const int levels = im.p.pyramid_levels;
    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
    MTL::CommandBuffer* cmd = im.ctx->queue()->commandBuffer();
    MTL::ComputeCommandEncoder* enc = cmd->computeCommandEncoder();

    auto dispatch2d = [&](MTL::ComputePipelineState* pso, int w, int h) {
        enc->setComputePipelineState(pso);
        const NS::UInteger tw = 16, th = 16;
        enc->dispatchThreads(MTL::Size(static_cast<NS::UInteger>(w), static_cast<NS::UInteger>(h), 1), MTL::Size(tw, th, 1));
    };

    if (from_raw) {
        const RectifyArgs ra{static_cast<std::uint32_t>(im.raw_w), static_cast<std::uint32_t>(im.raw_h),
                             static_cast<std::uint32_t>(im.full_w), static_cast<std::uint32_t>(im.full_h),
                             static_cast<std::uint32_t>(im.w), static_cast<std::uint32_t>(im.h)};
        for (int side = 0; side < 2; ++side) {
            enc->setBuffer(side ? im.raw_r.get() : im.raw_l.get(), 0, 0);
            enc->setBuffer(side ? im.map_r.get() : im.map_l.get(), 0, 1);
            enc->setBuffer(side ? im.img_r[0].get() : im.img_l[0].get(), 0, 2);
            enc->setBytes(&ra, sizeof(ra), 3);
            dispatch2d(im.rectify.get(), im.w, im.h);
        }
    }
    for (int l = 0; l < levels; ++l) {
        const Size2 s{static_cast<std::uint32_t>(im.lw[static_cast<std::size_t>(l)]), static_cast<std::uint32_t>(im.lh[static_cast<std::size_t>(l)])};
        for (int side = 0; side < 2; ++side) {
            enc->setBuffer((side ? im.img_r : im.img_l)[static_cast<std::size_t>(l)].get(), 0, 0);
            enc->setBuffer((side ? im.img_r : im.img_l)[static_cast<std::size_t>(l + 1)].get(), 0, 1);
            enc->setBytes(&s, sizeof(s), 2);
            dispatch2d(im.down.get(), im.lw[static_cast<std::size_t>(l + 1)], im.lh[static_cast<std::size_t>(l + 1)]);
        }
    }
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
    enc->endEncoding();
    // Zero the aggregated sums, then accumulate each path direction in turn.
    MTL::BlitCommandEncoder* blit = cmd->blitCommandEncoder();
    blit->fillBuffer(im.sum_vol.get(), NS::Range(0, im.sum_vol->length()), 0);
    blit->endEncoding();
    enc = cmd->computeCommandEncoder();
    struct Dir { int dx, dy; };
    std::vector<Dir> dirs = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    if (sp.eight_paths) dirs.insert(dirs.end(), {{1, 1}, {-1, -1}, {1, -1}, {-1, 1}});
    const NS::UInteger tg = static_cast<NS::UInteger>((im.nd + 31) / 32 * 32);
    for (const Dir d : dirs) {
        const PathArgs pa{static_cast<std::uint32_t>(cw), static_cast<std::uint32_t>(ch), static_cast<std::uint32_t>(im.nd),
                          d.dx, d.dy, sp.p1, sp.p2, sp.adaptive_p2 ? 1u : 0u, sp.slant_steps ? 1u : 0u};
        const int lines = d.dy == 0 ? ch : d.dx == 0 ? cw : ch + cw - 1;
        enc->setComputePipelineState(im.path.get());
        enc->setBuffer(im.cost_vol.get(), 0, 0);
        enc->setBuffer(im.sum_vol.get(), 0, 1);
        enc->setBuffer(im.img_l[top].get(), 0, 2);
        enc->setBytes(&pa, sizeof(pa), 3);
        enc->dispatchThreadgroups(MTL::Size(static_cast<NS::UInteger>(lines), 1, 1), MTL::Size(tg, 1, 1));
        // Directions accumulate into the same sums: serialise them.
        enc->memoryBarrier(MTL::BarrierScopeBuffers);
    }
    const WtaArgs wa{static_cast<std::uint32_t>(cw), static_cast<std::uint32_t>(ch), static_cast<std::uint32_t>(im.nd),
                     sp.min_disparity, sp.uniqueness, im.p.subpixel ? 1u : 0u};
    enc->setBuffer(im.sum_vol.get(), 0, 0);
    enc->setBuffer(im.disp[top].get(), 0, 1);
    enc->setBytes(&wa, sizeof(wa), 2);
    dispatch2d(im.wta_l.get(), cw, ch);
    enc->setBuffer(im.sum_vol.get(), 0, 0);
    enc->setBuffer(im.rdisp.get(), 0, 1);
    enc->setBytes(&wa, sizeof(wa), 2);
    dispatch2d(im.wta_r.get(), cw, ch);
    enc->memoryBarrier(MTL::BarrierScopeBuffers);
    const LrArgs la{static_cast<std::uint32_t>(cw), static_cast<std::uint32_t>(ch), static_cast<float>(sp.lr_max_diff)};
    enc->setBuffer(im.disp[top].get(), 0, 0);
    enc->setBuffer(im.rdisp.get(), 0, 1);
    enc->setBytes(&la, sizeof(la), 2);
    dispatch2d(im.lr.get(), cw, ch);

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
    cmd->commit();
    cmd->waitUntilCompleted();
    timings_.gpu_ms = (cmd->GPUEndTime() - cmd->GPUStartTime()) * 1000.0;
    timings_.speckle_ms = 0;
    const bool failed = cmd->status() == MTL::CommandBufferStatusError;
    pool->release();
    if (failed) return make_error(Errc::io, "Metal stereo command buffer failed");
    const auto n = static_cast<std::size_t>(fw * fh);
    if (rect_left) {
        *rect_left = ImageU8(fw, fh);
        std::memcpy(rect_left->data(), im.img_l[0]->contents(), n);
    }
    if (rect_right) {
        *rect_right = ImageU8(fw, fh);
        std::memcpy(rect_right->data(), im.img_r[0]->contents(), n);
    }
    timings_.total_ms = total.elapsed_ms();
    return {};
}

}  // namespace einstar::depth_metal
