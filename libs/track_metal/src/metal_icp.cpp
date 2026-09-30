#include "einstar/track_metal/metal_icp.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include <Eigen/Eigenvalues>

#include "einstar/core/log.hpp"
#include "einstar/gpu/profile.hpp"
#include "einstar/gpu/device_data.hpp"

namespace einstar::track_metal {
namespace {

constexpr const char* kSource =
#include "icp_kernels.metal.inc"
    ;

struct IcpState {
    float T[16];
    float T_init[16];
    std::uint32_t failed, level_done, stats_written;
    std::int32_t correspondences, candidates, degenerate_dirs;
    float rms, inlier_ratio, coverage, eig_ratio;
    float marker_rms;
    std::uint32_t iterations;
    float basis[36];
    float hessian[36];
};

struct AccArgs {
    std::uint32_t W, H, step;
    float gate, huber, cos_max;
    float mfx, mfy, mcx, mcy;
    std::uint32_t mw, mh;
    float M_cw[16];
    float c[4];
};

struct SolveArgs {
    std::uint32_t num_partials, final_level, last_iter;
    float lambda_t, lambda_r, degenerate_ratio;
    float c[4];
    std::uint32_t marker_count;
    float marker_weight;
    float pad2[2];
};
// Layouts must match icp_kernels.metal.inc exactly (all members are 4-byte scalars).
static_assert(sizeof(SolveArgs) == 56);
static_assert(sizeof(AccArgs) == 128);
static_assert(sizeof(IcpState) == 464);

using gpu::Ref;

void store(float* dst, const SE3& T) {
    const Eigen::Matrix4f m = T.matrix().cast<float>();
    std::memcpy(dst, m.data(), 16 * sizeof(float));
}

SE3 load(const float* src) {
    Eigen::Matrix4f m;
    std::memcpy(m.data(), src, 16 * sizeof(float));
    SE3 T = SE3::Identity();
    T.matrix() = m.cast<double>();
    // Re-orthonormalise (float accumulation).
    const Eigen::Quaterniond q(T.linear());
    T.linear() = q.normalized().toRotationMatrix();
    return T;
}

}  // namespace

struct MetalIcp::Impl {
    std::shared_ptr<gpu::Context> ctx;
    Ref<MTL::ComputePipelineState> level_begin, iterate, bal_hist, bal_bins, bal_apply;
    Ref<MTL::Buffer> hist, w_bin;
    Ref<MTL::Buffer> state, partials, markers, arrived;
    Ref<MTL::Buffer> dispatch;  // indirect threadgroup counts of each level's iterations (3 uints per level)
    Ref<MTL::Buffer> src_pts, src_nrm, src_w, mdl_pts, mdl_nrm;
    std::size_t src_n = 0, mdl_n = 0;
    // Cache of the last uploaded frame (the tracker calls ICP several times per frame).
    const void* cached_frame = nullptr;
    std::uint64_t cached_index = ~0ull;
    double cached_time = -1;
    double cached_alpha = -1;
};

MetalIcp::MetalIcp(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
MetalIcp::~MetalIcp() = default;

Result<std::unique_ptr<MetalIcp>> MetalIcp::create(std::shared_ptr<gpu::Context> ctx) {
    auto im = std::make_unique<Impl>();
    im->ctx = std::move(ctx);
    auto lib = im->ctx->library("icp", kSource);
    if (!lib) return std::unexpected(lib.error());
    for (auto [name, slot] : {std::pair{"icp_level_begin", &im->level_begin}, std::pair{"icp_iterate", &im->iterate},
                              std::pair{"balance_hist", &im->bal_hist},
                              std::pair{"balance_bins", &im->bal_bins}, std::pair{"balance_apply", &im->bal_apply}}) {
        auto p = im->ctx->compute_pipeline(*lib, name);
        if (!p) return std::unexpected(p.error());
        *slot = std::move(*p);
    }
    // Storage by access (see gpu::Context): the state and histogram are also touched by the CPU.
    im->state = im->ctx->mirrored_buffer(sizeof(IcpState));
    im->arrived = im->ctx->gpu_buffer(4);  // zeroed on allocation; reset by icp_level_begin and each iteration
    im->hist = im->ctx->mirrored_buffer(128 * 4);
    im->w_bin = im->ctx->gpu_buffer(128 * 4);
    return std::unique_ptr<MetalIcp>(new MetalIcp(std::move(im)));
}

track::IcpFunction MetalIcp::as_function() {
    return [this](const track::DepthFrame& f, const track::RaycastResult& m, const SE3& Tm, const SE3& Ti,
                  const track::IcpParams& p) { return solve(f, m, Tm, Ti, p); };
}

track::IcpResult MetalIcp::solve(const track::DepthFrame& frame, const track::RaycastResult& model, const SE3& T_model_camera,
                                 const SE3& T_init, const track::IcpParams& p) {
    auto& im = *impl_;
    track::IcpResult res;
    const int W = frame.width(), H = frame.height();
    const auto n = static_cast<std::size_t>(W * H);

    // Source frame: bind GPU-resident frames directly, otherwise upload once per frame.
    const auto* dev_frame = dynamic_cast<const gpu::MetalFrameData*>(frame.device.get());
    const bool same_frame = im.cached_frame == static_cast<const void*>(dev_frame ? static_cast<const void*>(dev_frame) : frame.points.data()) &&
                            im.cached_index == frame.index && im.cached_time == frame.timestamp_s &&
                            im.cached_alpha == p.normal_balance_alpha && im.src_n == n;
    MTL::Buffer* src_pts = nullptr;
    MTL::Buffer* src_nrm = nullptr;
    if (dev_frame) {
        src_pts = dev_frame->points_buffer();
        src_nrm = dev_frame->normals_buffer();
    }
    if (!same_frame) {
        if (im.src_n != n) {
            im.src_pts = im.ctx->mirrored_buffer(n * 16);
            im.src_nrm = im.ctx->mirrored_buffer(n * 16);
            im.src_w = im.ctx->mirrored_buffer(n * 4);
            im.src_n = n;
        }
        if (dev_frame) {
            // Balancing weights on the GPU from the resident normals.
            const struct { std::uint32_t w, h; float alpha; std::uint32_t enabled; } ba{
                static_cast<std::uint32_t>(W), static_cast<std::uint32_t>(H), static_cast<float>(p.normal_balance_alpha),
                p.normal_balance_alpha > 0 ? 1u : 0u};
            std::memset(im.hist->contents(), 0, 128 * 4);
            gpu::Context::cpu_modified(im.hist.get());
            NS::AutoreleasePool* bp = NS::AutoreleasePool::alloc()->init();
            MTL::CommandBuffer* bcmd = im.ctx->queue()->commandBuffer();
            MTL::ComputeCommandEncoder* be = bcmd->computeCommandEncoder();
            be->setComputePipelineState(im.bal_hist.get());
            be->setBuffer(src_pts, 0, 0);
            be->setBuffer(src_nrm, 0, 1);
            be->setBuffer(im.hist.get(), 0, 2);
            be->setBytes(&ba, sizeof(ba), 3);
            be->dispatchThreads(MTL::Size(static_cast<NS::UInteger>(W), static_cast<NS::UInteger>(H), 1), MTL::Size(16, 16, 1));
            be->memoryBarrier(MTL::BarrierScopeBuffers);
            be->setComputePipelineState(im.bal_bins.get());
            be->setBuffer(im.hist.get(), 0, 0);
            be->setBuffer(im.w_bin.get(), 0, 1);
            be->setBytes(&ba, sizeof(ba), 2);
            be->dispatchThreads(MTL::Size(1, 1, 1), MTL::Size(1, 1, 1));
            be->memoryBarrier(MTL::BarrierScopeBuffers);
            be->setComputePipelineState(im.bal_apply.get());
            be->setBuffer(src_nrm, 0, 0);
            be->setBuffer(dev_frame->weights_buffer(), 0, 1);
            be->setBuffer(im.w_bin.get(), 0, 2);
            be->setBuffer(im.src_w.get(), 0, 3);
            be->setBytes(&ba, sizeof(ba), 4);
            be->dispatchThreads(MTL::Size(static_cast<NS::UInteger>(W), static_cast<NS::UInteger>(H), 1), MTL::Size(16, 16, 1));
            be->endEncoding();
            if (gpu::profile::enabled()) gpu::profile::commit_and_wait(bcmd, "icp/normal balance");
            else bcmd->commit();  // same queue: ordered before the ICP command buffer
            bp->release();
        } else {
            frame.ensure_cpu();
            const auto balance = track::normal_balance_weights(frame, p.normal_balance_alpha);
            auto* sp = static_cast<float*>(im.src_pts->contents());
            auto* sn = static_cast<float*>(im.src_nrm->contents());
            auto* sw = static_cast<float*>(im.src_w->contents());
            for (std::size_t i = 0; i < n; ++i) {
                const Vec3f& pt = frame.points.data()[i];
                const Vec3f& nr = frame.normals.data()[i];
                sp[4 * i] = pt.x(), sp[4 * i + 1] = pt.y(), sp[4 * i + 2] = pt.z(), sp[4 * i + 3] = 1;
                sn[4 * i] = nr.x(), sn[4 * i + 1] = nr.y(), sn[4 * i + 2] = nr.z(), sn[4 * i + 3] = 0;
                float w = std::max(0.1f, frame.weights.empty() ? 1.0f : frame.weights.data()[i]);
                if (!balance.empty()) w *= balance[i];
                sw[i] = w;
            }
            gpu::Context::cpu_modified(im.src_pts.get());
            gpu::Context::cpu_modified(im.src_nrm.get());
            gpu::Context::cpu_modified(im.src_w.get());
        }
        im.cached_frame = dev_frame ? static_cast<const void*>(dev_frame) : static_cast<const void*>(frame.points.data());
        im.cached_index = frame.index;
        im.cached_time = frame.timestamp_s;
        im.cached_alpha = p.normal_balance_alpha;
    }
    if (!dev_frame) {
        src_pts = im.src_pts.get();
        src_nrm = im.src_nrm.get();
    }
    // Model view: bind GPU raycasts directly, otherwise upload.
    const auto& mk = model.intrinsics;
    const auto mn = static_cast<std::size_t>(mk.width * mk.height);
    MTL::Buffer* mdl_pts = nullptr;
    MTL::Buffer* mdl_nrm = nullptr;
    if (const auto* dev_model = dynamic_cast<const gpu::MetalRaycastData*>(model.device.get())) {
        mdl_pts = dev_model->points_buffer();
        mdl_nrm = dev_model->normals_buffer();
    } else {
        if (im.mdl_n != mn) {
            im.mdl_pts = im.ctx->mirrored_buffer(mn * 16);
            im.mdl_nrm = im.ctx->mirrored_buffer(mn * 16);
            im.mdl_n = mn;
        }
        model.ensure_cpu();
        auto* mp = static_cast<float*>(im.mdl_pts->contents());
        auto* mnr = static_cast<float*>(im.mdl_nrm->contents());
        for (std::size_t i = 0; i < mn; ++i) {
            const Vec3f& q = model.points.data()[i];
            const Vec3f& nq = model.normals.data()[i];
            mp[4 * i] = q.x(), mp[4 * i + 1] = q.y(), mp[4 * i + 2] = q.z(), mp[4 * i + 3] = model.valid.data()[i] ? 1.0f : 0.0f;
            mnr[4 * i] = nq.x(), mnr[4 * i + 1] = nq.y(), mnr[4 * i + 2] = nq.z(), mnr[4 * i + 3] = 0;
        }
        gpu::Context::cpu_modified(im.mdl_pts.get());
        gpu::Context::cpu_modified(im.mdl_nrm.get());
        mdl_pts = im.mdl_pts.get();
        mdl_nrm = im.mdl_nrm.get();
    }
    // Linearisation centre: same definition as the CPU solver.
    const Vec3 c = track::icp_center(frame, T_init);
    res.center = c;

    // Marker pairs (camera p, world q) as float4 pairs.
    const auto marker_count = static_cast<std::uint32_t>(std::min<std::size_t>(p.markers.size(), 256));
    if (!im.markers) im.markers = im.ctx->mirrored_buffer(256 * 32);
    {
        auto* mk4 = static_cast<float*>(im.markers->contents());
        for (std::uint32_t m = 0; m < marker_count; ++m) {
            const auto& mm = p.markers[m];
            mk4[8 * m + 0] = static_cast<float>(mm.p_camera.x()), mk4[8 * m + 1] = static_cast<float>(mm.p_camera.y());
            mk4[8 * m + 2] = static_cast<float>(mm.p_camera.z()), mk4[8 * m + 3] = 1;
            mk4[8 * m + 4] = static_cast<float>(mm.q_world.x()), mk4[8 * m + 5] = static_cast<float>(mm.q_world.y());
            mk4[8 * m + 6] = static_cast<float>(mm.q_world.z()), mk4[8 * m + 7] = 1;
        }
        gpu::Context::cpu_modified(im.markers.get(), 0, marker_count * 32);
    }
    IcpState st{};
    store(st.T, T_init);
    store(st.T_init, T_init);
    std::memcpy(im.state->contents(), &st, sizeof(st));
    gpu::Context::cpu_modified(im.state.get());

    const float cos_max = std::cos(p.max_normal_angle_deg * static_cast<float>(M_PI) / 180.0f);
    const double s2 = p.data_sigma_mm * p.data_sigma_mm;
    const double sr_mm = p.prior_sigma_deg * M_PI / 180.0 * 100.0;
    const Eigen::Matrix4f M_cw = T_model_camera.inverse().matrix().cast<float>();

    // Partials buffer sized for the finest level.
    const std::size_t max_groups = static_cast<std::size_t>((W + 15) / 16) * static_cast<std::size_t>((H + 15) / 16);
    if (!im.partials || im.partials->length() < max_groups * 32 * 4) im.partials = im.ctx->gpu_buffer(max_groups * 32 * 4);
    if (!im.dispatch || im.dispatch->length() < static_cast<std::size_t>(p.levels) * 12)
        im.dispatch = im.ctx->gpu_buffer(static_cast<std::size_t>(p.levels) * 12);

    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
    MTL::CommandBuffer* cmd = im.ctx->queue()->commandBuffer();
    MTL::ComputeCommandEncoder* enc = cmd->computeCommandEncoder();
    for (int level = p.levels - 1; level >= 0; --level) {
        const int step = 1 << level;
        const int gw = ((W + step - 1) / step + 3) / 4, gh = (H + step - 1) / step;  // 4 pixels per thread along x
        const auto groups = static_cast<std::uint32_t>(((gw + 15) / 16) * ((gh + 15) / 16));
        AccArgs aa{};
        aa.W = static_cast<std::uint32_t>(W);
        aa.H = static_cast<std::uint32_t>(H);
        aa.step = static_cast<std::uint32_t>(step);
        aa.gate = p.max_distance_mm * static_cast<float>(1 + level);
        aa.huber = static_cast<float>(p.huber_mm * static_cast<float>(1 + level));
        aa.cos_max = cos_max;
        aa.mfx = static_cast<float>(mk.fx), aa.mfy = static_cast<float>(mk.fy), aa.mcx = static_cast<float>(mk.cx),
        aa.mcy = static_cast<float>(mk.cy);
        aa.mw = static_cast<std::uint32_t>(mk.width), aa.mh = static_cast<std::uint32_t>(mk.height);
        std::memcpy(aa.M_cw, M_cw.data(), sizeof(aa.M_cw));
        aa.c[0] = static_cast<float>(c.x()), aa.c[1] = static_cast<float>(c.y()), aa.c[2] = static_cast<float>(c.z());
        const int iters = p.iterations[static_cast<std::size_t>(std::min(level, 2))];

        // Accumulate grid in whole threadgroups (the kernel bounds-checks its pixels); launched
        // indirectly so iterations after convergence cost nothing.
        const std::uint32_t acc_groups[2] = {static_cast<std::uint32_t>((gw + 15) / 16), static_cast<std::uint32_t>((gh + 15) / 16)};
        const auto disp_offset = static_cast<NS::UInteger>(level) * 12;
        enc->setComputePipelineState(im.level_begin.get());
        enc->setBuffer(im.state.get(), 0, 0);
        enc->setBuffer(im.dispatch.get(), disp_offset, 1);
        enc->setBytes(acc_groups, sizeof(acc_groups), 2);
        enc->setBuffer(im.arrived.get(), 0, 3);
        enc->dispatchThreads(MTL::Size(1, 1, 1), MTL::Size(1, 1, 1));
        enc->memoryBarrier(MTL::BarrierScopeBuffers);
        for (int it = 0; it < iters; ++it) {
            SolveArgs sa{};
            sa.num_partials = groups;
            sa.final_level = level == 0 ? 1u : 0u;
            sa.last_iter = it == iters - 1 ? 1u : 0u;
            sa.lambda_t = static_cast<float>(s2 / (p.prior_sigma_mm * p.prior_sigma_mm));
            sa.lambda_r = static_cast<float>(s2 / (sr_mm * sr_mm));
            sa.degenerate_ratio = static_cast<float>(p.degenerate_direction_ratio);
            sa.c[0] = aa.c[0], sa.c[1] = aa.c[1], sa.c[2] = aa.c[2];
            sa.marker_count = marker_count;
            sa.marker_weight = static_cast<float>(p.marker_weight);
            enc->setComputePipelineState(im.iterate.get());
            enc->setBuffer(src_pts, 0, 0);
            enc->setBuffer(src_nrm, 0, 1);
            enc->setBuffer(im.src_w.get(), 0, 2);
            enc->setBuffer(mdl_pts, 0, 3);
            enc->setBuffer(mdl_nrm, 0, 4);
            enc->setBuffer(im.state.get(), 0, 5);
            enc->setBuffer(im.partials.get(), 0, 6);
            enc->setBytes(&aa, sizeof(aa), 7);
            enc->setBytes(&sa, sizeof(sa), 8);
            enc->setBuffer(im.markers.get(), 0, 9);
            enc->setBuffer(im.dispatch.get(), disp_offset, 10);
            enc->setBuffer(im.arrived.get(), 0, 11);
            enc->dispatchThreadgroups(im.dispatch.get(), disp_offset, MTL::Size(16, 16, 1));
            enc->memoryBarrier(MTL::BarrierScopeBuffers);
        }
    }
    enc->endEncoding();
    gpu::Context::sync_for_cpu(cmd, {im.state.get()});
    gpu::profile::commit_and_wait(cmd, "icp/iterations (all levels)");
    last_gpu_ms_ = (cmd->GPUEndTime() - cmd->GPUStartTime()) * 1000.0;
    last_iterations_ = static_cast<const IcpState*>(im.state->contents())->iterations;
    const bool gpu_error = cmd->status() == MTL::CommandBufferStatusError;
    pool->release();
    if (gpu_error) {
        log::error("Metal ICP command buffer failed");
        return res;
    }

    std::memcpy(&st, im.state->contents(), sizeof(st));
    if (st.failed) {
        res.converged = false;
        res.correspondences = st.correspondences;
        return res;
    }
    res.T_world_camera = load(st.T);
    res.converged = true;
    res.correspondences = st.correspondences;
    res.candidates = st.candidates;
    res.rms_mm = st.rms;
    res.inlier_ratio = st.inlier_ratio;
    res.coverage = st.coverage;
    res.min_eigenvalue_ratio = st.eig_ratio;
    res.marker_rms_mm = st.marker_rms;
    res.degenerate_directions = st.degenerate_dirs;
    for (int col = 0; col < 6; ++col)
        for (int row = 0; row < 6; ++row) res.degenerate_basis(row, col) = st.basis[col * 6 + row];
    return res;
}

}  // namespace einstar::track_metal
