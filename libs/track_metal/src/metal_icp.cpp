#include "einstar/track_metal/metal_icp.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include <Eigen/Eigenvalues>

#include "einstar/core/log.hpp"

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
    float pad[2];
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
};
// Layouts must match icp_kernels.metal.inc exactly (all members are 4-byte scalars).
static_assert(sizeof(SolveArgs) == 40);
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
    Ref<MTL::ComputePipelineState> level_begin, accumulate, solve;
    Ref<MTL::Buffer> state, partials;
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
    for (auto [name, slot] : {std::pair{"icp_level_begin", &im->level_begin}, std::pair{"icp_accumulate", &im->accumulate},
                              std::pair{"icp_solve", &im->solve}}) {
        auto p = im->ctx->compute_pipeline(*lib, name);
        if (!p) return std::unexpected(p.error());
        *slot = std::move(*p);
    }
    im->state = im->ctx->buffer(sizeof(IcpState));
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
    const int W = frame.points.width(), H = frame.points.height();
    const auto n = static_cast<std::size_t>(W * H);

    // Source frame (uploaded once per frame).
    const bool same_frame = im.cached_frame == frame.points.data() && im.cached_index == frame.index &&
                            im.cached_time == frame.timestamp_s && im.cached_alpha == p.normal_balance_alpha && im.src_n == n;
    if (!same_frame) {
        if (im.src_n != n) {
            im.src_pts = im.ctx->buffer(n * 16);
            im.src_nrm = im.ctx->buffer(n * 16);
            im.src_w = im.ctx->buffer(n * 4);
            im.src_n = n;
        }
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
        im.cached_frame = frame.points.data();
        im.cached_index = frame.index;
        im.cached_time = frame.timestamp_s;
        im.cached_alpha = p.normal_balance_alpha;
    }
    // Model view.
    const auto& mk = model.intrinsics;
    const auto mn = static_cast<std::size_t>(mk.width * mk.height);
    if (im.mdl_n != mn) {
        im.mdl_pts = im.ctx->buffer(mn * 16);
        im.mdl_nrm = im.ctx->buffer(mn * 16);
        im.mdl_n = mn;
    }
    {
        auto* mp = static_cast<float*>(im.mdl_pts->contents());
        auto* mnr = static_cast<float*>(im.mdl_nrm->contents());
        for (std::size_t i = 0; i < mn; ++i) {
            const Vec3f& q = model.points.data()[i];
            const Vec3f& nq = model.normals.data()[i];
            mp[4 * i] = q.x(), mp[4 * i + 1] = q.y(), mp[4 * i + 2] = q.z(), mp[4 * i + 3] = model.valid.data()[i] ? 1.0f : 0.0f;
            mnr[4 * i] = nq.x(), mnr[4 * i + 1] = nq.y(), mnr[4 * i + 2] = nq.z(), mnr[4 * i + 3] = 0;
        }
    }
    // Linearisation centre: same definition as the CPU solver.
    const Vec3 c = track::icp_center(frame, T_init);
    res.center = c;

    IcpState st{};
    store(st.T, T_init);
    store(st.T_init, T_init);
    std::memcpy(im.state->contents(), &st, sizeof(st));

    const float cos_max = std::cos(p.max_normal_angle_deg * static_cast<float>(M_PI) / 180.0f);
    const double s2 = p.data_sigma_mm * p.data_sigma_mm;
    const double sr_mm = p.prior_sigma_deg * M_PI / 180.0 * 100.0;
    const Eigen::Matrix4f M_cw = T_model_camera.inverse().matrix().cast<float>();

    // Partials buffer sized for the finest level.
    const std::size_t max_groups = static_cast<std::size_t>((W + 15) / 16) * static_cast<std::size_t>((H + 15) / 16);
    if (!im.partials || im.partials->length() < max_groups * 32 * 4) im.partials = im.ctx->buffer(max_groups * 32 * 4);

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

        enc->setComputePipelineState(im.level_begin.get());
        enc->setBuffer(im.state.get(), 0, 0);
        enc->dispatchThreads(MTL::Size(1, 1, 1), MTL::Size(1, 1, 1));
        enc->memoryBarrier(MTL::BarrierScopeBuffers);
        for (int it = 0; it < iters; ++it) {
            enc->setComputePipelineState(im.accumulate.get());
            enc->setBuffer(im.src_pts.get(), 0, 0);
            enc->setBuffer(im.src_nrm.get(), 0, 1);
            enc->setBuffer(im.src_w.get(), 0, 2);
            enc->setBuffer(im.mdl_pts.get(), 0, 3);
            enc->setBuffer(im.mdl_nrm.get(), 0, 4);
            enc->setBuffer(im.state.get(), 0, 5);
            enc->setBuffer(im.partials.get(), 0, 6);
            enc->setBytes(&aa, sizeof(aa), 7);
            enc->dispatchThreads(MTL::Size(static_cast<NS::UInteger>(gw), static_cast<NS::UInteger>(gh), 1), MTL::Size(16, 16, 1));
            enc->memoryBarrier(MTL::BarrierScopeBuffers);
            SolveArgs sa{};
            sa.num_partials = groups;
            sa.final_level = level == 0 ? 1u : 0u;
            sa.last_iter = it == iters - 1 ? 1u : 0u;
            sa.lambda_t = static_cast<float>(s2 / (p.prior_sigma_mm * p.prior_sigma_mm));
            sa.lambda_r = static_cast<float>(s2 / (sr_mm * sr_mm));
            sa.degenerate_ratio = static_cast<float>(p.degenerate_direction_ratio);
            sa.c[0] = aa.c[0], sa.c[1] = aa.c[1], sa.c[2] = aa.c[2];
            enc->setComputePipelineState(im.solve.get());
            enc->setBuffer(im.state.get(), 0, 0);
            enc->setBuffer(im.partials.get(), 0, 1);
            enc->setBytes(&sa, sizeof(sa), 2);
            enc->dispatchThreadgroups(MTL::Size(1, 1, 1), MTL::Size(256, 1, 1));
            enc->memoryBarrier(MTL::BarrierScopeBuffers);
        }
    }
    enc->endEncoding();
    cmd->commit();
    cmd->waitUntilCompleted();
    last_gpu_ms_ = (cmd->GPUEndTime() - cmd->GPUStartTime()) * 1000.0;
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
    res.degenerate_directions = st.degenerate_dirs;
    for (int col = 0; col < 6; ++col)
        for (int row = 0; row < 6; ++row) res.degenerate_basis(row, col) = st.basis[col * 6 + row];
    return res;
}

}  // namespace einstar::track_metal
