#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <print>

#include "einstar/core/timing.hpp"
#include "einstar/track/icp.hpp"
#include "einstar/track/tsdf.hpp"
#include "einstar/track_metal/metal_tsdf.hpp"

using namespace einstar;
using namespace einstar::track;

namespace {

const Intrinsics kK{640, 512, 580.0, 580.0, 320.0, 256.0};

ImageF32 render_depth(const SE3& T_world_camera, const Intrinsics& k) {
    ImageF32 d(k.width, k.height, 0.0f);
    const SE3 T_cw = T_world_camera.inverse();
    for (int v = 0; v < k.height; ++v)
        for (int u = 0; u < k.width; ++u) {
            const Vec3 dir = T_world_camera.linear() * Vec3((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0);
            const Vec3 o = T_world_camera.translation();
            double t = 150, prev_f = 0;
            bool have = false;
            for (int i = 0; i < 1200; ++i, t += 0.5) {
                const Vec3 p = o + t * dir;
                const double bump1 = 15 * std::exp(-(p.x() * p.x() + p.y() * p.y()) / 800.0);
                const double bump2 = -8 * std::exp(-((p.x() - 35) * (p.x() - 35) + (p.y() + 25) * (p.y() + 25)) / 300.0);
                const double ridge = 5 * std::exp(-(p.y() - 0.5 * p.x() - 30) * (p.y() - 0.5 * p.x() - 30) / 50.0);
                const double f = p.z() - (300 + 0.2 * p.x() + bump1 + bump2 + ridge);
                if (have && prev_f < 0 && f >= 0) {
                    d(u, v) = static_cast<float>((T_cw * (o + (t - 0.5 * f / (f - prev_f)) * dir)).z());
                    break;
                }
                prev_f = f;
                have = true;
            }
        }
    return d;
}

}  // namespace

TEST_CASE("Metal TSDF matches the CPU volume") {
    auto ctx = gpu::Context::create();
    REQUIRE(ctx.has_value());
    track_metal::MetalTsdfOptions opt;
    opt.brick_capacity = 65536;
    opt.table_size = 1u << 18;
    auto gpu_vol = track_metal::MetalTsdfVolume::create(*ctx, {}, opt);
    if (!gpu_vol) FAIL(gpu_vol.error().message);
    TsdfVolume cpu_vol;

    const SE3 pose0 = SE3::Identity();
    const auto f0 = make_depth_frame(render_depth(pose0, kK), kK);
    Vec6 xi;
    xi << 4.0, -2.0, 3.0, 0.02, -0.03, 0.015;
    const SE3 pose1 = se3_exp(xi);
    const auto f1 = make_depth_frame(render_depth(pose1, kK), kK);

    Stopwatch sw;
    cpu_vol.integrate(f0, pose0);
    const double cpu_int = sw.elapsed_ms();
    (*gpu_vol)->integrate(f0, pose0);  // warm-up (pipeline creation)
    (*gpu_vol)->clear();
    sw.reset();
    (*gpu_vol)->integrate(f0, pose0);
    const double gpu_int = sw.elapsed_ms();
    CHECK((*gpu_vol)->brick_count() == cpu_vol.brick_count());

    const auto mk = kK.scaled(0.5);
    sw.reset();
    const auto rc = cpu_vol.raycast(pose0, mk);
    const double cpu_ray = sw.elapsed_ms();
    sw.reset();
    const auto rg = (*gpu_vol)->raycast(pose0, mk);
    const double gpu_ray = sw.elapsed_ms();
    rg.ensure_cpu();  // GPU raycasts are device-resident; CPU images on demand
    int vc = 0, vg = 0, both = 0, close = 0;
    for (std::size_t i = 0; i < rc.valid.size(); ++i) {
        vc += rc.valid.data()[i];
        vg += rg.valid.data()[i];
        if (rc.valid.data()[i] && rg.valid.data()[i]) {
            ++both;
            if ((rc.points.data()[i] - rg.points.data()[i]).norm() < 0.05f && rc.normals.data()[i].dot(rg.normals.data()[i]) > 0.99f) ++close;
        }
    }
    sw.reset();
    const auto pc = cpu_vol.extract_points();
    const double cpu_ext = sw.elapsed_ms();
    sw.reset();
    const auto pg = (*gpu_vol)->extract_points();
    const double gpu_ext = sw.elapsed_ms();
    std::println("integrate cpu {:.1f} ms gpu {:.1f} ms | raycast cpu {:.1f} gpu {:.1f} ms, valid {} vs {}, agree {:.2f}% | extract cpu {} pts {:.1f} ms, gpu {} pts {:.1f} ms",
                 cpu_int, gpu_int, cpu_ray, gpu_ray, vc, vg, 100.0 * close / std::max(1, both), pc.size(), cpu_ext, pg.size(), gpu_ext);
    CHECK(std::abs(vc - vg) < vc / 50);
    CHECK(static_cast<double>(close) / both > 0.99);
    CHECK(std::abs(static_cast<double>(pc.size()) - static_cast<double>(pg.size())) < 0.02 * static_cast<double>(pc.size()));

    // ICP against the GPU model recovers the motion as well as against the CPU model.
    const auto r = icp_point_to_plane(f1, rg, pose0, pose0, {});
    const SE3 err = pose1.inverse() * r.T_world_camera;
    std::println("icp on gpu model: err {:.3f} mm {:.4f} deg", translation_norm(err), rotation_angle(err) * 180 / M_PI);
    CHECK(translation_norm(err) < 0.1);
    CHECK(rotation_angle(err) * 180 / M_PI < 0.02);

    // Incremental: second frame, extend-only mode, updated-brick queries.
    (*gpu_vol)->integrate(f1, pose1, 0.5f, true);
    cpu_vol.integrate(f1, pose1, 0.5f, true);
    CHECK((*gpu_vol)->brick_count() == cpu_vol.brick_count());
    CHECK((*gpu_vol)->bricks_updated_since(1).size() > 0);
}

#include "einstar/track_metal/metal_icp.hpp"

TEST_CASE("Metal ICP matches the CPU solver") {
    auto ctx = gpu::Context::create();
    REQUIRE(ctx.has_value());
    auto icp = track_metal::MetalIcp::create(*ctx);
    if (!icp) FAIL(icp.error().message);
    TsdfVolume vol;
    const SE3 pose0 = SE3::Identity();
    vol.integrate(make_depth_frame(render_depth(pose0, kK), kK), pose0);
    const auto model = vol.raycast(pose0, kK.scaled(0.5));

    for (const Vec6 xi : {Vec6{1.5, -1.0, 2.0, 0.01, -0.015, 0.008}, Vec6{4.0, 2.0, -3.0, -0.02, 0.03, 0.015}}) {
        const SE3 pose1 = se3_exp(xi);
        auto f1 = make_depth_frame(render_depth(pose1, kK), kK);
        f1.index = static_cast<std::uint64_t>(xi(0) * 10);
        Stopwatch sw;
        const auto cpu = icp_point_to_plane(f1, model, pose0, pose0, {});
        const double cpu_ms = sw.elapsed_ms();
        (void)(*icp)->solve(f1, model, pose0, pose0, {});  // warm-up
        sw.reset();
        const auto gpu = (*icp)->solve(f1, model, pose0, pose0, {});
        const double gpu_ms = sw.elapsed_ms();
        REQUIRE(gpu.converged);
        const SE3 d = cpu.T_world_camera.inverse() * gpu.T_world_camera;
        const SE3 e = pose1.inverse() * gpu.T_world_camera;
        std::println("icp cpu {:.2f} ms, gpu {:.2f} ms (gpu time {:.2f} ms): gpu-cpu {:.4f} mm {:.5f} deg; gpu err {:.4f} mm | rms {:.4f}/{:.4f} inl {:.3f}/{:.3f} cov {:.3f}/{:.3f} eig {:.2e}/{:.2e} n {}/{}",
                     cpu_ms, gpu_ms, (*icp)->last_gpu_ms(), translation_norm(d), rotation_angle(d) * 180 / M_PI, translation_norm(e),
                     cpu.rms_mm, gpu.rms_mm, cpu.inlier_ratio, gpu.inlier_ratio, cpu.coverage, gpu.coverage,
                     cpu.min_eigenvalue_ratio, gpu.min_eigenvalue_ratio, cpu.correspondences, gpu.correspondences);
        CHECK(translation_norm(d) < 0.01);
        CHECK(rotation_angle(d) * 180 / M_PI < 0.002);
        CHECK(std::abs(cpu.rms_mm - gpu.rms_mm) < 0.002);
        CHECK(std::abs(cpu.correspondences - gpu.correspondences) < cpu.correspondences / 100 + 5);
        CHECK(std::abs(std::log(cpu.min_eigenvalue_ratio / gpu.min_eigenvalue_ratio)) < 0.05);
    }
}

#include "einstar/gpu/device_data.hpp"

TEST_CASE("Metal ICP on a GPU-resident frame matches the CPU solver") {
    auto ctx = gpu::Context::create();
    REQUIRE(ctx.has_value());
    auto icp = track_metal::MetalIcp::create(*ctx);
    REQUIRE(icp.has_value());
    TsdfVolume vol;
    vol.integrate(make_depth_frame(render_depth(SE3::Identity(), kK), kK), SE3::Identity());
    const auto model = vol.raycast(SE3::Identity(), kK.scaled(0.5));
    Vec6 xi;
    xi << 2.5, -1.5, 1.0, 0.012, -0.02, 0.01;
    const SE3 pose1 = se3_exp(xi);
    const auto cpu_frame = make_depth_frame(render_depth(pose1, kK), kK);

    // Same frame, but only as device buffers (as produced by the Metal stereo frontend).
    const auto n = static_cast<std::size_t>(kK.width * kK.height);
    auto pts = (*ctx)->buffer(n * 16), nrm = (*ctx)->buffer(n * 16), wts = (*ctx)->buffer(n * 4);
    auto* p = static_cast<float*>(pts->contents());
    auto* q = static_cast<float*>(nrm->contents());
    auto* w = static_cast<float*>(wts->contents());
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3f a = cpu_frame.points.data()[i], b = cpu_frame.normals.data()[i];
        p[4 * i] = a.x(), p[4 * i + 1] = a.y(), p[4 * i + 2] = a.z(), p[4 * i + 3] = a.z() > 0 ? 1.0f : 0.0f;
        q[4 * i] = b.x(), q[4 * i + 1] = b.y(), q[4 * i + 2] = b.z(), q[4 * i + 3] = 0;
        w[i] = cpu_frame.weights.data()[i];
    }
    DepthFrame dev_frame;
    dev_frame.intrinsics = kK;
    dev_frame.index = 77;
    dev_frame.device = std::make_shared<gpu::MetalFrameData>(kK.width, kK.height, pts, nrm, wts);

    const auto cpu = icp_point_to_plane(cpu_frame, model, SE3::Identity(), SE3::Identity(), {});
    const auto gpu = (*icp)->solve(dev_frame, model, SE3::Identity(), SE3::Identity(), {});
    REQUIRE(gpu.converged);
    const SE3 d = cpu.T_world_camera.inverse() * gpu.T_world_camera;
    std::println("device-frame icp: gpu-cpu {:.4f} mm {:.5f} deg, n {}/{}, rms {:.4f}/{:.4f}", translation_norm(d),
                 rotation_angle(d) * 180 / M_PI, cpu.correspondences, gpu.correspondences, cpu.rms_mm, gpu.rms_mm);
    CHECK(translation_norm(d) < 0.01);
    CHECK(rotation_angle(d) * 180 / M_PI < 0.002);
    CHECK(std::abs(cpu.correspondences - gpu.correspondences) <= cpu.correspondences / 200);
}

TEST_CASE("joint marker terms: Metal ICP matches the CPU solver") {
    auto ctx = gpu::Context::create();
    REQUIRE(ctx.has_value());
    auto icp = track_metal::MetalIcp::create(*ctx);
    REQUIRE(icp.has_value());
    TsdfVolume vol;
    vol.integrate(make_depth_frame(render_depth(SE3::Identity(), kK), kK), SE3::Identity());
    const auto model = vol.raycast(SE3::Identity(), kK.scaled(0.5));
    Vec6 xi;
    xi << 3.0, -2.0, 1.5, 0.015, -0.02, 0.012;
    const SE3 pose1 = se3_exp(xi);
    auto frame = make_depth_frame(render_depth(pose1, kK), kK);
    frame.index = 99;
    IcpParams ip;
    for (const Vec3 q : {Vec3(-40, -30, 292), Vec3(35, -25, 305), Vec3(-20, 40, 297), Vec3(50, 35, 310), Vec3(0, 0, 315)})
        ip.markers.push_back({pose1.inverse() * q, q});
    const auto cpu = icp_point_to_plane(frame, model, SE3::Identity(), SE3::Identity(), ip);
    const auto gpu = (*icp)->solve(frame, model, SE3::Identity(), SE3::Identity(), ip);
    REQUIRE((cpu.converged && gpu.converged));
    const SE3 d = cpu.T_world_camera.inverse() * gpu.T_world_camera;
    const SE3 e = pose1.inverse() * gpu.T_world_camera;
    std::println("markers joint icp: gpu-cpu {:.4f} mm {:.5f} deg; err {:.4f} mm; marker rms cpu {:.4f} gpu {:.4f}", translation_norm(d),
                 rotation_angle(d) * 180 / M_PI, translation_norm(e), cpu.marker_rms_mm, gpu.marker_rms_mm);
    CHECK(translation_norm(d) < 0.01);
    CHECK(rotation_angle(d) * 180 / M_PI < 0.002);
    CHECK(translation_norm(e) < 0.05);
    CHECK(std::abs(cpu.marker_rms_mm - gpu.marker_rms_mm) < 0.01);
}

TEST_CASE("Metal surface extraction is reproducible (canonical order)") {
    // Relocalisation samples and averages extracted points, so their order must not depend on
    // GPU scheduling; otherwise identical replays diverge.
    auto ctx = gpu::Context::create();
    if (!ctx) SKIP("no Metal device");
    auto extract = [&] {
        auto vol = track_metal::MetalTsdfVolume::create(*ctx);
        REQUIRE(vol.has_value());
        SE3 pose1 = SE3::Identity();
        pose1.translation() = Vec3(4, -2, 1);
        (*vol)->integrate(make_depth_frame(render_depth(SE3::Identity(), kK), kK), SE3::Identity());
        (*vol)->integrate(make_depth_frame(render_depth(pose1, kK), kK), pose1);
        return (*vol)->extract_points(0, 0.5f);
    };
    const auto a = extract();
    const auto b = extract();
    REQUIRE(a.size() == b.size());
    REQUIRE(!a.empty());
    bool identical = true;
    for (std::size_t i = 0; i < a.size() && identical; ++i)
        identical = a[i].position == b[i].position && a[i].normal == b[i].normal && a[i].weight == b[i].weight;
    CHECK(identical);
}
