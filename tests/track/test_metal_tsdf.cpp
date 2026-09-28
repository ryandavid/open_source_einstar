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
