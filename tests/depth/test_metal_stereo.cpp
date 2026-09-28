#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <print>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/depth/point_image.hpp"
#include "einstar/depth/stereo.hpp"
#include "einstar/depth_metal/metal_stereo.hpp"
#include "einstar/synth/speckle_scene.hpp"

using namespace einstar;

namespace {

struct Pair {
    RigCalibration rig;
    ImageU8 left, right;
};

// Realistic pair: Einstar-like rig (real calibration when available), cluttered lit scene.
Pair make_pair() {
    Pair p;
    const char* cache = "/Applications/EXStar.app/Contents/Resources/res/Einscan-E10/200x150";
    if (auto cal = calib::load_ccf_directory(cache)) {
        p.rig = cal->rig();
    } else {
        auto& r = p.rig;
        r.left.width = r.right.width = 1280;
        r.left.height = r.right.height = 1024;
        r.left.fx = r.left.fy = r.right.fx = r.right.fy = 1157.3;
        r.left.cx = 625.4; r.left.cy = 522.4; r.right.cx = 633.8; r.right.cy = 506.0;
        SE3 T = SE3::Identity();
        T.linear() = Eigen::AngleAxisd(-22.15 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
        T.translation() = -T.linear() * Vec3(156.9, 0.2, -30.7);
        r.T_right_left = T;
    }
    const SE3 T_lr = p.rig.T_right_left.inverse();
    synth::Scene scene;
    scene.primitives.push_back(synth::Plane{Vec3(0, 70, 0), Vec3(0, -1, 0)});
    scene.primitives.push_back(synth::Sphere{Vec3(-80, 20, 20), 45.0});
    SE3 B = SE3::Identity();
    B.linear() = Eigen::AngleAxisd(0.5, Vec3::UnitY()).toRotationMatrix();
    B.translation() = Vec3(-10, 45, 40);
    scene.primitives.push_back(synth::Box{B, Vec3(25, 25, 18)});
    synth::Projector proj;
    proj.model.fx = proj.model.fy = 800;
    proj.model.cx = 640;
    proj.model.cy = 400;
    proj.pattern = synth::DotPattern::random(1280, 800, 9000, 3.5, 11);
    SE3 T_wl = SE3::Identity();
    T_wl.linear() = Eigen::AngleAxisd(11.07 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
    T_wl.translation() = Vec3(-160, -40, -260);
    proj.T_world_projector = T_wl;
    proj.T_world_projector.translation() = T_wl * (0.5 * T_lr.translation());
    synth::RenderParams rp;
    rp.supersample = 1;
    p.left = synth::render_view(scene, proj, p.rig.left, T_wl, rp).image;
    p.right = synth::render_view(scene, proj, p.rig.right, T_wl * T_lr, rp).image;
    return p;
}

}  // namespace

TEST_CASE("Metal stereo matches the CPU reference") {
    auto ctx = gpu::Context::create();
    REQUIRE(ctx.has_value());
    const Pair pr = make_pair();
    const auto rect = calib::compute_rectification(pr.rig);
    const auto ml = calib::build_remap(pr.rig.left, rect.R_left, rect.rectified);
    const auto mr = calib::build_remap(pr.rig.right, rect.R_right, rect.rectified);

    // CPU reference path (as in the stereo frontend).
    const auto hl = depth::downsample2(calib::remap(pr.left.view(), ml).view());
    const auto hr = depth::downsample2(calib::remap(pr.right.view(), mr).view());
    depth::StereoParams sp;
    sp.pyramid_levels = 1;
    const auto& g = rect.geometry;
    sp.sgm.min_disparity = static_cast<int>(g.disparity_from_depth(700) / 4) - 2;
    sp.sgm.num_disparities = static_cast<int>(g.disparity_from_depth(150) / 4) - sp.sgm.min_disparity + 4;
    Stopwatch sw;
    const auto cpu = depth::compute_disparity(hl.view(), hr.view(), sp);
    const double cpu_ms = sw.elapsed_ms();

    auto gpu = depth_metal::MetalStereo::create(*ctx, sp, hl.width(), hl.height());
    if (!gpu) FAIL(gpu.error().message);
    REQUIRE((*gpu)->set_rectification(ml, mr, pr.left.width(), pr.left.height()).has_value());
    ImageU8 gl, gr;
    auto res = (*gpu)->compute_raw(pr.left.view(), pr.right.view(), &gl, &gr);
    if (!res) FAIL(res.error().message);
    // Timing after warm-up (pipeline compilation, first-touch allocations).
    for (int i = 0; i < 3; ++i) res = (*gpu)->compute_raw(pr.left.view(), pr.right.view(), &gl, &gr);
    const auto t = (*gpu)->last_timings();

    // Rectified images are bit-exact.
    int img_diff = 0;
    for (std::size_t i = 0; i < hl.size(); ++i) img_diff += (hl.data()[i] != gl.data()[i]) + (hr.data()[i] != gr.data()[i]);
    CHECK(img_diff == 0);

    int both = 0, only_cpu = 0, only_gpu = 0, close = 0;
    for (std::size_t i = 0; i < cpu.disparity.size(); ++i) {
        const float a = cpu.disparity.data()[i], b = res->disparity.data()[i];
        if (a >= 0 && b >= 0) {
            ++both;
            if (std::abs(a - b) <= 0.25f) ++close;
        } else if (a >= 0) {
            ++only_cpu;
        } else if (b >= 0) {
            ++only_gpu;
        }
    }
    std::println("metal stereo: gpu {:.2f} ms (+ speckle {:.2f} ms, total {:.2f} ms) vs cpu {:.1f} ms; valid both {}, cpu-only {}, gpu-only {}, within 0.25 px {:.3f}%",
                 t.gpu_ms, t.speckle_ms, t.total_ms, cpu_ms, both, only_cpu, only_gpu, 100.0 * close / std::max(1, both));
    REQUIRE(both > 20000);
    CHECK(static_cast<double>(close) / both >= 0.995);
    CHECK(static_cast<double>(only_cpu + only_gpu) / both < 0.01);
}

TEST_CASE("Metal stereo produces GPU-resident frames matching the CPU point conversion") {
    auto ctx = gpu::Context::create();
    REQUIRE(ctx.has_value());
    const Pair pr = make_pair();
    const auto rect = calib::compute_rectification(pr.rig);
    const auto ml = calib::build_remap(pr.rig.left, rect.R_left, rect.rectified);
    const auto mr = calib::build_remap(pr.rig.right, rect.R_right, rect.rectified);
    depth::StereoParams sp;
    sp.pyramid_levels = 1;
    const auto& g = rect.geometry;
    sp.sgm.min_disparity = static_cast<int>(g.disparity_from_depth(700) / 4) - 2;
    sp.sgm.num_disparities = static_cast<int>(g.disparity_from_depth(150) / 4) - sp.sgm.min_disparity + 4;
    auto gpu = depth_metal::MetalStereo::create(*ctx, sp, ml.width / 2, ml.height / 2);
    REQUIRE(gpu.has_value());
    REQUIRE((*gpu)->set_rectification(ml, mr, pr.left.width(), pr.left.height()).has_value());
    depth::RectifiedGeometry half = g;
    half.f = g.f / 2;
    half.cx = (g.cx + 0.5) / 2 - 0.5;
    half.cy = (g.cy + 0.5) / 2 - 0.5;
    (*gpu)->set_point_params(half, 150.0f, 700.0f, 4.0f);

    // Reference: GPU disparity (already speckle-filtered on the GPU) converted on the CPU.
    auto disp = (*gpu)->compute_raw(pr.left.view(), pr.right.view());
    REQUIRE(disp.has_value());
    depth::PointImageParams pp;
    pp.min_depth = 150.0f;
    pp.max_depth = 700.0f;
    const auto ref = depth::disparity_to_points(disp->disparity, disp->confidence, half, pp);

    auto frame = (*gpu)->compute_frame_raw(pr.left.view(), pr.right.view());
    for (int i = 0; i < 3; ++i) frame = (*gpu)->compute_frame_raw(pr.left.view(), pr.right.view());  // exercise the pool
    REQUIRE(frame.has_value());
    const auto t = (*gpu)->last_timings();
    const float* p = (*frame)->points_xyzw();
    const float* n = (*frame)->normals_xyzw();
    const float* w = (*frame)->weights();
    int valid = 0, mismatch = 0, nvalid = 0, nmis = 0, wmis = 0;
    for (std::size_t i = 0; i < ref.points.size(); ++i) {
        const Vec3f rp = ref.points.data()[i];
        const bool rv = rp.z() > 0, gv = p[4 * i + 3] > 0;
        if (rv != gv) { ++mismatch; continue; }
        if (!rv) continue;
        ++valid;
        if ((rp - Vec3f(p[4 * i], p[4 * i + 1], p[4 * i + 2])).norm() > 1e-3f) ++mismatch;
        const Vec3f rn = ref.normals.data()[i];
        if (rn.squaredNorm() > 0) {
            ++nvalid;
            if (rn.dot(Vec3f(n[4 * i], n[4 * i + 1], n[4 * i + 2])) < 0.9999f) ++nmis;
        }
        if (std::abs(ref.weights.data()[i] - w[i]) > 1e-4f) ++wmis;
    }
    std::println("gpu frame: {} valid points ({} mismatched), {} normals ({} mismatched), {} weight mismatches; gpu {:.2f} ms total {:.2f} ms",
                 valid, mismatch, nvalid, nmis, wmis, t.gpu_ms, t.total_ms);
    REQUIRE(valid > 15000);
    CHECK(mismatch == 0);
    CHECK(nmis <= nvalid / 1000);
    CHECK(wmis <= valid / 1000);
}
