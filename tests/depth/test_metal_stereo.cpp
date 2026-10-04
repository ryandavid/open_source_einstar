#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <print>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/depth/point_image.hpp"
#include "einstar/depth/stereo.hpp"
#include "einstar/depth_metal/depth_packer.hpp"
#include "einstar/depth_metal/metal_stereo.hpp"
#include "einstar/markers/detect.hpp"
#include "einstar/synth/demo.hpp"
#include "einstar/synth/speckle_scene.hpp"

using namespace einstar;

namespace {

struct Pair {
    RigCalibration rig;
    ImageU8 left, right;
};

// Realistic pair: the scanner's calibration (tests/fixtures), cluttered lit scene.
Pair make_pair() {
    Pair p;
    if (auto cal = calib::load_ccf_directory(EINSTAR_TEST_CALIBRATION_DIR)) {
        p.rig = cal->rig();
    } else {
        auto& r = p.rig;
        r.left.width = r.right.width = 1280;
        r.left.height = r.right.height = 1024;
        r.left.fx = r.left.fy = r.right.fx = r.right.fy = 1157.3;
        r.left.cx = 625.4; r.left.cy = 522.4; r.right.cx = 633.8; r.right.cy = 506.0;
        SE3 T = SE3::Identity();
        T.linear() = Eigen::AngleAxisd(22.15 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
        T.translation() = -T.linear() * Vec3(156.9, 0.2, 30.7);
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
    const auto mr = calib::build_remap(pr.rig.right, rect.R_right, rect.rectified_right);

    // CPU reference path (as in the stereo frontend).
    const auto hl = depth::downsample2(calib::remap(pr.left.view(), ml).view());
    const auto hr = depth::downsample2(calib::remap(pr.right.view(), mr).view());
    depth::StereoParams sp;
    sp.pyramid_levels = 1;
    const auto& g = rect.geometry;
    sp.sgm.min_disparity = static_cast<int>(std::floor(g.disparity_from_depth(700) / 4)) - 2;
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
        if (depth::valid_disparity(a) && depth::valid_disparity(b)) {
            ++both;
            if (std::abs(a - b) <= 0.25f) ++close;
        } else if (depth::valid_disparity(a)) {
            ++only_cpu;
        } else if (depth::valid_disparity(b)) {
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
    const auto mr = calib::build_remap(pr.rig.right, rect.R_right, rect.rectified_right);
    depth::StereoParams sp;
    sp.pyramid_levels = 1;
    const auto& g = rect.geometry;
    sp.sgm.min_disparity = static_cast<int>(std::floor(g.disparity_from_depth(700) / 4)) - 2;
    sp.sgm.num_disparities = static_cast<int>(g.disparity_from_depth(150) / 4) - sp.sgm.min_disparity + 4;
    auto gpu = depth_metal::MetalStereo::create(*ctx, sp, ml.width / 2, ml.height / 2);
    REQUIRE(gpu.has_value());
    REQUIRE((*gpu)->set_rectification(ml, mr, pr.left.width(), pr.left.height()).has_value());
    const depth::RectifiedGeometry half = g.scaled(1);
    (*gpu)->set_point_params(half, 150.0f, 700.0f, 4.0f);

    // Reference: GPU disparity (already speckle-filtered on the GPU) converted on the CPU.
    auto disp = (*gpu)->compute_raw(pr.left.view(), pr.right.view());
    REQUIRE(disp.has_value());
    depth::PointImageParams pp;
    pp.min_depth = 150.0f;
    pp.max_depth = 700.0f;
    const auto ref = depth::disparity_to_points(disp->disparity, disp->confidence, half, pp);

    // Several frames in a row: exercises the buffer pool, and state that must not leak between frames
    // (e.g. the SGM sums are initialised by the first path, not cleared).
    auto frame = (*gpu)->compute_frame_raw(pr.left.view(), pr.right.view());
    for (int i = 0; i < 3; ++i) frame = (*gpu)->compute_frame_raw(pr.left.view(), pr.right.view());
    {
        auto again = (*gpu)->compute_raw(pr.left.view(), pr.right.view());
        REQUIRE(again.has_value());
        CHECK(std::equal(again->disparity.pixels().begin(), again->disparity.pixels().end(), disp->disparity.pixels().begin()));
    }
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

TEST_CASE("compute_frame (raw images used in place, previews, blobs) matches the copying path; GPU packing matches the CPU") {
    auto ctx = gpu::Context::create();
    REQUIRE(ctx.has_value());
    // The table scene with marker stickers, seen from above.
    Pair pr = make_pair();
    {
        synth::Scene scene = synth::table_scene();
        scene.markers = synth::scatter_markers(45, 5);
        const SE3 T_lr = pr.rig.T_right_left.inverse();
        const double half_toe = 0.5 * rotation_angle(pr.rig.T_right_left);
        SE3 T = synth::look_at(Vec3(-160, -150, -230), Vec3(-80, 30, 20));
        T.linear() = T.linear() * Eigen::AngleAxisd(half_toe, Vec3::UnitY()).toRotationMatrix();
        synth::Projector proj = synth::speckle_projector();
        proj.T_world_projector = T;
        proj.T_world_projector.translation() = T * (0.5 * T_lr.translation());
        proj.T_world_projector.linear() = T.linear() * Eigen::AngleAxisd(-half_toe, Vec3::UnitY()).toRotationMatrix();
        synth::RenderParams rp;
        rp.supersample = 1;
        pr.left = synth::render_view(scene, proj, pr.rig.left, T, rp).image;
        pr.right = synth::render_view(scene, proj, pr.rig.right, T * T_lr, rp).image;
    }
    const auto rect = calib::compute_rectification(pr.rig);
    const auto ml = calib::build_remap(pr.rig.left, rect.R_left, rect.rectified);
    const auto mr = calib::build_remap(pr.rig.right, rect.R_right, rect.rectified_right);
    depth::StereoParams sp;
    sp.pyramid_levels = 1;
    const auto& g = rect.geometry;
    sp.sgm.min_disparity = static_cast<int>(std::floor(g.disparity_from_depth(700) / 4)) - 2;
    sp.sgm.num_disparities = static_cast<int>(g.disparity_from_depth(150) / 4) - sp.sgm.min_disparity + 4;
    auto gpu = depth_metal::MetalStereo::create(*ctx, sp, ml.width / 2, ml.height / 2);
    REQUIRE(gpu.has_value());
    REQUIRE((*gpu)->set_rectification(ml, mr, pr.left.width(), pr.left.height()).has_value());
    const depth::RectifiedGeometry half = g.scaled(1);
    (*gpu)->set_point_params(half, 150.0f, 700.0f, 4.0f);
    (*gpu)->set_blob_params({});
    REQUIRE(reinterpret_cast<std::uintptr_t>(pr.left.data()) % kImagePageBytes == 0);  // eligible for in-place use

    ImageU8 rl, rr;
    auto copied = (*gpu)->compute_frame_raw(pr.left.view(), pr.right.view(), &rl, &rr);
    REQUIRE(copied.has_value());
    std::vector<float> ref((*copied)->points_xyzw(), (*copied)->points_xyzw() + 4 * rl.size());
    depth_metal::FrameRequest req;
    req.preview_textures = req.preview_images = req.marker_blobs = true;
    auto out = (*gpu)->compute_frame(pr.left, pr.right, req);
    REQUIRE(out.has_value());
    CHECK(std::equal(ref.begin(), ref.end(), out->frame->points_xyzw()));
    CHECK(std::equal(rl.pixels().begin(), rl.pixels().end(), out->rect_left.pixels().begin()));
    REQUIRE(out->preview_left);
    CHECK(out->preview_left->width() == static_cast<NS::UInteger>(rl.width()));

    // GPU blob candidates must include every blob the CPU detector accepts.
    const auto cpu_left = markers::detect_markers(pr.left.view());
    const auto gpu_left = markers::fit_blobs(pr.left.view(), [&] {
        std::vector<markers::Blob> b;
        for (const auto& x : out->blobs[0])
            b.push_back({static_cast<int>(x.x0), static_cast<int>(x.y0), static_cast<int>(x.x1), static_cast<int>(x.y1), static_cast<int>(x.pixels),
                         static_cast<int>(x.peak)});
        return b;
    }(), {});
    std::println("blobs: {} GPU candidates of {} found; CPU detector {} markers, GPU path {}", out->blobs[0].size(), out->blobs_found[0],
                 cpu_left.size(), gpu_left.size());
    REQUIRE(cpu_left.size() >= 10);
    CHECK(gpu_left.size() == cpu_left.size());

    // Packing for recording: same bytes as the CPU quantisation (1/50 mm, delta along rows).
    auto packer = depth_metal::DepthPacker::create(*ctx);
    REQUIRE(packer.has_value());
    auto packed = (*packer)->pack(out->frame);
    packed->wait();
    const auto bytes = packed->bytes();
    const int w = out->frame->width(), h = out->frame->height();
    const auto* d16 = reinterpret_cast<const std::uint16_t*>(bytes.data());
    int off = 0, worst = 0;
    for (int y = 0; y < h; ++y) {
        std::uint16_t acc = 0;  // decode the row (deltas -> absolute), as the reader does
        for (int x = 0; x < w; ++x) {
            acc = static_cast<std::uint16_t>(acc + d16[y * w + x]);
            const float z = out->frame->points_xyzw()[4 * (y * w + x) + 2];
            const long q = z > 0 ? std::clamp(std::lround(static_cast<double>(z) * 50.0), 1L, 65535L) : 0;
            const int d = std::abs(static_cast<int>(acc) - static_cast<int>(q));
            off += d != 0;
            worst = std::max(worst, d);
        }
    }
    std::println("packing: {} of {} pixels differ from the CPU quantisation, by at most {} (1/50 mm)", off, w * h, worst);
    CHECK(worst <= 1);  // float vs double rounding only
}
