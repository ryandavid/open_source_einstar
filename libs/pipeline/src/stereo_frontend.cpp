#include "einstar/pipeline/stereo_frontend.hpp"

#include <algorithm>
#include <cmath>

#include "einstar/core/log.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/depth/point_image.hpp"

namespace einstar::pipeline {

StereoFrontend::StereoFrontend(const RigCalibration& rig, StereoFrontendParams params) : params_(params) {
    rect_ = calib::compute_rectification(rig);
    map_left_ = calib::build_remap(rig.left, rect_.R_left, rect_.rectified);
    map_right_ = calib::build_remap(rig.right, rect_.R_right, rect_.rectified);

    // Stereo runs on the half-resolution rectified pair; SGM one level below (quarter resolution).
    const auto& g = rect_.geometry;
    stereo_.pyramid_levels = 1;
    const double d_max = g.disparity_from_depth(params_.min_depth_mm) / 4.0;
    const double d_min = g.disparity_from_depth(params_.max_depth_mm) / 4.0;
    stereo_.sgm.min_disparity = std::max(0, static_cast<int>(std::floor(d_min)) - 2);
    stereo_.sgm.num_disparities = static_cast<int>(std::ceil(d_max - d_min)) + 4;
    stereo_.refine = params_.refine;
    stereo_.speckle = params_.speckle;

    const track::Intrinsics full{rect_.rectified.width, rect_.rectified.height, rect_.rectified.fx, rect_.rectified.fy,
                                 rect_.rectified.cx, rect_.rectified.cy};
    depth_k_ = full.scaled(0.5);

    if (params_.backend != StereoBackend::cpu) {
        auto ctx = gpu::Context::create();
        auto m = ctx ? depth_metal::MetalStereo::create(*ctx, stereo_, rect_.rectified.width / 2, rect_.rectified.height / 2)
                     : Result<std::unique_ptr<depth_metal::MetalStereo>>(std::unexpected(ctx.error()));
        if (m && (*m)->set_rectification(map_left_, map_right_, rig.left.width, rig.left.height)) {
            metal_ = std::move(*m);
        } else if (params_.backend == StereoBackend::metal) {
            log::error("Metal stereo unavailable: {}", m ? "rectification setup failed" : m.error().message);
        }
        if (!metal_) log::warn("stereo: falling back to the CPU implementation");
    }
    log::info("stereo frontend: rectified f {:.1f}, baseline {:.2f} mm, SGM disparities {}..{} at quarter res ({})",
              rect_.rectified.fx, g.baseline, stereo_.sgm.min_disparity,
              stereo_.sgm.min_disparity + stereo_.sgm.num_disparities, metal_ ? "Metal" : "CPU");
}

DepthOutput StereoFrontend::process(const ImageU8& raw_left, const ImageU8& raw_right) const {
    Stopwatch sw;
    DepthOutput out;
    depth::StereoResult stereo;
    bool done = false;
    if (metal_) {
        std::lock_guard lock(metal_mutex_);
        if (auto r = metal_->compute_raw(raw_left.view(), raw_right.view(), &out.rectified_left, &out.rectified_right)) {
            stereo = std::move(*r);
            done = true;
        } else {
            log::warn("Metal stereo failed ({}); using CPU for this frame", r.error().message);
        }
    }
    if (!done) {
        const ImageU8 rl = calib::remap(raw_left.view(), map_left_);
        const ImageU8 rr = calib::remap(raw_right.view(), map_right_);
        out.rectified_left = depth::downsample2(rl.view());
        out.rectified_right = depth::downsample2(rr.view());
        stereo = depth::compute_disparity(out.rectified_left.view(), out.rectified_right.view(), stereo_);
    }

    // Disparity at half resolution -> depth with the half-resolution geometry.
    depth::RectifiedGeometry half = rect_.geometry;
    half.f = depth_k_.fx;
    half.cx = depth_k_.cx;
    half.cy = depth_k_.cy;
    depth::PointImageParams pp;
    pp.min_depth = static_cast<float>(params_.min_depth_mm);
    pp.max_depth = static_cast<float>(params_.max_depth_mm);
    const auto pts = depth::disparity_to_points(stereo.disparity, stereo.confidence, half, pp);
    out.frame.intrinsics = depth_k_;
    out.frame.points = pts.points;
    out.frame.normals = pts.normals;
    out.frame.weights = pts.weights;
    out.stereo_ms = sw.elapsed_ms();
    return out;
}

std::optional<DepthOutput> StereoFrontend::process(const usb::FrameGroup& group) const {
    const auto& l = group.sensors[static_cast<std::size_t>(left_sensor_)];
    const auto& r = group.sensors[static_cast<std::size_t>(1 - left_sensor_)];
    if (!l || !r) return std::nullopt;
    auto out = process(l->pixels, r->pixels);
    out.frame.index = group.frame_id;
    out.frame.timestamp_s = static_cast<double>(group.timestamp) * 1e-6;
    return out;
}

int StereoFrontend::detect_sensor_order(const usb::FrameGroup& group) {
    if (!group.sensors[0] || !group.sensors[1]) return left_sensor_;
    auto count = [](const DepthOutput& o) {
        return std::ranges::count_if(o.frame.points.pixels(), [](const Vec3f& p) { return p.z() > 0; });
    };
    const auto a = count(process(group.sensors[0]->pixels, group.sensors[1]->pixels));
    const auto b = count(process(group.sensors[1]->pixels, group.sensors[0]->pixels));
    left_sensor_ = a >= b ? 0 : 1;
    log::info("sensor order: left IR is sensor {} ({} vs {} valid depth pixels)", left_sensor_, a, b);
    return left_sensor_;
}

}  // namespace einstar::pipeline
