#include "einstar/pipeline/stereo_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <future>

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
    marker_stereo_ = std::make_unique<markers::MarkerStereo>(rig, rect_, params_.marker_stereo);

    if (params_.backend != StereoBackend::cpu) {
        auto ctx = gpu::Context::create();
        auto m = ctx ? depth_metal::MetalStereo::create(*ctx, stereo_, rect_.rectified.width / 2, rect_.rectified.height / 2)
                     : Result<std::unique_ptr<depth_metal::MetalStereo>>(std::unexpected(ctx.error()));
        if (m && (*m)->set_rectification(map_left_, map_right_, rig.left.width, rig.left.height)) {
            depth::RectifiedGeometry half = rect_.geometry;
            half.f = depth_k_.fx;
            half.cx = depth_k_.cx;
            half.cy = depth_k_.cy;
            (*m)->set_point_params(half, static_cast<float>(params_.min_depth_mm), static_cast<float>(params_.max_depth_mm), 4.0f);
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

namespace {

// CPU view of a depth frame, wherever it lives.
struct FrameAccess {
    const float* dev_points = nullptr;
    const float* dev_normals = nullptr;
    const track::DepthFrame* frame = nullptr;
    int width = 0, height = 0;

    explicit FrameAccess(const track::DepthFrame& f) : frame(&f) {
        if (f.device) {
            dev_points = f.device->points_xyzw();
            dev_normals = f.device->normals_xyzw();
            width = f.device->width();
            height = f.device->height();
        } else {
            width = f.points.width();
            height = f.points.height();
        }
    }
    [[nodiscard]] Vec3f point(int x, int y) const {
        if (dev_points) {
            const float* p = dev_points + 4 * (static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x));
            return {p[0], p[1], p[2]};
        }
        return frame->points(x, y);
    }
    [[nodiscard]] Vec3f normal(int x, int y) const {
        if (dev_normals) {
            const float* n = dev_normals + 4 * (static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x));
            return {n[0], n[1], n[2]};
        }
        return frame->normals(x, y);
    }
};

}  // namespace

void StereoFrontend::add_markers(const ImageU8& raw_left, const ImageU8& raw_right, DepthOutput& out) const {
    Stopwatch sw;
    auto right_future = std::async(std::launch::async, [&] { return markers::detect_markers(raw_right.view(), params_.marker_detect); });
    const auto left = markers::detect_markers(raw_left.view(), params_.marker_detect);
    const auto right = right_future.get();
    if (left.empty() || right.empty()) {
        for (const auto& e : left) out.unmatched_left.push_back(marker_stereo_->rectify_left(e.center));
        out.marker_ms = sw.elapsed_ms();
        return;
    }

    // Markers are retro-reflective and usually leave a hole in the speckle depth, so the prior (and
    // the surface normal) come from the valid depth in a window around the marker.
    const FrameAccess fa(out.frame);
    // Annulus (in half-resolution pixels) just outside the sticker's dark ring (ring ~1.7x the disc).
    auto neighbourhood = [&](const Vec2& rect_full, double radius_full, std::vector<float>& zs, Vec3f& nsum) {
        const double cx = rect_full.x() * 0.5, cy = rect_full.y() * 0.5;
        const double r0 = std::max(2.0, 1.9 * radius_full * 0.5);
        const double r1 = r0 + std::max(3.0, 0.8 * radius_full * 0.5);
        zs.clear();
        nsum.setZero();
        const int R = static_cast<int>(std::ceil(r1));
        const int ix = static_cast<int>(std::lround(cx)), iy = static_cast<int>(std::lround(cy));
        for (int y = std::max(0, iy - R); y <= std::min(fa.height - 1, iy + R); ++y)
            for (int x = std::max(0, ix - R); x <= std::min(fa.width - 1, ix + R); ++x) {
                const double d2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
                if (d2 < r0 * r0 || d2 > r1 * r1) continue;
                const Vec3f p = fa.point(x, y);
                if (p.z() <= 0) continue;
                zs.push_back(p.z());
                nsum += fa.normal(x, y);
            }
    };
    const auto& g = rect_.geometry;
    std::vector<float> zs;
    Vec3f nsum;
    const markers::MarkerStereo::DisparityPrior prior = [&](const Vec2& rl, double radius) -> double {
        const double x = rl.x() * 0.5, y = rl.y() * 0.5, r = radius;
        if (x - r < 0 || y - r < 0 || x + r >= fa.width || y + r >= fa.height) return 0.0;  // outside the depth image
        neighbourhood(rl, radius, zs, nsum);
        if (zs.size() < 6) return -1.0;
        auto mid = zs.begin() + static_cast<std::ptrdiff_t>(zs.size() / 2);
        std::nth_element(zs.begin(), mid, zs.end());
        return g.disparity_from_depth(*mid);
    };
    out.markers = marker_stereo_->reconstruct(left, right, prior);

    std::vector<bool> used(left.size(), false);
    for (const auto& m : out.markers) {
        used[static_cast<std::size_t>(m.left_index)] = true;
        neighbourhood(m.left_rect, left[static_cast<std::size_t>(m.left_index)].a, zs, nsum);
        Vec3 n = nsum.cast<double>();
        // Fall back to facing the scanner when the surrounding surface has no normals.
        n = n.norm() > 1e-3 ? n.normalized() : Vec3(-m.position.normalized());
        track::MarkerPoint mp;
        mp.position = m.position;
        mp.normal = n;
        mp.diameter = m.diameter;
        mp.left_rect = m.left_rect;
        mp.right_rect = m.right_rect;
        out.frame.markers.push_back(mp);
    }
    for (std::size_t i = 0; i < left.size(); ++i)
        if (!used[i]) out.unmatched_left.push_back(marker_stereo_->rectify_left(left[i].center));
    out.marker_ms = sw.elapsed_ms();
}

DepthOutput StereoFrontend::process(const ImageU8& raw_left, const ImageU8& raw_right) const {
    Stopwatch sw;
    DepthOutput out;
    if (metal_) {
        // GPU path: the whole frame (points, normals, weights) stays resident on the GPU.
        std::lock_guard lock(metal_mutex_);
        if (auto f = metal_->compute_frame_raw(raw_left.view(), raw_right.view(), &out.rectified_left, &out.rectified_right)) {
            out.frame.intrinsics = depth_k_;
            out.frame.device = std::move(*f);
            out.stereo_ms = sw.elapsed_ms();
            if (params_.detect_markers) add_markers(raw_left, raw_right, out);
            return out;
        } else {
            log::warn("Metal stereo failed ({}); using CPU for this frame", f.error().message);
        }
    }
    const ImageU8 rl = calib::remap(raw_left.view(), map_left_);
    const ImageU8 rr = calib::remap(raw_right.view(), map_right_);
    out.rectified_left = depth::downsample2(rl.view());
    out.rectified_right = depth::downsample2(rr.view());
    const auto stereo = depth::compute_disparity(out.rectified_left.view(), out.rectified_right.view(), stereo_);
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
    if (params_.detect_markers) add_markers(raw_left, raw_right, out);
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
        o.frame.ensure_cpu();  // (markers are irrelevant here)
        return std::ranges::count_if(o.frame.points.pixels(), [](const Vec3f& p) { return p.z() > 0; });
    };
    const auto a = count(process(group.sensors[0]->pixels, group.sensors[1]->pixels));
    const auto b = count(process(group.sensors[1]->pixels, group.sensors[0]->pixels));
    left_sensor_ = a >= b ? 0 : 1;
    log::info("sensor order: left IR is sensor {} ({} vs {} valid depth pixels)", left_sensor_, a, b);
    return left_sensor_;
}

}  // namespace einstar::pipeline
