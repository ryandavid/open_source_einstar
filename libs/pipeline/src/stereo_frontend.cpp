#include "einstar/pipeline/stereo_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <tbb/parallel_invoke.h>
#include <tbb/task_arena.h>

#include <Eigen/Eigenvalues>

#include "einstar/core/log.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/gpu/device_data.hpp"
#include "einstar/depth/point_image.hpp"

namespace einstar::pipeline {

StereoFrontend::StereoFrontend(const RigCalibration& rig, StereoFrontendParams params) : params_(params) {
    rect_ = calib::compute_rectification(rig, {.reference_depth_mm = params_.reference_depth_mm});
    map_left_ = calib::build_remap(rig.left, rect_.R_left, rect_.rectified);
    map_right_ = calib::build_remap(rig.right, rect_.R_right, rect_.rectified_right);

    // Stereo runs on the half-resolution rectified pair; SGM one level below (quarter resolution).
    // Disparities are signed (zero at the rectification's reference depth).
    const auto& g = rect_.geometry;
    stereo_.pyramid_levels = 1;
    const auto quarter = g.scaled(2);
    const double d_max = quarter.disparity_from_depth(params_.min_depth_mm);
    const double d_min = quarter.disparity_from_depth(params_.max_depth_mm);
    stereo_.sgm.min_disparity = static_cast<int>(std::floor(d_min)) - 2;
    stereo_.sgm.num_disparities = static_cast<int>(std::ceil(d_max - d_min)) + 4;
    stereo_.refine = params_.refine;
    stereo_.detail = params_.detail;
    stereo_.filter = params_.filter;
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
            (*m)->set_point_params(rect_.geometry.scaled(1), static_cast<float>(params_.min_depth_mm), static_cast<float>(params_.max_depth_mm), 4.0f);
            depth_metal::BlobParams bp;
            const auto& d = params_.marker_detect;
            bp.threshold = static_cast<std::uint32_t>(d.threshold);
            bp.min_diameter = static_cast<std::uint32_t>(std::ceil(d.min_diameter_px));
            bp.max_diameter = static_cast<std::uint32_t>(d.max_diameter_px);
            bp.max_aspect = static_cast<float>(d.max_aspect);
            bp.min_fill = static_cast<float>(d.min_fill);
            bp.border = static_cast<std::uint32_t>(d.border);
            bp.max_axis_ratio = static_cast<float>(d.max_axis_ratio);
            (*m)->set_blob_params(bp);
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

    // Pixels written through set() (to publish to a managed GPU copy; see gpu::Context::cpu_modified).
    mutable std::size_t dirty_begin = SIZE_MAX, dirty_end = 0;

    FrameAccess(const FrameAccess&) = delete;
    FrameAccess& operator=(const FrameAccess&) = delete;
    ~FrameAccess() {
        if (dirty_begin >= dirty_end) return;
        if (const auto* md = dynamic_cast<const gpu::MetalFrameData*>(frame->device.get())) {
            const std::size_t n = dirty_end - dirty_begin;
            gpu::Context::cpu_modified(md->points_buffer(), 16 * dirty_begin, 16 * n);
            gpu::Context::cpu_modified(md->normals_buffer(), 16 * dirty_begin, 16 * n);
            gpu::Context::cpu_modified(md->weights_buffer(), 4 * dirty_begin, 4 * n);
        }
    }
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
    // Writes a point (CPU images, or the GPU frame's shared buffers in place).
    void set(int x, int y, const Vec3f& p, const Vec3f& n, float w) const {
        const auto i = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
        if (const auto* md = dynamic_cast<const gpu::MetalFrameData*>(frame->device.get())) {
            auto* P = static_cast<float*>(md->points_buffer()->contents()) + 4 * i;
            auto* N = static_cast<float*>(md->normals_buffer()->contents()) + 4 * i;
            P[0] = p.x(), P[1] = p.y(), P[2] = p.z(), P[3] = 1.0f;
            N[0] = n.x(), N[1] = n.y(), N[2] = n.z(), N[3] = 0.0f;
            static_cast<float*>(md->weights_buffer()->contents())[i] = w;
            dirty_begin = std::min(dirty_begin, i);
            dirty_end = std::max(dirty_end, i + 1);
            return;
        }
        frame->points(x, y) = p;
        frame->normals(x, y) = n;
        if (!frame->weights.empty()) frame->weights(x, y) = w;
    }
    [[nodiscard]] Vec3f normal(int x, int y) const {
        if (dev_normals) {
            const float* n = dev_normals + 4 * (static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x));
            return {n[0], n[1], n[2]};
        }
        return frame->normals(x, y);
    }
};

void fill_marker_hole(const FrameAccess& fa, const track::Intrinsics& k, const StereoFrontendParams& params, const markers::Marker3D& m,
                      double radius_full) {
    // The retro-reflective sticker (and its dark ring) returns no speckle depth. It sits flat on the
    // surface, so the surface around it, fitted as a plane, fills the hole: no dimple in the model,
    // and tracking gets that little bit of surface back.
    const double cx = m.left_rect.x() * 0.5, cy = m.left_rect.y() * 0.5, r_half = radius_full * 0.5;
    const double r_fill = 2.0 * r_half, r1 = r_fill + std::max(3.0, 0.8 * r_half);
    const int R = static_cast<int>(std::ceil(r1));
    const int ix = static_cast<int>(std::lround(cx)), iy = static_cast<int>(std::lround(cy));
    std::vector<Vec3> ring;
    for (int y = std::max(0, iy - R); y <= std::min(fa.height - 1, iy + R); ++y)
        for (int x = std::max(0, ix - R); x <= std::min(fa.width - 1, ix + R); ++x) {
            const double d2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
            if (d2 < r_fill * r_fill || d2 > r1 * r1) continue;
            const Vec3f p = fa.point(x, y);
            if (p.z() > 0) ring.push_back(p.cast<double>());
        }
    if (ring.size() < 12) return;
    // Robust plane: least squares, then again on the points within 2.5x the median residual (stereo
    // errors bleed from the textureless ring into its surroundings).
    Vec3 c = Vec3::Zero(), n = Vec3::UnitZ();
    double rms = 0;
    for (int pass = 0; pass < 2; ++pass) {
        c.setZero();
        for (const auto& p : ring) c += p;
        c /= static_cast<double>(ring.size());
        Mat3 cov = Mat3::Zero();
        for (const auto& p : ring) cov += (p - c) * (p - c).transpose();
        const Eigen::SelfAdjointEigenSolver<Mat3> es(cov);
        n = es.eigenvectors().col(0);
        rms = std::sqrt(std::max(0.0, es.eigenvalues()(0)) / static_cast<double>(ring.size()));
        if (pass == 1) break;
        std::vector<double> res;
        for (const auto& p : ring) res.push_back(std::abs(n.dot(p - c)));
        auto mid = res.begin() + static_cast<std::ptrdiff_t>(res.size() / 2);
        std::nth_element(res.begin(), mid, res.end());
        const double gate = std::max(0.05, 2.5 * *mid);
        std::erase_if(ring, [&](const Vec3& p) { return std::abs(n.dot(p - c)) > gate; });
        if (ring.size() < 12) return;
    }
    if (n.dot(c) > 0) n = -n;  // towards the camera
    // Only flat surroundings, and the marker itself must lie on that plane.
    // Stereo depth noise grows with the square of the distance.
    const double zr = m.position.z() / 300.0;
    if (rms > params.marker_fill_max_rms_mm * std::max(1.0, zr * zr) || std::abs(n.dot(m.position - c)) > params.marker_fill_max_offset_mm)
        return;
    const Vec3f nf = n.cast<float>();
    for (int y = std::max(0, iy - R); y <= std::min(fa.height - 1, iy + R); ++y)
        for (int x = std::max(0, ix - R); x <= std::min(fa.width - 1, ix + R); ++x) {
            // The whole sticker (disc and ring) is textureless for stereo: its depth is either missing
            // or guessed, so all of it is replaced.
            if ((x - cx) * (x - cx) + (y - cy) * (y - cy) > r_fill * r_fill) continue;
            const Vec3 dir((x - k.cx) / k.fx, (y - k.cy) / k.fy, 1.0);
            const double den = n.dot(dir);
            if (std::abs(den) < 1e-6) continue;
            const double t = n.dot(c) / den;
            if (t <= 0) continue;
            fa.set(x, y, (t * dir).cast<float>(), nf, params.marker_fill_weight);
        }
}

}  // namespace

StereoFrontend::FittedMarkers StereoFrontend::fit_gpu_blobs(const ImageU8& raw_left, const ImageU8& raw_right,
                                                            const std::array<std::vector<depth_metal::BlobBox>, 2>& blobs) const {
    // The GPU already found the candidate blobs: only the sub-pixel fits run here.
    Stopwatch sw;
    auto to_blobs = [](const std::vector<depth_metal::BlobBox>& in) {
        std::vector<markers::Blob> b;
        b.reserve(in.size());
        for (const auto& g : in)
            b.push_back({static_cast<int>(g.x0), static_cast<int>(g.y0), static_cast<int>(g.x1), static_cast<int>(g.y1),
                         static_cast<int>(g.pixels), static_cast<int>(g.peak)});
        return b;
    };
    FittedMarkers f;
    f.candidates = static_cast<int>(blobs[0].size() + blobs[1].size());
    // Two threads: the fits overlap the GPU stereo. The whole TBB pool spent ~10 ms of CPU per frame in
    // workers spinning while the frame waits for the GPU; one thread alone no longer hides ~440
    // candidates behind the stereo (+3 ms per frame). Two keep the wall time and most of the saving
    // (einstar-bench pipeline, M4 Max: CPU 19 -> 8.9 ms/frame, wall 8.4 -> 8.1 ms).
    tbb::task_arena fit_arena(2);
    fit_arena.execute([&] {
        f.left = markers::fit_blobs(raw_left.view(), to_blobs(blobs[0]), params_.marker_detect);
        f.right = markers::fit_blobs(raw_right.view(), to_blobs(blobs[1]), params_.marker_detect);
    });
    f.ms = sw.elapsed_ms();
    return f;
}

void StereoFrontend::add_markers(const ImageU8& raw_left, const ImageU8& raw_right, DepthOutput& out, FittedMarkers* fitted) const {
    Stopwatch sw;
    std::vector<markers::Ellipse> left, right;
    double fit_ms = 0;  // (fits done while the GPU ran the stereo still count as marker CPU time)
    if (fitted) {
        out.marker_candidates = fitted->candidates;
        left = std::move(fitted->left);
        right = std::move(fitted->right);
        fit_ms = fitted->ms;
    } else {
        tbb::parallel_invoke([&] { left = markers::detect_markers(raw_left.view(), params_.marker_detect); },
                             [&] { right = markers::detect_markers(raw_right.view(), params_.marker_detect); });
    }
    if (left.empty() || right.empty()) {
        for (const auto& e : left) out.unmatched_left.push_back(marker_stereo_->rectify_left(e.center));
        out.marker_ms = fit_ms + sw.elapsed_ms();
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
    std::vector<float> zs;
    Vec3f nsum;
    const markers::MarkerStereo::DepthPrior prior = [&](const Vec2& rl, double radius) -> double {
        const double x = rl.x() * 0.5, y = rl.y() * 0.5, r = radius;
        if (x - r < 0 || y - r < 0 || x + r >= fa.width || y + r >= fa.height) return 0.0;  // outside the depth image
        neighbourhood(rl, radius, zs, nsum);
        if (zs.size() < 6) return -1.0;
        auto mid = zs.begin() + static_cast<std::ptrdiff_t>(zs.size() / 2);
        std::nth_element(zs.begin(), mid, zs.end());
        return *mid;
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
    if (params_.fill_marker_holes)
        for (const auto& m : out.markers) fill_marker_hole(fa, depth_k_, params_, m, left[static_cast<std::size_t>(m.left_index)].a);
    out.marker_ms = fit_ms + sw.elapsed_ms();
}


DepthOutput StereoFrontend::process(const ImageU8& raw_left, const ImageU8& raw_right) const {
    Stopwatch sw;
    DepthOutput out;
    if (metal_) {
        // GPU path: the whole frame (points, normals, weights) stays resident on the GPU.
        std::lock_guard lock(metal_mutex_);
        depth_metal::FrameRequest req;
        req.preview_textures = params_.gpu_previews;
        req.preview_images = params_.cpu_previews;
        req.marker_blobs = params_.detect_markers;
        // Fit the blob candidates while the GPU is still busy with the stereo.
        FittedMarkers fitted;
        if (params_.detect_markers)
            req.on_blobs = [&](const std::array<std::vector<depth_metal::BlobBox>, 2>& blobs) { fitted = fit_gpu_blobs(raw_left, raw_right, blobs); };
        req.cpu_frame_access = true;  // marker hole filling and the pipeline's depth statistics read it
        if (auto f = metal_->compute_frame(raw_left, raw_right, req)) {
            out.frame.intrinsics = depth_k_;
            out.frame.device = std::move(f->frame);
            out.rectified_left = std::move(f->rect_left);
            out.rectified_right = std::move(f->rect_right);
            out.preview_left = std::move(f->preview_left);
            out.preview_right = std::move(f->preview_right);
            out.stereo_ms = sw.elapsed_ms();
            if (params_.detect_markers) add_markers(raw_left, raw_right, out, &fitted);
            return out;
        } else {
            log::warn("Metal stereo failed ({}); using CPU for this frame", f.error().message);
        }
    }
    const ImageU8 rl = calib::remap(raw_left.view(), map_left_);
    const ImageU8 rr = calib::remap(raw_right.view(), map_right_);
    out.rectified_left = depth::downsample2(rl.view());
    out.rectified_right = depth::downsample2(rr.view());
    const auto stereo = depth::compute_disparity(out.rectified_left.view(), out.rectified_right.view(), rl.view(), rr.view(), stereo_);
    // Disparity at half resolution -> depth with the half-resolution geometry.
    const depth::RectifiedGeometry half = rect_.geometry.scaled(1);
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
    left_sensor_ = a > b ? 0 : b > a ? 1 : usb::kLeftSensor;
    log::info("sensor order: left IR is sensor {} ({} vs {} valid depth pixels)", left_sensor_, a, b);
    return left_sensor_;
}

}  // namespace einstar::pipeline
