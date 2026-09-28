#include "einstar/pipeline/scan_pipeline.hpp"

#include "einstar/core/log.hpp"
#include "einstar/gpu/context.hpp"
#include "einstar/track_metal/metal_tsdf.hpp"
#include "gpu_overlay.hpp"

#include <algorithm>

namespace einstar::pipeline {

namespace {

render::PointVertex to_vertex(const track::SurfacePoint& p) {
    // Shade by confidence: well-observed surface is lighter, thin coverage is tinted blue.
    const float w = std::clamp(p.weight / 8.0f, 0.0f, 1.0f);
    const auto grey = static_cast<std::uint8_t>(150 + 70 * w);
    const render::Rgba8 col{static_cast<std::uint8_t>(grey - static_cast<std::uint8_t>(40 * (1 - w))),
                            static_cast<std::uint8_t>(grey - static_cast<std::uint8_t>(20 * (1 - w))), grey, 255};
    return {p.position.x(), p.position.y(), p.position.z(), p.normal.x(), p.normal.y(), p.normal.z(), col};
}

}  // namespace

void ModelPointCache::update(const track::Volume& volume) {
    if (volume.fast_full_extraction()) {
        // GPU volume: re-extracting everything is cheaper than per-brick bookkeeping.
        const auto pts = volume.extract_points(0);
        full_.clear();
        full_.reserve(pts.size());
        for (const auto& p : pts) full_.push_back(to_vertex(p));
        total_ = full_.size();
        use_full_ = true;
        return;
    }
    use_full_ = false;
    const auto dirty = volume.bricks_updated_since(last_frame_);
    last_frame_ = volume.frame_counter();
    if (dirty.empty()) return;
    // Re-extract each dirty brick as a group so its entry is replaced wholesale.
    for (const auto& c : dirty) {
        auto pts = volume.extract_points(std::vector<track::BrickCoord>{c});
        auto& slot = bricks_[c];
        total_ -= slot.size();
        slot.clear();
        slot.reserve(pts.size());
        for (const auto& p : pts) slot.push_back(to_vertex(p));
        total_ += slot.size();
    }
}

std::vector<render::PointVertex> ModelPointCache::flatten() const {
    if (use_full_) return full_;
    std::vector<render::PointVertex> out;
    out.reserve(total_);
    for (const auto& [c, pts] : bricks_) out.insert(out.end(), pts.begin(), pts.end());
    return out;
}

void ModelPointCache::clear() {
    bricks_.clear();
    full_.clear();
    last_frame_ = 0;
    total_ = 0;
}

namespace {

std::unique_ptr<track::Volume> make_volume(const ScanPipelineParams& p) {
    if (!p.gpu_volume) return nullptr;
    auto ctx = gpu::Context::create();
    if (!ctx) return nullptr;
    auto v = track_metal::MetalTsdfVolume::create(*ctx, p.tracker.tsdf);
    if (!v) {
        log::warn("Metal TSDF unavailable ({}); using the CPU volume", v.error().message);
        return nullptr;
    }
    return std::move(*v);
}

}  // namespace

ScanPipeline::ScanPipeline(std::unique_ptr<StereoFrontend> frontend, ScanPipelineParams params, Sink sink)
    : frontend_(std::move(frontend)), params_(params), sink_(std::move(sink)), tracker_(params.tracker, make_volume(params)) {
    if (params_.gpu_volume) {
        if (auto ctx = gpu::Context::create())
            if (auto icp = track_metal::MetalIcp::create(*ctx)) {
                gpu_icp_ = std::move(*icp);
                tracker_.set_icp_solver(gpu_icp_->as_function());
                overlay_ = GpuOverlay::create(*ctx);
            }
        log::info("tracking: {} volume, {} ICP", tracker_.volume().fast_full_extraction() ? "Metal" : "CPU", gpu_icp_ ? "Metal" : "CPU");
    }
}

ScanPipeline::~ScanPipeline() { stop(); }

void ScanPipeline::start() {
    if (worker_.joinable()) return;
    worker_ = std::jthread([this](std::stop_token st) { run(st); });
}

void ScanPipeline::stop() {
    if (!worker_.joinable()) return;
    worker_.request_stop();
    cv_.notify_all();
    worker_.join();
    worker_ = {};
    space_cv_.notify_all();
}

void ScanPipeline::reset_model() { reset_requested_ = true; }

void ScanPipeline::push(usb::FrameGroup&& group) {
    ++frames_in_;
    {
        std::unique_lock lock(mutex_);
        if (params_.block_when_full) {
            space_cv_.wait(lock, [&] { return queue_.size() < params_.queue_capacity || !worker_.joinable(); });
        } else if (queue_.size() >= params_.queue_capacity) {
            queue_.pop_front();
            ++dropped_;
        }
        queue_.push_back(std::move(group));
    }
    cv_.notify_one();
}

std::vector<SE3> ScanPipeline::trajectory() const {
    std::lock_guard lock(trajectory_mutex_);
    return trajectory_;
}

void ScanPipeline::run(std::stop_token st) {
    while (!st.stop_requested()) {
        usb::FrameGroup group;
        {
            std::unique_lock lock(mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(100), [&] { return !queue_.empty() || st.stop_requested(); });
            if (queue_.empty()) continue;
            group = std::move(queue_.front());
            queue_.pop_front();
            stats_.queue_depth = static_cast<int>(queue_.size());
        }
        space_cv_.notify_one();
        process(std::move(group));
    }
}

void ScanPipeline::process(usb::FrameGroup&& group) {
    if (reset_requested_.exchange(false)) {
        tracker_.reset();
        cache_.clear();
        std::lock_guard lock(trajectory_mutex_);
        trajectory_.clear();
    }
    if (!order_detected_) {
        frontend_->detect_sensor_order(group);
        order_detected_ = true;
    }
    auto depth = frontend_->process(group);
    if (!depth) return;
    stereo_times_.add(depth->stereo_ms);

    const auto r = tracker_.process(depth->frame);
    track_times_.add(r.ms);
    ++stats_.frames_processed;
    if (r.accepted) {
        ++stats_.accepted;
        if (r.relocalized) ++stats_.relocalized;
        std::lock_guard lock(trajectory_mutex_);
        trajectory_.push_back(r.T_world_camera);
    } else {
        ++stats_.lost;
    }
    stats_.state = r.state;
    stats_.reason = r.reason;
    stats_.frames_in = frames_in_;
    stats_.dropped = dropped_;
    stats_.stereo_ms = stereo_times_.median();
    stats_.track_ms = track_times_.median();
    ++fps_frames_;
    if (fps_clock_.elapsed_ms() > 1000) {
        stats_.fps = 1000.0 * static_cast<double>(fps_frames_) / fps_clock_.elapsed_ms();
        fps_frames_ = 0;
        fps_clock_.reset();
    }

    LiveUpdate up;
    up.frame_id = group.frame_id;
    up.accepted = r.accepted;
    up.tracking_lost = !r.accepted;
    up.pose = r.T_world_camera;
    up.last_good_pose = tracker_.last_good_pose();
    // Current frame overlay: green when tracked, red when not.
    const render::Rgba8 col = r.accepted ? render::Rgba8{90, 230, 120, 255} : render::Rgba8{235, 70, 70, 255};
    const auto* dev = dynamic_cast<const gpu::MetalFrameData*>(depth->frame.device.get());
    double zsum = 0;
    int zn = 0;
    if (dev && overlay_) {
        auto pts = overlay_->frame_points(*dev, r.T_world_camera, params_.frame_point_step, col);
        up.frame_buffer = std::move(pts.buffer);
        up.frame_buffer_count = pts.count;
        // Mean depth from a sparse in-place read of the shared buffer.
        const float* p = dev->points_xyzw();
        for (int y = 0; y < dev->height(); y += 8)
            for (int x = 0; x < dev->width(); x += 8) {
                const float z = p[4 * (static_cast<std::size_t>(y) * static_cast<std::size_t>(dev->width()) + static_cast<std::size_t>(x)) + 2];
                if (z > 0) {
                    zsum += z;
                    ++zn;
                }
            }
    } else {
        depth->frame.ensure_cpu();
        const Eigen::Matrix4f T = r.T_world_camera.matrix().cast<float>();
        const auto& pts = depth->frame.points;
        for (int y = 0; y < pts.height(); y += params_.frame_point_step)
            for (int x = 0; x < pts.width(); x += params_.frame_point_step) {
                const Vec3f& p = pts(x, y);
                if (p.z() <= 0) continue;
                zsum += p.z();
                ++zn;
                const Vec3f w = (T * p.homogeneous()).head<3>();
                const Vec3f n = T.topLeftCorner<3, 3>() * depth->frame.normals(x, y);
                up.frame_points.push_back({w.x(), w.y(), w.z(), n.x(), n.y(), n.z(), col});
            }
    }
    stats_.mean_depth_mm = zn ? static_cast<float>(zsum / zn) : 0.0f;

    if (r.integrated && model_clock_.elapsed_ms() > params_.model_refresh_s * 1000.0) {
        if (const auto* mv = dynamic_cast<const track_metal::MetalTsdfVolume*>(&tracker_.volume())) {
            auto rp = mv->extract_render_points();
            up.model_buffer = std::move(rp.buffer);
            up.model_buffer_count = rp.count;
            stats_.model_points = rp.count;
        } else {
            cache_.update(tracker_.volume());
            up.model = cache_.flatten();
            stats_.model_points = cache_.size();
        }
        up.model_changed = true;
        model_clock_.reset();
    }
    up.preview_left = std::move(depth->rectified_left);
    up.preview_right = std::move(depth->rectified_right);
    up.stats = stats_;
    sink_(std::move(up));
}

}  // namespace einstar::pipeline
