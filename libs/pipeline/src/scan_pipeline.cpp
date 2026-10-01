#include "einstar/pipeline/scan_pipeline.hpp"

#include "einstar/core/log.hpp"
#include "einstar/gpu/context.hpp"
#include "einstar/track_metal/metal_tsdf.hpp"
#include "gpu_overlay.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <format>
#include <map>
#include <future>
#include <set>

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
    : frontend_(std::move(frontend)), params_(params), sink_(std::move(sink)), tracker_(params.tracker, make_volume(params)),
      surface_mode_(params.tracker.mode), record_raw_ir_(params.record_raw_ir) {
    bundle_.geometry = frontend_->rectification().geometry;
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

void ScanPipeline::reset_model(bool discard) {
    discard_on_reset_ = discard;
    reset_requested_ = true;
}

std::string ScanPipeline::recording_path() const {
    std::lock_guard lock(recorder_mutex_);
    return recorder_ ? recorder_->path() : std::string{};
}

void ScanPipeline::post(std::function<void()> cmd) {
    {
        std::lock_guard lock(command_mutex_);
        commands_.push_back({frames_in_.load(), std::move(cmd)});
    }
    cv_.notify_one();
    if (!worker_.joinable()) run_commands();  // not running: execute inline
}

void ScanPipeline::run_commands() {
    std::vector<std::function<void()>> cmds;
    {
        std::lock_guard lock(command_mutex_);
        const auto taken = frames_taken_.load();
        auto it = commands_.begin();
        while (it != commands_.end() && it->after_frames <= taken) cmds.push_back(std::move((it++)->fn));
        commands_.erase(commands_.begin(), it);
    }
    for (auto& c : cmds) c();
}

void ScanPipeline::apply_phase() {
    auto& tp = tracker_.params();
    if (phase_ == ScanPhase::global_markers) {
        tp.mode = track::AlignMode::markers;
        tp.fuse_surface = false;
        tp.global_relocalization = false;  // no surface model to register against
    } else {
        tp.mode = surface_mode_ == track::AlignMode::geometry && !global_map_.empty() ? track::AlignMode::hybrid : surface_mode_;
        tp.fuse_surface = true;
        tp.global_relocalization = params_.tracker.global_relocalization;
    }
}

void ScanPipeline::set_phase(ScanPhase phase) {
    post([this, phase] {
        if (phase == phase_) return;
        phase_ = phase;
        // Either way the tracker restarts: a fresh constellation capture continues on the current
        // (running-mean) marker map; a surface scan starts on the fixed global map if there is one.
        if (phase == ScanPhase::global_markers) {
            tracker_.reset(true);
            if (tracker_.marker_map().empty() && bundle_.observations.empty()) restart_recording(false);  // new world frame
            if (!bundle_.observations.empty()) {
                // Resume a capture: the map the keyframes refer to must be kept.
                std::vector<markers::MapMarker> m;
                for (const auto& [id, p] : bundle_.markers) m.push_back({id, p, 6.0, 1, false});
                tracker_.set_marker_map(std::move(m));
            }
        } else {
            tracker_.reset(true);
        }
        cache_.clear();
        last_keyframe_.reset();
        apply_phase();
        std::lock_guard lock(trajectory_mutex_);
        trajectory_.clear();
        log::info("scan phase: {}", phase == ScanPhase::global_markers ? "global markers" : "surface");
    });
}

void ScanPipeline::collect_keyframe(const track::TrackResult& r, const track::DepthFrame& frame) {
    if (!r.accepted || static_cast<int>(r.marker_ids.size()) < params_.keyframe_min_markers) return;
    if (static_cast<int>(bundle_.T_world_camera.size()) >= params_.max_keyframes) return;
    if (last_keyframe_) {
        const SE3 d = last_keyframe_->inverse() * r.T_world_camera;
        const double rot = Eigen::AngleAxisd(d.linear()).angle() * 180.0 / M_PI;
        if (translation_norm(d) < params_.keyframe_translation_mm && rot < params_.keyframe_rotation_deg) return;
    }
    const int kf = bundle_.T_world_camera.empty() ? 0 : bundle_.T_world_camera.rbegin()->first + 1;
    bundle_.T_world_camera[kf] = r.T_world_camera;
    const auto& map = tracker_.marker_map();
    for (const auto& [fi, id] : r.marker_ids) {
        const auto& m = frame.markers[static_cast<std::size_t>(fi)];
        if (m.left_rect.x() < 0) continue;
        bundle_.observations.push_back({kf, id, m.left_rect, m.right_rect});
    }
    // Current running-mean positions as the initial guess.
    for (const auto& m : map.markers()) bundle_.markers[m.id] = m.position;
    last_keyframe_ = r.T_world_camera;
}

GlobalMarkerReport ScanPipeline::run_bundle_adjustment() {
    GlobalMarkerReport rep;
    rep.keyframes = static_cast<int>(bundle_.T_world_camera.size());
    if (rep.keyframes < 2) {
        rep.error = "capture markers from at least two positions first";
        return rep;
    }
    // Only markers seen from enough keyframes are adjusted (and kept).
    std::map<int, std::set<int>> seen;
    for (const auto& o : bundle_.observations) seen[o.marker].insert(o.frame);
    optim::MarkerBundle b;
    b.geometry = bundle_.geometry;
    b.T_world_camera = bundle_.T_world_camera;
    for (const auto& o : bundle_.observations)
        if (static_cast<int>(seen[o.marker].size()) >= params_.global_marker_min_keyframes && bundle_.markers.contains(o.marker)) {
            b.observations.push_back(o);
            b.markers[o.marker] = bundle_.markers.at(o.marker);
        }
    if (b.markers.size() < 3) {
        rep.error = "fewer than 3 markers were seen from two or more positions";
        return rep;
    }
    const auto before = b.markers;
    rep.bundle = optim::optimize(b, params_.bundle);
    // Markers that lost all their observations as outliers are dropped.
    std::map<int, std::set<int>> kept;
    for (const auto& o : b.observations) kept[o.marker].insert(o.frame);
    std::map<int, double> diam;
    for (const auto& m : tracker_.marker_map().markers()) diam[m.id] = m.diameter;
    std::vector<markers::MapMarker> map;
    for (const auto& [id, p] : b.markers) {
        if (static_cast<int>(kept[id].size()) < params_.global_marker_min_keyframes) continue;
        rep.max_shift_mm = std::max(rep.max_shift_mm, (p - before.at(id)).norm());
        map.push_back({id, p, diam.contains(id) ? diam[id] : 6.0, static_cast<int>(kept[id].size()), true});
    }
    rep.markers = static_cast<int>(map.size());
    // Keep the refined state so a later capture can extend and re-optimise it.
    bundle_.T_world_camera = b.T_world_camera;
    for (const auto& [id, p] : b.markers) bundle_.markers[id] = p;
    {
        std::lock_guard lock(global_mutex_);
        global_map_ = map;
    }
    {
        std::lock_guard rlock(recorder_mutex_);
        if (recorder_) recorder_->write_global_markers(map);
    }
    tracker_.set_marker_map(std::move(map));
    log::info("global markers: {} keyframes, {} markers, reprojection rms {:.3f} -> {:.3f} px, {} outliers, max shift {:.3f} mm",
              rep.keyframes, rep.markers, rep.bundle.rms_before_px, rep.bundle.rms_after_px, rep.bundle.outliers_removed, rep.max_shift_mm);
    return rep;
}

void ScanPipeline::optimize_global_markers(std::function<void(const GlobalMarkerReport&)> done) {
    post([this, done = std::move(done)] {
        const auto rep = run_bundle_adjustment();
        if (rep.error.empty()) {
            // Tracking restarts on the optimised map (relocalising against it on the next frame).
            tracker_.reset(true);
            last_keyframe_.reset();
            apply_phase();
        } else {
            log::warn("global markers: {}", rep.error);
        }
        if (done) done(rep);
    });
}

void ScanPipeline::clear_global_markers() {
    post([this] {
        bundle_ = {};
        bundle_.geometry = frontend_->rectification().geometry;
        last_keyframe_.reset();
        {
            std::lock_guard lock(global_mutex_);
            global_map_.clear();
        }
        tracker_.reset(false);
        restart_recording(false);
        apply_phase();
    });
}

void ScanPipeline::set_global_markers(std::vector<markers::MapMarker> map) {
    post([this, map = std::move(map)]() mutable {
        for (auto& m : map) m.fixed = true;
        {
            std::lock_guard lock(global_mutex_);
            global_map_ = map;
        }
        tracker_.reset(false);
        tracker_.set_marker_map(std::move(map));
        restart_recording(false);  // the loaded map defines a new world frame
        apply_phase();
    });
}

void ScanPipeline::set_recording_directory(std::string directory) {
    post([this, directory = std::move(directory)] {
        {
            std::lock_guard lock(recorder_mutex_);
            if (directory == record_dir_) return;
        }
        restart_recording(false);
        std::lock_guard lock(recorder_mutex_);  // push() reads it on the device thread
        record_dir_ = directory;
    });
}

std::string ScanPipeline::flush_recording() {
    // Ordered with frames (everything pushed before this call is recorded), then drained to disk.
    std::promise<std::string> done;
    auto fut = done.get_future();
    post([this, &done] {
        std::lock_guard lock(recorder_mutex_);
        if (recorder_) recorder_->flush();
        done.set_value(recorder_ ? recorder_->path() : std::string{});
    });
    return fut.get();
}

void ScanPipeline::restart_recording(bool delete_current) {
    std::lock_guard lock(recorder_mutex_);
    if (!recorder_) return;
    const auto path = recorder_->path();
    const auto frames = recorder_->frames_written();
    recorder_->close();
    recorder_.reset();
    std::error_code ec;
    if (delete_current || frames == 0) {
        std::filesystem::remove(path, ec);
        log::info("recording: discarded {}", path);
    } else {
        log::info("recording: closed {} ({} frames)", path, frames);
    }
    stats_.recorded_frames = 0;
    stats_.raw_frames = 0;
    raw_dropped_ = 0;
}

session::SessionWriter* ScanPipeline::ensure_recorder() {
    if (record_dir_.empty()) return nullptr;
    if (recorder_) return recorder_.get();
    const auto now = std::chrono::system_clock::now();
    const auto path = (std::filesystem::path(record_dir_) /
                       std::format("scan-{:%Y%m%d-%H%M%S}.estr", std::chrono::floor<std::chrono::seconds>(now)))
                          .string();
    session::SessionHeader h;
    h.depth_intrinsics = frontend_->depth_intrinsics();
    const auto& g = frontend_->rectification().geometry;
    h.rect_f = g.f;
    h.rect_cx = g.cx;
    h.rect_cy = g.cy;
    h.baseline_mm = g.baseline;
    h.description = "live scan";
    auto w = session::SessionWriter::create(path, h);
    if (!w) {
        log::error("recording disabled: {}", w.error().message);
        record_dir_.clear();
        return nullptr;
    }
    recorder_ = std::move(*w);
    if (device_record_) recorder_->write_device(*device_record_);
    std::lock_guard glock(global_mutex_);
    if (!global_map_.empty()) recorder_->write_global_markers(global_map_);
    log::info("recording: {}", path);
    return recorder_.get();
}

void ScanPipeline::set_device_record(session::DeviceRecord device) {
    std::lock_guard lock(recorder_mutex_);
    if (recorder_) recorder_->write_device(device);
    device_record_ = std::move(device);
}

void ScanPipeline::set_capture_settings(const session::CaptureSettings& settings) {
    std::lock_guard lock(recorder_mutex_);
    const float t = capture_.temperature_c;
    capture_ = settings;
    capture_.temperature_c = t;
}

void ScanPipeline::set_temperature(float celsius) {
    std::lock_guard lock(recorder_mutex_);
    capture_.temperature_c = celsius;
}

void ScanPipeline::record_dropped(const usb::FrameGroup& group, std::string reason) {
    std::lock_guard lock(recorder_mutex_);
    if (auto* w = ensure_recorder()) w->write_dropped({group.frame_id, static_cast<double>(group.timestamp) * 1e-6, std::move(reason)});
}

void ScanPipeline::record(const track::TrackResult& r, const DepthOutput& depth) {
    std::lock_guard lock(recorder_mutex_);
    if (!ensure_recorder()) return;
    session::FrameRecord rec;
    rec.index = depth.frame.index;
    rec.timestamp_s = depth.frame.timestamp_s;
    rec.flags = (r.accepted ? session::frame_accepted : 0u) | (r.degenerate ? session::frame_degenerate : 0u) |
                (r.relocalized ? session::frame_relocalized : 0u) | (r.marker_pose ? session::frame_marker_pose : 0u) |
                (r.integrated ? session::frame_integrated : 0u) |
                (phase_ == ScanPhase::global_markers ? session::frame_global_marker_capture : 0u);
    rec.T_world_camera = r.T_world_camera;
    std::map<int, int> ids(r.marker_ids.begin(), r.marker_ids.end());
    for (std::size_t m = 0; m < depth.frame.markers.size(); ++m) {
        const auto& mk = depth.frame.markers[m];
        const auto it = ids.find(static_cast<int>(m));
        rec.markers.push_back({mk.position, mk.normal, mk.diameter, it == ids.end() ? -1 : it->second, mk.left_rect, mk.right_rect});
    }
    const auto* dev = dynamic_cast<const gpu::MetalFrameData*>(depth.frame.device.get());
    if (dev && !packer_)
        if (auto ctx = gpu::Context::create())
            if (auto pk = depth_metal::DepthPacker::create(*ctx)) packer_ = std::move(*pk);
    if (dev && packer_) {
        // Quantisation and delta coding on the GPU; the recording thread waits for it and compresses.
        auto packed = packer_->pack(std::dynamic_pointer_cast<const gpu::MetalFrameData>(depth.frame.device));
        auto pi = std::make_shared<session::PackedImages>();
        pi->width = packed->width;
        pi->height = packed->height;
        pi->bytes = packed->bytes();
        pi->ready = [packed] { packed->wait(); };
        pi->owner = packed;
        rec.packed = std::move(pi);
    } else {
        session::capture_depth(depth.frame, rec.depth, rec.confidence);
    }
    session::FrameExtras ex;
    ex.left_sensor = frontend_->left_sensor();
    ex.capture = capture_;
    auto& d = ex.tracking;
    d.state = static_cast<std::uint8_t>(r.state);
    d.icp_rms_mm = static_cast<float>(r.icp.rms_mm);
    d.inlier_ratio = static_cast<float>(r.icp.inlier_ratio);
    d.coverage = static_cast<float>(r.icp.coverage);
    d.eigen_ratio = static_cast<float>(r.icp.min_eigenvalue_ratio);
    d.marker_rms_mm = static_cast<float>(r.icp.marker_rms_mm);
    d.correspondences = r.icp.correspondences;
    d.degenerate_directions = r.icp.degenerate_directions;
    d.markers_seen = r.markers_seen;
    d.stereo_ms = static_cast<float>(depth.stereo_ms);
    d.track_ms = static_cast<float>(r.ms);
    d.reason = r.reason;
    rec.extras = std::move(ex);
    recorder_->write(std::move(rec));
    stats_.recorded_frames = recorder_->frames_written() + recorder_->backlog();
    stats_.raw_frames = recorder_->raw_frames_written();
    stats_.raw_dropped = raw_dropped_;
}

void ScanPipeline::set_surface_mode(track::AlignMode mode) {
    post([this, mode] {
        surface_mode_ = mode;
        apply_phase();
    });
}

std::vector<markers::MapMarker> ScanPipeline::global_markers() const {
    std::lock_guard lock(global_mutex_);
    return global_map_;
}

void ScanPipeline::fill_marker_overlays(const track::TrackResult& r, const DepthOutput& depth, LiveUpdate& up) const {
    using render::MarkerState;
    const auto& frame = depth.frame;
    const Eigen::Matrix4f T = r.T_world_camera.matrix().cast<float>();
    std::vector<bool> matched(frame.markers.size(), false);
    for (const auto& [fi, id] : r.marker_ids) matched[static_cast<std::size_t>(fi)] = true;
    for (const auto& m : tracker_.marker_map().markers()) {
        if (!tracker_.marker_map().confirmed(m)) continue;
        const Vec3f p = m.position.cast<float>();
        // Map markers carry no normal; drawn as camera-facing discs by the renderer when n = 0.
        up.markers.push_back({p.x(), p.y(), p.z(), 0, 0, 0, static_cast<float>(0.5 * m.diameter),
                              render::marker_color(m.fixed ? MarkerState::global_fixed : MarkerState::in_map)});
    }
    if (r.accepted)
        for (std::size_t i = 0; i < frame.markers.size(); ++i) {
            const auto& m = frame.markers[i];
            const Vec3f p = (T * m.position.cast<float>().homogeneous()).head<3>();
            const Vec3f n = T.topLeftCorner<3, 3>() * m.normal.cast<float>();
            // Slightly larger so the current frame's discs ring the map discs.
            up.markers.push_back({p.x(), p.y(), p.z(), n.x(), n.y(), n.z(), static_cast<float>(0.5 * m.diameter * 1.3),
                                  render::marker_color(matched[i] ? MarkerState::in_frame : MarkerState::rejected)});
        }
    const double f_half = 0.5 * frontend_->rectification().geometry.f;
    for (const auto& m : depth.markers) {
        const double z = m.position.z();
        up.preview_markers.push_back({static_cast<float>(0.5 * m.left_rect.x()), static_cast<float>(0.5 * m.left_rect.y()),
                                      static_cast<float>(0.5 * m.diameter * f_half / z), true});
    }
    for (const auto& c : depth.unmatched_left)
        up.preview_markers.push_back({static_cast<float>(0.5 * c.x()), static_cast<float>(0.5 * c.y()), 4.0f, false});
}

void ScanPipeline::push(usb::FrameGroup&& group) {
    ++frames_in_;
    if (record_raw_ir_) {
        // Raw IR is recorded on arrival, so frames the live pipeline drops are still kept.
        session::RawFrame raw;
        raw.index = group.frame_id;
        raw.timestamp_s = static_cast<double>(group.timestamp) * 1e-6;
        for (int sensor = 0; sensor < 2; ++sensor)
            if (const auto& sf = group.sensors[static_cast<std::size_t>(sensor)]) raw.images.emplace_back(sensor, sf->pixels);
        std::lock_guard lock(recorder_mutex_);
        if (auto* w = ensure_recorder(); w && !w->write_raw(std::move(raw))) {
            if (raw_dropped_++ % 50 == 0) log::warn("recording: disk too slow for raw IR, {} raw frames not written", raw_dropped_.load());
        }
    }
    std::optional<usb::FrameGroup> overflow;
    {
        std::unique_lock lock(mutex_);
        if (params_.block_when_full) {
            space_cv_.wait(lock, [&] { return queue_.size() < params_.queue_capacity || !worker_.joinable(); });
        } else if (queue_.size() >= params_.queue_capacity) {
            overflow = std::move(queue_.front());
            queue_.pop_front();
            ++dropped_;
            ++frames_taken_;
        }
        queue_.push_back(std::move(group));
    }
    cv_.notify_one();
    if (overflow) record_dropped(*overflow, "live queue full");
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
            cv_.wait_for(lock, std::chrono::milliseconds(100), [&] {
                std::lock_guard cl(command_mutex_);
                return !queue_.empty() || (!commands_.empty() && commands_.front().after_frames <= frames_taken_) || st.stop_requested();
            });
        }
        run_commands();
        {
            std::unique_lock lock(mutex_);
            if (queue_.empty()) continue;
            group = std::move(queue_.front());
            queue_.pop_front();
            ++frames_taken_;
            stats_.queue_depth = static_cast<int>(queue_.size());
        }
        space_cv_.notify_one();
        process(std::move(group));
        run_commands();
    }
}

void ScanPipeline::process(usb::FrameGroup&& group) {
    if (reset_requested_.exchange(false)) {
        // Clearing the model keeps an optimised global-marker map (it describes the scene, not the scan).
        tracker_.reset(true);
        restart_recording(discard_on_reset_.load());
        if (phase_ == ScanPhase::global_markers) {
            bundle_ = {};
            bundle_.geometry = frontend_->rectification().geometry;
            last_keyframe_.reset();
        }
        cache_.clear();
        std::lock_guard lock(trajectory_mutex_);
        trajectory_.clear();
    }
    if (!order_detected_) {
        frontend_->detect_sensor_order(group);
        order_detected_ = true;
    }
    auto depth = frontend_->process(group);
    if (!depth) {
        record_dropped(group, "no depth (incomplete image group)");
        return;
    }
    stereo_times_.add(depth->stereo_ms);
    marker_times_.add(depth->marker_ms);

    const auto r = tracker_.process(depth->frame);
    if (r.restarted) {
        // The tracker started over in a new world frame (unless a global marker map fixes it): the
        // frames recorded so far belong to the abandoned one, so they go in their own file.
        const bool fixed = std::ranges::any_of(tracker_.marker_map().markers(), [](const auto& m) { return m.fixed; });
        if (!fixed) restart_recording(false);
        std::lock_guard lock(trajectory_mutex_);
        trajectory_.clear();
    }
    if (phase_ == ScanPhase::global_markers) collect_keyframe(r, depth->frame);
    record(r, *depth);
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
    stats_.marker_ms = marker_times_.median();
    stats_.phase = phase_;
    stats_.markers_in_frame = r.markers_seen;
    stats_.markers_matched = static_cast<int>(r.marker_ids.size());
    stats_.map_markers = static_cast<int>(tracker_.marker_map().confirmed_count());
    stats_.keyframes = static_cast<int>(bundle_.T_world_camera.size());
    {
        std::lock_guard lock(global_mutex_);
        stats_.global_markers = static_cast<int>(global_map_.size());
    }
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
    fill_marker_overlays(r, *depth, up);
    up.preview_left = std::move(depth->rectified_left);
    up.preview_right = std::move(depth->rectified_right);
    up.preview_left_tex = std::move(depth->preview_left);
    up.preview_right_tex = std::move(depth->preview_right);
    up.stats = stats_;
    sink_(std::move(up));
}

}  // namespace einstar::pipeline
