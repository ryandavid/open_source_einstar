#pragma once

// Live scan pipeline: device frame groups -> stereo depth -> tracking -> TSDF fusion -> live view
// updates. Runs on its own worker thread with a bounded input queue whose overflows are counted
// and reported (never silent).

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

#include "einstar/core/timing.hpp"
#include "einstar/optim/marker_bundle.hpp"
#include "einstar/pipeline/stereo_frontend.hpp"
#include "einstar/render/types.hpp"
#include "einstar/track/tracker.hpp"
#include "einstar/gpu/device_data.hpp"
#include "einstar/track_metal/metal_icp.hpp"
#include "einstar/usb/stream.hpp"

namespace einstar::pipeline {

// surface: normal scanning (tracking + fusion). global_markers: markers-only capture of the marker
// constellation (nothing fused), keyframes collected for bundle adjustment; the optimised map then
// anchors every later surface scan (EXStar's "global markers").
enum class ScanPhase { surface, global_markers };

struct GlobalMarkerReport {
    int keyframes = 0;
    int markers = 0;             // fixed markers in the resulting map
    optim::BundleReport bundle;
    double max_shift_mm = 0;     // largest marker correction vs. the running-mean map
    std::string error;
};

struct LiveStats {
    std::uint64_t frames_in = 0;
    std::uint64_t frames_processed = 0;
    std::uint64_t dropped = 0;          // input queue overflow (pipeline slower than the scanner)
    int queue_depth = 0;
    double fps = 0;
    double stereo_ms = 0;               // median
    double track_ms = 0;                // median
    track::TrackState state = track::TrackState::initializing;
    std::string reason;                 // last rejection reason
    int accepted = 0, lost = 0, relocalized = 0;
    float mean_depth_mm = 0;
    std::size_t model_points = 0;
    ScanPhase phase = ScanPhase::surface;
    int markers_in_frame = 0;
    int markers_matched = 0;
    int map_markers = 0;
    int global_markers = 0;      // fixed markers of the optimised map in use
    int keyframes = 0;           // global-marker keyframes collected
    double marker_ms = 0;        // median
};

struct LiveUpdate {
    std::uint64_t frame_id = 0;
    bool accepted = false;
    bool model_changed = false;
    std::vector<render::PointVertex> model;         // full model snapshot when model_changed (CPU path)
    std::vector<render::PointVertex> frame_points;  // current frame in world coordinates (CPU path)
    // GPU path: the same data as GPU buffers of render::PointVertex, drawn without copies.
    gpu::Ref<MTL::Buffer> model_buffer;
    std::size_t model_buffer_count = 0;
    gpu::Ref<MTL::Buffer> frame_buffer;
    std::size_t frame_buffer_count = 0;
    std::optional<SE3> pose;
    std::optional<SE3> last_good_pose;              // for the lost-tracking ghost
    bool tracking_lost = false;
    ImageU8 preview_left, preview_right;
    // Marker overlays: world discs (map + current frame) and preview-image ellipses (half-res rectified px).
    std::vector<render::MarkerInstance> markers;
    struct PreviewMarker {
        float x, y, radius;
        bool matched;
    };
    std::vector<PreviewMarker> preview_markers;
    LiveStats stats;
};

// Keeps extracted surface points per brick so only changed bricks are re-extracted.
class ModelPointCache {
public:
    void update(const track::Volume& volume);
    [[nodiscard]] std::vector<render::PointVertex> flatten() const;
    [[nodiscard]] std::size_t size() const { return total_; }
    void clear();

private:
    struct Key {
        std::size_t operator()(const track::BrickCoord& c) const { return track::BrickCoordHash{}(c); }
    };
    std::unordered_map<track::BrickCoord, std::vector<render::PointVertex>, Key> bricks_;
    std::uint32_t last_frame_ = 0;
    std::size_t total_ = 0;
    bool use_full_ = false;
    std::vector<render::PointVertex> full_;
};

struct ScanPipelineParams {
    track::TrackerParams tracker;
    std::size_t queue_capacity = 6;
    // Live USB input must never stall, so overflow drops the oldest group (and counts it). Replays,
    // the emulator and tests can instead block the producer so every frame is processed.
    bool block_when_full = false;
    double model_refresh_s = 0.25;   // how often the full model snapshot is re-published
    int frame_point_step = 2;        // subsampling of the current-frame overlay
    bool gpu_volume = true;          // Metal TSDF when available, CPU reference otherwise
    // Global-marker keyframes: a new one after this much motion, with enough identified markers.
    double keyframe_translation_mm = 25.0;
    double keyframe_rotation_deg = 10.0;
    int keyframe_min_markers = 4;
    int max_keyframes = 2000;
    int global_marker_min_keyframes = 2;  // a marker must be seen from this many keyframes to be kept
    optim::BundleParams bundle;
};

class ScanPipeline {
public:
    using Sink = std::function<void(LiveUpdate&&)>;
    ScanPipeline(std::unique_ptr<StereoFrontend> frontend, ScanPipelineParams params, Sink sink);
    ~ScanPipeline();

    void start();
    void stop();
    void reset_model();

    // Global markers. Commands run on the worker between frames (thread-safe to call from the UI).
    void set_phase(ScanPhase phase);
    // Bundle-adjusts the collected keyframes and installs the result as a fixed map; the report is
    // delivered through `done` on the worker thread.
    void optimize_global_markers(std::function<void(const GlobalMarkerReport&)> done = {});
    void clear_global_markers();
    void set_global_markers(std::vector<markers::MapMarker> map);  // e.g. loaded from disk
    // Alignment used while scanning surfaces (geometry falls back to hybrid while a global map is set).
    void set_surface_mode(track::AlignMode mode);
    [[nodiscard]] std::vector<markers::MapMarker> global_markers() const;

    // Called from the device thread; never blocks.
    void push(usb::FrameGroup&& group);

    [[nodiscard]] const track::Tracker& tracker() const { return tracker_; }
    [[nodiscard]] std::vector<SE3> trajectory() const;

private:
    void run(std::stop_token st);
    void process(usb::FrameGroup&& group);
    void run_commands();
    void post(std::function<void()> cmd);
    void apply_phase();
    void collect_keyframe(const track::TrackResult& r, const track::DepthFrame& frame);
    GlobalMarkerReport run_bundle_adjustment();
    void fill_marker_overlays(const track::TrackResult& r, const DepthOutput& depth, LiveUpdate& up) const;

    std::unique_ptr<StereoFrontend> frontend_;
    ScanPipelineParams params_;
    Sink sink_;
    track::Tracker tracker_;
    std::unique_ptr<track_metal::MetalIcp> gpu_icp_;
    std::unique_ptr<class GpuOverlay> overlay_;
    ModelPointCache cache_;

    ScanPhase phase_ = ScanPhase::surface;
    track::AlignMode surface_mode_;
    optim::MarkerBundle bundle_;     // global-marker keyframes and observations
    std::optional<SE3> last_keyframe_;
    std::vector<markers::MapMarker> global_map_;
    mutable std::mutex global_mutex_;
    std::mutex command_mutex_;
    // Commands run in order with frames: each waits until the frames pushed before it were taken.
    struct Command {
        std::uint64_t after_frames;
        std::function<void()> fn;
    };
    std::vector<Command> commands_;
    std::atomic<std::uint64_t> frames_taken_{0};
    TimingStats marker_times_{64};

    std::mutex mutex_;
    std::condition_variable cv_;
    std::condition_variable space_cv_;
    std::deque<usb::FrameGroup> queue_;
    std::atomic<std::uint64_t> frames_in_{0}, dropped_{0};
    std::atomic<bool> reset_requested_{false};
    bool order_detected_ = false;

    LiveStats stats_;
    TimingStats stereo_times_{64}, track_times_{64};
    Stopwatch fps_clock_, model_clock_;
    std::uint64_t fps_frames_ = 0;
    mutable std::mutex trajectory_mutex_;
    std::vector<SE3> trajectory_;
    std::jthread worker_;
};

}  // namespace einstar::pipeline
