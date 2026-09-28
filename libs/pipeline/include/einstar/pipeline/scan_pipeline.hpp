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
#include "einstar/pipeline/stereo_frontend.hpp"
#include "einstar/render/types.hpp"
#include "einstar/track/tracker.hpp"
#include "einstar/usb/stream.hpp"

namespace einstar::pipeline {

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
};

struct LiveUpdate {
    std::uint64_t frame_id = 0;
    bool accepted = false;
    bool model_changed = false;
    std::vector<render::PointVertex> model;         // full model snapshot when model_changed
    std::vector<render::PointVertex> frame_points;  // current frame in world coordinates
    std::optional<SE3> pose;
    std::optional<SE3> last_good_pose;              // for the lost-tracking ghost
    bool tracking_lost = false;
    ImageU8 preview_left, preview_right;
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
};

class ScanPipeline {
public:
    using Sink = std::function<void(LiveUpdate&&)>;
    ScanPipeline(std::unique_ptr<StereoFrontend> frontend, ScanPipelineParams params, Sink sink);
    ~ScanPipeline();

    void start();
    void stop();
    void reset_model();

    // Called from the device thread; never blocks.
    void push(usb::FrameGroup&& group);

    [[nodiscard]] const track::Tracker& tracker() const { return tracker_; }
    [[nodiscard]] std::vector<SE3> trajectory() const;

private:
    void run(std::stop_token st);
    void process(usb::FrameGroup&& group);

    std::unique_ptr<StereoFrontend> frontend_;
    ScanPipelineParams params_;
    Sink sink_;
    track::Tracker tracker_;
    ModelPointCache cache_;

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
