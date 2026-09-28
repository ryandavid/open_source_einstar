#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "einstar/core/image.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/render/overlays.hpp"
#include "einstar/render/types.hpp"
#include "session.hpp"

namespace einstar::app {

// Everything the render thread needs from the pipeline since the last UI frame.
struct GpuPoints {
    gpu::Ref<MTL::Buffer> buffer;  // render::PointVertex records
    std::size_t count = 0;
};

struct RenderUpdate {
    std::optional<std::vector<render::PointVertex>> model;  // full replacement when present (CPU path)
    std::optional<GpuPoints> model_gpu;                     // full replacement when present (GPU path)
    std::vector<render::PointVertex> frame_points;
    std::optional<GpuPoints> frame_gpu;
    std::vector<render::MarkerInstance> markers;
    std::vector<render::LineVertex> lines;
    ImageU8 ir_left, ir_right;
    std::optional<Eigen::Matrix4f> scanner_pose;
};

struct Hud {
    bool tracking_lost = false;
    std::string reason;
    float fps = 0;
    int frames = 0;
    int markers = 0;
    float point_distance_mm = 0.5f;
    float depth_ms = 0;
    float track_ms = 0;
    int queue_depth = 0;
    int dropped = 0;
    float temperature_c = -273.0f;
    float distance_mm = 0;
    int distance_step = -1;  // 0..9, -1 = out of range
    std::size_t model_points = 0;
};

class AppState {
public:
    AppState();
    ~AppState();

    void connect(bool emulator);
    [[nodiscard]] bool connected() const { return session_ != nullptr; }
    [[nodiscard]] std::string status() const;

    void start_scan();
    void stop_scan();
    void toggle_scan();
    void clear_model();
    [[nodiscard]] bool scanning() const { return session_ && session_->scanning(); }
    void apply_settings();

    void update();  // periodic housekeeping from the UI thread (temperature, LEDs)
    [[nodiscard]] std::optional<RenderUpdate> take_render_update();
    [[nodiscard]] Hud hud() const;

    ScanSettings settings;
    bool follow_scanner = false;

private:
    void on_live_update(pipeline::LiveUpdate&& u);

    std::unique_ptr<Session> session_;
    std::string error_;
    mutable std::mutex mutex_;
    std::optional<RenderUpdate> pending_;
    Hud hud_;
    std::vector<Eigen::Vector3f> trail_;
    std::atomic<bool> toggle_requested_{false};
    Stopwatch housekeeping_;
    float last_depth_ = 0;
};

}  // namespace einstar::app
