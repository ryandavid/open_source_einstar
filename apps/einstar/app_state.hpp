#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include "einstar/core/image.hpp"
#include "einstar/recon/process.hpp"
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
    ImageU8 ir_left, ir_right;                        // CPU backend
    gpu::Ref<MTL::Texture> ir_left_tex, ir_right_tex;  // GPU backend
    std::vector<pipeline::LiveUpdate::PreviewMarker> preview_markers;  // on the left preview (half-res px)
    std::optional<Eigen::Matrix4f> scanner_pose;
    // Processed mesh (full replacement when present; an empty mesh clears it).
    std::optional<std::pair<std::vector<render::MeshVertex>, std::vector<std::uint32_t>>> mesh;
};

struct Hud {
    bool tracking_lost = false;
    std::string reason;
    float fps = 0;
    int frames = 0;
    int markers = 0;           // stereo markers in the current frame
    int markers_matched = 0;   // of those, identified in the map
    int map_markers = 0;
    int global_markers = 0;
    int keyframes = 0;
    float marker_ms = 0;
    pipeline::ScanPhase phase = pipeline::ScanPhase::surface;
    float point_distance_mm = 0.5f;
    float depth_ms = 0;
    float track_ms = 0;
    int queue_depth = 0;
    int dropped = 0;
    float temperature_c = -273.0f;
    float distance_mm = 0;
    int distance_step = -1;  // 0..9, -1 = out of range
    std::size_t model_points = 0;
    std::uint64_t recorded_frames = 0;
};

struct ProcessStatus {
    bool running = false;
    bool done = false;         // a mesh is available
    std::string stage;
    double fraction = 0;
    std::string summary;       // report of the last run (or its error)
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

    // Global markers / alignment.
    void set_phase(pipeline::ScanPhase phase);
    void set_align_mode(track::AlignMode mode);
    void optimize_global_markers();
    void clear_global_markers();
    bool save_global_markers(const std::string& path);
    bool load_global_markers(const std::string& path);
    [[nodiscard]] std::string global_marker_status() const;

    // Process step on the recorded scan (background thread).
    void process_scan(const recon::ProcessParams& params);
    void cancel_processing();
    [[nodiscard]] ProcessStatus process_status() const;
    bool export_mesh(const std::string& path);

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
    std::string global_status_;
    ProcessStatus process_;
    std::shared_ptr<recon::TriangleMesh> mesh_;
    std::atomic<bool> cancel_{false};
    std::jthread process_thread_;
};

}  // namespace einstar::app
