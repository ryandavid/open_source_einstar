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
#include "einstar/core/lasso.hpp"
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
    // An edit's update: only the model changed; keep everything else (the stale frame overlay goes).
    bool model_only = false;
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
    std::string temperature_note;  // why there is no temperature (e.g. the sensor reads nothing)
    float distance_mm = 0;
    int distance_step = -1;  // 0..9, -1 = out of range
    std::size_t model_points = 0;
    std::uint64_t recorded_frames = 0;
    std::uint64_t raw_frames = 0, raw_dropped = 0;
    std::string notice;  // short-lived message (e.g. a brightness change from the scanner's buttons)
};

// Paused-scan editing.
struct EditStatus {
    std::size_t undo_depth = 0;  // deletes that can still be undone
    bool busy = false;           // a delete or undo is running
    std::string message;         // the last one's outcome
};

struct ProcessStatus {
    bool running = false;
    bool done = false;         // a mesh is available
    std::string stage;
    double fraction = 0;
    std::string summary;       // report of the last run (or its error)
};

// A recording opened as a paused scan.
struct LoadStatus {
    bool loading = false;
    bool loaded = false;      // the live model is the recording's
    std::string path;
    std::size_t frames = 0, fused = 0;
    int markers = 0;
    bool resumable = false;   // scanning continues it (once the scanner is connected)
    std::string message;      // the outcome, or why it cannot be resumed
};

// What the scanner connection is, for the UI.
struct Connection {
    enum class Kind { none, scanner, emulator, recording } kind = Kind::none;  // recording: a loaded one, no scanner
    bool online = false;         // false while a real scanner is off the bus (it is reopened automatically)
    std::string device;          // product, serial, firmware
    std::string calibration;     // where the calibration came from
    std::string error;           // why the last connection attempt failed
};

class AppState {
public:
    AppState();
    ~AppState();

    // The real scanner (an error if none is attached) or the emulator. Nothing connects by itself.
    bool connect(bool emulator);
    void disconnect();
    [[nodiscard]] bool connected() const { return session_ != nullptr; }
    [[nodiscard]] Connection connection() const;
    [[nodiscard]] std::string status() const;
    // The scan file being recorded ("" until the first frame of a scan).
    [[nodiscard]] std::string recording_path() const;
    // Starts a new scan (new recording and world frame). `discard` deletes the current recording.
    void new_scan(bool discard);
    // Opens a recording (.estr) as a paused scan: its model rebuilt, to view, edit (a lasso delete is recorded in
    // it) and process; scanning continues it once the scanner it was made with is connected. Works without a
    // scanner. Runs in the background (load_status()).
    void open_recording(const std::string& path);
    [[nodiscard]] LoadStatus load_status() const;

    void start_scan();
    void stop_scan();
    void toggle_scan();
    void clear_model();
    [[nodiscard]] bool scanning() const { return session_ && session_->scanning(); }
    void apply_settings();
    void set_brightness(int level);  // exposure and gain from the brightness ladder
    // Also record raw IR images (for re-running future depth / marker algorithms on the scan).
    void set_record_raw_ir(bool on);
    [[nodiscard]] bool record_raw_ir() const;

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

    // Paused-scan editing (UI thread): a lasso selection built in the 3D view -- everything inside each
    // stroke, front to back -- is deleted from the live model and from the recording's frames so far
    // (rescanning the area brings it back). Deletes can be undone until scanning resumes.
    [[nodiscard]] bool can_edit() const;
    void add_lasso(LassoStroke stroke);
    void clear_selection();
    [[nodiscard]] const LassoSelection& selection() const { return selection_; }
    [[nodiscard]] std::uint64_t selection_version() const { return selection_version_; }
    void delete_selection();
    void undo_delete();
    [[nodiscard]] EditStatus edit_status() const;

    // Process step on the recorded scan (background thread).
    void process_scan(const recon::ProcessParams& params);
    void cancel_processing();
    [[nodiscard]] ProcessStatus process_status() const;
    bool export_mesh(const std::string& path);

    ScanSettings settings;
    bool follow_scanner = false;
    // Whether the scanner's start / pause button starts and pauses the scanner (the UI enables it only
    // in the steps where that is the next action).
    std::atomic<bool> scanner_button_enabled{true};

private:
    void on_live_update(pipeline::LiveUpdate&& u);

    std::unique_ptr<Session> session_;
    std::string error_;
    mutable std::mutex mutex_;
    std::optional<RenderUpdate> pending_;
    LassoSelection selection_;  // UI thread
    std::uint64_t selection_version_ = 0;
    EditStatus edit_;           // guarded by mutex_
    void on_edit(const pipeline::EditResult& r, bool undo);
    void on_loaded(pipeline::LoadResult&& r);
    LoadStatus load_;  // guarded by mutex_
    Hud hud_;
    std::vector<Eigen::Vector3f> trail_;
    std::atomic<bool> toggle_requested_{false};
    std::atomic<int> brightness_steps_{0};  // from the scanner's buttons, applied on the UI thread
    Stopwatch housekeeping_;
    Stopwatch notice_clock_;
    std::string notice_;
    float last_depth_ = 0;
    std::string global_status_;
    ProcessStatus process_;
    std::shared_ptr<recon::TriangleMesh> mesh_;
    std::atomic<bool> cancel_{false};
    std::jthread process_thread_;
};

}  // namespace einstar::app
