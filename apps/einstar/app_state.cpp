#include "app_state.hpp"

#include <algorithm>
#include <format>

#include "einstar/session/session.hpp"

#include "einstar/core/log.hpp"

namespace einstar::app {
namespace {

constexpr render::Rgba8 kTrackedColor{90, 230, 120, 255};
constexpr render::Rgba8 kLostColor{235, 70, 70, 255};
constexpr render::Rgba8 kGhostColor{200, 200, 200, 255};
constexpr render::Rgba8 kTrailColor{255, 200, 60, 255};

}  // namespace

AppState::AppState() { connect(false); }

AppState::~AppState() {
    cancel_processing();
    if (process_thread_.joinable()) process_thread_.join();
    session_.reset();
}

void AppState::process_scan(const recon::ProcessParams& params) {
    if (!session_ || scanning()) return;
    {
        std::lock_guard lock(mutex_);
        if (process_.running) return;
        process_ = {};
        process_.running = true;
        process_.stage = "Saving the recording";
    }
    if (process_thread_.joinable()) process_thread_.join();
    cancel_ = false;
    process_thread_ = std::jthread([this, params = recon::ProcessParams(params)]() mutable {
        const auto path = session_->pipeline().flush_recording();
        auto fail = [&](const std::string& msg) {
            std::lock_guard lock(mutex_);
            process_.running = false;
            process_.summary = msg;
            log::error("process: {}", msg);
        };
        if (path.empty()) return fail("nothing has been recorded yet");
        auto reader = session::SessionReader::open(path);
        if (!reader) return fail(reader.error().message);
        params.cancel = &cancel_;
        params.progress = [this](const std::string& stage, double f) {
            std::lock_guard lock(mutex_);
            process_.stage = stage;
            process_.fraction = f;
        };
        auto r = recon::process_session(**reader, params);
        if (!r) return fail(r.error().message);
        const auto& rep = r->report;
        std::string times;
        const double total_s = rep.stage_ms.contains("total") ? rep.stage_ms.at("total") / 1000.0 : 0.0;
        auto summary = std::format("{} frames, {} loop closures, {} marker landmarks; poses moved {:.2f} mm (median), {:.1f} mm max; "
                                   "{} triangles; {:.1f} s",
                                   rep.frames_used, rep.loop_edges, rep.marker_landmarks, rep.median_correction_mm, rep.max_correction_mm,
                                   rep.triangles, total_s);
        if (rep.frames_excluded > 0) summary += std::format("; {} frames left out (inconsistent segment)", rep.frames_excluded);
        auto mesh = std::make_shared<recon::TriangleMesh>(std::move(r->mesh));
        std::vector<render::MeshVertex> verts(mesh->vertices.size());
        for (std::size_t i = 0; i < verts.size(); ++i) {
            const auto& p = mesh->vertices[i];
            const Vec3f n = mesh->normals.size() == verts.size() ? mesh->normals[i] : Vec3f::Zero();
            verts[i] = {p.x(), p.y(), p.z(), n.x(), n.y(), n.z()};
        }
        std::vector<std::uint32_t> idx;
        idx.reserve(mesh->triangles.size() * 3);
        for (const auto& t : mesh->triangles) idx.insert(idx.end(), t.begin(), t.end());
        std::lock_guard lock(mutex_);
        mesh_ = std::move(mesh);
        process_.running = false;
        process_.done = true;
        process_.summary = std::move(summary);
        if (!pending_) pending_.emplace();
        pending_->mesh = std::pair{std::move(verts), std::move(idx)};
        log::info("process: {}", process_.summary);
    });
}

void AppState::cancel_processing() { cancel_ = true; }

ProcessStatus AppState::process_status() const {
    std::lock_guard lock(mutex_);
    return process_;
}

bool AppState::export_mesh(const std::string& path) {
    std::shared_ptr<recon::TriangleMesh> mesh;
    {
        std::lock_guard lock(mutex_);
        mesh = mesh_;
    }
    if (!mesh) return false;
    auto r = recon::save_mesh(*mesh, path);
    std::lock_guard lock(mutex_);
    process_.summary = r ? std::format("Exported {} triangles to {}", mesh->triangles.size(), path) : r.error().message;
    return r.has_value();
}

void AppState::connect(bool emulator) {
    session_.reset();
    trail_.clear();
    auto s = Session::open(
        emulator, [this](pipeline::LiveUpdate&& u) { on_live_update(std::move(u)); },
        [this](int, device::ButtonAction a) {
            // Any single click on the scanner toggles scanning (handled on the UI thread).
            if (a == device::ButtonAction::single_click) toggle_requested_ = true;
        });
    if (!s) {
        error_ = s.error().message;
        log::error("connect failed: {}", error_);
        return;
    }
    error_.clear();
    session_ = std::move(*s);
    log::info("{}", session_->description());
}

std::string AppState::status() const {
    if (!session_) return error_.empty() ? "Not connected" : "Connection failed: " + error_;
    return session_->description();
}

void AppState::start_scan() {
    if (!session_) return;
    if (auto r = session_->start_scan(settings); !r) {
        error_ = r.error().message;
        log::error("start scan: {}", error_);
    }
}

void AppState::stop_scan() {
    if (session_) session_->stop_scan();
}

void AppState::toggle_scan() {
    if (scanning()) stop_scan();
    else start_scan();
}

void AppState::clear_model() {
    if (session_) session_->reset_model();
    std::lock_guard lock(mutex_);
    trail_.clear();
    if (!pending_) pending_.emplace();
    pending_->model = std::vector<render::PointVertex>{};
    pending_->mesh = std::pair<std::vector<render::MeshVertex>, std::vector<std::uint32_t>>{};
    mesh_.reset();
    process_ = {};
}

void AppState::apply_settings() {
    if (session_ && session_->scanning()) (void)session_->apply(settings);
}

void AppState::set_phase(pipeline::ScanPhase phase) {
    if (session_) session_->pipeline().set_phase(phase);
    std::lock_guard lock(mutex_);
    trail_.clear();
}

void AppState::set_align_mode(track::AlignMode mode) {
    if (session_) session_->pipeline().set_surface_mode(mode);
}

void AppState::optimize_global_markers() {
    if (!session_) return;
    {
        std::lock_guard lock(mutex_);
        global_status_ = "Optimising...";
    }
    session_->pipeline().optimize_global_markers([this](const pipeline::GlobalMarkerReport& r) {
        std::lock_guard lock(mutex_);
        global_status_ = r.error.empty()
                             ? std::format("{} markers from {} keyframes, reprojection {:.3f} px ({} outliers), max correction {:.2f} mm",
                                           r.markers, r.keyframes, r.bundle.rms_after_px, r.bundle.outliers_removed, r.max_shift_mm)
                             : "Not optimised: " + r.error;
    });
}

void AppState::clear_global_markers() {
    if (session_) session_->pipeline().clear_global_markers();
    std::lock_guard lock(mutex_);
    global_status_.clear();
}

bool AppState::save_global_markers(const std::string& path) {
    if (!session_) return false;
    const auto map = session_->pipeline().global_markers();
    const bool ok = !map.empty() && markers::save_markers(path, map);
    std::lock_guard lock(mutex_);
    global_status_ = ok ? std::format("Saved {} markers to {}", map.size(), path)
                        : map.empty() ? "Nothing to save: optimise a global-marker capture first" : "Could not write " + path;
    return ok;
}

bool AppState::load_global_markers(const std::string& path) {
    if (!session_) return false;
    auto map = markers::load_markers(path);
    std::lock_guard lock(mutex_);
    if (!map || map->empty()) {
        global_status_ = "Could not read a marker map from " + path;
        return false;
    }
    global_status_ = std::format("Loaded {} global markers", map->size());
    session_->pipeline().set_global_markers(std::move(*map));
    return true;
}

std::string AppState::global_marker_status() const {
    std::lock_guard lock(mutex_);
    return global_status_;
}

void AppState::update() {
    if (toggle_requested_.exchange(false)) toggle_scan();
    if (!session_ || housekeeping_.elapsed_ms() < 500) return;
    housekeeping_.reset();
    float depth;
    {
        std::lock_guard lock(mutex_);
        depth = last_depth_;
    }
    if (session_->scanning()) session_->set_distance_indication(depth);
    static int ticks = 0;
    if (++ticks % 4 == 0)
        if (auto t = session_->temperature()) {
            std::lock_guard lock(mutex_);
            hud_.temperature_c = static_cast<float>(*t);
        }
}

std::optional<RenderUpdate> AppState::take_render_update() {
    std::lock_guard lock(mutex_);
    return std::exchange(pending_, std::nullopt);
}

Hud AppState::hud() const {
    std::lock_guard lock(mutex_);
    return hud_;
}

void AppState::on_live_update(pipeline::LiveUpdate&& u) {
    RenderUpdate up;
    if (u.model_changed) {
        if (u.model_buffer) up.model_gpu = GpuPoints{std::move(u.model_buffer), u.model_buffer_count};
        else up.model = std::move(u.model);
    }
    up.frame_points = std::move(u.frame_points);
    if (u.frame_buffer) up.frame_gpu = GpuPoints{std::move(u.frame_buffer), u.frame_buffer_count};
    up.ir_left = std::move(u.preview_left);
    up.ir_right = std::move(u.preview_right);
    up.markers = std::move(u.markers);
    up.preview_markers = std::move(u.preview_markers);

    const float fx = 579.0f, fy = 579.0f, cx = 320.0f, cy = 256.0f;  // display frustum only
    std::lock_guard lock(mutex_);
    if (u.pose) {
        const Eigen::Matrix4f T = u.pose->matrix().cast<float>();
        up.scanner_pose = T;
        if (u.accepted) trail_.push_back(T.block<3, 1>(0, 3));
        render::append_polyline(up.lines, trail_, kTrailColor);
        render::append_frustum(up.lines, T, fx, fy, cx, cy, 640, 512, 80.0f, u.accepted ? kTrackedColor : kLostColor);
    }
    if (u.tracking_lost && u.last_good_pose) {
        // Ghost frustum: where the scanner needs to go back to for tracking to resume.
        render::append_frustum(up.lines, u.last_good_pose->matrix().cast<float>(), fx, fy, cx, cy, 640, 512, 120.0f, kGhostColor);
    }

    const auto& st = u.stats;
    hud_.tracking_lost = u.tracking_lost;
    hud_.reason = st.reason;
    hud_.fps = static_cast<float>(st.fps);
    hud_.frames = static_cast<int>(st.frames_processed);
    hud_.depth_ms = static_cast<float>(st.stereo_ms);
    hud_.track_ms = static_cast<float>(st.track_ms);
    hud_.queue_depth = st.queue_depth;
    hud_.dropped = static_cast<int>(st.dropped);
    hud_.distance_mm = st.mean_depth_mm;
    hud_.model_points = st.model_points;
    hud_.recorded_frames = st.recorded_frames;
    hud_.markers = st.markers_in_frame;
    hud_.markers_matched = st.markers_matched;
    hud_.map_markers = st.map_markers;
    hud_.global_markers = st.global_markers;
    hud_.keyframes = st.keyframes;
    hud_.marker_ms = static_cast<float>(st.marker_ms);
    hud_.phase = st.phase;
    // Working range 175..625 mm mapped onto the 10-step bar.
    hud_.distance_step = st.mean_depth_mm > 0 ? std::clamp(static_cast<int>((st.mean_depth_mm - 175.0f) / 45.0f), 0, 9) : -1;
    last_depth_ = st.mean_depth_mm;

    if (!pending_) {
        pending_ = std::move(up);
        return;
    }
    // UI hasn't consumed the previous update: keep the newest of everything.
    if (up.model) {
        pending_->model = std::move(up.model);
        pending_->model_gpu.reset();
    }
    if (up.model_gpu) {
        pending_->model_gpu = std::move(up.model_gpu);
        pending_->model.reset();
    }
    pending_->frame_points = std::move(up.frame_points);
    pending_->frame_gpu = std::move(up.frame_gpu);
    pending_->lines = std::move(up.lines);
    pending_->ir_left = std::move(up.ir_left);
    pending_->ir_right = std::move(up.ir_right);
    pending_->markers = std::move(up.markers);
    pending_->preview_markers = std::move(up.preview_markers);
    if (up.mesh) pending_->mesh = std::move(up.mesh);
    pending_->scanner_pose = up.scanner_pose;
}

}  // namespace einstar::app
