#include "app_state.hpp"

#include <algorithm>
#include <filesystem>
#include <format>

#include "einstar/session/session.hpp"
#include "einstar/synth/demo.hpp"

#include "einstar/core/log.hpp"

namespace einstar::app {
namespace {

constexpr render::Rgba8 kTrackedColor{90, 230, 120, 255};
constexpr render::Rgba8 kLostColor{235, 70, 70, 255};
constexpr render::Rgba8 kGhostColor{200, 200, 200, 255};
constexpr render::Rgba8 kTrailColor{255, 200, 60, 255};

}  // namespace

AppState::AppState() = default;

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
    process_thread_ = std::jthread([this, run_params = params]() mutable {
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
        run_params.cancel = &cancel_;
        run_params.progress = [this](const std::string& stage, double f) {
            std::lock_guard lock(mutex_);
            process_.stage = stage;
            process_.fraction = f;
        };
        auto r = recon::process_session(**reader, run_params);
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

bool AppState::connect(bool emulator) {
    if (scanning()) stop_scan();
    // A recording opened without a scanner goes on in the scanner's session (loaded again there).
    std::string reopen;
    {
        std::lock_guard lock(mutex_);
        if (session_ && session_->offline() && load_.loaded) reopen = load_.path;
    }
    session_.reset();
    trail_.clear();
    auto s = Session::open(
        emulator, [this](pipeline::LiveUpdate&& u) { on_live_update(std::move(u)); },
        [this](int button, device::ButtonAction a) {
            // Heartbeat thread: hand the command to the UI thread (update()).
            switch (device::button_command(button, a)) {
                case device::ButtonCommand::toggle_scan: toggle_requested_ = true; break;
                case device::ButtonCommand::brightness_down: --brightness_steps_; break;
                case device::ButtonCommand::brightness_up: ++brightness_steps_; break;
                case device::ButtonCommand::none: break;
            }
        });
    if (!s) {
        error_ = s.error().message;
        log::error("connect failed: {}", error_);
        return false;
    }
    error_.clear();
    session_ = std::move(*s);
    log::info("{}", session_->description());
    if (!reopen.empty()) open_recording(reopen);
    return true;
}

void AppState::open_recording(const std::string& path) {
    if (process_status().running) return;
    if (scanning()) stop_scan();
    if (!session_) {
        // No scanner: a pipeline with the recording's own calibration.
        auto reader = session::SessionReader::open(path);
        if (!reader) {
            std::lock_guard lock(mutex_);
            load_ = {};
            load_.path = path;
            load_.message = reader.error().message;
            return;
        }
        session::DeviceRecord recorded = (*reader)->device().value_or(session::DeviceRecord{});
        if (!(*reader)->device()) recorded.rig = synth::synthetic_einstar_rig();
        auto s = Session::open_offline(recorded, [this](pipeline::LiveUpdate&& u) { on_live_update(std::move(u)); });
        if (!s) {
            std::lock_guard lock(mutex_);
            load_ = {};
            load_.path = path;
            load_.message = s.error().message;
            return;
        }
        session_ = std::move(*s);
    }
    clear_selection();
    {
        std::lock_guard lock(mutex_);
        load_ = {};
        load_.loading = true;
        load_.path = path;
        load_.message = "Loading " + std::filesystem::path(path).filename().string();
        edit_ = {};
        trail_.clear();
        mesh_.reset();
        process_ = {};
        if (!pending_) pending_.emplace();
        pending_->mesh = std::pair<std::vector<render::MeshVertex>, std::vector<std::uint32_t>>{};
    }
    session_->pipeline().load_recording(path, [this](pipeline::LoadResult r) { on_loaded(std::move(r)); });
}

void AppState::on_loaded(pipeline::LoadResult&& r) {
    const auto trajectory = session_->pipeline().trajectory();
    std::lock_guard lock(mutex_);
    load_.loading = false;
    load_.loaded = r.ok;
    load_.frames = r.frames;
    load_.fused = r.fused;
    load_.markers = r.markers;
    load_.resumable = r.ok && r.resumable;
    const auto name = std::filesystem::path(r.path).filename().string();
    if (!r.ok) {
        load_.message = std::format("Could not open {}: {}", name, r.error);
        return;
    }
    load_.message = std::format("Opened {}: {} frames, {} fused, {} markers{}", name, r.frames, r.fused, r.markers, r.note.empty() ? "" : "; " + r.note);
    hud_.model_points = r.model.size();
    hud_.map_markers = r.markers;
    trail_.clear();
    for (const auto& T : trajectory) trail_.push_back(T.translation().cast<float>());
    if (!pending_) pending_.emplace();
    if (r.model.buffer) {
        pending_->model_gpu = GpuPoints{r.model.buffer, r.model.buffer_count};
        pending_->model.reset();
    } else {
        pending_->model = r.model.points;
        pending_->model_gpu.reset();
    }
    pending_->frame_points.clear();
    log::info("{}", load_.message);
}

LoadStatus AppState::load_status() const {
    std::lock_guard lock(mutex_);
    return load_;
}

void AppState::disconnect() {
    cancel_processing();
    if (process_thread_.joinable()) process_thread_.join();
    if (scanning()) stop_scan();
    session_.reset();
    std::lock_guard lock(mutex_);
    trail_.clear();
    hud_ = {};
}

Connection AppState::connection() const {
    Connection c;
    c.error = error_;
    if (!session_) return c;
    c.kind = session_->offline() ? Connection::Kind::recording : session_->emulated() ? Connection::Kind::emulator : Connection::Kind::scanner;
    c.online = session_->online();
    const auto& i = session_->info();
    c.device = std::format("{}{}, serial {}, firmware {}", i.product_name, session_->emulated() ? " (emulated)" : "", i.serial, i.firmware);
    c.calibration = session_->calibration();
    return c;
}

std::string AppState::recording_path() const { return session_ ? session_->pipeline().recording_path() : std::string{}; }

void AppState::new_scan(bool discard) {
    if (!session_) return;
    if (scanning()) stop_scan();
    session_->reset_model(discard);
    clear_selection();
    std::lock_guard lock(mutex_);
    edit_ = {};
    load_ = {};
    trail_.clear();
    if (!pending_) pending_.emplace();
    pending_->model = std::vector<render::PointVertex>{};
    pending_->mesh = std::pair<std::vector<render::MeshVertex>, std::vector<std::uint32_t>>{};
    mesh_.reset();
    process_ = {};
}

std::string AppState::status() const {
    if (!session_) return error_.empty() ? "Not connected" : "Connection failed: " + error_;
    return session_->description();
}

void AppState::start_scan() {
    if (!session_) return;
    {
        std::lock_guard lock(mutex_);
        if (load_.loading) return;
        if (load_.loaded && !load_.resumable) {
            error_ = "this recording was made with another scanner or calibration: start a new scan to scan";
            log::warn("start scan: {}", error_);
            return;
        }
    }
    clear_selection();
    {
        std::lock_guard lock(mutex_);
        edit_ = {};  // deletes are final once frames fuse again
    }
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
    clear_selection();
    std::lock_guard lock(mutex_);
    edit_ = {};
    load_ = {};
    trail_.clear();
    if (!pending_) pending_.emplace();
    pending_->model = std::vector<render::PointVertex>{};
    pending_->mesh = std::pair<std::vector<render::MeshVertex>, std::vector<std::uint32_t>>{};
    mesh_.reset();
    process_ = {};
}

void AppState::set_record_raw_ir(bool on) {
    if (session_) session_->pipeline().set_record_raw_ir(on);
}

bool AppState::record_raw_ir() const { return session_ && session_->pipeline().record_raw_ir(); }

void AppState::apply_settings() {
    if (session_ && session_->scanning()) (void)session_->apply(settings);
}

void AppState::set_brightness(int level) {
    settings.brightness = std::clamp(level, 0, device::kBrightnessLevels - 1);
    const auto eg = device::brightness_level(settings.brightness);
    settings.exposure = static_cast<int>(eg.exposure);
    settings.gain = eg.gain;
    apply_settings();
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
    // A markers file, or the global markers recorded in an earlier scan.
    std::optional<std::vector<markers::MapMarker>> map;
    if (path.ends_with(".estr")) {
        if (auto reader = session::SessionReader::open(path); reader && !(*reader)->global_markers().empty())
            map = (*reader)->global_markers();
    } else if (auto m = markers::load_markers(path)) {
        map = std::move(*m);
    }
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
    if (toggle_requested_.exchange(false) && scanner_button_enabled) toggle_scan();
    if (const int steps = brightness_steps_.exchange(0); steps != 0) {
        set_brightness(settings.brightness + steps);
        std::lock_guard lock(mutex_);
        notice_ = std::format("Brightness {} / {}", settings.brightness + 1, device::kBrightnessLevels);
        notice_clock_.reset();
    }
    if (!session_ || housekeeping_.elapsed_ms() < 500) return;
    housekeeping_.reset();
    float depth;
    {
        std::lock_guard lock(mutex_);
        depth = last_depth_;
    }
    if (session_->scanning()) session_->set_distance_indication(depth);
    static int ticks = 0;
    if (++ticks % 4 == 0) {
        if (auto t = session_->temperature()) {
            session_->pipeline().set_temperature(static_cast<float>(*t));
            std::lock_guard lock(mutex_);
            hud_.temperature_c = static_cast<float>(*t);
            hud_.temperature_note.clear();
        } else if (t.error().code == Errc::unsupported) {
            std::lock_guard lock(mutex_);
            if (hud_.temperature_note.empty()) log::info("temperature: {}", t.error().message);
            hud_.temperature_c = -273.0f;
            hud_.temperature_note = "no sensor reading";
        }
    }
}

bool AppState::can_edit() const {
    if (!session_ || scanning()) return false;
    std::lock_guard lock(mutex_);
    return hud_.model_points > 0 && !edit_.busy;
}

void AppState::add_lasso(LassoStroke stroke) {
    selection_.add(std::move(stroke));
    ++selection_version_;
}

void AppState::clear_selection() {
    if (selection_.strokes().empty()) return;
    selection_.clear();
    ++selection_version_;
}

void AppState::delete_selection() {
    if (!can_edit() || selection_.empty()) return;
    {
        std::lock_guard lock(mutex_);
        edit_.busy = true;
    }
    session_->pipeline().erase(selection_, [this](pipeline::EditResult r) { on_edit(r, false); });
    clear_selection();
}

void AppState::undo_delete() {
    if (!session_ || scanning()) return;
    {
        std::lock_guard lock(mutex_);
        if (edit_.undo_depth == 0 || edit_.busy) return;
        edit_.busy = true;
    }
    session_->pipeline().undo_erase([this](pipeline::EditResult r) { on_edit(r, true); });
}

EditStatus AppState::edit_status() const {
    std::lock_guard lock(mutex_);
    return edit_;
}

void AppState::on_edit(const pipeline::EditResult& r, bool undo) {
    std::lock_guard lock(mutex_);
    const std::size_t before = hud_.model_points, after = r.model.size();
    hud_.model_points = after;
    edit_.busy = false;
    edit_.undo_depth = r.undo_depth;
    if (undo) edit_.message = std::format("Undone: {} points back", after > before ? after - before : 0);
    else if (r.erased_voxels == 0) edit_.message = "Nothing in the selection";
    else edit_.message = std::format("Deleted {} points", before > after ? before - after : 0);
    if (!pending_) {
        pending_.emplace();
        pending_->model_only = true;
    }
    if (r.model.buffer) {
        pending_->model_gpu = GpuPoints{r.model.buffer, r.model.buffer_count};
        pending_->model.reset();
    } else {
        pending_->model = r.model.points;
        pending_->model_gpu.reset();
    }
}

std::optional<RenderUpdate> AppState::take_render_update() {
    std::lock_guard lock(mutex_);
    return std::exchange(pending_, std::nullopt);
}

Hud AppState::hud() const {
    std::lock_guard lock(mutex_);
    Hud h = hud_;
    if (notice_clock_.elapsed_ms() < 2000) h.notice = notice_;
    return h;
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
    up.ir_left_tex = std::move(u.preview_left_tex);
    up.ir_right_tex = std::move(u.preview_right_tex);
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
    hud_.raw_frames = st.raw_frames;
    hud_.raw_dropped = st.raw_dropped;
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
    pending_->ir_left_tex = std::move(up.ir_left_tex);
    pending_->ir_right_tex = std::move(up.ir_right_tex);
    pending_->ir_right = std::move(up.ir_right);
    pending_->markers = std::move(up.markers);
    pending_->preview_markers = std::move(up.preview_markers);
    if (up.mesh) pending_->mesh = std::move(up.mesh);
    pending_->scanner_pose = up.scanner_pose;
}

}  // namespace einstar::app
