#include "app_state.hpp"

#include <algorithm>
#include <format>

#include "einstar/core/log.hpp"

namespace einstar::app {
namespace {

constexpr render::Rgba8 kTrackedColor{90, 230, 120, 255};
constexpr render::Rgba8 kLostColor{235, 70, 70, 255};
constexpr render::Rgba8 kGhostColor{200, 200, 200, 255};
constexpr render::Rgba8 kTrailColor{255, 200, 60, 255};

}  // namespace

AppState::AppState() { connect(false); }

AppState::~AppState() { session_.reset(); }

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
}

void AppState::apply_settings() {
    if (session_ && session_->scanning()) (void)session_->apply(settings);
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
    if (u.model_changed) up.model = std::move(u.model);
    up.frame_points = std::move(u.frame_points);
    up.ir_left = std::move(u.preview_left);
    up.ir_right = std::move(u.preview_right);

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
    // Working range 175..625 mm mapped onto the 10-step bar.
    hud_.distance_step = st.mean_depth_mm > 0 ? std::clamp(static_cast<int>((st.mean_depth_mm - 175.0f) / 45.0f), 0, 9) : -1;
    last_depth_ = st.mean_depth_mm;

    if (!pending_) {
        pending_ = std::move(up);
        return;
    }
    // UI hasn't consumed the previous update: keep the newest of everything.
    if (up.model) pending_->model = std::move(up.model);
    pending_->frame_points = std::move(up.frame_points);
    pending_->lines = std::move(up.lines);
    pending_->ir_left = std::move(up.ir_left);
    pending_->ir_right = std::move(up.ir_right);
    pending_->scanner_pose = up.scanner_pose;
}

}  // namespace einstar::app
