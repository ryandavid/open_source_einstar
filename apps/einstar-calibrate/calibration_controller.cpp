#include "calibration_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <format>
#include <fstream>
#include <iterator>
#include <random>

#include <tbb/parallel_invoke.h>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/calibrate/captures.hpp"
#include "einstar/calibrate/synthetic.hpp"
#include "einstar/core/log.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/sim/sim_transport.hpp"
#include "einstar/synth/demo.hpp"

namespace einstar::app {
namespace {

namespace fs = std::filesystem;
using calibrate::BoardSpec;

constexpr const char* kExstarCalibrationCache = "/Applications/EXStar.app/Contents/Resources/res/Einscan-E10/200x150";

std::string now_string(const char* fmt) {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[64];
    std::strftime(buf, sizeof buf, fmt, &tm);
    return buf;
}

double seconds_now() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

fs::path calibration_root() {
    if (const char* dir = std::getenv("EINSTAR_CALIBRATION_DIR"); dir && *dir) return dir;
    const char* home = std::getenv("HOME");
    return fs::path(home ? home : ".") / "Documents" / "Einstar" / "Calibration";
}

// The emulator: a board the virtual scanner moves towards the pose being asked for, seen through a
// rig whose right camera has moved slightly since the calibration in its flash (so a fresh
// calibration has something to find).
struct Emulator {
    RigCalibration truth;
    BoardSpec board;
    std::mutex mutex;
    SE3 hand = SE3::Identity();  // board -> left camera
    SE3 goal = SE3::Identity();
    bool has_goal = false;
    std::uint32_t cached_id = ~0u;
    ImageU8 left, right;
    std::mt19937 rng{7};

    void step() {
        std::normal_distribution<double> n(0.0, 1.0);
        if (has_goal) {
            // Move a quarter of the way per frame, like a hand settling on a pose.
            const Eigen::Quaterniond q0(hand.linear()), q1(goal.linear());
            hand.linear() = q0.slerp(0.25, q1).toRotationMatrix();
            hand.translation() += 0.25 * (goal.translation() - hand.translation());
        }
        hand.translation() += 0.08 * Vec3(n(rng), n(rng), n(rng));
        hand.linear() = Eigen::AngleAxisd(0.0003, Vec3(n(rng), n(rng), n(rng)).normalized()).toRotationMatrix() * hand.linear();
    }
};

RigCalibration moved_rig(RigCalibration r) {
    r.right.cy += 1.5;
    r.right.fx += 0.8;
    const SE3 turn = [] {
        SE3 t = SE3::Identity();
        t.linear() = Eigen::AngleAxisd(0.25 * M_PI / 180.0, Vec3(1, 0.3, 0).normalized()).toRotationMatrix();
        return t;
    }();
    r.T_right_left = turn * r.T_right_left;
    return r;
}

}  // namespace

CalibrationController::CalibrationController() : plan_(calibrate::default_plan()), captures_(plan_.size()) {
    guidance_rig_ = synth::synthetic_einstar_rig();
}

CalibrationController::~CalibrationController() { disconnect(); }

std::string CalibrationController::status() const {
    std::lock_guard lk(state_mutex_);
    return status_;
}

Result<void> CalibrationController::connect(bool emulator) {
    disconnect();
    std::unique_ptr<usb::Transport> transport;
    std::shared_ptr<Emulator> emu;
    // The scanner reboots or restarts its USB side on its own (docs/firmware.md 5): reopen it and replay.
    device::ConnectOptions opts;
    if (!emulator) {
        auto devices = usb::enumerate_devices();
        if (!devices || devices->empty()) return make_error(Errc::not_found, "No Einstar found on USB (use the emulator to try the procedure)");
        const usb::UsbDeviceInfo info = devices->front();
        auto t = usb::open_libusb(info);
        if (!t) return std::unexpected(t.error());
        transport = std::move(*t);
        opts.reopen = [info] { return usb::reopen_libusb(info); };
    } else {
        auto sim = std::make_shared<sim::SimDevice>();
        emu = std::make_shared<Emulator>();
        RigCalibration stored = synth::synthetic_einstar_rig();
        for (const char* dir : {kExstarCalibrationCache, EINSTAR_CALIBRATION_FIXTURE}) {
            if (auto blob = calib::encode_quick_flash_blob_from_directory(dir)) {
                sim->set_flash(0, *blob);
                if (auto cal = calib::decode_flash_blob(*blob)) stored = cal->rig();
                break;
            }
        }
        emu->truth = moved_rig(stored);
        // Start well away from the first pose so the guidance has something to say.
        emu->hand = calibrate::target_pose(plan_.front(), 90, stored);
        emu->hand.translation() += Vec3(40, -30, 160);
        sim->set_frame_provider([emu](int sensor, std::uint32_t frame_id, ImageU8& out) {
            std::lock_guard lk(emu->mutex);
            if (sensor == 2) {
                out.fill(40);
                return;
            }
            if (frame_id != emu->cached_id) {
                emu->step();
                calibrate::SyntheticBoardParams sp;
                sp.supersample = 1;
                sp.seed = frame_id + 1;
                std::tie(emu->left, emu->right) = calibrate::render_board_pair(emu->truth, emu->hand, emu->board, sp);
                emu->cached_id = frame_id;
            }
            out = sensor == 0 ? emu->left : emu->right;
        });
        auto t = sim->connect();
        if (!t) return std::unexpected(t.error());
        transport = std::move(*t);
        opts.reopen = [sim] { return sim->connect(); };
    }

    auto dev = device::EinstarDevice::connect(std::move(transport), opts);
    if (!dev) return std::unexpected(dev.error());
    const auto& info = (*dev)->info();
    serial_ = info.serial;
    auto blob = (*dev)->read_flash(0, calib::kFlashBlobSize);
    flash_.reset();
    factory_.reset();
    flash_time_.clear();
    if (blob) {
        if (auto cal = calib::decode_flash_blob(*blob)) flash_ = cal->rig(), flash_time_ = cal->calibration_time;
        if (auto cal = calib::decode_factory_section(*blob)) factory_ = cal->rig();
    }
    guidance_rig_ = flash_ ? *flash_ : synth::synthetic_einstar_rig();
    emulated_ = emulator;
    emulator_ = emu;
    description_ = std::format("{} {} (serial {}, firmware {})", emulator ? "Emulated" : "Scanner", info.product_name, info.serial, info.firmware);
    device_ = std::move(*dev);
    device_->set_button_sink([this](int button, device::ButtonAction a) {
        if (button == 1 && a == device::ButtonAction::single_click) capture_now();  // start / pause: capture
    });
    if (auto r = apply_lighting(); !r) {
        disconnect();
        return r;
    }
    if (auto r = device_->configure_texture_mode(100000); !r) {
        disconnect();
        return r;
    }
    (void)device_->set_indication(device::DistanceIndication::zone2);
    worker_ = std::jthread([this](std::stop_token st) { worker_loop(st); });
    if (auto r = device_->start_stream([this](usb::FrameGroup&& g) { on_group(std::move(g)); }); !r) {
        disconnect();
        return r;
    }
    {
        std::lock_guard lk(state_mutex_);
        status_ = flash_ ? std::format("Connected. Guidance uses the calibration in the scanner ({}).", flash_time_)
                         : "Connected. No calibration in the scanner: guidance uses a nominal one.";
    }
    new_session_dir();
    return {};
}

void CalibrationController::disconnect() {
    if (device_) {
        (void)device_->set_trigger(0, 0);
        (void)device_->set_strobe(0, 0);
        (void)device_->set_strobe(1, 0);
        device_->stop_stream();
    }
    if (worker_.joinable()) {
        worker_.request_stop();
        mailbox_cv_.notify_all();
        worker_.join();
    }
    device_.reset();
    emulator_.reset();
}

Result<void> CalibrationController::apply_lighting() {
    if (!device_) return {};
    if (auto r = device_->set_exposure(0, static_cast<std::uint32_t>(lighting.exposure)); !r) return r;
    for (int sensor = 0; sensor < 2; ++sensor)
        if (auto r = device_->set_gain(sensor, static_cast<std::uint16_t>(lighting.gain)); !r) return r;
    if (auto r = device_->set_laser_percent(0); !r) return r;
    if (auto r = device_->set_strobe(0, lighting.ring_light); !r) return r;
    return device_->set_strobe(1, lighting.white_leds);
}

void CalibrationController::new_session_dir() {
    // The emulator reports the real unit's serial: keep its sessions apart.
    session_dir_ = calibration_root() / (emulated_ ? "emulator" : serial_.empty() ? "unknown" : serial_) / now_string("%Y-%m-%d_%H%M%S");
}

void CalibrationController::set_active_group(int g) {
    group_ = std::clamp(g, 0, static_cast<int>(plan_.back().group));
    gate_.reset();
}

std::vector<std::optional<CaptureRecord>> CalibrationController::captures() const {
    std::lock_guard lk(state_mutex_);
    return captures_;
}

int CalibrationController::captured_count() const {
    std::lock_guard lk(state_mutex_);
    return static_cast<int>(std::ranges::count_if(captures_, [](const auto& c) { return c.has_value(); }));
}

void CalibrationController::clear_capture(int index) {
    std::lock_guard lk(state_mutex_);
    if (index >= 0 && index < static_cast<int>(captures_.size())) captures_[static_cast<std::size_t>(index)].reset();
}

void CalibrationController::clear_all() {
    {
        std::lock_guard lk(state_mutex_);
        for (auto& c : captures_) c.reset();
    }
    group_ = 0;
    {
        std::lock_guard lk(solve_mutex_);
        solve_ = {};
    }
    new_session_dir();
}

LiveState CalibrationController::live() const {
    std::lock_guard lk(state_mutex_);
    return live_;
}

void CalibrationController::on_group(usb::FrameGroup&& g) {
    if (!g.sensors[0] || !g.sensors[1]) return;
    {
        std::lock_guard lk(mailbox_mutex_);
        mailbox_ = std::move(g);  // only the newest frame matters
    }
    mailbox_cv_.notify_one();
}

void CalibrationController::worker_loop(std::stop_token st) {
    while (!st.stop_requested()) {
        std::optional<usb::FrameGroup> g;
        {
            std::unique_lock lk(mailbox_mutex_);
            mailbox_cv_.wait_for(lk, std::chrono::milliseconds(200), [&] { return mailbox_.has_value() || st.stop_requested(); });
            if (!mailbox_) continue;
            g = std::move(mailbox_);
            mailbox_.reset();
        }
        process(std::move(*g));
    }
}

int CalibrationController::guided_target(const calibrate::BoardMeasure* m) const {
    // The active group's uncaptured step nearest the current distance (the ladder is climbed in any
    // order); once a group is complete, the next incomplete one.
    const int groups = plan_.back().group + 1;
    for (int k = 0; k < groups; ++k) {
        const int g = (group_.load() + k) % groups;
        int best = -1;
        double bd = 1e18;
        for (std::size_t i = 0; i < plan_.size(); ++i) {
            if (plan_[i].group != g || captures_[i]) continue;
            const double d = m ? std::abs(m->distance_mm - plan_[i].distance_mm) : static_cast<double>(plan_[i].step);
            if (d < bd) bd = d, best = static_cast<int>(i);
        }
        if (best >= 0) return best;
    }
    return -1;
}

void CalibrationController::process(usb::FrameGroup&& g) {
    auto left = std::make_shared<ImageU8>(std::move(g.sensors[0]->pixels));
    auto right = std::make_shared<ImageU8>(std::move(g.sensors[1]->pixels));
    std::shared_ptr<const ImageU8> tex = g.sensors[2] ? std::make_shared<ImageU8>(std::move(g.sensors[2]->pixels)) : nullptr;
    Stopwatch sw;
    std::optional<calibrate::BoardDetection> dl, dr;
    tbb::parallel_invoke([&] { dl = calibrate::detect_board(left->view()); }, [&] { dr = calibrate::detect_board(right->view()); });
    LiveState s;
    s.detect_ms = sw.elapsed_ms();
    s.left = left;
    s.right = right;
    {
        std::uint64_t sum = 0, sat = 0, n = 0;
        for (int y = 0; y < left->height(); y += 4)
            for (int x = 0; x < left->width(); x += 4) {
                const auto v = (*left)(x, y);
                sum += v, sat += v == 255, ++n;
            }
        s.mean_level = n ? static_cast<int>(sum / n) : 0;
        s.saturated_permille = n ? static_cast<int>(1000 * sat / n) : 0;
    }
    if (dl && dr)
        for (const auto& a : dl->grid) s.common_dots += static_cast<int>(std::ranges::count(dr->grid, a));
    if (dl) s.pose = calibrate::board_pose(*dl, guidance_rig_.left);
    if (s.pose) s.measure = calibrate::measure_board(s.pose->T_cam_board, guidance_rig_);
    const double now = seconds_now();

    std::lock_guard lk(state_mutex_);
    s.sequence = live_.sequence + 1;
    s.fps = last_frame_time_ > 0 ? 0.8 * live_.fps + 0.2 / std::max(1e-3, now - last_frame_time_) : 0;
    last_frame_time_ = now;
    s.target = guided_target(s.measure ? &*s.measure : nullptr);
    if (s.target >= 0) group_ = plan_[static_cast<std::size_t>(s.target)].group;
    s.steady_needed_s = gate_.limits().hold_s;
    if (s.pose) s.steady_s = gate_.update(s.pose->T_cam_board, now);
    else gate_.reset();
    if (s.measure && s.target >= 0) s.guidance = calibrate::guide(*s.measure, plan_[static_cast<std::size_t>(s.target)]);

    // The scanner's top LED: red too near, green in range, blue too far (as EXStar does).
    if (device_ && s.guidance) {
        const auto& t = plan_[static_cast<std::size_t>(s.target)];
        const auto zone = s.guidance->distance_error_mm < -t.distance_tol_mm ? device::DistanceIndication::zone0
                          : s.guidance->distance_error_mm > t.distance_tol_mm ? device::DistanceIndication::zone2
                                                                               : device::DistanceIndication::zone1;
        if (static_cast<int>(zone) != last_led_zone_) {
            last_led_zone_ = static_cast<int>(zone);
            (void)device_->set_indication(zone);
        }
    }

    const bool manual = capture_requested_.exchange(false);
    const bool steady = gate_.ready(s.steady_s);
    if (s.target >= 0 && s.common_dots >= 20 && ((auto_capture && s.guidance && s.guidance->ok() && steady) || manual)) {
        CaptureRecord rec;
        rec.capture.name = std::format("imageLeft{}", s.target + 1);
        rec.capture.left = *dl;
        rec.capture.right = *dr;
        rec.measure = *s.measure;
        rec.left = left;
        rec.right = right;
        rec.time = now_string("%H:%M:%S");
        store_capture(s.target, std::move(rec), tex);
        gate_.reset();
        s.target = guided_target(&*s.measure);
    }
    s.det_left = std::move(dl);
    s.det_right = std::move(dr);
    live_ = std::move(s);

    if (auto emu = std::static_pointer_cast<Emulator>(emulator_)) {
        std::lock_guard elk(emu->mutex);
        emu->has_goal = live_.target >= 0;
        if (emu->has_goal) emu->goal = calibrate::target_pose(plan_[static_cast<std::size_t>(live_.target)], 90, guidance_rig_);
    }
}

// Called with state_mutex_ held.
void CalibrationController::store_capture(int index, CaptureRecord rec, std::shared_ptr<const ImageU8> tex) {
    std::error_code ec;
    fs::create_directories(session_dir_, ec);
    const auto n = index + 1;
    (void)calibrate::write_pgm(session_dir_ / std::format("imageLeft{}.pgm", n), rec.left->view());
    (void)calibrate::write_pgm(session_dir_ / std::format("imageRight{}.pgm", n), rec.right->view());
    if (tex) (void)calibrate::write_pgm(session_dir_ / std::format("imageTex{}.pgm", n), tex->view());
    status_ = std::format("Captured {} ({}) into {}", plan_[static_cast<std::size_t>(index)].label, rec.time, session_dir_.string());
    captures_[static_cast<std::size_t>(index)] = std::move(rec);
}

Result<void> CalibrationController::load_folder(const fs::path& dir) {
    auto loaded = calibrate::load_captures(dir);
    if (!loaded) return std::unexpected(loaded.error());
    const auto pairs = calibrate::list_capture_pairs(dir);
    std::lock_guard lk(state_mutex_);
    for (auto& c : captures_) c.reset();
    for (std::size_t i = 0; i < loaded->captures.size() && i < captures_.size(); ++i) {
        CaptureRecord rec;
        rec.capture = std::move(loaded->captures[i]);
        if (auto p = calibrate::board_pose(rec.capture.left, guidance_rig_.left)) rec.measure = calibrate::measure_board(p->T_cam_board, guidance_rig_);
        if (auto l = calibrate::read_image(pairs[i].left)) rec.left = std::make_shared<ImageU8>(std::move(*l));
        if (auto r = calibrate::read_image(pairs[i].right)) rec.right = std::make_shared<ImageU8>(std::move(*r));
        rec.time = "loaded";
        captures_[i] = std::move(rec);
    }
    // Results go next to our own sessions, never into the folder read (it may be EXStar's).
    session_dir_ = calibration_root() / "offline" / std::format("{}_{}", dir.filename().string(), now_string("%Y-%m-%d_%H%M%S"));
    status_ = std::format("Loaded {} captures from {}", std::min(loaded->captures.size(), captures_.size()), dir.string());
    return {};
}

Result<void> CalibrationController::load_reference(const fs::path& path) {
    std::optional<RigCalibration> flash, factory;
    std::string time;
    if (fs::is_directory(path)) {
        auto cal = calib::load_ccf_directory(path.string());
        if (!cal) return std::unexpected(cal.error());
        flash = cal->rig(), time = cal->calibration_time;
    } else if (path.extension() == ".bin") {
        std::ifstream f(path, std::ios::binary);
        const std::vector<std::uint8_t> blob((std::istreambuf_iterator<char>(f)), {});
        auto cal = calib::decode_flash_blob(blob);
        if (!cal) return std::unexpected(cal.error());
        flash = cal->rig(), time = cal->calibration_time;
        if (auto fac = calib::decode_factory_section(blob)) factory = fac->rig();
    } else {
        auto file = calibrate::read_calibration_file(path.string());
        if (!file) return std::unexpected(file.error());
        flash = file->rig, time = file->created;
    }
    flash_ = flash;
    factory_ = factory;
    flash_time_ = time.empty() ? path.filename().string() : time;
    guidance_rig_ = *flash_;
    std::lock_guard lk(state_mutex_);
    status_ = std::format("Reference calibration: {}", path.string());
    return {};
}

void CalibrationController::solve() {
    std::vector<calibrate::StereoCapture> caps;
    int width = 1280, height = 1024;
    {
        std::lock_guard lk(state_mutex_);
        for (const auto& c : captures_)
            if (c) {
                caps.push_back(c->capture);
                if (c->left) width = c->left->width(), height = c->left->height();
            }
    }
    {
        std::lock_guard lk(solve_mutex_);
        if (solve_.running) return;
        solve_ = {};
        solve_.running = true;
    }
    if (solver_.joinable()) solver_.join();
    calibrate::SolveOptions so;
    if (keep_factory_distortion) {
        const auto& ref = factory_ ? factory_ : flash_;
        if (ref) so.fixed_distortion = std::array{ref->left.dist, ref->right.dist};
    }
    solver_ = std::jthread([this, caps = std::move(caps), width, height, so, flash = flash_, factory = factory_](std::stop_token) {
        SolveState out;
        Stopwatch sw;
        auto r = calibrate::solve_stereo(caps, width, height, {}, so);
        out.solve_ms = sw.elapsed_ms();
        if (!r) {
            out.error = r.error().message;
        } else {
            out.ours = std::move(*r);
            if (flash) {
                if (auto e = calibrate::evaluate_calibration(*flash, caps)) out.flash = std::move(*e);
                out.vs_flash = calibrate::compare_calibrations(*flash, out.ours->rig);
            }
            if (factory) {
                if (auto e = calibrate::evaluate_calibration(*factory, caps)) out.factory = std::move(*e);
                out.vs_factory = calibrate::compare_calibrations(*factory, out.ours->rig);
            }
        }
        std::lock_guard lk(solve_mutex_);
        solve_ = std::move(out);
    });
}

SolveState CalibrationController::solve_state() const {
    std::lock_guard lk(solve_mutex_);
    return solve_;
}

Result<fs::path> CalibrationController::save_result() {
    const auto st = solve_state();
    if (!st.ours) return make_error(Errc::invalid_argument, "nothing solved yet");
    std::error_code ec;
    fs::create_directories(session_dir_, ec);
    calibrate::CalibrationFile f;
    f.rig = st.ours->rig;
    if (flash_) f.rig.texture = flash_->texture, f.rig.T_texture_left = flash_->T_texture_left;  // the colour camera is not re-calibrated
    f.serial = serial_;
    f.created = now_string("%Y-%m-%d %H:%M");
    f.source = std::format("einstar-calibrate, {} views{}", std::ranges::count_if(st.ours->views, &calibrate::ViewReport::used),
                           keep_factory_distortion ? ", factory distortion" : "");
    f.rms_px = st.ours->rms_px, f.row_rms_px = st.ours->row_rms_px;
    f.views = static_cast<int>(std::ranges::count_if(st.ours->views, &calibrate::ViewReport::used));
    const auto path = session_dir_ / "calibration.txt";
    if (auto r = calibrate::write_calibration_file(path.string(), f); !r) return std::unexpected(r.error());

    std::ofstream rep(session_dir_ / "report.txt");
    rep << std::format("{}\n{}\nsolve: {} views, {} dots, reprojection {:.3f} px, rectified rows {:.3f} px (max {:.2f})\n", description_, f.created,
                       f.views, st.ours->dots, st.ours->rms_px, st.ours->row_rms_px, st.ours->max_row_px);
    if (st.flash) rep << std::format("flash calibration ({}) on these captures: reprojection {:.3f} px, rows {:.3f} px (max {:.2f})\n", flash_time_, st.flash->rms_px,
                                     st.flash->row_rms_px, st.flash->max_row_px);
    if (st.vs_flash) {
        const auto& d = *st.vs_flash;
        rep << std::format("ours - flash: left f {:+.2f} {:+.2f} c {:+.2f} {:+.2f}; right f {:+.2f} {:+.2f} c {:+.2f} {:+.2f}; rig rotation ({:+.3f} {:+.3f} {:+.3f}) deg, "
                           "translation ({:+.2f} {:+.2f} {:+.2f}) mm\n",
                           d.left.dfx, d.left.dfy, d.left.dcx, d.left.dcy, d.right.dfx, d.right.dfy, d.right.dcx, d.right.dcy, d.rotation_deg.x(),
                           d.rotation_deg.y(), d.rotation_deg.z(), d.translation_mm.x(), d.translation_mm.y(), d.translation_mm.z());
    }
    for (const auto& v : st.ours->views)
        rep << std::format("{:14} {:>3} dots  {:>5.0f} mm  tilt {:+5.1f} {:+5.1f}  rms {:.3f}  rows {:.3f}{}\n", v.name, v.dots, v.distance_mm, v.tilt_x_deg,
                           v.tilt_y_deg, v.rms_px, v.row_rms_px, v.used ? "" : "  (not used)");
    {
        std::lock_guard lk(solve_mutex_);
        solve_.saved_path = path.string();
    }
    return path;
}

fs::path CalibrationController::active_calibration_path() const { return calibrate::active_calibration_path(serial_); }

Result<void> CalibrationController::use_for_scanning(bool use) {
    const auto active = active_calibration_path();
    std::error_code ec;
    if (use && (emulated_ || !device_))
        return make_error(Errc::invalid_argument, "Only a calibration made with the scanner connected can be used for scanning");
    if (!use) {
        fs::remove(active, ec);
        return {};
    }
    const auto st = solve_state();
    if (st.saved_path.empty()) {
        if (auto r = save_result(); !r) return std::unexpected(r.error());
    }
    fs::create_directories(active.parent_path(), ec);
    fs::copy_file(solve_state().saved_path, active, fs::copy_options::overwrite_existing, ec);
    if (ec) return make_error(Errc::io, std::format("cannot write {}: {}", active.string(), ec.message()));
    return {};
}

}  // namespace einstar::app
