#pragma once

// einstar-calibrate's model: the scanner (or the emulator), live board detection with guidance towards
// the capture plan, auto-capture, the solve, and saving. The UI (main.mm) only reads snapshots and
// calls these methods; the work runs on a detection worker and a solver thread.

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "einstar/calibrate/plan.hpp"
#include "einstar/calibrate/solve.hpp"
#include "einstar/device/einstar_device.hpp"

namespace einstar::app {

// EXStar's calibration lighting (rapidCameraCalibrate.xml): IR exposure 1500, gain 400, ring light
// 1000, white LEDs 300, projector off, texture mode (IR pair + colour at 10 Hz).
struct CaptureLighting {
    int exposure = 1500;
    int gain = 400;
    int ring_light = 1000;  // strobe route 0
    int white_leds = 300;   // strobe route 1
};

struct LiveState {
    std::uint64_t sequence = 0;  // increments per processed frame
    std::shared_ptr<const ImageU8> left, right;
    std::optional<calibrate::BoardDetection> det_left, det_right;
    std::optional<calibrate::BoardPose> pose;  // in the left camera, with the guidance calibration
    std::optional<calibrate::BoardMeasure> measure;
    std::optional<calibrate::Guidance> guidance;
    int target = -1;           // plan index being guided to (-1: none left in the group)
    double steady_s = 0;
    double steady_needed_s = 0;
    int common_dots = 0;
    double detect_ms = 0;
    double fps = 0;
    int saturated_permille = 0;  // share of left pixels at 255
    int mean_level = 0;          // left image mean grey
};

struct CaptureRecord {
    calibrate::StereoCapture capture;
    calibrate::BoardMeasure measure;
    std::shared_ptr<const ImageU8> left, right;
    std::string time;
};

struct SolveState {
    bool running = false;
    std::string error;
    std::optional<calibrate::CalibrationReport> ours;
    // The scanner's stored calibrations evaluated on the same captures (board poses fitted only).
    std::optional<calibrate::CalibrationReport> flash, factory;
    std::optional<calibrate::CalibrationDiff> vs_flash, vs_factory;
    std::string saved_path;
    double solve_ms = 0;
};

class CalibrationController {
public:
    CalibrationController();
    ~CalibrationController();

    // ---- connection ----
    Result<void> connect(bool emulator);
    void disconnect();
    [[nodiscard]] bool connected() const { return device_ != nullptr; }
    [[nodiscard]] bool emulated() const { return emulated_; }
    [[nodiscard]] const std::string& description() const { return description_; }
    [[nodiscard]] std::string status() const;
    [[nodiscard]] const std::optional<RigCalibration>& flash_rig() const { return flash_; }
    [[nodiscard]] const std::optional<RigCalibration>& factory_rig() const { return factory_; }
    [[nodiscard]] const std::string& flash_time() const { return flash_time_; }
    [[nodiscard]] const RigCalibration& guidance_rig() const { return guidance_rig_; }

    CaptureLighting lighting;
    Result<void> apply_lighting();

    // ---- plan and capture ----
    [[nodiscard]] const std::vector<calibrate::PoseTarget>& plan() const { return plan_; }
    [[nodiscard]] int active_group() const { return group_.load(); }
    void set_active_group(int g);
    [[nodiscard]] std::vector<std::optional<CaptureRecord>> captures() const;
    [[nodiscard]] int captured_count() const;
    void clear_capture(int index);
    void clear_all();
    std::atomic<bool> auto_capture{true};
    void capture_now() { capture_requested_ = true; }
    [[nodiscard]] LiveState live() const;
    [[nodiscard]] const std::filesystem::path& session_dir() const { return session_dir_; }

    // Offline: the captures in a folder (einstar-calibrate's session or EXStar's imageLeftN.bmp).
    Result<void> load_folder(const std::filesystem::path& dir);
    // Offline: the calibration to compare with (and to guide with) when no scanner is connected: a CCF
    // directory (EXStar's cache), a flash blob (.bin; its factory section too) or a calibration file.
    Result<void> load_reference(const std::filesystem::path& path);

    // ---- solve ----
    bool keep_factory_distortion = false;  // EXStar's quick calibration re-fits all but the distortion
    void solve();
    [[nodiscard]] SolveState solve_state() const;
    // Writes calibration.txt (and report.txt) to the session folder.
    Result<std::filesystem::path> save_result();
    // ---- writing the result into the scanner (as EXStar's quick calibration does) ----
    struct Gate {
        std::string what;
        bool ok = false;
    };
    struct WritePlan {
        std::optional<calibrate::FlashUpdate> update;  // the new blob (only its quick section differs)
        std::vector<Gate> gates;                       // all must pass
        std::string reference_view;                    // the board pose the stored extrinsics refer to
        std::filesystem::path backup_path;             // where the current pages 0-1 will be saved
        std::string error;
        [[nodiscard]] bool ready() const {
            return update && std::ranges::all_of(gates, [](const Gate& g) { return g.ok; });
        }
    };
    [[nodiscard]] WritePlan plan_write() const;
    // Writes the plan into the scanner's flash (stream paused; backup first; read back and verified),
    // then re-reads the calibration from the scanner. The Einstar app reads it at connect.
    Result<std::string> write_to_scanner(const WritePlan& plan);
    // Puts a backup made by write_to_scanner back.
    Result<std::string> restore_backup(const std::filesystem::path& file);
    [[nodiscard]] const std::vector<std::uint8_t>& flash_blob() const { return flash_blob_; }

private:
    void on_group(usb::FrameGroup&& g);
    void worker_loop(std::stop_token st);
    void process(usb::FrameGroup&& g);
    void store_capture(int index, CaptureRecord rec, std::shared_ptr<const ImageU8> tex);
    [[nodiscard]] int guided_target(const calibrate::BoardMeasure* m) const;
    void new_session_dir();

    std::unique_ptr<device::EinstarDevice> device_;
    bool emulated_ = false;
    std::string description_, serial_;
    std::optional<RigCalibration> flash_, factory_;
    std::string flash_time_;
    std::vector<std::uint8_t> flash_blob_;  // the scanner's 6568-byte calibration blob, as last read
    Result<void> reread_flash();
    Result<void> resume_stream();
    RigCalibration guidance_rig_;
    std::shared_ptr<void> emulator_;  // emulator scene state (see .cpp)

    std::vector<calibrate::PoseTarget> plan_;
    std::atomic<int> group_{0};
    std::atomic<bool> capture_requested_{false};

    mutable std::mutex mailbox_mutex_;
    std::condition_variable mailbox_cv_;
    std::optional<usb::FrameGroup> mailbox_;
    std::jthread worker_;

    mutable std::mutex state_mutex_;
    LiveState live_;
    std::vector<std::optional<CaptureRecord>> captures_;
    std::filesystem::path session_dir_;
    std::string status_;
    calibrate::SteadinessGate gate_;
    double last_frame_time_ = 0;
    int last_led_zone_ = -1;

    mutable std::mutex solve_mutex_;
    SolveState solve_;
    std::jthread solver_;
};

}  // namespace einstar::app
