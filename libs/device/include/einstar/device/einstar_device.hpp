#pragma once

// High-level Einstar scanner API on top of a Transport (real USB or the emulator).
//
// Bring-up order is enforced by the API shape:
//   connect()            read-only identification (no trigger, laser or strobe changes)
//   read_flash()         read-only calibration blob
//   configure_*()        volatile settings (exposure, gain, trigger, laser, strobe, LEDs)
//   start_stream()       image delivery
// Destruction (or disconnect()) always switches the trigger, laser and strobe off.
//
// The firmware leaves the bus on its own (reboots after a suspend or VBUS loss,
// and a full restart after a cleared image-endpoint halt; docs/firmware.md 5). With ConnectOptions::reopen
// set, the heartbeat then reopens the scanner, checks its serial, replays every setting made through this
// API (the restart resets the FPGA and sensors), sends ClearState and resumes the stream, as EXStar does.
// While offline, commands fail at once with Errc::disconnected.

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "einstar/core/error.hpp"
#include "einstar/device/firmware_package.hpp"
#include "einstar/device/opcodes.hpp"
#include "einstar/usb/command.hpp"
#include "einstar/usb/stream.hpp"
#include "einstar/usb/transport.hpp"

namespace einstar::device {

struct SensorInfo {
    int width = 0;
    int height = 0;
    std::uint32_t exposure_min = 0, exposure_max = 0;
    std::uint16_t gain_min = 0, gain_max = 0;
    std::uint8_t pixel_bits = 0;
    // No colour flag: the firmware's colour-mode reply (10/5D) is an unwritten buffer byte for the
    // payload the protocol uses. Sensor 2 is the colour camera.
};

struct DeviceInfo {
    std::string vendor_name;
    std::string product_name;
    std::string serial;
    std::string firmware;
    std::uint16_t vendor_id = 0;
    std::uint16_t product_id = 0;
    int sensor_count = 0;
    std::array<SensorInfo, 3> sensors{};
};

// Codes in the device-state reply, cleared once read (measured on the scanner). BUTTON1 (start / pause)
// reports all three. BUTTON0 / BUTTON2 (brightness - / +) report a double click as two singles, and a
// long press as 3 followed 1-2 s later by 2, so 2 is only a double click on BUTTON1.
enum class ButtonAction : std::uint8_t { none = 0, single_click = 1, double_click = 2, long_click = 3 };

// What the scanner's buttons do, as in EXStar (its device log: every BUTTON1 click switches the light
// sources off / on for pause / start, BUTTON0 and BUTTON2 clicks step the camera brightness down / up).
// Double and long clicks do nothing.
enum class ButtonCommand : std::uint8_t { none, toggle_scan, brightness_down, brightness_up };
[[nodiscard]] ButtonCommand button_command(int button, ButtonAction action);

// Camera brightness ladder: each level ~8% more exposure x gain than the one below; the middle level is
// EXStar's scan default (exposure 4400, gain 120). Exposure moves first (up to 5600, to bound motion
// blur), then gain.
inline constexpr int kBrightnessLevels = 21;
inline constexpr int kDefaultBrightness = 10;
struct ExposureGain {
    std::uint32_t exposure = 0;
    std::uint16_t gain = 0;
};
[[nodiscard]] ExposureGain brightness_level(int level);  // level clamped to 0..kBrightnessLevels-1

struct DeviceState {
    std::array<ButtonAction, 3> buttons{};
    std::array<std::uint8_t, 14> raw{};  // reply bytes 9..22, all the firmware writes (docs/firmware.md 5.1)
};

// 10/62 DISTANCE. The firmware maps 0/1/2 to FPGA LED/laser modes 4/1/2 (docs/firmware.md 5): the top LED
// shows red / green / blue (measured on the scanner, idle and scanning).
enum class DistanceIndication : std::uint8_t { zone0 = 0, zone1 = 1, zone2 = 2 };

struct ConnectOptions {
    bool verbose_transcript = false;             // log every request/reply as hex
    std::function<void(std::string_view)> transcript_sink;  // defaults to log::debug
    int heartbeat_ms = 250;                      // 0 disables polling (buttons, offline detection, reconnection)
    int command_retries = 5;
    // Opens the scanner again after it left the bus (usb::reopen_libusb, or SimDevice::connect). Empty: no
    // reconnection, the device stays offline. Needs the heartbeat.
    std::function<Result<std::unique_ptr<usb::Transport>>()> reopen;
    int reconnect_interval_ms = 500;
    // Sequence number of the first request. The firmware silently drops a request whose number repeats
    // the previous one on that channel, including the last request of an earlier session
    // (docs/firmware.md 5), so a session starts at a random number; set it only for tests.
    std::optional<std::uint8_t> first_sequence;
};

class EinstarDevice {
public:
    // Groups carry extended (non-wrapping) ids. Their `timestamp` is microseconds on the trigger
    // schedule: group id x (trigger period / IR triggers per cycle), accumulated across period changes.
    // The scanner's own header field is a constant (docs/protocol-transport.md 3.6), so it is replaced.
    using GroupSink = std::function<void(usb::FrameGroup&&)>;
    using ButtonSink = std::function<void(int button, ButtonAction action)>;

    static Result<std::unique_ptr<EinstarDevice>> connect(std::unique_ptr<usb::Transport> transport,
                                                          ConnectOptions options = {});
    ~EinstarDevice();

    EinstarDevice(const EinstarDevice&) = delete;
    EinstarDevice& operator=(const EinstarDevice&) = delete;

    [[nodiscard]] const DeviceInfo& info() const { return info_; }
    [[nodiscard]] bool online() const { return online_.load(); }
    [[nodiscard]] int reconnects() const { return reconnects_.load(); }  // successful reattachments

    // ---- read-only ----
    Result<std::vector<std::uint8_t>> read_flash(std::uint32_t offset, std::uint32_t size);
    // The ADT7420 on the scanner's board (10/50). Errc::unsupported when it reads exactly 0x0000: on the
    // tested unit it does so always (idle, streaming, vendor and open firmware), the signature of an I2C
    // segment with nothing answering (all-zero bits, which also read as acknowledgements), not 0 degC.
    Result<double> temperature_c();
    Result<DeviceState> read_state();
    // Sensors 0 and 1 (the IR pair) share one exposure register in the scanner: reading or setting
    // either one reads or sets both. Sensor 2 has its own (docs/firmware.md 5).
    Result<std::uint32_t> exposure(int sensor);
    Result<std::uint16_t> gain(int sensor);

    // ---- calibration write (the only persistent write this API can make) ----
    // Rewrites the quick-calibration section of the calibration blob in flash pages 0-1, as EXStar's
    // quick calibration does (docs/protocol-device.md 3.12). `blob` is the complete new 6568-byte blob;
    // it is refused unless it differs from the scanner's current blob only inside the quick section
    // (kQuickSectionBegin..kQuickSectionEnd) and both carry the section's tag. Before anything is written
    // the current pages 0-1 (8192 bytes) are handed to `save_backup`, which must succeed. Each changed
    // page is written (the firmware erases its 4 KB sector and programs it, reporting no errors), read
    // back and compared; a page that does not verify is written once more, then the old pages are put
    // back. The stream must be stopped. Returns the pages written.
    static constexpr std::size_t kCalibrationBlobSize = 6568;
    static constexpr std::size_t kQuickSectionBegin = 0x39B, kQuickSectionEnd = 0x12BC;
    static constexpr std::size_t kCalibrationPagesSize = 8192;
    using BackupSink = std::function<Result<void>(std::span<const std::uint8_t> pages)>;
    Result<std::vector<int>> write_calibration_blob(std::span<const std::uint8_t> blob, const BackupSink& save_backup);
    // Puts back pages 0-1 saved by write_calibration_blob's backup (e.g. after an interrupted write).
    // Refused unless the backup carries the quick-calibration section. Returns the pages rewritten.
    Result<std::vector<int>> restore_calibration_pages(std::span<const std::uint8_t> backup);

    // ---- firmware (docs/firmware.md 4; use device::flash_firmware, which wraps these safely) ----
    // Reboots the scanner (00/08): it replies, then resets 25 ms later and re-enumerates. This object is
    // offline afterwards; open the scanner again.
    Result<void> reboot();
    // Writes `package` into the scanner's inactive A/B slot as EXStar's updateFirmware does (bulk 00/06: the
    // data size, then one packet per page with a 50 ms pause before each). After the last page the firmware
    // points its boot record at that slot and resets within ~1 s; this object is then offline. Any failure
    // abandons the update (the old firmware keeps booting). Every packet is sent exactly once: the firmware
    // takes a repeated page as the next one. The scanner must have been rebooted since any earlier update
    // attempt -- the firmware keeps an abandoned update's page count, and a new update would then be written
    // shifted and booted (docs/firmware.md 5.1). The stream must be stopped.
    using FirmwareProgress = std::function<void(int pages_done, int pages)>;
    Result<void> write_firmware(const FirmwarePackage& package, const FirmwareProgress& progress = {},
                                std::chrono::milliseconds page_pause = std::chrono::milliseconds(50));

    // ---- volatile configuration ----
    Result<void> set_trigger(int mono_count, int rgb_count);
    // kMinTriggerPeriodUs..kMaxTriggerPeriodUs, else invalid_argument (the firmware would ignore it
    // and still reply OK).
    Result<void> set_trigger_period_us(std::uint32_t period);
    Result<void> set_exposure(int sensor, std::uint32_t value);  // clamped to the sensor's range; 0 and 1 shared
    // Clamped to the sensor's range. Percent; reads back rounded (register = percent x 32 / 100). If the
    // firmware's write of the low byte fails, its retry writes the high byte there (docs/firmware.md 5.1):
    // callers that must be sure read the gain back.
    Result<void> set_gain(int sensor, std::uint16_t value);
    Result<void> set_laser_percent(int percent);                 // clamped to 0..100
    Result<void> set_strobe(int route, int luminance);           // route 0/1, clamped to 0..kMaxStrobeLuminance
    Result<void> set_indication(DistanceIndication distance);
    Result<void> clear_state();

    // EXStar's scan configuration: 3 IR triggers per cycle, no RGB, 68 ms period.
    Result<void> configure_scan_mode(std::uint32_t period_us = 68000);
    // Texture capture: 1 IR + 1 RGB trigger, 100 ms period.
    Result<void> configure_texture_mode(std::uint32_t period_us = 100000);

    // ---- streaming ----
    Result<void> start_stream(GroupSink sink);
    void stop_stream();
    [[nodiscard]] usb::StreamStats stream_stats() const;
    [[nodiscard]] usb::GroupStats group_stats() const;
    [[nodiscard]] usb::TransportStats transport_stats() const;

    void set_button_sink(ButtonSink sink);

    // Trigger off, laser/strobe off, stream stopped. Safe to call repeatedly.
    void disconnect();

private:
    explicit EinstarDevice(std::unique_ptr<usb::Transport> t, ConnectOptions o);

    template <const OpcodeInfo& Op>
        requires SafeOpcode<Op>
    Result<usb::Reply> send(std::span<const std::uint8_t> payload = {});
    Result<usb::Reply> send_checked(const OpcodeInfo& op, std::span<const std::uint8_t> payload, int attempts = 0);
    // The send itself, after the guard. Called directly only by write_user_page (10/58 on pages 0-1).
    Result<usb::Reply> send_unguarded(const OpcodeInfo& op, std::span<const std::uint8_t> payload, int attempts = 0);
    // Writes one 4 KB calibration page (0 or 1 only), reads it back and compares.
    Result<void> write_user_page(int page, std::span<const std::uint8_t> data);

    Result<void> identify(DeviceInfo& out);
    Result<std::string> read_string(const OpcodeInfo& op);
    void heartbeat_loop(std::stop_token st);
    void mark_offline(std::string_view reason);
    void reattach();
    usb::Transport::PacketHandler packet_handler();
    void transcript(std::string_view dir, std::span<const std::uint8_t> bytes);
    [[nodiscard]] std::uint8_t next_sequence();
    [[nodiscard]] std::shared_ptr<usb::Transport> transport() const;
    void set_transport(std::shared_ptr<usb::Transport> t);

    // Everything set through the API, replayed after a reconnection.
    struct Settings {
        std::optional<std::pair<int, int>> trigger;       // mono, rgb
        std::optional<std::uint32_t> trigger_period;
        std::array<std::optional<std::uint32_t>, 2> exposure;  // IR pair (sensors 0, 1), colour (sensor 2)
        std::array<std::optional<std::uint16_t>, 3> gain;
        std::optional<int> laser;
        std::array<std::optional<int>, 2> strobe;
        std::optional<DistanceIndication> indication;
    };
    template <typename F>
    void remember(F&& update) {
        std::lock_guard lock(settings_mutex_);
        update(settings_);
    }

    mutable std::mutex transport_mutex_;
    std::shared_ptr<usb::Transport> transport_;  // (guarded; null while offline between reopen attempts)
    ConnectOptions options_;
    DeviceInfo info_;
    std::mutex seq_mutex_;
    std::uint8_t seq_ = 0;
    std::atomic<bool> online_{true};
    std::atomic<bool> streaming_{false};
    std::atomic<int> reconnects_{0};
    std::atomic<bool> updating_{false};  // a firmware update owns the bulk channel
    std::atomic<std::thread::id> heartbeat_id_{};  // the heartbeat's own commands pass while offline
    std::string foreign_serial_;                   // another scanner found while reconnecting (heartbeat only)
    std::atomic<int> rgb_triggers_{0};  // (also set by the heartbeat when it replays the trigger)
    std::atomic<int> mono_triggers_{1};
    std::atomic<std::uint32_t> trigger_period_us_{68000};
    [[nodiscard]] std::uint64_t group_interval_us() const {
        return trigger_period_us_.load() / static_cast<std::uint32_t>(std::max(1, mono_triggers_.load()));
    }
    std::mutex settings_mutex_;
    Settings settings_;

    std::mutex stream_control_mutex_;  // start / stop, against the heartbeat restarting the stream
    std::mutex stream_mutex_;
    std::unique_ptr<usb::FrameAssembler> frames_;
    std::unique_ptr<usb::GroupAssembler> groups_;

    std::mutex button_mutex_;
    ButtonSink button_sink_;
    std::jthread heartbeat_;
};

// Formats the 8 serial bytes as EXStar does (16 upper-case hex digits).
[[nodiscard]] std::string format_serial(std::span<const std::uint8_t> bytes);
[[nodiscard]] std::uint8_t sensor_mask(int sensor);  // 0 -> 1, 1 -> 2, 2 -> 4

}  // namespace einstar::device
