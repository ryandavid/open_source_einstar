#pragma once

// High-level Einstar scanner API on top of a Transport (real USB or the emulator).
//
// Bring-up order is enforced by the API shape:
//   connect()            read-only identification (no trigger, laser or strobe changes)
//   read_flash()         read-only calibration blob
//   configure_*()        volatile settings (exposure, gain, trigger, laser, strobe, LEDs)
//   start_stream()       image delivery
// Destruction (or disconnect()) always switches the trigger, laser and strobe off.

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "einstar/core/error.hpp"
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
    std::uint8_t color_mode = 0;  // 0 = mono
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

enum class ButtonAction : std::uint8_t { none = 0, single_click = 1, double_click = 2, long_click = 3 };

struct DeviceState {
    std::array<ButtonAction, 3> buttons{};
};

enum class DistanceIndication : std::uint8_t { zone0 = 0, zone1 = 1, zone2 = 2 };

struct ConnectOptions {
    bool verbose_transcript = false;             // log every request/reply as hex
    std::function<void(std::string_view)> transcript_sink;  // defaults to log::debug
    int heartbeat_ms = 250;                      // 0 disables polling
    int command_retries = 5;
};

class EinstarDevice {
public:
    using GroupSink = std::function<void(usb::FrameGroup&&)>;
    using ButtonSink = std::function<void(int button, ButtonAction action)>;

    static Result<std::unique_ptr<EinstarDevice>> connect(std::unique_ptr<usb::Transport> transport,
                                                          ConnectOptions options = {});
    ~EinstarDevice();

    EinstarDevice(const EinstarDevice&) = delete;
    EinstarDevice& operator=(const EinstarDevice&) = delete;

    [[nodiscard]] const DeviceInfo& info() const { return info_; }
    [[nodiscard]] bool online() const { return online_.load(); }

    // ---- read-only ----
    Result<std::vector<std::uint8_t>> read_flash(std::uint32_t offset, std::uint32_t size);
    Result<double> temperature_c();
    Result<DeviceState> read_state();
    Result<std::uint32_t> exposure(int sensor);
    Result<std::uint16_t> gain(int sensor);

    // ---- volatile configuration ----
    Result<void> set_trigger(int mono_count, int rgb_count);
    Result<void> set_trigger_period_us(std::uint32_t period);
    Result<void> set_exposure(int sensor, std::uint32_t value);  // clamped to the sensor's range
    Result<void> set_gain(int sensor, std::uint16_t value);      // clamped to the sensor's range
    Result<void> set_laser_percent(int percent);                 // clamped to 0..100
    Result<void> set_strobe(int route, int luminance);           // route 0/1, clamped to 0..kMaxStrobeLuminance
    Result<void> set_indication(DistanceIndication distance, bool active);
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
    [[nodiscard]] usb::TransportStats transport_stats() const { return transport_->stats(); }

    void set_button_sink(ButtonSink sink);

    // Trigger off, laser/strobe off, stream stopped. Safe to call repeatedly.
    void disconnect();

private:
    explicit EinstarDevice(std::unique_ptr<usb::Transport> t, ConnectOptions o);

    template <const OpcodeInfo& Op>
        requires SafeOpcode<Op>
    Result<usb::Reply> send(std::span<const std::uint8_t> payload = {});
    Result<usb::Reply> send_checked(const OpcodeInfo& op, std::span<const std::uint8_t> payload);

    Result<void> identify();
    Result<std::string> read_string(const OpcodeInfo& op);
    void heartbeat_loop(std::stop_token st);
    void transcript(std::string_view dir, std::span<const std::uint8_t> bytes);
    [[nodiscard]] std::uint8_t next_sequence();

    std::unique_ptr<usb::Transport> transport_;
    ConnectOptions options_;
    DeviceInfo info_;
    std::mutex seq_mutex_;
    std::uint8_t seq_ = 0;
    std::atomic<bool> online_{true};
    std::atomic<bool> streaming_{false};
    int rgb_triggers_ = 0;

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
