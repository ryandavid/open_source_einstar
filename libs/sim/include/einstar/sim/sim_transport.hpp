#pragma once

// Device emulator speaking the reverse-engineered Einstar protocol at the transfer level.
// Lets the device layer, pipeline and app run end-to-end without hardware, and records every
// command it receives so tests can compare against EXStar's logged sequence.

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "einstar/core/image.hpp"
#include "einstar/usb/transport.hpp"

namespace einstar::sim {

struct SimConfig {
    std::string vendor_name = "SHINING3D";
    std::string product_name = "EINSCAN10_01";
    std::array<std::uint8_t, 8> serial{0x00, 0x09, 0x01, 0x14, 0x02, 0xCF, 0x0C, 0x20};
    std::string firmware = "EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN";
    int width = 1280;
    int height = 1024;
    std::uint32_t exposure_min = 100, exposure_max = 20000;
    std::uint16_t gain_min = 16, gain_max = 1024;
    double temperature_c = 38.5;
    bool mask_replies = false;         // exercise the XOR decode path
    double packet_drop_rate = 0.0;     // fault injection on the image stream
    // The firmware drops (no reply, not executed) a request whose sequence number repeats the previous
    // one on its channel; the command channel exempts 0xFE / 0xFF (docs/firmware.md 5). These are the
    // last numbers an earlier session left behind.
    std::optional<std::uint8_t> previous_command_sequence, previous_bulk_sequence;
    // Fault injection: reply to (group << 8 | opcode) with this status instead of executing it.
    std::map<std::uint16_t, std::uint8_t> status_override;
    std::uint32_t seed = 7;
};

struct ReceivedCommand {
    std::uint8_t sequence, group, opcode;
    std::vector<std::uint8_t> payload;
};

// Produces the image for (sensor, frame id). Sensor 2 is the RGB Bayer sensor.
using FrameProvider = std::function<void(int sensor, std::uint32_t frame_id, ImageU8& out)>;

class SimTransport final : public usb::Transport {
public:
    explicit SimTransport(SimConfig config = {});
    ~SimTransport() override;

    void set_frame_provider(FrameProvider p);
    void set_flash(std::uint32_t offset, std::span<const std::uint8_t> data);
    void press_button(int button, std::uint8_t action = 1);

    [[nodiscard]] std::vector<ReceivedCommand> received() const;
    // Requests dropped as repeats of the previous sequence number (see SimConfig).
    [[nodiscard]] std::uint64_t dropped_repeats() const;
    [[nodiscard]] bool dangerous_command_seen() const { return dangerous_seen_.load(); }

    // Emulated device state (for assertions).
    struct State {
        int mono_triggers = 1, rgb_triggers = 1;
        std::uint32_t trigger_period_us = 200000;
        std::array<std::uint32_t, 3> exposure{1000, 1000, 1000};
        std::array<std::uint16_t, 3> gain{100, 100, 100};
        int laser = 0;
        std::array<int, 2> strobe{0, 0};
        int indication_distance = 0, indication_active = 0;
    };
    [[nodiscard]] State state() const;

    // Snapshot hook that survives the transport (the device owns and destroys it on disconnect).
    struct Observer {
        std::mutex mutex;
        State state;
        std::vector<ReceivedCommand> received;
    };
    [[nodiscard]] std::shared_ptr<Observer> observer() const { return observer_; }

    // Transport interface
    [[nodiscard]] std::string description() const override { return "emulated Einstar"; }
    Result<std::vector<std::uint8_t>> command(std::span<const std::uint8_t> request, std::size_t reply_capacity,
                                              unsigned timeout_ms) override;
    Result<std::vector<std::uint8_t>> bulk(std::span<const std::uint8_t> request, std::size_t reply_capacity,
                                           unsigned timeout_ms) override;
    Result<void> reset_command_pipe() override { return {}; }
    Result<void> reset_bulk_pipe() override { return {}; }
    Result<void> start_stream(PacketHandler handler) override;
    void stop_stream() override;
    [[nodiscard]] usb::TransportStats stats() const override;

private:
    std::vector<std::uint8_t> handle(std::span<const std::uint8_t> request);
    void stream_loop(std::stop_token st);
    void emit_frame(int sensor, std::uint32_t frame_id, std::uint64_t timestamp, const ImageU8& img);

    SimConfig config_;
    mutable std::mutex mutex_;
    State state_;
    std::vector<std::uint8_t> flash_;
    std::array<std::uint8_t, 3> buttons_{};
    std::vector<ReceivedCommand> received_;
    std::optional<std::uint8_t> last_command_seq_, last_bulk_seq_;
    std::uint64_t dropped_repeats_ = 0;
    std::atomic<bool> dangerous_seen_{false};
    std::shared_ptr<Observer> observer_ = std::make_shared<Observer>();

    FrameProvider provider_;
    PacketHandler handler_;
    std::jthread stream_thread_;
    std::mt19937 rng_;
    std::uint64_t virtual_time_us_ = 0;
    std::atomic<std::uint64_t> packets_{0}, bytes_{0}, commands_{0};
};

}  // namespace einstar::sim
