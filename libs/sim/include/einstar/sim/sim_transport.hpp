#pragma once

// Device emulator speaking the Einstar protocol at the transfer level, modelled on the scanner's firmware
// (firmware/src, docs/firmware.md): the same payload-length checks, status codes, reply sizes, register
// quirks (shared IR exposure, gain rounding through the sensor register, laser halving, strobe wrap) and
// the firmware's restart paths. Lets the device layer, pipeline and app run end-to-end without hardware,
// and records every command it receives so tests can compare against EXStar's logged sequence.
//
// SimDevice is the scanner; SimTransport is one USB connection to it (what open_libusb gives for the real
// one). A restart takes the device off the bus: its connections then fail with Errc::disconnected and a
// new one has to be made with SimDevice::connect(), as with the real scanner.

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "einstar/core/image.hpp"
#include "einstar/usb/transport.hpp"

namespace einstar::sim {

struct SimConfig {
    // What the firmware reports (commands 00/00, 00/01, 00/04, 00/05 and the constant info commands).
    std::string vendor_name = "Shining3D";
    std::string product_name = "EinScan10_01";
    std::array<std::uint8_t, 8> serial{0x00, 0x09, 0x01, 0x14, 0x02, 0xCF, 0x0C, 0x20};
    std::string firmware = "EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN";
    int width = 1280;
    int height = 1024;
    std::uint32_t exposure_min = 1, exposure_max = 10000;
    std::uint16_t gain_min = 1, gain_max = 800;
    double temperature_c = 38.5;
    // FPGA state bit 17 (00/07 reply byte 20; meaning unknown). The firmware's restart after a cleared
    // image-endpoint halt waits for it, so it is set by default to exercise that path.
    bool state_bit17 = true;
    // How long a restart keeps the device off the bus (the scanner: seconds; kept short for tests).
    std::chrono::milliseconds restart_time{200};
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

class SimTransport;

class SimDevice : public std::enable_shared_from_this<SimDevice> {
public:
    // Create with std::make_shared (connections keep the device alive).
    explicit SimDevice(SimConfig config = {});

    // A new USB connection. Fails with Errc::not_found while the device is restarting.
    Result<std::unique_ptr<usb::Transport>> connect();

    void set_frame_provider(FrameProvider p);
    void set_flash(std::uint32_t offset, std::span<const std::uint8_t> data);
    void press_button(int button, std::uint8_t action = 1);

    // ---- fault injection ----
    // The image endpoint halts. The connection clears the halt as LibusbTransport does, which arms the
    // firmware's full restart on the next 00/07 (when state bit 17 is set).
    void stall_image_endpoint();
    // Any of the firmware's reboot paths (VBUS, suspend + 10 s, 00/08): off
    // the bus for restart_time, then power-on state.
    void reboot();

    // ---- observation ----
    [[nodiscard]] std::vector<ReceivedCommand> received() const;
    [[nodiscard]] std::uint64_t dropped_repeats() const;   // requests dropped as repeated sequence numbers
    [[nodiscard]] bool dangerous_command_seen() const { return dangerous_seen_.load(); }
    [[nodiscard]] int restarts() const { return restarts_.load(); }
    [[nodiscard]] bool on_bus() const;

    // Emulated state, decoded from the FPGA registers and sensor gain registers as the firmware's read
    // commands would report it.
    struct State {
        int mono_triggers = 0, rgb_triggers = 0;
        std::uint32_t trigger_period_us = 0;
        std::array<std::uint32_t, 3> exposure{};   // sensors 0 and 1 share one field
        std::array<std::uint16_t, 3> gain{};       // percent, as 10/26 reads it back (rounded)
        int laser = 0;                             // percent, as 10/67 reads it back (even)
        std::array<int, 2> strobe{};               // routes 0 and 1, as 10/6F reads them back
        int indication_distance = -1;              // 10/62 DISTANCE, -1 if the FPGA mode is not one of its
        std::uint32_t control = 0;                 // FPGA register 12 (10/7B writes 4)
    };
    [[nodiscard]] State state() const;

private:
    friend class SimTransport;

    struct Reply {
        std::vector<std::uint8_t> bytes;
        bool restart = false;   // the firmware restarts right after this command (the reply is lost)
        bool reboot = false;
    };
    // `generation` identifies the connection; stale connections see the device as gone.
    [[nodiscard]] bool alive(std::uint64_t generation) const;
    [[nodiscard]] bool alive_locked(std::uint64_t generation) const;  // (mutex_ held)
    Result<std::vector<std::uint8_t>> command(std::uint64_t generation, std::span<const std::uint8_t> request,
                                              std::size_t reply_capacity);
    Result<std::vector<std::uint8_t>> bulk(std::uint64_t generation, std::span<const std::uint8_t> request,
                                           std::size_t reply_size);
    Reply handle_command(std::span<const std::uint8_t> request);          // (mutex_ held)
    std::vector<std::uint8_t> handle_bulk(std::span<const std::uint8_t> request);  // (mutex_ held)
    void restart_locked(bool reboot);
    void power_on_locked();
    [[nodiscard]] State state_locked() const;

    // Stream side, called from a connection's stream thread.
    bool take_image_halt(std::uint64_t generation);   // a pending halt, which the connection then clears
    void clear_image_halt();
    bool stream_cycle(std::uint64_t generation, const usb::Transport::PacketHandler& handler, std::stop_token st,
                      std::atomic<std::uint64_t>& packets, std::atomic<std::uint64_t>& bytes);
    void emit_frame(int sensor, std::uint32_t frame_id, std::uint64_t timestamp, const ImageU8& img,
                    const usb::Transport::PacketHandler& handler, std::atomic<std::uint64_t>& packets,
                    std::atomic<std::uint64_t>& bytes);

    SimConfig config_;
    mutable std::mutex mutex_;
    std::uint64_t generation_ = 1;
    std::chrono::steady_clock::time_point off_bus_until_{};
    std::array<std::uint32_t, 32> fpga_{};              // FPGA registers (I2C 0x20)
    std::array<std::uint16_t, 3> gain_register_{};      // sensor registers 0x3E08/0x3E09
    std::array<std::uint8_t, 3> buttons_{};
    bool ep83_halt_cleared_ = false;
    bool image_halted_ = false;
    std::vector<std::uint8_t> flash_;
    std::vector<ReceivedCommand> received_;
    std::optional<std::uint8_t> last_command_seq_, last_bulk_seq_;
    std::uint64_t dropped_repeats_ = 0;
    std::atomic<bool> dangerous_seen_{false};
    std::atomic<int> restarts_{0};

    FrameProvider provider_;
    std::mt19937 rng_;
    std::uint32_t frame_id_ = 0;
    std::uint64_t virtual_time_us_ = 0;
};

class SimTransport final : public usb::Transport {
public:
    SimTransport(std::shared_ptr<SimDevice> device, std::uint64_t generation);
    ~SimTransport() override;

    [[nodiscard]] std::string description() const override { return "emulated Einstar"; }
    Result<std::vector<std::uint8_t>> command(std::span<const std::uint8_t> request, std::size_t reply_capacity,
                                              unsigned timeout_ms) override;
    Result<std::vector<std::uint8_t>> bulk(std::span<const std::uint8_t> request, std::size_t reply_size,
                                           unsigned timeout_ms) override;
    Result<void> reset_command_pipe() override { return {}; }
    Result<void> reset_bulk_pipe() override { return {}; }
    Result<void> start_stream(PacketHandler handler) override;
    void stop_stream() override;
    [[nodiscard]] usb::TransportStats stats() const override;

private:
    void stream_loop(std::stop_token st);

    std::shared_ptr<SimDevice> device_;
    std::uint64_t generation_;
    PacketHandler handler_;
    std::jthread stream_thread_;
    std::atomic<std::uint64_t> packets_{0}, bytes_{0}, commands_{0}, stalls_{0};
};

// Convenience: a fresh emulated scanner and a connection to it.
struct SimScanner {
    std::shared_ptr<SimDevice> device;
    std::unique_ptr<usb::Transport> transport;
};
[[nodiscard]] SimScanner make_sim_scanner(SimConfig config = {});

}  // namespace einstar::sim
