#pragma once

// A connected scanner (real over USB, or the emulator when none is attached) plus the live
// scan pipeline fed by it.

#include <functional>
#include <memory>
#include <string>

#include "einstar/device/einstar_device.hpp"
#include "einstar/pipeline/scan_pipeline.hpp"

namespace einstar::app {

// EXStar's own scan-mode values (from its session log) as defaults.
struct ScanSettings {
    int brightness = device::kDefaultBrightness;  // sets exposure and gain (see device::brightness_level)
    int exposure = 4400;
    int gain = 120;
    int laser_percent = 100;
    int strobe = 6000;
    std::uint32_t trigger_period_us = 68000;
};

class Session {
public:
    using UpdateSink = std::function<void(pipeline::LiveUpdate&&)>;
    using ButtonSink = std::function<void(int, device::ButtonAction)>;

    // Opens the first attached scanner, or the emulator if `force_emulator` or none is found.
    static Result<std::unique_ptr<Session>> open(bool force_emulator, UpdateSink updates, ButtonSink buttons);
    ~Session();

    [[nodiscard]] const std::string& description() const { return description_; }
    [[nodiscard]] bool emulated() const { return emulated_; }
    [[nodiscard]] const device::DeviceInfo& info() const { return device_->info(); }
    [[nodiscard]] bool scanning() const { return scanning_; }

    Result<void> start_scan(const ScanSettings& s);
    void stop_scan();
    Result<void> apply(const ScanSettings& s);
    void reset_model() { pipeline_->reset_model(); }
    [[nodiscard]] pipeline::ScanPipeline& pipeline() { return *pipeline_; }
    Result<double> temperature() { return device_->temperature_c(); }
    void set_distance_indication(float mean_depth_mm);

private:
    Session() = default;

    std::unique_ptr<device::EinstarDevice> device_;
    std::unique_ptr<pipeline::ScanPipeline> pipeline_;
    std::string description_;
    bool emulated_ = false;
    bool scanning_ = false;
    int last_zone_ = -1;
    std::shared_ptr<void> emulator_state_;  // keeps the emulator scene alive
};

}  // namespace einstar::app
