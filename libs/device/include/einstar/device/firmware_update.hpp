#pragma once

// Flashing the scanner's firmware (FX3 application + FPGA bitstream) from an update package, the way the
// firmware expects it (firmware/src/update.c, docs/firmware.md 4):
//
//   1. connect and identify (product name, serial, current version);
//   2. reboot (00/08) and reconnect: an update abandoned earlier leaves the firmware's page count set, and a
//      new update would then be written shifted and booted (docs/firmware.md 5.1); a reboot clears it;
//   3. write the package into the inactive A/B slot (00/06); the firmware then switches its boot record to
//      that slot and resets;
//   4. reconnect and read the version the scanner now reports.
//
// The previous firmware stays in the other slot, but no command switches back to it: rolling back means
// flashing the previous package (e.g. EXStar's) the same way.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <string_view>

#include "einstar/core/error.hpp"
#include "einstar/device/einstar_device.hpp"
#include "einstar/device/firmware_package.hpp"
#include "einstar/usb/transport.hpp"

namespace einstar::device {

using TransportOpener = std::function<Result<std::unique_ptr<usb::Transport>>()>;

// Connects for firmware work: identification only, no heartbeat, and a check that it is an Einstar.
[[nodiscard]] Result<std::unique_ptr<EinstarDevice>> connect_for_firmware(const TransportOpener& open);

struct FlashOptions {
    TransportOpener open;                                  // opens the scanner (again, after each reboot)
    std::function<void(std::string_view)> status;          // one line per step
    EinstarDevice::FirmwareProgress progress;              // pages written
    std::chrono::milliseconds page_pause{50};              // before each page, as EXStar
    std::chrono::milliseconds settle{1500};                // after a reboot, before looking for the scanner
    std::chrono::milliseconds reappear_timeout{60000};     // for it to come back
    std::chrono::milliseconds poll_interval{500};
};

struct FlashReport {
    std::string serial;
    std::string version_before;
    std::string version_after;
};

[[nodiscard]] Result<FlashReport> flash_firmware(const FirmwarePackage& package, const FlashOptions& options);

}  // namespace einstar::device
