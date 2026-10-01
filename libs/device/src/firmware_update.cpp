#include "einstar/device/firmware_update.hpp"

#include <algorithm>
#include <cctype>
#include <format>
#include <thread>

#include "einstar/usb/constants.hpp"

namespace einstar::device {
namespace {

Result<std::unique_ptr<EinstarDevice>> open_device(const TransportOpener& open) {
    auto t = open();
    if (!t) return std::unexpected(t.error());
    ConnectOptions o;
    o.heartbeat_ms = 0;  // nothing else talks to the scanner while its firmware is replaced
    return EinstarDevice::connect(std::move(*t), o);
}

// Waits for the scanner with `serial` to come back after a reboot.
Result<std::unique_ptr<EinstarDevice>> reconnect(const FlashOptions& o, const std::string& serial, std::string_view why) {
    std::this_thread::sleep_for(o.settle);
    const auto deadline = std::chrono::steady_clock::now() + o.reappear_timeout;
    Error last{Errc::not_found, "not seen"};
    for (;;) {
        auto dev = open_device(o.open);
        if (dev && (*dev)->info().serial == serial) return dev;
        last = dev ? Error{Errc::not_found, std::format("found serial {} instead", (*dev)->info().serial)} : dev.error();
        if (std::chrono::steady_clock::now() > deadline)
            return make_error(Errc::timeout, std::format("the scanner did not come back {} ({})", why, last.message));
        std::this_thread::sleep_for(o.poll_interval);
    }
}

}  // namespace

Result<std::unique_ptr<EinstarDevice>> connect_for_firmware(const TransportOpener& open) {
    auto dev = open_device(open);
    if (!dev) return dev;
    std::string upper = (*dev)->info().product_name;
    std::ranges::transform(upper, upper.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    if (!upper.starts_with(usb::kProductNamePrefix))
        return make_error(Errc::unsupported, std::format("not an Einstar (product name '{}')", (*dev)->info().product_name));
    return dev;
}

Result<FlashReport> flash_firmware(const FirmwarePackage& package, const FlashOptions& o) {
    auto say = [&](std::string_view s) {
        if (o.status) o.status(s);
    };
    if (!o.open) return make_error(Errc::invalid_argument, "no way to open the scanner");
    auto dev = connect_for_firmware(o.open);
    if (!dev) return std::unexpected(dev.error());
    FlashReport report;
    report.serial = (*dev)->info().serial;
    report.version_before = (*dev)->info().firmware;
    say(std::format("scanner {} runs {}", report.serial, report.version_before));

    // A reboot first: it clears any earlier, abandoned update's page count in the firmware.
    say("rebooting it first (clears any earlier, unfinished update)");
    if (auto r = (*dev)->reboot(); !r) return make_error(r.error().code, "reboot before the update failed: " + r.error().message);
    dev->reset();
    dev = reconnect(o, report.serial, "after the reboot");
    if (!dev) return std::unexpected(dev.error());

    // The update's first command (00/06) is sent once, without retries (a repeat would land in the update
    // as page data), so check the bulk channel first with a read-only command that can be retried.
    if (auto r = (*dev)->read_flash(0, 4096); !r)
        return make_error(r.error().code, std::format("the scanner's bulk channel does not answer after the reboot ({}); nothing was written. "
                                                      "Unplug and replug it, then try again",
                                                      r.error().message));
    say(std::format("writing {} pages into the inactive slot", package.pages));
    if (auto r = (*dev)->write_firmware(package, o.progress, o.page_pause); !r) return std::unexpected(r.error());
    dev->reset();

    say("written; the scanner switches to the new slot and restarts");
    dev = reconnect(o, report.serial, "after the update (it may still be starting: check with `einstar-firmware version`)");
    if (!dev) return std::unexpected(dev.error());
    report.version_after = (*dev)->info().firmware;
    return report;
}

}  // namespace einstar::device
