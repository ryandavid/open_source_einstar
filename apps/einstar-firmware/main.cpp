// einstar-firmware: report the scanner's firmware version, check an update package, and flash one.
//
// The package is what firmware/ builds (firmware/build/EN/*_IAP.img) or EXStar ships
// (EXStar.app/Contents/MacOS/fabu_UPDATE/Configure/*_IAP.img). Flashing follows docs/firmware.md 4 and
// libs/device/include/einstar/device/firmware_update.hpp.

#include <cstdio>
#include <format>
#include <iostream>
#include <memory>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "einstar/core/log.hpp"
#include "einstar/device/firmware_update.hpp"
#include "einstar/sim/sim_transport.hpp"
#include "einstar/usb/transport.hpp"

using namespace einstar;

namespace {

int usage() {
    std::println(stderr,
                 "usage: einstar-firmware version [--emulator]\n"
                 "       einstar-firmware inspect <package.img>\n"
                 "       einstar-firmware flash <package.img> [--yes] [--emulator]\n"
                 "\n"
                 "  version   what the scanner reports (command 00/05), its serial and USB ids\n"
                 "  inspect   check an update package offline: pages, check bytes, FX3 image, FPGA bitstream\n"
                 "  flash     reboot the scanner, write the package into its inactive slot, and read the new version\n"
                 "  --emulator  run against the built-in emulated scanner instead of the USB device\n"
                 "  --yes       do not ask for confirmation\n"
                 "\n"
                 "Our build's package: firmware/build/EN/EinScan10_01_SC130_OPN_V2.10_FPGA_V3.7_EN_IAP.img\n"
                 "EXStar's (to go back): /Applications/EXStar.app/Contents/MacOS/fabu_UPDATE/Configure/*_EN_IAP.img");
    return 2;
}

bool has_flag(std::span<char*> args, std::string_view f) {
    for (auto* a : args)
        if (f == a) return true;
    return false;
}

std::vector<std::string_view> positional(std::span<char*> args) {
    std::vector<std::string_view> out;
    for (auto* a : args)
        if (a[0] != '-') out.emplace_back(a);
    return out;
}

// How to reach the scanner: the first Shining3D device on USB (the same port first after a reboot), or one
// emulated scanner that keeps its state across reboots.
device::TransportOpener make_opener(bool emulator) {
    if (emulator) {
        auto sim = std::make_shared<sim::SimDevice>();
        return [sim] { return sim->connect(); };
    }
    auto first = std::make_shared<std::optional<usb::UsbDeviceInfo>>();
    return [first]() -> Result<std::unique_ptr<usb::Transport>> {
        if (*first) return usb::reopen_libusb(**first);
        auto devices = usb::enumerate_devices();
        if (!devices) return std::unexpected(devices.error());
        if (devices->empty()) return make_error(Errc::not_found, "no Shining3D device (vid 3267) on USB");
        *first = devices->front();
        return usb::open_libusb(devices->front());
    };
}

void print_version(std::string_view text) {
    const auto v = device::parse_firmware_version(text);
    std::println("firmware        {}", v.text);
    if (v.fx3.empty()) return;
    std::println("                {}: FX3 application {}, FPGA {}, sensor {}, language {}",
                 v.open_build() ? "open build (firmware/)" : v.tag == "FX3" ? "Shining3D's firmware" : std::format("unknown build '{}'", v.tag),
                 v.fx3, v.fpga, v.sensor, v.language);
}

int version_cmd(bool emulator) {
    auto dev = device::connect_for_firmware(make_opener(emulator));
    if (!dev) {
        std::println(stderr, "{}", dev.error().message);
        return 1;
    }
    const auto& i = (*dev)->info();
    std::println("scanner         {} / {}", i.vendor_name, i.product_name);
    std::println("serial          {}", i.serial);
    print_version(i.firmware);
    std::println("ids             reported VID {:04x} PID {:04x} (the USB descriptor's PID is 0003)", i.vendor_id, i.product_id);
    return 0;
}

void print_package(const std::string& path, const device::FirmwarePackage& p) {
    std::println("package         {}", path);
    std::println("pages           {} x 4096 (+ check byte each), all check bytes valid; {} data bytes", p.pages, p.data_size);
    std::println("FX3 image       {} bytes, {} sections, entry {:#010x}, checksum {:#010x} (valid)", p.fx3_image_size, p.fx3_sections,
                 p.fx3_entry, p.fx3_checksum);
    std::println("FPGA bitstream  {} bytes at +0x40000", p.fpga_size);
    std::println("CRC-32          {:08x}", p.crc32);
}

int inspect_cmd(const std::string& path) {
    auto p = device::load_firmware_package(path);
    if (!p) {
        std::println(stderr, "{}", p.error().message);
        return 1;
    }
    print_package(path, *p);
    return 0;
}

int flash_cmd(const std::string& path, bool yes, bool emulator) {
    auto pkg = device::load_firmware_package(path);
    if (!pkg) {
        std::println(stderr, "{}", pkg.error().message);
        return 1;
    }
    print_package(path, *pkg);
    auto open = make_opener(emulator);
    {
        auto dev = device::connect_for_firmware(open);
        if (!dev) {
            std::println(stderr, "{}", dev.error().message);
            return 1;
        }
        std::println("scanner         {} (serial {})", (*dev)->info().product_name, (*dev)->info().serial);
        print_version((*dev)->info().firmware);
    }
    std::println("\nThis replaces the scanner's firmware. It is written into the slot the scanner does not boot from and\n"
                 "only switched to once every page verified, so an interrupted write leaves the current firmware booting.\n"
                 "What cannot be undone from here: a new firmware that is broken (it may not come back on USB). Going\n"
                 "back to EXStar's firmware means flashing its package the same way. Keep the scanner connected and\n"
                 "powered, and keep EXStar and other scanner tools closed, until this finishes.");
    if (!yes) {
        std::print("\nType FLASH to continue: ");
        std::fflush(stdout);
        std::string answer;
        if (!std::getline(std::cin, answer) || answer != "FLASH") {
            std::println("not flashed");
            return 1;
        }
    }
    device::FlashOptions o;
    o.open = open;
    o.status = [](std::string_view s) { std::println("- {}", s); };
    int last_percent = -1;
    o.progress = [&](int done, int pages) {
        const int percent = 100 * done / pages;
        if (percent / 10 != last_percent / 10 || done == pages) std::println("  {:3}% ({} / {} pages)", percent, done, pages);
        last_percent = percent;
    };
    if (emulator) o.settle = std::chrono::milliseconds(300);
    auto r = device::flash_firmware(*pkg, o);
    if (!r) {
        std::println(stderr, "FAILED: {}", r.error().message);
        return 1;
    }
    std::println("\nflashed. before: {}\n         now:    {}", r->version_before, r->version_after);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    log::set_level(log::Level::warn);
    const std::string_view cmd = argv[1];
    const std::span<char*> rest(argv + 2, static_cast<std::size_t>(argc - 2));
    const auto pos = positional(rest);
    const bool emulator = has_flag(rest, "--emulator");
    if (cmd == "version") return version_cmd(emulator);
    if (cmd == "inspect" && pos.size() == 1) return inspect_cmd(std::string(pos[0]));
    if (cmd == "flash" && pos.size() == 1) return flash_cmd(std::string(pos[0]), has_flag(rest, "--yes"), emulator);
    return usage();
}
