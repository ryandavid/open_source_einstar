#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "einstar/device/firmware_update.hpp"
#include "einstar/sim/sim_transport.hpp"
#include "einstar/usb/command.hpp"
#include "einstar/usb/constants.hpp"

using namespace einstar;
using namespace std::chrono_literals;

namespace {

constexpr std::uint32_t kSlotA = 0x040000, kSlotB = 0x1C0000;

void put32(std::vector<std::uint8_t>& b, std::size_t o, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) b[o + static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(v >> (8 * i));
}

// A flash slot laid out as firmware/tools/mkpackage builds it: a valid FX3 boot image at +0 (one section),
// a stand-in bitstream at +0x40000, 0xFF elsewhere.
std::vector<std::uint8_t> make_slot(std::uint32_t seed, int pages = 320) {
    std::vector<std::uint8_t> slot(static_cast<std::size_t>(pages) * 4096, 0xFF);
    std::mt19937 rng(seed);
    slot[0] = 'C', slot[1] = 'Y', slot[2] = 0x1C, slot[3] = 0xB0;
    const std::uint32_t words = 3000, addr = 0x40003000;
    put32(slot, 4, words);
    put32(slot, 8, addr);
    std::uint32_t sum = 0;
    for (std::uint32_t w = 0; w < words; ++w) {
        const std::uint32_t v = rng();
        put32(slot, 12 + 4 * w, v);
        sum += v;
    }
    const std::size_t end = 12 + 4 * words;
    put32(slot, end, 0);
    put32(slot, end + 4, addr);
    put32(slot, end + 8, sum);
    for (std::size_t i = 0; i < 300000; ++i) slot[0x40000 + i] = static_cast<std::uint8_t>(rng() & 0x7F);
    return slot;
}

std::vector<std::uint8_t> package_of(const std::vector<std::uint8_t>& slot) {
    std::vector<std::uint8_t> out;
    for (std::size_t p = 0; p < slot.size(); p += 4096) {
        std::uint8_t sum = 0;
        for (std::size_t i = 0; i < 4096; ++i) sum = static_cast<std::uint8_t>(sum + slot[p + i]);
        out.insert(out.end(), slot.begin() + static_cast<std::ptrdiff_t>(p), slot.begin() + static_cast<std::ptrdiff_t>(p + 4096));
        out.push_back(sum);
    }
    return out;
}

device::FirmwarePackage parsed(const std::vector<std::uint8_t>& slot) {
    auto p = device::parse_firmware_package(package_of(slot));
    REQUIRE(p.has_value());
    return *p;
}

sim::SimConfig test_config() {
    sim::SimConfig c;
    c.updated_firmware = "EinScan10_01_SC130_FX3_V9.99_FPGA_V3.7_EN";
    c.update_timeout = 60s;  // (the firmware's 2 s; a descheduled test process would trip it)
    c.restart_time = 50ms;
    return c;
}

device::FlashOptions test_options(const std::shared_ptr<sim::SimDevice>& sim) {
    device::FlashOptions o;
    o.open = [sim] { return sim->connect(); };
    o.page_pause = 0ms;
    o.settle = 80ms;
    o.poll_interval = 20ms;
    return o;
}

// A raw bulk request straight to the emulated firmware.
std::uint8_t raw_bulk(usb::Transport& t, std::uint8_t seq, std::span<const std::uint8_t> payload) {
    const auto req = usb::build_device_request({seq, 0x00, 0x06}, payload, usb::kBulkRequestSize);
    auto r = t.bulk(req, usb::kBulkReplySize, 1000);
    REQUIRE(r.has_value());
    return (*r)[4];
}

}  // namespace

TEST_CASE("an update package built like mkpackage's parses") {
    const auto slot = make_slot(1);
    const auto p = parsed(slot);
    CHECK(p.pages == 320);
    CHECK(p.data_size == 320u * 4096);
    CHECK(p.fx3_sections == 1);
    CHECK(p.fx3_entry == 0x40003000);
    CHECK(p.fx3_image_size == 12 + 4 * 3000 + 12);
    CHECK(p.fpga_size == 300000);
}

TEST_CASE("damaged update packages are refused before anything is sent") {
    const auto good = package_of(make_slot(2));
    auto bad_check = good;
    bad_check[5 * 4097 + 4096] ^= 1;
    auto r = device::parse_firmware_package(bad_check);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().message.find("page 5") != std::string::npos);

    auto truncated = good;
    truncated.resize(truncated.size() - 100);
    CHECK_FALSE(device::parse_firmware_package(truncated).has_value());

    auto slot = make_slot(3);
    slot[100] ^= 0x40;  // inside the FX3 image: its checksum no longer matches
    r = device::parse_firmware_package(package_of(slot));
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().message.find("checksum") != std::string::npos);

    slot = make_slot(4);
    std::fill(slot.begin() + 0x40000, slot.end(), std::uint8_t{0xFF});
    r = device::parse_firmware_package(package_of(slot));
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().message.find("FPGA") != std::string::npos);

    slot = make_slot(5);
    slot[0] = 'X';
    CHECK_FALSE(device::parse_firmware_package(package_of(slot)).has_value());
    CHECK_FALSE(device::parse_firmware_package(package_of(make_slot(6, 385))).has_value());  // more than a slot
}

TEST_CASE("the real update packages parse: EXStar's and our build's") {
    const std::string exstar = "/Applications/EXStar.app/Contents/MacOS/fabu_UPDATE/Configure/EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN_IAP.img";
    const std::string ours = std::string(EINSTAR_SOURCE_DIR) + "/firmware/build/EN/EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN_IAP.img";
    int found = 0;
    for (const auto& path : {exstar, ours}) {
        if (!std::filesystem::exists(path)) continue;
        ++found;
        auto p = device::load_firmware_package(path);
        REQUIRE(p.has_value());
        CHECK(p->pages == 320);
        CHECK(p->data_size == 1310720);  // what EXStar announces in 00/06: the data bytes, no check bytes
        CHECK(p->fpga_size == 1006076);
    }
    if (found == 0) SKIP("no update package on this machine");
}

TEST_CASE("firmware version strings split into their fields") {
    const auto v = device::parse_firmware_version("EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN");
    CHECK(v.product == "EinScan10_01");
    CHECK(v.sensor == "SC130");
    CHECK(v.fx3 == "2.10");
    CHECK(v.fpga == "3.7");
    CHECK(v.language == "EN");
    CHECK(device::parse_firmware_version("something else").fx3.empty());
}

TEST_CASE("flashing writes the package into the inactive slot, boots it and reports the new version") {
    auto sim = std::make_shared<sim::SimDevice>(test_config());
    REQUIRE(sim->boot_slot() == kSlotA);
    const auto slot = make_slot(7);
    const auto pkg = parsed(slot);
    int last_progress = 0;
    auto o = test_options(sim);
    o.progress = [&](int done, int pages) {
        CHECK(pages == 320);
        last_progress = done;
    };
    auto r = device::flash_firmware(pkg, o);
    REQUIRE(r.has_value());
    CHECK(last_progress == 320);
    CHECK(r->version_before == "EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN");
    CHECK(r->version_after == "EinScan10_01_SC130_FX3_V9.99_FPGA_V3.7_EN");
    CHECK(r->serial == "0009011402CF0C20");
    CHECK(sim->restarts() == 2);  // the reboot first, then the switch to the new slot
    CHECK(sim->boot_slot() == kSlotB);
    CHECK(sim->system_flash(kSlotB, static_cast<std::uint32_t>(slot.size())) == slot);

    // Again: the other slot, and back.
    const auto slot2 = make_slot(8);
    REQUIRE(device::flash_firmware(parsed(slot2), test_options(sim)).has_value());
    CHECK(sim->boot_slot() == kSlotA);
    CHECK(sim->system_flash(kSlotA, static_cast<std::uint32_t>(slot2.size())) == slot2);
    CHECK(sim->system_flash(kSlotB, static_cast<std::uint32_t>(slot.size())) == slot);  // the previous one stays
    CHECK(sim->dangerous_command_seen());  // (00/08 and 00/06, as intended)
}

namespace {

// An update abandoned after three pages (00/0D cancels it; the 2 s timer does the same): the firmware
// keeps its page count.
void abandon_an_update(sim::SimDevice& sim, const device::FirmwarePackage& pkg) {
    auto t = sim.connect();
    REQUIRE(t.has_value());
    const std::uint8_t head[5] = {0, static_cast<std::uint8_t>(pkg.data_size >> 24), static_cast<std::uint8_t>(pkg.data_size >> 16),
                                  static_cast<std::uint8_t>(pkg.data_size >> 8), static_cast<std::uint8_t>(pkg.data_size)};
    REQUIRE(raw_bulk(**t, 1, head) == 0);
    std::vector<std::uint8_t> page(1 + 4097, 0);
    for (int i = 0; i < 3; ++i) {
        std::ranges::copy(pkg.page_with_check(i), page.begin() + 1);
        REQUIRE(raw_bulk(**t, static_cast<std::uint8_t>(2 + i), page) == 0);
    }
    const auto cancel = usb::build_device_request({9, 0x00, 0x0D}, {}, 16);
    REQUIRE((*t)->command(cancel, 16, 1000).has_value());
    REQUIRE_FALSE(sim.update_active());
}

}  // namespace

TEST_CASE("the reboot before flashing clears an abandoned update, which would otherwise be written shifted") {
    const auto slot = make_slot(9);
    const auto pkg = parsed(slot);
    {
        // The firmware defect (docs/firmware.md 5.1), without the reboot: the new update continues the
        // abandoned one's page count, lands three pages late, and "finishes" after page 316: the firmware
        // boots the shifted image, and the host only notices when the scanner is gone for page 317.
        auto sim = std::make_shared<sim::SimDevice>(test_config());
        abandon_an_update(*sim, pkg);
        auto t = sim->connect();
        REQUIRE(t.has_value());
        device::ConnectOptions co;
        co.heartbeat_ms = 0;
        auto dev = device::EinstarDevice::connect(std::move(*t), co);
        REQUIRE(dev.has_value());
        auto w = (*dev)->write_firmware(pkg, {}, 0ms);
        REQUIRE_FALSE(w.has_value());
        CHECK(w.error().message.find("page 317") != std::string::npos);
        CHECK(sim->boot_slot() == kSlotB);
        CHECK(sim->system_flash(kSlotB + 3 * 4096, 4096) == std::vector<std::uint8_t>(slot.begin(), slot.begin() + 4096));
        CHECK(sim->system_flash(kSlotB, static_cast<std::uint32_t>(slot.size())) != slot);
    }
    {
        // flash_firmware reboots first, so the same history gives a correct slot.
        auto sim = std::make_shared<sim::SimDevice>(test_config());
        abandon_an_update(*sim, pkg);
        REQUIRE(device::flash_firmware(pkg, test_options(sim)).has_value());
        CHECK(sim->boot_slot() == kSlotB);
        CHECK(sim->system_flash(kSlotB, static_cast<std::uint32_t>(slot.size())) == slot);
    }
}

TEST_CASE("a rejected update stops at once, keeps the old firmware booting and says to reboot") {
    auto cfg = test_config();
    cfg.status_override[0x0006] = 2;
    auto sim = std::make_shared<sim::SimDevice>(cfg);
    auto r = device::flash_firmware(parsed(make_slot(10)), test_options(sim));
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().message.find("reboot it before trying again") != std::string::npos);
    CHECK(sim->boot_slot() == kSlotA);
    const auto rx = sim->received();
    CHECK(std::ranges::count_if(rx, [](const sim::ReceivedCommand& c) { return c.group == 0x00 && c.opcode == 0x06; }) == 1);  // never retried
}

TEST_CASE("the firmware update's timer drops an update that stalls") {
    auto cfg = test_config();
    cfg.update_timeout = 100ms;
    auto sim = std::make_shared<sim::SimDevice>(cfg);
    const auto pkg = parsed(make_slot(11));
    auto t = sim->connect();
    REQUIRE(t.has_value());
    const std::uint8_t head[5] = {0, 0, 0x14, 0, 0};
    REQUIRE(raw_bulk(**t, 1, head) == 0);
    std::this_thread::sleep_for(300ms);
    std::vector<std::uint8_t> page(1 + 4097, 0);
    std::ranges::copy(pkg.page_with_check(0), page.begin() + 1);
    CHECK(raw_bulk(**t, 2, page) == 2);  // no update any more: a 00/06 with the wrong length
    CHECK(sim->boot_slot() == kSlotA);
}

TEST_CASE("firmware work refuses a device that is not an Einstar") {
    auto cfg = test_config();
    cfg.product_name = "SomethingElse";
    auto sim = std::make_shared<sim::SimDevice>(cfg);
    auto dev = device::connect_for_firmware([sim] { return sim->connect(); });
    REQUIRE_FALSE(dev.has_value());
    CHECK(dev.error().code == Errc::unsupported);
}

namespace {

// Records every request the host sends and what the emulated firmware answered, for replaying the same
// conversation into the real firmware code (tools/fx3emu: python -m fx3emu replay TRACE).
struct Trace {
    std::mutex m;
    std::vector<std::string> lines;
    static std::string hex(std::span<const std::uint8_t> b) {
        std::string s;
        for (auto x : b) s += std::format("{:02x}", x);
        return s;
    }
    void add(std::string_view channel, std::span<const std::uint8_t> req, const Result<std::vector<std::uint8_t>>& r) {
        std::lock_guard lock(m);
        lines.push_back(std::format("{} {} {}", channel, hex(req), r ? hex(*r) : std::format("error:{}", static_cast<int>(r.error().code))));
    }
};

class RecordingTransport final : public usb::Transport {
public:
    RecordingTransport(std::unique_ptr<usb::Transport> inner, std::shared_ptr<Trace> trace)
        : inner_(std::move(inner)), trace_(std::move(trace)) {}
    std::string description() const override { return inner_->description(); }
    Result<std::vector<std::uint8_t>> command(std::span<const std::uint8_t> req, std::size_t cap, unsigned t) override {
        auto r = inner_->command(req, cap, t);
        trace_->add("command", req, r);
        return r;
    }
    Result<std::vector<std::uint8_t>> bulk(std::span<const std::uint8_t> req, std::size_t n, unsigned t) override {
        auto r = inner_->bulk(req, n, t);
        trace_->add("bulk", req, r);
        return r;
    }
    Result<void> reset_command_pipe() override { return inner_->reset_command_pipe(); }
    Result<void> reset_bulk_pipe() override { return inner_->reset_bulk_pipe(); }
    Result<void> start_stream(PacketHandler h) override { return inner_->start_stream(std::move(h)); }
    void stop_stream() override { inner_->stop_stream(); }
    usb::TransportStats stats() const override { return inner_->stats(); }

private:
    std::unique_ptr<usb::Transport> inner_;
    std::shared_ptr<Trace> trace_;
};

}  // namespace

TEST_CASE("record the host's firmware-flash conversation for the fx3emu cross-check") {
    const char* out = std::getenv("EINSTAR_FIRMWARE_TRACE");
    if (!out) SKIP("set EINSTAR_FIRMWARE_TRACE=<file> to record (tools/fx3emu: python -m fx3emu replay <file>)");
    // The real package when given (EINSTAR_FIRMWARE_TRACE_PACKAGE), else a synthetic one.
    device::FirmwarePackage pkg;
    if (const char* path = std::getenv("EINSTAR_FIRMWARE_TRACE_PACKAGE")) {
        auto p = device::load_firmware_package(path);
        REQUIRE(p.has_value());
        pkg = std::move(*p);
    } else {
        pkg = parsed(make_slot(12));
    }
    auto cfg = test_config();
    cfg.updated_firmware.reset();  // (fx3emu reboots the image it runs, not the one written)
    auto sim = std::make_shared<sim::SimDevice>(cfg);
    auto trace = std::make_shared<Trace>();
    auto o = test_options(sim);
    o.open = [sim, trace]() -> Result<std::unique_ptr<usb::Transport>> {
        auto t = sim->connect();
        if (!t) return t;
        {
            std::lock_guard lock(trace->m);
            trace->lines.emplace_back("open");
        }
        return std::unique_ptr<usb::Transport>(new RecordingTransport(std::move(*t), trace));
    };
    REQUIRE(device::flash_firmware(pkg, o).has_value());
    std::ofstream f(out);
    for (const auto& l : trace->lines) f << l << "\n";
    REQUIRE(f.good());
}

TEST_CASE("record a typical session's conversation for the fx3emu cross-check") {
    const char* out = std::getenv("EINSTAR_SESSION_TRACE");
    if (!out) SKIP("set EINSTAR_SESSION_TRACE=<file> to record (tools/fx3emu: python -m fx3emu replay <file>)");
    auto sim = std::make_shared<sim::SimDevice>();
    auto trace = std::make_shared<Trace>();
    auto t = sim->connect();
    REQUIRE(t.has_value());
    trace->lines.emplace_back("open");
    device::ConnectOptions co;
    co.heartbeat_ms = 0;
    auto dev = device::EinstarDevice::connect(std::unique_ptr<usb::Transport>(new RecordingTransport(std::move(*t), trace)), co);
    REQUIRE(dev.has_value());
    auto& d = **dev;
    REQUIRE(d.read_flash(0, 6568).has_value());
    for (int s = 0; s < 3; ++s) {
        REQUIRE(d.set_exposure(s, s == 2 ? 3000u : 4400u).has_value());
        REQUIRE(d.exposure(s).has_value());
        for (const std::uint16_t g : {100, 110, 120, 125, 333, 800}) {
            REQUIRE(d.set_gain(s, g).has_value());
            REQUIRE(d.gain(s).has_value());
        }
    }
    REQUIRE(d.set_laser_percent(61).has_value());
    REQUIRE(d.set_strobe(0, 9000).has_value());
    REQUIRE(d.set_strobe(1, 1234).has_value());
    for (auto z : {device::DistanceIndication::zone0, device::DistanceIndication::zone1, device::DistanceIndication::zone2})
        REQUIRE(d.set_indication(z).has_value());
    REQUIRE(d.configure_scan_mode(68000).has_value());
    REQUIRE(d.configure_texture_mode(100000).has_value());
    REQUIRE(d.read_state().has_value());
    REQUIRE(d.temperature_c().has_value());
    REQUIRE(d.clear_state().has_value());
    d.disconnect();
    std::ofstream f(out);
    for (const auto& l : trace->lines) f << l << "\n";
    REQUIRE(f.good());
}
