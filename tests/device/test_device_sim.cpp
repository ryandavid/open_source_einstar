#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "einstar/device/einstar_device.hpp"
#include "einstar/sim/sim_transport.hpp"
#include "einstar/usb/command.hpp"
#include "einstar/usb/constants.hpp"

using namespace einstar;
using namespace std::chrono_literals;

namespace {

struct Harness {
    std::shared_ptr<sim::SimDevice> sim;
    std::unique_ptr<device::EinstarDevice> dev;
};

Harness connect(sim::SimConfig cfg = {}, int heartbeat_ms = 0, bool reconnect = false) {
    auto [emulator, transport] = sim::make_sim_scanner(cfg);
    Harness h;
    h.sim = emulator;
    device::ConnectOptions opts;
    opts.heartbeat_ms = heartbeat_ms;
    if (reconnect) {
        opts.reopen = [d = h.sim] { return d->connect(); };
        opts.reconnect_interval_ms = 50;
    }
    auto d = device::EinstarDevice::connect(std::move(transport), opts);
    REQUIRE(d.has_value());
    h.dev = std::move(*d);
    return h;
}

// Polls up to `polls` times, 5 ms apart. Counting polls rather than wall time keeps the limit meaningful
// when the whole test process is descheduled for a while (seen for many seconds on a loaded machine).
template <typename Pred>
bool wait_for(Pred pred, int polls = 1000) {
    for (int i = 0; i < polls; ++i) {
        if (pred()) return true;
        std::this_thread::sleep_for(5ms);
    }
    return pred();
}

// A raw request straight to the emulated firmware (bypassing EinstarDevice and its guard).
std::vector<std::uint8_t> raw_command(sim::SimDevice& d, std::uint8_t seq, std::uint8_t group, std::uint8_t op,
                                      std::vector<std::uint8_t> payload, std::size_t size = 16) {
    auto t = d.connect();
    REQUIRE(t.has_value());
    const auto req = usb::build_device_request({seq, group, op}, payload, size);
    auto r = (*t)->command(req, 64, 1000);
    REQUIRE(r.has_value());
    return *r;
}

}  // namespace

TEST_CASE("connect identifies the device with read-only commands only") {
    auto h = connect();
    const auto& info = h.dev->info();
    CHECK(info.serial == "0009011402CF0C20");
    CHECK(info.firmware == "EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN");
    CHECK(info.sensor_count == 3);
    CHECK(info.sensors[0].width == 1280);
    CHECK(info.sensors[0].height == 1024);
    for (const auto& c : h.sim->received()) {
        const auto* op = device::find_opcode(c.group, c.opcode);
        REQUIRE(op != nullptr);
        CHECK(op->safety == device::Safety::read);
    }
}

TEST_CASE("the emulator frames replies as the firmware does") {
    auto d = std::make_shared<sim::SimDevice>();
    // Byte 1 is always 2 ("not masked"); status and BE32 length follow.
    auto r = raw_command(*d, 1, 0x10, 0x16, {0x01});
    CHECK(r[1] == 0x02);
    CHECK(r[4] == 0);
    CHECK(usb::read_be32(r, 5) == 2);
    CHECK(((r[9] << 8) | r[10]) == 1280);
    // Wrong payload length: status 2, no data. Unknown key: status 3 on the command channel.
    r = raw_command(*d, 2, 0x10, 0x16, {});
    CHECK(r[4] == 2);
    CHECK(r.size() == 9);
    r = raw_command(*d, 3, 0x10, 0x99, {});
    CHECK(r[4] == 3);
    // Device state declares 20 bytes and fills 14; the serial declares 12 and fills 8.
    r = raw_command(*d, 4, 0x00, 0x07, {}, 50);
    CHECK(usb::read_be32(r, 5) == 20);
    CHECK(r.size() == 29);
    r = raw_command(*d, 5, 0x00, 0x04, {}, 64);
    CHECK(usb::read_be32(r, 5) == 12);
    // PID as the firmware reports it (the USB descriptor says 3).
    r = raw_command(*d, 6, 0x10, 0x01, {});
    CHECK(((r[9] << 8) | r[10]) == 1);
}

TEST_CASE("bulk requests must fill the firmware's 5120-byte buffer and replies have its fixed sizes") {
    auto d = std::make_shared<sim::SimDevice>();
    auto t = d->connect();
    REQUIRE(t.has_value());
    const std::uint8_t page[2] = {0, 0};
    const auto req = usb::build_device_request({1, 0x10, 0x57}, page, 5120);
    auto r = (*t)->bulk(req, usb::kBulkPageReplySize, 1000);
    REQUIRE(r.has_value());
    CHECK(r->size() == usb::kBulkPageReplySize);
    CHECK(usb::read_be32(*r, 5) == 4096);
    // Asking for more than the firmware sends times out (it ends on a full packet, no zero-length packet).
    const std::uint8_t bad_page[2] = {0x01, 0x00};  // page 256: status 2 in a 1024-byte reply
    const auto req2 = usb::build_device_request({2, 0x10, 0x57}, bad_page, 5120);
    auto r2 = (*t)->bulk(req2, usb::kBulkPageReplySize, 1000);
    CHECK_FALSE(r2.has_value());
    const auto req3 = usb::build_device_request({3, 0x10, 0x57}, bad_page, 5120);
    auto r3 = (*t)->bulk(req3, usb::kBulkReplySize, 1000);
    REQUIRE(r3.has_value());
    CHECK((*r3)[4] == 2);
}

TEST_CASE("register quirks: gain rounds through the sensor register, the laser keeps half steps") {
    auto h = connect();
    REQUIRE(h.dev->set_gain(0, 110).has_value());
    CHECK(h.dev->gain(0).value() == 109);
    REQUIRE(h.dev->set_gain(0, 125).has_value());
    CHECK(h.dev->gain(0).value() == 125);
    REQUIRE(h.dev->set_gain(1, 120).has_value());
    CHECK(h.dev->gain(1).value() == 119);
    REQUIRE(h.dev->set_laser_percent(61).has_value());
    CHECK(h.sim->state().laser == 60);
}

TEST_CASE("scan configuration matches EXStar's logged bytes") {
    auto h = connect();
    const auto before = h.sim->received().size();
    REQUIRE(h.dev->configure_scan_mode(68000).has_value());
    REQUIRE(h.dev->set_exposure(0, 4400).has_value());
    REQUIRE(h.dev->set_gain(1, 120).has_value());
    REQUIRE(h.dev->set_strobe(0, 6000).has_value());
    REQUIRE(h.dev->set_indication(device::DistanceIndication::zone2).has_value());
    const auto cmds = h.sim->received();
    REQUIRE(cmds.size() == before + 6);
    auto expect = [&](std::size_t i, std::uint8_t op, std::vector<std::uint8_t> payload) {
        CHECK(cmds[before + i].group == 0x10);
        CHECK(cmds[before + i].opcode == op);
        CHECK(cmds[before + i].payload == payload);
    };
    expect(0, 0x41, {0x03});
    expect(1, 0x49, {0x00, 0x01, 0x09, 0xA0});
    expect(2, 0x23, {0x01, 0x00, 0x00, 0x11, 0x30});
    expect(3, 0x27, {0x02, 0x00, 0x78});
    expect(4, 0x70, {0x00, 0x17, 0x70});
    expect(5, 0x62, {0x02, 0x01});
}

TEST_CASE("unsafe values are clamped before they reach the device") {
    auto h = connect();
    REQUIRE(h.dev->set_laser_percent(250).has_value());
    REQUIRE(h.dev->set_strobe(0, 60000).has_value());
    REQUIRE(h.dev->set_exposure(0, 999999).has_value());
    const auto s = h.sim->state();
    CHECK(s.laser == 100);
    CHECK(s.strobe[0] == device::kMaxStrobeLuminance);
    CHECK(s.exposure[0] == 10000);  // the firmware reports 1..10000
    CHECK_FALSE(h.sim->dangerous_command_seen());
}

TEST_CASE("the IR pair shares one exposure; the colour sensor has its own") {
    auto h = connect();
    REQUIRE(h.dev->set_exposure(0, 4400).has_value());
    REQUIRE(h.dev->set_exposure(2, 3000).has_value());
    CHECK(h.dev->exposure(1).value() == 4400);
    REQUIRE(h.dev->set_exposure(1, 5000).has_value());
    CHECK(h.dev->exposure(0).value() == 5000);
    CHECK(h.dev->exposure(2).value() == 3000);
}

TEST_CASE("trigger periods the firmware would ignore are rejected") {
    auto h = connect();
    const auto before = h.sim->received().size();
    CHECK_FALSE(h.dev->set_trigger_period_us(999).has_value());
    CHECK_FALSE(h.dev->set_trigger_period_us(1000001).has_value());
    CHECK(h.sim->received().size() == before);
    REQUIRE(h.dev->set_trigger_period_us(1000).has_value());
    REQUIRE(h.dev->set_trigger_period_us(1000000).has_value());
    CHECK(h.sim->state().trigger_period_us == 1000000);
}

TEST_CASE("connect does not query the colour mode") {
    auto h = connect();
    for (const auto& c : h.sim->received()) CHECK_FALSE((c.group == 0x10 && c.opcode == 0x5D));
}

TEST_CASE("flash reads span pages and return the exact range") {
    auto h = connect();
    std::vector<std::uint8_t> blob(6568);
    for (std::size_t i = 0; i < blob.size(); ++i) blob[i] = static_cast<std::uint8_t>(i * 13 + 1);
    h.sim->set_flash(0, blob);
    auto all = h.dev->read_flash(0, 6568);
    REQUIRE(all.has_value());
    CHECK(*all == blob);
    auto mid = h.dev->read_flash(4000, 200);  // crosses the page boundary
    REQUIRE(mid.has_value());
    CHECK(std::equal(mid->begin(), mid->end(), blob.begin() + 4000));
}

TEST_CASE("temperature uses the signed ADT7420 scale") {
    sim::SimConfig cfg;
    cfg.temperature_c = -5.25;
    auto h = connect(cfg);
    auto t = h.dev->temperature_c();
    REQUIRE(t.has_value());
    CHECK(std::abs(*t + 5.25) < 1e-6);
}

TEST_CASE("heartbeat reports button presses") {
    auto h = connect({}, 20);
    std::atomic<int> presses{0};
    h.dev->set_button_sink([&](int button, device::ButtonAction a) {
        if (button == 1 && a == device::ButtonAction::single_click) ++presses;
    });
    h.sim->press_button(1);
    for (int i = 0; i < 50 && presses == 0; ++i) std::this_thread::sleep_for(10ms);
    CHECK(presses == 1);
}

TEST_CASE("streaming delivers synchronised IR pairs") {
    auto h = connect();
    REQUIRE(h.dev->configure_scan_mode(30000).has_value());
    std::atomic<int> groups{0};
    std::atomic<bool> ok{true};
    REQUIRE(h.dev->start_stream([&](usb::FrameGroup&& g) {
        if (g.mask() != 0b011 || !g.sensors[0] || g.sensors[0]->pixels.width() != 1280) ok = false;
        ++groups;
    }).has_value());
    for (int i = 0; i < 200 && groups < 6; ++i) std::this_thread::sleep_for(10ms);
    h.dev->stop_stream();
    CHECK(groups >= 6);
    CHECK(ok);
    CHECK(h.dev->stream_stats().resyncs == 0);
}

TEST_CASE("the upside-down IR camera is delivered upright") {
    auto h = connect();
    h.sim->set_frame_provider([](int sensor, std::uint32_t, ImageU8& out) {
        for (int y = 0; y < out.height(); ++y)
            for (int x = 0; x < out.width(); ++x) out(x, y) = static_cast<std::uint8_t>((x / 16 + 3 * (y / 16) + 40 * sensor) & 0xFF);
    });
    REQUIRE(h.dev->configure_scan_mode(30000).has_value());
    std::atomic<int> groups{0};
    std::atomic<bool> upright{true};
    REQUIRE(h.dev->start_stream([&](usb::FrameGroup&& g) {
        for (int s = 0; s < 2; ++s) {
            if (!g.sensors[static_cast<std::size_t>(s)]) continue;
            const auto& img = g.sensors[static_cast<std::size_t>(s)]->pixels;
            for (const auto [x, y] : {std::pair{0, 0}, std::pair{1279, 0}, std::pair{17, 1023}, std::pair{640, 512}})
                if (img(x, y) != static_cast<std::uint8_t>((x / 16 + 3 * (y / 16) + 40 * s) & 0xFF)) upright = false;
        }
        ++groups;
    }).has_value());
    for (int i = 0; i < 200 && groups < 3; ++i) std::this_thread::sleep_for(10ms);
    h.dev->stop_stream();
    CHECK(groups >= 3);
    CHECK(upright);
}

TEST_CASE("group ids keep increasing past the scanner's 8-bit counter") {
    auto h = connect();
    REQUIRE(h.dev->configure_scan_mode(1000).has_value());  // 1 ms: 300 groups in well under a second
    std::mutex m;
    std::vector<std::uint32_t> ids;
    REQUIRE(h.dev->start_stream([&](usb::FrameGroup&& g) {
        std::lock_guard lock(m);
        ids.push_back(g.frame_id);
    }).has_value());
    for (int i = 0; i < 500; ++i) {
        std::this_thread::sleep_for(10ms);
        std::lock_guard lock(m);
        if (ids.size() >= 300) break;
    }
    h.dev->stop_stream();
    std::lock_guard lock(m);
    REQUIRE(ids.size() >= 300);
    for (std::size_t i = 1; i < ids.size(); ++i) REQUIRE(ids[i] == ids[i - 1] + 1);
    CHECK(ids.back() > 255);
}

TEST_CASE("a request repeating the previous session's sequence number is dropped, and the retry recovers") {
    // The scanner still holds the numbers the previous session ended on; this session happens to
    // start on the same command number, and its first flash read on the same bulk number.
    sim::SimConfig cfg;
    cfg.previous_command_sequence = 9;
    auto [sim, t] = sim::make_sim_scanner(cfg);
    device::ConnectOptions opts;
    opts.heartbeat_ms = 0;
    opts.first_sequence = 9;
    auto dev = device::EinstarDevice::connect(std::move(t), opts);
    REQUIRE(dev.has_value());  // the dropped first request was retried under the next number
    CHECK(sim->dropped_repeats() == 1);
    CHECK((*dev)->info().vendor_name == cfg.vendor_name);
}

TEST_CASE("sessions start at a random sequence number") {
    std::set<std::uint8_t> firsts;
    for (int i = 0; i < 8; ++i) {
        auto [sim, t] = sim::make_sim_scanner();
        device::ConnectOptions opts;
        opts.heartbeat_ms = 0;
        auto dev = device::EinstarDevice::connect(std::move(t), opts);  // (owns the emulator)
        REQUIRE(dev.has_value());
        const auto rx = sim->received();
        REQUIRE(!rx.empty());
        CHECK(rx.front().sequence < 0xFE);
        firsts.insert(rx.front().sequence);
    }
    CHECK(firsts.size() > 1);  // (all eight equal by chance: 254^-7)
}

TEST_CASE("a device status error is reported with its meaning and not retried") {
    sim::SimConfig cfg;
    cfg.status_override[0x1027] = 2;  // set gain: "bad payload length"
    auto h = connect(cfg);
    auto r = h.dev->set_gain(0, 120);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().message.find("bad payload length") != std::string::npos);
    const auto rx = h.sim->received();
    CHECK(std::ranges::count_if(rx, [](const sim::ReceivedCommand& c) { return c.group == 0x10 && c.opcode == 0x27; }) == 1);
}

TEST_CASE("disconnect turns light sources and triggers off") {
    auto h = connect();
    REQUIRE(h.dev->configure_scan_mode().has_value());
    REQUIRE(h.dev->set_laser_percent(60).has_value());
    REQUIRE(h.dev->set_strobe(0, 3000).has_value());
    REQUIRE(h.dev->set_strobe(1, 2000).has_value());
    h.dev->disconnect();
    const auto s = h.sim->state();
    CHECK(s.mono_triggers == 0);
    CHECK(s.rgb_triggers == 0);
    CHECK(s.laser == 0);
    CHECK(s.strobe[0] == 0);
    CHECK(s.strobe[1] == 0);
}

TEST_CASE("destroying the device without disconnect still switches everything off") {
    auto h = connect();
    REQUIRE(h.dev->configure_scan_mode().has_value());
    REQUIRE(h.dev->set_laser_percent(60).has_value());
    h.dev.reset();
    CHECK(h.sim->state().mono_triggers == 0);
    CHECK(h.sim->state().laser == 0);
}

TEST_CASE("a cleared image-endpoint halt restarts the scanner; the device reconnects, replays and streams on") {
    auto h = connect({}, 20, true);
    REQUIRE(h.dev->set_exposure(0, 4400).has_value());
    REQUIRE(h.dev->set_gain(0, 120).has_value());
    REQUIRE(h.dev->set_gain(1, 125).has_value());
    REQUIRE(h.dev->set_laser_percent(60).has_value());
    REQUIRE(h.dev->set_strobe(0, 3000).has_value());
    REQUIRE(h.dev->set_indication(device::DistanceIndication::zone1).has_value());
    REQUIRE(h.dev->configure_scan_mode(30000).has_value());
    std::atomic<int> groups{0};
    REQUIRE(h.dev->start_stream([&](usb::FrameGroup&&) { ++groups; }).has_value());
    REQUIRE(wait_for([&] { return groups >= 3; }));

    // The halt is cleared by the transport; the next heartbeat 00/07 then restarts the firmware (USB off,
    // FPGA and sensors reset), which the device survives by reopening and replaying.
    h.sim->stall_image_endpoint();
    REQUIRE(wait_for([&] { return h.dev->reconnects() == 1 && h.dev->online(); }));
    CHECK(h.sim->restarts() == 1);
    const auto s = h.sim->state();
    CHECK(s.exposure[0] == 4400);
    CHECK(s.gain[0] == 119);
    CHECK(s.gain[1] == 125);
    CHECK(s.laser == 60);
    CHECK(s.strobe[0] == 3000);
    CHECK(s.indication_distance == 1);
    CHECK(s.mono_triggers == 3);
    CHECK(s.trigger_period_us == 30000);
    CHECK(s.control == 4);  // ClearState after the replay, as EXStar
    const int before = groups;
    CHECK(wait_for([&] { return groups >= before + 3; }));
    h.dev->stop_stream();
}

TEST_CASE("after a reboot without reconnection, commands fail at once") {
    auto h = connect({}, 20, false);
    h.sim->reboot();
    REQUIRE(wait_for([&] { return !h.dev->online(); }));
    const auto t0 = std::chrono::steady_clock::now();
    auto r = h.dev->set_laser_percent(10);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().code == Errc::disconnected);
    CHECK(std::chrono::steady_clock::now() - t0 < 100ms);
}

TEST_CASE("reconnection only accepts the same scanner") {
    sim::SimConfig other_cfg;
    other_cfg.serial = {1, 2, 3, 4, 5, 6, 7, 8};
    auto other = std::make_shared<sim::SimDevice>(other_cfg);
    auto scanner = sim::make_sim_scanner();
    auto ours = scanner.device;
    std::atomic<bool> offer_ours{false};
    device::ConnectOptions opts;
    opts.heartbeat_ms = 20;
    opts.reconnect_interval_ms = 20;
    // Reopening first finds another scanner.
    opts.reopen = [&] { return offer_ours ? ours->connect() : other->connect(); };
    auto transport = std::move(scanner.transport);
    auto dev = device::EinstarDevice::connect(std::move(transport), opts);
    REQUIRE(dev.has_value());
    REQUIRE((*dev)->set_laser_percent(40).has_value());
    ours->reboot();
    REQUIRE(wait_for([&] { return !(*dev)->online(); }));
    std::this_thread::sleep_for(400ms);  // several reopen attempts, all on the other scanner
    CHECK_FALSE((*dev)->online());
    CHECK(other->state().laser == 0);  // nothing replayed onto it
    offer_ours = true;
    REQUIRE(wait_for([&] { return (*dev)->online(); }));
    CHECK(ours->state().laser == 40);
    dev->reset();
}

TEST_CASE("scanner buttons: EXStar's assignment") {
    using device::ButtonAction, device::ButtonCommand;
    CHECK(device::button_command(1, ButtonAction::single_click) == ButtonCommand::toggle_scan);
    CHECK(device::button_command(0, ButtonAction::single_click) == ButtonCommand::brightness_down);
    CHECK(device::button_command(2, ButtonAction::single_click) == ButtonCommand::brightness_up);
    CHECK(device::button_command(1, ButtonAction::double_click) == ButtonCommand::none);
    CHECK(device::button_command(1, ButtonAction::long_click) == ButtonCommand::none);
    CHECK(device::button_command(3, ButtonAction::single_click) == ButtonCommand::none);
}

TEST_CASE("brightness ladder") {
    const auto mid = device::brightness_level(device::kDefaultBrightness);
    CHECK(mid.exposure == 4400);
    CHECK(mid.gain == 120);
    double prev = 0;
    for (int l = 0; l < device::kBrightnessLevels; ++l) {
        const auto eg = device::brightness_level(l);
        const double product = static_cast<double>(eg.exposure) * eg.gain;
        CHECK(product > prev * 1.05);  // every step is visibly brighter
        CHECK(eg.exposure <= 5600);
        prev = product;
    }
    // Out-of-range levels clamp.
    CHECK(device::brightness_level(-5).exposure == device::brightness_level(0).exposure);
    CHECK(device::brightness_level(99).gain == device::brightness_level(device::kBrightnessLevels - 1).gain);
}
