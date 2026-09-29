#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <thread>

#include "einstar/device/einstar_device.hpp"
#include "einstar/sim/sim_transport.hpp"

using namespace einstar;
using namespace std::chrono_literals;

namespace {

struct Harness {
    sim::SimTransport* sim = nullptr;
    std::unique_ptr<device::EinstarDevice> dev;
};

Harness connect(sim::SimConfig cfg = {}, int heartbeat_ms = 0) {
    auto t = std::make_unique<sim::SimTransport>(cfg);
    Harness h;
    h.sim = t.get();
    device::ConnectOptions opts;
    opts.heartbeat_ms = heartbeat_ms;
    auto d = device::EinstarDevice::connect(std::move(t), opts);
    REQUIRE(d.has_value());
    h.dev = std::move(*d);
    return h;
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

TEST_CASE("masked replies are decoded transparently") {
    sim::SimConfig cfg;
    cfg.mask_replies = true;
    auto h = connect(cfg);
    CHECK(h.dev->info().serial == "0009011402CF0C20");
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
    CHECK(s.exposure[0] == 20000);
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

TEST_CASE("disconnect turns light sources and triggers off") {
    auto h = connect();
    REQUIRE(h.dev->configure_scan_mode().has_value());
    REQUIRE(h.dev->set_laser_percent(60).has_value());
    REQUIRE(h.dev->set_strobe(0, 3000).has_value());
    auto obs = h.sim->observer();  // outlives the transport
    h.dev->disconnect();
    std::lock_guard lock(obs->mutex);
    CHECK(obs->state.mono_triggers == 0);
    CHECK(obs->state.rgb_triggers == 0);
    CHECK(obs->state.laser == 0);
    CHECK(obs->state.strobe[0] == 0);
}

TEST_CASE("destroying the device without disconnect still switches everything off") {
    auto h = connect();
    REQUIRE(h.dev->configure_scan_mode().has_value());
    REQUIRE(h.dev->set_laser_percent(60).has_value());
    auto obs = h.sim->observer();
    h.dev.reset();
    std::lock_guard lock(obs->mutex);
    CHECK(obs->state.mono_triggers == 0);
    CHECK(obs->state.laser == 0);
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
