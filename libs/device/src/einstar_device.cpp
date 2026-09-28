#include <cmath>
#include "einstar/device/einstar_device.hpp"

#include <algorithm>
#include <chrono>
#include <format>

#include "einstar/core/log.hpp"
#include "einstar/usb/command.hpp"
#include "einstar/usb/constants.hpp"

namespace einstar::device {
namespace {

using namespace std::chrono_literals;

std::array<std::uint8_t, 4> be32(std::uint32_t v) {
    return {static_cast<std::uint8_t>(v >> 24), static_cast<std::uint8_t>(v >> 16), static_cast<std::uint8_t>(v >> 8),
            static_cast<std::uint8_t>(v)};
}

std::uint16_t reply_u16(const usb::Reply& r) {
    return r.raw.size() >= 11 ? static_cast<std::uint16_t>((r.raw[9] << 8) | r.raw[10]) : 0;
}
std::uint32_t reply_u32(const usb::Reply& r) { return r.raw.size() >= 13 ? usb::read_be32(r.raw, 9) : 0; }
std::uint8_t reply_u8(const usb::Reply& r) { return r.raw.size() >= 10 ? r.raw[9] : 0; }

Result<void> check_sensor(int sensor, int count) {
    if (sensor < 0 || sensor > 2 || sensor >= std::max(count, 1))
        return make_error(Errc::invalid_argument, std::format("sensor index {} out of range", sensor));
    return {};
}

}  // namespace

std::string format_serial(std::span<const std::uint8_t> bytes) {
    std::string s;
    for (auto b : bytes) s += std::format("{:02X}", b);
    return s;
}

std::uint8_t sensor_mask(int sensor) { return static_cast<std::uint8_t>(1u << sensor); }

EinstarDevice::EinstarDevice(std::unique_ptr<usb::Transport> t, ConnectOptions o)
    : transport_(std::move(t)), options_(std::move(o)) {}

EinstarDevice::~EinstarDevice() { disconnect(); }

Result<std::unique_ptr<EinstarDevice>> EinstarDevice::connect(std::unique_ptr<usb::Transport> transport,
                                                               ConnectOptions options) {
    auto dev = std::unique_ptr<EinstarDevice>(new EinstarDevice(std::move(transport), std::move(options)));
    if (auto r = dev->identify(); !r) return std::unexpected(r.error());
    if (dev->options_.heartbeat_ms > 0)
        dev->heartbeat_ = std::jthread([d = dev.get()](std::stop_token st) { d->heartbeat_loop(st); });
    return dev;
}

std::uint8_t EinstarDevice::next_sequence() {
    std::lock_guard lock(seq_mutex_);
    const std::uint8_t s = seq_;
    seq_ = static_cast<std::uint8_t>((seq_ + 1) % 254);  // 0xFE / 0xFF are reserved by the transport
    return s;
}

void EinstarDevice::transcript(std::string_view dir, std::span<const std::uint8_t> bytes) {
    if (!options_.verbose_transcript) return;
    std::string hex;
    // Trailing zero padding is noise; keep at least the 8-byte header.
    std::size_t n = bytes.size();
    while (n > 9 && bytes[n - 1] == 0) --n;
    for (std::size_t i = 0; i < n; ++i) hex += std::format("{:02X} ", bytes[i]);
    if (n < bytes.size()) hex += std::format("(+{} zero)", bytes.size() - n);
    const std::string line = std::format("{} [{}] {}", dir, bytes.size(), hex);
    if (options_.transcript_sink) options_.transcript_sink(line);
    else log::debug("{}", line);
}

template <const OpcodeInfo& Op>
    requires SafeOpcode<Op>
Result<usb::Reply> EinstarDevice::send(std::span<const std::uint8_t> payload) {
    return send_checked(Op, payload);
}

Result<usb::Reply> EinstarDevice::send_checked(const OpcodeInfo& op, std::span<const std::uint8_t> payload) {
    if (!guard_allows(op.group, op.opcode))
        return make_error(Errc::blocked, std::format("opcode {:02X}/{:02X} ({}) is blocked", op.group, op.opcode, op.name));
    Error last{Errc::io, "no attempt"};
    for (int attempt = 0; attempt < std::max(1, options_.command_retries); ++attempt) {
        const auto request = usb::build_device_request({next_sequence(), op.group, op.opcode}, payload, op.buffer);
        transcript(">>", request);
        auto raw = op.channel == Channel::bulk ? transport_->bulk(request, op.buffer, 2000)
                                               : transport_->command(request, op.buffer, usb::kCommandTimeoutMs);
        if (raw) {
            transcript("<<", *raw);
            auto reply = usb::validate_reply(request, std::move(*raw), true);
            if (reply) return reply;
            last = reply.error();
        } else {
            last = raw.error();
            if (last.code == Errc::disconnected) break;
        }
        log::debug("{} attempt {} failed: {}", op.name, attempt + 1, last.message);
        if (op.channel == Channel::bulk) (void)transport_->reset_bulk_pipe();
        else (void)transport_->reset_command_pipe();
        std::this_thread::sleep_for(20ms);
    }
    return std::unexpected(Error{last.code, std::format("{}: {}", op.name, last.message)});
}

Result<std::string> EinstarDevice::read_string(const OpcodeInfo& op) {
    auto r = send_checked(op, {});
    if (!r) return std::unexpected(r.error());
    const auto p = r->payload();
    std::string s(p.begin(), p.end());
    if (auto nul = s.find('\0'); nul != std::string::npos) s.resize(nul);
    return s;
}

Result<void> EinstarDevice::identify() {
    auto vendor = read_string(op::kVendorName);
    if (!vendor) return std::unexpected(vendor.error());
    info_.vendor_name = *vendor;
    auto product = read_string(op::kProductName);
    if (!product) return std::unexpected(product.error());
    info_.product_name = *product;
    {
        std::string upper = info_.product_name;
        std::ranges::transform(upper, upper.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        if (!upper.starts_with(usb::kProductNamePrefix))
            log::warn("unexpected product name '{}' (expected {}*)", info_.product_name, usb::kProductNamePrefix);
    }
    if (auto r = send<op::kVendorId>(); r) info_.vendor_id = reply_u16(*r);
    if (auto r = send<op::kProductId>(); r) info_.product_id = reply_u16(*r);
    auto serial = send<op::kSerial>();
    if (!serial) return std::unexpected(serial.error());
    if (serial->raw.size() >= 17) info_.serial = format_serial(std::span(serial->raw).subspan(9, 8));
    auto fw = read_string(op::kFirmwareVersion);
    if (!fw) return std::unexpected(fw.error());
    info_.firmware = *fw;

    auto count = send<op::kSensorCount>();
    if (!count) return std::unexpected(count.error());
    info_.sensor_count = reply_u8(*count);
    if (info_.sensor_count < 2 || info_.sensor_count > 3)
        return make_error(Errc::protocol, std::format("unexpected sensor count {}", info_.sensor_count));

    for (int i = 0; i < info_.sensor_count; ++i) {
        const std::uint8_t m[1] = {sensor_mask(i)};
        auto& s = info_.sensors[static_cast<std::size_t>(i)];
        auto w = send<op::kMaxWidth>(m);
        auto h = send<op::kMaxHeight>(m);
        if (!w || !h) return make_error(Errc::protocol, std::format("sensor {} size query failed", i));
        s.width = reply_u16(*w);
        s.height = reply_u16(*h);
        if (auto r = send<op::kMaxExposure>(m)) s.exposure_max = reply_u32(*r);
        if (auto r = send<op::kMinExposure>(m)) s.exposure_min = reply_u32(*r);
        if (auto r = send<op::kMaxGain>(m)) s.gain_max = reply_u16(*r);
        if (auto r = send<op::kMinGain>(m)) s.gain_min = reply_u16(*r);
        if (auto r = send<op::kPixelBits>(m)) s.pixel_bits = reply_u8(*r);
        if (auto r = send<op::kColorMode>(m)) s.color_mode = reply_u8(*r);
    }
    log::info("connected: {} / {} serial {} firmware {} ({} sensors, {}x{})", info_.vendor_name, info_.product_name,
              info_.serial, info_.firmware, info_.sensor_count, info_.sensors[0].width, info_.sensors[0].height);
    return {};
}

Result<std::vector<std::uint8_t>> EinstarDevice::read_flash(std::uint32_t offset, std::uint32_t size) {
    constexpr std::uint32_t kPage = 4096;
    if (size == 0 || offset + size > (1u << 20)) return make_error(Errc::invalid_argument, "flash range outside 1 MiB");
    std::vector<std::uint8_t> out;
    out.reserve(size);
    const std::uint32_t first = offset / kPage, last = (offset + size - 1) / kPage;
    for (std::uint32_t page = first; page <= last; ++page) {
        const std::uint8_t p[2] = {static_cast<std::uint8_t>(page >> 8), static_cast<std::uint8_t>(page)};
        auto r = send<op::kFlashRead>(p);
        if (!r) return std::unexpected(r.error());
        if (r->raw.size() < 9 + kPage)
            return make_error(Errc::protocol, std::format("flash page {} reply too short ({} bytes)", page, r->raw.size()));
        const std::uint32_t page_start = page * kPage;
        const std::uint32_t from = std::max(offset, page_start) - page_start;
        const std::uint32_t to = std::min(offset + size, page_start + kPage) - page_start;
        out.insert(out.end(), r->raw.begin() + 9 + from, r->raw.begin() + 9 + to);
    }
    return out;
}

Result<double> EinstarDevice::temperature_c() {
    auto r = send<op::kTemperature>();
    if (!r) return std::unexpected(r.error());
    // ADT7420 format: signed 16-bit, 1/128 degC per LSB (EXStar's integer conversion mishandles negatives).
    return static_cast<std::int16_t>(reply_u16(*r)) / 128.0;
}

Result<DeviceState> EinstarDevice::read_state() {
    auto r = send<op::kDeviceState>();
    if (!r) return std::unexpected(r.error());
    DeviceState s;
    for (int i = 0; i < 3; ++i) {
        const std::size_t off = 9 + static_cast<std::size_t>(i);
        const std::uint8_t v = off < r->raw.size() ? r->raw[off] : 0;
        s.buttons[static_cast<std::size_t>(i)] = v <= 3 ? static_cast<ButtonAction>(v) : ButtonAction::none;
    }
    return s;
}

Result<std::uint32_t> EinstarDevice::exposure(int sensor) {
    if (auto c = check_sensor(sensor, info_.sensor_count); !c) return std::unexpected(c.error());
    const std::uint8_t m[1] = {sensor_mask(sensor)};
    auto r = send<op::kGetExposure>(m);
    if (!r) return std::unexpected(r.error());
    return reply_u32(*r);
}

Result<std::uint16_t> EinstarDevice::gain(int sensor) {
    if (auto c = check_sensor(sensor, info_.sensor_count); !c) return std::unexpected(c.error());
    const std::uint8_t m[1] = {sensor_mask(sensor)};
    auto r = send<op::kGetGain>(m);
    if (!r) return std::unexpected(r.error());
    return reply_u16(*r);
}

Result<void> EinstarDevice::set_trigger(int mono, int rgb) {
    mono = std::clamp(mono, 0, 15);
    rgb = std::clamp(rgb, 0, 15);
    const std::uint8_t p[1] = {static_cast<std::uint8_t>((rgb << 4) | mono)};
    auto r = send<op::kSetTrigger>(p);
    if (!r) return std::unexpected(r.error());
    rgb_triggers_ = rgb;
    std::lock_guard lock(stream_mutex_);
    if (groups_) groups_->set_expected_mask(rgb > 0 ? 0b111u : 0b011u);
    return {};
}

Result<void> EinstarDevice::set_trigger_period_us(std::uint32_t period) {
    const auto p = be32(period);
    auto r = send<op::kSetTriggerPeriod>(p);
    if (!r) return std::unexpected(r.error());
    return {};
}

Result<void> EinstarDevice::set_exposure(int sensor, std::uint32_t value) {
    if (auto c = check_sensor(sensor, info_.sensor_count); !c) return std::unexpected(c.error());
    const auto& s = info_.sensors[static_cast<std::size_t>(sensor)];
    if (s.exposure_max > 0) value = std::clamp(value, s.exposure_min, s.exposure_max);
    const auto v = be32(value);
    const std::uint8_t p[5] = {sensor_mask(sensor), v[0], v[1], v[2], v[3]};
    auto r = send<op::kSetExposure>(p);
    if (!r) return std::unexpected(r.error());
    return {};
}

Result<void> EinstarDevice::set_gain(int sensor, std::uint16_t value) {
    if (auto c = check_sensor(sensor, info_.sensor_count); !c) return std::unexpected(c.error());
    const auto& s = info_.sensors[static_cast<std::size_t>(sensor)];
    if (s.gain_max > 0) value = std::clamp(value, s.gain_min, s.gain_max);
    const std::uint8_t p[3] = {sensor_mask(sensor), static_cast<std::uint8_t>(value >> 8), static_cast<std::uint8_t>(value)};
    auto r = send<op::kSetGain>(p);
    if (!r) return std::unexpected(r.error());
    return {};
}

Result<void> EinstarDevice::set_laser_percent(int percent) {
    const std::uint8_t p[1] = {static_cast<std::uint8_t>(std::clamp(percent, 0, kMaxLaserPercent))};
    auto r = send<op::kSetLaser>(p);
    if (!r) return std::unexpected(r.error());
    return {};
}

Result<void> EinstarDevice::set_strobe(int route, int luminance) {
    if (route < 0 || route > 1) return make_error(Errc::invalid_argument, "strobe route must be 0 or 1");
    const auto lum = static_cast<std::uint16_t>(std::clamp(luminance, 0, kMaxStrobeLuminance));
    const std::uint8_t p[3] = {static_cast<std::uint8_t>(route), static_cast<std::uint8_t>(lum >> 8),
                               static_cast<std::uint8_t>(lum)};
    auto r = send<op::kSetStrobe>(p);
    if (!r) return std::unexpected(r.error());
    return {};
}

Result<void> EinstarDevice::set_indication(DistanceIndication distance, bool active) {
    const std::uint8_t p[2] = {static_cast<std::uint8_t>(distance), static_cast<std::uint8_t>(active ? 1 : 0)};
    auto r = send<op::kIndication>(p);
    if (!r) return std::unexpected(r.error());
    return {};
}

Result<void> EinstarDevice::clear_state() {
    auto r = send<op::kClearState>();
    if (!r) return std::unexpected(r.error());
    return {};
}

Result<void> EinstarDevice::configure_scan_mode(std::uint32_t period_us) {
    if (auto r = set_trigger(3, 0); !r) return r;
    return set_trigger_period_us(period_us);
}

Result<void> EinstarDevice::configure_texture_mode(std::uint32_t period_us) {
    if (auto r = set_trigger(1, 1); !r) return r;
    return set_trigger_period_us(period_us);
}

Result<void> EinstarDevice::start_stream(GroupSink sink) {
    if (streaming_) return make_error(Errc::busy, "stream already running");
    const auto& s0 = info_.sensors[0];
    if (s0.width <= 0 || s0.height <= 0) return make_error(Errc::protocol, "unknown sensor size");
    {
        std::lock_guard lock(stream_mutex_);
        groups_ = std::make_unique<usb::GroupAssembler>(std::move(sink));
        groups_->set_expected_mask(rgb_triggers_ > 0 ? 0b111u : 0b011u);
        frames_ = std::make_unique<usb::FrameAssembler>(s0.width, s0.height,
                                                        [this](usb::StreamFrame&& f) { groups_->push(std::move(f)); });
    }
    auto r = transport_->start_stream([this](std::span<const std::uint8_t> packet) {
        std::lock_guard lock(stream_mutex_);
        if (frames_) frames_->push(packet);
    });
    if (!r) return r;
    streaming_ = true;
    return {};
}

void EinstarDevice::stop_stream() {
    if (!streaming_) return;
    transport_->stop_stream();
    std::lock_guard lock(stream_mutex_);
    if (groups_) groups_->flush();
    streaming_ = false;
}

usb::StreamStats EinstarDevice::stream_stats() const {
    std::lock_guard lock(const_cast<std::mutex&>(stream_mutex_));
    return frames_ ? frames_->stats() : usb::StreamStats{};
}

usb::GroupStats EinstarDevice::group_stats() const {
    std::lock_guard lock(const_cast<std::mutex&>(stream_mutex_));
    return groups_ ? groups_->stats() : usb::GroupStats{};
}

void EinstarDevice::set_button_sink(ButtonSink sink) {
    std::lock_guard lock(button_mutex_);
    button_sink_ = std::move(sink);
}

void EinstarDevice::heartbeat_loop(std::stop_token st) {
    int failures = 0;
    while (!st.stop_requested()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(options_.heartbeat_ms));
        if (st.stop_requested()) break;
        auto state = read_state();
        if (!state) {
            if (++failures > 3 && online_) {
                online_ = false;
                log::warn("scanner not responding: {}", state.error().message);
            }
            continue;
        }
        if (!online_) log::info("scanner responding again");
        online_ = true;
        failures = 0;
        std::lock_guard lock(button_mutex_);
        for (int i = 0; i < 3; ++i) {
            const auto a = state->buttons[static_cast<std::size_t>(i)];
            if (a != ButtonAction::none && button_sink_) button_sink_(i, a);
        }
    }
}

ButtonCommand button_command(int button, ButtonAction action) {
    if (action != ButtonAction::single_click) return ButtonCommand::none;
    switch (button) {
        case 0: return ButtonCommand::brightness_down;
        case 1: return ButtonCommand::toggle_scan;
        case 2: return ButtonCommand::brightness_up;
        default: return ButtonCommand::none;
    }
}

ExposureGain brightness_level(int level) {
    constexpr double kStep = 1.08, kDefaultExposure = 4400, kGain = 120, kMinExposure = 1500, kMaxExposure = 5600;
    const int l = std::clamp(level, 0, kBrightnessLevels - 1);
    const double product = kDefaultExposure * kGain * std::pow(kStep, l - kDefaultBrightness);
    const double exposure = std::clamp(product / kGain, kMinExposure, kMaxExposure);
    const double gain = std::clamp(product / exposure, 16.0, 400.0);
    return {static_cast<std::uint32_t>(std::lround(exposure)), static_cast<std::uint16_t>(std::lround(gain))};
}

void EinstarDevice::disconnect() {
    heartbeat_.request_stop();
    if (heartbeat_.joinable()) heartbeat_.join();
    if (!transport_) return;
    // Light sources and triggers off first so nothing keeps firing if the host goes away.
    (void)set_trigger(0, 0);
    (void)set_laser_percent(0);
    (void)set_strobe(0, 0);
    std::this_thread::sleep_for(100ms);
    stop_stream();
    transport_.reset();
}

}  // namespace einstar::device
