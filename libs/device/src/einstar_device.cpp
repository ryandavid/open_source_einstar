#include <cmath>
#include "einstar/device/einstar_device.hpp"

#include <algorithm>
#include <optional>
#include <random>
#include <chrono>
#include <cstring>
#include <format>
#include <utility>

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

std::shared_ptr<usb::Transport> EinstarDevice::transport() const {
    std::lock_guard lock(transport_mutex_);
    return transport_;
}

void EinstarDevice::set_transport(std::shared_ptr<usb::Transport> t) {
    std::shared_ptr<usb::Transport> old;
    {
        std::lock_guard lock(transport_mutex_);
        old = std::exchange(transport_, std::move(t));
    }
    // (the old transport is destroyed here, outside the lock, once no call is using it)
}

usb::TransportStats EinstarDevice::transport_stats() const {
    const auto t = transport();
    return t ? t->stats() : usb::TransportStats{};
}

EinstarDevice::~EinstarDevice() { disconnect(); }

Result<std::unique_ptr<EinstarDevice>> EinstarDevice::connect(std::unique_ptr<usb::Transport> transport,
                                                               ConnectOptions options) {
    auto dev = std::unique_ptr<EinstarDevice>(new EinstarDevice(std::move(transport), std::move(options)));
    dev->seq_ = dev->options_.first_sequence ? static_cast<std::uint8_t>(*dev->options_.first_sequence % 254)
                                             : static_cast<std::uint8_t>(std::random_device{}() % 254);
    if (auto r = dev->identify(dev->info_); !r) return std::unexpected(r.error());
    const auto& i = dev->info_;
    log::info("connected: {} / {} serial {} firmware {} ({} sensors, {}x{})", i.vendor_name, i.product_name, i.serial,
              i.firmware, i.sensor_count, i.sensors[0].width, i.sensors[0].height);
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

Result<usb::Reply> EinstarDevice::send_checked(const OpcodeInfo& op, std::span<const std::uint8_t> payload, int attempts) {
    if (!guard_allows(op.group, op.opcode))
        return make_error(Errc::blocked, std::format("opcode {:02X}/{:02X} ({}) is blocked", op.group, op.opcode, op.name));
    return send_unguarded(op, payload, attempts);
}

Result<usb::Reply> EinstarDevice::send_unguarded(const OpcodeInfo& op, std::span<const std::uint8_t> payload, int attempts) {
    // While an update runs, the firmware takes every bulk packet as an update page.
    if (updating_ && op.channel == Channel::bulk && &op != &op::kFirmwareUpdate)
        return make_error(Errc::busy, std::format("{}: a firmware update is using the bulk channel", op.name));
    // While offline only the heartbeat (probing, reattaching, replaying) talks to the scanner.
    const auto t = transport();
    if (!t || (!online_ && std::this_thread::get_id() != heartbeat_id_.load()))
        return make_error(Errc::disconnected, std::format("{}: scanner offline", op.name));
    Error last{Errc::io, "no attempt"};
    for (int attempt = 0; attempt < std::max(1, attempts > 0 ? attempts : options_.command_retries); ++attempt) {
        const auto request = usb::build_device_request({next_sequence(), op.group, op.opcode}, payload, op.buffer);
        transcript(">>", request);
        auto raw = op.channel == Channel::bulk ? t->bulk(request, op.bulk_reply, 2000)
                                               : t->command(request, op.buffer, usb::kCommandTimeoutMs);
        if (raw) {
            transcript("<<", *raw);
            auto reply = usb::validate_reply(request, std::move(*raw), false);
            if (reply) {
                // A status is the firmware's answer, not a transport fault: retrying would repeat it (and
                // re-execute a write).
                if (const auto status = reply->raw[4]; status != 0)
                    return make_error(Errc::protocol, std::format("{}: device status {} ({})", op.name, status,
                                                                  usb::status_meaning(status, op.channel == Channel::bulk)));
                return reply;
            }
            last = reply.error();
        } else {
            last = raw.error();
            if (last.code == Errc::disconnected) {
                mark_offline(last.message);
                break;
            }
        }
        log::debug("{} attempt {} failed: {}", op.name, attempt + 1, last.message);
        if (op.channel == Channel::bulk) (void)t->reset_bulk_pipe();
        else (void)t->reset_command_pipe();
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

Result<void> EinstarDevice::identify(DeviceInfo& info) {
    auto vendor = read_string(op::kVendorName);
    if (!vendor) return std::unexpected(vendor.error());
    info.vendor_name = *vendor;
    auto product = read_string(op::kProductName);
    if (!product) return std::unexpected(product.error());
    info.product_name = *product;
    {
        std::string upper = info.product_name;
        std::ranges::transform(upper, upper.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        if (!upper.starts_with(usb::kProductNamePrefix))
            log::warn("unexpected product name '{}' (expected {}*)", info.product_name, usb::kProductNamePrefix);
    }
    if (auto r = send<op::kVendorId>(); r) info.vendor_id = reply_u16(*r);
    if (auto r = send<op::kProductId>(); r) info.product_id = reply_u16(*r);
    auto serial = send<op::kSerial>();
    if (!serial) return std::unexpected(serial.error());
    if (serial->raw.size() >= 17) info.serial = format_serial(std::span(serial->raw).subspan(9, 8));
    auto fw = read_string(op::kFirmwareVersion);
    if (!fw) return std::unexpected(fw.error());
    info.firmware = *fw;

    auto count = send<op::kSensorCount>();
    if (!count) return std::unexpected(count.error());
    info.sensor_count = reply_u8(*count);
    if (info.sensor_count < 2 || info.sensor_count > 3)
        return make_error(Errc::protocol, std::format("unexpected sensor count {}", info.sensor_count));

    for (int i = 0; i < info.sensor_count; ++i) {
        const std::uint8_t m[1] = {sensor_mask(i)};
        auto& s = info.sensors[static_cast<std::size_t>(i)];
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
    }
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

Result<void> EinstarDevice::write_user_page(int page, std::span<const std::uint8_t> data) {
    constexpr std::size_t kPage = 4096;
    if ((page != 0 && page != 1) || data.size() != kPage) return make_error(Errc::blocked, "only calibration pages 0 and 1 can be written");
    std::vector<std::uint8_t> payload{static_cast<std::uint8_t>(page >> 8), static_cast<std::uint8_t>(page)};
    payload.insert(payload.end(), data.begin(), data.end());
    log::warn("writing flash page {} (calibration)", page);
    if (auto r = send_unguarded(op::kFlashWrite, payload); !r) return std::unexpected(r.error());
    // The firmware reports no erase / program errors: the read-back is the only check.
    auto back = read_flash(static_cast<std::uint32_t>(page) * kPage, kPage);
    if (!back) return make_error(back.error().code, std::format("flash page {} written but not read back: {}", page, back.error().message));
    if (!std::ranges::equal(*back, data)) return make_error(Errc::io, std::format("flash page {} did not verify after writing", page));
    return {};
}

namespace {

// The quick-calibration section's type (u32 4) and tag ("FQFQ\0"), docs/calibration.md 1.2.
bool has_quick_section(std::span<const std::uint8_t> b) {
    constexpr std::size_t kType = 0x39B, kTag = 0x12B7;
    return b.size() >= kTag + 5 && std::memcmp(b.data() + kTag, "FQFQ", 5) == 0 && b[kType] == 4 && b[kType + 1] == 0 &&
           b[kType + 2] == 0 && b[kType + 3] == 0;
}

}  // namespace

Result<std::vector<int>> EinstarDevice::write_calibration_blob(std::span<const std::uint8_t> blob, const BackupSink& save_backup) {
    constexpr std::size_t kPage = 4096;
    if (streaming_) return make_error(Errc::busy, "stop the stream before writing the calibration");
    if (blob.size() != kCalibrationBlobSize) return make_error(Errc::invalid_argument, "calibration blob must be 6568 bytes");
    if (!has_quick_section(blob)) return make_error(Errc::invalid_argument, "new calibration blob has no quick-calibration section");
    auto current = read_flash(0, kCalibrationPagesSize);
    if (!current) return std::unexpected(current.error());
    if (!has_quick_section(*current))
        return make_error(Errc::blocked, "the scanner's flash has no quick-calibration section: not writing over unknown contents");
    for (std::size_t i = 0; i < blob.size(); ++i)
        if ((i < kQuickSectionBegin || i >= kQuickSectionEnd) && blob[i] != (*current)[i])
            return make_error(Errc::blocked, std::format("refused: byte {:#x} outside the quick-calibration section would change", i));
    if (!save_backup) return make_error(Errc::invalid_argument, "a backup is required");
    if (auto r = save_backup(*current); !r) return make_error(r.error().code, "backup failed, nothing written: " + r.error().message);

    std::vector<std::uint8_t> next = *current;
    std::ranges::copy(blob, next.begin());
    std::vector<int> written;
    for (int page = 0; page < 2; ++page) {
        const std::span<const std::uint8_t> want(next.data() + page * kPage, kPage), old(current->data() + page * kPage, kPage);
        if (std::ranges::equal(want, old)) continue;
        auto r = write_user_page(page, want);
        if (!r) {
            log::warn("{}; writing page {} once more", r.error().message, page);
            r = write_user_page(page, want);
        }
        if (!r) {
            // Put back every page touched so far.
            std::string outcome = "the previous calibration was put back";
            for (int p = 0; p <= page; ++p)
                if (auto back = write_user_page(p, std::span<const std::uint8_t>(current->data() + p * kPage, kPage)); !back)
                    outcome = std::format("page {} could NOT be put back ({}): restore it from the backup", p, back.error().message);
            return make_error(r.error().code, std::format("{}; {}", r.error().message, outcome));
        }
        written.push_back(page);
    }
    return written;
}

Result<std::vector<int>> EinstarDevice::restore_calibration_pages(std::span<const std::uint8_t> backup) {
    constexpr std::size_t kPage = 4096;
    if (streaming_) return make_error(Errc::busy, "stop the stream before restoring the calibration");
    if (backup.size() != kCalibrationPagesSize) return make_error(Errc::invalid_argument, "a calibration backup is 8192 bytes (pages 0-1)");
    if (!has_quick_section(backup)) return make_error(Errc::blocked, "not a calibration backup (no quick-calibration section)");
    const auto current = read_flash(0, kCalibrationPagesSize);  // (may fail after an interrupted write: then write both)
    std::vector<int> written;
    for (int page = 0; page < 2; ++page) {
        const std::span<const std::uint8_t> want(backup.data() + page * kPage, kPage);
        if (current && std::ranges::equal(want, std::span<const std::uint8_t>(current->data() + page * kPage, kPage))) continue;
        auto r = write_user_page(page, want);
        if (!r) r = write_user_page(page, want);
        if (!r) return std::unexpected(r.error());
        written.push_back(page);
    }
    return written;
}

Result<void> EinstarDevice::reboot() {
    log::warn("rebooting the scanner");
    // One attempt: a lost reply may mean it already rebooted, and a second 00/08 would reboot it again.
    auto r = send_unguarded(op::kReboot, {}, 1);
    mark_offline("rebooting");
    if (!r) return std::unexpected(r.error());
    return {};
}

Result<void> EinstarDevice::write_firmware(const FirmwarePackage& package, const FirmwareProgress& progress,
                                           std::chrono::milliseconds page_pause) {
    if (streaming_) return make_error(Errc::busy, "stop the stream before updating the firmware");
    if (package.pages <= 0 || package.bytes.size() != static_cast<std::size_t>(package.pages) * kFirmwarePackagePage)
        return make_error(Errc::invalid_argument, "not a parsed firmware package");
    if (updating_.exchange(true)) return make_error(Errc::busy, "a firmware update is already running");
    struct Done {
        std::atomic<bool>& flag;
        ~Done() { flag = false; }
    } done{updating_};

    const auto abandoned = [&](int page, const Error& e) {
        const bool status = e.code == Errc::protocol;  // the firmware answered with an error status
        return make_error(e.code, std::format(
            "firmware update stopped at {} of {}: {}. {} The scanner keeps booting its current firmware; reboot it "
            "before trying again (the firmware keeps an abandoned update's page count, docs/firmware.md 5.1)",
            page < 0 ? std::string("the start") : std::format("page {}", page), package.pages, e.message,
            status ? "The firmware abandoned the update." : "The scanner's state is unknown: it drops the update 2 s after the last page it got."));
    };
    // 00/06: payload byte 0 unused, then the data size (BE32).
    const std::uint32_t n = package.data_size;
    const std::uint8_t head[5] = {0, static_cast<std::uint8_t>(n >> 24), static_cast<std::uint8_t>(n >> 16),
                                  static_cast<std::uint8_t>(n >> 8), static_cast<std::uint8_t>(n)};
    log::warn("firmware update: {} pages, {} bytes", package.pages, n);
    if (auto r = send_unguarded(op::kFirmwareUpdate, head, 1); !r) return abandoned(-1, r.error());
    // Pages: payload byte 0 unused, the 4096 data bytes, the check byte.
    std::vector<std::uint8_t> payload(1 + kFirmwarePackagePage, 0);
    for (int page = 0; page < package.pages; ++page) {
        std::this_thread::sleep_for(page_pause);
        std::ranges::copy(package.page_with_check(page), payload.begin() + 1);
        if (auto r = send_unguarded(op::kFirmwareUpdate, payload, 1); !r) return abandoned(page, r.error());
        if (progress) progress(page + 1, package.pages);
    }
    // The firmware now switches its boot record to the new slot; its watchdog resets it within ~1 s.
    mark_offline("rebooting into the new firmware");
    return {};
}

Result<double> EinstarDevice::temperature_c() {
    auto r = send<op::kTemperature>();
    if (!r) return std::unexpected(r.error());
    // ADT7420 format: signed 16-bit, 1/128 degC per LSB (EXStar's integer conversion mishandles negatives).
    return static_cast<std::int16_t>(reply_u16(*r)) / 128.0;
}

namespace {

DeviceState state_from_reply(const usb::Reply& r) {
    DeviceState s;
    for (std::size_t i = 0; i < s.raw.size(); ++i) s.raw[i] = 9 + i < r.raw.size() ? r.raw[9 + i] : 0;
    for (int i = 0; i < 3; ++i) {
        const std::size_t off = 9 + static_cast<std::size_t>(i);
        const std::uint8_t v = off < r.raw.size() ? r.raw[off] : 0;
        s.buttons[static_cast<std::size_t>(i)] = v <= 3 ? static_cast<ButtonAction>(v) : ButtonAction::none;
    }
    return s;
}

}  // namespace

Result<DeviceState> EinstarDevice::read_state() {
    auto r = send<op::kDeviceState>();
    if (!r) return std::unexpected(r.error());
    return state_from_reply(*r);
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
    remember([&](Settings& s) { s.trigger = std::pair{mono, rgb}; });
    rgb_triggers_ = rgb;
    mono_triggers_ = mono;
    std::lock_guard lock(stream_mutex_);
    if (groups_) groups_->set_expected_mask(rgb > 0 ? 0b111u : 0b011u);
    return {};
}

Result<void> EinstarDevice::set_trigger_period_us(std::uint32_t period) {
    if (period < kMinTriggerPeriodUs || period > kMaxTriggerPeriodUs)
        return make_error(Errc::invalid_argument, std::format("trigger period {} us outside {}..{}", period,
                                                              kMinTriggerPeriodUs, kMaxTriggerPeriodUs));
    const auto p = be32(period);
    auto r = send<op::kSetTriggerPeriod>(p);
    if (!r) return std::unexpected(r.error());
    remember([&](Settings& s) { s.trigger_period = period; });
    trigger_period_us_ = period;
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
    remember([&](Settings& st) { st.exposure[sensor == 2 ? 1 : 0] = value; });
    return {};
}

Result<void> EinstarDevice::set_gain(int sensor, std::uint16_t value) {
    if (auto c = check_sensor(sensor, info_.sensor_count); !c) return std::unexpected(c.error());
    const auto& s = info_.sensors[static_cast<std::size_t>(sensor)];
    if (s.gain_max > 0) value = std::clamp(value, s.gain_min, s.gain_max);
    const std::uint8_t p[3] = {sensor_mask(sensor), static_cast<std::uint8_t>(value >> 8), static_cast<std::uint8_t>(value)};
    auto r = send<op::kSetGain>(p);
    if (!r) return std::unexpected(r.error());
    remember([&](Settings& st) { st.gain[static_cast<std::size_t>(sensor)] = value; });
    return {};
}

Result<void> EinstarDevice::set_laser_percent(int percent) {
    percent = std::clamp(percent, 0, kMaxLaserPercent);
    const std::uint8_t p[1] = {static_cast<std::uint8_t>(percent)};
    auto r = send<op::kSetLaser>(p);
    if (!r) return std::unexpected(r.error());
    remember([&](Settings& s) { s.laser = percent; });
    return {};
}

Result<void> EinstarDevice::set_strobe(int route, int luminance) {
    if (route < 0 || route > 1) return make_error(Errc::invalid_argument, "strobe route must be 0 or 1");
    const auto lum = static_cast<std::uint16_t>(std::clamp(luminance, 0, kMaxStrobeLuminance));
    const std::uint8_t p[3] = {static_cast<std::uint8_t>(route), static_cast<std::uint8_t>(lum >> 8),
                               static_cast<std::uint8_t>(lum)};
    auto r = send<op::kSetStrobe>(p);
    if (!r) return std::unexpected(r.error());
    remember([&](Settings& s) { s.strobe[static_cast<std::size_t>(route)] = lum; });
    return {};
}

Result<void> EinstarDevice::set_indication(DistanceIndication distance) {
    // The payload must be 2 bytes. The second (EXStar's DEVICESTATE) is ignored by the firmware;
    // send 1 as EXStar does while scanning.
    const std::uint8_t p[2] = {static_cast<std::uint8_t>(distance), 1};
    auto r = send<op::kIndication>(p);
    if (!r) return std::unexpected(r.error());
    remember([&](Settings& s) { s.indication = distance; });
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
    std::lock_guard control(stream_control_mutex_);
    if (streaming_) return make_error(Errc::busy, "stream already running");
    const auto& s0 = info_.sensors[0];
    if (s0.width <= 0 || s0.height <= 0) return make_error(Errc::protocol, "unknown sensor size");
    {
        std::lock_guard lock(stream_mutex_);
        // The scanner's group id counts modulo 256 (measured; the header field is 32 bits wide): extend
        // it so ids keep increasing for the whole stream (recordings use them as frame indices).
        groups_ = std::make_unique<usb::GroupAssembler>(
            [this, sink = std::move(sink), base = std::uint32_t{0}, last = std::optional<std::uint32_t>{},
             previous = std::optional<std::uint32_t>{}, time_us = std::uint64_t{0}](usb::FrameGroup&& g) mutable {
                if (last && g.frame_id < *last && *last <= 0xFFu) base += 256;
                last = g.frame_id;
                g.frame_id += base;
                // Time from the trigger schedule (the header's timestamp field is a constant on the scanner).
                const std::uint64_t interval = group_interval_us();
                time_us = previous && g.frame_id > *previous ? time_us + (g.frame_id - *previous) * interval
                                                             : previous ? time_us + interval : (g.frame_id + 1ull) * interval;
                previous = g.frame_id;
                g.timestamp = time_us;
                for (auto& s : g.sensors)
                    if (s) s->frame_id = g.frame_id;
                sink(std::move(g));
            });
        groups_->set_expected_mask(rgb_triggers_ > 0 ? 0b111u : 0b011u);
        frames_ = std::make_unique<usb::FrameAssembler>(s0.width, s0.height, [this](usb::StreamFrame&& f) {
            // Upright, as the calibration expects (tightly packed: a 180-degree turn is a reversal).
            if (f.sensor == usb::kUpsideDownSensor) std::ranges::reverse(f.pixels.pixels());
            groups_->push(std::move(f));
        });
    }
    const auto t = transport();
    if (!t) return make_error(Errc::disconnected, "start stream: scanner offline");
    auto r = t->start_stream(packet_handler());
    if (!r) return r;
    streaming_ = true;
    return {};
}

usb::Transport::PacketHandler EinstarDevice::packet_handler() {
    return [this](std::span<const std::uint8_t> packet) {
        std::lock_guard lock(stream_mutex_);
        if (frames_) frames_->push(packet);
    };
}

void EinstarDevice::stop_stream() {
    std::lock_guard control(stream_control_mutex_);
    if (!streaming_) return;
    if (const auto t = transport()) t->stop_stream();
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

void EinstarDevice::mark_offline(std::string_view reason) {
    if (online_.exchange(false)) log::warn("scanner offline: {}", reason);
}

void EinstarDevice::heartbeat_loop(std::stop_token st) {
    heartbeat_id_ = std::this_thread::get_id();
    int failures = 0;
    while (!st.stop_requested()) {
        const int wait = online_ ? options_.heartbeat_ms : std::max(options_.heartbeat_ms, options_.reconnect_interval_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(wait));
        if (st.stop_requested()) break;
        if (!online_ && options_.reopen) {
            reattach();
            failures = 0;
            continue;
        }
        // One attempt, as EXStar's device-state poll: a missed poll is not worth retrying.
        auto r = send_checked(op::kDeviceState, {}, 1);
        if (!r) {
            if (r.error().code == Errc::disconnected || ++failures > 3) mark_offline(r.error().message);
            continue;
        }
        if (!online_) log::info("scanner responding again");
        online_ = true;
        failures = 0;
        const auto state = state_from_reply(*r);
        std::lock_guard lock(button_mutex_);
        for (int i = 0; i < 3; ++i) {
            const auto a = state.buttons[static_cast<std::size_t>(i)];
            if (a != ButtonAction::none && button_sink_) button_sink_(i, a);
        }
    }
}

void EinstarDevice::reattach() {
    // The old connection is dead; close it first (it holds the interface claim a new one needs).
    if (const auto old = transport()) {
        std::lock_guard control(stream_control_mutex_);
        if (streaming_) old->stop_stream();
    }
    set_transport(nullptr);
    auto opened = options_.reopen();
    if (!opened) {
        log::debug("reopen: {}", opened.error().message);
        return;
    }
    set_transport(std::shared_ptr<usb::Transport>(std::move(*opened)));
    DeviceInfo fresh;
    if (auto r = identify(fresh); !r) {
        log::warn("reconnect: identification failed: {}", r.error().message);
        set_transport(nullptr);
        return;
    }
    if (fresh.serial != info_.serial) {
        if (std::exchange(foreign_serial_, fresh.serial) != fresh.serial)
            log::warn("reconnect: found serial {}, not {}; waiting for our scanner", fresh.serial, info_.serial);
        set_transport(nullptr);
        return;
    }
    foreign_serial_.clear();
    // The restart reset the FPGA and the sensors: replay every setting, triggers last so nothing fires
    // before the rest is in place, then ClearState as EXStar does.
    Settings s;
    {
        std::lock_guard lock(settings_mutex_);
        s = settings_;
    }
    bool ok = true, lost = false;
    auto step = [&](Result<void> r) {
        if (r) return;
        log::warn("reconnect: replay failed: {}", r.error().message);
        ok = false;
        lost = lost || r.error().code == Errc::disconnected;
    };
    if (s.exposure[0]) step(set_exposure(0, *s.exposure[0]));
    if (s.exposure[1] && info_.sensor_count > 2) step(set_exposure(2, *s.exposure[1]));
    for (int i = 0; i < info_.sensor_count; ++i)
        if (s.gain[static_cast<std::size_t>(i)]) step(set_gain(i, *s.gain[static_cast<std::size_t>(i)]));
    if (s.laser) step(set_laser_percent(*s.laser));
    for (int route = 0; route < 2; ++route)
        if (s.strobe[static_cast<std::size_t>(route)]) step(set_strobe(route, *s.strobe[static_cast<std::size_t>(route)]));
    if (s.indication) step(set_indication(*s.indication));
    if (s.trigger_period) step(set_trigger_period_us(*s.trigger_period));
    step(clear_state());
    if (std::lock_guard control(stream_control_mutex_); streaming_) {
        {
            std::lock_guard lock(stream_mutex_);
            if (groups_) groups_->flush();
            if (frames_) frames_->reset();
        }
        const auto t = transport();
        step(t ? t->start_stream(packet_handler()) : make_error(Errc::disconnected, "no transport"));
    }
    if (s.trigger) step(set_trigger(s.trigger->first, s.trigger->second));
    if (lost) return;  // gone again during the replay: the next tick starts over
    online_ = true;
    reconnects_.fetch_add(1);
    log::info("scanner reconnected{}", ok ? "; settings replayed" : " (some settings could not be replayed)");
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
    if (!transport()) return;
    // Light sources and triggers off first so nothing keeps firing if the host goes away.
    (void)set_trigger(0, 0);
    (void)set_laser_percent(0);
    (void)set_strobe(0, 0);
    (void)set_strobe(1, 0);
    std::this_thread::sleep_for(100ms);
    stop_stream();
    set_transport(nullptr);
}

}  // namespace einstar::device
