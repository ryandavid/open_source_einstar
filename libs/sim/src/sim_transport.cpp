#include "einstar/sim/sim_transport.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <format>

#include "einstar/usb/command.hpp"
#include "einstar/usb/constants.hpp"

namespace einstar::sim {
namespace {

using clock = std::chrono::steady_clock;

// Reply bytes the firmware does not write hold stale heap data; the emulator fills them with this so the
// host can never come to depend on their value.
constexpr std::uint8_t kStale = 0xCD;

// Status codes (firmware/include/protocol.h)
constexpr std::uint8_t kOk = 0, kUnknownBulk = 1, kBadLength = 2, kFailed = 3;

// Flash layout (firmware/include/board.h)
constexpr std::uint32_t kSlotA = 0x040000, kSlotB = 0x1C0000, kBootRecord = 0x03F000, kSystemFlash = 0x400000;

std::uint32_t le32(const std::vector<std::uint8_t>& b, std::size_t o) {
    return std::uint32_t{b[o]} | std::uint32_t{b[o + 1]} << 8 | std::uint32_t{b[o + 2]} << 16 | std::uint32_t{b[o + 3]} << 24;
}

// FPGA registers (firmware/include/fpga_regs.h)
constexpr int kRegExposure = 1, kRegTriggerPeriod = 2, kRegTriggerSwitch = 3, kRegStrobe0 = 4, kRegStrobe1 = 5,
              kRegTriggerAux = 7, kRegLdBrightness = 8, kRegMode = 10, kRegControl = 12;

bool is_dangerous(std::uint8_t group, std::uint8_t op) {
    return (group == 0x10 && op == 0x58) || (group == 0x00 && (op == 0x06 || op == 0x08)) || group == 0xCC;
}

// Camera selector -> index into the sensor gain registers (the firmware's I2C targets 1, 2, 4).
int camera_index(std::uint8_t selector) {
    switch (selector) {
        case 1: return 0;
        case 2: return 1;
        case 4: return 2;
        default: return -1;
    }
}

// Gain percent <-> sensor register (1/32 steps), with the firmware's float rounding: nearest, halves down.
std::uint16_t gain_to_register(std::uint16_t percent) {
    const float g = static_cast<float>(percent) * 32.0f / 100.0f;
    auto r = static_cast<std::uint16_t>(g);
    if (g - 0.5 > r) ++r;
    return r;
}
std::uint16_t register_to_gain(std::uint16_t reg) {
    const float g = static_cast<float>(reg) * 100.0f / 32.0f;
    auto r = static_cast<std::uint16_t>(g);
    if (g - 0.5 > r) ++r;
    return r;
}

// Laser mode written by 10/62 for DISTANCE 0/1/2; anything else selects mode 0.
int distance_to_mode(std::uint8_t d) { return d == 0 ? 4 : d == 1 ? 1 : d == 2 ? 2 : 0; }
int mode_to_distance(int mode) { return mode == 4 ? 0 : mode == 1 ? 1 : mode == 2 ? 2 : -1; }

// A reply under construction: header, status, declared length, data (stale until written).
struct ReplyBuilder {
    std::vector<std::uint8_t> bytes;
    ReplyBuilder(std::uint8_t seq, std::uint8_t group, std::uint8_t op) : bytes{seq, 0x02, group, op} {}
    // reply_status(): OK with `reply_len` data bytes when the payload length matches, else status 2.
    bool check(std::uint32_t length, std::uint32_t expected, std::uint32_t reply_len) {
        const bool ok = length == expected;
        set_status(ok ? kOk : kBadLength, ok ? reply_len : 0);
        return ok;
    }
    void set_status(std::uint8_t status, std::uint32_t len) {
        bytes.resize(9);
        bytes[4] = status;
        usb::write_be32(bytes, 5, len);
        bytes.resize(9 + len, kStale);
    }
    std::uint8_t& data(std::size_t i) { return bytes[9 + i]; }
    void put_be(std::size_t at, std::uint32_t v, int n) {
        for (int i = 0; i < n; ++i) data(at + static_cast<std::size_t>(i)) = static_cast<std::uint8_t>(v >> (8 * (n - 1 - i)));
    }
};

}  // namespace

// ---------------------------------------------------------------------------------------------------------
// SimDevice

SimDevice::SimDevice(SimConfig config)
    : config_(std::move(config)), flash_(1u << 20, 0xFF), system_flash_(kSystemFlash, 0xFF), rng_(config_.seed) {
    // Boot record: slot A written last and booted (new_app, old_app, boot_new; little-endian, 12 bytes).
    const std::uint8_t record[9] = {0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x01};
    std::ranges::copy(record, system_flash_.begin() + kBootRecord);
    slot_firmware_[kSlotA] = slot_firmware_[kSlotB] = config_.firmware;
    last_command_seq_ = config_.previous_command_sequence;
    last_bulk_seq_ = config_.previous_bulk_sequence;
    flash_faults_left_ = config_.flash_fault_count;
    power_on_locked();
    provider_ = [](int sensor, std::uint32_t frame_id, ImageU8& out) {
        // Default content: a moving gradient so frames are distinguishable.
        for (int y = 0; y < out.height(); ++y)
            for (int x = 0; x < out.width(); ++x)
                out(x, y) = static_cast<std::uint8_t>((x + y + static_cast<int>(frame_id) * 7 + sensor * 50) & 0xFF);
    };
}

void SimDevice::power_on_locked() {
    // Register values after a (re)start. The FPGA's own power-on values are not known; these are
    // assumptions (trigger off, so nothing streams until the host configures it). The rest is what the
    // firmware writes while starting: sensor tables (gain register 0xA0 = 500 %) and laser mode 2 with the
    // status LED "ok".
    fpga_.fill(0);
    fpga_[kRegExposure] = (1000u << 16) | 1000u;
    fpga_[kRegTriggerPeriod] = 200000;
    fpga_[kRegMode] = (2u << 4) | 2u;
    gain_register_.fill(0xA0);
    buttons_ = {};
    ep83_halt_cleared_ = false;
    image_halted_ = false;
}

void SimDevice::restart_locked(bool reboot) {
    ++generation_;
    restarts_.fetch_add(1);
    off_bus_until_ = clock::now() + config_.restart_time;
    power_on_locked();
    // A reboot also resets the firmware's globals (sequence memory, update state); app_restart() (after a
    // cleared halt) does not.
    if (reboot) {
        last_command_seq_ = last_bulk_seq_ = std::nullopt;
        update_ = {};
    }
}

Result<std::unique_ptr<usb::Transport>> SimDevice::connect() {
    std::uint64_t gen;
    {
        std::lock_guard lock(mutex_);
        if (clock::now() < off_bus_until_) return make_error(Errc::not_found, "emulated scanner is restarting");
        gen = generation_;
    }
    return std::unique_ptr<usb::Transport>(new SimTransport(shared_from_this(), gen));
}

bool SimDevice::alive_locked(std::uint64_t generation) const {
    return generation == generation_ && clock::now() >= off_bus_until_;
}

bool SimDevice::alive(std::uint64_t generation) const {
    std::lock_guard lock(mutex_);
    return alive_locked(generation);
}

std::vector<std::uint8_t> SimDevice::flash(std::uint32_t offset, std::uint32_t size) const {
    std::lock_guard lock(mutex_);
    const auto end = std::min<std::size_t>(flash_.size(), static_cast<std::size_t>(offset) + size);
    if (offset >= end) return {};
    return {flash_.begin() + offset, flash_.begin() + static_cast<std::ptrdiff_t>(end)};
}

std::vector<std::uint8_t> SimDevice::system_flash(std::uint32_t offset, std::uint32_t size) const {
    std::lock_guard lock(mutex_);
    const auto end = std::min<std::size_t>(system_flash_.size(), static_cast<std::size_t>(offset) + size);
    if (offset >= end) return {};
    return {system_flash_.begin() + offset, system_flash_.begin() + static_cast<std::ptrdiff_t>(end)};
}

std::uint32_t SimDevice::boot_slot() const {
    std::lock_guard lock(mutex_);
    return boot_slot_locked();
}

std::uint32_t SimDevice::boot_slot_locked() const {
    // fpga_load_addr_get(): the slot the boot record selects (the bootloader is assumed to agree).
    const std::uint32_t new_app = le32(system_flash_, kBootRecord);
    const bool boot_new = system_flash_[kBootRecord + 8] == 1;
    if (new_app == kSlotB) return boot_new ? kSlotB : kSlotA;
    if (new_app == kSlotA) return boot_new ? kSlotA : kSlotB;
    return kSlotA;
}

bool SimDevice::update_active() const {
    std::lock_guard lock(mutex_);
    return update_.active && clock::now() <= update_.deadline;
}

int SimDevice::flash_page_writes() const {
    std::lock_guard lock(mutex_);
    return flash_page_writes_;
}

bool SimDevice::on_bus() const {
    std::lock_guard lock(mutex_);
    return clock::now() >= off_bus_until_;
}

void SimDevice::set_frame_provider(FrameProvider p) {
    std::lock_guard lock(mutex_);
    provider_ = std::move(p);
}

void SimDevice::set_flash(std::uint32_t offset, std::span<const std::uint8_t> data) {
    std::lock_guard lock(mutex_);
    std::ranges::copy(data, flash_.begin() + offset);
}

void SimDevice::press_button(int button, std::uint8_t action) {
    std::lock_guard lock(mutex_);
    buttons_[static_cast<std::size_t>(button)] = static_cast<std::uint8_t>(action & 3);
}

void SimDevice::stall_image_endpoint() {
    std::lock_guard lock(mutex_);
    image_halted_ = true;
}

void SimDevice::reboot() {
    std::lock_guard lock(mutex_);
    restart_locked(true);
}

std::vector<ReceivedCommand> SimDevice::received() const {
    std::lock_guard lock(mutex_);
    return received_;
}

std::uint64_t SimDevice::dropped_repeats() const {
    std::lock_guard lock(mutex_);
    return dropped_repeats_;
}

SimDevice::State SimDevice::state() const {
    std::lock_guard lock(mutex_);
    return state_locked();
}

SimDevice::State SimDevice::state_locked() const {
    State s;
    const std::uint8_t trig = static_cast<std::uint8_t>(fpga_[kRegTriggerSwitch]);
    s.mono_triggers = trig & 0x0F;
    s.rgb_triggers = trig >> 4;
    s.trigger_period_us = fpga_[kRegTriggerPeriod];
    s.exposure[0] = s.exposure[1] = fpga_[kRegExposure] & 0x7FFF;
    s.exposure[2] = (fpga_[kRegExposure] >> 16) & 0x7FFF;
    for (std::size_t i = 0; i < 3; ++i) s.gain[i] = register_to_gain(gain_register_[i]);
    s.laser = static_cast<std::uint8_t>(fpga_[kRegLdBrightness] >> 2) << 1;
    s.strobe[0] = static_cast<int>((fpga_[kRegStrobe0] >> 2) & 0xFFFF);
    s.strobe[1] = static_cast<int>((fpga_[kRegStrobe1] >> 2) & 0xFFFF);
    s.indication_distance = mode_to_distance(static_cast<int>((fpga_[kRegMode] >> 4) & 7));
    s.control = fpga_[kRegControl];
    return s;
}

Result<std::vector<std::uint8_t>> SimDevice::command(std::uint64_t generation, std::span<const std::uint8_t> request,
                                                     std::size_t cap) {
    if (request.size() < 8) return make_error(Errc::invalid_argument, "short request");
    std::lock_guard lock(mutex_);
    if (!alive_locked(generation)) return make_error(Errc::disconnected, "emulated scanner left the bus");
    // A repeated sequence number (0xFE / 0xFF exempt) gets no reply and clears the stored number.
    const std::uint8_t seq = request[0];
    const bool repeat = seq < 0xFE && last_command_seq_ == seq;
    last_command_seq_ = repeat ? std::nullopt : std::optional<std::uint8_t>(seq);
    if (repeat) {
        ++dropped_repeats_;
        return make_error(Errc::timeout, "no reply (repeated sequence number)");
    }
    auto reply = handle_command(request);
    using Then = Reply::Then;
    if (reply.then == Then::restart_no_reply || reply.then == Then::reboot_no_reply) {
        restart_locked(reply.then == Then::reboot_no_reply);
        return make_error(Errc::disconnected, "emulated scanner restarted");
    }
    if (reply.then == Then::reboot_after_reply) restart_locked(true);  // (the reply is already on its way)
    if (reply.bytes.size() > cap)
        return make_error(Errc::io, std::format("reply of {} bytes overflows a {}-byte read", reply.bytes.size(), cap));
    return std::move(reply.bytes);
}

Result<std::vector<std::uint8_t>> SimDevice::bulk(std::uint64_t generation, std::span<const std::uint8_t> request,
                                                  std::size_t reply_size) {
    // The firmware's bulk DMA buffer only reaches it once full: a shorter padded request would never arrive.
    if (request.size() != usb::kBulkRequestSize)
        return make_error(Errc::timeout, std::format("bulk request of {} bytes never reaches the firmware (needs {})",
                                                     request.size(), usb::kBulkRequestSize));
    std::lock_guard lock(mutex_);
    if (!alive_locked(generation)) return make_error(Errc::disconnected, "emulated scanner left the bus");
    const std::uint8_t seq = request[0];
    const bool repeat = last_bulk_seq_ == seq;  // (no exemption on the bulk channel)
    last_bulk_seq_ = repeat ? std::nullopt : std::optional<std::uint8_t>(seq);
    if (repeat) {
        ++dropped_repeats_;
        return make_error(Errc::timeout, "no reply (repeated sequence number)");
    }
    bool reboot = false;
    auto reply = handle_bulk(request, reboot);
    // A completed update: the reply goes out, then the watchdog thread resets the device within ~1 s.
    if (reboot) restart_locked(true);
    // Reading more than the firmware sends waits for data that never comes.
    if (reply.size() < reply_size)
        return make_error(Errc::timeout, std::format("bulk reply is {} bytes, {} were expected", reply.size(), reply_size));
    reply.resize(reply_size);
    return reply;
}

SimDevice::Reply SimDevice::handle_command(std::span<const std::uint8_t> req) {
    const std::uint8_t seq = req[0], group = req[2], op = req[3];
    const std::uint32_t length = usb::read_be32(req, 4);
    // ARG(n): request payload byte n. The firmware reads beyond the declared length (10/5D), so this reads
    // the padded buffer, not just the payload.
    auto arg = [&](std::size_t n) -> std::uint8_t { return 8 + n < req.size() ? req[8 + n] : 0; };
    const std::size_t payload_n = std::min<std::size_t>(length, req.size() - 8);
    received_.push_back({seq, group, op, std::vector<std::uint8_t>(req.begin() + 8, req.begin() + 8 + static_cast<std::ptrdiff_t>(payload_n))});
    if (is_dangerous(group, op)) dangerous_seen_ = true;

    ReplyBuilder r(seq, group, op);
    Reply out;
    if (const auto it = config_.status_override.find(static_cast<std::uint16_t>(group << 8 | op)); it != config_.status_override.end()) {
        r.set_status(it->second, 0);
        out.bytes = std::move(r.bytes);
        return out;
    }
    // The reply is 9 + `declared` bytes; the firmware may copy more into its buffer (00/01 copies the NUL
    // too), but only the declared length is sent.
    auto string_reply = [&](const std::string& s, std::uint32_t declared, std::size_t copied) {
        if (!r.check(length, 0, declared)) return;
        for (std::size_t i = 0; i < std::min<std::size_t>(copied, declared); ++i) r.data(i) = i < s.size() ? static_cast<std::uint8_t>(s[i]) : 0;
    };

    switch (group << 8 | op) {
        case 0x0000: string_reply(config_.vendor_name, 18, 18); break;
        case 0x0001: string_reply(config_.product_name, 12, 13); break;  // (copies the NUL too)
        case 0x0005: string_reply(slot_firmware_[boot_slot_locked()], 42, 42); break;
        case 0x000D:  // clears an FPGA register and cancels a pending reboot or update (its page count stays)
            if (r.check(length, 0, 0)) update_.active = false;
            break;
        case 0x0004:  // declares 12 bytes, fills 8: the FX3 die id
            if (r.check(length, 0, 12))
                for (std::size_t i = 0; i < 8; ++i) r.data(i) = config_.serial[i];
            break;
        case 0x0007: {  // FPGA state register: six 2-bit buttons from bit 5, flags; declares 20, fills 14
            if (!r.check(length, 0, 20)) break;
            const auto& t = fpga_[kRegTriggerSwitch];
            const std::uint32_t run = (t & 0xFF) == 0 ? 0x01 : (t >> 4) & 0x0F ? 0x40 : 0x08;  // (measured)
            std::uint32_t v = (run << 18) | (config_.state_bit17 ? 1u << 17 : 0u);
            for (std::size_t b = 0; b < 3; ++b) v |= static_cast<std::uint32_t>(buttons_[b] & 3) << (5 + 2 * b);
            buttons_ = {};  // (cleared once read, measured)
            for (std::size_t b = 0; b < 6; ++b) r.data(b) = static_cast<std::uint8_t>((v >> (5 + 2 * b)) & 3);
            for (std::size_t b = 0; b < 5; ++b) r.data(6 + b) = static_cast<std::uint8_t>((v >> b) & 1);
            r.data(11) = static_cast<std::uint8_t>((v >> 17) & 1);
            r.data(12) = static_cast<std::uint8_t>(v >> 18);
            r.data(13) = static_cast<std::uint8_t>((v >> 26) & 1);
            // After a cleared image-endpoint halt: the full restart (USB, FPGA, sensors).
            if (ep83_halt_cleared_ && (v >> 17) & 1) {
                ep83_halt_cleared_ = false;
                out.then = Reply::Then::restart_no_reply;
            }
            break;
        }
        case 0x0008:  // reboot after the reply
            if (r.check(length, 0, 0)) out.then = Reply::Then::reboot_after_reply;
            break;
        case 0xCC00:  // erases flash block 0 and resets, no reply
            out.then = Reply::Then::reboot_no_reply;
            break;
        case 0x1000: if (r.check(length, 0, 2)) r.put_be(0, usb::kVendorId, 2); break;
        case 0x1001: if (r.check(length, 0, 2)) r.put_be(0, 1, 2); break;  // (the USB PID is 3)
        case 0x1016: if (r.check(length, 1, 2)) r.put_be(0, static_cast<std::uint32_t>(config_.width), 2); break;
        case 0x1017: if (r.check(length, 1, 2)) r.put_be(0, static_cast<std::uint32_t>(config_.height), 2); break;
        case 0x1020: if (r.check(length, 1, 4)) r.put_be(0, config_.exposure_max, 4); break;
        case 0x1021: if (r.check(length, 1, 4)) r.put_be(0, config_.exposure_min, 4); break;
        case 0x1024: if (r.check(length, 1, 2)) r.put_be(0, config_.gain_max, 2); break;
        case 0x1025: if (r.check(length, 1, 2)) r.put_be(0, config_.gain_min, 2); break;
        case 0x102E: if (r.check(length, 1, 1)) r.data(0) = 8; break;
        case 0x1022:  // selectors <= 3 read the low 15-bit field (the IR pair), others the high one
            if (r.check(length, 1, 4)) r.put_be(0, (arg(0) <= 3 ? fpga_[kRegExposure] : fpga_[kRegExposure] >> 16) & 0x7FFF, 4);
            break;
        case 0x1023: {  // values above 50000 are ignored with status 0; 15 bits are kept
            if (!r.check(length, 5, 0)) break;
            const std::uint32_t v = static_cast<std::uint32_t>(arg(1)) << 24 | static_cast<std::uint32_t>(arg(2)) << 16 |
                                    static_cast<std::uint32_t>(arg(3)) << 8 | arg(4);
            if (v > 50000) break;
            const std::uint32_t field = ((arg(3) & 0x7Fu) << 8) | arg(4);
            auto& reg = fpga_[kRegExposure];
            reg = arg(0) <= 3 ? (reg & 0xFFFF0000u) | field : (reg & 0x0000FFFFu) | (field << 16);
            break;
        }
        case 0x1026: {  // a camera that is not there reads as 0 (a failed I2C read)
            if (!r.check(length, 1, 2)) break;
            const int c = camera_index(arg(0));
            r.put_be(0, c < 0 ? 0 : register_to_gain(gain_register_[static_cast<std::size_t>(c)]), 2);
            break;
        }
        case 0x1027: {
            if (!r.check(length, 3, 0)) break;
            const int c = camera_index(arg(0));
            if (c < 0) {
                r.bytes[4] = kFailed;  // the sensor write fails on a bus without that camera
                break;
            }
            gain_register_[static_cast<std::size_t>(c)] = gain_to_register(static_cast<std::uint16_t>(arg(1) << 8 | arg(2)));
            break;
        }
        case 0x1040: if (r.check(length, 0, 1)) r.data(0) = static_cast<std::uint8_t>(fpga_[kRegTriggerSwitch]); break;
        case 0x1041:
            if (r.check(length, 1, 0)) {
                fpga_[kRegTriggerSwitch] = arg(0);
                fpga_[kRegTriggerAux] = (0x3FFFu << 17) | 7u;
            }
            break;
        case 0x1048: if (r.check(length, 0, 4)) r.put_be(0, fpga_[kRegTriggerPeriod], 4); break;
        case 0x1049:  // outside 1000..1000000: ignored with status 0
            if (r.check(length, 4, 0)) {
                const std::uint32_t v = static_cast<std::uint32_t>(arg(0)) << 24 | static_cast<std::uint32_t>(arg(1)) << 16 |
                                        static_cast<std::uint32_t>(arg(2)) << 8 | arg(3);
                if (v >= 1000 && v <= 1000000) fpga_[kRegTriggerPeriod] = v;
            }
            break;
        case 0x1050:  // ADT7420: signed, 1/128 degC
            if (r.check(length, 0, 2)) r.put_be(0, static_cast<std::uint16_t>(static_cast<std::int16_t>(config_.temperature_c * 128.0)), 2);
            break;
        case 0x1051: if (r.check(length, 0, 1)) r.data(0) = 3; break;
        case 0x105D:  // replies 0 only when payload byte 2 is 2; otherwise the byte stays stale
            if (r.check(length, 1, 1) && arg(2) == 2) r.data(0) = 0;
            break;
        case 0x1062:  // only DISTANCE is used; DEVICESTATE is ignored
            if (r.check(length, 2, 0)) fpga_[kRegMode] = (fpga_[kRegMode] & ~0x70u) | (static_cast<std::uint32_t>(distance_to_mode(arg(0))) << 4);
            break;
        case 0x1067: if (r.check(length, 0, 1)) r.data(0) = static_cast<std::uint8_t>(static_cast<std::uint8_t>(fpga_[kRegLdBrightness] >> 2) << 1); break;
        case 0x1068:  // clamped to 100; the FPGA holds level / 2
            if (r.check(length, 1, 0)) {
                const std::uint32_t level = std::min<std::uint8_t>(arg(0), 100) >> 1;
                fpga_[kRegLdBrightness] = (level << 2) | 0x80000000u | 1u;
            }
            break;
        case 0x106F: {  // routes other than 0/1 read FPGA register 0
            if (!r.check(length, 1, 2)) break;
            const int reg = arg(0) == 0 ? kRegStrobe0 : arg(0) == 1 ? kRegStrobe1 : 0;
            r.put_be(0, fpga_[static_cast<std::size_t>(reg)] >> 2, 2);
            break;
        }
        case 0x1070: {  // luminance << 2 truncated to 16 bits (wraps above 16383); routes other than 0/1 hit register 0
            if (!r.check(length, 3, 0)) break;
            const int reg = arg(0) == 0 ? kRegStrobe0 : arg(0) == 1 ? kRegStrobe1 : 0;
            const std::uint32_t lum = static_cast<std::uint32_t>(arg(1) << 8 | arg(2));
            fpga_[static_cast<std::size_t>(reg)] = ((lum << 2) & 0xFFFFu) | 1u;
            break;
        }
        case 0x107B: if (r.check(length, 0, 0)) fpga_[kRegControl] = 4; break;
        case 0x1077: case 0x1078: case 0x107A:  // no effect; status and length are left stale
            r.bytes.resize(9, kStale);
            break;
        default:  // unknown (and the undocumented keys, which the host never sends)
            r.set_status(kFailed, 0);
            break;
    }
    out.bytes = std::move(r.bytes);
    return out;
}

std::vector<std::uint8_t> SimDevice::handle_bulk(std::span<const std::uint8_t> req, bool& reboot) {
    const std::uint8_t seq = req[0], group = req[2], op = req[3];
    const std::uint32_t length = usb::read_be32(req, 4);
    const std::size_t payload_n = std::min<std::size_t>(length, req.size() - 8);
    received_.push_back({seq, group, op, std::vector<std::uint8_t>(req.begin() + 8, req.begin() + 8 + static_cast<std::ptrdiff_t>(payload_n))});
    if (is_dangerous(group, op)) dangerous_seen_ = true;

    std::vector<std::uint8_t> reply(usb::kBulkReplySize, kStale);
    reply[0] = seq;
    reply[1] = 0x02;
    reply[2] = group;
    reply[3] = op;
    usb::write_be32(reply, 5, 0);
    auto status = [&](std::uint8_t s) {
        reply[4] = s;
        return reply;
    };
    // (Fault injection first, so a test can fail an update page.)
    if (const auto it = config_.status_override.find(static_cast<std::uint16_t>(group << 8 | op)); it != config_.status_override.end())
        return status(it->second);
    // update_timer: 2 s without a page drops the update (its page and byte counts stay).
    if (update_.active && clock::now() > update_.deadline) update_.active = false;
    // While an update runs, every bulk packet is an update page, whatever its key.
    if (update_.active) return status(update_write_page(req, reboot));
    const std::uint16_t page = static_cast<std::uint16_t>(req[8] << 8 | req[9]);
    switch (group << 8 | op) {
        case 0x1057:
            if (length != 2) return status(kBadLength);
            if (page > 255) return status(kBadLength);
            reply.resize(usb::kBulkPageReplySize, kStale);
            usb::write_be32(reply, 5, 4096);
            std::copy_n(flash_.begin() + static_cast<std::ptrdiff_t>(page) * 4096, 4096, reply.begin() + 9);
            return status(kOk);
        case 0x1058: {  // only a zero length is rejected; erases the 4 KB sector, then programs the 4096 bytes
            if (length == 0 || page > 255) return status(kBadLength);
            const auto at = flash_.begin() + static_cast<std::ptrdiff_t>(page) * 4096;
            std::fill_n(at, 4096, std::uint8_t{0xFF});
            ++flash_page_writes_;
            const bool fault = flash_faults_left_ > 0 && config_.flash_fault != SimConfig::FlashFault::none;
            if (fault) --flash_faults_left_;
            if (!fault || config_.flash_fault != SimConfig::FlashFault::erased_only) std::copy_n(req.begin() + 10, 4096, at);
            if (fault && config_.flash_fault == SimConfig::FlashFault::corrupt_byte) at[1234] = static_cast<std::uint8_t>(at[1234] ^ 0x10);
            usb::write_be32(reply, 5, 4);
            return status(kOk);  // (erase / program results are not reported)
        }
        case 0x0006:  // update_begin(): payload byte 0 unused, then the data size (BE32). The page and byte
                      // counts are NOT reset (docs/firmware.md 5.1).
            if (length != 5) return status(kBadLength);
            update_.size = usb::read_be32(req, 9);
            update_.active = true;
            update_.deadline = clock::now() + config_.update_timeout;
            return status(kOk);
        default:
            return status(kUnknownBulk);
    }
}

std::uint8_t SimDevice::update_write_page(std::span<const std::uint8_t> req, bool& reboot) {
    // update_write_page(): payload byte 0 unused, the data, the check byte (8-bit sum of the data).
    const std::uint32_t data_len = usb::read_be32(req, 4) - 2;
    auto fail = [&] {
        update_.page = 0;
        update_.received = 0;
        update_.active = false;
        return kBadLength;
    };
    if (data_len > 4096) return fail();
    const auto data = req.subspan(9);
    const std::uint8_t check = data[data_len];
    update_.deadline = clock::now() + config_.update_timeout;
    if (update_.page == 0) {
        // Target: the slot that does not boot, from the boot record (update.c's first-page rule).
        const std::uint32_t new_app = le32(system_flash_, kBootRecord);
        const bool boot_new = system_flash_[kBootRecord + 8] == 1;
        const bool write_a = new_app == kSlotB ? boot_new : new_app == kSlotA ? !boot_new : false;
        update_.new_app = write_a ? kSlotA : kSlotB;
        update_.old_app = write_a ? kSlotB : kSlotA;
        update_.boot_new = 1;
    }
    // Erase the sector, program 4096 bytes (always a whole page), read back and sum the data length.
    const auto at = system_flash_.begin() + static_cast<std::ptrdiff_t>(update_.new_app + static_cast<std::uint32_t>(update_.page) * 4096);
    std::copy_n(data.begin(), 4096, at);
    std::uint8_t sum = 0;
    for (std::uint32_t i = 0; i < data_len; ++i) sum = static_cast<std::uint8_t>(sum + at[i]);
    if (sum != check) return fail();
    ++update_.page;
    update_.received += data_len;
    if (update_.size <= update_.received) {
        // Done: erase the boot-record sector, write the 12-byte record, reset (emc_wdg_thread).
        std::fill_n(system_flash_.begin() + kBootRecord, 4096, std::uint8_t{0xFF});
        const std::uint8_t record[12] = {
            static_cast<std::uint8_t>(update_.new_app), static_cast<std::uint8_t>(update_.new_app >> 8),
            static_cast<std::uint8_t>(update_.new_app >> 16), static_cast<std::uint8_t>(update_.new_app >> 24),
            static_cast<std::uint8_t>(update_.old_app), static_cast<std::uint8_t>(update_.old_app >> 8),
            static_cast<std::uint8_t>(update_.old_app >> 16), static_cast<std::uint8_t>(update_.old_app >> 24),
            update_.boot_new, 0, 0, 0};
        std::ranges::copy(record, system_flash_.begin() + kBootRecord);
        slot_firmware_[update_.new_app] = config_.updated_firmware.value_or(config_.firmware);
        update_.page = 0;
        update_.received = 0;
        update_.active = false;
        reboot = true;
    }
    return kOk;
}

bool SimDevice::take_image_halt(std::uint64_t generation) {
    std::lock_guard lock(mutex_);
    return alive_locked(generation) && image_halted_;
}

void SimDevice::clear_image_halt() {
    // The firmware's CLEAR_FEATURE(EP 0x83): GPIF, DMA channel and endpoint restarted, restart armed.
    std::lock_guard lock(mutex_);
    image_halted_ = false;
    ep83_halt_cleared_ = true;
}

bool SimDevice::stream_cycle(std::uint64_t generation, const usb::Transport::PacketHandler& handler, std::stop_token st,
                             std::atomic<std::uint64_t>& packets, std::atomic<std::uint64_t>& bytes) {
    State s;
    FrameProvider provider;
    {
        std::lock_guard lock(mutex_);
        if (!alive_locked(generation) || image_halted_) return false;
        s = state_locked();
        provider = provider_;
    }
    if (s.mono_triggers == 0 && s.rgb_triggers == 0) return false;
    // One trigger cycle: mono triggers produce IR pairs, rgb triggers add a colour frame.
    const int groups = std::max(s.mono_triggers, 1);
    const auto period = std::chrono::microseconds(s.trigger_period_us) / groups;
    auto next = clock::now();
    ImageU8 img(config_.width, config_.height);
    for (int g = 0; g < groups && !st.stop_requested(); ++g) {
        std::uint32_t frame_id;
        std::uint64_t ts;
        {
            std::lock_guard lock(mutex_);
            if (!alive_locked(generation) || image_halted_) return true;
            // Device clock: frames are stamped on a virtual timeline (frame index x trigger interval) so
            // time and emulated motion stay consistent even if the consumer applies backpressure.
            virtual_time_us_ += static_cast<std::uint64_t>(s.trigger_period_us / static_cast<std::uint32_t>(groups));
            ts = virtual_time_us_;
            frame_id = frame_id_++;
        }
        for (int sensor = 0; sensor < 2; ++sensor) {
            provider(sensor, frame_id, img);
            // As the scanner sends it: this camera is mounted upside down (usb::kUpsideDownSensor).
            if (sensor == usb::kUpsideDownSensor) std::ranges::reverse(img.pixels());
            emit_frame(sensor, frame_id, ts, img, handler, packets, bytes);
        }
        if (g == 0 && s.rgb_triggers > 0) {
            provider(2, frame_id, img);
            emit_frame(2, frame_id, ts, img, handler, packets, bytes);
        }
        next += period;
        std::this_thread::sleep_until(next);
    }
    return true;
}

void SimDevice::emit_frame(int sensor, std::uint32_t frame_id, std::uint64_t timestamp, const ImageU8& img,
                           const usb::Transport::PacketHandler& handler, std::atomic<std::uint64_t>& packets,
                           std::atomic<std::uint64_t>& bytes) {
    const std::size_t total = img.size();
    // One device packet per DMA buffer (0xA020 bytes): 32-byte header + up to kStreamMaxPayload pixels.
    std::vector<std::uint8_t> packet(usb::kStreamDeviceBufferSize);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    for (std::size_t off = 0; off < total; off += usb::kStreamMaxPayload) {
        const std::size_t n = std::min(usb::kStreamMaxPayload, total - off);
        std::fill(packet.begin(), packet.begin() + usb::kStreamHeaderSize, 0);
        packet[2] = static_cast<std::uint8_t>(((1u << sensor) & 0x7) << 3);
        packet[8] = off + n == total ? 1 : 0;
        usb::write_be32(packet, 12, frame_id & 0xFFu);  // the scanner's group id counts modulo 256
        usb::write_be32(packet, 16, static_cast<std::uint32_t>(timestamp >> 32));
        usb::write_be32(packet, 20, static_cast<std::uint32_t>(timestamp));
        std::memcpy(packet.data() + usb::kStreamHeaderSize, img.data() + off, n);
        bool drop = false;
        if (config_.packet_drop_rate > 0) {
            std::lock_guard lock(mutex_);
            drop = u(rng_) < config_.packet_drop_rate;
        }
        if (drop) continue;
        packets.fetch_add(1);
        bytes.fetch_add(usb::kStreamHeaderSize + n);
        handler(std::span(packet.data(), usb::kStreamHeaderSize + n));
    }
}

// ---------------------------------------------------------------------------------------------------------
// SimTransport

SimTransport::SimTransport(std::shared_ptr<SimDevice> device, std::uint64_t generation)
    : device_(std::move(device)), generation_(generation) {}

SimTransport::~SimTransport() { stop_stream(); }

usb::TransportStats SimTransport::stats() const {
    usb::TransportStats s;
    s.stream_packets = packets_;
    s.stream_bytes = bytes_;
    s.stream_stalls = stalls_;
    s.commands = commands_;
    return s;
}

Result<std::vector<std::uint8_t>> SimTransport::command(std::span<const std::uint8_t> request, std::size_t cap, unsigned) {
    ++commands_;
    return device_->command(generation_, request, cap);
}

Result<std::vector<std::uint8_t>> SimTransport::bulk(std::span<const std::uint8_t> request, std::size_t reply_size, unsigned) {
    // As LibusbTransport: pad to the firmware's bulk buffer.
    if (request.size() > usb::kBulkRequestSize) return make_error(Errc::invalid_argument, "bulk request too long");
    std::vector<std::uint8_t> padded(request.begin(), request.end());
    padded.resize(usb::kBulkRequestSize, 0);
    return device_->bulk(generation_, padded, reply_size);
}

Result<void> SimTransport::start_stream(PacketHandler handler) {
    if (stream_thread_.joinable()) return make_error(Errc::busy, "stream running");
    if (!device_->alive(generation_)) return make_error(Errc::disconnected, "emulated scanner left the bus");
    handler_ = std::move(handler);
    stream_thread_ = std::jthread([this](std::stop_token st) { stream_loop(st); });
    return {};
}

void SimTransport::stop_stream() {
    if (!stream_thread_.joinable()) return;
    stream_thread_.request_stop();
    stream_thread_.join();
    stream_thread_ = {};
}

void SimTransport::stream_loop(std::stop_token st) {
    while (!st.stop_requested()) {
        if (device_->take_image_halt(generation_)) {
            // As LibusbTransport: clear the halt and carry on.
            stalls_.fetch_add(1);
            device_->clear_image_halt();
            continue;
        }
        if (!device_->stream_cycle(generation_, handler_, st, packets_, bytes_))
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

SimScanner make_sim_scanner(SimConfig config) {
    SimScanner s;
    s.device = std::make_shared<SimDevice>(std::move(config));
    s.transport = std::move(*s.device->connect());
    return s;
}

}  // namespace einstar::sim
