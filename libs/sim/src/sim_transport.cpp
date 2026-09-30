#include "einstar/sim/sim_transport.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

#include "einstar/usb/command.hpp"
#include "einstar/usb/constants.hpp"

namespace einstar::sim {
namespace {

std::uint32_t be32_at(std::span<const std::uint8_t> b, std::size_t o) { return usb::read_be32(b, o); }

void put_be16(std::vector<std::uint8_t>& v, std::uint16_t x) {
    v.push_back(static_cast<std::uint8_t>(x >> 8));
    v.push_back(static_cast<std::uint8_t>(x));
}
void put_be32(std::vector<std::uint8_t>& v, std::uint32_t x) {
    for (int s = 24; s >= 0; s -= 8) v.push_back(static_cast<std::uint8_t>(x >> s));
}

int sensor_from_mask(std::uint8_t m) {
    switch (m) {
        case 1: return 0;
        case 2: return 1;
        case 4: return 2;
        default: return -1;
    }
}

bool is_dangerous(std::uint8_t group, std::uint8_t op) {
    return (group == 0x10 && op == 0x58) || (group == 0x00 && (op == 0x06 || op == 0x08)) || group == 0xCC;
}

}  // namespace

SimTransport::SimTransport(SimConfig config) : config_(std::move(config)), flash_(1u << 20, 0xFF), rng_(config_.seed) {
    last_command_seq_ = config_.previous_command_sequence;
    last_bulk_seq_ = config_.previous_bulk_sequence;
    provider_ = [](int sensor, std::uint32_t frame_id, ImageU8& out) {
        // Default content: a moving gradient so frames are distinguishable.
        for (int y = 0; y < out.height(); ++y)
            for (int x = 0; x < out.width(); ++x)
                out(x, y) = static_cast<std::uint8_t>((x + y + static_cast<int>(frame_id) * 7 + sensor * 50) & 0xFF);
    };
}

SimTransport::~SimTransport() { stop_stream(); }

void SimTransport::set_frame_provider(FrameProvider p) {
    std::lock_guard lock(mutex_);
    provider_ = std::move(p);
}

void SimTransport::set_flash(std::uint32_t offset, std::span<const std::uint8_t> data) {
    std::lock_guard lock(mutex_);
    std::ranges::copy(data, flash_.begin() + offset);
}

void SimTransport::press_button(int button, std::uint8_t action) {
    std::lock_guard lock(mutex_);
    buttons_[static_cast<std::size_t>(button)] = action;
}

std::vector<ReceivedCommand> SimTransport::received() const {
    std::lock_guard lock(mutex_);
    return received_;
}

SimTransport::State SimTransport::state() const {
    std::lock_guard lock(mutex_);
    return state_;
}

usb::TransportStats SimTransport::stats() const {
    usb::TransportStats s;
    s.stream_packets = packets_;
    s.stream_bytes = bytes_;
    s.commands = commands_;
    return s;
}

std::vector<std::uint8_t> SimTransport::handle(std::span<const std::uint8_t> req) {
    std::lock_guard lock(mutex_);
    ++commands_;
    const std::uint8_t seq = req[0], group = req[2], op = req[3];
    const std::uint32_t len = req.size() >= 8 ? be32_at(req, 4) : 0;
    std::span<const std::uint8_t> payload = req.size() >= 8 ? req.subspan(8, std::min<std::size_t>(len, req.size() - 8))
                                                            : std::span<const std::uint8_t>{};
    received_.push_back({seq, group, op, std::vector<std::uint8_t>(payload.begin(), payload.end())});
    if (const auto it = config_.status_override.find(static_cast<std::uint16_t>(group << 8 | op)); it != config_.status_override.end())
        return {seq, 0x00, group, op, it->second, 0, 0, 0, 0};
    if (is_dangerous(group, op)) dangerous_seen_ = true;

    std::uint8_t status = 0;
    std::vector<std::uint8_t> data;
    auto sensor = [&]() { return payload.empty() ? -1 : sensor_from_mask(payload[0]); };
    auto string_reply = [&](const std::string& s) { data.assign(s.begin(), s.end()); };

    if (group == 0x00) {
        switch (op) {
            case 0x00: string_reply(config_.vendor_name); break;
            case 0x01: string_reply(config_.product_name); break;
            case 0x04: data.assign(config_.serial.begin(), config_.serial.end()); break;
            case 0x05: string_reply(config_.firmware); break;
            case 0x07:
                data.assign(buttons_.begin(), buttons_.end());
                buttons_ = {};  // latched until read
                break;
            default: status = 1; break;
        }
    } else if (group == 0x10) {
        const int s = sensor();
        switch (op) {
            case 0x00: put_be16(data, usb::kVendorId); break;
            case 0x01: put_be16(data, usb::kKnownProductIds[0]); break;
            case 0x16: put_be16(data, static_cast<std::uint16_t>(config_.width)); break;
            case 0x17: put_be16(data, static_cast<std::uint16_t>(config_.height)); break;
            case 0x20: put_be32(data, config_.exposure_max); break;
            case 0x21: put_be32(data, config_.exposure_min); break;
            // Exposure as in the firmware: one 15-bit FPGA field for selectors 1..3 (sensors 0 and 1),
            // another for sensor 2; values above 50000 are ignored with status 0.
            case 0x22: if (s >= 0) put_be32(data, state_.exposure[static_cast<std::size_t>(s)]); else status = 2; break;
            case 0x23: {
                if (s < 0 || payload.size() < 5) { status = 2; break; }
                const std::uint32_t v = be32_at(payload, 1);
                if (v > 50000) break;
                const std::uint32_t field = v & 0x7FFF;
                if (s == 2) state_.exposure[2] = field;
                else state_.exposure[0] = state_.exposure[1] = field;
                break;
            }
            case 0x24: put_be16(data, config_.gain_max); break;
            case 0x25: put_be16(data, config_.gain_min); break;
            case 0x26: if (s >= 0) put_be16(data, state_.gain[static_cast<std::size_t>(s)]); else status = 2; break;
            case 0x27:
                if (s < 0 || payload.size() < 3) status = 2;
                else state_.gain[static_cast<std::size_t>(s)] = static_cast<std::uint16_t>((payload[1] << 8) | payload[2]);
                break;
            case 0x2E: data.push_back(s == 2 ? 8 : 8); break;
            case 0x40: data.push_back(static_cast<std::uint8_t>((state_.rgb_triggers << 4) | state_.mono_triggers)); break;
            case 0x41:
                if (payload.empty()) { status = 2; break; }
                state_.mono_triggers = payload[0] & 0x0F;
                state_.rgb_triggers = payload[0] >> 4;
                break;
            case 0x48: put_be32(data, state_.trigger_period_us); break;
            case 0x49: {  // out-of-range periods are ignored with status 0, as in the firmware
                if (payload.size() < 4) { status = 2; break; }
                const std::uint32_t v = be32_at(payload, 0);
                if (v >= 1000 && v <= 1000000) state_.trigger_period_us = v;
                break;
            }
            case 0x50: put_be16(data, static_cast<std::uint16_t>(static_cast<std::int16_t>(config_.temperature_c * 128.0))); break;
            case 0x51: data.push_back(3); break;
            case 0x5D: data.push_back(s == 2 ? 1 : 0); break;
            case 0x62:
                if (payload.size() < 2 || payload[0] > 2 || payload[1] > 1) { status = 2; break; }
                state_.indication_distance = payload[0];
                state_.indication_active = payload[1];
                break;
            case 0x67: data.push_back(static_cast<std::uint8_t>(state_.laser)); break;
            case 0x68: if (!payload.empty() && payload[0] <= 100) state_.laser = payload[0]; else status = 2; break;
            case 0x6F: if (!payload.empty() && payload[0] < 2) put_be16(data, static_cast<std::uint16_t>(state_.strobe[payload[0]])); else status = 2; break;
            case 0x70:
                if (payload.size() < 3 || payload[0] > 1) { status = 2; break; }
                state_.strobe[payload[0]] = (payload[1] << 8) | payload[2];
                break;
            case 0x7B: buttons_ = {}; break;
            case 0x57: {
                if (payload.size() < 2) { status = 2; break; }
                const std::size_t page = static_cast<std::size_t>((payload[0] << 8) | payload[1]);
                if (page >= flash_.size() / 4096) { status = 2; break; }
                data.assign(flash_.begin() + static_cast<std::ptrdiff_t>(page * 4096),
                            flash_.begin() + static_cast<std::ptrdiff_t>((page + 1) * 4096));
                break;
            }
            default: status = 1; break;
        }
    } else {
        status = 1;
    }

    {
        std::lock_guard olock(observer_->mutex);
        observer_->state = state_;
        observer_->received = received_;
    }
    std::vector<std::uint8_t> reply{seq, 0x00, group, op, status};
    put_be32(reply, static_cast<std::uint32_t>(data.size()));
    reply.insert(reply.end(), data.begin(), data.end());
    if (config_.mask_replies) {
        const auto key = static_cast<std::uint8_t>(1 + rng_() % 15);
        usb::mask_encode(reply, key);
    }
    return reply;
}

std::uint64_t SimTransport::dropped_repeats() const {
    std::lock_guard lock(mutex_);
    return dropped_repeats_;
}

Result<std::vector<std::uint8_t>> SimTransport::command(std::span<const std::uint8_t> request, std::size_t cap,
                                                        unsigned) {
    if (request.size() < 4) return make_error(Errc::invalid_argument, "short request");
    {
        std::lock_guard lock(mutex_);
        const std::uint8_t seq = request[0];
        const bool repeat = seq < 0xFE && last_command_seq_ == seq;
        last_command_seq_ = repeat ? std::nullopt : std::optional<std::uint8_t>(seq);  // a drop clears it
        if (repeat) {
            ++dropped_repeats_;
            return make_error(Errc::timeout, "no reply (repeated sequence number)");
        }
    }
    auto reply = handle(request);
    if (reply.size() > cap) reply.resize(cap);
    return reply;
}

Result<std::vector<std::uint8_t>> SimTransport::bulk(std::span<const std::uint8_t> request, std::size_t cap, unsigned) {
    if (request.size() % usb::kBulkChunk != 0)
        return make_error(Errc::protocol, "bulk request not padded to 1 KiB");  // the real transport pads
    {
        std::lock_guard lock(mutex_);
        const std::uint8_t seq = request[0];
        const bool repeat = last_bulk_seq_ == seq;  // (no exemption on the bulk channel)
        last_bulk_seq_ = repeat ? std::nullopt : std::optional<std::uint8_t>(seq);
        if (repeat) {
            ++dropped_repeats_;
            return make_error(Errc::timeout, "no reply (repeated sequence number)");
        }
    }
    auto reply = handle(request);
    if (reply.size() > cap) reply.resize(cap);
    return reply;
}

Result<void> SimTransport::start_stream(PacketHandler handler) {
    if (stream_thread_.joinable()) return make_error(Errc::busy, "stream running");
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

void SimTransport::emit_frame(int sensor, std::uint32_t frame_id, std::uint64_t timestamp, const ImageU8& img) {
    const std::size_t total = img.size();
    std::vector<std::uint8_t> packet(usb::kStreamPacketSize);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    for (std::size_t off = 0; off < total; off += usb::kStreamPayloadPerPacket) {
        const std::size_t n = std::min(usb::kStreamPayloadPerPacket, total - off);
        std::fill(packet.begin(), packet.begin() + usb::kStreamHeaderSize, 0);
        packet[2] = static_cast<std::uint8_t>(((1u << sensor) & 0x7) << 3);
        packet[8] = off + n == total ? 1 : 0;
        usb::write_be32(packet, 12, frame_id & 0xFFu);  // the scanner's group id counts modulo 256
        usb::write_be32(packet, 16, static_cast<std::uint32_t>(timestamp >> 32));
        usb::write_be32(packet, 20, static_cast<std::uint32_t>(timestamp));
        std::memcpy(packet.data() + usb::kStreamHeaderSize, img.data() + off, n);
        if (config_.packet_drop_rate > 0 && u(rng_) < config_.packet_drop_rate) continue;
        packets_++;
        bytes_ += usb::kStreamHeaderSize + n;
        handler_(std::span(packet.data(), usb::kStreamHeaderSize + n));
    }
}

void SimTransport::stream_loop(std::stop_token st) {
    using clock = std::chrono::steady_clock;
    std::uint32_t frame_id = 0;
    auto next = clock::now();
    ImageU8 img(config_.width, config_.height);
    while (!st.stop_requested()) {
        State s;
        FrameProvider provider;
        {
            std::lock_guard lock(mutex_);
            s = state_;
            provider = provider_;
        }
        if (s.mono_triggers == 0 && s.rgb_triggers == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            next = clock::now();
            continue;
        }
        // One trigger cycle: mono triggers produce IR pairs, rgb triggers add a colour frame.
        const int groups = std::max(s.mono_triggers, 1);
        const auto period = std::chrono::microseconds(s.trigger_period_us) / groups;
        for (int g = 0; g < groups && !st.stop_requested(); ++g) {
            // Device clock: frames are stamped on a virtual timeline (frame index x trigger interval) so
            // time and emulated motion stay consistent even if the consumer applies backpressure.
            virtual_time_us_ += static_cast<std::uint64_t>(s.trigger_period_us / static_cast<std::uint32_t>(groups));
            const std::uint64_t ts = virtual_time_us_;
            for (int sensor = 0; sensor < 2; ++sensor) {
                provider(sensor, frame_id, img);
                // As the scanner sends it: this camera is mounted upside down (usb::kUpsideDownSensor).
                if (sensor == usb::kUpsideDownSensor) std::ranges::reverse(img.pixels());
                emit_frame(sensor, frame_id, ts, img);
            }
            if (g == 0 && s.rgb_triggers > 0) {
                provider(2, frame_id, img);
                emit_frame(2, frame_id, ts, img);
            }
            ++frame_id;
            next += period;
            std::this_thread::sleep_until(next);
        }
    }
}

}  // namespace einstar::sim
