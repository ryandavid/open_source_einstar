#include "einstar/usb/command.hpp"

#include <algorithm>
#include <format>

namespace einstar::usb {

std::vector<std::uint8_t> build_request(CommandHeader h, std::span<const std::uint8_t> body, std::size_t total_size) {
    std::vector<std::uint8_t> out(std::max<std::size_t>(total_size, 4 + body.size()), 0);
    out[0] = h.sequence;
    out[1] = 0x00;
    out[2] = h.group;
    out[3] = h.opcode;
    std::ranges::copy(body, out.begin() + 4);
    return out;
}

std::vector<std::uint8_t> build_device_request(CommandHeader h, std::span<const std::uint8_t> payload,
                                               std::size_t total_size) {
    std::vector<std::uint8_t> out(std::max<std::size_t>(total_size, 8 + payload.size()), 0);
    out[0] = h.sequence;
    out[2] = h.group;
    out[3] = h.opcode;
    write_be32(out, 4, static_cast<std::uint32_t>(payload.size()));
    std::ranges::copy(payload, out.begin() + 8);
    return out;
}

void mask_encode(std::span<std::uint8_t> buffer, std::uint8_t key) {
    const auto m = static_cast<std::uint8_t>((key & 0x0F) * 0x11);
    for (auto& b : buffer) b ^= m;
}

void mask_decode(std::span<std::uint8_t> buffer) {
    if (buffer.size() < 2 || buffer[1] == 0x02) return;
    mask_encode(buffer, static_cast<std::uint8_t>(buffer[1] >> 4));
}

std::uint32_t read_be32(std::span<const std::uint8_t> b, std::size_t o) {
    return (std::uint32_t{b[o]} << 24) | (std::uint32_t{b[o + 1]} << 16) | (std::uint32_t{b[o + 2]} << 8) | b[o + 3];
}

std::uint64_t read_be64(std::span<const std::uint8_t> b, std::size_t o) {
    return (std::uint64_t{read_be32(b, o)} << 32) | read_be32(b, o + 4);
}

void write_be32(std::span<std::uint8_t> b, std::size_t o, std::uint32_t v) {
    b[o] = static_cast<std::uint8_t>(v >> 24);
    b[o + 1] = static_cast<std::uint8_t>(v >> 16);
    b[o + 2] = static_cast<std::uint8_t>(v >> 8);
    b[o + 3] = static_cast<std::uint8_t>(v);
}

std::span<const std::uint8_t> Reply::payload() const {
    if (raw.size() < 9) return {};
    const std::size_t n = std::min<std::size_t>(read_be32(raw, 5), raw.size() - 9);
    return std::span(raw).subspan(9, n);
}

std::string_view status_meaning(std::uint8_t status, bool bulk) {
    // docs/firmware.md 5 (the scanner's firmware).
    if (bulk) return status == 1 ? "unknown command" : status == 2 ? "bad length or update error" : "unknown status";
    return status == 2 ? "bad payload length or value out of range" : status == 3 ? "command failed or unknown" : "unknown status";
}

Result<Reply> validate_reply(std::span<const std::uint8_t> request, std::vector<std::uint8_t> reply,
                             bool require_status_ok) {
    if (reply.size() < 5) return make_error(Errc::protocol, std::format("short reply ({} bytes)", reply.size()));
    mask_decode(reply);
    if (reply[0] != request[0] || reply[2] != request[2] || reply[3] != request[3]) {
        return make_error(Errc::protocol, std::format("reply header mismatch: sent {:02X} {:02X} {:02X}, got {:02X} {:02X} {:02X}",
                                                      request[0], request[2], request[3], reply[0], reply[2], reply[3]));
    }
    if (require_status_ok && reply[4] != 0)
        return make_error(Errc::protocol, std::format("device status 0x{:02X} for opcode 0x{:02X}", reply[4], request[3]));
    return Reply{std::move(reply)};
}

}  // namespace einstar::usb
