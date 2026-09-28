#pragma once

// Command-channel packet framing and the optional XOR mask.
//
// Request:  [0] sequence  [1] 0x00 (mask key after masking)  [2] group  [3] opcode  [4..] body
// Reply:    [0] seq echo  [1] 0x00 / key / 0x02  [2] group echo  [3] opcode echo  [4] status (0 = OK)
//           [5..8] BE32 payload length  [9..] payload
//
// EXStar never enables masking on transmit; decode is applied to replies unless byte1 == 0x02
// (a no-op for unmasked replies).

#include <cstdint>
#include <span>
#include <vector>

#include "einstar/core/error.hpp"

namespace einstar::usb {

inline constexpr std::uint8_t kGroupInternal = 0x00;  // serial / product-name queries
inline constexpr std::uint8_t kGroupDevice = 0x10;    // device commands (trigger, exposure, ...)

inline constexpr std::uint8_t kSeqSerial = 0xFF;
inline constexpr std::uint8_t kSeqProductName = 0xFE;

struct CommandHeader {
    std::uint8_t sequence = 0;
    std::uint8_t group = 0;
    std::uint8_t opcode = 0;
};

// Builds a request of exactly `total_size` bytes (zero padded); body is copied from byte 4.
[[nodiscard]] std::vector<std::uint8_t> build_request(CommandHeader h, std::span<const std::uint8_t> body,
                                                      std::size_t total_size);

// Device-command form: bytes 4..7 hold the BE32 payload length, the payload starts at byte 8.
[[nodiscard]] std::vector<std::uint8_t> build_device_request(CommandHeader h, std::span<const std::uint8_t> payload,
                                                             std::size_t total_size);

// XOR every byte with k*0x11 (k in 0..15). Applying it twice restores the input.
void mask_encode(std::span<std::uint8_t> buffer, std::uint8_t key);
// Recovers the key from the high nibble of byte 1 and removes the mask (skipped when byte1 == 0x02).
void mask_decode(std::span<std::uint8_t> buffer);

struct Reply {
    std::vector<std::uint8_t> raw;
    [[nodiscard]] std::uint8_t status() const { return raw.size() > 4 ? raw[4] : 0xFF; }
    // Length-prefixed payload (bytes 5..8 BE32, data from byte 9), clamped to what was received.
    [[nodiscard]] std::span<const std::uint8_t> payload() const;
};

// Checks that bytes 0, 2, 3 echo the request and (optionally) status byte 4 == 0.
[[nodiscard]] Result<Reply> validate_reply(std::span<const std::uint8_t> request, std::vector<std::uint8_t> reply,
                                           bool require_status_ok);

[[nodiscard]] std::uint32_t read_be32(std::span<const std::uint8_t> b, std::size_t offset);
[[nodiscard]] std::uint64_t read_be64(std::span<const std::uint8_t> b, std::size_t offset);
void write_be32(std::span<std::uint8_t> b, std::size_t offset, std::uint32_t v);

}  // namespace einstar::usb
