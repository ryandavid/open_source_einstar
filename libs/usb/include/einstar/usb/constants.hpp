#pragma once

// USB-level constants for the Shining3D Einstar (internal name "E10" / "EINSCAN10").
// Source: docs/protocol-transport.md (static analysis of EXStar's transport layer).

#include <array>
#include <cstddef>
#include <cstdint>

namespace einstar::usb {

inline constexpr std::uint16_t kVendorId = 0x3267;
// The firmware descriptors say 0x0003; EXStar's config mentions 0x0002. EXStar itself accepts any PID
// under the vendor id and filters on the product-name string, so we do the same.
inline constexpr std::array<std::uint16_t, 2> kKnownProductIds{0x0003, 0x0002};
inline constexpr std::string_view kProductNamePrefix = "EINSCAN10";

inline constexpr int kInterface = 0;
inline constexpr std::uint8_t kEpCommandOut = 0x01;
inline constexpr std::uint8_t kEpCommandIn = 0x81;
inline constexpr std::uint8_t kEpBulkOut = 0x02;
inline constexpr std::uint8_t kEpBulkIn = 0x82;
inline constexpr std::uint8_t kEpStreamIn = 0x83;

inline constexpr unsigned kCommandTimeoutMs = 1000;
inline constexpr std::size_t kBulkChunk = 1024;

// Image stream packetisation.
inline constexpr std::size_t kStreamPacketSize = 0xA400;  // 41984 bytes per bulk transfer
inline constexpr std::size_t kStreamHeaderSize = 32;
inline constexpr std::size_t kStreamPayloadPerPacket = kStreamPacketSize - kStreamHeaderSize;
inline constexpr unsigned kStreamTransferTimeoutMs = 500;
inline constexpr std::size_t kStreamTransfersInFlight = 32;

// Sensor 1 (the second IR camera) is mounted upside down: its frames arrive rotated by 180 degrees
// relative to the calibration (measured on the scanner: only that rotation gives stereo depth).
// EinstarDevice returns it upright; the emulator sends it rotated like the scanner.
inline constexpr int kUpsideDownSensor = 1;

}  // namespace einstar::usb
