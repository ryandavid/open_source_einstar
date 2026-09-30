#pragma once

// USB-level constants for the Shining3D Einstar (internal name "E10" / "EINSCAN10").
// Source: docs/protocol-transport.md (static analysis of EXStar's transport layer).

#include <cstddef>
#include <cstdint>

namespace einstar::usb {

inline constexpr std::uint16_t kVendorId = 0x3267;
// Product ids: the firmware's USB descriptors say 0x0003 (its 10/01 command reports 1); EXStar's config
// mentions 0x0002. EXStar accepts any PID under the vendor id and filters on the product-name string, so we
// do the same.
inline constexpr std::string_view kProductNamePrefix = "EINSCAN10";

inline constexpr int kInterface = 0;
inline constexpr std::uint8_t kEpCommandOut = 0x01;
inline constexpr std::uint8_t kEpCommandIn = 0x81;
inline constexpr std::uint8_t kEpBulkOut = 0x02;
inline constexpr std::uint8_t kEpBulkIn = 0x82;
inline constexpr std::uint8_t kEpStreamIn = 0x83;

inline constexpr unsigned kCommandTimeoutMs = 1000;
inline constexpr std::size_t kBulkChunk = 1024;
// The firmware's bulk OUT DMA buffer. With 512-byte packets the FX3 only hands a request to the firmware
// once the buffer is full (or a short packet ends it), so every bulk request is exactly this long.
inline constexpr std::size_t kBulkRequestSize = 5120;
// Bulk replies are a fixed size set by the firmware, whatever the request: a page read (10/57) sends
// 5120 bytes (9-byte header + 4096 data + filler), everything else, errors included, 1024.
inline constexpr std::size_t kBulkReplySize = 1024;
inline constexpr std::size_t kBulkPageReplySize = 5120;

// Image stream packetisation.
inline constexpr std::size_t kStreamTransferSize = 0xA400;  // 41984 bytes per bulk IN transfer (as EXStar)
// The firmware's image DMA buffer (0xA020): one device packet (32-byte header + pixels) is at most this long.
inline constexpr std::size_t kStreamDeviceBufferSize = 0xA020;
inline constexpr std::size_t kStreamHeaderSize = 32;
inline constexpr std::size_t kStreamMaxPayload = kStreamDeviceBufferSize - kStreamHeaderSize;
inline constexpr unsigned kStreamTransferTimeoutMs = 500;
inline constexpr std::size_t kStreamTransfersInFlight = 32;

// Sensor 1 (the second IR camera) is mounted upside down: its frames arrive rotated by 180 degrees
// relative to the calibration (measured on the scanner: only that rotation gives stereo depth).
// EinstarDevice returns it upright; the emulator sends it rotated like the scanner.
inline constexpr int kUpsideDownSensor = 1;

}  // namespace einstar::usb
