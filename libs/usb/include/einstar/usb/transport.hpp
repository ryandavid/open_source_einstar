#pragma once

// Abstract link to the scanner. LibusbTransport talks to real hardware; the device emulator
// (libs/sim) implements the same interface so everything above it can run without a scanner.

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "einstar/core/error.hpp"

namespace einstar::usb {

struct TransportStats {
    std::uint64_t stream_packets = 0;
    std::uint64_t stream_bytes = 0;
    std::uint64_t stream_timeouts = 0;   // idle 500 ms transfers (normal when not triggering)
    std::uint64_t stream_errors = 0;
    std::uint64_t stream_stalls = 0;     // halts on the image endpoint, cleared and resumed (see LibusbTransport)
    std::uint64_t commands = 0;
    std::uint64_t command_failures = 0;
};

class Transport {
public:
    virtual ~Transport() = default;

    [[nodiscard]] virtual std::string description() const = 0;

    // Command channel (EP 0x01 -> 0x81). Sends `request` and reads up to `reply_capacity` bytes.
    virtual Result<std::vector<std::uint8_t>> command(std::span<const std::uint8_t> request, std::size_t reply_capacity,
                                                      unsigned timeout_ms) = 0;
    // Bulk channel (EP 0x02 -> 0x82). The request is padded to kBulkRequestSize (longer is an error); the
    // reply is read to exactly `reply_size` bytes, the fixed size the firmware sends for that command
    // (kBulkReplySize or kBulkPageReplySize). Reading more would wait for data that never comes: the
    // firmware ends its replies on a full packet, without a zero-length packet.
    virtual Result<std::vector<std::uint8_t>> bulk(std::span<const std::uint8_t> request, std::size_t reply_size,
                                                   unsigned timeout_ms) = 0;
    virtual Result<void> reset_command_pipe() = 0;
    virtual Result<void> reset_bulk_pipe() = 0;

    // Image stream (EP 0x83). The handler runs on the transport's thread and must be quick.
    using PacketHandler = std::function<void(std::span<const std::uint8_t> packet)>;
    virtual Result<void> start_stream(PacketHandler handler) = 0;
    virtual void stop_stream() = 0;

    [[nodiscard]] virtual TransportStats stats() const = 0;
};

struct UsbDeviceInfo {
    std::uint16_t vendor_id = 0;
    std::uint16_t product_id = 0;
    std::uint8_t bus = 0;
    std::uint8_t port = 0;
    std::string path;  // "bus-port.port..." for display / selection
};

// Lists attached devices with the Shining3D vendor id.
[[nodiscard]] Result<std::vector<UsbDeviceInfo>> enumerate_devices();

// Opens a device: claims interface 0, drains stale replies on 0x81/0x82, clears halt on both.
[[nodiscard]] Result<std::unique_ptr<Transport>> open_libusb(const UsbDeviceInfo& device);

// Opens the scanner again after it re-enumerated (the firmware reboots or restarts its USB side on its own;
// docs/firmware.md 5): the device at the same port if it is there, else the first one with the vendor id.
// The caller checks the serial.
[[nodiscard]] Result<std::unique_ptr<Transport>> reopen_libusb(const UsbDeviceInfo& previous);

}  // namespace einstar::usb
