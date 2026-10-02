#pragma once

// Image-stream reassembly for EP 0x83.
//
// Each bulk transfer carries one packet: a 32-byte header followed by 8-bit pixels.
//   byte 2       bits 5..3 = camera bit (1, 2, 4 -> sensor 0, 1, 2); bits 2..0 = sub-field
//   byte 8       1 = last packet of the frame
//   bytes 12..15 BE32 frame/group id (valid in the last packet)
//   bytes 16..23 BE64 timestamp (valid in the first packet)
// Frames are width*height bytes. Resync rules follow docs/protocol-transport.md §3.6.

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>

#include "einstar/core/image.hpp"

namespace einstar::usb {

struct StreamFrame {
    int sensor = -1;               // 0, 1, 2
    std::uint8_t subfield = 0;     // header byte 2 bits 2..0
    std::uint32_t frame_id = 0;
    std::uint64_t timestamp = 0;
    ImageU8 pixels;                // width x height, row-major
};

struct StreamStats {
    std::uint64_t packets = 0;
    std::uint64_t bad_packets = 0;      // length outside (32, transfer size]
    std::size_t max_packet = 0;         // longest packet seen (the firmware's DMA buffer allows 0xA020)
    std::uint64_t frames = 0;
    std::uint64_t resyncs = 0;          // EOF arrived with the wrong byte count
    std::uint64_t overflows = 0;        // more bytes than a frame holds before EOF
    std::uint64_t bad_camera = 0;       // camera bits not one of 1/2/4
};

[[nodiscard]] int sensor_from_camera_bits(std::uint8_t bits);  // -1 if invalid

class FrameAssembler {
public:
    using Sink = std::function<void(StreamFrame&&)>;
    FrameAssembler(int width, int height, Sink sink);

    // Feed one packet exactly as received (header + payload, `packet.size()` = actual length).
    void push(std::span<const std::uint8_t> packet);
    void reset();

    [[nodiscard]] const StreamStats& stats() const { return stats_; }

private:
    void start_frame(std::span<const std::uint8_t> packet);

    int width_, height_;
    std::size_t frame_bytes_;
    Sink sink_;
    std::size_t received_ = 0;
    StreamFrame current_;
    StreamStats stats_;
};

// Groups per-sensor frames sharing a frame id into a capture group (IR pair [+ RGB]).
struct FrameGroup {
    std::uint32_t frame_id = 0;
    std::uint64_t timestamp = 0;
    std::array<std::optional<StreamFrame>, 3> sensors;

    [[nodiscard]] unsigned mask() const {
        unsigned m = 0;
        for (std::size_t i = 0; i < sensors.size(); ++i)
            if (sensors[i]) m |= 1u << i;
        return m;
    }
};

struct GroupStats {
    std::uint64_t groups = 0;
    std::uint64_t incomplete_dropped = 0;
    std::uint64_t ir_only_emitted = 0;  // expected RGB but emitted the IR pair anyway
};

class GroupAssembler {
public:
    using Sink = std::function<void(FrameGroup&&)>;
    explicit GroupAssembler(Sink sink) : sink_(std::move(sink)) {}

    // Mask of sensors expected per group: 0b011 (scan, IR only) or 0b111 (with RGB).
    void set_expected_mask(unsigned mask) { expected_ = mask; }
    void push(StreamFrame&& frame);
    void flush();

    [[nodiscard]] const GroupStats& stats() const { return stats_; }

private:
    void emit_or_drop();

    Sink sink_;
    unsigned expected_ = 0b011;
    std::optional<FrameGroup> current_;
    GroupStats stats_;
};

}  // namespace einstar::usb
