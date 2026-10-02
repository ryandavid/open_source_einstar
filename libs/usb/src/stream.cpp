#include "einstar/usb/stream.hpp"

#include <algorithm>
#include <cstring>

#include "einstar/usb/command.hpp"
#include "einstar/usb/constants.hpp"

namespace einstar::usb {

int sensor_from_camera_bits(std::uint8_t bits) {
    switch (bits) {
        case 1: return 0;
        case 2: return 1;
        case 4: return 2;
        default: return -1;
    }
}

FrameAssembler::FrameAssembler(int width, int height, Sink sink)
    : width_(width), height_(height), frame_bytes_(static_cast<std::size_t>(width) * static_cast<std::size_t>(height)), sink_(std::move(sink)) {}

void FrameAssembler::reset() {
    received_ = 0;
    current_ = {};
}

void FrameAssembler::start_frame(std::span<const std::uint8_t> packet) {
    received_ = 0;
    current_.sensor = sensor_from_camera_bits(static_cast<std::uint8_t>((packet[2] >> 3) & 0x7));
    current_.subfield = static_cast<std::uint8_t>(packet[2] & 0x7);
    current_.timestamp = read_be64(packet, 16);
    if (current_.pixels.width() != width_ || current_.pixels.height() != height_) current_.pixels = ImageU8(width_, height_);
}

void FrameAssembler::push(std::span<const std::uint8_t> packet) {
    ++stats_.packets;
    stats_.max_packet = std::max(stats_.max_packet, packet.size());
    if (packet.size() <= kStreamHeaderSize || packet.size() > kStreamTransferSize) {
        ++stats_.bad_packets;
        return;
    }
    const bool eof = packet[8] == 1;
    const std::size_t n = packet.size() - kStreamHeaderSize;
    if (received_ == 0) start_frame(packet);

    if (eof && received_ + n != frame_bytes_) {
        ++stats_.resyncs;
        received_ = 0;
        return;
    }
    if (received_ + n > frame_bytes_) {
        ++stats_.overflows;
        start_frame(packet);
    }
    const std::size_t take = std::min(n, frame_bytes_ - received_);
    std::memcpy(current_.pixels.data() + received_, packet.data() + kStreamHeaderSize, take);
    received_ += n;

    if (eof) {
        if (received_ == frame_bytes_) {
            current_.frame_id = read_be32(packet, 12);
            if (current_.sensor < 0) {
                ++stats_.bad_camera;
            } else {
                ++stats_.frames;
                StreamFrame done = std::move(current_);
                current_ = {};
                sink_(std::move(done));
            }
        }
        received_ = 0;
    }
}

void GroupAssembler::emit_or_drop() {
    if (!current_) return;
    const unsigned m = current_->mask();
    if ((m & expected_) == expected_) {
        ++stats_.groups;
        sink_(std::move(*current_));
    } else if ((m & 0b011) == 0b011) {
        // Both IR images present: usable for depth even without the colour frame.
        ++stats_.groups;
        ++stats_.ir_only_emitted;
        sink_(std::move(*current_));
    } else {
        ++stats_.incomplete_dropped;
    }
    current_.reset();
}

void GroupAssembler::push(StreamFrame&& frame) {
    if (frame.sensor < 0 || frame.sensor > 2) return;
    if (current_ && current_->frame_id != frame.frame_id) emit_or_drop();
    if (!current_) {
        current_.emplace();
        current_->frame_id = frame.frame_id;
        current_->timestamp = frame.timestamp;
    }
    const int s = frame.sensor;
    current_->sensors[static_cast<std::size_t>(s)] = std::move(frame);
    if ((current_->mask() & expected_) == expected_) emit_or_drop();
}

void GroupAssembler::flush() { emit_or_drop(); }

}  // namespace einstar::usb
