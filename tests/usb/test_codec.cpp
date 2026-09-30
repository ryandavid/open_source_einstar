#include <catch2/catch_test_macros.hpp>

#include <vector>

#include "einstar/usb/command.hpp"
#include "einstar/usb/constants.hpp"
#include "einstar/usb/stream.hpp"

using namespace einstar;
using namespace einstar::usb;

TEST_CASE("mask matches the documented worked example") {
    std::vector<std::uint8_t> buf{0x07, 0x00, 0x10, 0x16, 0x00, 0x00, 0x00, 0x01, 0x02};
    mask_encode(buf, 3);
    REQUIRE(buf == std::vector<std::uint8_t>{0x34, 0x33, 0x23, 0x25, 0x33, 0x33, 0x33, 0x32, 0x31});
    mask_decode(buf);
    REQUIRE(buf == std::vector<std::uint8_t>{0x07, 0x00, 0x10, 0x16, 0x00, 0x00, 0x00, 0x01, 0x02});
}

TEST_CASE("mask decode leaves byte1 == 0x02 replies alone") {
    std::vector<std::uint8_t> buf{0x07, 0x02, 0x10, 0x16, 0xAA};
    const auto copy = buf;
    mask_decode(buf);
    REQUIRE(buf == copy);
}

TEST_CASE("device request layout: seq, 0, group, opcode, BE32 length, payload, zero padding") {
    const std::uint8_t payload[] = {0x00, 0x03, 0x0D, 0x40};
    const auto req = build_device_request({0x01, kGroupDevice, 0x49}, payload, 50);
    REQUIRE(req.size() == 50);
    const std::vector<std::uint8_t> head(req.begin(), req.begin() + 12);
    REQUIRE(head == std::vector<std::uint8_t>{0x01, 0x00, 0x10, 0x49, 0x00, 0x00, 0x00, 0x04, 0x00, 0x03, 0x0D, 0x40});
    for (std::size_t i = 12; i < req.size(); ++i) REQUIRE(req[i] == 0);
}

TEST_CASE("reply validation checks echo and status") {
    const auto req = build_device_request({0x05, kGroupDevice, 0x51}, {}, 16);
    REQUIRE(validate_reply(req, {0x05, 0, 0x10, 0x51, 0, 0, 0, 0, 1, 3}, true).has_value());
    REQUIRE_FALSE(validate_reply(req, {0x06, 0, 0x10, 0x51, 0, 0, 0, 0, 1, 3}, true).has_value());
    REQUIRE_FALSE(validate_reply(req, {0x05, 0, 0x10, 0x51, 1, 0, 0, 0, 0}, true).has_value());
    // Masked reply decodes before validation.
    std::vector<std::uint8_t> masked{0x05, 0, 0x10, 0x51, 0, 0, 0, 0, 1, 3};
    mask_encode(masked, 9);
    auto r = validate_reply(req, masked, true);
    REQUIRE(r.has_value());
    REQUIRE(r->payload().size() == 1);
    REQUIRE(r->payload()[0] == 3);
}

namespace {

std::vector<std::vector<std::uint8_t>> packetize(int sensor, std::uint32_t frame_id, std::uint64_t ts,
                                                 const std::vector<std::uint8_t>& pixels) {
    std::vector<std::vector<std::uint8_t>> out;
    for (std::size_t off = 0; off < pixels.size(); off += kStreamMaxPayload) {
        const std::size_t n = std::min(kStreamMaxPayload, pixels.size() - off);
        std::vector<std::uint8_t> p(kStreamHeaderSize + n, 0);
        p[2] = static_cast<std::uint8_t>((1u << sensor) << 3) | 0x5;
        p[8] = off + n == pixels.size() ? 1 : 0;
        write_be32(p, 12, frame_id);
        write_be32(p, 16, static_cast<std::uint32_t>(ts >> 32));
        write_be32(p, 20, static_cast<std::uint32_t>(ts));
        std::copy(pixels.begin() + static_cast<std::ptrdiff_t>(off), pixels.begin() + static_cast<std::ptrdiff_t>(off + n),
                  p.begin() + kStreamHeaderSize);
        out.push_back(std::move(p));
    }
    return out;
}

}  // namespace

TEST_CASE("frame assembler rebuilds full frames and resyncs after a lost packet") {
    constexpr int W = 1280, H = 1024;
    std::vector<std::uint8_t> pixels(W * H);
    for (std::size_t i = 0; i < pixels.size(); ++i) pixels[i] = static_cast<std::uint8_t>(i * 31 + 7);

    std::vector<StreamFrame> frames;
    FrameAssembler fa(W, H, [&](StreamFrame&& f) { frames.push_back(std::move(f)); });

    const auto a = packetize(1, 100, 0x0102030405060708ULL, pixels);
    REQUIRE(a.size() == 32);  // 1 310 720 bytes = 31 full packets + 1 partial
    for (const auto& p : a) fa.push(p);
    REQUIRE(frames.size() == 1);
    CHECK(frames[0].sensor == 1);
    CHECK(frames[0].frame_id == 100);
    CHECK(frames[0].timestamp == 0x0102030405060708ULL);
    CHECK(frames[0].subfield == 5);
    CHECK(std::equal(pixels.begin(), pixels.end(), frames[0].pixels.data()));

    // Drop a middle packet: that frame is discarded, the next one arrives intact.
    auto b = packetize(0, 101, 1, pixels);
    b.erase(b.begin() + 10);
    for (const auto& p : b) fa.push(p);
    for (const auto& p : packetize(0, 102, 2, pixels)) fa.push(p);
    REQUIRE(frames.size() == 2);
    CHECK(frames[1].frame_id == 102);
    CHECK(fa.stats().resyncs == 1);
}

TEST_CASE("group assembler pairs IR frames and emits IR-only groups when RGB is missing") {
    std::vector<FrameGroup> groups;
    GroupAssembler ga([&](FrameGroup&& g) { groups.push_back(std::move(g)); });
    auto frame = [](int sensor, std::uint32_t id) {
        StreamFrame f;
        f.sensor = sensor;
        f.frame_id = id;
        return f;
    };
    ga.set_expected_mask(0b011);
    ga.push(frame(0, 1));
    ga.push(frame(1, 1));
    REQUIRE(groups.size() == 1);
    ga.set_expected_mask(0b111);
    ga.push(frame(0, 2));
    ga.push(frame(1, 2));
    ga.push(frame(0, 3));  // group 2 never got RGB
    REQUIRE(groups.size() == 2);
    CHECK(ga.stats().ir_only_emitted == 1);
    ga.push(frame(1, 3));
    ga.push(frame(2, 3));
    REQUIRE(groups.size() == 3);
    CHECK(groups[2].mask() == 0b111);
}
