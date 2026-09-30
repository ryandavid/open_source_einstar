#include "einstar/device/firmware_package.hpp"

#include <algorithm>
#include <format>
#include <fstream>
#include <iterator>

namespace einstar::device {
namespace {

std::uint32_t le32(std::span<const std::uint8_t> b, std::size_t o) {
    return std::uint32_t{b[o]} | std::uint32_t{b[o + 1]} << 8 | std::uint32_t{b[o + 2]} << 16 | std::uint32_t{b[o + 3]} << 24;
}

// FX3 memory a boot-image section may load into: the 16 KB ITCM and the 512 KB system RAM.
bool loadable(std::uint32_t addr, std::uint64_t bytes) {
    const std::uint64_t end = addr + bytes;
    return end <= 0x4000 || (addr >= 0x40000000u && end <= 0x40080000u);
}

}  // namespace

std::uint32_t crc32(std::span<const std::uint8_t> data) {
    std::uint32_t c = 0xFFFFFFFFu;
    for (const auto b : data) {
        c ^= b;
        for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

Result<FirmwarePackage> parse_firmware_package(std::vector<std::uint8_t> bytes) {
    FirmwarePackage p;
    if (bytes.empty() || bytes.size() % kFirmwarePackagePage != 0)
        return make_error(Errc::invalid_argument,
                          std::format("{} bytes is not a whole number of {}-byte pages (4096 data + check byte)", bytes.size(),
                                      kFirmwarePackagePage));
    p.pages = static_cast<int>(bytes.size() / kFirmwarePackagePage);
    if (p.pages > kFirmwareMaxPages)
        return make_error(Errc::invalid_argument, std::format("{} pages do not fit a {}-page flash slot", p.pages, kFirmwareMaxPages));
    const auto fpga_page = static_cast<int>(kFirmwareFpgaOffset / kFirmwarePage);
    if (p.pages <= fpga_page)
        return make_error(Errc::invalid_argument, std::format("{} pages end before the FPGA bitstream at +0x40000", p.pages));

    // The slot's data, and every page's check byte.
    std::vector<std::uint8_t> slot;
    slot.reserve(static_cast<std::size_t>(p.pages) * kFirmwarePage);
    for (int i = 0; i < p.pages; ++i) {
        const auto page = std::span(bytes).subspan(static_cast<std::size_t>(i) * kFirmwarePackagePage, kFirmwarePackagePage);
        std::uint8_t sum = 0;
        for (std::size_t j = 0; j < kFirmwarePage; ++j) sum = static_cast<std::uint8_t>(sum + page[j]);
        if (sum != page[kFirmwarePage])
            return make_error(Errc::invalid_argument,
                              std::format("page {}: check byte {:#04x}, the data sums to {:#04x}", i, page[kFirmwarePage], sum));
        slot.insert(slot.end(), page.begin(), page.begin() + kFirmwarePage);
    }
    p.data_size = static_cast<std::uint32_t>(slot.size());

    // FX3 boot image: "CY", I2C config, image type 0xB0 (binary); sections {length in words, address, data};
    // a zero length ends them, followed by the entry address and the 32-bit sum of all section words.
    const std::span<const std::uint8_t> img(slot.data(), kFirmwareFpgaOffset);
    if (img[0] != 'C' || img[1] != 'Y' || img[3] != 0xB0)
        return make_error(Errc::invalid_argument, "no FX3 boot image (\"CY\", type 0xB0) at the start of the slot");
    std::size_t off = 4;
    std::uint32_t sum = 0;
    for (;;) {
        if (off + 8 > img.size()) return make_error(Errc::invalid_argument, "FX3 image: sections run past +0x40000");
        const std::uint32_t words = le32(img, off), addr = le32(img, off + 4);
        off += 8;
        if (words == 0) {
            p.fx3_entry = addr;
            break;
        }
        const std::uint64_t n = std::uint64_t{words} * 4;
        if (off + n > img.size()) return make_error(Errc::invalid_argument, "FX3 image: a section runs past +0x40000");
        if (!loadable(addr, n))
            return make_error(Errc::invalid_argument, std::format("FX3 image: section at {:#x} ({} bytes) is outside the FX3's memory", addr, n));
        for (std::size_t w = 0; w < words; ++w) sum += le32(img, off + 4 * w);
        off += static_cast<std::size_t>(n);
        ++p.fx3_sections;
    }
    if (off + 4 > img.size()) return make_error(Errc::invalid_argument, "FX3 image: no checksum before +0x40000");
    p.fx3_checksum = le32(img, off);
    if (p.fx3_checksum != sum)
        return make_error(Errc::invalid_argument, std::format("FX3 image: checksum {:#010x}, the sections sum to {:#010x}", p.fx3_checksum, sum));
    p.fx3_image_size = static_cast<std::uint32_t>(off + 4);
    if (p.fx3_sections == 0) return make_error(Errc::invalid_argument, "FX3 image: no sections");
    if (!loadable(p.fx3_entry, 4)) return make_error(Errc::invalid_argument, std::format("FX3 image: entry {:#x} is outside the FX3's memory", p.fx3_entry));

    // FPGA bitstream: whatever follows +0x40000 up to the 0xFF fill.
    const auto fpga = std::span(slot).subspan(kFirmwareFpgaOffset);
    const auto last = std::ranges::find_if(fpga.rbegin(), fpga.rend(), [](std::uint8_t b) { return b != 0xFF; });
    p.fpga_size = static_cast<std::uint32_t>(std::distance(last, fpga.rend()));
    if (p.fpga_size == 0) return make_error(Errc::invalid_argument, "no FPGA bitstream at +0x40000 (all 0xFF)");

    p.crc32 = crc32(bytes);
    p.bytes = std::move(bytes);
    return p;
}

Result<FirmwarePackage> load_firmware_package(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return make_error(Errc::not_found, std::format("cannot open {}", path));
    std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    auto p = parse_firmware_package(std::move(bytes));
    if (!p) return make_error(p.error().code, std::format("{}: {}", path, p.error().message));
    return p;
}

FirmwareVersion parse_firmware_version(std::string_view text) {
    FirmwareVersion v;
    v.text = std::string(text);
    // <product>_<sensor>_FX3_V<fx3>_FPGA_V<fpga>_<language>; the product itself contains one '_'.
    const auto fx3 = text.find("_FX3_V"), fpga = text.find("_FPGA_V");
    if (fx3 == std::string_view::npos || fpga == std::string_view::npos || fpga < fx3) return v;
    const auto head = text.substr(0, fx3);
    const auto sensor_sep = head.rfind('_');
    if (sensor_sep == std::string_view::npos) return v;
    const auto tail = text.substr(fpga + 7);
    const auto lang_sep = tail.find('_');
    v.product = std::string(head.substr(0, sensor_sep));
    v.sensor = std::string(head.substr(sensor_sep + 1));
    v.fx3 = std::string(text.substr(fx3 + 6, fpga - fx3 - 6));
    v.fpga = std::string(lang_sep == std::string_view::npos ? tail : tail.substr(0, lang_sep));
    v.language = lang_sep == std::string_view::npos ? std::string{} : std::string(tail.substr(lang_sep + 1));
    return v;
}

}  // namespace einstar::device
