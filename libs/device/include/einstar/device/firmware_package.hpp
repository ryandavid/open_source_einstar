#pragma once

// Scanner firmware update packages and version strings (docs/firmware.md 1, 4).
//
// A package is one flash slot -- the FX3 boot image at +0, the FPGA bitstream at +0x40000, 0xFF fill -- as
// 4096-byte pages, each followed by the 8-bit sum of the page. EXStar ships them
// (fabu_UPDATE/Configure/*_IAP.img) and firmware/tools/mkpackage builds ours the same way.

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "einstar/core/error.hpp"

namespace einstar::device {

inline constexpr std::size_t kFirmwarePage = 4096;
inline constexpr std::size_t kFirmwarePackagePage = kFirmwarePage + 1;  // + check byte
inline constexpr std::size_t kFirmwareFpgaOffset = 0x40000;
// A slot is 0x180000 bytes (A at 0x040000, B at 0x1C0000); EXStar's packages use 320 of its 384 pages.
inline constexpr int kFirmwareMaxPages = 0x180000 / 4096;

struct FirmwarePackage {
    std::vector<std::uint8_t> bytes;  // as sent: pages of 4096 data bytes + check byte
    int pages = 0;
    std::uint32_t data_size = 0;      // the size 00/06 announces: the data bytes, without the check bytes
    // The FX3 boot image ("CY" header, sections, entry, word-sum checksum), validated.
    std::uint32_t fx3_image_size = 0;
    int fx3_sections = 0;
    std::uint32_t fx3_entry = 0;
    std::uint32_t fx3_checksum = 0;
    std::uint32_t fpga_size = 0;      // bitstream length up to the trailing 0xFF fill
    std::uint32_t crc32 = 0;          // of the whole package, to tell packages apart

    [[nodiscard]] std::span<const std::uint8_t> page_with_check(int page) const {
        return std::span(bytes).subspan(static_cast<std::size_t>(page) * kFirmwarePackagePage, kFirmwarePackagePage);
    }
};

// Validates everything that can be checked offline: whole pages, every check byte, the FX3 image's format,
// sections and checksum, a bitstream at +0x40000, and the size limits of a slot.
[[nodiscard]] Result<FirmwarePackage> parse_firmware_package(std::vector<std::uint8_t> bytes);
[[nodiscard]] Result<FirmwarePackage> load_firmware_package(const std::string& path);

// "EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN", as command 00/05 reports it. Fields stay empty when the
// text does not have that shape.
struct FirmwareVersion {
    std::string text;
    std::string product;   // EinScan10_01
    std::string sensor;    // SC130
    std::string fx3;       // 2.10
    std::string fpga;      // 3.7
    std::string language;  // EN / CH
};
[[nodiscard]] FirmwareVersion parse_firmware_version(std::string_view text);

[[nodiscard]] std::uint32_t crc32(std::span<const std::uint8_t> data);

}  // namespace einstar::device
