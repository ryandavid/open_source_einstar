#pragma once

// Single source of truth for every command the host may send (docs/protocol-device.md §3.1).
// Anything not listed here, and anything marked `dangerous`, is refused by DeviceGuard.

#include <array>
#include <cstdint>
#include <string_view>

namespace einstar::device {

enum class Safety : std::uint8_t {
    read,            // no side effects
    volatile_write,  // changes runtime state only (trigger, exposure, LEDs...)
    dangerous,       // persistent or bricking risk: flash write, firmware, bootloader, reboot
};

enum class Channel : std::uint8_t { command, bulk };

struct OpcodeInfo {
    std::uint8_t group;
    std::uint8_t opcode;
    std::string_view name;
    Safety safety;
    Channel channel;
    std::uint16_t buffer;  // request buffer size = reply capacity
};

namespace op {
// group 0x00: system / info
inline constexpr OpcodeInfo kVendorName{0x00, 0x00, "VendorName", Safety::read, Channel::command, 100};
inline constexpr OpcodeInfo kProductName{0x00, 0x01, "ProductName", Safety::read, Channel::command, 100};
inline constexpr OpcodeInfo kSerial{0x00, 0x04, "Serial", Safety::read, Channel::command, 64};
inline constexpr OpcodeInfo kFirmwareVersion{0x00, 0x05, "FirmwareVersion", Safety::read, Channel::command, 64};
inline constexpr OpcodeInfo kFirmwareUpdate{0x00, 0x06, "FirmwareUpdate", Safety::dangerous, Channel::bulk, 5120};
inline constexpr OpcodeInfo kDeviceState{0x00, 0x07, "DeviceState", Safety::read, Channel::command, 50};
inline constexpr OpcodeInfo kReboot{0x00, 0x08, "Reboot", Safety::dangerous, Channel::command, 10};
// group 0x10: controller / sensor registers
inline constexpr OpcodeInfo kVendorId{0x10, 0x00, "Vid", Safety::read, Channel::command, 16};
inline constexpr OpcodeInfo kProductId{0x10, 0x01, "Pid", Safety::read, Channel::command, 16};
inline constexpr OpcodeInfo kMaxWidth{0x10, 0x16, "MaxWidth", Safety::read, Channel::command, 16};
inline constexpr OpcodeInfo kMaxHeight{0x10, 0x17, "MaxHeight", Safety::read, Channel::command, 16};
inline constexpr OpcodeInfo kMaxExposure{0x10, 0x20, "MaxExposure", Safety::read, Channel::command, 16};
inline constexpr OpcodeInfo kMinExposure{0x10, 0x21, "MinExposure", Safety::read, Channel::command, 16};
inline constexpr OpcodeInfo kGetExposure{0x10, 0x22, "GetExposure", Safety::read, Channel::command, 16};
inline constexpr OpcodeInfo kSetExposure{0x10, 0x23, "SetExposure", Safety::volatile_write, Channel::command, 16};
inline constexpr OpcodeInfo kMaxGain{0x10, 0x24, "MaxGain", Safety::read, Channel::command, 16};
inline constexpr OpcodeInfo kMinGain{0x10, 0x25, "MinGain", Safety::read, Channel::command, 16};
inline constexpr OpcodeInfo kGetGain{0x10, 0x26, "GetGain", Safety::read, Channel::command, 16};
inline constexpr OpcodeInfo kSetGain{0x10, 0x27, "SetGain", Safety::volatile_write, Channel::command, 16};
inline constexpr OpcodeInfo kPixelBits{0x10, 0x2E, "PixelBits", Safety::read, Channel::command, 16};
inline constexpr OpcodeInfo kGetTrigger{0x10, 0x40, "GetTrigger", Safety::read, Channel::command, 10};
inline constexpr OpcodeInfo kSetTrigger{0x10, 0x41, "SetTrigger", Safety::volatile_write, Channel::command, 10};
inline constexpr OpcodeInfo kGetTriggerPeriod{0x10, 0x48, "GetTriggerPeriod", Safety::read, Channel::command, 50};
inline constexpr OpcodeInfo kSetTriggerPeriod{0x10, 0x49, "SetTriggerPeriod", Safety::volatile_write, Channel::command, 50};
inline constexpr OpcodeInfo kTemperature{0x10, 0x50, "Temperature", Safety::read, Channel::command, 50};
inline constexpr OpcodeInfo kSensorCount{0x10, 0x51, "SensorCount", Safety::read, Channel::command, 16};
inline constexpr OpcodeInfo kFlashRead{0x10, 0x57, "FlashRead", Safety::read, Channel::bulk, 5120};
inline constexpr OpcodeInfo kFlashWrite{0x10, 0x58, "FlashWrite", Safety::dangerous, Channel::bulk, 5120};
// Not queried: for a 1-byte payload the firmware leaves the reply byte unwritten (docs/firmware.md 5.1).
inline constexpr OpcodeInfo kColorMode{0x10, 0x5D, "ColorMode", Safety::read, Channel::command, 16};
inline constexpr OpcodeInfo kIndication{0x10, 0x62, "Indication", Safety::volatile_write, Channel::command, 16};
inline constexpr OpcodeInfo kGetLaser{0x10, 0x67, "GetLaser", Safety::read, Channel::command, 50};
inline constexpr OpcodeInfo kSetLaser{0x10, 0x68, "SetLaser", Safety::volatile_write, Channel::command, 50};
inline constexpr OpcodeInfo kGetStrobe{0x10, 0x6F, "GetStrobe", Safety::read, Channel::command, 50};
inline constexpr OpcodeInfo kSetStrobe{0x10, 0x70, "SetStrobe", Safety::volatile_write, Channel::command, 50};
inline constexpr OpcodeInfo kClearState{0x10, 0x7B, "ClearState", Safety::volatile_write, Channel::command, 10};
// group 0xCC: bootloader
inline constexpr OpcodeInfo kEraseAppHeader{0xCC, 0x00, "EraseAppHeader", Safety::dangerous, Channel::command, 16};
}  // namespace op

inline constexpr std::array kAllOpcodes{
    op::kVendorName, op::kProductName, op::kSerial, op::kFirmwareVersion, op::kFirmwareUpdate, op::kDeviceState,
    op::kReboot, op::kVendorId, op::kProductId, op::kMaxWidth, op::kMaxHeight, op::kMaxExposure, op::kMinExposure,
    op::kGetExposure, op::kSetExposure, op::kMaxGain, op::kMinGain, op::kGetGain, op::kSetGain, op::kPixelBits,
    op::kGetTrigger, op::kSetTrigger, op::kGetTriggerPeriod, op::kSetTriggerPeriod, op::kTemperature,
    op::kSensorCount, op::kFlashRead, op::kFlashWrite, op::kColorMode, op::kIndication, op::kGetLaser, op::kSetLaser,
    op::kGetStrobe, op::kSetStrobe, op::kClearState, op::kEraseAppHeader,
};

[[nodiscard]] constexpr const OpcodeInfo* find_opcode(std::uint8_t group, std::uint8_t opcode) {
    for (const auto& o : kAllOpcodes)
        if (o.group == group && o.opcode == opcode) return &o;
    return nullptr;
}

// Compile-time gate: the typed API can only instantiate sends for non-dangerous opcodes.
template <const OpcodeInfo& Op>
concept SafeOpcode = Op.safety != Safety::dangerous;

// Runtime gate for anything built dynamically (e.g. a debug console).
[[nodiscard]] constexpr bool guard_allows(std::uint8_t group, std::uint8_t opcode) {
    const auto* o = find_opcode(group, opcode);
    return o != nullptr && o->safety != Safety::dangerous;
}

static_assert(!guard_allows(0x10, 0x58), "flash write must be blocked");
static_assert(!guard_allows(0x00, 0x06), "firmware update must be blocked");
static_assert(!guard_allows(0xCC, 0x00), "bootloader erase must be blocked");
static_assert(!guard_allows(0x10, 0x99), "unknown opcodes must be blocked");
static_assert(guard_allows(0x10, 0x57), "flash read is allowed");

// Hardware limits we never exceed (maxima observed from EXStar's own session).
inline constexpr int kMaxLaserPercent = 100;
inline constexpr int kMaxStrobeLuminance = 9000;
// The firmware ignores trigger periods outside this range but still replies OK.
inline constexpr std::uint32_t kMinTriggerPeriodUs = 1000;
inline constexpr std::uint32_t kMaxTriggerPeriodUs = 1000000;

}  // namespace einstar::device
