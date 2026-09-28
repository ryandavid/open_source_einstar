# Scanner firmware (static analysis of the update package)

Source: `EXStar.app/Contents/MacOS/fabu_UPDATE/Configure/EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_{EN,CH}_IAP.img`
(the firmware the analysed scanner reports). Everything here comes from reading the package and
the FX3 application in Ghidra. Nothing was sent to a scanner. Behaviour is described; no vendor
code or data is reproduced. Confidence tags as in protocol-device.md: **[H]** read directly from
code or data, **[M]** inferred from structure, **[L]** guess.

## 1. Update package format **[H]**
* 320 pages of 4096 bytes, each followed by one check byte: the 8-bit sum of the page. Total
  1 311 040 bytes. This is the `0x1001`-byte block the host plugin sends (protocol-device.md 3.14).
* The EN and CH packages differ in 6 bytes: a language tag in the FX3 image, that page's check
  byte, the FX3 image checksum and its page's check byte.
* Payload (1 310 720 bytes) = the contents of one flash **slot**:
  * `+0x000000`: FX3 application in Cypress's boot-image format ("CY", control 0x1C, type 0xB0):
    5 sections (0x100 ITCM trampolines 9.5 KB; 0x40003000 64 KB; 0x40013000 64 KB; 0x40023000 28 KB;
    0x40030000 21.7 KB data), entry 0x40016F50, word-sum checksum valid.
  * `+0x040000`: FPGA bitstream, 1 006 076 bytes (8.05 Mbit), then 0xFF.

## 2. Hardware seen from the firmware **[H]** (roles **[M]**)
* **Cypress FX3** (ARM926, USB 3.0) running Cypress's SDK on the **ThreadX** RTOS. Threads: FPGA
  boot, command handling, control, and a USB watchdog that reboots the device on transfer errors.
* **SPI flash**, at least 5 MB (8 MB part likely): the slots, the boot record and a user area.
* **I2C**: an EEPROM at address 0xA0 (the FX3's first-stage boot image, see 4); three SmartSens
  sensors (SC130GS; code also handles an SC132 variant) selected via "LVDS_CAM1/2/4" GPIOs; an
  ADT7420 temperature sensor; the light/laser driver.
* **FPGA**: configured by the FX3 at every boot (a GPIO pulse on its program/reset line, then
  1.5 MB streamed from flash into its configuration port). It takes the sensors' LVDS
  outputs, and the FX3 writes its registers for triggers, laser and strobe. Images reach the FX3
  through its GPIF-II parallel port.

## 3. Flash layout **[H]**
| Address | Contents |
|---|---|
| 0x03F000 | boot record: new-slot address, old-slot address, "boot new" flag (12 bytes) |
| 0x040000 / 0x080000 | slot A: FX3 application / FPGA bitstream |
| 0x1C0000 / 0x200000 | slot B: FX3 application / FPGA bitstream |
| 0x400000–0x4FFFFF | user area, 256 × 4 KB pages: the calibration blob (pages 0–1) and custom info |

The page read/write commands (10/57, 10/58) take a page number below 256 and only reach the
user area. The slots, the boot record and the EEPROM cannot be read back with stock commands.

## 4. Boot chain and updates
1. FX3 boot ROM → first-stage image in the **I2C EEPROM** (Shining3D's bootloader; not in the
   package) **[M]**. It reads the boot record and loads the application from the selected slot **[M]**.
2. The application picks the FPGA bitstream of the slot it runs from and configures the FPGA **[H]**.
3. **Updates are done by the running application** (bulk 00/06) **[H]**. The size comes first; then
   each 4 KB page is written to the *inactive* slot: its sector is erased, the page written, read
   back and checked against the page's check byte. Only after the last page is the boot record
   rewritten to point at the new slot. An interrupted update leaves the old firmware booting.
   Whether the bootloader falls back to the old slot when a new application is valid but broken
   is not visible (the bootloader is not in the package).
4. **EEPROM invalidation** **[H]**: a USB control request (class, host-to-device, bRequest 9,
   wIndex 2, 64-byte data stage whose second byte is 0xAA) zeroes the
   first two bytes of the I2C EEPROM, i.e. the "CY" boot signature; 0xBB resets. On FX3 boards whose
   boot-mode pins allow it, the chip then boots into Cypress's ROM USB bootloader (VID 04B4,
   PID 00F3), which loads firmware into RAM over USB. Writing "CY" back restores the original boot.
   Whether this board's boot-mode pins allow USB fallback is **unknown** (hardware).
5. EXStar's "EraseAppHeader" (group 0xCC) is not handled by the application, so it is presumably
   a bootloader command **[M]**.

## 5. Command handling **[H]**
* Command channel: one large dispatcher; the bulk channel: a 3-entry table (00/06 update,
  10/57 page read, 10/58 page write). Each entry has a group/opcode key, an expected payload
  size, and a handler. The reply status byte means 0 ok, 1 unknown, 2 bad length, 3 failed.
  A repeated sequence number is ignored (retransmission).
* Sensor setup is register tables written over I2C at start-up ("Config cameras"). Exposure and gain
  commands write sensor registers (0x3E01/0x3E02/0x3E08/0x3E09 are read back). LED brightness and
  laser mode are FPGA register writes; the trigger probably is too **[M]**.

## 6. FPGA bitstream **[H]** (vendor unidentified)
* Uncompressed. After a lead-in word, 32-bit packets: a header whose top nibble is 0xA, with a
  6-bit register field and a word count, followed by data. `0xA0000000` is a no-op (the file
  ends in a long run of them). The packets write a dozen configuration registers, then frame data.
* It resembles Xilinx's type-1 packets but has none of the Xilinx, Lattice, Intel or Gowin sync
  words (plain or bit-reversed), and no vendor strings. Probably a Chinese vendor (Anlogic, Pango
  or similar) **[L]**. An 8 Mbit bitstream suggests a mid-size part. A board photo would settle it.

## 7. An open-source replacement: assessment
**FX3 firmware (keeping the vendor FPGA bitstream).** Feasible. The application is thin: USB
descriptors and endpoints, the command protocol (already documented and implemented host-side),
sensor register tables over I2C, temperature, lights, flash access, loading the FPGA, and
streaming from GPIF-II to bulk. Cypress's SDK is free but closed (it includes ThreadX), so
"fully open" would mean bare-metal FX3 code without the SDK, which is a much larger job. What it
would buy: control over frame timing, hardware timestamps, raw or packed pixel modes, diagnostics.
Nothing we need for scan quality today is blocked by the stock firmware.

**FPGA gateware.** Hard. It needs the part identified, the board's pinout (probing the hardware),
the vendor toolchain unless an open one supports the part, and the sensor LVDS receive, trigger and
laser timing, and FPGA-to-FX3 interface all re-implemented.

**Risk.**
* The stock A/B update protects against interrupted writes, but not against a valid-but-broken
  application.
* The EEPROM-invalidation route (4.4) would give RAM-only testing with no flash writes, but only if
  the boot-mode pins allow USB fallback. Otherwise it leaves a device that needs an EEPROM programmer.
* Neither the EEPROM nor the slots can be backed up through stock commands.

Every one of these operations stays blocked in `DeviceGuard`.

**Next steps if this is pursued:**
1. Photograph the board: the FPGA part, the boot-mode (PMODE) pin straps, and whether the EEPROM
   and flash are in reach of a clip for an external backup.
2. Take a full external backup of the EEPROM and flash.
3. Only then try RAM-loaded firmware through the USB-boot fallback.
