# Scanner firmware (static analysis of the update package)

Source: `EXStar.app/Contents/MacOS/fabu_UPDATE/Configure/EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_{EN,CH}_IAP.img`
(the firmware the analysed scanner reports). Everything here comes from reading the package and
the FX3 application in Ghidra, cross-checked against a source reconstruction that rebuilds the
application byte for byte with Cypress's SDK (kept outside the repository). Nothing was sent to
a scanner. Behaviour is described; no vendor code or data is reproduced. Confidence tags as in
protocol-device.md: **[H]** read directly from code or data, **[M]** inferred from structure,
**[L]** guess.

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
* **Cypress FX3** (ARM926, USB 3.0) running Cypress's SDK on the **ThreadX** RTOS. Four threads:
  FPGA boot (peripherals, flash, FPGA configuration; then starts the next two), the bulk channel,
  the command channel, and a once-a-second watchdog (see 5.1).
* **SPI flash** (30 MHz, mode 3), at least 5 MB (8 MB part likely): the slots, the boot record and
  a user area. Winbond-style status registers: **at every boot the application clears the
  block-protection bits and sets QE** (SR1 = 0, SR2 = 0x02, SR3 = 0), so the flash is never
  write-protected.
* **I2C** (100 kHz): an EEPROM at address 0xA0 (the FX3's first-stage boot image, see 4); three
  SmartSens sensors (SC130GS; code also handles an SC132 variant); an ADT7420 temperature sensor;
  the light/laser driver (address 0xA6, chip unidentified); and the FPGA's register interface
  (address 0x20: numbered 32-bit big-endian registers). Most devices sit behind a switch: a 4-bit
  code on GPIOs 23/25/26/27 selects cameras 1, 2 and 4 (codes 1, 2, 4), the FPGA (5), the light
  driver (7) or the ADT7420 (8); 0 deselects. The EEPROM is written without selecting anything.
* **FPGA**: configured by the FX3 at every boot: a low pulse on GPIO 37 (program), then 384 × 4 KB
  (1.5 MB) read from the boot slot's bitstream address with GPIO 51 held low during each data
  phase. The FPGA takes the bitstream straight off the SPI bus; the FX3 discards its copy. It
  takes the sensors' LVDS outputs; the FX3 writes its registers for triggers, laser, strobe and
  the status LED. Images reach the FX3 through its GPIF-II parallel port (16-bit).
* **USB**: VID 0x3267, PID 0x0003 in the descriptors. EP 0x01/0x81 command channel (**interrupt**
  endpoints), EP 0x02/0x82 bulk channel, EP 0x83 image stream (GPIF → USB, 4 × 0xA020-byte DMA
  buffers). All endpoints are set up with **512-byte packets and burst 1, also on SuperSpeed**
  (the speed is fixed at "high speed" in the code). The device connects to USB only after the
  cameras are configured.

## 3. Flash layout **[H]**
| Address | Contents |
|---|---|
| 0x000000–0x00FFFF | not in the update; erased by CC/00 (see 4.5). Contents unknown |
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
   rewritten to point at the new slot. An interrupted update leaves the old firmware booting
   (but see 5.1 for what a retry without a reset does).
   Whether the bootloader falls back to the old slot when a new application is valid but broken
   is not visible (the bootloader is not in the package).
4. **EEPROM invalidation** **[H]**: a USB control request (bmRequestType 0x20: class,
   host-to-device, recipient device; bRequest 9, i.e. HID SET_REPORT numbering; wIndex 2; a data
   stage whose second byte is 0xAA) zeroes the first two bytes of the I2C EEPROM, i.e. the "CY"
   boot signature. A second byte of 0xBB only logs "Reset!" and does nothing. On FX3 boards whose
   boot-mode pins allow it, the chip then boots into Cypress's ROM USB bootloader (VID 04B4,
   PID 00F3), which loads firmware into RAM over USB. Writing "CY" back restores the original boot.
   Whether this board's boot-mode pins allow USB fallback is **unknown** (hardware).
5. **EraseAppHeader (CC/00) is handled by the application** **[H]**: it re-initialises GPIO and
   SPI, erases the 64 KB flash block at 0x000000, waits 25 ms and resets the device. It sends no
   reply and checks no payload. Block 0 is outside the update and the documented layout; the
   name suggests an application header that the bootloader checks **[M]**, so this probably makes
   the bootloader stop starting the application. Not something to send.
6. **Update details** **[H]**: the first 00/06 packet carries a byte (not used) and the image size
   (BE32). Each following bulk packet is one page: a byte (not used), 4096 data bytes and the
   check byte, with payload length 4098. Every page restarts a 2 s timer; when it expires the
   update is dropped (see 5.1). While the update runs, *every* bulk packet is treated as an update
   page, whatever its key. Any error (page too long, check-byte mismatch) abandons the update with
   status 2; the old slot keeps booting. After the last page the boot record is written and the
   watchdog thread resets the device within 1 s.

## 5. Command handling **[H]**
* Command channel: one large dispatcher; the bulk channel: a 3-entry table (00/06 update,
  10/57 page read, 10/58 page write). Each entry has a group/opcode key, an expected payload
  size (00/06: 5, 10/57: 2, 10/58: not checked, only non-zero), and a handler.
* Reply status: **command channel** 0 ok, 2 bad payload length (also 10/70 out of range), 3 failed
  *or unknown command*; **bulk channel** 0 ok, 1 unknown command, 2 bad length or update error.
* Reply byte 1 is always 0x02 ("not masked"). The firmware never decodes masked requests; it
  ignores request byte 1 (protocol-transport.md 2.4).
* **Retransmissions**: a request whose sequence number equals the previous request's gets no
  reply and is not executed, and the stored number is cleared, so a third copy *is* executed.
  On the command channel, 0xFE and 0xFF are never treated as repeats. On the bulk channel there is
  no such exception, and a dropped repeat makes the bulk thread commit a 0xFFFF-byte reply, which
  most likely fails and resets EP 0x82 **[M]**.
* Replies are built in a fresh, uninitialised heap buffer. Bytes a command does not write hold
  stale data (see 5.1).
* Sensor setup is register tables written over I2C at start-up ("Config cameras"). Gain commands
  write and read the sensors' registers 0x3E08/0x3E09. LED brightness, laser mode, the trigger
  switch and the trigger period are FPGA register writes. Exposure (10/22, 10/23) is an FPGA
  register too, not a sensor register: it holds two 15-bit values, cameras 1 and 2 (selector ≤ 3)
  in the low half and camera 4 in the high half.
* Values the info commands report are constants: VID 0x3267, **PID 1** (the USB PID is 3),
  1280 × 1024, exposure 1..10000, gain 1..800, 8 bits per pixel, 3 sensors.
* Gain (10/26, 10/27) is percent. The sensor register value is percent × 32 / 100, rounded to
  nearest (exact halves round down). The read-back is converted the other way.
* 00/04 (serial) is the FX3's 64-bit eFuse die ID.
* 00/07 reads one FPGA register: six 2-bit button codes (from bit 5; the host uses the first
  three), then single-bit flags. It also drives a recovery hook: after the host clears a halt on
  EP 0x83, the next 00/07 that sees FPGA state bit 17 set restarts the endpoints and DMA channels.
* 10/7B (ClearState) writes 4 to an FPGA control register; an undocumented 00/EF writes 1 to the
  same register and then sleeps 1 s.
* 10/62 (indication) uses only DISTANCE: 0/1/2 select FPGA laser modes 4/1/2, and any other value
  selects mode 0. **DEVICESTATE is ignored.** The status LED is driven by the watchdog thread
  (ok / fault) and laser mode 2 is set at start-up.

### 5.1 Firmware defects and oddities **[H]**
Found while reconstructing the application. None is visible in normal EXStar use unless noted.
* **A retried update can write a shifted image and boot it.** When an update stalls for 2 s (the
  page timer) or 00/0D is received, the update is marked inactive, but the page and byte counters
  are left as they were. Only a completed update or a failed page resets them. If the host then
  starts again with 00/06 without a power cycle, the first page is not treated as the first. The
  slot is not re-chosen, the new pages are written after the old ones (image page 0 at the old
  page count), the byte count finishes early, and the boot record is switched to that slot.
  Whether the bootloader rejects the result (the FX3 image carries a checksum) is unknown. The
  host must never retry an interrupted update without first resetting the device.
* **Flash access is not locked.** The flash semaphore is never created, so the lock and unlock
  calls around flash access do nothing. The bulk thread (updates, user pages) and CC/00 on the
  command thread could interleave SPI transactions.
* **SET_REPORT overflow**: the EEPROM request (4.4) copies 64 bytes into an 8-byte buffer. That
  overwrites the GET_REPORT data and the start of a DMA channel structure. It also reads the data
  stage twice. GET_REPORT and GET_IDLE are only answered on a *host-to-device* request type.
* **Gain write retry uses the wrong byte**: when writing the gain's low byte fails, the retry
  writes the high byte to the low register.
* **Strobe luminance wraps**: 10/70's value is shifted left by 2 and truncated to 16 bits, so
  luminance above 16383 wraps. The range checks (1 000 000 / 5 000 000) can never trigger.
  A route other than 0 or 1 reads or writes FPGA register 0.
* **Silent range checks**: 10/23 (exposure > 50 000) and 10/49 (period outside 1000..1 000 000)
  are ignored with status 0. 10/23 never reports a failed write. 10/20 reports a maximum of 10 000.
* **Payload read beyond the declared length**: 10/5D (colour mode) replies 0 only when payload
  byte 2 is 2, otherwise the reply byte is stale. With the host's 1-byte payload and zero padding,
  it is always stale. The undocumented 10/3F takes its mode from payload byte 1.
* **Declared vs written reply lengths**: 00/04 declares 12 bytes and fills 8; 00/07 declares 20
  and fills 14. 00/05's reply (51 bytes) is one byte longer than its 50-byte buffer (harmless if
  the allocator rounds up, as ThreadX does).
* **Watchdog progress check is inert**: the 1 s progress timer is created but never started. The
  watchdog thread still resets the device after a SuperSpeed endpoint-reset event, after an update,
  and after 10 s powered but not configured.

### 5.2 Undocumented command keys **[H]**
00/02 (replies 0), 00/03 (replies 1), 00/09 / 00/0A / 00/0B (the lengths 18, 12, 42 of vendor
name, product name and version), 00/0C (replies 7), 00/0D (clears an FPGA register and cancels a
pending reboot or update), 00/BB (reads an FPGA register), 00/EF (see above), CC/CC (raw FX3
GPIO 50 and 52 pin registers), 10/36 and 10/37 (length check only), 10/3E / 10/3F (an FPGA
register's mode), 10/42 / 10/43 (32-bit form of the trigger switch), 10/56 (replies 0x00100000),
10/71, 10/72, 10/73, 10/75 (read one byte, write one of three bytes of an FPGA register), 10/77, 10/78, 10/7A (no effect). Writes to the
trigger switch also rewrite a second FPGA register with a fixed value.

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
* The running application removes the flash's write protection at every boot, and CC/00 erases
  flash block 0 with no confirmation or reply.
* The EEPROM-invalidation route (4.4) would give RAM-only testing with no flash writes, but only if
  the boot-mode pins allow USB fallback. Otherwise it leaves a device that needs an EEPROM programmer.
* Neither the EEPROM nor the slots can be backed up through stock commands.

Every one of these operations stays blocked in `DeviceGuard`.

**Next steps if this is pursued:**
1. Photograph the board: the FPGA part, the boot-mode (PMODE) pin straps, and whether the EEPROM
   and flash are in reach of a clip for an external backup.
2. Take a full external backup of the EEPROM and flash.
3. Only then try RAM-loaded firmware through the USB-boot fallback.
