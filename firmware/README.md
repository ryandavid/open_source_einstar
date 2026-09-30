# Einstar FX3 firmware (open build): reorganised source

A readable C project reconstructed from the vendor's FX3 application, built with the Arm GNU Toolchain
against the Cypress FX3 SDK. The reconstructed sources and build scaffolding are tracked here; the
third-party proprietary inputs (`sdk/`, `fpga/`, `reference/`) are gitignored and user-supplied
(see "Inputs you must supply" below).

Not byte-identical to the vendor image, but **verified behaviourally equivalent** to it with the
differential emulator (`tools/fx3emu`) across every scenario, plus the correctness fixes below.
```
cmake -S . -B build -G Ninja && cmake --build build   # -> build/EN/einstar_fx3.{elf,img} + update package
./check.sh            # build + differential vs the vendor image (FUZZ=n, default 16)
./check.sh O2         # same, forcing an optimisation level (verified equivalent at -O0/-Os/-O2)
```
The default optimisation is **-O0**, matching the vendor's own shipping build (closest codegen/timing
to the known-good firmware); `FX3_OPT` overrides it and higher levels are verified equivalent.

## Correctness fixes over the vendor (tracked in tools/fx3emu/fx3emu/expected.py)
These are the *only* intended differences from the vendor image; the harness treats them as expected
and flags anything else. None changes the USB wire protocol, so stock EXStar still drives it.
* `volatile` on cross-thread flags and the direct FX3 register reads (die id, GPIO pin regs).
* Defined output when an I2C read fails, instead of vendor stack garbage: `fpga_reg_read`,
  the gain readback, the 10/50 temperature copy; the IO-matrix config structs are memset
  (`s0Mode`/`s1Mode` were uninitialised).
* SET_REPORT no longer overflows its 8-byte buffer into `ch_bulk_in`.

The bit-for-bit vendor rebuild (gcc 4.8.1) it descends from lives only in the 2026-09-29 snapshot
(`.re/snapshots/`); `reference/vendor-names.elf` keeps a copy of that ELF as the harness's symbol map.

## Inputs you must supply (not in git)
Everything below is vendor/third-party material, so it is **gitignored** and never committed. A fresh
clone will not build or run the checks until you put these in place. Paths are relative to this
directory (`firmware/`).

### 1. Cypress FX3 SDK 1.3.4 -> `sdk/`  (required to build)
Download the **EZ-USB FX3 SDK v1.3.4 for Linux** from Infineon (formerly Cypress; free, registration
required) and unpack it. Copy these files into `sdk/`, keeping the layout on the left. The right column
is where each lives in the SDK install (`<sdk>` = the unpacked `cyfx3sdk`/`FX3_SDK_Linux` root; the C
library comes from the SDK's bundled CodeSourcery `arm-2013.11` toolchain):

| Place at | From the SDK |
|---|---|
| `sdk/inc/` (the 22 headers the sources `#include`) | `<sdk>/fw_lib/1_3_4/inc/` |
| `sdk/lib/{libcyu3sport,libcyu3lpp,libcyfxapi,libcyu3threadx}.a` | `<sdk>/fw_lib/1_3_4/fx3_debug/` |
| `sdk/fw_build/{cyfx_gcc_startup.S,fx3_512k.ld,cyfxtx.c}` | `<sdk>/fw_build/fx3_fw/` |
| `sdk/util/elf2img/elf2img.c` | `<sdk>/util/elf2img/` |
| `sdk/newlib/include/` (18 headers) | `<toolchain>/arm-none-eabi/include/` |
| `sdk/newlib/lib/libc.a` | `<toolchain>/arm-none-eabi/lib/libc.a` |
| `sdk/newlib/lib/libgcc.a` | `<toolchain>/lib/gcc/arm-none-eabi/4.8.1/libgcc.a` |

The build only reads `sdk/inc`, `sdk/lib/*.a`, `sdk/fw_build/*`, `sdk/newlib/*` and
`sdk/util/elf2img/elf2img.c`; missing headers surface as ordinary compile errors, so nothing goes
stale silently.

### 2. FPGA bitstream -> `fpga/EinScan10_01_FPGA_V3.7.bin`  (required to build a flashable package)
Extract it from your scanner's update package (`..._FPGA_V3.7_EN_IAP.img`, from EXStar's
`fabu_UPDATE/Configure/`). The package is 4096-byte pages each followed by one check byte; the FPGA
bitstream is the tail of the flash slot after the FX3 image:
```
python3 - <<'PY'
pkg = open("EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN_IAP.img","rb").read()
slot = b"".join(pkg[i:i+4096] for i in range(0, len(pkg), 4097))   # drop the per-page check byte
open("fpga/EinScan10_01_FPGA_V3.7.bin","wb").write(slot[0x40000:].rstrip(b"\xff"))
PY
```
Building the ELF alone does not need it; only the packaged `..._IAP.img` output does.

### 3. Differential-check inputs (only to run `./check.sh` / `tools/fx3emu`, not to build)
* The vendor update package itself, as the ground truth to diff against. Point `EXSTAR_FW_DIR` at the
  directory holding `EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN_IAP.img` (default: EXStar.app's
  `fabu_UPDATE/Configure`).
* `reference/vendor-names.elf` -- an ELF byte-identical to the vendor firmware, used only to attach
  symbol names to the vendor image. It is a product of the original byte-exact (gcc 4.8.1) build and is
  kept in the 2026-09-29 snapshot (`.re/snapshots/`); copy it from there, or set `FX3_VENDOR_NAMES` to
  its path. Without it you can still build the firmware, just not run the differential check.

## Build
Apart from those inputs and the toolchain the tree is self-contained. The FX3 SDK subset lives in
`sdk/` (see `sdk/README.md`); the FPGA bitstream in `fpga/`. Toolchain in `cmake/fx3-toolchain.cmake`
(overridable):
the Arm GNU Toolchain gcc (`FX3_ARMGNU_GCC`) and a host C compiler for `elf2img`/`tools/mkpackage.c`
(`FX3_HOST_CC`).
* **Link order** is the source order in `CMakeLists.txt`; `common.c` (COMMON vars) is linked last.
  Only `version.c` is compiled per language (`FW_LANG`); `FW_LANGUAGES` defaults to EN.
* **Package** (`tools/mkpackage.c`): FX3 image, 0xFF to 0x40000, bitstream, 0xFF to 1.25 MB, as
  4 KB pages each followed by its 8-bit sum -- the update package EXStar sends over bulk 00/06.

## Flashing
The host tool `einstar-firmware` (built with the main project, `apps/einstar-firmware`):
```
build/default/apps/einstar-firmware version                    # what the scanner runs (00/05), serial
build/default/apps/einstar-firmware inspect firmware/build/EN/EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN_IAP.img
build/default/apps/einstar-firmware flash   firmware/build/EN/EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN_IAP.img
build/default/apps/einstar-firmware flash   <EXStar.app>/Contents/MacOS/fabu_UPDATE/Configure/..._EN_IAP.img   # back to the vendor's
```
`flash` checks the package, reboots the scanner (clearing any abandoned update, see the retry bug
below), writes the inactive slot exactly as EXStar does, and reconnects to read the version; `--emulator`
runs it against the emulated scanner. The previous firmware stays in the other slot but no command
selects it, so going back is flashing EXStar's package. Our build reports the same version string as the
vendor's, so `version` cannot tell them apart. The host side was checked against this firmware with
`tools/fx3emu` (`python -m fx3emu replay`, see its README). Not yet run on the scanner.

## Layout
| File | Contents |
|---|---|
| `include/app.h` | SDK headers + the headers below |
| `include/firmware.h` | functions and shared data, by module |
| `include/protocol.h` | packet layout, status codes, packet structs, command keys (names as in `docs/protocol-device.md`, `(?)` = undocumented) |
| `include/fpga_regs.h` | FPGA register numbers (I2C register bridge), `struct fpga_word` |
| `include/board.h` | GPIOs, USB endpoints, I2C target codes, FPGA mode constants, flash layout |
| `src/gpif_config.c` | GPIF II Designer tables + `CyFxGpifConfig` |
| `src/globals.c` | application state (`app_state`, flags, semaphores, EMC timer) |
| `src/sync.c` | bus locks, timer callbacks |
| `src/version.c` | version string |
| `src/boot.c` | EEPROM boot-signature erase, debug UART, GPIF start |
| `src/usb.c` | endpoints/DMA channels, USB event/setup/LPM callbacks, USB/I2C/IO-matrix init |
| `src/app.c` | the four threads, `CyFxApplicationDefine`, `main` |
| `src/sensor_i2c.c` | image sensors, FPGA register bridge, ADT7420, light driver |
| `src/usb_descriptors.c` | USB descriptors |
| `sdk/fw_build/cyfxtx.c` | heap / DMA buffers (SDK file, compiled as-is) |
| `src/spi_flash.c` | SPI flash, FPGA configuration |
| `src/gpio_led.c` | GPIO set-up, I2C target select lines, heartbeat |
| `src/commands.c` | command channel dispatcher |
| `src/update.c` | bulk channel: firmware update, user pages, boot record |
| `src/common.c` | the vendor's COMMON variables (linked after the libraries) |

`symbols.tsv` / `data.tsv` map every function / variable to its vendor address (reference only).

## Vendor file structure (recovered from the rodata layout)
The vendor had **one main file** (sync .. main), **one sensor file**, **a descriptor file**, then
flash/GPIO/commands/update (split unknown). Evidence: per-file literal pools, a const table
opening each file's rodata, and duplicated strings across files. (The byte-exact build re-added the
vendor's dead functions for string placement; the open build has dropped them.)

## Editing
Change the source freely; `./check.sh` proves it still matches the vendor's behaviour (or that a new
difference is one you meant). Helpers: `tools/summary.py` and `tools/globals.py` summarise the modules;
`tools/rename.py` renames identifiers across the tree.

## Behaviour found while reorganising (for the spec)
* **Command 0xCC00 ("EraseAppHeader")** erases flash 0x000000-0x00FFFF (64 KB block 0) and reboots.
* **Boot rewrites the flash status registers**: SR1 = 0, SR2 = 0x02 (QE), SR3 = 0: block protection off.
* **`flash_sem` is never created**, so `flash_lock()` does not lock.
* **EMC watchdog progress check is inert**: `emc_wdg_timer` is never started. The watchdog thread
  still reboots on an endpoint reset (SS reset event), after an update, or after 10 s powered
  but unconfigured.
* **Control requests**: class requests to the *device* with wIndex 2 use HID request numbers.
  SET_REPORT with byte 1 = 0xAA erases the EEPROM "CY" signature; **0xBB only logs "Reset!"**.
  The vendor's SET_REPORT copies 64 bytes into an 8-byte buffer, overwriting the GET_REPORT data
  and the start of `ch_bulk_in` (fixed here, see above).
* Endpoints are configured for **high speed (512-byte packets, burst 1) even on SuperSpeed**; the
  scanner's hardware is USB 2.0 only, so the SuperSpeed descriptors and paths never run.
* EP 0x01/0x81 (commands) are **interrupt** endpoints; EP 0x02/0x82 and 0x83 are bulk.
* `usb_init()` does not connect; `com_thread()` connects after the cameras are configured.
* **Gain**: percent -> 1/32 steps (nearest, exact halves down) in 0x3e08/0x3e09; a failed low-byte write is
  **retried with the high byte**.
* **Strobe luminance** is written as `((lum << 2) & 0xFFFF) | 1`: values above 16383 wrap; the
  1 000 000 / 5 000 000 limits can never trigger.
* 10/23 (set exposure) ignores the FPGA write status; 10/22's error branch is dead.
* **Update retry bug**: the 2 s page timer (`update_timer_cb`) and 00/0D clear `update_active`
  but not `update_page_index` / `update_bytes_received`. A new 00/06 then continues at the old page
  index with the old slot, finishes early and rewrites the boot record: a shifted image, booted.
* Status codes: command channel 0 ok, 2 bad length (also 10/70 out of range), 3 failed *or unknown
  key*; bulk channel 0 ok, 1 unknown key, 2 bad length / update error. Reply byte 1 is always 2:
  the firmware never unmasks requests.
* A repeated sequence number is dropped and clears the stored one (a third copy is executed).
  0xFE/0xFF are exempt on the command channel only. On the bulk channel the drop returns -1,
  which `com_thread()` commits as a 0xFFFF-byte reply (most likely failing and resetting EP 0x82).
* Declared vs written reply lengths: 00/04 12 vs 8, 00/07 20 vs 14. 00/05's reply is 51 bytes in
  a 50-byte buffer (the allocator rounds up, so probably harmless).
* 10/01 reports PID 1 (USB PID is 3); 10/20 reports max exposure 10000 but 10/23 accepts 50000;
  10/23 and 10/49 ignore out-of-range values with status 0.
* 10/62 uses only DISTANCE (0/1/2 -> laser mode 4/1/2, anything else 0); DEVICESTATE is ignored.
  Laser mode 2 is set at start-up.
* 10/6F/10/70 with a route other than 0/1 read/write FPGA register 0.
* 10/3F takes its mode from payload byte 1 (declared length 1); 10/3E cannot report mode 2
  (0x..F3 reads back as low bits 3, which leaves the reply byte unwritten).
* CC/CC returns the raw FX3 GPIO 50 and 52 pin registers (52 is a configured input, 50 is not).
* Class requests are only handled with bmRequestType 0x20 (host-to-device), so GET_REPORT/GET_IDLE
  answer a host-to-device request; SET_REPORT reads the data stage twice.
* `usb.c`'s old comment named GPIOs 52/53 for `gpioSimpleEn[1]`: it is 36, 37, 51, 52.
* Commands 10/3F and 10/5D read payload bytes beyond their declared length.
* 00/07 runs `app_restart()` only after a CLEAR_FEATURE on EP 0x83 and with FPGA state bit 17. It is
  a full restart: `usb_init()` disconnects from USB, then the FPGA and the sensor tables are reloaded
  and the device reconnects (re-enumerates).
* A USB suspend clears `usb_configured`, and only SET_CONFIGURATION sets it again: ~10 s after a
  suspend the watchdog disconnects and resets the device, even if the bus resumed.
* Bulk replies are 1024 bytes (0x1400 for 10/57) whatever the request, ending on a full packet.
* I2C select: a 4-bit code on GPIO 23/25/26/27 (cameras 1/2/4 one-hot, FPGA 5, light 7, ADT7420 8).
* FPGA configuration: PROGRAM pulse on GPIO 37, then 384 x 4 KB read from the active slot with
  GPIO 51 low (the FPGA snoops the SPI data).
