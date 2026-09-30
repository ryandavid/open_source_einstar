# fx3emu: differential emulator for the scanner's FX3 firmware

Runs the scanner's FX3 application under emulation, drives it the way the host and the hardware would,
and compares two builds' behaviour event by event. Its purpose is changing the firmware's toolchain
and code without changing its behaviour. The reference is the vendor's own image, taken from EXStar's
update package; the candidate is a firmware ELF.

This directory contains only our code. Its inputs are not in the repository: the Cypress FX3 SDK
(headers for struct layouts, libraries for the list of SDK functions), the firmware builds and EXStar's
update package. By default they are found in the untracked `.re/` working area (`fx3emu/paths.py`;
override with `EINSTAR_RE`, `FX3_FIRMWARE`, `FX3_GCC`, `EXSTAR_FW_DIR`).

## Use
```
python3 -m venv .venv && .venv/bin/pip install -r tools/fx3emu/requirements.txt
cd tools/fx3emu
python -m fx3emu diff                      # vendor image vs the open build: all scenarios
python -m fx3emu diff --b path/to/fw.elf   # vendor image vs another build
python -m fx3emu coverage --image build    # which application code the scenarios reach, stack use
python -m fx3emu trace update-good --grep flash
python -m fx3emu replay TRACE --image vendor # a host conversation, recorded by the host's own code
```
`replay` checks the host against the firmware: `test_device` records its conversations when
`EINSTAR_FIRMWARE_TRACE=<file>` (a full firmware flash; `EINSTAR_FIRMWARE_TRACE_PACKAGE=<img>` for a real
package) or `EINSTAR_SESSION_TRACE=<file>` (a typical session) is set, and `replay` sends the same
requests to the firmware, compares every reply with what the host's emulator answered, checks that the
firmware reset wherever the host reconnected, and, for an update, the written slot and boot record.
`diff` exits non-zero if any scenario differs and prints the first differing event with context, the
differing replies and the differing final state (application variables by name, flash, I2C devices,
GPIOs).

## How it works
* **CPU**: Unicorn (ARM926, ARM mode). Application code, libc and libgcc run as real ARM code, which
  is where a new compiler or C library changes things.
* **SDK and ThreadX**: every call to a function defined in the Cypress libraries (or in the SDK's
  `cyfxtx.c`) is intercepted at its entry and answered by a model in `sdk.py`. Struct arguments are
  decoded field by field (layouts compiled from the local SDK headers, `layouts.py`), so padding bytes
  never reach a trace. Pointers are named (`symbol+offset`, `stack`, `heap+off`, `dma+off`), so two
  builds with different addresses produce identical traces when they behave the same.
* **Threads**: the application's ThreadX threads run cooperatively on their own stacks until they
  block (DMA get-buffer with nothing queued, semaphore, sleep, relinquish). Time is virtual (1 ms
  ticks); timers fire as time passes. USB callbacks run to completion on a separate stack.
* **Hardware** (`models.py`): the I2C mux (4-bit select on GPIOs 23/25/26/27) and its devices (FPGA
  register bridge, three cameras, ADT7420, light driver, EEPROM), a Winbond-style SPI NOR flash with
  write-enable semantics, GPIOs, and the FPGA taking its bitstream off the SPI bus. Unwritten
  registers read seeded values; I2C faults can be injected.
* **Host** (`device.py`): boots the firmware, enumerates it (the USB events the SDK would deliver), sends
  command and bulk packets (stale bytes of earlier packets remain in the DMA buffers beyond a new one,
  as on the device), issues control requests.
* **Scenarios** (`scenarios.py`): command sweeps over every key and many payloads, the documented
  settings over their ranges, sequence-number rules, device state and the EP 0x83 restart hook, reboot
  and erase commands, user pages, firmware updates (good, bad check byte, oversize page, timeout then
  retry, cancel then retry), control requests, USB events, I2C faults, every SDK call failing once or
  always, and seeded random sessions.
* **Reports**: coverage of the application's instructions (literal pools excluded via ARM mapping
  symbols) and the deepest stack use per thread.

## Intended differences and how they are judged
The open build fixes a few vendor bugs whose output was undefined, so it deliberately differs from the
vendor image at those points. `diff` separates these from regressions:
* **Failed-read divergences.** The read helpers now return `0` instead of the vendor's uninitialised
  stack bytes when an FPGA/sensor read fails (a real I2C error, or a malformed request that selects no
  device). `diff` acknowledges a divergence *by cause*: it is intended only when a failed read precedes
  it within the same command on the same thread (`read_failure_before`). Every differing event must be
  individually explained, so a real regression elsewhere is still caught.
* **The SET_REPORT overflow fix** (recorded in `expected.py`) stops a 64-byte copy into an 8-byte HID
  buffer; the vendor's corruption (and, on some fuzzed inputs, a resulting crash in `usb_setup_cb`) no
  longer happens. `diff` acknowledges the EP0 reply difference and the vendor-only crash.
* `diff --strict` ignores all of the above and shows every raw difference.

To hammer it, raise the fuzz count (each seed is a full randomised session):
```
python -m fx3emu diff --fuzz 256          # ~370 scenarios per optimisation level
python -m fx3emu diff --b <O2.elf> --fuzz 256
```
The gate is validated by injecting a one-byte change into the firmware and confirming `diff` flags it
(both a readback value and a control-register constant are caught).

## Limits
The SDK is a model: DMA completion, interrupts and USB timing are not emulated, and the SDK's own stack
use is not counted. A behaviour the model does not expose cannot differ in a trace. Equal traces mean
the two builds make the same SDK calls with the same arguments and leave the same state, for these
scenarios.
