# Einstar device-control protocol (CEinScan10 plugin)

Source analysed: `einscan10.2.0.0.dylib` (arm64), class `CEinScan10`, built from
`SyncV2/EinScan10Plugin/einscan10.cpp` and `einscan10threads.cpp`. The device's
firmware string in the logs is `EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN`,
which points to a Cypress FX3 USB bridge in front of an FPGA.

This document covers **what CEinScan10 sends and how it reads the replies**. USB
endpoints, masking/XOR coding, unit buffering and hot-plug handling are in
`protocol-transport.md`. All byte values here are **pre-mask** (plain) bytes.

Confidence tags: **[H]** high (read directly from code), **[M]** medium
(inferred from code and logs, or naming), **[L]** low (plausible guess).
Addresses are function entry points in `einscan10.2.0.0.dylib` unless marked
`logic:`, which means `3DDigitalSyncInterface.2.64.8-alpha.5.dylib`.

Clean-room note: this is a behavioural description written from static
analysis. No decompiled code is reproduced.

---

## 0. Layering

```
EXStar / scanservice
   │  (MQTT hub, shared memory)
SnSyncService ── logic: InternalE10Device   (property names, button actions, CaptureThread,
   │                                         "InternalE10Image%d" shared-memory publishing)
   │  C++ virtual calls: getProperty / setProperty / executeCommand / grabImage /
   │                     setGain / setExposureTime / read/writeCustomInfo ...
CEinScan10 (einscan10.2.0.0.dylib)          ← this document
   │  CommunicateLayer::sendCommandAndWaitReply  (command channel, "interruptIO")
   │  CommunicateLayer::sendDataAndWaitReply     (bulk channel,    "bulkIO")
   │  CommunicateLayer::receiveImageStream       (image stream)
libsn3DCommunicateLayerM                     ← protocol-transport.md
```

Property and command names reach CEinScan10 as strings. They are dispatched on
a 64-bit hash of the Latin-1 name: start at 0x811C9DC5, then for each signed
char `h = (h * 0x01000193) XOR c` modulo 2^64. This is an FNV-1 variant that
uses the 32-bit constants in 64-bit arithmetic. The hash only matters for
matching names in the binary and never goes on the wire. **[H]** (0x10954,
0x12e78, 0xad84)

---

## 1. Command packet format

### 1.1 Request **[H]**

| Offset | Size | Meaning |
|---|---|---|
| 0 | 1 | Sequence number. Per-device counter at `this+0x40`, starts at 0, post-increment modulo **254** (0..253), mutex-protected (`getPacketID` 0xb488). The transport reserves 0xFE and 0xFF for its own queries. |
| 1 | 1 | 0x00 (the transport overwrites it with the mask key) |
| 2 | 1 | **Group**: `0x00` = system/info, `0x10` = controller/sensor registers, `0xCC` = bootloader |
| 3 | 1 | Opcode |
| 4..7 | 4 | Payload length, **big-endian u32** |
| 8.. | n | Payload |

The buffer is zero-padded to a fixed size per command: 10, 16, 50 (0x32), 64
(0x40), 100 or 5120 (0x1400) bytes. That size is passed as both the send
length and the reply capacity. The padding bytes are always zero.

**Camera selector byte.** Per-sensor commands put a one-byte sensor **bit
mask** at payload byte 0 (packet offset 8). Sensor index 0 → 0x01, 1 → 0x02,
2 → 0x04. The code computes `idx+1`, with index 2 special-cased to 4. **[H]**

### 1.2 Reply **[H]**

| Offset | Size | Meaning |
|---|---|---|
| 0 | 1 | Echo of the request sequence |
| 1 | 1 | 0x00 (after unmasking) |
| 2 | 1 | Echo of the group |
| 3 | 1 | Echo of the opcode |
| 4 | 1 | **Status**: 0 = OK. Anything else is treated as failure. |
| 5..8 | 4 | Reply payload length, BE u32. Only the string replies read it. |
| 9.. | n | Reply payload. All multi-byte values are **big-endian**. |

### 1.3 Retry and validation **[H]**
* `interruptIO` (0xb4f4, command channel) and `bulkIO` (0x10414, bulk
  channel) both work on a copy of the request.
* A reply is accepted only when the transport call succeeds, reply[0], [2]
  and [3] equal the request's, and reply[4] == 0.
* On failure the plugin resets the pipe (`resetCommandPipe(3)` or
  `resetBulkPipe(1)`), sleeps 20 ms (command channel only) and retries. It
  makes up to 5 attempts.
* Every caller in CEinScan10 uses the retrying variant. The non-retrying
  `bulkIO` path has an inverted success test (a bug), but nothing calls it.
* `readDeviceState` (0x15e8c) is the exception: it calls
  `sendCommandAndWaitReply` directly, once, with no retry. It checks only the
  transport result and reply[4].
* Commands on the bulk channel: flash page read/write (0x10/0x57, 0x10/0x58)
  and firmware IAP (0x00/0x06). Everything else goes on the command channel.

### 1.4 Authentication / unlock **[H]**
CEinScan10 performs **no** authentication, challenge/response, licence or
unlock exchange. After the transport attaches, the first command is simply a
trigger-period write (section 2). The only obfuscation is the transport's XOR
masking. The logic plugin also does no device handshake before `openDevices`
completes.

---

## 2. Connect / open sequence

`connectDeviceBySerial(serial)` at 0x6b90. Everything below runs on the
command channel unless marked otherwise. `seq` values assume a freshly
constructed object (counter = 0). The heartbeat thread starts near the end
and interleaves its own sequence numbers from then on.

| # | Action | Request bytes (pre-mask, trailing zero padding omitted) | Buf | Reply parse |
|---|---|---|---|---|
| 0 | `CommunicateLayer::attachDevice(serial)` | – | – | 2 or 4 → error 0x301, other non-zero → 0x303 |
| 1 | **Trigger period write** 200 000. `triggerPeriodIO` (0x8234) increments the seq counter **twice** and wastes one value, so seq 0 is never used here. | `01 00 10 49 00 00 00 04 00 03 0D 40` | 50 | status only |
| 2 | **Trigger switch write**, mono=1, rgb=1 | `02 00 10 41 00 00 00 01 11` | 10 | status only |
| 3 | Transport configuration (no USB traffic): attemptPkgSize = 0xA400 (41 984), pkgCountPerUnit = 32, cacheUnitCount = 100, maxGrabWaitTime = 100 000 µs, then `driverStreamSwitch(true)` (image IN transfers start here) | – | – | – |
| 4 | Vendor name (group 0) | `03 00 00 00 00 00 00 00` | 100 | length = BE32 @5..8. Accepted if ≤ 0x5C; the string starts @9. |
| 5 | Vendor ID | `04 00 10 00 00 00 00 00` | 16 | BE16 @9..10 → `this+0x3c4` |
| 6 | Product name (group 0) | `05 00 00 01 00 00 00 00` | 100 | as #4. Split on `_`; if token[1] upper-cased == `GSY`, set a flag (`this+0x340`) that enables the extra temperature fields in device state (section 3.10). |
| 7 | Product ID | `06 00 10 01 00 00 00 00` | 16 | BE16 @9..10 → `this+0x3c8` |
| 8 | **Sensor count** | `07 00 10 51 00 00 00 00` | 16 | u8 @9 → `this+0x3d4` (3 on Einstar) |
| 9 | For each sensor i = 0..count−1, with mask m = 1, 2, 4, eight reads: | | | |
| 9a | Max width | `ss 00 10 16 00 00 00 01 m` | 16 | BE16 @9 (1280) |
| 9b | Max height | `ss 00 10 17 00 00 00 01 m` | 16 | BE16 @9 (1024) |
| 9c | Max exposure | `ss 00 10 20 00 00 00 01 m` | 16 | BE32 @9..12 |
| 9d | Min exposure | `ss 00 10 21 00 00 00 01 m` | 16 | BE32 @9..12 |
| 9e | Max gain | `ss 00 10 24 00 00 00 01 m` | 16 | BE16 @9 |
| 9f | Min gain | `ss 00 10 25 00 00 00 01 m` | 16 | BE16 @9 |
| 9g | Pixel bits | `ss 00 10 2E 00 00 00 01 m` | 16 | u8 @9 (8 / 16 / 24) |
| 9h | Colour mode | `ss 00 10 5D 00 00 00 01 m` | 16 | u8 @9 (0 = mono, else colour) |
| 10 | Firmware version (group 0) | `ss 00 00 05 00 00 00 00` | 64 | length BE32 @5..8, accepted if ≤ 0x38; string @9 |
| 11 | Set online flag. Create the heartbeat `CTimerThread` (period `this+0x2d4`, default **250 ms**) and connect it to `queriedHeartBeat`. From here the device-state poll (section 3.10) runs periodically. | – | – | – |
| 12 | Strobe luminance read, route 1 | `ss 00 10 6F 00 00 00 01 01` | 50 | BE16 @9..10. The result (≠ 0) goes to `this+0x420` ("strobe on"); it chooses which colour-correction context is used (section 4.6). |
| 13 | `reloadCustomInfo` → `readCustomInfo(0, 6568)`: flash pages 0 and 1 (**bulk**) | `ss 00 10 57 00 00 00 02 00 00`, then `… 00 01` | 5120 | 4096 data bytes @9 per page (section 3.12) |

Timing in the log: attach to "end connectDeviceBySerial" took 313 ms
(13:51:23.887 → 24.200). **[H]** (SnSyncService log)

Construction defaults (0x4cd4), cached only and not sent at connect **[H]**:
* Heartbeat period 250 ms.
* Trigger cache: mono 3, rgb 1, period 200 000.
* Strobe flag on.
* YGamma DIAT1 = 0.2, DIAT3 = 0.1.
* EnableRGB = true.
* Per-sensor gain cache 100, exposure cache 1000.
* ConvertParam: all flips and rotations off, CLOCKWISEROTATE = true, angle 0.

Enumeration (`enumEinScan10Devices` 0x6864) counts only devices whose
transport product name starts with `EINSCAN10` (case-insensitive). **[H]**

### Disconnect (`disconnectDevice` 0xaab8) **[H]**
1. Trigger switch write 0/0: `ss 00 10 41 00 00 00 01 00`. After a 0/0 write
   the plugin sleeps 1000 ms.
2. Sleep 100 ms.
3. `stopStream`.
4. Stop the heartbeat thread.
5. `driverStreamSwitch(false)`, `resetDeviceInDriver`, `detachDevice`.
6. Release the GPU colour contexts.

### Reconnect / property replay (inside `queriedHeartBeat` 0x9518) **[H]**
When the heartbeat sees the device come back (online flag goes 0 → 1), it:
1. Re-sends the cached trigger period (0x49) and trigger switch (0x41).
2. Toggles `driverStreamSwitch` off and on, re-applying the transport
   parameters from #3.
3. Replays every property previously set successfully through
   `setProperty`. The cache is `this+0x2b8`. StrobeLuminance entries with
   ROUTEINDEX 1 are skipped.
4. Re-sends exposure and gain for every sensor.
5. Sends **ClearState** (0x10/0x7B).

This explains the extra `TriggerMode` / `ScanTriggerPeriod` debug lines in the
log (17 plugin-side versus 10 API calls) after the 5 disconnect/reconnect
events.

---

## 3. Command and property reference

### 3.1 Complete opcode table

"R" = read, "W" = write. The payload column shows request payload bytes
(starting at packet offset 8). The reply column shows fields from offset 9.

| Group/Op | Name (plugin fn) | Dir | Req len (bytes 4..7) | Request payload | Reply payload (@9) | Buf | Conf |
|---|---|---|---|---|---|---|---|
| 00/00 | Vendor name (`readVendorName` 0x8544) | R | 0 | – | ASCII, length BE32 @5..8, max 92 | 100 | H |
| 00/01 | Product name (`readProductName` 0x87bc) | R | 0 | – | ASCII, same format | 100 | H |
| 00/04 | Serial (`readSerial` 0x12a20) | R | 0 | – | 8 raw bytes, formatted as 16 uppercase hex chars (e.g. `0009011402CF0C20`) | 64 | H |
| 00/05 | Firmware version (`readFirmwareVersion` 0x938c) | R | 0 | – | ASCII, max 56 | 64 | H |
| 00/06 | **IAP firmware download** (`updateFirmware` 0xffdc), bulk. **DANGEROUS** | W | see 3.14 | | | 5120 | H |
| 00/07 | Device state / buttons (`readDeviceState` 0x15e8c) | R | 0 | – | BUTTON0 @9, BUTTON1 @10, BUTTON2 @11. GSY variant only: board temp BE16 @12, IR temp BE16 @14 (÷100 °C) | 50 | H |
| 00/08 | Reboot (`rebootDevice` 0xb0d4, command "RebootDevice") | W | 0 | – | – | 10 | H |
| 10/00 | Vendor ID ("Vid") | R | 0 | – | BE16 | 16 | H |
| 10/01 | Product ID ("Pid") | R | 0 | – | BE16 | 16 | H |
| 10/16 | Max width | R | 1 | mask | BE16 | 16 | H |
| 10/17 | Max height | R | 1 | mask | BE16 | 16 | H |
| 10/20 | Max exposure | R | 1 | mask | BE32 | 16 | H |
| 10/21 | Min exposure | R | 1 | mask | BE32 | 16 | H |
| 10/22 | Get exposure (`getExposureTime` 0xb6d8) | R | 1 | mask | BE32 | 16 | H |
| 10/23 | **Set exposure** (`setExposureTime` 0xb334) | W | 5 | mask, BE32 exposure | – | 16 | H |
| 10/24 | Max gain | R | 1 | mask | BE16 | 16 | H |
| 10/25 | Min gain | R | 1 | mask | BE16 | 16 | H |
| 10/26 | Get gain (`getGain` 0xb9f4) | R | 1 | mask | BE16 | 16 | H |
| 10/27 | **Set gain** (`setGain` 0xb8a8) | W | 3 | mask, BE16 gain | – | 16 | H |
| 10/2E | Pixel bits | R | 1 | mask | u8 | 16 | H |
| 10/40 | Trigger switch read (`triggerSwitchIO` 0x83c0) | R | 0 | – | u8: low nibble = mono count, high nibble = rgb count | 10 | H |
| 10/41 | **Trigger switch write** | W | 1 | u8 `(rgb<<4) \| (mono & 0x0F)` | – | 10 | H |
| 10/48 | Trigger period read (`triggerPeriodIO` 0x8234) | R | 0 | – | BE32 | 50 | H |
| 10/49 | **Trigger period write** | W | 4 | BE32 period | – | 50 | H |
| 10/50 | Temperature (`readTemperature` 0x12c0c, property "Temperature") | R | 0 | – | BE16 raw, °C = raw >> 7 (sign-extended) | 50 | H |
| 10/51 | Sensor count | R | 0 | – | u8 | 16 | H |
| 10/57 | **Flash page read** (bulk) | R | 2 | BE16 page number | 4096 bytes | 5120 | H |
| 10/58 | **Flash page write** (bulk). **DANGEROUS** | W | 0x1002 | BE16 page number, 4096 bytes | (reply capacity 1024) | 5120 | H |
| 10/5D | Colour mode | R | 1 | mask | u8 | 16 | H |
| 10/62 | **Indication** LEDs (`setDeviceIndication` 0x15d5c) | W | 2 | u8 DISTANCE (0..2), u8 DEVICESTATE (0/1) | – | 16 | H |
| 10/67 | Laser / "LD RGB" brightness read (`laserLightBrightnessIO` 0x12d24) | R | 0 | – | u8 | 50 | H |
| 10/68 | **Laser / "LD RGB" brightness write** | W | 1 | u8 0..100 | – | 50 | H |
| 10/6F | Strobe luminance read (`strobeBrightnessIO` 0xa1cc) | R | 1 | u8 route (0/1) | BE16 | 50 | H |
| 10/70 | **Strobe luminance write** | W | 3 | u8 route, BE16 luminance | – | 50 | H |
| 10/7B | **ClearState** (`clearButtonState` 0xb1b4) | W | 0 | – | – | 10 | H |
| CC/00 | **Erase application header** (`eraseAppHead` 0x10788). **DANGEROUS** | W | 0 | – | – | 16 | H |

No other opcodes appear anywhere in CEinScan10. Opcodes not listed (for
example 0x10/0x02–0x15, 0x28–0x2D, 0x42–0x47 …) are **never sent by the
host**. **[H]**

### 3.2 getProperty names (0x10954)

Every name returns its result under key `"value"` in the output map.
Unknown names return 0x307. A failed transfer returns 0x30A. Calling without
an attached device returns 0x200.

| Property | Device traffic | Returned value | Conf |
|---|---|---|---|
| `VendorName` | 00/00 | QString | H |
| `ProductName` | 00/01 | QString | H |
| `Vid` | 10/00 | uint, BE16 | H |
| `Pid` | 10/01 | uint, BE16 | H |
| `Serial` | 00/04 | 16-char hex QString | H |
| `FirmwareVersion` | 00/05 | QString (`EinScan10_01_SC130_FX3_V2.10_FPGA_V3.7_EN`) | H |
| `SensorCount` | 10/51 | uint (3) | H |
| `Temperature` | 10/50 | double, integer °C (see 3.11) | H |
| `TriggerMode` | 10/40 | map {MONOTRIGCOUNT = low nibble, RGBTRIGCOUNT = high nibble} | H |
| `ScanTriggerPeriod` | 10/48 | uint | H |
| `LDRGBLuminance` | 10/67 | uint (u8) | H |
| `StrobeLuminance` | 10/6F | input `value` = map {ROUTEINDEX < 2}; output map {LUMINANCE: BE16} | H |
| `HeartBeat` | none (cached) | map {BUTTON0, BUTTON1, BUTTON2 [, BOARDTEMPERATURE, IRTEMPERATURE]}. **Read-and-clear**: after returning, the cache is reset to BUTTONn = 0. Fails with 0x30A if the device is marked offline. | H |
| `HeartBeatPeriod` | none | int ms (`this+0x2d4`) | H |
| `HeartBeatQueryOtherDevLine` | none | bool: while offline, the heartbeat saw other serials but not ours | H |
| `EnableRGB` | none | bool | H |
| `YGamma` | none | map {DIAT1, DIAT3} floats | H |
| `ConvertParam` | none | input {SENSORINDEX}; output {SENSORINDEX, HORFLIP, VERFLIP, ENABLEROTATE, CLOCKWISEROTATE, ROTATEANGLE} | H |
| `FrameRate` | none | double fps, measured on sensor 0 images (section 4.7) | H |
| `Progress` | none | int firmware-update progress (0, 100–950, 1000) | H |

### 3.3 setProperty names (0x12e78)

Each name first logs `Einscan10Plugin:: <name>`, then `after set Property`.

Result codes:
* 0 on success. The value is then cached in `this+0x2b8` for replay on
  reconnect.
* 0x30B: device I/O failed.
* 0x30C: parameter invalid.
* 0x307: unknown name.

"CameraGain" and "CameraExposure" are **not** CEinScan10 property names. The
logic layer turns `[idx, value]` into the virtual calls `setGain(value, idx)`
and `setExposureTime(value, idx)`, which send 10/27 and 10/23.

| Property | `value` type | Validation / clamping | Wire | Conf |
|---|---|---|---|---|
| `TriggerMode` | map {MONOTRIGCOUNT, RGBTRIGCOUNT} | Both keys required. mono is masked to 4 bits. rgb is shifted into the high nibble and truncated to a byte. **If both are 0 the plugin sends mono=1, rgb=1 instead**, after first writing period 200 000 (10/49). This is "free-run preview" mode, so 0/0 (off) is only sent by disconnect. | 10/41 [(rgb<<4)\|mono] | H |
| `ScanTriggerPeriod` | int/uint | none | 10/49 [BE32] | H |
| `LDRGBLuminance` | int/uint | must be ≤ 100, otherwise 0x30C | 10/68 [u8] | H |
| `StrobeLuminance` | map {ROUTEINDEX, LUMINANCE} | route must be < 2. Luminance is sent as u16 (truncated, not clamped). A successful route-1 write updates the "strobe on" flag (luminance ≠ 0). | 10/70 [route, BE16] | H |
| `Indication` | map {DISTANCE, DEVICESTATE} | DISTANCE must be 0..2 and DEVICESTATE 0..1, otherwise 0x30C | 10/62 [DISTANCE, DEVICESTATE] | H |
| `HeartBeatPeriod` | int ms | must be ≥ 0; changes the timer thread period | none | H |
| `EnableRGB` | bool | – | none. Host-side: demosaic plus colour-correct the RGB sensor (true), or deliver raw Bayer (false) | H |
| `YGamma` | map {DIAT1, DIAT3} | Both keys required. Each must be in [1e-6, 1.000001]. The values are stored before the check, so an invalid set still overwrites them. | none. Parameters of the host colour-correction "Ygamma" stage | H |
| `ConvertParam` | map {SENSORINDEX, HORFLIP?, VERFLIP?, ENABLEROTATE?, CLOCKWISEROTATE?, ROTATEANGLE?} | per-sensor, only the keys present are updated | none. Host-side flip/rotate (section 4.5) | H |

### 3.4 executeCommand names (0xad84)

| Command | Action | Result | Conf |
|---|---|---|---|
| `RebootDevice` | 00/08, buffer 10 | 0 or 0x30D | H |
| `ClearState` | 10/7B, buffer 10 | 0 or 0x30D | H (name recovered by hash match) |
| `ReloadCustomInfo` | re-reads flash pages 0 and 1 (6568 bytes) and re-extracts the colour-calibration fields | 0 or 0x30D | H |
| anything else | – | 0x307 | H |

Logic-layer execute names seen in the log, `ReadData(offset, size)` and
`WriteData(offset, base64)`, map onto the virtual
`readCustomInfo(offset, QByteArray)` / `writeCustomInfo(offset, QByteArray)`
(section 3.12). **[M]** (log plus vtable use)

### 3.5 Trigger **[H]** (units **[M]**)
* 10/41 byte = `(RGBTRIGCOUNT << 4) | (MONOTRIGCOUNT & 0xF)`.
* Values seen: 0x03 (scan: 3 mono triggers, no RGB), 0x11 (preview/texture:
  1 mono + 1 RGB), 0x00 (off, disconnect only).
* 10/49 period, BE32. Values seen: 200 000 (connect default), 68 000 (scan),
  100 000 (texture/preview). The unit is most likely **µs** (68 ms ≈ 14.7 Hz
  trigger cycle). [M]
* The read-back 10/40 decodes the same nibbles.

### 3.6 Exposure / gain per camera **[H]** (units **[M]**)
* Sensor index 0..2 maps to mask 1, 2, 4. An index > 2 gives error 0x40C.
* Exposure is BE32. Values seen: 1500–5200. The unit is probably µs. [M]
* Gain is BE16. Values seen: 80–120 on IR, 180 on RGB (index 2), 400 on
  index 0 in a texture/preview phase. There is no host clamp. The device range
  comes from 10/24 and 10/25.
* On success the value is cached (`this+0x3f8` exposure, `this+0x3f0` gain)
  for reconnect replay.
* Error codes: set exposure 0x40D, get exposure 0x40E, set gain 0x410,
  get gain 0x411.

### 3.7 Laser brightness ("LDRGBLuminance") **[H]** (meaning **[M]**)
* 10/68, payload is one byte 0..100, most likely percent.
* Read with 10/67 (reply u8 @9).
* Values seen: 0 when idle, 60 during a phase with TriggerMode 3/0 and strobe
  route 0 = 3000. This suggests "LD" is the structured-light laser/VCSEL
  projector. [M/L]

### 3.8 Strobe **[H]** (meaning **[M]**)
* 10/70 payload `[route][lum_hi][lum_lo]`. Read with 10/6F `[route]`, reply
  BE16.
* Route 0: set to 3000–9000 during scanning, together with the camera
  exposure ramp. Probably the IR illumination/flash drive.
* Route 1: always set to 0 in the log. The plugin treats it as the
  "WithStrobe" condition for RGB colour correction, so it is probably the
  white LED for texture capture. [M]

### 3.9 Indication LEDs **[H]**
* 10/62 payload `[DISTANCE][DEVICESTATE]`.
* DISTANCE ∈ {0, 1, 2}: the distance-indicator LED. In the log it cycles
  1/2/0 while the host is scanning; likely too-close / OK / too-far. [M]
* DEVICESTATE ∈ {0, 1}. The log always shows 1.
* This is the most frequent setProperty (783 calls in about 34 minutes).

### 3.10 Heartbeat, device state and buttons **[H]**
* **No unsolicited events.** Buttons are **polled**. The timer thread
  (`CTimerThread::run` 0x1801c) calls `queriedHeartBeat` every
  HeartBeatPeriod ms (default 250). It sleeps 100 ms while "held". The poll
  is skipped while a firmware update runs (`this+0x330`) or while no device
  handle exists.
* Request 00/07 (`ss 00 00 07 00 00 00 00`, 50-byte buffer). Reply:

  | Offset | Field |
  |---|---|
  | 9 | BUTTON0 action code |
  | 10 | BUTTON1 action code |
  | 11 | BUTTON2 action code |
  | 12..13 | BE16 board temperature ×100 (only if product name token[1] == `GSY`) |
  | 14..15 | BE16 IR temperature ×100 (same condition) |

* The action code is passed unchanged by logic `InternalE10Device::updateHeartBeat`
  (logic:0x5a298) as the `ButtonAction` enum when non-zero. The enum's Qt
  meta names, in order, are `NoClick, SingleClick, DoubleClick, LongClick`,
  so **0 = none, 1 = single click, 2 = double click, 3 = long click**. [M]
  The log shows only `SingleClick` events: 9× BUTTON0, 32× BUTTON1,
  28× BUTTON2.
* The firmware probably latches these codes until **ClearState** (10/7B) is
  sent. The host only sends ClearState after a reconnect, and the host-side
  cache is cleared on each `HeartBeat` read. Not verified on hardware whether
  the device auto-clears on read. [L]
* Online/offline logic:
  * A failed 00/07 increments a failure counter. After 3 consecutive failures
    (on the 4th) the device is marked offline and the cached state is emptied.
  * While offline, each tick calls `getOnlineDeviceSerial` and compares the
    result with our serial. A match sets the device back online and triggers
    the property replay from section 2.
* The logic layer reports disconnect error code 17302024 (0x01080208) and
  reconnects within 0.25–3 s. There are 6 such events in the log.

### 3.11 Temperature **[H]**
* 10/50 → BE16 in ADT7420 format (13-bit, 1/16 °C per LSB after a >> 3).
* The plugin shifts right by 7 (integer °C).
* Sign extension ORs 0x0FFFFE00 (not 0xFFFFFE00) and the value is then
  converted as **unsigned**. Negative temperatures therefore come out as
  about 2.7e8, a plugin bug.
* The open-source driver should compute `int16(raw) / 128.0`.
* The GSY-variant board/IR temperatures in 00/07 are BE16 hundredths of °C.
* No thresholds or alarms exist in CEinScan10.

### 3.12 Flash "custom info" (calibration blob) **[H]**
* Address space: 1 MiB (offset and offset+size must be < 2^20, else 0x805).
  Organised as 4096-byte pages. The page number is `offset >> 12`.
* **Read** `readCustomInfo(offset, buf)` (0xf6c0):
  * For each page p from `offset>>12` to `(offset+size−1)>>12`, send on
    the bulk channel `ss 00 10 57 00 00 00 02 p_hi p_lo` (5120-byte buffer).
    The reply carries 4096 data bytes at offset 9.
  * The requested byte range is then copied out of the page images.
  * A failed page gives 0x80B.
  * Example: `ReadData(0, 6568)` reads pages 0 and 1.
* **Write** `writeCustomInfo(offset, data)` (0xea4c). **DANGEROUS**
  (it overwrites factory calibration):
  1. Read every affected page (read-modify-write).
  2. Patch the pages.
  3. For each page send `ss 00 10 58 00 00 10 02 p_hi p_lo <4096 bytes>`
     (length 0x1002, 5120-byte buffer, reply capacity 1024).

  There is no separate erase command, so the firmware presumably erases on
  write. Write failures are **ignored** (the function still returns 0).
  EXStar did this once in the logged session: `WriteData` at offset 923,
  3873 bytes, pages 0–1, followed by `ReloadCustomInfo`.
* **Blob layout (partial)** [M]. Fields read by `reloadCustomInfo` (0xa364)
  from the 6568-byte blob (packed, unaligned):

  | Blob offset | Size | Use |
  |---|---|---|
  | 0x2FD | 48 | colour-correction params, **with strobe** context |
  | 0x339 | 12 | " (3× float/int?), with strobe |
  | 0x34E | 48 | colour-correction params, **no strobe** context |
  | 0x38A | 12 | ", no strobe |
  | 0x12C0 | 72 | 9 doubles: 3×3 colour matrix, with strobe |
  | 0x1308 | 72 | 9 doubles: 3×3 colour matrix, no strobe |

  The rest of the blob (probably stereo calibration and device info) is
  consumed by EXStar's calibration libraries, not by CEinScan10.

### 3.13 Other read-only / local queries **[H]**
* `getExposureTimeRange`, `getGainRange` and `getMaxImageSize` return values
  cached at connect.
* `getPixelFormat` returns an enum from pixel bits and colour mode:
  * mono: 8 → 0x108, 16 → 0x110, 24 → 0x118
  * colour: 24 → 0x218, otherwise 0x208
* `set/getInterestSize`, `set/getAOIOffset` and `set/getTriggerMode`
  (camera-interface versions) are stubs with no device traffic.

### 3.14 Firmware update / IAP. **DANGEROUS, existence only** **[H]**
* `updateFirmware` (0xffdc):
  1. Verify the image with an 8-bit additive checksum after each 4096-byte
     block, so blocks are 0x1001 bytes.
  2. Set the "updating" flag, which pauses the heartbeat.
  3. On the bulk channel, send a header packet group 00 op 06 carrying the
     total image size.
  4. Send 0x1001-byte chunks with op 06, pausing 50 ms between chunks.
  5. `Progress` runs 100 → 950, then 1000.
* `eraseAppHead` (CC/00) presumably invalidates the application so the
  device boots into the bootloader.
* **Do not implement either in the open driver without a recovery path.**

---

## 4. Image handling

### 4.1 Stream start / stop **[H]**
* `startStream` (0xbea0) sends **no device command**. It:
  * creates an `ImageStreamThread` with payloadSize = width₀ × height₀
    (1280 × 1024 = 1 310 720 bytes, taken from sensor 0 for all sensors) and
    pkgSize 0xA400;
  * connects its `receivedNewImage` signal (direct connection);
  * resets the 16-slot ring.
* Images flow whenever the transport stream switch is on (set at connect) and
  the trigger switch is non-zero.
* `stopStream` stops the thread only. The logic flag `ImageStream` maps to
  the logic CaptureThread plus start/stopStream. `ProtectCapture` is
  logic-only. [M]

### 4.2 Packet reassembly (`ImageStreamThread::run` 0x17480) **[H]**
See `protocol-transport.md` §3.6 for the full algorithm. Offsets are within
the USB packet (after the transport's 4-byte BE32 length prefix):

| Pkt offset | Use in CEinScan10 |
|---|---|
| 2 | bits 5..3 = **camera id** (1 = sensor 0, 2 = sensor 1, 4 = sensor 2/RGB). Bits 2..0 = **sub-field B**, copied into the image (4.4). |
| 8 | end-of-frame flag (1) |
| 12..15 | BE32 **group/frame id**, from the last packet. Used to group the three sensors' images. |
| 16..23 | BE64 **timestamp/counter**, from the first packet. Copied into the image. |
| 32.. | pixel payload, 8-bit, row-major, stride = width (no padding) |

Each frame is 1 310 720 bytes. At 41 952 payload bytes per full packet that
is 31 full packets plus one partial, which matches pkgCountPerUnit = 32.
[M] Sanity messages: `pkgSize:` (bad length) and
`Error:missed hasReceived:… payloadSize:… realPkgSize:… pkg_12:…` (short
frame, resync).

### 4.3 Triplet assembly (`CEinScan10::receivedNewImage` 0xc078) **[H]**
* The ring has 16 slots. Each slot holds up to 3 image buffers, a sensor
  bitmask and the group id.
* When an image arrives whose group id differs from the current slot's
  (non-empty) group id:
  * if the slot mask == 0b011 (both IR, no RGB), the slot is **published**
    and the ring advances. IR-only pairs are therefore emitted one frame
    late;
  * otherwise the slot is discarded.
* The image is stored in the current slot, and bit (camera index) is set.
* When the mask reaches 0b111, the slot is published immediately and the ring
  advances.
* Publishing appends the slot index to a ready queue (max 16; the oldest is
  dropped) and wakes `grabImage`.
* Log statistics: 25 936 groups of 2 images (scan mode, RGB count 0) and
  1 243 groups of 3.
* Errors:
  * `stream not opened`
  * `error input data [ReceiveNewImage]` (null data, zero length or null
    info)
  * `grabImage`: `Einscan10 GrabImage Timeout` → 0x40A, seen 142 times as
    errorCode 1034
  * `GrabImage get error index` → 0x41E
  * stream not running → 0x409
  * no device → 0x200

### 4.4 Per-image processing and embedded metadata **[H]**
**Mono sensors (index 0, 1).** `convertImage` (0xdde4) applies that sensor's
ConvertParam in place, then copies the frame.

**RGB sensor (index 2):**
* **EnableRGB = false:** raw 8-bit Bayer is copied; marker byte = 1.
* **EnableRGB = true (default):** `cv::cvtColor` with code **48 =
  `COLOR_BayerRG2BGR`** gives 3-channel 8-bit BGR; marker byte = 3.
  * OpenCV names Bayer patterns after the second row, second/third column.
    Code 48 therefore means the sensor's top-left 2×2 is **B G / G R**
    (**BGGR**) in OpenCV's convention. [M on physical CFA; H on the code
    constant]
  * Only bilinear (non-VNG, non-EA) demosaic is used.

**Metadata written over the first pixels** of each stored buffer (this
overwrites pixel data):

| Buffer byte | Content |
|---|---|
| 0 | sub-field B (pkt byte 2, bits 2..0). Meaning unknown; possibly the pattern/trigger index within the group [L]. |
| 0x0B..0x12 | the 64-bit timestamp/counter, **little-endian** (pkt bytes 23..16 reversed) |
| 0x14 | RGB only: 1 = raw Bayer, 3 = colour |

`grabImage` then **zeroes bytes 2..5** of every output image. Downstream code
therefore sees a small metadata header in row 0.

### 4.5 ConvertParam (host-side flip/rotate) **[H]**
Applied per sensor, in this order:
1. VERFLIP (swap rows).
2. HORFLIP (mirror each row).
3. If ENABLEROTATE is set:
   * ROTATEANGLE 90: transpose with direction set by CLOCKWISEROTATE;
     width and height swap.
   * ROTATEANGLE 180: rotate 180°.

If HORFLIP, VERFLIP and ENABLEROTATE with 180 are all set, the function does
nothing (they cancel). RGB uses the 3-byte-pixel variant `convertRGBImage`
(0xe49c). Defaults are all off. ConvertParam is not set in the logged
session.

### 4.6 RGB colour correction (inside `grabImage` 0xcb04) **[H]**
Only when the RGB buffer is colour (marker 3):
* On first use, two GPU contexts are created with
  `SNColorAlgorithm::snColorCorrectGPUIni_scan` from `libSn2DColorCorrect`
  (Metal kernel `Sn2DColorCorrect.metallib`), using the flash blob
  parameters (3.12). One context is "no strobe", the other "with strobe".
* The context is chosen by the strobe-on flag (route-1 luminance ≠ 0).
* Then `snColorCorrectYgamma_scan(ctx, img, &DIAT1, &DIAT3)` runs.
* Then **R and B are swapped** (BGR → RGB).
* Then `convertRGBImage` runs.
* Failure logs `snColorCorrectYgamma_scan return false` or
  `…(WithStrobe) return false`.
* An open driver can skip this and deliver the demosaiced image, applying
  the 3×3 matrix from the blob if wanted.

### 4.7 Output `ImageBlock` (`grabImage` 0xcb04) **[H]**
* One block per present sensor, in the order sensor 0, 1, 2. Each block has:
  * camera index (0/1/2)
  * width, height (1280 × 1024, swapped if a 90° rotation is enabled)
  * bits per pixel: 8 for mono and raw Bayer, 24 for colour RGB
  * a QByteArray of W×H or W×H×3 bytes
* Frame rate: every sensor-0 image increments a counter, and
  `FrameRate = count × 1000 / elapsed_ms` is recomputed about once per
  second.

### 4.8 Publishing (logic plugin, context only) **[M]**
* The logic `InternalE10Device::CaptureThread` calls `grabImage` in a loop.
  It logs "this group image size: 2|3" and pushes each image to shared
  memory under names formatted as `InternalE10Image%d`, via
  `sendRealTimeImageThread` / `sendImageInfo` with memKey.
* `CompressImage` / `RotateParam` options also exist there.
* This is outside CEinScan10. No `InternalE10Image` strings exist in the
  device plugin.

---

## 5. Error codes returned by CEinScan10 **[H]**

| Code | Meaning |
|---|---|
| 0x200 | no device handle |
| 0x201 | page buffer bookkeeping failure |
| 0x301 / 0x303 | attach failed (transport returned 2/4, or other) |
| 0x305 | reconnect attach failed |
| 0x307 | unknown property or command |
| 0x309 | stream start or disconnect without a handle |
| 0x30A | getProperty I/O failed or invalid |
| 0x30B | setProperty I/O failed |
| 0x30C | setProperty invalid parameter |
| 0x30D | executeCommand I/O failed |
| 0x409 | grab while stream not running |
| 0x40A | grab timeout |
| 0x40C | camera index > 2 |
| 0x40D / 0x40E | set / get exposure failed |
| 0x410 / 0x411 | set / get gain failed |
| 0x41E | bad ring index |
| 0x804 | empty command buffer |
| 0x805 | flash range invalid |
| 0x806 | empty firmware |
| 0x807 | firmware checksum failed |
| 0x808 | firmware transfer failed |
| 0x809 / 0x80A | transport send / receive failed |
| 0x80B | flash page read failed |

---

## 6. EXStar session sequence (reconstructed)

Source: `SnSyncService_2026_09_27_135121_671_1.txt` (API calls, in order),
mapped through sections 2–3. Sequence bytes are written as `ss` (they depend
on interleaved heartbeats). The heartbeat request `ss 00 00 07 00 00 00 00`
(50-byte buffer) repeats every HeartBeatPeriod throughout and is not
repeated below. The `HeartBeatPeriod` value the logic sets in `initFunc` is
not logged. Default 250 ms.

**13:51:23.887 open (connectDeviceBySerial):** exactly as in section 2:
`49 [00 03 0D 40]`, `41 [11]`, 00/00, 10/00, 00/01, 10/01, 10/51, then 3 × 8
per-sensor reads with masks 01/02/04, 00/05, (heartbeat starts), 6F [01],
57 [00 00], 57 [00 01].

**initFunc (13:51:24.200):**
* `HeartBeatPeriod` = ? (local only)
* `ScanTriggerPeriod` = ?: `ss 00 10 49 00 00 00 04 <BE32>`
* start stream (host only)

**Queries:**
* `FirmwareVersion`: `ss 00 00 05 00 00 00 00`
* `SensorCount`: `ss 00 10 51 00 00 00 00` → 3
* `ReadData(0, 6568)`: bulk `ss 00 10 57 00 00 00 02 00 00`, then `… 00 01`
* `Serial`: `ss 00 00 04 00 00 00 00` → `0009011402CF0C20`

**13:51:30:** `ProtectCapture` (logic), `EnableRGB` = true (local),
`YGamma` 0.2/0.1 (local).

**13:51:51 scan setup:**
```
TriggerMode {3,0}        ss 00 10 41 00 00 00 01 03
ScanTriggerPeriod 68000  ss 00 10 49 00 00 00 04 00 01 09 A0
ImageStream on           (host)
EnableRGB true           (local)
LDRGBLuminance 0         ss 00 10 68 00 00 00 01 00
StrobeLuminance r0=0     ss 00 10 70 00 00 00 03 00 00 00
StrobeLuminance r1=0     ss 00 10 70 00 00 00 03 01 00 00
```
**13:51:54 onward, auto-exposure ramp** (repeating pattern):
```
CameraGain  [0,80]       ss 00 10 27 00 00 00 03 01 00 50
CameraGain  [1,80]       ss 00 10 27 00 00 00 03 02 00 50
CameraExposure [0,2200]  ss 00 10 23 00 00 00 05 01 00 00 08 98
CameraExposure [1,2200]  ss 00 10 23 00 00 00 05 02 00 00 08 98
StrobeLuminance r0=3000  ss 00 10 70 00 00 00 03 00 0B B8
```
…gain rises through 95, 100, 105, 110, 115 to 120 and exposure through 2400 …
4400 (`00 00 11 30`) to 5200 (`00 00 14 50`), with strobe 3000, 4000
(`0F A0`), 5000 (`13 88`), 6000 (`17 70`) and 9000 (`23 28`).
`LDRGBLuminance` 60 = `ss 00 10 68 00 00 00 01 3C`.

**Distance LEDs**, about 4 Hz while scanning:
```
Indication {DEVICESTATE 1, DISTANCE 2}  ss 00 10 62 00 00 00 02 02 01
                         DISTANCE 1     ss 00 10 62 00 00 00 02 01 01
                         DISTANCE 0     ss 00 10 62 00 00 00 02 00 01
```
**Buttons:** 00/07 replies with byte 9/10/11 = 1 → logged `BUTTONn SingleClick`
(first at 13:52:05.690, BUTTON1).

**Texture / preview mode:**
```
ImageStream off
TriggerMode {1,1}          ss 00 10 41 00 00 00 01 11
ScanTriggerPeriod 100000   ss 00 10 49 00 00 00 04 00 01 86 A0
ImageStream on; EnableRGB true
CameraExposure [0,1500]    ss 00 10 23 00 00 00 05 01 00 00 05 DC
CameraGain [0,400]         ss 00 10 27 00 00 00 03 01 01 90
CameraGain [2,180]         ss 00 10 27 00 00 00 03 04 00 B4
```
**13:57:23.935 calibration write**, `WriteData(923, 3873 bytes)` →
`writeCustomInfo`:
```
bulk  ss 00 10 57 00 00 00 02 00 00          (read page 0)
bulk  ss 00 10 57 00 00 00 02 00 01          (read page 1)
bulk  ss 00 10 58 00 00 10 02 00 00 <4096>   (write page 0)   DANGEROUS
bulk  ss 00 10 58 00 00 10 02 00 01 <4096>   (write page 1)   DANGEROUS
```
Then `ReloadCustomInfo`: two page reads again. The payload content is
deliberately not reproduced here.

**Disconnects / reconnects:** 14:12:04, 14:12:26, 14:12:47, 14:24:12, 14:24:36
and 14:24:56. Each is followed by the replay (section 2) and ClearState
`ss 00 10 7B 00 00 00 00` (10-byte buffer).

**Close:** `ss 00 10 41 00 00 00 01 00`, then stream/heartbeat teardown.

---

## 7. Open questions

1. **Units** of trigger period and exposure (assumed µs) and the gain scale
   (register value; SC130GS analog/digital gain encoding unknown). Needs
   hardware or FPGA documentation.
2. **Button codes 2/3** (DoubleClick/LongClick) are inferred from enum
   order only. The log only shows code 1. It is also unknown whether the
   firmware clears the latch on read or only on ClearState (10/7B).
3. **Packet byte 2 bits 2..0** (sub-field B, written to image byte 0): is it
   the pattern/trigger index within a MONOTRIGCOUNT burst?
4. **Which physical sensor is index 0 or 1** (left or right IR)? The
   stereo calibration in the flash blob must be matched.
5. **Bayer phase.** Code 48 implies BGGR at (0,0) under OpenCV naming. Check
   the SC130GS CFA readout and any FPGA flip.
6. **Strobe route 0 / route 1 and "LD"** hardware mapping (IR flood versus
   white LED versus laser projector) is inferred from usage.
7. **Distance indication semantics** (0/1/2 = near/ok/far?) and the
   DEVICESTATE meanings beyond 1.
8. **Full flash blob layout** (6568 bytes). Only the colour-correction
   offsets are known from this plugin. See libSn3DCalibDevFile /
   libSn3DCalibrationJR.
9. **GSY product variant.** Which product-name token triggers the extra
   temperature fields? It is not observed on this unit; the product name was
   not logged.
10. **10/58 reply.** Its size and content are not checked by the plugin
    (reply capacity 1024), and write errors are ignored.
11. **The HeartBeatPeriod value** the logic layer sets in `initFunc` (not
    logged).
12. **Reply length field for register reads.** It is only verified for
    string replies. Numeric replies are read at fixed offsets and the length
    is ignored.
