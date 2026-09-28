# Einstar USB transport layer (libsn3DCommunicateLayerM 2.1.8)

Static analysis of `libsn3DCommunicateLayerM.2.1.8.dylib` (arm64, from
`EXStar.app/.../SnSyncService.app/Contents/Frameworks/`). The
`einscan10.2.0.0.dylib` plugin was consulted read-only to see how it calls the
layer and how it parses stream packets. Everything here was derived from the
binaries. No hardware was involved.

Clean-room note: this document describes behaviour, layouts and constants in
prose and tables. It contains no decompiled code.

Confidence tags: **[H]** high (read directly from unambiguous code), **[M]**
medium (inferred from several consistent observations), **[L]** low
(hypothesis). Addresses are in the CommunicateLayer dylib unless they are
prefixed `plugin:`.

---

## 0. Architecture overview

* The library is a thin libusb-1.0 wrapper. There are two layers:
  * `CCommunicateManager` is one object per physical device. It owns one
    `libusb_context` (created in its ctor, 0x3ea8) and one `libusb_device_handle`.
  * `CommunicateLayer::*` is the exported free-function API (0x15988–0x1886c).
    It works on an opaque `void*` handle, which is really a `_DEVICENODE*` held
    in a global `std::map<std::string serial, _DEVICENODE*>` (`g_deviceMap`).
* A background **monitor thread** (`moniteDeviceThread`, 0x13984) does all
  enumeration, opening and reconnection. `attachDevice` never opens USB. It
  only looks a serial up in the map that the monitor thread keeps filled.
* Endpoint table (object +0x13c..+0x140, filled by the ctor 0x3ea8) **[H]**:

| Role | Addr | libusb call used | Notes |
|---|---|---|---|
| Command OUT | 0x01 | `libusb_interrupt_transfer` | sendCommandLine 0x4acc |
| Command IN (reply) | 0x81 | `libusb_interrupt_transfer` | receiveCommandReply 0x4f28 |
| Bulk data OUT | 0x02 | `libusb_bulk_transfer` | sendBulkData 0x6240 |
| Bulk data IN | 0x82 | `libusb_bulk_transfer` | receiveBulkData 0x5db8 |
| Image stream IN | 0x83 | async bulk transfers | captureImageCallBack 0x683c |

The library never reads the configuration or endpoint descriptors. These five
addresses are hard-coded. `getPipeAddressList` (0x7c88) returns exactly
`01 81 02 82 83`, and `getPipeCount` (0x7c04) returns 5. **[H]**

### Status/return codes used across the API [H]
| Code | Meaning |
|---|---|
| 0 | OK |
| 2 | no device found (getOnlineDeviceSerial*) |
| 3 | device handle not open (manager's libusb handle is null) |
| 4 | node not registered in the map, or node offline |
| 5 | bad argument |
| 0xFF | generic failure (libusb error, timeout, stream running, and so on) |

---

## 1. Enumeration, open and identification

### 1.1 VID/PID matching [H]
* The monitor thread hard-codes **VID 0x3267** (0x13984, local constant).
* `enumDevices` (0x13340) walks `libusb_get_device_list`. It accepts **every
  device whose idVendor == 0x3267, whatever its PID**, and records two
  parallel lists: the PID, and `libusb_get_port_number(dev) & 0xFF`.
* No PID list exists in this library. Product discrimination happens later, in
  the plugin, through the product-name string (see 1.6). The PID only matters
  for the "is it still there" check (section 5).

### 1.2 Open (`CCommunicateManager::attach(vid, pid, port)`, 0x522c) [H]
Steps, in this order:
1. If vid or pid is 0, return 5. If a handle is already open, call `detach()`
   first (release interface 0, then close).
2. Open:
   * If port == -1, call `libusb_open_device_with_vid_pid(vid, pid)`. The
     monitor never uses this path.
   * Otherwise call `openDeviceByPortnumber` (0x5c64). It walks the device
     list and opens the first device with matching idVendor, idProduct and
     `(port_number & 0xFF) == port`. Only the last-hop port number is
     compared, not bus or path. Two scanners on the same port number of
     different hubs would be ambiguous. **[H]**
3. **`libusb_claim_interface(handle, 0)`**, interface 0 only. There is **no**
   `set_configuration`, `set_interface_alt_setting`,
   `detach_kernel_driver` or `reset_device` call anywhere in the library.
   **[H]**
4. Drain the command IN pipe. While holding the command mutex, read EP 0x81
   with a 1024-byte buffer and a 100 ms timeout, up to 5 times, stopping at
   the first error or timeout.
5. Drain the bulk IN pipe. Read EP 0x82 in 1024-byte reads with a 100 ms
   timeout, up to 30 times, stopping at the first error. `receiveBulkData`
   itself calls `clear_halt(0x82)` when a read ends with
   LIBUSB_ERROR_TIMEOUT (-7). In practice the drain therefore always ends
   with one clear-halt on 0x82.
6. `libusb_clear_halt(0x81)`, then `libusb_clear_halt(0x82)` (`resetPipe`,
   0x4e78). EP 0x01, 0x02 and 0x83 are **not** cleared.
7. **Serial query** over the command channel (see 1.4). If it fails, release
   interface 0, close, and return 0xFF.
8. Store the serial string, vid and pid.

### 1.3 `resetPipe(ep)` (0x4e78) [H]
Calls `libusb_clear_halt(ep)` only when `(ep & 0xF0)` is 0x00 or 0x80.
Otherwise it returns 5. `resetCommandPipe` clears 0x01 then 0x81.
`resetBulkPipe` clears 0x02 then 0x82. Their int argument is ignored.
`resetDeviceInDriver` is a no-op that returns 0xFF on macOS.

### 1.4 Serial number [H]
The serial does **not** come from a USB string descriptor. It is read with a
command-channel request (in attach, 0x522c):

* Request: 30 bytes (0x1E), `FF 00 00 04` followed by 26 zero bytes. That is
  seq=0xFF, byte1=0, group byte2=0x00, opcode byte3=0x04.
* It goes through `interruptIOAndCheck` (see 2.4). The reply buffer is 100
  bytes.
* Serial = reply bytes **9..16** (8 bytes), formatted as 16 upper-case hex
  digits (`%02X` ×8). For example, bytes `12 34 56 78 9A BC DE F0` at offsets
  9..16 give the serial `123456789ABCDEF0`.
* This string is the key into `g_deviceMap` and is what
  `getOnlineDeviceSerial` returns.

### 1.5 Product name (`queryProductName`, 0x7dd8) [H]
* Request: 100 bytes, `FE 00 00 01` followed by zeros (seq=0xFE, group=0x00,
  opcode=0x01).
* Reply: bytes **5..8 are a big-endian u32 length N**. The ASCII name starts at
  byte **9** and ends at 9+N. The code writes a NUL at index 9+N without any
  bounds check.
* It is called by the monitor thread when a node is created, and the result is
  stored in the node (+0x68).

### 1.6 How the plugin selects Einstar devices (plugin, context only) [M]
`plugin:0x6864` calls `getOnlineDeviceSerialEx` and counts devices whose
product name, upper-cased, starts with `"EINSCAN10"` (it compares the first 9
characters). `plugin:0x6b90` then splits the product name on `_` and sets a
flag if the second token is `GSY`.

---

## 2. Command channel

### 2.1 Packet framing (as used by this library and the plugin) [M]
Command packets have a common header:

| Offset | Request | Reply |
|---|---|---|
| 0 | sequence byte | echo of the request's seq |
| 1 | 0x00 (holds the mask key after masking; see 2.5) | 0x00, or the key; 0x02 is treated specially (see 2.4) |
| 2 | command group (0x00 for the layer's own queries, 0x10 for plugin commands) | echo |
| 3 | opcode | echo |
| 4 | request: first byte of the length field | **status**: 0 = OK (the plugin checks this) |
| 4..7 | request: BE32 payload length (plugin sends `00 00 00 01` + 1 payload byte) **[L/M]** | — |
| 5..8 | — | reply: BE32 payload length **[H for product name]** |
| 8.. | request payload | — |
| 9.. | — | reply payload |

* The layer's own queries use fixed sequence numbers: 0xFF for the serial,
  0xFE for the product name. **[H]**
* The plugin's sequence counter is `(n+1) % 0xFE`, so it runs 0..253 and never
  collides with 0xFE or 0xFF. **[H]** (plugin:0x6b90)
* Reply validation (layer and plugin): reply bytes 0, 2 and 3 must equal the
  request's. The plugin also requires reply byte 4 == 0. **[H]**

### 2.2 Transfer sizes and timeouts [H]
* `sendCommandLine` (0x4acc) sends a single `libusb_interrupt_transfer` of
  exactly the caller's length to EP 0x01. The C API always uses a 1000 ms
  timeout.
* `receiveCommandReply` (0x4f28) sends a single interrupt IN of the caller's
  buffer length on EP 0x81. It returns the actual length.
* `CommunicateLayer::sendCommandAndWaitReply(h, buf, len&)` (0x17778) takes
  the command mutex, sends `len` bytes, then reads **up to `len` bytes** into
  the **same buffer**, and updates `len` to the actual reply length. Timeouts
  are 1000 ms each, with no retry. The requested reply length equals the
  request length. The device must therefore reply with no more bytes than it
  was sent, or libusb reports an overflow. The plugin uses 0x10 (16) or 0x32
  (50) bytes for most commands.
* `sendCommandAndWaitReplyEx(h, buf, len&, replyMax)` (0x178bc) is the same,
  except the IN buffer size is `replyMax`.
* `sendCommand` (0x175f4) **always returns 0**, even when the libusb transfer
  failed. `receiveCommandReply` (0x176a4) returns 0 or 0xFF.
* `getMaxCommandSize` (0x17414) is broken. It never writes its output, and
  returns 0xFF for a valid node.
* The C API applies **no mask decode** on replies. Only the internal
  `interruptIOAndCheck` path decodes.

### 2.3 Retry logic
* Library path, `interruptIOAndCheck` (0x477c) **[H]**. It is used only for
  the serial and product-name queries. It copies at most 100 bytes of the
  request into a 101-byte scratch buffer and makes up to **5 attempts**:
  1. Send the request (1000 ms). If the send fails: `clear_halt(0x01)` and
     **stop at once** (no retry).
  2. Read a reply of up to 100 bytes from 0x81 (1000 ms).
  3. If reply byte1 != 0x02, apply `maskDecode` to the reply.
  4. If the read succeeded and reply bytes 0, 2 and 3 equal the request's,
     accept it. Otherwise `clear_halt(0x01)` and `clear_halt(0x81)`, then
     retry.

  Quirk: success is reported only when the retry counter is still ≥ 1 after
  the decrement. A match on the 5th attempt is therefore reported as a
  failure (off-by-one).
* Plugin path, `plugin:CEinScan10::interruptIO` (plugin:0xb4f4) **[H]**. With
  retry enabled it makes up to 5 attempts. Each attempt calls
  `sendCommandAndWaitReply` and validates reply bytes 0, 2 and 3 plus
  byte 4 == 0. After a failure it calls `resetCommandPipe` (clear 0x01 and
  0x81) and sleeps 20 ms.

### 2.4 Coder type selection [H]
* `selectInterruptCoderType(u16)` (0x7f8c) and `selectBulkCoderType(u16)`
  (0x803c) accept a bitmask with **at most one bit set** (a popcount ≤ 1 test)
  and store it at +0x138 and +0x13a. Both are 0 after construction.
* Only **bit 0 of the interrupt coder type** is ever tested. `sendCommandLine`
  runs `maskEncode` on the outgoing buffer, in place, when that bit is set.
  No other coder type is implemented. The bulk coder type is stored but never
  read, so bulk and stream data are never masked by this layer.
* Neither selector is exported through the `CommunicateLayer` C API, and the
  einscan10 plugin does not import them. **In the shipping EXStar
  configuration, outgoing commands are sent unmasked.** **[H for the plugin's
  imports; M for "shipping configuration"]**
* Decoding is keyed on the data itself. `interruptIOAndCheck` always runs
  `maskDecode` unless reply byte1 == 0x02. An unmasked reply with byte1 == 0
  decodes with key 0, which leaves it unchanged, so this is harmless for
  plain replies.

### 2.5 Mask algorithms [H]
**Encode** (`maskEncode`, 0x6044):
1. Seed the C PRNG with the current time: `srand(time(NULL))` on **every**
   call. Two commands in the same second therefore get the same key.
2. `k = rand() % 16` (0..15). The mask byte is `M = (k << 4) | k`, which is
   `k × 0x11`.
3. XOR **every byte** of the buffer, 0..len-1 including bytes 0..3, with `M`.
   There is no separate key field. The key is implied because request
   byte1 is always 0x00, so after masking byte1 == `M`.
4. If k == 0, the buffer is unchanged.

**Decode** (`maskDecode`, 0x5128):
1. `k = buf[1] >> 4` (high nibble of byte1). `M = k × 0x11`.
2. XOR every byte 0..len-1 with `M`. This restores byte1 to 0x00 as long as
   the plaintext byte1 had a zero high nibble.

**Worked examples**
* Serial request, k = 0xA (M = 0xAA). Plain:
  `FF 00 00 04 00 00 00 00 00 00` becomes masked
  `55 AA AA AE AA AA AA AA AA AA`. To decode, byte1 = 0xAA gives k = 0xA, the
  mask is 0xAA, and the original is restored.
* Plugin-style command, k = 3 (M = 0x33). Plain:
  `07 00 10 16 00 00 00 01 02` becomes
  `34 33 23 25 33 33 33 32 31`.
* A reply `07 33 23 25 33 …` decodes with k = 3 and gives `07 00 10 16 00 …`.
* A reply whose byte1 is 0x02 is passed through without decoding.

---

## 3. Image stream (EP 0x83)

### 3.1 Parameters and defaults

| Field | Offset | Ctor default | Plugin setting (plugin:0x6b90) | Meaning |
|---|---|---|---|---|
| attemptPkgSize | +0x144 | 0x400 (1024) | **0xA400 (41 984)** | length of each libusb bulk IN transfer = one device "packet" |
| pkgCountPerUnit | +0x148 | 10 | **32 (0x20)** | number of transfers in flight = maximum packets per unit |
| cacheUnitCount | +0x14c | 2 | **100** | ring size of unit buffers |
| maxGrabWaitTime | +0x150 | 10 000 | **100 000** | wait in `receiveImageStream`, in **microseconds** (100 ms in the plugin) |
| unitSize | +0x154 | 10×1024 = 10 240 (**missing the +4**, a ctor bug) | 32 × (41 984 + 4) = **1 343 616** | bytes returned per `receiveImageStream` |

**[H]** for all values. Rules:
* The setters (0x76f4, 0x78a0, 0x7970) return 0xFF while the stream is
  running and 5 for a 0 argument.
* When a setter changes a value, it recomputes
  `unitSize = pkgCountPerUnit × (attemptPkgSize + 4)` and frees all cached
  unit buffers (`releasePkgUnitBuf`, 0x77c4). If the value did not change,
  nothing is recomputed.
* The constructor's unitSize omits the +4 per packet. A caller that never
  changes the defaults would overflow the unit buffer. The plugin always
  changes both values, so it is not affected.
* `setMaxGrabWaitTime` (0x7a2c) may be called while the stream is running.
* `getImageDataUnitSize` (0x7ab8) returns **cacheUnitCount** instead of
  unitSize (a bug). The plugin does not call it.

### 3.2 streamSwitch / driverStreamSwitch (0x4534 / 0x17e14) [H]
* **No USB control or command traffic is sent.** Starting the stream only
  starts host-side reading of EP 0x83. The device-side trigger is done with
  separate plugin commands (`triggerSwitchIO` and similar, out of scope here).
* On `true`, if the stream is not already running: join any old thread, set
  the "run" flag (+0x218), and start the `captureImageCallBack` thread.
* On `false`: clear the run flag and join the thread.
* The C wrapper holds the stream mutex (+0xc8) around the call. It checks only
  that the node is in the map, not that it is online.
* The plugin switches the stream on right after connecting, and it stays on
  (plugin:0x6b90). It switches it off in `disconnectDevice` (plugin:0xaab8).

### 3.3 Capture thread (`captureImageCallBack`, 0x683c) [H]
1. Set the "resubmit" flag (+0x240).
2. Create **pkgCountPerUnit** transfers (32). Each transfer gets:
   * its own buffer of attemptPkgSize bytes;
   * `libusb_fill_bulk_transfer` on EP **0x83**, length attemptPkgSize,
     **timeout 500 ms**;
   * immediate submission (lambda 0x86b0).

   A submit failure is only logged.
3. While the run flag is set, loop on `libusb_handle_events_timeout` on the
   manager's context with a 200 ms tv.
4. On stop:
   1. Clear the resubmit flag.
   2. For up to 1.5 s: call `libusb_cancel_transfer` on every transfer that is
      not yet cancelled, pump events with 50 ms timeouts, and free each
      transfer and its buffer once its status is CANCELLED.
   3. Free whatever remains in the CANCELLED or COMPLETED state.

   Unit buffers and the ready queue are **not** cleared on stop. Stale units
   can therefore be delivered after a restart unless
   `discardDriverFrameCaches` is called.

### 3.4 Transfer completion (`captureImageTransferCallBack`, 0x8a80) [H]
State: `writeUnit` (+0x21c), `writeSlot` (+0x220), a vector of unit buffers
(+0x158), and a FIFO of ready unit indices (+0x170) guarded by a mutex (+0x1a0)
and a condition variable (+0x1e0).

If the run flag is clear, the callback logs and returns without resubmitting.

If `status == COMPLETED`:
1. If no buffer exists yet for `writeUnit`, allocate unitSize bytes (lazy
   allocation). At most cacheUnitCount buffers are ever created.
2. `slot = writeSlot × (attemptPkgSize + 4)`.
3. Write the transfer's **actual_length as a big-endian u32** at
   `unit[slot+0..3]`.
4. Copy the **whole attemptPkgSize buffer**, whatever the actual length, to
   `unit[slot+4 …]`.
5. Read `eof = packet[8]`, the 9th byte of the raw USB packet.
6. Increment `writeSlot`.
7. Commit the unit if either of these holds:
   * `writeSlot < pkgCountPerUnit` and `eof == 1`, meaning end of frame;
   * `writeSlot == pkgCountPerUnit`, meaning the unit is full.

   Commit (lambda 0x8fc4), done under the mutex:
   1. Push `writeUnit` onto the ready FIFO.
   2. `writeUnit = (writeUnit+1) % cacheUnitCount`, and `writeSlot = 0`.
   3. If the FIFO's oldest entry equals the new `writeUnit`, pop it. This
      drops the oldest unread unit on overrun.
   4. `notify_one`.

If the status is anything else (timeout, stall, error), the callback only
logs. There is no clear-halt and no unit reset.

In every case except `status == CANCELLED`, the transfer is resubmitted while
the resubmit flag is set.

Consequences:
* A **unit** is the set of consecutive packets that ends at an EOF-flagged
  packet, or 32 packets, whichever comes first. With the plugin's settings, a
  frame of W×H bytes takes ⌈W·H / 41 952⌉ packets. For example, 1280×1024 =
  1 310 720 bytes needs 32 packets, so in practice **one unit ≈ one frame**.
* Slots after the EOF slot in a committed unit hold stale data from earlier
  frames. Consumers must stop at the EOF slot.
* Timed-out transfers (no data within 500 ms) are simply reposted. This is
  the normal idle behaviour.
* Stream data is **not** masked or decoded by the layer.

### 3.5 `receiveImageStream(h, buf, len&)` (0x80ec / 0x17d3c) [H]
1. Return 5 unless `buf` is non-null and `len ≥ unitSize`.
2. Take the stream mutex (C wrapper), then the queue mutex.
3. If the FIFO is empty, wait on the condition variable for maxGrabWaitTime
   **µs**. On timeout, return 0xFF.
4. Pop the oldest unit index and memcpy **unitSize bytes** of that unit into
   `buf`. Return 0.

`len` is **not updated**. The caller always receives a full unit:
pkgCountPerUnit slots of `[BE32 actual_len][attemptPkgSize raw bytes]`.

`discardDriverFrameCaches` (0x7b08) empties the FIFO and resets `writeUnit`
and `writeSlot` to 0.

### 3.6 Per-packet header on EP 0x83 (parsed by the plugin, plugin:0x17480)
The layer interprets only byte 8. The rest of this layout comes from the
plugin's `ImageStreamThread::run`. Offsets are within the raw USB packet,
which starts at slot+4. Multi-byte fields are big-endian.

| Pkt offset | Size | Meaning | Conf. |
|---|---|---|---|
| 0–1 | 2 | not read by host (magic or packet counter?) | [L] |
| 2 | 1 | bits 5..3 → field A (0–7), bits 2..0 → field B (0–7), both passed as frame metadata. Probably camera/sensor id and image type or channel. | [M] exists / [L] meaning |
| 3–7 | 5 | not read by host | [L] |
| 8 | 1 | **EOF flag**: 1 = last packet of the frame (used by both layer and plugin) | [H] |
| 9–11 | 3 | not read by host | [L] |
| 12–15 | 4 | BE32, taken from the **last** packet of the frame → metadata field (frame index/counter) | [M] |
| 16–23 | 8 | BE64, taken from the **first** packet of the frame → metadata field (timestamp) | [M] |
| 24–31 | 8 | not read by host | [L] |
| 32… | actual_len−32 | image payload (raw 8-bit pixels, row-major) | [H] |

**Header = 32 bytes.** Payload per full packet = 41 984 − 32 = 41 952 bytes.
**[H]**

**Plugin reassembly algorithm** [H]. `payloadSize` = width × height, read from
the device at connect time with commands 0x10/0x16 and 0x10/0x17.

For each unit, iterate over the 32 slots:
1. Read `len` = the slot's BE32 prefix. If `len ≤ 32` or `len > 41 984`, log
   `pkgSize:` and skip the slot.
2. If this is the first packet of a frame (`hasReceived == 0`), latch the
   BE64 timestamp from bytes 16..23.
3. `n = len − 32`.
4. If `EOF == 1` and `hasReceived + n ≠ payloadSize`: log
   `Error:missed hasReceived…payloadSize…realPkgSize…pkg_12…`, reset
   `hasReceived = 0`, and abandon the rest of the unit. This is the resync
   rule.
5. If `hasReceived + n > payloadSize`, restart the frame (`hasReceived = 0`).
6. Copy `min(n, payloadSize − hasReceived)` bytes to the frame at
   `hasReceived`, then add `n` to `hasReceived`.
7. If `hasReceived == payloadSize` and `EOF == 1`: emit the frame with
   metadata {timestamp, BE32 counter from bytes 12..15, fieldA, fieldB}.
8. If `EOF == 1`, reset and stop processing this unit.

A frame can span units. `hasReceived` is carried across units until EOF.

---

## 4. Bulk data channel (EP 0x02 / 0x82)

### 4.1 Framing [H]
* `sendBulkData(buf, len, timeout)` (0x6240) sends
  **floor(len/1024)** chunks of exactly 1024 bytes to EP 0x02, sleeping 1 ms
  between chunks.
  * **Any remainder below 1024 is silently dropped.** Callers must pad to a
    multiple of 1024.
  * The chunk loop stops at the first error.
* `receiveBulkData(buf, len&, timeout)` (0x5db8) reads **floor(len/1024)**
  chunks of 1024 bytes from EP 0x82 into consecutive 1 KiB slots, sleeping
  1 ms between reads.
  * Short reads do not end the loop. The next read still goes to the next
    1 KiB offset.
  * On LIBUSB_ERROR_TIMEOUT it calls `clear_halt(0x82)` and fails.
  * On success, `len` is set to the sum of the actual lengths.
* There is no masking on bulk (see 2.4).
* The payload appears to use the same header convention as commands. The
  plugin's `bulkIO` (plugin:0x10414) validates reply bytes 0, 2 and 3 against
  the request and requires byte 4 == 0. **[M]**

### 4.2 C API wrappers [H]
* `sendBulkData(h, buf, len)` (0x17a0c): 1000 ms per chunk. It does **not**
  take the data mutex.
* `receiveBulkReply(h, buf, len&)` (0x17ac0): 1000 ms per chunk.
* `sendDataAndWaitReply(h, buf, len&, replyLen)` (0x17b94):
  1. Take the data mutex (+0x88).
  2. Send `len` bytes with a 2000 ms per-chunk timeout.
  3. Try up to **3 times** to read `replyLen` bytes into the **same buffer**,
     with a 600 ms per-chunk timeout and an 80 ms sleep between attempts.

  `len` is **not** updated with the reply size; the internal count is lost.
  The plugin retries the whole exchange up to 5 times, calling
  `resetBulkPipe` (clear 0x02 and 0x82) between tries.
* `dataStreamlock(h, bool)` (0x1754c) locks or unlocks the data mutex.
  `commandChannelLock(h, bool)` (0x174a4) locks or unlocks the command mutex
  (+0x48).

---

## 5. Hot-plug monitoring and auto-reconnect (`moniteDeviceThread`, 0x13984) [H]

`initializeSystem(const GUID&)` (0x15988) ignores its GUID and starts the
thread. The thread keeps its **own** libusb context, which it uses for
enumeration only. There is **no libusb hotplug callback**; the thread polls.

Each iteration, about every **500 ms**:
1. `enumDevices(VID 0x3267)` → PID list and port list.
2. For every node in the map whose manager currently has an open handle:
   * If the node's PID is **absent** from the PID list **and** its port is
     absent from the port list, the device has disconnected. Set
     `online = 0` and call `detach()` (release interface 0, then close). The
     capture thread is not stopped.
   * Otherwise, if the node is online, remove that PID and port from the lists
     so the device is not re-opened.
3. For every remaining (PID, port) pair, while holding the map lock:
   1. Create a new `CCommunicateManager`, which has its own libusb context.
   2. Call `attach(0x3267, pid, port)` (the full sequence in 1.2). On failure,
      delete the manager.
   3. Read the serial.
      * **Serial is new:** create a `_DEVICENODE` with the manager,
        `online = 1`, the serial, pid, port and `refcount = 0`. Query the
        product name, then insert the node into the map.
      * **Serial is already known** and `g_bEnableAutoReconnect` is set
        (the **default is true**, byte at 0x285b8 = 1): take the node mutex
        and call `updateDeviceHandle` (0x65dc). That function:
        * stops the node's stream;
        * takes the command, data and stream mutexes;
        * calls `libusb_exit` on the old context;
        * moves the new handle, context, vid, pid and serial into the
          **existing** manager;
        * restarts the stream if it was running.

        Stream parameters (pkg size, count, cache, wait) live in the existing
        manager and so survive the reconnect. The monitor then sets
        `online = 1`, updates serial, pid and port, and deletes the temporary
        manager. It also calls `queryProductName` on the temporary manager
        after its handle has been moved, so the call fails silently and the
        product name is not refreshed.
      * **Serial is known but auto-reconnect is disabled:** nothing is done,
        and the temporary manager, with its claimed handle, is leaked (a
        bug).
4. `notify_one(g_enumFinishEvent)`, then sleep 500 ms.

Supporting behaviour:
* `getOnlineDeviceSerial(vector<string>&)` (0x1602c) and
  `getOnlineDeviceSerialEx(SHNDEVINFO&)` (0x16854) wait for the next
  enumeration pass, up to 10 × 200 ms. They then list the online nodes. The
  vector must be empty on input.
* `SHNDEVINFO` layout (0x1C24 = 7204 bytes, zeroed first):

  | Offset | Content |
  |---|---|
  | 0 | int32 count |
  | 4 | char serial[60][60] |
  | 0xE14 | char productName[60][60] |

  At most 59 entries are filled, because the code checks `count < 59`.
* `_DEVICENODE` (0x88 bytes, ctor 0x18bbc):

  | Offset | Field |
  |---|---|
  | +0 | manager* |
  | +8 | node mutex |
  | +0x48 | pid |
  | +0x4c | port (−1 initially) |
  | +0x50 | serial string |
  | +0x68 | product name string |
  | +0x80 | attach refcount |
  | +0x84 | online flag |

  Every C API call validates the handle by map lookup (`isDevNodeValid`,
  0x158b4) and requires `online == 1`, except `driverStreamSwitch`, which
  checks only the map lookup.
* `enableAutoReconnect(bool)` (0x1886c) sets the global flag.
  `deleteSystem()` (0x15c54) stops the monitor and destroys every node,
  which closes the devices.
* `reAttach()` returns 0xFF and `isOnLine()` returns 0. Both are stubs.

---

## 6. Exported C API (namespace `CommunicateLayer`)

`h` is the opaque `void*` (really a `_DEVICENODE*`). Status codes are those in
section 0. "Plugin?" marks the functions that einscan10 imports. **[H]**

| Function (addr) | Args | Behaviour | Plugin? |
|---|---|---|---|
| `initializeSystem(const _GUID&)` 0x15988 | GUID is ignored | starts the monitor thread | yes |
| `deleteSystem()` 0x15c54 | — | stops the monitor, frees all nodes | yes |
| `getOnlineDeviceSerial(vector<string>&)` 0x1602c | empty vector in | serials of online nodes; 2 if none, 0xFF if timeout or no monitor | yes |
| `getOnlineDeviceSerialEx(SHNDEVINFO&)` 0x16854 | struct out | serials and product names | yes |
| `attachDevice(void*& h, std::string serial)` 0x16e5c | `h` must be null | map lookup, `h = node`, refcount++; 4 if unknown or offline | yes |
| `detachDevice(void*& h)` 0x17284 | | refcount--, `h = null` (does not close USB) | yes |
| `getMaxCommandSize(h, uint&)` 0x17414 | | broken, returns 0xFF | no |
| `commandChannelLock(h, bool)` 0x174a4 | | command mutex | yes |
| `dataStreamlock(h, bool)` 0x1754c | | data mutex | no |
| `sendCommand(h, u8*, uint len)` 0x175f4 | | interrupt OUT 0x01, 1 s; always returns 0 | yes |
| `receiveCommandReply(h, u8*, uint& len)` 0x176a4 | | interrupt IN 0x81, 1 s | yes |
| `sendCommandAndWaitReply(h, u8*, uint& len)` 0x17778 | in/out length | locked send and receive, same buffer | yes |
| `sendCommandAndWaitReplyEx(h, u8*, uint& len, uint replyMax)` 0x178bc | | as above, separate reply capacity | no |
| `sendBulkData(h, u8*, uint len)` 0x17a0c | | 1 KiB chunks to 0x02 | yes |
| `receiveBulkReply(h, u8*, uint& len)` 0x17ac0 | | 1 KiB chunks from 0x82 | yes |
| `sendDataAndWaitReply(h, u8*, uint& len, uint replyLen)` 0x17b94 | | locked bulk exchange | yes |
| `receiveImageStream(h, u8*, uint& len)` 0x17d3c | `len ≥ unitSize` | one unit (section 3.5) | yes |
| `driverStreamSwitch(h, bool)` 0x17e14 | | start or stop EP 0x83 capture | yes |
| `setDriverAttemptPkgSize(h, uint)` 0x1803c | | stream must be stopped | yes |
| `setDriverPkgCountPerUnit(h, uint)` 0x180e4 | | stream must be stopped | yes |
| `setDriverCacheUnitCount(h, uint)` 0x1818c | | stream must be stopped | yes |
| `setMaxGrabWaitTime(h, uint µs)` 0x18234 | | any time | yes |
| `getImageDataUnitSize(h, uint&)` 0x182dc | | returns cacheUnitCount (bug) | no |
| `discardDriverFrameCaches(h)` 0x18384 | | flushes the ready FIFO | no |
| `resetDeviceInDriver(h)` 0x18424 | | no-op, returns 0xFF | yes |
| `resetCommandPipe(h, int)` 0x184c4 | int ignored | clear_halt 0x01, 0x81 (under the command lock) | yes |
| `resetBulkPipe(h, int)` 0x185d8 | int ignored | clear_halt 0x02, 0x82 (under the data lock) | yes |
| `getDevicePipeAddressList(h, u8* list, uint& n)` 0x186e8 | `list == null` → count | `01 81 02 82 83` | no |
| `resetPipe(h, u8 ep)` 0x187c4 | | clear_halt(ep) | no |
| `enableAutoReconnect(bool)` 0x1886c | | global flag, default true | no |

The plugin's connect sequence (plugin:0x6b90), for context:
1. `attachDevice`.
2. Trigger-period and trigger-switch commands.
3. `setDriverAttemptPkgSize(0xA400)`, `setDriverPkgCountPerUnit(32)`,
   `setDriverCacheUnitCount(100)`, `setMaxGrabWaitTime(100000)`.
4. `driverStreamSwitch(true)`.
5. Vendor, product and parameter queries over the command channel (group
   0x10).

Its disconnect sequence (plugin:0xaab8):
1. Trigger off.
2. Sleep 100 ms.
3. `driverStreamSwitch(false)`.
4. `resetDeviceInDriver`.
5. `detachDevice`.

---

## 7. Recommendations for the open-source driver

* Claim interface 0 and skip set_configuration. Drain 0x81 and 0x82 with short
  timeouts, then clear halt on 0x81 and 0x82 before the first command.
* Read the serial with `FF 00 00 04` + zeros (30 bytes), then take reply bytes
  9..16 as hex. Read the product name with `FE 00 00 01` (100-byte buffer),
  then take the BE32 length at bytes 5..8 and the string at byte 9.
* Masking can be omitted on transmit. On receive, apply the XOR decode keyed
  on the high nibble of byte1, unless byte1 == 0x02. It is a no-op for
  unmasked replies.
* Stream: keep about 32 async bulk-IN transfers of 41 984 bytes in flight on
  0x83 with a 500 ms timeout, and resubmit on timeout. Treat packet byte 8 == 1
  as end of frame. Strip the 32-byte header and reassemble W×H bytes, using
  the plugin's resync rules. There is no need to reproduce the unit/slot
  container; it is an internal artifact of this layer.
* For bulk sends, pad the length to a multiple of 1024.

---

## 8. Open questions

1. **Transfer types of EP 0x01/0x81.** The code uses
   `libusb_interrupt_transfer`, but libusb would issue the same call against a
   bulk endpoint. The descriptor must be checked (the firmware notes say
   interrupt, 1024 B).
2. **PID.** The layer accepts any PID under VID 0x3267, so it cannot settle
   0x0002 vs 0x0003. Both are fine for this layer.
3. **Request length field.** Is it bytes 4..7, as the plugin's
   `00 00 00 01 <arg>` suggests? Replies clearly use a status byte at 4 and
   the length at 5..8. The asymmetric layout is inferred **[M/L]**.
4. **Meaning of reply byte1 == 0x02**, the value that skips decoding.
   Possibly an "unmasked/raw" marker, or an error class. **[L]**
5. **Stream header bytes 0–7, 9–11 and 24–31** are unused by the host.
   A magic value, packet sequence, image size or exposure id may live there.
   Also: are the two 3-bit fields in byte 2 the camera index and the image
   type? Are bytes 12–15 a frame counter, and bytes 16–23 a device timestamp
   (in what unit)?
6. Does the firmware ever send masked replies, or ever require masked
   requests? Nothing in EXStar enables masking. The code path may only be
   used by other products or older firmware.
7. **Interrupt IN reply sizes.** The C API requests exactly the request length
   (16 or 50 bytes). The device must be sending short replies. The exact
   reply length per opcode should be confirmed from the firmware.
8. **Bulk-channel payload format** for firmware/calibration transfers. The
   header convention (echo bytes 0/2/3, status at byte 4) is inferred from the
   plugin's validation only.
9. **Disconnect detection** compares PID and port lists. Behaviour with two
   identical scanners, or after a re-plug to a different port while one is
   still attached, is quirky and was not analysed further.
