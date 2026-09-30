"""Scenarios: scripted and randomised sessions that drive every path of the firmware. Each is a
function(Device) -> None; run it on two images and compare (diff.py)."""
import random

from .device import (Device, packet, default_flash, SLOT_A_APP, SLOT_B_APP, BOOT_RECORD, EV_CONNECT, EV_DISCONNECT, EV_SUSPEND, EV_RESUME, EV_RESET, EV_SETCONF,
                     EV_SPEED, EV_SET_SEL, EV_VBUS_VALID, EV_VBUS_REMOVED, EV_USB3_LNKFAIL, USER_AREA)

GROUPS = (0x00, 0x10)
PAD = (10, 16, 50, 64, 100)                   # the host's request buffer sizes
DESTRUCTIVE = {(0xCC, 0x00), (0x00, 0x08)}     # erase + reset, reboot: scenarios of their own


def up(d):
    return d.boot().enumerate()


# ---------------------------------------------------------------- command channel
def payloads(rnd):
    yield b""
    for n in range(1, 6):
        yield bytes([0] * n)
        yield bytes([0xff] * n)
        yield bytes(range(1, n + 1))
    for mask in (1, 2, 3, 4, 7):
        yield bytes([mask, 0x12, 0x34, 0x56, 0x78])
        yield bytes([mask, 0, 0, 0x11, 0x30])
        yield bytes([mask, 0x03, 0x20])
    for _ in range(3):
        yield bytes(rnd.getrandbits(8) for _ in range(rnd.randrange(1, 9)))


def command_sweep(group):
    def scenario(d):
        rnd = random.Random("sweep/%d/%d" % (group, d.seed))
        up(d)
        seq = 0
        for op in range(256):
            if (group, op) in DESTRUCTIVE:
                continue
            for p in payloads(rnd):
                for declared in {len(p), max(len(p) - 1, 0)}:
                    d.command(packet(seq, group, op, p, declared), pad_to=rnd.choice(PAD))
                    seq = (seq + 1) % 254
            if d.halted:
                return
    scenario.__name__ = "commands-%02x" % group
    return scenario


def command_values(d):
    """The documented settings over their ranges and beyond (as the host would send them)."""
    up(d)
    seq = 0

    def cmd(g, o, p, pad=50):
        nonlocal seq
        d.command(packet(seq, g, o, p), pad_to=pad)
        seq = (seq + 1) % 254
    for v in (0, 1, 99, 100, 101, 200, 255):
        cmd(0x10, 0x68, [v]); cmd(0x10, 0x67, [])
    for route in (0, 1, 2, 5):
        for lum in (0, 1, 6000, 9000, 16383, 16384, 65535):
            cmd(0x10, 0x70, [route, lum >> 8, lum & 0xff]); cmd(0x10, 0x6f, [route])
    for per in (0, 999, 1000, 68000, 200000, 1000000, 1000001, 0xffffffff):
        cmd(0x10, 0x49, per.to_bytes(4, "big")); cmd(0x10, 0x48, [])
    for mask in (1, 2, 3, 4, 8):
        for e in (0, 1, 4400, 32767, 32768, 50000, 50001, 0x7fffffff):
            cmd(0x10, 0x23, bytes([mask]) + e.to_bytes(4, "big"), 16); cmd(0x10, 0x22, [mask], 16)
        for g in (0, 1, 16, 100, 120, 400, 800, 801, 2047, 65535):
            cmd(0x10, 0x27, [mask, g >> 8, g & 0xff], 16); cmd(0x10, 0x26, [mask], 16)
    for sw in (0x00, 0x03, 0x11, 0x13, 0xff):
        cmd(0x10, 0x41, [sw], 10); cmd(0x10, 0x40, [], 10)
    for dist in range(5):
        for state in (0, 1, 2):
            cmd(0x10, 0x62, [dist, state], 16)
    for mode in (0, 1, 2, 3):
        cmd(0x10, 0x4f, [mode]); cmd(0x10, 0x4e, [])
        d.command(packet(seq, 0x10, 0x3f, [0, mode], 1), pad_to=16); seq += 1     # mode: payload byte 1
        cmd(0x10, 0x3e, [])
        d.fpga_reg(6, 0x1f40 | mode); cmd(0x10, 0x3e, [])
    for b2 in (0, 1, 2, 3):                                        # 10/5D reads payload byte 2
        d.command(packet(seq, 0x10, 0x5d, [1, 0, b2], 1), pad_to=16); seq += 1
    for op in range(1, 256):                                      # group 0xCC (0xCC00 has its own scenario)
        cmd(0xCC, op, [], 16)


def sequence_numbers(d):
    """Repeated sequence numbers (retransmission rule), the transport's 0xFE/0xFF, and wrap-around."""
    up(d)
    for seq in (5, 5, 5, 6, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0, 0, 253, 253):
        d.command(packet(seq, 0x10, 0x51), pad_to=16, wait_ms=5)
    for seq in (7, 7, 7):
        d.bulk(packet(seq, 0x10, 0x57, b"\x00\x01"), pad_to=5120)


def device_state(d):
    """00/07 over FPGA state values, the button codes and the EP 0x83 restart hook."""
    up(d)
    rnd = random.Random("state/%d" % d.seed)
    for i, v in enumerate([0, 0xffffffff, 1 << 17, 0x155 << 5, 0x2aa << 5] + [rnd.getrandbits(32) for _ in range(20)]):
        d.fpga_reg(9, v).command(packet(i, 0x00, 0x07), pad_to=50, wait_ms=5)
    d.setup(0x02, 0x01, 0, 0x83)                                   # CLEAR_FEATURE(ENDPOINT_HALT) on 0x83
    d.fpga_reg(9, 0).command(packet(40, 0x00, 0x07), pad_to=50)   # bit 17 clear: no restart
    d.fpga_reg(9, 1 << 17).command(packet(41, 0x00, 0x07), pad_to=50, wait_ms=1500)   # restart
    d.command(packet(42, 0x00, 0x05), pad_to=64)


def reboot(d):
    up(d)
    d.command(packet(1, 0x00, 0x08), pad_to=10, wait_ms=200)


def erase_app_header(d):
    up(d)
    d.command(packet(1, 0xCC, 0x00), pad_to=16, wait_ms=200)


def cancel_reboot(d):
    up(d)
    d.command(packet(1, 0x00, 0x0D), pad_to=16, wait_ms=5)


# ---------------------------------------------------------------- bulk channel
def page(n_data, rnd, first=0):
    data = bytes(rnd.getrandbits(8) for _ in range(n_data))
    return data, sum(data) & 0xff


def update_packets(size, rnd, seq0=10, corrupt_page=None, big_page=None):
    """00/06 header + pages as EXStar sends them (payload: 1 unused byte, data, check byte)."""
    out = [packet(seq0, 0x00, 0x06, b"\x00" + size.to_bytes(4, "big"))]
    left, i = size, 0
    while left > 0:
        n = min(4096, left)
        data, chk = page(n, rnd)
        if i == corrupt_page:
            chk ^= 0x5a
        body = b"\x00" + data + bytes([chk])
        length = len(data) + 2 if i != big_page else 4096 + 3
        out.append(packet(seq0 + 1 + i, 0x00, 0x06, body, length))
        left -= n
        i += 1
    return out


def firmware_update(kind):
    def scenario(d):
        rnd = random.Random("update/%s/%d" % (kind, d.seed))
        up(d)
        size = rnd.choice((4096 * 3, 4096 * 2 + 700, 5000))
        pk = update_packets(size, rnd, corrupt_page=1 if kind == "bad-check" else None,
                            big_page=1 if kind == "big-page" else None)
        if kind == "timeout-retry":
            for p in pk[:2]:
                d.bulk(p, pad_to=5120)
            d.run(2500)                                            # the 2 s page timer expires
            pk = update_packets(size, rnd, seq0=40)
        if kind == "cancel-retry":
            for p in pk[:2]:
                d.bulk(p, pad_to=5120)
            d.command(packet(1, 0x00, 0x0D), pad_to=16)
            pk = update_packets(size, rnd, seq0=40)
        for p in pk:
            d.bulk(p, pad_to=5120)
        d.run(1500)                                                # watchdog thread resets after an update
    scenario.__name__ = "update-" + kind
    return scenario


def user_pages(d):
    rnd = random.Random("pages/%d" % d.seed)
    up(d)
    seq = 0
    for pg in (0, 1, 255, 256, 0xffff, 7):
        data = bytes(rnd.getrandbits(8) for _ in range(4096))
        d.bulk(packet(seq, 0x10, 0x58, pg.to_bytes(2, "big") + data), pad_to=5120); seq += 1
        d.bulk(packet(seq, 0x10, 0x57, pg.to_bytes(2, "big")), pad_to=5120); seq += 1
    d.bulk(packet(seq, 0x10, 0x58, b"", 0), pad_to=1024); seq += 1                  # zero length
    d.bulk(packet(seq, 0x10, 0x57, b"\x00\x01", 3), pad_to=1024); seq += 1          # wrong length
    d.bulk(packet(seq, 0x10, 0x99, b""), pad_to=1024); seq += 1                     # unknown key
    d.bulk(packet(seq, 0x00, 0x06, b"\x00\x00\x00\x10", 4), pad_to=1024)           # wrong update length


# ---------------------------------------------------------------- control requests and events
def setup_requests(d):
    up(d)
    rnd = random.Random("setup/%d" % d.seed)
    for ep in (0x01, 0x81, 0x02, 0x82, 0x83, 0x84, 0x00):
        d.setup(0x02, 0x01, 0, ep); d.run(5)                        # CLEAR_FEATURE(ENDPOINT_HALT)
        d.setup(0x02, 0x03, 0, ep); d.run(5)                        # SET_FEATURE
    for req in (1, 2, 3, 9, 10, 11):                                 # the HID-numbered class requests
        for value in (0x0000, 0x0002, 0x0007, 0x0302, 0x0507, 0x2a07):
            for idx in (0, 2):
                d.setup(0x20, req, value, idx, 64, bytes([1, 0x42] + [0] * 62))
                d.setup(0x20, 2, value, idx, 1)                     # GET_IDLE after each
    for bm in (0x00, 0x01, 0x02, 0x20, 0x21, 0x22, 0x40, 0x80, 0xA0, 0xA1, 0xC0):
        for req in list(range(0, 13)) + [48, 49]:
            for idx in (0, 1, 2):
                data = bytes([rnd.getrandbits(8), rnd.choice((0xAA, 0xBB, 0x00, 0x42))] + [rnd.getrandbits(8) for _ in range(62)])
                d.setup(bm, req, rnd.choice((0x0000, 0x0002, 0x0007, 0x0302, 0x0507)), idx, 64, data)
    d.run(50)


def usb_events(d):
    d.boot()
    for ev, data in ((EV_VBUS_VALID, 0), (EV_CONNECT, 2), (EV_RESET, 0), (EV_SPEED, 2), (EV_SETCONF, 1),
                     (EV_SET_SEL, 0), (EV_SUSPEND, 0), (EV_RESUME, 0), (EV_SETCONF, 1), (EV_RESET, 0),
                     (EV_SETCONF, 1), (EV_DISCONNECT, 0), (EV_CONNECT, 3), (EV_SETCONF, 1), (EV_USB3_LNKFAIL, 0),
                     (EV_SETCONF, 1), (99, 7)):
        d.event(ev, data).run(20)
        d.command(packet(ev & 0xff, 0x10, 0x51), pad_to=16, wait_ms=5)
    lpm = d.m.callbacks.get("lpm")
    if lpm:
        for mode in range(4):
            d.m.call(lpm, (mode,), label="lpm")
    for evt in (1, 2, 4, 8, 16, 32, 64, 128):
        d.ep_event(evt, 2, 1)
    d.run(1200)
    d.ep_event(256, 3, 3)                                           # SS endpoint reset: watchdog reboots
    d.run(1500)


def unplugged(d):
    d.boot().event(EV_VBUS_VALID).run(12000)                        # powered, never configured: reset


def vbus_removed(d):
    up(d)
    d.event(EV_VBUS_REMOVED).run(100)


# ---------------------------------------------------------------- faults
def i2c_faults(d):
    up(d)
    seq = 0
    for target in (1, 2, 4, 5, 8):
        for write in (True, False):
            for g, o, p in ((0x10, 0x27, [1 << (target >> 1) if target < 5 else 1, 0, 120]), (0x10, 0x26, [1]),
                            (0x10, 0x50, []), (0x10, 0x68, [50]), (0x10, 0x67, []), (0x10, 0x72, [3]),
                            (0x10, 0x73, [3]), (0x10, 0x75, [3]), (0x10, 0x23, [1, 0, 0, 1, 0]),
                            (0x10, 0x71, []), (0x10, 0x22, [1]), (0x00, 0x07, [])):
                d.i2c_fail_next(target, write).command(packet(seq, g, o, p), pad_to=50, wait_ms=10)
                seq = (seq + 1) % 254
    d.m.i2c.fail_rate = 0.3
    for k in range(200):
        g, o = random.Random(k).choice(((0x10, 0x27), (0x10, 0x26), (0x10, 0x50), (0x10, 0x68), (0x00, 0x07)))
        d.command(packet(k % 254, g, o, [1, 0, k & 0xff]), pad_to=50, wait_ms=10)


def boot_faults(d):
    d.m.i2c.fail_rate = 0.2
    up(d)
    d.run(3000)


# ---------------------------------------------------------------- rarely reached paths
def eeprom_requests(d):
    up(d)
    for second in (0xAA, 0xBB, 0x00):
        d.setup(0x20, 0x09, 0x0200, 2, 64, bytes([0, second] + list(range(62))))
    d.setup(0x20, 0x01, 0x0000, 2, 64)                               # GET_REPORT after them
    d.run(20)


def gpio_interrupts(d):
    up(d)
    cb = d.m.callbacks.get("gpio")
    for pin in (36, 52, 37, 0):
        for level in (False, True):
            d.m.gpio.inputs[pin] = level
            if cb:
                d.m.log("host", "gpio-irq", pin, int(level))
                d.m.call(cb, (pin,), label="gpio-irq")
    d.run(10)


def watchdog_timer(d):
    """emc_wdg_timer is never started by the firmware; its callback and the watchdog's timeout branch
    are compared by calling the callback directly."""
    up(d)
    for _ in range(3):
        d.m.call("emc_wdg_timer_cb", (0,), label="timer:emc_wdg_timer")
        d.run(1100)


def boot_record(new_app, old_app, boot_new):
    def scenario(d):
        flash = [x for x in default_flash(d.seed) if x[0] != BOOT_RECORD]
        rec = new_app.to_bytes(4, "little") + old_app.to_bytes(4, "little") + bytes([boot_new])
        flash.append((BOOT_RECORD, rec.ljust(256, b"\xff")))
        d.m.flash.__init__(flash)
        rnd = random.Random("bootrec/%d" % d.seed)
        up(d)
        for p in update_packets(4096 + 10, rnd):
            d.bulk(p, pad_to=5120)
        d.run(1500)
    scenario.__name__ = "bootrecord-%x-%x-%d" % (new_app, old_app, boot_new)
    return scenario


def boot_records():
    return [boot_record(n, o, b) for n, o, b in ((SLOT_A_APP, SLOT_B_APP, 1), (SLOT_A_APP, SLOT_B_APP, 0),
                                                  (SLOT_B_APP, SLOT_A_APP, 1), (SLOT_B_APP, SLOT_A_APP, 0),
                                                  (0xffffffff, 0xffffffff, 0xff), (0x123456, 0, 1))]


# ---------------------------------------------------------------- SDK failures
# SDK calls whose failure the firmware can observe (returns a status or a pointer)
FAILABLE = ("CyU3PDeviceInit", "CyU3PDeviceConfigureIOMatrix", "CyU3PMemAlloc", "_txe_thread_create",
            "_txe_semaphore_create", "_txe_semaphore_get", "CyU3PSpiInit", "CyU3PSpiSetConfig",
            "CyU3PSpiTransmitWords", "CyU3PSpiReceiveWords", "CyU3PGpioInit", "CyU3PGpioSetSimpleConfig",
            "CyU3PDeviceGpioOverride", "CyU3PGpioSimpleSetValue", "CyU3PUartInit", "CyU3PUartSetConfig",
            "CyU3PUartTxSetBlockXfer", "CyU3PDebugInit", "CyU3PI2cInit", "CyU3PI2cSetConfig",
            "CyU3PI2cTransmitBytes", "CyU3PI2cReceiveBytes", "CyU3PPibInit", "CyU3PUsbStart", "CyU3PUsbSetDesc",
            "CyU3PConnectState", "CyU3PGpifLoad", "CyU3PGpifSMStart", "CyU3PSetEpConfig", "CyU3PDmaChannelCreate",
            "CyU3PDmaChannelSetXfer", "CyU3PDmaChannelGetBuffer", "CyU3PDmaChannelDiscardBuffer",
            "CyU3PDmaChannelCommitBuffer", "CyU3PDmaChannelSendData", "CyU3PUsbGetEP0Data", "CyU3PUsbSendEP0Data",
            "CyU3PGpioGetValue", "CyU3PDmaChannelReset", "CyU3PGetConnectState")


def sdk_failure(name, count):
    def scenario(d):
        d.m.forced[name] = [0 if name == "CyU3PMemAlloc" else 0x40 + (k % 3) for k in range(count)]
        d.boot().enumerate()
        seq = 0
        for g, o, p in ((0x00, 0x00, b""), (0x10, 0x27, b"\x01\x00\x78"), (0x00, 0x07, b"")):
            d.command(packet(seq, g, o, p), pad_to=50); seq += 1
        d.bulk(packet(seq, 0x10, 0x57, b"\x00\x02"), pad_to=5120); seq += 1
        d.bulk(packet(seq, 0x10, 0x58, b"\x00\x02" + bytes(4096)), pad_to=5120); seq += 1
        d.setup(0x21, 0x09, 0x0200, 2, 64, bytes([0, 0x42] + [0] * 62))
        d.setup(0x21, 0x01, 0x0002, 2, 8)
        d.setup(0x02, 0x01, 0, 0x81)
        d.command(packet(seq, 0x00, 0x05), pad_to=64)
        d.run(2000)
    scenario.__name__ = "fail-%s-x%d" % (name, count)
    return scenario


def sdk_failures():
    return [sdk_failure(n, c) for n in FAILABLE for c in (1, 1000)]


# ---------------------------------------------------------------- random sessions
def fuzz(seed):
    def scenario(d):
        rnd = random.Random("fuzz/%d" % seed)
        up(d)
        for step in range(400):
            k = rnd.random()
            if k < 0.55:
                g = rnd.choice((0x00, 0x10, 0x10, 0x10))
                o = rnd.randrange(256) if rnd.random() < 0.3 else rnd.choice(
                    (0x00, 0x01, 0x04, 0x05, 0x07, 0x16, 0x20, 0x22, 0x23, 0x26, 0x27, 0x40, 0x41, 0x48, 0x49,
                     0x4e, 0x4f, 0x50, 0x51, 0x5d, 0x62, 0x67, 0x68, 0x6f, 0x70, 0x71, 0x72, 0x7b))
                if (g, o) in DESTRUCTIVE:
                    continue
                p = bytes(rnd.getrandbits(8) for _ in range(rnd.choice((0, 1, 2, 3, 4, 5, 8))))
                declared = len(p) if rnd.random() < 0.8 else rnd.randrange(8)
                d.command(packet(rnd.randrange(256), g, o, p, declared), pad_to=rnd.choice(PAD),
                          wait_ms=rnd.choice((1, 5, 20)))
            elif k < 0.65:
                pg = rnd.choice((0, 1, 2, 255, 256, rnd.randrange(65536)))
                if rnd.random() < 0.5:
                    d.bulk(packet(rnd.randrange(256), 0x10, 0x57, pg.to_bytes(2, "big")), pad_to=5120)
                else:
                    data = bytes(rnd.getrandbits(8) for _ in range(rnd.choice((0, 100, 4096))))
                    d.bulk(packet(rnd.randrange(256), 0x10, 0x58, pg.to_bytes(2, "big") + data), pad_to=5120)
            elif k < 0.72:
                d.fpga_reg(rnd.randrange(20), rnd.getrandbits(32))
            elif k < 0.78:
                d.i2c_fail_next(rnd.choice((1, 2, 4, 5, 7, 8, None)), rnd.choice((True, False, None)))
            elif k < 0.84:
                bm = rnd.choice((0x00, 0x02, 0x20, 0x21, 0xA1, 0x40))
                data = bytes(rnd.getrandbits(8) for _ in range(64))
                d.setup(bm, rnd.randrange(13), rnd.getrandbits(16), rnd.choice((0, 1, 2, 0x81, 0x82, 0x83)), 64, data)
            elif k < 0.86:
                d.event(rnd.choice((EV_SUSPEND, EV_RESUME, EV_SET_SEL, EV_SPEED, EV_SETCONF, EV_RESET)), 2)
            else:
                d.run(rnd.choice((1, 10, 100, 700)))
            if d.halted:
                return
    scenario.__name__ = "fuzz-%d" % seed
    return scenario


def interleaved_channels(d):
    """Command and bulk traffic interleaved, updates crossed with normal commands, mid-update commands."""
    rnd = random.Random("interleave/%d" % d.seed)
    up(d)
    seq = 0
    upk = update_packets(4096 * 2 + 100, rnd)
    d.bulk(upk[0], pad_to=5120)                       # start an update
    for i, pk in enumerate(upk[1:], 1):
        d.command(packet(seq, 0x10, 0x51), pad_to=16); seq += 1      # commands mid-update (become pages)
        d.command(packet(seq, 0x00, 0x07), pad_to=50); seq += 1
        d.bulk(pk, pad_to=5120)
        d.bulk(packet(seq, 0x10, 0x57, b"\x00\x01"), pad_to=5120); seq += 1   # a read mid-update
    d.run(1500)
    for g, o, p in ((0x10, 0x27, [1, 0, 120]), (0x10, 0x22, [1]), (0x10, 0x70, [0, 0x10, 0x00])):
        d.command(packet(seq, g, o, p), pad_to=50); seq += 1
        d.bulk(packet(seq, 0x10, 0x58, b"\x00\x02" + bytes(4096)), pad_to=5120); seq += 1


def reconnect_churn(d):
    """Repeated connect/disconnect/reset/reconfigure with commands in between (the SDK's event storm)."""
    up(d)
    rnd = random.Random("churn/%d" % d.seed)
    evs = [EV_SUSPEND, EV_RESUME, EV_RESET, EV_SETCONF, EV_DISCONNECT, EV_CONNECT, EV_SETCONF, EV_SET_SEL,
           EV_VBUS_REMOVED, EV_VBUS_VALID, EV_CONNECT, EV_RESET, EV_SPEED, EV_SETCONF]
    for k in range(60):
        d.event(rnd.choice(evs), rnd.choice((0, 2, 3)))
        if rnd.random() < 0.5:
            d.command(packet(k % 254, 0x10, 0x51), pad_to=16, wait_ms=2)
        if d.halted:
            return
    d.run(200)


def all_keys_all_lengths(group):
    """Every opcode 0x00..0xFF x declared length 0..6 x a fixed payload, both channels' buffer sizes."""
    def scenario(d):
        up(d)
        seq = 0
        for op in range(256):
            if (group, op) in DESTRUCTIVE:
                continue
            for dl in range(7):
                d.command(packet(seq, group, op, bytes(range(dl)), dl), pad_to=64, wait_ms=2)
                seq = (seq + 1) % 254
            if d.halted:
                return
    scenario.__name__ = "allkeys-%02x" % group
    return scenario


def boundary_values(d):
    """Every documented setter at its exact min/max and +-1, plus 0/0xFF/0xFFFF/0xFFFFFFFF edges."""
    up(d)
    seq = 0
    def cmd(g, o, p, pad=50):
        nonlocal seq
        d.command(packet(seq, g, o, p), pad_to=pad); seq = (seq + 1) % 254
    edges8 = (0, 1, 0x7f, 0x80, 0xfe, 0xff)
    edges16 = (0, 1, 0x7fff, 0x8000, 0xfffe, 0xffff)
    edges32 = (0, 1, 0x7fffffff, 0x80000000, 0xfffffffe, 0xffffffff)
    for v in edges8:
        cmd(0x10, 0x68, [v]); cmd(0x10, 0x2e, [v], 16); cmd(0x10, 0x62, [v, v & 1], 16)
    for route in edges8:
        for lum in edges16:
            cmd(0x10, 0x70, [route, lum >> 8, lum & 0xff])
    for m in (1, 2, 4):
        for e in edges32:
            cmd(0x10, 0x23, bytes([m]) + e.to_bytes(4, "big"), 16)
        for g in edges16:
            cmd(0x10, 0x27, [m, g >> 8, g & 0xff], 16)
    for per in edges32:
        cmd(0x10, 0x49, per.to_bytes(4, "big"))
    for sw in edges8:
        cmd(0x10, 0x41, [sw], 10)
    for b in edges8:
        cmd(0x10, 0x72, [b]); cmd(0x10, 0x73, [b]); cmd(0x10, 0x75, [b])


def all_scenarios(fuzz_seeds=range(8)):
    s = [command_sweep(0x00), command_sweep(0x10), command_values, sequence_numbers, device_state, reboot,
         erase_app_header, cancel_reboot, user_pages, setup_requests, usb_events, unplugged, vbus_removed,
         i2c_faults, boot_faults]
    s += [firmware_update(k) for k in ("good", "bad-check", "big-page", "timeout-retry", "cancel-retry")]
    s += [fuzz(k) for k in fuzz_seeds]
    s += [eeprom_requests, gpio_interrupts, watchdog_timer, interleaved_channels, reconnect_churn,
          boundary_values, all_keys_all_lengths(0x00), all_keys_all_lengths(0x10)] + boot_records()
    s += sdk_failures()
    return s
