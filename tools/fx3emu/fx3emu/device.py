"""The host's view of an emulated scanner: boot it, enumerate it, send commands and bulk packets,
issue control requests, and poke its hardware (FPGA registers, I2C faults, button state)."""
import random

from . import models
from .machine import Machine

# USB events (CyU3PUsbEventType_t)
EV_CONNECT, EV_DISCONNECT, EV_SUSPEND, EV_RESUME, EV_RESET, EV_SETCONF, EV_SPEED = 0, 1, 2, 3, 4, 5, 6
EV_SET_SEL, EV_VBUS_VALID, EV_VBUS_REMOVED, EV_USB3_LNKFAIL = 8, 11, 12, 20

# flash layout (docs/firmware.md section 3)
SLOT_A_APP, SLOT_B_APP, SLOT_A_FPGA, SLOT_B_FPGA = 0x040000, 0x1C0000, 0x080000, 0x200000
BOOT_RECORD, USER_AREA = 0x03F000, 0x400000


def default_flash(seed, boot_new=1, new_app=SLOT_A_APP, old_app=SLOT_B_APP):
    r = random.Random("flash/%d" % seed)
    rb = lambda n: bytes(r.getrandbits(8) for _ in range(n))   # noqa: E731
    return [(BOOT_RECORD, models.boot_record(new_app, old_app, boot_new)),
            (SLOT_A_FPGA, rb(0x180000 // 16) * 16), (SLOT_B_FPGA, rb(0x180000 // 16) * 16),
            (USER_AREA, rb(0x10000) * 16)]


class Device:
    def __init__(self, image, seed=0, flash=None, label=None):
        self.m = Machine(image, seed=seed, flash=default_flash(seed) if flash is None else flash, label=label)
        self.seed = seed

    @property
    def trace(self):
        return self.m.trace

    @property
    def halted(self):
        return self.m.halted

    # ---------------------------------------------------------------- life cycle
    def boot(self, settle_ms=4000):
        m = self.m
        m.call("main")
        if m.halted == "kernel":
            m.halted = None
        if not m.halted:
            m.call("CyFxApplicationDefine")
        return self.run(settle_ms)

    def run(self, ms):
        if not self.m.halted:
            self.m.run(ms)
        return self

    def event(self, ev, data=0):
        cb = self.m.callbacks.get("event")
        if cb and not self.m.halted:
            self.m.log("host", "usb-event", ev, data)
            self.m.call(cb, (ev, data), label="usb-event")
        return self

    def ep_event(self, evtype, speed, ep):
        cb = self.m.callbacks.get("ep_event")
        if cb and not self.m.halted:
            self.m.log("host", "ep-event", evtype, speed, ep)
            self.m.call(cb, (evtype, speed, ep), label="ep-event")
        return self

    def enumerate(self, speed=2):
        """VBUS, reset, speed and SET_CONFIGURATION, as the SDK reports them to the application."""
        for ev, d in ((EV_VBUS_VALID, 0), (EV_CONNECT, speed), (EV_RESET, 0), (EV_SPEED, speed), (EV_SETCONF, 1)):
            self.event(ev, d).run(5)
        return self

    def setup(self, bm, req, value=0, index=0, length=0, data=None):
        """A control request; `data` is the host's data stage (for host-to-device requests)."""
        cb = self.m.callbacks.get("setup")
        if not cb or self.m.halted:
            return None
        if data is not None:
            self.m.ep0_out.append(bytes(data))
        self.m.log("host", "setup", "%02x %02x %04x %04x %04x" % (bm, req, value, index, length))
        r = self.m.call(cb, (bm | req << 8 | value << 16, index | length << 16), label="setup")
        self.m.log("host", "setup-handled", r)
        self.m.ep0_out.clear()
        return r

    # ---------------------------------------------------------------- channels
    def _send(self, channel, packet, wait_ms, timeout_ms=1000):
        """Queue a transfer. With wait_ms None, wait for a reply like the host does (up to timeout_ms);
        otherwise just let wait_ms pass."""
        n0 = len(self.m.replies)
        self.m.queues.setdefault(channel, []).append(bytes(packet))
        self.m.log("host", channel, len(packet), bytes(packet)[:16].hex())
        if wait_ms is not None:
            self.run(wait_ms)
        else:
            waited = 0
            while waited < timeout_ms and len(self.m.replies) == n0 and not self.m.halted:
                self.run(1)
                waited += 1
        return [r for r in self.m.replies[n0:]]

    def command(self, packet, pad_to=64, wait_ms=None):
        """One command-channel transfer (EP 0x01); returns the replies seen (EP 0x81)."""
        p = bytes(packet)
        return self._send("ch_ctrl_out", p + b"\0" * max(0, pad_to - len(p)), wait_ms)

    def bulk(self, packet, pad_to=1024, wait_ms=None):
        """One bulk-channel transfer (EP 0x02, padded to whole 1 KiB packets); returns replies (EP 0x82)."""
        p = bytes(packet)
        n = max(pad_to, (len(p) + 1023) // 1024 * 1024)
        return self._send("ch_bulk_out", p + b"\0" * (n - len(p)), wait_ms)

    # ---------------------------------------------------------------- hardware
    def fpga_reg(self, reg, value):
        self.m.i2c.fpga.regs[reg] = value.to_bytes(4, "big")
        self.m.log("host", "fpga-reg", reg, "%08x" % value)
        return self

    def i2c_fail_next(self, target=None, write=None, status=models.I2C_ERROR):
        self.m.i2c.fail.append((lambda t, a, w: (target is None or t == target) and (write is None or w == write),
                                status))
        self.m.log("host", "i2c-fail-next", target, write)
        return self


def packet(seq, group, opcode, payload=b"", length=None):
    """A request as the host builds it: seq, 0, group, opcode, BE32 payload length, payload."""
    payload = bytes(payload)
    n = len(payload) if length is None else length
    return bytes([seq & 0xff, 0, group & 0xff, opcode & 0xff]) + n.to_bytes(4, "big") + payload
