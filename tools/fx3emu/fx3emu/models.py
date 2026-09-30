"""Behavioural models of what sits around the FX3: I2C mux and devices, SPI flash, GPIOs.

The models aim at determinism and responsiveness, not electrical fidelity: every read is a function
of what was written before and of the scenario's seed, so a behaviour change in the firmware shows up
as a different trace or final state.
"""
import random, zlib

# I2C target select (board.h): 4-bit code on GPIO 23/25/26/27
SEL_PINS = (23, 25, 26, 27)
TARGET_NAMES = {0: "none", 1: "cam1", 2: "cam2", 4: "cam4", 5: "fpga", 7: "light", 8: "adt7420"}
I2C_ERROR = 0x46                # status returned on a NAK / injected failure (any non-zero will do)


class Gpio:
    def __init__(self):
        self.level = {}                              # pin -> bool
        self.inputs = {}                             # pin -> bool, set by the scenario

    def set(self, pin, v):
        self.level[pin] = bool(v)

    def get(self, pin):
        return self.inputs.get(pin, self.level.get(pin, False))

    def i2c_target(self):
        return sum(1 << i for i, p in enumerate(SEL_PINS) if self.level.get(p, False))


class RegDevice:
    """Register file: address width `aw` bytes; reads of unwritten registers give seeded bytes."""

    def __init__(self, name, aw, seed, width=1):
        self.name, self.aw, self.width = name, aw, width
        self.regs = {}
        self.rnd = random.Random("%s/%d" % (name, seed))
        self.defaults = {}

    def write(self, reg, data):
        for i, b in enumerate(data):
            self.regs[reg + i] = b

    def read(self, reg, n):
        out = bytearray()
        for i in range(n):
            r = reg + i
            if r not in self.regs:
                self.regs[r] = self.defaults.get(r, self.rnd.randrange(256))
            out.append(self.regs[r])
        return bytes(out)


class Fpga:
    """FPGA register bridge: 8-bit register number, 32-bit big-endian values (one per register)."""

    def __init__(self, seed):
        self.regs = {}
        self.rnd = random.Random("fpga/%d" % seed)
        self.config_crc = None          # checksum of the bitstream it was configured with
        self.config_bytes = 0

    def write(self, reg, data):
        self.regs[reg] = bytes(data[:4]).ljust(4, b"\0")

    def read(self, reg, n):
        if reg not in self.regs:
            self.regs[reg] = bytes(self.rnd.randrange(256) for _ in range(4))
        return (self.regs[reg] * ((n + 3) // 4))[:n]


class I2cBus:
    """Devices keyed by (mux target or None for always-connected, 7-bit address)."""

    def __init__(self, gpio, seed):
        self.gpio = gpio
        self.fpga = Fpga(seed)
        self.devices = {
            (5, 0x10): self.fpga,
            (1, 0x32): RegDevice("cam1", 2, seed), (2, 0x32): RegDevice("cam2", 2, seed),
            (4, 0x32): RegDevice("cam4", 2, seed),
            (8, 0x48): RegDevice("adt7420", 1, seed),
            (7, 0x53): RegDevice("light", 1, seed),
            (None, 0x50): RegDevice("eeprom", 2, seed),
        }
        self.fail = []                  # [(predicate(target, addr7, write), status)] one-shot failures
        self.fail_rate, self.fail_rnd = 0.0, random.Random("i2cfail/%d" % seed)
        self.faults_enabled = True      # False suppresses all injected/random faults (fault-free gate)

    def device(self, addr7):
        t = self.gpio.i2c_target()
        return self.devices.get((t, addr7)) or self.devices.get((None, addr7)), t

    def _injected(self, t, addr7, write):
        if not self.faults_enabled:
            return 0
        for i, (pred, st) in enumerate(self.fail):
            if pred(t, addr7, write):
                del self.fail[i]
                return st
        if self.fail_rate and self.fail_rnd.random() < self.fail_rate:
            return I2C_ERROR
        return 0

    def transmit(self, preamble, data):
        """Write: preamble = [addr W, register bytes...], then data."""
        addr7 = preamble[0] >> 1
        dev, t = self.device(addr7)
        st = self._injected(t, addr7, True)
        if st or dev is None:
            return st or I2C_ERROR, dev, t
        body = bytes(preamble[1:]) + bytes(data)
        aw = 1 if isinstance(dev, Fpga) else dev.aw
        reg = int.from_bytes(body[:aw], "big") if aw else 0
        dev.write(reg, body[aw:])
        return 0, dev, t

    def receive(self, preamble, n):
        """Read: preamble = [addr W, register bytes..., addr R] (repeated start before the last)."""
        addr7 = preamble[0] >> 1
        dev, t = self.device(addr7)
        st = self._injected(t, addr7, False)
        if st or dev is None:
            return st or I2C_ERROR, b"\0" * n, dev, t
        regb = bytes(preamble[1:-1])
        reg = int.from_bytes(regb, "big") if regb else 0
        return 0, dev.read(reg, n), dev, t


class SpiFlash:
    """Winbond-style SPI NOR (16 MB): WREN, RDSR1-3, WRSR1-3 (+ volatile enable), READ, PP, SE (4 KB),
    BE (64 KB). Each chip-select period is one transaction; `select(False)` returns its summary."""

    SIZE = 16 << 20
    NAMES = {0x06: "WREN", 0x04: "WRDI", 0x50: "VSR-WREN", 0x05: "RDSR1", 0x35: "RDSR2", 0x15: "RDSR3",
             0x01: "WRSR1", 0x31: "WRSR2", 0x11: "WRSR3", 0x03: "READ", 0x02: "PP", 0x20: "SE", 0xD8: "BE"}

    def __init__(self, contents=()):
        self.mem = bytearray(b"\xff" * self.SIZE)
        for off, data in contents:
            self.mem[off:off + len(data)] = data
        self.sr = [0x00, 0x00, 0x00]       # SR1: BUSY bit 0 (never set: operations are instant), WEL bit 1
        self.vsr = False
        self.selected = False
        self.fpga_sink = None              # callable(bytes): the FPGA snoops READ data (GPIO 51 low)

    def select(self, low):
        """Chip select. Returns the transaction summary when a transaction ends, else None."""
        if low and not self.selected:
            self.selected, self.cmd, self.nin, self.nout = True, bytearray(), 0, 0
            self.crc_in = self.crc_out = 0
            self.addr = None
            return None
        if not low and self.selected:
            self.selected = False
            return self._finish()
        return None

    def transmit(self, data):
        if not self.selected:
            return
        data = bytes(data)
        need = 4 if (self.cmd[:1] or data[:1]) in (b"\x02", b"\x03", b"\x20", b"\xd8") else 2
        take = max(0, min(len(data), need - len(self.cmd)))
        self.cmd += data[:take]
        rest = data[take:]
        if self.cmd[:1] == b"\x02" and len(self.cmd) == 4 and rest:
            if self.addr is None:
                self.addr = int.from_bytes(self.cmd[1:4], "big")
                self.pp_ok = bool(self.sr[0] & 2)
            self.nout += len(rest)
            self.crc_out = zlib.crc32(rest, self.crc_out)
            if self.pp_ok:
                a = self.addr
                for b in rest:
                    self.mem[a] &= b
                    a = (a & ~0xff) | ((a + 1) & 0xff)          # wraps within the 256-byte page
                self.addr = a

    def receive(self, n):
        if not self.selected or not self.cmd:
            return b"\xff" * n
        c = self.cmd[0]
        if c in (0x05, 0x35, 0x15):
            out = bytes([self.sr[{0x05: 0, 0x35: 1, 0x15: 2}[c]]]) * n
        elif c == 0x03 and len(self.cmd) >= 4:
            if self.addr is None:
                self.addr = int.from_bytes(self.cmd[1:4], "big")
            a = self.addr
            out = bytes(self.mem[(a + i) % self.SIZE] for i in range(n)) if a + n > self.SIZE else bytes(self.mem[a:a + n])
            self.addr = (a + n) % self.SIZE
            if self.fpga_sink:
                self.fpga_sink(out)
        else:
            out = b"\xff" * n
        self.nin += n
        self.crc_in = zlib.crc32(out, self.crc_in)
        return out

    def _finish(self):
        if not self.cmd:
            return None
        c = self.cmd[0]
        a = int.from_bytes(self.cmd[1:4], "big") if len(self.cmd) >= 4 else None
        wel = bool(self.sr[0] & 2)
        note = ""
        if c == 0x06:
            self.sr[0] |= 2
        elif c == 0x04:
            self.sr[0] &= ~2
        elif c == 0x50:
            self.vsr = True
        elif c in (0x01, 0x31, 0x11) and len(self.cmd) >= 2:
            if wel or self.vsr:
                i = {0x01: 0, 0x31: 1, 0x11: 2}[c]
                self.sr[i] = self.cmd[1] & (0xfc if i == 0 else 0xff)
            else:
                note = "ignored: no WREN"
            self.sr[0] &= ~2
            self.vsr = False
        elif c == 0x02:
            if not wel:
                note = "ignored: no WREN"
            self.sr[0] &= ~2
        elif c in (0x20, 0xD8) and a is not None:
            size = 0x1000 if c == 0x20 else 0x10000
            if wel:
                base = a & ~(size - 1)
                self.mem[base:base + size] = b"\xff" * size
            else:
                note = "ignored: no WREN"
            self.sr[0] &= ~2
        return {"cmd": self.NAMES.get(c, "%02x" % c), "arg": self.cmd[1:].hex(), "addr": a,
                "out": self.nout, "out_crc": self.crc_out, "in": self.nin, "in_crc": self.crc_in, "note": note}


def boot_record(new_app, old_app, boot_new):
    return (new_app.to_bytes(4, "little") + old_app.to_bytes(4, "little") + bytes([boot_new])).ljust(256, b"\xff")
