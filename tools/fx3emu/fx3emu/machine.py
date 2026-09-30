"""The emulated FX3: ARM926 core (Unicorn) running a firmware image, with the Cypress SDK and ThreadX
replaced by Python models (sdk.py) and a cooperative scheduler for the application's threads.

Application code, libc and libgcc run as real ARM code; every call to an SDK/ThreadX function (or to
cyfxtx.c) is intercepted at its entry, logged in normalised form, and answered by a model. Pointers in
the trace are named (symbol+offset, "stack", "heap+off", "dma+off"), so traces of two different builds
of the same source compare equal when their behaviour is the same.

Threads run one at a time until they block (DMA get-buffer with nothing queued, semaphore, sleep,
relinquish); time is virtual (1 tick = 1 ms, ThreadX's FX3 setting). Callbacks (USB events, setup
requests, timers) run to completion on a separate stack, as the SDK's driver thread would.
"""
import random, struct, zlib

from unicorn import (Uc, UcError, UC_ARCH_ARM, UC_MODE_ARM, UC_HOOK_BLOCK, UC_HOOK_MEM_READ,
                     UC_HOOK_MEM_WRITE, UC_HOOK_MEM_UNMAPPED, UC_PROT_ALL)
from unicorn.arm_const import (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP,
                               UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_CPSR)

from . import layouts, models
from .image import sdk_names

ITCM, ITCM_SIZE = 0x00000000, 0x4000
DTCM, DTCM_SIZE = 0x10000000, 0x2000           # callback stack
SYSMEM, SYSMEM_SIZE = 0x40000000, 0x80000
MMIO, MMIO_SIZE = 0xE0000000, 0x100000
DMA_BASE, DMA_END = 0x40040000, 0x40060000      # DMA buffers handed to the firmware
HEAP_BASE, HEAP_END = 0x40060000, 0x40078000    # CyU3PMemAlloc (thread stacks come from here)
CALL_RET, THREAD_EXIT, NEVER = 0x4007ff00, 0x4007ff10, 0x4007ff20
BUDGET = 2_000_000                              # instructions per slice before declaring a hang

ERR_BAD_ARGUMENT, ERR_TIMEOUT, ERR_ABORTED = 0x40, 0x45, 0x48
TX_PTR_ERROR, TX_SEMAPHORE_ERROR, TX_NO_INSTANCE, TX_CALLER_ERROR, TX_ACTIVATE_ERROR = 0x03, 0x0C, 0x0D, 0x13, 0x17


class Halt(Exception):
    pass


class Thread:
    def __init__(self, name, entry, arg, lo, hi, tcb):
        self.name, self.entry, self.arg, self.lo, self.hi, self.tcb = name, entry, arg, lo, hi, tcb
        self.state = "suspended"
        self.ctx = None
        self.wake = 0
        self.pred = self.resume = None
        self.min_sp = hi


class Machine:
    def __init__(self, image, seed=0, flash=(), label=None):
        self.image, self.seed = image, seed
        self.label = label or image.label
        self.layouts = layouts.load()
        uc = self.uc = Uc(UC_ARCH_ARM, UC_MODE_ARM)
        for base, size in ((ITCM, ITCM_SIZE), (DTCM, DTCM_SIZE), (SYSMEM, SYSMEM_SIZE), (MMIO, MMIO_SIZE)):
            uc.mem_map(base, size, UC_PROT_ALL)
        for addr, data in image.segments:
            uc.mem_write(addr, data)
        for a in (CALL_RET, THREAD_EXIT, NEVER):
            uc.mem_write(a, b"\x00\x00\xa0\xe1" * 4)        # nop
        sdk = sdk_names()
        self.stub_at = {s.addr: s.name for s in image.funcs if s.name in sdk}
        from . import sdk as sdk_models
        self.stubs = sdk_models.STUBS
        uc.hook_add(UC_HOOK_BLOCK, self._block)
        uc.hook_add(UC_HOOK_MEM_READ | UC_HOOK_MEM_WRITE, self._mmio, begin=MMIO, end=MMIO + MMIO_SIZE - 1)
        uc.hook_add(UC_HOOK_MEM_UNMAPPED, self._unmapped)

        # models
        self.rnd = random.Random("machine/%d" % seed)
        self.gpio = models.Gpio()
        self.i2c = models.I2cBus(self.gpio, seed)
        self.flash = models.SpiFlash(flash)
        self.mmio_values = {}
        self.connected = False
        self.ep0_out = []                 # data stages for SET-type setup requests
        self.channels = {}                # handle address -> channel dict
        self.queues = {}                  # channel name -> [bytes] from the host
        self.dma_next = DMA_BASE
        self.dma_pool = {}
        self.timers = {}                  # timer address -> dict
        self.sems = {}                    # semaphore address -> count
        self.callbacks = {}               # "setup"/"event"/"lpm"/"ep_event"/"gpio" -> function address
        self.heap_free, self.heap_top, self.heap_blocks = [], HEAP_BASE, {}
        self.fpga_stream = None

        self.threads, self.rr = [], 0
        self.cur = None                   # running Thread, or a string naming a callback context
        self.now = 0
        self.trace, self.replies = [], []
        self.covered = set()
        self.call_min_sp = {}
        self.halted = None
        self._yield = None
        self.forced = {}                  # SDK name -> return values to force on its next calls
        self._last_block, self._repeat = None, 0

    # ---------------------------------------------------------------- registers and memory
    def reg(self, r):
        return self.uc.reg_read(r)

    def arg(self, i):
        if i < 4:
            return self.uc.reg_read((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3)[i])
        return self.rd32(self.uc.reg_read(UC_ARM_REG_SP) + 4 * (i - 4))

    def rd(self, a, n):
        return bytes(self.uc.mem_read(a, n)) if n else b""

    def rd32(self, a):
        return struct.unpack("<I", self.rd(a, 4))[0]

    def wr(self, a, data):
        self.uc.mem_write(a, bytes(data))

    def cstr(self, a, n=256):
        if not a:
            return "(null)"
        s = self.rd(a, n)
        return (s[:s.index(0)] if 0 in s else s).decode("latin1")

    def struct(self, name, addr):
        """Decode an SDK struct at addr into {field: value} (padding never read)."""
        lay = self.layouts[name]
        out = {}
        for f, off, size in lay["fields"]:
            raw = self.rd(addr + off, size)
            out[f] = int.from_bytes(raw, "little") if size in (1, 2, 4) else raw
        return out

    # struct fields the SDK ignores here: their value is don't-care, so they are left out of traces
    DONTCARE = {"CyU3PIoMatrixConfig_t": ("s0Mode", "s1Mode")}     # S-ports unused (useI2S = false)

    def fmt_struct(self, name, addr):
        if not addr:
            return "NULL"
        skip = self.DONTCARE.get(name, ())
        return "{" + ", ".join("%s=%s" % (k, self.norm(v) if isinstance(v, int) else v.hex())
                               for k, v in self.struct(name, addr).items() if k not in skip) + "}"

    # ---------------------------------------------------------------- naming pointers
    def norm(self, v):
        v &= 0xffffffff
        if DTCM <= v < DTCM + DTCM_SIZE:
            return "stack"
        for t in self.threads:
            if t.lo <= v < t.hi:
                return "stack"
        if HEAP_BASE <= v < HEAP_END:
            return "heap+%x" % (v - HEAP_BASE)
        if DMA_BASE <= v < DMA_END:
            return "dma+%x" % (v - DMA_BASE)
        if SYSMEM <= v < SYSMEM + SYSMEM_SIZE:            # (ITCM addresses are not named: small numbers)
            s = self.image.object_at(v)                   # a pointer into data, or to a function's start;
            if s:                                         # a value inside a function is data that looks
                return s.name if v == s.addr else "%s+%d" % (s.name, v - s.addr)     # like an address
            f = self.image.func_at(v)
            if f and f.addr == v:
                return f.name
        return "0x%x" % v if v > 9 else str(v)

    def norm_pc(self, v):
        f = self.image.func_at(v)
        return "%s+%d" % (f.name, v - f.addr) if f else "0x%x" % v

    def ctx_name(self):
        return self.cur.name if isinstance(self.cur, Thread) else (self.cur or "?")

    def log(self, *ev):
        self.trace.append((self.now, self.ctx_name()) + tuple(str(x) for x in ev))

    # ---------------------------------------------------------------- execution
    def _block(self, uc, addr, size, _):
        if addr in self.stub_at:
            name = self.stub_name = self.stub_at[addr]
            handler = self.stubs.get(name)
            if handler is None:
                self.log("UNMODELLED", name, *[self.norm(self.arg(i)) for i in range(4)])
                ret = 0
            else:
                ret = handler(self)
            forced = self.forced.get(name)
            if forced and not self._yield:
                ret = forced.pop(0)
                self.log("forced-return", name, "%x" % ret)
            if self._yield:
                uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))
                uc.emu_stop()
                return
            uc.reg_write(UC_ARM_REG_R0, (ret or 0) & 0xffffffff)
            uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))
            return
        if addr in (CALL_RET, THREAD_EXIT):
            self._yield = ("return",) if addr == CALL_RET else ("exit",)
            uc.emu_stop()
            return
        self.covered.add((addr, size))
        if addr == self._last_block:                      # a block that loops to itself: for (;;) {}
            self._repeat += 1
            if self._repeat > 10000:
                self.log("HANG", "endless loop in %s" % self.norm_pc(addr).split("+")[0])     # (offsets differ by build)
                self.halted = "hang"
                uc.emu_stop()
                return
        else:
            self._last_block, self._repeat = addr, 0
        sp = uc.reg_read(UC_ARM_REG_SP)
        t = self.cur
        if isinstance(t, Thread):
            if sp < t.min_sp:
                t.min_sp = sp
                if sp < t.lo and not getattr(t, "overflowed", False):
                    t.overflowed = True
                    self.log("STACK-OVERFLOW", t.name, "sp=%x below %x" % (sp, t.lo))
        elif t:
            if sp < self.call_min_sp.get(t, DTCM + DTCM_SIZE):
                self.call_min_sp[t] = sp

    def _mmio(self, uc, access, addr, size, value, _):
        if access == 16:        # read
            if addr not in self.mmio_values:
                self.mmio_values[addr] = self.rnd.getrandbits(32)
            v = self.mmio_values[addr] & ((1 << (8 * size)) - 1)
            uc.mem_write(addr, v.to_bytes(size, "little"))
            self.log("mmio-rd", "%08x" % addr, "%x" % v)
        else:
            self.log("mmio-wr", "%08x" % addr, "%x" % value)

    def _unmapped(self, uc, access, addr, size, value, _):
        self.log("CRASH", "unmapped access %08x" % addr, "in %s" % self.norm_pc(uc.reg_read(UC_ARM_REG_PC)).split("+")[0])
        self.halted = "crash"
        return False

    def _start(self, pc, until_budget=BUDGET):
        self._yield = None
        if self.uc.reg_read(UC_ARM_REG_CPSR) & 0x20:
            pc |= 1
        try:
            self.uc.emu_start(pc, NEVER, count=until_budget)
        except UcError as e:
            if not self.halted:
                self.log("CRASH", str(e).split("(")[0].strip(), "in %s" % self.norm_pc(self.uc.reg_read(UC_ARM_REG_PC)).split("+")[0])
                self.halted = "crash"
        if self._yield is None and not self.halted:
            self.log("HANG", "no SDK call in %d instructions" % until_budget,
                     "in %s" % self.norm_pc(self.uc.reg_read(UC_ARM_REG_PC)).split("+")[0])
            self.halted = "hang"

    def call(self, fn, args=(), label=None):
        """Run a function to completion on the callback stack (main, callbacks, timers)."""
        addr = self.image.addr(fn) if isinstance(fn, str) else fn
        prev, self.cur = self.cur, label or (fn if isinstance(fn, str) else self.norm(fn))
        saved = self.uc.context_save()
        self.uc.reg_write(UC_ARM_REG_CPSR, 0x1f)            # system mode, ARM (before SP: SP is banked)
        for i, v in enumerate(args):
            self.uc.reg_write((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3)[i], v & 0xffffffff)
        self.uc.reg_write(UC_ARM_REG_SP, DTCM + DTCM_SIZE - 64)
        self.uc.reg_write(UC_ARM_REG_LR, CALL_RET)
        while True:
            self._start(addr)
            if self.halted or self._yield[0] == "return":
                break
            # a blocking call inside a callback: the SDK would fail it; resume with an error
            self.log("BLOCK-IN-CALLBACK", self._yield[0])
            self.uc.reg_write(UC_ARM_REG_R0, ERR_TIMEOUT)
            addr = self.uc.reg_read(UC_ARM_REG_PC)
        r0 = self.uc.reg_read(UC_ARM_REG_R0)
        self.uc.context_restore(saved)
        self.cur = prev
        return r0

    def block(self, kind, pred, resume):
        """From a stub: suspend the current thread until pred(); resume() then supplies the result."""
        if not isinstance(self.cur, Thread):
            self._yield = (kind,)
            return
        t = self.cur
        t.state, t.pred, t.resume = "blocked", pred, resume
        self._yield = (kind,)

    def sleep(self, ticks):
        if not isinstance(self.cur, Thread):
            return TX_CALLER_ERROR
        t = self.cur
        t.state, t.wake = "sleeping", self.now + max(ticks, 1)
        self._yield = ("sleep",)
        t.resume = lambda: 0
        return 0

    def _run_thread(self, t):
        self.cur = t
        if t.ctx is None:
            self.uc.reg_write(UC_ARM_REG_CPSR, 0x1f)
            self.uc.reg_write(UC_ARM_REG_R0, t.arg)
            self.uc.reg_write(UC_ARM_REG_SP, t.hi & ~7)
            self.uc.reg_write(UC_ARM_REG_LR, THREAD_EXIT)
            pc = t.entry
        else:
            self.uc.context_restore(t.ctx)
            if t.resume:
                r = t.resume()
                self.uc.reg_write(UC_ARM_REG_R0, (r or 0) & 0xffffffff)
                t.resume = None
            pc = self.uc.reg_read(UC_ARM_REG_PC)
        t.state = "ready"
        self._start(pc)
        if self._yield and self._yield[0] == "exit":
            t.state = "done"
            self.log("thread-exit", t.name)
        t.ctx = self.uc.context_save()
        self.cur = None

    # ---------------------------------------------------------------- scheduler
    def run(self, ms):
        """Advance virtual time by ms, running threads and timers."""
        end = self.now + ms
        idle_spins = 0
        while not self.halted:
            ran = False
            n = len(self.threads)
            for k in range(n):
                t = self.threads[(self.rr + k) % n]
                if t.state == "sleeping" and t.wake <= self.now:
                    t.state = "ready"
                if t.state == "blocked" and t.pred():
                    t.state = "ready"
                if t.state == "ready":
                    self.rr = (self.rr + k + 1) % n
                    self._run_thread(t)
                    ran = True
                    break
            if self.halted:
                break
            if ran:
                idle_spins = 0
                continue
            nxt = [t.wake for t in self.threads if t.state == "sleeping"]
            nxt += [tm["expiry"] for tm in self.timers.values() if tm["active"]]
            nxt = min([x for x in nxt if x > self.now] or [end + 1])
            if nxt > end:
                self.now = end
                break
            self.now = nxt
            self._fire_timers()
        return self

    def _fire_timers(self):
        for addr, tm in sorted(self.timers.items()):
            if tm["active"] and tm["expiry"] <= self.now:
                if tm["resched"]:
                    tm["expiry"] = self.now + tm["resched"]
                else:
                    tm["active"] = False
                self.log("timer-expired", tm["name"])
                self.call(tm["cb"], (tm["input"],), label="timer:" + tm["name"])
                if self.halted:
                    return

    def thread_by_tcb(self, tcb):
        for t in self.threads:
            if t.tcb == tcb:
                return t
        return None

    # ---------------------------------------------------------------- heap
    def alloc(self, n):
        n = (n + 7) & ~7
        for i, (a, sz) in enumerate(self.heap_free):
            if sz >= n:
                del self.heap_free[i]
                if sz > n:
                    self.heap_free.insert(i, (a + n, sz - n))
                self.heap_blocks[a] = n
                self.wr(a, b"\0" * n)
                return a
        if self.heap_top + n > HEAP_END:
            return 0
        a, self.heap_top = self.heap_top, self.heap_top + n
        self.heap_blocks[a] = n
        self.wr(a, b"\0" * n)
        return a

    def free(self, a):
        n = self.heap_blocks.pop(a, None)
        if n is None:
            self.log("BAD-FREE", self.norm(a))
            return
        self.heap_free.append((a, n))
        self.heap_free.sort()

    # ---------------------------------------------------------------- results
    def final_state(self, names=None):
        """Application data by name (pointer words named), plus the peripheral models."""
        out = {}
        for s in self.image.objects:
            if names is not None and s.name not in names:
                continue
            if not (SYSMEM <= s.addr < SYSMEM + SYSMEM_SIZE) or s.size == 0 or s.size > 0x10000:
                continue
            data = self.rd(s.addr, s.size)
            words = []
            for i in range(0, len(data) - len(data) % 4, 4):
                v = struct.unpack_from("<I", data, i)[0]
                nv = self.norm(v)
                words.append(nv if not nv.startswith("0x") and not nv.isdigit() else "%08x" % v)
            tail = data[len(data) - len(data) % 4:].hex()
            out["mem:" + s.name] = " ".join(words) + (" +" + tail if tail else "")
        out["flash"] = "%08x" % zlib.crc32(bytes(self.flash.mem))
        out["flash-sr"] = str(self.flash.sr)
        for key, dev in sorted(self.i2c.devices.items(), key=lambda kv: str(kv[0])):
            regs = dev.regs
            out["i2c:%s" % getattr(dev, "name", "fpga")] = " ".join(
                "%x=%s" % (r, (v.hex() if isinstance(v, bytes) else "%02x" % v)) for r, v in sorted(regs.items()))
        out["fpga-config"] = "%s/%d" % (self.i2c.fpga.config_crc, self.i2c.fpga.config_bytes)
        out["gpio"] = " ".join("%d=%d" % (p, v) for p, v in sorted(self.gpio.level.items()))
        out["connected"] = str(self.connected)
        return out

    def stack_report(self):
        rows = [(t.name, t.hi - t.lo, t.hi - t.min_sp) for t in self.threads]
        rows += [(k, None, DTCM + DTCM_SIZE - 64 - v) for k, v in sorted(self.call_min_sp.items())]
        return rows
