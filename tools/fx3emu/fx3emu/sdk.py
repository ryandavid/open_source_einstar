"""Models of the Cypress FX3 SDK and ThreadX functions the application calls.

Each stub reads its arguments from the emulated CPU, updates the models, logs a normalised event and
returns the SDK's return value. Names and signatures are those of the public SDK API.
"""
import zlib

from . import models
from .machine import (Thread, ERR_BAD_ARGUMENT, ERR_TIMEOUT, ERR_ABORTED, TX_PTR_ERROR, TX_SEMAPHORE_ERROR,
                      TX_NO_INSTANCE, TX_ACTIVATE_ERROR)

STUBS = {}


def stub(*names):
    def deco(fn):
        for n in names:
            STUBS[n] = fn
        return fn
    return deco


def quiet(*names, ret=0):
    """SDK calls whose arguments are plain values: log name and arguments."""
    for n in names:
        def f(m, n=n):
            m.log(n, *[m.norm(m.arg(i)) for i in range(ARGC.get(n, 0))])
            return ret
        STUBS[n] = f


# number of arguments to log for the plain stubs
ARGC = {
    "CyU3PVicDisableInt": 1, "CyU3PLppEventSend": 3, "CyU3PVicEnableInt": 1, "CyU3PBusyWait": 1,
    "CyU3PThreadIdentify": 0,
    "CyU3PDebugInit": 2, "CyU3PDebugPreamble": 1, "CyU3PDeviceCacheControl": 3, "CyU3PDeviceGpioOverride": 2,
    "CyU3PUartInit": 0, "CyU3PUartTxSetBlockXfer": 1, "CyU3PI2cInit": 0, "CyU3PSpiInit": 0, "CyU3PGpioDeInit": 0,
    "CyU3PUsbStart": 0, "CyU3PUsbLPMDisable": 0, "CyU3PUsbFlushEp": 1, "CyU3PUsbResetEp": 1,
    "CyU3PUsbSetEpNak": 2, "CyU3PUsbStall": 3, "CyU3PUsbAckSetup": 0, "CyU3PGpifSocketConfigure": 4,
    "CyU3PGpifSMStart": 2, "CyU3PGpifDisable": 1, "CyU3PDmaChannelSetXfer": 2, "CyFx3BusyWait": 1,
}
quiet(*ARGC)


# ---------------------------------------------------------------- debug output
def printf(m, fmt, argi):
    out, i = [], 0
    while i < len(fmt):
        c = fmt[i]
        if c == "%" and i + 1 < len(fmt):
            j = i + 1
            while j < len(fmt) and fmt[j] in "0123456789.-+ #l":
                j += 1
            conv = fmt[j] if j < len(fmt) else "%"
            if conv == "%":
                out.append("%")
            else:
                v = m.arg(argi)
                argi += 1
                if conv == "s":
                    out.append(m.cstr(v))
                elif conv in "di":
                    out.append(str(v - (1 << 32) if v & 0x80000000 else v))
                elif conv == "u":
                    out.append(str(v))
                elif conv in "xX":
                    out.append(("%x" if conv == "x" else "%X") % v)
                elif conv == "c":
                    out.append(chr(v & 0xff))
                elif conv == "p":
                    out.append(m.norm(v))
                else:
                    out.append("%" + conv)
            i = j + 1
        else:
            out.append(c)
            i += 1
    return "".join(out)


@stub("CyU3PDebugPrint")
def debug_print(m):
    m.log("print", m.arg(0), printf(m, m.cstr(m.arg(1), 512), 2).rstrip())
    return 0


# ---------------------------------------------------------------- memory (cyfxtx.c)
@stub("CyU3PMemAlloc")
def mem_alloc(m):
    a = m.alloc(m.arg(0))
    m.log("MemAlloc", m.arg(0), m.norm(a))
    return a


@stub("CyU3PMemFree")
def mem_free(m):
    m.log("MemFree", m.norm(m.arg(0)))
    m.free(m.arg(0))
    return 0


@stub("CyU3PMemSet")
def mem_set(m):
    p, v, n = m.arg(0), m.arg(1) & 0xff, m.arg(2)
    m.wr(p, bytes([v]) * n)
    return 0


@stub("CyU3PMemCopy")
def mem_copy(m):
    d, s, n = m.arg(0), m.arg(1), m.arg(2)
    m.wr(d, m.rd(s, n))
    return 0


@stub("CyU3PMemCmp")
def mem_cmp(m):
    a, b = m.rd(m.arg(0), m.arg(2)), m.rd(m.arg(1), m.arg(2))
    return 0 if a == b else (1 if a > b else 0xffffffff)


# ---------------------------------------------------------------- device
@stub("CyU3PDeviceInit")
def device_init(m):
    m.log("DeviceInit", m.fmt_struct("CyU3PSysClockConfig_t", m.arg(0)))
    return 0


@stub("CyU3PDeviceConfigureIOMatrix")
def io_matrix(m):
    m.log("ConfigureIOMatrix", m.fmt_struct("CyU3PIoMatrixConfig_t", m.arg(0)))
    return 0


@stub("CyU3PDeviceReset")
def device_reset(m):
    m.log("DeviceReset", m.arg(0))
    m.halted = "reset"
    m._yield = ("reset",)
    return 0


@stub("_tx_initialize_kernel_enter")
def kernel_enter(m):
    m.log("KernelEnter")
    m._yield = ("kernel",)
    m.halted = "kernel"
    return 0


# ---------------------------------------------------------------- ThreadX
@stub("_txe_thread_create")
def thread_create(m):
    tcb, name, entry, inp, stack, size = (m.arg(i) for i in range(6))
    prio, thres, slice_, auto = (m.arg(i) for i in range(6, 10))
    tname = m.cstr(name)
    m.log("ThreadCreate", tname, m.norm(entry), inp, m.norm(stack), size, prio, thres, slice_, auto)
    if not stack or not entry:
        return TX_PTR_ERROR
    t = Thread(tname, entry, inp, stack, stack + size, tcb)
    t.state = "ready" if auto else "suspended"
    m.threads.append(t)
    return 0


@stub("_txe_thread_resume")
def thread_resume(m):
    t = m.thread_by_tcb(m.arg(0))
    m.log("ThreadResume", t.name if t else m.norm(m.arg(0)))
    if t and t.state == "suspended":
        t.state = "ready"
    return 0


@stub("_txe_thread_relinquish")
def thread_relinquish(m):
    m.sleep(1)                   # lets the others run; costs a tick of virtual time
    return 0


@stub("_tx_thread_sleep", "_txe_thread_sleep")
def thread_sleep(m):
    m.log("Sleep", m.arg(0))
    return m.sleep(m.arg(0))


@stub("_txe_semaphore_create")
def sem_create(m):
    m.log("SemaphoreCreate", m.norm(m.arg(0)), m.cstr(m.arg(1)), m.arg(2))
    m.sems[m.arg(0)] = m.arg(2)
    return 0


@stub("_txe_semaphore_get")
def sem_get(m):
    s, wait = m.arg(0), m.arg(1)
    if s not in m.sems:
        return TX_SEMAPHORE_ERROR            # never created (flash_sem)
    if m.sems[s] > 0:
        m.sems[s] -= 1
        return 0
    if not wait:
        return TX_NO_INSTANCE
    m.log("SemaphoreWait", m.norm(s))

    def ready(s=s):
        return m.sems[s] > 0

    def resume(s=s):
        m.sems[s] -= 1
        return 0
    m.block("semaphore", ready, resume)
    return 0


@stub("_txe_semaphore_put")
def sem_put(m):
    s = m.arg(0)
    if s not in m.sems:
        return TX_SEMAPHORE_ERROR
    m.sems[s] += 1
    return 0


@stub("_txe_timer_create")
def timer_create(m):
    t, name, cb, inp, initial, resched, auto = (m.arg(i) for i in range(7))
    m.log("TimerCreate", m.norm(t), m.cstr(name), m.norm(cb), inp, initial, resched, auto)
    m.timers[t] = {"name": m.norm(t), "cb": cb, "input": inp, "initial": initial, "resched": resched,
                   "active": bool(auto), "expiry": m.now + initial}
    return 0


@stub("_txe_timer_activate")
def timer_activate(m):
    tm = m.timers.get(m.arg(0))
    m.log("TimerStart", tm["name"] if tm else m.norm(m.arg(0)))
    if tm is None:
        return 0x15                          # TX_TIMER_ERROR
    if tm["active"]:
        return TX_ACTIVATE_ERROR
    tm["active"], tm["expiry"] = True, m.now + max(tm["initial"], 1)
    return 0


@stub("_txe_timer_deactivate")
def timer_deactivate(m):
    tm = m.timers.get(m.arg(0))
    m.log("TimerStop", tm["name"] if tm else m.norm(m.arg(0)))
    if tm:
        tm["active"] = False
    return 0


@stub("_txe_timer_change")
def timer_change(m):
    tm = m.timers.get(m.arg(0))
    m.log("TimerModify", tm["name"] if tm else m.norm(m.arg(0)), m.arg(1), m.arg(2))
    if tm:
        tm["initial"], tm["resched"] = m.arg(1), m.arg(2)
    return 0


# ---------------------------------------------------------------- UART, I2C, SPI, GPIO
@stub("CyU3PUartSetConfig")
def uart_config(m):
    m.log("UartSetConfig", m.fmt_struct("CyU3PUartConfig_t", m.arg(0)), m.norm(m.arg(1)))
    return 0


@stub("CyU3PI2cSetConfig")
def i2c_config(m):
    m.log("I2cSetConfig", m.fmt_struct("CyU3PI2cConfig_t", m.arg(0)), m.norm(m.arg(1)))
    return 0


def _preamble(m, p):
    pre = m.struct("CyU3PI2cPreamble_t", p)
    return pre["buffer"][:pre["length"]], pre["ctrlMask"]


@stub("CyU3PI2cTransmitBytes")
def i2c_tx(m):
    pre, ctrl = _preamble(m, m.arg(0))
    data = m.rd(m.arg(1), m.arg(2))
    st, dev, t = m.i2c.transmit(pre, data)
    m.log("i2c-wr", models.TARGET_NAMES.get(t, t), pre.hex(), "ctrl=%x" % ctrl, data.hex(), "retry=%d" % m.arg(3),
          "status=%x" % st)
    return st


@stub("CyU3PI2cReceiveBytes")
def i2c_rx(m):
    pre, ctrl = _preamble(m, m.arg(0))
    n = m.arg(2)
    st, data, dev, t = m.i2c.receive(pre, n)
    if not st:
        m.wr(m.arg(1), data)
    m.log("i2c-rd", models.TARGET_NAMES.get(t, t), pre.hex(), "ctrl=%x" % ctrl, data.hex() if not st else "",
          "retry=%d" % m.arg(3), "status=%x" % st)
    return st


@stub("CyU3PSpiSetConfig")
def spi_config(m):
    m.log("SpiSetConfig", m.fmt_struct("CyU3PSpiConfig_t", m.arg(0)), m.norm(m.arg(1)))
    return 0


@stub("CyU3PSpiSetSsnLine")
def spi_ssn(m):
    summary = m.flash.select(not m.arg(0))
    if summary:
        s = summary
        m.log("flash", s["cmd"], s["arg"], "out=%d/%08x" % (s["out"], s["out_crc"]),
              "in=%d/%08x" % (s["in"], s["in_crc"]), s["note"])
    return 0


@stub("CyU3PSpiTransmitWords")
def spi_tx(m):
    m.flash.transmit(m.rd(m.arg(0), m.arg(1)))
    return 0


@stub("CyU3PSpiReceiveWords")
def spi_rx(m):
    m.wr(m.arg(0), m.flash.receive(m.arg(1)))
    return 0


@stub("CyU3PGpioInit")
def gpio_init(m):
    m.log("GpioInit", m.fmt_struct("CyU3PGpioClock_t", m.arg(0)), m.norm(m.arg(1)))
    m.callbacks["gpio"] = m.arg(1)
    return 0


@stub("CyU3PGpioSetSimpleConfig")
def gpio_config(m):
    m.log("GpioSetSimpleConfig", m.arg(0), m.fmt_struct("CyU3PGpioSimpleConfig_t", m.arg(1)))
    cfg = m.struct("CyU3PGpioSimpleConfig_t", m.arg(1))
    if cfg["driveLowEn"] or cfg["driveHighEn"]:
        m.gpio.set(m.arg(0), cfg["outValue"])
    return 0


def _fpga_snoop(m):
    fpga = m.i2c.fpga

    def sink(data):
        fpga.config_crc = zlib.crc32(data, fpga.config_crc or 0)
        fpga.config_bytes += len(data)
    return sink


@stub("CyU3PGpioSimpleSetValue")
def gpio_set(m):
    pin, v = m.arg(0), bool(m.arg(1))
    old = m.gpio.level.get(pin)
    m.gpio.set(pin, v)
    if pin == 37 and not v:                       # FPGA PROGRAM pulse: configuration starts over
        m.i2c.fpga.config_crc, m.i2c.fpga.config_bytes = None, 0
    if pin == 51:                                 # FPGA takes the SPI read data while low
        m.flash.fpga_sink = None if v else _fpga_snoop(m)
        return 0
    if pin not in models.SEL_PINS and old != v:
        m.log("gpio", pin, int(v))
    return 0


@stub("CyU3PGpioGetValue")
def gpio_get(m):
    v = m.gpio.get(m.arg(0))
    m.wr(m.arg(1), int(v).to_bytes(4, "little"))
    m.log("GpioGetValue", m.arg(0), int(v))
    return 0


# ---------------------------------------------------------------- USB
@stub("CyU3PPibInit")
def pib_init(m):
    m.log("PibInit", m.arg(0), m.fmt_struct("CyU3PPibClock_t", m.arg(1)))
    return 0


@stub("CyU3PUsbRegisterSetupCallback", "CyU3PUsbRegisterEventCallback", "CyU3PUsbRegisterLPMRequestCallback",
      "CyU3PUsbRegisterEpEvtCallback")
def usb_register(m):
    name = m.stub_name
    kind = {"CyU3PUsbRegisterSetupCallback": "setup", "CyU3PUsbRegisterEventCallback": "event",
            "CyU3PUsbRegisterLPMRequestCallback": "lpm", "CyU3PUsbRegisterEpEvtCallback": "ep_event"}.get(name, name)
    m.callbacks[kind] = m.arg(0)
    extra = [m.arg(i) for i in range(1, 4)] if kind == "ep_event" else [m.arg(1)] if kind == "setup" else []
    m.log(name, m.norm(m.arg(0)), *["0x%x" % x for x in extra])
    return 0


def _descriptor(m, p):
    if not p:
        return b""
    head = m.rd(p, 4)
    n = head[2] | head[3] << 8 if head[1] in (2, 7, 0x0f) else head[0]
    return m.rd(p, n)


@stub("CyU3PUsbSetDesc")
def usb_set_desc(m):
    m.log("UsbSetDesc", m.arg(0), m.arg(1), _descriptor(m, m.arg(2)).hex())
    return 0


@stub("CyU3PConnectState")
def connect_state(m):
    m.connected = bool(m.arg(0))
    m.log("ConnectState", m.arg(0), m.arg(1))
    return 0


@stub("CyU3PGetConnectState")
def get_connect_state(m):
    return int(m.connected)


@stub("CyU3PUsbGetDevProperty")
def get_dev_property(m):
    m.wr(m.arg(1), bytes(range(1, 7)))
    m.log("UsbGetDevProperty", m.arg(0))
    return 0


@stub("CyU3PSetEpConfig")
def set_ep_config(m):
    m.log("SetEpConfig", "%02x" % m.arg(0), m.fmt_struct("CyU3PEpConfig_t", m.arg(1)))
    return 0


@stub("CyU3PUsbSendEP0Data")
def send_ep0(m):
    data = m.rd(m.arg(1), m.arg(0))
    m.log("EP0-in", data.hex())
    m.replies.append(("ep0", data))
    return 0


@stub("CyU3PUsbGetEP0Data")
def get_ep0(m):
    n, buf, cnt = m.arg(0), m.arg(1), m.arg(2)
    if not m.ep0_out:
        m.log("EP0-out", "none")
        return ERR_TIMEOUT
    data = m.ep0_out.pop(0)[:n]
    m.wr(buf, data)
    if cnt:
        m.wr(cnt, len(data).to_bytes(2, "little"))
    m.log("EP0-out", data.hex())
    return 0


# ---------------------------------------------------------------- GPIF
@stub("CyU3PGpifLoad")
def gpif_load(m):
    p = m.arg(0)
    cfg = m.struct("CyU3PGpifConfig_t", p)
    parts = []
    for cnt, ptr in (("stateCount", "stateData"), ("stateCount", "statePosition"),
                     ("functionCount", "functionData"), ("regCount", "regData")):
        obj = m.image.object_at(cfg[ptr]) if cfg[ptr] else None      # each table checksummed whole
        data = m.rd(obj.addr, obj.size) if obj else b""
        parts.append("%s=%s[%d]/%d/%08x" % (ptr, m.norm(cfg[ptr]), cfg[cnt], len(data), zlib.crc32(data)))
    m.log("GpifLoad", m.norm(p), *parts)
    return 0


# ---------------------------------------------------------------- DMA channels
INPUT_CHANNELS = ("ch_ctrl_out", "ch_bulk_out")       # host -> firmware
OUTPUT_CHANNELS = ("ch_bulk_in",)                     # firmware -> host via get/commit


@stub("CyU3PDmaChannelCreate")
def dma_create(m):
    h, typ, cfg = m.arg(0), m.arg(1), m.arg(2)
    c = m.struct("CyU3PDmaChannelConfig_t", cfg)
    name = m.norm(h)
    m.log("DmaChannelCreate", name, typ, m.fmt_struct("CyU3PDmaChannelConfig_t", cfg))
    ch = {"name": name, "size": c["size"], "count": max(c["count"], 1), "bufs": [], "next": 0, "live": True,
          "given": None, "gen": 0}
    if name in INPUT_CHANNELS + OUTPUT_CHANNELS:        # only these carry data in the model
        key = (name, c["size"], ch["count"])
        if key not in m.dma_pool:                        # a re-created channel gets its old buffers
            m.dma_pool[key] = []
            for _ in range(ch["count"]):
                m.dma_pool[key].append(m.dma_next)
                m.dma_next += (c["size"] + 15) & ~15
        ch["bufs"] = m.dma_pool[key]
    m.channels[h] = ch
    m.queues.setdefault(name, [])
    return 0


@stub("CyU3PDmaChannelDestroy", "CyU3PDmaChannelReset")
def dma_destroy(m):
    h = m.arg(0)
    ch = m.channels.get(h)
    m.log(m.stub_name.replace("CyU3P", ""), ch["name"] if ch else m.norm(h))
    if ch:
        ch["gen"] += 1                          # waiting get-buffer calls are aborted
        if m.stub_name == "CyU3PDmaChannelDestroy":
            ch["live"] = False
    return 0


def _fill_buffer(m, bufp, addr, count, size):
    lay = {f: (off, sz) for f, off, sz in m.layouts["CyU3PDmaBuffer_t"]["fields"]}
    m.wr(bufp + lay["buffer"][0], addr.to_bytes(4, "little"))
    m.wr(bufp + lay["count"][0], count.to_bytes(2, "little"))
    m.wr(bufp + lay["size"][0], size.to_bytes(2, "little"))
    m.wr(bufp + lay["status"][0], (0).to_bytes(2, "little"))


@stub("CyU3PDmaChannelGetBuffer")
def dma_get(m):
    h, bufp, wait = m.arg(0), m.arg(1), m.arg(2)
    ch = m.channels.get(h)
    if ch is None or not ch["live"]:
        m.log("DmaChannelGetBuffer", m.norm(h), "not created")
        return ERR_BAD_ARGUMENT
    name = ch["name"]
    if name in INPUT_CHANNELS:
        q = m.queues[name]

        def deliver():
            data = q.pop(0)
            addr = ch["bufs"][ch["next"]]
            ch["next"] = (ch["next"] + 1) % ch["count"]
            m.wr(addr, data[:ch["size"]])            # stale bytes of earlier packets stay beyond it
            _fill_buffer(m, bufp, addr, min(len(data), ch["size"]), ch["size"])
            m.log("DmaChannelGetBuffer", name, len(data))
            return 0
        if q:
            return deliver()
        if not wait:
            return ERR_TIMEOUT
        gen = ch["gen"]

        def ready():
            return bool(q) or ch["gen"] != gen

        def resume():
            if ch["gen"] != gen:
                m.log("DmaChannelGetBuffer", name, "aborted")
                return ERR_ABORTED
            return deliver()
        m.block("dma", ready, resume)
        return 0
    addr = ch["bufs"][ch["next"]]
    ch["next"] = (ch["next"] + 1) % ch["count"]
    ch["given"] = addr
    _fill_buffer(m, bufp, addr, 0, ch["size"])
    m.log("DmaChannelGetBuffer", name)
    return 0


@stub("CyU3PDmaChannelCommitBuffer")
def dma_commit(m):
    h, count = m.arg(0), m.arg(1) & 0xffff
    ch = m.channels.get(h)
    if ch is None or ch["given"] is None:
        m.log("DmaChannelCommitBuffer", m.norm(h), count, "no buffer")
        return ERR_BAD_ARGUMENT
    if count > ch["size"]:
        m.log("DmaChannelCommitBuffer", ch["name"], count, "too long")
        return ERR_BAD_ARGUMENT
    data = m.rd(ch["given"], count)
    ch["given"] = None
    m.log("DmaChannelCommitBuffer", ch["name"], count, "%08x" % zlib.crc32(data), data[:32].hex())
    m.replies.append((ch["name"], data))
    return 0


@stub("CyU3PDmaChannelDiscardBuffer")
def dma_discard(m):
    ch = m.channels.get(m.arg(0))
    m.log("DmaChannelDiscardBuffer", ch["name"] if ch else m.norm(m.arg(0)))
    return 0


@stub("CyU3PDmaChannelSendData")
def dma_send(m):
    h, p, n = m.arg(0), m.arg(1), m.arg(2) & 0xffff
    ch = m.channels.get(h)
    data = m.rd(p, n)
    name = ch["name"] if ch else m.norm(h)
    m.log("DmaChannelSendData", name, n, data.hex())
    m.replies.append((name, data))
    return 0
