"""Field offsets of the SDK structures the firmware passes to the SDK, computed from the local SDK
headers with the ARM compiler (nothing from the SDK is stored in this repository). Stubs decode
struct arguments field by field, so padding bytes (uninitialised stack) never reach a trace."""
import hashlib, json, os, re, subprocess, tempfile

from . import paths

# (struct, [fields]) as named in the SDK headers
STRUCTS = {
    "CyU3PUartConfig_t": ["txEnable", "rxEnable", "flowCtrl", "isDma", "baudRate", "stopBit", "parity"],
    "CyU3PI2cConfig_t": ["bitRate", "isDma", "busTimeout", "dmaTimeout"],
    "CyU3PSpiConfig_t": ["isLsbFirst", "cpol", "cpha", "ssnPol", "ssnCtrl", "leadTime", "lagTime", "clock", "wordLen"],
    "CyU3PGpioClock_t": ["fastClkDiv", "slowClkDiv", "halfDiv", "simpleDiv", "clkSrc"],
    "CyU3PGpioSimpleConfig_t": ["outValue", "driveLowEn", "driveHighEn", "inputEn", "intrMode"],
    "CyU3PIoMatrixConfig_t": ["isDQ32Bit", "useUart", "useI2C", "useI2S", "useSpi", "s0Mode", "s1Mode", "lppMode",
                              "gpioSimpleEn[0]", "gpioSimpleEn[1]", "gpioComplexEn[0]", "gpioComplexEn[1]"],
    "CyU3PSysClockConfig_t": ["setSysClk400", "cpuClkDiv", "dmaClkDiv", "mmioClkDiv", "useStandbyClk", "clkSrc"],
    "CyU3PPibClock_t": ["clkDiv", "isHalfDiv", "isDllEnable", "clkSrc"],
    "CyU3PEpConfig_t": ["enable", "epType", "streams", "pcktSize", "burstLen", "isoPkts"],
    "CyU3PDmaChannelConfig_t": ["size", "count", "prodSckId", "consSckId", "prodAvailCount", "prodHeader",
                                "prodFooter", "consHeader", "dmaMode", "notification", "cb"],
    "CyU3PDmaBuffer_t": ["buffer", "count", "size", "status"],
    "CyU3PI2cPreamble_t": ["buffer", "length", "ctrlMask"],
    "CyU3PGpifConfig_t": ["stateCount", "stateData", "statePosition", "functionCount", "functionData",
                          "regCount", "regData"],
}
HEADERS = ["cyu3types.h", "cyu3system.h", "cyu3dma.h", "cyu3usb.h", "cyu3gpif.h", "cyu3pib.h", "cyu3i2c.h",
           "cyu3spi.h", "cyu3uart.h", "cyu3gpio.h"]


def _probe_source():
    lines = ["#include <stddef.h>"] + ["#include <%s>" % h for h in HEADERS]
    for s, fields in STRUCTS.items():
        lines.append("const unsigned int L_%s[] = { sizeof(%s)," % (s, s))
        for f in fields:
            lines.append("  offsetof(%s, %s), sizeof(((%s *)0)->%s)," % (s, f, s, f))
        lines.append("};")
    return "\n".join(lines) + "\n"


def load():
    """{struct: {"size": n, "fields": [(name, offset, size)]}}, cached per header contents."""
    src = _probe_source()
    h = hashlib.sha256(src.encode())
    for name in HEADERS:
        h.update(open(os.path.join(paths.SDK_INC, name), "rb").read())
    cache = os.path.join(paths.CACHE, "layouts-%s.json" % h.hexdigest()[:16])
    if os.path.exists(cache):
        return json.load(open(cache))
    with tempfile.TemporaryDirectory() as tmp:
        c = os.path.join(tmp, "probe.c")
        open(c, "w").write(src)
        r = subprocess.run([paths.FX3_GCC, "-S", "-O0", "-mcpu=arm926ej-s", "-DCYU3P_FX3=1", "-D__CYU3P_TX__=1",
                            "-I", paths.SDK_INC, "-isystem", paths.NEWLIB_INC, c, "-o", "-"],
                           capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError("layout probe failed:\n" + r.stderr)
    out, cur = {}, None
    for line in r.stdout.split("\n"):
        m = re.match(r"^L_(\w+):", line)
        if m:
            cur = m.group(1); out[cur] = []
            continue
        m = re.match(r"\s+\.word\s+(\d+)", line)
        if m and cur:
            out[cur].append(int(m.group(1)))
    res = {}
    for s, fields in STRUCTS.items():
        v = out[s]
        res[s] = {"size": v[0], "fields": [(f, v[1 + 2 * i], v[2 + 2 * i]) for i, f in enumerate(fields)]}
    os.makedirs(paths.CACHE, exist_ok=True)
    json.dump(res, open(cache, "w"), indent=1)
    return res
