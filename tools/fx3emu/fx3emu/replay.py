"""Replay a host conversation into the firmware and check that the firmware agrees with the host.

The trace comes from the host's own code talking to its emulator (tests/device/test_firmware.cpp, "record
the host's firmware-flash conversation"): one line per event,

    open                          the host opened the scanner (after the first: it must have rebooted)
    command <request> <reply>     hex; <reply> is what the host's emulator answered, or error:<code>
    bulk <request> <reply>

Each request is sent to the emulated firmware as the host sends it (commands at their buffer size, bulk
padded to the 5120-byte DMA buffer). Checked: every reply's header (sequence, 0x02, key, status, declared
length) and size, the data of the constant replies, that the firmware reset wherever the host reconnected,
and, for a firmware update, that the slot the firmware wrote holds the package and its boot record selects
that slot. The SPI flash carries over each reset.
"""
from .device import Device, default_flash, BOOT_RECORD, SLOT_A_APP, SLOT_B_APP

STALE = 0xCD                       # bytes the host's emulator leaves unwritten (stale in the firmware)
NO_DATA_CHECK = {0x0004, 0x0007, 0x1050}  # hardware-dependent: FX3 die id, FPGA state register, temperature


def replay(image, path, seed=0, out=print):
    events = [line.split() for line in open(path) if line.strip()]
    flash = default_flash(seed)
    d = None
    problems = []
    pages = []                     # update pages as sent: (payload length, payload)
    replies = 0

    def problem(i, text):
        problems.append("event %d: %s" % (i + 1, text))

    for i, ev in enumerate(events):
        if ev[0] == "open":
            if d is not None:
                d.run(1500)        # a reboot resets 25 ms after its reply; an update's within ~1 s
                if d.halted != "reset":
                    problem(i, "the host reconnected, but the firmware did not reset (%s)" % d.halted)
                flash = [(0, bytes(d.m.flash.mem))]
            d = Device(image, seed=seed, flash=flash).boot().enumerate()
            continue
        channel, req, want = ev[0], bytes.fromhex(ev[1]), ev[2]
        key = req[2] << 8 | req[3]
        if channel == "command":
            got = d.command(req, pad_to=len(req))
        else:
            got = d.bulk(req, pad_to=5120)
            if key == 0x0006 and int.from_bytes(req[4:8], "big") != 5:
                pages.append(req[8:8 + int.from_bytes(req[4:8], "big")])
        got = [data for ch, data in got if ch != "ep0"]
        if want.startswith("error"):
            if got:
                problem(i, "%s %04x: the host got no reply (%s), the firmware sent %s" % (channel, key, want, got[-1][:9].hex()))
            continue
        want = bytes.fromhex(want)
        if not got:
            problem(i, "%s %04x: the firmware sent no reply, the host expected %s" % (channel, key, want[:9].hex()))
            continue
        reply = got[-1]
        replies += 1
        if reply[:9] != want[:9]:
            problem(i, "%s %04x: header %s, the host expected %s" % (channel, key, reply[:9].hex(), want[:9].hex()))
        if len(reply) != len(want):
            problem(i, "%s %04x: %d reply bytes, the host read %d" % (channel, key, len(reply), len(want)))
        if channel == "command" and key not in NO_DATA_CHECK:
            n = 9 + int.from_bytes(want[5:9], "big")
            diff = [j for j in range(9, min(n, len(want), len(reply))) if want[j] != STALE and want[j] != reply[j]]
            if diff:
                problem(i, "command %04x: data differs at %s (firmware %s, host %s)" % (
                    key, diff[:4], reply[9:n].hex(), want[9:n].hex()))

    summary = ["%d events, %d replies compared" % (len(events), replies)]
    if pages:
        # The update: which slot, what it holds, and the boot record.
        mem = d.m.flash.mem if d is not None else None
        d.run(1500)
        if d.halted == "reset":
            mem = d.m.flash.mem
        rec = bytes(mem[BOOT_RECORD:BOOT_RECORD + 9])
        new_app, boot_new = int.from_bytes(rec[0:4], "little"), rec[8]
        data = b"".join(p[1:-1] for p in pages)     # payload: unused byte, data, check byte
        summary.append("update: %d pages; boot record new %#x, old %#x, boot_new %d" % (
            len(pages), new_app, int.from_bytes(rec[4:8], "little"), boot_new))
        if new_app not in (SLOT_A_APP, SLOT_B_APP) or boot_new != 1:
            problems.append("the boot record does not select a freshly written slot")
        elif bytes(mem[new_app:new_app + len(data)]) != data:
            problems.append("slot %#x does not hold the package" % new_app)
        else:
            summary.append("slot %#x holds the package byte for byte" % new_app)
    for s in summary:
        out(s)
    for p in problems:
        out("MISMATCH " + p)
    return problems
