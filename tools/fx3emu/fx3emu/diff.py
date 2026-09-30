"""Run scenarios on two firmware images and report every behavioural difference."""
import time

from .device import Device
from .expected import acknowledged

FAILURE_MARKERS = ("status=46", "failed ret", "Failed to ReadI2C", "from 3e0", "SPI wait status", "command failed")

# The open build's version string differs from the vendor's in one token, by design (firmware/src/version.c):
# "..._SC130_OPN_V2.10_..." for "..._SC130_FX3_V2.10_...", same length. Compare with the tag mapped back,
# so the rest of every trace, reply and state stays under the strict comparison (acknowledging the first
# divergence instead would wave through everything after a 00/05 in a scenario).
BUILD_TAG = [(b"_OPN_V", b"_FX3_V")]
_TAG_HEX = [(o.hex(), v.hex()) for o, v in BUILD_TAG]
_TAG_WORDS = [("4f5f3033 565f4e50", "465f3033 565f3358")]   # fw_version as little-endian words ("0_O"/"PN_V")


def _untag_text(t):
    for o, v in _TAG_HEX + _TAG_WORDS:
        t = t.replace(o, v)
    return t


def _untag_bytes(b):
    for o, v in BUILD_TAG:
        b = b.replace(o, v)
    return b


def untagged(dev):
    """A device's trace, replies and final state with the open build's version tag mapped to the vendor's."""
    trace = [tuple(_untag_text(x) if isinstance(x, str) else x for x in ev) for ev in dev.trace]
    replies = [(ch, _untag_bytes(data)) for ch, data in dev.m.replies]
    state = {k: _untag_text(v) for k, v in dev.m.final_state().items()}
    return trace, replies, state


def run(image, scenario, seed=0, faults=True):
    d = Device(image, seed=seed)
    d.m.i2c.faults_enabled = faults
    scenario(d)
    d.run(2000)                     # let queued work finish
    return d


def first_divergence(a, b, context=4):
    """The first differing trace event with a little context, as one string (or '' if traces match)."""
    ta, tb = a.trace, b.trace
    n = min(len(ta), len(tb))
    i = next((k for k in range(n) if ta[k] != tb[k]), None)
    if i is None:
        return ""
    lines = [fmt(ta[k]) for k in range(max(0, i - context), i)]
    lines += ["A " + fmt(ta[i])] if i < len(ta) else []
    lines += ["B " + fmt(tb[i])] if i < len(tb) else []
    return "\n".join(lines)


def read_failure_before(dev, i, window=400):
    """True if a failed peripheral read belongs to the same command as the divergence at index i.

    Bound the search to the current command on the divergence's own thread: scan back over events with
    the same context, stopping at that thread's DmaChannelGetBuffer (the fetch that began this command).
    A failure marker in that span means a read returned failure, so the read helpers' defined 0 (vs the
    vendor's stack garbage) is the intended difference. Interleaved events from other threads/host are
    skipped, not treated as boundaries."""
    tr = dev.trace
    ctx = tr[i][1]
    for k in range(i, max(0, i - window) - 1, -1):
        ev = tr[k]
        if ev[1] != ctx:
            continue                    # another thread / host: not part of this command
        text = " ".join(str(x) for x in ev)
        if any(m in text for m in FAILURE_MARKERS):
            return True
        if k != i and "DmaChannelGetBuffer" in text and ("ch_ctrl_out" in text or "ch_bulk_out" in text):
            break                       # reached the start of this command on this thread
    return False


def compare(a, b, context=4):
    """Differences between two finished Devices: trace, replies, final state. [] if identical."""
    out = []
    (ta, ra, sa), (tb, rb, sb) = untagged(a), untagged(b)
    n = min(len(ta), len(tb))
    first = next((i for i in range(n) if ta[i] != tb[i]), None)
    if first is None and len(ta) != len(tb):
        first = n
    if first is not None:
        lines = ["trace differs at event %d (of %d / %d):" % (first, len(ta), len(tb))]
        for i in range(max(0, first - context), first):
            lines.append("    " + fmt(ta[i]))
        for i in range(first, min(first + context, max(len(ta), len(tb)))):
            if i < len(ta):
                lines.append("  A " + fmt(ta[i]))
            if i < len(tb):
                lines.append("  B " + fmt(tb[i]))
        out.append("\n".join(lines))
    if ra != rb:
        k = next((i for i in range(min(len(ra), len(rb))) if ra[i] != rb[i]), min(len(ra), len(rb)))
        out.append("replies differ at #%d: A %s / B %s" % (
            k, (ra[k][0], ra[k][1][:24].hex()) if k < len(ra) else "-", (rb[k][0], rb[k][1][:24].hex()) if k < len(rb) else "-"))
    for key in sorted(set(sa) & set(sb)):
        if sa[key] != sb[key]:
            out.append("final %s differs:\n  A %s\n  B %s" % (key, sa[key][:200], sb[key][:200]))
    missing = [k for k in set(sa) ^ set(sb) if not k.startswith("mem:")]
    if missing:
        out.append("final state keys only on one side: %s" % sorted(missing))
    if a.halted != b.halted:
        out.append("ended differently: A %s / B %s" % (a.halted, b.halted))
    return out


def fmt(ev):
    return "%6d %-18s %s" % (ev[0], ev[1][:18], " ".join(ev[2:]))[:220]


def differential(image_a, image_b, scenarios, seed=0, log=print, strict=False, faults=True):
    """Run every scenario on both images; returns {scenario: [unexpected diffs]} and the B devices.
    Intended fixes (expected.py) are separated out unless strict."""
    results, devices, expected_total = {}, [], 0
    for sc in scenarios:
        t = time.time()
        a, b = run(image_a, sc, seed, faults), run(image_b, sc, seed, faults)
        blocks = compare(a, b)
        # If the first trace divergence is an intended fix, everything downstream is an expected
        # consequence and the whole scenario is acknowledged.
        # A scenario's diffs are intended if the first trace divergence is a known fix (everything
        # after it is a consequence), or -- when only final state differs -- if every block is.
        (ta, _, _), (tb, _, _) = untagged(a), untagged(b)   # (the version tag is not a difference)
        n = min(len(ta), len(tb))
        diff_idx = [k for k in range(n) if ta[k] != tb[k]]

        def explained(k):
            # a differing event is intended if a failed read caused it (garbage -> 0), or it matches a
            # recorded fix signature (the SET_REPORT overflow: EP0-in / hid buffers).
            if read_failure_before(a, k) or read_failure_before(b, k):
                return True
            lines = "A " + fmt(ta[k]) + "\nB " + fmt(tb[k])
            return acknowledged(sc.__name__, lines) is not None

        if strict:
            ack = False
        elif diff_idx and all(explained(k) for k in diff_idx) and len(ta) == len(tb):
            ack = True                          # every divergence individually intended, traces aligned
        elif diff_idx and explained(diff_idx[0]) and a.halted != b.halted and "crash" in (a.halted, b.halted):
            ack = True                          # an intended fix prevents a vendor crash (control flow then differs)
        elif not diff_idx:
            ack = bool(blocks) and all(acknowledged(sc.__name__, b_) for b_ in blocks)
        else:
            ack = False
        unexpected = [] if ack else blocks
        expected_total += len(blocks) if ack else 0
        results[sc.__name__] = unexpected
        devices.append(b)
        tag = "SAME" if not blocks else ("OK*" if not unexpected else "DIFFERS")
        log("%-24s %-8s %6d events  %-6s %.1fs" % (sc.__name__, tag, len(b.trace), b.halted or "", time.time() - t))
        for d in unexpected:
            log("  " + d.replace("\n", "\n  "))
    if expected_total:
        log("(%d expected diff block(s) from intended fixes acknowledged; OK* marks scenarios with only those)"
            % expected_total)
    return results, devices
