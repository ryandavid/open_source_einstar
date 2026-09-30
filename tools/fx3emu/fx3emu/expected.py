"""Intended behavioural differences from the vendor image.

The vendor binary is the ground truth, but a few of its behaviours are bugs whose output is undefined
(reads of uninitialised memory, a buffer overflow). We fix them in the source, so the modern build
deliberately stops matching the vendor's garbage at those points. `fx3emu diff` separates these
intended deviations from regressions; `diff --strict` shows every difference.

Two mechanisms:
* **Failed-read divergences** -- the read helpers now return 0 instead of the vendor's stack garbage
  when an FPGA/sensor read fails (a failed I2C transfer, or a malformed request that selects no
  device). These are recognised *by cause* in diff.py (`read_failure_before`): a divergence is intended
  when a failed read precedes it within the same command. This covers every readback command
  (exposure, gain, temperature, strobe, LD mode, trigger, device state, ...) without enumerating them.
* **Final-state-only fixes** below are matched by signature when a scenario differs only in end state.

The open build's version tag (00/05 reports ..._OPN_V2.10_... for ..._FX3_V2.10_...) is not a fix but a
deliberate identity change; diff.py maps it back before comparing (BUILD_TAG), so it needs no entry here.
"""

EXPECTED = [
    {
        "fix": "SET_REPORT no longer overflows the 8-byte HID buffer, so GET_REPORT data and the start "
               "of ch_bulk_in are not corrupted (usb.c usb_setup_cb copies sizeof(hid_set_report_buf)).",
        "scenarios": ["setup_requests", "eeprom_requests", "fuzz"],
        "signatures": ["mem:ch_bulk_in", "mem:hid_get_report_data", "mem:hid_set_report_buf", "EP0-in"],
    },
]


def acknowledged(scenario, block):
    for e in EXPECTED:
        scs = e["scenarios"]
        if scs is not None:
            scs = [scs] if isinstance(scs, str) else scs
            if not any(s in scenario for s in scs):
                continue
        if any(sig in block for sig in e["signatures"]):
            return e
    return None
