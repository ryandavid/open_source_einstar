"""Firmware images for the emulator: memory contents plus a symbol table.

* `Image.from_elf(path)`: any build of the firmware.
* `Image.from_vendor(package, names_elf)`: the vendor's own image, taken from EXStar's update package.
  It has no symbols; names come from an ELF that is byte-identical to it (reference/vendor-names.elf),
  so every vendor address gets its real name.
"""
import os, struct

from . import paths


class Symbol:
    __slots__ = ("name", "addr", "size", "kind", "bind")

    def __init__(self, name, addr, size, kind, bind):
        self.name, self.addr, self.size, self.kind, self.bind = name, addr, size, kind, bind

    def __repr__(self):
        return "<%s %s @%08x+%d>" % (self.kind, self.name, self.addr, self.size)


class Image:
    def __init__(self, label, segments, symbols, mapping, entry):
        self.label = label
        self.segments = segments          # [(addr, bytes)]: loaded contents (bss is zero)
        self.symbols = symbols            # name -> Symbol (functions and objects)
        self.mapping = mapping            # sorted [(addr, 'a'|'d')]: ARM code / literal data markers
        self.entry = entry
        self.funcs = sorted((s for s in symbols.values() if s.kind == "func"), key=lambda s: s.addr)
        self.objects = sorted((s for s in symbols.values() if s.kind == "object"), key=lambda s: s.addr)

    # ---------------------------------------------------------------- loaders
    @classmethod
    def from_elf(cls, path, label=None):
        data = open(path, "rb").read()
        segs, syms, mapping, entry = _parse_elf(data)
        return cls(label or os.path.basename(path), segs, syms, mapping, entry)

    @classmethod
    def from_vendor(cls, package, names_elf, label="vendor"):
        pkg = open(package, "rb").read()
        slot = b"".join(pkg[i:i + 4096] for i in range(0, len(pkg), 4097))
        segs, entry = _parse_cy_image(slot)
        ref = cls.from_elf(names_elf)
        # the named build must be the vendor image byte for byte
        for a, d in segs:
            got = ref.read(a, len(d))
            if got != d:
                raise ValueError("%s does not match the vendor image at 0x%08x: names would be wrong" % (names_elf, a))
        return cls(label, segs, ref.symbols, ref.mapping, entry)

    # ---------------------------------------------------------------- queries
    def read(self, addr, n):
        out = bytearray(n)
        for a, d in self.segments:
            lo, hi = max(a, addr), min(a + len(d), addr + n)
            if lo < hi:
                out[lo - addr:hi - addr] = d[lo - a:hi - a]
        return bytes(out)

    def addr(self, name):
        return self.symbols[name].addr

    def func_at(self, addr):
        """The function containing addr (binary search over sized function symbols)."""
        lo, hi = 0, len(self.funcs)
        while lo < hi:
            mid = (lo + hi) // 2
            if self.funcs[mid].addr <= addr:
                lo = mid + 1
            else:
                hi = mid
        if lo and self.funcs[lo - 1].addr <= addr < self.funcs[lo - 1].addr + max(self.funcs[lo - 1].size, 1):
            return self.funcs[lo - 1]
        return None

    def object_at(self, addr):
        lo, hi = 0, len(self.objects)
        while lo < hi:
            mid = (lo + hi) // 2
            if self.objects[mid].addr <= addr:
                lo = mid + 1
            else:
                hi = mid
        if lo:
            s = self.objects[lo - 1]
            if s.addr <= addr < s.addr + max(s.size, 1):
                return s
        return None

    def code_words(self, sym):
        """Addresses of the instruction words of a function (literal-pool words excluded)."""
        marks = [(a, m) for a, m in self.mapping if sym.addr <= a < sym.addr + sym.size]
        state, out, i = "a", [], 0
        for a in range(sym.addr, sym.addr + sym.size, 4):
            while i < len(marks) and marks[i][0] <= a:
                state = marks[i][1]
                i += 1
            if state == "a":
                out.append(a)
        return out


# ---------------------------------------------------------------- formats
def _parse_cy_image(b):
    assert b[:2] == b"CY", "not an FX3 boot image"
    i, segs = 4, []
    while True:
        n, addr = struct.unpack_from("<II", b, i)
        i += 8
        if n == 0:
            return segs, addr
        segs.append((addr, bytes(b[i:i + 4 * n])))
        i += 4 * n


def _parse_elf(data):
    assert data[:4] == b"\x7fELF" and data[4] == 1 and data[5] == 1, "need a 32-bit little-endian ELF"
    (e_type, e_machine, _, e_entry, e_phoff, e_shoff, _, _, e_phentsize, e_phnum, e_shentsize,
     e_shnum, e_shstrndx) = struct.unpack_from("<HHIIIIIHHHHHH", data, 16)
    # allocated sections, not program segments: a segment can also cover the file headers, which the
    # device never has in memory
    shdrs = [struct.unpack_from("<10I", data, e_shoff + k * e_shentsize) for k in range(e_shnum)]
    segs = []
    for sh in shdrs:
        sh_type, sh_flags, sh_addr, sh_off, sh_size = sh[1], sh[2], sh[3], sh[4], sh[5]
        if sh_flags & 2 and sh_size:                     # SHF_ALLOC
            segs.append((sh_addr, b"\0" * sh_size if sh_type == 8 else data[sh_off:sh_off + sh_size]))
    syms, mapping = {}, []
    for sh in shdrs:
        if sh[1] != 2:                       # SHT_SYMTAB
            continue
        strtab = shdrs[sh[6]]
        s_off = strtab[4]
        for j in range(sh[5] // 16):
            st_name, st_value, st_size, st_info, _, st_shndx = struct.unpack_from("<IIIBBH", data, sh[4] + j * 16)
            end = data.index(b"\0", s_off + st_name)
            name = data[s_off + st_name:end].decode("latin1")
            typ, bind = st_info & 0xf, st_info >> 4
            if name in ("$a", "$d", "$t") or name.startswith(("$a.", "$d.", "$t.")):
                mapping.append((st_value & ~1, "d" if name[1] == "d" else "a"))
                continue
            if not name or st_shndx == 0 or typ not in (1, 2):
                continue
            kind = "func" if typ == 2 else "object"
            if name in syms and syms[name].bind == 1 and bind != 1:
                continue                         # keep the global over a same-named local
            addr = st_value & ~1 if kind == "func" else st_value   # bit 0 is Thumb, only on functions
            syms[name] = Symbol(name, addr, st_size, kind, bind)
    mapping.sort()
    return segs, syms, mapping, e_entry


def library_names():
    """Every name (global or local) defined in the SDK and C libraries: not application code."""
    names = set(CYFXTX_NAMES)
    for d in (paths.SDK_LIB, os.path.join(paths.FIRMWARE, "sdk", "newlib", "lib")):
        if os.path.isdir(d):
            for f in sorted(os.listdir(d)):
                if f.endswith(".a"):
                    names |= _archive_defined(os.path.join(d, f), local=True)
    return names


def sdk_names():
    """Names defined by the Cypress libraries (and cyfxtx.c): calls to these are modelled, not run."""
    names = set(CYFXTX_NAMES)
    for f in sorted(os.listdir(paths.SDK_LIB)):
        if f.endswith(".a"):
            names |= _archive_defined(os.path.join(paths.SDK_LIB, f))
    return names


def _archive_defined(path, local=False):
    data = open(path, "rb").read()
    assert data[:8] == b"!<arch>\n"
    i, names, longnames = 8, set(), b""
    while i + 60 <= len(data):
        hdr = data[i:i + 60]
        name = hdr[:16].decode("latin1").strip()
        size = int(hdr[48:58].decode().strip())
        body = data[i + 60:i + 60 + size]
        if name == "//":
            longnames = body
        elif name not in ("/", "/SYM64/") and body[:4] == b"\x7fELF":
            names |= _elf_globals(body, local)
        i += 60 + size + (size & 1)
    return names


def _elf_globals(data, local=False):
    e_shoff, = struct.unpack_from("<I", data, 32)
    e_shentsize, e_shnum = struct.unpack_from("<HH", data, 46)
    shdrs = [struct.unpack_from("<10I", data, e_shoff + k * e_shentsize) for k in range(e_shnum)]
    out = set()
    for sh in shdrs:
        if sh[1] != 2:
            continue
        s_off = shdrs[sh[6]][4]
        for j in range(sh[5] // 16):
            st_name, _, _, st_info, _, st_shndx = struct.unpack_from("<IIIBBH", data, sh[4] + j * 16)
            if st_shndx and ((st_info >> 4) in (1, 2) or local) and (st_info & 0xf) in (0, 1, 2):
                end = data.index(b"\0", s_off + st_name)
                out.add(data[s_off + st_name:end].decode("latin1"))
    return out


# cyfxtx.c (the SDK's heap/DMA-buffer glue compiled with the application): modelled like the SDK
CYFXTX_NAMES = {
    "CyU3PAbortHandler", "CyU3PBufCorruptionCheck", "CyU3PBufEnableChecks", "CyU3PBufGetActiveList",
    "CyU3PBufGetCounts", "CyU3PDmaBufferAlloc", "CyU3PDmaBufferDeInit", "CyU3PDmaBufferFree",
    "CyU3PDmaBufferInit", "CyU3PDmaBufMgrSetStatus", "CyU3PFreeHeaps", "CyU3PMemAlloc", "CyU3PMemCmp",
    "CyU3PMemCopy", "CyU3PMemCorruptionCheck", "CyU3PMemEnableChecks", "CyU3PMemFree",
    "CyU3PMemGetActiveList", "CyU3PMemGetCounts", "CyU3PMemInit", "CyU3PMemSet", "CyU3PPrefetchHandler",
    "CyU3PUndefinedHandler", "tx_application_define",
}
