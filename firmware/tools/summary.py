"""Per-function evidence for naming: SDK calls, app calls, and the text of referenced strings."""
import os, re, sys
T = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
img = open(os.path.join(os.path.dirname(T), "fw/fx3_sysmem.bin"), "rb").read()
def cstr(a):
    o = a - 0x40003000; e = img.index(b"\0", o); return img[o:e].decode("latin1")
names = [l.split("\t")[1] for l in open(os.path.join(T, "symbols.tsv")).read().split("\n")[1:] if l]
for m in sorted(f for f in os.listdir(os.path.join(T, "src")) if f.endswith(".c")):
    s = open(os.path.join(T, "src", m)).read()
    for f in re.finditer(r"^[A-Za-z_][^;\n{}]*\b(\w+)\s*\([^;{}]*\)\s*\{", s, re.M):
        i, d = f.end(), 1
        while d: d += {"{": 1, "}": -1}.get(s[i], 0); i += 1
        body = s[f.end():i]
        sdk = sorted(set(re.findall(r"\b(CyU3P\w+|CyFx3\w+)\s*\(", body)) - {"CyU3PDebugPrint"})
        app = sorted(set(n for n in re.findall(r"\b(\w+)\s*\(", body) if n in names))
        strs = [cstr(int(g, 16))[:48] for g in dict.fromkeys(re.findall(r"\bg_(4000[de][0-9a-f]{3})\b", body))]
        print("%s  %s\n    sdk: %s\n    app: %s\n    str: %s" % (m, f.group(1), " ".join(sdk), " ".join(app), " | ".join(repr(x) for x in strs[:6])))
