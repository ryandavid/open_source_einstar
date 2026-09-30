#!/usr/bin/env python3
"""Rename identifiers across the tree (src/, include/, symbols.tsv).  rename.py names.tsv
names.tsv lines: old<TAB>new[<TAB>note]. Whole-word replacement."""
import os, re, sys
T = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
pairs = [l.split("\t") for l in open(sys.argv[1]).read().split("\n") if l and not l.startswith("#")]
files = [os.path.join(T, d, f) for d in ("src", "include") for f in os.listdir(os.path.join(T, d))]
files.append(os.path.join(T, "symbols.tsv"))
for p in files:
    s = o = open(p).read()
    for row in pairs:
        s = re.sub(r"\b%s\b" % re.escape(row[0]), row[1], s)
    if s != o:
        open(p, "w").write(s)
# renamed data placeholders keep their vendor address in data.tsv
dp = os.path.join(T, "data.tsv")
existing = open(dp).read() if os.path.exists(dp) else "# address\tname  (application data; used by build.py to label the generated data)\n"
for row in pairs:
    m = re.match(r"g_([0-9a-f]{8})", row[0])
    if m:
        existing += "0x%s\t%s%s\n" % (m.group(1), row[1], ("\t" + row[2]) if len(row) > 2 else "")
open(dp, "w").write(existing)
# notes into symbols.tsv
notes = {r[1]: r[2] for r in pairs if len(r) > 2}
p = os.path.join(T, "symbols.tsv")
rows = []
for l in open(p).read().split("\n"):
    f = l.split("\t")
    if l and not l.startswith("#") and f[1] in notes:
        l = "%s\t%s\t%s" % (f[0], f[1], notes[f[1]])
    rows.append(l)
open(p, "w").write("\n".join(rows))
